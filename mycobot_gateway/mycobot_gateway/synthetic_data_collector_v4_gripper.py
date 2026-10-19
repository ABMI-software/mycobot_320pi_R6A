#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Enhanced Synthetic Data Collector v3 for MyCobot 320 Pi Pose Estimation.

Improvements over v2:
- Uniform joint sampling over the PRACTICAL (green) limits, no home-pose bias.
- Settle verification: a pose is recorded only if the robot actually REACHED
  the commanded target (within SETTLE_TOL). Stuck / gravity-drooped poses are
  skipped instead of being logged — this removes the sharp distribution spikes
  seen in v2 (e.g. j2≈-135°, j5≈-90°).
- More realistic office-style domain randomization (brighter, neutral-white
  lighting; warm-wood table; light walls; grey ground) to better match the
  real arducam / svpro / astra cameras.
- 4 camera viewpoints kept (front, right, left, top).

Usage (standalone):
    ros2 run mycobot_gateway synthetic_data_collector_v3 \
        --ros-args -p num_samples:=3 -p output_dir:=/tmp/synth_v3_test \
        -p multi_view:=true -p domain_randomize:=true

Or via launch:
    ros2 launch mycobot_gateway synthetic_data_v3.launch.py num_samples:=3
"""

import csv
import math
import os
import random
import subprocess
import time
import zlib
from typing import Dict, List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from builtin_interfaces.msg import Duration
from sensor_msgs.msg import Image, JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


class SyntheticDataCollectorV3(Node):
    """Data collector with uniform sampling, settle check + realistic DR."""

    JOINT_NAMES = [
        'joint2_to_joint1',
        'joint3_to_joint2',
        'joint4_to_joint3',
        'joint5_to_joint4',
        'joint6_to_joint5',
        'joint6output_to_joint6',
    ]

    # Practical MyCobot 320 Pi joint limits (green spec sheet — usable range)
    # J1 ±168°  J2 ±135°  J3 ±150°  J4 ±145°  J5 ±165°  J6 ±180°
    JOINT_LIMITS = [
        (-2.9322, 2.9322),   # J1 ±168° – base rotation
        (-2.3562, 2.3562),   # J2 ±135° – shoulder
        (-2.6180, 2.6180),   # J3 ±150° – elbow
        (-2.5307, 2.5307),   # J4 ±145° – wrist 1
        (-2.8798, 2.8798),   # J5 ±165° – wrist 2
        (-3.1416, 3.1416),   # J6 ±180° – wrist 3 (flange)
    ]

    # Max allowed deviation (rad) between commanded and actual joint angle for a
    # pose to be considered "settled" and worth recording. ~0.035 rad ≈ 2°.
    SETTLE_TOL = 0.035

    # DH link lengths (metres) – used for self-collision check
    L1 = 0.162    # base to J2
    L2 = 0.13635  # J2–J3  (upper arm)
    L3 = 0.1205   # J3–J4  (forearm)
    L4 = 0.084    # J4–J5  (wrist offset)
    L5 = 0.06635  # J5–J6  (flange)

    # Camera topic names (must match URDF sensor topics)
    CAMERA_TOPICS = {
        'front': '/synth_camera/image',
        'right': '/synth_camera_right/image',
        'left':  '/synth_camera_left/image',
        'top':   '/synth_camera_top/image',
    }

    def __init__(self):
        super().__init__('synthetic_data_collector_v4_gripper')

        # ---------- parameters ----------
        self.declare_parameter('num_samples', 5000)
        self.declare_parameter('output_dir', '/tmp/mycobot_synth_v4_gripper')
        self.declare_parameter('settle_time', 1.5)
        # Full practical (green) range by default — uniform coverage.
        self.declare_parameter('joint_limit_fraction', 1.0)
        # Fraction of poses that are grasp-approach (EE reaching down over the
        # table) vs uniform random — for a pick-and-place-oriented dataset.
        self.declare_parameter('grasp_fraction', 0.5)
        self.declare_parameter('multi_view', True)
        self.declare_parameter('domain_randomize', True)
        # Pixel-noise σ (0–255). Default 0 = clean, photo-realistic images like
        # the /tmp/synth_v3_nodr reference (no grain / colour-shift / vignette).
        self.declare_parameter('noise_stddev', 0.0)
        self.declare_parameter('world_name', 'randomized')
        # Max consecutive non-settled skips before accepting the target anyway,
        # so the collection can never stall forever on a hard pose.
        self.declare_parameter('max_settle_skips', 4)
        # JointTrajectory move duration (s) — must be < settle_time.
        self.declare_parameter('move_time', 1.0)
        # ros2_control JointTrajectoryController command topic.
        self.declare_parameter('traj_topic', '/mycobot_controller/joint_trajectory')

        self.num_samples = self.get_parameter('num_samples').value
        self.output_dir = self.get_parameter('output_dir').value
        self.settle_time = self.get_parameter('settle_time').value
        self.limit_frac = self.get_parameter('joint_limit_fraction').value
        self.grasp_fraction = self.get_parameter('grasp_fraction').value
        self.multi_view = self.get_parameter('multi_view').value
        self.domain_randomize = self.get_parameter('domain_randomize').value
        self.noise_stddev = self.get_parameter('noise_stddev').value
        self.world_name = self.get_parameter('world_name').value
        self.max_settle_skips = self.get_parameter('max_settle_skips').value
        self.move_time = self.get_parameter('move_time').value
        self.traj_topic = self.get_parameter('traj_topic').value

        # ---------- state ----------
        self.current_images: Dict[str, Optional[Image]] = {}
        self.current_joints: Optional[List[float]] = None
        self.sample_idx = 0
        self.collecting = False
        self._settle_skips = 0       # consecutive skips for the current pose
        self.n_recorded = 0
        self.n_skipped = 0
        # Anti render-freeze: CRC32 of the last SAVED frame per camera. If a new
        # pose's frame is identical, the render stalled → skip (see _capture).
        self._last_sig: Dict[str, int] = {}
        self._frozen_skips = 0

        # ---------- which cameras to use ----------
        if self.multi_view:
            self.active_cameras = list(self.CAMERA_TOPICS.keys())
        else:
            self.active_cameras = ['front']

        # ---------- publisher (JointTrajectoryController) ----------
        # This URDF is actuated by gz_ros2_control + a JointTrajectoryController
        # ('mycobot_controller'). Commanding individual /model/.../cmd_pos
        # Float64 topics does NOT move the robot (joints owned by ros2_control).
        self.traj_pub = self.create_publisher(JointTrajectory, self.traj_topic, 10)

        # ---------- subscribers ----------
        img_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )

        for cam_name in self.active_cameras:
            topic = self.CAMERA_TOPICS[cam_name]
            self.current_images[cam_name] = None
            self.create_subscription(
                Image, topic,
                lambda msg, cn=cam_name: self._image_cb(cn, msg),
                img_qos,
            )

        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_cb, 10,
        )

        # ---------- output dirs ----------
        for cam_name in self.active_cameras:
            os.makedirs(os.path.join(self.output_dir, 'images', cam_name), exist_ok=True)
        self.csv_path = os.path.join(self.output_dir, 'labels.csv')

        # Write CSV header
        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            header = [
                'index', 'j1_rad', 'j2_rad', 'j3_rad',
                'j4_rad', 'j5_rad', 'j6_rad',
                'j1_deg', 'j2_deg', 'j3_deg',
                'j4_deg', 'j5_deg', 'j6_deg',
                'camera', 'image_path',
            ]
            writer.writerow(header)

        self.get_logger().info(
            f'🎬 Synthetic Data Collector v3 initialised\n'
            f'   Samples       : {self.num_samples}\n'
            f'   Output        : {self.output_dir}\n'
            f'   Settle        : {self.settle_time}s (tol {math.degrees(self.SETTLE_TOL):.1f}°)\n'
            f'   Limit frac    : {self.limit_frac} (practical/green limits)\n'
            f'   Multi-view    : {self.multi_view} ({self.active_cameras})\n'
            f'   Domain random : {self.domain_randomize}\n'
            f'   Noise σ       : {self.noise_stddev}'
        )

        self._startup_timer = self.create_timer(5.0, self._start_collection_once)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _image_cb(self, cam_name: str, msg: Image):
        self.current_images[cam_name] = msg

    def _joint_cb(self, msg: JointState):
        if not msg.name:
            return
        angles = [0.0] * len(self.JOINT_NAMES)
        for i, jn in enumerate(self.JOINT_NAMES):
            if jn in msg.name:
                idx = list(msg.name).index(jn)
                angles[i] = msg.position[idx]
        self.current_joints = angles

    # ------------------------------------------------------------------
    # Domain randomization (office-realistic)
    # ------------------------------------------------------------------
    # All scene lights to randomize (name → type)
    SCENE_LIGHTS = {
        'sun':           'directional',
        'fill_light':    'directional',
        'back_light':    'directional',
        'warm_point':    'point',
        'cool_point':    'point',
        'overhead_spot': 'point',
    }

    # Clutter models whose material colour can be randomized
    CLUTTER_MODELS = [
        'clutter_box_1', 'clutter_box_2', 'clutter_box_3',
        'clutter_box_4', 'clutter_box_5', 'clutter_box_6',
        'clutter_cylinder_1', 'clutter_cylinder_2',
        'clutter_cylinder_3', 'clutter_cylinder_4',
        'clutter_sphere_1', 'clutter_sphere_2',
    ]

    # Surfaces whose colour can be randomized
    SURFACE_MODELS = ['back_wall', 'left_wall', 'right_wall', 'table', 'ground_plane']

    def _randomize_scene(self):
        """Realistic domain randomization: lighting, clutter, surfaces."""
        if not self.domain_randomize:
            return

        try:
            self._randomize_lights()
            # Randomize clutter/surface colours every 5th sample (expensive)
            if self.sample_idx % 5 == 0:
                self._randomize_material_colours()
        except Exception as e:
            self.get_logger().debug(f'Domain randomization failed: {e}')

    def _gz_light_cmd(self, name, **kwargs):
        """Build and fire a gz service light_config command.

        IMPORTANT: light_config replaces the whole gz.msgs.Light, so any field
        we omit is reset to its proto3 default. In particular ``intensity``
        defaults to 0.0, which silently *dims the scene to ~half* every sample
        (this was the cause of the dark/grey captures). We therefore ALWAYS send
        an explicit intensity (default 1.0, matching the world's base lights).
        """
        parts = [f'name: \\"{name}\\"']
        if 'direction' in kwargs:
            d = kwargs['direction']
            parts.append(f'direction: {{x: {d[0]}, y: {d[1]}, z: {d[2]}}}')
        if 'diffuse' in kwargs:
            c = kwargs['diffuse']
            parts.append(f'diffuse: {{r: {c[0]}, g: {c[1]}, b: {c[2]}, a: 1.0}}')
        if 'pose' in kwargs:
            p = kwargs['pose']
            parts.append(f'pose: {{position: {{x: {p[0]}, y: {p[1]}, z: {p[2]}}}}}')
        # Always set intensity so it is never reset to the proto3 default (0).
        intensity = kwargs.get('intensity', 1.0)
        parts.append(f'intensity: {intensity}')

        req = ', '.join(parts)
        cmd = (
            f'gz service -s /world/{self.world_name}/light_config '
            f'--reqtype gz.msgs.Light --reptype gz.msgs.Boolean '
            f'--timeout 300 --req "{req}"'
        )
        subprocess.Popen(cmd, shell=True,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def _randomize_lights(self):
        """Neutral, sun-like office lighting.

        The real captures look like a single bright, near-white sun (fluorescent
        + window daylight): high illumination, NO colour cast, with the shadow
        direction being the main thing that changes shot to shot.

        We therefore drive each light with a single NEUTRAL scalar brightness
        (r == g == b) so the scene never picks up the cyan/yellow tints the old
        per-channel randomization produced, and only vary the SUN DIRECTION (for
        shadow variety) plus a tiny intensity wobble. Only lights that actually
        exist in randomized.sdf are commanded (sun, fill_light, warm_point) to
        avoid "light not found" spam.
        """
        # Sun — bright neutral white, STRONG and consistently RAKING from one
        # upper-side (like the nice 000003 capture). A fixed-ish side angle (only
        # a small jitter) guarantees every frame gets the same flattering shadow
        # definition on the joint seams — instead of some frames being flat /
        # front-lit and washed out (the ugly 000001 look). Brightness still
        # varies via intensity so there is some lighting variety for sim2real.
        sun_b = random.uniform(0.97, 1.0)
        self._gz_light_cmd(
            'sun',
            # Direction matched to the realistic /tmp/synth_v3_nodr reference
            # (light travels -x,+y, down) → same flattering shadow to the right.
            direction=(random.uniform(-0.55, -0.40),
                       random.uniform(0.20, 0.40),
                       random.uniform(-0.92, -0.80)),
            diffuse=(sun_b, sun_b, sun_b),
            # SOFT sun (~0.9), slightly dimmer than nodr per the user's request
            # ("un peu moins d'éclairage"); a strong sun blew out the white
            # plastic and looked over-lit.
            intensity=random.uniform(0.85, 1.0),
        )

        # Fill light — neutral white, fairly strong for a bright/clean look
        # (the joint definition now comes from the unsharp-mask sharpening in
        # post, so we no longer starve the fill to force dark contrast).
        fill_b = random.uniform(0.46, 0.56)
        self._gz_light_cmd(
            'fill_light',
            direction=(random.uniform(-0.5, 0.5),
                       random.uniform(-0.5, 0.5),
                       random.uniform(-0.9, -0.5)),
            diffuse=(fill_b, fill_b, fill_b),
            intensity=1.0,
        )

        # Overhead point light — neutral white top-up, mild position variation.
        pt_b = random.uniform(0.40, 0.55)
        self._gz_light_cmd(
            'warm_point',
            pose=(random.uniform(-0.6, 1.0),
                  random.uniform(-0.6, 0.6),
                  random.uniform(1.4, 2.4)),
            diffuse=(pt_b, pt_b, pt_b),
            intensity=0.9,
        )

    def _gz_visual_cmd(self, model_name, r, g, b):
        """Fire a gz service visual_config colour command for a model."""
        cmd = (
            f'gz service -s /world/{self.world_name}/visual_config '
            f'--reqtype gz.msgs.Visual --reptype gz.msgs.Boolean '
            f'--timeout 200 '
            f'--req "parent_name: \\"{model_name}::link\\", '
            f'name: \\"visual\\", '
            f'material: {{ambient: {{r: {r}, g: {g}, b: {b}, a: 1}}, '
            f'diffuse: {{r: {min(r+0.05,1)}, g: {min(g+0.05,1)}, b: {min(b+0.05,1)}, a: 1}}}}"'
        )
        subprocess.Popen(cmd, shell=True,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def _randomize_material_colours(self):
        """Realistic surfaces: warm-wood table, light walls, grey ground.

        Clutter objects stay fully random (real desks have varied items).
        """
        # Clutter — full RGB range
        for model_name in self.CLUTTER_MODELS:
            self._gz_visual_cmd(model_name,
                                random.uniform(0.1, 0.95),
                                random.uniform(0.1, 0.95),
                                random.uniform(0.1, 0.95))

        # Surfaces — realistic per-type palettes (matches the real office)
        for model_name in self.SURFACE_MODELS:
            if model_name == 'table':
                # Warm wood (the real wooden tabletop)
                r = random.uniform(0.45, 0.62)
                g = random.uniform(0.30, 0.44)
                b = random.uniform(0.15, 0.26)
            elif model_name == 'ground_plane':
                # Neutral grey carpet/floor
                base = random.uniform(0.25, 0.45)
                r, g, b = (base + random.uniform(-0.03, 0.03) for _ in range(3))
            else:
                # Walls — clean near-WHITE office partitions (tiny jitter only,
                # so they never pick up a colour cast).
                base = random.uniform(0.80, 0.92)
                r, g, b = (base + random.uniform(-0.02, 0.02) for _ in range(3))
            self._gz_visual_cmd(model_name, r, g, b)

    # ------------------------------------------------------------------
    # Collection loop
    # ------------------------------------------------------------------
    def _start_collection_once(self):
        if self.collecting:
            return
        self.collecting = True
        self._startup_timer.cancel()
        self.get_logger().info('🚀 Starting v3 data collection…')
        self._collect_next()

    def _collect_next(self):
        if self.sample_idx >= self.num_samples:
            total_images = self.sample_idx * len(self.active_cameras)
            self.get_logger().info(
                f'✅ Collection complete! {self.sample_idx} poses '
                f'({self.n_skipped} non-settled skipped), '
                f'{total_images} images saved to {self.output_dir}'
            )
            rclpy.shutdown()
            return

        # 1. Domain randomization (lighting)
        self._randomize_scene()

        # 2. Random joint command (uniform over practical limits)
        target_angles = self._random_joint_angles()
        self._command_joints(target_angles)

        # 3. Wait for settle, then capture
        timer = self.create_timer(
            self.settle_time,
            lambda: self._on_settle_timeout(timer, target_angles),
        )

    def _on_settle_timeout(self, timer, target_angles):
        timer.cancel()
        self._capture_and_save(target_angles)

    def _random_joint_angles(self) -> List[float]:
        """Generate a physically-valid joint configuration.

        A fraction ``grasp_fraction`` of the poses are GRASP-APPROACH poses
        (end-effector reaching out and pointing DOWN over the table, as if about
        to pick an object) so the pose-estimation network sees the arm in the
        configurations it will actually be in during pick-and-place — not only
        random poses waving in the air. The rest are uniform over the practical
        limits for broad coverage. All poses still pass the collision / table
        clearance check.
        """
        if random.random() < self.grasp_fraction:
            g = self._grasp_approach_angles()
            if g is not None:
                return g
        # Uniform fallback / the rest of the distribution.
        last = None
        for _ in range(200):
            angles = self._sample_raw_angles()
            last = angles
            if self._is_collision_free(angles):
                return angles
        return last

    def _grasp_approach_angles(self) -> Optional[List[float]]:
        """Sample a grasp-approach pose via planar inverse kinematics.

        Places the wrist above a random reachable point on the table and orients
        the flange to point downward (a top-down grasp approach), then adds small
        jitter on the last wrist joints. Returns None if no valid pose is found
        (caller falls back to uniform sampling).
        """
        for _ in range(60):
            # 1. Random reachable target on the table (polar around the base).
            theta = random.uniform(self.JOINT_LIMITS[0][0], self.JOINT_LIMITS[0][1])
            r_w = random.uniform(0.14, 0.36)     # horizontal wrist reach (m)
            z_w = random.uniform(0.30, 0.46)     # wrist height (keeps tip clear)

            # 2. Planar 2-link IK (shoulder at (0, L1)) for J2, J3.
            dr, dz = r_w, z_w - self.L1
            D = math.hypot(dr, dz)
            if D > (self.L2 + self.L3) - 1e-3 or D < abs(self.L2 - self.L3) + 1e-3:
                continue
            cos_e = (D * D - self.L2 ** 2 - self.L3 ** 2) / (2 * self.L2 * self.L3)
            cos_e = max(-1.0, min(1.0, cos_e))
            j3 = -math.acos(cos_e)                # elbow-up solution
            j2 = math.atan2(dz, dr) - math.atan2(
                self.L3 * math.sin(j3), self.L2 + self.L3 * math.cos(j3))

            # 3. Orient the flange to point DOWN (s4 ≈ -pi/2), with small jitter.
            s3 = j2 + j3
            j4 = (-math.pi / 2.0 - s3) + random.uniform(-0.30, 0.30)
            j5 = random.uniform(-0.6, 0.6)        # approach tilt
            j6 = random.uniform(*self.JOINT_LIMITS[5])  # free gripper roll

            angles = [theta, j2, j3, j4, j5, j6]
            # Clamp to practical limits, then keep only collision-free results.
            angles = [max(lo, min(hi, a)) for a, (lo, hi) in
                      zip(angles, self.JOINT_LIMITS)]
            angles = [round(a, 4) for a in angles]
            if self._is_collision_free(angles):
                return angles
        return None

    def _sample_raw_angles(self) -> List[float]:
        """Sample angles uniformly within ``limit_frac`` of the limits."""
        angles = []
        for lo, hi in self.JOINT_LIMITS:
            span = (hi - lo) * self.limit_frac
            mid = (hi + lo) / 2.0
            a = random.uniform(mid - span / 2, mid + span / 2)
            angles.append(round(a, 4))
        return angles

    # Safety margin (m) the arm must keep ABOVE the table top (z=0). Must stay
    # BELOW the fixed shoulder height (~0.162 m, top of the base column) or the
    # near-horizontal poses needed for a mean≈0 distribution would be rejected.
    TABLE_CLEARANCE = 0.13

    # URDF revolute-joint origins (xyz, rpy) in kinematic order j1..j6, used to
    # build the TRUE forward kinematics for the table-clearance check. At the
    # all-zero pose the arm points straight UP (flange z≈0.50) and j2 tilts it
    # SYMMETRICALLY, so the feasible angle distribution stays centred on 0 —
    # matching the real robot. (The previous hand-derived planar model wrongly
    # treated j2=0 as horizontal and skewed j2 to +64°.)
    _JOINT_TF = [
        ((0.0, 0.0, 0.162), (0.0, 0.0, 0.0)),
        ((0.0, 0.0, 0.0), (0.0, -1.5708, 1.5708)),
        ((0.13635, 0.0, 0.0), (0.0, 0.0, 0.0)),
        ((0.1205, 0.0, 0.082), (0.0, 0.0, 1.5708)),
        ((0.0, -0.084, 0.0), (1.5708, 0.0, 0.0)),
        ((0.0, 0.06635, 0.0), (-1.5708, 0.0, 0.0)),
    ]

    @staticmethod
    def _rpy_mat(r: float, p: float, y: float) -> np.ndarray:
        cr, sr = math.cos(r), math.sin(r)
        cp, sp = math.cos(p), math.sin(p)
        cy, sy = math.cos(y), math.sin(y)
        Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
        Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
        Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
        return Rz @ Ry @ Rx

    # Bout du gripper dans le repère link6 (bride), pour la collision.
    # URDF : gripper_base attaché à link6 en (0,-0.007,0.056) rpy=(π/2,0,0),
    # doigts ~0.10 m au-delà -> bout ≈ (0, -0.107, 0.056) dans le repère bride.
    # Conservateur : on l'utilise comme extrémité de la capsule gripper (R1 anti-sol
    # + R3 auto-collision). À valider visuellement dans Gazebo (pas de doigt sous le sol).
    _GRIPPER_TIP = np.array([0.0, -0.107, 0.056, 1.0])

    def _fk_points(self, angles: List[float]) -> List[np.ndarray]:
        """World XYZ de chaque origine de repère (base → bride) + bout du gripper."""
        M = np.eye(4)
        pts = [M[:3, 3].copy()]
        for (xyz, rpy), th in zip(self._JOINT_TF, angles):
            Torg = np.eye(4)
            Torg[:3, :3] = self._rpy_mat(*rpy)
            Torg[:3, 3] = xyz
            c, s = math.cos(th), math.sin(th)
            Rz = np.array([[c, -s, 0, 0], [s, c, 0, 0], [0, 0, 1, 0], [0, 0, 0, 1]])
            M = M @ Torg @ Rz
            pts.append(M[:3, 3].copy())
        pts.append((M @ self._GRIPPER_TIP)[:3].copy())   # 8ᵉ point = bout du gripper
        return pts

    # ARM SELF-COLLISION — capsule model (segment + radius per link).
    # Gazebo's mesh-based self_collide is unreliable (misses interpenetration),
    # so this software check is the ONLY guard. Each physical link is a capsule:
    # a centre-line segment with a radius; two links collide when the distance
    # between their segments is below the sum of their radii (+ safety margin).
    #
    # The FK has pts[1]==pts[2] (compound shoulder, zero translation), so the
    # DISTINCT skeleton is [pts0, pts1, pts3, pts4, pts5, pts6] -> 5 links:
    #   L0 base column, L1 upper-arm, L2 forearm, L3 wrist, L4 flange.
    # +1 capsule pour le gripper (6ᵉ lien : bride → bout du gripper). Rayon large
    # (~0.05) pour couvrir l'enveloppe du gripper (doigts écartés).
    LINK_RADII = [0.055, 0.045, 0.040, 0.036, 0.045, 0.050]
    SELF_MARGIN = 0.005
    # Non-adjacent link pairs to test. (L2,L4) forearm<->flange is EXCLUDED: those
    # two are separated only by the 0.084 m wrist and are naturally close when the
    # arm is straight (they form one wrist unit) — testing them false-rejects
    # extended poses. Real folds are caught by the proximal<->distal pairs below.
    # +3 paires testant le GRIPPER (lien 5) contre les liens éloignés (base, bras,
    # avant-bras). On NE teste PAS (3,5)/(4,5) : poignet+bride+gripper forment une
    # unité naturellement proche (même raison que l'exclusion (2,4)).
    SELF_PAIRS = [(0, 2), (0, 3), (0, 4), (1, 3), (1, 4), (0, 5), (1, 5), (2, 5)]

    @staticmethod
    def _seg_seg_dist(p1, p2, p3, p4) -> float:
        """Minimum distance between the 3-D segments [p1,p2] and [p3,p4]."""
        d1 = p2 - p1
        d2 = p4 - p3
        r = p1 - p3
        a = float(d1 @ d1)
        e = float(d2 @ d2)
        f = float(d2 @ r)
        if a <= 1e-9 and e <= 1e-9:
            return float(np.linalg.norm(p1 - p3))
        if a <= 1e-9:
            s = 0.0
            t = min(1.0, max(0.0, f / e))
        else:
            c = float(d1 @ r)
            if e <= 1e-9:
                t = 0.0
                s = min(1.0, max(0.0, -c / a))
            else:
                b = float(d1 @ d2)
                den = a * e - b * b
                s = min(1.0, max(0.0, (b * f - c * e) / den)) if den > 1e-9 else 0.0
                t = (b * s + f) / e
                if t < 0.0:
                    t = 0.0
                    s = min(1.0, max(0.0, -c / a))
                elif t > 1.0:
                    t = 1.0
                    s = min(1.0, max(0.0, (b - c) / a))
        cp1 = p1 + d1 * s
        cp2 = p3 + d2 * t
        return float(np.linalg.norm(cp1 - cp2))

    def _is_collision_free(self, angles: List[float]) -> bool:
        """Table-crash / base-collision check using the TRUE URDF kinematics.

        Samples the arm skeleton beyond the base column (shoulder → … → flange)
        and rejects any pose whose links dip below TABLE_CLEARANCE above the
        table top (z=0), or fold back into the base column. The base column
        (mount, z:0→0.162) is fixed and excluded.
        """
        pts = self._fk_points(angles)
        chain = pts[1:]                        # skip the fixed base column

        # R1: no arm point may drop below the table-clearance margin.
        for a, b in zip(chain[:-1], chain[1:]):
            for t in (0.0, 0.2, 0.4, 0.6, 0.8, 1.0):
                z = a[2] + (b[2] - a[2]) * t
                if z < self.TABLE_CLEARANCE:
                    return False

        # R2: distal links must not wrap back into the base column cylinder.
        BASE_RADIUS = 0.055
        BASE_HEIGHT = 0.16
        for p in chain[2:]:                    # elbow, wrist, flange
            if math.hypot(p[0], p[1]) < BASE_RADIUS and p[2] < BASE_HEIGHT:
                return False

        # R3: ARM SELF-COLLISION (capsule model). The real robot cannot fold a
        # link through another; such poses look broken and must never enter the
        # dataset. Build the DISTINCT skeleton (dropping the duplicate shoulder
        # point) -> 5 links, and reject any tested non-adjacent pair whose
        # segment distance is below the sum of their radii + margin.
        # pts[7] = bout du gripper -> 6ᵉ lien (bride → gripper).
        skel = [pts[0], pts[1], pts[3], pts[4], pts[5], pts[6], pts[7]]
        segs = [(skel[i], skel[i + 1]) for i in range(6)]
        for i, j in self.SELF_PAIRS:
            thr = self.LINK_RADII[i] + self.LINK_RADII[j] + self.SELF_MARGIN
            if self._seg_seg_dist(*segs[i], *segs[j]) < thr:
                return False

        return True

    def _command_joints(self, angles: List[float]):
        """Send the target pose to the JointTrajectoryController."""
        traj = JointTrajectory()
        traj.joint_names = list(self.JOINT_NAMES)
        point = JointTrajectoryPoint()
        point.positions = [float(a) for a in angles]
        sec = int(self.move_time)
        nanosec = int((self.move_time - sec) * 1e9)
        point.time_from_start = Duration(sec=sec, nanosec=nanosec)
        traj.points = [point]
        self.traj_pub.publish(traj)

    def _capture_and_save(self, target_angles: List[float]):
        # Check we have at least one image
        available = {cn for cn, img in self.current_images.items() if img is not None}
        if not available:
            self.get_logger().warn(
                f'[{self.sample_idx}] No images received — skipping'
            )
            self._collect_next()
            return

        # --- Settle verification --------------------------------------
        # Only record the pose if the robot actually REACHED the target.
        # A stuck / gravity-drooped robot would otherwise log a repeated
        # pose and create sharp spikes in the angle distribution.
        if self.current_joints:
            max_err = max(abs(a - t)
                          for a, t in zip(self.current_joints, target_angles))
            settled = max_err <= self.SETTLE_TOL
        else:
            max_err = float('inf')
            settled = False

        if not settled and self._settle_skips < self.max_settle_skips:
            self._settle_skips += 1
            self.n_skipped += 1
            self.get_logger().debug(
                f'[{self.sample_idx}] not settled (err={math.degrees(max_err):.1f}°) '
                f'— retry {self._settle_skips}/{self.max_settle_skips}'
            )
            # Re-command the SAME target and wait again (give it more time).
            self._command_joints(target_angles)
            timer = self.create_timer(
                self.settle_time,
                lambda: self._on_settle_timeout(timer, target_angles),
            )
            return

        # Settled (or gave up after max retries → use the actual readings).
        self._settle_skips = 0
        angles = self.current_joints if self.current_joints else target_angles

        # If the pose was FORCE-recorded (never settled), the arm may have
        # gravity-drooped BELOW the table-clearance margin even though the
        # commanded target was valid. Never record such a pose — it would put a
        # too-low / near-table configuration into the dataset. Discard it and
        # move on to a fresh pose.
        if not settled and not self._is_collision_free(list(angles)):
            self.n_skipped += 1
            self.get_logger().debug(
                f'[{self.sample_idx}] forced pose dips below clearance — discarded'
            )
            self._collect_next()
            return

        degs = [round(math.degrees(a), 2) for a in angles]

        # --- FRESHNESS GUARD (anti render-freeze) ---------------------------
        # When the camera render stalls, Gazebo keeps handing us the SAME frame,
        # so it would be logged against the NEW joint angles (frozen-arm data).
        # Detect it: each camera's current frame must DIFFER from the one we last
        # SAVED (CRC32). If any camera is identical, the render is frozen → skip
        # this pose. A persistent stall then yields 0 new images (a VISIBLE
        # failure) instead of silently corrupting the dataset.
        sigs = {
            c: (zlib.crc32(bytes(m.data)) if m is not None else None)
            for c, m in ((c, self.current_images.get(c)) for c in self.active_cameras)
        }
        frozen = [c for c in self.active_cameras
                  if sigs[c] is not None and sigs[c] == self._last_sig.get(c)]
        if frozen:
            self.n_skipped += 1
            self._frozen_skips += 1
            self.get_logger().warn(
                f'[{self.sample_idx}] FROZEN render on {frozen} '
                f'(frame identical to last saved) — pose SKIPPED '
                f'(frozen skips so far: {self._frozen_skips})'
            )
            self._collect_next()
            return

        # Save images from ALL active cameras for this pose
        with open(self.csv_path, 'a', newline='') as f:
            writer = csv.writer(f)
            for cam_name in self.active_cameras:
                img_msg = self.current_images.get(cam_name)
                if img_msg is None:
                    self.get_logger().debug(
                        f'[{self.sample_idx}] Missing {cam_name} image — skip view'
                    )
                    continue

                img_filename = f'{self.sample_idx:06d}.png'
                img_rel = f'images/{cam_name}/{img_filename}'
                img_path = os.path.join(self.output_dir, img_rel)

                noise_sigma = random.uniform(0, self.noise_stddev) if self.noise_stddev > 0 else 0
                self._save_image(img_msg, img_path, noise_sigma=noise_sigma)
                self._last_sig[cam_name] = sigs[cam_name]   # remember what we saved

                writer.writerow([
                    self.sample_idx,
                    *[round(a, 4) for a in angles],
                    *degs,
                    cam_name,
                    img_rel,
                ])

        self.n_recorded += 1
        if (self.sample_idx + 1) % 50 == 0 or self.sample_idx == 0:
            self.get_logger().info(
                f'📸 [{self.sample_idx + 1}/{self.num_samples}] '
                f'views={len(available)} angles(deg)={degs}'
            )

        self.sample_idx += 1
        self._collect_next()

    # ------------------------------------------------------------------
    # Image helper
    # ------------------------------------------------------------------
    @staticmethod
    def _save_image(img_msg: Image, path: str, noise_sigma: float = 0.0):
        """Save sensor_msgs/Image to PNG, optionally adding Gaussian noise."""
        try:
            from PIL import Image as PILImage

            h, w = img_msg.height, img_msg.width
            encoding = img_msg.encoding.lower()

            data = bytes(img_msg.data)
            arr = np.frombuffer(data, dtype=np.uint8).copy()

            if encoding in ('rgb8',):
                arr = arr.reshape((h, w, 3))
            elif encoding in ('bgr8',):
                arr = arr.reshape((h, w, 3))[:, :, ::-1].copy()
            elif encoding in ('rgba8',):
                arr = arr.reshape((h, w, 4))[:, :, :3].copy()
            elif encoding in ('bgra8',):
                arr = arr.reshape((h, w, 4))[:, :, 2::-1].copy()
            else:
                arr = arr.reshape((h, w, -1))
                if arr.shape[2] == 4:
                    arr = arr[:, :, :3].copy()

            # Domain randomization: sensor noise simulation
            if noise_sigma > 0:
                noise = np.random.normal(0, noise_sigma, arr.shape).astype(np.float32)
                arr = np.clip(arr.astype(np.float32) + noise, 0, 255).astype(np.uint8)

                if random.random() < 0.3:
                    shift = np.array([random.uniform(-15, 15),
                                      random.uniform(-10, 10),
                                      random.uniform(-15, 15)], dtype=np.float32)
                    arr = np.clip(arr.astype(np.float32) + shift, 0, 255).astype(np.uint8)

                if random.random() < 0.2:
                    rows, cols = arr.shape[:2]
                    Y, X = np.ogrid[:rows, :cols]
                    cy, cx = rows / 2, cols / 2
                    dist = np.sqrt((X - cx)**2 + (Y - cy)**2)
                    max_dist = np.sqrt(cx**2 + cy**2)
                    vignette = 1.0 - 0.3 * (dist / max_dist)**2
                    arr = np.clip(arr.astype(np.float32) * vignette[:, :, None],
                                  0, 255).astype(np.uint8)

            # Save the raw render — NO unsharp-mask sharpening. Strong sharpening
            # added halo artefacts that don't exist on the real cameras and broke
            # sim/real preprocessing parity (the real pipeline doesn't sharpen),
            # so we keep the image clean / camera-like. The joint angles are
            # recovered by the network from the overall arm shape + the 4 views,
            # not from sharpened seams.
            PILImage.fromarray(arr).save(path)

        except ImportError:
            with open(path.replace('.png', '.raw'), 'wb') as f:
                f.write(bytes(img_msg.data))


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticDataCollectorV3()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
