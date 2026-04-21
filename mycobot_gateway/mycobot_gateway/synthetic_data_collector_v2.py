#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Enhanced Synthetic Data Collector v2 for MyCobot 320 Pi Pose Estimation.

Improvements over v1:
- Multi-camera support (front, right, left, top)
- Domain randomization (Gazebo light intensity/direction via gz service)
- Per-image Gaussian noise injection at capture time
- Higher throughput (parallel image capture from all cameras)
- Camera-view label in CSV for multi-view training

Usage (standalone):
    ros2 run mycobot_gateway synthetic_data_collector_v2 \
        --ros-args -p num_samples:=5000 -p output_dir:=/tmp/synth_v2 \
        -p multi_view:=true -p domain_randomize:=true

Or via launch:
    ros2 launch mycobot_gateway synthetic_data_v2.launch.py num_samples:=5000
"""

import csv
import math
import os
import random
import subprocess
import time
from typing import Dict, List, Optional

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64


class SyntheticDataCollectorV2(Node):
    """Enhanced data collector with domain randomization + multi-view."""

    JOINT_NAMES = [
        'joint2_to_joint1',
        'joint3_to_joint2',
        'joint4_to_joint3',
        'joint5_to_joint4',
        'joint6_to_joint5',
        'joint6output_to_joint6',
    ]

    # Official MyCobot 320 Pi joint limits (from elephantrobotics URDF)
    # J1 ±167.9°  J2 ±134.6°  J3 ±145.0°  J4 ±145.0°  J5 ±167.9°  J6 ±180.0°
    JOINT_LIMITS = [
        (-2.93, 2.93),   # J1 – base rotation
        (-2.35, 2.35),   # J2 – shoulder
        (-2.53, 2.53),   # J3 – elbow
        (-2.53, 2.53),   # J4 – wrist 1
        (-2.93, 2.93),   # J5 – wrist 2
        (-3.14, 3.14),   # J6 – wrist 3 (flange)
    ]

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
        super().__init__('synthetic_data_collector_v2')

        # ---------- parameters ----------
        self.declare_parameter('num_samples', 5000)
        self.declare_parameter('output_dir', '/tmp/mycobot_synth_v2')
        self.declare_parameter('settle_time', 1.5)
        self.declare_parameter('joint_limit_fraction', 0.85)
        self.declare_parameter('multi_view', True)
        self.declare_parameter('domain_randomize', True)
        self.declare_parameter('noise_stddev', 5.0)  # pixel noise σ (0–255 scale)
        self.declare_parameter('world_name', 'randomized')

        self.num_samples = self.get_parameter('num_samples').value
        self.output_dir = self.get_parameter('output_dir').value
        self.settle_time = self.get_parameter('settle_time').value
        self.limit_frac = self.get_parameter('joint_limit_fraction').value
        self.multi_view = self.get_parameter('multi_view').value
        self.domain_randomize = self.get_parameter('domain_randomize').value
        self.noise_stddev = self.get_parameter('noise_stddev').value
        self.world_name = self.get_parameter('world_name').value

        # ---------- state ----------
        self.current_images: Dict[str, Optional[Image]] = {}
        self.current_joints: Optional[List[float]] = None
        self.sample_idx = 0
        self.collecting = False

        # ---------- which cameras to use ----------
        if self.multi_view:
            self.active_cameras = list(self.CAMERA_TOPICS.keys())
        else:
            self.active_cameras = ['front']

        # ---------- publishers (one per joint) ----------
        self.joint_pubs = {}
        for jname in self.JOINT_NAMES:
            topic = f'/model/mycobot_320/joint/{jname}/cmd_pos'
            self.joint_pubs[jname] = self.create_publisher(Float64, topic, 10)

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
            f'🎬 Synthetic Data Collector v2 initialised\n'
            f'   Samples       : {self.num_samples}\n'
            f'   Output        : {self.output_dir}\n'
            f'   Settle        : {self.settle_time}s\n'
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
    # Domain randomization
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
        """Aggressively randomize lighting, clutter colours, and surfaces."""
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
        """Build and fire a gz service light_config command."""
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

        req = ', '.join(parts)
        cmd = (
            f'gz service -s /world/{self.world_name}/light_config '
            f'--reqtype gz.msgs.Light --reptype gz.msgs.Boolean '
            f'--timeout 300 --req "{req}"'
        )
        subprocess.Popen(cmd, shell=True,
                         stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)

    def _randomize_lights(self):
        """Randomize direction, colour, and position of all scene lights."""
        # Sun — varying direction + warm/cool colour temperature
        self._gz_light_cmd(
            'sun',
            direction=(random.uniform(-1.0, 0.0),
                       random.uniform(-0.5, 0.5),
                       random.uniform(-1.0, -0.3)),
            diffuse=(random.uniform(0.4, 1.0),
                     random.uniform(0.4, 1.0),
                     random.uniform(0.35, 0.95)),
        )

        # Fill light — opposite side, variable colour
        self._gz_light_cmd(
            'fill_light',
            direction=(random.uniform(0.0, 1.0),
                       random.uniform(-0.5, 0.5),
                       random.uniform(-0.9, -0.3)),
            diffuse=(random.uniform(0.2, 0.6),
                     random.uniform(0.2, 0.6),
                     random.uniform(0.2, 0.7)),
        )

        # Back light — subtle rim
        self._gz_light_cmd(
            'back_light',
            direction=(random.uniform(-0.5, 1.0),
                       random.uniform(-0.5, 0.5),
                       random.uniform(-0.8, -0.3)),
            diffuse=(random.uniform(0.1, 0.4),
                     random.uniform(0.1, 0.4),
                     random.uniform(0.1, 0.5)),
        )

        # Point lights — random position + colour
        for name in ('warm_point', 'cool_point', 'overhead_spot'):
            self._gz_light_cmd(
                name,
                pose=(random.uniform(-1.0, 1.5),
                      random.uniform(-1.0, 1.0),
                      random.uniform(0.6, 2.5)),
                diffuse=(random.uniform(0.15, 0.8),
                         random.uniform(0.15, 0.8),
                         random.uniform(0.1, 0.7)),
            )

    def _randomize_material_colours(self):
        """Randomize clutter and surface material colours via gz service."""
        # Randomize clutter objects — full RGB range
        for model_name in self.CLUTTER_MODELS:
            r = random.uniform(0.1, 0.95)
            g = random.uniform(0.1, 0.95)
            b = random.uniform(0.1, 0.95)
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

        # Randomize surfaces — more subtle
        for model_name in self.SURFACE_MODELS:
            base = random.uniform(0.2, 0.7)
            # Add slight colour tint
            r = base + random.uniform(-0.1, 0.1)
            g = base + random.uniform(-0.1, 0.1)
            b = base + random.uniform(-0.1, 0.1)
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

    # ------------------------------------------------------------------
    # Collection loop
    # ------------------------------------------------------------------
    def _start_collection_once(self):
        if self.collecting:
            return
        self.collecting = True
        self._startup_timer.cancel()
        self.get_logger().info('🚀 Starting enhanced data collection…')
        self._collect_next()

    def _collect_next(self):
        if self.sample_idx >= self.num_samples:
            total_images = self.sample_idx * len(self.active_cameras)
            self.get_logger().info(
                f'✅ Collection complete! {self.num_samples} poses, '
                f'{total_images} images saved to {self.output_dir}'
            )
            rclpy.shutdown()
            return

        # 1. Domain randomization (lighting)
        self._randomize_scene()

        # 2. Random joint command
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
        """Generate physically-valid random joint angles.

        Applies ``joint_limit_fraction`` to the official limits and then
        rejects configurations that would cause self-collision on the
        real MyCobot 320 Pi.  Tries up to 50 times before falling back.
        """
        for _ in range(50):
            angles = self._sample_raw_angles()
            if self._is_collision_free(angles):
                return angles
        # Fallback: return a conservative home-ish pose with slight randomness
        return [random.uniform(-0.3, 0.3) for _ in range(6)]

    def _sample_raw_angles(self) -> List[float]:
        """Sample angles within ``limit_frac`` of the official limits."""
        angles = []
        for lo, hi in self.JOINT_LIMITS:
            span = (hi - lo) * self.limit_frac
            mid = (hi + lo) / 2.0
            a = random.uniform(mid - span / 2, mid + span / 2)
            angles.append(round(a, 4))
        return angles

    def _is_collision_free(self, angles: List[float]) -> bool:
        """Approximate self-collision check via forward-kinematics.

        Computes elbow and wrist positions using the planar arm formed
        by J1-J2-J3 and rejects poses where the wrist or end-effector
        dips below the table / into the base, or where the forearm
        doubles back close to the base column.

        This is intentionally conservative: it rejects ~15-20 % of
        random samples, eliminating unreachable / physically-dangerous
        poses that the real robot would refuse or collide on.
        """
        j1, j2, j3, j4, j5, j6 = angles

        # --- Planar kinematics in the J1-rotation plane -----------------
        # After J1 rotation, the arm lies in a vertical plane.
        # Shoulder (J2 axis) is at height L1 above the base.
        # We compute 2-D coordinates (r, z) where r = radial from base axis.
        #
        # Note: in the MyCobot 320 Pi kinematic convention J2=0 is horizontal
        # and J3=0 extends the forearm in-line with the upper arm.

        # Elbow position
        elbow_r = self.L2 * math.cos(j2)
        elbow_z = self.L1 + self.L2 * math.sin(j2)

        # Wrist position (end of forearm)
        wrist_r = elbow_r + self.L3 * math.cos(j2 + j3)
        wrist_z = elbow_z + self.L3 * math.sin(j2 + j3)

        # End-effector (approx, adding wrist offset vertically)
        ee_r = wrist_r + self.L4 * math.cos(j2 + j3 + j4)
        ee_z = wrist_z + self.L4 * math.sin(j2 + j3 + j4)

        # ---- Rejection rules ----

        # R1: End-effector or wrist below the table surface (z < 0.02 m)
        #     The real robot base sits on the table; anything below z=0
        #     means the gripper would crash into the table.
        TABLE_CLEARANCE = 0.02
        if wrist_z < TABLE_CLEARANCE or ee_z < TABLE_CLEARANCE:
            return False

        # R2: Wrist too close to the base column (r < 0.05 m)
        #     and not high enough to clear the base housing.
        BASE_RADIUS = 0.06   # approximate base housing radius
        BASE_HEIGHT = 0.20   # height of the base enclosure
        if abs(wrist_r) < BASE_RADIUS and wrist_z < BASE_HEIGHT:
            return False
        if abs(ee_r) < BASE_RADIUS and ee_z < BASE_HEIGHT:
            return False

        # R3: Elbow below the table
        if elbow_z < TABLE_CLEARANCE:
            return False

        # R4: Arm fully folded back (J2+J3 summing to bring the forearm
        #     inside the upper arm envelope) — reject extreme fold-back
        fold_angle = j2 + j3
        if fold_angle < -3.8 or fold_angle > 3.8:
            return False

        return True

    def _command_joints(self, angles: List[float]):
        for jname, angle in zip(self.JOINT_NAMES, angles):
            msg = Float64()
            msg.data = angle
            self.joint_pubs[jname].publish(msg)

    def _capture_and_save(self, target_angles: List[float]):
        # Check we have at least one image
        available = {cn for cn, img in self.current_images.items() if img is not None}
        if not available:
            self.get_logger().warn(
                f'[{self.sample_idx}] No images received — skipping'
            )
            self._collect_next()
            return

        # Use actual joint readings if available
        if self.current_joints:
            angles = self.current_joints
        else:
            angles = target_angles

        degs = [round(math.degrees(a), 2) for a in angles]

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

                # Add noise if enabled
                noise_sigma = random.uniform(0, self.noise_stddev) if self.noise_stddev > 0 else 0
                self._save_image(img_msg, img_path, noise_sigma=noise_sigma)

                writer.writerow([
                    self.sample_idx,
                    *[round(a, 4) for a in angles],
                    *degs,
                    cam_name,
                    img_rel,
                ])

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

                # Randomly apply additional sensor effects (30% chance each)
                if random.random() < 0.3:
                    # Slight colour temperature shift
                    shift = np.array([random.uniform(-15, 15),
                                      random.uniform(-10, 10),
                                      random.uniform(-15, 15)], dtype=np.float32)
                    arr = np.clip(arr.astype(np.float32) + shift, 0, 255).astype(np.uint8)

                if random.random() < 0.2:
                    # Vignetting effect (darken corners)
                    from PIL import ImageFilter
                    rows, cols = arr.shape[:2]
                    Y, X = np.ogrid[:rows, :cols]
                    cy, cx = rows / 2, cols / 2
                    dist = np.sqrt((X - cx)**2 + (Y - cy)**2)
                    max_dist = np.sqrt(cx**2 + cy**2)
                    vignette = 1.0 - 0.3 * (dist / max_dist)**2
                    arr = np.clip(arr.astype(np.float32) * vignette[:, :, None],
                                  0, 255).astype(np.uint8)

            PILImage.fromarray(arr).save(path)

        except ImportError:
            with open(path.replace('.png', '.raw'), 'wb') as f:
                f.write(bytes(img_msg.data))


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticDataCollectorV2()
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
