#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
DREAM Inference ROS2 Node for MyCobot 320 Pi.

Subscribes to Gazebo camera images, runs DREAM keypoint detection,
solves PnP for robot pose estimation, and publishes results.

Published Topics:
    /dream/keypoints        (std_msgs/Float64MultiArray) — [u0,v0,valid0,...,u6,v6,valid6,
                                                             img_stamp_sec, img_stamp_nanosec,
                                                             inference_ms] — 24 floats. The
                                                             trailing 3 let a downstream
                                                             consumer compute true image->angles
                                                             latency instead of guessing from its
                                                             own unrelated camera subscription.
    /dream/pose             (geometry_msgs/PoseStamped)  — estimated robot base pose
    /dream/belief_image     (sensor_msgs/Image)          — debug visualization
    /dream/status           (std_msgs/String)            — detection status

Subscribed Topics:
    /synth_camera/image     (sensor_msgs/Image)          — Gazebo front camera

Parameters:
    model_path    : str  — Path to best_network.pth
    config_path   : str  — Path to best_network.yaml  
    camera_topic  : str  — Camera image topic
    publish_rate  : float — Max inference rate (Hz)
    visualize     : bool — Publish debug belief map overlay
"""

import os
import sys
import time
import math
from typing import List, Optional, Tuple

import cv2
import numpy as np
import torch
from PIL import Image as PILImage

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray, String

# colcon --symlink-install runs this module from a symlink under build/; resolve
# it to the real source path first, or the three dirname()s below land in
# build/ (which has no training/ dir) instead of the actual repo root.
_REAL_FILE = os.path.realpath(__file__)

# Add dream module path
DREAM_DIR = os.path.dirname(_REAL_FILE)
sys.path.insert(0, DREAM_DIR)

# Add training/dream directory for mycobot_fk imports
_TRAINING_DREAM = os.path.join(
    os.path.dirname(os.path.dirname(os.path.dirname(_REAL_FILE))),
    'training', 'dream',
)
if os.path.isdir(_TRAINING_DREAM):
    sys.path.insert(0, _TRAINING_DREAM)

# Also try well-known workspace location
_WORKSPACE_DREAM = '/home/genji/ros_jazzy/src/mycobot_R6A/training/dream'
if os.path.isdir(_WORKSPACE_DREAM) and _WORKSPACE_DREAM not in sys.path:
    sys.path.insert(0, _WORKSPACE_DREAM)

DREAM_REPO = '/tmp/DREAM'
if os.path.isdir(DREAM_REPO):
    sys.path.insert(0, DREAM_REPO)

# Import DREAM library
try:
    import dream
    DREAM_AVAILABLE = True
except ImportError:
    DREAM_AVAILABLE = False

from mycobot_fk import (
    forward_kinematics,
    GAZEBO_INTRINSICS,
    KEYPOINT_NAMES,
)

# /joint_states order → [j1..j6] (same convention as calibrate_hand_eye_node)
JOINT_NAMES = [
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
]


class DreamInferenceNode(Node):
    """ROS2 node for real-time DREAM keypoint inference."""

    def __init__(self):
        super().__init__('dream_inference_node')

        # ── Parameters ──
        self.declare_parameter('model_name', 'vgg_weighted_e50')
        self.declare_parameter('camera_topic', '/synth_camera/image')
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('visualize', True)
        self.declare_parameter('min_keypoints_pnp', 4)
        # Préfixe des topics publiés — paramétrable pour lancer une instance
        # par caméra sans collision (ex. /dream_svpro). Défaut = legacy /dream.
        self.declare_parameter('output_prefix', '/dream')

        _model_name = self.get_parameter('model_name').value
        # Prefer the checkout this node actually runs from (symlink-install
        # source tree); _WORKSPACE_DREAM is a fallback for a specific machine
        # layout and can point at a stale checkout with different checkpoints.
        _ckpt_root = _TRAINING_DREAM if os.path.isdir(_TRAINING_DREAM) else _WORKSPACE_DREAM
        _ckpt_dir = os.path.join(_ckpt_root, 'checkpoints_dream', _model_name)
        self.model_path = os.path.join(_ckpt_dir, 'best_network.pth')
        self.config_path = os.path.join(_ckpt_dir, 'best_network.yaml')
        self.get_logger().info(f'🔧 Model name: {_model_name}')
        self.get_logger().info(f'   Checkpoint dir: {_ckpt_dir}')
        self.camera_topic = self.get_parameter('camera_topic').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.visualize = self.get_parameter('visualize').value
        self.min_kp_pnp = self.get_parameter('min_keypoints_pnp').value
        prefix = self.get_parameter('output_prefix').value.rstrip('/')

        # ── Camera intrinsics (from Gazebo config) ──
        self.camera_K = GAZEBO_INTRINSICS.copy()

        # ── Load DREAM network ──
        self.dream_network = None
        self._load_model()

        # ── Live joint state → FK 3D keypoints for PnP ──
        # DREAM needs FK at the ACTUAL joint config, not the home pose.
        self.latest_joint_q = None

        # ── State ──
        self.last_inference_time = 0.0
        self.min_interval = 1.0 / self.publish_rate
        self.latest_image_msg = None
        self.inference_count = 0
        self.detection_count = 0

        # ── Publishers ──
        self.pub_keypoints = self.create_publisher(
            Float64MultiArray, f'{prefix}/keypoints', 10)
        self.pub_pose = self.create_publisher(
            PoseStamped, f'{prefix}/pose', 10)
        self.pub_status = self.create_publisher(
            String, f'{prefix}/status', 10)
        if self.visualize:
            self.pub_belief = self.create_publisher(
                Image, f'{prefix}/belief_image', 10)

        # ── Subscriber ──
        img_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.create_subscription(
            Image, self.camera_topic, self._image_callback, img_qos)
        self.create_subscription(
            JointState, '/joint_states', self._joint_callback, 10)

        # ── Inference timer ──
        self.create_timer(self.min_interval, self._inference_tick)

        self.get_logger().info(
            f'🧠 DREAM Inference Node started\n'
            f'   Model     : {self.model_path}\n'
            f'   Camera    : {self.camera_topic}\n'
            f'   Rate      : {self.publish_rate} Hz\n'
            f'   Visualize : {self.visualize}'
        )

    # ── Model loading ──────────────────────────────────────────
    def _load_model(self):
        """Load DREAM network from checkpoint."""
        if not DREAM_AVAILABLE:
            self.get_logger().error('DREAM library not found! Install from /tmp/DREAM')
            return

        if not os.path.isfile(self.model_path):
            self.get_logger().error(f'Model not found: {self.model_path}')
            return

        try:
            # Load config
            config_path = self.config_path
            if not os.path.isfile(config_path):
                # Try deriving from model path
                config_path = self.model_path.replace('.pth', '.yaml')

            self.get_logger().info(f'Loading DREAM config: {config_path}')
            from ruamel.yaml import YAML
            yaml_parser = YAML(typ='safe')
            with open(config_path, 'r') as f:
                dream_config = yaml_parser.load(f)

            # Force CPU-friendly config (will move to GPU later if available)
            dream_config['training']['platform']['gpu_ids'] = [0]

            # Create network
            arch_type = dream_config['architecture']['type']
            n_keypoints = len(dream_config['manipulator']['keypoints'])
            net_input_res = dream_config['training']['config']['net_input_resolution']
            net_output_res = dream_config['training']['config']['net_output_resolution']

            self.get_logger().info(
                f'  Architecture: {arch_type}, keypoints: {n_keypoints}, '
                f'input: {net_input_res}, output: {net_output_res}'
            )

            self.dream_network = dream.create_network_from_config_data(dream_config)
            map_loc = 'cuda:0' if torch.cuda.is_available() else 'cpu'
            self.dream_network.model.load_state_dict(
                torch.load(self.model_path, map_location=map_loc, weights_only=True)
            )
            self.dream_network.enable_evaluation()

            # Move to GPU if available
            if torch.cuda.is_available():
                self.dream_network.model = self.dream_network.model.cuda()
                self.get_logger().info('  Model on GPU ✓')

            self.get_logger().info('✅ DREAM model loaded successfully')

        except Exception as e:
            self.get_logger().error(f'Failed to load DREAM model: {e}')
            import traceback
            traceback.print_exc()
            self.dream_network = None

    def _joint_callback(self, msg: JointState):
        """Cache the current joint configuration (radians, [j1..j6])."""
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            self.latest_joint_q = np.array(
                [name_to_pos[j] for j in JOINT_NAMES], dtype=np.float64)
        except KeyError:
            pass  # partial /joint_states (e.g. gripper-only) — keep last good q

    def _current_keypoints_3d(self) -> Optional[np.ndarray]:
        """FK 3D keypoints at the CURRENT joint config, or None if unknown."""
        if self.latest_joint_q is None:
            return None
        positions, _ = forward_kinematics(self.latest_joint_q)
        return np.array([positions[name] for name in KEYPOINT_NAMES],
                        dtype=np.float64)

    # ── Image callback ─────────────────────────────────────────
    def _image_callback(self, msg: Image):
        """Store latest image for processing."""
        self.latest_image_msg = msg

    def _msg_to_numpy(self, msg: Image) -> np.ndarray:
        """Convert sensor_msgs/Image to numpy RGB array."""
        h, w = msg.height, msg.width
        encoding = msg.encoding.lower()
        data = bytes(msg.data)
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

        return arr

    # ── Inference loop ─────────────────────────────────────────
    def _inference_tick(self):
        """Run DREAM inference on latest image."""
        if self.dream_network is None or self.latest_image_msg is None:
            return

        now = time.time()
        if now - self.last_inference_time < self.min_interval:
            return
        self.last_inference_time = now

        msg = self.latest_image_msg
        self.latest_image_msg = None  # consume

        try:
            # Convert to RGB numpy
            image_rgb = self._msg_to_numpy(msg)
            
            # Convert to PIL for DREAM
            pil_image = PILImage.fromarray(image_rgb)

            # Run DREAM inference (debug=True to get belief maps). Timed
            # separately from the rest of _inference_tick (PnP, publishing,
            # visualization) so a downstream consumer can tell the NN forward
            # pass apart from the angle-solver step it runs afterward.
            t_inf_start = time.time()
            result = self.dream_network.keypoints_from_image(
                pil_image, debug=True
            )
            inference_ms = (time.time() - t_inf_start) * 1000.0
            detected_kp = result["detected_keypoints"]
            belief_maps = result.get("belief_maps", None)

            self.inference_count += 1

            # Parse keypoints. DREAM's own "not detected" sentinel is
            # [-999.999, -999.999] in net-output space (dream/network.py); by
            # the time it's rescaled to raw image pixels it lands somewhere
            # far outside the frame, but is never NaN — so an in-bounds check
            # is required, not just a NaN check, or "not found" reads as a
            # confident detection at (-4720, -4800).
            img_h, img_w = image_rgb.shape[:2]
            kp_2d = []
            kp_valid = []
            for kp in detected_kp:
                if kp is not None and len(kp) == 2:
                    u, v = float(kp[0]), float(kp[1])
                    if not (np.isnan(u) or np.isnan(v)) and 0 <= u < img_w and 0 <= v < img_h:
                        kp_2d.append([u, v])
                        kp_valid.append(True)
                        continue
                kp_2d.append([0.0, 0.0])
                kp_valid.append(False)

            n_detected = sum(kp_valid)

            # Publish keypoints, with the SOURCE image's own stamp + the
            # measured inference time appended — msg.header.stamp is set by
            # camera_publisher_tour from its own wall clock, same epoch as a
            # downstream consumer's time.time() on the same host, so this is
            # what lets that consumer compute true end-to-end latency instead
            # of using the arrival time of whatever camera frame happens to
            # be newest when it gets around to reading /dream/keypoints.
            kp_msg = Float64MultiArray()
            flat = []
            for uv, valid in zip(kp_2d, kp_valid):
                flat.extend([uv[0], uv[1], 1.0 if valid else 0.0])
            flat.extend([
                float(msg.header.stamp.sec),
                float(msg.header.stamp.nanosec),
                inference_ms,
            ])
            kp_msg.data = flat
            self.pub_keypoints.publish(kp_msg)

            # Status
            status_msg = String()

            # Solve PnP if enough keypoints AND we know the joint config
            kp_3d = self._current_keypoints_3d()
            if kp_3d is None:
                status_msg.data = f'NO_JOINTS|kp={n_detected}/7'
            elif n_detected >= self.min_kp_pnp:
                self.detection_count += 1
                success, rvec, tvec = self._solve_pnp(kp_2d, kp_valid, kp_3d)

                if success:
                    # Publish pose
                    pose_msg = self._rvec_tvec_to_pose(rvec, tvec, msg.header)
                    self.pub_pose.publish(pose_msg)
                    status_msg.data = (
                        f'OK|kp={n_detected}/7|'
                        f'tx={tvec[0][0]:.3f}|ty={tvec[1][0]:.3f}|tz={tvec[2][0]:.3f}'
                    )
                else:
                    status_msg.data = f'PNP_FAILED|kp={n_detected}/7'
            else:
                status_msg.data = f'LOW_KP|kp={n_detected}/7'

            self.pub_status.publish(status_msg)

            # Publish visualization
            if self.visualize and belief_maps is not None:
                self._publish_belief_overlay(image_rgb, belief_maps, kp_2d, kp_valid, msg)

            # Log periodically
            if self.inference_count % 20 == 0:
                det_rate = (self.detection_count / self.inference_count * 100
                            if self.inference_count > 0 else 0)
                self.get_logger().info(
                    f'📊 Inference #{self.inference_count}: '
                    f'{n_detected}/7 kp, detection rate: {det_rate:.0f}%'
                )

        except Exception as e:
            self.get_logger().error(f'Inference error: {e}')
            import traceback
            traceback.print_exc()

    # ── PnP solving ────────────────────────────────────────────
    def _solve_pnp(
        self,
        kp_2d: List[List[float]],
        kp_valid: List[bool],
        kp_3d: np.ndarray,
    ) -> Tuple[bool, Optional[np.ndarray], Optional[np.ndarray]]:
        """Solve PnP: DREAM 2D keypoints ↔ FK 3D keypoints (current joints)."""
        pts_2d = []
        pts_3d = []
        for i, (uv, valid) in enumerate(zip(kp_2d, kp_valid)):
            if valid:
                pts_2d.append(uv)
                pts_3d.append(kp_3d[i])

        pts_2d = np.array(pts_2d, dtype=np.float64)
        pts_3d = np.array(pts_3d, dtype=np.float64)

        if len(pts_2d) < 4:
            return False, None, None

        try:
            success, rvec, tvec = cv2.solvePnP(
                pts_3d, pts_2d,
                self.camera_K, None,
                flags=cv2.SOLVEPNP_EPNP,
            )
            if success:
                # Refine
                rvec, tvec = cv2.solvePnPRefineLM(
                    pts_3d, pts_2d, self.camera_K, None, rvec, tvec
                )
            return success, rvec, tvec
        except Exception:
            return False, None, None

    def _rvec_tvec_to_pose(self, rvec, tvec, header) -> PoseStamped:
        """Convert OpenCV rvec/tvec to PoseStamped."""
        pose = PoseStamped()
        pose.header = header
        pose.header.frame_id = 'camera_optical'

        # Translation
        pose.pose.position.x = float(tvec[0][0])
        pose.pose.position.y = float(tvec[1][0])
        pose.pose.position.z = float(tvec[2][0])

        # Rotation: rodrigues → rotation matrix → quaternion
        R, _ = cv2.Rodrigues(rvec)
        quat = self._rotation_matrix_to_quaternion(R)
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        return pose

    @staticmethod
    def _rotation_matrix_to_quaternion(R: np.ndarray) -> Tuple[float, float, float, float]:
        """Convert 3x3 rotation matrix to quaternion (x, y, z, w)."""
        trace = R[0, 0] + R[1, 1] + R[2, 2]
        if trace > 0:
            s = 0.5 / np.sqrt(trace + 1.0)
            w = 0.25 / s
            x = (R[2, 1] - R[1, 2]) * s
            y = (R[0, 2] - R[2, 0]) * s
            z = (R[1, 0] - R[0, 1]) * s
        elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
            w = (R[2, 1] - R[1, 2]) / s
            x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s
            z = (R[0, 2] + R[2, 0]) / s
        elif R[1, 1] > R[2, 2]:
            s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
            w = (R[0, 2] - R[2, 0]) / s
            x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s
            z = (R[1, 2] + R[2, 1]) / s
        else:
            s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
            w = (R[1, 0] - R[0, 1]) / s
            x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s
            z = 0.25 * s
        return (x, y, z, w)

    # ── Visualization ──────────────────────────────────────────
    def _publish_belief_overlay(self, image_rgb, belief_maps, kp_2d, kp_valid, original_msg):
        """Publish debug overlay with keypoints drawn on the image."""
        vis = image_rgb.copy()
        h, w = vis.shape[:2]

        # Draw keypoints
        colors = [
            (255, 0, 0), (0, 255, 0), (0, 0, 255),
            (255, 255, 0), (255, 0, 255), (0, 255, 255),
            (255, 128, 0),
        ]
        for i, (uv, valid) in enumerate(zip(kp_2d, kp_valid)):
            if valid:
                u, v = int(round(uv[0])), int(round(uv[1]))
                if 0 <= u < w and 0 <= v < h:
                    color = colors[i % len(colors)]
                    cv2.circle(vis, (u, v), 5, color, -1)
                    cv2.putText(vis, KEYPOINT_NAMES[i].replace('mycobot320_', ''),
                                (u + 7, v - 5), cv2.FONT_HERSHEY_SIMPLEX,
                                0.35, color, 1)

        # Convert back to ROS Image
        out_msg = Image()
        out_msg.header = original_msg.header
        out_msg.height = h
        out_msg.width = w
        out_msg.encoding = 'rgb8'
        out_msg.step = w * 3
        out_msg.data = vis.tobytes()
        self.pub_belief.publish(out_msg)


def main(args=None):
    rclpy.init(args=args)
    node = DreamInferenceNode()
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
