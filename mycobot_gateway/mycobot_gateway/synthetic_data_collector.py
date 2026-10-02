#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Synthetic Data Collector — 4 cameras, calibrated Arducam intrinsics.

For each robot pose: commands random joint angles, waits for settle,
then captures one image from each of the 4 Gazebo cameras simultaneously.

Intrinsic distribution (ChArUco calibration 28/04/2026):
  cam_0, cam_2 → real Arducam cam_0  (fx=525.671, fy=529.700)
  cam_1, cam_3 → real Arducam cam_3  (fx=496.308, fy=494.143)

Output layout
-------------
<output_dir>/
├── images/
│   ├── cam_0/   (front  view)  000000.png … 012499.png
│   ├── cam_1/   (left   view)  000000.png … 012499.png
│   ├── cam_2/   (back   view)  000000.png … 012499.png
│   └── cam_3/   (right  view)  000000.png … 012499.png
├── labels.csv                  one row per pose, all 4 cameras share it
├── cam_0_camera_settings.json
├── cam_1_camera_settings.json
├── cam_2_camera_settings.json
└── cam_3_camera_settings.json

num_samples = robot poses  (default 12 500 → 50 000 total images across 4 cams)

Usage
-----
  ros2 launch mycobot_gateway synthetic_data.launch.py
  ros2 launch mycobot_gateway synthetic_data.launch.py num_samples:=12500 \
      output_dir:=/mnt/data/synth_50k
"""

import csv
import json
import math
import os
import random
from typing import Dict, List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64


# ---------------------------------------------------------------------------
# Camera configuration
# ---------------------------------------------------------------------------
# Real Arducam intrinsics from ChArUco calibration (feature/calibration-cam)
_CAM0_K = dict(
    image_width=640, image_height=480,
    fx=525.671, fy=529.700, cx=317.729, cy=226.003,
    k1=0.0, k2=0.0, p1=0.0, p2=0.0, k3=0.0,
    rms_reprojection_error_px=0.674,
    calibration_source='ChArUco 6x9, 18 views, real Arducam cam_0, 28/04/2026',
)
_CAM3_K = dict(
    image_width=640, image_height=480,
    fx=496.308, fy=494.143, cx=313.374, cy=248.006,
    k1=0.0, k2=0.0, p1=0.0, p2=0.0, k3=0.0,
    rms_reprojection_error_px=0.678,
    calibration_source='ChArUco 6x9, 21 views, real Arducam cam_3, 28/04/2026',
)

# Virtual camera name → (ROS topic, camera settings dict)
CAMERAS: Dict[str, tuple] = {
    'cam_0': ('/synth_camera/image',   _CAM0_K),   # front,  cam_0 intrinsics
    'cam_1': ('/synth_camera_1/image', _CAM3_K),   # left,   cam_3 intrinsics
    'cam_2': ('/synth_camera_2/image', _CAM0_K),   # back,   cam_0 intrinsics
    'cam_3': ('/synth_camera_3/image', _CAM3_K),   # right,  cam_3 intrinsics
}


class SyntheticDataCollector(Node):
    """Collect (4× image, joint_angles) from Gazebo Harmonic."""

    JOINT_NAMES = [
        'joint2_to_joint1',
        'joint3_to_joint2',
        'joint4_to_joint3',
        'joint5_to_joint4',
        'joint6_to_joint5',
        'joint6output_to_joint6',
    ]
    JOINT_LIMITS = [
        (-2.96, 2.96),
        (-2.79, 2.79),
        (-2.79, 2.79),
        (-2.79, 2.79),
        (-2.96, 2.96),
        (-3.05, 3.05),
    ]

    def __init__(self):
        super().__init__('synthetic_data_collector')

        self.declare_parameter('num_samples', 12500)
        self.declare_parameter('output_dir', '/tmp/mycobot_synth_dataset')
        self.declare_parameter('settle_time', 1.0)
        self.declare_parameter('joint_limit_fraction', 0.75)

        self.num_samples   = self.get_parameter('num_samples').value
        self.output_dir    = self.get_parameter('output_dir').value
        self.settle_time   = self.get_parameter('settle_time').value
        self.limit_frac    = self.get_parameter('joint_limit_fraction').value

        # State
        self.current_images: Dict[str, Optional[Image]] = {k: None for k in CAMERAS}
        self.current_joints: Optional[List[float]] = None
        self.sample_idx = 0
        self.collecting = False

        # Publishers — one per joint
        self.joint_pubs = {}
        for jname in self.JOINT_NAMES:
            topic = f'/model/mycobot_320/joint/{jname}/cmd_pos'
            self.joint_pubs[jname] = self.create_publisher(Float64, topic, 10)

        # Subscribers — one per camera + joint_states
        img_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        for cam_name, (topic, _) in CAMERAS.items():
            self.create_subscription(
                Image, topic,
                lambda msg, cn=cam_name: self._image_cb(cn, msg),
                img_qos,
            )
        self.create_subscription(JointState, '/joint_states', self._joint_cb, 10)

        # Output directories
        self.cam_dirs: Dict[str, str] = {}
        for cam_name in CAMERAS:
            d = os.path.join(self.output_dir, 'images', cam_name)
            os.makedirs(d, exist_ok=True)
            self.cam_dirs[cam_name] = d

        # CSV header
        self.csv_path = os.path.join(self.output_dir, 'labels.csv')
        with open(self.csv_path, 'w', newline='') as f:
            csv.writer(f).writerow([
                'index',
                'j1_rad', 'j2_rad', 'j3_rad', 'j4_rad', 'j5_rad', 'j6_rad',
                'j1_deg', 'j2_deg', 'j3_deg', 'j4_deg', 'j5_deg', 'j6_deg',
            ])

        # Camera settings JSON (DREAM-compatible, one per virtual camera)
        for cam_name, (_, intrinsics) in CAMERAS.items():
            path = os.path.join(self.output_dir, f'{cam_name}_camera_settings.json')
            with open(path, 'w') as f:
                json.dump(intrinsics, f, indent=2)

        total_images = self.num_samples * len(CAMERAS)
        eta_h = self.num_samples * self.settle_time / 3600.0
        self.get_logger().info(
            f'\n'
            f'  Synthetic Data Collector — 4 cameras, calibrated Arducam intrinsics\n'
            f'  ---------------------------------------------------------------\n'
            f'  Poses        : {self.num_samples}\n'
            f'  Cameras      : {list(CAMERAS.keys())}\n'
            f'  Total images : {total_images}  ({total_images // 1000}k)\n'
            f'  Settle time  : {self.settle_time} s/pose\n'
            f'  ETA          : ~{eta_h:.1f} h\n'
            f'  Output       : {self.output_dir}\n'
            f'  Intrinsics   : cam_0/cam_2 use real Arducam cam_0 '
            f'(fx=525.671), cam_1/cam_3 use cam_3 (fx=496.308)\n'
        )

        self._startup_timer = self.create_timer(3.0, self._start_collection_once)

    # ------------------------------------------------------------------
    def _image_cb(self, cam_name: str, msg: Image):
        self.current_images[cam_name] = msg

    def _joint_cb(self, msg: JointState):
        if not msg.name:
            return
        angles = [0.0] * len(self.JOINT_NAMES)
        for i, jn in enumerate(self.JOINT_NAMES):
            if jn in msg.name:
                angles[i] = msg.position[list(msg.name).index(jn)]
        self.current_joints = angles

    # ------------------------------------------------------------------
    def _start_collection_once(self):
        if self.collecting:
            return
        self.collecting = True
        self._startup_timer.cancel()
        self.get_logger().info('Starting collection...')
        self._collect_next()

    def _collect_next(self):
        if self.sample_idx >= self.num_samples:
            total = self.num_samples * len(CAMERAS)
            self.get_logger().info(
                f'Collection complete! {self.num_samples} poses / {total} images → {self.output_dir}'
            )
            rclpy.shutdown()
            return

        target = self._random_joint_angles()
        self._command_joints(target)
        timer = self.create_timer(
            self.settle_time,
            lambda: self._on_settle(timer, target),
        )

    def _on_settle(self, timer, target):
        timer.cancel()
        self._capture_and_save(target)

    def _random_joint_angles(self) -> List[float]:
        angles = []
        for lo, hi in self.JOINT_LIMITS:
            span = (hi - lo) * self.limit_frac
            mid = (hi + lo) / 2.0
            angles.append(round(random.uniform(mid - span / 2, mid + span / 2), 4))
        return angles

    def _command_joints(self, angles: List[float]):
        for jname, angle in zip(self.JOINT_NAMES, angles):
            msg = Float64()
            msg.data = angle
            self.joint_pubs[jname].publish(msg)

    def _capture_and_save(self, target_angles: List[float]):
        missing = [k for k, v in self.current_images.items() if v is None]
        if missing:
            self.get_logger().warn(f'[{self.sample_idx}] Missing images from {missing} — skip')
            self._collect_next()
            return

        angles = self.current_joints if self.current_joints else target_angles
        fname = f'{self.sample_idx:06d}.png'

        for cam_name, img_msg in self.current_images.items():
            self._save_image(img_msg, os.path.join(self.cam_dirs[cam_name], fname))

        degs = [round(math.degrees(a), 2) for a in angles]
        with open(self.csv_path, 'a', newline='') as f:
            csv.writer(f).writerow([
                self.sample_idx,
                *[round(a, 4) for a in angles],
                *degs,
            ])

        if self.sample_idx % 500 == 0 or self.sample_idx < 3:
            pct = 100.0 * self.sample_idx / self.num_samples
            self.get_logger().info(
                f'[{self.sample_idx + 1}/{self.num_samples}] ({pct:.1f}%) '
                f'joints(deg)={[round(d, 1) for d in degs]}'
            )

        self.sample_idx += 1
        self._collect_next()

    # ------------------------------------------------------------------
    @staticmethod
    def _save_image(img_msg: Image, path: str):
        try:
            import numpy as np
            from PIL import Image as PILImage

            h, w = img_msg.height, img_msg.width
            data = bytes(img_msg.data)
            enc = img_msg.encoding.lower()

            if enc == 'rgb8':
                arr = np.frombuffer(data, dtype=np.uint8).reshape((h, w, 3))
            elif enc == 'bgr8':
                arr = np.frombuffer(data, dtype=np.uint8).reshape((h, w, 3))[:, :, ::-1]
            elif enc == 'rgba8':
                arr = np.frombuffer(data, dtype=np.uint8).reshape((h, w, 4))[:, :, :3]
            elif enc == 'bgra8':
                arr = np.frombuffer(data, dtype=np.uint8).reshape((h, w, 4))[:, :, 2::-1]
            else:
                arr = np.frombuffer(data, dtype=np.uint8).reshape((h, w, -1))
                if arr.shape[2] == 4:
                    arr = arr[:, :, :3]

            PILImage.fromarray(arr).save(path)

        except ImportError:
            with open(path.replace('.png', '.raw'), 'wb') as f:
                f.write(bytes(img_msg.data))


def main(args=None):
    rclpy.init(args=args)
    node = SyntheticDataCollector()
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
