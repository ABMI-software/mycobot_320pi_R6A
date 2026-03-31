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

    JOINT_LIMITS = [
        (-2.96, 2.96),
        (-2.79, 2.79),
        (-2.79, 2.79),
        (-2.79, 2.79),
        (-2.96, 2.96),
        (-3.05, 3.05),
    ]

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
    def _randomize_scene(self):
        """Randomize lighting via gz service calls (best-effort)."""
        if not self.domain_randomize:
            return

        try:
            # Randomize sun direction
            dx = random.uniform(-1.0, 0.0)
            dy = random.uniform(-0.5, 0.5)
            dz = random.uniform(-1.0, -0.3)
            # Randomize sun intensity
            r = random.uniform(0.5, 1.0)
            g = random.uniform(0.5, 1.0)
            b = random.uniform(0.45, 0.95)

            # Use gz service to update light (Gazebo Harmonic)
            cmd = (
                f'gz service -s /world/{self.world_name}/light_config '
                f'--reqtype gz.msgs.Light '
                f'--reptype gz.msgs.Boolean '
                f'--timeout 500 '
                f'--req "name: \\"sun\\", '
                f'direction: {{x: {dx}, y: {dy}, z: {dz}}}, '
                f'diffuse: {{r: {r}, g: {g}, b: {b}, a: 1.0}}"'
            )
            subprocess.Popen(cmd, shell=True,
                             stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL)

            # Also randomize the warm point light position
            px = random.uniform(-0.5, 1.5)
            py = random.uniform(-1.0, 1.0)
            pz = random.uniform(0.8, 2.0)
            wr = random.uniform(0.3, 0.8)
            wg = random.uniform(0.25, 0.7)
            wb = random.uniform(0.1, 0.5)
            cmd2 = (
                f'gz service -s /world/{self.world_name}/light_config '
                f'--reqtype gz.msgs.Light '
                f'--reptype gz.msgs.Boolean '
                f'--timeout 500 '
                f'--req "name: \\"warm_point\\", '
                f'pose: {{position: {{x: {px}, y: {py}, z: {pz}}}}}, '
                f'diffuse: {{r: {wr}, g: {wg}, b: {wb}, a: 1.0}}"'
            )
            subprocess.Popen(cmd2, shell=True,
                             stdout=subprocess.DEVNULL,
                             stderr=subprocess.DEVNULL)

        except Exception as e:
            self.get_logger().debug(f'Domain randomization failed: {e}')

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
        angles = []
        for lo, hi in self.JOINT_LIMITS:
            span = (hi - lo) * self.limit_frac
            mid = (hi + lo) / 2.0
            a = random.uniform(mid - span / 2, mid + span / 2)
            angles.append(round(a, 4))
        return angles

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

            # Domain randomization: add Gaussian noise to pixels
            if noise_sigma > 0:
                noise = np.random.normal(0, noise_sigma, arr.shape).astype(np.float32)
                arr = np.clip(arr.astype(np.float32) + noise, 0, 255).astype(np.uint8)

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
