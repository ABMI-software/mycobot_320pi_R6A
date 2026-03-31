#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Synthetic Data Collector for MyCobot 320 Pi Pose Estimation.

This ROS2 node automates the generation of a labelled image dataset
inside Gazebo Harmonic:

1. Commands a random joint configuration via Gz joint-position topics
2. Waits for the robot to settle
3. Captures the camera image from the Gz camera bridge
4. Saves image + ground-truth joint angles to disk

Output structure:
    <output_dir>/
    ├── images/
    │   ├── 000000.png
    │   ├── 000001.png
    │   └── ...
    └── labels.csv          # idx, j1, j2, j3, j4, j5, j6, image_path

Usage (standalone):
    ros2 run mycobot_gateway synthetic_data_collector \
        --ros-args -p num_samples:=1000 -p output_dir:=/tmp/synth_dataset

Or via the provided launch file:
    ros2 launch mycobot_gateway synthetic_data.launch.py num_samples:=500
"""

import csv
import math
import os
import random
import time
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import Float64


class SyntheticDataCollector(Node):
    """Collect (image, joint_angles) pairs from Gazebo."""

    # Joint names in order, matching the URDF
    JOINT_NAMES = [
        'joint2_to_joint1',
        'joint3_to_joint2',
        'joint4_to_joint3',
        'joint5_to_joint4',
        'joint6_to_joint5',
        'joint6output_to_joint6',
    ]

    # Joint limits (rad) — from the URDF
    JOINT_LIMITS = [
        (-2.96, 2.96),   # joint1
        (-2.79, 2.79),   # joint2
        (-2.79, 2.79),   # joint3
        (-2.79, 2.79),   # joint4
        (-2.96, 2.96),   # joint5
        (-3.05, 3.05),   # joint6
    ]

    def __init__(self):
        super().__init__('synthetic_data_collector')

        # ---------- parameters ----------
        self.declare_parameter('num_samples', 1000)
        self.declare_parameter('output_dir', '/tmp/mycobot_synth_dataset')
        self.declare_parameter('settle_time', 1.5)   # seconds to wait after commanding
        self.declare_parameter('image_topic', '/synth_camera/image')
        self.declare_parameter('joint_limit_fraction', 0.7)  # use 70% of joint range

        self.num_samples = self.get_parameter('num_samples').value
        self.output_dir = self.get_parameter('output_dir').value
        self.settle_time = self.get_parameter('settle_time').value
        self.image_topic = self.get_parameter('image_topic').value
        self.limit_frac = self.get_parameter('joint_limit_fraction').value

        # ---------- state ----------
        self.current_image: Optional[Image] = None
        self.current_joints: Optional[List[float]] = None
        self.sample_idx = 0
        self.collecting = False

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
        self.image_sub = self.create_subscription(
            Image, self.image_topic, self._image_cb, img_qos,
        )
        self.joint_sub = self.create_subscription(
            JointState, '/joint_states', self._joint_cb, 10,
        )

        # ---------- output dirs ----------
        self.img_dir = os.path.join(self.output_dir, 'images')
        os.makedirs(self.img_dir, exist_ok=True)
        self.csv_path = os.path.join(self.output_dir, 'labels.csv')

        # Write CSV header
        with open(self.csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'index', 'j1_rad', 'j2_rad', 'j3_rad',
                'j4_rad', 'j5_rad', 'j6_rad',
                'j1_deg', 'j2_deg', 'j3_deg',
                'j4_deg', 'j5_deg', 'j6_deg',
                'image_path',
            ])

        self.get_logger().info(
            f'🎬 Synthetic Data Collector initialised\n'
            f'   Samples : {self.num_samples}\n'
            f'   Output  : {self.output_dir}\n'
            f'   Settle  : {self.settle_time}s\n'
            f'   Image   : {self.image_topic}'
        )

        # Start collection after a short delay (let Gazebo settle)
        self._startup_timer = self.create_timer(3.0, self._start_collection_once)

    # ------------------------------------------------------------------
    # Callbacks
    # ------------------------------------------------------------------
    def _image_cb(self, msg: Image):
        self.current_image = msg

    def _joint_cb(self, msg: JointState):
        # Re-order joints to match our canonical order
        if not msg.name:
            return
        angles = [0.0] * len(self.JOINT_NAMES)
        for i, jn in enumerate(self.JOINT_NAMES):
            if jn in msg.name:
                idx = list(msg.name).index(jn)
                angles[i] = msg.position[idx]
        self.current_joints = angles

    # ------------------------------------------------------------------
    # Collection loop
    # ------------------------------------------------------------------
    def _start_collection_once(self):
        """Called once by the timer to kick off the async collection."""
        if self.collecting:
            return
        self.collecting = True
        # Cancel the startup timer so it doesn't fire again
        self._startup_timer.cancel()
        self.get_logger().info('🚀 Starting data collection…')
        self._collect_next()

    def _collect_next(self):
        """Schedule the next sample."""
        if self.sample_idx >= self.num_samples:
            self.get_logger().info(
                f'✅ Collection complete! {self.num_samples} samples saved to {self.output_dir}'
            )
            rclpy.shutdown()
            return

        # 1. Random joint command
        target_angles = self._random_joint_angles()
        self._command_joints(target_angles)

        # 2. Wait for settle, then capture (one-shot timer)
        timer = self.create_timer(
            self.settle_time,
            lambda: self._on_settle_timeout(timer, target_angles),
        )

    def _on_settle_timeout(self, timer, target_angles):
        """Called once after settle_time — cancel timer and capture."""
        timer.cancel()
        self._capture_and_save(target_angles)

    def _random_joint_angles(self) -> List[float]:
        """Generate random joint angles within safe limits."""
        angles = []
        for lo, hi in self.JOINT_LIMITS:
            span = (hi - lo) * self.limit_frac
            mid = (hi + lo) / 2.0
            a = random.uniform(mid - span / 2, mid + span / 2)
            angles.append(round(a, 4))
        return angles

    def _command_joints(self, angles: List[float]):
        """Send position commands to each Gz joint controller."""
        for jname, angle in zip(self.JOINT_NAMES, angles):
            msg = Float64()
            msg.data = angle
            self.joint_pubs[jname].publish(msg)

    def _capture_and_save(self, target_angles: List[float]):
        """Grab current image + joint state and persist to disk."""
        if self.current_image is None:
            self.get_logger().warn(
                f'[{self.sample_idx}] No image received yet — skipping'
            )
            self._collect_next()
            return

        # Use actual joint readings if available, else target
        # Note: target_angles are the commanded values (ground truth)
        # current_joints are the actual measured values (may lag slightly)
        if self.current_joints:
            angles = self.current_joints
        else:
            self.get_logger().warn(
                f'[{self.sample_idx}] No joint_state feedback — using target angles'
            )
            angles = target_angles

        # Save image as raw PNG via cv_bridge-free approach
        img_filename = f'{self.sample_idx:06d}.png'
        img_path = os.path.join(self.img_dir, img_filename)
        self._save_image(self.current_image, img_path)

        # Append CSV row
        degs = [round(math.degrees(a), 2) for a in angles]
        with open(self.csv_path, 'a', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                self.sample_idx,
                *[round(a, 4) for a in angles],
                *degs,
                f'images/{img_filename}',
            ])

        self.get_logger().info(
            f'📸 [{self.sample_idx + 1}/{self.num_samples}] '
            f'angles(deg)={degs}'
        )

        self.sample_idx += 1
        self._collect_next()

    # ------------------------------------------------------------------
    # Image helper
    # ------------------------------------------------------------------
    @staticmethod
    def _save_image(img_msg: Image, path: str):
        """Save a sensor_msgs/Image to a PNG file without cv_bridge.

        Supports rgb8, bgr8, rgba8, bgra8 encodings.
        Falls back to raw bytes if the encoding is unknown.
        """
        try:
            import numpy as np
            from PIL import Image as PILImage

            h, w = img_msg.height, img_msg.width
            encoding = img_msg.encoding.lower()

            data = bytes(img_msg.data)
            if encoding in ('rgb8',):
                arr = np.frombuffer(data, dtype=np.uint8).reshape((h, w, 3))
            elif encoding in ('bgr8',):
                arr = np.frombuffer(data, dtype=np.uint8).reshape((h, w, 3))
                arr = arr[:, :, ::-1]  # BGR → RGB
            elif encoding in ('rgba8',):
                arr = np.frombuffer(data, dtype=np.uint8).reshape((h, w, 4))
                arr = arr[:, :, :3]
            elif encoding in ('bgra8',):
                arr = np.frombuffer(data, dtype=np.uint8).reshape((h, w, 4))
                arr = arr[:, :, 2::-1]
            else:
                # Best-effort: try 3-channel
                arr = np.frombuffer(data, dtype=np.uint8).reshape((h, w, -1))
                if arr.shape[2] == 4:
                    arr = arr[:, :, :3]

            pil_img = PILImage.fromarray(arr)
            pil_img.save(path)

        except ImportError:
            # Fallback: dump raw bytes
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
