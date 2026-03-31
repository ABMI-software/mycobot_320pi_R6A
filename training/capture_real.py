#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Capture real images from the Pi camera for fine-tuning.

Connects to the MyCobot 320 Pi via the bridge, commands joint angles,
captures the USB/Pi camera frame, and saves labelled data in the same
format as the synthetic dataset.

Usage::

    python3 training/capture_real.py \\
        --output /tmp/real_dataset \\
        --num-samples 200 \\
        --camera 0 \\
        --pi-host 10.10.0.225

Press Ctrl+C to stop early. The partial dataset will still be valid.
"""

import argparse
import csv
import json
import math
import os
import random
import socket
import time
from typing import List, Optional

import numpy as np


# Joint limits (same as dataset.py)
JOINT_LIMITS = [
    (-2.96, 2.96),
    (-2.79, 2.79),
    (-2.79, 2.79),
    (-2.79, 2.79),
    (-2.96, 2.96),
    (-3.05, 3.05),
]


def random_joint_angles(limit_fraction: float = 0.6) -> List[float]:
    """Generate random safe joint angles in degrees."""
    angles_deg = []
    for lo, hi in JOINT_LIMITS:
        lo_d, hi_d = math.degrees(lo), math.degrees(hi)
        span = (hi_d - lo_d) * limit_fraction
        mid = (hi_d + lo_d) / 2.0
        a = random.uniform(mid - span / 2, mid + span / 2)
        angles_deg.append(round(a, 1))
    return angles_deg


class RobotBridge:
    """Simple TCP client to talk to bridge_pi_simple."""

    def __init__(self, host: str, port: int = 5005, timeout: float = 10.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock: Optional[socket.socket] = None

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(self.timeout)
        self.sock.connect((self.host, self.port))
        print(f'✅ Connected to Pi bridge at {self.host}:{self.port}')

    def send_command(self, cmd: dict) -> str:
        data = json.dumps(cmd).encode() + b'\n'
        self.sock.sendall(data)
        response = self.sock.recv(4096).decode().strip()
        return response

    def send_angles(self, angles_deg: List[float], speed: int = 30):
        cmd = {'action': 'send_angles', 'angles': angles_deg, 'speed': speed}
        return self.send_command(cmd)

    def get_angles(self) -> List[float]:
        resp = self.send_command({'action': 'get_angles'})
        # Parse "ANGLES: [a, b, c, d, e, f]"
        if 'ANGLES' in resp:
            import re
            match = re.search(r'\[(.*?)\]', resp)
            if match:
                return [float(x.strip()) for x in match.group(1).split(',')]
        return []

    def close(self):
        if self.sock:
            self.sock.close()


def main():
    parser = argparse.ArgumentParser(
        description='Capture real (image, angles) data from MyCobot Pi')
    parser.add_argument('--output', default='/tmp/real_dataset',
                        help='Output directory')
    parser.add_argument('--num-samples', type=int, default=200,
                        help='Number of samples to collect')
    parser.add_argument('--camera', type=int, default=0,
                        help='OpenCV camera index')
    parser.add_argument('--pi-host', default='10.10.0.225',
                        help='Pi IP address')
    parser.add_argument('--pi-port', type=int, default=5005,
                        help='Pi bridge port')
    parser.add_argument('--speed', type=int, default=25,
                        help='Robot movement speed')
    parser.add_argument('--settle-time', type=float, default=3.0,
                        help='Seconds to wait after commanding')
    parser.add_argument('--limit-fraction', type=float, default=0.6,
                        help='Fraction of joint range to use (safety)')
    parser.add_argument('--image-width', type=int, default=640)
    parser.add_argument('--image-height', type=int, default=480)
    args = parser.parse_args()

    # Setup output
    img_dir = os.path.join(args.output, 'images')
    os.makedirs(img_dir, exist_ok=True)
    csv_path = os.path.join(args.output, 'labels.csv')

    # Setup camera
    try:
        import cv2
    except ImportError:
        print('❌ OpenCV not found. Install: pip install opencv-python')
        return

    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.image_width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.image_height)
    if not cap.isOpened():
        print(f'❌ Cannot open camera {args.camera}')
        return
    print(f'📷 Camera {args.camera} opened ({args.image_width}x{args.image_height})')

    # Connect to robot
    bridge = RobotBridge(args.pi_host, args.pi_port)
    try:
        bridge.connect()
    except Exception as e:
        print(f'❌ Cannot connect to Pi: {e}')
        cap.release()
        return

    # CSV header
    with open(csv_path, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow([
            'index', 'j1_rad', 'j2_rad', 'j3_rad',
            'j4_rad', 'j5_rad', 'j6_rad',
            'j1_deg', 'j2_deg', 'j3_deg',
            'j4_deg', 'j5_deg', 'j6_deg',
            'image_path',
        ])

    print(f'\n🚀 Collecting {args.num_samples} real samples…\n')

    try:
        for i in range(args.num_samples):
            # 1. Random pose
            target_deg = random_joint_angles(args.limit_fraction)
            bridge.send_angles(target_deg, args.speed)

            # 2. Wait for motion
            time.sleep(args.settle_time)

            # 3. Read actual angles
            actual_deg = bridge.get_angles()
            if not actual_deg or len(actual_deg) != 6:
                print(f'  [{i}] ⚠️  Could not read angles, using target')
                actual_deg = target_deg

            actual_rad = [math.radians(a) for a in actual_deg]

            # 4. Capture image
            ret, frame = cap.read()
            if not ret:
                print(f'  [{i}] ⚠️  Camera read failed — skipping')
                continue

            img_filename = f'{i:06d}.png'
            img_path = os.path.join(img_dir, img_filename)
            cv2.imwrite(img_path, frame)

            # 5. Save CSV row
            with open(csv_path, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow([
                    i,
                    *[round(a, 4) for a in actual_rad],
                    *[round(a, 2) for a in actual_deg],
                    f'images/{img_filename}',
                ])

            if (i + 1) % 10 == 0 or i == 0:
                print(f'  📸 [{i + 1}/{args.num_samples}] '
                      f'angles={[round(a, 1) for a in actual_deg]}')

    except KeyboardInterrupt:
        print(f'\n⏹️  Stopped early at sample {i}')

    finally:
        cap.release()
        bridge.close()

    print(f'\n✅ Real dataset saved to {args.output}')
    print(f'   Use for fine-tuning:')
    print(f'   python3 training/train.py \\')
    print(f'     --dataset {args.output} \\')
    print(f'     --checkpoint training/checkpoints/best_model.pth \\')
    print(f'     --finetune --lr 1e-5 --epochs 50')


if __name__ == '__main__':
    main()
