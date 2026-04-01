#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Capture real images from Pi Arducam cameras for fine-tuning.

Connects to TWO services on the Raspberry Pi:
  1. ``bridge_pi_simple.py`` (port 5005) — robot joint commands
  2. ``pi_camera_server.py`` (port 5006) — USB camera JPEG streaming

The PC Tour orchestrates: send random pose → wait → capture all cameras →
save images and joint labels in v2-compatible format.

Setup on the Pi
---------------
Terminal 1::

    python3 bridge_pi_simple.py

Terminal 2::

    python3 pi_camera_server.py --cameras 0 2 --names front side

Then on the PC Tour
-------------------
::

    python3 training/capture_real.py \\
        --output /tmp/real_dataset \\
        --num-samples 200 \\
        --pi-host 10.10.0.225

Press Ctrl+C to stop early. The partial dataset will still be valid.
"""

import argparse
import csv
import json
import math
import os
import random
import re
import socket
import sys
import time
from typing import Dict, List, Optional, Tuple

import numpy as np


# ---------------------------------------------------------------------------
# Joint limits (same as dataset.py / collector)
# ---------------------------------------------------------------------------
JOINT_LIMITS = [
    (-2.96, 2.96),   # J1
    (-2.79, 2.79),   # J2
    (-2.79, 2.79),   # J3
    (-2.79, 2.79),   # J4
    (-2.96, 2.96),   # J5
    (-3.05, 3.05),   # J6
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


# ---------------------------------------------------------------------------
# Robot bridge client (TCP to bridge_pi_simple.py)
# ---------------------------------------------------------------------------
class RobotBridge:
    """TCP client talking to bridge_pi_simple on the Pi."""

    def __init__(self, host: str, port: int = 5005, timeout: float = 10.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock: Optional[socket.socket] = None

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(self.timeout)
        self.sock.connect((self.host, self.port))
        print(f'  ✅ Robot bridge connected ({self.host}:{self.port})')

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
        if 'ANGLES' in resp:
            match = re.search(r'\[(.*?)\]', resp)
            if match:
                return [float(x.strip()) for x in match.group(1).split(',')]
        return []

    def go_home(self):
        return self.send_command({'action': 'go_home'})

    def close(self):
        if self.sock:
            self.sock.close()


# ---------------------------------------------------------------------------
# Camera client (TCP to pi_camera_server.py)
# ---------------------------------------------------------------------------
class CameraClient:
    """TCP client talking to pi_camera_server on the Pi."""

    def __init__(self, host: str, port: int = 5006, timeout: float = 10.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock: Optional[socket.socket] = None
        self.camera_indices: List[int] = []
        self.camera_names: Dict[int, str] = {}

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(self.timeout)
        self.sock.connect((self.host, self.port))
        print(f'  ✅ Camera server connected ({self.host}:{self.port})')
        # Discover cameras
        self._list_cameras()

    def _send_json(self, obj: dict):
        self.sock.sendall(json.dumps(obj).encode() + b'\n')

    def _recv_line(self) -> str:
        """Read until newline."""
        buf = b''
        while True:
            chunk = self.sock.recv(1)
            if not chunk:
                raise ConnectionError('Camera server disconnected')
            if chunk == b'\n':
                return buf.decode('utf-8')
            buf += chunk

    def _recv_exact(self, n: int) -> bytes:
        """Read exactly n bytes."""
        buf = b''
        while len(buf) < n:
            chunk = self.sock.recv(min(n - len(buf), 65536))
            if not chunk:
                raise ConnectionError('Camera server disconnected')
            buf += chunk
        return buf

    def _list_cameras(self):
        self._send_json({'action': 'list_cameras'})
        resp = json.loads(self._recv_line())
        self.camera_indices = resp['cameras']
        self.camera_names = {}
        names_dict = resp.get('names', {})
        for idx in self.camera_indices:
            self.camera_names[idx] = names_dict.get(str(idx), f'cam{idx}')
        print(f'  📷 Cameras available: {self.camera_names}')

    def capture_all(self, quality: int = 90) -> Dict[str, bytes]:
        """Capture from all cameras. Returns {name: jpeg_bytes}."""
        self._send_json({'action': 'capture_all', 'quality': quality})
        resp = json.loads(self._recv_line())
        if not resp.get('ok'):
            return {}
        result = {}
        names = resp.get('names', [])
        sizes = resp.get('sizes', [])
        for name, sz in zip(names, sizes):
            if sz > 0:
                data = self._recv_exact(sz)
                result[name] = data
            else:
                result[name] = None
        return result

    def close(self):
        if self.sock:
            self.sock.close()


# ---------------------------------------------------------------------------
# Main capture loop
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description='Capture real (image, angles) data from MyCobot Pi + Arducams')
    parser.add_argument('--output', default='/tmp/real_dataset',
                        help='Output directory (default: /tmp/real_dataset)')
    parser.add_argument('--num-samples', type=int, default=200,
                        help='Number of pose samples to collect')
    parser.add_argument('--pi-host', default='10.10.0.225',
                        help='Pi IP address')
    parser.add_argument('--bridge-port', type=int, default=5005,
                        help='Pi bridge port (robot commands)')
    parser.add_argument('--camera-port', type=int, default=5006,
                        help='Pi camera server port')
    parser.add_argument('--speed', type=int, default=25,
                        help='Robot movement speed (1-100)')
    parser.add_argument('--settle-time', type=float, default=3.0,
                        help='Seconds to wait after commanding a pose')
    parser.add_argument('--limit-fraction', type=float, default=0.6,
                        help='Fraction of joint range to use (safety)')
    parser.add_argument('--quality', type=int, default=95,
                        help='JPEG quality for captured images (1-100)')
    parser.add_argument('--go-home-first', action='store_true', default=True,
                        help='Send robot home before starting (default: yes)')
    parser.add_argument('--no-home', dest='go_home_first', action='store_false')
    args = parser.parse_args()

    print('=' * 60)
    print('📸 Real Data Capture — MyCobot 320 Pi + Arducams')
    print('=' * 60)

    # --- Connect to Pi services ---
    bridge = RobotBridge(args.pi_host, args.bridge_port)
    cam_client = CameraClient(args.pi_host, args.camera_port)

    try:
        bridge.connect()
    except Exception as e:
        print(f'❌ Cannot connect to robot bridge: {e}')
        print(f'   → Start bridge_pi_simple.py on the Pi first')
        return

    try:
        cam_client.connect()
    except Exception as e:
        print(f'❌ Cannot connect to camera server: {e}')
        print(f'   → Start pi_camera_server.py on the Pi first')
        bridge.close()
        return

    cam_names = list(cam_client.camera_names.values())
    print(f'\n  🎥 Using cameras: {cam_names}')

    # --- Setup output dirs ---
    for cname in cam_names:
        os.makedirs(os.path.join(args.output, 'images', cname), exist_ok=True)

    csv_path = os.path.join(args.output, 'labels.csv')
    csv_exists = os.path.exists(csv_path)

    # Support resuming an interrupted capture
    start_idx = 0
    if csv_exists:
        with open(csv_path, 'r') as f:
            reader = csv.reader(f)
            next(reader, None)  # skip header
            for row in reader:
                if row:
                    start_idx = max(start_idx, int(row[0]) + 1)
        if start_idx > 0:
            print(f'  ♻️  Resuming from sample {start_idx} (found existing labels)')

    if not csv_exists:
        with open(csv_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                'index', 'j1_rad', 'j2_rad', 'j3_rad',
                'j4_rad', 'j5_rad', 'j6_rad',
                'j1_deg', 'j2_deg', 'j3_deg',
                'j4_deg', 'j5_deg', 'j6_deg',
                'camera', 'image_path',
            ])

    # --- Go home first ---
    if args.go_home_first:
        print('  🏠 Sending robot home…')
        bridge.go_home()
        time.sleep(3)

    total_to_collect = args.num_samples - start_idx
    print(f'\n🚀 Collecting {total_to_collect} real samples '
          f'({len(cam_names)} cameras each)…\n')

    collected = 0
    try:
        for i in range(start_idx, args.num_samples):
            # 1. Random pose
            target_deg = random_joint_angles(args.limit_fraction)
            bridge.send_angles(target_deg, args.speed)

            # 2. Wait for motion to complete
            time.sleep(args.settle_time)

            # 3. Read actual joint angles
            actual_deg = bridge.get_angles()
            if not actual_deg or len(actual_deg) != 6:
                print(f'  [{i}] ⚠️  Could not read angles, using target')
                actual_deg = target_deg

            actual_rad = [math.radians(a) for a in actual_deg]

            # 4. Capture images from all cameras
            images = cam_client.capture_all(quality=args.quality)
            if not images:
                print(f'  [{i}] ⚠️  No images captured — skipping')
                continue

            # 5. Save images and CSV rows
            with open(csv_path, 'a', newline='') as f:
                writer = csv.writer(f)
                for cname, jpeg_data in images.items():
                    if jpeg_data is None:
                        continue
                    img_filename = f'{i:06d}.png'
                    img_rel = f'images/{cname}/{img_filename}'
                    img_path = os.path.join(args.output, img_rel)

                    # Decode JPEG and save as PNG for consistency with synthetic data
                    _save_jpeg_as_png(jpeg_data, img_path)

                    writer.writerow([
                        i,
                        *[round(a, 4) for a in actual_rad],
                        *[round(a, 2) for a in actual_deg],
                        cname,
                        img_rel,
                    ])

            collected += 1
            if collected % 10 == 0 or collected == 1:
                print(f'  📸 [{collected}/{total_to_collect}] '
                      f'cameras={list(images.keys())} '
                      f'angles={[round(a, 1) for a in actual_deg]}')

    except KeyboardInterrupt:
        print(f'\n⏹️  Stopped early at sample {collected}')

    finally:
        # Go home safely
        print('  🏠 Returning home…')
        try:
            bridge.go_home()
        except Exception:
            pass
        bridge.close()
        cam_client.close()

    total_images = collected * len(cam_names)
    print(f'\n✅ Real dataset saved to {args.output}')
    print(f'   Samples: {collected} poses × {len(cam_names)} cameras = {total_images} images')
    print(f'   Camera names: {cam_names}')
    print(f'\n💡 To fine-tune on this data:')
    print(f'   python3 training/train.py \\')
    print(f'     --dataset {args.output} \\')
    print(f'     --checkpoint training/checkpoints_mv_resnet50/best_model.pth \\')
    print(f'     --finetune --lr 1e-5 --epochs 50')
    if len(cam_names) == 1:
        print(f'\n   (single-camera → use single-view mode, no --multi-view)')
    else:
        print(f'\n   For multi-view fine-tune (both cameras):')
        print(f'     --multi-view --num-views {len(cam_names)}')
        print(f'\n   For single-view fine-tune (one camera):')
        print(f'     --camera-filter {cam_names[0]}')


def _save_jpeg_as_png(jpeg_bytes: bytes, out_path: str):
    """Decode JPEG bytes and save as PNG."""
    import io
    try:
        from PIL import Image
        img = Image.open(io.BytesIO(jpeg_bytes))
        img.save(out_path)
    except ImportError:
        # Fallback: save as JPEG directly if Pillow not available
        jpg_path = out_path.replace('.png', '.jpg')
        with open(jpg_path, 'wb') as f:
            f.write(jpeg_bytes)


if __name__ == '__main__':
    main()
