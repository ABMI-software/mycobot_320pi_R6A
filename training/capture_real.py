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
    (-2.96, 2.96),   # J1  base rotation
    (-2.79, 2.79),   # J2  shoulder
    (-2.79, 2.79),   # J3  elbow
    (-2.79, 2.79),   # J4  wrist pitch
    (-2.96, 2.96),   # J5  wrist roll
    (-3.05, 3.05),   # J6  flange
]

# ---------------------------------------------------------------------------
# Kinematic safety — proper FK from real URDF dimensions (metres)
# ---------------------------------------------------------------------------
# From mycobot_description/urdf/mycobot_320_pi_2022.urdf:
#   joint2_to_joint1: xyz="0 0 0.162"        → base height
#   joint3_to_joint2: rpy="0 -π/2 π/2"       → frame rotation (no translation)
#   joint4_to_joint3: xyz="0.13635 0 0"       → upper arm
#   joint5_to_joint4: xyz="0.1205 0 0.082"    → forearm + Z-offset
#   joint6_to_joint5: xyz="0 -0.084 0"        → wrist
#   joint6output_to_joint6: xyz="0 0.06635 0" → end-effector
_BASE_H   = 162.0    # base to shoulder (mm)
_L_UPPER  = 136.35   # shoulder to elbow (mm)
_L_FORE   = 120.5    # elbow to wrist (mm)
_L_FORE_Z = 82.0     # forearm Z-offset (mm)
_L_WRIST  = 84.0     # wrist length (mm)
_L_EE     = 66.35    # end-effector length (mm)

# Safety thresholds
_TABLE_Z_MIN   = 60.0   # minimum Z above the table for any arm point (mm)
_BASE_R_MIN    = 90.0   # minimum radial distance from base axis (mm)
                         # — protects power / USB / HDMI cables behind the base


def _fk_key_points(j2_deg: float, j3_deg: float,
                    j4_deg: float) -> List[Tuple[float, float]]:
    """Compute (z_mm, r_mm) for elbow, wrist, and end-effector.

    Uses planar forward kinematics in the arm's sagittal plane.
    J2, J3, J4 are cumulative pitch angles from vertical.
    Returns list of (z_height, radial_distance) tuples.
    """
    a2 = math.radians(j2_deg)
    a3 = math.radians(j2_deg + j3_deg)
    a4 = math.radians(j2_deg + j3_deg + j4_deg)

    # Elbow (end of upper arm)
    z_elbow = _BASE_H + _L_UPPER * math.cos(a2)
    r_elbow = _L_UPPER * math.sin(a2)

    # Wrist (end of forearm — includes the 82 mm perpendicular offset)
    z_wrist = z_elbow + _L_FORE * math.cos(a3) - _L_FORE_Z * math.sin(a3)
    r_wrist = r_elbow + _L_FORE * math.sin(a3) + _L_FORE_Z * math.cos(a3)

    # End-effector (wrist + EE combined ≈ 150 mm)
    l_ee_total = _L_WRIST + _L_EE
    z_ee = z_wrist + l_ee_total * math.cos(a4)
    r_ee = r_wrist + l_ee_total * math.sin(a4)

    return [
        (z_elbow, abs(r_elbow)),
        (z_wrist, abs(r_wrist)),
        (z_ee,    abs(r_ee)),
    ]


def _pose_is_safe(angles_deg: List[float]) -> bool:
    """Return True if the pose keeps the entire arm above the table
    and away from the base column (cable protection)."""
    pts = _fk_key_points(angles_deg[1], angles_deg[2], angles_deg[3])
    for z, r in pts:
        # Must be above the table
        if z < _TABLE_Z_MIN:
            return False
        # Must not be too close to the base axis (cable protection)
        # Only enforce when the point is low (below shoulder height)
        # — high poses close to base axis are fine (arm pointing up)
        if z < _BASE_H and r < _BASE_R_MIN:
            return False
    return True


def random_joint_angles(limit_fraction: float = 0.5,
                        max_attempts: int = 500) -> List[float]:
    """Generate random safe joint angles in degrees.

    Rejects poses where any arm segment (elbow, wrist, or end-effector):
      - goes below the table surface (Z < TABLE_Z_MIN)
      - comes too close to the base column when low (R < BASE_R_MIN)
    """
    for _ in range(max_attempts):
        angles_deg = []
        for lo, hi in JOINT_LIMITS:
            lo_d, hi_d = math.degrees(lo), math.degrees(hi)
            span = (hi_d - lo_d) * limit_fraction
            mid = (hi_d + lo_d) / 2.0
            a = random.uniform(mid - span / 2, mid + span / 2)
            angles_deg.append(round(a, 1))

        if _pose_is_safe(angles_deg):
            return angles_deg

    # Fallback: safe upright pose
    return [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]


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
        try:
            data = json.dumps(cmd).encode() + b'\n'
            self.sock.sendall(data)
            response = self.sock.recv(4096).decode().strip()
            return response
        except (TimeoutError, OSError) as e:
            return f'ERROR: {e}'

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
    parser.add_argument('--limit-fraction', type=float, default=0.5,
                        help='Fraction of joint range to use (safety, default 0.5)')
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

    # --- Test capture before starting ---
    print('  🧪 Test capture…')
    test_imgs = cam_client.capture_all(quality=args.quality)
    for cname, data in test_imgs.items():
        sz = len(data) if data else 0
        print(f'     {cname}: {sz} bytes ({sz // 1024} KB)')
    if not test_imgs or all(v is None or len(v) == 0 for v in test_imgs.values()):
        print('  ❌ Test capture failed — no valid frames. Check cameras on Pi.')
        bridge.close()
        cam_client.close()
        return
    print('  ✅ Test capture OK')

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
          f'({len(cam_names)} cameras each)…')
    print(f'   Safety: table clearance ≥ {_TABLE_Z_MIN} mm, '
          f'base distance ≥ {_BASE_R_MIN} mm (when low)')
    print(f'   Joint range fraction: {args.limit_fraction}\n')

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
            saved_any = False
            with open(csv_path, 'a', newline='') as f:
                writer = csv.writer(f)
                for cname, jpeg_data in images.items():
                    if jpeg_data is None or len(jpeg_data) == 0:
                        print(f'  [{i}] ⚠️  Empty frame from {cname}')
                        continue
                    img_filename = f'{i:06d}.png'
                    img_rel = f'images/{cname}/{img_filename}'
                    img_path = os.path.join(args.output, img_rel)

                    # Decode JPEG and save as PNG for consistency with synthetic data
                    ok = _save_jpeg_as_png(jpeg_data, img_path)
                    if not ok:
                        print(f'  [{i}] ⚠️  Failed to save {cname} '
                              f'({len(jpeg_data)} bytes JPEG)')
                        continue

                    writer.writerow([
                        i,
                        *[round(a, 4) for a in actual_rad],
                        *[round(a, 2) for a in actual_deg],
                        cname,
                        img_rel,
                    ])
                    saved_any = True

            if not saved_any:
                print(f'  [{i}] ⚠️  No images saved for this pose')
                continue

            collected += 1
            if collected % 10 == 0 or collected == 1:
                sizes_kb = {k: (len(v) // 1024 if v else 0)
                            for k, v in images.items()}
                print(f'  📸 [{collected}/{total_to_collect}] '
                      f'cameras={list(images.keys())} '
                      f'jpeg_kb={sizes_kb} '
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


def _save_jpeg_as_png(jpeg_bytes: bytes, out_path: str) -> bool:
    """Decode JPEG bytes and save as PNG. Returns True on success."""
    import io
    try:
        from PIL import Image
        img = Image.open(io.BytesIO(jpeg_bytes))
        img.load()  # force decode
        img.save(out_path)
        return True
    except ImportError:
        # Fallback: save as JPEG directly if Pillow not available
        try:
            jpg_path = out_path.replace('.png', '.jpg')
            with open(jpg_path, 'wb') as f:
                f.write(jpeg_bytes)
            return True
        except Exception as e:
            print(f'    ❌ Save fallback failed: {e}')
            return False
    except Exception as e:
        print(f'    ❌ Image decode/save failed: {e}')
        return False


if __name__ == '__main__':
    main()
