#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Capture real images from 3 cameras simultaneously:
  - CAM-A : ArduCam (local PC Tour, USB, V4L2 — 640x480)
  - CAM-B : Astra    (local PC Tour, oni_grabber + shared memory)
  - CAM-C : SVPRO    (local PC Tour, USB, V4L2 — 5MP, captured at 640x480)

All three cameras are on the PC Tour. The robot stays on the Raspberry Pi,
reached over the TCP bridge (--pi-host, port 5005) for joint commands only.

Calibrated intrinsics (calibration/<stem>.npz) are written as an NDDS-style
_camera_settings.json into each camera's image folder, scaled to the captured
frame size. ArduCam=cam_0, SVPRO=cam_2; Astra is uncalibrated (not used for PnP).

Usage:
    python3 training/capture_real_3cam.py \
        --output /tmp/dream_data/real_3cam_session1 \
        --num-samples 500 \
        --pi-host 10.10.0.221 \
        --arducam-index 0 \
        --svpro-index 3
"""

import argparse
import csv
import json
import math
import os
import random
import re
import socket
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import Dict, List, Optional

import cv2
import numpy as np


# ---------------------------------------------------------------------------
# oni_grabber path
# ---------------------------------------------------------------------------
ONI_GRABBER = (
    "/home/genji/Downloads/Orbbec_OpenNI_v2.3.0.86-beta6_linux_release/"
    "OpenNI_2.3.0.86_202210111154_4c8f5aa4_beta6_linux_x64/"
    "OpenNI_2.3.0.86_202210111154_4c8f5aa4_beta6_linux/samples/bin/oni_grabber"
)
ONI_COLOR = Path("/dev/shm/oni_color.rgb")
ONI_TICK  = Path("/dev/shm/oni_tick.txt")
ONI_INFO  = Path("/dev/shm/oni_info.txt")


# ---------------------------------------------------------------------------
# Camera calibration (intrinsics → NDDS _camera_settings.json)
# ---------------------------------------------------------------------------
CALIBRATION_DIR = Path(__file__).resolve().parent / "calibration"

# Map capture-camera name → calibration basename in CALIBRATION_DIR.
# Astra (CAM-B) is deliberately absent: it is not used for PnP, so it carries
# no intrinsics. SVPRO was the missing one — cam_2 is its calibration.
CALIBRATION_MAP = {
    "svpro":   "cam_2",   # SVPRO (calibrated at 800x600; intrinsics auto-scaled to captured size)
    "arducam": "cam_0",   # ArduCam on the PC (calibrated at 640x480)
}


def load_calibration(cam_name):
    """Return (K 3x3, dist, (calib_w, calib_h)) for cam_name, or None.

    Reads <stem>.npz (mtx, dist) and <stem>.meta.json (resolution) produced by
    training/calibration/calibrate_camera.py.
    """
    stem = CALIBRATION_MAP.get(cam_name)
    if not stem:
        return None
    npz_path  = CALIBRATION_DIR / f'{stem}.npz'
    meta_path = CALIBRATION_DIR / f'{stem}.meta.json'
    if not npz_path.exists():
        print(f'  ⚠️  {cam_name}: calibration {npz_path.name} not found — no intrinsics written')
        return None
    data = np.load(npz_path)
    K    = data['mtx']
    dist = data['dist'] if 'dist' in data.files else np.zeros((1, 5))
    calib_w = calib_h = None
    if meta_path.exists():
        res = json.loads(meta_path.read_text()).get('resolution')
        if res:
            calib_w, calib_h = int(res[0]), int(res[1])
    return K, dist, (calib_w, calib_h)


def write_camera_settings(cam_name, output_dir, frame_w, frame_h):
    """Write an NDDS-style _camera_settings.json into the camera's image folder,
    scaling the calibrated intrinsics to the actually-captured frame size.

    Returns True if written, False if no calibration is available.
    """
    calib = load_calibration(cam_name)
    if calib is None:
        return False
    K, dist, (calib_w, calib_h) = calib

    fx, fy = float(K[0, 0]), float(K[1, 1])
    cx, cy = float(K[0, 2]), float(K[1, 2])
    s      = float(K[0, 1])

    # fx, cx, s scale with width; fy, cy with height.
    if calib_w and calib_h and (calib_w != frame_w or calib_h != frame_h):
        sx, sy = frame_w / calib_w, frame_h / calib_h
        fx, cx, s = fx * sx, cx * sx, s * sx
        fy, cy    = fy * sy, cy * sy
        print(f'  ↳ {cam_name}: intrinsics scaled {calib_w}x{calib_h} → {frame_w}x{frame_h}')

    settings = {
        'camera_settings': [{
            'name': cam_name,
            'intrinsic_settings': {'fx': fx, 'fy': fy, 'cx': cx, 'cy': cy, 's': s},
            'captured_image_size': {'width': frame_w, 'height': frame_h},
            'dist_coeffs': [float(c) for c in np.asarray(dist).ravel()],
        }]
    }
    out_path = Path(output_dir) / 'images' / cam_name / '_camera_settings.json'
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(json.dumps(settings, indent=2))
    return True


# ---------------------------------------------------------------------------
# V4L2 device identity (detect two indices that are the SAME physical camera)
# ---------------------------------------------------------------------------
def int_or_path(value):
    """argparse type: a bare V4L2 index (int) or a device / by-id path (str)."""
    return int(value) if value.isdigit() else value


def set_arducam_exposure(spec, exposure):
    """Fully-specified manual exposure for a reproducible ArduCam image.

    Matches the session4 reference dataset (mean≈87, p5≈22, contrast≈61) by lowering
    the *exposure* (light) with brightness=0 — NOT a negative brightness offset. The
    offset crushes the blacks (p5→2, harsh contrast); lowering exposure dims the whole
    image proportionally, keeping shadows soft like session4. exposure=75 → mean≈86,
    p5≈18. gain=0. Pinning exposure also avoids the random black frames seen before.
    auto_exposure must be set to manual *first* so exposure_time_absolute activates.

    NB: the right exposure depends on ambient light — re-tune (--arducam-exposure) if
    the room lighting changes (session4 was shot under different light).
    """
    dev = str(resolve_v4l2(spec))
    subprocess.run(['v4l2-ctl', '-d', dev, '--set-ctrl', 'auto_exposure=1'],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(['v4l2-ctl', '-d', dev, '--set-ctrl',
                    f'exposure_time_absolute={exposure},gain=0,brightness=0'],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def set_svpro_focus(spec, focus):
    """Lock the SVPRO focus + image processing for a sharp, reproducible image.

    The SVPRO has a real variable-focus lens: sharp on a plateau focus_absolute≈1-90
    (peak on the wrist/LED at ~90) then COLLAPSING to ~1-15 in the mid range (128-930).
    We disable continuous autofocus (it would hunt/pump between poses) and pin
    focus_absolute (default 90 = sharpest on the wrist/LED), so captures never drift
    into the catastrophically-blurry middle. autofocus must be off *first* to set focus.

    We also pin sharpness=0 + contrast=1 — the session9 reference look. In-camera
    sharpening/contrast looks artificial ("trop") and isn't wanted for the dataset;
    these are the camera defaults so the saved images match session9's natural render.
    """
    dev = str(resolve_v4l2(spec))
    subprocess.run(['v4l2-ctl', '-d', dev, '--set-ctrl', 'focus_automatic_continuous=0'],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    subprocess.run(['v4l2-ctl', '-d', dev, '--set-ctrl',
                    f'focus_absolute={focus},sharpness=0,contrast=1'],
                   stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def resolve_v4l2(spec):
    """Return the real /dev/videoN path for an int index or a device path/by-id link."""
    if isinstance(spec, int) or (isinstance(spec, str) and spec.isdigit()):
        return f'/dev/video{int(spec)}'
    return os.path.realpath(str(spec))


def physical_cam_id(dev):
    """Identify the *physical* camera behind a /dev/videoN node.

    A single UVC camera exposes several /dev/videoN nodes (capture + metadata)
    that share one by-id stem (…-video-indexN). Stripping the trailing
    `-video-indexN` yields a per-camera identity, so two different nodes of the
    same sensor collapse to the same id. Falls back to the realpath when no
    by-id link exists.
    """
    real = os.path.realpath(dev)
    by_id = Path('/dev/v4l/by-id')
    if by_id.is_dir():
        for link in by_id.iterdir():
            if os.path.realpath(link) == real:
                return re.sub(r'-video-index\d+$', '', link.name)
    return real


def assert_distinct_cameras(cameras):
    """Abort if two V4L2 cameras resolve to the same physical sensor."""
    seen = {}
    for name, cam in cameras.items():
        spec = getattr(cam, 'index', None)
        if spec is None:
            continue  # non-V4L2 source (Astra)
        cid = physical_cam_id(resolve_v4l2(spec))
        if cid in seen:
            print(f'\n  ❌ {name!r} and {seen[cid]!r} are the SAME physical camera '
                  f'({cid}).\n     Two nodes of one sensor → duplicate images. '
                  f'Use distinct /dev/v4l/by-id/ paths.')
            return False
        seen[cid] = name
    return True


# ---------------------------------------------------------------------------
# Joint limits
# ---------------------------------------------------------------------------
JOINT_LIMITS = [
    (-2.96, 2.96),
    (-2.79, 2.79),
    (-2.79, 2.79),
    (-2.79, 2.79),
    (-2.96, 2.96),
    (-3.05, 3.05),
]

_BASE_H      = 162.0
_L_UPPER     = 136.35
_L_FORE      = 120.5
_L_FORE_Z    = 82.0
_L_WRIST     = 84.0
_L_EE        = 66.35
# Gripper mounted on the flange: its fingers extend ~110 mm BELOW link6. The
# safety FK below must include it, else poses judged "safe" drive the gripper
# tip into the table (link6 clears 60 mm but the fingers are 110 mm lower).
# Set to 0.0 when running WITHOUT a gripper.
_L_GRIPPER   = 110.0
_TABLE_Z_MIN = 60.0
_BASE_R_MIN  = 90.0


def _fk_key_points(j2, j3, j4):
    a2 = math.radians(j2)
    a3 = math.radians(j2 + j3)
    a4 = math.radians(j2 + j3 + j4)
    z_elbow = _BASE_H + _L_UPPER * math.cos(a2)
    r_elbow = _L_UPPER * math.sin(a2)
    z_wrist = z_elbow + _L_FORE * math.cos(a3) - _L_FORE_Z * math.sin(a3)
    r_wrist = r_elbow + _L_FORE * math.sin(a3) + _L_FORE_Z * math.cos(a3)
    l_ee    = _L_WRIST + _L_EE + _L_GRIPPER
    z_ee    = z_wrist + l_ee * math.cos(a4)
    r_ee    = r_wrist + l_ee * math.sin(a4)
    return [(z_elbow, abs(r_elbow)), (z_wrist, abs(r_wrist)), (z_ee, abs(r_ee))]


def _pose_is_safe(angles_deg):
    pts = _fk_key_points(angles_deg[1], angles_deg[2], angles_deg[3])
    for z, r in pts:
        if z < _TABLE_Z_MIN:
            return False
        if z < _BASE_H and r < _BASE_R_MIN:
            return False
    return True


def random_joint_angles(limit_fraction=0.5, max_attempts=500):
    for _ in range(max_attempts):
        angles_deg = []
        for lo, hi in JOINT_LIMITS:
            lo_d, hi_d = math.degrees(lo), math.degrees(hi)
            span = (hi_d - lo_d) * limit_fraction
            mid  = (hi_d + lo_d) / 2.0
            angles_deg.append(round(random.uniform(mid - span/2, mid + span/2), 1))
        if _pose_is_safe(angles_deg):
            return angles_deg
    return [0.0] * 6


def format_time(seconds):
    if seconds < 60:
        return f'{int(seconds)}s'
    elif seconds < 3600:
        return f'{int(seconds//60)}m{int(seconds%60)}s'
    else:
        h = int(seconds // 3600)
        m = int((seconds % 3600) // 60)
        return f'{h}h{m}m'


def print_progress(collected, total, elapsed, time_per_sample, all_frames):
    remaining = total - collected
    eta_sec   = remaining * time_per_sample if time_per_sample > 0 else 0
    pct       = collected / total * 100
    bar_len   = 30
    filled    = int(bar_len * collected / total)
    bar       = '█' * filled + '░' * (bar_len - filled)
    cams_str  = '+'.join(all_frames.keys())
    print(
        f'\r  📸 [{bar}] {collected}/{total} ({pct:.0f}%)'
        f'  ⏱ {format_time(elapsed)} écoulé'
        f'  🕐 {format_time(eta_sec)} restant'
        f'  ⚡ {time_per_sample:.1f}s/pose'
        f'  📷 {cams_str}',
        end='', flush=True
    )


# ---------------------------------------------------------------------------
# Robot bridge (TCP → Pi)
# ---------------------------------------------------------------------------
class RobotBridge:
    def __init__(self, host, port=5005, timeout=10.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock = None

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(self.timeout)
        self.sock.connect((self.host, self.port))
        print(f'  ✅ Robot bridge connected ({self.host}:{self.port})')

    def send_command(self, cmd):
        try:
            self.sock.sendall(json.dumps(cmd).encode() + b'\n')
            return self.sock.recv(4096).decode().strip()
        except Exception as e:
            return f'ERROR: {e}'

    def send_angles(self, angles_deg, speed=30):
        return self.send_command({'action': 'send_angles', 'angles': angles_deg, 'speed': speed})

    def get_angles(self):
        resp = self.send_command({'action': 'get_angles'})
        if 'ANGLES' in resp:
            m = re.search(r'\[(.*?)\]', resp)
            if m:
                return [float(x.strip()) for x in m.group(1).split(',')]
        return []

    def go_home(self):
        return self.send_command({'action': 'go_home'})

    def close(self):
        if self.sock:
            self.sock.close()


# ---------------------------------------------------------------------------
# Astra camera (oni_grabber + shared memory)
# ---------------------------------------------------------------------------
class AstraCamera:
    """
    Reads RGB frames from /dev/shm/oni_color.rgb written by oni_grabber.
    Spawns oni_grabber automatically if not running.
    Architecture by Dr. José BERNARDO / ABMI team (orbbec_capture.py).
    """

    def __init__(self, width=640, height=480):
        self.width  = width
        self.height = height
        self._proc  = None
        self.ok     = False
        self._last  = None

        # Clean old shm
        for f in [ONI_COLOR, ONI_TICK, ONI_INFO]:
            if f.exists():
                try:
                    f.unlink()
                except Exception:
                    pass

        # Check oni_grabber exists
        if not os.path.isfile(ONI_GRABBER):
            print(f'  ❌ Astra: oni_grabber not found at {ONI_GRABBER}')
            return

        # Spawn oni_grabber
        print('  🔄 Astra: spawning oni_grabber…')
        self._proc = subprocess.Popen(
            [ONI_GRABBER],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL
        )

        # Wait for first frame (max 5s)
        for _ in range(50):
            time.sleep(0.1)
            if ONI_COLOR.exists() and ONI_COLOR.stat().st_size == width * height * 3:
                self.ok = True
                break

        if self.ok:
            print(f'  ✅ Astra (oni_grabber) ready — {width}x{height}')
        else:
            print('  ❌ Astra: no frame from oni_grabber after 5s')
            self.close()

    def capture(self):
        if not self.ok or not ONI_COLOR.exists():
            return self._last
        expected = self.width * self.height * 3
        try:
            data = ONI_COLOR.read_bytes()
        except Exception:
            return self._last
        # oni_grabber rewrites the file in place; a read landing mid-write
        # returns a short/partial buffer. Skip it and keep the last good frame.
        if len(data) != expected:
            return self._last
        self._last = np.frombuffer(data, dtype=np.uint8).reshape(
            (self.height, self.width, 3)).copy()
        return self._last

    def close(self):
        if self._proc:
            self._proc.terminate()
            try:
                self._proc.wait(timeout=3)
            except Exception:
                self._proc.kill()
            self._proc = None
        for f in [ONI_COLOR, ONI_TICK, ONI_INFO]:
            if f.exists():
                try:
                    f.unlink()
                except Exception:
                    pass


# ---------------------------------------------------------------------------
# Local camera (ArduCam via V4L2)
# ---------------------------------------------------------------------------
class LocalCamera:
    def __init__(self, index, name, width=640, height=480, fourcc=None, exposure=None, focus=None):
        self.name = name
        self.index = index
        dev = resolve_v4l2(index) if isinstance(index, str) else index
        label = f'video{index}' if isinstance(index, int) else os.path.basename(dev)
        self.cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if fourcc:
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)  # ask the driver for the smallest queue
        time.sleep(2)
        for _ in range(5):
            self.cap.read()
        # Apply exposure only once the stream is live (a V4L2 control set before
        # the first read() can be reset by the driver on stream start).
        if exposure is not None:
            set_arducam_exposure(index, exposure)
            for _ in range(5):
                self.cap.read()
        if focus is not None:
            set_svpro_focus(index, focus)
            for _ in range(5):
                self.cap.read()
        if self.cap.isOpened():
            ret, _ = self.cap.read()
            if ret:
                print(f'  ✅ {name} ({label}) opened')
            else:
                print(f'  ⚠️  {name} ({label}) opened but no frame')
        else:
            print(f'  ❌ {name} ({label}) failed to open')

    def capture(self, flush=5):
        # USB/V4L2 cameras queue several frames: right after the robot moves,
        # the first read() returns a stale buffered frame from the previous
        # pose. Drop the queued frames first so the image matches the current
        # pose (BUFFERSIZE=1 is not honoured by every driver).
        for _ in range(flush):
            self.cap.grab()
        ret, frame = self.cap.read()
        if not ret:
            return None
        return cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    def close(self):
        self.cap.release()


# ---------------------------------------------------------------------------
# Save image
# ---------------------------------------------------------------------------
def save_image(img_array, path):
    from PIL import Image
    os.makedirs(os.path.dirname(path), exist_ok=True)
    Image.fromarray(img_array).save(path)


# ---------------------------------------------------------------------------
# Live preview of the exact sources that will be captured
# ---------------------------------------------------------------------------
def preview_cameras(cameras, panel_h=480):
    """Live side-by-side window of the real capture sources.

    ENTER → start capture, q/ESC → abort. Returns True to proceed.
    cam.capture() returns RGB; OpenCV windows expect BGR.
    """
    win = '3-cam preview — ENTER: start capture   q: abort'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    print('\n  👁  Preview — ENTER to start capture, q/ESC to abort')
    while True:
        panels = []
        for name, cam in cameras.items():
            frame = cam.capture()
            if frame is None:
                panel = np.zeros((panel_h, panel_h, 3), dtype=np.uint8)
                cv2.putText(panel, f'{name}: NO FRAME', (10, panel_h // 2),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            else:
                bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                h, w = bgr.shape[:2]
                panel = cv2.resize(bgr, (int(w * panel_h / h), panel_h))
                cv2.putText(panel, f'{name} {w}x{h}', (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
            panels.append(panel)

        cv2.imshow(win, np.hstack(panels))
        key = cv2.waitKey(30) & 0xFF
        if key == 13:                      # ENTER → start
            cv2.destroyWindow(win)
            return True
        if key in (ord('q'), 27):          # q / ESC → abort
            cv2.destroyWindow(win)
            return False


def cooldown_countdown(minutes):
    """Live MM:SS countdown so the operator can let the servos cool between
    sessions. Ctrl+C skips it (the robot is already home and idle)."""
    if minutes <= 0:
        return
    total = int(round(minutes * 60))
    print(f'\n  🌡️  Refroidissement robot — {minutes:.0f} min '
          f'(servos chauds après {minutes:.0f} min de mouvement). Ctrl+C pour passer.')
    try:
        for remaining in range(total, 0, -1):
            m, s = divmod(remaining, 60)
            print(f'\r  ⏳ {m:02d}:{s:02d} restant avant la prochaine session   ',
                  end='', flush=True)
            time.sleep(1)
        print('\r  ✅ Refroidissement terminé — prêt pour la prochaine session.        ')
    except KeyboardInterrupt:
        print('\n  ⏭️  Refroidissement interrompu — à toi de juger si le robot est froid.')


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(description='Capture 3 cameras simultaneously')
    parser.add_argument('--output',         default=os.path.expanduser(
                        '~/Osama_ws/src/mycobot_R6A/training/dream_data/real_3cam'),
                        help='Accumulating dataset dir. Each run appends --num-samples NEW poses, '
                             'continuing the index from the previous session.')
    parser.add_argument('--num-samples',    type=int,   default=500,
                        help='NEW poses to collect THIS session (delta, not a total). '
                             'Session 2 continues from where session 1 stopped.')
    parser.add_argument('--cooldown-min',   type=float, default=60.0,
                        help='Cooldown countdown in minutes shown after capture so the servos cool. '
                             '0 = skip. Ctrl+C skips it live.')
    parser.add_argument('--pi-host',        default='10.10.0.221', help='Raspberry Pi IP (robot bridge, port 5005)')
    parser.add_argument('--arducam-index',  type=int_or_path, default=0, help='V4L2 index or /dev/v4l/by-id path of the ArduCam')
    parser.add_argument('--svpro-index',    type=int_or_path, default=3, help='V4L2 index or /dev/v4l/by-id path of the SVPRO 5MP USB camera')
    parser.add_argument('--arducam-exposure', type=int, default=75,
                        help='ArduCam exposure_time_absolute (manual, brightness=0). 75 ~ session4 tone (mean~86, soft shadows). Lower=darker')
    parser.add_argument('--svpro-focus',    type=int, default=90,
                        help='SVPRO manual focus_absolute (autofocus off). 90 = sharpest on wrist/LED; avoid 128-930 (blurry)')
    parser.add_argument('--speed',          type=int,   default=25)
    parser.add_argument('--settle-time',    type=float, default=3.0)
    parser.add_argument('--limit-fraction', type=float, default=0.5)
    parser.add_argument('--no-astra',       action='store_true', help='Skip Astra camera')
    parser.add_argument('--preview',        action='store_true',
                        help='Show a live window of the 3 sources first; ENTER starts capture, q aborts')
    args = parser.parse_args()

    print('=' * 60)
    print('📸 Real Data Capture — 3 Cameras')
    print('   ArduCam + SVPRO (USB/V4L2) + Astra (oni_grabber) — all on PC Tour')
    print('=' * 60)

    # --- Open local cameras (all on PC Tour) — no robot needed yet ---
    arducam = LocalCamera(args.arducam_index, 'arducam', exposure=args.arducam_exposure)
    svpro   = LocalCamera(args.svpro_index, 'svpro', width=640, height=480, fourcc='MJPG', focus=args.svpro_focus)
    astra   = None if args.no_astra else AstraCamera()

    cameras = {'arducam': arducam, 'svpro': svpro}
    if astra and astra.ok:
        cameras['astra'] = astra

    print(f'\n  🎥 Cameras : {list(cameras.keys())}')

    # --- Refuse to start if two V4L2 indices are the same physical sensor ---
    if not assert_distinct_cameras(cameras):
        for cam in cameras.values():
            cam.close()
        return

    # --- Live preview before touching the robot (ENTER to start, q to abort) ---
    if args.preview:
        if not preview_cameras(cameras):
            print('  ⏹️  Preview aborted — no capture, robot untouched.')
            for cam in cameras.values():
                cam.close()
            return

    # --- Connect robot bridge (Pi) — only now that we commit to capturing ---
    bridge = RobotBridge(args.pi_host)
    try:
        bridge.connect()
    except Exception as e:
        print(f'❌ Robot bridge: {e}')
        for cam in cameras.values():
            cam.close()
        return

    # --- Output dirs ---
    all_cam_names = list(cameras.keys())
    for cname in all_cam_names:
        os.makedirs(os.path.join(args.output, 'images', cname), exist_ok=True)

    csv_path      = os.path.join(args.output, 'labels.csv')
    manifest_path = os.path.join(args.output, 'sessions.json')

    # Where the accumulating dataset left off (continuity across sessions).
    start_idx = 0
    if os.path.exists(csv_path):
        with open(csv_path) as f:
            rows = list(csv.reader(f))
            if len(rows) > 1:
                start_idx = int(rows[-1][0]) + 1

    sessions = []
    if os.path.exists(manifest_path):
        with open(manifest_path) as f:
            sessions = json.load(f)
    session_num = len(sessions) + 1

    if start_idx > 0:
        print(f'  ♻️  Session {session_num} — reprend le dataset à l\'index {start_idx} '
              f'(session {session_num - 1} s\'est arrêtée là). Aucune pose répétée.')
    else:
        print(f'  🆕 Session {session_num} — nouveau dataset')

    if start_idx == 0:
        with open(csv_path, 'w', newline='') as f:
            csv.writer(f).writerow([
                'index','j1_rad','j2_rad','j3_rad','j4_rad','j5_rad','j6_rad',
                'j1_deg','j2_deg','j3_deg','j4_deg','j5_deg','j6_deg',
                'camera','image_path'
            ])

    # Delta semantics: collect num_samples NEW poses THIS session, continuing
    # the global index. Session 2 picks up exactly where session 1 stopped.
    end_idx = start_idx + args.num_samples

    # --- Go home ---
    print('  🏠 Sending robot home…')
    bridge.go_home()
    time.sleep(3)

    total_to_collect = args.num_samples
    print(f'\n🚀 Session {session_num}: collecting {total_to_collect} samples '
          f'(index {start_idx}→{end_idx - 1}) × {len(all_cam_names)} cameras'
          f' = {total_to_collect * len(all_cam_names)} images total')
    print(f'   Settle time : {args.settle_time}s/pose')
    print(f'   ETA estimée : ~{format_time(total_to_collect * (args.settle_time + 1.5))}')
    print()

    collected            = 0
    session_start        = time.time()
    times_per_sample     = []
    cam_settings_written = set()

    try:
        for i in range(start_idx, end_idx):
            t0 = time.time()

            # 1. Random pose
            target = random_joint_angles(args.limit_fraction)
            bridge.send_angles(target, args.speed)
            time.sleep(args.settle_time)

            # 2. Read actual angles
            actual_deg = bridge.get_angles()
            if not actual_deg or len(actual_deg) != 6:
                actual_deg = target
            actual_rad = [math.radians(a) for a in actual_deg]

            # 3. Capture all cameras
            all_frames = {}
            for cname, cam in cameras.items():
                frame = cam.capture()
                if frame is not None:
                    all_frames[cname] = frame

            if not all_frames:
                print(f'\n  [{i}] ⚠️  No images — skipping')
                continue

            # Belt-and-suspenders: two cameras must never return the same frame.
            names = list(all_frames)
            for a in range(len(names)):
                for b in range(a + 1, len(names)):
                    fa, fb = all_frames[names[a]], all_frames[names[b]]
                    if fa.shape == fb.shape and np.array_equal(fa, fb):
                        print(f'\n  [{i}] ⚠️  {names[a]} and {names[b]} are identical '
                              f'— same physical source? skipping pose')
                        all_frames = None
                        break
                if all_frames is None:
                    break
            if all_frames is None:
                continue

            # 4b. Write per-camera intrinsics once, scaled to the real frame size
            for cname, frame in all_frames.items():
                if cname not in cam_settings_written:
                    h, w = frame.shape[:2]
                    write_camera_settings(cname, args.output, w, h)
                    cam_settings_written.add(cname)

            # 5. Save
            with open(csv_path, 'a', newline='') as f:
                writer = csv.writer(f)
                for cname, frame in all_frames.items():
                    img_rel  = f'images/{cname}/{i:06d}.png'
                    img_path = os.path.join(args.output, img_rel)
                    save_image(frame, img_path)
                    writer.writerow([
                        i,
                        *[round(a, 4) for a in actual_rad],
                        *[round(a, 2) for a in actual_deg],
                        cname, img_rel
                    ])

            # 6. Timing
            t1  = time.time()
            dt  = t1 - t0
            times_per_sample.append(dt)
            avg_time = sum(times_per_sample[-20:]) / len(times_per_sample[-20:])

            collected += 1
            elapsed    = t1 - session_start

            print_progress(collected, total_to_collect, elapsed, avg_time, all_frames)

            if collected % 10 == 0:
                print(f'\n  ✅ [{collected}/{total_to_collect}]'
                      f'  angles={[round(a,1) for a in actual_deg]}'
                      f'  {dt:.1f}s/pose')

    except KeyboardInterrupt:
        print(f'\n\n⏹️  Stopped at {collected} samples')

    finally:
        elapsed_total = time.time() - session_start
        print(f'\n\n  🏠 Returning home…')
        try:
            bridge.go_home()
        except Exception:
            pass
        bridge.close()
        for cam in cameras.values():
            cam.close()

    avg_final    = sum(times_per_sample) / len(times_per_sample) if times_per_sample else 0
    total_images = collected * len(all_cam_names)

    # Record this session in the manifest (traceability per session within the
    # single accumulating dataset).
    if collected > 0:
        sessions.append({
            'session':        session_num,
            'start_index':    start_idx,
            'end_index':      start_idx + collected - 1,
            'poses':          collected,
            'cameras':        all_cam_names,
            'limit_fraction': args.limit_fraction,
            'timestamp':      datetime.now().isoformat(timespec='seconds'),
        })
        with open(manifest_path, 'w') as f:
            json.dump(sessions, f, indent=2)

    idx_range = f'index {start_idx}→{start_idx + collected - 1}' if collected else 'aucune pose'
    print(f'\n{"="*60}')
    print(f'✅ Session {session_num} terminée !')
    print(f'   📁 Dataset     : {args.output}')
    print(f'   📸 Poses       : {collected}/{total_to_collect}  ({idx_range})')
    print(f'   🖼  Images      : {total_images} ({len(all_cam_names)} caméras)')
    print(f'   📊 Total dataset: {start_idx + collected} poses cumulées ({len(sessions)} sessions)')
    print(f'   ⏱  Durée totale: {format_time(elapsed_total)}')
    print(f'   ⚡ Moy/pose    : {avg_final:.1f}s')
    print(f'{"="*60}')

    # Cooldown countdown so the servos cool before the next session.
    if collected > 0:
        cooldown_countdown(args.cooldown_min)


if __name__ == '__main__':
    main()