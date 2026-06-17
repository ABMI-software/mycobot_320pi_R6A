#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Live preview of Pi Arducam cameras — check framing & quality.

Opens an OpenCV window showing both Pi cameras side-by-side.
Uses a buffered socket reader for speed.

Controls (in the OpenCV window):
  q / ESC    — quit
  s          — save snapshot
  g          — cycle grid overlay (none / center-crop / thirds)

Usage::

    python3 training/preview_cameras.py
    python3 training/preview_cameras.py --pi-host 10.10.0.225
"""

import argparse
import json
import socket
import time
from datetime import datetime
from typing import Dict, Optional

import cv2
import numpy as np


# ---------------------------------------------------------------------------
# Buffered socket reader — avoids slow recv(1) byte-by-byte
# ---------------------------------------------------------------------------
class BufferedSocketReader:
    """Wraps a socket with an internal buffer for efficient line reading."""

    def __init__(self, sock: socket.socket, bufsize: int = 131072):
        self.sock = sock
        self.buf = b''
        self.bufsize = bufsize

    def readline(self) -> str:
        """Read until newline. Returns decoded string WITHOUT the newline."""
        while b'\n' not in self.buf:
            chunk = self.sock.recv(self.bufsize)
            if not chunk:
                raise ConnectionError('Server disconnected')
            self.buf += chunk
        line, self.buf = self.buf.split(b'\n', 1)
        return line.decode('utf-8')

    def read_exact(self, n: int) -> bytes:
        """Read exactly n bytes."""
        while len(self.buf) < n:
            chunk = self.sock.recv(max(self.bufsize, n - len(self.buf)))
            if not chunk:
                raise ConnectionError('Server disconnected')
            self.buf += chunk
        data, self.buf = self.buf[:n], self.buf[n:]
        return data


# ---------------------------------------------------------------------------
# Fast camera client
# ---------------------------------------------------------------------------
class CameraClient:
    """TCP client for pi_camera_server — optimised for live preview."""

    def __init__(self, host: str, port: int = 5006, timeout: float = 10.0):
        self.host = host
        self.port = port
        self.timeout = timeout
        self.sock: Optional[socket.socket] = None
        self.reader: Optional[BufferedSocketReader] = None
        self.camera_names: Dict[int, str] = {}

    def connect(self):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(self.timeout)
        self.sock.connect((self.host, self.port))
        self.sock.settimeout(None)  # blocking for streaming
        self.sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
        self.reader = BufferedSocketReader(self.sock)
        self._list_cameras()

    def _send(self, obj: dict):
        self.sock.sendall(json.dumps(obj).encode() + b'\n')

    def _list_cameras(self):
        self._send({'action': 'list_cameras'})
        resp = json.loads(self.reader.readline())
        names_dict = resp.get('names', {})
        for idx in resp['cameras']:
            self.camera_names[idx] = names_dict.get(str(idx), f'cam{idx}')

    def capture_all(self, quality: int = 50) -> Dict[str, np.ndarray]:
        """Capture from all cameras. Returns {name: BGR ndarray}."""
        self._send({'action': 'capture_all', 'quality': quality})
        resp = json.loads(self.reader.readline())
        if not resp.get('ok'):
            return {}
        result = {}
        for name, sz in zip(resp['names'], resp['sizes']):
            if sz > 0:
                data = self.reader.read_exact(sz)
                frame = cv2.imdecode(
                    np.frombuffer(data, dtype=np.uint8), cv2.IMREAD_COLOR)
                if frame is not None:
                    result[name] = frame
        return result

    def close(self):
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None


# ---------------------------------------------------------------------------
# Overlay helpers
# ---------------------------------------------------------------------------
def draw_overlay(frame, cam_name, fps, grid_mode):
    """Draw info bar + optional grid guides on a frame."""
    h, w = frame.shape[:2]

    # --- Info bar (top) ---
    bar = frame[0:30, :].copy()
    cv2.rectangle(frame, (0, 0), (w, 30), (0, 0, 0), -1)
    cv2.addWeighted(frame[0:30, :], 0.5, bar, 0.5, 0, frame[0:30, :])
    txt = f'{cam_name}  {w}x{h}  {fps:.1f}fps'
    cv2.putText(frame, txt, (8, 20),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 200), 1, cv2.LINE_AA)

    # --- Grid overlay ---
    if grid_mode == 1:
        # Center 80% crop guide
        mx, my = int(w * 0.1), int(h * 0.1)
        cv2.rectangle(frame, (mx, my), (w - mx, h - my), (0, 255, 0), 1)
        cx, cy = w // 2, h // 2
        cv2.line(frame, (cx - 15, cy), (cx + 15, cy), (0, 255, 0), 1)
        cv2.line(frame, (cx, cy - 15), (cx, cy + 15), (0, 255, 0), 1)
    elif grid_mode == 2:
        # Rule of thirds
        for i in range(1, 3):
            cv2.line(frame, (w * i // 3, 0), (w * i // 3, h), (255, 255, 0), 1)
            cv2.line(frame, (0, h * i // 3), (w, h * i // 3), (255, 255, 0), 1)

    return frame


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser(description='Live preview of Pi cameras')
    ap.add_argument('--pi-host', default='10.10.0.225')
    ap.add_argument('--camera-port', type=int, default=5006)
    ap.add_argument('--quality', type=int, default=50,
                    help='JPEG quality for preview (lower = faster, default 50)')
    args = ap.parse_args()

    print('=' * 55)
    print('📷  Camera Preview — Pi Arducams')
    print('=' * 55)
    print(f'  Pi: {args.pi_host}:{args.camera_port}')
    print('  Keys: q=quit  s=snapshot  g=grid overlay')
    print('=' * 55)

    # --- Connect ---
    client = CameraClient(args.pi_host, args.camera_port)
    try:
        client.connect()
    except Exception as e:
        print(f'\n❌ Cannot connect: {e}')
        print('   → Start pi_camera_server.py on the Pi first')
        return

    names = list(client.camera_names.values())
    print(f'\n  📷 Cameras: {names}')
    print('  ▶  Live preview… (press q in the window to quit)\n')

    win = 'Pi Camera Preview'
    cv2.namedWindow(win, cv2.WINDOW_NORMAL | cv2.WINDOW_KEEPRATIO)
    cv2.resizeWindow(win, 640, 960)  # vertical: 2 cameras stacked

    grid_mode = 0
    fps_count = 0
    fps_t0 = time.time()
    fps_val = 0.0
    frame_num = 0

    try:
        while True:
            t0 = time.time()

            # --- Grab frames ---
            try:
                frames = client.capture_all(quality=args.quality)
            except (ConnectionError, OSError) as e:
                print(f'  ⚠️  Lost connection: {e} — reconnecting…')
                client.close()
                time.sleep(1)
                try:
                    client = CameraClient(args.pi_host, args.camera_port)
                    client.connect()
                    print('  ✅ Reconnected')
                except Exception:
                    pass
                frames = {}

            if not frames:
                ph = np.zeros((480, 640, 3), dtype=np.uint8)
                cv2.putText(ph, 'Waiting for cameras…', (100, 250),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
                cv2.imshow(win, ph)
                if cv2.waitKey(500) & 0xFF in (ord('q'), 27):
                    break
                continue

            # --- Annotate each frame ---
            frame_num += 1
            if frame_num <= 3:
                print(f'  [frame {frame_num}] got {len(frames)} cameras: '
                      f'{list(frames.keys())}, '
                      f'sizes={[f.shape for f in frames.values()]}')
            panels = []
            for name, frame in frames.items():
                panels.append((name, draw_overlay(frame, name, fps_val, grid_mode)))

            # --- Build mosaic (top/bottom for 2 cams, side-by-side for 3+) ---
            imgs = [p[1] for p in panels]
            if len(imgs) == 1:
                mosaic = imgs[0]
            elif len(imgs) == 2:
                # Stack vertically — fits better in a window
                # Match widths
                max_w = max(p.shape[1] for p in imgs)
                resized = []
                for p in imgs:
                    if p.shape[1] != max_w:
                        s = max_w / p.shape[1]
                        p = cv2.resize(p, (max_w, int(p.shape[0] * s)))
                    resized.append(p)
                sep = np.full((4, max_w, 3), (0, 200, 255), dtype=np.uint8)
                mosaic = np.vstack([resized[0], sep, resized[1]])
            else:
                # 2×N grid for 3+ cameras
                max_h = max(p.shape[0] for p in imgs)
                resized = []
                for p in imgs:
                    if p.shape[0] != max_h:
                        s = max_h / p.shape[0]
                        p = cv2.resize(p, (int(p.shape[1] * s), max_h))
                    resized.append(p)
                sep = np.full((max_h, 4, 3), (0, 200, 255), dtype=np.uint8)
                parts = []
                for i, r in enumerate(resized):
                    if i > 0:
                        parts.append(sep)
                    parts.append(r)
                mosaic = np.hstack(parts)

            cv2.imshow(win, mosaic)

            # --- FPS ---
            fps_count += 1
            if time.time() - fps_t0 >= 1.0:
                fps_val = fps_count / (time.time() - fps_t0)
                fps_count = 0
                fps_t0 = time.time()

            # --- Keys ---
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            elif key == ord('s'):
                ts = datetime.now().strftime('%Y%m%d_%H%M%S')
                p = f'/tmp/camera_snapshot_{ts}.png'
                cv2.imwrite(p, mosaic)
                print(f'  💾 Saved: {p}')
            elif key == ord('g'):
                grid_mode = (grid_mode + 1) % 3
                print(f'  🔲 Grid: {["none","center crop","thirds"][grid_mode]}')

    except KeyboardInterrupt:
        pass
    finally:
        client.close()
        cv2.destroyAllWindows()
        print('👋 Preview closed')


if __name__ == '__main__':
    main()
