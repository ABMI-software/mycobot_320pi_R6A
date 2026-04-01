#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Pi Camera Server — lightweight TCP image server for real data capture.

Runs on the Raspberry Pi alongside bridge_pi_simple.py.
Serves JPEG-encoded frames from USB Arducam cameras to the PC Tour.

Protocol (request/response, newline-delimited JSON):
  → {"action": "list_cameras"}
  ← {"cameras": [0, 2], "names": {"0": "cam0", "2": "cam1"}}

  → {"action": "capture", "camera": 0}
  ← {"ok": true, "width": 640, "height": 480, "size": 23456}
     <23456 raw bytes of JPEG data>

  → {"action": "capture_all"}
  ← {"ok": true, "cameras": [0, 2], "sizes": [23456, 24567]}
     <23456 bytes for cam 0><24567 bytes for cam 1>

  → {"action": "ping"}
  ← {"ok": true, "msg": "pong"}

Usage on the Pi::

    python3 pi_camera_server.py
    python3 pi_camera_server.py --cameras 0 2 --port 5006
    python3 pi_camera_server.py --cameras 0 2 --names front side --width 640 --height 480
"""

import argparse
import json
import socket
import sys
import time


def detect_cameras(max_index: int = 10):
    """Auto-detect available camera indices using OpenCV."""
    import cv2
    found = []
    for i in range(max_index):
        cap = cv2.VideoCapture(i)
        if cap.isOpened():
            ret, _ = cap.read()
            if ret:
                found.append(i)
            cap.release()
        else:
            cap.release()
    return found


class PiCameraServer:
    """Simple TCP server that streams JPEG frames from USB cameras."""

    def __init__(self, camera_indices, camera_names=None,
                 width=640, height=480, host='0.0.0.0', port=5006):
        import cv2
        self.cv2 = cv2
        self.host = host
        self.port = port
        self.width = width
        self.height = height
        self.running = False

        # Open cameras
        self.caps = {}
        self.cam_names = {}  # index → name
        for i, idx in enumerate(camera_indices):
            cap = cv2.VideoCapture(idx)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
            if cap.isOpened():
                self.caps[idx] = cap
                name = camera_names[i] if camera_names and i < len(camera_names) else f'cam{idx}'
                self.cam_names[idx] = name
                print(f'  📷 Camera {idx} ({name}) opened')
            else:
                print(f'  ⚠️  Camera {idx} failed to open — skipping')
                cap.release()

        if not self.caps:
            raise RuntimeError('No cameras available')

    def capture_jpeg(self, cam_idx, quality=90):
        """Capture a frame and return JPEG bytes."""
        cap = self.caps.get(cam_idx)
        if cap is None:
            return None
        ret, frame = cap.read()
        if not ret:
            return None
        ok, jpeg = self.cv2.imencode('.jpg', frame,
                                     [self.cv2.IMWRITE_JPEG_QUALITY, quality])
        if not ok:
            return None
        return jpeg.tobytes()

    def handle_client(self, conn, addr):
        """Handle one client connection."""
        print(f'  ✅ Client connected: {addr}')
        conn.settimeout(None)
        buf = b''

        try:
            while self.running:
                data = conn.recv(4096)
                if not data:
                    break
                buf += data

                while b'\n' in buf:
                    line, buf = buf.split(b'\n', 1)
                    line = line.strip()
                    if not line:
                        continue
                    try:
                        req = json.loads(line.decode('utf-8'))
                    except (json.JSONDecodeError, UnicodeDecodeError):
                        resp = json.dumps({'ok': False, 'error': 'invalid json'})
                        conn.sendall(resp.encode() + b'\n')
                        continue

                    self._dispatch(conn, req)

        except (ConnectionResetError, BrokenPipeError):
            pass
        except Exception as e:
            print(f'  ❌ Client error: {e}')
        finally:
            conn.close()
            print(f'  🔄 Client {addr} disconnected')

    def _dispatch(self, conn, req):
        action = req.get('action', '')

        if action == 'ping':
            resp = json.dumps({'ok': True, 'msg': 'pong'})
            conn.sendall(resp.encode() + b'\n')

        elif action == 'list_cameras':
            indices = sorted(self.caps.keys())
            names = {str(k): v for k, v in self.cam_names.items()}
            resp = json.dumps({'cameras': indices, 'names': names})
            conn.sendall(resp.encode() + b'\n')

        elif action == 'capture':
            cam_idx = req.get('camera', sorted(self.caps.keys())[0])
            quality = req.get('quality', 90)
            jpeg = self.capture_jpeg(cam_idx, quality)
            if jpeg is None:
                resp = json.dumps({'ok': False, 'error': f'capture failed for camera {cam_idx}'})
                conn.sendall(resp.encode() + b'\n')
            else:
                resp = json.dumps({
                    'ok': True,
                    'camera': cam_idx,
                    'name': self.cam_names.get(cam_idx, ''),
                    'width': self.width,
                    'height': self.height,
                    'size': len(jpeg),
                })
                conn.sendall(resp.encode() + b'\n')
                conn.sendall(jpeg)

        elif action == 'capture_all':
            quality = req.get('quality', 90)
            indices = sorted(self.caps.keys())
            jpegs = []
            sizes = []
            names = []
            for idx in indices:
                jpeg = self.capture_jpeg(idx, quality)
                if jpeg is not None:
                    jpegs.append(jpeg)
                    sizes.append(len(jpeg))
                    names.append(self.cam_names.get(idx, f'cam{idx}'))
                else:
                    jpegs.append(b'')
                    sizes.append(0)
                    names.append(self.cam_names.get(idx, f'cam{idx}'))

            resp = json.dumps({
                'ok': True,
                'cameras': indices,
                'names': names,
                'sizes': sizes,
            })
            conn.sendall(resp.encode() + b'\n')
            for jpeg_data in jpegs:
                if jpeg_data:
                    conn.sendall(jpeg_data)

        else:
            resp = json.dumps({'ok': False, 'error': f'unknown action: {action}'})
            conn.sendall(resp.encode() + b'\n')

    def start(self):
        """Run the server (blocking)."""
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        srv.bind((self.host, self.port))
        srv.listen(1)
        self.running = True

        print(f'\n🌐 Pi Camera Server listening on {self.host}:{self.port}')
        print(f'   Cameras: {list(self.caps.keys())}')
        print(f'   Resolution: {self.width}×{self.height}')
        print(f'   Names: {self.cam_names}\n')

        try:
            while self.running:
                conn, addr = srv.accept()
                self.handle_client(conn, addr)
        except KeyboardInterrupt:
            print('\n⏹️  Shutting down…')
        finally:
            self.running = False
            for cap in self.caps.values():
                cap.release()
            srv.close()


def main():
    parser = argparse.ArgumentParser(
        description='Pi Camera Server — stream USB camera frames over TCP')
    parser.add_argument('--cameras', type=int, nargs='*', default=None,
                        help='Camera indices to use (default: auto-detect)')
    parser.add_argument('--names', nargs='*', default=None,
                        help='Camera names (matching --cameras order)')
    parser.add_argument('--width', type=int, default=640)
    parser.add_argument('--height', type=int, default=480)
    parser.add_argument('--host', default='0.0.0.0')
    parser.add_argument('--port', type=int, default=5006)
    args = parser.parse_args()

    print('=' * 50)
    print('📷 Pi Camera Server')
    print('=' * 50)

    # Auto-detect cameras if not specified
    if args.cameras is None:
        print('🔍 Auto-detecting cameras…')
        import cv2
        indices = detect_cameras()
        if not indices:
            print('❌ No cameras found!')
            sys.exit(1)
        print(f'   Found cameras at indices: {indices}')
    else:
        indices = args.cameras

    server = PiCameraServer(
        camera_indices=indices,
        camera_names=args.names,
        width=args.width,
        height=args.height,
        host=args.host,
        port=args.port,
    )
    server.start()


if __name__ == '__main__':
    main()
