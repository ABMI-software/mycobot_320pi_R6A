#!/usr/bin/env python3
"""Web live viewer for ROS camera topic.

Serves /camera/color/image_raw as MJPEG on http://127.0.0.1:8090
Useful when DISPLAY is not available.

Layout can be configured with CAM_WEB_LAYOUT:
    - single (default): only /camera/color/image_raw
    - split: left raw + right /aruco/debug_image
    - dual: left /camera/color/image_raw + right /camera/image_raw (cam_0)
"""

from __future__ import annotations

import threading
import time
from http.server import BaseHTTPRequestHandler, HTTPServer
import os

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


_ARUCO_DICT_BY_NAME: dict[str, int] = {}
for _name in [
    "DICT_4X4_50",
    "DICT_4X4_100",
    "DICT_4X4_250",
    "DICT_4X4_1000",
    "DICT_5X5_50",
    "DICT_5X5_100",
    "DICT_5X5_250",
    "DICT_5X5_1000",
    "DICT_6X6_50",
    "DICT_6X6_100",
    "DICT_6X6_250",
    "DICT_6X6_1000",
    "DICT_7X7_50",
    "DICT_7X7_100",
    "DICT_7X7_250",
    "DICT_7X7_1000",
    "DICT_APRILTAG_16h5",
    "DICT_APRILTAG_25h9",
    "DICT_APRILTAG_36h10",
    "DICT_APRILTAG_36h11",
]:
    if hasattr(cv2.aruco, _name):
        _ARUCO_DICT_BY_NAME[_name] = getattr(cv2.aruco, _name)


_latest_jpg: bytes | None = None
_lock = threading.Lock()


def _placeholder_jpg() -> bytes:
    img = np.zeros((480, 1280, 3), dtype=np.uint8)
    cv2.putText(
        img,
        "Waiting for /camera/color/image_raw ...",
        (30, 250),
        cv2.FONT_HERSHEY_SIMPLEX,
        1.0,
        (0, 165, 255),
        2,
    )
    ok, buf = cv2.imencode(".jpg", img, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
    if ok:
        return buf.tobytes()
    return b""


class _CamNode(Node):
    def __init__(self) -> None:
        super().__init__("camera_web_view")
        self.bridge = CvBridge()
        self.raw = None
        self.debug = None
        self.cam0 = None
        self.layout = os.environ.get("CAM_WEB_LAYOUT", "single").strip().lower()
        if self.layout not in {"single", "split", "dual"}:
            self.layout = "single"
        self.dual_use_debug = os.environ.get("CAM_WEB_DUAL_USE_DEBUG", "1").strip().lower() not in {"0", "false", "no"}
        self._aruco_internal = self.layout == "dual" and (not self.dual_use_debug)

        dict_name = os.environ.get("CAM_WEB_ARUCO_DICT", "DICT_4X4_1000").strip()
        if dict_name not in _ARUCO_DICT_BY_NAME:
            dict_name = "DICT_4X4_1000"
        self.aruco_dict_name = dict_name
        self.detect_scale = float(os.environ.get("CAM_WEB_ARUCO_SCALE", "1.8"))
        self._aruco_dict = None
        self._aruco_params = None
        self._use_new_aruco_api = False
        self._detector = None
        if self._aruco_internal:
            self._aruco_dict = cv2.aruco.getPredefinedDictionary(_ARUCO_DICT_BY_NAME[self.aruco_dict_name])
            if hasattr(cv2.aruco, "DetectorParameters"):
                self._aruco_params = cv2.aruco.DetectorParameters()
            else:
                self._aruco_params = cv2.aruco.DetectorParameters_create()
            self._aruco_params.adaptiveThreshWinSizeMin = 3
            self._aruco_params.adaptiveThreshWinSizeMax = 61
            self._aruco_params.adaptiveThreshWinSizeStep = 4
            self._aruco_params.minMarkerPerimeterRate = 0.01
            self._aruco_params.maxMarkerPerimeterRate = 4.0
            if hasattr(cv2.aruco, "CORNER_REFINE_SUBPIX"):
                self._aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
            self._use_new_aruco_api = hasattr(cv2.aruco, "ArucoDetector")
            if self._use_new_aruco_api:
                self._detector = cv2.aruco.ArucoDetector(self._aruco_dict, self._aruco_params)

        self.left_topic = os.environ.get("CAM_WEB_LEFT_TOPIC", "/camera/color/image_raw")
        self.cam0_topic = os.environ.get("CAM_WEB_CAM0_TOPIC", "/camera/image_raw")
        self.create_subscription(Image, self.left_topic, self._cb_raw, 10)
        self.create_subscription(Image, self.cam0_topic, self._cb_cam0, 10)
        self.create_subscription(Image, "/aruco/debug_image", self._cb_debug, 10)

    def _cb_raw(self, msg: Image) -> None:
        try:
            self.raw = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            self.raw = None

    def _cb_debug(self, msg: Image) -> None:
        try:
            self.debug = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            self.debug = None

    def _cb_cam0(self, msg: Image) -> None:
        try:
            self.cam0 = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            self.cam0 = None

    def _placeholder(self, text: str, shape_like: np.ndarray | None = None) -> np.ndarray:
        if shape_like is not None:
            h, w = shape_like.shape[:2]
        else:
            h, w = 480, 640
        img = np.zeros((h, w, 3), dtype=np.uint8)
        cv2.putText(img, text, (20, min(240, h // 2)), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 165, 255), 2)
        return img

    def _annotate_aruco(self, frame: np.ndarray) -> np.ndarray:
        if not self._aruco_internal or self._aruco_dict is None or self._aruco_params is None:
            return frame
        out = frame.copy()
        gray = cv2.cvtColor(out, cv2.COLOR_BGR2GRAY)

        src = gray
        inv_scale = 1.0
        if self.detect_scale > 1.01:
            src = cv2.resize(gray, dsize=None, fx=self.detect_scale, fy=self.detect_scale, interpolation=cv2.INTER_LINEAR)
            inv_scale = 1.0 / self.detect_scale

        if self._use_new_aruco_api and self._detector is not None:
            corners, ids, _ = self._detector.detectMarkers(src)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(src, self._aruco_dict, parameters=self._aruco_params)

        if inv_scale != 1.0 and corners is not None:
            corners = [(corner * inv_scale).astype(np.float32) for corner in corners]

        ids_list: list[int] = []
        if ids is not None and len(ids) > 0:
            cv2.aruco.drawDetectedMarkers(out, corners, ids)
            ids_list = [int(x) for x in ids.flatten().tolist()]
            for i, mid in enumerate(ids_list):
                c = corners[i][0].astype(int)
                cx = int(np.mean(c[:, 0]))
                cy = int(np.mean(c[:, 1]))
                cv2.putText(out, f"ID:{mid}", (cx - 20, cy - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 255, 0), 2)

        status = f"cam_0 ArUco {self.aruco_dict_name} | IDs: {ids_list if ids_list else 'none'}"
        cv2.putText(out, status, (10, out.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)
        return out

    def compose_frame(self) -> bytes | None:
        global _latest_jpg
        try:
            if self.raw is None:
                return None

            if self.layout == "single":
                frame = self.raw.copy()
                cv2.putText(
                    frame,
                    "Source: /camera/color/image_raw",
                    (12, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )
                ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                if ok:
                    return buf.tobytes()
                return None

            left = self.raw.copy()
            if self.layout == "dual":
                if self.dual_use_debug and self.debug is not None:
                    right = self.debug.copy()
                    right_label = "/aruco/debug_image (cam_0 ArUco)"
                else:
                    right = self.cam0.copy() if self.cam0 is not None else self._placeholder(f"No {self.cam0_topic}", left)
                    right = self._annotate_aruco(right)
                    right_label = f"{self.cam0_topic} (ArUco internal)"
                if right.shape[:2] != left.shape[:2]:
                    right = cv2.resize(right, (left.shape[1], left.shape[0]))
                frame = np.hstack([left, right])
                cv2.putText(
                    frame,
                    f"Left: {self.left_topic}   Right: {right_label}",
                    (12, 30),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (0, 255, 0),
                    2,
                )
                ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
                if ok:
                    return buf.tobytes()
                return None

            if self.debug is None:
                right = np.zeros_like(left)
                cv2.putText(
                    right,
                    "No /aruco/debug_image",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.8,
                    (0, 165, 255),
                    2,
                )
            else:
                right = self.debug.copy()
                if right.shape[:2] != left.shape[:2]:
                    right = cv2.resize(right, (left.shape[1], left.shape[0]))

            frame = np.hstack([left, right])
            cv2.putText(
                frame,
                "Left: /camera/color/image_raw   Right: /aruco/debug_image",
                (12, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
            ok, buf = cv2.imencode(".jpg", frame, [int(cv2.IMWRITE_JPEG_QUALITY), 80])
            if ok:
                return buf.tobytes()
        except Exception:
            return None
        return None


class _Handler(BaseHTTPRequestHandler):
    def do_GET(self):
        if self.path in ("/", "/index.html"):
            html = (
                "<html><head><title>Camera Web View</title></head>"
                "<body style='margin:0;background:#101214;color:#e5e7eb;font-family:sans-serif;'>"
                "<div style='padding:10px 14px;'>Camera live MJPEG</div>"
                "<img src='/stream.mjpg' style='width:100%;height:auto;display:block'/>"
                "</body></html>"
            ).encode("utf-8")
            self.send_response(200)
            self.send_header("Content-Type", "text/html; charset=utf-8")
            self.send_header("Content-Length", str(len(html)))
            self.end_headers()
            self.wfile.write(html)
            return

        if self.path == "/stream.mjpg":
            self.send_response(200)
            self.send_header("Cache-Control", "no-cache, private")
            self.send_header("Pragma", "no-cache")
            self.send_header("Content-Type", "multipart/x-mixed-replace; boundary=frame")
            self.end_headers()
            try:
                while True:
                    with _lock:
                        jpg = _latest_jpg
                    if jpg is None:
                        jpg = _placeholder_jpg()
                    self.wfile.write(b"--frame\r\n")
                    self.wfile.write(b"Content-Type: image/jpeg\r\n")
                    self.wfile.write(f"Content-Length: {len(jpg)}\r\n\r\n".encode("utf-8"))
                    self.wfile.write(jpg)
                    self.wfile.write(b"\r\n")
                    time.sleep(0.03)
            except Exception:
                return

        self.send_error(404)

    def log_message(self, fmt, *args):
        return


def _spin_ros() -> None:
    global _latest_jpg
    rclpy.init()
    node = _CamNode()
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.03)
        jpg = node.compose_frame()
        if jpg is not None:
            with _lock:
                _latest_jpg = jpg
    node.destroy_node()
    rclpy.shutdown()


def main() -> None:
    host = os.environ.get("CAM_WEB_HOST", "0.0.0.0")
    port = int(os.environ.get("CAM_WEB_PORT", "8090"))
    thread = threading.Thread(target=_spin_ros, daemon=True)
    thread.start()
    server = HTTPServer((host, port), _Handler)
    print(f"Camera Web View running on http://{host}:{port}")
    server.serve_forever()


if __name__ == "__main__":
    main()
