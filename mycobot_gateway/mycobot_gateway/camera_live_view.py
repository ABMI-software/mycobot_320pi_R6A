#!/usr/bin/env python3
"""Live camera viewer for quick field diagnostics.

Layout can be configured with CAM_LIVE_LAYOUT:
- single (default): only /camera/color/image_raw
- split: left /camera/color/image_raw + right /aruco/debug_image
"""

from __future__ import annotations

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


class CameraLiveView(Node):
    def __init__(self) -> None:
        super().__init__("camera_live_view")
        self.bridge = CvBridge()
        self.raw = None
        self.debug = None
        self.layout = os.environ.get("CAM_LIVE_LAYOUT", "single").strip().lower()
        if self.layout not in {"single", "split"}:
            self.layout = "single"

        dict_name = os.environ.get("CAM_LIVE_ARUCO_DICT", "DICT_4X4_1000").strip()
        if dict_name not in _ARUCO_DICT_BY_NAME:
            dict_name = "DICT_4X4_1000"
        self.aruco_dict_name = dict_name

        self.detect_scale = float(os.environ.get("CAM_LIVE_ARUCO_SCALE", "1.8"))

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
        else:
            self._detector = None

        self.create_subscription(Image, "/camera/color/image_raw", self._cb_raw, 10)
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

    def annotate_aruco(self, frame: np.ndarray) -> np.ndarray:
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

        status = f"ArUco {self.aruco_dict_name} | IDs: {ids_list if ids_list else 'none'}"
        cv2.putText(out, status, (10, out.shape[0] - 12), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
        return out


def _placeholder(text: str, color: tuple[int, int, int]) -> np.ndarray:
    img = np.zeros((480, 640, 3), dtype=np.uint8)
    cv2.putText(img, text, (25, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2)
    return img


def main(args=None) -> None:
    if not os.environ.get("DISPLAY") and not os.environ.get("WAYLAND_DISPLAY"):
        print("No GUI display detected (DISPLAY/WAYLAND_DISPLAY not set).")
        print("Run this command from a graphical local session.")
        return

    rclpy.init(args=args)
    node = CameraLiveView()

    win = "Camera Live - press q to quit"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, 1400, 520)

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.03)

            left = node.raw if node.raw is not None else _placeholder("No /camera/color/image_raw", (0, 0, 255))
            if node.raw is not None:
                left = node.annotate_aruco(left)
            if node.layout == "single":
                frame = left
                cv2.putText(frame, "Source: /camera/color/image_raw", (12, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            else:
                right = node.debug if node.debug is not None else _placeholder("No /aruco/debug_image", (0, 165, 255))
                if left.shape[:2] != right.shape[:2]:
                    right = cv2.resize(right, (left.shape[1], left.shape[0]))
                frame = np.hstack([left, right])
            cv2.imshow(win, frame)
            if (cv2.waitKey(1) & 0xFF) == ord("q"):
                break
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
