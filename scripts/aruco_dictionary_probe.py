#!/usr/bin/env python3
"""Probe ArUco/AprilTag dictionary against a live ROS image topic.

Collects N frames from a ROS topic, runs marker detection for multiple
predefined dictionaries, and reports which dictionary is the most consistent.
"""

from __future__ import annotations

import argparse
import sys
import time
from collections import Counter, defaultdict

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


ARUCO_DICT_NAMES = [
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
]

APRILTAG_DICT_NAMES = [
    "DICT_APRILTAG_16h5",
    "DICT_APRILTAG_25h9",
    "DICT_APRILTAG_36h10",
    "DICT_APRILTAG_36h11",
]


class FrameGrabber(Node):
    def __init__(self, topic: str) -> None:
        super().__init__("aruco_dictionary_probe")
        self._bridge = CvBridge()
        self._latest: np.ndarray | None = None
        self.create_subscription(Image, topic, self._cb, 10)

    def _cb(self, msg: Image) -> None:
        try:
            self._latest = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            self._latest = None

    @property
    def latest(self) -> np.ndarray | None:
        return self._latest


def build_dict_map(include_apriltag: bool) -> dict[str, int]:
    result: dict[str, int] = {}
    names = list(ARUCO_DICT_NAMES)
    if include_apriltag:
        names.extend(APRILTAG_DICT_NAMES)
    for name in names:
        if hasattr(cv2.aruco, name):
            result[name] = getattr(cv2.aruco, name)
    return result


def build_params() -> object:
    if hasattr(cv2.aruco, "DetectorParameters"):
        p = cv2.aruco.DetectorParameters()
    else:
        p = cv2.aruco.DetectorParameters_create()

    p.adaptiveThreshWinSizeMin = 3
    p.adaptiveThreshWinSizeMax = 61
    p.adaptiveThreshWinSizeStep = 4
    p.minMarkerPerimeterRate = 0.01
    p.maxMarkerPerimeterRate = 4.0
    if hasattr(cv2.aruco, "CORNER_REFINE_SUBPIX"):
        p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return p


def detect_ids(gray: np.ndarray, dict_id: int, params: object, scale: float) -> list[int]:
    src = gray
    inv_scale = 1.0
    if scale > 1.01:
        src = cv2.resize(gray, dsize=None, fx=scale, fy=scale, interpolation=cv2.INTER_LINEAR)
        inv_scale = 1.0 / scale

    dictionary = cv2.aruco.getPredefinedDictionary(dict_id)
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, params)
        corners, ids, _ = detector.detectMarkers(src)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(src, dictionary, parameters=params)

    if inv_scale != 1.0 and corners is not None:
        corners = [(corner * inv_scale).astype(np.float32) for corner in corners]

    if ids is None:
        return []
    return [int(x) for x in ids.flatten().tolist()]


def main() -> int:
    ap = argparse.ArgumentParser(description="Probe best ArUco dictionary from live ROS camera frames")
    ap.add_argument("--topic", default="/camera/color/image_raw")
    ap.add_argument("--frames", type=int, default=30)
    ap.add_argument("--warmup-sec", type=float, default=1.5)
    ap.add_argument("--timeout-sec", type=float, default=20.0)
    ap.add_argument("--scale", type=float, default=1.8)
    ap.add_argument("--include-apriltag", action="store_true")
    args = ap.parse_args()

    dict_map = build_dict_map(include_apriltag=args.include_apriltag)
    if not dict_map:
        print("No cv2.aruco dictionaries available on this system.")
        return 2

    params = build_params()
    stats = defaultdict(lambda: {"total": 0, "frames": 0, "id_counts": Counter()})

    rclpy.init()
    node = FrameGrabber(args.topic)

    deadline = time.time() + args.timeout_sec
    while time.time() < deadline and node.latest is None:
        rclpy.spin_once(node, timeout_sec=0.05)
    if node.latest is None:
        print(f"No image received on {args.topic} before timeout.")
        node.destroy_node()
        rclpy.shutdown()
        return 3

    warmup_end = time.time() + args.warmup_sec
    while time.time() < warmup_end:
        rclpy.spin_once(node, timeout_sec=0.05)

    captured = 0
    while captured < args.frames and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.08)
        frame = node.latest
        if frame is None:
            continue

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        for name, dict_id in dict_map.items():
            ids = detect_ids(gray, dict_id, params, args.scale)
            if ids:
                stats[name]["total"] += len(ids)
                stats[name]["frames"] += 1
                stats[name]["id_counts"].update(ids)
        captured += 1

    node.destroy_node()
    rclpy.shutdown()

    if not stats:
        print("No markers detected for any dictionary.")
        return 4

    ranked = sorted(
        stats.items(),
        key=lambda kv: (kv[1]["frames"], kv[1]["total"]),
        reverse=True,
    )

    print(f"Frames analyzed: {captured}")
    print("Top dictionaries:")
    for name, s in ranked[:8]:
        common = s["id_counts"].most_common(8)
        ids_text = ", ".join([f"{mid}x{cnt}" for mid, cnt in common]) if common else "-"
        print(f"- {name}: frames_with_ids={s['frames']} total_ids={s['total']} ids=[{ids_text}]")

    best_name, best = ranked[0]
    best_ids = [mid for mid, _ in best["id_counts"].most_common(8)]
    print("\nSuggested config:")
    print(f"aruco_dict_name={best_name}")
    print(f"candidate_ids={best_ids}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
