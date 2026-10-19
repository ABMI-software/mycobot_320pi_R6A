#!/usr/bin/env python3
"""Capture une image ROS 2 sans ouvrir directement la camera V4L2."""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import rclpy
from rclpy.executors import SingleThreadedExecutor
from sensor_msgs.msg import Image

sys.path.insert(0, str(Path(__file__).resolve().parent))
from adaptive_pick_by_demo import decode_ros_image_bgr8  # noqa: E402


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--topic", default="/camera/image_raw")
    parser.add_argument("--output", type=Path, default=Path("/tmp/arducam_ros.jpg"))
    parser.add_argument("--timeout", type=float, default=5.0)
    args = parser.parse_args()

    if args.timeout <= 0:
        raise SystemExit("--timeout doit etre > 0")
    rclpy.init(args=None)
    node = rclpy.create_node("capture_ros_image_once")
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    frames = []

    def callback(message: Image) -> None:
        if not frames:
            frames.append(decode_ros_image_bgr8(message))

    node.create_subscription(Image, args.topic, callback, 10)
    try:
        deadline = time.monotonic() + args.timeout
        while not frames and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.1)
        if not frames:
            raise SystemExit(f"aucune image recue sur {args.topic}")
        args.output.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(args.output), frames[0]):
            raise SystemExit(f"echec ecriture {args.output}")
        print(args.output)
        return 0
    finally:
        executor.remove_node(node)
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
