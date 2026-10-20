#!/usr/bin/env python3
"""Part 11.4 -- headless single-frame camera capture, for spot-checking a
scene without a GUI. Uses rclpy + cv_bridge; run inside the container."""
import argparse
import sys

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image


class Grabber(Node):
    def __init__(self, topic):
        super().__init__('grab_frame')
        self.bridge = CvBridge()
        self.frame = None
        self.create_subscription(Image, topic, self._cb, 10)

    def _cb(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--topic', default='/camera/image_raw')
    ap.add_argument('--out', required=True)
    ap.add_argument('--timeout', type=float, default=15.0)
    args = ap.parse_args()

    rclpy.init()
    node = Grabber(args.topic)
    import time
    deadline = time.time() + args.timeout
    while node.frame is None and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    if node.frame is None:
        print(f"FATAL: no frame received on {args.topic} within {args.timeout}s")
        sys.exit(1)
    cv2.imwrite(args.out, node.frame)
    print(f"wrote {args.out} ({node.frame.shape[1]}x{node.frame.shape[0]})")
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == '__main__':
    main()
