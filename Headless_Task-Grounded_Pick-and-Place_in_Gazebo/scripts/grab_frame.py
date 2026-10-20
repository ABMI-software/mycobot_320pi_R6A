#!/usr/bin/env python3
"""grab_frame.py -- save one camera frame to PNG, headless."""
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage
import numpy as np, cv2, sys

class Grab(Node):
    def __init__(self, out):
        super().__init__('grab_frame')
        self.out = out
        self.done = False
        self.create_subscription(
            CompressedImage, '/synth_camera/image/compressed', self.cb, 10)

    def cb(self, msg):
        if self.done:
            return
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        # BGR->RGB, per the empirically-confirmed format string finding in the
        # RLDS conversion work: 'rgb8; jpeg compressed bgr8' -- cv2.imdecode's
        # output is genuinely BGR, so it must be swapped before saving with a
        # tool (PIL) that writes channels as-is, or left as BGR for cv2.imwrite
        # (which itself expects and correctly reinterprets BGR-on-write). Using
        # cv2.imwrite here, so NO swap: cv2's read/write pair is self-consistent.
        cv2.imwrite(self.out, img)
        self.get_logger().info(f'saved {self.out}  shape={img.shape}')
        self.done = True

def main():
    out = sys.argv[1] if len(sys.argv) > 1 else 'frame.png'
    rclpy.init()
    n = Grab(out)
    while rclpy.ok() and not n.done:
        rclpy.spin_once(n, timeout_sec=1.0)
    n.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
