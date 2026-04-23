#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Color-based object detector for the pick-and-place sorting demo.

Subscribes to a top-down camera (default: /synth_camera_top/image) and
finds the centroid of each color blob (red, blue, green, yellow) using
HSV segmentation. Centroids are back-projected to world coordinates
assuming a pinhole camera looking straight down at the table plane (z=0).

Published as a single std_msgs/String on /sorting/detections, formatted:
    "color,x,y;color,x,y;..."   (positions in metres, robot base frame)

A latched "ready" flag is also published on /sorting/detector_status.
"""

import math
from typing import Dict, List, Tuple

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge


# HSV ranges tuned for the pure-saturated SDF materials.
# OpenCV HSV: H in [0,180], S/V in [0,255]. Red wraps around 0/180.
HSV_RANGES: Dict[str, List[Tuple[Tuple[int, int, int], Tuple[int, int, int]]]] = {
    'red':    [((0,   120, 80),  (10,  255, 255)),
               ((170, 120, 80),  (180, 255, 255))],
    'blue':   [((100, 120, 80),  (130, 255, 255))],
    'green':  [((40,  100, 60),  (80,  255, 255))],
    'yellow': [((20,  120, 100), (35,  255, 255))],
}

# Minimum blob area in pixels — filters bin walls and reflections.
MIN_BLOB_AREA = 80


class ColorObjectDetector(Node):

    def __init__(self):
        super().__init__('color_object_detector')

        # Camera + back-projection parameters (overridable by launch args).
        self.declare_parameter('camera_topic', '/synth_camera_top/image')
        self.declare_parameter('image_width', 640)
        self.declare_parameter('image_height', 480)
        self.declare_parameter('camera_height', 1.2)        # m above table
        self.declare_parameter('camera_hfov', 1.047)        # rad (60°)
        self.declare_parameter('camera_world_x', 0.0)       # camera XY in world
        self.declare_parameter('camera_world_y', 0.0)
        # Image-axis → world-axis mapping (default for our top camera with
        # rpy=0,1.5708,0: image v↑ corresponds to world +X, image u→ to world +Y).
        # Flip if axes come out reversed empirically.
        # Use 'world_x' / 'world_y' (not bare 'x'/'y') because YAML 1.1
        # parses 'y' as the boolean True, which rclpy then rejects.
        self.declare_parameter('image_u_to_world_axis_name', 'world_y')
        self.declare_parameter('image_v_to_world_axis_name', 'world_x')
        self.declare_parameter('flip_u', False)
        self.declare_parameter('flip_v', True)
        self.declare_parameter('publish_rate', 2.0)
        self.declare_parameter('debug_publish', True)
        self.declare_parameter('pick_x_min', 0.10)         # only keep detections
        self.declare_parameter('pick_x_max', 0.40)         # inside the pick zone
        self.declare_parameter('pick_y_min', -0.25)
        self.declare_parameter('pick_y_max', 0.25)

        self.cam_topic = self.get_parameter('camera_topic').value
        self.W = int(self.get_parameter('image_width').value)
        self.H = int(self.get_parameter('image_height').value)
        self.h_cam = float(self.get_parameter('camera_height').value)
        self.hfov = float(self.get_parameter('camera_hfov').value)
        self.cam_x = float(self.get_parameter('camera_world_x').value)
        self.cam_y = float(self.get_parameter('camera_world_y').value)
        self.u_axis = str(self.get_parameter('image_u_to_world_axis_name').value).replace('world_', '')
        self.v_axis = str(self.get_parameter('image_v_to_world_axis_name').value).replace('world_', '')
        self.flip_u = bool(self.get_parameter('flip_u').value)
        self.flip_v = bool(self.get_parameter('flip_v').value)
        self.debug_publish = bool(self.get_parameter('debug_publish').value)

        # Pinhole back-projection scale (m per pixel) on the table plane.
        half_w_world = self.h_cam * math.tan(self.hfov / 2.0)
        self.m_per_px_u = (2.0 * half_w_world) / self.W
        # Square pixels: assume vertical FOV from aspect ratio.
        self.m_per_px_v = self.m_per_px_u  # cv camera with square pixels

        self.x_min = float(self.get_parameter('pick_x_min').value)
        self.x_max = float(self.get_parameter('pick_x_max').value)
        self.y_min = float(self.get_parameter('pick_y_min').value)
        self.y_max = float(self.get_parameter('pick_y_max').value)

        self.bridge = CvBridge()
        self.last_image = None
        self.image_count = 0

        self.create_subscription(Image, self.cam_topic, self._on_image, 10)

        self.pub_det = self.create_publisher(String, '/sorting/detections', 10)
        self.pub_status = self.create_publisher(String, '/sorting/detector_status', 10)
        if self.debug_publish:
            self.pub_debug = self.create_publisher(Image, '/sorting/debug_image', 5)

        rate = float(self.get_parameter('publish_rate').value)
        self.create_timer(1.0 / max(rate, 0.1), self._tick)

        self.get_logger().info(
            f'Color detector — topic {self.cam_topic}, '
            f'{self.W}x{self.H}, m/px={self.m_per_px_u:.4f}, '
            f'cam@({self.cam_x:.2f},{self.cam_y:.2f},{self.h_cam:.2f}), '
            f"u→{self.u_axis}{'(flip)' if self.flip_u else ''}, "
            f"v→{self.v_axis}{'(flip)' if self.flip_v else ''}"
        )

    def _on_image(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'cv_bridge error: {e}')
            return
        self.last_image = img
        self.image_count += 1

    def _pixel_to_world(self, u: float, v: float) -> Tuple[float, float]:
        # Image origin at top-left; centre the coords first.
        du = (u - self.W / 2.0) * (-1.0 if self.flip_u else 1.0)
        dv = (v - self.H / 2.0) * (-1.0 if self.flip_v else 1.0)
        comp_u = du * self.m_per_px_u
        comp_v = dv * self.m_per_px_v
        wx = self.cam_x
        wy = self.cam_y
        if self.u_axis == 'x':
            wx += comp_u
        else:
            wy += comp_u
        if self.v_axis == 'x':
            wx += comp_v
        else:
            wy += comp_v
        return wx, wy

    def _detect_color(self, hsv: np.ndarray, color: str
                      ) -> List[Tuple[float, float, float, int]]:
        """Return list of (cx_pix, cy_pix, area, num_contours) for one color."""
        mask = None
        for lo, hi in HSV_RANGES[color]:
            m = cv2.inRange(hsv, np.array(lo, dtype=np.uint8),
                            np.array(hi, dtype=np.uint8))
            mask = m if mask is None else cv2.bitwise_or(mask, m)
        # Open + close to drop noise and merge speckles.
        k = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, k, iterations=1)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, k, iterations=2)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL,
                                       cv2.CHAIN_APPROX_SIMPLE)
        out = []
        for c in contours:
            area = float(cv2.contourArea(c))
            if area < MIN_BLOB_AREA:
                continue
            M = cv2.moments(c)
            if M['m00'] == 0:
                continue
            cx = M['m10'] / M['m00']
            cy = M['m01'] / M['m00']
            out.append((cx, cy, area, len(contours)))
        return out

    def _tick(self):
        if self.last_image is None:
            self.pub_status.publish(String(data='WAITING_IMAGE'))
            return
        img = self.last_image.copy()
        hsv = cv2.cvtColor(img, cv2.COLOR_BGR2HSV)

        # Per color, keep the largest blob whose back-projected world
        # position lies in the pick zone. Cache pixel centroid for overlay.
        best: Dict[str, Tuple[float, float, float, float, float]] = {}
        # color -> (wx, wy, area, pixel_cx, pixel_cy)
        for color in HSV_RANGES.keys():
            for cx, cy, area, _ in self._detect_color(hsv, color):
                wx, wy = self._pixel_to_world(cx, cy)
                if not (self.x_min <= wx <= self.x_max
                        and self.y_min <= wy <= self.y_max):
                    continue
                if color not in best or area > best[color][2]:
                    best[color] = (wx, wy, area, cx, cy)

        if best:
            payload = ';'.join(
                f'{c},{v[0]:.4f},{v[1]:.4f}' for c, v in best.items()
            )
            self.pub_det.publish(String(data=payload))
            self.pub_status.publish(String(data=f'OK|n={len(best)}'))
        else:
            self.pub_status.publish(String(data='NO_DETECTIONS'))

        if self.debug_publish:
            for color, (wx, wy, _area, cx, cy) in best.items():
                cv2.circle(img, (int(cx), int(cy)), 6, (255, 255, 255), 2)
                cv2.putText(img, f'{color} ({wx:+.2f},{wy:+.2f})',
                            (int(cx) + 8, int(cy) - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (255, 255, 255), 1, cv2.LINE_AA)
            self.pub_debug.publish(self.bridge.cv2_to_imgmsg(img, encoding='bgr8'))


def main(args=None):
    rclpy.init(args=args)
    node = ColorObjectDetector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
