#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Camera Publisher Node - Runs on Tour (PC)
Captures video from USB camera and publishes to ROS2 topic.
"""

import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraPublisher(Node):
    """
    Camera node running on Tour (PC).
    Publishes camera images for processing by marker_detector.
    """
    
    def __init__(self):
        super().__init__('camera_publisher_tour')
        
        # Declare parameters
        self.declare_parameter('camera_index', 0)
        self.declare_parameter('frame_width', 640)
        self.declare_parameter('frame_height', 480)
        self.declare_parameter('fps', 30.0)
        
        # Get parameters
        camera_index = self.get_parameter('camera_index').value
        frame_width = self.get_parameter('frame_width').value
        frame_height = self.get_parameter('frame_height').value
        fps = self.get_parameter('fps').value
        
        # Publisher
        self.publisher = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()
        
        # Camera setup
        self.cap = self.find_camera(camera_index)
        
        if self.cap and self.cap.isOpened():
            self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
            self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, frame_width)
            self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, frame_height)
            
            # Timer for capture
            timer_period = 1.0 / fps
            self.timer = self.create_timer(timer_period, self.timer_callback)
            
            self.get_logger().info(f"📷 Camera initialized at index {camera_index}")
            self.get_logger().info(f"📐 Resolution: {frame_width}x{frame_height} @ {fps}fps")
        else:
            self.get_logger().error("❌ CRITICAL: No camera found!")

    def find_camera(self, preferred_index):
        """Find a working camera"""
        # Try preferred index first
        cap = cv2.VideoCapture(preferred_index)
        if cap.isOpened():
            ret, frame = cap.read()
            if ret:
                self.get_logger().info(f"✅ Camera found at index {preferred_index}")
                return cap
            cap.release()
        
        # Search other indices
        for index in range(6):
            if index == preferred_index:
                continue
            self.get_logger().info(f"🔍 Testing camera index {index}...")
            cap = cv2.VideoCapture(index)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    self.get_logger().info(f"✅ Camera found at index {index}")
                    return cap
                cap.release()
        
        return None

    def timer_callback(self):
        """Capture and publish frame"""
        ret, frame = self.cap.read()
        if ret:
            msg = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "camera_link"
            self.publisher.publish(msg)
        else:
            self.get_logger().warn("⚠️ Dropped frame")

    def destroy_node(self):
        if self.cap:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
