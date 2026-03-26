#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Marker Detector Node - Runs on Tour (PC)
Performs heavy ArUco marker detection and publishes commands to Pi via bridge.
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge, CvBridgeError
import json


class MarkerDetectorTour(Node):
    """
    Heavy computation node for ArUco marker detection.
    Runs on Tour (PC) and sends simple commands to Pi via /to_robot topic.
    """
    
    def __init__(self):
        super().__init__('marker_detector_tour')
        
        # Publisher to send commands to Pi
        self.cmd_publisher = self.create_publisher(String, '/to_robot', 10)
        
        # Subscriber for robot feedback
        self.feedback_sub = self.create_subscription(
            String, '/from_robot', self.feedback_callback, 10)
        
        # Camera subscriber (local camera on Tour)
        self.bridge = CvBridge()
        self.image_sub = self.create_subscription(
            Image, 'camera/image_raw', self.image_callback, 10)
        
        # ArUco setup
        try:
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        except AttributeError:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
            self.aruco_params = cv2.aruco.DetectorParameters()

        # Camera matrix (will be computed from first frame)
        self.camera_matrix = None
        self.dist_coeffs = np.zeros((4, 1))
        
        # State tracking
        self.last_command_time = self.get_clock().now()
        self.command_cooldown = 0.2  # 200ms between commands
        
        self.get_logger().info("🎯 Marker Detector Tour started")
        self.get_logger().info("📤 Publishing commands to /to_robot")
        self.get_logger().info("📥 Listening for feedback on /from_robot")

    def feedback_callback(self, msg):
        """Handle feedback from Pi"""
        self.get_logger().info(f"📥 Pi feedback: {msg.data}")

    def image_callback(self, data):
        """Process camera image for marker detection"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
        except CvBridgeError as e:
            self.get_logger().error(f"CV Bridge error: {e}")
            return

        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
        
        # Initialize camera matrix from first frame
        if self.camera_matrix is None:
            h, w = gray.shape
            focal_length = w
            center = (w / 2, h / 2)
            self.camera_matrix = np.array([
                [focal_length, 0, center[0]],
                [0, focal_length, center[1]],
                [0, 0, 1]
            ], dtype=np.float32)

        # Detect ArUco markers
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params)

        if ids is not None:
            # Estimate pose for each marker
            rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
                corners, 0.05, self.camera_matrix, self.dist_coeffs)
            
            for i, marker_id in enumerate(ids):
                tvec = tvecs[i][0]
                
                # Convert to mm and compute target coordinates
                x_mm = tvec[0] * 1000
                y_mm = tvec[1] * 1000
                z_mm = tvec[2] * 1000
                
                # Apply transformations for robot coordinate system
                # Heavy computation done here on Tour
                target_coords = self.compute_robot_coords(x_mm, y_mm, z_mm)
                
                if target_coords:
                    self.send_move_command(target_coords)
                
                # Draw for visualization
                cv2.drawFrameAxes(cv_image, self.camera_matrix, 
                                  self.dist_coeffs, rvecs[i], tvecs[i], 0.03)

        # Display detection (on Tour)
        cv2.imshow("Tour - Marker Detection", cv_image)
        cv2.waitKey(1)

    def compute_robot_coords(self, x, y, z):
        """
        Complex coordinate transformation (runs on Tour).
        Converts camera coordinates to robot workspace coordinates.
        """
        # Check if marker is in valid range
        if not (-800 < x < 800 and -500 < y < 500 and 100 < z < 1000):
            return None
        
        # Transform camera frame to robot frame
        # This is where heavy computation happens
        target_x = abs(x) if x < 0 else x
        target_y = -y  # Mirror correction
        target_z = z
        
        # Apply safety limits
        target_x = max(130.0, min(target_x, 350.0))
        target_y = max(-200.0, min(target_y, 200.0))
        target_z = max(100.0, min(target_z, 400.0))
        
        return [target_x, target_y, target_z, 180.0, 0.0, 0.0]

    def send_move_command(self, coords):
        """Send movement command to Pi via bridge"""
        now = self.get_clock().now()
        elapsed = (now - self.last_command_time).nanoseconds / 1e9
        
        if elapsed < self.command_cooldown:
            return
        
        self.last_command_time = now
        
        # Create command message
        cmd = {
            "action": "send_coords",
            "coords": coords,
            "speed": 40,
            "mode": 1
        }
        
        msg = String()
        msg.data = json.dumps(cmd)
        self.cmd_publisher.publish(msg)
        
        self.get_logger().info(f"📤 Sent coords: {coords[:3]}")

    def destroy_node(self):
        cv2.destroyAllWindows()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MarkerDetectorTour()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
