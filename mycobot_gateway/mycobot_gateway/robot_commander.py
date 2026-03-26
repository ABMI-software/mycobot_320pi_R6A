#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Command Sender - Runs on Tour (PC)
Provides high-level robot control interface that sends commands via bridge.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time


class RobotCommandSender(Node):
    """
    High-level command interface for the robot.
    Runs on Tour and sends commands to Pi via /to_robot topic.
    """
    
    def __init__(self):
        super().__init__('robot_command_sender')
        
        # Publisher for commands to Pi
        self.cmd_publisher = self.create_publisher(String, '/to_robot', 10)
        
        # Subscriber for feedback from Pi
        self.feedback_sub = self.create_subscription(
            String, '/from_robot', self.feedback_callback, 10)
        
        # State
        self.last_feedback = None
        self.waiting_for_response = False
        
        self.get_logger().info("🤖 Robot Command Sender initialized")
        self.get_logger().info("📤 Commands sent via /to_robot")
        self.get_logger().info("📥 Feedback received on /from_robot")

    def feedback_callback(self, msg):
        """Handle feedback from robot"""
        self.last_feedback = msg.data
        self.waiting_for_response = False
        self.get_logger().info(f"📥 Robot feedback: {msg.data}")

    def send_raw_command(self, command: str):
        """Send a raw string command"""
        msg = String()
        msg.data = command
        self.cmd_publisher.publish(msg)
        self.get_logger().info(f"📤 Sent: {command}")

    def send_json_command(self, action: str, **kwargs):
        """Send a JSON-formatted command"""
        cmd = {"action": action, **kwargs}
        msg = String()
        msg.data = json.dumps(cmd)
        self.cmd_publisher.publish(msg)
        self.get_logger().info(f"📤 Sent action: {action}")

    # ==================== HIGH-LEVEL COMMANDS ====================

    def send_angles(self, angles: list, speed: int = 30):
        """
        Send joint angles to robot.
        Args:
            angles: List of 6 joint angles in degrees
            speed: Movement speed (1-100)
        """
        self.send_json_command("send_angles", angles=angles, speed=speed)

    def send_coords(self, coords: list, speed: int = 40, mode: int = 1):
        """
        Send Cartesian coordinates to robot.
        Args:
            coords: [x, y, z, rx, ry, rz] in mm and degrees
            speed: Movement speed (1-100)
            mode: 0=angular, 1=linear interpolation
        """
        self.send_json_command("send_coords", coords=coords, speed=speed, mode=mode)

    def send_radians(self, radians: list, speed: int = 30):
        """
        Send joint angles in radians.
        Args:
            radians: List of 6 joint angles in radians
            speed: Movement speed (1-100)
        """
        self.send_json_command("send_radians", radians=radians, speed=speed)

    def go_home(self):
        """Move robot to home position"""
        self.send_json_command("go_home")

    def go_zero(self):
        """Move robot to zero/init position"""
        self.send_json_command("go_zero")

    def gripper_open(self):
        """Open the gripper"""
        self.send_json_command("gripper_open")

    def gripper_close(self):
        """Close the gripper"""
        self.send_json_command("gripper_close")

    def power_on(self):
        """Power on servos"""
        self.send_json_command("power_on")

    def power_off(self):
        """Power off servos (release all)"""
        self.send_json_command("power_off")

    def get_angles(self):
        """Request current joint angles"""
        self.waiting_for_response = True
        self.send_json_command("get_angles")

    def get_coords(self):
        """Request current Cartesian coordinates"""
        self.waiting_for_response = True
        self.send_json_command("get_coords")

    def emergency_stop(self):
        """Emergency stop - release all servos immediately"""
        self.send_json_command("emergency_stop")
        self.get_logger().warn("🚨 EMERGENCY STOP SENT!")


def main(args=None):
    """
    Demo: Interactive command sender
    """
    rclpy.init(args=args)
    node = RobotCommandSender()
    
    # Give time for connections
    time.sleep(1)
    
    print("\n" + "="*50)
    print("🤖 Robot Command Sender - Interactive Mode")
    print("="*50)
    print("\nRobot Commands:")
    print("  home     - Go to home position")
    print("  zero     - Go to zero position")
    print("  open     - Open gripper")
    print("  close    - Close gripper")
    print("  angles   - Get current angles")
    print("  coords   - Get current coords")
    print("  stop     - Emergency stop")
    print("\nVision Commands (Pi with camera):")
    print("  follow   - Start marker following mode")
    print("  nofollow - Stop marker following")
    print("  marker   - Get current marker position")
    print("  status   - Get full robot status")
    print("\nOther:")
    print("  quit/q   - Exit")
    print("  <custom> - Send raw command")
    print("="*50 + "\n")
    
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            
            try:
                cmd = input("Command> ").strip().lower()
            except EOFError:
                break
            
            if cmd in ['quit', 'q', 'exit']:
                break
            elif cmd == 'home':
                node.go_home()
            elif cmd == 'zero':
                node.go_zero()
            elif cmd == 'open':
                node.gripper_open()
            elif cmd == 'close':
                node.gripper_close()
            elif cmd == 'angles':
                node.get_angles()
            elif cmd == 'coords':
                node.get_coords()
            elif cmd == 'stop':
                node.emergency_stop()
            elif cmd == 'follow':
                node.send_json_command("follow_on")
            elif cmd == 'nofollow':
                node.send_json_command("follow_off")
            elif cmd == 'marker':
                node.send_json_command("get_marker")
            elif cmd == 'status':
                node.send_raw_command("status")
            elif cmd:
                node.send_raw_command(cmd)
            
            # Wait a bit for response
            time.sleep(0.5)
            rclpy.spin_once(node, timeout_sec=0.1)
            
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
