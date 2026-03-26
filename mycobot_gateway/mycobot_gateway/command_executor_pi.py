#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Command Executor - Runs on Raspberry Pi
Receives JSON commands from Tour and executes them on MyCobot.
This is the SIMPLE task executor that the Pi handles.

This file should be copied to the Pi's mycobot_320pi package.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from pymycobot.mycobot import MyCobot
import json
import time


class CommandExecutor(Node):
    """
    Simple command executor running on Raspberry Pi.
    Receives commands from bridge and executes them on MyCobot.
    """
    
    def __init__(self):
        super().__init__('command_executor')
        
        # Parameters
        self.declare_parameter('port', '/dev/ttyAMA0')
        self.declare_parameter('baud', 115200)
        
        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value
        
        # Initialize robot
        self.get_logger().info(f"🤖 Connecting to MyCobot on {port}...")
        try:
            self.mc = MyCobot(port, baud)
            time.sleep(1)
            self.mc.power_on()
            self.get_logger().info("✅ MyCobot connected and powered on!")
        except Exception as e:
            self.get_logger().error(f"❌ Failed to connect: {e}")
            self.mc = None
        
        # Subscriber for commands from bridge
        self.cmd_sub = self.create_subscription(
            String, '/robot_commands', self.command_callback, 10)
        
        # Publisher for feedback
        self.feedback_pub = self.create_publisher(String, '/robot_feedback', 10)
        
        # Home and zero positions
        self.home_angles = [0, 8, -127, 40, 0, 0]
        self.zero_angles = [0, 0, 0, 0, 0, 0]
        
        self.get_logger().info("📥 Listening for commands on /robot_commands")
        self.get_logger().info("📤 Publishing feedback to /robot_feedback")

    def send_feedback(self, message: str):
        """Send feedback to Tour"""
        msg = String()
        msg.data = message
        self.feedback_pub.publish(msg)
        self.get_logger().info(f"📤 Feedback: {message}")

    def command_callback(self, msg):
        """Process incoming command"""
        if not self.mc:
            self.send_feedback("ERROR: Robot not connected")
            return
        
        raw_data = msg.data.strip()
        self.get_logger().info(f"📥 Received: {raw_data}")
        
        try:
            # Try to parse as JSON
            cmd = json.loads(raw_data)
            self.execute_json_command(cmd)
        except json.JSONDecodeError:
            # Treat as raw command string
            self.execute_raw_command(raw_data)

    def execute_json_command(self, cmd: dict):
        """Execute a JSON-formatted command"""
        action = cmd.get('action', '').lower()
        
        try:
            if action == 'send_angles':
                angles = cmd.get('angles', [])
                speed = cmd.get('speed', 30)
                self.mc.send_angles(angles, speed)
                self.send_feedback(f"OK: send_angles {angles}")
                
            elif action == 'send_coords':
                coords = cmd.get('coords', [])
                speed = cmd.get('speed', 40)
                mode = cmd.get('mode', 1)
                self.mc.send_coords(coords, speed, mode)
                self.send_feedback(f"OK: send_coords {coords[:3]}")
                
            elif action == 'send_radians':
                radians = cmd.get('radians', [])
                speed = cmd.get('speed', 30)
                self.mc.send_radians(radians, speed)
                self.send_feedback(f"OK: send_radians")
                
            elif action == 'go_home':
                self.mc.send_angles(self.home_angles, 30)
                self.send_feedback("OK: going home")
                
            elif action == 'go_zero':
                self.mc.send_angles(self.zero_angles, 30)
                self.send_feedback("OK: going to zero")
                
            elif action == 'gripper_open':
                self.mc.set_gripper_state(0, 50)
                self.send_feedback("OK: gripper opened")
                
            elif action == 'gripper_close':
                self.mc.set_gripper_state(1, 50)
                self.send_feedback("OK: gripper closed")
                
            elif action == 'power_on':
                self.mc.power_on()
                self.send_feedback("OK: powered on")
                
            elif action == 'power_off':
                self.mc.release_all_servos()
                self.send_feedback("OK: servos released")
                
            elif action == 'get_angles':
                angles = self.mc.get_angles()
                self.send_feedback(f"ANGLES: {angles}")
                
            elif action == 'get_coords':
                coords = self.mc.get_coords()
                self.send_feedback(f"COORDS: {coords}")
                
            elif action == 'emergency_stop':
                self.mc.release_all_servos()
                self.send_feedback("🚨 EMERGENCY STOP EXECUTED")
                
            else:
                self.send_feedback(f"ERROR: Unknown action '{action}'")
                
        except Exception as e:
            self.send_feedback(f"ERROR: {str(e)}")

    def execute_raw_command(self, cmd: str):
        """Execute a raw string command (legacy support)"""
        cmd_lower = cmd.lower()
        
        try:
            if cmd_lower == 'home':
                self.mc.send_angles(self.home_angles, 30)
                self.send_feedback("OK: going home")
                
            elif cmd_lower == 'zero':
                self.mc.send_angles(self.zero_angles, 30)
                self.send_feedback("OK: going to zero")
                
            elif cmd_lower == 'stop':
                self.mc.release_all_servos()
                self.send_feedback("OK: stopped")
                
            elif cmd_lower.startswith('angle'):
                # Format: angle <joint> <value>
                parts = cmd.split()
                if len(parts) >= 3:
                    joint = int(parts[1])
                    value = float(parts[2])
                    self.mc.send_angle(joint, value, 30)
                    self.send_feedback(f"OK: joint {joint} -> {value}")
                    
            else:
                self.send_feedback(f"ECHO: {cmd}")
                
        except Exception as e:
            self.send_feedback(f"ERROR: {str(e)}")


def main(args=None):
    rclpy.init(args=args)
    node = CommandExecutor()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
