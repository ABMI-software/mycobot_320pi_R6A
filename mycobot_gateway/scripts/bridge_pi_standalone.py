#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Bridge Pi - TCP Server + Command Executor
Runs on Raspberry Pi to receive commands from Tour and execute on MyCobot.

DEPLOYMENT: Copy this file to the Pi and run it there.
Location: ~/ros_ws/src/mycobot_gateway_pi/bridge_pi.py
"""

import socket
import threading
import json
import time
from pymycobot.mycobot import MyCobot


class BridgePi:
    """
    TCP Server that receives commands from Tour and executes them on MyCobot.
    Combines bridge and command executor functionality.
    """
    
    def __init__(self, host='0.0.0.0', port=5005, robot_port='/dev/ttyAMA0', baud=115200):
        self.host = host
        self.port = port
        
        # Initialize robot
        print(f"🤖 Connecting to MyCobot on {robot_port}...")
        try:
            self.mc = MyCobot(robot_port, baud)
            time.sleep(1)
            self.mc.power_on()
            print("✅ MyCobot connected and powered on!")
        except Exception as e:
            print(f"❌ Failed to connect to robot: {e}")
            self.mc = None
        
        # Preset positions
        self.home_angles = [0, 8, -127, 40, 0, 0]
        self.zero_angles = [0, 0, 0, 0, 0, 0]
        
        # Socket setup
        self.server_socket = None
        self.client_socket = None
        self.running = False

    def start(self):
        """Start the TCP server"""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)
        self.running = True
        
        print(f"🌐 Server listening on {self.host}:{self.port}")
        print("⏳ Waiting for Tour connection...")
        
        while self.running:
            try:
                self.client_socket, addr = self.server_socket.accept()
                print(f"✅ Tour connected from {addr}")
                self.handle_client()
            except Exception as e:
                if self.running:
                    print(f"❌ Connection error: {e}")

    def handle_client(self):
        """Handle incoming commands from Tour"""
        buffer = ""
        
        while self.running:
            try:
                data = self.client_socket.recv(1024)
                if not data:
                    print("⚠️ Tour disconnected")
                    break
                
                buffer += data.decode('utf-8')
                
                # Process complete messages (newline-terminated)
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    if line:
                        response = self.process_command(line)
                        self.send_response(response)
                        
            except Exception as e:
                print(f"❌ Error receiving data: {e}")
                break
        
        if self.client_socket:
            self.client_socket.close()
            self.client_socket = None
        
        print("⏳ Waiting for new connection...")

    def send_response(self, message):
        """Send response back to Tour"""
        if self.client_socket:
            try:
                self.client_socket.sendall((message + '\n').encode('utf-8'))
                print(f"📤 Response: {message}")
            except Exception as e:
                print(f"❌ Failed to send response: {e}")

    def process_command(self, raw_cmd):
        """Process incoming command and return response"""
        print(f"📥 Received: {raw_cmd}")
        
        if not self.mc:
            return "ERROR: Robot not connected"
        
        try:
            # Try JSON format first
            cmd = json.loads(raw_cmd)
            return self.execute_json_command(cmd)
        except json.JSONDecodeError:
            # Fall back to raw command
            return self.execute_raw_command(raw_cmd)

    def execute_json_command(self, cmd):
        """Execute a JSON-formatted command"""
        action = cmd.get('action', '').lower()
        
        try:
            if action == 'send_angles':
                angles = cmd.get('angles', [])
                speed = cmd.get('speed', 30)
                self.mc.send_angles(angles, speed)
                return f"OK: send_angles {angles}"
                
            elif action == 'send_coords':
                coords = cmd.get('coords', [])
                speed = cmd.get('speed', 40)
                mode = cmd.get('mode', 1)
                self.mc.send_coords(coords, speed, mode)
                return f"OK: send_coords {coords[:3]}"
                
            elif action == 'send_radians':
                radians = cmd.get('radians', [])
                speed = cmd.get('speed', 30)
                self.mc.send_radians(radians, speed)
                return "OK: send_radians"
                
            elif action == 'go_home':
                self.mc.send_angles(self.home_angles, 30)
                return "OK: going home"
                
            elif action == 'go_zero':
                self.mc.send_angles(self.zero_angles, 30)
                return "OK: going to zero"
                
            elif action == 'gripper_open':
                self.mc.set_gripper_state(0, 50)
                return "OK: gripper opened"
                
            elif action == 'gripper_close':
                self.mc.set_gripper_state(1, 50)
                return "OK: gripper closed"
                
            elif action == 'power_on':
                self.mc.power_on()
                return "OK: powered on"
                
            elif action == 'power_off':
                self.mc.release_all_servos()
                return "OK: servos released"
                
            elif action == 'get_angles':
                angles = self.mc.get_angles()
                return f"ANGLES: {angles}"
                
            elif action == 'get_coords':
                coords = self.mc.get_coords()
                return f"COORDS: {coords}"
                
            elif action == 'get_radians':
                radians = self.mc.get_radians()
                return f"RADIANS: {radians}"
                
            elif action == 'emergency_stop':
                self.mc.release_all_servos()
                return "🚨 EMERGENCY STOP EXECUTED"
                
            elif action == 'set_color':
                r = cmd.get('r', 0)
                g = cmd.get('g', 0)
                b = cmd.get('b', 0)
                self.mc.set_color(r, g, b)
                return f"OK: LED color set to ({r},{g},{b})"
                
            else:
                return f"ERROR: Unknown action '{action}'"
                
        except Exception as e:
            return f"ERROR: {str(e)}"

    def execute_raw_command(self, cmd):
        """Execute a raw string command (legacy/simple commands)"""
        cmd_lower = cmd.lower().strip()
        
        try:
            if cmd_lower == 'home':
                self.mc.send_angles(self.home_angles, 30)
                return "OK: going home"
                
            elif cmd_lower == 'zero':
                self.mc.send_angles(self.zero_angles, 30)
                return "OK: going to zero"
                
            elif cmd_lower == 'stop':
                self.mc.release_all_servos()
                return "OK: stopped"
                
            elif cmd_lower == 'ping':
                return "PONG"
                
            elif cmd_lower == 'status':
                angles = self.mc.get_angles()
                coords = self.mc.get_coords()
                return f"STATUS: angles={angles}, coords={coords}"
                
            elif cmd_lower.startswith('led '):
                # Format: led r g b
                parts = cmd.split()
                if len(parts) >= 4:
                    r, g, b = int(parts[1]), int(parts[2]), int(parts[3])
                    self.mc.set_color(r, g, b)
                    return f"OK: LED set to ({r},{g},{b})"
                return "ERROR: Usage: led <r> <g> <b>"
                
            else:
                return f"ECHO: {cmd}"
                
        except Exception as e:
            return f"ERROR: {str(e)}"

    def stop(self):
        """Stop the server"""
        self.running = False
        if self.client_socket:
            self.client_socket.close()
        if self.server_socket:
            self.server_socket.close()
        print("🛑 Server stopped")


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='MyCobot Bridge Pi - TCP Server')
    parser.add_argument('--host', default='0.0.0.0', help='Listen address')
    parser.add_argument('--port', type=int, default=5005, help='Listen port')
    parser.add_argument('--robot-port', default='/dev/ttyAMA0', help='Robot serial port')
    parser.add_argument('--baud', type=int, default=115200, help='Robot baud rate')
    
    args = parser.parse_args()
    
    print("="*50)
    print("🤖 MyCobot Bridge Pi - Command Executor")
    print("="*50)
    print(f"📡 Network: {args.host}:{args.port}")
    print(f"🔌 Robot: {args.robot_port} @ {args.baud}")
    print("="*50)
    
    bridge = BridgePi(
        host=args.host,
        port=args.port,
        robot_port=args.robot_port,
        baud=args.baud
    )
    
    try:
        bridge.start()
    except KeyboardInterrupt:
        print("\n⏹️ Interrupted by user")
    finally:
        bridge.stop()


if __name__ == '__main__':
    main()
