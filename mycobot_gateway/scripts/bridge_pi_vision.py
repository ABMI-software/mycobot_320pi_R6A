#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Bridge Pi with Vision - TCP Server + Command Executor + Marker Detection
Runs on Raspberry Pi with camera connected.

This version handles:
  - TCP server for commands from Tour
  - Camera capture and ArUco marker detection
  - Robot control execution
  - Sends marker position data back to Tour

DEPLOYMENT: Copy this file to the Pi and run it there.
"""

import socket
import threading
import json
import time
import cv2
import numpy as np
from pymycobot.mycobot import MyCobot


class BridgePiVision:
    """
    TCP Server with integrated vision and robot control.
    Camera and robot are on the Pi, heavy decisions can come from Tour.
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
        
        # Initialize camera
        print("📷 Initializing camera...")
        self.cap = self.find_camera()
        if self.cap:
            print("✅ Camera initialized!")
        else:
            print("⚠️ No camera found - vision disabled")
        
        # ArUco setup
        try:
            self.aruco_dict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_6X6_250)
            self.aruco_params = cv2.aruco.DetectorParameters_create()
        except AttributeError:
            self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
            self.aruco_params = cv2.aruco.DetectorParameters()
        
        self.camera_matrix = None
        self.dist_coeffs = np.zeros((4, 1))
        
        # Preset positions
        self.home_angles = [0, 8, -127, 40, 0, 0]
        self.zero_angles = [0, 0, 0, 0, 0, 0]
        
        # Socket setup
        self.server_socket = None
        self.client_socket = None
        self.running = False
        
        # Vision mode
        self.vision_enabled = False
        self.follow_mode = False
        self.last_detection_time = 0
        self.detection_cooldown = 0.2  # 200ms

    def find_camera(self):
        """Find a working camera"""
        for index in range(6):
            print(f"  Testing camera index {index}...")
            cap = cv2.VideoCapture(index)
            if cap.isOpened():
                ret, frame = cap.read()
                if ret:
                    print(f"  ✅ Camera found at index {index}")
                    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
                    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
                    return cap
                cap.release()
        return None

    def start(self):
        """Start the TCP server and vision processing"""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)
        self.running = True
        
        # Start vision thread if camera available
        if self.cap:
            vision_thread = threading.Thread(target=self.vision_loop, daemon=True)
            vision_thread.start()
        
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

    def vision_loop(self):
        """Continuous vision processing loop"""
        while self.running:
            if not self.vision_enabled and not self.follow_mode:
                time.sleep(0.1)
                continue
            
            ret, frame = self.cap.read()
            if not ret:
                continue
            
            # Detect markers
            marker_data = self.detect_markers(frame)
            
            # If in follow mode, move robot
            if self.follow_mode and marker_data:
                self.follow_marker(marker_data)
            
            # Send marker data to Tour if connected and vision enabled
            if self.vision_enabled and marker_data and self.client_socket:
                self.send_to_tour(f"MARKER: {json.dumps(marker_data)}")
            
            # Small delay to prevent CPU overload
            time.sleep(0.033)  # ~30fps

    def detect_markers(self, frame):
        """Detect ArUco markers and return position data"""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        if self.camera_matrix is None:
            h, w = gray.shape
            focal_length = w
            center = (w / 2, h / 2)
            self.camera_matrix = np.array([
                [focal_length, 0, center[0]],
                [0, focal_length, center[1]],
                [0, 0, 1]
            ], dtype=np.float32)
        
        corners, ids, _ = cv2.aruco.detectMarkers(
            gray, self.aruco_dict, parameters=self.aruco_params)
        
        if ids is None:
            return None
        
        rvecs, tvecs, _ = cv2.aruco.estimatePoseSingleMarkers(
            corners, 0.05, self.camera_matrix, self.dist_coeffs)
        
        markers = []
        for i, marker_id in enumerate(ids):
            tvec = tvecs[i][0]
            markers.append({
                "id": int(marker_id[0]),
                "x": float(tvec[0] * 1000),  # mm
                "y": float(tvec[1] * 1000),
                "z": float(tvec[2] * 1000)
            })
        
        return markers

    def follow_marker(self, markers):
        """Move robot to follow detected marker"""
        if not self.mc or not markers:
            return
        
        now = time.time()
        if now - self.last_detection_time < self.detection_cooldown:
            return
        self.last_detection_time = now
        
        # Use first detected marker
        m = markers[0]
        x, y, z = m['x'], m['y'], m['z']
        
        # Check if in valid range
        if not (-800 < x < 800 and -500 < y < 500 and 100 < z < 1000):
            return
        
        # Transform camera frame to robot frame
        target_x = abs(x) if x < 0 else x
        target_y = -y
        target_z = z
        
        # Apply safety limits
        target_x = max(130.0, min(target_x, 350.0))
        target_y = max(-200.0, min(target_y, 200.0))
        target_z = max(100.0, min(target_z, 400.0))
        
        coords = [target_x, target_y, target_z, 180.0, 0.0, 0.0]
        
        try:
            self.mc.send_coords(coords, 40, 1)
            print(f"🎯 Following marker: {coords[:3]}")
        except Exception as e:
            print(f"❌ Follow error: {e}")

    def send_to_tour(self, message):
        """Send message back to Tour"""
        if self.client_socket:
            try:
                self.client_socket.sendall((message + '\n').encode('utf-8'))
            except:
                pass

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
                
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    if line:
                        response = self.process_command(line)
                        self.send_to_tour(response)
                        
            except Exception as e:
                print(f"❌ Error receiving data: {e}")
                break
        
        if self.client_socket:
            self.client_socket.close()
            self.client_socket = None
        
        print("⏳ Waiting for new connection...")

    def process_command(self, raw_cmd):
        """Process incoming command and return response"""
        print(f"📥 Received: {raw_cmd}")
        
        try:
            cmd = json.loads(raw_cmd)
            return self.execute_json_command(cmd)
        except json.JSONDecodeError:
            return self.execute_raw_command(raw_cmd)

    def execute_json_command(self, cmd):
        """Execute a JSON-formatted command"""
        action = cmd.get('action', '').lower()
        
        # Vision control commands
        if action == 'vision_on':
            self.vision_enabled = True
            return "OK: vision streaming enabled"
        
        elif action == 'vision_off':
            self.vision_enabled = False
            return "OK: vision streaming disabled"
        
        elif action == 'follow_on':
            self.follow_mode = True
            return "OK: follow mode enabled"
        
        elif action == 'follow_off':
            self.follow_mode = False
            return "OK: follow mode disabled"
        
        elif action == 'get_marker':
            if self.cap:
                ret, frame = self.cap.read()
                if ret:
                    markers = self.detect_markers(frame)
                    if markers:
                        return f"MARKERS: {json.dumps(markers)}"
                    return "MARKERS: none"
            return "ERROR: no camera"
        
        # Robot control commands
        if not self.mc:
            return "ERROR: Robot not connected"
        
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
                
            elif action == 'emergency_stop':
                self.follow_mode = False
                self.mc.release_all_servos()
                return "🚨 EMERGENCY STOP EXECUTED"
                
            else:
                return f"ERROR: Unknown action '{action}'"
                
        except Exception as e:
            return f"ERROR: {str(e)}"

    def execute_raw_command(self, cmd):
        """Execute a raw string command"""
        cmd_lower = cmd.lower().strip()
        
        # Vision commands
        if cmd_lower == 'follow':
            self.follow_mode = True
            return "OK: follow mode ON"
        elif cmd_lower == 'nofollow':
            self.follow_mode = False
            return "OK: follow mode OFF"
        elif cmd_lower == 'vision':
            self.vision_enabled = True
            return "OK: vision ON"
        elif cmd_lower == 'novision':
            self.vision_enabled = False
            return "OK: vision OFF"
        
        # Robot commands
        if not self.mc:
            return "ERROR: Robot not connected"
        
        try:
            if cmd_lower == 'home':
                self.mc.send_angles(self.home_angles, 30)
                return "OK: going home"
            elif cmd_lower == 'zero':
                self.mc.send_angles(self.zero_angles, 30)
                return "OK: going to zero"
            elif cmd_lower == 'stop':
                self.follow_mode = False
                self.mc.release_all_servos()
                return "OK: stopped"
            elif cmd_lower == 'ping':
                return "PONG"
            elif cmd_lower == 'status':
                angles = self.mc.get_angles()
                coords = self.mc.get_coords()
                return f"STATUS: angles={angles}, coords={coords}, follow={self.follow_mode}"
            else:
                return f"ECHO: {cmd}"
        except Exception as e:
            return f"ERROR: {str(e)}"

    def stop(self):
        """Stop the server"""
        self.running = False
        self.follow_mode = False
        if self.cap:
            self.cap.release()
        if self.client_socket:
            self.client_socket.close()
        if self.server_socket:
            self.server_socket.close()
        print("🛑 Server stopped")


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='MyCobot Bridge Pi with Vision')
    parser.add_argument('--host', default='0.0.0.0', help='Listen address')
    parser.add_argument('--port', type=int, default=5005, help='Listen port')
    parser.add_argument('--robot-port', default='/dev/ttyAMA0', help='Robot serial port')
    parser.add_argument('--baud', type=int, default=115200, help='Robot baud rate')
    
    args = parser.parse_args()
    
    print("="*60)
    print("🤖 MyCobot Bridge Pi with Vision")
    print("="*60)
    print(f"📡 Network: {args.host}:{args.port}")
    print(f"🔌 Robot: {args.robot_port} @ {args.baud}")
    print("="*60)
    print("\nNew commands available:")
    print("  follow   - Start marker following mode")
    print("  nofollow - Stop marker following mode")
    print("  vision   - Start sending marker data to Tour")
    print("  novision - Stop sending marker data")
    print("="*60)
    
    bridge = BridgePiVision(
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
