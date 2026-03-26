#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Teleop Keyboard - Version Tour (PC)
Contrôle du robot MyCobot au clavier via le bridge.

Cette version tourne sur le PC Tour et envoie les commandes
de déplacement incrémental au robot via le bridge TCP.

Usage:
    ros2 run mycobot_gateway teleop_keyboard

Commandes:
    w/s - X+/X-
    a/d - Y-/Y+
    z/x - Z-/Z+
    u/j - RX+/RX-
    i/k - RY+/RY-
    o/l - RZ+/RZ-
    g/h - Gripper open/close
    1   - Position INIT (zero)
    2   - Position HOME
    q   - Quitter
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import sys
import termios
import tty
import time


MSG = """\
╔══════════════════════════════════════════════════════════╗
║       MyCobot Teleop Keyboard - Tour Control             ║
╠══════════════════════════════════════════════════════════╣
║  Déplacement (coordonnées [x,y,z,rx,ry,rz]):             ║
║                                                          ║
║                 w (x+)                                   ║
║                                                          ║
║       a (y-)    s (x-)    d (y+)                         ║
║                                                          ║
║       z (z-)    x (z+)                                   ║
║                                                          ║
║  Rotation:                                               ║
║       u (rx+)   i (ry+)   o (rz+)                        ║
║       j (rx-)   k (ry-)   l (rz-)                        ║
║                                                          ║
║  Gripper:                                                ║
║       g - Ouvrir                                         ║
║       h - Fermer                                         ║
║                                                          ║
║  Positions:                                              ║
║       1 - Position INIT (tous à zéro)                    ║
║       2 - Position HOME                                  ║
║                                                          ║
║  Autres:                                                 ║
║       +/- - Augmenter/Diminuer la vitesse                ║
║       q   - Quitter                                      ║
╚══════════════════════════════════════════════════════════╝
"""


class Raw:
    """Context manager pour lire les touches sans buffer"""
    def __init__(self, stream):
        self.stream = stream
        self.fd = self.stream.fileno()

    def __enter__(self):
        self.original_stty = termios.tcgetattr(self.stream)
        tty.setcbreak(self.stream)

    def __exit__(self, type, value, traceback):
        termios.tcsetattr(self.stream, termios.TCSANOW, self.original_stty)


class TeleopKeyboard(Node):
    """Node ROS2 pour le contrôle clavier"""
    
    def __init__(self):
        super().__init__('teleop_keyboard')
        
        # Publisher pour les commandes
        self.cmd_publisher = self.create_publisher(String, '/to_robot', 10)
        
        # Subscriber pour les réponses
        self.feedback_sub = self.create_subscription(
            String, '/from_robot', self.feedback_callback, 10)
        
        # État
        self.speed = 40
        self.change_percent = 5
        self.change_len = 250 * self.change_percent / 100  # mm
        self.change_angle = 180 * self.change_percent / 100  # degrees
        
        # Position actuelle (sera mise à jour par feedback)
        self.current_coords = [200.0, 0.0, 200.0, 180.0, 0.0, 0.0]
        
        # Positions prédéfinies
        self.init_angles = [0, 0, 0, 0, 0, 0]
        self.home_angles = [0, 8, -127, 40, 0, 0]
        
        self.get_logger().info("⌨️ Teleop Keyboard initialized")
    
    def feedback_callback(self, msg):
        """Traite les réponses du robot"""
        data = msg.data
        
        # Parser les coordonnées si présentes
        if data.startswith("coords:"):
            try:
                coords_str = data.replace("coords:", "")
                self.current_coords = eval(coords_str)
                self.get_logger().info(f"📍 Coords: {self.current_coords}")
            except:
                pass
    
    def send_json(self, action: str, **kwargs):
        """Envoie une commande JSON"""
        cmd = {"action": action, **kwargs}
        msg = String()
        msg.data = json.dumps(cmd)
        self.cmd_publisher.publish(msg)
    
    def send_coords(self):
        """Envoie les coordonnées actuelles"""
        self.send_json("send_coords", 
                      coords=self.current_coords, 
                      speed=self.speed, 
                      mode=1)
        self.get_logger().info(f"📤 Coords: {self.current_coords}")
    
    def move_x(self, delta):
        self.current_coords[0] += delta
        self.send_coords()
    
    def move_y(self, delta):
        self.current_coords[1] += delta
        self.send_coords()
    
    def move_z(self, delta):
        self.current_coords[2] += delta
        self.send_coords()
    
    def move_rx(self, delta):
        self.current_coords[3] += delta
        self.send_coords()
    
    def move_ry(self, delta):
        self.current_coords[4] += delta
        self.send_coords()
    
    def move_rz(self, delta):
        self.current_coords[5] += delta
        self.send_coords()
    
    def gripper_open(self):
        self.send_json("gripper_open")
        self.get_logger().info("✋ Gripper OPEN")
    
    def gripper_close(self):
        self.send_json("gripper_close")
        self.get_logger().info("✊ Gripper CLOSE")
    
    def go_init(self):
        self.send_json("send_angles", angles=self.init_angles, speed=self.speed)
        self.get_logger().info("0️⃣ Go to INIT position")
    
    def go_home(self):
        self.send_json("go_home")
        self.get_logger().info("🏠 Go to HOME position")
    
    def emergency_stop(self):
        self.send_json("emergency_stop")
        self.get_logger().warn("🚨 EMERGENCY STOP!")
    
    def run(self):
        """Boucle principale de lecture clavier"""
        print(MSG)
        print(f"Vitesse: {self.speed}  |  Incrément: {self.change_len:.1f}mm / {self.change_angle:.1f}°")
        print("\nAppuyez sur une touche...")
        
        # Demander les coordonnées initiales
        self.send_json("get_coords")
        time.sleep(0.5)
        rclpy.spin_once(self, timeout_sec=0.5)
        
        try:
            while rclpy.ok():
                # Traiter les messages ROS en attente
                rclpy.spin_once(self, timeout_sec=0.1)
                
                try:
                    with Raw(sys.stdin):
                        key = sys.stdin.read(1)
                    
                    if key == 'q':
                        self.emergency_stop()
                        print("\n👋 Au revoir!")
                        break
                    
                    # Mouvements de position
                    elif key in ['w', 'W']:
                        self.move_x(self.change_len)
                    elif key in ['s', 'S']:
                        self.move_x(-self.change_len)
                    elif key in ['a', 'A']:
                        self.move_y(-self.change_len)
                    elif key in ['d', 'D']:
                        self.move_y(self.change_len)
                    elif key in ['z', 'Z']:
                        self.move_z(-self.change_len)
                    elif key in ['x', 'X']:
                        self.move_z(self.change_len)
                    
                    # Mouvements de rotation
                    elif key in ['u', 'U']:
                        self.move_rx(self.change_angle)
                    elif key in ['j', 'J']:
                        self.move_rx(-self.change_angle)
                    elif key in ['i', 'I']:
                        self.move_ry(self.change_angle)
                    elif key in ['k', 'K']:
                        self.move_ry(-self.change_angle)
                    elif key in ['o', 'O']:
                        self.move_rz(self.change_angle)
                    elif key in ['l', 'L']:
                        self.move_rz(-self.change_angle)
                    
                    # Gripper
                    elif key in ['g', 'G']:
                        self.gripper_open()
                    elif key in ['h', 'H']:
                        self.gripper_close()
                    
                    # Positions prédéfinies
                    elif key == '1':
                        self.go_init()
                    elif key == '2':
                        self.go_home()
                    
                    # Vitesse
                    elif key == '+':
                        self.speed = min(100, self.speed + 10)
                        print(f"Vitesse: {self.speed}")
                    elif key == '-':
                        self.speed = max(10, self.speed - 10)
                        print(f"Vitesse: {self.speed}")
                    
                    # Afficher position courante
                    coords_str = ", ".join([f"{c:.1f}" for c in self.current_coords])
                    print(f"\r📍 [{coords_str}]  Vitesse: {self.speed}    ", end="", flush=True)
                    
                except Exception as e:
                    continue
                
                time.sleep(0.05)
        
        except KeyboardInterrupt:
            print("\n⚠️ Interruption clavier")
            self.emergency_stop()


def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    
    try:
        node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
