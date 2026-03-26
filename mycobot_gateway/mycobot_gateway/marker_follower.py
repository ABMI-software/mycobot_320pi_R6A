#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Marker Follower - Version Tour (PC)
Suit un marqueur ArUco détecté et envoie les commandes au robot via le bridge.

Cette version tourne sur le PC Tour :
- Écoute les TF du marqueur détecté (de detect_marker ou simulation)
- Calcule la position cible pour le robot
- Envoie les commandes via /to_robot

Prérequis:
- detect_marker node doit tourner (ou simulation Gazebo avec ArUco)
- bridge_tour connecté au Pi

Usage:
    ros2 run mycobot_gateway marker_follower
"""

import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
import json
import math


class MarkerFollower(Node):
    """
    Suit un marqueur ArUco et envoie les commandes au robot.
    Tourne sur le PC Tour.
    """
    
    def __init__(self):
        super().__init__('marker_follower')
        
        # TF Buffer pour écouter les transformations
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # Publisher pour envoyer les commandes au robot
        self.cmd_publisher = self.create_publisher(String, '/to_robot', 10)
        
        # Timer pour le suivi (5 Hz)
        self.timer = self.create_timer(0.2, self.timer_callback)
        
        # Paramètres
        self.enabled = False  # Désactivé par défaut
        self.source_frame = "camera_link"  # Frame de la caméra
        self.target_frame = "basic_shapes"  # Frame du marqueur ArUco
        self.robot_base_frame = "base"  # Frame de base du robot
        
        # Limites de sécurité (mm)
        self.x_min, self.x_max = 130, 350
        self.y_min, self.y_max = -200, 200
        self.z_min, self.z_max = 100, 400
        
        # Vitesse de suivi
        self.follow_speed = 40
        
        # Subscriber pour les commandes de contrôle
        self.control_sub = self.create_subscription(
            String, '/to_robot', self.control_callback, 10)
        
        self.get_logger().info("🎯 Marker Follower initialized")
        self.get_logger().info("   Source frame: " + self.source_frame)
        self.get_logger().info("   Target frame: " + self.target_frame)
        self.get_logger().info("   Following: DISABLED (send 'follow_on' to enable)")
    
    def control_callback(self, msg):
        """Traite les commandes de contrôle du follower"""
        try:
            data = json.loads(msg.data)
            action = data.get('action', '')
            
            if action == 'follow_on':
                self.enabled = True
                self.get_logger().info("✅ Marker following ENABLED")
            elif action == 'follow_off':
                self.enabled = False
                self.get_logger().info("⏸️ Marker following DISABLED")
            elif action == 'follow_speed':
                self.follow_speed = data.get('speed', 40)
                self.get_logger().info(f"🚀 Follow speed set to {self.follow_speed}")
        except:
            pass
    
    def timer_callback(self):
        """Callback périodique pour le suivi du marqueur"""
        if not self.enabled:
            return
        
        try:
            # Essayer d'obtenir la transformation du marqueur
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform(
                self.robot_base_frame,  # Target frame (base du robot)
                self.target_frame,      # Source frame (marqueur)
                now,
                timeout=rclpy.duration.Duration(seconds=0.1)
            )
            
            # Extraire les coordonnées (convertir en mm)
            x = trans.transform.translation.x * 1000
            y = trans.transform.translation.y * 1000
            z = trans.transform.translation.z * 1000
            
            self.get_logger().info(f"📍 Marker detected: X={x:.0f} Y={y:.0f} Z={z:.0f}")
            
            # Vérifier si le marqueur est dans la zone de travail
            # Note: Les coordonnées peuvent être négatives selon l'orientation de la caméra
            if not self.is_in_workspace(x, y, z):
                self.get_logger().warn("⚠️ Marker outside workspace")
                return
            
            # Calculer la position cible pour le robot
            target_coords = self.compute_target(x, y, z)
            
            if target_coords:
                self.send_coords(target_coords)
        
        except Exception as e:
            # Marqueur non détecté - c'est normal si pas de marqueur en vue
            pass
    
    def is_in_workspace(self, x, y, z):
        """Vérifie si les coordonnées sont dans l'espace de travail"""
        # Adapter selon la configuration de votre caméra
        # Généralement X est négatif si la caméra est orientée vers le robot
        abs_x = abs(x)
        return (100 < abs_x < 800) and (-500 < y < 500) and (0 < z < 600)
    
    def compute_target(self, x, y, z):
        """
        Calcule les coordonnées cibles pour le robot.
        Adapte les coordonnées du marqueur vers les coordonnées robot.
        """
        # Transformation des coordonnées caméra vers robot
        # À adapter selon votre configuration physique
        
        # Si X est négatif (caméra devant le robot orientée vers lui)
        target_x = abs(x)
        target_y = -y  # Inversion miroir
        target_z = z
        
        # Appliquer les limites de sécurité
        target_x = max(self.x_min, min(target_x, self.x_max))
        target_y = max(self.y_min, min(target_y, self.y_max))
        target_z = max(self.z_min, min(target_z, self.z_max))
        
        # Orientation fixe (outil vers le bas)
        rx, ry, rz = 180.0, 0.0, 0.0
        
        return [float(target_x), float(target_y), float(target_z), rx, ry, rz]
    
    def send_coords(self, coords):
        """Envoie les coordonnées au robot via le bridge"""
        cmd = {
            "action": "send_coords",
            "coords": coords,
            "speed": self.follow_speed,
            "mode": 1  # Interpolation linéaire
        }
        
        msg = String()
        msg.data = json.dumps(cmd)
        self.cmd_publisher.publish(msg)
        
        coords_str = ", ".join([f"{c:.1f}" for c in coords[:3]])
        self.get_logger().info(f"📤 Target: [{coords_str}]")


def main(args=None):
    rclpy.init(args=args)
    node = MarkerFollower()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
