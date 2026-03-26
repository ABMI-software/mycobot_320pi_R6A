#!/usr/bin/env python3
"""
Joint State Synchronizer - Synchronise les joints RViz avec le robot réel.

Ce node:
1. Écoute les angles du robot réel via /from_robot
2. Publie les JointState pour que RViz affiche la position réelle
3. Permet de demander périodiquement les angles au robot

Usage:
    ros2 run mycobot_gateway joint_sync
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import re
import math


class JointStateSynchronizer(Node):
    """
    Synchronise l'état des joints entre le robot réel et RViz.
    """
    
    def __init__(self):
        super().__init__('joint_state_synchronizer')
        
        # Noms des joints (doivent correspondre à l'URDF)
        self.joint_names = [
            'joint2_to_joint1',  # Joint 1 (base)
            'joint3_to_joint2',  # Joint 2
            'joint4_to_joint3',  # Joint 3
            'joint5_to_joint4',  # Joint 4
            'joint6_to_joint5',  # Joint 5
            'joint6output_to_joint6'  # Joint 6 (end effector)
        ]
        
        # Derniers angles connus (en radians)
        self.current_angles = [0.0] * 6
        
        # Publisher pour les JointState (vers RViz)
        self.joint_state_pub = self.create_publisher(
            JointState,
            '/joint_states',
            10
        )
        
        # Publisher pour envoyer des commandes au robot
        self.command_pub = self.create_publisher(
            String,
            '/to_robot',
            10
        )
        
        # Subscriber pour recevoir les réponses du robot
        self.response_sub = self.create_subscription(
            String,
            '/from_robot',
            self.response_callback,
            10
        )
        
        # Timer pour publier les JointState régulièrement
        self.publish_timer = self.create_timer(0.033, self.publish_joint_states)  # 30 Hz
        
        # Timer pour demander les angles au robot (optionnel, peut être désactivé)
        self.declare_parameter('auto_sync', True)
        self.declare_parameter('sync_rate', 5.0)  # Hz
        
        auto_sync = self.get_parameter('auto_sync').value
        sync_rate = self.get_parameter('sync_rate').value
        
        if auto_sync:
            self.sync_timer = self.create_timer(1.0 / sync_rate, self.request_angles)
            self.get_logger().info(f'🔄 Auto-sync activé à {sync_rate} Hz')
        
        self.get_logger().info('🤖 Joint State Synchronizer démarré')
        self.get_logger().info('   Écoute sur /from_robot pour les angles')
        self.get_logger().info('   Publie sur /joint_states pour RViz')
    
    def degrees_to_radians(self, degrees):
        """Convertit des degrés en radians."""
        return degrees * math.pi / 180.0
    
    def response_callback(self, msg: String):
        """Traite les réponses du robot."""
        data = msg.data.strip()
        
        # Parser les angles reçus: "angles:[0.35, 0.0, 0.0, 0.35, 0.35, 0.26]"
        if data.startswith('angles:'):
            try:
                # Extraire la liste d'angles
                angles_str = data.replace('angles:', '')
                # Parser la liste Python
                angles_deg = eval(angles_str)
                
                if len(angles_deg) == 6:
                    # Convertir en radians
                    self.current_angles = [
                        self.degrees_to_radians(a) for a in angles_deg
                    ]
                    self.get_logger().debug(f'📐 Angles mis à jour: {angles_deg}°')
                    
            except Exception as e:
                self.get_logger().warn(f'⚠️ Erreur parsing angles: {e}')
        
        # Parser aussi angles_ok après set_angles
        elif 'angles_ok:' in data:
            try:
                # Format: "angles_ok:[0.0, 0.0, ...],s=20"
                match = re.search(r'\[([\d\., -]+)\]', data)
                if match:
                    angles_str = match.group(1)
                    angles_deg = [float(a.strip()) for a in angles_str.split(',')]
                    if len(angles_deg) == 6:
                        self.current_angles = [
                            self.degrees_to_radians(a) for a in angles_deg
                        ]
                        self.get_logger().debug(f'📐 Angles confirmés: {angles_deg}°')
            except Exception as e:
                self.get_logger().warn(f'⚠️ Erreur parsing angles_ok: {e}')
    
    def request_angles(self):
        """Demande les angles actuels au robot."""
        msg = String()
        msg.data = 'get_angles'
        self.command_pub.publish(msg)
    
    def publish_joint_states(self):
        """Publie l'état actuel des joints."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = self.joint_names
        msg.position = self.current_angles
        msg.velocity = []
        msg.effort = []
        
        self.joint_state_pub.publish(msg)
    
    def set_angles(self, angles_deg: list, speed: int = 20):
        """
        Envoie une commande pour définir les angles du robot.
        
        Args:
            angles_deg: Liste de 6 angles en degrés
            speed: Vitesse du mouvement (1-100)
        """
        if len(angles_deg) != 6:
            self.get_logger().error('❌ Il faut exactement 6 angles')
            return
        
        angles_str = ','.join([str(a) for a in angles_deg])
        msg = String()
        msg.data = f'set_angles:{angles_str}:{speed}'
        self.command_pub.publish(msg)
        self.get_logger().info(f'📤 Envoi angles: {angles_deg} (vitesse {speed})')


def main(args=None):
    rclpy.init(args=args)
    node = JointStateSynchronizer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
