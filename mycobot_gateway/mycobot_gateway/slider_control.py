#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Slider Control - Version Tour (PC)
Écoute le topic joint_states et envoie les positions au robot via le bridge.

Cette version tourne sur le PC Tour. Quand on bouge les sliders dans
joint_state_publisher_gui, les commandes sont envoyées au robot réel
via le bridge TCP.

Usage:
    ros2 run mycobot_gateway slider_control
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
import json
import math


class SliderControl(Node):
    """
    Écoute joint_states (de joint_state_publisher_gui)
    et envoie les positions au robot via /to_robot
    """
    
    def __init__(self):
        super().__init__('slider_control')
        
        # Subscriber pour joint_states (du GUI ou de RViz)
        self.joint_sub = self.create_subscription(
            JointState,
            'joint_states',
            self.joint_callback,
            10
        )
        
        # Publisher pour envoyer au bridge
        self.cmd_publisher = self.create_publisher(String, '/to_robot', 10)
        
        # État pour éviter le spam
        self.last_angles = None
        self.min_change = 0.02  # Radians minimum pour déclencher un envoi
        
        # Paramètres
        self.speed = 30
        
        self.get_logger().info("🎚️ Slider Control Node started")
        self.get_logger().info("   Listening to /joint_states")
        self.get_logger().info("   Publishing to /to_robot")
    
    def joint_callback(self, msg: JointState):
        """Callback quand on reçoit des joint_states"""
        
        # Vérifier qu'on a au moins 6 joints
        if len(msg.position) < 6:
            self.get_logger().warn(f"JointState trop court: {len(msg.position)} joints")
            return
        
        # Extraire les 6 premiers joints (en radians)
        radians = list(msg.position[:6])
        
        # Vérifier si c'est significativement différent
        if self.last_angles is not None:
            max_diff = max(abs(a - b) for a, b in zip(radians, self.last_angles))
            if max_diff < self.min_change:
                return  # Pas assez de changement
        
        self.last_angles = radians
        
        # Convertir en degrés pour l'envoi
        degrees = [math.degrees(r) for r in radians]
        
        # Envoyer la commande JSON
        cmd = {
            "action": "send_angles",
            "angles": degrees,
            "speed": self.speed
        }
        
        msg_out = String()
        msg_out.data = json.dumps(cmd)
        self.cmd_publisher.publish(msg_out)
        
        # Log (pas trop souvent)
        angles_str = ", ".join([f"{a:.1f}°" for a in degrees])
        self.get_logger().info(f"📤 Angles: [{angles_str}]")


def main(args=None):
    rclpy.init(args=args)
    node = SliderControl()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
