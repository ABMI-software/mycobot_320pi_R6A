#!/usr/bin/env python3
"""Localizer Gazebo — remplace aruco_localizer_node en mode simulation.

En simulation, la pose de l'objet est connue avec précision (vérité terrain
Gazebo). Ce nœud lit la pose du cube cible depuis le pont ros_gz_bridge et
la republication sur /aruco/object_pose, assurant une interface identique
au localizer ArUco pour le robot réel.

/aruco/workspace_valid est toujours True en simulation.

Paramètres ROS 2
----------------
  target_x     : position X initiale de l'objet (défaut : 0.25)
  target_y     : position Y initiale de l'objet (défaut : 0.10)
  target_z     : position Z initiale de l'objet (défaut : 0.02)
  gz_pose_topic: topic Gazebo bridgé (défaut : /gz/model/target_cube/pose)

Si gz_pose_topic n'est pas disponible, publie la position statique fournie
par les paramètres (suffit pour la grille de benchmark).
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
from std_msgs.msg import Bool


class GzSimLocalizerNode(Node):
    """Localizer simulation : republication de la pose Gazebo GT."""

    def __init__(self) -> None:
        super().__init__("gz_sim_localizer")

        self.declare_parameter("target_x",      0.25)
        self.declare_parameter("target_y",      0.10)
        self.declare_parameter("target_z",      0.02)
        self.declare_parameter("gz_pose_topic", "/gz/model/target_cube/pose")

        self._tx = float(self.get_parameter("target_x").value)
        self._ty = float(self.get_parameter("target_y").value)
        self._tz = float(self.get_parameter("target_z").value)
        gz_topic = self.get_parameter("gz_pose_topic").value

        # publishers
        self._pub_pose  = self.create_publisher(PoseStamped, "/aruco/object_pose",     5)
        self._pub_valid = self.create_publisher(Bool,         "/aruco/workspace_valid", 5)

        # abonnement à la pose Gazebo bridgée (geometry_msgs/Pose)
        self._gz_sub = self.create_subscription(
            Pose, gz_topic, self._gz_pose_cb, 5
        )

        # fallback : timer qui publie la position statique (paramètres)
        # s'active si aucune donnée Gazebo n'arrive après 2 secondes
        self._got_gz_pose = False
        self.create_timer(2.0, self._fallback_timer)

        self.get_logger().info(
            f"GZ sim localizer  |  cible initiale "
            f"({self._tx:.3f}, {self._ty:.3f}, {self._tz:.3f})"
            f"  |  topic Gazebo : {gz_topic}"
        )

    # ──────────────────────────────────────────────────────────────────────────

    def _gz_pose_cb(self, msg: Pose) -> None:
        """Reçoit la pose du cube depuis Gazebo (geometry_msgs/Pose)."""
        self._got_gz_pose = True
        pose = PoseStamped()
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.header.frame_id = "base_link"
        pose.pose            = msg
        self._pub_pose.publish(pose)
        self._pub_valid.publish(Bool(data=True))

    def _fallback_timer(self) -> None:
        """Publie la position statique si Gazebo n'envoie rien."""
        if self._got_gz_pose:
            # La publication Gazebo est active → fallback inutile
            return

        pose = PoseStamped()
        pose.header.stamp    = self.get_clock().now().to_msg()
        pose.header.frame_id = "base_link"
        pose.pose.position.x = self._tx
        pose.pose.position.y = self._ty
        pose.pose.position.z = self._tz
        pose.pose.orientation.w = 1.0
        self._pub_pose.publish(pose)
        self._pub_valid.publish(Bool(data=True))


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GzSimLocalizerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
