#!/usr/bin/env python3
"""FK EE pose publisher — pose du bout du bras depuis les encodeurs.

Souscrit à /joint_states (depuis le pont Gazebo ou le robot réel),
exécute la cinématique directe URDF (mycobot_fk.py) et publie la pose
de l'effecteur terminal dans le repère base.

Publié
------
  /fk/ee_pose   (geometry_msgs/PoseStamped, frame_id='base_link')

Ce nœud est identique en simulation et sur robot réel : il suffit que
/joint_states soit correctement alimenté.
"""

from __future__ import annotations

import os
import sys
import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.node import Node
from sensor_msgs.msg import JointState

# ── import du module FK du dépôt ─────────────────────────────────────────────
_DREAM_DIR_ALT = '/home/genji/ros_jazzy/src/mycobot_R6A/training/dream'
_DREAM_DIR = os.path.normpath(os.path.join(
    os.path.dirname(os.path.realpath(__file__)),
    '..', '..', 'training', 'dream'
))
for _p in [_DREAM_DIR, _DREAM_DIR_ALT]:
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from mycobot_fk import forward_kinematics  # type: ignore  # noqa: E402


# ── Noms des joints dans l'ordre de la chaîne URDF ───────────────────────────
JOINT_NAMES = [
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
]


def _rotation_matrix_to_quaternion(R: np.ndarray) -> Quaternion:
    """Matrice 3×3 → geometry_msgs/Quaternion (méthode de Shepperd, stable)."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    q = Quaternion()
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        q.w, q.x, q.y, q.z = (
            0.25 / s,
            (R[2, 1] - R[1, 2]) * s,
            (R[0, 2] - R[2, 0]) * s,
            (R[1, 0] - R[0, 1]) * s,
        )
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        q.w, q.x, q.y, q.z = (
            (R[2, 1] - R[1, 2]) / s, 0.25 * s,
            (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s,
        )
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        q.w, q.x, q.y, q.z = (
            (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s,
            0.25 * s,                 (R[1, 2] + R[2, 1]) / s,
        )
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        q.w, q.x, q.y, q.z = (
            (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s,
            (R[1, 2] + R[2, 1]) / s, 0.25 * s,
        )
    return q


class FkEePoseNode(Node):
    """Publie la pose EE issue de la cinématique directe."""

    def __init__(self) -> None:
        super().__init__("fk_ee_pose")

        self._pub = self.create_publisher(PoseStamped, "/fk/ee_pose", 10)
        self._sub = self.create_subscription(
            JointState, "/joint_states", self._joint_cb, 10
        )
        self._last_angles: np.ndarray | None = None
        self.get_logger().info("FK EE pose publisher prêt.")

    # ──────────────────────────────────────────────────────────────────────────

    def _joint_cb(self, msg: JointState) -> None:
        """Extrait les angles dans l'ordre URDF et exécute la FK."""
        name_to_pos = dict(zip(msg.name, msg.position))

        try:
            angles = np.array(
                [name_to_pos[jn] for jn in JOINT_NAMES], dtype=np.float64
            )
        except KeyError as missing:
            # joint_states parfois incomplet au démarrage
            self.get_logger().debug(f"Joint manquant dans joint_states : {missing}")
            return

        self._last_angles = angles

        # forward_kinematics retourne (positions_list, transforms_list)
        # Le dernier élément = transformée homogène world→link6 (EE)
        positions, transforms = forward_kinematics(angles)
        T_ee = transforms[-1]          # 4×4, repère base → EE

        pose = PoseStamped()
        pose.header.stamp    = msg.header.stamp
        pose.header.frame_id = "base_link"
        pose.pose.position.x = float(T_ee[0, 3])
        pose.pose.position.y = float(T_ee[1, 3])
        pose.pose.position.z = float(T_ee[2, 3])
        pose.pose.orientation = _rotation_matrix_to_quaternion(T_ee[:3, :3])

        self._pub.publish(pose)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = FkEePoseNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
