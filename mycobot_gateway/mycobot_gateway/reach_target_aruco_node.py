#!/usr/bin/env python3
"""One-shot reach test — atteint la position ArUco objet sans gripper.

Attend que workspace_valid=true et que /aruco/object_pose soit publié,
calcule l'IK pour la position cible (objet + offsets), publie une seule
JointTrajectory, attend move_time secondes puis s'arrête.

Usage
-----
  ros2 run mycobot_gateway reach_target_aruco --ros-args \\
      -p approach_height:=0.10 \\
      -p target_z_offset:=-0.03 \\
      -p move_time:=3.0

Paramètres
----------
  approach_height  : hauteur au-dessus du marqueur objet détecté (m)  défaut 0.10
  target_z_offset  : biais Z sur la pose ArUco (ex. top→centre objet) défaut -0.03
  move_time        : durée du mouvement (time_from_start, s)          défaut 3.0
  pose_timeout     : délai max attente /aruco/object_pose (s)         défaut 30.0
  require_ws_valid : attendre workspace_valid=true avant de bouger     défaut True
"""

from __future__ import annotations

import os
import sys
from typing import Optional

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

_DREAM_DIR_ALT = "/home/genji/ros_jazzy/src/mycobot_R6A/training/dream"
_DREAM_DIR = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "..", "..", "..", "training", "dream",
))
for _p in [_DREAM_DIR, _DREAM_DIR_ALT]:
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from mycobot_fk import forward_kinematics           # type: ignore  # noqa: E402
from mycobot_ik import inverse_kinematics_position  # type: ignore  # noqa: E402

JOINT_NAMES = [
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
]

HOME_ANGLES = np.array([0.0, -0.8, 1.4, -0.8, 0.0, 0.0], dtype=np.float64)


class ReachTargetArucoNode(Node):
    def __init__(self) -> None:
        super().__init__("reach_target_aruco")

        self.declare_parameter("approach_height",  0.10)
        self.declare_parameter("target_z_offset",  -0.03)
        self.declare_parameter("move_time",         3.0)
        self.declare_parameter("pose_timeout",      30.0)
        self.declare_parameter("require_ws_valid",  True)

        self._approach_h    = float(self.get_parameter("approach_height").value)
        self._z_offset      = float(self.get_parameter("target_z_offset").value)
        self._move_time     = float(self.get_parameter("move_time").value)
        self._pose_timeout  = float(self.get_parameter("pose_timeout").value)
        self._require_ws    = bool(self.get_parameter("require_ws_valid").value)

        self._object_pos : Optional[np.ndarray] = None
        self._joint_pos  : Optional[np.ndarray] = None
        self._ws_valid   : bool                  = False
        self._fired      : bool                  = False
        self._deadline   : float                 = self._now() + self._pose_timeout

        self._traj_pub = self.create_publisher(
            JointTrajectory, "/mycobot_controller/joint_trajectory", 1
        )

        self.create_subscription(PoseStamped, "/aruco/object_pose",     self._obj_cb,  5)
        self.create_subscription(Bool,        "/aruco/workspace_valid", self._ws_cb,   5)
        self.create_subscription(JointState,  "/joint_states",          self._js_cb,  10)

        self.create_timer(0.1, self._step)
        self.get_logger().info(
            f"reach_target_aruco prêt — "
            f"approach={self._approach_h}m  z_offset={self._z_offset}m  "
            f"move_time={self._move_time}s  require_ws={self._require_ws}"
        )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _obj_cb(self, msg: PoseStamped) -> None:
        self._object_pos = np.array([
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z,
        ])

    def _ws_cb(self, msg: Bool) -> None:
        self._ws_valid = msg.data

    def _js_cb(self, msg: JointState) -> None:
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            self._joint_pos = np.array(
                [name_to_pos[j] for j in JOINT_NAMES], dtype=np.float64
            )
        except KeyError:
            pass

    def _step(self) -> None:
        if self._fired:
            return

        ws_ok = (not self._require_ws) or self._ws_valid
        ready = (
            self._object_pos is not None
            and self._joint_pos is not None
            and ws_ok
        )

        if not ready:
            if self._now() > self._deadline:
                self.get_logger().error(
                    f"Timeout {self._pose_timeout}s — "
                    f"object_pose={self._object_pos is not None}  "
                    f"joint_states={self._joint_pos is not None}  "
                    f"ws_valid={self._ws_valid}"
                )
                rclpy.shutdown()
            return

        target = self._object_pos.copy()
        target[2] += self._z_offset + self._approach_h

        self.get_logger().info(
            f"Objet détecté  xyz={np.round(self._object_pos, 3)}  "
            f"→ cible  xyz={np.round(target, 3)}"
        )

        ok, q, err = inverse_kinematics_position(target, q0=self._joint_pos)
        if not ok:
            self.get_logger().error(
                f"IK échoué pour {np.round(target, 3)} — err={err*1000:.1f} mm"
            )
            rclpy.shutdown()
            return

        positions, _ = forward_kinematics(q)
        ee = np.array(positions["mycobot320_link6"])
        dist_mm = np.linalg.norm(ee - target) * 1000.0
        self.get_logger().info(
            f"IK OK  joints={np.round(np.degrees(q), 1)}°  err={dist_mm:.1f} mm"
        )

        traj = JointTrajectory()
        traj.header.stamp = self.get_clock().now().to_msg()
        traj.joint_names = list(JOINT_NAMES)
        pt = JointTrajectoryPoint()
        pt.positions = [float(a) for a in q]
        secs = int(self._move_time)
        pt.time_from_start = Duration(sec=secs, nanosec=int((self._move_time - secs) * 1e9))
        traj.points = [pt]
        self._traj_pub.publish(traj)

        self._fired = True
        self.get_logger().info(
            f"Trajectoire publiée — attente {self._move_time}s puis arrêt."
        )
        self.create_timer(self._move_time + 0.5, self._shutdown)

    def _shutdown(self) -> None:
        self.get_logger().info("Mouvement terminé.")
        rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ReachTargetArucoNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == "__main__":
    main()
