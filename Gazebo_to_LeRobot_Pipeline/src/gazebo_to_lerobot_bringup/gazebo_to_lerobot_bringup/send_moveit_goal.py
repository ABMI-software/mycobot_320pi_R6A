#!/usr/bin/env python3
"""Send a joint-space goal to MoveIt2's move_group (plan AND execute), as a
non-GUI way to prove the MoveIt2 pipeline commands the arm. Requires
move_group.launch.py (package mycobot_moveit_config) already running.

Usage:
  ros2 run gazebo_to_lerobot_bringup send_moveit_goal
  ros2 run gazebo_to_lerobot_bringup send_moveit_goal --positions 0.3 -0.2 0.2 0.0 0.0 0.0
"""

import argparse
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint

JOINT_NAMES = [
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
]


class MoveItGoalSender(Node):
    def __init__(self, positions):
        super().__init__("send_moveit_goal")
        self._client = ActionClient(self, MoveGroup, "/move_action")
        self._positions = positions
        self._done = False
        self._success = False

    def send(self):
        self.get_logger().info("Waiting for /move_action server...")
        self._client.wait_for_server()

        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 15.0
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5

        constraints = Constraints()
        for name, pos in zip(JOINT_NAMES, self._positions):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = pos
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = False  # plan AND execute

        self.get_logger().info(f"Sending goal: {self._positions}")
        future = self._client.send_goal_async(goal)
        future.add_done_callback(self._goal_response_callback)

    def _goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected by move_group")
            self._done = True
            return
        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._result_callback)

    def _result_callback(self, future):
        result = future.result().result
        self.get_logger().info(
            f"MoveGroup finished with error_code.val={result.error_code.val} "
            "(1 = SUCCESS, see moveit_msgs/MoveItErrorCodes)"
        )
        self._success = result.error_code.val == 1
        self._done = True


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--positions", type=float, nargs=6, default=[0.3, -0.2, 0.2, 0.0, 0.0, 0.0],
        help="6 target joint positions in radians, in JOINT_NAMES order.",
    )
    args = parser.parse_args()

    rclpy.init(args=sys.argv)
    node = MoveItGoalSender(args.positions)
    node.send()
    while rclpy.ok() and not node._done:
        rclpy.spin_once(node, timeout_sec=1.0)

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if node._success else 1)


if __name__ == "__main__":
    main()
