#!/usr/bin/env python3
"""Publish one JointTrajectory point to /mycobot_controller/joint_trajectory
as a direct (non-MoveIt2) motion test. Confirms the sim + controller path
works: watch the arm move in Gazebo and /joint_states converge to the
commanded positions.

Usage:
  ros2 run gazebo_to_lerobot_bringup send_test_trajectory
  ros2 run gazebo_to_lerobot_bringup send_test_trajectory --positions 0.3 -0.2 0.2 0.0 0.0 0.0 --duration 4.0
"""

import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINT_NAMES = [
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--positions", type=float, nargs=6, default=[0.3, -0.2, 0.2, 0.0, 0.0, 0.0],
        help="6 target positions in radians, in JOINT_NAMES order.",
    )
    parser.add_argument("--duration", type=float, default=4.0, help="time_from_start, seconds")
    args = parser.parse_args()

    rclpy.init(args=sys.argv)
    node = Node("send_test_trajectory")
    pub = node.create_publisher(JointTrajectory, "/mycobot_controller/joint_trajectory", 10)

    # give the publisher a moment to match with the controller's subscription
    time.sleep(1.0)

    msg = JointTrajectory()
    msg.joint_names = JOINT_NAMES
    point = JointTrajectoryPoint()
    point.positions = list(args.positions)
    point.time_from_start.sec = int(args.duration)
    point.time_from_start.nanosec = int((args.duration % 1.0) * 1e9)
    msg.points = [point]

    node.get_logger().info(f"Publishing target positions {args.positions} over {args.duration}s")
    pub.publish(msg)
    time.sleep(0.5)  # let the message actually go out before shutdown

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
