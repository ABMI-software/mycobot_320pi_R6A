"""Bridge JointTrajectory → JSON send_angles for the real MyCobot.

Subscribes to /mycobot_controller/joint_trajectory (populated by the teleop script or
any other trajectory publisher) and forwards each point to /to_robot as a JSON
send_angles command that bridge_tour relays to the Raspberry Pi.

Key behaviour:
- Converts positions from radians (ROS convention) to degrees (pymycobot convention)
- Reorders values to match the URDF joint_names in case the publisher uses a
  different ordering
- Rate-limits outgoing commands (pymycobot can't ingest 60 Hz over serial)
- Drops commands when consecutive positions are too close (deadband) to reduce
  servo wear
"""

from __future__ import annotations

import json
import math
import time
from typing import List

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from trajectory_msgs.msg import JointTrajectory


MYCOBOT_JOINT_ORDER = [
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
]


class TrajectoryToRobotBridge(Node):
    def __init__(self) -> None:
        super().__init__("trajectory_to_robot_bridge")

        self.declare_parameter("trajectory_topic", "/mycobot_controller/joint_trajectory")
        self.declare_parameter("out_topic", "/to_robot")
        self.declare_parameter("speed", 40)
        self.declare_parameter("rate_hz", 15.0)
        self.declare_parameter("deadband_deg", 1.0)
        self.declare_parameter("enable", True)

        trajectory_topic = self.get_parameter("trajectory_topic").value
        out_topic = self.get_parameter("out_topic").value
        self.speed = int(self.get_parameter("speed").value)
        self.min_period = 1.0 / float(self.get_parameter("rate_hz").value)
        self.deadband_deg = float(self.get_parameter("deadband_deg").value)
        self.enable = bool(self.get_parameter("enable").value)

        self.last_send_time = 0.0
        self.last_angles_deg: List[float] | None = None

        self.pub = self.create_publisher(String, out_topic, 10)
        self.sub = self.create_subscription(
            JointTrajectory, trajectory_topic, self.on_trajectory, 10
        )

        self.get_logger().info(
            f"Bridging {trajectory_topic} → {out_topic} "
            f"(speed={self.speed}, rate={1.0/self.min_period:.1f}Hz, "
            f"deadband={self.deadband_deg}°, enable={self.enable})"
        )

    def on_trajectory(self, msg: JointTrajectory) -> None:
        if not self.enable or not msg.points:
            return

        now = time.monotonic()
        if now - self.last_send_time < self.min_period:
            return

        positions = self._reorder(msg.joint_names, msg.points[0].positions)
        if positions is None:
            return

        angles_deg = [math.degrees(p) for p in positions]

        if self.last_angles_deg is not None:
            max_delta = max(abs(a - b) for a, b in zip(angles_deg, self.last_angles_deg))
            if max_delta < self.deadband_deg:
                return

        payload = {
            "action": "send_angles",
            "angles": [round(a, 2) for a in angles_deg],
            "speed": self.speed,
        }
        self.pub.publish(String(data=json.dumps(payload)))
        self.last_send_time = now
        self.last_angles_deg = angles_deg

    def _reorder(self, names: List[str], positions) -> List[float] | None:
        if not names:
            if len(positions) != len(MYCOBOT_JOINT_ORDER):
                self.get_logger().warn(
                    f"Received {len(positions)} positions without joint_names; "
                    f"expected {len(MYCOBOT_JOINT_ORDER)}."
                )
                return None
            return list(positions)

        try:
            name_to_pos = dict(zip(names, positions))
            return [name_to_pos[j] for j in MYCOBOT_JOINT_ORDER]
        except KeyError as e:
            self.get_logger().warn(f"Missing joint {e} in trajectory; skipping.")
            return None


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TrajectoryToRobotBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
