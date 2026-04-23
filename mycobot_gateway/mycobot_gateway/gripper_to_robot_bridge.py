"""Bridge gripper command → JSON gripper_open / gripper_close for the real robot.

The sim pipeline publishes a continuous position in radians on
/gripper_position_controller/commands (Float64MultiArray), but
bridge_pi_simple.py on the Pi only understands two discrete gripper
actions. This node does the continuous-to-binary translation with a
Schmitt-trigger (hysteresis) + debounce, modelled on the hand_control
project's gripper conditioning pipeline (deadband → EMA → slew) — here
the EMA/deadband/slew are done upstream in mycobot_teleop.py, and this
node just converts the clean signal to discrete open/close events.

State machine:
    unknown ──(cmd > open_threshold)──▶ open
        open ──(cmd < close_threshold AND debounce elapsed)──▶ close
        close ──(cmd > open_threshold AND debounce elapsed)──▶ open

Any state change publishes a single {"action": "gripper_open"|"gripper_close"}
JSON to /to_robot. No repeated publishes while state is stable — pymycobot
holds the last command on the servo.
"""
from __future__ import annotations

import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, String


class GripperToRobotBridge(Node):
    def __init__(self) -> None:
        super().__init__("gripper_to_robot_bridge")

        self.declare_parameter("gripper_topic", "/gripper_position_controller/commands")
        self.declare_parameter("out_topic", "/to_robot")
        # URDF limit is -0.7 (closed) → 0 (open). Hysteresis zone in the middle:
        # above -0.20 rad we call it "open", below -0.50 rad we call it "closed".
        self.declare_parameter("open_threshold_rad", -0.20)
        self.declare_parameter("close_threshold_rad", -0.50)
        # Minimum time between two state transitions (seconds). Stops the
        # operator's hand-tremor from flapping the servo back and forth.
        self.declare_parameter("min_state_change_s", 0.4)
        self.declare_parameter("enable", True)

        gripper_topic = self.get_parameter("gripper_topic").value
        out_topic = self.get_parameter("out_topic").value
        self.open_th = float(self.get_parameter("open_threshold_rad").value)
        self.close_th = float(self.get_parameter("close_threshold_rad").value)
        self.min_change = float(self.get_parameter("min_state_change_s").value)
        self.enable = bool(self.get_parameter("enable").value)

        if self.close_th >= self.open_th:
            self.get_logger().error(
                f"close_threshold ({self.close_th}) must be STRICTLY LESS than "
                f"open_threshold ({self.open_th}) for hysteresis to work. "
                "Swap or tune them."
            )

        self.state = "unknown"  # "open" | "close" | "unknown"
        self.last_change_t = 0.0

        self.pub = self.create_publisher(String, out_topic, 10)
        self.sub = self.create_subscription(
            Float64MultiArray, gripper_topic, self.on_gripper, 10
        )

        self.get_logger().info(
            f"Bridging {gripper_topic} → {out_topic}  "
            f"(open≥{self.open_th:.2f} rad, close≤{self.close_th:.2f} rad, "
            f"debounce={self.min_change:.2f}s, enable={self.enable})"
        )

    def on_gripper(self, msg: Float64MultiArray) -> None:
        if not self.enable or not msg.data:
            return
        cmd = float(msg.data[0])

        now = time.monotonic()
        if now - self.last_change_t < self.min_change:
            return

        new_state = self.state
        if self.state in ("unknown", "close") and cmd > self.open_th:
            new_state = "open"
        elif self.state in ("unknown", "open") and cmd < self.close_th:
            new_state = "close"
        # In the hysteresis band, keep current state.

        if new_state != self.state:
            action = "gripper_open" if new_state == "open" else "gripper_close"
            payload = {"action": action}
            self.pub.publish(String(data=json.dumps(payload)))
            self.get_logger().info(
                f"state {self.state} → {new_state}  (cmd={cmd:+.3f} rad)  "
                f"sent {action}"
            )
            self.state = new_state
            self.last_change_t = now


def main(args=None) -> None:
    rclpy.init(args=args)
    node = GripperToRobotBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
