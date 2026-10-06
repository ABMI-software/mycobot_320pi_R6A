#!/usr/bin/env python3
"""Continuous keyboard teleop for the simulated MyCobot 320 Pi arm + gripper.

Unlike send_test_trajectory (one discrete waypoint per call), this publishes
a fresh short-horizon JointTrajectory point at a fixed rate, so holding a key
down (terminal key-autorepeat) drives continuous motion via mycobot_controller's
spline interpolation between closely-spaced targets -- a real "drag", not
a sequence of jumps.

Starts from the arm's CURRENT position (read from /joint_states) to avoid a
jump on launch.

Usage:
  ros2 run gazebo_to_lerobot_bringup teleop_arm_keyboard

Keys:
  q/a : joint2_to_joint1     (base)      +/-
  w/s : joint3_to_joint2     (shoulder)  +/-
  e/d : joint4_to_joint3     (elbow)     +/-
  r/f : joint5_to_joint4                +/-
  t/g : joint6_to_joint5                +/-
  y/h : joint6output_to_joint6 (wrist)  +/-
  o   : open gripper
  c   : close gripper
  z   : reset arm to home (all zero)
  CTRL-C : quit
"""

import select
import sys
import termios
import tty

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

JOINT_NAMES = [
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
]

# From mycobot_pro_320_pi_gazebo.urdf's <limit> tags.
JOINT_LIMITS = [
    (-2.93, 2.93),
    (-2.35, 2.35),
    (-2.53, 2.53),
    (-2.53, 2.53),
    (-2.93, 2.93),
    (-3.14, 3.14),
]

# key -> (joint index, direction)
KEY_MAP = {
    "q": (0, 1), "a": (0, -1),
    "w": (1, 1), "s": (1, -1),
    "e": (2, 1), "d": (2, -1),
    "r": (3, 1), "f": (3, -1),
    "t": (4, 1), "g": (4, -1),
    "y": (5, 1), "h": (5, -1),
}

STEP = 0.05  # rad per keypress
PUBLISH_PERIOD_S = 0.1
TIME_FROM_START_S = 0.2

# gripper_position_controller joint order (see controller.yaml):
# [servo_left, servo_right, tip_left, tip_right, bar_left, bar_right]
# close = [-a, +a, +a, -a, -a, +a], open = six zeros.
GRIPPER_CLOSE_A = 0.7
GRIPPER_OPEN = [0.0] * 6
GRIPPER_CLOSE = [-GRIPPER_CLOSE_A, GRIPPER_CLOSE_A, GRIPPER_CLOSE_A,
                 -GRIPPER_CLOSE_A, -GRIPPER_CLOSE_A, GRIPPER_CLOSE_A]

HELP = """
MyCobot 320 Pi -- sim keyboard teleop

  q/a : joint1 (base)       +/-
  w/s : joint2 (shoulder)   +/-
  e/d : joint3 (elbow)      +/-
  r/f : joint4              +/-
  t/g : joint5              +/-
  y/h : joint6 (wrist)      +/-

  o   : open gripper
  c   : close gripper
  z   : reset arm to home (all zero)
  CTRL-C : quit
"""


class ArmTeleop(Node):
    def __init__(self):
        super().__init__("teleop_arm_keyboard")
        self.traj_pub = self.create_publisher(
            JointTrajectory, "/mycobot_controller/joint_trajectory", 10)
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, "/gripper_position_controller/commands", 10)

        self.positions = None
        self.js_sub = self.create_subscription(
            JointState, "/joint_states", self._on_joint_states, 10)

    def _on_joint_states(self, msg: JointState):
        if self.positions is not None:
            return  # only need the first message to seed our target
        try:
            self.positions = [msg.position[msg.name.index(j)] for j in JOINT_NAMES]
            self.get_logger().info(f"Seeded start position: {self.positions}")
        except ValueError:
            pass  # not all joint names present yet

    def apply_key(self, key: str) -> bool:
        """Returns True if a redraw/publish is warranted."""
        if key in KEY_MAP and self.positions is not None:
            idx, direction = KEY_MAP[key]
            lo, hi = JOINT_LIMITS[idx]
            self.positions[idx] = max(lo, min(hi, self.positions[idx] + direction * STEP))
            return True
        if key == "z" and self.positions is not None:
            self.positions = [0.0] * 6
            return True
        if key == "o":
            self._publish_gripper(GRIPPER_OPEN)
        elif key == "c":
            self._publish_gripper(GRIPPER_CLOSE)
        return False

    def _publish_gripper(self, values):
        msg = Float64MultiArray()
        msg.data = values
        self.gripper_pub.publish(msg)

    def publish_target(self):
        if self.positions is None:
            return
        msg = JointTrajectory()
        msg.joint_names = JOINT_NAMES
        point = JointTrajectoryPoint()
        point.positions = list(self.positions)
        point.time_from_start.sec = 0
        point.time_from_start.nanosec = int(TIME_FROM_START_S * 1e9)
        msg.points = [point]
        self.traj_pub.publish(msg)


def _read_key(timeout_s: float) -> str:
    ready, _, _ = select.select([sys.stdin], [], [], timeout_s)
    if ready:
        return sys.stdin.read(1)
    return ""


def main():
    rclpy.init(args=sys.argv)
    node = ArmTeleop()
    print(HELP)
    print("Waiting for /joint_states to seed starting position...")

    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        while node.positions is None and rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.2)

        print("Ready. Hold a key to move continuously; release to stop.\n")
        while rclpy.ok():
            key = _read_key(PUBLISH_PERIOD_S)
            if key == "\x03":  # Ctrl-C
                break
            if key:
                node.apply_key(key)
                print(f"\rjoints: {['%.2f' % p for p in node.positions]}   ", end="", flush=True)
            node.publish_target()
            rclpy.spin_once(node, timeout_sec=0.0)
    except KeyboardInterrupt:
        pass
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        print("\nExiting teleop.")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
