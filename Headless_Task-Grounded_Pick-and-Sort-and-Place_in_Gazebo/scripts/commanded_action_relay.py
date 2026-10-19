#!/usr/bin/env python3
"""Republish what the controllers COMMAND as one JointState, the episode action.

  /mycobot_controller/controller_state  reference.positions  (arm, interpolated target)
  /gripper_position_controller/commands data[0]              (gripper_controller target)
      -> /tri_sort/commanded_action  sensor_msgs/JointState, 6 arm joints + gripper_controller

rosetta has no decoder for JointTrajectoryControllerState; it reads JointState.
One message per controller_state message, stamped with its header (sim time).
The gripper value is the last command sent, held; before the first command it
is the measured gripper position, so the action never starts undefined.
"""
import rclpy
from control_msgs.msg import JointTrajectoryControllerState
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

ARM_JOINTS = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
              'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
GRIPPER = 'gripper_controller'


class CommandedActionRelay(Node):
    def __init__(self):
        super().__init__('commanded_action_relay',
                         parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.gripper = None
        self.pub = self.create_publisher(JointState, '/tri_sort/commanded_action', 50)
        self.create_subscription(JointTrajectoryControllerState,
                                 '/mycobot_controller/controller_state', self.on_arm, 50)
        self.create_subscription(Float64MultiArray, '/gripper_position_controller/commands',
                                 self.on_gripper, 10)
        self.create_subscription(JointState, '/joint_states', self.on_joint_states, 10)

    def on_gripper(self, msg):
        self.gripper = float(msg.data[0])

    def on_joint_states(self, msg):
        if self.gripper is None and GRIPPER in msg.name:
            self.gripper = float(msg.position[msg.name.index(GRIPPER)])

    def on_arm(self, msg):
        if self.gripper is None or len(msg.reference.positions) != len(msg.joint_names):
            return
        target = dict(zip(msg.joint_names, msg.reference.positions))
        out = JointState()
        out.header.stamp = msg.header.stamp
        out.name = ARM_JOINTS + [GRIPPER]
        out.position = [float(target[j]) for j in ARM_JOINTS] + [self.gripper]
        self.pub.publish(out)


def main():
    rclpy.init()
    node = CommandedActionRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
