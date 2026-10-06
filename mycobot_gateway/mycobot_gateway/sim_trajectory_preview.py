"""One slow demonstration sweep in an isolated Gazebo session, not A/B/G."""
import math
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from controller_manager_msgs.srv import ListControllers
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint


JOINTS = [
    'joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
    'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6',
]
# Elevated slow sweep; joint interpolation, not a Cartesian straight line.
POSES_DEG = [
    [0, -25, -35, 0, 0, 0],
    [25, -25, -35, 0, 0, 0],
    [-25, -25, -35, 0, 0, 0],
    [0, -15, -25, 0, 0, 0],
    [0, -25, -35, 0, 0, 0],
]


class TrajectoryPreview(Node):
    def __init__(self):
        super().__init__('sim_trajectory_preview')
        if not self.get_parameter('use_sim_time').value:
            raise RuntimeError('This demonstration requires use_sim_time:=true.')
        self.positions = None
        self.received_at = 0.0
        self.subscription = self.create_subscription(JointState, '/joint_states', self.state, 10)
        self.client = ActionClient(self, FollowJointTrajectory,
                                   '/mycobot_controller/follow_joint_trajectory')
        self.controllers = self.create_client(ListControllers, '/controller_manager/list_controllers')
        self.get_logger().info('Demonstration de trajectoire uniquement — pas les essais A/B/G.')

    def state(self, message):
        values = dict(zip(message.name, message.position))
        if all(name in values and math.isfinite(values[name]) for name in JOINTS):
            self.positions = [values[name] for name in JOINTS]
            self.received_at = time.monotonic()

    def run(self):
        deadline = time.monotonic() + 90
        check = None
        next_check = 0.0
        active = False
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)
            if check is not None and check.done():
                states = {item.name: item.state for item in check.result().controller}
                active = all(states.get(name) == 'active' for name in
                             ('mycobot_controller', 'gripper_position_controller'))
                check = None
                next_check = time.monotonic() + 0.5
            if (not active and check is None and time.monotonic() >= next_check
                    and self.controllers.service_is_ready()):
                check = self.controllers.call_async(ListControllers.Request())
            if (self.positions is not None and self.get_clock().now().nanoseconds > 0
                    and time.monotonic() - self.received_at < 1
                    and active and self.client.server_is_ready()):
                break
            if time.monotonic() > deadline:
                raise RuntimeError('Gazebo/controller/joint states unavailable after 90 seconds.')
        if not rclpy.ok():
            return
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = JOINTS
        start = JointTrajectoryPoint()
        start.positions = self.positions
        start.velocities = [0.0] * 6
        goal.trajectory.points.append(start)
        for index, pose in enumerate(POSES_DEG, start=1):
            point = JointTrajectoryPoint()
            point.positions = [math.radians(value) for value in pose]
            point.velocities = [0.0] * 6
            point.time_from_start.sec = index * 7
            goal.trajectory.points.append(point)
        future = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10)
        if not future.done() or not future.result().accepted:
            raise RuntimeError('The simulated controller did not accept the trajectory.')
        handle = future.result()
        self.get_logger().info('Mouvement lent de 35 s ; la courbe bleue montre le trajet simule.')
        result = handle.get_result_async()
        try:
            rclpy.spin_until_future_complete(self, result)
        except KeyboardInterrupt:
            cancel = handle.cancel_goal_async()
            rclpy.spin_until_future_complete(self, cancel, timeout_sec=2)
            raise
        if result.done():
            response = result.result().result
            if response.error_code != FollowJointTrajectory.Result.SUCCESSFUL:
                raise RuntimeError(f'Trajectory failed: {response.error_string}')
            self.get_logger().info('Trajectoire terminee. La courbe reste visible dans Gazebo.')


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = TrajectoryPreview()
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
