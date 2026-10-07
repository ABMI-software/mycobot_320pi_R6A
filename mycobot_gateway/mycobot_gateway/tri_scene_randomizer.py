#!/usr/bin/env python3
"""Redraw the sorting scene without restarting Gazebo (protocol, step 2).

Service /tri_scene/randomize (std_srvs/Trigger): the arm goes back to the
observation pose, then the four pieces and four bins move in a single
set_pose_vector request to a layout drawn by tri_scene.sample_tri_scene, the
same rules as at launch. Draw k uses the generator [seed, k]: a scene is
replayed from the launch seed and the draw number, both appended to run.yaml.

Use it between two sorts only; it does not stop a sort in progress.
"""
import math
import os
import re
import subprocess
import time

import numpy as np
import rclpy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from ament_index_python.packages import get_package_share_directory

from mycobot_gateway.vision import tri_scene
from mycobot_gateway.vision.sim_multicam_geometry import load_cameras

ARM_JOINTS = ('joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
              'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6')
MOVE_S = 3
ARRIVED_DEG = 1.0
# Simulated seconds: the real-time factor is far below 1 right after launch.
ARRIVAL_TIMEOUT_S = 15.0
WALL_TIMEOUT_S = 180.0


class TriSceneRandomizer(Node):
    def __init__(self):
        super().__init__('tri_scene_randomizer')
        self.world = self.declare_parameter('world_name', 'tri_yolo').value
        self.seed = self.declare_parameter('seed', 0).value
        self.piece_reach = self.declare_parameter('piece_reach', 0.0).value or None
        self.run_yaml = self.declare_parameter('run_yaml', '').value
        desc = get_package_share_directory('mycobot_description')
        models = os.path.join(desc, 'models')
        urdf = os.path.join(desc, 'urdf', '320_pi', 'mycobot_pro_320_pi_gazebo.urdf')
        self.footprints = tri_scene.load_footprints(models)
        self.board = tri_scene.load_board(os.path.join(desc, 'worlds', 'real_table.sdf'), models)
        self.top = load_cameras(urdf, {'camera_layout': 'dream50k'})['synth_camera_top']
        self.draw = 0
        self.q_deg = None
        self.trajectory = self.create_publisher(JointTrajectory, '/mycobot_controller/joint_trajectory', 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.create_service(Trigger, '/tri_scene/randomize', self.randomize,
                            callback_group=ReentrantCallbackGroup())

    def on_joints(self, msg):
        positions = dict(zip(msg.name, msg.position))
        if all(j in positions for j in ARM_JOINTS):
            self.q_deg = np.degrees([positions[j] for j in ARM_JOINTS])

    def sim_now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def go_to_observation(self):
        target = np.array(tri_scene.OBSERVATION_Q_DEG)
        msg = JointTrajectory(joint_names=list(ARM_JOINTS))
        msg.points = [JointTrajectoryPoint(positions=[math.radians(a) for a in target])]
        msg.points[0].time_from_start.sec = MOVE_S
        self.trajectory.publish(msg)
        start, wall_deadline = self.sim_now(), time.monotonic() + WALL_TIMEOUT_S
        while self.sim_now() - start < ARRIVAL_TIMEOUT_S and time.monotonic() < wall_deadline:
            if self.q_deg is not None and np.max(np.abs(self.q_deg - target)) < ARRIVED_DEG:
                return True
            time.sleep(0.1)
        return False

    def randomize(self, _request, response):
        self.draw += 1
        layout = tri_scene.sample_tri_scene(
            np.random.default_rng([self.seed, self.draw]), self.footprints, self.board,
            self.top, piece_reach=self.piece_reach)
        if not self.go_to_observation():
            response.success = False
            response.message = 'Le bras n a pas rejoint la pose d observation : scène inchangée.'
            return response
        request = ' '.join(
            f'pose {{ name: "{name}" position {{ x: {x:.9f} y: {y:.9f} z: 0 }} orientation {{ w: 1 }} }}'
            for name, (x, y) in layout.items())
        result = subprocess.run([
            'gz', 'service', '-s', f'/world/{self.world}/set_pose_vector',
            '--reqtype', 'gz.msgs.Pose_V', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '4000', '--req', request,
        ], capture_output=True, text=True, timeout=8)
        if result.returncode or not re.search(r'data:\s*true', result.stdout):
            response.success = False
            response.message = 'Gazebo n a pas confirmé le repositionnement.'
            return response
        if self.run_yaml:
            with open(self.run_yaml, 'a') as f:
                f.write(f'randomize_{self.draw}:\n  rng: [{self.seed}, {self.draw}]\n'
                        f'  sim_time_s: {self.sim_now():.3f}\n  layout_xy_m:\n')
                f.writelines(f'    {n}: [{x:.4f}, {y:.4f}]\n' for n, (x, y) in layout.items())
        beyond = tri_scene.beyond_reach(layout)
        response.success = True
        response.message = (f'Tirage {self.draw} (graine [{self.seed}, {self.draw}]) en place.'
                            + (f' Hors portée outil vertical : {", ".join(beyond)}.' if beyond else ''))
        self.get_logger().info(response.message + ' ' + ', '.join(
            f'{n} ({x * 1000:.0f}, {y * 1000:.0f})' for n, (x, y) in layout.items()) + ' mm')
        return response


def main(args=None):
    rclpy.init(args=args)
    node = TriSceneRandomizer()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
