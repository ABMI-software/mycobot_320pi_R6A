#!/usr/bin/env python3
"""Sorting scene for YOLO validation: four pieces, four bins, the DREAM 50K cameras.

The board, markers and robot are those of real_table; the four cameras are at
the poses that rendered the 50K DREAM dataset (camera_layout:=dream50k); robot
masses are the original ones. Pieces and bins are randomized on every launch
(vision/tri_scene.py); the seed is logged and seed:=N replays a scene.
Protocol: docs/PROTOCOLE_YOLO_GAZEBO.md.

    ros2 launch mycobot_gateway tri_yolo.launch.py
    ros2 launch mycobot_gateway tri_yolo.launch.py seed:=7 headless:=true
"""

import math
import os
import tempfile

import numpy as np

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            LogInfo, OpaqueFunction, RegisterEventHandler, TimerAction)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from mycobot_gateway.vision import tri_scene
from mycobot_gateway.vision.sim_multicam_geometry import load_cameras

WORLD_NAME = 'tri_yolo'
# Front, right, left, top: the 2x2 order of the YOLO vs ground-truth mosaic.
CAMERAS = ('synth_camera', 'synth_camera_right', 'synth_camera_left', 'synth_camera_top')
ARM_JOINTS = ('joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
              'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6')


def launch_scene(context):
    desc_pkg = get_package_share_directory('mycobot_description')
    gateway_pkg = get_package_share_directory('mycobot_gateway')
    models = os.path.join(desc_pkg, 'models')
    source = os.path.join(desc_pkg, 'worlds', 'real_table.sdf')
    urdf = os.path.join(desc_pkg, 'urdf', '320_pi', 'mycobot_pro_320_pi_gazebo.urdf')

    seed = int(LaunchConfiguration('seed').perform(context))
    if seed < 0:
        seed = int(np.random.default_rng().integers(2**31))
    top = load_cameras(urdf, {'camera_layout': 'dream50k'})['synth_camera_top']
    layout = tri_scene.sample_tri_scene(
        np.random.default_rng(seed), tri_scene.load_footprints(models),
        tri_scene.load_board(source, models), top,
        piece_reach=float(LaunchConfiguration('piece_reach').perform(context)) or None)

    fd, world_file = tempfile.mkstemp(prefix='mycobot_tri_yolo_', suffix='.sdf')
    with os.fdopen(fd, 'w') as f:
        f.write(tri_scene.build_world(source, layout, WORLD_NAME))

    def cleanup(_context):
        if os.path.exists(world_file):
            os.unlink(world_file)
        return []

    placed = ', '.join(f'{n} ({x * 1000:.0f}, {y * 1000:.0f})' for n, (x, y) in layout.items())
    q = [math.radians(a) for a in tri_scene.OBSERVATION_Q_DEG]
    trajectory = ('{joint_names: [' + ', '.join(ARM_JOINTS) + '], points: [{positions: ['
                  + ', '.join(f'{v:.6f}' for v in q) + '], time_from_start: {sec: 3}}]}')
    return [
        RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
        LogInfo(msg=f'tri_yolo seed={seed} — {placed} mm'),
        LogInfo(msg='beyond vertical-tool reach (0.28 m): '
                    + (', '.join(tri_scene.beyond_reach(layout)) or 'none')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(gateway_pkg, 'launch', 'sim_grasp.launch.py')),
            launch_arguments={
                'world_name': WORLD_NAME, 'world_file': world_file,
                'camera_layout': 'dream50k', 'bridge_multicam': 'true',
                'headless': LaunchConfiguration('headless'),
                'robot_appearance': LaunchConfiguration('robot_appearance'),
                'gui_config': os.path.join(
                    desc_pkg, 'config',
                    'tri_yolo_gui.config' if LaunchConfiguration('panel').perform(context) == 'true'
                    else 'trajectory_gui.config'),
            }.items()),
        # Ground truth for VALIDATION only: under /validation/gt, never read by
        # perception or planning (protocol invariant I4).
        Node(package='mycobot_gateway', executable='gazebo_ground_truth', output='screen',
             parameters=[{'use_sim_time': True, 'world_name': WORLD_NAME}]),
        # Perception: yolo26 on the four DREAM cameras, each localized on its
        # own (no fusion, protocol step 7).
        Node(package='mycobot_gateway', executable='yolo_gazebo_node', output='screen',
             condition=IfCondition(LaunchConfiguration('yolo')),
             parameters=[{'use_sim_time': True, 'cameras': list(CAMERAS)}]),
        Node(package='mycobot_gateway', executable='yolo_localizer', output='screen',
             condition=IfCondition(LaunchConfiguration('yolo')),
             parameters=[{'use_sim_time': True, 'cameras': list(CAMERAS)}]),
        # Validation view in a Gazebo GUI panel: the YOLO boxes, classes and
        # confidences, plus the XY error against the ground truth per piece.
        Node(package='mycobot_gateway', executable='yolo_gt_overlay', output='screen',
             condition=IfCondition(LaunchConfiguration('yolo')),
             parameters=[{'use_sim_time': True, 'cameras': list(CAMERAS)}]),
        Node(package='ros_gz_bridge', executable='parameter_bridge', output='screen',
             condition=IfCondition(LaunchConfiguration('yolo')),
             arguments=['/validation/yolo_gt/image@sensor_msgs/msg/Image]gz.msgs.Image']),
        TimerAction(period=12.0, condition=IfCondition(LaunchConfiguration('observe')), actions=[
            LogInfo(msg=f'observation pose {tri_scene.OBSERVATION_Q_DEG} deg'),
            ExecuteProcess(cmd=['ros2', 'topic', 'pub', '--once', '-w', '1',
                                '/mycobot_controller/joint_trajectory',
                                'trajectory_msgs/msg/JointTrajectory', trajectory],
                           output='screen')]),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('seed', default_value='-1',
                              description='Scene seed; -1 draws one and logs it'),
        DeclareLaunchArgument('piece_reach', default_value='0.0',
                              description='Max base distance of the four objects (m); 0 = anywhere'),
        DeclareLaunchArgument('observe', default_value='true',
                              description='Lean the arm back so the top camera sees the board'),
        DeclareLaunchArgument('yolo', default_value='true', choices=['true', 'false'],
                              description='Run yolo26 on the four cameras'),
        DeclareLaunchArgument('panel', default_value='true', choices=['true', 'false'],
                              description='YOLO vs GT image panel in the Gazebo GUI'),
        DeclareLaunchArgument('robot_appearance', default_value='original',
                              choices=['original', 'realistic']),
        OpaqueFunction(function=launch_scene),
    ])
