#!/usr/bin/env python3
"""Measured 62.2 x 44.9 cm tabletop, four ArUco tags and physical grasping.

By default, randomize the cube and bin and run a four-camera visual pick-and-place.
Pass demo:=false for the scene only, randomize:=false for the original position.
Pass robot_appearance:=realistic for the grey base and satin off-white shells.
"""

import os
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            OpaqueFunction, RegisterEventHandler, LogInfo)
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from mycobot_gateway.vision.sim_multicam_geometry import sample_scene_xy


def launch_scene(context):
    desc_pkg = get_package_share_directory('mycobot_description')
    gateway_pkg = get_package_share_directory('mycobot_gateway')
    world_file = os.path.join(desc_pkg, 'worlds', 'real_table.sdf')
    bin_xy = [.22, .10]
    actions = []
    if LaunchConfiguration('randomize').perform(context).lower() == 'true':
        seed_text = LaunchConfiguration('seed').perform(context)
        seed = int(seed_text)
        rng = np.random.default_rng(None if seed < 0 else seed)
        xy, sampled_bin = sample_scene_xy(rng)
        bin_xy = list(map(float, sampled_bin))
        tree = ET.parse(world_file)
        tree.find("world/model[@name='red_cube']/pose").text = (
            f'{xy[0]:.8f} {xy[1]:.8f} 0.020 0 0 0')
        tree.find("world/model[@name='red_bin']/pose").text = (
            f'{bin_xy[0]:.8f} {bin_xy[1]:.8f} 0 0 0 0')
        fd, world_file = tempfile.mkstemp(prefix='mycobot_real_table_', suffix='.sdf')
        os.close(fd)
        tree.write(world_file, encoding='utf-8', xml_declaration=True)

        def cleanup(_context):
            if os.path.exists(world_file):
                os.unlink(world_file)
            return []

        actions.extend([
            RegisterEventHandler(OnShutdown(on_shutdown=[OpaqueFunction(function=cleanup)])),
            LogInfo(msg=f'Cube and bin randomized on reachable table area (seed={seed}); '
                        'target coordinates will be measured by the cameras.'),
        ])
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(gateway_pkg, 'launch', 'sim_grasp.launch.py')),
        launch_arguments={
            'world_name': 'real_table', 'world_file': world_file,
            'headless': LaunchConfiguration('headless'),
            'bridge_camera': LaunchConfiguration('bridge_camera'),
            'bridge_multicam': 'true',
            'robot_appearance': LaunchConfiguration('robot_appearance'),
        }.items()))
    actions.extend([
        Node(package='mycobot_gateway', executable='sim_multicam_detector',
             output='screen', parameters=[{'use_sim_time': True, 'bin_xy': bin_xy}]),
        Node(package='mycobot_gateway', executable='sim_scene_control',
             output='screen', parameters=[{
                 'use_sim_time': True, 'world_name': 'real_table', 'only': 'red_cube',
                 'bin_xy.red_cube': bin_xy, 'pose_source': 'vision', 'max_attempts': 3,
                 'autostart': ParameterValue(LaunchConfiguration('demo'), value_type=bool),
             }]),
    ])
    if (LaunchConfiguration('headless').perform(context).lower() != 'true'
            and LaunchConfiguration('panel').perform(context).lower() == 'true'):
        actions.append(Node(package='mycobot_gateway', executable='sim_scene_panel',
                            output='screen'))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument(
            'bridge_camera', default_value='true',
            description='Publish overhead camera images and calibration in ROS'),
        DeclareLaunchArgument(
            'robot_appearance', default_value='original',
            choices=['original', 'realistic'],
            description='Robot visual materials; kinematics and collisions are unchanged'),
        DeclareLaunchArgument(
            'demo', default_value='true',
            description='Run one visual physical grasp of the red cube into the red bin'),
        DeclareLaunchArgument('randomize', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('seed', default_value='-1',
                              description='Random seed; -1 gives a new position each launch'),
        DeclareLaunchArgument('panel', default_value='true', choices=['true', 'false'],
                              description='Show Randomize / Pick buttons when GUI is enabled'),
        OpaqueFunction(function=launch_scene),
    ])
