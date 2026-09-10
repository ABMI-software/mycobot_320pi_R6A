#!/usr/bin/env python3
"""Measured 62.2 x 44.9 cm tabletop, four ArUco tags and physical grasping.

By default this starts the scene and controllers. To run one physical
pick-and-place cycle with the red cube, pass demo:=true.
Pass robot_appearance:=realistic for the grey base and satin off-white shells.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    gateway_pkg = get_package_share_directory('mycobot_gateway')
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
            'demo', default_value='false',
            description='Run one physical grasp of the red cube into the red bin'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gateway_pkg, 'launch', 'sim_grasp.launch.py')),
            launch_arguments={
                'world_name': 'real_table',
                'headless': LaunchConfiguration('headless'),
                'bridge_camera': LaunchConfiguration('bridge_camera'),
                'robot_appearance': LaunchConfiguration('robot_appearance'),
            }.items(),
        ),
        # The node waits for active controllers and joint states; no fixed
        # startup delay is needed, even when Gazebo takes longer to initialize.
        Node(
            package='mycobot_gateway', executable='sim_sorting_grasp',
            output='screen',
            parameters=[{
                'use_sim_time': True,
                'world_name': 'real_table',
                'only': 'red_cube',
                'bin_xy.red_cube': [0.22, 0.10],
            }],
            condition=IfCondition(LaunchConfiguration('demo')),
        ),
    ])
