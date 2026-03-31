#!/usr/bin/env python3
"""Launch Gazebo Harmonic and spawn the MyCobot 320 Pi.

This launch file:
1. Starts Gazebo Sim (Harmonic) via ros_gz_sim
2. Publishes the robot_description via robot_state_publisher
3. Spawns the Gazebo-compatible URDF into the simulation
4. Bridges /joint_states from Gazebo to ROS2 via ros_gz_bridge
5. Optionally opens RViz2 alongside Gazebo

Usage:
  ros2 launch mycobot_description gazebo_sim.launch.py
  ros2 launch mycobot_description gazebo_sim.launch.py rviz:=true

Requirements:
  - ros_gz_sim, ros_gz_bridge packages (ROS2 Jazzy + Gazebo Harmonic)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory('mycobot_description')

    # ---------- paths ----------
    urdf_path = os.path.join(
        pkg_share, 'urdf', '320_pi', 'mycobot_pro_320_pi_gazebo.urdf'
    )
    rviz_config = os.path.join(pkg_share, 'config', 'mycobot_320_pi.rviz')

    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f'Gazebo URDF not found: {urdf_path}')

    # ---------- launch arguments ----------
    rviz_arg = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Also open RViz2 alongside Gazebo',
    )

    # ---------- robot_state_publisher ----------
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]),
        value_type=str,
    )
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}],
    )

    # ---------- Gazebo Sim (Harmonic) ----------
    gz_sim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('ros_gz_sim'),
                'launch',
                'gz_sim.launch.py',
            )
        ),
        launch_arguments={
            'gz_args': '-r empty.sdf',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ---------- Spawn robot into Gazebo ----------
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'mycobot_320',
            '-z', '0.0',
        ],
    )

    # ---------- ros_gz_bridge: forward /joint_states ----------
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        arguments=[
            '/world/empty/model/mycobot_320/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            ('/world/empty/model/mycobot_320/joint_state', '/joint_states'),
        ],
    )

    # ---------- ros_gz_image bridge (camera) ----------
    gz_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='gz_image_bridge',
        output='screen',
        arguments=['/synth_camera/image'],
    )

    # ---------- RViz (optional) ----------
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    # ---------- assemble ----------
    return LaunchDescription([
        rviz_arg,
        robot_state_publisher,
        gz_sim_launch,
        spawn_entity,
        gz_bridge,
        gz_image_bridge,
        rviz_node,
    ])
