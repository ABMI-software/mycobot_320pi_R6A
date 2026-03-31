#!/usr/bin/env python3
"""Launch Gazebo + Camera + Synthetic Data Collector.

Starts the full pipeline:
1. Gazebo Harmonic with the camera-equipped URDF
2. ros_gz_bridge for /joint_states AND camera image
3. ros_gz_image bridge for the camera
4. The synthetic_data_collector node

Usage:
  ros2 launch mycobot_gateway synthetic_data.launch.py
  ros2 launch mycobot_gateway synthetic_data.launch.py num_samples:=500 output_dir:=/tmp/my_dataset
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_pkg = get_package_share_directory('mycobot_description')
    gz_pkg = get_package_share_directory('ros_gz_sim')

    urdf_path = os.path.join(
        desc_pkg, 'urdf', '320_pi', 'mycobot_pro_320_pi_gazebo.urdf',
    )

    # ---------- launch arguments ----------
    num_samples_arg = DeclareLaunchArgument(
        'num_samples', default_value='1000',
        description='Number of (image, angles) pairs to collect',
    )
    output_dir_arg = DeclareLaunchArgument(
        'output_dir', default_value='/tmp/mycobot_synth_dataset',
        description='Directory for images/ and labels.csv',
    )
    settle_arg = DeclareLaunchArgument(
        'settle_time', default_value='1.5',
        description='Seconds to wait after commanding joints before capture',
    )

    # ---------- robot_state_publisher ----------
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]), value_type=str,
    )
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    # ---------- Gazebo Harmonic ----------
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_pkg, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={
            'gz_args': '-r empty.sdf',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ---------- Spawn robot ----------
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-name', 'mycobot_320',
            '-z', '0.0',
        ],
    )

    # ---------- ros_gz_bridge (joint states + joint commands) ----------
    # Joint names matching the URDF
    joint_names = [
        'joint2_to_joint1',
        'joint3_to_joint2',
        'joint4_to_joint3',
        'joint5_to_joint4',
        'joint6_to_joint5',
        'joint6output_to_joint6',
    ]
    # Bridge joint state (Gz → ROS)
    bridge_args = [
        '/world/empty/model/mycobot_320/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model',
    ]
    # Bridge per-joint position commands (ROS → Gz)
    for jn in joint_names:
        bridge_args.append(
            f'/model/mycobot_320/joint/{jn}/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double'
        )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        arguments=bridge_args,
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

    # ---------- Synthetic Data Collector (delayed start) ----------
    collector = TimerAction(
        period=5.0,  # let Gazebo + camera fully start
        actions=[
            Node(
                package='mycobot_gateway',
                executable='synthetic_data_collector',
                name='synthetic_data_collector',
                output='screen',
                parameters=[{
                    'num_samples': LaunchConfiguration('num_samples'),
                    'output_dir': LaunchConfiguration('output_dir'),
                    'settle_time': LaunchConfiguration('settle_time'),
                    'image_topic': '/synth_camera/image',
                }],
            ),
        ],
    )

    return LaunchDescription([
        num_samples_arg,
        output_dir_arg,
        settle_arg,
        rsp,
        gz_sim,
        spawn,
        gz_bridge,
        gz_image_bridge,
        collector,
    ])
