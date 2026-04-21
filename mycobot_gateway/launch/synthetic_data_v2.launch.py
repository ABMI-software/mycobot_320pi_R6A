#!/usr/bin/env python3
"""Launch Gazebo + Multi-Camera + Domain Randomization + Collector v2.

Enhanced pipeline for collecting high-quality synthetic training data:
1. Gazebo Harmonic with domain-randomized world (varied lighting, clutter)
2. 4 camera viewpoints (front, right, left, top)
3. ros_gz_bridge for joints + all camera topics
4. Enhanced collector with noise injection + multi-view

Usage:
  ros2 launch mycobot_gateway synthetic_data_v2.launch.py
  ros2 launch mycobot_gateway synthetic_data_v2.launch.py \
      num_samples:=5000 multi_view:=true domain_randomize:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
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
    world_path = os.path.join(desc_pkg, 'worlds', 'randomized.sdf')

    # Gazebo needs to resolve package:// URIs for meshes
    # The share parent dir contains 'mycobot_description/' subfolder
    gz_resource_path = os.path.dirname(desc_pkg)  # …/install/mycobot_description/share
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    full_resource_path = f'{gz_resource_path}:{existing}' if existing else gz_resource_path
    set_gz_resource = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', full_resource_path,
    )

    # ---------- launch arguments ----------
    num_samples_arg = DeclareLaunchArgument(
        'num_samples', default_value='5000',
        description='Number of pose samples to collect',
    )
    output_dir_arg = DeclareLaunchArgument(
        'output_dir', default_value='/tmp/mycobot_synth_v2',
        description='Directory for images/ and labels.csv',
    )
    settle_arg = DeclareLaunchArgument(
        'settle_time', default_value='1.5',
        description='Seconds to wait after commanding joints',
    )
    multi_view_arg = DeclareLaunchArgument(
        'multi_view', default_value='true',
        description='Capture from all 4 cameras (true) or front only (false)',
    )
    domain_rand_arg = DeclareLaunchArgument(
        'domain_randomize', default_value='true',
        description='Randomize lighting per sample',
    )
    noise_arg = DeclareLaunchArgument(
        'noise_stddev', default_value='5.0',
        description='Max Gaussian pixel noise sigma (0 = disabled)',
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

    # ---------- Gazebo Harmonic (with domain-randomized world) ----------
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_pkg, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={
            'gz_args': f'-r {world_path}',
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

    # ---------- ros_gz_bridge (joints) ----------
    joint_names = [
        'joint2_to_joint1',
        'joint3_to_joint2',
        'joint4_to_joint3',
        'joint5_to_joint4',
        'joint6_to_joint5',
        'joint6output_to_joint6',
    ]

    bridge_args = [
        '/world/randomized/model/mycobot_320/joint_state'
        '@sensor_msgs/msg/JointState[gz.msgs.Model',
    ]
    for jn in joint_names:
        bridge_args.append(
            f'/model/mycobot_320/joint/{jn}/cmd_pos'
            f'@std_msgs/msg/Float64]gz.msgs.Double'
        )

    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        arguments=bridge_args,
        remappings=[
            ('/world/randomized/model/mycobot_320/joint_state', '/joint_states'),
        ],
    )

    # ---------- ros_gz_image bridges (all 4 cameras) ----------
    camera_topics = [
        '/synth_camera/image',
        '/synth_camera_right/image',
        '/synth_camera_left/image',
        '/synth_camera_top/image',
    ]

    gz_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='gz_image_bridge',
        output='screen',
        arguments=camera_topics,
    )

    # ---------- Collector v2 (delayed start) ----------
    collector = TimerAction(
        period=8.0,  # more time for 4 cameras to initialise
        actions=[
            Node(
                package='mycobot_gateway',
                executable='synthetic_data_collector_v2',
                name='synthetic_data_collector_v2',
                output='screen',
                parameters=[{
                    'num_samples': LaunchConfiguration('num_samples'),
                    'output_dir': LaunchConfiguration('output_dir'),
                    'settle_time': LaunchConfiguration('settle_time'),
                    'multi_view': LaunchConfiguration('multi_view'),
                    'domain_randomize': LaunchConfiguration('domain_randomize'),
                    'noise_stddev': LaunchConfiguration('noise_stddev'),
                    'world_name': 'randomized',
                }],
            ),
        ],
    )

    return LaunchDescription([
        # Environment
        set_gz_resource,
        # Arguments
        num_samples_arg,
        output_dir_arg,
        settle_arg,
        multi_view_arg,
        domain_rand_arg,
        noise_arg,
        # Core
        rsp,
        gz_sim,
        spawn,
        gz_bridge,
        gz_image_bridge,
        collector,
    ])
