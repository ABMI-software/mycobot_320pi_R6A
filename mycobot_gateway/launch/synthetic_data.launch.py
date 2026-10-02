#!/usr/bin/env python3
"""Launch Gazebo + 4 cameras + Synthetic Data Collector.

Pipeline:
  1. Gazebo Harmonic with the 4-camera URDF (calibrated Arducam intrinsics)
  2. ros_gz_bridge  — /joint_states + 6 joint position cmd topics
  3. ros_gz_image   — 4 camera image bridges
  4. synthetic_data_collector node (delayed 5 s for Gazebo to fully start)

Camera layout (90° symmetric ring, z=0.4 m):
  cam_0 front  (0.8,  0.0, 0.4) — cam_0 intrinsics  fx=525.671
  cam_1 left   (0.0,  0.8, 0.4) — cam_3 intrinsics  fx=496.308
  cam_2 back  (-0.7,  0.0, 0.4) — cam_0 intrinsics  fx=525.671
  cam_3 right  (0.0, -0.8, 0.4) — cam_3 intrinsics  fx=496.308

Default: 12 500 poses × 4 cameras = 50 000 images  (~3.5 h at 1 s/pose)

Usage
-----
  # Full 50 k run
  ros2 launch mycobot_gateway synthetic_data.launch.py

  # Custom
  ros2 launch mycobot_gateway synthetic_data.launch.py \\
      num_samples:=12500 output_dir:=/mnt/data/synth_50k settle_time:=1.0

  # Preview only (use synthetic_data_preview.launch.py for 5-sample visual check)
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_pkg = get_package_share_directory('mycobot_description')
    gw_pkg   = get_package_share_directory('mycobot_gateway')
    gz_pkg   = get_package_share_directory('ros_gz_sim')

    urdf_path = os.path.join(
        desc_pkg, 'urdf', '320_pi', 'mycobot_pro_320_pi_gazebo.urdf',
    )
    world_path = os.path.join(gw_pkg, 'worlds', 'lab_room.sdf')

    # ---------- launch arguments ----------
    num_samples_arg = DeclareLaunchArgument(
        'num_samples', default_value='12500',
        description='Number of robot poses (total images = poses × 4 cameras)',
    )
    output_dir_arg = DeclareLaunchArgument(
        'output_dir', default_value='/tmp/mycobot_synth_dataset',
        description='Root output directory for images/ and labels.csv',
    )
    settle_arg = DeclareLaunchArgument(
        'settle_time', default_value='1.0',
        description='Seconds to wait after commanding joints before capture',
    )

    # ---------- robot_state_publisher ----------
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': ParameterValue(
                Command(['xacro ', urdf_path]), value_type=str,
            ),
        }],
        output='screen',
    )

    # ---------- Gazebo Harmonic ----------
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_pkg, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={
            'gz_args': ['-r -s --headless-rendering ', world_path],
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ---------- Spawn robot ----------
    spawn = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description', '-name', 'mycobot_320', '-z', '0.0'],
    )

    # ---------- ros_gz_bridge (joint states + joint commands) ----------
    joint_names = [
        'joint2_to_joint1',
        'joint3_to_joint2',
        'joint4_to_joint3',
        'joint5_to_joint4',
        'joint6_to_joint5',
        'joint6output_to_joint6',
    ]
    bridge_args = [
        '/world/empty/model/mycobot_320/joint_state'
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
            ('/world/empty/model/mycobot_320/joint_state', '/joint_states'),
        ],
    )

    # ---------- ros_gz_image bridges — one per camera ----------
    camera_topics = [
        '/synth_camera/image',
        '/synth_camera_1/image',
        '/synth_camera_2/image',
        '/synth_camera_3/image',
    ]
    gz_image_bridge = Node(
        package='ros_gz_image',
        executable='image_bridge',
        name='gz_image_bridge',
        output='screen',
        arguments=camera_topics,
    )

    # ---------- Synthetic Data Collector (delayed) ----------
    collector = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='mycobot_gateway',
                executable='synthetic_data_collector',
                name='synthetic_data_collector',
                output='screen',
                parameters=[{
                    'num_samples':  LaunchConfiguration('num_samples'),
                    'output_dir':   LaunchConfiguration('output_dir'),
                    'settle_time':  LaunchConfiguration('settle_time'),
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
