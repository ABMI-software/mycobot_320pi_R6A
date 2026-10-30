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

    # No-gripper variant: gripper links/joints removed so joint 6 / the flange
    # is fully visible, and the 4 cameras are pulled back + widened so the whole
    # arm stays in frame. To restore the gripper, swap back to
    # 'mycobot_pro_320_pi_gazebo.urdf'.
    urdf_path = os.path.join(
        desc_pkg, 'urdf', '320_pi', 'mycobot_pro_320_pi_gazebo_nogripper.urdf',
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
        'noise_stddev', default_value='0.0',
        description='Max Gaussian pixel noise sigma (0 = disabled, clean images)',
    )
    grasp_fraction_arg = DeclareLaunchArgument(
        'grasp_fraction', default_value='0.0',
        description='Fraction of grasp-approach poses (0.0 = pure uniform angle '
                    'distribution, no spikes; 0.5 = half grasp poses for pick-and-place)',
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

    # ---------- ros_gz_bridge (/clock + gz joint_state) ----------
    # /clock is CRITICAL: gz_ros2_control runs with use_sim_time and will not
    # tick its controllers without it (this was the missing piece — the robot
    # never moved). The gz-side joint_state is mapped to /gz/joint_states to
    # avoid colliding with the joint_state_broadcaster's /joint_states.
    gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='gz_bridge',
        output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/world/randomized/model/mycobot_320/joint_state'
            '@sensor_msgs/msg/JointState[gz.msgs.Model',
        ],
        remappings=[
            ('/world/randomized/model/mycobot_320/joint_state', '/gz/joint_states'),
        ],
    )

    # ---------- ros2_control controller spawners ----------
    # The robot is actuated by a JointTrajectoryController ('mycobot_controller')
    # commanded on /mycobot_controller/joint_trajectory. These MUST be spawned
    # or the joints stay frozen at 0.
    controller_cfg = os.path.join(desc_pkg, 'config', 'controller.yaml')
    joint_state_broadcaster = Node(
        package='controller_manager', executable='spawner',
        arguments=['joint_state_broadcaster', '--controller-manager', '/controller_manager'],
        output='screen',
    )
    mycobot_controller_spawner = Node(
        package='controller_manager', executable='spawner',
        arguments=['mycobot_controller', '--controller-manager', '/controller_manager',
                   '--param-file', controller_cfg],
        output='screen',
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
                executable='synthetic_data_collector_v3',
                name='synthetic_data_collector_v3',
                output='screen',
                parameters=[{
                    'num_samples': LaunchConfiguration('num_samples'),
                    'output_dir': LaunchConfiguration('output_dir'),
                    'settle_time': LaunchConfiguration('settle_time'),
                    'multi_view': LaunchConfiguration('multi_view'),
                    'domain_randomize': LaunchConfiguration('domain_randomize'),
                    'noise_stddev': LaunchConfiguration('noise_stddev'),
                    'grasp_fraction': LaunchConfiguration('grasp_fraction'),
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
        grasp_fraction_arg,
        # Core
        rsp,
        gz_sim,
        spawn,
        gz_bridge,
        gz_image_bridge,
        # Controllers come up after Gazebo has spawned the robot
        TimerAction(period=4.0, actions=[joint_state_broadcaster]),
        TimerAction(period=5.0, actions=[mycobot_controller_spawner]),
        collector,
    ])
