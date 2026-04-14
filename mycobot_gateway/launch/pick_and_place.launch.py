#!/usr/bin/env python3
"""Launch Pick-and-Place demo in Gazebo with DREAM vision.

Full pipeline:
1. Gazebo Harmonic with pick_and_place.sdf world (target cube + drop zone)
2. MyCobot 320 Pi robot with 4 cameras
3. ros_gz_bridge for joints + camera images
4. DREAM inference node (keypoint detection + PnP)
5. Pick-and-place orchestrator (IK planning + execution)

Usage:
  ros2 launch mycobot_gateway pick_and_place.launch.py
  ros2 launch mycobot_gateway pick_and_place.launch.py use_vision:=false
  ros2 launch mycobot_gateway pick_and_place.launch.py target_x:=0.20 target_y:=0.0
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
    world_path = os.path.join(desc_pkg, 'worlds', 'pick_and_place.sdf')

    # DREAM model paths
    dream_dir = os.path.join(
        os.path.dirname(os.path.dirname(os.path.dirname(
            os.path.abspath(desc_pkg)
        ))),
        'training', 'dream'
    )
    # Fallback to known absolute path
    if not os.path.isdir(dream_dir):
        dream_dir = '/home/genji/ros_jazzy/src/mycobot_R6A/training/dream'

    model_path = os.path.join(
        dream_dir, 'checkpoints_dream', 'vgg_augmented_e25', 'best_network.pth'
    )
    config_path = os.path.join(
        dream_dir, 'checkpoints_dream', 'vgg_augmented_e25', 'best_network.yaml'
    )

    # ── Launch arguments ──
    use_vision_arg = DeclareLaunchArgument(
        'use_vision', default_value='true',
        description='Use DREAM vision feedback (false = open-loop IK only)',
    )
    target_x_arg = DeclareLaunchArgument('target_x', default_value='0.25')
    target_y_arg = DeclareLaunchArgument('target_y', default_value='0.10')
    target_z_arg = DeclareLaunchArgument('target_z', default_value='0.04')
    place_x_arg = DeclareLaunchArgument('place_x', default_value='-0.20')
    place_y_arg = DeclareLaunchArgument('place_y', default_value='0.15')
    place_z_arg = DeclareLaunchArgument('place_z', default_value='0.04')
    rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='false',
        description='Launch RViz for visualization',
    )

    # ── Robot State Publisher ──
    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]), value_type=str,
    )
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    # ── Gazebo Harmonic ──
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_pkg, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={
            'gz_args': f'-r {world_path}',
            'on_exit_shutdown': 'true',
        }.items(),
    )

    # ── Spawn robot ──
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

    # ── ros_gz_bridge: joints + commands ──
    joint_names = [
        'joint2_to_joint1',
        'joint3_to_joint2',
        'joint4_to_joint3',
        'joint5_to_joint4',
        'joint6_to_joint5',
        'joint6output_to_joint6',
    ]

    bridge_args = [
        '/world/pick_and_place/model/mycobot_320/joint_state'
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
        arguments=bridge_args,
        remappings=[
            (
                '/world/pick_and_place/model/mycobot_320/joint_state',
                '/joint_states',
            ),
        ],
        output='screen',
    )

    # ── ros_gz_image: camera bridges ──
    camera_bridges = []
    camera_topics = [
        '/synth_camera/image',
        '/synth_camera_right/image',
        '/synth_camera_left/image',
        '/synth_camera_top/image',
    ]
    for cam_topic in camera_topics:
        camera_bridges.append(
            Node(
                package='ros_gz_image',
                executable='image_bridge',
                arguments=[cam_topic],
                output='screen',
            )
        )

    # ── DREAM Inference Node (delayed 8s for camera init) ──
    dream_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='mycobot_gateway',
                executable='dream_inference',
                name='dream_inference_node',
                parameters=[{
                    'model_path': model_path,
                    'config_path': config_path,
                    'camera_topic': '/synth_camera/image',
                    'publish_rate': 5.0,
                    'visualize': True,
                    'min_keypoints_pnp': 4,
                }],
                output='screen',
            ),
        ],
    )

    # ── Pick-and-Place Orchestrator (delayed 12s) ──
    pickplace_node = TimerAction(
        period=12.0,
        actions=[
            Node(
                package='mycobot_gateway',
                executable='pick_and_place',
                name='pick_and_place_node',
                parameters=[{
                    'target_x': LaunchConfiguration('target_x'),
                    'target_y': LaunchConfiguration('target_y'),
                    'target_z': LaunchConfiguration('target_z'),
                    'place_x': LaunchConfiguration('place_x'),
                    'place_y': LaunchConfiguration('place_y'),
                    'place_z': LaunchConfiguration('place_z'),
                    'use_vision_feedback': LaunchConfiguration('use_vision'),
                    'approach_height': 0.10,
                    'grasp_settle_time': 1.5,
                    'move_step_time': 1.5,
                    'interpolation_steps': 10,
                }],
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        # Arguments
        use_vision_arg,
        target_x_arg, target_y_arg, target_z_arg,
        place_x_arg, place_y_arg, place_z_arg,
        rviz_arg,
        # Core
        rsp,
        gz_sim,
        spawn,
        gz_bridge,
        # Camera bridges
        *camera_bridges,
        # AI nodes (delayed)
        dream_node,
        pickplace_node,
    ])
