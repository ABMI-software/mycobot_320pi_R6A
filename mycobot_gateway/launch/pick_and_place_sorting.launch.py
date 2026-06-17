#!/usr/bin/env python3
"""Launch the multi-object color-sorting pick-and-place demo in Gazebo.

Pipeline:
  1. Gazebo Harmonic with pick_and_place_sorting.sdf
     (4 colored objects + 4 colored bins on a 1.0×0.6 m table)
  2. MyCobot 320 Pi spawn + ros_gz_bridge for joints + cmd_pos topics
  3. ros_gz_image bridges for the four onboard cameras
  4. color_object_detector — HSV segmentation on the top-down camera
  5. sorting_orchestrator — pick each object, drop into matching bin

Usage:
  ros2 launch mycobot_gateway pick_and_place_sorting.launch.py
  ros2 launch mycobot_gateway pick_and_place_sorting.launch.py use_detector:=false
  ros2 launch mycobot_gateway pick_and_place_sorting.launch.py process_order:=blue,green
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


WORLD_NAME = 'pick_and_place_sorting'


def generate_launch_description():
    desc_pkg = get_package_share_directory('mycobot_description')
    gz_pkg = get_package_share_directory('ros_gz_sim')

    urdf_path = os.path.join(
        desc_pkg, 'urdf', '320_pi', 'mycobot_pro_320_pi_gazebo.urdf',
    )
    world_path = os.path.join(desc_pkg, 'worlds', f'{WORLD_NAME}.sdf')

    gz_resource_path = os.path.dirname(desc_pkg)
    existing = os.environ.get('GZ_SIM_RESOURCE_PATH', '')
    full_resource_path = (
        f'{gz_resource_path}:{existing}' if existing else gz_resource_path
    )
    set_gz_resource = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH', full_resource_path,
    )

    use_detector_arg = DeclareLaunchArgument(
        'use_detector', default_value='true',
        description='Use HSV color detector (false = SDF-known positions)',
    )
    process_order_arg = DeclareLaunchArgument(
        'process_order', default_value='red,blue,green,yellow',
        description='Comma-separated order in which colors are sorted',
    )
    rviz_arg = DeclareLaunchArgument(
        'use_rviz', default_value='false',
    )

    robot_description = ParameterValue(
        Command(['xacro ', urdf_path]), value_type=str,
    )
    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_pkg, 'launch', 'gz_sim.launch.py'),
        ),
        launch_arguments={
            'gz_args': f'-r {world_path}',
            'on_exit_shutdown': 'true',
        }.items(),
    )

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

    joint_names = [
        'joint2_to_joint1',
        'joint3_to_joint2',
        'joint4_to_joint3',
        'joint5_to_joint4',
        'joint6_to_joint5',
        'joint6output_to_joint6',
    ]

    bridge_args = [
        f'/world/{WORLD_NAME}/model/mycobot_320/joint_state'
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
        remappings=[(
            f'/world/{WORLD_NAME}/model/mycobot_320/joint_state',
            '/joint_states',
        )],
        output='screen',
    )

    camera_topics = [
        '/synth_camera/image',
        '/synth_camera_right/image',
        '/synth_camera_left/image',
        '/synth_camera_top/image',
    ]
    camera_bridges = [
        Node(
            package='ros_gz_image',
            executable='image_bridge',
            arguments=[t],
            output='screen',
        )
        for t in camera_topics
    ]

    detector_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='mycobot_gateway',
                executable='color_object_detector',
                name='color_object_detector',
                parameters=[{
                    'camera_topic': '/synth_camera_top/image',
                    'image_width': 640,
                    'image_height': 480,
                    'camera_height': 1.2,
                    'camera_hfov': 1.047,
                    'camera_world_x': 0.0,
                    'camera_world_y': 0.0,
                    'image_u_to_world_axis_name': 'world_y',
                    'image_v_to_world_axis_name': 'world_x',
                    'flip_u': True,
                    'flip_v': True,
                    'publish_rate': 2.0,
                    'debug_publish': True,
                }],
                output='screen',
            ),
        ],
    )

    orchestrator_node = TimerAction(
        period=10.0,
        actions=[
            Node(
                package='mycobot_gateway',
                executable='sorting_orchestrator',
                name='sorting_orchestrator',
                parameters=[{
                    'world_name': WORLD_NAME,
                    'use_detector': LaunchConfiguration('use_detector'),
                    'process_order': LaunchConfiguration('process_order'),
                    'approach_height': 0.10,
                    'grasp_z': 0.04,
                    'release_z': 0.05,
                    'grasp_settle_time': 1.0,
                    'step_duration': 0.15,
                    'settle_time': 1.5,
                    'detection_timeout': 15.0,
                    'interpolation_steps': 12,
                    'startup_delay': 5.0,
                }],
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        set_gz_resource,
        use_detector_arg,
        process_order_arg,
        rviz_arg,
        rsp,
        gz_sim,
        spawn,
        gz_bridge,
        *camera_bridges,
        detector_node,
        orchestrator_node,
    ])
