#!/usr/bin/env python3
"""Gazebo bench for PHYSICAL grasping — no teleport, no cmd_pos.

Differences with pick_and_place_sorting.launch.py, which this replaces for
grasp work:

  * the arm is driven by `mycobot_controller` (JTC) and the gripper by
    `gripper_position_controller`, both through gz_ros2_control. The
    `/model/mycobot_320/joint/<j>/cmd_pos` bridge is gone with the
    `gz-sim-joint-position-controller-system` plugin it fed (see the URDF
    comment at the end of mycobot_pro_320_pi_gazebo.urdf);
  * `/clock` is bridged — without it ros2_control's `use_sim_time: true`
    never advances and the controllers never leave `unconfigured`;
  * object poses are bridged out of Gazebo so a grasp can be CHECKED
    (the object rises with the fingers) instead of emulated.

Usage:
  ros2 launch mycobot_gateway sim_grasp.launch.py
  ros2 launch mycobot_gateway sim_grasp.launch.py headless:=true
  ros2 launch mycobot_gateway sim_grasp.launch.py robot_appearance:=realistic
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
from launch.conditions import IfCondition, LaunchConfigurationEquals, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


WORLD_NAME = 'pick_and_place_sorting'


def generate_launch_description():
    desc_pkg = get_package_share_directory('mycobot_description')
    gz_pkg = get_package_share_directory('ros_gz_sim')

    # nogripper = the 50K DREAM render, file used as is (protocol I7); its
    # cameras are the dream50k ones and it has no xacro arguments.
    urdf_path = [os.path.join(desc_pkg, 'urdf', '320_pi', 'mycobot_pro_320_pi_gazebo'),
                 LaunchConfiguration('robot_model_suffix'), '.urdf']
    world_name = LaunchConfiguration('world_name')
    # Concatenation a plat : selon la version de launch, PathJoinSubstitution
    # rejette la liste imbriquee que '<world_name>.sdf' impose.
    default_world_path = [os.path.join(desc_pkg, 'worlds') + os.sep, world_name, '.sdf']
    world_path = LaunchConfiguration('world_file')
    controller_cfg = LaunchConfiguration('controller_config')

    set_gz_resource = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        ':'.join(filter(None, [
            os.path.dirname(desc_pkg),
            os.environ.get('GZ_SIM_RESOURCE_PATH', ''),
        ])),
    )

    headless_arg = DeclareLaunchArgument('headless', default_value='false')
    cameras_arg = DeclareLaunchArgument('enable_cameras', default_value='true')
    camera_layout_arg = DeclareLaunchArgument(
        'camera_layout', default_value='legacy', choices=['legacy', 'dream50k'],
        description='dream50k = the four camera poses that rendered the 50K DREAM dataset')
    gui_config_arg = DeclareLaunchArgument(
        'gui_config', default_value=os.path.join(desc_pkg, 'config', 'trajectory_gui.config'),
        description='Gazebo GUI with live tool trajectory')
    world_arg = DeclareLaunchArgument(
        'world_name', default_value=WORLD_NAME,
        description='World name and SDF basename in mycobot_description/worlds')
    camera_arg = DeclareLaunchArgument(
        'bridge_camera', default_value='false',
        description='Bridge /camera/image_raw and /camera/camera_info to ROS')
    world_file_arg = DeclareLaunchArgument('world_file', default_value=default_world_path)
    multicam_arg = DeclareLaunchArgument('bridge_multicam', default_value='false')
    gripper_mass_arg = DeclareLaunchArgument(
        'gripper_mass_kg', default_value='0.22',
        description='Gripper mass in kg; 0.22 = original model, 0.34 = datasheet (opt-in)')
    arm_mass_arg = DeclareLaunchArgument(
        'arm_mass_kg', default_value='1.58',
        description='Bare arm incl. base, excl. gripper; 1.58 = original model, 3.0 = datasheet (opt-in)')
    appearance_arg = DeclareLaunchArgument(
        'robot_appearance', default_value='original',
        choices=['original', 'realistic'],
        description='Robot visual materials; kinematics and collisions are unchanged')
    headless = LaunchConfiguration('headless')

    robot_description = ParameterValue(
        Command(['xacro ', *urdf_path, ' robot_appearance:=',
                 LaunchConfiguration('robot_appearance'), ' gripper_mass_kg:=',
                 LaunchConfiguration('gripper_mass_kg'), ' enable_cameras:=',
                 LaunchConfiguration('enable_cameras'), ' camera_layout:=',
                 LaunchConfiguration('camera_layout'), ' arm_mass_kg:=',
                 LaunchConfiguration('arm_mass_kg'), ' arm_control_mode:=',
                 LaunchConfiguration('arm_control_mode'), ' controller_config:=',
                 controller_cfg]), value_type=str)
    rsp = Node(
        package='robot_state_publisher', executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description,
                     'use_sim_time': True}],
    )

    gz_gui = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_pkg, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r ', world_path, ' --gui-config ',
                                     LaunchConfiguration('gui_config')],
                          'on_exit_shutdown': 'true'}.items(),
        condition=UnlessCondition(headless),
    )
    gz_headless = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_pkg, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': ['-r -s --headless-rendering ', world_path],
                          'on_exit_shutdown': 'true'}.items(),
        condition=IfCondition(headless),
    )

    spawn = Node(
        package='ros_gz_sim', executable='create', output='screen',
        arguments=['-topic', 'robot_description', '-name', 'mycobot_320',
                   '-world', world_name, '-z', '0.0'],
    )

    # /clock is what makes use_sim_time work; dynamic_pose/info is how a grasp
    # gets verified (object pose vs. finger pose) instead of assumed.
    gz_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=[
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            ['/world/', world_name, '/dynamic_pose/info'
             '@geometry_msgs/msg/PoseArray[gz.msgs.Pose_V'],
        ],
        remappings=[
            (['/world/', world_name, '/dynamic_pose/info'], '/gz/dynamic_poses'),
        ],
    )
    camera_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=[
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
        ],
        condition=IfCondition(LaunchConfiguration('bridge_camera')),
    )
    multicam_bridge = Node(
        package='ros_gz_bridge', executable='parameter_bridge', output='screen',
        arguments=[f'/{name}/{topic}@sensor_msgs/msg/{msg}[gz.msgs.{gz}'
                   for name in ('synth_camera', 'synth_camera_right',
                                'synth_camera_left', 'synth_camera_top')
                   for topic, msg, gz in (('image', 'Image', 'Image'),
                                          ('camera_info', 'CameraInfo', 'CameraInfo'))],
        condition=IfCondition(LaunchConfiguration('bridge_multicam')),
    )

    # Activation laissee a 30 s : avec les quatre cameras rendues, le pas
    # Gazebo depasse parfois les 5 s par defaut et le controleur reste inactif
    # (tri_yolo, 2 lancements sur 5 le 02/10).
    switch_timeout = ['--switch-timeout', '30']

    def spawner(name):
        return Node(
            package='controller_manager', executable='spawner',
            arguments=[name, '--controller-manager', '/controller_manager',
                       '--param-file', controller_cfg, *switch_timeout],
            output='screen',
        )

    return LaunchDescription([
        set_gz_resource,
        DeclareLaunchArgument('robot_model_suffix', default_value='', choices=['', '_nogripper'],
                              description="'' = with gripper; _nogripper = DREAM 50K robot"),
        DeclareLaunchArgument('arm_control_mode', default_value='position', choices=['position', 'effort']),
        DeclareLaunchArgument('controller_config', default_value=os.path.join(desc_pkg, 'config', 'controller.yaml')),
        headless_arg,
        cameras_arg,
        camera_layout_arg,
        gui_config_arg,
        world_arg,
        world_file_arg,
        camera_arg,
        multicam_arg,
        gripper_mass_arg,
        arm_mass_arg,
        appearance_arg,
        rsp,
        gz_gui,
        gz_headless,
        spawn,
        gz_bridge,
        camera_bridge,
        multicam_bridge,
        TimerAction(period=4.0, actions=[Node(
            package='controller_manager', executable='spawner',
            arguments=['joint_state_broadcaster',
                       '--controller-manager', '/controller_manager', *switch_timeout],
            output='screen')]),
        TimerAction(period=5.0, actions=[spawner('mycobot_controller')]),
        TimerAction(period=6.0, actions=[spawner('gripper_position_controller')],
                    condition=LaunchConfigurationEquals('robot_model_suffix', '')),
    ])
