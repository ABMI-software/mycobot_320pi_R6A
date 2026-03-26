#!/usr/bin/env python3
"""
Launch file: Slider Control
Lance RViz avec joint_state_publisher_gui + bridge pour contrôler
le robot réel avec les sliders.

Le robot dans RViz et le robot réel bougent en même temps!

Usage:
    ros2 launch mycobot_gateway slider_control.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Chemins
    description_pkg = get_package_share_directory('mycobot_description')
    gateway_pkg = get_package_share_directory('mycobot_gateway')
    
    # Arguments
    model_arg = DeclareLaunchArgument(
        'model',
        default_value=os.path.join(description_pkg, 'urdf/320_pi/mycobot_pro_320_pi.urdf'),
        description='Path to URDF file'
    )
    
    pi_ip_arg = DeclareLaunchArgument(
        'pi_ip',
        default_value='10.10.0.218',
        description='IP address of Raspberry Pi'
    )
    
    # Robot description
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{'robot_description': robot_description}]
    )
    
    # Joint State Publisher GUI (sliders)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
    )
    
    # Bridge Tour
    bridge_tour_node = Node(
        package='mycobot_gateway',
        executable='bridge_tour',
        name='bridge_tour',
        output='screen',
        parameters=[{
            'pi_ip': LaunchConfiguration('pi_ip'),
        }]
    )
    
    # Slider Control (écoute joint_states, envoie au robot)
    slider_control_node = Node(
        package='mycobot_gateway',
        executable='slider_control',
        name='slider_control',
        output='screen',
    )
    
    # Configuration RViz
    rviz_config = os.path.join(description_pkg, 'config', 'mycobot_320_pi.rviz')
    
    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([
        model_arg,
        pi_ip_arg,
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        bridge_tour_node,
        slider_control_node,
        rviz_node,
    ])
