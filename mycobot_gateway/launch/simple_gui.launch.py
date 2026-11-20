#!/usr/bin/env python3
"""
Launch file: Simple GUI
Lance le bridge + l'interface graphique simple pour contrôler le robot.

Usage:
    MYCOBOT_PI=<ip> ros2 launch mycobot_gateway simple_gui.launch.py
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import EnvironmentVariable, LaunchConfiguration
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command


def generate_launch_description():
    # Arguments
    pi_ip_arg = DeclareLaunchArgument(
        'pi_ip',
        default_value=EnvironmentVariable('MYCOBOT_PI'),
        description='IP address of Raspberry Pi (défaut : $MYCOBOT_PI)'
    )
    
    pi_port_arg = DeclareLaunchArgument(
        'pi_port',
        default_value='5005',
        description='TCP port on Raspberry Pi'
    )
    
    # Bridge Tour node
    bridge_tour_node = Node(
        package='mycobot_gateway',
        executable='bridge_tour',
        name='bridge_tour',
        output='screen',
        parameters=[{
            'pi_ip': LaunchConfiguration('pi_ip'),
            'pi_port': LaunchConfiguration('pi_port'),
        }]
    )
    
    # Simple GUI node
    simple_gui_node = Node(
        package='mycobot_gateway',
        executable='simple_gui',
        name='simple_gui',
        output='screen',
    )
    
    return LaunchDescription([
        pi_ip_arg,
        pi_port_arg,
        bridge_tour_node,
        simple_gui_node,
    ])
