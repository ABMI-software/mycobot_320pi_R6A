#!/usr/bin/env python3
"""
Launch file: Teleop Keyboard
Lance le bridge + le contrôle clavier pour piloter le robot.

Usage:
    ros2 launch mycobot_gateway teleop_keyboard.launch.py

NOTE: Ce launch file lance le teleop dans le terminal courant.
      Assurez-vous que le terminal a le focus pour les entrées clavier.
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Arguments
    pi_ip_arg = DeclareLaunchArgument(
        'pi_ip',
        default_value='10.10.0.218',
        description='IP address of Raspberry Pi'
    )
    
    # Bridge Tour node
    bridge_tour_node = Node(
        package='mycobot_gateway',
        executable='bridge_tour',
        name='bridge_tour',
        output='screen',
        parameters=[{
            'pi_ip': LaunchConfiguration('pi_ip'),
        }]
    )
    
    # Teleop Keyboard node
    teleop_node = Node(
        package='mycobot_gateway',
        executable='teleop_keyboard',
        name='teleop_keyboard',
        output='screen',
        prefix='xterm -e',  # Lance dans un nouveau terminal
    )
    
    return LaunchDescription([
        pi_ip_arg,
        bridge_tour_node,
        teleop_node,
    ])
