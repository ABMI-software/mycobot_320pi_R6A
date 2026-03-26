#!/usr/bin/env python3
"""
Launch file for interactive robot commander.
Launches bridge + interactive command interface.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Bridge to Raspberry Pi
        Node(
            package='mycobot_gateway',
            executable='bridge_tour',
            name='bridge_tour',
            output='screen',
            emulate_tty=True,
        ),
        
        # Interactive commander
        # Note: Remove 'prefix' if xterm is not installed
        # Install xterm: sudo apt install xterm
        Node(
            package='mycobot_gateway',
            executable='robot_commander',
            name='robot_commander',
            output='screen',
            emulate_tty=True,
            # prefix='xterm -e',  # Uncomment if xterm is installed
        ),
    ])
