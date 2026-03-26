#!/usr/bin/env python3
"""
Launch file for bridge only.
Use this when you want to control the robot manually or from another node.
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
    ])
