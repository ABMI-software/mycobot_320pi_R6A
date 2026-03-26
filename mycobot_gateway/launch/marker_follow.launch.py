#!/usr/bin/env python3
"""
Launch file for full marker following system on Tour (PC).
This launches:
  - bridge_tour: TCP bridge to Raspberry Pi
  - camera_publisher: Camera image capture
  - marker_detector: ArUco marker detection and command generation
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare arguments
    camera_index_arg = DeclareLaunchArgument(
        'camera_index',
        default_value='0',
        description='Camera device index'
    )
    
    return LaunchDescription([
        camera_index_arg,
        
        # Bridge to Raspberry Pi
        Node(
            package='mycobot_gateway',
            executable='bridge_tour',
            name='bridge_tour',
            output='screen',
            emulate_tty=True,
        ),
        
        # Camera publisher
        Node(
            package='mycobot_gateway',
            executable='camera_publisher',
            name='camera_publisher',
            output='screen',
            parameters=[{
                'camera_index': LaunchConfiguration('camera_index'),
                'frame_width': 640,
                'frame_height': 480,
                'fps': 30.0,
            }]
        ),
        
        # Marker detector (heavy computation)
        Node(
            package='mycobot_gateway',
            executable='marker_detector',
            name='marker_detector',
            output='screen',
        ),
    ])
