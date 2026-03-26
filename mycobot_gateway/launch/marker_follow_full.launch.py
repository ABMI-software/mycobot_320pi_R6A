#!/usr/bin/env python3
"""
Launch file: Marker Follow (Full Stack)
Lance toute la stack pour suivre un marqueur ArUco avec le robot réel.

Architecture:
    [Caméra Pi] → stream → [Tour: detect_marker] → TF → [Tour: marker_follower] 
                                                              ↓
    [Robot Pi] ← bridge_pi ← TCP ← [Tour: bridge_tour] ← /to_robot

Usage:
    ros2 launch mycobot_gateway marker_follow_full.launch.py

Prérequis:
    - Caméra connectée au Pi ou Topic /camera/image_raw disponible
    - bridge_pi_debug.py en cours sur le Pi
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
    
    # Joint Sync (pour synchroniser RViz avec le robot réel)
    joint_sync_node = Node(
        package='mycobot_gateway',
        executable='joint_sync',
        name='joint_sync',
        output='screen',
    )
    
    # Marker Detector (traitement vision sur Tour)
    # Note: Nécessite que le topic camera/image_raw soit disponible
    marker_detector_node = Node(
        package='mycobot_gateway',
        executable='marker_detector',
        name='marker_detector',
        output='screen',
    )
    
    # Marker Follower (calcule et envoie les commandes)
    marker_follower_node = Node(
        package='mycobot_gateway',
        executable='marker_follower',
        name='marker_follower',
        output='screen',
    )
    
    # RViz pour visualisation
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
    )
    
    return LaunchDescription([
        model_arg,
        pi_ip_arg,
        robot_state_publisher_node,
        bridge_tour_node,
        joint_sync_node,
        marker_detector_node,
        marker_follower_node,
        rviz_node,
    ])
