"""
Launch file pour visualiser le MyCobot 320 Pi dans RViz2.

Ce launch file:
1. Charge le modèle URDF du robot
2. Publie les transforms via robot_state_publisher
3. Lance joint_state_publisher_gui pour contrôler les joints manuellement
4. Ouvre RViz2 avec la configuration appropriée

Usage:
    ros2 launch mycobot_description display.launch.py
    ros2 launch mycobot_description display.launch.py gui:=false  # Sans GUI
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Chemins des packages
    pkg_description = get_package_share_directory('mycobot_description')
    
    # Arguments de lancement
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value=os.path.join(pkg_description, 'urdf', '320_pi', 'mycobot_pro_320_pi.urdf'),
        description='Chemin vers le fichier URDF du robot'
    )
    
    rvizconfig_arg = DeclareLaunchArgument(
        name='rvizconfig',
        default_value=os.path.join(pkg_description, 'config', 'mycobot_320_pi.rviz'),
        description='Chemin vers la configuration RViz'
    )
    
    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='true',
        description='Lancer joint_state_publisher_gui (true/false)'
    )
    
    # Description du robot (URDF)
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )
    
    # Node robot_state_publisher - publie les transforms TF
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 30.0
        }]
    )
    
    # Node joint_state_publisher (sans GUI)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        condition=UnlessCondition(LaunchConfiguration('gui'))
    )
    
    # Node joint_state_publisher_gui (avec GUI)
    joint_state_publisher_gui_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        condition=IfCondition(LaunchConfiguration('gui'))
    )
    
    # Node RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', LaunchConfiguration('rvizconfig')]
    )
    
    return LaunchDescription([
        model_arg,
        rvizconfig_arg,
        gui_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node
    ])
