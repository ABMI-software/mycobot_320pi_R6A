"""
Launch file pour la visualisation complète du MyCobot 320 avec synchronisation.

Ce launch file:
1. Lance le bridge_tour pour la communication avec la Pi
2. Lance le joint_sync pour synchroniser RViz avec le robot réel
3. Charge le modèle URDF et lance robot_state_publisher
4. Lance RViz2 pour la visualisation

Usage:
    ros2 launch mycobot_gateway rviz_sync.launch.py
    
Note: Le bridge_pi doit être lancé sur la Raspberry Pi
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Chemins des packages
    pkg_description = get_package_share_directory('mycobot_description')
    pkg_gateway = get_package_share_directory('mycobot_gateway')
    
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
    
    auto_sync_arg = DeclareLaunchArgument(
        name='auto_sync',
        default_value='true',
        description='Synchroniser automatiquement avec le robot réel'
    )
    
    sync_rate_arg = DeclareLaunchArgument(
        name='sync_rate',
        default_value='5.0',
        description='Fréquence de synchronisation (Hz)'
    )
    
    launch_bridge_arg = DeclareLaunchArgument(
        name='launch_bridge',
        default_value='true',
        description='Lancer le bridge_tour'
    )
    
    # Description du robot (URDF)
    robot_description = ParameterValue(
        Command(['xacro ', LaunchConfiguration('model')]),
        value_type=str
    )
    
    # Node bridge_tour (communication avec la Pi)
    bridge_tour_node = Node(
        package='mycobot_gateway',
        executable='bridge_tour',
        name='bridge_tour',
        output='screen',
        condition=IfCondition(LaunchConfiguration('launch_bridge'))
    )
    
    # Node robot_state_publisher
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
    
    # Node joint_sync (synchronisation des joints)
    joint_sync_node = Node(
        package='mycobot_gateway',
        executable='joint_sync',
        name='joint_sync',
        output='screen',
        parameters=[{
            'auto_sync': LaunchConfiguration('auto_sync'),
            'sync_rate': LaunchConfiguration('sync_rate')
        }]
    )
    
    # Node RViz2 (lancé après un délai pour laisser le temps aux autres nodes)
    rviz_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                output='screen',
                arguments=['-d', LaunchConfiguration('rvizconfig')]
            )
        ]
    )
    
    return LaunchDescription([
        model_arg,
        rvizconfig_arg,
        auto_sync_arg,
        sync_rate_arg,
        launch_bridge_arg,
        bridge_tour_node,
        robot_state_publisher_node,
        joint_sync_node,
        rviz_node
    ])
