"""Robot-only trajectory display; demo motion is not the precision campaign."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package = get_package_share_directory('mycobot_gateway')
    return LaunchDescription([
        DeclareLaunchArgument('demo', default_value='true', choices=['true', 'false']),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('gripper_mass_kg', default_value='0.22'),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(package, 'launch', 'sim_grasp.launch.py')),
            launch_arguments={
                'world_name': 'real_table',
                'headless': LaunchConfiguration('headless'),
                'gripper_mass_kg': LaunchConfiguration('gripper_mass_kg'),
            }.items()),
        Node(package='mycobot_gateway', executable='sim_trajectory_preview',
             output='screen', parameters=[{'use_sim_time': True}],
             condition=IfCondition(LaunchConfiguration('demo'))),
    ])
