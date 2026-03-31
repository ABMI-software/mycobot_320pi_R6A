#!/usr/bin/env python3
"""Launch Gazebo and spawn the MyCobot 320 Pi URDF.

Minimal integration for local testing / simulation.

Usage:
  ros2 launch mycobot_description gazebo_sim.launch.py

Requirements:
  - gazebo_ros package installed
  - the URDF file exists at `urdf/320_pi/mycobot_pro_320_pi.urdf`
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_share = get_package_share_directory('mycobot_description')

    # URDF file for the robot (adjust name/path if you use a different file)
    urdf_path = os.path.join(pkg_share, 'urdf', '320_pi', 'mycobot_pro_320_pi.urdf')
    if not os.path.exists(urdf_path):
        raise FileNotFoundError(f"URDF not found: {urdf_path}")

    # Include the standard gazebo launch from gazebo_ros (starts gzserver + gzclient)
    gazebo_pkg = 'gazebo_ros'
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory(gazebo_pkg), 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={'verbose': 'true'}.items(),
    )

    # Spawn the robot into Gazebo using the URDF file
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-file', urdf_path, '-entity', 'mycobot_320'],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(gazebo_launch)
    ld.add_action(spawn_entity)
    return ld
