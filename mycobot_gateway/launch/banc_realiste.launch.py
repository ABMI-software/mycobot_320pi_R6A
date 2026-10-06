#!/usr/bin/env python3
"""Banc REALISTE : les 4 pieces peintes et leurs bacs, vus par arducam et SVPRO.

Variante realiste de `real_table.launch.py`. Ce qu'elle apporte :

  * la scene `banc_realiste_yolo26.sdf` — plateau mesure, marqueurs remis aux
    positions relevees AU ROBOT, les 4 pieces et les 4 bacs aux cotes du dossier
    de fabrication, et plus aucun reste de l'ancien banc ;
  * les deux cameras a LEUR pose calibree, avec LEURS intrinseques, donc les
    memes millimetres robot qu'en reel ;
  * les masses de la fiche 320 Pi 2022 — bras 3 kg, pince 0,340 kg — et les
    butees MESUREES en pratique, deja portees par l'URDF
    (J1 168, J2 135, J3 150, J4 145, J5 165, J6 180 degres).

    ros2 launch mycobot_gateway banc_realiste.launch.py
    ros2 launch mycobot_gateway banc_realiste.launch.py headless:=true

Les images sortent sur /arducam/image_raw et /svpro/image_raw, ou
`scripts/gazebo_yolo26.py` vient les lire.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            SetEnvironmentVariable)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

MONDE = 'banc_realiste_yolo26'
CAMERAS = ('arducam', 'svpro')


def generate_launch_description():
    desc_pkg = get_package_share_directory('mycobot_description')
    gateway_pkg = get_package_share_directory('mycobot_gateway')
    monde = os.path.join(desc_pkg, 'worlds', f'{MONDE}.sdf')

    ponts = []
    for camera in CAMERAS:
        ponts += [f'/{camera}/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
                  f'/{camera}/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo']

    return LaunchDescription([
        # SANS CES DEUX LIGNES, GAZEBO SEGFAUTE AU PREMIER RENDU DE CAPTEUR.
        # libEGL choisit Mesa (« egl: failed to create dri2 screen ») au lieu de
        # la NVIDIA, et le fil qui rend les cameras meurt — la scene tourne, les
        # sujets image restent muets, et rien ne dit pourquoi.
        SetEnvironmentVariable('__EGL_VENDOR_LIBRARY_FILENAMES',
                               '/usr/share/glvnd/egl_vendor.d/10_nvidia.json'),
        SetEnvironmentVariable('__GLX_VENDOR_LIBRARY_NAME', 'nvidia'),
        DeclareLaunchArgument('headless', default_value='false'),
        DeclareLaunchArgument('arm_mass_kg', default_value='3.0',
                              description='fiche 320 Pi 2022'),
        DeclareLaunchArgument('gripper_mass_kg', default_value='0.34',
                              description='fiche 320 Pi 2022'),
        DeclareLaunchArgument('robot_appearance', default_value='realistic',
                              choices=['original', 'realistic']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(gateway_pkg, 'launch', 'sim_grasp.launch.py')),
            launch_arguments={
                'world_name': MONDE,
                'world_file': monde,
                'headless': LaunchConfiguration('headless'),
                'arm_mass_kg': LaunchConfiguration('arm_mass_kg'),
                'gripper_mass_kg': LaunchConfiguration('gripper_mass_kg'),
                'robot_appearance': LaunchConfiguration('robot_appearance'),
                'enable_cameras': 'false',
            }.items()),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             output='screen', arguments=ponts),
    ])
