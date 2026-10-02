#!/usr/bin/env python3
"""Launch du benchmark de précision sur robot réel.

Pipeline lancé
--------------
  1. aruco_localizer_node  →  détecte workspace (IDs 0-3) + objet (ID 10)
                              publie /aruco/object_pose + /aruco/workspace_valid
  2. fk_ee_pose_node       →  /joint_states → FK → /fk/ee_pose
  3. precision_benchmark_node  →  benchmark 9 cibles + test saisie ArUco

Prérequis
---------
  - MyCobot 320 Pi allumé et connecté (ROS 2 bridge côté Pi actif)
  - /joint_states disponible sur le réseau ROS 2
  - Caméra Arducam cam_0 publiée sur /camera/image_raw (ou paramètre camera_topic)
  - 4 marqueurs workspace (IDs 0-3, 50 mm, DICT_4X4_1000) placés à :
        ID 0 : ( 150, -150, 0 ) mm  [avant-gauche,  bleu ]
        ID 1 : ( 150,  150, 0 ) mm  [avant-droit,   vert ]
        ID 2 : ( 280, -150, 0 ) mm  [arrière-gauche, jaune]
        ID 3 : ( 280,  150, 0 ) mm  [arrière-droit, orange]
  - 1 marqueur objet (ID 10, 40 mm) sur le cube cible

Usage
-----
  ros2 launch mycobot_gateway precision_benchmark_real.launch.py
  ros2 launch mycobot_gateway precision_benchmark_real.launch.py \
      settle_time:=3.0 camera_topic:=/cam_0/image_raw

Arguments
---------
  settle_time    : attente stabilisation par cible (s)  (défaut : 3.0)
  camera_topic   : topic image caméra                  (défaut : /camera/image_raw)
  calib_file     : chemin vers cam_0.npz               (défaut : auto)
  output_dir     : dossier CSV résultats               (défaut : ~)
"""

from __future__ import annotations

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    # ── Arguments ─────────────────────────────────────────────────────────────
    settle_arg  = DeclareLaunchArgument("settle_time",   default_value="3.0",
                                        description="Temps stabilisation (s)")
    cam_arg     = DeclareLaunchArgument("camera_topic",  default_value="/camera/image_raw",
                                        description="Topic image caméra")
    calib_arg   = DeclareLaunchArgument("calib_file",    default_value="",
                                        description="Chemin cam_0.npz (vide = auto)")
    outdir_arg  = DeclareLaunchArgument("output_dir",    default_value=os.path.expanduser("~"),
                                        description="Dossier CSV résultats")

    # ── fk_ee_pose ────────────────────────────────────────────────────────────
    fk_node = Node(
        package="mycobot_gateway",
        executable="fk_ee_pose",
        name="fk_ee_pose_node",
        output="screen",
    )

    # ── ArUco localizer ───────────────────────────────────────────────────────
    aruco_node = Node(
        package="mycobot_gateway",
        executable="aruco_localizer",
        name="aruco_localizer_node",
        parameters=[{
            "camera_topic":    LaunchConfiguration("camera_topic"),
            "calib_file":      LaunchConfiguration("calib_file"),
            "ws_marker_size":  0.050,
            "obj_marker_size": 0.040,
            "obj_marker_id":   10,
        }],
        output="screen",
    )

    # ── benchmark ─────────────────────────────────────────────────────────────
    benchmark_node = Node(
        package="mycobot_gateway",
        executable="precision_benchmark",
        name="precision_benchmark_node",
        parameters=[{
            "settle_time": LaunchConfiguration("settle_time"),
            "use_aruco":   True,
            "output_dir":  LaunchConfiguration("output_dir"),
        }],
        output="screen",
    )

    return LaunchDescription([
        settle_arg, cam_arg, calib_arg, outdir_arg,
        fk_node,
        aruco_node,
        benchmark_node,
    ])
