#!/usr/bin/env python3
"""Launch pick-and-place ArUco sur robot réel MyCobot 320 Pi.

Pipeline lancé
--------------
  1. bridge_tour       → pont TCP Tour ↔ Pi (10.10.0.223:5005)
  2. trajectory_to_robot_bridge → /mycobot_controller/joint_trajectory
                                  → JSON /to_robot → bridge_tour → Pi
  3. fk_ee_pose        → /joint_states (Pi) → FK → /fk/ee_pose
  4. aruco_localizer   → cam_0 → ArUco → /aruco/object_pose
  5. pick_and_place_aruco (mode=real) → cycle pick-place complet

Prérequis avant lancement
--------------------------
  1. Vérifier ping Pi : ping -c 1 10.10.0.223
  2. Lancer le bridge sur le Pi :
       ssh er@10.10.0.223 'python3 ~/bridge_pi_simple.py'
  3. S'assurer que la caméra cam_0 est branchée (USB) et visible.
  4. Imprimer et positionner les marqueurs ArUco :
       - 4 marqueurs workspace (IDs 0-3, 50 mm, DICT_4X4_1000)
         aux coins de la zone de travail (voir docs/CAMERA_CALIBRATION.md)
       - 1 marqueur objet (ID 10, 40 mm) sur le cube à saisir
  5. Lancer le preflight : bash scripts/real_robot_preflight.sh

Usage
-----
  ros2 launch mycobot_gateway pick_and_place_aruco_real.launch.py
  ros2 launch mycobot_gateway pick_and_place_aruco_real.launch.py \
      settle_time:=3.0 pi_ip:=10.10.0.223

Arguments
---------
  pi_ip           : IP du Raspberry Pi                    (défaut : 10.10.0.223)
  pi_port         : port TCP bridge Pi                    (défaut : 5005)
  camera_topic    : topic image caméra                   (défaut : /camera/image_raw)
  calib_file      : chemin vers cam_0.npz                (défaut : auto)
  settle_time     : attente stabilisation par segment (s) (défaut : 3.0)
  speed           : vitesse pymycobot (1-100)             (défaut : 40)
  place_x/y/z     : position de dépose (m)               (défaut : 0.20/-0.18/0.04)
  approach_height : hauteur approche (m)                  (défaut : 0.12)
"""

from __future__ import annotations

import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:

    # ── Arguments ─────────────────────────────────────────────────────────────
    pi_ip_arg    = DeclareLaunchArgument("pi_ip",           default_value="10.10.0.223")
    pi_port_arg  = DeclareLaunchArgument("pi_port",         default_value="5005")
    cam_arg      = DeclareLaunchArgument("camera_topic",    default_value="/camera/image_raw")
    calib_arg    = DeclareLaunchArgument("calib_file",      default_value="")
    settle_arg   = DeclareLaunchArgument("settle_time",     default_value="3.0")
    speed_arg    = DeclareLaunchArgument("speed",           default_value="40")
    place_x_arg  = DeclareLaunchArgument("place_x",        default_value="0.20")
    place_y_arg  = DeclareLaunchArgument("place_y",        default_value="-0.18")
    place_z_arg  = DeclareLaunchArgument("place_z",        default_value="0.04")
    app_h_arg    = DeclareLaunchArgument("approach_height", default_value="0.12")

    # ── bridge_tour : Tour ↔ Pi TCP ───────────────────────────────────────────
    bridge_tour = Node(
        package="mycobot_gateway",
        executable="bridge_tour",
        name="bridge_tour",
        parameters=[{
            "pi_ip":   LaunchConfiguration("pi_ip"),
            "pi_port": LaunchConfiguration("pi_port"),
        }],
        output="screen",
    )

    # ── trajectory_to_robot_bridge ────────────────────────────────────────────
    # Convertit JointTrajectory (rad) → JSON send_angles (degrés) → /to_robot
    traj_bridge = Node(
        package="mycobot_gateway",
        executable="trajectory_to_robot_bridge",
        name="trajectory_to_robot_bridge",
        parameters=[{
            "trajectory_topic": "/mycobot_controller/joint_trajectory",
            "out_topic":        "/to_robot",
            "speed":            LaunchConfiguration("speed"),
            "rate_hz":          15.0,
            "deadband_deg":     0.5,
        }],
        output="screen",
    )

    # ── fk_ee_pose : /joint_states → FK → /fk/ee_pose ─────────────────────────
    fk_node = Node(
        package="mycobot_gateway",
        executable="fk_ee_pose",
        name="fk_ee_pose_node",
        output="screen",
    )

    # ── aruco_localizer : caméra → ArUco → /aruco/object_pose ────────────────
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

    # ── pick_and_place_aruco (mode real) ─────────────────────────────────────
    pp_node = Node(
        package="mycobot_gateway",
        executable="pick_and_place_aruco",
        name="pick_and_place_aruco",
        parameters=[{
            "mode":            "real",
            "settle_time":     LaunchConfiguration("settle_time"),
            "speed":           LaunchConfiguration("speed"),
            "place_x":         LaunchConfiguration("place_x"),
            "place_y":         LaunchConfiguration("place_y"),
            "place_z":         LaunchConfiguration("place_z"),
            "approach_height": LaunchConfiguration("approach_height"),
            "pose_timeout":    30.0,
        }],
        output="screen",
    )

    return LaunchDescription([
        pi_ip_arg, pi_port_arg, cam_arg, calib_arg,
        settle_arg, speed_arg,
        place_x_arg, place_y_arg, place_z_arg, app_h_arg,
        bridge_tour,
        traj_bridge,
        fk_node,
        aruco_node,
        pp_node,
    ])
