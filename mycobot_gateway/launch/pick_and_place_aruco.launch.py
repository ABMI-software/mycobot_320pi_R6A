#!/usr/bin/env python3
"""Launch pick-and-place ArUco en simulation Gazebo.

Pipeline lancé
--------------
  1. Gazebo Harmonic → monde precision_benchmark.sdf
     (robot + cube cible rouge + marqueurs workspace)
  2. robot_state_publisher  (URDF benchmark)
  3. ros_gz_bridge → /clock + /joint_states + pose cube
  4. Spawners ros2_control → joint_state_broadcaster (3.5s) +
     mycobot_controller (4.5s)
  5. gz_sim_localizer  (5s) → /aruco/object_pose depuis GT Gazebo
  6. fk_ee_pose        (5s) → /fk/ee_pose depuis /joint_states
  7. pick_and_place_aruco  (8s) → cycle pick-place complet

Usage
-----
  ros2 launch mycobot_gateway pick_and_place_aruco.launch.py
  ros2 launch mycobot_gateway pick_and_place_aruco.launch.py \
      settle_time:=3.0 target_x:=0.25 target_y:=0.05

Arguments
---------
  settle_time     : attente stabilisation par segment (s)  (défaut : 2.0)
  target_x/y/z    : position initiale du cube cible        (défaut : 0.22/0.00/0.02)
  place_x/y/z     : position de dépose                     (défaut : 0.20/-0.18/0.04)
  approach_height : hauteur approche au-dessus des cibles  (défaut : 0.12)
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description() -> LaunchDescription:
    desc_pkg = get_package_share_directory("mycobot_description")
    gz_pkg   = get_package_share_directory("ros_gz_sim")

    urdf_path  = os.path.join(
        desc_pkg, "urdf", "320_pi", "mycobot_pro_320_pi_benchmark.urdf"
    )
    world_path = os.path.join(desc_pkg, "worlds", "precision_benchmark.sdf")

    gz_resource_path = os.path.dirname(desc_pkg)
    existing_path    = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    full_gz_path = (
        f"{gz_resource_path}:{existing_path}" if existing_path else gz_resource_path
    )

    # ── Arguments ─────────────────────────────────────────────────────────────
    settle_arg   = DeclareLaunchArgument("settle_time",       default_value="2.0")
    target_x_arg = DeclareLaunchArgument("target_x",         default_value="0.22")
    target_y_arg = DeclareLaunchArgument("target_y",         default_value="0.00")
    target_z_arg = DeclareLaunchArgument("target_z",         default_value="0.02")
    place_x_arg  = DeclareLaunchArgument("place_x",          default_value="0.20")
    place_y_arg  = DeclareLaunchArgument("place_y",          default_value="-0.18")
    place_z_arg  = DeclareLaunchArgument("place_z",          default_value="0.04")
    app_h_arg    = DeclareLaunchArgument("approach_height",  default_value="0.12")

    set_gz_resource = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", full_gz_path)

    # ── Robot State Publisher ─────────────────────────────────────────────────
    robot_desc = ParameterValue(Command(["xacro ", urdf_path]), value_type=str)
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}],
        output="screen",
    )

    # ── Gazebo Harmonic (mode serveur, sans GUI Qt) ───────────────────────────
    # -s = server-only, évite le crash Qt "could not connect to display"
    # Retirer -s si vous souhaitez la visualisation (DISPLAY doit être défini)
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_pkg, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": f"-r -s {world_path}",
            "on_exit_shutdown": "true",
        }.items(),
    )

    # ── Spawn robot ───────────────────────────────────────────────────────────
    spawn = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=["-topic", "robot_description", "-name", "mycobot_320", "-z", "0.0"],
        output="screen",
    )

    # ── ros_gz_bridge ─────────────────────────────────────────────────────────
    bridge_args = [
        "/world/precision_benchmark/model/mycobot_320/joint_state"
        "@sensor_msgs/msg/JointState[gz.msgs.Model",
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        "/model/target_cube/pose@geometry_msgs/msg/Pose[gz.msgs.Pose",
    ]
    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=bridge_args,
        remappings=[
            (
                "/world/precision_benchmark/model/mycobot_320/joint_state",
                "/joint_states",
            ),
            ("/model/target_cube/pose", "/gz/model/target_cube/pose"),
        ],
        output="screen",
    )

    controller_cfg = os.path.join(desc_pkg, "config", "controller.yaml")

    jsb_spawner = TimerAction(
        period=3.5,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster",
                       "--controller-manager", "/controller_manager"],
            output="screen",
        )],
    )
    mycobot_spawner = TimerAction(
        period=4.5,
        actions=[Node(
            package="controller_manager",
            executable="spawner",
            arguments=["mycobot_controller",
                       "--controller-manager", "/controller_manager",
                       "--param-file", controller_cfg],
            output="screen",
        )],
    )

    # ── gz_sim_localizer : /aruco/object_pose depuis GT Gazebo ───────────────
    gz_localizer = TimerAction(
        period=5.0,
        actions=[Node(
            package="mycobot_gateway",
            executable="gz_sim_localizer",
            name="gz_sim_localizer",
            parameters=[{
                "target_x":      LaunchConfiguration("target_x"),
                "target_y":      LaunchConfiguration("target_y"),
                "target_z":      LaunchConfiguration("target_z"),
                "gz_pose_topic": "/gz/model/target_cube/pose",
                "use_sim_time":  True,
            }],
            output="screen",
        )],
    )

    # ── fk_ee_pose ────────────────────────────────────────────────────────────
    fk_node = TimerAction(
        period=5.0,
        actions=[Node(
            package="mycobot_gateway",
            executable="fk_ee_pose",
            name="fk_ee_pose_node",
            parameters=[{"use_sim_time": True}],
            output="screen",
        )],
    )

    # ── pick_and_place_aruco (délai 8s) ───────────────────────────────────────
    pp_node = TimerAction(
        period=8.0,
        actions=[Node(
            package="mycobot_gateway",
            executable="pick_and_place_aruco",
            name="pick_and_place_aruco",
            parameters=[{
                "mode":            "sim",
                "settle_time":     LaunchConfiguration("settle_time"),
                "place_x":         LaunchConfiguration("place_x"),
                "place_y":         LaunchConfiguration("place_y"),
                "place_z":         LaunchConfiguration("place_z"),
                "approach_height": LaunchConfiguration("approach_height"),
                "gz_world":        "precision_benchmark",
                "gz_object":       "target_cube",
                "use_sim_time":    True,
            }],
            output="screen",
        )],
    )

    return LaunchDescription([
        set_gz_resource,
        settle_arg, target_x_arg, target_y_arg, target_z_arg,
        place_x_arg, place_y_arg, place_z_arg, app_h_arg,
        rsp,
        gz_sim,
        spawn,
        gz_bridge,
        jsb_spawner,
        mycobot_spawner,
        fk_node,
        gz_localizer,
        pp_node,
    ])
