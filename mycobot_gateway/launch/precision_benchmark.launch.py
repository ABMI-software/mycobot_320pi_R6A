#!/usr/bin/env python3
"""Launch du benchmark de précision en simulation Gazebo.

Pipeline lancé
--------------
  1. Gazebo Harmonic  →  monde precision_benchmark.sdf
  2. robot_state_publisher  (URDF MyCobot 320 Pi)
  3. ros_gz_bridge  →  joint_states + commandes joints
  4. gz_sim_localizer_node  (délai 4s)  →  /aruco/object_pose depuis GT Gazebo
  5. fk_ee_pose_node  (délai 4s)  →  /fk/ee_pose depuis /joint_states
  6. precision_benchmark_node  (délai 6s)  →  benchmark 9 cibles + rapport CSV

Usage
-----
  ros2 launch mycobot_gateway precision_benchmark.launch.py
  ros2 launch mycobot_gateway precision_benchmark.launch.py settle_time:=3.0
  ros2 launch mycobot_gateway precision_benchmark.launch.py target_x:=0.22 target_y:=0.05

Arguments
---------
  settle_time   : temps de stabilisation par cible en secondes  (défaut : 2.0)
  target_x/y/z  : position initiale du cube cible (défaut : 0.22 / 0.00 / 0.02)
  output_dir    : dossier de sortie du CSV                       (défaut : ~)
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
        desc_pkg, "urdf", "320_pi", "mycobot_pro_320_pi_gazebo.urdf"
    )
    world_path = os.path.join(desc_pkg, "worlds", "precision_benchmark.sdf")

    # GZ_SIM_RESOURCE_PATH pour résoudre les meshes
    gz_resource_path = os.path.dirname(desc_pkg)
    existing_path    = os.environ.get("GZ_SIM_RESOURCE_PATH", "")
    full_gz_path = (
        f"{gz_resource_path}:{existing_path}" if existing_path else gz_resource_path
    )

    # ── Arguments ─────────────────────────────────────────────────────────────
    settle_arg   = DeclareLaunchArgument("settle_time",  default_value="2.0",
                                         description="Temps stabilisation (s)")
    target_x_arg = DeclareLaunchArgument("target_x",    default_value="0.22",
                                         description="X cube cible (m)")
    target_y_arg = DeclareLaunchArgument("target_y",    default_value="0.00",
                                         description="Y cube cible (m)")
    target_z_arg = DeclareLaunchArgument("target_z",    default_value="0.02",
                                         description="Z cube cible (m)")
    outdir_arg   = DeclareLaunchArgument("output_dir",  default_value=os.path.expanduser("~"),
                                         description="Dossier CSV résultats")

    # ── Variables Gazebo ──────────────────────────────────────────────────────
    set_gz_resource = SetEnvironmentVariable("GZ_SIM_RESOURCE_PATH", full_gz_path)

    # ── Robot State Publisher ─────────────────────────────────────────────────
    robot_desc = ParameterValue(Command(["xacro ", urdf_path]), value_type=str)
    rsp = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}],
        output="screen",
    )

    # ── Gazebo Harmonic ───────────────────────────────────────────────────────
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_pkg, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": f"-r {world_path}",
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

    # ── ros_gz_bridge : joint_states + commandes ──────────────────────────────
    joint_names = [
        "joint2_to_joint1",
        "joint3_to_joint2",
        "joint4_to_joint3",
        "joint5_to_joint4",
        "joint6_to_joint5",
        "joint6output_to_joint6",
    ]

    bridge_args = [
        "/world/precision_benchmark/model/mycobot_320/joint_state"
        "@sensor_msgs/msg/JointState[gz.msgs.Model",
        # Pose du cube cible (pour gz_sim_localizer)
        "/model/target_cube/pose@geometry_msgs/msg/Pose[gz.msgs.Pose",
    ]
    for jn in joint_names:
        bridge_args.append(
            f"/model/mycobot_320/joint/{jn}/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double"
        )

    gz_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=bridge_args,
        remappings=[
            (
                "/world/precision_benchmark/model/mycobot_320/joint_state",
                "/joint_states",
            ),
            (
                "/model/target_cube/pose",
                "/gz/model/target_cube/pose",
            ),
        ],
        output="screen",
    )

    # ── gz_sim_localizer : publie /aruco/object_pose (délai 4s) ─────────────
    gz_localizer = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="mycobot_gateway",
                executable="gz_sim_localizer",
                name="gz_sim_localizer",
                parameters=[{
                    "target_x":      LaunchConfiguration("target_x"),
                    "target_y":      LaunchConfiguration("target_y"),
                    "target_z":      LaunchConfiguration("target_z"),
                    "gz_pose_topic": "/gz/model/target_cube/pose",
                }],
                output="screen",
            ),
        ],
    )

    # ── fk_ee_pose : publie /fk/ee_pose (délai 4s) ───────────────────────────
    fk_node = TimerAction(
        period=4.0,
        actions=[
            Node(
                package="mycobot_gateway",
                executable="fk_ee_pose",
                name="fk_ee_pose_node",
                output="screen",
            ),
        ],
    )

    # ── benchmark principal (délai 6s) ────────────────────────────────────────
    benchmark_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package="mycobot_gateway",
                executable="precision_benchmark",
                name="precision_benchmark_node",
                parameters=[{
                    "settle_time": LaunchConfiguration("settle_time"),
                    "use_aruco":   True,
                    "output_dir":  LaunchConfiguration("output_dir"),
                }],
                output="screen",
            ),
        ],
    )

    return LaunchDescription([
        set_gz_resource,
        # Arguments
        settle_arg, target_x_arg, target_y_arg, target_z_arg, outdir_arg,
        # Infrastructure
        rsp,
        gz_sim,
        spawn,
        gz_bridge,
        # Nœuds benchmark (délayés)
        fk_node,
        gz_localizer,
        benchmark_node,
    ])
