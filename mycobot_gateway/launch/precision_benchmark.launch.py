#!/usr/bin/env python3
"""Launch du benchmark de précision en simulation Gazebo.

Pipeline lancé
--------------
  1. Gazebo Harmonic  →  monde precision_benchmark.sdf
  2. robot_state_publisher  (URDF MyCobot 320 Pi)
  3. ros_gz_bridge  →  /clock + joint_states
  4. Spawners ros2_control  →  joint_state_broadcaster (3.5s) + mycobot_controller (4.5s)
  5. gz_sim_localizer_node  (délai 5s)  →  /aruco/object_pose depuis GT Gazebo
  6. fk_ee_pose_node  (délai 5s)  →  /fk/ee_pose depuis /joint_states
  7. precision_benchmark_node  (délai 8s)  →  benchmark 9 cibles + rapport CSV

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
        desc_pkg, "urdf", "320_pi", "mycobot_pro_320_pi_benchmark.urdf"
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

    # ── Gazebo Harmonic (mode serveur : pas d'IHM Qt, compatible SSH/VS Code) ─
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gz_pkg, "launch", "gz_sim.launch.py")
        ),
        launch_arguments={
            "gz_args": f"-r -s {world_path}",   # -s = server-only, pas de GUI Qt
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
        # Horloge Gazebo → ROS (nécessaire pour ros2_control)
        "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
        # Pose du cube cible (pour gz_sim_localizer)
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
            (
                "/model/target_cube/pose",
                "/gz/model/target_cube/pose",
            ),
        ],
        output="screen",
    )

    controller_cfg = os.path.join(desc_pkg, "config", "controller.yaml")

    # ── ros2_control controller spawners ─────────────────────────────────────
    jsb_spawner = TimerAction(
        period=3.5,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster",
                           "--controller-manager", "/controller_manager"],
                output="screen",
            ),
        ],
    )
    mycobot_spawner = TimerAction(
        period=4.5,
        actions=[
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["mycobot_controller",
                           "--controller-manager", "/controller_manager",
                           "--param-file", controller_cfg],
                output="screen",
            ),
        ],
    )

    # ── gz_sim_localizer : publie /aruco/object_pose (délai 5s) ─────────────
    gz_localizer = TimerAction(
        period=5.0,
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
                    "use_sim_time":  True,
                }],
                output="screen",
            ),
        ],
    )

    # ── fk_ee_pose : publie /fk/ee_pose (délai 5s) ───────────────────────────
    fk_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package="mycobot_gateway",
                executable="fk_ee_pose",
                name="fk_ee_pose_node",
                parameters=[{"use_sim_time": True}],
                output="screen",
            ),
        ],
    )

    # ── benchmark principal (délai 8s) ────────────────────────────────────────
    benchmark_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package="mycobot_gateway",
                executable="precision_benchmark",
                name="precision_benchmark_node",
                parameters=[{
                    "settle_time": LaunchConfiguration("settle_time"),
                    "use_aruco":   True,
                    "output_dir":  LaunchConfiguration("output_dir"),
                    "use_sim_time": True,
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
        jsb_spawner,
        mycobot_spawner,
        fk_node,
        gz_localizer,
        benchmark_node,
    ])
