"""Full hand-teleoperation launch for the MyCobot 320 Pi.

Brings up:
  1. Gazebo Harmonic with the robot spawned from the ros2_control URDF
  2. robot_state_publisher publishing /robot_description + TF
  3. ros_gz_bridge forwarding /joint_states
  4. Controller spawners (joint_state_broadcaster + mycobot_controller)
  5. rosbridge_server on ws://localhost:9090 (for the teleop script)
  6. bridge_tour TCP client → Raspberry Pi (real robot) — optional via target arg
  7. trajectory_to_robot_bridge node converting /mycobot_controller/joint_trajectory
     into JSON send_angles messages forwarded to bridge_tour

Usage:
  # Gazebo + real robot (default)
  ros2 launch mycobot_gateway mycobot_teleop.launch.py

  # Gazebo only (no bridge_tour, no trajectory→robot forwarding)
  ros2 launch mycobot_gateway mycobot_teleop.launch.py target:=sim

  # Real robot only (no Gazebo)
  ros2 launch mycobot_gateway mycobot_teleop.launch.py target:=real

Then in a separate terminal (conda env hand-teleop activated):
  python3 teleop/mycobot_teleop.py --ros --use-rosbridge --time-from-start 0.8

Launch arguments:
  target       : sim | real | both   (default: both)
  pi_ip        : Raspberry Pi IP      (default: 10.10.0.225)
  rosbridge    : Start rosbridge ws   (default: true)
  real_speed   : MyCobot servo speed  (default: 40)
  real_rate_hz : max JSON send rate   (default: 15)
"""

from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    desc_share = get_package_share_directory("mycobot_description")
    urdf_path = os.path.join(desc_share, "urdf", "320_pi", "mycobot_pro_320_pi_gazebo.urdf")
    controller_cfg = os.path.join(desc_share, "config", "controller.yaml")

    # ------------------------------- arguments ------------------------------- #
    target_arg = DeclareLaunchArgument(
        "target", default_value="both", choices=["sim", "real", "both"],
        description="Where trajectories go: sim (Gazebo only), real (Pi only), or both.",
    )
    pi_ip_arg = DeclareLaunchArgument("pi_ip", default_value="10.10.0.225")
    pi_port_arg = DeclareLaunchArgument("pi_port", default_value="5005")
    rosbridge_arg = DeclareLaunchArgument("rosbridge", default_value="true")
    real_speed_arg = DeclareLaunchArgument("real_speed", default_value="40")
    real_rate_arg = DeclareLaunchArgument("real_rate_hz", default_value="15.0")

    target = LaunchConfiguration("target")
    sim_enabled = PythonExpression(["'", target, "' in ('sim', 'both')"])
    real_enabled = PythonExpression(["'", target, "' in ('real', 'both')"])

    # ----------------------- robot_state_publisher --------------------------- #
    robot_description = ParameterValue(Command(["xacro ", urdf_path]), value_type=str)
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{"robot_description": robot_description, "use_sim_time": True}],
    )

    # ------------------------------ Gazebo sim ------------------------------- #
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("ros_gz_sim"), "launch", "gz_sim.launch.py"
            )
        ),
        launch_arguments={"gz_args": "-r empty.sdf", "on_exit_shutdown": "true"}.items(),
        condition=IfCondition(sim_enabled),
    )

    spawn_entity = Node(
        package="ros_gz_sim", executable="create", output="screen",
        arguments=["-topic", "robot_description", "-name", "mycobot_320", "-z", "0.0"],
        condition=IfCondition(sim_enabled),
    )

    # Gazebo → ROS2 bridges: /clock (CRITICAL for ros2_control use_sim_time)
    # and /joint_states (mapped to /gz/joint_states to avoid collision with
    # the joint_state_broadcaster's /joint_states).
    gz_bridge = Node(
        package="ros_gz_bridge", executable="parameter_bridge", output="screen",
        arguments=[
            "/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock",
            "/world/empty/model/mycobot_320/joint_state@sensor_msgs/msg/JointState[gz.msgs.Model",
        ],
        remappings=[
            ("/world/empty/model/mycobot_320/joint_state", "/gz/joint_states"),
        ],
        condition=IfCondition(sim_enabled),
    )

    # -------------- ros2_control controller spawners (sim only) -------------- #
    joint_state_broadcaster = Node(
        package="controller_manager", executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
        output="screen",
        condition=IfCondition(sim_enabled),
    )
    mycobot_controller_spawner = Node(
        package="controller_manager", executable="spawner",
        arguments=["mycobot_controller", "--controller-manager", "/controller_manager",
                   "--param-file", controller_cfg],
        output="screen",
        condition=IfCondition(sim_enabled),
    )

    # --------------------------- rosbridge_server ---------------------------- #
    rosbridge_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rosbridge_server"),
                "launch", "rosbridge_websocket_launch.xml",
            )
        ),
        condition=IfCondition(LaunchConfiguration("rosbridge")),
    )

    # -------------------------- bridge_tour (real) --------------------------- #
    bridge_tour = Node(
        package="mycobot_gateway", executable="bridge_tour",
        output="screen",
        parameters=[{
            "pi_ip": LaunchConfiguration("pi_ip"),
            "pi_port": LaunchConfiguration("pi_port"),
        }],
        condition=IfCondition(real_enabled),
    )

    # ----------------- trajectory → JSON send_angles forwarder --------------- #
    trajectory_bridge = Node(
        package="mycobot_gateway", executable="trajectory_to_robot_bridge",
        output="screen",
        parameters=[{
            "trajectory_topic": "/mycobot_controller/joint_trajectory",
            "out_topic": "/to_robot",
            "speed": LaunchConfiguration("real_speed"),
            "rate_hz": LaunchConfiguration("real_rate_hz"),
            "deadband_deg": 1.0,
            "enable": True,
        }],
        condition=IfCondition(real_enabled),
    )

    return LaunchDescription([
        target_arg, pi_ip_arg, pi_port_arg, rosbridge_arg, real_speed_arg, real_rate_arg,
        robot_state_publisher,
        gz_sim,
        spawn_entity,
        gz_bridge,
        # Controllers come up after Gazebo has spawned the robot
        TimerAction(period=3.0, actions=[joint_state_broadcaster]),
        TimerAction(period=3.5, actions=[mycobot_controller_spawner]),
        rosbridge_launch,
        bridge_tour,
        # Start the trajectory bridge last so bridge_tour has time to connect
        TimerAction(period=2.0, actions=[trajectory_bridge]),
    ])
