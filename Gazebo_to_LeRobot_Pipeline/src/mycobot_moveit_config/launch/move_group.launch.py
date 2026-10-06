"""Hand-written MoveIt2 move_group bringup for the MyCobot 320 Pi.

Deliberately not generated via moveit_setup_assistant (which requires an
interactive GUI wizard) or moveit_configs_utils (version-sensitive
conventions) -- loads the yaml files in mycobot_moveit_config/config/
directly and hands them to the move_group node, the same way MoveIt2
tutorials write a "your own robot" launch file by hand.

Usage:
  ros2 launch mycobot_moveit_config move_group.launch.py
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def load_yaml(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    with open(os.path.join(package_path, relative_path), "r") as f:
        return yaml.safe_load(f)


def generate_launch_description():
    desc_share = get_package_share_directory("mycobot_description")
    moveit_share = get_package_share_directory("mycobot_moveit_config")

    urdf_path = os.path.join(desc_share, "urdf", "320_pi", "mycobot_pro_320_pi_gazebo.urdf")
    srdf_path = os.path.join(moveit_share, "config", "mycobot.srdf")

    robot_description = {
        "robot_description": ParameterValue(Command(["xacro ", urdf_path]), value_type=str),
    }
    with open(srdf_path, "r") as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}

    kinematics_yaml = load_yaml("mycobot_moveit_config", "config/kinematics.yaml")
    joint_limits_yaml = load_yaml("mycobot_moveit_config", "config/joint_limits.yaml")

    # These two are passed as FILE PATHS, not loaded-and-rewrapped Python
    # dicts: both contain list-valued parameters (request_adapters, joints,
    # controller_names, planning_pipelines), and launch_ros's dict->YAML
    # writer tags Python lists as `!!python/tuple`, which ROS2's own
    # parameter parser can't read. See the header comments in each file.
    moveit_controllers_yaml_path = os.path.join(moveit_share, "config", "moveit_controllers.yaml")
    ompl_planning_yaml_path = os.path.join(moveit_share, "config", "ompl_planning.yaml")

    trajectory_execution = {
        "moveit_manage_controllers": True,
        "trajectory_execution.allowed_execution_duration_scaling": 1.2,
        "trajectory_execution.allowed_goal_duration_margin": 0.5,
        "trajectory_execution.allowed_start_tolerance": 0.01,
    }

    planning_scene_monitor_parameters = {
        "publish_planning_scene": True,
        "publish_geometry_updates": True,
        "publish_state_updates": True,
        "publish_transforms_updates": True,
    }

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            {"robot_description_kinematics": kinematics_yaml},
            {"robot_description_planning": joint_limits_yaml},
            ompl_planning_yaml_path,
            trajectory_execution,
            moveit_controllers_yaml_path,
            planning_scene_monitor_parameters,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([move_group_node])
