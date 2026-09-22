"""RViz with the MotionPlanning plugin, wired to the same MoveIt2 config
move_group.launch.py uses. Run move_group.launch.py first, in a separate
terminal.

Usage:
  ros2 launch mycobot_moveit_config moveit_rviz.launch.py
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
    rviz_config_path = os.path.join(moveit_share, "config", "moveit.rviz")

    robot_description = {
        "robot_description": ParameterValue(Command(["xacro ", urdf_path]), value_type=str),
    }
    with open(srdf_path, "r") as f:
        robot_description_semantic = {"robot_description_semantic": f.read()}

    kinematics_yaml = load_yaml("mycobot_moveit_config", "config/kinematics.yaml")
    joint_limits_yaml = load_yaml("mycobot_moveit_config", "config/joint_limits.yaml")
    # Passed as a file path, not a loaded dict -- see ompl_planning.yaml's
    # header comment (launch_ros mis-serializes list-valued dict params).
    ompl_planning_yaml_path = os.path.join(moveit_share, "config", "ompl_planning.yaml")

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", rviz_config_path],
        parameters=[
            robot_description,
            robot_description_semantic,
            {"robot_description_kinematics": kinematics_yaml},
            {"robot_description_planning": joint_limits_yaml},
            ompl_planning_yaml_path,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([rviz_node])
