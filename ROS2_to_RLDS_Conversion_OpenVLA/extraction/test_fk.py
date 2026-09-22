#!/usr/bin/env python3
"""One-off test: can moveit_py's RobotState compute FK for our arm?
Run inside gazebo_to_lerobot with: source /opt/ros/jazzy/setup.bash && source
/workspace/install/setup.bash && python3 test_fk.py
"""
import os
import subprocess

import yaml
from ament_index_python.packages import get_package_share_directory
from moveit.planning import MoveItPy


def load_yaml(package_name, relative_path):
    package_path = get_package_share_directory(package_name)
    with open(os.path.join(package_path, relative_path), "r") as f:
        return yaml.safe_load(f)


desc_share = get_package_share_directory("mycobot_description")
moveit_share = get_package_share_directory("mycobot_moveit_config")

urdf_path = os.path.join(desc_share, "urdf", "320_pi", "mycobot_pro_320_pi_gazebo.urdf")
srdf_path = os.path.join(moveit_share, "config", "mycobot.srdf")

robot_description = subprocess.check_output(["xacro", urdf_path]).decode()
with open(srdf_path) as f:
    robot_description_semantic = f.read()

kinematics_yaml = load_yaml("mycobot_moveit_config", "config/kinematics.yaml")
joint_limits_yaml = load_yaml("mycobot_moveit_config", "config/joint_limits.yaml")

# MoveItPy insists on loading planning pipelines even when all we want is
# RobotState/FK -- merge in the same ompl_planning.yaml content move_group
# loads (its "/**: ros__parameters:" section, flattened to top level, since
# config_dict is a flat param-name->value map, not a ros2 param YAML file).
ompl_share = get_package_share_directory("mycobot_moveit_config")
with open(os.path.join(ompl_share, "config", "ompl_planning.yaml")) as f:
    ompl_full = yaml.safe_load(f)
ompl_params = ompl_full["/**"]["ros__parameters"]

config_dict = {
    "robot_description": robot_description,
    "robot_description_semantic": robot_description_semantic,
    "robot_description_kinematics": kinematics_yaml,
    "robot_description_planning": joint_limits_yaml,
    "planning_pipelines": {"pipeline_names": ompl_params["planning_pipelines"]},
    **{k: v for k, v in ompl_params.items() if k != "planning_pipelines"},
    "use_sim_time": False,
}

print("Constructing MoveItPy...")
moveit = MoveItPy(node_name="test_fk", config_dict=config_dict)
print("MoveItPy constructed OK")

robot_model = moveit.get_robot_model()
print("Robot model name:", robot_model.name)
print("Joint model group names:", robot_model.joint_model_group_names)

robot_state = robot_model.robot_state if hasattr(robot_model, "robot_state") else None

from moveit.core.robot_state import RobotState
state = RobotState(robot_model)

JOINT_NAMES = [
    "joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
    "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6",
]

# Test 1: all-zero configuration
state.set_joint_group_positions("arm", [0.0] * 6)
state.update()
pose = state.get_pose("link6")
print("\n--- All-zero joint config ---")
print("link6 pose:", pose.position.x, pose.position.y, pose.position.z,
      "| quat:", pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w)

# Test 2: a real recorded pose from episode 1's first /joint_states sample
# (from earlier bag inspection): joint2_to_joint1=0.65, joint3=0.20,
# joint4=0.15, joint5=0.15, joint6=0.15, joint6output=0.15
real_positions = [0.6499999999973346, 0.20000000019207484, 0.15000000067088426,
                  0.1500000004956243, 0.14999999985116685, 0.15000000004570863]
state.set_joint_group_positions("arm", real_positions)
state.update()
pose2 = state.get_pose("link6")
print("\n--- Recorded joint config from bag episode 1 ---")
print("link6 pose:", pose2.position.x, pose2.position.y, pose2.position.z,
      "| quat:", pose2.orientation.x, pose2.orientation.y, pose2.orientation.z, pose2.orientation.w)

print("\nFK smoke test done.")
