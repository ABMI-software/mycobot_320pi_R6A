#!/usr/bin/env python3
"""One-off: does moveit_py's RobotState.set_from_ik() actually work for our
arm? Test before building the full pick-and-place sequence around it."""
import os
import subprocess

import yaml
from ament_index_python.packages import get_package_share_directory
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
from geometry_msgs.msg import Pose


def load_yaml(package_name, relative_path):
    with open(os.path.join(get_package_share_directory(package_name), relative_path)) as f:
        return yaml.safe_load(f)


def build_moveit():
    desc_share = get_package_share_directory("mycobot_description")
    moveit_share = get_package_share_directory("mycobot_moveit_config")
    urdf_path = os.path.join(desc_share, "urdf", "320_pi", "mycobot_pro_320_pi_gazebo.urdf")
    srdf_path = os.path.join(moveit_share, "config", "mycobot.srdf")

    robot_description = subprocess.check_output(["xacro", urdf_path]).decode()
    with open(srdf_path) as f:
        robot_description_semantic = f.read()

    kinematics_yaml = load_yaml("mycobot_moveit_config", "config/kinematics.yaml")
    joint_limits_yaml = load_yaml("mycobot_moveit_config", "config/joint_limits.yaml")
    with open(os.path.join(moveit_share, "config", "ompl_planning.yaml")) as f:
        ompl_params = yaml.safe_load(f)["/**"]["ros__parameters"]

    config_dict = {
        "robot_description": robot_description,
        "robot_description_semantic": robot_description_semantic,
        "robot_description_kinematics": kinematics_yaml,
        "robot_description_planning": joint_limits_yaml,
        "planning_pipelines": {"pipeline_names": ompl_params["planning_pipelines"]},
        **{k: v for k, v in ompl_params.items() if k != "planning_pipelines"},
        "use_sim_time": False,
    }
    moveit = MoveItPy(node_name="test_ik", config_dict=config_dict)
    return moveit.get_robot_model()


print("Building MoveItPy robot model...")
robot_model = build_moveit()
print("Robot model ready. Group names:", robot_model.joint_model_group_names)
print("End effector links for group 'arm':",
      robot_model.get_joint_model_group("arm").link_model_names)

state = RobotState(robot_model)
state.set_to_default_values()

target = Pose()
target.position.x = 0.25
target.position.y = 0.0
target.position.z = 0.10
# top-down: gripper pointing straight down at the table
target.orientation.x = 1.0
target.orientation.y = 0.0
target.orientation.z = 0.0
target.orientation.w = 0.0

print("\n--- attempting set_from_ik ---")
try:
    ok = state.set_from_ik("arm", target, "link6", timeout=1.0)
except TypeError as e:
    print("TypeError with that signature:", e)
    print("retrying with default timeout")
    ok = state.set_from_ik("arm", target, "link6")

print("IK success:", ok)
if ok:
    q = state.get_joint_group_positions("arm")
    print("joint solution:", list(q))
    print("J3 (index 2):", q[2], "-- elbow-up (J3<0)?" , q[2] < 0)

    # cross-check: run FK on this solution and see if it reproduces the target
    state.update()
    pose_check = state.get_pose("link6")
    print("FK of IK solution:", pose_check.position.x, pose_check.position.y, pose_check.position.z)
