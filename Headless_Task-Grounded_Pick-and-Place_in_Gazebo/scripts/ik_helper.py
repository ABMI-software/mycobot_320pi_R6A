#!/usr/bin/env python3
"""ik_helper.py -- Cartesian -> joint-angle IK for the mycobot arm, via
moveit_py's RobotState (the same object already used for FK in the RLDS
conversion work). Verified working: FK-of-IK-solution reproduces the
requested pose to ~1e-7 m (see test_ik.py).

Enforces the elbow-up branch (J3 < 0) per the documented real-robot
finding that the elbow-down branch runs against the J2 limit with 0
degrees of margin, causing large closed-loop branch jumps. That finding
is from the physical robot's own pymycobot/send_coords IK, not this
sim's OMPL/KDL solver -- but rejecting elbow-down here costs nothing and
keeps every recorded episode on one consistent, well-understood branch,
which is the actual property we want (see Part 5.4's "approach from one
consistent direction" point).
"""
import os
import subprocess

import yaml
from ament_index_python.packages import get_package_share_directory
from geometry_msgs.msg import Pose
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy

ARM_JOINTS = [
    "joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
    "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6",
]

# IK/FK target frame: gripper_base, NOT link6. Found the hard way -- link6
# to the actual gripper fingertips (gripper_left1/gripper_right1) is
# ~12cm, and treating link6 itself as "the gripper" while targeting
# block_z+0.01 for it drove the real fingertips underground, producing a
# hard physical clamp regardless of command source (confirmed via a raw
# joint_trajectory publish bypassing MoveIt2/OMPL entirely -- same
# clamp). gripper_base is a fixed STATIC transform from link6 (confirmed
# via /tf_static: translation (0, -0.007, 0.056), so it moves rigidly
# with link6 regardless of the gripper's own open/close articulation,
# unlike the fingertip frames -- a stable, predictable reference point
# roughly at the base of the gripper mechanism.
TARGET_LINK = "gripper_base"

# Top-down, for TARGET_LINK (gripper_base) specifically -- NOT the (1,0,0,0)
# that was correct for link6. gripper_base's local frame is rotated ~90
# degrees from link6's (per /tf_static), so the same literal quaternion
# does not mean the same physical orientation for the two frames -- using
# link6's value here produced "no elbow-up IK solution" failures for
# otherwise-reachable points. Derived empirically: commanded a known-good
# top-down link6 joint configuration, then read gripper_base's actual
# resulting orientation via TF.
TOP_DOWN_ORIENTATION = (-0.6573799189762979, -0.11357491738209825,
                         -0.11353891545027413, 0.7362481205047255)  # (x, y, z, w)


def _load_yaml(package_name, relative_path):
    with open(os.path.join(get_package_share_directory(package_name), relative_path)) as f:
        return yaml.safe_load(f)


def build_moveit():
    """Same params as mycobot_moveit_config/launch/move_group.launch.py,
    minus the trajectory-execution/controller-manager parts MoveItPy
    doesn't need for pure FK/IK -- see test_fk.py for why this exact
    shape is required (MoveItPy refuses to start without a loadable
    planning pipeline even though none is ever invoked here).

    Returns the MoveItPy instance itself, not just its RobotModel.
    CRITICAL: the caller must keep this object alive for as long as any
    other rclpy activity (a separate Node, spin_once, etc.) needs to
    happen in the same process. Letting it get garbage-collected tears
    down its internal rclcpp executor/context, which was found (the
    hard way, via a script that hung indefinitely with near-zero CPU
    progress) to break rclpy's ability to receive any further messages
    in that process -- fine in a pure FK/IK script that does nothing
    else afterward, fatal in anything that also runs its own Node."""
    desc_share = get_package_share_directory("mycobot_description")
    moveit_share = get_package_share_directory("mycobot_moveit_config")
    urdf_path = os.path.join(desc_share, "urdf", "320_pi", "mycobot_pro_320_pi_gazebo.urdf")
    srdf_path = os.path.join(moveit_share, "config", "mycobot.srdf")

    robot_description = subprocess.check_output(["xacro", urdf_path]).decode()
    with open(srdf_path) as f:
        robot_description_semantic = f.read()

    kinematics_yaml = _load_yaml("mycobot_moveit_config", "config/kinematics.yaml")
    joint_limits_yaml = _load_yaml("mycobot_moveit_config", "config/joint_limits.yaml")
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
    return MoveItPy(node_name="ik_helper", config_dict=config_dict)


def ik_for_point(robot_model, x, y, z, orientation=TOP_DOWN_ORIENTATION, max_seeds=8, seed_q=None):
    """Solve joint angles for link6 at (x, y, z), top-down by default.
    Rejects elbow-down solutions (J3 >= 0) and retries from a fresh
    random seed, up to max_seeds times. Returns a list of 6 joint
    positions in ARM_JOINTS order, or None if no elbow-up solution was
    found within the seed budget.

    seed_q, if given, seeds the FIRST attempt with a specific joint
    configuration (typically the previous waypoint's solution) instead
    of the default all-zero pose. Numerical IK converges to whichever
    solution is nearest the seed, so this is what keeps a sequence of
    calls on one consistent branch -- solving each waypoint from
    scratch (always seeded at default) was found to land on visibly
    different, larger-swing branches for the same or nearby targets
    (see the spec's own "approach from one consistent direction, never
    mix" requirement)."""
    target = Pose()
    target.position.x, target.position.y, target.position.z = x, y, z
    (target.orientation.x, target.orientation.y,
     target.orientation.z, target.orientation.w) = orientation

    state = RobotState(robot_model)
    if seed_q is not None:
        state.set_joint_group_positions("arm", seed_q)
    else:
        state.set_to_default_values()

    for attempt in range(max_seeds):
        if attempt > 0:
            state.set_to_random_positions(robot_model.get_joint_model_group("arm"))
        ok = state.set_from_ik("arm", target, TARGET_LINK, timeout=1.0)
        if not ok:
            continue
        q = list(state.get_joint_group_positions("arm"))
        if q[2] < 0:  # J3 (elbow) must be negative -- elbow-up branch
            return q
    return None


def fk_check(robot_model, q):
    """Cross-check: FK of a joint solution, for verifying IK results."""
    state = RobotState(robot_model)
    state.set_joint_group_positions("arm", q)
    state.update()
    pose = state.get_pose(TARGET_LINK)
    return (pose.position.x, pose.position.y, pose.position.z)


if __name__ == "__main__":
    print("Building robot model...")
    _moveit = build_moveit()  # kept alive for the script's duration, see build_moveit's docstring
    model = _moveit.get_robot_model()
    print("Ready.\n")

    # Every Cartesian point the actual pick-and-place sequence will need:
    # block hover/grasp at 3 documented spawn variations, plate hover/place.
    test_points = {
        "block_A_hover (0.25,0.00)":  (0.25, 0.00, 0.10),
        "block_A_grasp":              (0.25, 0.00, 0.03),
        "block_B_hover (0.24,0.08)":  (0.24, 0.08, 0.10),
        "block_B_grasp":              (0.24, 0.08, 0.03),
        "block_C_hover (0.20,-0.05)": (0.20, -0.05, 0.10),
        "block_C_grasp":              (0.20, -0.05, 0.03),
        "plate_hover":                (0.20, -0.15, 0.144),
        "plate_place":                (0.20, -0.15, 0.074),
    }

    all_ok = True
    for name, (x, y, z) in test_points.items():
        q = ik_for_point(model, x, y, z)
        if q is None:
            print(f"{name:28s}  FAILED -- no elbow-up IK solution in seed budget")
            all_ok = False
            continue
        fx, fy, fz = fk_check(model, q)
        err = ((fx - x) ** 2 + (fy - y) ** 2 + (fz - z) ** 2) ** 0.5
        print(f"{name:28s}  J3={q[2]:+.3f}  FK-error={err * 1000:.4f} mm  "
              f"q={[round(v, 3) for v in q]}")
        if err > 0.001:
            print(f"  ^^ WARNING: FK-error exceeds 1mm")
            all_ok = False

    print("\nALL POINTS OK" if all_ok else "\nSOME POINTS FAILED -- see above")
