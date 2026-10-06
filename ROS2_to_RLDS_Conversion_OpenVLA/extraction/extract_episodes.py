#!/usr/bin/env python3
"""ROS2 -> intermediate .npy extraction for the gazebo_to_lerobot -> RLDS conversion.

Implements steps 1-6 of the conversion (topic extraction, sim-time sync,
FK, pos/rot deltas, gripper channel, image resize+RGB). Must run inside
the gazebo_to_lerobot container (needs rosbag2_py + moveit_py); the RLDS/TFDS
side (rlds_builder container, step 7 onward) only ever touches the .npy
files this script writes -- it has no ROS2 dependency at all.

Usage (inside gazebo_to_lerobot, ROS2 + workspace sourced):
    python3 extract_episodes.py

Per-episode output: one .npy (list of per-timestep dicts, allow_pickle),
matching the shape example_dataset_dataset_builder.py already expects,
so the real dataset_builder.py barely differs from the upstream example.
"""
import os

import cv2
import numpy as np
import rosbag2_py
from ament_index_python.packages import get_package_share_directory
from moveit.core.robot_state import RobotState
from moveit.planning import MoveItPy
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation as Rot
import subprocess
import yaml

# ---------------------------------------------------------------- config
CAMERA_TOPIC = "/synth_camera/image/compressed"
JOINT_STATES_TOPIC = "/joint_states"
ARM_JOINTS = [
    "joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
    "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6",
]
GRIPPER_JOINT = "gripper_controller"
TARGET_SIZE = 224  # matches the earlier LeRobot conversion's resolution

EPISODES = [
    {
        "bag": "/workspace/datasets/bags/1789171382_864718428",
        "out": "/workspace/rlds_extraction_out/episode_0.npy",
        # These two bags are scripted keyboard motion with no object/goal --
        # there is no real task instruction to recover from the source data
        # (the earlier LeRobot conversion of the same bags also carried an
        # empty task string). Naming this honestly rather than inventing a
        # task the episode doesn't actually show.
        "instruction": "move the arm through a scripted joint trajectory (no object, mechanics test)",
    },
    {
        "bag": "/workspace/datasets/bags/1789172093_868865993",
        "out": "/workspace/rlds_extraction_out/episode_1.npy",
        "instruction": "move the arm through a scripted joint trajectory (no object, mechanics test)",
    },
]


def load_yaml(package_name, relative_path):
    with open(os.path.join(get_package_share_directory(package_name), relative_path)) as f:
        return yaml.safe_load(f)


def build_moveit():
    """Same params as mycobot_moveit_config/launch/move_group.launch.py,
    minus the parts MoveItPy doesn't need for pure FK (trajectory
    execution / controller manager); see test_fk.py for how this was
    worked out (MoveItPy refuses to start without a loaded planning
    pipeline even though we never call it)."""
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
    moveit = MoveItPy(node_name="extract_episodes", config_dict=config_dict)
    return moveit.get_robot_model()


def read_bag(bag_path):
    """Single pass: return sorted (t, {joint_name: pos}) list for
    /joint_states and sorted (t, compressed_bytes, format_str) list for the
    camera topic. t is each message's OWN header.stamp (already sim time --
    confirmed empirically: header stamps track /clock, bag receive time
    does not, see the runbook conversation for the ~10x RTF finding)."""
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_path, storage_id="mcap"),
                rosbag2_py.ConverterOptions("", ""))
    type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}

    joint_samples = []
    camera_samples = []
    while reader.has_next():
        topic, data, _recv_t = reader.read_next()
        if topic == JOINT_STATES_TOPIC:
            msg = deserialize_message(data, get_message(type_map[topic]))
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            joint_samples.append((t, dict(zip(msg.name, msg.position))))
        elif topic == CAMERA_TOPIC:
            msg = deserialize_message(data, get_message(type_map[topic]))
            t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            camera_samples.append((t, bytes(msg.data), msg.format))

    joint_samples.sort(key=lambda x: x[0])
    camera_samples.sort(key=lambda x: x[0])
    return joint_samples, camera_samples


def interpolate_joints(joint_samples, t):
    """Linear-interpolate every joint angle onto camera timestamp t.
    Never the reverse (never interpolate images) -- images are held at
    their own native timestamp, joints are interpolated onto it. Clamps
    (holds) at the ends instead of extrapolating."""
    times = [s[0] for s in joint_samples]
    if t <= times[0]:
        return joint_samples[0][1]
    if t >= times[-1]:
        return joint_samples[-1][1]
    import bisect
    i = bisect.bisect_right(times, t) - 1
    t0, pos0 = joint_samples[i]
    t1, pos1 = joint_samples[i + 1]
    alpha = (t - t0) / (t1 - t0)
    return {j: pos0[j] + alpha * (pos1[j] - pos0[j]) for j in pos0}


def fk_pose(robot_model, arm_positions):
    state = RobotState(robot_model)
    state.set_joint_group_positions("arm", arm_positions)
    state.update()
    pose = state.get_pose("link6")
    pos = np.array([pose.position.x, pose.position.y, pose.position.z])
    quat = np.array([pose.orientation.x, pose.orientation.y,
                      pose.orientation.z, pose.orientation.w])
    return pos, quat


def decode_and_resize(jpeg_bytes, fmt_str):
    # Colour order, checked empirically, not assumed (see
    # extraction/check_color_order.py): this camera's CompressedImage
    # `format` field reads 'rgb8; jpeg compressed bgr8' -- the trailing
    # token is ROS's compressed_image_transport recording that it
    # converted to bgr8 before handing pixels to OpenCV's BGR-native JPEG
    # encoder. So cv2.imdecode's output is genuinely BGR (matches ROS
    # convention), and the swap to RGB below is required, not conditional.
    assert "bgr8" in fmt_str, f"unexpected CompressedImage format {fmt_str!r}, re-check colour order"
    arr = np.frombuffer(jpeg_bytes, dtype=np.uint8)
    img = cv2.imdecode(arr, cv2.IMREAD_COLOR)
    img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

    h, w = img.shape[:2]
    scale = TARGET_SIZE / min(h, w)
    new_w, new_h = round(w * scale), round(h * scale)
    img = cv2.resize(img, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    top = (new_h - TARGET_SIZE) // 2
    left = (new_w - TARGET_SIZE) // 2
    img = img[top:top + TARGET_SIZE, left:left + TARGET_SIZE]
    return img


def extract_episode(robot_model, bag_path, instruction):
    joint_samples, camera_samples = read_bag(bag_path)
    print(f"  {bag_path}: {len(joint_samples)} joint_states, {len(camera_samples)} camera frames")
    print(f"  camera format string (first msg): {camera_samples[0][2]!r}")

    poses = []  # (pos, quat, gripper) per camera frame, sim-time ordered
    images = []
    for t, jpeg_bytes, fmt_str in camera_samples:
        positions = interpolate_joints(joint_samples, t)
        arm_pos = [positions[j] for j in ARM_JOINTS]
        gripper_pos = positions.get(GRIPPER_JOINT, 0.0)
        pos, quat = fk_pose(robot_model, arm_pos)
        poses.append((pos, quat, gripper_pos))
        images.append(decode_and_resize(jpeg_bytes, fmt_str))

    n = len(poses)
    steps = []
    for i in range(n):
        pos_i, quat_i, grip_i = poses[i]
        state = np.concatenate([pos_i, quat_i, [grip_i]]).astype(np.float32)  # 8-dim

        if i < n - 1:
            pos_j, quat_j, _ = poses[i + 1]
            d_pos = (pos_j - pos_i).astype(np.float32)
            # rotation delta as small-angle Euler xyz: R_delta = R_i^-1 R_j
            r_i = Rot.from_quat(quat_i)
            r_j = Rot.from_quat(quat_j)
            d_rot = (r_i.inv() * r_j).as_euler("xyz").astype(np.float32)
        else:
            # No "next" state for the last frame -- zero delta, matches the
            # common OXE convention of a zero final action rather than
            # extrapolating motion that never happened.
            d_pos = np.zeros(3, dtype=np.float32)
            d_rot = np.zeros(3, dtype=np.float32)
        action = np.concatenate([d_pos, d_rot, [grip_i]]).astype(np.float32)  # 7-dim, gripper is absolute not delta (matches bridge_oxe convention)

        steps.append({
            "image": images[i],
            "state": state,
            "action": action,
            "language_instruction": instruction,
            "is_first": i == 0,
            "is_last": i == n - 1,
            "is_terminal": i == n - 1,
            "discount": 1.0,
            "reward": 1.0 if i == n - 1 else 0.0,
        })
    return steps


def main():
    os.makedirs("/workspace/rlds_extraction_out", exist_ok=True)
    print("Building MoveItPy robot model (one-time, ~15-30s)...")
    robot_model = build_moveit()
    print("Robot model ready.\n")

    for ep in EPISODES:
        print(f"Extracting {ep['bag']} ...")
        steps = extract_episode(robot_model, ep["bag"], ep["instruction"])
        np.save(ep["out"], steps, allow_pickle=True)
        print(f"  wrote {ep['out']} ({len(steps)} steps)\n")

    print("Done.")


if __name__ == "__main__":
    main()
