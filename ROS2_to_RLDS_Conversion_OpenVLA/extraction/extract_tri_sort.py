#!/usr/bin/env python3
"""Recorded tri_yolo sort episodes (rosetta bags) -> per-episode .npy for RLDS.

  python3 extract_tri_sort.py --episodes '/workspace/tri_sort/seed_*/ep_*.json' \
      --out /workspace/rlds_tri_sort/data/train

Runs in gazebo_to_lerobot (rosbag2_py + moveit_py, system Python). One .npy
per episode JSON written by record_tri_sort.py whose verdict is OK; the
builder in the rlds_builder container reads only these files.

Timeline: the top camera's own header stamps (10 Hz sim time); the right and
left frames are the ones with the same stamp (the cameras tick together).
Joint state and commanded action are linearly interpolated onto each stamp,
never the images.

Per step:
  image_top / image_right / image_left  224x224 RGB, resized WITHOUT crop: a
      centre crop of 640x480 cuts the board edge, where bins sit in the side views
  state  [x, y, z, qx, qy, qz, qw, gripper_open]  link6 pose (MoveIt FK) +
      gripper opening in [0, 1], 1 = open (OXE convention)
  joint_state  6 arm joints + gripper_controller (rad), as in the LeRobot dataset
  action  [dx, dy, dz, droll, dpitch, dyaw, gripper_open]  from the measured
      pose to the COMMANDED pose (FK of /tri_sort/commanded_action), rotation as
      small-angle Euler xyz of R_state^-1 R_command; gripper = commanded opening
  joint_action  the commanded 6 joints + gripper (rad)
"""
import argparse
import bisect
import glob
import json
from pathlib import Path

import cv2
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation as Rot

from extract_episodes import ARM_JOINTS, GRIPPER_JOINT, build_moveit, fk_pose

CAMERAS = {'top': '/synth_camera_top/image', 'right': '/synth_camera_right/image',
           'left': '/synth_camera_left/image'}
STATE_TOPIC = '/joint_states'
ACTION_TOPIC = '/tri_sort/commanded_action'
SIZE = 224
# gripper_controller runs from 0 (open) to -1.11 rad (faces touching).
GRIPPER_CLOSED_RAD = 1.11


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(path), storage_id='mcap'),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    joints = {STATE_TOPIC: [], ACTION_TOPIC: []}
    images = {name: {} for name in CAMERAS}
    by_topic = {topic: name for name, topic in CAMERAS.items()}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        if topic in joints:
            msg = deserialize_message(data, get_message(types[topic]))
            joints[topic].append((stamp(msg), dict(zip(msg.name, msg.position))))
        elif topic in by_topic:
            msg = deserialize_message(data, get_message(types[topic]))
            if msg.encoding != 'rgb8':
                raise ValueError(f'{topic}: encoding {msg.encoding}, expected rgb8')
            img = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.width, 3)
            images[by_topic[topic]][round(stamp(msg), 3)] = img
    for series in joints.values():
        series.sort(key=lambda s: s[0])
    return joints, images


def interpolate(series, t):
    times = [s[0] for s in series]
    if t <= times[0]:
        return series[0][1]
    if t >= times[-1]:
        return series[-1][1]
    i = bisect.bisect_right(times, t) - 1
    (t0, a), (t1, b) = series[i], series[i + 1]
    w = (t - t0) / (t1 - t0)
    return {j: a[j] + w * (b[j] - a[j]) for j in a}


def opening(gripper_rad):
    return float(np.clip(1.0 - abs(gripper_rad) / GRIPPER_CLOSED_RAD, 0.0, 1.0))


def extract(robot_model, meta):
    joints, images = read_bag(meta['bag_path'])
    stamps = sorted(images['top'])
    missing = {c: sum(t not in images[c] for t in stamps) for c in ('right', 'left')}
    if any(missing.values()):
        raise ValueError(f"{meta['bag_path']}: side frames without a top frame: {missing}")
    steps = []
    for i, t in enumerate(stamps):
        q = interpolate(joints[STATE_TOPIC], t)
        u = interpolate(joints[ACTION_TOPIC], t)
        q_vec = [q[j] for j in ARM_JOINTS] + [q[GRIPPER_JOINT]]
        u_vec = [u[j] for j in ARM_JOINTS] + [u[GRIPPER_JOINT]]
        pos, quat = fk_pose(robot_model, q_vec[:6])
        cpos, cquat = fk_pose(robot_model, u_vec[:6])
        d_rot = (Rot.from_quat(quat).inv() * Rot.from_quat(cquat)).as_euler('xyz')
        last = i == len(stamps) - 1
        steps.append({
            'image_top': cv2.resize(images['top'][t], (SIZE, SIZE), interpolation=cv2.INTER_AREA),
            'image_right': cv2.resize(images['right'][t], (SIZE, SIZE), interpolation=cv2.INTER_AREA),
            'image_left': cv2.resize(images['left'][t], (SIZE, SIZE), interpolation=cv2.INTER_AREA),
            'state': np.concatenate([pos, quat, [opening(q_vec[6])]]).astype(np.float32),
            'joint_state': np.array(q_vec, np.float32),
            'action': np.concatenate([cpos - pos, d_rot, [opening(u_vec[6])]]).astype(np.float32),
            'joint_action': np.array(u_vec, np.float32),
            'language_instruction': meta['instruction'],
            'is_first': i == 0, 'is_last': last, 'is_terminal': last,
            'discount': 1.0, 'reward': 1.0 if last else 0.0,
        })
    return steps


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--episodes', required=True, help='glob of ep_*.json from record_tri_sort.py')
    ap.add_argument('--out', type=Path, required=True)
    args = ap.parse_args()
    args.out.mkdir(parents=True, exist_ok=True)
    robot_model = build_moveit()
    for path in sorted(glob.glob(args.episodes)):
        meta = json.loads(Path(path).read_text())
        if not meta['ok']:
            print(f'{path}: verdict not OK, skipped ({meta["verdict"]})')
            continue
        steps = extract(robot_model, meta)
        name = f"seed{meta['seed']:03d}_{meta['piece']}_a{meta['attempt']}"
        np.save(args.out / f'episode_{name}.npy', steps, allow_pickle=True)
        a = np.stack([s['action'] for s in steps])
        print(f"{name}: {len(steps)} steps, |dpos| max {np.abs(a[:, :3]).max() * 1000:.1f} mm, "
              f"|drot| max {np.degrees(np.abs(a[:, 3:6]).max()):.1f} deg, "
              f"gripper action {a[:, 6].min():.2f}..{a[:, 6].max():.2f}")


if __name__ == '__main__':
    main()
