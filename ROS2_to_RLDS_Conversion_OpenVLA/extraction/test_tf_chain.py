#!/usr/bin/env python3
"""Compose base->link6 from the bag's own per-joint /tf messages (as
robot_state_publisher/Gazebo computed them at record time) at the exact
same instant as a /joint_states sample, to cross-check our own MoveItPy FK
against an independent ground truth already present in the bag.
"""
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
from scipy.spatial.transform import Rotation as R
import numpy as np

CHAIN = [("base", "link1"), ("link1", "link2"), ("link2", "link3"),
         ("link3", "link4"), ("link4", "link5"), ("link5", "link6")]

bag = "/workspace/datasets/bags/1789171382_864718428"
reader = rosbag2_py.SequentialReader()
reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id="mcap"), rosbag2_py.ConverterOptions("", ""))
type_map = {t.name: t.type for t in reader.get_all_topics_and_types()}

joint_states_by_time = []
tf_snapshot = None
tf_time = None

while reader.has_next():
    topic, data, t = reader.read_next()
    if topic == "/joint_states" and len(joint_states_by_time) < 5:
        msg = deserialize_message(data, get_message(type_map[topic]))
        stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        joint_states_by_time.append((stamp, dict(zip(msg.name, msg.position))))
    if topic == "/tf" and tf_snapshot is None:
        msg = deserialize_message(data, get_message(type_map[topic]))
        transforms = {(tr.header.frame_id, tr.child_frame_id): tr for tr in msg.transforms}
        if all(k in transforms for k in CHAIN):
            tf_snapshot = transforms
            tf_time = msg.transforms[0].header.stamp.sec + msg.transforms[0].header.stamp.nanosec * 1e-9
    if tf_snapshot is not None and len(joint_states_by_time) >= 5:
        break

print("tf snapshot time:", tf_time)
print("joint_states samples (time, arm joints only):")
JOINT_NAMES = ["joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
               "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6"]
best = min(joint_states_by_time, key=lambda x: abs(x[0] - tf_time))
print("closest joint_states time:", best[0], "(delta", best[0]-tf_time, "s)")
positions = [best[1][j] for j in JOINT_NAMES]
print("positions:", positions)

# Compose base->link6 by chaining translation+rotation
T = np.eye(4)
for parent, child in CHAIN:
    tr = tf_snapshot[(parent, child)].transform
    t_vec = np.array([tr.translation.x, tr.translation.y, tr.translation.z])
    q = [tr.rotation.x, tr.rotation.y, tr.rotation.z, tr.rotation.w]
    Ti = np.eye(4)
    Ti[:3, :3] = R.from_quat(q).as_matrix()
    Ti[:3, 3] = t_vec
    T = T @ Ti

pos = T[:3, 3]
quat = R.from_matrix(T[:3, :3]).as_quat()
print("\nComposed base->link6 from bag's own /tf chain:")
print("position:", pos)
print("quat (x,y,z,w):", quat)
