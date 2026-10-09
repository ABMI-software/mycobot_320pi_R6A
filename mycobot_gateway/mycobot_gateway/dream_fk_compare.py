"""DREAM keypoints against the Gazebo joint angles — VALIDATION ONLY (protocol step 10).

For every DREAM result of every camera: the 7 keypoints projected from FK at
the joint angles Gazebo had when THAT image was rendered (the image stamp
travels in /dream_<cam>/keypoints, invariant I5; /joint_states is interpolated
on it), and the DREAM camera pose against the camera's true pose. World = base
(invariant I3), so the true T_cam<-base is the fixed camera of the URDF.

    /dream_<cam>/keypoints + /dream_<cam>/pose + /joint_states
        -> <log_dir>/dream_vs_fk.csv   (one row per keypoint and image)
        -> <log_dir>/dream_pose.csv    (one row per DREAM pose)

Nothing here feeds perception or planning.
"""

import csv
import os
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
import numpy as np
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray

from .joint_history import JointHistory
from .vision.sim_multicam_geometry import load_cameras

# mycobot_fk holds the DREAM labels (protocol I7: imported, never edited).
sys.path.insert(0, str(Path(os.path.realpath(__file__)).parents[2] / 'training' / 'dream'))
from mycobot_fk import forward_kinematics, KEYPOINT_NAMES  # noqa: E402

# Topic suffix of each camera's DREAM instance (tri_yolo.launch.py).
PREFIXES = {'synth_camera': '/dream_front', 'synth_camera_right': '/dream_right',
            'synth_camera_left': '/dream_left', 'synth_camera_top': '/dream_top'}
# An image farther than this from any joint sample is not compared.
MAX_JOINT_GAP_S = 0.2
KP_FIELDS = ('stamp_sim', 'seed', 'camera_id', 'keypoint', 'valid', 'u_dream', 'v_dream',
             'u_fk', 'v_fk', 'fk_in_image', 'du', 'dv', 'pixel_error', 'joint_gap_ms',
             'q1', 'q2', 'q3', 'q4', 'q5', 'q6', 'inference_ms')
POSE_FIELDS = ('stamp_sim', 'seed', 'camera_id', 'n_valid', 'dx', 'dy', 'dz', 't_error_mm',
               'r_error_deg')


def stamp_s(sec, nanosec):
    return sec + nanosec * 1e-9


def pose_error(T_true, T_est):
    """(dx, dy, dz) in m and rotation angle in degrees between two T_cam<-base."""
    d = T_est[:3, 3] - T_true[:3, 3]
    angle = Rotation.from_matrix(T_true[:3, :3].T @ T_est[:3, :3]).magnitude()
    return d, float(np.degrees(angle))


class DreamFkCompare(Node):
    def __init__(self):
        super().__init__('dream_fk_compare')
        self.declare_parameter('cameras', list(PREFIXES))
        self.declare_parameter('log_dir', '')
        self.declare_parameter('seed', -1)
        self.seed = int(self.get_parameter('seed').value)
        desc = get_package_share_directory('mycobot_description')
        cameras = load_cameras(
            os.path.join(desc, 'urdf', '320_pi', 'mycobot_pro_320_pi_gazebo.urdf'),
            {'camera_layout': 'dream50k'})
        self.cams = list(self.get_parameter('cameras').value)
        self.cameras = {cam: cameras[cam] for cam in self.cams}
        self.joints = JointHistory()
        self.n_valid = {}
        log_dir = Path(str(self.get_parameter('log_dir').value))
        self.kp_file = open(log_dir / 'dream_vs_fk.csv', 'w', newline='')
        self.pose_file = open(log_dir / 'dream_pose.csv', 'w', newline='')
        self.kp_csv = csv.DictWriter(self.kp_file, fieldnames=KP_FIELDS)
        self.pose_csv = csv.DictWriter(self.pose_file, fieldnames=POSE_FIELDS)
        self.kp_csv.writeheader()
        self.pose_csv.writeheader()
        self.create_subscription(JointState, '/joint_states', self.joints.add, 50)
        for cam in self.cams:
            self.create_subscription(Float64MultiArray, f'{PREFIXES[cam]}/keypoints',
                                     lambda m, c=cam: self.on_keypoints(c, m), 10)
            self.create_subscription(PoseStamped, f'{PREFIXES[cam]}/pose',
                                     lambda m, c=cam: self.on_pose(c, m), 10)
        self.get_logger().info(f'DREAM vs FK, {self.cams} -> {log_dir} (VALIDATION ONLY)')

    def on_keypoints(self, cam, msg):
        data = np.asarray(msg.data)
        uvv = data[:21].reshape(7, 3)
        t = stamp_s(int(data[21]), int(data[22]))
        self.n_valid[(cam, t)] = int(uvv[:, 2].sum())
        q, gap = self.joints.at(t)
        if gap > MAX_JOINT_GAP_S:
            return
        camera = self.cameras[cam]
        positions, _ = forward_kinematics(q)
        fk = camera.project(np.array([positions[k] for k in KEYPOINT_NAMES]))
        base = {'stamp_sim': f'{t:.3f}', 'seed': self.seed, 'camera_id': cam,
                'joint_gap_ms': round(gap * 1000, 1), 'inference_ms': round(float(data[23]), 1),
                **{f'q{i + 1}': round(float(np.degrees(v)), 3) for i, v in enumerate(q)}}
        for name, (u, v, valid), (uf, vf) in zip(KEYPOINT_NAMES, uvv, fk):
            row = {**base, 'keypoint': name.replace('mycobot320_', ''), 'valid': bool(valid),
                   'u_fk': round(float(uf), 2), 'v_fk': round(float(vf), 2),
                   'fk_in_image': bool(0 <= uf < camera.width and 0 <= vf < camera.height)}
            if valid:
                row.update({'u_dream': round(float(u), 2), 'v_dream': round(float(v), 2),
                            'du': round(float(u - uf), 2), 'dv': round(float(v - vf), 2),
                            'pixel_error': round(float(np.hypot(u - uf, v - vf)), 2)})
            self.kp_csv.writerow(row)
        self.kp_file.flush()

    def on_pose(self, cam, msg):
        t = stamp_s(msg.header.stamp.sec, msg.header.stamp.nanosec)
        p, o = msg.pose.position, msg.pose.orientation
        T_est = np.eye(4)
        T_est[:3, :3] = Rotation.from_quat([o.x, o.y, o.z, o.w]).as_matrix()
        T_est[:3, 3] = [p.x, p.y, p.z]
        d, angle = pose_error(np.linalg.inv(self.cameras[cam].world_from_optical), T_est)
        self.pose_csv.writerow({
            'stamp_sim': f'{t:.3f}', 'seed': self.seed, 'camera_id': cam,
            'n_valid': self.n_valid.pop((cam, t), ''),
            'dx': round(d[0] * 1000, 2), 'dy': round(d[1] * 1000, 2), 'dz': round(d[2] * 1000, 2),
            't_error_mm': round(float(np.linalg.norm(d)) * 1000, 2), 'r_error_deg': round(angle, 3)})
        self.pose_file.flush()

    def destroy_node(self):
        self.kp_file.close()
        self.pose_file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = DreamFkCompare()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
