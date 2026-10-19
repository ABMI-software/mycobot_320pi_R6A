#!/usr/bin/env python3
"""Replay a recorded episode in Gazebo from the dataset's actions alone.

  replay_episode.py <episode.npz> [--out result.json]

The .npz comes from export_lerobot_episode.py (LeRobotDataset) or
export_rlds_episode.py (tfds). Nothing of sim_sorting_grasp runs: the arm and
the gripper receive only what the dataset stores, on the simulation clock.

  lerobot  action = commanded 6 joints + gripper_controller, sent as is.
  rlds     action = OpenVLA's [dx, dy, dz, droll, dpitch, dyaw, gripper_open]
           relative to the stored link6 pose; every joint command is rebuilt
           by inverse kinematics (MoveIt KDL, the model the extraction's FK
           used), seeded with the scene's observation pose -- where tri_yolo
           puts the arm -- then the previous solution. joint_action is read
           only to report the round-trip error. MoveItPy cannot share a
           process with a plain rclpy node, so the IK runs in a child process
           (--ik) before this node starts.

Before replaying, the arm goes to the episode's first pose (as the original
run did, from the observation pose). Afterwards: measured joints vs the
dataset's states, and where the target ended up (Gazebo ground truth, the
same in-bin test as sim_sorting_grasp).
"""
import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.parameter import Parameter
from scipy.spatial.transform import Rotation as Rot
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from vision_msgs.msg import Detection3DArray

sys.path.insert(0, str(Path(__file__).resolve().parent / 'rlds'))

ARM_JOINTS = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
              'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
GRIPPER = 'gripper_controller'
# sim_sorting_grasp's command layout and limits: [-a, a, a, -a, -a, a] closes by a.
GRIPPER_LIMITS = [(-1.20, 0.10), (-0.10, 1.20), (-1.10, 1.10), (-1.10, 1.10),
                  (-1.20, 0.10), (-0.10, 1.20)]
GRIPPER_CLOSED_RAD = 1.11
PIECE_BIN = {'red cube': ('cube_rouge', 'bac_rouge'), 'blue cube': ('cube_bleu', 'bac_bleu'),
             'green cylinder': ('cylindre_vert', 'bac_vert'), 'yellow box': ('pave_jaune', 'bac_jaune')}
START_MOVE_S = 3.0
OBSERVATION_Q_DEG = (0.0, 60.0, -70.0, 0.0, 0.0, 0.0)


def gripper_command(g):
    a = -g
    return [float(np.clip(v, lo, hi)) for v, (lo, hi)
            in zip([-a, a, a, -a, -a, a], GRIPPER_LIMITS)]


def rlds_to_joints(data, seed_q):
    """OpenVLA actions -> joint commands by IK; also the start pose's joints."""
    from extract_episodes import build_moveit
    from moveit.core.robot_state import RobotState
    model = build_moveit()
    state = RobotState(model)

    def ik(pos, rot, seed):
        state.set_joint_group_positions('arm', np.asarray(seed, float))
        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = map(float, pos)
        q = rot.as_quat()
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, q)
        if not state.set_from_ik('arm', pose, 'link6', 0.2):
            raise RuntimeError(f'no IK solution at {np.round(pos, 4)}')
        return np.array(state.get_joint_group_positions('arm'))

    s, a = data['state'], data['action']
    start_q = ik(s[0, :3], Rot.from_quat(s[0, 3:7]), seed_q)
    targets, q = [], start_q
    for i in range(len(a)):
        rot = Rot.from_quat(s[i, 3:7]) * Rot.from_euler('xyz', a[i, 3:6])
        q = ik(s[i, :3] + a[i, :3], rot, q)
        targets.append(q)
    targets = np.array(targets)
    err = np.degrees(np.abs(targets - data['joint_action'][:, :6]))
    gripper = -(1.0 - a[:, 6]) * GRIPPER_CLOSED_RAD
    start_g = -(1.0 - s[0, 7]) * GRIPPER_CLOSED_RAD
    return start_q, start_g, targets, gripper, err


class Replayer(Node):
    def __init__(self):
        super().__init__('replay_episode', parameter_overrides=[Parameter('use_sim_time', value=True)])
        self.q = None
        self.log = []
        self.objects = {}
        self.arm = self.create_publisher(JointTrajectory, '/mycobot_controller/joint_trajectory', 10)
        self.grip = self.create_publisher(Float64MultiArray, '/gripper_position_controller/commands', 10)
        self.create_subscription(JointState, '/joint_states', self.on_joints, 50)
        self.create_subscription(Detection3DArray, '/validation/gt/objects', self.on_objects, 10)

    def now(self):
        return self.get_clock().now().nanoseconds / 1e9

    def on_joints(self, msg):
        if all(j in msg.name for j in ARM_JOINTS + [GRIPPER]):
            self.q = [msg.position[msg.name.index(j)] for j in ARM_JOINTS + [GRIPPER]]
            self.log.append((self.now(), self.q))

    def on_objects(self, msg):
        self.objects = {d.results[0].hypothesis.class_id: d.results[0].pose.pose.position
                        for d in msg.detections}

    def spin_until(self, t):
        while rclpy.ok() and self.now() < t:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_ready(self):
        while rclpy.ok() and (self.q is None or self.now() == 0.0):
            rclpy.spin_once(self, timeout_sec=0.2)

    def send(self, points):
        msg = JointTrajectory(joint_names=ARM_JOINTS)
        for t, q in points:
            msg.points.append(JointTrajectoryPoint(
                positions=[float(v) for v in q],
                time_from_start=Duration(sec=int(t), nanosec=int(round((t % 1) * 1e9)))))
        self.arm.publish(msg)


def ik_child(npz, out):
    data = dict(np.load(npz))
    start_q, start_g, targets, gripper, err = rlds_to_joints(data, np.radians(OBSERVATION_Q_DEG))
    np.savez(out, start_q=start_q, start_g=start_g, targets=targets, gripper=gripper, err=err)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('npz')
    ap.add_argument('--out', default=None)
    ap.add_argument('--ik', default=None, help=argparse.SUPPRESS)
    args = ap.parse_args()
    if args.ik:
        ik_child(args.npz, args.ik)
        return 0
    data = dict(np.load(args.npz))
    source = str(data['source'])
    instruction = str(data['instruction'])
    piece, bin_ = next(v for k, v in PIECE_BIN.items() if k in instruction)

    print(f'[{source}] "{instruction}" -- {len(data["action"])} steps', flush=True)
    if source == 'lerobot':
        start_q, start_g = data['state'][0, :6], data['state'][0, 6]
        targets, gripper = data['action'][:, :6], data['action'][:, 6]
        ik_err = None
    else:
        ik_out = str(Path(args.npz).with_suffix('.ik.npz'))
        subprocess.run([sys.executable, __file__, args.npz, '--ik', ik_out], check=True,
                       stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
        ik = dict(np.load(ik_out))
        start_q, start_g, targets, gripper, ik_err = (ik['start_q'], float(ik['start_g']),
                                                      ik['targets'], ik['gripper'], ik['err'])
        print(f'[rlds] {len(targets)} joint commands rebuilt by IK; round trip vs commanded '
              f'joints: max {ik_err.max():.3f} deg, median {np.median(ik_err):.4f} deg', flush=True)

    rclpy.init()
    node = Replayer()
    node.wait_ready()

    node.grip.publish(Float64MultiArray(data=gripper_command(start_g)))
    node.send([(START_MOVE_S, start_q)])
    node.spin_until(node.now() + START_MOVE_S + 1.0)
    print(f'[{source}] at the episode start pose -- replaying', flush=True)

    dt = 1.0 / float(data['fps'])
    t0 = node.now()
    node.log.clear()
    node.send([((i + 1) * dt, q) for i, q in enumerate(targets)])
    last_g = None
    for i, g in enumerate(gripper):
        node.spin_until(t0 + i * dt)
        if last_g is None or abs(g - last_g) > 1e-3:
            node.grip.publish(Float64MultiArray(data=gripper_command(g)))
            print(f'[{source}] t={i * dt:5.1f}s gripper -> {g:+.3f} rad', flush=True)
            last_g = g
    node.spin_until(t0 + len(targets) * dt + 3.0)

    ts = np.array([t - t0 for t, _ in node.log])
    qs = np.array([q for _, q in node.log])
    ref = data['state'][:, :7] if source == 'lerobot' else data['joint_state']
    measured = np.stack([np.interp(np.arange(len(ref)) * dt + dt, ts, qs[:, j]) for j in range(7)], 1)
    track = np.degrees(np.abs(measured[:, :6] - ref[:, :6]))
    p, b = node.objects.get(piece), node.objects.get(bin_)
    result = {'source': source, 'instruction': instruction, 'steps': len(targets),
              'tracking_vs_dataset_state_deg': {'median': float(np.median(track)),
                                                'p95': float(np.percentile(track, 95)),
                                                'max': float(track.max())},
              'ik_round_trip_max_deg': None if ik_err is None else float(ik_err.max())}
    if p is not None and b is not None:
        dx, dy = p.x - b.x, p.y - b.y
        ok = abs(dx) < 0.047 and abs(dy) < 0.047 and p.z < 0.06
        result.update({'piece_in_bin': ok, 'offset_mm': [round(dx * 1000, 1), round(dy * 1000, 1)],
                       'piece_z_mm': round(p.z * 1000, 1)})
    print('=' * 62)
    print(f'  REPLAY [{source}]  {instruction}')
    print(f"  joints vs dataset state: median {result['tracking_vs_dataset_state_deg']['median']:.2f} deg, "
          f"p95 {result['tracking_vs_dataset_state_deg']['p95']:.2f}, "
          f"max {result['tracking_vs_dataset_state_deg']['max']:.2f}")
    if 'piece_in_bin' in result:
        print(f"  {piece}: {'IN ' + bin_ if result['piece_in_bin'] else 'NOT in ' + bin_}, "
              f"offset {result['offset_mm'][0]:+.0f}/{result['offset_mm'][1]:+.0f} mm, "
              f"z={result['piece_z_mm']:.0f} mm")
    print('=' * 62, flush=True)
    if args.out:
        Path(args.out).write_text(json.dumps(result, indent=2) + '\n')
    node.destroy_node()
    rclpy.shutdown()
    return 0 if result.get('piece_in_bin') else 1


if __name__ == '__main__':
    sys.exit(main())
