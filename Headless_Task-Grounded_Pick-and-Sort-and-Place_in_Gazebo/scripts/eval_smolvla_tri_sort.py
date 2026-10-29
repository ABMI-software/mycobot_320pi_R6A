#!/usr/bin/env python3
"""Run a SmolVLA checkpoint on one tri_yolo seed in Gazebo and score it like the scripted sorter.

  eval_smolvla_tri_sort.py --checkpoint <pretrained_model> --seed 11 --out <dir>
                           [--start-pose observation|random] [--n-action-steps 50] [--max-steps 300]
                           [--step-locked]

For each piece, in the sorter's order: the arm goes where a recorded episode starts (the
observation pose, or a random pose drawn as for the random-start recordings), the gripper to
-0.28 (its first-frame value in all 40 episodes of seeds 1-10), then the policy drives:

  Gazebo paused -> 3 images + 7-value state -> 50-action chunk (about 10 s on this CPU)
  -> Gazebo resumed -> the first n_action_steps actions played at 10 Hz of sim time -> repeat

The pause keeps the scene still while the CPU computes, so the policy is judged on what it
learned, not on inference lag. Actions go to the same controllers that produced the recorded
ones: the 6 joints as JTC waypoints, the gripper value as data[0] of the 6-value command
[g, -g, -g, g, g, -g] (one value alone makes the controller switch itself off).

Scoring is Gazebo ground truth, which the policy never sees, with the sorter's test: picked when
the piece rises above 6 cm, placed when it ends within 47 mm of its bin's centre (x and y) below
6 cm. Writes <out>/result.json and <out>/trace_<piece>.jsonl (every step: state, action, tip;
piece positions about every 0.5 s of sim time).

--step-locked (step_lock.py): Gazebo only ever advances an exact number of physics steps, every
command is sent while it is paused, observations are taken on camera ticks, moves last a fixed
number of steps, and the policy runs with PyTorch's deterministic algorithms. Each piece then gets
a fingerprint (sha256 of its start state at full precision, of every step's state, action and
piece poses, and of every camera frame the policy saw), so two runs can be shown identical, or
where they part. Without it, the evaluator behaves exactly as it did for the 27/10 batch.

Inside gazebo_to_lerobot, ROS sourced, /workspace/venv_lerobot's python (it sees rclpy).
"""
import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from sensor_msgs.msg import Image
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from mycobot_gateway.sim_sorting_grasp import ARM_JOINTS, SimSortingGrasp, TRI_TARGETS, tool_tip
from mycobot_gateway.vision import tri_scene

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from smolvla_policy import CAMERAS, SmolVLA  # noqa: E402
import random_start  # noqa: E402
from preplace_pieces import model_poses  # noqa: E402
from step_lock import GRIPPER_SETTLE_STEPS, START_SETTLE_STEPS, StepLock, sha  # noqa: E402

INSTRUCTIONS = {
    'cube_rouge': 'pick up the red cube and place it in the red bin',
    'cube_bleu': 'pick up the blue cube and place it in the blue bin',
    'cylindre_vert': 'pick up the green cylinder and place it in the green bin',
    'pave_jaune': 'pick up the yellow box and place it in the yellow bin',
}
START_GRIPPER = -0.28
DT = 0.1                      # dataset fps 10
PICKED_Z = 0.06
BIN_HALF = 0.047
POSE_EVERY_S = 0.5


def decode(msg):
    img = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)[:, :msg.width * 3]
    img = img.reshape(msg.height, msg.width, 3)
    if msg.encoding == 'bgr8':
        return img[:, :, ::-1].copy()
    if msg.encoding != 'rgb8':
        raise ValueError(f'unexpected image encoding {msg.encoding}')
    return img.copy()


class PolicyRunner(StepLock, SimSortingGrasp):
    def __init__(self):
        super().__init__()
        self.init_step_lock()
        self.images = {}
        qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST, depth=1)
        for cam in CAMERAS:
            self.create_subscription(Image, f'/synth_camera_{cam}/image',
                                     lambda msg, c=cam: self.images.__setitem__(c, msg), qos)

    def now(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def gz_pause(self, pause):
        subprocess.run(['gz', 'service', '-s', f'/world/{self.world}/control',
                        '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '5000', '--req', f'pause: {str(pause).lower()}'],
                       check=True, capture_output=True, timeout=30)

    def state(self):
        return np.concatenate([np.radians(self.q_deg), [self.grip_pos]])

    def observe(self):
        """Paused: the newest frame of each camera and the state. Frames older than 0.25 s of sim
        time would show the scene before the last motion, so wait for fresh ones first."""
        deadline = time.monotonic() + 60.0
        while True:
            rclpy.spin_once(self, timeout_sec=0.05)
            stamps = {c: m.header.stamp.sec + m.header.stamp.nanosec * 1e-9 for c, m in self.images.items()}
            if len(stamps) == 3 and min(stamps.values()) > self.now() - 0.25:
                break
            if time.monotonic() > deadline:
                raise RuntimeError(f'no fresh camera frames (stamps {stamps}, sim {self.now():.2f})')
            self.gz_pause(False)
            self.spin_for(0.15)
            self.gz_pause(True)
        return {c: decode(m) for c, m in self.images.items()}, self.state()

    def set_gripper_value(self, g):
        """g is the recorded gripper action, data[0] of the controller command."""
        self.set_gripper(-float(g), settle=0.0)

    def play(self, actions, poses_out):
        traj = JointTrajectory()
        traj.joint_names = ARM_JOINTS
        for k, a in enumerate(actions):
            p = JointTrajectoryPoint()
            p.positions = [float(v) for v in a[:6]]
            t = DT * (k + 1)
            p.time_from_start = Duration(sec=int(t), nanosec=int(round((t % 1) * 1e9)))
            traj.points.append(p)
        t0 = self.now()
        self.pub_arm.publish(traj)
        steps, next_pose, grip = [], t0, None
        for k, a in enumerate(actions):
            if grip is None or abs(a[6] - grip) > 1e-4:
                grip = float(a[6])
                self.set_gripper_value(grip)
            self.spin_until(t0 + DT * (k + 1))
            steps.append({'t': round(self.now(), 3), 'state': self.state().round(5).tolist(),
                          'action': np.round(a, 5).tolist(),
                          'tip': np.round(tool_tip(self.q_deg), 4).tolist()})
            if self.now() >= next_pose:
                poses_out.append({'t': round(self.now(), 3), 'poses': {
                    n: np.round(p, 4).tolist() for n, p in self.object_poses().items()}})
                next_pose = self.now() + POSE_EVERY_S
        return steps

    def spin_until(self, t_sim):
        watchdog = time.monotonic() + 120.0 * self.wall_scale
        while rclpy.ok() and self.now() < t_sim:
            if time.monotonic() > watchdog:
                raise RuntimeError('simulation clock stopped')
            rclpy.spin_once(self, timeout_sec=0.02)


def run_piece(node, model, target, start_q, args, out):
    name = target.model
    bin_name = tri_scene.PAIRS[name]
    gt = node.perceived_poses({name, bin_name})            # observation pose, as recorded
    if start_q is not None and not node.transit(start_q, 'pose de depart aleatoire'):
        return {'piece': name, 'error': 'random start pose unreachable without scraping'}
    node.set_gripper_value(START_GRIPPER)
    node.spin_for(1.5)
    piece0, bin_xy = gt[name], gt[bin_name][:2]
    trace, poses, chunks, infer_s = [], [], 0, []
    max_z, placed_after_pick = float(piece0[2]), False
    node.gz_pause(True)
    try:
        while len(trace) < args.max_steps:
            images, state = node.observe()
            t = time.monotonic()
            chunk = model.predict(images, state, INSTRUCTIONS[name], seed=args.seed * 100000 + len(trace))
            infer_s.append(time.monotonic() - t)
            node.gz_pause(False)
            n = min(args.n_action_steps, args.max_steps - len(trace))
            for s in node.play(chunk[:n], poses):
                s['chunk'] = chunks
                trace.append(s)
            chunks += 1
            node.gz_pause(True)
            here = node.object_poses().get(name)
            poses.append({'t': round(node.now(), 3), 'poses': {name: np.round(here, 4).tolist()}})
            max_z = max([max_z] + [p['poses'][name][2] for p in poses if name in p['poses']])
            if (max_z > PICKED_Z and here is not None and abs(here[0] - bin_xy[0]) < BIN_HALF
                    and abs(here[1] - bin_xy[1]) < BIN_HALF and here[2] < PICKED_Z):
                placed_after_pick = True
                break
    finally:
        node.gz_pause(False)
    node.open_gripper()
    end = node.object_poses().get(name)
    placed = bool(end is not None and abs(end[0] - bin_xy[0]) < BIN_HALF
                  and abs(end[1] - bin_xy[1]) < BIN_HALF and end[2] < PICKED_Z)
    with open(out / f'trace_{name}.jsonl', 'w') as f:
        f.write(json.dumps({'piece': name, 'bin': bin_name, 'piece0': np.round(piece0, 4).tolist(),
                            'bin_xy': np.round(bin_xy, 4).tolist(), 'start_q_deg': None if start_q is None
                            else np.round(start_q, 2).tolist()}) + '\n')
        for s in trace:
            f.write(json.dumps(s) + '\n')
        for p in poses:
            f.write(json.dumps(p) + '\n')
    return {'piece': name, 'picked': max_z > PICKED_Z, 'placed': placed,
            'placed_after_pick': placed and max_z > PICKED_Z, 'stopped_in_bin': placed_after_pick,
            'max_z': round(max_z, 4), 'piece0': np.round(piece0, 4).tolist(),
            'end': None if end is None else np.round(end, 4).tolist(), 'bin_xy': np.round(bin_xy, 4).tolist(),
            'steps': len(trace), 'chunks': chunks, 'infer_s_median': round(float(np.median(infer_s)), 1)}


def run_piece_locked(node, model, target, start_q, args, out):
    """run_piece, step-locked (step_lock.py): same sequence, same scoring, plus a fingerprint."""
    name = target.model
    bin_name = tri_scene.PAIRS[name]
    if not node.locked_transit(np.array(tri_scene.OBSERVATION_Q_DEG), 'pose d observation'):
        return {'piece': name, 'error': 'observation pose unreachable without scraping'}
    node.step(START_SETTLE_STEPS)
    piece0, bin_xy = node.object_poses()[name], node.bins[bin_name][:2]
    if start_q is not None and not node.locked_transit(start_q, 'pose de depart aleatoire'):
        return {'piece': name, 'error': 'random start pose unreachable without scraping'}
    node.locked_gripper(START_GRIPPER)
    node.align_to_tick()
    start_state = list(node.joint_full)
    start_t = node.now_ns()
    trace, poses, chunks, infer_s, frames = [], [], 0, [], []
    max_z, placed_after_pick = float(piece0[2]), False
    while len(trace) < args.max_steps:
        images, state, hashes = node.locked_observe(decode)
        frames.append({'step': len(trace), 't_ns': node.now_ns() - start_t, 'sha256': hashes})
        t = time.monotonic()
        chunk = model.predict(images, state, INSTRUCTIONS[name], seed=args.seed * 100000 + len(trace))
        infer_s.append(time.monotonic() - t)
        n = min(args.n_action_steps, args.max_steps - len(trace))
        node.publish_arm([a[:6] for a in chunk[:n]], dt=DT)
        grip = None
        for k, a in enumerate(chunk[:n]):
            if grip is None or abs(a[6] - grip) > 1e-4:
                grip = float(a[6])
                node.set_gripper_value(grip)
            node.deliver()
            node.step(int(round(DT * 1000)))
            trace.append({'t_ns': node.now_ns() - start_t, 'chunk': chunks, 'state': list(node.joint_full),
                          'action': [float(v) for v in a], 'tip': np.round(tool_tip(node.q_deg), 4).tolist()})
            if k % 5 == 4:
                poses.append({'t_ns': node.now_ns() - start_t,
                              'poses': {m: p.tolist() for m, p in node.object_poses().items()}})
        chunks += 1
        here = node.object_poses().get(name)
        poses.append({'t_ns': node.now_ns() - start_t, 'poses': {name: here.tolist()}})
        max_z = max([max_z] + [p['poses'][name][2] for p in poses if name in p['poses']])
        if (max_z > PICKED_Z and here is not None and abs(here[0] - bin_xy[0]) < BIN_HALF
                and abs(here[1] - bin_xy[1]) < BIN_HALF and here[2] < PICKED_Z):
            placed_after_pick = True
            break
    node.locked_gripper(0.0, GRIPPER_SETTLE_STEPS)
    end = node.object_poses().get(name)
    placed = bool(end is not None and abs(end[0] - bin_xy[0]) < BIN_HALF
                  and abs(end[1] - bin_xy[1]) < BIN_HALF and end[2] < PICKED_Z)
    fingerprint = {'start_state': sha(start_state), 'trace': sha(trace), 'piece_poses': sha(poses),
                   'frames': sha(frames), 'end': sha(None if end is None else end.tolist())}
    fingerprint['all'] = sha(fingerprint)
    with open(out / f'trace_{name}.jsonl', 'w') as f:
        f.write(json.dumps({'piece': name, 'bin': bin_name, 'step_locked': True, 'piece0': piece0.tolist(),
                            'bin_xy': list(bin_xy), 'start_state': start_state, 'start_q_deg': None if start_q is None
                            else np.round(start_q, 2).tolist(), 'fingerprint': fingerprint}) + '\n')
        for s in trace:
            f.write(json.dumps(s) + '\n')
        for p in poses:
            f.write(json.dumps(p) + '\n')
        for fr in frames:
            f.write(json.dumps({'frame': fr}) + '\n')
    return {'piece': name, 'picked': max_z > PICKED_Z, 'placed': placed,
            'placed_after_pick': placed and max_z > PICKED_Z, 'stopped_in_bin': placed_after_pick,
            'max_z': round(max_z, 4), 'piece0': np.round(piece0, 4).tolist(),
            'end': None if end is None else np.round(end, 4).tolist(), 'bin_xy': np.round(bin_xy, 4).tolist(),
            'steps': len(trace), 'chunks': chunks, 'infer_s_median': round(float(np.median(infer_s)), 1),
            'fingerprint': fingerprint}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--checkpoint', required=True)
    ap.add_argument('--seed', type=int, required=True)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--start-pose', choices=('observation', 'random'), default='observation')
    ap.add_argument('--n-action-steps', type=int, default=50)
    ap.add_argument('--max-steps', type=int, default=300, help='per piece; recorded episodes are 161-208')
    ap.add_argument('--only', default='', help='comma-separated pieces')
    ap.add_argument('--step-locked', action='store_true', help='step_lock.py: reproducible to the bit')
    args, ros_args = ap.parse_known_args()
    args.out.mkdir(parents=True, exist_ok=True)
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true', '-p', 'world_name:=tri_yolo',
                     '-p', 'pose_source:=ground_truth', '-p', 'wall_timeout_scale:=15.0',
                     '-p', 'startup_timeout:=600.0'] + ros_args)
    node = PolicyRunner()
    if args.step_locked:
        import torch
        torch.use_deterministic_algorithms(True)
        torch.set_num_threads(4)
    model = SmolVLA(args.checkpoint)
    node.wait_until_ready()
    if args.step_locked:
        bins = sorted(set(tri_scene.PAIRS.values()))
        node.bins = {k: np.asarray(v) for k, v in model_poses(node.world, bins).items()}
        node.lock()
    rng = np.random.default_rng(args.seed)
    only = [p for p in args.only.split(',') if p]
    results = []
    for target in TRI_TARGETS:
        if only and target.model not in only:
            continue
        start_q = random_start.sample(rng) if args.start_pose == 'random' else None
        try:
            r = (run_piece_locked if args.step_locked else run_piece)(node, model, target, start_q, args, args.out)
        except Exception as exc:                                # noqa: BLE001
            r = {'piece': target.model, 'error': str(exc)}
        results.append(r)
        node.status(f'[eval] {json.dumps(r)}')
        (args.out / 'result.json').write_text(json.dumps({
            'checkpoint': args.checkpoint, 'seed': args.seed, 'start_pose': args.start_pose,
            'step_locked': args.step_locked,
            'n_action_steps': args.n_action_steps, 'max_steps': args.max_steps, 'pieces': results},
            indent=1) + '\n')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
