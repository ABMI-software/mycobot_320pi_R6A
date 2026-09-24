#!/usr/bin/env python3
"""Independent pose logger for the Part 3.3 grasp-decision gate.

Does NOT import anything from sim_sorting_grasp.py and does not trust its
console output. Computes the end-effector (fingertip) position itself, from
/joint_states via forward kinematics, and reads the target object's pose
directly from Gazebo's own dynamic_pose/info topic. Phase (grasp/lift/
released) is inferred purely from the object's own z-trajectory, not from
any signal the pipeline under test emits.

Usage:
  python3 track_grasp.py --model red_cube --world pick_and_place_sorting \
      --out /tmp/refgrasp.csv --hz 10 --duration 90
"""
import argparse
import re
import subprocess
import sys
import time
from pathlib import Path

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

_VENDOR = Path(__file__).resolve().parents[1] / 'vendor'
# diff_ik.py internally does its own sys.path.insert for mycobot_fk.py at a
# path relative to ITS OWN location (parents[1]/'training'/'dream'), which
# does not match this project's flattened vendor/training_dream/ layout.
# Inserting the real location here first means that internal insert (which
# will point nowhere) is harmless -- Python just falls through to this path.
sys.path.insert(0, str(_VENDOR / 'training_dream'))
sys.path.insert(0, str(_VENDOR / 'scripts'))
from diff_ik import fk_pose  # noqa: E402

ARM_JOINTS = [
    'joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
    'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6',
]
# Measured fingertip offset from link6, in the link6 frame -- same constant
# sim_sorting_grasp.py uses (mycobot_gateway/mycobot_gateway/sim_sorting_grasp.py,
# TOOL_OFFSET). Re-declared here, not imported, so this logger has no code
# dependency on the pipeline it is verifying.
TOOL_OFFSET = np.array([-0.001, 0.0078, 0.166])
RISE_THRESH = 0.03      # m above baseline object height => "lifted"
RELEASE_BAND = 0.015    # m above baseline => back down close enough to call "released"


def tool_tip(q_deg):
    p_mm, rot = fk_pose(q_deg)
    return p_mm / 1000.0 + rot @ TOOL_OFFSET


def read_object_pose(world: str, model: str):
    out = subprocess.run(
        ['gz', 'topic', '-e', '-n', '1', '-t', f'/world/{world}/dynamic_pose/info'],
        capture_output=True, text=True, timeout=10).stdout
    for block in out.split('pose {')[1:]:
        name = re.search(r'name: "([^"]+)"', block)
        if not (name and name.group(1) == model):
            continue
        pos = re.search(
            r'position \{\s*x: ([-\d.e+]+)\s*y: ([-\d.e+]+)\s*z: ([-\d.e+]+)', block)
        if pos:
            return np.array([float(pos.group(i)) for i in (1, 2, 3)])
    return None


class TrackGrasp(Node):
    def __init__(self, model, world, out_path, hz, duration):
        super().__init__('track_grasp')
        self.model = model
        self.world = world
        self.hz = hz
        self.duration = duration
        self.q_deg = None
        self.create_subscription(JointState, '/joint_states', self._joint_cb, 10)
        self.rows = []
        self.baseline_z = None
        self.phase = 'grasp'
        self.was_lifted = False

    def _joint_cb(self, msg):
        names = list(msg.name)
        if all(j in names for j in ARM_JOINTS):
            self.q_deg = np.degrees(
                [msg.position[names.index(j)] for j in ARM_JOINTS])

    def run(self, out_fh):
        t0 = time.time()
        period = 1.0 / self.hz
        n_no_pose = 0
        while time.time() - t0 < self.duration:
            tick_start = time.time()
            rclpy.spin_once(self, timeout_sec=0.02)
            obj = read_object_pose(self.world, self.model)
            if obj is None:
                n_no_pose += 1
            elif self.q_deg is not None:
                if self.baseline_z is None:
                    self.baseline_z = obj[2]
                rel = obj[2] - self.baseline_z
                if not self.was_lifted and rel >= RISE_THRESH:
                    self.was_lifted = True
                    self.phase = 'lift'
                elif self.was_lifted and rel < RELEASE_BAND:
                    self.phase = 'released'
                elif self.was_lifted:
                    self.phase = 'lift'
                tip = tool_tip(self.q_deg)
                dz = float(np.linalg.norm(tip - obj))
                row = (round(time.time() - t0, 3), self.phase, round(dz, 5),
                       round(float(obj[2]), 5), round(float(tip[0]), 5),
                       round(float(tip[1]), 5), round(float(tip[2]), 5))
                self.rows.append(row)
                out_fh.write(','.join(str(v) for v in row) + '\n')
                out_fh.flush()
            elapsed = time.time() - tick_start
            time.sleep(max(0.0, period - elapsed))
        self.get_logger().info(
            f'{len(self.rows)} samples written, {n_no_pose} ticks with no pose '
            f'for "{self.model}" (object not yet spawned or model name mismatch)')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--model', required=True)
    ap.add_argument('--world', required=True)
    ap.add_argument('--out', required=True)
    ap.add_argument('--hz', type=float, default=10.0)
    ap.add_argument('--duration', type=float, default=90.0)
    args = ap.parse_args()

    rclpy.init()
    node = TrackGrasp(args.model, args.world, args.out, args.hz, args.duration)
    with open(args.out, 'w', buffering=1) as f:
        f.write('t,phase,dz,bz,ex,ey,ez\n')
        f.flush()
        try:
            node.run(f)
        finally:
            node.destroy_node()
            rclpy.try_shutdown()
    print(f'wrote {len(node.rows)} rows to {args.out}')


if __name__ == '__main__':
    main()
