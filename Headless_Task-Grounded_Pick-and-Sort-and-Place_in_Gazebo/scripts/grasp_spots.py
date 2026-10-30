#!/usr/bin/env python3
"""Where the gripper closes: on the real piece, or on a spot remembered from training?

  grasp_spots.py <eval root> <dataset root> [<dataset root> ...]

Training grasp spots: in each recorded episode, the gripper-tip position (FK of the measured
joints, the sorter's tool_tip) at the first step where the gripper action reaches its most
closed value. Evaluation grasps: in each trace_<piece>.jsonl, the tip at every step where the
gripper action first crosses that piece's training close value (+0.05), after being above it.

For each evaluation grasp: distance (x, y) to the real piece, and to the nearest training spot,
with the seed and piece it belongs to. Baseline: how far the real piece already sits from the
nearest training position of the same piece, so "near a training spot" means something.
A grasp near a training spot and far from the piece is memorisation; near the piece but
missing is an accuracy problem.

Inside gazebo_to_lerobot, ROS sourced, /workspace/venv_lerobot's python.
"""
import argparse
import json
from pathlib import Path

import numpy as np
import pandas as pd
from mycobot_gateway.sim_sorting_grasp import tool_tip

PIECE_OF_TASK = {'red cube': 'cube_rouge', 'blue cube': 'cube_bleu',
                 'green cylinder': 'cylindre_vert', 'yellow box': 'pave_jaune'}


def tip(q_rad):
    return tool_tip(np.degrees(np.asarray(q_rad[:6])))


def training_spots(root):
    sources = {s['episode_index']: s for s in json.loads((root / 'meta' / 'tri_sort_sources.json').read_text())}
    df = pd.concat(pd.read_parquet(f) for f in sorted((root / 'data').rglob('*.parquet')))
    spots = []
    for ep, g in df.groupby('episode_index'):
        act = np.stack(g['action'].values)
        state = np.stack(g['observation.state'].values)
        k = int(np.argmax(act[:, 6] <= act[:, 6].min() + 1e-3))
        spots.append({'seed': sources[ep]['seed'], 'piece': sources[ep]['piece'],
                      'close_value': float(act[:, 6].min()), 'tip': tip(state[k])[:2],
                      'step': k, 'dataset': root.name})
    return spots


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('eval_root', type=Path)
    ap.add_argument('datasets', type=Path, nargs='+')
    args = ap.parse_args()
    spots = [s for root in args.datasets for s in training_spots(root)]
    close_of = {p: float(np.median([s['close_value'] for s in spots if s['piece'] == p]))
                for p in {s['piece'] for s in spots}}
    print(f'{len(spots)} training grasp spots; close values {close_of}')
    for trace in sorted(args.eval_root.rglob('trace_*.jsonl')):
        lines = [json.loads(l) for l in trace.read_text().splitlines()]
        head, steps = lines[0], [l for l in lines[1:] if 'action' in l]
        piece, piece_xy = head['piece'], np.array(head['piece0'][:2])
        same = [s for s in spots if s['piece'] == piece]
        baseline = min(np.linalg.norm(s['tip'] - piece_xy) for s in same) * 1000
        print(f'\n{trace.parent.name}/{piece}: piece at {np.round(piece_xy, 3).tolist()}, '
              f'nearest training position of this piece {baseline:.0f} mm away')
        threshold, armed = close_of[piece] + 0.05, True
        for i, s in enumerate(steps):
            closing = s['action'][6] <= threshold
            if closing and armed:
                xy, z = np.array(s['tip'][:2]), s['tip'][2]
                near = min(spots, key=lambda t: np.linalg.norm(t['tip'] - xy))
                print(f'  close at step {i:3d} (chunk {s["chunk"]}), tip z={z * 1000:.0f} mm: '
                      f'{np.linalg.norm(xy - piece_xy) * 1000:4.0f} mm from the piece, '
                      f'{np.linalg.norm(near["tip"] - xy) * 1000:4.0f} mm from training spot '
                      f'seed {near["seed"]} {near["piece"]}')
            armed = not closing
        if all(s['action'][6] > threshold for s in steps):
            print('  never closed')


if __name__ == '__main__':
    main()
