#!/usr/bin/env python3
"""Open-loop check of a SmolVLA checkpoint on recorded episodes, without the simulator.

  open_loop_check.py <checkpoint/pretrained_model> <dataset root> [--per-piece 2] [--starts 0 60 120]

Feeds recorded frames (the three images, the state, the instruction) through the checkpoint
and compares each predicted 50-action chunk with the actions recorded over the same steps:
gripper-tip distance (FK of the predicted vs recorded joints, the sorter's own tool_tip), joint
error and gripper error. Catches what a live evaluation cannot separate from the policy itself:
camera order, RGB order, image size, normalisation, action units.

Run inside gazebo_to_lerobot with ROS sourced (tool_tip) and /workspace/venv_lerobot's python.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset
from mycobot_gateway.sim_sorting_grasp import tool_tip

sys.path.insert(0, str(Path(__file__).resolve().parent))
from smolvla_policy import SmolVLA  # noqa: E402


def tips(actions):
    return np.array([tool_tip(np.degrees(a[:6])) for a in actions])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('checkpoint')
    ap.add_argument('dataset_root', type=Path)
    ap.add_argument('--per-piece', type=int, default=2, help='episodes per piece, spread over seeds')
    ap.add_argument('--starts', type=int, nargs='+', default=[0, 60, 120], help='frame of each chunk start')
    ap.add_argument('--out', type=Path)
    args = ap.parse_args()

    ds = LeRobotDataset(f'local/{args.dataset_root.name}', root=args.dataset_root)
    sources = json.loads((args.dataset_root / 'meta' / 'tri_sort_sources.json').read_text())
    episode_of = np.asarray(ds.hf_dataset['episode_index'])
    chosen = []
    for piece in sorted({s['piece'] for s in sources}):
        eps = [s for s in sources if s['piece'] == piece]
        chosen += [eps[i] for i in np.linspace(0, len(eps) - 1, args.per_piece).round().astype(int)]

    model = SmolVLA(args.checkpoint)
    rows = []
    for src in chosen:
        idx = np.flatnonzero(episode_of == src['episode_index'])
        recorded = np.stack([ds.hf_dataset[int(i)]['action'] for i in idx])
        for t in args.starts:
            if t >= len(idx):
                continue
            pred = model.predict_frame(ds[int(idx[t])], seed=src['episode_index'] * 1000 + t)
            n = min(len(pred), len(idx) - t)
            ref = recorded[t:t + n]
            pred_tip, ref_tip = tips(pred[:n]), tips(ref)
            tip_mm = np.linalg.norm(pred_tip - ref_tip, axis=1) * 1000.0
            # Step-by-step distance also counts timing: a lead of 0.3 s in a fast swing is centimetres
            # on a correct path. Path error drops the timing: nearest point of the recorded path.
            path_mm = np.min(np.linalg.norm(pred_tip[:, None] - ref_tip[None], axis=2), axis=1) * 1000.0
            row = {'episode': src['episode_index'], 'seed': src['seed'], 'piece': src['piece'],
                   'start': t, 'steps': n,
                   'tip_mean_mm': round(float(tip_mm.mean()), 1), 'tip_max_mm': round(float(tip_mm.max()), 1),
                   'path_mean_mm': round(float(path_mm.mean()), 1), 'path_max_mm': round(float(path_mm.max()), 1),
                   'joint_mean_deg': round(float(np.degrees(np.abs(pred[:n, :6] - ref[:, :6])).mean()), 2),
                   'gripper_mean_abs': round(float(np.abs(pred[:n, 6] - ref[:, 6]).mean()), 3)}
            # Where the gripper closes on the piece: the step its action first reaches the
            # episode's most closed value, recorded vs predicted (within this chunk).
            close = recorded[:, 6].min() + 0.05
            k_ref = np.flatnonzero(ref[:, 6] <= close)
            k_pred = np.flatnonzero(pred[:n, 6] <= close)
            if len(k_ref) and len(k_pred) and (k_ref[0] > 0 or t == 0 or recorded[t - 1, 6] > close):
                row['grasp_point_mm'] = round(float(np.linalg.norm(pred_tip[k_pred[0]] - ref_tip[k_ref[0]]) * 1000), 1)
                row['grasp_step_lag'] = int(k_pred[0] - k_ref[0])
            rows.append(row)
            print(json.dumps(row), flush=True)

    tip = np.array([r['tip_mean_mm'] for r in rows])
    summary = {'checkpoint': str(args.checkpoint), 'dataset': str(args.dataset_root), 'chunks': len(rows),
               'tip_mean_mm_median': round(float(np.median(tip)), 1),
               'tip_mean_mm_range': [float(tip.min()), float(tip.max())],
               'tip_max_mm_overall': max(r['tip_max_mm'] for r in rows),
               'path_mean_mm_median': round(float(np.median([r['path_mean_mm'] for r in rows])), 1),
               'path_max_mm_overall': max(r['path_max_mm'] for r in rows),
               'grasp_point_mm': [r['grasp_point_mm'] for r in rows if 'grasp_point_mm' in r],
               'grasp_step_lag': [r['grasp_step_lag'] for r in rows if 'grasp_step_lag' in r],
               'joint_mean_deg_median': round(float(np.median([r['joint_mean_deg'] for r in rows])), 2),
               'gripper_mean_abs_median': round(float(np.median([r['gripper_mean_abs'] for r in rows])), 3)}
    print('SUMMARY ' + json.dumps(summary), flush=True)
    if args.out:
        args.out.write_text(json.dumps({'summary': summary, 'chunks': rows}, indent=1) + '\n')


if __name__ == '__main__':
    main()
