#!/usr/bin/env python3
"""Open-loop check of a SmolVLA checkpoint on recorded episodes, without the simulator.

  open_loop_check.py <checkpoint/pretrained_model> <dataset root> [--per-piece 2] [--starts 0 60 120]
                     [--before-close 30 25 20 15 10]

Feeds recorded frames (the three images, the state, the instruction) through the checkpoint
and compares each predicted 50-action chunk with the actions recorded over the same steps:
gripper-tip distance (FK of the predicted vs recorded joints, the sorter's own tool_tip), joint
error and gripper error. Catches what a live evaluation cannot separate from the policy itself:
camera order, RGB order, image size, normalisation, action units.

--swap-images feeds each episode the camera images of the same piece's episode on the next seed
(cyclic, same frame index, clamped), with its own state and instruction: whether the policy uses
the images at all. Every chunk that closes the gripper also reports how far that close is from
the episode's own recorded grasp and from the image donor's.

--before-close starts each episode's chunks that many frames before its recorded close (the first
frame whose recorded gripper action is within 0.05 of the episode's most closed value) instead of
at fixed frames: a grasp-focused set where the recorded close falls inside every chunk.

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
    ap.add_argument('--before-close', type=int, nargs='+', help='frames before the recorded close, instead of --starts')
    ap.add_argument('--out', type=Path)
    ap.add_argument('--swap-images', action='store_true')
    args = ap.parse_args()

    ds = LeRobotDataset(f'local/{args.dataset_root.name}', root=args.dataset_root)
    sources = json.loads((args.dataset_root / 'meta' / 'tri_sort_sources.json').read_text())
    episode_of = np.asarray(ds.hf_dataset['episode_index'])
    chosen = []
    for piece in sorted({s['piece'] for s in sources}):
        eps = [s for s in sources if s['piece'] == piece]
        chosen += [eps[i] for i in np.linspace(0, len(eps) - 1, args.per_piece).round().astype(int)]

    def frames(src):
        return np.flatnonzero(episode_of == src['episode_index'])

    def close_frame(recorded):
        return int(np.argmax(recorded[:, 6] <= recorded[:, 6].min() + 0.05))

    def grasp_tip(src):
        recorded = np.stack([ds.hf_dataset[int(i)]['action'] for i in frames(src)])
        k = close_frame(recorded)
        return tips(recorded[k:k + 1])[0]

    seeds = sorted({s['seed'] for s in sources})
    by_key = {(s['seed'], s['piece']): s for s in sources}
    donor_of = {s['episode_index']: by_key[(seeds[(seeds.index(s['seed']) + 1) % len(seeds)], s['piece'])]
                for s in sources}

    model = SmolVLA(args.checkpoint)
    rows = []
    for src in chosen:
        idx = frames(src)
        recorded = np.stack([ds.hf_dataset[int(i)]['action'] for i in idx])
        donor = donor_of[src['episode_index']]
        donor_idx = frames(donor)
        own_grasp, donor_grasp = grasp_tip(src), grasp_tip(donor)
        k_close = close_frame(recorded)
        starts = ([(k_close - b, b) for b in args.before_close if k_close - b >= 0] if args.before_close
                  else [(t, None) for t in args.starts])
        for t, before in starts:
            if t >= len(idx):
                continue
            frame = ds[int(idx[t])]
            if args.swap_images:
                donor_frame = ds[int(donor_idx[min(t, len(donor_idx) - 1)])]
                for k in [k for k in frame if k.startswith('observation.images.')]:
                    frame[k] = donor_frame[k]
            pred = model.predict_frame(frame, seed=src['episode_index'] * 1000 + t)
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
                   'gripper_mean_abs': round(float(np.abs(pred[:n, 6] - ref[:, 6]).mean()), 3),
                   'images_from': donor['source'] if args.swap_images else src['source']}
            if before is not None:
                row['before_close'] = before
            # Where the gripper closes on the piece: the step its action first reaches the
            # episode's most closed value, recorded vs predicted (within this chunk).
            close = recorded[:, 6].min() + 0.05
            k_ref = np.flatnonzero(ref[:, 6] <= close)
            k_pred = np.flatnonzero(pred[:n, 6] <= close)
            if len(k_ref) and len(k_pred) and (k_ref[0] > 0 or t == 0 or recorded[t - 1, 6] > close):
                row['grasp_point_mm'] = round(float(np.linalg.norm(pred_tip[k_pred[0]] - ref_tip[k_ref[0]]) * 1000), 1)
                row['grasp_step_lag'] = int(k_pred[0] - k_ref[0])
            if len(k_pred) and recorded[t, 6] > close:  # a chunk that starts holding the piece has no grasp
                row['close_to_own_grasp_mm'] = round(float(np.linalg.norm(pred_tip[k_pred[0]] - own_grasp) * 1000), 1)
                row['close_to_donor_grasp_mm'] = round(float(np.linalg.norm(pred_tip[k_pred[0]] - donor_grasp) * 1000), 1)
            rows.append(row)
            print(json.dumps(row), flush=True)

    tip = np.array([r['tip_mean_mm'] for r in rows])
    closes = [r for r in rows if 'close_to_own_grasp_mm' in r]
    summary = {'checkpoint': str(args.checkpoint), 'dataset': str(args.dataset_root), 'chunks': len(rows),
               'swap_images': args.swap_images, 'before_close': args.before_close,
               'chunks_with_predicted_close': sum('grasp_point_mm' in r for r in rows),
               'closes': len(closes),
               'close_to_own_grasp_mm_median': round(float(np.median([r['close_to_own_grasp_mm'] for r in closes])), 1) if closes else None,
               'close_to_donor_grasp_mm_median': round(float(np.median([r['close_to_donor_grasp_mm'] for r in closes])), 1) if closes else None,
               'closes_nearer_donor': sum(r['close_to_donor_grasp_mm'] < r['close_to_own_grasp_mm'] for r in closes),
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
