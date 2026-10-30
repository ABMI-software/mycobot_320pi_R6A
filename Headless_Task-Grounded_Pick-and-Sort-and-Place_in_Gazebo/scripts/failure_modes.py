#!/usr/bin/env python3
"""One failure mode per piece of a Gazebo evaluation (eval_tri_seed.sh output).

  failure_modes.py <eval dir> <dataset root> [<dataset root> ...]

The gripper "closes" where grasp_spots.py says it does (the action crosses the piece's training
close value + 0.05). Of all closes, the attempt is the one nearest the piece in 3D. Modes, first
that applies:

  sorted                  picked and placed in its own bin
  picked, not placed      lifted (z > 6 cm), then left outside its bin (end position, nearest bin)
  never closed            the gripper never reached the close value
  closed in the air       every close more than AIR_MM above the training grasp height
  memorised spot          attempt > NEAR_MM from the piece, within SPOT_MM of a training grasp
  wrong place             attempt > NEAR_MM from the piece, not near any training grasp
  near miss               attempt within NEAR_MM of the piece, at grasp height, not lifted

Pieces moved more than PUSH_MM without being lifted are flagged "pushed". Writes
<eval dir>/failure_modes.json. Inside gazebo_to_lerobot, ROS sourced, /workspace/venv_lerobot.
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from grasp_spots import tip, training_spots  # noqa: E402

NEAR_MM = 30.0
SPOT_MM = 20.0
AIR_MM = 40.0
PUSH_MM = 10.0


def grasp_heights(root):
    """Tip z (m) at each recorded grasp, per piece."""
    sources = {s['episode_index']: s for s in json.loads((root / 'meta' / 'tri_sort_sources.json').read_text())}
    df = pd.concat(pd.read_parquet(f) for f in sorted((root / 'data').rglob('*.parquet')))
    out = {}
    for ep, g in df.groupby('episode_index'):
        act, state = np.stack(g['action'].values), np.stack(g['observation.state'].values)
        k = int(np.argmax(act[:, 6] <= act[:, 6].min() + 1e-3))
        out.setdefault(sources[ep]['piece'], []).append(float(tip(state[k])[2]))
    return out


def classify(trace, result, spots, close_of, grasp_z, bins):
    lines = [json.loads(line) for line in trace.read_text().splitlines()]
    head, steps = lines[0], [line for line in lines[1:] if 'action' in line]
    piece = head['piece']
    p0 = np.array(head['piece0'])
    end = np.array(result['end'])
    moved_mm = float(np.linalg.norm(end[:2] - p0[:2]) * 1000)
    z_ref = float(np.median(grasp_z[piece]))
    threshold, armed, closes = close_of[piece] + 0.05, True, []
    for i, s in enumerate(steps):
        closing = s['action'][6] <= threshold
        if closing and armed:
            t = np.array(s['tip'])
            near = min(spots, key=lambda sp: np.linalg.norm(sp['tip'] - t[:2]))
            closes.append({'step': i, 'chunk': s['chunk'],
                           'xy_from_piece_mm': round(float(np.linalg.norm(t[:2] - p0[:2]) * 1000), 1),
                           'z_above_grasp_mm': round((float(t[2]) - z_ref) * 1000, 1),
                           'nearest_training_spot_mm': round(float(np.linalg.norm(near['tip'] - t[:2]) * 1000), 1),
                           'training_spot': f"seed {near['seed']} {near['piece']}"})
        armed = not closing
    row = {'piece': piece, 'closes': closes, 'moved_mm': round(moved_mm, 1),
           'pushed': moved_mm > PUSH_MM and not result['picked']}
    if result.get('placed_after_pick'):
        row['mode'] = 'sorted'
    elif result['picked']:
        nb = min(bins, key=lambda b: np.linalg.norm(np.asarray(bins[b]) - end[:2]))
        row['mode'] = 'picked, not placed'
        row['detail'] = f'ended at {np.round(end[:2], 3).tolist()}, nearest bin: that of {nb}'
    elif not closes:
        row['mode'] = 'never closed'
    elif all(c['z_above_grasp_mm'] > AIR_MM for c in closes):
        row['mode'] = 'closed in the air'
    else:
        a = min(closes, key=lambda c: np.hypot(c['xy_from_piece_mm'], max(c['z_above_grasp_mm'], 0)))
        row['attempt'] = a
        if a['xy_from_piece_mm'] > NEAR_MM:
            row['mode'] = 'memorised spot' if a['nearest_training_spot_mm'] <= SPOT_MM else 'wrong place'
        else:
            row['mode'] = 'near miss'
    return row


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('eval_dir', type=Path)
    ap.add_argument('datasets', type=Path, nargs='+')
    args = ap.parse_args()
    spots = [s for root in args.datasets for s in training_spots(root)]
    close_of = {p: float(np.median([s['close_value'] for s in spots if s['piece'] == p]))
                for p in {s['piece'] for s in spots}}
    grasp_z = {}
    for root in args.datasets:
        for p, zs in grasp_heights(root).items():
            grasp_z.setdefault(p, []).extend(zs)
    results = json.loads((args.eval_dir / 'result.json').read_text())
    bins = {}
    for r in results['pieces']:
        bins[r['piece']] = r['bin_xy']
    rows = []
    for r in results['pieces']:
        rows.append(classify(args.eval_dir / f"trace_{r['piece']}.jsonl", r, spots, close_of, grasp_z, bins))
        row = rows[-1]
        a = row.get('attempt')
        extra = (f" — attempt {a['xy_from_piece_mm']:.0f} mm from the piece, {a['z_above_grasp_mm']:+.0f} mm "
                 f"vs grasp height, {a['nearest_training_spot_mm']:.0f} mm from {a['training_spot']}" if a else
                 f" — {row['detail']}" if 'detail' in row else '')
        print(f"{row['piece']:14s} {row['mode']}{' (pushed %.0f mm)' % row['moved_mm'] if row['pushed'] else ''}"
              f"{extra}; {len(row['closes'])} close(s)")
    (args.eval_dir / 'failure_modes.json').write_text(json.dumps(rows, indent=1) + '\n')


if __name__ == '__main__':
    main()
