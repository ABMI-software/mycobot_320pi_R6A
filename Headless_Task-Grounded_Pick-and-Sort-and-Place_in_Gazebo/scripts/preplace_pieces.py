#!/usr/bin/env python3
"""One-episode-per-layout recordings: choose the target piece and put 0-3 others in their bins.

  preplace_pieces.py --seed 1001 --out /workspace/tri_sort_B/seed_1001

The target is PIECES[seed % 4]; numpy default_rng([seed, 1]) (a stream distinct from the scene
sampler's and the random start's default_rng(seed)) draws how many of the three other pieces are
already sorted (0-3, uniform) and which. The target is never moved. Each chosen piece is set,
upright, on its bin's floor at the bin centre +- 8 mm, then the sim runs; the result is read back
from Gazebo. Fails (exit 1, nothing recorded) unless every moved piece rests inside its bin and
no other piece moved more than 1 mm. Writes <out>/preplaced.json; prints the target last.
Plain python3 with ROS sourced, inside gazebo_to_lerobot, scene running, arm at its start pose.
"""
import argparse
import json
import re
import subprocess
import time
from pathlib import Path

import numpy as np

PIECES = ('cube_rouge', 'pave_jaune', 'cylindre_vert', 'cube_bleu')
BIN_OF = {'cube_rouge': 'bac_rouge', 'pave_jaune': 'bac_jaune',
          'cylindre_vert': 'bac_vert', 'cube_bleu': 'bac_bleu'}
HALF_XY = {'cube_rouge': 0.025, 'pave_jaune': 0.025, 'cylindre_vert': 0.025, 'cube_bleu': 0.025}
BIN_INNER_HALF = 0.0475   # bac_*/model.sdf: walls at +-0.050, 5 mm thick
BIN_FLOOR_Z = 0.002
JITTER = 0.008
SETTLE_WALL_S = 20.0      # ~2 s of sim time at the recording RTF (~0.1)


def model_poses(world, names):
    """x, y, z of the named models; pose/info, because dynamic_pose/info leaves out the static bins."""
    for _ in range(10):
        out = subprocess.run(['gz', 'topic', '-e', '-n', '1', '-t', f'/world/{world}/pose/info'],
                             capture_output=True, text=True, timeout=30).stdout
        poses = {}
        for block in out.split('pose {')[1:]:
            name = re.search(r'name: "([^"]+)"', block)
            pos = re.search(r'position \{\s*(?:x: ([-\d.e+]+))?\s*(?:y: ([-\d.e+]+))?\s*(?:z: ([-\d.e+]+))?', block)
            if name and pos and name.group(1) in names and name.group(1) not in poses:
                poses[name.group(1)] = np.array([float(pos.group(i) or 0.0) for i in (1, 2, 3)])
        if set(poses) == set(names):
            return poses
        time.sleep(2)
    raise RuntimeError(f'pose/info lacks {set(names) - set(poses)}')


def set_poses(world, placed):
    req = ' '.join(f'pose {{ name: "{n}" position {{ x: {x:.6f} y: {y:.6f} z: {z:.6f} }} '
                   f'orientation {{ w: 1 }} }}' for n, (x, y, z) in placed.items())
    subprocess.run(['gz', 'service', '-s', f'/world/{world}/set_pose_vector',
                    '--reqtype', 'gz.msgs.Pose_V', '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '5000', '--req', req], check=True, capture_output=True, timeout=30)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--seed', type=int, required=True)
    ap.add_argument('--out', type=Path, required=True)
    ap.add_argument('--world', default='tri_yolo')
    args = ap.parse_args()
    target = PIECES[args.seed % 4]
    rng = np.random.default_rng([args.seed, 1])
    others = [p for p in PIECES if p != target]
    k = int(rng.integers(0, 4))
    moved = [str(p) for p in rng.choice(others, k, replace=False)] if k else []
    assert target not in moved
    names = list(PIECES) + list(BIN_OF.values())
    before = model_poses(args.world, names)
    placed = {}
    for p in moved:
        b = before[BIN_OF[p]]
        dx, dy = rng.uniform(-JITTER, JITTER, 2)
        placed[p] = (b[0] + dx, b[1] + dy, b[2] + BIN_FLOOR_Z + 0.0005)
    if placed:
        set_poses(args.world, placed)
        time.sleep(SETTLE_WALL_S)
    after = model_poses(args.world, names)
    errors = []
    report = {}
    for p in moved:
        b = after[BIN_OF[p]]
        off = after[p][:2] - b[:2]
        inside = bool(np.all(np.abs(off) <= BIN_INNER_HALF - HALF_XY[p] + 0.002))
        resting = bool(after[p][2] - b[2] < 0.01)
        if not (inside and resting):
            errors.append(f'{p}: not resting inside {BIN_OF[p]} (offset {np.round(off * 1000, 1).tolist()} mm, '
                          f'z {after[p][2]:.4f})')
        report[p] = {'bin': BIN_OF[p], 'set_xyz': np.round(placed[p], 5).tolist(),
                     'after_xyz': np.round(after[p], 5).tolist(),
                     'offset_from_bin_mm': np.round(off * 1000, 1).tolist()}
    still = {p: round(float(np.linalg.norm(after[p][:2] - before[p][:2]) * 1000), 2)
             for p in PIECES if p not in moved}
    errors += [f'{p} moved {d} mm' for p, d in still.items() if d > 1.0]
    out = {'seed': args.seed, 'target': target, 'rng': 'numpy default_rng([seed, 1])',
           'n_presorted': k, 'presorted': report, 'unmoved_drift_mm': still,
           'pieces_before': {n: np.round(before[n], 5).tolist() for n in names},
           'pieces_after': {n: np.round(after[n], 5).tolist() for n in names},
           'ok': not errors, 'errors': errors}
    args.out.mkdir(parents=True, exist_ok=True)
    (args.out / 'preplaced.json').write_text(json.dumps(out, indent=1) + '\n')
    if errors:
        raise SystemExit('; '.join(errors))
    print(target)


if __name__ == '__main__':
    main()
