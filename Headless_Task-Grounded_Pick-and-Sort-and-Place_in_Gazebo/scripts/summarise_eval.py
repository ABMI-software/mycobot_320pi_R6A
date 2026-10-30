#!/usr/bin/env python3
"""Table of SmolVLA Gazebo evaluations: picked / placed / placed-after-pick, training vs new seeds.

  summarise_eval.py <eval root> [--train-seeds 1-10,21-50]

Reads every <eval root>/**/result.json written by eval_smolvla_tri_sort.py. Groups by checkpoint,
start pose and action steps per chunk, then by seed kind (a training seed, or a new one). Rates
come with a 95 % Wilson interval: on 20 pieces, 3/20 reads 5-36 %, too wide to rank two runs.
"""
import argparse
import json
import math
from collections import defaultdict
from pathlib import Path


def seeds(spec):
    out = set()
    for part in spec.split(','):
        lo, _, hi = part.partition('-')
        out.update(range(int(lo), int(hi or lo) + 1))
    return out


def wilson(k, n, z=1.96):
    if n == 0:
        return 0.0, 0.0
    p = k / n
    centre = (p + z * z / (2 * n)) / (1 + z * z / n)
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return max(0.0, centre - half), min(1.0, centre + half)


def rate(k, n):
    lo, hi = wilson(k, n)
    return f'{k:3d}/{n:<3d} {100 * k / n:5.1f} % [{100 * lo:4.0f}-{100 * hi:3.0f}]' if n else '   -'


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('root', type=Path)
    ap.add_argument('--train-seeds', default='1-10,21-50')
    args = ap.parse_args()
    train = seeds(args.train_seeds)
    groups = defaultdict(list)
    for f in sorted(args.root.rglob('result.json')):
        r = json.loads(f.read_text())
        ckpt = Path(r['checkpoint'])
        key = (f'{ckpt.parent.name}/{ckpt.name}', r['start_pose'], r['n_action_steps'])
        for p in r['pieces']:
            groups[key].append(dict(p, seed=r['seed'], kind='train' if r['seed'] in train else 'new'))
    for (ckpt, start, n_steps), pieces in sorted(groups.items()):
        print(f'\n== {ckpt}   start {start}   {n_steps} actions/chunk')
        print(f'   {"seeds":24s} {"picked":24s} {"placed":24s} {"placed after pick":24s} errors')
        for kind in ('train', 'new'):
            ps = [p for p in pieces if p['kind'] == kind]
            if not ps:
                continue
            ok = [p for p in ps if 'error' not in p]
            label = f'{kind} {sorted({p["seed"] for p in ps})}'
            print(f'   {label[:24]:24s} {rate(sum(p["picked"] for p in ok), len(ok)):24s} '
                  f'{rate(sum(p["placed"] for p in ok), len(ok)):24s} '
                  f'{rate(sum(p["placed_after_pick"] for p in ok), len(ok)):24s} {len(ps) - len(ok)}')
            for piece in sorted({p['piece'] for p in ok}):
                pp = [p for p in ok if p['piece'] == piece]
                print(f'     {piece:22s} {rate(sum(p["picked"] for p in pp), len(pp)):24s} '
                      f'{rate(sum(p["placed"] for p in pp), len(pp)):24s} '
                      f'{rate(sum(p["placed_after_pick"] for p in pp), len(pp))}')


if __name__ == '__main__':
    main()
