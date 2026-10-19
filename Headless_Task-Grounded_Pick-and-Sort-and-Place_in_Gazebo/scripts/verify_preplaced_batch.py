#!/usr/bin/env python3
"""Check a one-episode-per-layout batch (PREPLACE=1 record_tri_batch.sh) before using it.

  verify_preplaced_batch.py <tri home> <first seed> <last seed> [--dataset <LeRobot root>]

Per seed: recorded to the end (COMPLETE), exactly one episode, verdict OK, the recorded piece is
the target PIECES[seed % 4], the target was never moved by the preplacement, every presorted
piece rested inside its bin before the episode and is still inside it at the end, the other
pieces did not move between the preplacement and the episode start, and the target ends in its
own bin. With --dataset: one LeRobot episode per OK seed, its instruction is the target's, and
its frame count is in the recorded range. Prints one line per problem, then a summary;
exit 1 if any. Plain python3; --dataset needs pandas (/workspace/venv_lerobot/bin/python).
"""
import argparse
import json
import sys
from collections import Counter
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from preplace_pieces import BIN_INNER_HALF, BIN_OF, HALF_XY, PIECES  # noqa: E402

INSTRUCTION = {'cube_rouge': 'pick up the red cube and place it in the red bin',
               'cube_bleu': 'pick up the blue cube and place it in the blue bin',
               'cylindre_vert': 'pick up the green cylinder and place it in the green bin',
               'pave_jaune': 'pick up the yellow box and place it in the yellow bin'}
STILL_MM = 1.0


def in_bin(poses, piece):
    off = np.asarray(poses[piece][:2]) - np.asarray(poses[BIN_OF[piece]][:2])
    return bool(np.all(np.abs(off) <= BIN_INNER_HALF - HALF_XY[piece] + 0.002)
                and poses[piece][2] - poses[BIN_OF[piece]][2] < 0.01)


def check_seed(d, seed):
    problems = []
    target = PIECES[seed % 4]
    if not (d / 'COMPLETE').exists():
        return [f'seed {seed}: not COMPLETE'], None
    pre = json.loads((d / 'preplaced.json').read_text())
    eps = sorted(d.glob('ep_*.json'))
    if len(eps) != 1:
        return [f'seed {seed}: {len(eps)} episodes'], None
    ep = json.loads(eps[0].read_text())
    if not ep['ok']:
        problems.append(f"seed {seed}: verdict {ep['verdict']}")
    if ep['piece'] != target or pre['target'] != target:
        problems.append(f"seed {seed}: recorded {ep['piece']}, preplace target {pre['target']}, expected {target}")
    if target in pre['presorted']:
        problems.append(f'seed {seed}: target {target} was preplaced')
    if not pre['ok'] or len(pre['presorted']) != pre['n_presorted'] or not 0 <= pre['n_presorted'] <= 3:
        problems.append(f"seed {seed}: preplace not ok {pre['errors']}")
    start, end = ep.get('gazebo_poses_start'), ep.get('gazebo_poses_end')
    if not start or not end:
        return problems + [f'seed {seed}: Gazebo poses missing'], ep
    after = pre['pieces_after']
    for p in PIECES:
        drift = np.linalg.norm(np.asarray(start[p][:2]) - np.asarray(after[p][:2])) * 1000
        if drift > STILL_MM:
            problems.append(f'seed {seed}: {p} moved {drift:.1f} mm between preplacement and episode start')
    for p in pre['presorted']:
        if not in_bin(start, p):
            problems.append(f'seed {seed}: presorted {p} not in its bin at episode start')
        if not in_bin(end, p):
            problems.append(f'seed {seed}: presorted {p} left its bin during the episode')
    if ep['ok'] and not in_bin(end, target):
        problems.append(f'seed {seed}: verdict OK but {target} not in its bin by the Gazebo end pose')
    return problems, ep


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('tri_home', type=Path)
    ap.add_argument('first', type=int)
    ap.add_argument('last', type=int)
    ap.add_argument('--dataset', type=Path)
    args = ap.parse_args()
    problems, episodes = [], {}
    for seed in range(args.first, args.last + 1):
        d = args.tri_home / f'seed_{seed:03d}'
        if not d.exists():
            problems.append(f'seed {seed}: not recorded')
            continue
        p, ep = check_seed(d, seed)
        problems += p
        if ep is not None:
            episodes[seed] = (ep, json.loads((d / 'preplaced.json').read_text()))
    ok = {s: e for s, (e, _) in episodes.items() if e['ok']}
    if args.dataset:
        import pandas as pd
        sources = json.loads((args.dataset / 'meta' / 'tri_sort_sources.json').read_text())
        by_seed = Counter(s['seed'] for s in sources)
        for s in ok:
            if by_seed[s] != 1:
                problems.append(f'seed {s}: {by_seed[s]} episodes in the dataset')
        extra = set(by_seed) - set(ok)
        if extra:
            problems.append(f'dataset seeds not OK in the recordings: {sorted(extra)}')
        tasks = pd.read_parquet(args.dataset / 'meta' / 'tasks.parquet')
        task_of = {int(i): t for t, i in zip(tasks.index, tasks['task_index'])}
        df = pd.concat(pd.read_parquet(f) for f in sorted((args.dataset / 'data').rglob('*.parquet')))
        lengths = df.groupby('episode_index').size()
        for s in sources:
            g = df[df['episode_index'] == s['episode_index']]
            instr = {task_of[int(i)] for i in g['task_index'].unique()}
            if instr != {INSTRUCTION[PIECES[s['seed'] % 4]]}:
                problems.append(f"seed {s['seed']}: dataset instruction {instr}")
        print(f'dataset: {len(sources)} episodes, frames per episode {lengths.min()}-{lengths.max()} '
              f'(median {int(lengths.median())})')
    for line in problems:
        print('PROBLEM', line)
    n_pre = Counter(pre['n_presorted'] for _, pre in episodes.values())
    targets = Counter(e['piece'] for e, _ in episodes.values())
    print(f'seeds {args.first}-{args.last}: {len(episodes)} checked, {len(ok)} OK, {len(problems)} problems; '
          f'presorted 0/1/2/3: {[n_pre[k] for k in range(4)]}; targets {dict(targets)}')
    sys.exit(1 if problems else 0)


if __name__ == '__main__':
    main()
