#!/usr/bin/env python3
"""Per-frame piece positions for Run C's auxiliary head (design note D3, 19/10/2026).

  build_aux_labels.py <tri home> <LeRobot dataset root>

Writes <dataset root>/meta/aux_labels.npz: for every frame (by its dataset `index`) the x, y of
the four pieces in PIECES order (metres, Gazebo world) and which of them are valid, plus the
training-set mean and spread used to normalise them. The policy never reads this file; the
launcher looks the labels up by `index`. A one-episode-per-layout dataset only
(PREPLACE=1 recordings, the episode's JSON carries gazebo_poses_start / _end):

  non-target piece  its start position on every frame, unless start and end differ by more than
                    DRIFT_MAX_MM (then masked for the whole episode)
  target piece      its start position before the gripper's first close command; its end
                    position from SETTLE_S after the release command to the end; masked while
                    it is in the gripper (no measured pose there)

The gripper command is action[6]; more negative is more closed: start opening, a step down at the
grasp, a step back up at the release, 0 at the retract.

/workspace/venv_lerobot/bin/python (pandas).
"""
import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pandas as pd

sys.path.insert(0, str(Path(__file__).resolve().parent))
from preplace_pieces import PIECES  # noqa: E402

DRIFT_MAX_MM = 5.0
SETTLE_S = 1.0
STEP = 0.05


def gripper_events(cmd):
    """(first close frame, release frame) from the gripper command, or None where not found."""
    close = next((i for i in range(len(cmd)) if cmd[i] < cmd[0] - STEP), None)
    if close is None:
        return None, None
    release = next((i for i in range(close, len(cmd)) if cmd[i] > cmd[close] + STEP), None)
    return close, release


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('tri_home', type=Path)
    ap.add_argument('dataset', type=Path)
    args = ap.parse_args()
    fps = json.loads((args.dataset / 'meta' / 'info.json').read_text())['fps']
    sources = json.loads((args.dataset / 'meta' / 'tri_sort_sources.json').read_text())
    df = pd.concat(pd.read_parquet(f) for f in sorted((args.dataset / 'data').rglob('*.parquet')))
    index, episode, xy, mask = [], [], [], []
    report = {'episodes': 0, 'target_without_events': [], 'masked_drift': []}
    for s in sources:
        name = 'ep_' + s['source'].split('_ep_', 1)[1] + '.json'
        ep = json.loads((args.tri_home / f"seed_{s['seed']:03d}" / name).read_text())
        if ep['piece'] != s['piece']:
            raise SystemExit(f"episode {s['episode_index']}: source {s['source']} is {ep['piece']}")
        start, end = ep['gazebo_poses_start'], ep['gazebo_poses_end']
        g = df[df['episode_index'] == s['episode_index']].sort_values('frame_index')
        cmd = np.stack(g['action'].values)[:, 6]
        n = len(g)
        close, release = gripper_events(cmd)
        m = np.ones((n, len(PIECES)), dtype=bool)
        p = np.zeros((n, len(PIECES), 2), dtype=np.float32)
        for j, piece in enumerate(PIECES):
            p[:, j] = start[piece][:2]
            if piece == ep['piece']:
                if close is None or release is None:
                    m[:, j] = False
                    report['target_without_events'].append(s['episode_index'])
                    continue
                settled = release + int(round(SETTLE_S * fps))
                m[close:settled, j] = False
                p[settled:, j] = end[piece][:2]
            elif np.linalg.norm(np.subtract(start[piece][:2], end[piece][:2])) * 1000 > DRIFT_MAX_MM:
                m[:, j] = False
                report['masked_drift'].append([s['episode_index'], piece])
        index.append(g['index'].to_numpy())
        episode.append(np.full(n, s['episode_index']))
        xy.append(p.reshape(n, -1))
        mask.append(m)
        report['episodes'] += 1
    index, episode = np.concatenate(index), np.concatenate(episode)
    xy, mask = np.concatenate(xy), np.concatenate(mask)
    valid = np.repeat(mask, 2, axis=1)
    mean = np.array([xy[valid[:, k], k].mean() for k in range(xy.shape[1])], dtype=np.float32)
    std = np.array([xy[valid[:, k], k].std() for k in range(xy.shape[1])], dtype=np.float32)
    out = args.dataset / 'meta' / 'aux_labels.npz'
    np.savez(out, index=index, episode=episode, xy=xy, mask=mask, mean=mean, std=std,
             pieces=np.array(PIECES), drift_max_mm=DRIFT_MAX_MM, settle_s=SETTLE_S)
    report.update(frames=int(len(index)), valid_share_per_piece=[round(float(v), 3) for v in mask.mean(0)],
                  all_four_valid_share=round(float(mask.all(1).mean()), 3),
                  std_mm=[round(float(v) * 1000, 1) for v in std], out=str(out))
    print(json.dumps(report, indent=1))


if __name__ == '__main__':
    main()
