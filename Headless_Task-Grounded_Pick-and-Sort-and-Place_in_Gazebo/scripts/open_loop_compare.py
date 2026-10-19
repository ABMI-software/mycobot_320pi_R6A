#!/usr/bin/env python3
"""Paired comparison of open_loop_check.py results against a reference checkpoint.

  open_loop_compare.py <reference .json> <other .json> [<other .json> ...] [--boot 2000]

Chunks are paired by (episode, start); every file must hold the same chunks. For each metric,
the difference of medians (other minus reference; negative = other is closer to the recording)
with a 95 % bootstrap interval that resamples whole episodes, because the chunks of one episode
share its layout and are not independent. The grasp-point error only exists where both the
prediction and the recording close the gripper inside the chunk, so it is compared on the chunks
where both checkpoints close, and how many those are is reported.

For each file run with --swap-images (or its own-images twin), the donor share: the fraction of
predicted closes nearer the image donor's grasp than the episode's own grasp, over the chunks
that start with the gripper open, with a 95 % interval from the same episode bootstrap; and, between
the reference and each other file, the paired change in donor share (other minus reference).

Plain python3 with numpy (no ROS, no torch).
"""
import argparse
import json
from pathlib import Path

import numpy as np

METRICS = ('tip_mean_mm', 'joint_mean_deg', 'gripper_mean_abs', 'grasp_point_mm')


def load(path):
    rows = json.loads(Path(path).read_text())['chunks']
    return {(r['episode'], r['start']): r for r in rows}


def compare(ref, other, metric, boot, rng):
    keys = sorted(k for k in ref if metric in ref[k] and metric in other[k])
    episodes = sorted({k[0] for k in keys})
    by_ep = {e: [k for k in keys if k[0] == e] for e in episodes}
    a = {k: other[k][metric] for k in keys}
    b = {k: ref[k][metric] for k in keys}

    def diff(eps):
        ks = [k for e in eps for k in by_ep[e]]
        return np.median([a[k] for k in ks]) - np.median([b[k] for k in ks])

    point = diff(episodes)
    draws = [diff(rng.choice(episodes, len(episodes))) for _ in range(boot)]
    lo, hi = np.percentile(draws, [2.5, 97.5])
    better = np.mean([a[k] < b[k] for k in keys])
    return {'chunks': len(keys), 'episodes': len(episodes),
            'median_other': round(float(np.median(list(a.values()))), 3),
            'median_ref': round(float(np.median(list(b.values()))), 3),
            'diff': round(float(point), 3), 'ci95': [round(float(lo), 3), round(float(hi), 3)],
            'share_of_chunks_other_better': round(float(better), 3)}


def nearer_donor(path):
    rows = [r for r in json.loads(Path(path).read_text())['chunks'] if 'close_to_donor_grasp_mm' in r]
    by_ep = {}
    for r in rows:
        by_ep.setdefault(r['episode'], []).append(r['close_to_donor_grasp_mm'] < r['close_to_own_grasp_mm'])
    return by_ep


def share(by_ep, eps):
    v = [x for e in eps for x in by_ep.get(e, [])]
    return np.mean(v) if v else np.nan


def donor_share(path, boot):
    by_ep = nearer_donor(path)
    if not by_ep:
        return None
    episodes = sorted(by_ep)
    rng = np.random.default_rng(0)
    draws = [share(by_ep, rng.choice(episodes, len(episodes))) for _ in range(boot)]
    lo, hi = np.percentile(draws, [2.5, 97.5])
    n, closes = sum(sum(v) for v in by_ep.values()), sum(len(v) for v in by_ep.values())
    return {'closes': closes, 'episodes': len(episodes), 'nearer_donor': int(n),
            'share': round(n / closes, 3), 'ci95': [round(float(lo), 3), round(float(hi), 3)]}


def donor_share_change(ref_path, other_path, boot):
    ref, other = nearer_donor(ref_path), nearer_donor(other_path)
    if not ref or not other:
        return None
    episodes = sorted(set(ref) | set(other))
    rng = np.random.default_rng(0)
    draws = []
    for _ in range(boot):
        eps = rng.choice(episodes, len(episodes))
        draws.append(share(other, eps) - share(ref, eps))
    lo, hi = np.nanpercentile(draws, [2.5, 97.5])
    point = share(other, episodes) - share(ref, episodes)
    return {'diff': round(float(point), 3), 'ci95': [round(float(lo), 3), round(float(hi), 3)],
            'episodes': len(episodes)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('reference')
    ap.add_argument('others', nargs='+')
    ap.add_argument('--boot', type=int, default=2000)
    args = ap.parse_args()
    ref = load(args.reference)
    out = {}
    for path in args.others:
        other = load(path)
        if set(other) != set(ref):
            raise SystemExit(f'{path}: not the same chunks as {args.reference}')
        rng = np.random.default_rng(0)
        out[Path(path).stem] = {m: compare(ref, other, m, args.boot, rng) for m in METRICS}
        print(f'\n{Path(path).stem} minus {Path(args.reference).stem}')
        for m, r in out[Path(path).stem].items():
            print(f"  {m:17s} {r['median_other']:8.3f} vs {r['median_ref']:8.3f}  diff {r['diff']:+8.3f}  "
                  f"95% CI [{r['ci95'][0]:+.3f}, {r['ci95'][1]:+.3f}]  better on {r['share_of_chunks_other_better']:.0%} "
                  f"of {r['chunks']} chunks / {r['episodes']} episodes")
    for path in [args.reference] + args.others:
        share = donor_share(path, args.boot)
        if share:
            out.setdefault('donor_share', {})[Path(path).stem] = share
            print(f"donor share {Path(path).stem}: {share['nearer_donor']}/{share['closes']} = {share['share']:.0%} "
                  f"[95% CI {share['ci95'][0]:.0%}, {share['ci95'][1]:.0%}] ({share['episodes']} episodes)")
    for path in args.others:
        change = donor_share_change(args.reference, path, args.boot)
        if change:
            out.setdefault('donor_share_change', {})[Path(path).stem] = change
            print(f"donor share change {Path(path).stem} minus {Path(args.reference).stem}: {change['diff']:+.0%} "
                  f"[95% CI {change['ci95'][0]:+.0%}, {change['ci95'][1]:+.0%}] ({change['episodes']} episodes)")
    print(json.dumps(out))


if __name__ == '__main__':
    main()
