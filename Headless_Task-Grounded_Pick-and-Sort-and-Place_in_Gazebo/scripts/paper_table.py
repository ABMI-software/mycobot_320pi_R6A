#!/usr/bin/env python3
"""Pick / place / place-after-pick table in the ROS2SmolVLA paper's format (arXiv:2608.23320).

  paper_table.py <eval root> [--out table.json]

Reads <eval root>/{ood,id}_s*/result.json and failure_modes.json (eval_tri_seed.sh,
failure_modes.py). Rows: in distribution (id_*), out of distribution (ood_*), overall (all pieces
pooled). Pick = picked / pieces, place = placed / pieces, place after pick = placed after pick /
picked (conditional on the pick, as the paper's column, which is why it can exceed place). 95 %
Wilson intervals. Also the failure modes per row, with "never closed" counted on its own.
Plain python3.
"""
import argparse
import json
import math
from collections import Counter
from pathlib import Path

ROWS = (('in distribution', 'id_s'), ('out of distribution', 'ood_s'))


def wilson(k, n, z=1.96):
    if n == 0:
        return None
    p = k / n
    centre = (p + z * z / (2 * n)) / (1 + z * z / n)
    half = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / (1 + z * z / n)
    return [round(100 * (centre - half), 1), round(100 * (centre + half), 1)]


def rate(k, n):
    return {'k': k, 'n': n, 'pct': round(100 * k / n, 1) if n else None, 'ci95': wilson(k, n)}


def row(dirs):
    pieces, modes = [], Counter()
    for d in dirs:
        pieces += json.loads((d / 'result.json').read_text())['pieces']
        modes.update(m['mode'] for m in json.loads((d / 'failure_modes.json').read_text()))
    n = len(pieces)
    picked = sum(bool(p.get('picked')) for p in pieces)
    placed = sum(bool(p.get('placed')) for p in pieces)
    after = sum(bool(p.get('placed_after_pick')) for p in pieces)
    return {'seeds': [json.loads((d / 'result.json').read_text())['seed'] for d in dirs], 'pieces': n,
            'errors': sum('error' in p for p in pieces),
            'pick': rate(picked, n), 'place': rate(placed, n), 'place_after_pick': rate(after, picked),
            'modes': dict(modes.most_common()), 'never_closed': modes.get('never closed', 0)}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('root', type=Path)
    ap.add_argument('--out', type=Path)
    args = ap.parse_args()
    table, every = {}, []
    for name, prefix in ROWS:
        dirs = sorted(d for d in args.root.glob(prefix + '*') if d.is_dir())
        every += dirs
        table[name] = row(dirs)
    table['overall'] = row(every)
    print(f"{'':22s}{'pick':>22s}{'place':>22s}{'place after pick':>24s}   never closed")
    for name, r in table.items():
        cells = [f"{c['k']}/{c['n']} {c['pct']} % [{c['ci95'][0]}, {c['ci95'][1]}]" if c['n'] else 'n/a'
                 for c in (r['pick'], r['place'], r['place_after_pick'])]
        print(f'{name:22s}{cells[0]:>22s}{cells[1]:>22s}{cells[2]:>24s}   {r["never_closed"]}/{r["pieces"]}')
    for name, r in table.items():
        print(f'{name}: {r["modes"]}')
    if args.out:
        args.out.write_text(json.dumps(table, indent=1) + '\n')


if __name__ == '__main__':
    main()
