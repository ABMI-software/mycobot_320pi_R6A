#!/usr/bin/env python3
"""Summarise a record_tri_batch.sh run: per seed, each piece's offset from its
bin centre and the wall time; per episode, frames per camera, simulated
seconds and bag size. Reads only the batch's own files.

  summarise_tri_batch.py [/workspace/tri_sort] [--json out.json]
"""
import argparse
import json
import re
import statistics
import subprocess
from datetime import datetime
from pathlib import Path

import yaml

COLOURS = {'cube_rouge': 'red', 'pave_jaune': 'yellow', 'cylindre_vert': 'green', 'cube_bleu': 'blue'}
CAMERAS = ('/synth_camera_top/image', '/synth_camera_right/image', '/synth_camera_left/image')
OFFSET = re.compile(r'ecart ([+-]\d+)/([+-]\d+) mm')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('home', nargs='?', default='/workspace/tri_sort', type=Path)
    ap.add_argument('--json', type=Path)
    args = ap.parse_args()
    log = (args.home / 'batch.log').read_text().splitlines()
    stamp = lambda line: datetime.strptime(line[1:20], '%Y-%m-%d %H:%M:%S')
    starts = {int(m.group(1)): stamp(l) for l in log if (m := re.search(r'seed (\d+): start', l))}
    dones = {int(m.group(1)): stamp(l) for l in log if (m := re.search(r'seed (\d+): done', l))}

    seeds, episodes = [], []
    for d in sorted(args.home.glob('seed_[0-9][0-9][0-9]')):
        seed = int(d.name[5:])
        row = {'seed': seed}
        for meta_path in sorted(d.glob('ep_*.json')):
            m = json.loads(meta_path.read_text())
            off = OFFSET.search(m['verdict'])
            dx, dy = (int(off.group(1)), int(off.group(2))) if off else (None, None)
            info = yaml.safe_load((Path(m['bag_path']) / 'metadata.yaml').read_text())
            counts = {t['topic_metadata']['name']: t['message_count']
                      for t in info['rosbag2_bagfile_information']['topics_with_message_count']}
            size = int(subprocess.run(['du', '-sb', m['bag_path']], capture_output=True, text=True).stdout.split()[0])
            episodes.append({'seed': seed, 'piece': m['piece'], 'attempt': m['attempt'], 'ok': m['ok'],
                             'dx': dx, 'dy': dy, 'sim_seconds': m['sim_seconds'],
                             'frames': [counts.get(c, 0) for c in CAMERAS], 'bag_bytes': size})
            if m['ok'] and m['attempt'] == 1:
                row[COLOURS[m['piece']]] = f'{dx:+d}/{dy:+d}'
            elif m['ok']:
                row[COLOURS[m['piece']]] = f'{dx:+d}/{dy:+d} (attempt {m["attempt"]})'
            else:
                row.setdefault(COLOURS[m['piece']], 'FAILED')
        if seed in starts and seed in dones:
            row['wall_min'] = round((dones[seed] - starts[seed]).total_seconds() / 60, 1)
        seeds.append(row)

    ok = [e for e in episodes if e['ok']]
    first = [e for e in ok if e['attempt'] == 1]
    dist = [((e['dx'] ** 2 + e['dy'] ** 2) ** 0.5) for e in ok]
    frames = [f for e in ok for f in e['frames']]
    summary = {
        'seeds': seeds, 'episodes': episodes, 'total': len(seeds) * 4, 'ok': len(ok), 'first_attempt': len(first),
        'retries': len(episodes) - len(ok),
        'start': min(starts.values()).strftime('%Y-%m-%d %H:%M') if starts else None,
        'end': max(dones.values()).strftime('%H:%M') if dones else None,
        'median_offset_mm': round(statistics.median(dist), 1), 'max_offset_mm': round(max(dist), 1),
        'seed_minutes': [r.get('wall_min') for r in seeds],
        'frames_min': min(frames), 'frames_median': int(statistics.median(frames)), 'frames_max': max(frames),
        'sim_min': round(min(e['sim_seconds'] for e in ok), 1), 'sim_max': round(max(e['sim_seconds'] for e in ok), 1),
        'bag_gb': round(sum(e['bag_bytes'] for e in episodes) / 1e9, 1),
    }
    for r in seeds:
        print(r)
    print({k: v for k, v in summary.items() if k not in ('seeds', 'episodes')})
    if args.json:
        args.json.write_text(json.dumps(summary, indent=2) + '\n')


if __name__ == '__main__':
    main()
