"""One summary row per sort, appended to bilan_tris.csv, and the trial's raw rows.

Static = the arm at the observation pose (every joint within STATIC_TOL_DEG of
tri_scene.OBSERVATION_Q_DEG, at the samples on both sides of the image), as at
launch and between two objects; dynamic = the rest of the sort. DREAM rows come
from dream_vs_fk.csv, YOLO rows from yolo_vs_gt.csv, both cut to the sort's
simulated-time window; the grasp errors come from the dashboard's
tri_resultats.csv, rows dated after the sort started. The raw rows of the
window are copied to <trial folder>/brut/ for detailed analyses.
"""
import bisect
import csv
import datetime
from pathlib import Path

import numpy as np

from .vision import tri_scene

STATIC_TOL_DEG = 2.0
# A piece that left its launch height has been picked: YOLO is scored on it
# only before, as the sorter localizes it only there.
MOVED_M = 0.003
PIECES = tri_scene.OBJECTS
RAW_FILES = ('dream_vs_fk.csv', 'yolo_vs_gt.csv', 'dream_pose.csv')
COLUMNS = ['date', 'essai', 'seed', 'tirage', 'objets_tries', 'duree_sim_s',
           'dream_stat_images', 'dream_stat_detection_pct', 'dream_stat_mediane_px',
           'dream_dyn_images', 'dream_dyn_detection_pct', 'dream_dyn_mediane_px',
           'yolo_stat_n', 'yolo_stat_detection_pct', 'yolo_stat_mediane_px', 'yolo_stat_mediane_mm',
           'yolo_dyn_n', 'yolo_dyn_detection_pct', 'yolo_dyn_mediane_px', 'yolo_dyn_mediane_mm',
           'dream_gazebo_mediane_mm', 'yolo_gazebo_mediane_mm', 'codeurs_gazebo_mediane_mm']


class ArmTimeline:
    """(sim time, joint angles in degrees) samples, recorded during the sort."""

    def __init__(self, times, q_deg):
        self.times = list(times)
        self.static = [bool(np.max(np.abs(np.asarray(q) - tri_scene.OBSERVATION_Q_DEG)) < STATIC_TOL_DEG)
                       for q in q_deg]

    def is_static(self, t):
        i = bisect.bisect_left(self.times, t)
        around = self.static[max(i - 1, 0):i + 1]
        return bool(around) and all(around)


def _rows(path, t0, t1):
    if not path.exists():
        return []
    with open(path, newline='') as fh:
        return [r for r in csv.DictReader(fh) if t0 <= float(r['stamp_sim']) <= t1]


def _fmt(v, digits=2):
    return '' if v is None else f'{v:.{digits}f}'


def dream_stats(rows):
    rows = [r for r in rows if r['fk_in_image'] == 'True']
    if not rows:
        return '', '', ''
    images = len({(r['camera_id'], r['stamp_sim']) for r in rows})
    errors = [float(r['pixel_error']) for r in rows if r['valid'] == 'True']
    detection = 100 * len(errors) / len(rows)
    return images, _fmt(detection, 1), _fmt(float(np.median(errors)) if errors else None)


def yolo_stats(rows):
    rows = [r for r in rows if r['hidden'] == 'False' and r['truncated'] == 'False'
            and r['match'] in ('TP', 'FN', 'wrong_class')]
    if not rows:
        return '', '', '', ''
    tp = [r for r in rows if r['match'] == 'TP']
    px = [float(r['pixel_error']) for r in tp]
    mm = [float(r['error_3d']) for r in tp if r['error_3d']]
    return (len(rows), _fmt(100 * len(tp) / len(rows), 1),
            _fmt(float(np.median(px)) if px else None), _fmt(float(np.median(mm)) if mm else None))


def resting_yolo_rows(rows):
    """Bins, and pieces until they leave their launch height."""
    first, moved = {}, {}
    for r in rows:
        o = r['object_id']
        if o in PIECES and r['z_gt']:
            z = float(r['z_gt'])
            first.setdefault(o, z)
            if abs(z - first[o]) > MOVED_M:
                moved.setdefault(o, float(r['stamp_sim']))
    return [r for r in rows if r['object_id']
            and not (r['object_id'] in moved and float(r['stamp_sim']) >= moved[r['object_id']])]


def grasp_errors(path, since_iso):
    if not path.exists():
        return None, None, None
    with open(path, newline='') as fh:
        rows = [r for r in csv.DictReader(fh) if r['date'] >= since_iso and r['verdict'] != 'en cours']

    def median(key):
        v = [float(r[key]) for r in rows if r[key]]
        return float(np.median(v)) if v else None
    return (median('err_dream_gazebo_xy_mm'), median('err_yolo_gazebo_xy_mm'),
            median('err_encoder_gazebo_xy_mm'))


def sorted_count(tri_csv):
    if not tri_csv.exists():
        return 0, 0
    with open(tri_csv, newline='') as fh:
        rows = list(csv.DictReader(fh))
    objects = {r['object_id'] for r in rows}
    return sum(any(r['ok'] == 'True' for r in rows if r['object_id'] == o) for o in objects), len(objects)


def copy_raw(log_dir, brut, t0, t1):
    for name in RAW_FILES:
        src = Path(log_dir) / name
        if not src.exists():
            continue
        with open(src, newline='') as fh:
            reader = csv.reader(fh)
            header = next(reader)
            i = header.index('stamp_sim')
            rows = [r for r in reader if t0 <= float(r[i]) <= t1]
        with open(Path(brut) / name, 'w', newline='') as fh:
            w = csv.writer(fh)
            w.writerow(header)
            w.writerows(rows)


def write_bilan(log_dir, essai_dir, bilan_csv, seed, draw, t0, t1, since_iso, timeline, tri_csv):
    """Append the row of the sort run between sim times t0 and t1; returns it."""
    log_dir, essai_dir = Path(log_dir), Path(essai_dir)
    copy_raw(log_dir, essai_dir / 'brut', t0, t1)
    dream = _rows(log_dir / 'dream_vs_fk.csv', t0, t1)
    yolo = resting_yolo_rows(_rows(log_dir / 'yolo_vs_gt.csv', t0, t1))
    ok, total = sorted_count(Path(tri_csv))
    dream_gz, yolo_gz, enc_gz = grasp_errors(log_dir / 'tri_resultats.csv', since_iso)
    row = [datetime.datetime.now().isoformat(timespec='seconds'),
           essai_dir.name, seed, draw, f'{ok}/{total}',
           f'{t1 - t0:.1f}',
           *dream_stats([r for r in dream if timeline.is_static(float(r['stamp_sim']))]),
           *dream_stats([r for r in dream if not timeline.is_static(float(r['stamp_sim']))]),
           *yolo_stats([r for r in yolo if timeline.is_static(float(r['stamp_sim']))]),
           *yolo_stats([r for r in yolo if not timeline.is_static(float(r['stamp_sim']))]),
           _fmt(dream_gz, 1), _fmt(yolo_gz, 1), _fmt(enc_gz, 1)]
    path = Path(bilan_csv)
    new = not path.exists()
    with open(path, 'a', newline='') as fh:
        w = csv.writer(fh)
        if new:
            w.writerow(COLUMNS)
        w.writerow(row)
    return dict(zip(COLUMNS, row))
