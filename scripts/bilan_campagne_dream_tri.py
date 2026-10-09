"""Bilan de la campagne 10 graines (vgg_tri_mix_ft_e10) : tri, DREAM<->YOLO par saisie, T_DREAM, keypoints.

usage : bilan_campagne_dream_tri.py RESULTATS_DIR RESUMES_DIR SORTIE.txt
  RESULTATS_DIR/seed_N/ : log_dir de chaque graine ; RESUMES_DIR/resume_N.txt : verdicts du trieur (lignes ✔/✘) et Tmax
"""
import csv
import re
import sys
from pathlib import Path

import numpy as np

R, O, OUT = Path(sys.argv[1]), Path(sys.argv[2]), Path(sys.argv[3])
CAMS = {'synth_camera': 'front', 'synth_camera_right': 'right'}


def med(v):
    return f'{np.median(v):.1f}' if len(v) else '—'


def p90(v):
    return f'{np.percentile(v, 90):.1f}' if len(v) else '—'


lines = ['graine | tri | Tmax °C | arrets | DREAM<->YOLO mm (par objet) | codeurs<->YOLO mm (médiane)']
all_dy, all_ey, n_ok, n_tot, grasp_missing = [], [], 0, 0, 0
pose = {c: {'t': [], 'r': []} for c in CAMS}
kp = {c: {'n': 0, 'v': 0, 'px': []} for c in CAMS}
per_obj = {}
for s in range(1, 11):
    d = R / f'seed_{s}'
    res = (O / f'resume_{s}.txt').read_text() if (O / f'resume_{s}.txt').exists() else ''
    ok = len(re.findall(r'✔', res))
    n_ok += ok
    n_tot += 4
    tmax = re.search(r'Tmax (\d+)', res)
    arrets = len(list(O.glob(f'resume_{s}_arret_*.txt')))
    dys, eys = [], []
    if (d / 'dream_vs_yolo.csv').exists():
        for row in csv.DictReader(open(d / 'dream_vs_yolo.csv')):
            if row['encoder_yolo_xy_mm']:
                eys.append(float(row['encoder_yolo_xy_mm']))
            if row['dream_yolo_xy_mm']:
                v = float(row['dream_yolo_xy_mm'])
                dys.append(v)
                per_obj.setdefault(row['object'], []).append(v)
            else:
                grasp_missing += 1
    all_dy += dys
    all_ey += eys
    if (d / 'dream_pose.csv').exists():
        for row in csv.DictReader(open(d / 'dream_pose.csv')):
            if row['camera_id'] in pose and row['t_error_mm']:
                pose[row['camera_id']]['t'].append(float(row['t_error_mm']))
                pose[row['camera_id']]['r'].append(float(row['r_error_deg']))
    if (d / 'dream_vs_fk.csv').exists():
        for row in csv.DictReader(open(d / 'dream_vs_fk.csv')):
            c = row['camera_id']
            if c in kp and row['fk_in_image'] == 'True':
                kp[c]['n'] += 1
                if row['valid'] == 'True':
                    kp[c]['v'] += 1
                    kp[c]['px'].append(float(row['pixel_error']))
    lines.append(f'{s:6d} | {ok}/4 | {tmax.group(1) if tmax else "—":>7s} | {arrets} | '
                 f'{" ".join(f"{v:.1f}" for v in dys) or "—"} | {med(eys)}')

lines += ['', f'tri : {n_ok}/{n_tot}',
          f'DREAM<->YOLO par saisie : {len(all_dy)} saisies mesurées ({grasp_missing} sans DREAM), '
          f'médiane {med(all_dy)} mm, p90 {p90(all_dy)} mm, max {max(all_dy) if all_dy else 0:.1f} mm',
          f'codeurs<->YOLO : {len(all_ey)} saisies, médiane {med(all_ey)} mm, p90 {p90(all_ey)} mm',
          'par objet (médiane mm) : ' + ', '.join(f'{k} {med(v)} (n={len(v)})' for k, v in sorted(per_obj.items())),
          '', 'T_DREAM (pose caméra par DREAM) : médiane (p90)']
for c, n in CAMS.items():
    lines.append(f'  {n:5s} : translation {med(pose[c]["t"])} ({p90(pose[c]["t"])}) mm, '
                 f'rotation {med(pose[c]["r"])} ({p90(pose[c]["r"])}) °, n={len(pose[c]["t"])}')
lines += ['', 'keypoints (FK dans l image) :']
for c, n in CAMS.items():
    k = kp[c]
    lines.append(f'  {n:5s} : détection {100 * k["v"] / max(k["n"], 1):.1f} % ({k["v"]}/{k["n"]}), '
                 f'erreur médiane {med(k["px"])} px, p90 {p90(k["px"])} px')
OUT.write_text('\n'.join(lines) + '\n')
print('\n'.join(lines))
