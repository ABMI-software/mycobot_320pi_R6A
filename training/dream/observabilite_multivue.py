"""J1-J6 retrouves depuis les keypoints DREAM, caméras connues (simulation) : 1 vue contre 2 vues.

Entrée : un ou plusieurs dream_vs_fk.csv (une ligne par keypoint, caméra et image ; q vrai en degrés).
Caméras : géométrie dream50k du URDF Gazebo (monde = base du robot), donc aucune erreur d'extrinsèque :
ce qui reste est l'effet des keypoints et de la géométrie seule.

Trois sources de keypoints :
  dream  : détections DREAM réelles du run ;
  fk+3px : projection FK exacte + bruit gaussien de 3 px (taille de l'erreur DREAM mesurée), mêmes keypoints valides ;
  fk     : projection FK exacte (borne : 0 px).
Deux initialisations :
  froid  : 6 départs (zéro + 5 aléatoires dans les butées), on garde le plus faible coût ;
  oracle : départ sur le q vrai (précision locale seule).
Aucun a priori sur q (pas de rappel vers les codeurs).

usage : observabilite_multivue.py SORTIE.txt dream_vs_fk.csv [...]
        observabilite_multivue.py --fusion SORTIE.txt a.npz b.npz [...]  (runs lancés par graine, en parallèle)
Chaque run écrit aussi SORTIE.npz (erreurs brutes), que --fusion réunit.
Une vue = une caméra ; Gazebo doit avoir tourné avec log_dir et dream:=true.
"""
import csv
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(ROOT / 'training' / 'dream'))
sys.path.insert(0, str(ROOT / 'mycobot_gateway'))
from mycobot_fk import KEYPOINT_NAMES, forward_kinematics  # noqa: E402
from mycobot_gateway.vision.sim_multicam_geometry import load_cameras  # noqa: E402



def report(out, n_frames, path):
    lines = [f'images retenues (2 vues, >= 4 kp chacune) : {n_frames}',
             'erreur angulaire |q_estime - q_vrai|, en degrés : médiane (p90) par articulation', '']
    for (src, init, name), errs in sorted(out.items()):
        e = np.array(errs)
        cells = '  '.join(f'J{j + 1} {np.median(e[:, j]):6.1f} ({np.percentile(e[:, j], 90):6.1f})'
                          for j in range(6))
        lines.append(f'{src:7s} {init:6s} {name:24s} {cells}')
    Path(path).write_text('\n'.join(lines) + '\n')
    print('\n'.join(lines))


if sys.argv[1] == '--fusion':
    acc, n = {}, 0
    for f in sys.argv[3:]:
        z = np.load(f)
        n += int(z['n_frames'])
        for k in z.files:
            if k != 'n_frames':
                acc.setdefault(tuple(k.split('|')), []).append(z[k])
    report({k: np.concatenate(v) for k, v in acc.items()}, n, sys.argv[2])
    sys.exit()

LOW = np.array([-2.93, -2.35, -2.53, -2.53, -2.93, -3.14])
UP = -LOW
NAMES = [k.replace('mycobot320_', '') for k in KEYPOINT_NAMES]
rng = np.random.default_rng(0)

cams = load_cameras(ROOT / 'mycobot_description/urdf/320_pi/mycobot_pro_320_pi_gazebo.urdf',
                    {'camera_layout': 'dream50k'})


def project(cam, q):
    pos, _ = forward_kinematics(q)
    return cams[cam].project(np.array([pos[k] for k in KEYPOINT_NAMES]))


def solve(views, q0):
    def res(q):
        return np.concatenate([(project(c, q)[m] - uv[m]).ravel() for c, uv, m in views])
    r = least_squares(res, np.clip(q0, LOW + 1e-3, UP - 1e-3), bounds=(LOW, UP), method='trf')
    return r.x, r.cost


def solve_cold(views):
    starts = [np.zeros(6)] + [rng.uniform(LOW, UP) * 0.8 for _ in range(5)]
    return min((solve(views, s) for s in starts), key=lambda x: x[1])[0]


def wrap_deg(a):
    return (np.degrees(a) + 180) % 360 - 180


frames = defaultdict(dict)
for f in sys.argv[2:]:
    for row in csv.DictReader(open(f)):
        key = (f, round(float(row['stamp_sim']), 1))
        fr = frames[key].setdefault(row['camera_id'], {'uv': np.full((7, 2), np.nan),
                                                        'valid': np.zeros(7, bool)})
        i = NAMES.index(row['keypoint'])
        fr['q'] = np.radians([float(row[f'q{j}']) for j in range(1, 7)])
        if row['valid'] == 'True':
            fr['uv'][i] = float(row['u_dream']), float(row['v_dream'])
            fr['valid'][i] = True

out = defaultdict(list)
n_frames = 0
for key, per_cam in frames.items():
    if len(per_cam) < 2 or any(v['valid'].sum() < 4 for v in per_cam.values()):
        continue
    qs = [v['q'] for v in per_cam.values()]
    if np.max(np.abs(qs[0] - qs[1])) > np.radians(0.5):
        continue
    q = qs[0]
    n_frames += 1
    for src in ('dream', 'fk+3px', 'fk'):
        views = {}
        for cam, v in per_cam.items():
            if src == 'dream':
                uv = v['uv']
            else:
                uv = project(cam, q) + (rng.normal(0, 3, (7, 2)) if src == 'fk+3px' else 0)
            views[cam] = (cam, uv, v['valid'])
        configs = {f'1 vue {c}': [views[c]] for c in views}
        configs['2 vues'] = list(views.values())
        for name, vs in configs.items():
            for init in ('froid', 'oracle'):
                qh = solve_cold(vs) if init == 'froid' else solve(vs, q)[0]
                out[(src, init, name)].append(np.abs(wrap_deg(qh - q)))

np.savez(Path(sys.argv[1]).with_suffix('.npz'), n_frames=n_frames,
         **{'|'.join(k): np.array(v) for k, v in out.items()})
report(out, n_frames, sys.argv[1])
