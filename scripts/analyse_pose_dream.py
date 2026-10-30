#!/usr/bin/env python3
"""Pose camera estimee par DREAM, comparee a l'extrinseque ArUco : ΔXYZ (mm), Δrotation (deg).

    ~/ros_jazzy/venv_dream/bin/python scripts/analyse_pose_dream.py test
    ~/ros_jazzy/venv_dream/bin/python scripts/analyse_pose_dream.py results/dream_reel/<session>

L'architecture DREAM : image -> 7 keypoints 2D (VGG-19), FK des codeurs -> les
memes 7 points en 3D, PnP -> T_DREAM, pose camera/robot. Aucune extrinseque n'entre
dans le calcul. L'extrinseque ArUco tient ici le role de T_GT dans Gazebo : la
reference de la boite « Comparaison », exterieure a DREAM.

Ecarts calcules comme le protocole Gazebo (`dream_fk_compare.pose_error`) :
d = t_DREAM - t_ref dans le repere camera (dz = profondeur, axe optique), et
l'angle de R_ref^T . R_DREAM. PnP sur >= `KP_MIN` keypoints, comme le dashboard
(sur 4 points il est mal conditionne). Medianes (p90), comme le tableau du 05/10.

Sources : `test` (real_tri/test, DREAM passe ici, extrinseques du 13/10) ou une
session du dashboard (`keypoints_<camera>.csv` d'`analyse_tri_dream.py`,
extrinseques courantes), en trois fenetres comme `analyse_fusion_dream.py`.
"""
import argparse
import csv
import json
import sys
from pathlib import Path

import cv2
import numpy as np

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE / 'scripts'))
import analyse_fusion_dream as af  # noqa: E402

KP_MIN = 5


def pose_dream(q, kp, K, dist):
    vus = [k for k in range(7) if kp[k] is not None]
    if len(vus) < KP_MIN:
        return None
    pos, _ = af.forward_kinematics(np.radians(q))
    obj = np.array([pos[af.KEYPOINT_NAMES[k]] for k in vus], float)
    img = np.array([kp[k] for k in vus], float)
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist, flags=cv2.SOLVEPNP_EPNP)
    if not ok:
        return None
    rvec, tvec = cv2.solvePnPRefineLM(obj, img, K, dist, rvec, tvec)
    return cv2.Rodrigues(rvec)[0], tvec.ravel()


def ecarts(T_ref, R, t):
    d = (t - T_ref[:3, 3]) * 1000.0
    angle = np.degrees(np.arccos(np.clip((np.trace(T_ref[:3, :3].T @ R) - 1) / 2, -1, 1)))
    return d, float(angle)


def f(x):
    return f'{x:.2f}'.replace('.', ',')


def tableau(titre, cams, poses):
    print(f'\n### {titre}\n')
    print('| caméra | vues ≥ 5 kp | translation médiane (p90) | \\|dz\\| | \\|dx\\| / \\|dy\\| | '
          'rotation médiane (p90) |')
    print('|---|---|---|---|---|---|')
    for cam in af.CAMERAS:
        T_ref, K, dist = cams[cam]
        d, a = [], []
        for q, kp in poses:
            if cam not in kp:
                continue
            p = pose_dream(q, kp[cam], K, dist)
            if p is not None:
                dxyz, ang = ecarts(T_ref, *p)
                d.append(dxyz)
                a.append(ang)
        n = sum(cam in kp for _, kp in poses)
        d, a = np.abs(np.array(d)), np.array(a)
        nd = np.linalg.norm(d, axis=1)
        med = np.median(d, axis=0)
        print(f'| {cam} | {len(a)} / {n} | {f(np.median(nd))} ({f(np.percentile(nd, 90))}) mm | '
              f'{f(med[2])} mm | {f(med[0])} / {f(med[1])} mm | '
              f'{f(np.median(a))} ({f(np.percentile(a, 90))})° |')


def test(modele):
    import dream
    from PIL import Image
    poids = RACINE / 'training' / 'dream' / 'checkpoints_dream' / modele / 'best_network.pth'
    net = dream.create_network_from_config_file(str(poids.with_suffix('.yaml')), str(poids))
    poses = []
    for l in csv.DictReader((af.TEST / 'labels.csv').open()):
        image = cv2.imread(str(af.TEST / l['image_path']))
        h, w = image.shape[:2]
        kp = net.keypoints_from_image(Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB)))['detected_keypoints']
        poses.append(([float(l[f'j{j}_deg']) for j in range(1, 7)],
                      {l['camera']: [(float(u), float(v)) if np.isfinite([u, v]).all() and 0 <= u < w
                                     and 0 <= v < h else None for u, v in kp]}))
    print(f'# DREAM {modele} — pose caméra (T_DREAM) contre ArUco\n')
    tableau('1. Test sur images (real_tri_test, 13/10, référence ArUco du 13/10)', af.cameras(af.EXTR_1310), poses)


def session(dossier):
    s = json.loads((dossier / 'session.json').read_text())
    cycles = [(c['t0'], c['pose']) for c in s['cycles'] if c['serre'] and c['pose']]
    fenetres = {'2. Bras immobile pendant le tri': [], '3. Bras en mouvement pendant le tri': [],
                'Bras immobile sans tri (pose d’observation)': []}
    for cam in af.CAMERAS:
        for r in csv.DictReader((dossier / f'keypoints_{cam}.csv').open()):
            t = float(r['t'])
            kp = {cam: [(float(r[f'{n}_u']), float(r[f'{n}_v'])) if r[f'{n}_u'] != '' else None for n in af.NOMS]}
            q = [float(r[f'q{j}']) for j in range(1, 7)]
            if any(a <= t <= b for a, b in cycles):
                fenetres['2. Bras immobile pendant le tri' if r['regime'] == 'statique'
                         else '3. Bras en mouvement pendant le tri'].append((q, kp))
            elif r['regime'] == 'statique':
                fenetres['Bras immobile sans tri (pose d’observation)'].append((q, kp))
    print(f'# DREAM {s["modele"]} — pose caméra (T_DREAM) contre ArUco — {dossier.name}\n')
    cams = af.cameras(af.EXTR_COURANTES)
    for titre, poses in fenetres.items():
        tableau(titre, cams, poses)


def main():
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument('source', help='« test » ou un dossier de session du dashboard')
    a.add_argument('--modele', default=af.MODELE)
    args = a.parse_args()
    if args.source == 'test':
        test(args.modele)
    else:
        session(Path(args.source))


if __name__ == '__main__':
    main()
