#!/usr/bin/env python3
"""DREAM arducam + SVPRO fusionnes : chaque keypoint triangule, erreur 3D en mm.

    ~/ros_jazzy/venv_dream/bin/python scripts/analyse_fusion_dream.py test
    ~/ros_jazzy/venv_dream/bin/python scripts/analyse_fusion_dream.py results/dream_reel/<session>

`test` : `real_tri/test` (images jamais vues), DREAM passe ici sur les deux vues de
chaque pose, extrinseques du 13/10 (`*.avant_1410*`, la capture date de ce jour-la).
Une session du dashboard : `keypoints_<camera>.csv` d'`analyse_tri_dream.py`
(keypoints DREAM deja calcules), images appariees a `APPARIEMENT_S` pres,
extrinseques courantes. Trois fenetres : tri bras immobile, tri en mouvement (du
depart vers la piece a la pose dans le bac), bras immobile sans tri.

Fusion : un keypoint vu par les deux cameras est triangule (DLT sur points non
distordus) a travers les deux extrinseques ArUco ; reference = FK des codeurs, en
3D, sans projection. Vu par une seule camera, il n'a pas de profondeur : il compte
dans la detection fusionnee (union) mais pas dans l'erreur 3D. La reprojection du
point triangule dans chaque image dit ce que la fusion apporte en pixels.
"""
import argparse
import csv
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path.home() / 'DREAM'))
sys.path.insert(0, str(RACINE / 'training' / 'dream'))
sys.path.insert(0, str(RACINE / 'mycobot_gateway' / 'mycobot_gateway' / 'vision'))
import camera_registry as registre  # noqa: E402
from mycobot_fk import KEYPOINT_NAMES, forward_kinematics  # noqa: E402

CALIB = RACINE / 'training' / 'calibration'
EXTR_COURANTES = {'arducam': 'arducam_extrinsic_pick.yaml', 'svpro': 'svpro_extrinsic_labels.yaml'}
EXTR_1310 = {'arducam': 'arducam_extrinsic_pick.avant_1410_1454.yaml',
             'svpro': 'svpro_extrinsic_labels.avant_1410.yaml'}
TEST = RACINE / 'training' / 'dream' / 'dream_data' / 'real_tri' / 'test'
MODELE = 'vgg_tri_reel_ft_e10'
NOMS = [n.replace('mycobot320_', '') for n in KEYPOINT_NAMES]
CAMERAS = ('arducam', 'svpro')
APPARIEMENT_S = 0.1
SEUILS_MM = (5, 10, 20, 50)


def cameras(noms):
    out = {}
    for cam, nom in noms.items():
        d = yaml.safe_load((CALIB / nom).read_text())
        K, dist = registre.load_intrinsics(d['intrinsics_stem'])
        out[cam] = (np.array(d['T_cam_world'], float), K, dist)
    return out


def triangule(cams, uv):
    """uv = {camera: (u, v)} sur les deux cameras -> point 3D (m), monde."""
    P, X = [], []
    for cam in CAMERAS:
        T, K, dist = cams[cam]
        P.append(T[:3])
        X.append(cv2.undistortPoints(np.array([[uv[cam]]], float), K, dist).reshape(2, 1))
    h = cv2.triangulatePoints(P[0], P[1], X[0], X[1]).ravel()
    return h[:3] / h[3]


def reprojette(cams, cam, p):
    T, K, dist = cams[cam]
    return cv2.projectPoints(p.reshape(1, 3), cv2.Rodrigues(T[:3, :3])[0], T[:3, 3], K, dist)[0].ravel()


def mesure(cams, poses):
    """poses : [(q_deg, {camera: [(u, v) | None] x 7})] -> lignes par pose."""
    out = []
    for q, kp in poses:
        pos, _ = forward_kinematics(np.radians(q))
        ligne = []
        for k, nom in enumerate(KEYPOINT_NAMES):
            ref = np.asarray(pos[nom], float)
            vus = [cam for cam in CAMERAS if kp[cam][k] is not None]
            e3d, rep = None, {}
            if len(vus) == 2:
                p = triangule(cams, {cam: kp[cam][k] for cam in CAMERAS})
                e3d = float(np.linalg.norm(p - ref)) * 1000.0
                for cam in CAMERAS:
                    rep[cam] = float(np.linalg.norm(reprojette(cams, cam, p) - reprojette(cams, cam, ref)))
            ligne.append({'vus': len(vus), 'e3d': e3d, 'rep': rep})
        out.append(ligne)
    return out


def f(x):
    return 'nan' if x is None or not np.isfinite(x) else f'{x:.2f}'.replace('.', ',')


def tableau(titre, lignes):
    n = len(lignes)
    print(f'\n### {titre} — {n} poses (une image par caméra)\n')
    print('| Keypoint | Moyenne (mm) | Médiane (mm) | Écart-type (mm) | Max (mm) | '
          'Vu par les 2 caméras | Vu par ≥ 1 caméra (fusion) |')
    print('|---|---|---|---|---|---|---|')
    tout, deux, un, rep = [], 0, 0, {cam: [] for cam in CAMERAS}
    for k in range(7):
        e = np.array([l[k]['e3d'] for l in lignes if l[k]['e3d'] is not None])
        d2 = sum(l[k]['vus'] == 2 for l in lignes)
        d1 = sum(l[k]['vus'] >= 1 for l in lignes)
        tout += list(e)
        deux += d2
        un += d1
        for cam in CAMERAS:
            rep[cam] += [l[k]['rep'][cam] for l in lignes if cam in l[k]['rep']]
        st = (e.mean(), np.median(e), e.std(), e.max()) if e.size else (None,) * 4
        print(f'| {NOMS[k]} | {" | ".join(f(v) for v in st)} | {f(100 * d2 / n)} % | {f(100 * d1 / n)} % |')
    t = np.array(tout)
    print(f'| **GLOBAL** | {f(t.mean())} | {f(np.median(t))} | {f(t.std())} | {f(t.max())} | '
          f'{f(100 * deux / (7 * n))} % | {f(100 * un / (7 * n))} % |')
    print('\n| ' + ' | '.join(f'< {s} mm' for s in SEUILS_MM) + ' |')
    print('|' + '---|' * len(SEUILS_MM))
    print('| ' + ' | '.join(f'{f(100 * np.mean(t < s))} %' for s in SEUILS_MM) + ' |')
    print('\nPoint triangulé reprojeté dans chaque image (médiane) : '
          + ', '.join(f'{cam} {f(np.median(rep[cam]))} px' for cam in CAMERAS) + '.')


def test(modele):
    import dream
    from PIL import Image
    poids = RACINE / 'training' / 'dream' / 'checkpoints_dream' / modele / 'best_network.pth'
    net = dream.create_network_from_config_file(str(poids.with_suffix('.yaml')), str(poids))
    par_pose = {}
    for l in csv.DictReader((TEST / 'labels.csv').open()):
        image = cv2.imread(str(TEST / l['image_path']))
        h, w = image.shape[:2]
        kp = net.keypoints_from_image(Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB)))['detected_keypoints']
        e = par_pose.setdefault(l['index'], {'q': [float(l[f'j{j}_deg']) for j in range(1, 7)], 'kp': {}})
        e['kp'][l['camera']] = [(float(u), float(v)) if np.isfinite([u, v]).all() and 0 <= u < w and 0 <= v < h
                                else None for u, v in kp]
    poses = [(e['q'], e['kp']) for e in par_pose.values() if set(e['kp']) == set(CAMERAS)]
    print(f'# DREAM {modele} — arducam + SVPRO fusionnées\n')
    tableau('1. Test sur images (real_tri_test, 13/10, extrinsèques du 13/10)', mesure(cameras(EXTR_1310), poses))


def session(dossier):
    s = json.loads((dossier / 'session.json').read_text())
    cycles = [(c['t0'], c['pose']) for c in s['cycles'] if c['serre'] and c['pose']]
    vues = {}
    for cam in CAMERAS:
        rows = list(csv.DictReader((dossier / f'keypoints_{cam}.csv').open()))
        vues[cam] = (np.array([float(r['t']) for r in rows]), rows)
    ta, ra = vues['arducam']
    ts, rs = vues['svpro']
    fenetres = {'2. Bras immobile pendant le tri': [], '3. Bras en mouvement pendant le tri': [],
                'Bras immobile sans tri (pose d’observation)': []}
    for t, r in zip(ta, ra):
        i = int(np.argmin(np.abs(ts - t)))
        if abs(ts[i] - t) > APPARIEMENT_S:
            continue
        kp = {cam: [(float(rr[f'{n}_u']), float(rr[f'{n}_v'])) if rr[f'{n}_u'] != '' else None for n in NOMS]
              for cam, rr in (('arducam', r), ('svpro', rs[i]))}
        q = [float(r[f'q{j}']) for j in range(1, 7)]
        statique = r['regime'] == 'statique' and rs[i]['regime'] == 'statique'
        dans = any(a <= t <= b for a, b in cycles)
        if dans:
            fenetres['2. Bras immobile pendant le tri' if statique else '3. Bras en mouvement pendant le tri'].append((q, kp))
        elif statique:
            fenetres['Bras immobile sans tri (pose d’observation)'].append((q, kp))
    cams = cameras(EXTR_COURANTES)
    print(f'# DREAM {s["modele"]} — arducam + SVPRO fusionnées — {dossier.name}\n')
    for titre, poses in fenetres.items():
        if poses:
            tableau(titre, mesure(cams, poses))


def main():
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument('source', help='« test » ou un dossier de session du dashboard')
    a.add_argument('--modele', default=MODELE)
    args = a.parse_args()
    if args.source == 'test':
        test(args.modele)
    else:
        session(Path(args.source))


if __name__ == '__main__':
    main()
