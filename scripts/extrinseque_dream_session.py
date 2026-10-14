#!/usr/bin/env python3
"""Extrinseque de chaque camera par DREAM seul, sur les images d'un tri du dashboard.

    ~/ros_jazzy/venv_dream/bin/python scripts/extrinseque_dream_session.py results/dream_reel/<session>

Le robot est sa propre mire : pour chaque image prise pendant le tri (du depart
vers la piece a la pose dans le bac, J5 dans le domaine du modele), DREAM donne
les 7 keypoints 2D et la FK des codeurs les memes points en 3D. Toutes les poses
sont empilees dans UN PnP (RANSAC sur les correspondances, puis affinage
robuste) : sur une seule image les 7 points sont quasi coplanaires et la pose
est mal conditionnee, d'ou les 8 mm / 1,4 deg de dispersion image par image.

Aucun ArUco n'entre dans le calcul. L'extrinseque ArUco ne sert qu'a juger, a
la fin, sur ce qui compte pour la saisie : un point de la planche (Z = 0) vu par
la camera, ramene sur la table par l'extrinseque DREAM au lieu de l'ArUco —
l'ecart en mm est l'erreur que DREAM ajouterait a la position YOLO d'un objet.
Stabilite : la meme estimation sur la premiere moitie du tri puis la seconde (pieces
differentes ; des images paires/impaires seraient a 0,2 s l'une de l'autre, donc dependantes).

`--ecrire` enregistre `<camera>_extrinsic_dream_tri.yaml` (jamais un fichier existant).
"""
import argparse
import csv
import json
import sys
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import yaml
from PIL import Image
from scipy.optimize import least_squares

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(Path.home() / 'DREAM'))
sys.path.insert(0, str(RACINE / 'training' / 'dream'))
sys.path.insert(0, str(RACINE / 'mycobot_gateway' / 'mycobot_gateway' / 'vision'))
import camera_registry as registre  # noqa: E402
from mycobot_fk import KEYPOINT_NAMES, forward_kinematics  # noqa: E402

CALIB = RACINE / 'training' / 'calibration'
JUGES = {'arducam': 'arducam_extrinsic_pick.yaml', 'svpro': 'svpro_extrinsic_labels.yaml'}
MODELE = 'vgg_tri_reel_ft_e10'
KP_MIN = 5
J5_DOMAINE_DEG = (45.0, 93.0)
ENCADREMENT_S = 0.3
RANSAC_PX = 8.0
HUBER_PX = 3.0
# La planche, la ou sont les pieces et les bacs (mm, repere base).
GRILLE_X = np.arange(150.0, 425.0, 25.0)
GRILLE_Y = np.arange(-200.0, 225.0, 25.0)


def angles_a(t, tc, qc):
    i = int(np.searchsorted(tc, t))
    if i == 0 or i == len(tc) or tc[i] - t > ENCADREMENT_S or t - tc[i - 1] > ENCADREMENT_S:
        return None
    a = (t - tc[i - 1]) / (tc[i] - tc[i - 1])
    return (1 - a) * qc[i - 1] + a * qc[i]


def correspondances(net, dossier, cam, journal, cycles, tc, qc):
    """[(points 3D (n, 3) m, points 2D (n, 2) px)] par image retenue."""
    out = []
    for e in journal:
        t = e['t']
        if not any(a <= t <= b for a, b in cycles):
            continue
        q = angles_a(t, tc, qc)
        if q is None or not J5_DOMAINE_DEG[0] <= q[4] <= J5_DOMAINE_DEG[1]:
            continue
        image = cv2.imread(str(dossier / cam / e['image']))
        h, w = image.shape[:2]
        kp = np.array(net.keypoints_from_image(
            Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB)))['detected_keypoints'], float)
        # link1 et link2 sont le meme point 3D : un seul des deux, sinon il pese double.
        vus = [k for k in range(7) if k != 2 and np.isfinite(kp[k]).all()
               and 0 <= kp[k][0] < w and 0 <= kp[k][1] < h]
        if len(vus) < KP_MIN:
            continue
        pos, _ = forward_kinematics(np.radians(q))
        out.append((np.array([pos[KEYPOINT_NAMES[k]] for k in vus], float), kp[vus]))
    return out


def pnp_empile(corr, K, dist):
    obj = np.concatenate([o for o, _ in corr])
    img = np.concatenate([i for _, i in corr])
    ok, rvec, tvec, inl = cv2.solvePnPRansac(obj, img, K, dist, reprojectionError=RANSAC_PX,
                                             iterationsCount=5000, flags=cv2.SOLVEPNP_EPNP)
    if not ok:
        raise SystemExit('PnP RANSAC sans solution')
    inl = inl.ravel()

    def residus(x):
        return (cv2.projectPoints(obj[inl], x[:3], x[3:], K, dist)[0].reshape(-1, 2) - img[inl]).ravel()

    r = least_squares(residus, np.concatenate([rvec.ravel(), tvec.ravel()]), loss='huber', f_scale=HUBER_PX)
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(r.x[:3])[0]
    T[:3, 3] = r.x[3:]
    err = np.linalg.norm(residus(r.x).reshape(-1, 2), axis=1)
    return T, len(obj), len(inl), float(np.median(err))


def sur_la_table(T, K, dist, uv):
    """Pixels -> points de la table (Z = 0), repere base, mm."""
    rayons = cv2.undistortPoints(uv.reshape(-1, 1, 2), K, dist).reshape(-1, 2)
    R, t = T[:3, :3], T[:3, 3]
    centre = -R.T @ t
    out = []
    for x, y in rayons:
        d = R.T @ np.array([x, y, 1.0])
        out.append(centre + (-centre[2] / d[2]) * d)
    return np.array(out) * 1000.0


def ecart_pose(T_ref, T):
    d = (T[:3, 3] - T_ref[:3, 3]) * 1000.0
    ang = np.degrees(np.arccos(np.clip((np.trace(T_ref[:3, :3].T @ T[:3, :3]) - 1) / 2, -1, 1)))
    return d, float(ang)


def ecart_table(T_ref, T, K, dist, pts_mm):
    """Un point de la table vu par la camera (projete par l'ArUco), ramene par DREAM."""
    P = np.column_stack([pts_mm, np.zeros(len(pts_mm))]) / 1000.0
    uv = cv2.projectPoints(P, cv2.Rodrigues(T_ref[:3, :3])[0], T_ref[:3, 3], K, dist)[0].reshape(-1, 2)
    return sur_la_table(T, K, dist, uv)[:, :2] - pts_mm


def f(x):
    return f'{x:.1f}'.replace('.', ',')


def main():
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument('session', type=Path)
    a.add_argument('--modele', default=MODELE)
    a.add_argument('--ecrire', action='store_true')
    args = a.parse_args()

    import dream
    poids = RACINE / 'training' / 'dream' / 'checkpoints_dream' / args.modele / 'best_network.pth'
    net = dream.create_network_from_config_file(str(poids.with_suffix('.yaml')), str(poids))
    s = json.loads((args.session / 'session.json').read_text())
    cycles = [(c['t0'], c['pose']) for c in s['cycles'] if c['serre'] and c['pose']]
    codeurs = sorted(s['codeurs'], key=lambda e: e['t'])
    tc = np.array([e['t'] for e in codeurs])
    qc = np.array([e['q'] for e in codeurs], float)
    objets = []
    if (args.session / 'tri_dream.csv').exists():
        objets = [(r['piece'], np.array([float(r['yolo_x_mm']), float(r['yolo_y_mm'])]))
                  for r in csv.DictReader((args.session / 'tri_dream.csv').open()) if r['verdict'] == 'OK']
    grille = np.array([(x, y) for x in GRILLE_X for y in GRILLE_Y])

    print(f'# Extrinsèque par DREAM seul — {args.session.name} — {args.modele}\n')
    print('Aucun ArUco dans le calcul ; l’extrinsèque ArUco sert uniquement de juge.\n')
    for cam, journal in s['cameras'].items():
        juge = yaml.safe_load((CALIB / JUGES[cam]).read_text())
        K, dist = registre.load_intrinsics(juge['intrinsics_stem'])
        T_ref = np.array(juge['T_cam_world'], float)
        corr = correspondances(net, args.session, cam, journal, cycles, tc, qc)
        T, n, n_in, res = pnp_empile(corr, K, dist)
        T_pair, *_ = pnp_empile(corr[:len(corr) // 2], K, dist)
        T_impair, *_ = pnp_empile(corr[len(corr) // 2:], K, dist)
        d, ang = ecart_pose(T_ref, T)
        d_st, ang_st = ecart_pose(T_pair, T_impair)
        g = np.linalg.norm(ecart_table(T_ref, T, K, dist, grille), axis=1)

        print(f'## {cam}\n')
        print(f'{len(corr)} images, {n} correspondances, {n_in} gardées par RANSAC, '
              f'résidu médian {f(res)} px.\n')
        print('| | translation (repère caméra) | dx / dy / dz | rotation |\n|---|---|---|---|')
        print(f'| DREAM contre ArUco | {f(np.linalg.norm(d))} mm | {f(d[0])} / {f(d[1])} / {f(d[2])} mm | '
              f'{ang:.2f}° |'.replace('.', ','))
        print(f'| stabilité (1re moitié du tri contre 2de) | {f(np.linalg.norm(d_st))} mm | '
              f'{f(d_st[0])} / {f(d_st[1])} / {f(d_st[2])} mm | {ang_st:.2f}° |'.replace('.', ','))
        print(f'\n**Sur la table (Z = 0)**, point vu par la caméra et ramené par DREAM au lieu de l’ArUco, '
              f'sur la planche ({len(grille)} points) : médiane {f(np.median(g))} mm, '
              f'p90 {f(np.percentile(g, 90))} mm, max {f(g.max())} mm.\n')
        if objets:
            e = ecart_table(T_ref, T, K, dist, np.array([p for _, p in objets]))
            print('| objet (position YOLO) | Δx | Δy | écart XY |\n|---|---|---|---|')
            for (nom, _), (dx, dy) in zip(objets, e):
                print(f'| {nom} | {f(dx)} | {f(dy)} | {f(np.hypot(dx, dy))} mm |')
            print()
        if args.ecrire:
            sortie = CALIB / f'{cam}_extrinsic_dream_tri.yaml'
            if sortie.exists():
                raise SystemExit(f'{sortie.name} existe deja : pas d ecrasement')
            sortie.write_text(yaml.safe_dump({
                'source': f'{datetime.now():%d/%m/%Y %H:%M} — extrinseque_dream_session.py, DREAM seul '
                          f'({args.modele}), {len(corr)} images de {args.session.name}, aucun ArUco',
                'intrinsics_stem': juge['intrinsics_stem'],
                'residu_median_px': round(res, 3),
                'T_cam_world': T.tolist()}, allow_unicode=True, sort_keys=False))
            print(f'écrit : {sortie.relative_to(RACINE)}\n')


if __name__ == '__main__':
    main()
