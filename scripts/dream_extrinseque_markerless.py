#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Pose des cameras retrouvee par DREAM SEUL — aucun marqueur en entree.

Le principe de DREAM : le robot est sa propre mire. Pour chaque pose, la FK des
encodeurs donne les 7 keypoints en 3D dans le repere base, le reseau les detecte
en 2D dans l'image. Empiler ces correspondances sur N poses et resoudre un PnP
donne la pose de la camera — sans jamais regarder un ArUco.

Les 4 marqueurs ne servent QUE de juge : on compare a la fin, on ne s'en sert
jamais pour amorcer ni pour contraindre. L'amorcage vient d'un `solvePnPRansac`
sur les correspondances elles-memes, pas d'un extrinseque existant : un
extrinseque d'amorcage rendrait la comparaison circulaire.

Pourquoi plusieurs poses. Sur UNE pose les 7 keypoints sont quasi coplanaires et
la rotation admet plusieurs branches qui reprojettent aussi bien — mesure de
juillet, 27-30 deg d'erreur. Le volume balaye par une vingtaine de
configurations leve l'ambiguite.

Usage (venv_dream) :
    python scripts/dream_extrinseque_markerless.py \
        --capture training/dream/captures/markerless_0902
"""
import argparse
import csv
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml
from PIL import Image
from scipy.optimize import least_squares

RACINE = Path(__file__).resolve().parents[1]
CALIB = RACINE / 'training' / 'calibration'
sys.path.insert(0, '/tmp/DREAM')
sys.path.insert(0, str(RACINE / 'training' / 'dream'))
sys.path.insert(0, str(RACINE / 'mycobot_gateway' / 'mycobot_gateway' / 'vision'))

import dream                                                  # noqa: E402
import camera_registry as registre                            # noqa: E402
from mycobot_fk import forward_kinematics, KEYPOINT_NAMES     # noqa: E402

KP = ['base', 'link1', 'link2', 'link3', 'link4', 'link5', 'link6']
CAMERAS = {'arducam': {'calib': 'cam_3', 'juge': 'arducam_extrinsic_pick'},
           'svpro': {'calib': 'cam_2', 'juge': 'svpro_extrinsic_4marqueurs'}}


def fk_points(q_deg):
    pos, _ = forward_kinematics(np.radians(q_deg))
    return np.array([pos[n] for n in KEYPOINT_NAMES], float)


def ecart_pose(Ta, Tb):
    """(distance du centre optique en mm, angle entre orientations en deg)."""
    ca = -Ta[:3, :3].T @ Ta[:3, 3]
    cb = -Tb[:3, :3].T @ Tb[:3, 3]
    dR = Ta[:3, :3] @ Tb[:3, :3].T
    ang = np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1)))
    return float(np.linalg.norm(ca - cb) * 1000), float(ang)


def resout(objs, dets, K, dist):
    """PnP robuste, amorce par RANSAC sur les correspondances elles-memes."""
    ok, rvec, tvec, _ = cv2.solvePnPRansac(
        objs, dets, K, dist, flags=cv2.SOLVEPNP_EPNP,
        reprojectionError=12.0, iterationsCount=3000, confidence=0.999)
    if not ok:
        raise SystemExit('RANSAC a echoue — detections trop bruitees')
    x = np.concatenate([rvec.ravel(), tvec.ravel()])

    def erreurs(x, masque):
        p, _ = cv2.projectPoints(objs[masque], x[:3].reshape(3, 1),
                                 x[3:].reshape(3, 1), K, dist)
        return (p.reshape(-1, 2) - dets[masque]).ravel()

    garde = np.ones(len(objs), bool)
    for _ in range(5):
        x = least_squares(erreurs, x, args=(garde,),
                          loss='soft_l1', f_scale=5.0, max_nfev=3000).x
        err = np.linalg.norm(erreurs(x, np.ones(len(objs), bool)).reshape(-1, 2), axis=1)
        neuf = err <= max(8.0, 3.0 * np.median(err[garde]))
        if neuf.sum() == garde.sum() or neuf.sum() < 20:
            break
        garde = neuf
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(x[:3])[0]
    T[:3, 3] = x[3:]
    return T, err, garde


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--capture', required=True)
    ap.add_argument('--modele', default='vgg_montage0901_ft_e30')
    ap.add_argument('--cameras', default='arducam,svpro',
                    help='cameras a traiter, separees par une virgule')
    ap.add_argument('--ecrire', action='store_true',
                    help='ecrire les extrinseques markerless en YAML')
    args = ap.parse_args()

    poids = RACINE / 'training' / 'dream' / 'checkpoints_dream' / args.modele / 'best_network.pth'
    reseau = dream.create_network_from_config_file(str(poids.with_suffix('.yaml')), str(poids))
    reseau.enable_evaluation()
    print(f'modele : {args.modele}\n')

    capture = Path(args.capture).resolve()
    lignes = list(csv.DictReader((capture / 'labels.csv').open()))

    demandees = [c.strip() for c in args.cameras.split(',') if c.strip()]
    for nom in demandees:
        cfg = CAMERAS[nom]
        rangs = [r for r in lignes if r['camera'] == nom]
        K, dist = registre.load_intrinsics(cfg['calib'], 640, 480)
        objs, dets, kidx, vus = [], [], [], []
        for r in rangs:
            q = [float(r[f'j{j}_deg']) for j in range(1, 7)]
            pts = fk_points(q)
            img = Image.open(capture / r['image_path']).convert('RGB')
            kps = np.array(reseau.keypoints_from_image(img)['detected_keypoints'], float)
            n = 0
            for k in range(7):
                if kps[k, 0] > -900 and kps[k, 1] > -900:
                    objs.append(pts[k]); dets.append(kps[k]); kidx.append(k); n += 1
            vus.append(n)
        objs = np.array(objs); dets = np.array(dets); kidx = np.array(kidx)

        print(f'=== {nom} ===')
        print(f'{len(rangs)} poses, detection moyenne {np.mean(vus):.1f}/7 '
              f'({sum(v == 7 for v in vus)} poses a 7/7), {len(objs)} correspondances')

        T, err, garde = resout(objs, dets, K, dist)
        print(f'residu markerless : median {np.median(err[garde]):.2f} px '
              f'sur {garde.sum()} inliers')
        for k in range(7):
            m = (kidx == k) & garde
            if m.sum():
                print(f'    {KP[k]:6s} n={m.sum():3d}  {np.median(err[m]):5.2f} px')

        juge = yaml.safe_load((CALIB / f"{cfg['juge']}.yaml").read_text())
        Tj = np.array(juge['T_cam_world'], float)
        pj, _ = cv2.projectPoints(objs, cv2.Rodrigues(Tj[:3, :3])[0],
                                  Tj[:3, 3].reshape(3, 1), K, dist)
        ej = np.linalg.norm(pj.reshape(-1, 2) - dets, axis=1)
        d_mm, d_deg = ecart_pose(T, Tj)
        c = -T[:3, :3].T @ T[:3, 3]
        cj = -Tj[:3, :3].T @ Tj[:3, 3]
        print(f'  juge marqueurs ({cfg["juge"]}) : residu median {np.median(ej):.2f} px')
        print(f'  position markerless : ({c[0]:.3f}, {c[1]:.3f}, {c[2]:.3f}) m')
        print(f'  position marqueurs  : ({cj[0]:.3f}, {cj[1]:.3f}, {cj[2]:.3f}) m')
        print(f'  ECART : {d_mm:.1f} mm   {d_deg:.2f} deg\n')

        if args.ecrire:
            sortie = CALIB / f'{nom}_extrinsic_markerless.yaml'
            sortie.write_text(yaml.safe_dump({
                'source': 'DREAM markerless (robot comme mire, aucun ArUco en entree)',
                'camera': nom,
                'intrinsics_stem': cfg['calib'],
                'resolution': [640, 480],
                'frame_id': 'base_link',
                'modele': args.modele,
                'capture': str(capture.relative_to(RACINE)),
                'n_poses': len(rangs),
                'n_inliers': int(garde.sum()),
                'residu_median_px': float(np.median(err[garde])),
                'juge_marqueurs': cfg['juge'],
                'ecart_au_juge_mm': d_mm,
                'ecart_au_juge_deg': d_deg,
                'T_cam_world': T.tolist(),
                'T_world_cam': np.linalg.inv(T).tolist(),
                'camera_position_base_m': c.tolist(),
            }, sort_keys=False))
            print(f'  ecrit : {sortie.relative_to(RACINE)}\n')


if __name__ == '__main__':
    main()
