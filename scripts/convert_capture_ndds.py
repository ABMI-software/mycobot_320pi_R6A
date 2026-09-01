#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Conversion NDDS d'une capture de `capture_trajectoires.py`.

Script autonome : `convert_to_ndds.py` n'est ni modifie ni importe. Il n'est pas
reutilise ici pour deux raisons de fond.

1. Il mappe `arducam -> cam_0` (ligne 102) alors que l'arducam de ce banc est
   `cam_3` — 6,4 % d'ecart de focale (527 contre 495) et les mauvais
   coefficients de distorsion. C'est ce qui a rendu
   `arducam_extrinsic_dream_v4.yaml` incomparable a tout. Ici les intrinseques
   viennent du `_camera_settings.json` ecrit PAR la capture, donc de la camera
   qui a reellement pris les images.

2. Il utilise `_real_camera_transform`, des poses camera approximatives codees en
   dur et datees. On prend a la place les extrinseques MARQUEURS validees :
   `arducam_extrinsic_pick` (20/08) et `svpro_extrinsic_servo` (24/08). Leur
   justesse a ete verifiee le 31/08 en projetant le squelette FK — il epouse le
   bras dans les deux vues.

La projection applique la DISTORSION (`cv2.projectPoints`). Sans elle les
`projected_location` ne tombent pas ou le keypoint apparait vraiment, et DREAM
apprendrait des cartes de croyance decalees.

Usage (venv_dream) :
    python scripts/convert_capture_ndds.py --capture real_montage_0901
    python scripts/convert_capture_ndds.py --capture real_montage_0901 --verifier 6
"""
import argparse
import csv
import json
import shutil
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

RACINE = Path(__file__).resolve().parents[1]
CALIB = RACINE / 'training' / 'calibration'
CAPTURES = RACINE / 'training' / 'dream' / 'captures'

sys.path.insert(0, str(RACINE / 'training' / 'dream'))
from mycobot_fk import forward_kinematics, KEYPOINT_NAMES     # noqa: E402

EXTRINSEQUES = {'arducam': 'arducam_extrinsic_pick',
                'svpro': 'svpro_extrinsic_servo'}
# Fenetre reellement vue par le reseau apres `shrink-and-crop` 640x480 -> 400x400.
FENETRE_X = (80, 560)


def modele(cam, dossier):
    """(rvec, tvec, K, dist) — intrinseques de la CAPTURE, extrinseque marqueurs."""
    reglages = json.loads(
        (dossier / 'images' / cam / '_camera_settings.json').read_text())
    s = reglages['camera_settings'][0]
    i = s['intrinsic_settings']
    K = np.array([[i['fx'], i.get('s', 0.0), i['cx']],
                  [0.0, i['fy'], i['cy']], [0.0, 0.0, 1.0]], float)
    dist = np.array(s.get('dist_coeffs', []), float)
    d = yaml.safe_load((CALIB / f'{EXTRINSEQUES[cam]}.yaml').read_text())
    T = np.array(d['T_cam_world'], float)
    taille = (s['captured_image_size']['width'], s['captured_image_size']['height'])
    return cv2.Rodrigues(T[:3, :3])[0], T[:3, 3], K, dist, T, taille


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--capture', required=True)
    ap.add_argument('--cameras', nargs='+', default=['arducam', 'svpro'])
    ap.add_argument('--verifier', type=int, default=0,
                    help='ecrire N images de controle avec le squelette projete')
    args = ap.parse_args()

    dossier = CAPTURES / args.capture
    lignes = list(csv.DictReader((dossier / 'labels.csv').open()))

    for cam in args.cameras:
        rvec, tvec, K, dist, T, (w, h) = modele(cam, dossier)
        sortie = CAPTURES / f'{args.capture}_{cam}_ndds'
        sortie.mkdir(parents=True, exist_ok=True)
        json.dump({'camera_settings': [{
            'name': 'mycobot_camera',
            'intrinsic_settings': {'fx': float(K[0, 0]), 'fy': float(K[1, 1]),
                                   'cx': float(K[0, 2]), 'cy': float(K[1, 2]),
                                   's': 0.0},
            'captured_image_size': {'width': w, 'height': h},
        }]}, (sortie / '_camera_settings.json').open('w'), indent=2)

        n, hors_cadre, hors_fenetre = 0, 0, 0
        controles = []
        for l in (x for x in lignes if x['camera'] == cam):
            q = np.array([float(l[f'j{k}_deg']) for k in range(1, 7)], float)
            pos, _ = forward_kinematics(np.radians(q))
            monde = np.array([pos[nom] for nom in KEYPOINT_NAMES], float)
            uv = cv2.projectPoints(monde, rvec, tvec, K, dist)[0].reshape(-1, 2)
            # 3D dans le repere CAMERA : c'est ce que NDDS appelle `location`.
            cam3d = (T[:3, :3] @ monde.T).T + T[:3, 3]

            if not np.all((0 <= uv[:, 0]) & (uv[:, 0] < w)
                          & (0 <= uv[:, 1]) & (uv[:, 1] < h)):
                hors_cadre += 1
                continue
            if not np.all((FENETRE_X[0] <= uv[:, 0]) & (uv[:, 0] < FENETRE_X[1])):
                hors_fenetre += 1

            json.dump({'objects': [{'class': 'mycobot320', 'keypoints': [
                {'name': nom,
                 'location': [float(v) for v in cam3d[i]],
                 'projected_location': [float(v) for v in uv[i]]}
                for i, nom in enumerate(KEYPOINT_NAMES)]}]},
                (sortie / f'{n:06d}.json').open('w'), indent=2)
            shutil.copy2(dossier / l['image_path'], sortie / f'{n:06d}.rgb.png')
            if args.verifier and len(controles) < args.verifier and n % 97 == 0:
                controles.append((n, uv))
            n += 1

        print(f'{cam:8s} {n} trames ecrites  '
              f'({hors_cadre} hors image, {hors_fenetre} debordant la fenetre reseau)')
        print(f'         -> {sortie}')

        for idx, uv in controles:
            img = cv2.imread(str(sortie / f'{idx:06d}.rgb.png'))
            for a, b in zip(range(6), range(1, 7)):
                cv2.line(img, tuple(uv[a].astype(int)), tuple(uv[b].astype(int)),
                         (60, 220, 60), 2, cv2.LINE_AA)
            for p in uv:
                cv2.circle(img, tuple(p.astype(int)), 6, (60, 220, 60), 2, cv2.LINE_AA)
            cv2.imwrite(str(sortie / f'_controle_{idx:06d}.png'), img)
        if controles:
            print(f'         {len(controles)} images de controle _controle_*.png '
                  '— le vert doit epouser le bras')


if __name__ == '__main__':
    main()
