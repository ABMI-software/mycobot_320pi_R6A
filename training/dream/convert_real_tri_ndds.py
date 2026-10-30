#!/usr/bin/env python3
"""real_tri (capture_real_tri.py) -> NDDS, le format que lit l'entrainement DREAM.

    /usr/bin/python3 training/dream/convert_real_tri_ndds.py \
        training/dream/dream_data/real_tri/train training/dream/dream_data/real_tri_train_ndds

Une image par (pose, camera), arducam et SVPRO dans le meme dossier, comme
`real_3cam_train_x5_ndds`. `location` = FK des codeurs dans le repere de la
camera (extrinseque de la camera), `projected_location` = la meme projetee par
son intrinseque. Les images sont des liens symboliques. Refuse un dossier de
sortie non vide.
"""
import argparse
import csv
import json
import os
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

RACINE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(RACINE / 'training' / 'dream'))
sys.path.insert(0, str(RACINE / 'mycobot_gateway' / 'mycobot_gateway' / 'vision'))
import camera_registry as registre  # noqa: E402
from mycobot_fk import KEYPOINT_NAMES, forward_kinematics  # noqa: E402

CALIB = RACINE / 'training' / 'calibration'
EXTRINSEQUES = {'arducam': CALIB / 'arducam_extrinsic_pick.yaml',
                'svpro': CALIB / 'svpro_extrinsic_labels.yaml'}


def main():
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument('source', type=Path)
    a.add_argument('sortie', type=Path)
    a.add_argument('--cameras', nargs='+', default=['arducam', 'svpro'])
    args = a.parse_args()

    if args.sortie.exists() and any(args.sortie.iterdir()):
        raise SystemExit(f'{args.sortie} n est pas vide')
    args.sortie.mkdir(parents=True, exist_ok=True)

    cams = {}
    for nom in args.cameras:
        d = yaml.safe_load(EXTRINSEQUES[nom].read_text())
        K, dist = registre.load_intrinsics(d['intrinsics_stem'])
        cams[nom] = (np.array(d['T_cam_world'], float), K, dist)

    n = 0
    for ligne in csv.DictReader((args.source / 'labels.csv').open()):
        nom = ligne['camera']
        if nom not in cams:
            continue
        image = (args.source / ligne['image_path']).resolve()
        if not image.exists():
            continue
        T, K, dist = cams[nom]
        q = np.array([float(ligne[f'j{j}_deg']) for j in range(1, 7)])
        pos, _ = forward_kinematics(np.radians(q))
        monde = np.array([pos[k] for k in KEYPOINT_NAMES], float)
        cam = monde @ T[:3, :3].T + T[:3, 3]
        uv = cv2.projectPoints(monde, cv2.Rodrigues(T[:3, :3])[0], T[:3, 3], K, dist)[0].reshape(-1, 2)
        (args.sortie / f'{n:06d}.json').write_text(json.dumps({'objects': [{
            'class': 'mycobot320',
            'keypoints': [{'name': k, 'location': cam[i].tolist(), 'projected_location': uv[i].tolist()}
                          for i, k in enumerate(KEYPOINT_NAMES)],
        }], 'source': {'camera': nom, 'index': int(ligne['index']), 'angles_deg': q.tolist()}}))
        os.symlink(image, args.sortie / f'{n:06d}.rgb.png')
        n += 1

    T, K, _ = cams[args.cameras[0]]
    (args.sortie / '_camera_settings.json').write_text(json.dumps({'camera_settings': [{
        'name': 'mycobot_camera',
        'intrinsic_settings': {'fx': K[0, 0], 'fy': K[1, 1], 'cx': K[0, 2], 'cy': K[1, 2], 's': 0.0},
        'captured_image_size': {'width': 640, 'height': 480}}]}, indent=2))
    print(f'{n} images -> {args.sortie}')


if __name__ == '__main__':
    main()
