#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Conversion NDDS d'une collecte a POSE DE CAMERA RANDOMISEE.

`convert_to_ndds.convert_synthetic` appelle `get_camera_transform(nom_vue)`, qui
rend une CONSTANTE tiree de `GAZEBO_CAMERAS` — les 4 points de vue figes de
l'URDF. Une collecte ou la camera bouge a chaque trame n'est donc pas
convertible par ce chemin : les etiquettes 2D seraient toutes calculees avec la
pose d'une camera qui n'a pas pris l'image.

Ce script lit la pose reelle de la trame dans `camera_poses.csv`, ecrit par
`synthetic_data_collector_v3_camera_libre`. Tout le reste (intrinseques, format
de sortie, ecriture des JSON) vient de `convert_to_ndds`, importe et NON
modifie.

Une pose rejetee en cours de collecte (bras non stabilise, rendu gele) laisse
dans `camera_poses.csv` une ligne perimee portant le meme index : on garde donc
la DERNIERE de chaque index, qui est celle de la trame reellement enregistree.

Plusieurs collectes se fusionnent en passant `--input` separes par des
virgules : les index sont renumerotes a la volee, donc deux dossiers portant
tous deux un `000000` ne se marchent pas dessus.

SECTEUR TENU A L'ECART (`--azimut-tenu`). L'affaire qu'on corrige est que le
reseau memorise le cadrage au lieu de localiser. Une validation qui tire ses
trames au hasard ne peut pas le detecter : elles partagent les points de vue de
l'entrainement. On reserve donc un SECTEUR D'AZIMUT entier, jamais vu a
l'entrainement — c'est le seul decoupage qui teste la generalisation a une
camera deplacee. L'azimut est celui de la camera autour du point vise, le meme
angle que `_tire_pose` tire dans le collecteur.

Usage (venv_dream) :
    python training/dream/convert_to_ndds_camera_libre.py \
        --input dream_data/synth_cam_libre,dream_data/synth_cam_libre_b \
        --output dream_data/synth_cam_libre_ndds \
        --azimut-tenu 60,100 --output-tenu dream_data/synth_cam_libre_vue_tenue
"""
import argparse
import csv
import math
import os
import sys
from pathlib import Path

RACINE = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(RACINE / 'training' / 'dream'))

from convert_to_ndds import (                                    # noqa: E402
    GAZEBO_INTRINSICS, copy_image_as_ndds, load_labels,
    write_camera_settings, write_frame_json,
)
from mycobot_fk import (                                         # noqa: E402
    _camera_transform_from_urdf, keypoints_in_camera_frame, project_keypoints,
)


def charge_poses(chemin):
    """index -> (xyz, rpy), la DERNIERE ligne de chaque index faisant foi."""
    poses = {}
    with open(chemin) as f:
        for r in csv.DictReader(f):
            poses[int(r['index'])] = (
                (float(r['x']), float(r['y']), float(r['z'])),
                (float(r['roll']), float(r['pitch']), float(r['yaw'])),
            )
    return poses


# Point vise par le collecteur (`CollecteurCameraLibre.VISEE`) : l'azimut d'une
# trame est l'angle de la camera AUTOUR de ce point, pas autour de l'origine.
VISEE_XY = (0.10, 0.0)


def azimut_deg(xyz):
    return math.degrees(math.atan2(xyz[1] - VISEE_XY[1], xyz[0] - VISEE_XY[0]))


def dans_secteur(az, borne):
    """Secteur circulaire, borne haute possiblement inferieure a la basse."""
    a, b = borne
    az = (az - a) % 360.0
    return az <= (b - a) % 360.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--input', required=True,
                    help='un ou plusieurs dossiers de collecte, separes par des virgules')
    ap.add_argument('--output', required=True)
    ap.add_argument('--camera', default='libre')
    ap.add_argument('--azimut-tenu', default=None,
                    help='secteur reserve, en degres, ex. "60,100"')
    ap.add_argument('--output-tenu', default=None,
                    help='dossier du secteur reserve (obligatoire avec --azimut-tenu)')
    args = ap.parse_args()

    secteur = None
    if args.azimut_tenu:
        if not args.output_tenu:
            ap.error('--azimut-tenu exige --output-tenu')
        secteur = tuple(float(v) for v in args.azimut_tenu.split(','))
        os.makedirs(args.output_tenu, exist_ok=True)
        write_camera_settings(args.output_tenu, GAZEBO_INTRINSICS)

    os.makedirs(args.output, exist_ok=True)
    write_camera_settings(args.output, GAZEBO_INTRINSICS)

    sources = []
    for chemin in args.input.split(','):
        e = Path(chemin)
        sources.append((e, charge_poses(e / 'camera_poses.csv'),
                        load_labels(str(e / 'labels.csv'))))

    idx, idx_tenu = 0, 0
    sans_pose, hors_champ, sans_image = 0, 0, 0
    for entree, poses, lignes in sources:
      for r in lignes:
        if r['camera'] != args.camera:
            continue
        n = int(r['index'])
        if n not in poses:
            sans_pose += 1
            continue
        xyz, rpy = poses[n]
        T_cam = _camera_transform_from_urdf(xyz, rpy)
        joints = [float(r[f'j{i}_rad']) for i in range(1, 7)]
        kp_cam = keypoints_in_camera_frame(joints, T_cam)
        projs = project_keypoints(kp_cam, GAZEBO_INTRINSICS)

        if any(math.isnan(p[0]) or math.isnan(p[1])
               or p[0] < -50 or p[0] > 690 or p[1] < -50 or p[1] > 530
               for p in projs):
            hors_champ += 1
            continue

        img = entree / r['image_path']
        if not img.exists():
            sans_image += 1
            continue

        tenue = secteur is not None and dans_secteur(azimut_deg(xyz), secteur)
        dossier = args.output_tenu if tenue else args.output
        rang = idx_tenu if tenue else idx
        copy_image_as_ndds(str(img), dossier, rang)
        write_frame_json(dossier, rang, kp_cam, projs)
        if tenue:
            idx_tenu += 1
        else:
            idx += 1
        if (idx + idx_tenu) % 1000 == 0:
            print(f'  {idx + idx_tenu} trames converties…')

    print(f'  entrainement : {idx} trames  |  secteur tenu : {idx_tenu} trames')
    print(f'  ecartees : {hors_champ} hors champ, {sans_pose} sans pose, '
          f'{sans_image} sans image')


if __name__ == '__main__':
    main()
