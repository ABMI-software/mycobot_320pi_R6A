#!/usr/bin/env python3
"""Controle de sante de l'extrinseque arducam sur une image donnee.

À lancer avec `.venv/bin/python` : `cv2.aruco` fait planter l'OpenCV 4.6 du
systeme. Le tableau de bord, lui, tourne en Python systeme et lui passe une
image FICHIER — les deux ne peuvent pas ouvrir la camera en meme temps.

    .venv/bin/python scripts/aruco_check.py <image.png> [--recalibre]

`--recalibre` reecrit `arducam_extrinsic_pick.yaml` par solvePnP sur les quatre
centres. Une derive de la camera est une ROTATION autant qu'une translation :
un rattrapage rigide serait juste aux marqueurs et faux entre eux, la ou se
trouve l'objet. On recalcule, on ne rustine pas.
"""
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

CALIB = Path(__file__).resolve().parents[1] / 'training' / 'calibration'


def charge():
    fichier = CALIB / 'arducam_extrinsic_pick.yaml'
    if not fichier.exists():
        fichier = CALIB / 'arducam_extrinsic_servo.yaml'
    d = yaml.safe_load(fichier.read_text())
    intr = np.load(CALIB / f"{d['intrinsics_stem']}.npz")
    return (np.array(d['T_cam_world'], float), np.array(intr['mtx'], float),
            np.array(intr['dist'], float), fichier.name)


def centres(image):
    detecteur = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
        cv2.aruco.DetectorParameters())
    coins, ids, _ = detecteur.detectMarkers(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
    if ids is None:
        return {}
    return {int(i): c.reshape(4, 2).mean(axis=0) for c, i in zip(coins, ids.flatten())}


def vers_plan(uv, T, K, dist, z_mm=0.0):
    p = cv2.undistortPoints(np.array([[uv]], float), K, dist).reshape(2)
    R, t = T[:3, :3], T[:3, 3]
    centre = -R.T @ t
    direction = R.T @ np.array([p[0], p[1], 1.0])
    return (centre + (z_mm / 1000.0 - centre[2]) / direction[2] * direction) * 1000.0


def main():
    image = cv2.imread(sys.argv[1])
    if image is None:
        print('image illisible')
        return
    T, K, dist, source = charge()
    reference = yaml.safe_load((CALIB / 'workspace_markers.yaml').read_text())['markers']
    vus = {i: uv for i, uv in centres(image).items() if i in reference}
    if not vus:
        print('aucun marqueur vu')
        return
    ecarts = {i: float(np.linalg.norm(vers_plan(uv, T, K, dist)[:2]
                                      - np.array(reference[i][:2], float)))
              for i, uv in vus.items()}
    moyen = float(np.mean(list(ecarts.values())))
    print(f'{source} — {len(vus)}/4 marqueurs, ecart moyen {moyen:.2f} mm '
          + ' '.join(f'{i}:{e:.1f}' for i, e in sorted(ecarts.items())))
    if moyen > 5.0:
        print('DERIVE — recalculer avant de commander (relancer avec --recalibre)')

    if '--recalibre' in sys.argv:
        if len(vus) < 4:
            print('recalibration impossible : il faut les 4 marqueurs (dégager le bras)')
            return
        obj = np.array([reference[i] for i in sorted(vus)], float) / 1000.0
        img = np.array([vus[i] for i in sorted(vus)], float)
        ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
        if not ok:
            print('solvePnP a echoue')
            return
        R, _ = cv2.Rodrigues(rvec)
        neuf = np.eye(4)
        neuf[:3, :3], neuf[:3, 3] = R, tvec.ravel()
        controle = {i: float(np.linalg.norm(vers_plan(uv, neuf, K, dist)[:2]
                                            - np.array(reference[i][:2], float)))
                    for i, uv in vus.items()}
        cible = CALIB / 'arducam_extrinsic_pick.yaml'
        yaml.safe_dump({'source': 'solvePnP 4 centres de marqueurs, recalibration en séance',
                        'camera': 'arducam', 'intrinsics_stem': 'cam_3',
                        'resolution': [640, 480], 'frame_id': 'base_link',
                        'markers_used': sorted(vus),
                        'ecart_marqueurs_mm': {str(i): round(e, 3) for i, e in controle.items()},
                        'T_cam_world': [[float(v) for v in r] for r in neuf]},
                       cible.open('w'), sort_keys=False)
        print(f'recalibre -> {cible.name}, ecart moyen {np.mean(list(controle.values())):.2f} mm')


if __name__ == '__main__':
    main()
