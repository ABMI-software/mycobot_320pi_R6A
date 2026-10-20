#!/usr/bin/env python3
"""Extrinseque SVPRO pour ETIQUETER les images reelles de DREAM, rien d'autre.

    .venv/bin/python scripts/calibre_svpro_labels.py

DREAM n'en a pas besoin en marche (il trouve la pose camera par PnP). Elle ne
sert qu'a projeter la FK des codeurs dans l'image pour fabriquer les labels du
fine-tuning, comme Gazebo le faisait avec sa camera connue.

Meme methode que l'arducam (`calibration_extrinseque_auto.ajuste` et
`leave_one_out`, reference `planche_actuelle.yaml`), avec une difference : la
SVPRO perd 23 et 26 une image sur deux, donc chaque marqueur est moyenne sur
les images ou il est vu (au moins `PRESENCE_MIN`), au lieu d'exiger toutes les
images. Les marqueurs sont decodes sur l'image agrandie x3 (23 et 26 trop petits
sinon). Ecrit `svpro_extrinsic_labels.yaml` ; ne touche ni
`svpro_extrinsic_servo.yaml` ni rien de YOLO. Aucun mouvement.
"""
import json
import sys
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import yaml

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE / 'scripts'))
import calibration_extrinseque_auto as cea  # noqa: E402
import calibrer_extrinseque_4aruco as base  # noqa: E402

SORTIE = cea.CALIB / 'svpro_extrinsic_labels.yaml'
INTRINSEQUE = 'cam_2'
IMAGES = 40
PRESENCE_MIN = 0.3
# 23 et 26 font ~25 x 11 px a l'image : a 640x480 le 26 n'est jamais decode
# (0/40, 13/10). Agrandie x3, l'image le donne 16 fois sur 20.
AGRANDISSEMENT = 3


def detecte_agrandi(image):
    grand = cv2.resize(image, None, fx=AGRANDISSEMENT, fy=AGRANDISSEMENT,
                       interpolation=cv2.INTER_CUBIC)
    return {i: (q + 0.5) / AGRANDISSEMENT - 0.5 for i, q in base.detecte(grand).items()}


def main():
    ref = yaml.safe_load(cea.REFERENCE.read_text())
    monde = {int(k): np.array(v, float) / 1000.0 for k, v in ref['markers'].items()}
    K, dist = cea.registre.load_intrinsics(INTRINSEQUE)
    index = cea.registre.detect_cameras(['svpro'], probe_capture=False)[0].v4l2_index
    trames = base.capture(index, IMAGES, 640, 480, None)
    par_trame = [detecte_agrandi(t) for t in trames]
    presence = {i: sum(i in p for p in par_trame) / len(par_trame) for i in monde}
    print('presence :', {i: f'{100 * v:.0f} %' for i, v in presence.items()})
    utiles = sorted(i for i in monde if presence[i] >= PRESENCE_MIN)
    if len(utiles) < 4:
        raise SystemExit(f'{len(utiles)}/4 marqueurs assez vus — pas de validation possible')
    moyenne = [{i: np.mean([p[i] for p in par_trame if i in p], axis=0) for i in utiles}]

    T, rms, centres, cotes, brut = cea.ajuste(moyenne, utiles, monde, K, dist)
    pires = cea.leave_one_out(*brut, utiles, centres, monde, K, dist)
    print(f'RMS {rms:.3f} px')
    for i in utiles:
        print(f'  id {i} exclu : {pires[i]:.2f} mm')
    cam = cea.position_camera(T)
    print('camera (mm, repere base) :', np.round(cam, 1).tolist())

    cv2.imwrite(str(RACINE / 'results' / 'svpro_extrinsic_labels.png'), trames[-1])
    SORTIE.write_text(yaml.safe_dump({
        'source': f'{datetime.now():%d/%m/%Y %H:%M} — calibre_svpro_labels.py, 4 ArUco contre '
                  f'planche_actuelle.yaml, {IMAGES} images, marqueurs moyennes la ou vus, detection sur image agrandie x3',
        'usage': 'UNIQUEMENT les labels du fine-tuning DREAM reel. Ni DREAM en marche, ni YOLO.',
        'camera': 'svpro',
        'intrinsics_stem': INTRINSEQUE,
        'resolution': [640, 480],
        'frame_id': 'base_link',
        'markers_used': utiles,
        'presence': {int(i): round(float(v), 3) for i, v in presence.items()},
        'rms_reproj_px': round(rms, 4),
        'leave_one_out_mm': {int(i): round(v, 2) for i, v in pires.items()},
        'camera_position_mm': [round(float(v), 1) for v in cam],
        'T_cam_world': T.tolist(),
    }, sort_keys=False, allow_unicode=True))
    print('ecrit', SORTIE.name)
    print('RESULTAT', json.dumps({'rms': rms, 'loo': pires}))


if __name__ == '__main__':
    main()
