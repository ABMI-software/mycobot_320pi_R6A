#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Biais DREAM mesure sur des poses FAVORABLES — le bras BOUGE.

`fk_vs_dream_diagnostic.py` a valide la FK et les extrinseques marqueurs sur la
pose courante : le squelette projete epouse le bras dans les deux vues. Mais il
l'a fait sur des poses ou DREAM decroche (bras hors planche, fond encombre), donc
l'ecart mesure la-bas melange le biais du reseau et son echec de detection.

Ici on rejoue la meme comparaison sur les poses de travail, ou DREAM detecte :
bras au-dessus de la planche, pince pointee vers le bas a 7-12 deg de la
verticale. La FK etant desormais validee, l'ecart restant est du VRAI biais.

/!\\ LE BRAS BOUGE. Degager la zone avant de lancer.

Usage (venv_dream) :
    python scripts/fk_vs_dream_series.py            # les 8 poses
    python scripts/fk_vs_dream_series.py --n 3      # les 3 premieres
"""
import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE / 'scripts'))

from fk_vs_dream_diagnostic import (                            # noqa: E402
    CALIB, SORTIE, VUES, COURT, CHECKPOINT, VERT, ORANGE, BLANC, NOIR, FOND,
    FONT, capture, texte)
from mycobot_fk import forward_kinematics, KEYPOINT_NAMES       # noqa: E402
import camera_registry as registre                              # noqa: E402
import dream                                                    # noqa: E402

# Derivees des points de travail enregistres (pick_approach, pick,
# observation_clear) en ne faisant tourner que J1 : cette rotation conserve
# EXACTEMENT l'inclinaison de l'outil, donc la pince reste pointee vers le bas.
# Source : POSES_CALIB de pick_and_place_live_dashboard.py.
POSES = [
    [28.53, -118.74, 82.79, -102.56, -17.84, 49.83],
    [58.53, -118.74, 82.79, -102.56, -17.84, 49.83],
    [88.53, -118.74, 82.79, -102.56, -17.84, 49.83],
    [11.92, -128.67, 76.55, -53.43, -3.69, 21.79],
    [41.92, -128.67, 76.55, -53.43, -3.69, 21.79],
    [71.92, -128.67, 76.55, -53.43, -3.69, 21.79],
    [53.05, -120.05, 90.00, -60.55, 10.28, 6.24],
    [83.05, -120.05, 90.00, -60.55, 10.28, 6.24],
]
PAS_MAX_DEG = 30.0     # un ordre unique depuis une pose eloignee fait partir
                       # toutes les articulations a fond en meme temps


def rejoint(pont, cible, vitesse=25, tol=0.6, attente=10.0):
    """Rejoint `cible` par paliers, puis attend l'immobilisation reelle."""
    depart = np.array(pont.get_angles(), float)
    cible = np.array(cible, float)
    n = max(1, int(np.ceil(np.max(np.abs(cible - depart)) / PAS_MAX_DEG)))
    for i in range(1, n + 1):
        etape = depart + (cible - depart) * (i / n)
        pont.send({'action': 'send_angles',
                   'angles': [round(float(v), 2) for v in etape],
                   'speed': vitesse})
        time.sleep(1.6)
    t0, precedent = time.time(), None
    while time.time() - t0 < attente:
        q = np.array(pont.get_angles(), float)
        if precedent is not None and np.max(np.abs(q - precedent)) < tol:
            return q
        precedent = q
        time.sleep(0.5)
    return np.array(pont.get_angles(), float)


def modele_camera(cfg):
    d = yaml.safe_load((CALIB / f"{cfg['extr']}.yaml").read_text())
    T = np.array(d['T_cam_world'], float)
    K, dist = registre.load_intrinsics(d['intrinsics_stem'])
    return cv2.Rodrigues(T[:3, :3])[0], T[:3, 3], K, dist


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=len(POSES))
    ap.add_argument('--host', default='10.10.0.224')
    args = ap.parse_args()
    poses = POSES[:args.n]

    from pick_and_place_real import Bridge
    pont = Bridge(args.host)
    print(f'Depart : {np.round(pont.get_angles(), 1).tolist()}')
    print(f'{len(poses)} poses — LE BRAS VA BOUGER\n')

    net = dream.create_network_from_config_file(
        str(CHECKPOINT.with_suffix('.yaml')), str(CHECKPOINT))
    cameras = {nom: modele_camera(cfg) for nom, cfg in VUES}
    from PIL import Image

    # {camera: {keypoint: [ecarts px]}} + compte de detections
    ecarts = {nom: {k: [] for k in COURT} for nom, _ in VUES}
    detectes = {nom: [] for nom, _ in VUES}
    vignettes = []

    for i, pose in enumerate(poses, 1):
        q = rejoint(pont, pose)
        pos, _ = forward_kinematics(np.radians(q))
        obj = np.array([pos[n] for n in KEYPOINT_NAMES], float)
        ligne = f'pose {i}/{len(poses)}  q={np.round(q, 1).tolist()}'

        for nom, cfg in VUES:
            rvec, tvec, K, dist = cameras[nom]
            proj = cv2.projectPoints(obj, rvec, tvec, K, dist)[0].reshape(-1, 2)
            image = capture(cfg)
            h, w = image.shape[:2]
            kps = np.array(net.keypoints_from_image(
                Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
            )['detected_keypoints'], float)

            vus = 0
            for k, court in enumerate(COURT):
                fu, fv = proj[k]
                du, dv = kps[k]
                if np.all(np.isfinite(kps[k])) and 0 <= du < w and 0 <= dv < h:
                    ecarts[nom][court].append(float(np.hypot(fu - du, fv - dv)))
                    vus += 1
                    cv2.line(image, (int(fu), int(fv)), (int(du), int(dv)),
                             BLANC, 1, cv2.LINE_AA)
                    cv2.circle(image, (int(du), int(dv)), 5, ORANGE, -1, cv2.LINE_AA)
                cv2.circle(image, (int(fu), int(fv)), 7, VERT, 2, cv2.LINE_AA)
            for a, b in zip(range(6), range(1, 7)):
                cv2.line(image, tuple(proj[a].astype(int)),
                         tuple(proj[b].astype(int)), VERT, 2, cv2.LINE_AA)
            detectes[nom].append(vus)
            ligne += f'   {nom} {vus}/7'
            if nom == 'arducam':
                v = cv2.resize(image, (320, 240))
                texte(v, f'{i}: {vus}/7', (8, 22), BLANC, 0.5)
                vignettes.append(v)
        print(ligne)

    print('\n=== biais FK<->DREAM sur poses favorables ===')
    print(f'{"":8s} ' + ' '.join(f'{c:>9s}' for c in COURT))
    for nom, _ in VUES:
        med = []
        for c in COURT:
            e = ecarts[nom][c]
            med.append(f'{np.median(e):7.0f}px' if e else f'{"-":>9s}')
        print(f'{nom:8s} ' + ' '.join(f'{m:>9s}' for m in med))
        n = detectes[nom]
        print(f'{"":8s} detections {np.mean(n):.1f}/7 en moyenne '
              f'(min {min(n)}, max {max(n)})')

    if vignettes:
        cols = 4
        lignes = [np.hstack(vignettes[i:i + cols] + [np.zeros(
            (240, 320 * (cols - len(vignettes[i:i + cols])), 3), np.uint8)]
            if len(vignettes[i:i + cols]) < cols else vignettes[i:i + cols])
            for i in range(0, len(vignettes), cols)]
        grille = np.vstack(lignes)
        cv2.imwrite(str(SORTIE / 'fk_vs_dream_series.png'), grille)
        print('\ngrille :', SORTIE / 'fk_vs_dream_series.png')


if __name__ == '__main__':
    main()
