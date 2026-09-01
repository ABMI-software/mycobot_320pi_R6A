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

# Fenetre reellement vue par le reseau : `shrink-and-crop` 640x480 -> 400x400
# gardre x dans [80, 560] et jette 25 % de l'image en deux bandes verticales.
# Un keypoint hors de la fenetre n'est pas "mal detecte", il est INVISIBLE.
FENETRE_X = (80, 560)
MARGE_PX = 30          # on s'ecarte du bord de coupe, un keypoint a x=82 est
                       # dans la fenetre mais colle au bord


def variantes(base, j1_min=0.0, j1_max=95.0, pas=15.0):
    """Meme pose a differents azimuts : faire tourner J1 SEUL conserve
    exactement l'inclinaison de l'outil, donc la pince reste pointee vers le
    bas comme en travail."""
    for j1 in np.arange(j1_min, j1_max + 1e-6, pas):
        yield [float(j1)] + list(base[1:])


def visible(q, rvec, tvec, K, dist):
    """Les 7 keypoints tombent-ils dans la fenetre reseau, avec de la marge ?"""
    pos, _ = forward_kinematics(np.radians(q))
    obj = np.array([pos[n] for n in KEYPOINT_NAMES], float)
    proj = cv2.projectPoints(obj, rvec, tvec, K, dist)[0].reshape(-1, 2)
    x, y = proj[:, 0], proj[:, 1]
    return (bool(np.all((FENETRE_X[0] + MARGE_PX <= x) & (x < FENETRE_X[1] - MARGE_PX))
                 and np.all((0 <= y) & (y < 480))), proj)


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
    ap.add_argument('--balayage', action='store_true',
                    help='balayer J1 sur chaque pose de base, pour tester la '
                         'STABILITE du decalage plutot que sa valeur')
    ap.add_argument('--host', default='10.10.0.224')
    args = ap.parse_args()

    net = dream.create_network_from_config_file(
        str(CHECKPOINT.with_suffix('.yaml')), str(CHECKPOINT))
    cameras = {nom: modele_camera(cfg) for nom, cfg in VUES}
    rvec, tvec, K, dist = cameras['arducam']

    if args.balayage:
        # Ecran AVANT de bouger : une pose dont les keypoints tombent hors de la
        # fenetre reseau ne mesure rien, autant ne pas la jouer.
        candidates, rejetees = [], 0
        for base in (POSES[0], POSES[3], POSES[6]):
            for q in variantes(base):
                ok, _ = visible(q, rvec, tvec, K, dist)
                candidates.append(q) if ok else None
                rejetees += 0 if ok else 1
        poses = candidates
        print(f'{len(poses)} poses retenues, {rejetees} rejetees hors fenetre '
              f'reseau (x hors [{FENETRE_X[0]+MARGE_PX}, {FENETRE_X[1]-MARGE_PX}])')
    else:
        poses = POSES[:args.n]

    from pick_and_place_real import Bridge
    pont = Bridge(args.host)
    print(f'Depart : {np.round(pont.get_angles(), 1).tolist()}')
    print(f'{len(poses)} poses — LE BRAS VA BOUGER\n')
    from PIL import Image

    par_pose = []          # (q, n_detectes, decalage moyen dx dy, dispersion)
    P_tout, D_tout = [], []
    print(f'{"pose":>4s} {"J1":>6s} {"det":>5s} {"dx":>7s} {"dy":>7s} '
          f'{"|d|":>7s} {"disp":>6s}')
    for i, pose in enumerate(poses, 1):
        q = rejoint(pont, pose)
        pos, _ = forward_kinematics(np.radians(q))
        obj = np.array([pos[n] for n in KEYPOINT_NAMES], float)
        proj = cv2.projectPoints(obj, rvec, tvec, K, dist)[0].reshape(-1, 2)
        image = capture(dict(VUES)['arducam'])
        h, w = image.shape[:2]
        kps = np.array(net.keypoints_from_image(
            Image.fromarray(cv2.cvtColor(image, cv2.COLOR_BGR2RGB))
        )['detected_keypoints'], float)

        e = []
        for k in range(7):
            du, dv = kps[k]
            if np.all(np.isfinite(kps[k])) and 0 <= du < w and 0 <= dv < h:
                e.append(kps[k] - proj[k])
                P_tout.append(proj[k]); D_tout.append(kps[k])
        if len(e) >= 2:
            e = np.array(e)
            t = e.mean(axis=0)
            disp = float(np.sqrt(((e - t) ** 2).sum(axis=1).mean()))
            par_pose.append((q[0], len(e), t, disp))
            print(f'{i:4d} {q[0]:6.1f} {len(e):3d}/7 {t[0]:+7.1f} {t[1]:+7.1f} '
                  f'{np.hypot(*t):7.1f} {disp:6.1f}')
        else:
            print(f'{i:4d} {q[0]:6.1f} {len(e):3d}/7  trop peu de detections')

    if len(par_pose) < 2:
        print('\npas assez de poses exploitables')
        return

    T = np.array([t for _, _, t, _ in par_pose])
    moy, ecart = T.mean(axis=0), T.std(axis=0)
    print(f'\n=== stabilite du decalage sur {len(T)} poses ===')
    print(f'decalage moyen   ({moy[0]:+.1f}, {moy[1]:+.1f})  = {np.hypot(*moy):.1f} px')
    print(f'ecart-type       ({ecart[0]:5.1f}, {ecart[1]:5.1f}) px')
    print(f'etendue          dx {T[:,0].min():+.0f}..{T[:,0].max():+.0f}   '
          f'dy {T[:,1].min():+.0f}..{T[:,1].max():+.0f}')

    P, D = np.array(P_tout), np.array(D_tout)
    avant = float(np.sqrt(((D - P) ** 2).sum(axis=1).mean()))
    apres = float(np.sqrt(((D - P - moy) ** 2).sum(axis=1).mean()))
    print(f'\nRMS DREAM<->FK   avant correction {avant:5.1f} px')
    print(f'                 apres correction {apres:5.1f} px  '
          f'({100*(1-apres/avant):.0f} % absorbes)')
    mm = apres / 495.0 * 1060.0
    print(f'\nreste ~{mm:.0f} mm sur la planche apres correction constante')
    verdict = ('CORRIGEABLE : un decalage constant a deux parametres suffit'
               if np.hypot(*ecart) < 0.4 * np.hypot(*moy)
               else 'NON corrigeable par une constante : le decalage depend de la pose')
    print(f'=> {verdict}')
    np.savez(SORTIE.parent / 'training' / 'calibration' / 'dream_biais.npz',
             P=P, D=D, decalage=moy, ecart=ecart)


if __name__ == '__main__':
    main()
