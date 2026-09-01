#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Extrinseque SVPRO ajustee sur les 16 coins des 4 marqueurs de la planche.

Remplace l'ajustement a 3 marqueurs (`svpro_extrinsic_montage_0901.yaml`,
12 coins) : le marqueur 25 n'etait pas decode par la SVPRO, il sortait du champ
par le bas. Il y est entre apres inclinaison de la camera le 01/09, et le
verrouillage du focus a 40 (`svpro_verrou_focus.sh`) l'a rendu decodable.

Les positions 3D des coins ne viennent PAS du ruban mais de l'ARDUCAM : chaque
coin detecte y est retro-projete sur le plan Z=0 via son extrinseque marqueurs
validee. C'est plus precis que les centres mesures (+-5 mm), et surtout ca donne
16 correspondances la ou les centres n'en donnent que 4.

Les marqueurs de la planche ne bougent JAMAIS : on regle toujours la camera.

Usage (venv_dream ou systeme, aucun affichage) :
    python3 scripts/svpro_extrinsic_4_marqueurs.py [--trames 25] [--ecrire]
Sans --ecrire, rien n'est sauvegarde : le script mesure et rapporte.
"""
import argparse
import json
import subprocess
from pathlib import Path

import cv2
import numpy as np
import yaml

RACINE = Path(__file__).resolve().parents[1]
CALIB = RACINE / 'training' / 'calibration'
SORTIE = CALIB / 'svpro_extrinsic_4marqueurs.yaml'

MARQUEURS = yaml.safe_load((CALIB / 'workspace_markers.yaml').read_text())
ATTENDUS = sorted(MARQUEURS['markers'])


def index_v4l2(motif):
    sortie = subprocess.run(['v4l2-ctl', '--list-devices'],
                            capture_output=True, text=True).stdout
    for bloc in sortie.split('\n\n'):
        if motif.lower() in bloc.split('\n')[0].lower():
            for ligne in bloc.split('\n')[1:]:
                if ligne.strip().startswith('/dev/video'):
                    return int(ligne.strip().removeprefix('/dev/video'))
    raise SystemExit(f'camera « {motif} » introuvable')


def intrinseque(stem, largeur, hauteur):
    d = np.load(CALIB / f'{stem}.npz')
    K, dist = d['mtx'].astype(float), d['dist'].astype(float).ravel()
    meta = json.loads((CALIB / f'{stem}.meta.json').read_text())
    lc, hc = meta['resolution']
    if (lc, hc) != (largeur, hauteur):
        K = K.copy()
        K[0] *= largeur / lc
        K[1] *= hauteur / hc
    return K, dist


def detecteur():
    dico = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
    if hasattr(cv2.aruco, 'ArucoDetector'):
        par = cv2.aruco.DetectorParameters()
        par.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        moteur = cv2.aruco.ArucoDetector(dico, par)
        return lambda g: moteur.detectMarkers(g)[:2]
    par = cv2.aruco.DetectorParameters_create()
    par.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    return lambda g: cv2.aruco.detectMarkers(g, dico, parameters=par)[:2]


def moyenne_coins(idx, controles, trames, detecte):
    """Coins moyennes par marqueur, sur `trames` images. Renvoie aussi le compte
    de detections et la nettete, qui servent de verdict sur la mise au point."""
    cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    if not cap.isOpened():
        raise SystemExit(f'/dev/video{idx} ne s ouvre pas')
    # Les controles V4L2 doivent etre poses APRES l'ouverture du flux : avant,
    # le pilote les reinitialise au demarrage (autofocus rallume, expo auto).
    for c in controles:
        subprocess.run(['v4l2-ctl', '-d', f'/dev/video{idx}', '--set-ctrl', c],
                       capture_output=True)
    for _ in range(8):
        cap.read()

    accum, comptes, nettetes = {}, {}, []
    for _ in range(trames):
        ok, img = cap.read()
        if not ok:
            continue
        gris = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        nettetes.append(cv2.Laplacian(gris, cv2.CV_64F).var())
        coins, ids = detecte(gris)
        if ids is None:
            continue
        for c, i in zip(coins, ids.ravel()):
            i = int(i)
            if i not in ATTENDUS:
                continue
            accum.setdefault(i, []).append(c.reshape(4, 2))
            comptes[i] = comptes.get(i, 0) + 1
    cap.release()
    moyennes = {i: np.mean(v, axis=0) for i, v in accum.items()}
    return moyennes, comptes, float(np.median(nettetes)) if nettetes else 0.0


def retroprojette_sol(pix, K, dist, T_cam_world):
    """Pixel -> point du plan Z=0 dans le repere base, via l'extrinseque."""
    T_world_cam = np.linalg.inv(T_cam_world)
    C = T_world_cam[:3, 3]
    R = T_world_cam[:3, :3]
    norm = cv2.undistortPoints(pix.reshape(-1, 1, 2).astype(np.float64),
                               K, dist).reshape(-1, 2)
    pts = []
    for u, v in norm:
        d = R @ np.array([u, v, 1.0])
        pts.append(C + d * (-C[2] / d[2]))          # intersection avec Z=0
    return np.array(pts)


def pnp(objets, images, K, dist):
    ok, rvec, tvec = cv2.solvePnP(objets.reshape(-1, 1, 3),
                                  images.reshape(-1, 1, 2), K, dist,
                                  flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        raise SystemExit('solvePnP a echoue')
    rvec, tvec = cv2.solvePnPRefineLM(objets.reshape(-1, 1, 3),
                                      images.reshape(-1, 1, 2), K, dist,
                                      rvec, tvec)
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(rvec)[0]
    T[:3, 3] = tvec.ravel()
    return T, rvec, tvec


def residus(objets, images, K, dist, rvec, tvec):
    proj = cv2.projectPoints(objets.reshape(-1, 1, 3), rvec, tvec,
                             K, dist)[0].reshape(-1, 2)
    return np.linalg.norm(proj - images, axis=1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--trames', type=int, default=25)
    ap.add_argument('--focus', type=int, default=40)
    ap.add_argument('--ecrire', action='store_true')
    args = ap.parse_args()

    detecte = detecteur()

    i_ard = index_v4l2('Arducam')
    i_svp = index_v4l2('5MP')
    print(f'arducam /dev/video{i_ard}   svpro /dev/video{i_svp}\n')

    ard, n_ard, net_ard = moyenne_coins(
        i_ard, ('auto_exposure=1', 'exposure_time_absolute=75'),
        args.trames, detecte)
    print(f'ARDUCAM  nettete {net_ard:6.1f}   marqueurs '
          + ' '.join(f'{i}:{n_ard.get(i, 0)}/{args.trames}' for i in ATTENDUS))

    svp, n_svp, net_svp = moyenne_coins(
        i_svp, ('focus_automatic_continuous=0', f'focus_absolute={args.focus}',
                'sharpness=0', 'contrast=1'),
        args.trames, detecte)
    print(f'SVPRO    nettete {net_svp:6.1f}   marqueurs '
          + ' '.join(f'{i}:{n_svp.get(i, 0)}/{args.trames}' for i in ATTENDUS))

    communs = sorted(set(ard) & set(svp))
    print(f'\nmarqueurs exploitables : {communs}  ({len(communs) * 4} coins)')
    if len(communs) < 3:
        raise SystemExit('moins de 3 marqueurs communs — rien a ajuster')

    K_ard, d_ard = intrinseque('cam_3', 640, 480)
    K_svp, d_svp = intrinseque('cam_2', 640, 480)
    T_ard = np.array(yaml.safe_load(
        (CALIB / 'arducam_extrinsic_pick.yaml').read_text())['T_cam_world'])

    sol = {i: retroprojette_sol(ard[i], K_ard, d_ard, T_ard) for i in communs}
    for i in communs:
        cote = np.mean([np.linalg.norm(sol[i][k] - sol[i][(k + 1) % 4])
                        for k in range(4)]) * 1000
        centre = sol[i].mean(axis=0) * 1000
        ruban = np.array(MARQUEURS['markers'][i])
        print(f'  {i}: cote {cote:5.1f} mm   centre '
              f'({centre[0]:6.1f},{centre[1]:7.1f}) vs ruban '
              f'({ruban[0]:6.1f},{ruban[1]:7.1f})   ecart '
              f'{np.linalg.norm(centre[:2] - ruban[:2]):4.1f} mm')

    objets = np.concatenate([sol[i] for i in communs])
    images = np.concatenate([svp[i] for i in communs]).astype(np.float64)
    T, rvec, tvec = pnp(objets, images, K_svp, d_svp)
    r = residus(objets, images, K_svp, d_svp, rvec, tvec)
    rms = float(np.sqrt((r ** 2).mean()))
    print(f'\najustement {len(communs)} marqueurs : RMS {rms:.2f} px  '
          f'(max {r.max():.2f})')

    croisee = {}
    for i in communs:
        garde = [j for j in communs if j != i]
        o = np.concatenate([sol[j] for j in garde])
        im = np.concatenate([svp[j] for j in garde]).astype(np.float64)
        _, rv, tv = pnp(o, im, K_svp, d_svp)
        e = residus(sol[i], svp[i].astype(np.float64), K_svp, d_svp, rv, tv)
        croisee[i] = float(np.median(e))
        print(f'  sans {i} : erreur mediane sur {i} = {croisee[i]:5.2f} px')

    T_world_cam = np.linalg.inv(T)
    pos = T_world_cam[:3, 3]
    print(f'\ncamera SVPRO en base : ({pos[0]:.3f}, {pos[1]:.3f}, {pos[2]:.3f}) m')
    ancien = np.array(yaml.safe_load(
        (CALIB / 'svpro_extrinsic_montage_0901.yaml').read_text()
    )['camera_position_base_m'])
    print(f'ecart avec l ajustement 3 marqueurs : '
          f'{np.linalg.norm(pos - ancien) * 1000:.1f} mm')

    if not args.ecrire:
        print('\n(--ecrire absent : rien sauvegarde)')
        return

    SORTIE.write_text(yaml.safe_dump({
        'source': (f'PnP sur les {len(communs) * 4} coins des marqueurs '
                   f'{communs}, positions 3D retro-projetees depuis l ARDUCAM '
                   f'(extrinseque marqueurs validee). Coins moyennes sur '
                   f'{args.trames} trames live. Le 25 est devenu decodable '
                   f'apres inclinaison de la camera et verrouillage du focus '
                   f'a {args.focus} (svpro_verrou_focus.sh).'),
        'camera': 'svpro',
        'intrinsics_stem': 'cam_2',
        'resolution': [640, 480],
        'frame_id': 'base_link',
        'focus_absolute': args.focus,
        'markers_used': communs,
        'rms_reproj_px': round(rms, 2),
        'detections_par_marqueur': {int(i): int(n_svp.get(i, 0))
                                    for i in ATTENDUS},
        'nettete_laplacien': round(net_svp, 1),
        'validation_croisee_un_marqueur_retire_px': {
            int(i): round(v, 2) for i, v in croisee.items()},
        'remplace': ('svpro_extrinsic_montage_0901.yaml (3 marqueurs, 12 coins)'
                     ' — NON modifie : le jeu real_montage_0901 a ete converti'
                     ' avec, et doit le rester'),
        'T_cam_world': T.tolist(),
        'T_world_cam': T_world_cam.tolist(),
        'camera_position_base_m': pos.tolist(),
    }, sort_keys=False, default_flow_style=False))
    print(f'\necrit : {SORTIE.relative_to(RACINE)}')


if __name__ == '__main__':
    main()
