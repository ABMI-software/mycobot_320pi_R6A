#!/usr/bin/env python3
"""Extrinsèque par le robot comme mire : un marqueur ArUco solidaire de la pince.

Les images et les angles viennent de `training/capture_real_tri.py --jeu e2`
(labels.csv + images/<camera>/). Pour chaque pose, les 4 coins de chaque marqueur
de la pince sont détectés ; on ajuste ensemble, par reprojection des coins :
  - la pose de chaque marqueur dans le repère de link6 (inconnue, commune aux caméras),
  - la pose de chaque caméra dans le repère de la FK (celui des labels).
Aucune planche, aucun pointage : la référence est la FK, celle qui annote.

Contrôle : leave-one-out par pose. Chaque pose est retirée, l'ajustement refait,
et l'on mesure l'erreur de reprojection de ses coins (px). Critère (protocole E2,
amendement du 26/11) : médiane <= 2 px par caméra.

    .venv/bin/python scripts/extrinseque_robot_mire.py <session> [<session> ...] \\
        --marqueur 21=40 --marqueur 7=40 --intrinseques svpro=e2_20261124_svpro \\
        --intrinseques arducam=arducam_640_1911 --sortie-prefixe e2_P01
"""
import argparse
import csv
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.optimize import least_squares

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE / 'training' / 'dream'))
sys.path.insert(0, str(RACINE / 'mycobot_gateway' / 'mycobot_gateway' / 'vision'))
from camera_registry import load_intrinsics  # noqa: E402
from mycobot_fk import forward_kinematics  # noqa: E402

CALIB = RACINE / 'training' / 'calibration'
SEUIL_PX = 2.0
REJET_PX = 6.0
FIXE_PX = 3.0
# Intrinseques etalonnees en 640x480. La SVPRO capturee en 2592x1944 (--mire) a le meme
# champ (verifie a 0,1 px sur les marqueurs de la table) : ses coins se ramenent par echelle.
LARGEUR_INTRINSEQUES = 640


def T_de(p):
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(np.asarray(p[:3], float))[0]
    T[:3, 3] = p[3:6]
    return T


def p_de(T):
    return np.concatenate([cv2.Rodrigues(T[:3, :3])[0].ravel(), T[:3, 3]])


def centre(q):
    """Intersection des diagonales : le centre du carre, juste aussi en perspective."""
    a, b = np.cross(np.append(q[0], 1), np.append(q[2], 1)), np.cross(np.append(q[1], 1), np.append(q[3], 1))
    x = np.cross(a, b)
    return x[:2] / x[2]


def charge(sessions, marqueurs, intrinseques, planche=None):
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000),
                                  cv2.aruco.DetectorParameters())
    obs = []
    for s in sessions:
        for ligne in csv.DictReader((s / 'labels.csv').open()):
            cam = ligne['camera']
            if cam not in intrinseques:
                continue
            gris = cv2.imread(str(s / ligne['image_path']), cv2.IMREAD_GRAYSCALE)
            coins, ids, _ = det.detectMarkers(gris)
            if ids is None:
                continue
            echelle = LARGEUR_INTRINSEQUES / gris.shape[1]
            q = np.radians([float(ligne[f'j{j}_deg']) for j in range(1, 7)])
            T6 = forward_kinematics(q)[1][6]
            for c, i in zip(coins, ids.ravel()):
                if planche and int(i) in planche:
                    obs.append({'cam': cam, 'mk': 'planche', 'X': planche[int(i)], 'id': int(i),
                                'uv': centre(c.reshape(4, 2).astype(float) * echelle)[None],
                                'pose': f"{s.name}/{ligne['index']}"})
                elif int(i) in marqueurs:
                    obs.append({'cam': cam, 'mk': int(i), 'T6': T6, 'uv': c.reshape(4, 2).astype(float) * echelle,
                                'pose': f"{s.name}/{ligne['index']}"})
    # Un identifiant peut exister aussi sur la table (le 21 des cales) : une detection
    # qui ne bouge pas avec le bras n'est pas le marqueur de la pince.
    fixes = []
    for cam, mk in {(o['cam'], o['mk']) for o in obs if o['mk'] != 'planche'}:
        centres = np.array([o['uv'].mean(axis=0) for o in obs if (o['cam'], o['mk']) == (cam, mk)])
        mediane = np.median(centres, axis=0)
        fixes += [o for o in obs if (o['cam'], o['mk']) == (cam, mk)
                  and np.linalg.norm(o['uv'].mean(axis=0) - mediane) < FIXE_PX]
    if fixes:
        print(f'{len(fixes)} detections immobiles ecartees (marqueur de la table)')
    return [o for o in obs if not any(o is f for f in fixes)]


def init_planche(obs, cams, K):
    """Pose de chaque camera par PnP sur les centres des marqueurs de la table."""
    T = {}
    for cam in cams:
        vus = {}
        for o in obs:
            if o['cam'] == cam and o['mk'] == 'planche':
                vus.setdefault(o['id'], []).append(o)
        if len(vus) < 4:
            return None
        X = np.array([v[0]['X'] for v in vus.values()])
        uv = np.array([np.median([o['uv'][0] for o in v], axis=0) for v in vus.values()])
        ok, rv, tv = cv2.solvePnP(X, uv, *K[cam], flags=cv2.SOLVEPNP_SQPNP)
        T[cam] = np.concatenate([rv.ravel(), tv.ravel()])
    return T


def init(obs, cams, mks, objp, K):
    """Hand-eye eye-to-hand (base->link6 inversé) par caméra, sur son marqueur le plus vu ;
    chaque marqueur est ensuite placé sur link6 par une image où il est vu."""
    T_base_cam = {}
    for cam in cams:
        vus = [o['mk'] for o in obs if o['cam'] == cam]
        mk = max(set(vus), key=vus.count)
        A = []
        for o in obs:
            if o['cam'] == cam and o['mk'] == mk:
                ok, rv, tv = cv2.solvePnP(objp[mk], o['uv'], *K[cam], flags=cv2.SOLVEPNP_IPPE_SQUARE)
                Ti = np.linalg.inv(o['T6'])
                A.append((Ti[:3, :3], Ti[:3, 3], cv2.Rodrigues(rv)[0], tv.ravel()))
        if len(A) < 3:
            raise SystemExit(f'{cam} : {len(A)} images de son marqueur le plus vu, il en faut au moins 3')
        R, t = cv2.calibrateHandEye(*[[a[k] for a in A] for k in range(4)], method=cv2.CALIB_HAND_EYE_PARK)
        T_base_cam[cam] = np.eye(4)
        T_base_cam[cam][:3, :3], T_base_cam[cam][:3, 3] = R, t.ravel()
    Y = {}
    for mk in mks:
        o = next(o for o in obs if o['mk'] == mk)
        ok, rv, tv = cv2.solvePnP(objp[mk], o['uv'], *K[o['cam']], flags=cv2.SOLVEPNP_IPPE_SQUARE)
        Y[mk] = p_de(np.linalg.inv(o['T6']) @ T_base_cam[o['cam']] @ T_de(np.concatenate([rv.ravel(), tv.ravel()])))
    return Y, {c: p_de(np.linalg.inv(T)) for c, T in T_base_cam.items()}


def deplie(x, cams, mks):
    Y = {m: T_de(x[6 * i: 6 * i + 6]) for i, m in enumerate(mks)}
    n = 6 * len(mks)
    return Y, {c: T_de(x[n + 6 * i: n + 6 * i + 6]) for i, c in enumerate(cams)}


def residus(x, obs, cams, mks, objp, K):
    Y, Tc = deplie(x, cams, mks)
    r = []
    for o in obs:
        if o['mk'] == 'planche':
            T = Tc[o['cam']]
            uv = cv2.projectPoints(o['X'][None], cv2.Rodrigues(T[:3, :3])[0], T[:3, 3], *K[o['cam']])[0]
            r.append((uv.reshape(1, 2) - o['uv']).ravel())
            continue
        Tcm = Tc[o['cam']] @ o['T6'] @ Y[o['mk']]
        uv = cv2.projectPoints(objp[o['mk']], cv2.Rodrigues(Tcm[:3, :3])[0], Tcm[:3, 3],
                               *K[o['cam']])[0].reshape(4, 2)
        r.append((uv - o['uv']).ravel())
    return np.concatenate(r) if r else np.zeros(0)


def ajuste(x0, obs, cams, mks, objp, K):
    return least_squares(residus, x0, args=(obs, cams, mks, objp, K), loss='soft_l1', f_scale=1.0).x


def erreur_px(x, o, cams, mks, objp, K):
    return float(np.linalg.norm(residus(x, [o], cams, mks, objp, K).reshape(-1, 2), axis=1).mean())


def main():
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument('sessions', nargs='+', type=Path)
    a.add_argument('--marqueur', action='append', required=True, metavar='ID=MM',
                   help='marqueur colle sur la pince et son cote en mm (repetable)')
    a.add_argument('--intrinseques', action='append', required=True, metavar='CAMERA=STEM')
    a.add_argument('--planche', type=Path,
                   help='ajoute les centres des marqueurs de la table (ex. planche_actuelle.yaml, releves au robot)')
    a.add_argument('--sortie-prefixe', help='ecrit training/calibration/<prefixe>_<camera>_extrinsic_robot.yaml')
    args = a.parse_args()

    stems = dict(i.split('=', 1) for i in args.intrinseques)
    K = {c: load_intrinsics(s) for c, s in stems.items()}
    tailles = {int(k): float(v) for k, v in (m.split('=') for m in args.marqueur)}
    objp = {m: np.array([[-1, 1, 0], [1, 1, 0], [1, -1, 0], [-1, -1, 0]], float) * mm / 2000.0
            for m, mm in tailles.items()}

    planche = None
    if args.planche:
        ref = yaml.safe_load(args.planche.read_text())
        planche = {int(i): np.array(v, float) / 1000.0 for i, v in ref['markers'].items()}
    obs = charge(args.sessions, tailles, stems, planche)
    cams = sorted({o['cam'] for o in obs})
    mks = sorted({o['mk'] for o in obs if o['mk'] != 'planche'})
    if planche:
        print('planche : ' + ', '.join(f'{c} {sum(o["cam"] == c and o["mk"] == "planche" for o in obs)} centres'
                                       for c in cams))
    for c in stems:
        print(f'{c} : ' + ', '.join(f'marqueur {m} sur {sum(o["cam"] == c and o["mk"] == m for o in obs)} images'
                                    for m in tailles))
    p_planche = init_planche(obs, cams, K) if planche else None
    if p_planche:
        p0 = p_planche
        Y0 = {}
        for mk in mks:
            o = next(o for o in obs if o['mk'] == mk)
            ok, rv, tv = cv2.solvePnP(objp[mk], o['uv'], *K[o['cam']], flags=cv2.SOLVEPNP_IPPE_SQUARE)
            Y0[mk] = p_de(np.linalg.inv(o['T6']) @ np.linalg.inv(T_de(p0[o['cam']]))
                          @ T_de(np.concatenate([rv.ravel(), tv.ravel()])))
    else:
        Y0, p0 = init(obs, cams, mks, objp, K)
    x = ajuste(np.concatenate([*[Y0[m] for m in mks], *[p0[c] for c in cams]]), obs, cams, mks, objp, K)

    e = np.array([erreur_px(x, o, cams, mks, objp, K) for o in obs])
    garde = [o for o, ei in zip(obs, e) if ei <= REJET_PX]
    for o, ei in zip(obs, e):
        if ei > REJET_PX:
            print(f'  rejetee : {o["cam"]} marqueur {o["mk"]} {o["pose"]} a {ei:.1f} px (detection ou glissement)')
    x = ajuste(x, garde, cams, mks, objp, K)

    # Une pose retiree emporte toutes ses detections (toutes cameras, tous marqueurs).
    loo = {c: [] for c in cams}
    for pose in dict.fromkeys(o['pose'] for o in garde if o['mk'] != 'planche'):
        xk = ajuste(x, [o for o in garde if o['pose'] != pose or o['mk'] == 'planche'], cams, mks, objp, K)
        for o in garde:
            if o['pose'] == pose and o['mk'] != 'planche':
                loo[o['cam']].append(erreur_px(xk, o, cams, mks, objp, K))

    Ys, Tcs = deplie(x, cams, mks)
    print()
    for m in mks:
        print(f'marqueur {m} dans link6 : t = {np.round(Ys[m][:3, 3] * 1000, 1).tolist()} mm')
    for c in cams:
        Tcw = Tcs[c]
        l = np.array(loo[c])
        prof = np.median([(Tcw @ o['T6'] @ Ys[o['mk']])[2, 3] for o in garde if o['cam'] == c and o['mk'] != 'planche'])
        rp = [erreur_px(x, o, cams, mks, objp, K) for o in garde if o['cam'] == c and o['mk'] == 'planche']
        if rp:
            print(f'{c} : centres de la table, residu median {np.median(rp):.2f} px (ajustement, pas une justesse)')
        mm = l * prof / K[c][0][0, 0] * 1000
        ordre = np.array(loo[c])
        tiers = max(1, len(ordre) // 3)
        ok = np.median(l) <= SEUIL_PX
        print(f'{c} : {len(l)} images, leave-one-out mediane {np.median(l):.2f} px  p90 {np.percentile(l, 90):.2f}'
              f'  max {l.max():.2f}  (~{np.median(mm):.1f} mm a {prof:.2f} m)'
              f'  ; debut {np.median(ordre[:tiers]):.2f} / fin {np.median(ordre[-tiers:]):.2f} px'
              f'  -> {"CONFORME" if ok else "NON CONFORME"} (<= {SEUIL_PX} px)')
        print(f'   camera dans la base : {np.round(np.linalg.inv(Tcw)[:3, 3] * 1000).tolist()} mm')
        if args.sortie_prefixe:
            chemin = CALIB / f'{args.sortie_prefixe}_{c}_extrinsic_robot.yaml'
            chemin.write_text(yaml.safe_dump({
                'source': f'extrinseque_robot_mire.py, marqueurs {tailles} (mm) sur la pince',
                'sessions': [str(p) for p in args.sessions],
                'intrinsics_stem': stems[c],
                'frame_id': 'base_link',
                'n_images': len(l),
                'loo_mediane_px': float(np.median(l)), 'loo_p90_px': float(np.percentile(l, 90)),
                'loo_mediane_mm': float(np.median(mm)),
                'conforme': bool(ok),
                'T_marqueurs_link6': {m: Ys[m].tolist() for m in mks},
                'T_cam_world': Tcw.tolist()}, sort_keys=False))
            print(f'   -> {chemin}')


if __name__ == '__main__':
    main()
