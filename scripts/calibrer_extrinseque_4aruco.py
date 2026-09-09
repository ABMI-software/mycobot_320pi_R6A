#!/usr/bin/env python3
"""Calibration extrinseque camera -> base robot par 4 marqueurs ArUco fixes.

Script autonome : ne depend que d'opencv, numpy et pyyaml. Aucun import du
projet, il se donne tel quel.

    python calibrer_extrinseque_4aruco.py --index 3 --intrinsics cam_3.npz \
        --markers workspace_markers.yaml --out extrinseque.yaml

Ce qu'il produit : T_cam_world, la matrice 4x4 qui envoie un point du repere
base vers le repere camera. Son inverse donne la position de la camera.

Trois choix qui ne sont pas cosmetiques :

  - **16 coins, pas 4 centres.** Quatre centres, c'est 8 contraintes pour
    6 inconnues : le solveur interpole ses propres points, le residu tombe
    presque a zero et ne mesure plus rien. Seize coins donnent 32 equations,
    le residu redevient honnete et l'orientation dans le plan devient
    observable.
  - **La taille du marqueur est MESUREE sur l'image**, pas lue dans le YAML.
    Une erreur de 1,5 mm sur le cote decale chaque coin de 0,75 mm. Mesure du
    09/09/2026 : les marqueurs annonces a 50 mm font 48,4 mm.
  - **Validation leave-one-marker-out.** La pose est reajustee sur 3 marqueurs
    et l'erreur est mesuree sur le 4e, qui n'a pas servi. C'est la seule facon
    de distinguer « le modele colle a ses points » de « le modele predit un
    point neuf ».

Le YAML des marqueurs, en millimetres, repere base, Z=0 au plan de travail :

    frame_id: base_link
    units: mm
    marker_size_mm: 50.0
    markers:
      19: [ 96.8,  203.4, 0.0]
      23: [110.0, -179.0, 0.0]
      25: [535.0,  215.0, 0.0]
      26: [530.0, -176.0, 0.0]
"""
import argparse
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

DICO = cv2.aruco.DICT_4X4_1000


def parametres_detecteur():
    p = cv2.aruco.DetectorParameters()
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    p.adaptiveThreshWinSizeMin = 3
    p.adaptiveThreshWinSizeMax = 53
    p.adaptiveThreshWinSizeStep = 4
    return p


def detecte(bgr):
    """{id: (4,2) coins image}. Un id vu deux fois est ecarte : DICT_4X4_1000 a
    une faible distance de Hamming et decode regulierement du bois ou un cable
    en marqueur fantome."""
    gris = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    gris = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8)).apply(gris)
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(DICO),
                                  parametres_detecteur())
    coins, ids, _ = det.detectMarkers(gris)
    if ids is None:
        return {}
    plats = [int(i) for i in ids.flatten()]
    doubles = {i for i in plats if plats.count(i) > 1}
    return {i: c.reshape(4, 2).astype(float)
            for c, i in zip(coins, plats) if i not in doubles}


def capture(index, n, largeur, hauteur, exposition):
    if exposition is not None:
        for req in (['--set-ctrl', 'auto_exposure=1'],
                    ['--set-ctrl', f'exposure_time_absolute={exposition}']):
            subprocess.run(['v4l2-ctl', '-d', f'/dev/video{index}'] + req,
                           capture_output=True)
    cap = cv2.VideoCapture(index)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, largeur)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, hauteur)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    for _ in range(10):
        cap.read()
    trames = []
    for _ in range(n * 4):
        ok, img = cap.read()
        if ok:
            trames.append(img)
        if len(trames) >= n:
            break
    cap.release()
    if not trames:
        raise SystemExit(f'aucune image depuis /dev/video{index}')
    return trames


def vers_plan(uv, K, dist, T, z=0.0):
    """pixel -> point du plan horizontal Z=z dans le repere base (metres)."""
    p = cv2.undistortPoints(np.array([[uv]], float), K, dist).reshape(2)
    R, t = T[:3, :3], T[:3, 3]
    centre = -R.T @ t
    d = R.T @ np.array([p[0], p[1], 1.0])
    return centre + (z - centre[2]) / d[2] * d


def coins_3d(centre, cote, yaw_rad):
    """Les 4 coins d'un marqueur pose a plat, dans l'ordre d'OpenCV."""
    demi = cote / 2.0
    local = np.array([[-demi, demi], [demi, demi], [demi, -demi], [-demi, -demi]])
    c, s = np.cos(yaw_rad), np.sin(yaw_rad)
    tourne = local @ np.array([[c, s], [-s, c]])
    out = np.tile(centre, (4, 1))
    out[:, :2] += tourne
    return out


def pose(obj, img, K, dist):
    flag = cv2.SOLVEPNP_IPPE if len(obj) >= 4 else cv2.SOLVEPNP_SQPNP
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist, flags=flag)
    if not ok:
        raise SystemExit('solvePnP a echoue')
    rvec, tvec = cv2.solvePnPRefineLM(obj, img, K, dist, rvec, tvec)
    T = np.eye(4)
    T[:3, :3] = cv2.Rodrigues(rvec)[0]
    T[:3, 3] = tvec.ravel()
    return T, rvec, tvec


def main():
    a = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument('--index', type=int, default=3, help='index V4L2 de la camera')
    a.add_argument('--intrinsics', required=True, help='.npz contenant mtx et dist')
    a.add_argument('--markers', required=True, help='YAML des positions marqueurs')
    a.add_argument('--out', required=True, help='YAML extrinseque a ecrire')
    a.add_argument('--image', help='photo existante au lieu de la camera')
    a.add_argument('--frames', type=int, default=3,
                   help="images cumulees ; un marqueur doit etre vu sur TOUTES")
    a.add_argument('--exposure', type=int, help='exposition manuelle V4L2')
    a.add_argument('--width', type=int, default=640)
    a.add_argument('--height', type=int, default=480)
    a.add_argument('--table-z', type=float, default=0.0, help='plan de travail (mm)')
    args = a.parse_args()

    npz = np.load(args.intrinsics)
    K = npz['mtx'].astype(float)
    dist = npz['dist'].astype(float)

    d = yaml.safe_load(Path(args.markers).read_text())
    monde = {int(k): np.array(v, float) / 1000.0 for k, v in d['markers'].items()}
    cote_yaml = float(d.get('marker_size_mm', 50.0)) / 1000.0
    z_plan = args.table_z / 1000.0

    trames = ([cv2.imread(args.image)] if args.image else
              capture(args.index, args.frames, args.width, args.height, args.exposure))
    if trames[0] is None:
        raise SystemExit(f'image introuvable : {args.image}')

    par_trame = [detecte(t) for t in trames]
    # Un marqueur intermittent biaiserait l'ajustement entre les images qui
    # l'ont et celles qui ne l'ont pas : on exige de le voir partout.
    partout = set(par_trame[0])
    for p in par_trame[1:]:
        partout &= set(p)
    utiles = sorted(i for i in monde if i in partout)
    print(f'{len(trames)} image(s) — marqueurs connus {sorted(monde)} — '
          f'utilisables {utiles} ({len(utiles)}/{len(monde)})')
    if len(utiles) < 3:
        raise SystemExit('il faut au moins 3 marqueurs vus sur toutes les images '
                         '(degager le bras, ou baisser --frames)')
    if len(utiles) < 4:
        print('ATTENTION — 3 marqueurs : 6 contraintes pour 6 inconnues, pose '
              'tout juste determinee donc instable, et validation impossible.')

    # pose initiale sur les CENTRES, juste pour pouvoir retroprojeter les coins
    centres_img = {i: np.mean([p[i] for p in par_trame], axis=0).mean(axis=0)
                   for i in utiles}
    T0, _, _ = pose(np.array([monde[i] for i in utiles]),
                    np.array([centres_img[i] for i in utiles]), K, dist)

    # yaw et cote MESURES sur l'image, marqueur par marqueur
    obj, img, appartenance = [], [], []
    mesures = {}
    for i in utiles:
        quad = np.mean([p[i] for p in par_trame], axis=0)
        plan = np.array([vers_plan(uv, K, dist, T0, z_plan)[:2] for uv in quad])
        yaw = float(np.arctan2(*(plan[1] - plan[0])[::-1]))
        cote = float(np.mean([np.linalg.norm(plan[k] - plan[(k + 1) % 4])
                              for k in range(4)]))
        mesures[i] = (np.degrees(yaw), cote * 1000.0)
        for k, c3 in enumerate(coins_3d(monde[i], cote, yaw)):
            obj.append(c3)
            img.append(quad[k])
            appartenance.append(i)
    obj = np.array(obj)
    img = np.array(img)
    appartenance = np.array(appartenance)

    print('\nmesure sur l image, marqueur par marqueur :')
    for i in utiles:
        print(f'  id {i:3d} : yaw {mesures[i][0]:+7.1f} deg   cote {mesures[i][1]:6.2f} mm')
    moyen = np.mean([mesures[i][1] for i in utiles])
    ecart = np.std([mesures[i][1] for i in utiles])
    print(f'  cote moyen {moyen:.2f} mm (+-{ecart:.2f}) contre {cote_yaml*1000:.1f} '
          f'dans le YAML')
    if abs(moyen - cote_yaml * 1000) > 1.0:
        print(f'  -> ecart de {moyen - cote_yaml*1000:+.2f} mm : corriger '
              f'marker_size_mm apres verification au pied a coulisse')

    T, rvec, tvec = pose(obj, img, K, dist)
    proj = cv2.projectPoints(obj, rvec, tvec, K, dist)[0].reshape(-1, 2)
    err_px = np.linalg.norm(proj - img, axis=1)
    print(f'\najustement sur {len(obj)} coins :')
    for i in utiles:
        m = appartenance == i
        print(f'  id {i:3d} : reprojection {err_px[m].mean():5.2f} px')
    rms = float(np.sqrt((err_px ** 2).mean()))
    print(f'  RMS global {rms:.3f} px  ({"OK" if rms < 1.5 else "ELEVE"})')

    cam = -T[:3, :3].T @ T[:3, 3]
    print(f'\ncamera dans le repere base : X={cam[0]*1000:+.1f} Y={cam[1]*1000:+.1f} '
          f'Z={cam[2]*1000:+.1f} mm')

    print('\nposition vue par la camera contre position du YAML :')
    for i in utiles:
        vu = vers_plan(centres_img[i], K, dist, T, z_plan)[:2] * 1000.0
        att = monde[i][:2] * 1000.0
        print(f'  id {i:3d} : YAML ({att[0]:7.1f}, {att[1]:7.1f})  vu '
              f'({vu[0]:7.1f}, {vu[1]:7.1f})  ecart {np.linalg.norm(vu-att):5.2f} mm')

    if len(utiles) >= 4:
        print('\nvalidation leave-one-marker-out (le marqueur teste est EXCLU) :')
        pires = []
        for h in utiles:
            m = appartenance != h
            Th, rh, th = pose(obj[m], img[m], K, dist)
            vu = vers_plan(centres_img[h], K, dist, Th, z_plan)[:2] * 1000.0
            e = float(np.linalg.norm(vu - monde[h][:2] * 1000.0))
            pires.append(e)
            print(f'  id {h:3d} exclu : erreur au sol {e:5.2f} mm')
        print(f'  pire erreur sur un point NEUF : {max(pires):.2f} mm')
        print('  -> c est CE chiffre qui dit la justesse, pas le RMS ci-dessus')

    Path(args.out).write_text(yaml.safe_dump({
        'source': f'{len(obj)} coins ArUco, {len(trames)} image(s), IPPE + RefineLM',
        'camera_index': args.index,
        'reprojection_rms_px': round(rms, 4),
        'marker_size_mesure_mm': round(float(moyen), 3),
        'marker_size_yaml_mm': round(cote_yaml * 1000, 3),
        'markers_used': utiles,
        'frame_id': d.get('frame_id', 'base_link'),
        'resolution': [args.width, args.height],
        'T_cam_world': [[float(x) for x in r] for r in T],
    }, sort_keys=False))
    print(f'\necrit -> {args.out}')


if __name__ == '__main__':
    main()
