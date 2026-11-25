#!/usr/bin/env python3
"""Contrôle en hauteur de l'étape 2 de E2 (docs/PROTOCOLE_E2_MULTIVUES.md).

Un marqueur posé en hauteur, relevé au robot (XY, `pivot_pointe.py pointe`) et
au réglet (Z), est projeté par l'extrinsèque de la position ; on mesure l'écart
entre cette projection et le centre détecté du marqueur. L'extrinsèque est
ajustée sur des marqueurs au sol : cet écart dit ce qu'elle vaut à la hauteur
des keypoints.

    .venv/bin/python scripts/controle_hauteur_e2.py \\
        --extrinseque training/calibration/e2_P01_astra_extrinsic.yaml --astra \\
        --points results/pivot/<date>_pointe/points.csv --hauteur 1=57 --hauteur 21=115

Le centre détecté est l'intersection des diagonales du marqueur (la projection
exacte de son centre), moyenné sur les images où il est vu. Le XY d'un marqueur
est la moyenne de ses relevés `<id>` et `<id>b`.
"""
import argparse
import csv
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE / 'scripts'))
from calibrer_extrinseque_4aruco import capture, capture_astra, detecte  # noqa: E402

SEUIL_PX = 2.0


def centre_diagonales(quad):
    p0, p1, p2, p3 = quad
    a = np.array([p2 - p0, p1 - p3]).T
    s = np.linalg.solve(a, p1 - p0)[0]
    return p0 + s * (p2 - p0)


def main():
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument('--extrinseque', required=True)
    a.add_argument('--points', required=True, help='points.csv de pivot_pointe.py pointe')
    a.add_argument('--hauteur', action='append', required=True, metavar='ID=MM',
                   help='hauteur au reglet du centre du marqueur, au-dessus du plan du robot')
    a.add_argument('--astra', action='store_true')
    a.add_argument('--index', type=int, help='index V4L2 si ce n est pas l Astra')
    a.add_argument('--frames', type=int, default=30)
    a.add_argument('--critere', type=int, help='id dont l ecart decide (defaut : le plus haut)')
    args = a.parse_args()

    ext = yaml.safe_load(Path(args.extrinseque).read_text())
    T = np.array(ext['T_cam_world'], float)
    meta = json.loads((RACINE / 'training' / 'calibration' / f"{ext['intrinsics_stem']}.meta.json").read_text())
    r = meta['results']
    K = np.array([[r['fx'], 0, r['cx']], [0, r['fy'], r['cy']], [0, 0, 1]], float)
    dist = np.array(r['dist_coeffs'], float)

    hauteurs = {int(k): float(v) for k, v in (h.split('=') for h in args.hauteur)}
    releves = {}
    for ligne in csv.DictReader(open(args.points)):
        nom = ligne['nom'].rstrip('b')
        if nom.isdigit() and int(nom) in hauteurs:
            releves.setdefault(int(nom), []).append([float(ligne['x_mm']), float(ligne['y_mm'])])

    trames = capture_astra(args.frames) if args.astra else capture(args.index, args.frames, 640, 480, None)
    vus = {}
    for t in trames:
        for i, quad in detecte(t).items():
            if i in hauteurs:
                vus.setdefault(i, []).append(centre_diagonales(quad))

    rvec, tvec = cv2.Rodrigues(T[:3, :3])[0], T[:3, 3]
    resultats = {}
    for i in sorted(hauteurs):
        if i not in releves or i not in vus:
            print(f'id {i} : {"non pointe" if i not in releves else "non detecte"}')
            continue
        xy = np.mean(releves[i], axis=0)
        ecart_releves = np.linalg.norm(np.subtract(*releves[i][:2])) if len(releves[i]) > 1 else float('nan')
        monde = np.array([xy[0], xy[1], hauteurs[i]]) / 1000.0
        proj = cv2.projectPoints(monde.reshape(1, 3), rvec, tvec, K, dist)[0].reshape(2)
        det = np.mean(vus[i], axis=0)
        e_px = float(np.linalg.norm(proj - det))
        z_cam = float((T[:3, :3] @ monde + T[:3, 3])[2])
        e_mm = e_px * z_cam / r['fx'] * 1000.0
        resultats[i] = e_px
        print(f'id {i:3d} a {hauteurs[i]:5.1f} mm : projection ({proj[0]:.1f}, {proj[1]:.1f}) '
              f'detection ({det[0]:.1f}, {det[1]:.1f}) sur {len(vus[i])}/{len(trames)} images  '
              f'-> {e_px:.2f} px (~{e_mm:.1f} mm a {z_cam:.2f} m) ; releves ecartes de {ecart_releves:.1f} mm')

    if not resultats:
        raise SystemExit('aucun marqueur en hauteur a la fois pointe et detecte')
    decide = args.critere if args.critere is not None else max(resultats, key=hauteurs.get)
    ok = resultats.get(decide, float('inf')) <= SEUIL_PX
    print(f'critere : id {decide} <= {SEUIL_PX} px : {"CONFORME" if ok else "NON CONFORME"}')


if __name__ == '__main__':
    main()
