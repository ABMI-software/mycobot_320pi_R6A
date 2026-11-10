#!/usr/bin/env python3
"""Azimut, distance et élévation d'une caméra réelle, dans la convention du synthétique.

    python training/dream/geometrie_camera.py training/calibration/e2_P01_arducam_extrinsic.yaml [...]

Lit `T_cam_world` (repère base → caméra) d'une extrinsèque et place la caméra
comme le fait `synthetic_data_collector_v3_camera_libre` : distance et élévation
autour du point visé `VISEE`, azimut autour de `VISEE_XY` par la fonction même
qui découpe le secteur tenu à l'écart à la conversion NDDS. Une position réelle
« dans le secteur 60-100° » l'est donc au même sens qu'une trame synthétique
(protocole E2, docs/PROTOCOLE_E2_MULTIVUES.md).
"""
import argparse
import math
from pathlib import Path

import numpy as np
import yaml

from convert_to_ndds_camera_libre import azimut_deg, dans_secteur

# CollecteurCameraLibre.VISEE, DIST_MIN/MAX, ELEV_MIN/MAX.
VISEE = np.array([0.10, 0.0, 0.20])
DISTANCE_SYNTH_M = (0.70, 1.45)
ELEVATION_SYNTH_DEG = (18.0, 86.0)
SECTEUR_TENU_DEG = (60.0, 100.0)


def geometrie(T_cam_world):
    T = np.asarray(T_cam_world, float)
    centre = -T[:3, :3].T @ T[:3, 3]
    d = centre - VISEE
    distance = float(np.linalg.norm(d))
    return centre, azimut_deg(centre), distance, math.degrees(math.asin(d[2] / distance))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument('extrinseques', nargs='+', type=Path)
    args = ap.parse_args()
    print(f"{'extrinsèque':45s} {'x':>6s} {'y':>6s} {'z':>6s} {'azimut':>7s} {'dist':>5s} {'élév':>5s}  remarques")
    for chemin in args.extrinseques:
        centre, az, dist, elev = geometrie(yaml.safe_load(chemin.read_text())['T_cam_world'])
        remarques = []
        if dans_secteur(az, SECTEUR_TENU_DEG):
            remarques.append('secteur tenu')
        if not DISTANCE_SYNTH_M[0] <= dist <= DISTANCE_SYNTH_M[1]:
            remarques.append('distance hors synthétique')
        if not ELEVATION_SYNTH_DEG[0] <= elev <= ELEVATION_SYNTH_DEG[1]:
            remarques.append('élévation hors synthétique')
        print(f'{chemin.name:45s} {centre[0]:6.3f} {centre[1]:6.3f} {centre[2]:6.3f} '
              f'{az:7.1f} {dist:5.2f} {elev:5.1f}  {", ".join(remarques)}')


if __name__ == '__main__':
    main()
