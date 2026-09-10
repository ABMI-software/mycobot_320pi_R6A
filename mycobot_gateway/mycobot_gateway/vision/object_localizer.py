#!/usr/bin/env python3
"""Déprojection pixel → coordonnées 3D dans le repère BASE du robot.

Un objet posé sur la table est localisé à partir de :
  - son pixel (u, v) dans l'image (donné par YOLO),
  - l'intrinsèque caméra K (training/calibration/cam_3.meta.json via camera_registry),
  - l'extrinsèque caméra→base T_world_cam (arducam_extrinsic_dream_v4.yaml),
  - l'hypothèse "l'objet est sur le plan table" z = table_z (repère base).

Mono-caméra : on ne peut pas retrouver la profondeur sans hypothèse. Le plan
table fournit cette contrainte -> le rayon caméra qui passe par (u,v) est
intersecté avec z=table_z, ce qui donne un point 3D unique.

Tout est en MÈTRES (comme la FK DREAM). Pour le vrai robot, send_coords attend
des millimètres -> multiplier par 1000 au moment de commander.
"""
from __future__ import annotations

from pathlib import Path

import numpy as np
import yaml

_CALIB_DIR = Path(__file__).resolve().parents[3] / 'training' / 'calibration'
_DEFAULT_EXTRINSIC = _CALIB_DIR / 'arducam_extrinsic_dream_v4.yaml'


def load_extrinsic(path: Path | str = _DEFAULT_EXTRINSIC) -> np.ndarray:
    """Charge T_world_cam (4x4) : mappe un point du repère CAMÉRA vers le repère BASE."""
    data = yaml.safe_load(Path(path).read_text())
    T = np.asarray(data['T_world_cam'], dtype=np.float64)
    if T.shape != (4, 4):
        raise ValueError(f'T_world_cam doit être 4x4, reçu {T.shape}')
    return T


def pixel_to_base(u: float, v: float, K: np.ndarray, T_world_cam: np.ndarray,
                  table_z: float) -> np.ndarray:
    """(u,v) pixel -> point 3D (x,y,z) dans le repère base, sur le plan z=table_z.

    Args:
        u, v: pixel (colonne, ligne) du centre de l'objet.
        K: intrinsèque 3x3 (fx,fy,cx,cy) à la même résolution que l'image.
        T_world_cam: extrinsèque 4x4 caméra->base.
        table_z: hauteur du plan de la table dans le repère base (m).

    Returns:
        np.array([x, y, table_z]) en mètres, repère base.
    """
    # rayon dans le repère caméra (direction, non normalisée)
    d_cam = np.linalg.inv(K) @ np.array([u, v, 1.0], dtype=np.float64)

    R = T_world_cam[:3, :3]
    cam_origin = T_world_cam[:3, 3]          # position caméra dans le repère base
    d_world = R @ d_cam                        # direction du rayon dans le repère base

    if abs(d_world[2]) < 1e-9:
        raise ValueError('rayon parallèle au plan table : pas d\'intersection')

    # intersection rayon (cam_origin + s*d_world) avec le plan z = table_z
    s = (table_z - cam_origin[2]) / d_world[2]
    if s <= 0:
        raise ValueError('intersection derrière la caméra (s<=0) : extrinsèque/plan faux ?')

    return cam_origin + s * d_world


class ObjectLocalizer:
    """Localise un objet (pixel -> base) avec K + extrinsèque + plan table figés."""

    def __init__(self, K: np.ndarray, table_z: float,
                 extrinsic_path: Path | str = _DEFAULT_EXTRINSIC):
        self.K = np.asarray(K, dtype=np.float64)
        self.table_z = float(table_z)
        self.T_world_cam = load_extrinsic(extrinsic_path)

    def locate(self, u: float, v: float) -> np.ndarray:
        """Pixel -> np.array([x, y, table_z]) en mètres, repère base."""
        return pixel_to_base(u, v, self.K, self.T_world_cam, self.table_z)
