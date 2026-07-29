#!/usr/bin/env python3
"""Localisation d'objet multi-vues, extrinsèques LIVE via DREAM (sans calibration).

Principe (markerless, auto-adaptatif — cf. discussion pick-and-place) :

  1. Pour CHAQUE caméra qui voit le robot, DREAM détecte ses keypoints 2D.
     Avec les angles encodeurs -> FK -> keypoints 3D (repère base), un solvePnP
     donne l'extrinsèque caméra->base de CETTE frame. Aucune extrinsèque figée :
     si la caméra bouge, la frame suivante la recalcule (c'est le point de DREAM).
     -> `extrinsic_from_dream`  (réplique le solve du dashboard de validation).

  2. YOLO donne le pixel (u,v) de l'objet dans chaque vue. Chaque (pixel +
     extrinsèque) définit un RAYON dans le repère base -> `pixel_ray`.

  3. Deux rayons (ou plus) se croisent -> point 3D de l'objet par moindres
     carrés, SANS hypothèse de plan table -> `triangulate`. Avec une seule vue
     valable ce frame-là, on retombe sur l'intersection plan-table
     (`object_localizer.pixel_to_base`).

Tout est en MÈTRES, repère base (comme la FK DREAM). send_coords attend des mm.
"""
from __future__ import annotations

from typing import Sequence

import cv2
import numpy as np

# 7-keypoint FK — le modèle DREAM courant (vgg_ultimate_v4_mix_ft_e30). Passe à
# mycobot_fk_gripper quand le modèle 8-kp sera entraîné.
try:
    from training.dream.mycobot_fk import KEYPOINT_NAMES, forward_kinematics
except ImportError:  # exécuté hors package : chemin direct ajouté par l'appelant
    from mycobot_fk import KEYPOINT_NAMES, forward_kinematics


def extrinsic_from_dream(joint_q: Sequence[float], kp_2d: np.ndarray,
                         valid: np.ndarray, K: np.ndarray,
                         dist: np.ndarray | None = None) -> np.ndarray:
    """Extrinsèque LIVE caméra->base (T_world_cam 4x4) par solvePnP DREAM.

    Args:
        joint_q: 6 angles articulaires (rad), lus sur les encodeurs.
        kp_2d:   (N,2) pixels des keypoints DREAM (N = len(KEYPOINT_NAMES)).
        valid:   (N,) booléens — keypoints réellement détectés.
        K:        intrinsèque 3x3 de la caméra (même résolution que kp_2d).
        dist:     coeffs de distorsion (ou None -> zéros).

    Returns:
        T_world_cam (4x4) : mappe un point du repère CAMÉRA vers le repère BASE.

    Raises:
        ValueError si moins de 4 keypoints valides (PnP sous-déterminé).
    """
    valid = np.asarray(valid, dtype=bool)
    if valid.sum() < 4:
        raise ValueError(f'PnP: {int(valid.sum())} keypoints valides < 4')

    positions, _ = forward_kinematics(joint_q)
    pts_3d = np.array([positions[n] for n in KEYPOINT_NAMES], dtype=np.float64)
    idx = np.where(valid)[0]
    K = np.asarray(K, dtype=np.float64)
    dist = np.zeros(5) if dist is None else np.asarray(dist, dtype=np.float64)

    ok, rvec, tvec = cv2.solvePnP(
        pts_3d[idx], np.asarray(kp_2d, dtype=np.float64)[idx], K, dist,
        flags=cv2.SOLVEPNP_EPNP,
    )
    if not ok:
        raise ValueError('solvePnP a échoué')
    rvec, tvec = cv2.solvePnPRefineLM(
        pts_3d[idx], np.asarray(kp_2d, dtype=np.float64)[idx], K, dist, rvec, tvec)

    R, _ = cv2.Rodrigues(rvec)               # base -> caméra
    T_cam_world = np.eye(4)
    T_cam_world[:3, :3] = R
    T_cam_world[:3, 3] = tvec.flatten()
    return np.linalg.inv(T_cam_world)         # caméra -> base


def pixel_ray(u: float, v: float, K: np.ndarray,
              T_world_cam: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """(u,v) -> (origine caméra, direction unitaire) du rayon dans le repère base."""
    d_cam = np.linalg.inv(np.asarray(K, dtype=np.float64)) @ np.array([u, v, 1.0])
    R = T_world_cam[:3, :3]
    origin = T_world_cam[:3, 3].astype(np.float64)
    d_world = R @ d_cam
    n = np.linalg.norm(d_world)
    if n < 1e-9:
        raise ValueError('direction de rayon dégénérée')
    return origin, d_world / n


def triangulate(rays: Sequence[tuple[np.ndarray, np.ndarray]]) -> np.ndarray:
    """Point 3D minimisant la distance aux rayons (moindres carrés).

    Args:
        rays: liste de (origine, direction unitaire) dans le repère base.

    Returns:
        np.array([x, y, z]) en mètres, repère base.

    Raises:
        ValueError si moins de 2 rayons (triangulation impossible).
    """
    if len(rays) < 2:
        raise ValueError('triangulation: au moins 2 rayons requis')
    A = np.zeros((3, 3))
    b = np.zeros(3)
    for origin, d in rays:
        d = d / np.linalg.norm(d)
        P = np.eye(3) - np.outer(d, d)        # projette sur l'orthogonal du rayon
        A += P
        b += P @ origin
    return np.linalg.solve(A, b)


class MultiViewLocalizer:
    """Localise un objet à partir de 1+ vues, extrinsèques DREAM recalculées live.

    Une "vue" par caméra = (nom, K, dist). À chaque appel de `locate`, l'appelant
    fournit, par caméra : les angles encodeurs, les keypoints DREAM (2d+valid) et
    le pixel objet YOLO. Les extrinsèques sont TOUJOURS recalculées ce frame-là.
    """

    def __init__(self, cameras: dict[str, dict]):
        # cameras: {name: {'K': 3x3, 'dist': (5,) | None}}
        self.cameras = cameras

    def locate(self, joint_q: Sequence[float],
               observations: dict[str, dict]) -> np.ndarray:
        """observations: {cam_name: {'kp_2d','valid','obj_px'(u,v)}} -> (x,y,z) base.

        Utilise toutes les caméras dont l'extrinsèque DREAM se résout ce frame ;
        1 vue -> lève NeedTablePlane (l'appelant retombe sur pixel_to_base).
        """
        rays = []
        for name, obs in observations.items():
            cam = self.cameras[name]
            try:
                T_world_cam = extrinsic_from_dream(
                    joint_q, obs['kp_2d'], obs['valid'], cam['K'], cam.get('dist'))
            except ValueError:
                continue                       # DREAM n'a pas assez de kp cette vue
            u, v = obs['obj_px']
            rays.append(pixel_ray(u, v, cam['K'], T_world_cam))
        if len(rays) < 2:
            raise NeedTablePlane(len(rays))
        return triangulate(rays)


class NeedTablePlane(Exception):
    """Moins de 2 vues exploitables -> triangulation impossible, retomber mono."""

    def __init__(self, n_rays: int):
        super().__init__(f'{n_rays} vue(s) exploitable(s) < 2 : triangulation impossible')
        self.n_rays = n_rays
