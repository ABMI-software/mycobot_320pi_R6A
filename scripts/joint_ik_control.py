#!/usr/bin/env python3
"""Commande cartésienne sûre calculée sur PC, exécutée par ``send_angles``.

La position est exprimée dans le repère base réel du robot (mm). L'orientation
est celle de la démonstration manuelle validée. La FK du dépôt possède un petit
biais par rapport à ``get_coords`` ; il est estimé sur cette démonstration et
appliqué à toutes les cibles.
"""
from __future__ import annotations

import json
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path

import numpy as np
from scipy.optimize import least_squares

sys.path.insert(0, str(Path(__file__).resolve().parent))
from diff_ik import _rot_vec, fk_pose  # noqa: E402


# Limites pratiques cohérentes avec le firmware (J2 est refusé hors ±137°).
JOINT_LIMITS_DEG = np.asarray([
    (-168.0, 168.0), (-135.0, 135.0), (-150.0, 150.0),
    (-145.0, 145.0), (-165.0, 165.0), (-180.0, 180.0),
], dtype=np.float64)


class MotionSafetyError(RuntimeError):
    """Une mesure réelle interdit de continuer le mouvement."""


@dataclass(frozen=True)
class IKSolution:
    angles_deg: tuple[float, ...]
    xyz_error_mm: float
    orientation_error_deg: float
    score: float


def _parse_vector(response: str, prefix: str, size: int) -> np.ndarray:
    if prefix not in response:
        raise MotionSafetyError(f'réponse inattendue pour {prefix}: {response}')
    try:
        value = json.loads(response.split(prefix, 1)[1].strip())
        vector = np.asarray(value, dtype=np.float64)
    except (ValueError, TypeError, json.JSONDecodeError) as exc:
        raise MotionSafetyError(f'réponse {prefix} illisible: {response}') from exc
    if vector.shape[0] < size or not np.all(np.isfinite(vector[:size])):
        raise MotionSafetyError(f'réponse {prefix} invalide: {response}')
    return vector[:size]


def read_angles(bridge) -> np.ndarray:
    """Lecture fraîche des six encodeurs articulaires, en degrés."""
    if hasattr(bridge, 'get_angles'):
        q = np.asarray(bridge.get_angles(), dtype=np.float64)
    else:
        q = _parse_vector(bridge.send({'action': 'get_angles'}), 'ANGLES:', 6)
    if q.shape != (6,) or not np.all(np.isfinite(q)):
        raise MotionSafetyError(f'angles robot invalides: {q}')
    return q


def read_actual_xyz(bridge) -> np.ndarray:
    """Position XYZ réellement déclarée par le robot, sans utiliser ses Euler."""
    return _parse_vector(bridge.send({'action': 'get_coords'}), 'COORDS:', 6)[:3]


def angular_distance_deg(a, b) -> float:
    delta = (np.asarray(a) - np.asarray(b) + 180.0) % 360.0 - 180.0
    return float(np.linalg.norm(delta))


def solve_all_ik(target_robot_xyz_mm, target_rotation, current_deg, manual_deg,
                 model_bias_mm=None, random_restarts=48, seed=320):
    """Cherche les minima IK distincts puis les classe par branche articulaire.

    Le score privilégie d'abord une pose exacte, puis la proximité aux encodeurs
    courants et à la pose de prise manuelle. Les solutions à plus de 1 mm ou 2°
    sont éliminées.
    """
    target_robot = np.asarray(target_robot_xyz_mm, dtype=np.float64)
    bias = np.zeros(3) if model_bias_mm is None else np.asarray(model_bias_mm, float)
    target_model = target_robot - bias
    target_rotation = np.asarray(target_rotation, dtype=np.float64)
    current = np.asarray(current_deg, dtype=np.float64)
    manual = np.asarray(manual_deg, dtype=np.float64)
    lo, hi = JOINT_LIMITS_DEG[:, 0], JOINT_LIMITS_DEG[:, 1]

    def residual(q):
        xyz, rotation = fk_pose(q)
        # 1 radian d'orientation équivaut ici à 80 mm : orientation bien tenue,
        # sans masquer une erreur XYZ.
        return np.r_[xyz - target_model,
                     _rot_vec(target_rotation @ rotation.T) * 80.0]

    rng = np.random.default_rng(seed)
    seeds = [np.clip(current, lo, hi), np.clip(manual, lo, hi)]
    # Variantes coude/poignet déterministes, puis couverture de tout l'espace.
    for j2 in (-120.0, -60.0, 30.0, 100.0):
        for j3 in (-100.0, -30.0, 50.0, 110.0):
            q = np.clip(manual.copy(), lo, hi)
            q[1], q[2] = j2, j3
            seeds.append(q)
    seeds.extend(rng.uniform(lo, hi) for _ in range(int(random_restarts)))

    unique = []
    for q0 in seeds:
        result = least_squares(residual, q0, bounds=(lo, hi), max_nfev=500,
                               xtol=1e-10, ftol=1e-10, gtol=1e-10)
        q = result.x
        xyz, rotation = fk_pose(q)
        xyz_error = float(np.linalg.norm(xyz - target_model))
        rot_error = math.degrees(float(np.linalg.norm(
            _rot_vec(target_rotation @ rotation.T))))
        if xyz_error > 1.0 or rot_error > 2.0:
            continue
        if any(angular_distance_deg(q, old[0]) < 2.0 for old in unique):
            continue
        score = (20.0 * xyz_error + 2.0 * rot_error +
                 angular_distance_deg(q, current) +
                 0.35 * angular_distance_deg(q, manual))
        unique.append((q.copy(), xyz_error, rot_error, score))

    return sorted((IKSolution(tuple(map(float, q)), xyz, rot, score)
                   for q, xyz, rot, score in unique), key=lambda item: item.score)


class JointIKController:
    """IK multi-branche, retour encodeurs/XYZ et descente verticale pas-à-pas."""

    def __init__(self, bridge, manual_angles_deg, manual_robot_xyz_mm,
                 speed=8, xyz_tolerance_mm=5.0, descent_step_mm=2.0,
                 settle_s=1.2, dry_run=False):
        self.bridge = bridge
        self.manual = np.asarray(manual_angles_deg, dtype=np.float64)
        self.manual_robot_xyz = np.asarray(manual_robot_xyz_mm, dtype=np.float64)
        manual_fk_xyz, self.rotation = fk_pose(self.manual)
        self.model_bias = self.manual_robot_xyz - manual_fk_xyz
        self.speed = int(speed)
        self.tolerance = float(xyz_tolerance_mm)
        self.step = float(descent_step_mm)
        self.settle = float(settle_s)
        self.dry_run = bool(dry_run)
        self._sim_q = self.manual.copy()
        self._expected_xyz = None

    def _angles(self):
        return self._sim_q.copy() if self.dry_run else read_angles(self.bridge)

    def _actual_xyz(self):
        if self.dry_run:
            xyz, _ = fk_pose(self._sim_q)
            return xyz + self.model_bias
        return read_actual_xyz(self.bridge)

    def verify_xyz(self, expected_xyz_mm, context='mouvement'):
        actual = self._actual_xyz()
        error = actual - np.asarray(expected_xyz_mm, dtype=np.float64)
        norm = float(np.linalg.norm(error))
        print(f'  ↳ XYZ réel {np.round(actual, 1).tolist()} mm | erreur '
              f'{np.round(error, 1).tolist()} mm ({norm:.1f} mm)')
        if norm > self.tolerance:
            raise MotionSafetyError(
                f'{context}: erreur XYZ {norm:.1f} mm > tolérance '
                f'{self.tolerance:.1f} mm — descente interdite')
        return actual

    def move_pose(self, target_xyz_mm, context='approche', verify=True):
        current = self._angles()
        solutions = solve_all_ik(target_xyz_mm, self.rotation, current, self.manual,
                                 self.model_bias)
        if not solutions:
            raise MotionSafetyError(f'{context}: aucune solution IK exacte et sûre')
        chosen = solutions[0]
        print(f'  → IK PC: {len(solutions)} branche(s), choix score={chosen.score:.1f}, '
              f'erreur théorique={chosen.xyz_error_mm:.2f} mm')
        print(f'  → send_angles {[round(v, 2) for v in chosen.angles_deg]} v={self.speed}')
        if self.dry_run:
            self._sim_q = np.asarray(chosen.angles_deg)
        else:
            response = self.bridge.send({'action': 'send_angles',
                                         'angles': list(chosen.angles_deg),
                                         'speed': self.speed})
            if not response.startswith('OK:'):
                raise MotionSafetyError(f'{context}: send_angles refusé: {response}')
            time.sleep(self.settle)
        self._expected_xyz = np.asarray(target_xyz_mm, dtype=np.float64)
        if verify:
            self.verify_xyz(target_xyz_mm, context)
        return chosen

    def descend_vertical(self, target_xyz_mm):
        """Descend en petits pas IK ; X/Y restent ceux de la cible à chaque pas."""
        target = np.asarray(target_xyz_mm, dtype=np.float64)
        if self._expected_xyz is None:
            raise MotionSafetyError('descente sans approche contrôlée préalable')
        # La pose atteinte avant descente doit déjà être correcte.
        actual = self.verify_xyz(self._expected_xyz, 'avant descente')
        if target[2] >= actual[2] - 1e-6:
            return self.move_pose(target, context='ajustement Z final')
        z_values = list(np.arange(actual[2] - self.step, target[2], -self.step))
        z_values.append(float(target[2]))
        for index, z in enumerate(z_values, 1):
            waypoint = np.array([target[0], target[1], max(z, target[2])])
            self.move_pose(waypoint, context=f'descente pas {index}/{len(z_values)}')
        return True
