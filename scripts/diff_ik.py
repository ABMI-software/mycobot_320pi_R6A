#!/usr/bin/env python3
"""IK différentielle sur la FK du dépôt — pilotage cartésien via send_angles.

Pourquoi ne pas utiliser send_coords : l'IK du firmware refuse ou ignore
silencieusement beaucoup de poses (observé le 17/08 — le bras ne bouge pas alors
que le bridge répond OK), et get_coords renvoie des angles d'Euler non fiables
bras relâché. Les ANGLES, eux, marchent parfaitement : c'est déjà la conclusion
de pick_and_place_real.py.

On résout donc nous-mêmes : jacobien numérique de la FK, puis moindres carrés
amortis (Levenberg) pour trouver le petit dq qui réalise le déplacement demandé
tout en TENANT l'orientation du poignet.
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'training' / 'dream'))
from mycobot_fk import forward_kinematics                              # noqa: E402


def fk_pose(q_deg):
    """(position bride en mm, rotation 3x3) pour des angles en degrés."""
    positions, transforms = forward_kinematics(np.radians(np.asarray(q_deg, float)))
    return np.asarray(positions['mycobot320_link6']) * 1000.0, transforms[6][:3, :3]


def _rot_vec(R):
    """Rotation 3x3 -> vecteur axe-angle (rad), petit-angle compatible."""
    angle = np.arccos(np.clip((np.trace(R) - 1.0) / 2.0, -1.0, 1.0))
    if angle < 1e-9:
        return np.zeros(3)
    axis = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    return axis / (2.0 * np.sin(angle)) * angle


def jacobian(q_deg, eps_deg=0.25):
    """Jacobien 6x6 : colonnes = effet d'un degré sur (position mm, rotation rad)."""
    p0, R0 = fk_pose(q_deg)
    columns = []
    for j in range(6):
        q = np.asarray(q_deg, float).copy()
        q[j] += eps_deg
        p1, R1 = fk_pose(q)
        columns.append(np.concatenate([(p1 - p0) / eps_deg,
                                       _rot_vec(R1 @ R0.T) / eps_deg]))
    return np.column_stack(columns)


# 1 rad d'erreur d'orientation « pèse » autant que ROT_WEIGHT mm d'erreur de
# position. Sans pondération l'orientation dérive librement (mesuré jusqu'à
# 63 deg) ; trop serrée, elle étouffe le déplacement — à 150 un jog de 25 mm
# n'en parcourait que 9. À 40 on obtient 24.6/25 mm pour 2.6 deg de dérive, que
# l'appelant réimpose ponctuellement (cf. ALIGN_WEIGHT) au moment qui compte.
ROT_WEIGHT = 40.0
# Pour un recalage d'orientation pur, à position tenue : là on veut l'inverse.
ALIGN_WEIGHT = 400.0

# --------------------------------------------------------------------------- #
#  Butées articulaires — source unique
# --------------------------------------------------------------------------- #
# Le dépôt portait onze déclarations de butées pour trois jeux de valeurs
# différents, jusqu'à 25,3° d'écart sur J2. Les trois sont nommés ici, avec
# leur provenance, pour que personne n'en retranscrive un douzième à la main.
# `tests/test_joint_limits_coherence.py` vérifie que les URDF et les copies
# Python restent d'accord avec eux.
#
#   J1      J2      J3      J4      J5      J6
#   169.6   159.9   159.9   159.9   169.6   174.8   URDF d'origine
#   167.9   134.6   145.0   145.0   167.9   179.9   URDF Gazebo / benchmark
#   168.0   135.0   150.0   145.0   165.0   180.0   domaine pratique

# `mycobot_pro_320_pi.urdf` et `new_mycobot_pro_320_pi_moveit.urdf`.
URDF_JOINT_LIMITS_DEG = np.degrees(np.array([
    (-2.96, 2.96), (-2.79, 2.79), (-2.79, 2.79),
    (-2.79, 2.79), (-2.96, 2.96), (-3.05, 3.05),
], dtype=np.float64))

# `mycobot_pro_320_pi_gazebo.urdf` et `..._benchmark.urdf` — c'est le modèle
# que chargent le jumeau Gazebo, MoveIt et les collectes synthétiques.
URDF_GAZEBO_JOINT_LIMITS_DEG = np.degrees(np.array([
    (-2.93, 2.93), (-2.35, 2.35), (-2.53, 2.53),
    (-2.53, 2.53), (-2.93, 2.93), (-3.14, 3.14),
], dtype=np.float64))

# CE QUI FAIT FOI POUR TOUT CE QUI COMMANDE LE ROBOT. C'est le seul des trois
# jeux adossé à une contrainte mesurée plutôt qu'héritée d'un fichier : le
# firmware refuse J2 hors de ±137°, donc ±135° conserve une marge réelle.
# Attention, il n'est pas uniformément plus étroit que les URDF — il est plus
# LARGE de 5° sur J3 que l'URDF Gazebo. Ne pas l'employer pour échantillonner
# des poses destinées à la simulation sans vérifier ce que le contrôleur en
# fait.
PRACTICAL_JOINT_LIMITS_DEG = np.array([
    (-168.0, 168.0), (-135.0, 135.0), (-150.0, 150.0),
    (-145.0, 145.0), (-165.0, 165.0), (-180.0, 180.0),
], dtype=np.float64)


def solve_pose(q_deg, p_target_mm, R_target, iterations=80, damping=1e-3,
               max_joint_step_deg=15.0, tol_mm=0.05, rot_weight=None,
               joint_limits_deg=PRACTICAL_JOINT_LIMITS_DEG):
    """Angles atteignant la pose (position + orientation) demandée.

    On vise une POSE, pas un déplacement : l'écart d'orientation est réinjecté à
    chaque itération, sinon le poignet dérive sans jamais être rappelé. C'est
    cette dérive qui avait ruiné le teach par traction (6 à 27 deg d'un point à
    l'autre, soit ~35 mm sur les doigts).
    """
    weight = ROT_WEIGHT if rot_weight is None else float(rot_weight)
    q = np.asarray(q_deg, dtype=np.float64).copy()
    limits = None if joint_limits_deg is None else np.asarray(joint_limits_deg, float)
    if limits is not None:
        q = np.clip(q, limits[:, 0], limits[:, 1])
    for _ in range(iterations):
        p, R = fk_pose(q)
        error = np.concatenate([np.asarray(p_target_mm, float) - p,
                                _rot_vec(np.asarray(R_target) @ R.T) * weight])
        if np.linalg.norm(error) < tol_mm:
            break
        J = jacobian(q)
        J = np.vstack([J[:3], J[3:] * weight])
        dq = np.linalg.solve(J.T @ J + damping * np.eye(6), J.T @ error)
        largest = np.max(np.abs(dq))
        if largest > max_joint_step_deg:
            dq *= max_joint_step_deg / largest
        q += dq
        if limits is not None:
            q = np.clip(q, limits[:, 0], limits[:, 1])
    return q


def solve_step(q_deg, delta_mm, **kwargs):
    """Angles déplaçant la bride de ``delta_mm``, orientation courante tenue."""
    p, R = fk_pose(q_deg)
    return solve_pose(q_deg, np.asarray(p) + np.asarray(delta_mm, float), R, **kwargs)


def realign(q_deg, R_target):
    """Restaure l'orientation cible en tenant la position — à appeler juste avant
    d'enregistrer un point, pour que TOUS les points partagent la même."""
    p, _ = fk_pose(q_deg)
    return solve_pose(q_deg, p, R_target, rot_weight=ALIGN_WEIGHT)


def orientation_error_deg(q_deg, R_target):
    _, R = fk_pose(q_deg)
    return float(np.degrees(np.linalg.norm(_rot_vec(np.asarray(R_target) @ R.T))))


def solve_tcp(poses):
    """Déport bride->doigts, par calibration TCP « pivot ».

    ``poses`` : liste de (position bride mm, rotation 3x3) prises en touchant UN
    MÊME point avec des orientations de poignet différentes. On cherche t (déport,
    repère bride) et c (le point touché) tels que p_i + R_i·t = c pour tout i.
    Linéaire en (t, c) : [R_i | -I] (t;c) = -p_i.

    Bien conditionné justement quand les orientations varient beaucoup — ce qui
    échouait au teach (43 deg d'écart entre points) est ici l'ingrédient utile.
    Retourne (t, point_touché, résidu_rms_mm, conditionnement).
    """
    if len(poses) < 3:
        raise ValueError('au moins 3 poses sont nécessaires')
    rows, rhs = [], []
    for p, R in poses:
        rows.append(np.hstack([np.asarray(R), -np.eye(3)]))
        rhs.append(-np.asarray(p, dtype=np.float64))
    A = np.vstack(rows)
    b = np.concatenate(rhs)
    solution, *_ = np.linalg.lstsq(A, b, rcond=None)
    residual = float(np.sqrt(np.mean((A @ solution - b) ** 2)))
    singular = np.linalg.svd(A, compute_uv=False)
    return solution[:3], solution[3:], residual, float(singular[0] / singular[-1])
