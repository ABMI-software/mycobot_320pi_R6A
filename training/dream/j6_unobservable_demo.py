#!/usr/bin/env python3
"""Démonstration : pourquoi J6 est structurellement INOBSERVABLE par DREAM.

Symptôme observé sur le dashboard : la courbe DREAM de J6 est TOUJOURS collée
à l'encodeur (erreur 0.00° en permanence), quelle que soit la pose.

Raison : DREAM estime les angles en observant la position 2D de 7 keypoints
(base + link1..link6). Un joint n'est estimable QUE si le tourner déplace au
moins un keypoint dans l'image. Or tourner J6 ne déplace AUCUN keypoint.

Preuve mathématique (cinématique) :
  La position du dernier keypoint link6 vaut
      p_link6 = ( T_prev · T_fixe6 · Rz(q6) )[:3, 3]
  Rz(q6) est une rotation PURE (pas de translation), donc sa colonne de
  translation est [0,0,0,1]. Multiplier une matrice M par Rz(q6) ne change donc
  pas M[:3,3]. => p_link6 est INDÉPENDANT de q6.
  Et il n'existe aucun keypoint APRÈS link6 (pas de gripper/bride dans le schéma).
  => tourner J6 ne bouge aucun des 7 keypoints => zéro information dans l'image.

Ce script le vérifie numériquement : on fige une pose, on balaie J6 sur toute sa
plage, et on montre que les 7 keypoints (3D ET projetés en 2D) ne bougent pas
d'un iota — alors que le même balayage sur J5 les déplace nettement.
"""
import sys
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from mycobot_fk import (forward_kinematics, KEYPOINT_NAMES, get_camera_transform,
                        keypoints_in_camera_frame, project_keypoints, GAZEBO_INTRINSICS)


def kp_3d(q):
    pos, _ = forward_kinematics(q)
    return np.array([pos[n] for n in KEYPOINT_NAMES])       # (7,3)


def kp_2d(q):
    T_cam = get_camera_transform("front")
    cam = keypoints_in_camera_frame(q, T_cam)
    return np.array(project_keypoints(cam, GAZEBO_INTRINSICS))  # (7,2)


def sweep(joint_idx, base_q, angles_deg):
    """Déplacement 3D max (mm) et 2D max (px) de CHAQUE keypoint quand on
    balaie le joint joint_idx sur angles_deg, le reste de la pose figé."""
    ref3d = kp_3d(base_q)
    ref2d = kp_2d(base_q)
    max3d = np.zeros(len(KEYPOINT_NAMES))
    max2d = np.zeros(len(KEYPOINT_NAMES))
    for a in angles_deg:
        q = base_q.copy()
        q[joint_idx] = np.radians(a)
        d3 = np.linalg.norm(kp_3d(q) - ref3d, axis=1) * 1000.0     # mm
        d2 = np.linalg.norm(kp_2d(q) - ref2d, axis=1)              # px
        max3d = np.maximum(max3d, d3)
        max2d = np.maximum(max2d, d2)
    return max3d, max2d


def main():
    base_q = np.radians([15.0, -30.0, 40.0, -20.0, 25.0, 10.0])  # pose quelconque
    angles = np.linspace(-90, 90, 25)

    print(__doc__)
    print("=" * 74)
    for j in (4, 5):  # J5 (observable faible) puis J6 (inobservable)
        m3, m2 = sweep(j, base_q, angles)
        print(f"\n### On tourne J{j+1} de -90° à +90° — déplacement MAX de chaque keypoint :")
        print(f"{'keypoint':<12}{'Δ 3D (mm)':>12}{'Δ 2D (px)':>12}")
        for name, d3, d2 in zip(KEYPOINT_NAMES, m3, m2):
            short = name.replace("mycobot320_", "")
            flag = "  <-- BOUGE" if d2 > 0.5 else "  (immobile)"
            print(f"{short:<12}{d3:>12.4f}{d2:>12.4f}{flag}")
        tot2d = m2.max()
        print(f"  -> déplacement 2D max toutes cams confondues : {tot2d:.4f} px")
        if tot2d < 1e-6:
            print(f"  ==> J{j+1} ne bouge AUCUN keypoint => AUCUNE information image")
            print(f"      => DREAM ne peut PAS l'estimer => reste à l'amorce encodeur")
            print(f"      => erreur affichée = 0.00° en permanence (magenta collé au vert)")
        else:
            print(f"  ==> J{j+1} déplace des keypoints => observable (même faiblement)")

    print("\n" + "=" * 74)
    print("CONCLUSION : J6 n'est pas 'bloqué à 0' — DREAM n'a littéralement rien à")
    print("regarder pour lui. Ce n'est ni un bug ni un défaut de filtre/entraînement.")
    print("Seul un keypoint AJOUTÉ en aval de J6 (gripper/bride, excentré de l'axe)")
    print("le rendrait observable. Idem, dans une moindre mesure, pour J4/J5.")


if __name__ == "__main__":
    main()
