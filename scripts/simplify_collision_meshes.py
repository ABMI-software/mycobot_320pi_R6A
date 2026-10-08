#!/usr/bin/env python3
"""Maillages de collision allégés pour le robot Gazebo (mycobot_pro_320_pi_gazebo.urdf).

Les collisions reprenaient les DAE d'affichage, 52 000 à 243 000 triangles par
lien : ODE les teste à chaque pas de 1 ms et Gazebo tombait à 0,17x le temps
réel. Chaque DAE est remplacé, dans SON repère et SES unités (millimètres), par
son enveloppe extérieure : voxélisé et rempli au pas `--pitch-mm`, surface par
marching cubes, puis décimée. Écrit en STL dans `collision/` ; l'URDF les charge
avec `scale="0.001 0.001 0.001"` et la même `<origin>` que le DAE.

Ces exports CAO sont des soupes de centaines à milliers de fragments non fermés :
la décimation directe s'en écarte de 15 mm, l'enveloppe convexe double le volume
de link1. L'enveloppe voxélisée n'ajoute pas plus d'un pas de matière, et la
géométrie interne, inatteignable par un contact, disparaît. Deux écarts sont
imprimés et bornés par `--max-deviation-mm` : matière ajoutée par rapport au DAE,
et forme perdue par la décimation.

Usage (venv avec trimesh, pycollada, fast-simplification, rtree, scikit-image) :
    python3 scripts/simplify_collision_meshes.py
"""
import argparse
from pathlib import Path

import numpy as np
import trimesh

URDF_DIR = Path(__file__).resolve().parents[1] / 'mycobot_description' / 'urdf'
MESHES = [
    '320_pi/base.dae', '320_pi/link1.dae', '320_pi/link2.dae', '320_pi/link3.dae',
    '320_pi/link4.dae', '320_pi/link5.dae', '320_pi/link6_2022.dae',
    'pro_adaptive_gripper/gripper_base.dae',
    'pro_adaptive_gripper/gripper_left1.dae', 'pro_adaptive_gripper/gripper_left2.dae',
    'pro_adaptive_gripper/gripper_left3.dae', 'pro_adaptive_gripper/gripper_right1.dae',
    'pro_adaptive_gripper/gripper_right2.dae', 'pro_adaptive_gripper/gripper_right3.dae',
]
# Les doigts portent le contact de la saisie, avec 2,5 mm de jeu par côté : un pas
# de 1,5 mm leur ajouterait jusqu'à 3 mm d'épaisseur, d'où un pas plus fin.
FINGERS = {m for m in MESHES if 'gripper_left' in m or 'gripper_right' in m}
FINGER_PITCH_MM = 0.5


def p99_distance_mm(from_mesh, to_mesh, samples=20000):
    """99e centile de la distance de la surface `from_mesh` à `to_mesh` (mm)."""
    points = from_mesh.sample(samples)
    _, dist, _ = trimesh.proximity.closest_point(to_mesh, points)
    return float(np.percentile(dist, 99))


def outer_envelope(mesh, pitch_mm, target_faces):
    voxels = mesh.voxelized(pitch=pitch_mm).fill()
    surface = voxels.marching_cubes
    surface.apply_transform(voxels.transform)
    return surface, surface.simplify_quadric_decimation(face_count=target_faces)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--target-faces', type=int, default=3000)
    parser.add_argument('--pitch-mm', type=float, default=1.5)
    parser.add_argument('--max-deviation-mm', type=float, default=2.0)
    args = parser.parse_args()

    worst = 0.0
    for rel in MESHES:
        src = URDF_DIR / rel
        original = trimesh.load(src, force='mesh')
        pitch = FINGER_PITCH_MM if rel in FINGERS else args.pitch_mm
        envelope, simplified = outer_envelope(original, pitch, args.target_faces)
        added = p99_distance_mm(simplified, original)
        lost = max(p99_distance_mm(envelope, simplified), p99_distance_mm(simplified, envelope))
        worst = max(worst, added, lost)
        out = src.parent / 'collision' / (src.stem + '.stl')
        out.parent.mkdir(exist_ok=True)
        simplified.export(out)
        print(f'{rel:42s} {len(original.faces):7d} -> {len(simplified.faces):5d} faces  '
              f'pas {pitch} mm, ajouté p99 {added:.2f} mm, décimation p99 {lost:.2f} mm')
    if worst > args.max_deviation_mm:
        raise SystemExit(f'écart p99 {worst:.2f} mm > {args.max_deviation_mm} mm')


if __name__ == '__main__':
    main()
