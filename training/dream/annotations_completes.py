#!/usr/bin/env python3
"""Annotations complètes d'une trame caméra libre, au-delà des 7 keypoints de DREAM.

Le jeu synthétique doit pouvoir entraîner d'autres méthodes d'estimation de
pose (RoboPEPP, HPE, CtRNet…) : pose caméra-robot, angles, toutes les
articulations, tous les liens en 3D et en 2D, masque, boîtes englobantes. Le
format est documenté dans docs/SYNTHETIC_DATA.md, « Format de sortie ».

Les champs lus par DREAM (`objects[0].keypoints[*].name/location/
projected_location`) ne changent pas : tout le reste s'ajoute à côté.

Conventions : mètres, radians sauf `*_deg`, pixels de l'image 640x480,
repère caméra optique OpenCV (x à droite, y en bas, z vers l'avant),
quaternions dans l'ordre x, y, z, w.
"""
import csv
import math
from pathlib import Path

import numpy as np
from PIL import Image
from scipy.ndimage import binary_dilation, label

from fk_urdf import ModeleURDF

LARGEUR, HAUTEUR = 640, 480
ARTICULATIONS_BRAS = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
                      'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
# Bout de chaque doigt dans le repère de son lien : sommet le plus éloigné de
# la bride des maillages gripper_left1.dae / gripper_right1.dae, calculé le
# 03/11 par fk_urdf sur l'URDF de collecte. Fixe dans le repère du doigt, il
# suit l'ouverture de la pince.
BOUTS_DOIGTS = {'gripper_left1': np.array([0.0029, 0.0311, -0.0067, 1.0]),
                'gripper_right1': np.array([0.0019, 0.0300, -0.0067, 1.0])}


# Rayon autour d'un keypoint projeté dans lequel une région du masque est
# reconnue comme appartenant au bras : un keypoint est au centre d'une
# articulation, donc sur le corps ou à quelques pixels de son bord.
RAYON_ACCROCHE_PX = 6


def masque_du_robot(masque, points_uv):
    """Garde les régions du masque touchées par un keypoint du robot.

    Le label de segmentation est posé sur tout le modèle, et les 4 caméras
    fixes de l'URDF en sont des liens : sans ce tri, elles apparaissent comme
    des taches isolées dans le masque, gonflent sa boîte englobante (mesuré :
    [0, 13, 460, 382] pour un bras de 200 px) et restent collées sur les fonds
    composés. Le bras et la pince forment une région touchée par au moins un
    des 7 keypoints.
    """
    regions, n = label(masque)
    if n == 0:
        return masque
    accroches = np.zeros_like(masque, dtype=bool)
    for u, v in points_uv:
        if 0 <= u < masque.shape[1] and 0 <= v < masque.shape[0]:
            accroches[int(v), int(u)] = True
    accroches = binary_dilation(accroches, iterations=RAYON_ACCROCHE_PX)
    gardees = np.unique(regions[accroches & (regions > 0)])
    return np.isin(regions, gardees)


def quaternion_xyzw(rotation):
    m = rotation
    trace = m[0, 0] + m[1, 1] + m[2, 2]
    if trace > 0:
        s = 2.0 * math.sqrt(trace + 1.0)
        q = [(m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s, 0.25 * s]
    elif m[0, 0] > m[1, 1] and m[0, 0] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[0, 0] - m[1, 1] - m[2, 2])
        q = [0.25 * s, (m[0, 1] + m[1, 0]) / s, (m[0, 2] + m[2, 0]) / s, (m[2, 1] - m[1, 2]) / s]
    elif m[1, 1] > m[2, 2]:
        s = 2.0 * math.sqrt(1.0 + m[1, 1] - m[0, 0] - m[2, 2])
        q = [(m[0, 1] + m[1, 0]) / s, 0.25 * s, (m[1, 2] + m[2, 1]) / s, (m[0, 2] - m[2, 0]) / s]
    else:
        s = 2.0 * math.sqrt(1.0 + m[2, 2] - m[0, 0] - m[1, 1])
        q = [(m[0, 2] + m[2, 0]) / s, (m[1, 2] + m[2, 1]) / s, 0.25 * s, (m[1, 0] - m[0, 1]) / s]
    return [float(v) for v in q]


def _projection(point_cam, K):
    if point_cam[2] <= 0:
        return None
    return [float(K[0, 0] * point_cam[0] / point_cam[2] + K[0, 2]),
            float(K[1, 1] * point_cam[1] / point_cam[2] + K[1, 2])]


def _dans_image(uv):
    return uv is not None and 0 <= uv[0] < LARGEUR and 0 <= uv[1] < HAUTEUR


def _point(point_monde, T_cam_monde, K):
    cam = (T_cam_monde @ np.append(point_monde, 1.0))[:3]
    uv = _projection(cam, K)
    return {'location': cam.tolist(), 'location_monde': list(map(float, point_monde)),
            'projected_location': uv, 'profondeur_m': float(cam[2]), 'dans_image': _dans_image(uv)}


def _boite(us, vs):
    if len(us) == 0:
        return None
    return [float(min(us)), float(min(vs)), float(max(us)), float(max(vs))]


class SourceCollecte:
    """Fichiers compagnons d'une collecte ; chacun est facultatif."""

    def __init__(self, dossier):
        self.dossier = Path(dossier)
        urdf = self.dossier / 'robot.urdf'
        self.modele = ModeleURDF(urdf.read_text()) if urdf.exists() else None
        self.joints = self._csv_par_index('joints_complets.csv')
        self.masques = self._csv_par_index('masques.csv')
        self.composition = self._csv_par_index('composition.csv')
        chemin = self.dossier / 'distracteurs.csv'
        self.distracteurs = list(csv.DictReader(open(chemin))) if chemin.exists() else None

    def distracteurs_de(self, index):
        """Le lot en place a la trame `index` : le dernier pose avant elle."""
        debuts = [int(r['index_debut']) for r in self.distracteurs if int(r['index_debut']) <= index]
        if not debuts:
            return []
        return [r for r in self.distracteurs if int(r['index_debut']) == max(debuts)]

    def _csv_par_index(self, nom):
        chemin = self.dossier / nom
        if not chemin.exists():
            return {}
        return {int(r['index']): r for r in csv.DictReader(open(chemin))}

    def chemin_masque(self, index):
        ligne = self.masques.get(index)
        chemin = self.dossier / 'images' / 'masque' / f'{index:06d}.png'
        if ligne is None or ligne['frais'] != '1' or not chemin.exists():
            return None
        return chemin


def enrichir(data, source, index, T_monde_cam, joints_bras, K, pose_camera_xyz_rpy, visee_xy):
    """Ajoute à `data` (JSON NDDS de DREAM) tout ce que la collecte permet."""
    T_cam_monde = np.linalg.inv(T_monde_cam)
    objet = data['objects'][0]
    joints_ligne = source.joints.get(index)
    positions = {n: float(v) for n, v in joints_ligne.items()
                 if n not in ('index', 't_image_s', 't_joints_s')} if joints_ligne else {}
    for nom, valeur in zip(ARTICULATIONS_BRAS, joints_bras):
        positions.setdefault(nom, float(valeur))

    objet['joint_names'] = ARTICULATIONS_BRAS
    objet['joint_angles_rad'] = [float(v) for v in joints_bras]
    objet['joint_angles_deg'] = [math.degrees(v) for v in joints_bras]
    if joints_ligne:
        objet['joints_complets_rad'] = {n: v for n, v in positions.items()}

    masque_chemin = source.chemin_masque(index)
    masque = (masque_du_robot(np.asarray(Image.open(masque_chemin)) > 0,
                              [kp['projected_location'] for kp in objet['keypoints']])
              if masque_chemin else None)
    for kp in objet['keypoints']:
        cam = np.asarray(kp['location'])
        uv = kp['projected_location']
        kp['location_monde'] = (T_monde_cam @ np.append(cam, 1.0))[:3].tolist()
        kp['profondeur_m'] = float(cam[2])
        kp['dans_image'] = _dans_image(uv)
        kp['masque_au_point'] = (bool(masque[int(uv[1]), int(uv[0])])
                                 if masque is not None and kp['dans_image'] else None)

    if source.modele is not None:
        poses = source.modele.poses(positions)
        T_cam_base = T_cam_monde @ poses['base']
        objet['location'] = T_cam_base[:3, 3].tolist()
        objet['quaternion_xyzw'] = quaternion_xyzw(T_cam_base[:3, :3])
        objet['pose_transform'] = T_cam_base.tolist()
        objet['liens'] = []
        for nom, T in poses.items():
            if nom == source.modele.racine:
                continue
            lien = {'name': nom, **_point(T[:3, 3], T_cam_monde, K)}
            lien['quaternion_xyzw'] = quaternion_xyzw((T_cam_monde @ T)[:3, :3])
            objet['liens'].append(lien)
        bouts = {nom: (poses[nom] @ local)[:3] for nom, local in BOUTS_DOIGTS.items()}
        objet['bouts_doigts'] = {nom: _point(p, T_cam_monde, K) for nom, p in bouts.items()}
        objet['pointe_pince'] = _point(np.mean(list(bouts.values()), axis=0), T_cam_monde, K)

    visibles = [kp['projected_location'] for kp in objet['keypoints'] if kp['dans_image']]
    objet['bbox_keypoints'] = _boite([p[0] for p in visibles], [p[1] for p in visibles])
    if masque is not None:
        vs, us = np.nonzero(masque)
        objet['bbox_masque'] = _boite(us, vs)
        objet['part_robot'] = float(masque.mean())

    xyz, rpy = pose_camera_xyz_rpy
    dx, dy = xyz[0] - visee_xy[0], xyz[1] - visee_xy[1]
    data['camera_data'] = {
        'width': LARGEUR, 'height': HAUTEUR,
        'intrinsics': {'fx': float(K[0, 0]), 'fy': float(K[1, 1]), 'cx': float(K[0, 2]), 'cy': float(K[1, 2])},
        'location_monde': list(map(float, xyz)),
        'quaternion_xyzw_monde': quaternion_xyzw(T_monde_cam[:3, :3]),
        'pose_monde': T_monde_cam.tolist(),
        'rpy_lien_monde': list(map(float, rpy)),
        'azimut_deg': math.degrees(math.atan2(dy, dx)),
        'distance_visee_xy_m': math.hypot(dx, dy),
        'hauteur_m': float(xyz[2]),
        'repere': 'optique OpenCV : x droite, y bas, z avant ; pose_monde = monde <- camera',
    }
    if source.distracteurs is not None:
        data['distracteurs'] = [
            {'name': r['nom'], 'forme': r['forme'],
             'dimensions_m': [float(r[c]) for c in ('dim_x', 'dim_y', 'dim_z')],
             'couleur_rgb': [float(r[c]) for c in ('r', 'g', 'b')],
             'rpy_monde': [float(r[c]) for c in ('roulis', 'tangage', 'lacet')],
             **_point(np.array([float(r[c]) for c in ('x', 'y', 'z')]), T_cam_monde, K)}
            for r in source.distracteurs_de(index)]
    composition = source.composition.get(index, {})
    masque_ligne = source.masques.get(index, {})
    data['source'] = {
        'collecte': str(source.dossier), 'index': index,
        't_image_s': float(joints_ligne['t_image_s']) if joints_ligne else None,
        't_joints_s': float(joints_ligne['t_joints_s']) if joints_ligne else None,
        'masque_frais': masque is not None,
        'ecart_masque_s': float(masque_ligne['ecart_s']) if masque_ligne.get('ecart_s') else None,
        'fond_compose': composition.get('composee') == '1',
        'fond': composition.get('fond') or None,
    }
    return masque
