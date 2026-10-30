"""The four CSV of one sorting trial, enough to redraw the dashboard's curves.

Written by tri_dream_dashboard into the trial folder that tri_scene_panel opens
on Démarrer (<date>_tri_essai<k>_dream_yolo_gazebo):

  scene.csv        every piece and bin at the start: Gazebo centre, YOLO centre
                   and confidence, YOLO - Gazebo error in x, y, z; mean rows
  trajectoire.csv  every CURVE_PERIOD_S during an object's cycle: true gripper
                   (encoders), the target (object until held, then its bin) per
                   Gazebo and per YOLO; one row per fused DREAM tip, at its image
                   instant, with its error to the true gripper. The three curves
                   are dist_gazebo_mm, dist_yolo_mm and dist_dream_mm against temps_s
  resultat.csv     per object: sorted (Oui / Non), YOLO - Gazebo and DREAM errors
  temperature.csv  written by the panel (one row per second)

Positions in mm, in the robot base frame; Gazebo positions are object centres.
"""
import csv
from pathlib import Path

import numpy as np

from .vision import tri_scene

TRAJ_COLUMNS = ['temps_s', 'objet', 'cible',
                'pince_x_mm', 'pince_y_mm', 'pince_z_mm',
                'cible_gazebo_x_mm', 'cible_gazebo_y_mm', 'cible_gazebo_z_mm',
                'cible_yolo_x_mm', 'cible_yolo_y_mm', 'cible_yolo_z_mm',
                'dream_x_mm', 'dream_y_mm', 'dream_z_mm',
                'erreur_dream_x_mm', 'erreur_dream_y_mm', 'erreur_dream_z_mm',
                'dist_gazebo_mm', 'dist_yolo_mm', 'dist_dream_mm']


def _mm(p):
    return ['', '', ''] if p is None else [f'{v * 1000:.1f}' for v in p[:3]]


def _dist(a, b):
    return '' if a is None or b is None else f'{np.linalg.norm(np.asarray(a) - b) * 1000:.1f}'


def _write(path, header, rows):
    with open(path, 'w', newline='') as fh:
        w = csv.writer(fh)
        w.writerow(header)
        w.writerows(rows)


class EssaiLog:
    def __init__(self, folder, t0, first_grasp):
        self.dir = Path(folder)
        self.dir.mkdir(parents=True, exist_ok=True)
        self.t0 = t0
        self.first_grasp = first_grasp
        self.rows = []

    def write_scene(self, gt, yolo):
        """gt: name -> Gazebo centre (m); yolo: name -> (centre (m), confidence)."""
        rows, conf = [], {'objet': [], 'bac': []}
        for name in (*tri_scene.OBJECTS, *tri_scene.BINS):
            kind = 'objet' if name in tri_scene.OBJECTS else 'bac'
            g = gt.get(name)
            y, c = yolo.get(name, (None, None))
            err = (['', '', ''] if g is None or y is None
                   else [f'{v * 1000:+.1f}' for v in np.asarray(y) - g])
            if c is not None:
                conf[kind].append(c)
            rows.append([name, kind, *_mm(g), *_mm(y), '' if c is None else f'{c:.3f}', *err])
        for kind, label in (('objet', 'moyenne objets'), ('bac', 'moyenne bacs')):
            mean = f'{np.mean(conf[kind]):.3f}' if conf[kind] else ''
            rows.append([label, kind, '', '', '', '', '', '', mean, '', '', ''])
        _write(self.dir / 'scene.csv',
               ['element', 'type', 'gazebo_x_mm', 'gazebo_y_mm', 'gazebo_z_mm',
                'yolo_x_mm', 'yolo_y_mm', 'yolo_z_mm', 'yolo_confiance',
                'erreur_yolo_gazebo_x_mm', 'erreur_yolo_gazebo_y_mm', 'erreur_yolo_gazebo_z_mm'],
               rows)

    def sample(self, t, objet, cible, tip, gazebo, yolo):
        self.rows.append([t - self.t0, objet, cible, *_mm(tip), *_mm(gazebo), *_mm(yolo),
                          '', '', '', '', '', '', _dist(tip, gazebo), _dist(tip, yolo), ''])

    def dream(self, t, objet, cible, est, true_tip, gazebo):
        self.rows.append([t - self.t0, objet, cible, *_mm(true_tip), *_mm(gazebo), '', '', '',
                          *_mm(est), *[f'{v * 1000:+.1f}' for v in np.asarray(est) - true_tip],
                          '', '', _dist(est, gazebo)])

    def write_trajectoire(self):
        # DREAM rows arrive ~1 s after their image: sorted by time when written.
        rows = sorted(self.rows, key=lambda r: r[0])
        _write(self.dir / 'trajectoire.csv', TRAJ_COLUMNS,
               [[f'{r[0]:.3f}', *r[1:]] for r in rows])

    def write_resultat(self, grasps):
        rows = []
        for g in grasps:
            gt = g['gt']
            yolo = np.array([*g['yolo'], g['yolo_z']])

            def err(a, b):
                return (['', '', ''] if a is None or b is None
                        else [f'{v * 1000:+.1f}' for v in np.asarray(a) - b])
            rows.append([g['name'], tri_scene.PAIRS.get(g['name'], ''),
                         'Oui' if g['verdict'] == 'OK' else 'Non',
                         *err(yolo, gt), *err(g['dream'], gt), *err(g['dream'], g['encoder']),
                         g['views'], g['verdict_text']])
        _write(self.dir / 'resultat.csv',
               ['objet', 'bac', 'tri_ok',
                'erreur_yolo_gazebo_x_mm', 'erreur_yolo_gazebo_y_mm', 'erreur_yolo_gazebo_z_mm',
                'erreur_dream_gazebo_x_mm', 'erreur_dream_gazebo_y_mm', 'erreur_dream_gazebo_z_mm',
                'erreur_dream_pince_x_mm', 'erreur_dream_pince_y_mm', 'erreur_dream_pince_z_mm',
                'images_dream', 'detail'],
               rows)
