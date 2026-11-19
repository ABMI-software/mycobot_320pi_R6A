#!/usr/bin/env python3
"""Pointe d'un outil (stylo) par pivot, puis pointage de points au robot.

La pointe reste dans un creux fixe (pointeau, cone perce) pendant que la bride
prend 6 a 10 orientations differentes. Pour chaque pose i, bride f_i et
rotation R_i (FK des codeurs, `diff_ik.fk_pose`, la convention de
`tool_offset.json`) :

    f_i + R_i @ t = p        t : pointe dans le repere bride, p : le creux

Moindres carres sur (t, p). Le residu par pose et le leave-one-out sur t
disent si le pivot est bon ; un residu faible sur des orientations trop
proches ne prouve rien, d'ou l'exigence d'ecart angulaire.

    # pivot : relacher, placer a la main, verrouiller, mesurer, 6 fois et plus
    MYCOBOT_PI=<ip> /usr/bin/python3 scripts/pivot_pointe.py pivot \
        --sortie-outil scripts/tool_offset_stylo.json
    # pointer des points (marqueurs 19 et 23, bloc en hauteur)
    MYCOBOT_PI=<ip> /usr/bin/python3 scripts/pivot_pointe.py pointe \
        --outil scripts/tool_offset_stylo.json
    # recalculer un pivot depuis ses angles bruts
    /usr/bin/python3 scripts/pivot_pointe.py rejouer <dossier>/pivot.csv

Seuls `power_off` (relache tous les servos : TENIR LE BRAS) et `power_on` sont
envoyes au robot, jamais un ordre de mouvement. Le pont est
`gripper_bridge.py`, mono-client : aucun autre client ni script d'essai sur
la Pi en meme temps.
"""
from __future__ import annotations

import argparse
import csv
import json
import os
import socket
import sys
import time
from datetime import datetime
from pathlib import Path

import numpy as np

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE / 'scripts'))
from diff_ik import fk_pose                                              # noqa: E402

PORT_PI = 5005
LECTURES = 5
POSES_MIN = 6
ECART_AXES_MIN_DEG = 30.0
# Simulation (8 poses a +-20 deg, pas de codeur 0,09 deg + bruit 0,05 deg) :
# residu RMS median 0,49 mm, p95 0,66 ; residu max p95 1,22 ; erreur sur t
# mediane 0,54 mm, p90 0,97. Les seuils sont au-dessus du bruit de codeur seul.
RESIDU_RMS_MAX_MM = 0.8
RESIDU_MAX_MM = 1.5
LIMITES_DEG = (168, 135, 145, 148, 168, 180)


class Pont:
    def __init__(self, hote, timeout=8.0):
        self.sock = socket.create_connection((hote, PORT_PI), timeout=timeout)
        self.buf = b''

    def envoie(self, action):
        self.buf = b''
        self.sock.sendall((json.dumps({'action': action}) + '\n').encode())
        while b'\n' not in self.buf:
            bloc = self.sock.recv(4096)
            if not bloc:
                raise ConnectionError('pont ferme')
            self.buf += bloc
        return self.buf.split(b'\n', 1)[0].decode().strip()

    def angles(self):
        """Mediane de LECTURES lectures ; refuse une reponse hors domaine."""
        lues = []
        for _ in range(LECTURES * 2):
            rep = self.envoie('get_angles')
            if 'ANGLES' in rep.upper():
                q = np.asarray(json.loads(rep.split(':', 1)[-1].strip()), float)
                if q.size == 6 and np.all(np.abs(q) <= 200.0):
                    lues.append(q)
            if len(lues) == LECTURES:
                break
            time.sleep(0.1)
        if len(lues) < LECTURES:
            raise RuntimeError(f'get_angles illisible : {rep!r}')
        lues = np.array(lues)
        return np.median(lues, axis=0), float(np.ptp(lues, axis=0).max())


def resout_pivot(angles):
    """(t, p, residus) depuis une liste d'angles en degres."""
    A, b = [], []
    for q in angles:
        f, R = fk_pose(q)
        A.append(np.hstack([R, -np.eye(3)]))
        b.append(-f)
    x, *_ = np.linalg.lstsq(np.vstack(A), np.concatenate(b), rcond=None)
    t, p = x[:3], x[3:]
    residus = np.array([np.linalg.norm(fk_pose(q)[0] + fk_pose(q)[1] @ t - p) for q in angles])
    return t, p, residus


def ecart_axes_deg(angles):
    """Plus grand angle entre deux axes Z de bride parmi les poses."""
    axes = [fk_pose(q)[1][:, 2] for q in angles]
    return max(np.degrees(np.arccos(np.clip(a @ b, -1, 1))) for a in axes for b in axes)


def bilan_pivot(angles):
    t, p, residus = resout_pivot(angles)
    loo = []
    for k in range(len(angles)):
        tk, _, _ = resout_pivot([q for i, q in enumerate(angles) if i != k])
        loo.append(float(np.linalg.norm(tk - t)))
    rms = float(np.sqrt((residus ** 2).mean()))
    return {
        'poses': len(angles),
        'ecart_axes_max_deg': round(ecart_axes_deg(angles), 1),
        'tool_offset_mm': [round(float(v), 2) for v in t],
        'pivot_base_mm': [round(float(v), 2) for v in p],
        'residu_par_pose_mm': [round(float(r), 3) for r in residus],
        'residu_rms_mm': round(rms, 3),
        'residu_max_mm': round(float(residus.max()), 3),
        'leave_one_out_t_mm': [round(v, 3) for v in loo],
        'conforme': bool(len(angles) >= POSES_MIN
                         and ecart_axes_deg(angles) >= ECART_AXES_MIN_DEG
                         and rms <= RESIDU_RMS_MAX_MM and residus.max() <= RESIDU_MAX_MM),
    }


def affiche(b):
    print(f"\n{b['poses']} poses, axes ecartes jusqu'a {b['ecart_axes_max_deg']} deg")
    print(f"pointe dans la bride t = {b['tool_offset_mm']} mm")
    print(f"creux dans la base    p = {b['pivot_base_mm']} mm")
    print(f"residus {b['residu_par_pose_mm']} mm — RMS {b['residu_rms_mm']}, max {b['residu_max_mm']}")
    print(f"leave-one-out sur t   {b['leave_one_out_t_mm']} mm")
    print(f"critere (>= {POSES_MIN} poses, axes >= {ECART_AXES_MIN_DEG:g} deg, RMS <= "
          f"{RESIDU_RMS_MAX_MM} mm, max <= {RESIDU_MAX_MM} mm) : "
          f"{'CONFORME' if b['conforme'] else 'NON CONFORME'}")


def dossier_sortie(mode):
    d = RACINE / 'results' / 'pivot' / f'{datetime.now():%Y%m%d_%H%M%S}_{mode}'
    d.mkdir(parents=True, exist_ok=True)
    return d


def hors_limites(q):
    return [j + 1 for j in range(6) if abs(q[j]) > LIMITES_DEG[j] - 2]


def mesure(pont):
    q, etendue = pont.angles()
    if hors_limites(q):
        print(f'  ATTENTION joints {hors_limites(q)} a moins de 2 deg de leur limite '
              f'(erreur 5 du controleur au prochain mouvement)')
    return q, etendue


def mode_pivot(args):
    pont = Pont(os.environ['MYCOBOT_PI'])
    d = dossier_sortie('pivot')
    fichier = (d / 'pivot.csv').open('w', newline='')
    ecrit = csv.writer(fichier)
    ecrit.writerow(['pose', 'heure'] + [f'j{j}_deg' for j in range(1, 7)] + ['etendue_lectures_deg'])
    angles = []
    print(__doc__.split('\n\n')[1])
    print('\nr = relacher (TENIR LE BRAS)   v = verrouiller   m = mesurer   '
          'x = retirer la derniere   f = finir')
    while True:
        c = input(f'[{len(angles)} poses] > ').strip().lower()
        if c == 'r':
            if input('  tu tiens le bras ? (o/n) ').strip().lower() == 'o':
                print(' ', pont.envoie('power_off'))
        elif c == 'v':
            print(' ', pont.envoie('power_on'))
        elif c == 'm':
            if input('  verrouille, pointe toujours dans le creux ? (o/n) ').strip().lower() != 'o':
                continue
            q, etendue = mesure(pont)
            angles.append(q)
            ecrit.writerow([len(angles), f'{datetime.now():%H:%M:%S}'] + [f'{v:.2f}' for v in q]
                           + [f'{etendue:.2f}'])
            fichier.flush()
            print(f'  angles {np.round(q, 2).tolist()} (etendue des lectures {etendue:.2f} deg)')
            if len(angles) >= 3:
                b = bilan_pivot(angles)
                print(f"  provisoire : t = {b['tool_offset_mm']}, RMS {b['residu_rms_mm']} mm, "
                      f"axes {b['ecart_axes_max_deg']} deg")
        elif c == 'x' and angles:
            angles.pop()
            ecrit.writerow(['retiree', f'{datetime.now():%H:%M:%S}'] + [''] * 7)
            print('  derniere pose retiree (consignee dans le CSV)')
        elif c == 'f':
            break
    fichier.close()
    if len(angles) < 3:
        raise SystemExit('moins de 3 poses : rien a calculer')
    b = bilan_pivot(angles)
    affiche(b)
    (d / 'bilan.json').write_text(json.dumps(b, indent=2))
    print(f'\nbrut et bilan -> {d}')
    if args.sortie_outil and b['conforme']:
        Path(args.sortie_outil).write_text(json.dumps({
            'tool_offset_mm': b['tool_offset_mm'],
            'convention': 'pointe = flange_mm + R_bride @ tool_offset_mm, R rendue par diff_ik.fk_pose',
            'designe': 'la pointe du stylo, ressort au contact leger',
            'date': f'{datetime.now():%Y-%m-%d %H:%M}',
            'methode': f'pivot, {b["poses"]} poses, axes ecartes de {b["ecart_axes_max_deg"]} deg, '
                       f'residu RMS {b["residu_rms_mm"]} mm, max {b["residu_max_mm"]} mm',
            'donnees': str(d),
        }, indent=2, ensure_ascii=False))
        print(f'outil ecrit -> {args.sortie_outil}')
    elif args.sortie_outil:
        print(f'NON CONFORME : {args.sortie_outil} n est pas ecrit')


def mode_pointe(args):
    t = np.array(json.loads(Path(args.outil).read_text())['tool_offset_mm'], float)
    pont = Pont(os.environ['MYCOBOT_PI'])
    d = dossier_sortie('pointe')
    fichier = (d / 'points.csv').open('w', newline='')
    ecrit = csv.writer(fichier)
    ecrit.writerow(['nom', 'heure', 'x_mm', 'y_mm', 'z_mm'] + [f'j{j}_deg' for j in range(1, 7)]
                   + ['etendue_lectures_deg', 'outil'])
    print('nom du point puis Entree pour mesurer (ex. 19, 23, bloc_100) ; r / v pour '
          'relacher / verrouiller ; vide pour finir')
    while True:
        nom = input('point > ').strip()
        if not nom:
            break
        if nom == 'r':
            if input('  tu tiens le bras ? (o/n) ').strip().lower() == 'o':
                print(' ', pont.envoie('power_off'))
            continue
        if nom == 'v':
            print(' ', pont.envoie('power_on'))
            continue
        q, etendue = mesure(pont)
        f, R = fk_pose(q)
        x = f + R @ t
        ecrit.writerow([nom, f'{datetime.now():%H:%M:%S}'] + [f'{v:.2f}' for v in x]
                       + [f'{v:.2f}' for v in q] + [f'{etendue:.2f}', args.outil])
        fichier.flush()
        print(f'  {nom} : ({x[0]:.1f} ; {x[1]:.1f} ; {x[2]:.1f}) mm')
    fichier.close()
    print(f'brut -> {d}')


def mode_rejouer(args):
    angles = []
    for l in csv.DictReader(open(args.csv)):
        if l['pose'] == 'retiree':
            angles.pop()
        else:
            angles.append(np.array([float(l[f'j{j}_deg']) for j in range(1, 7)]))
    affiche(bilan_pivot(angles))


def main():
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    s = a.add_subparsers(dest='mode', required=True)
    p = s.add_parser('pivot')
    p.add_argument('--sortie-outil', help='JSON outil ecrit si le pivot est conforme')
    p = s.add_parser('pointe')
    p.add_argument('--outil', required=True, help='JSON de pivot (tool_offset_mm)')
    p = s.add_parser('rejouer')
    p.add_argument('csv')
    args = a.parse_args()
    {'pivot': mode_pivot, 'pointe': mode_pointe, 'rejouer': mode_rejouer}[args.mode](args)


if __name__ == '__main__':
    main()
