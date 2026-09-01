#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Capture d'un jeu reel POUR CE MONTAGE, par trajectoires a petits pas.

Pourquoi refaire un jeu. Le reseau actuel (`vgg_ultimate_v4_mix_ft_e30`) a ete
affine sur `real_3cam`, ou l'arducam etait sur un AUTRE montage. Sur le montage
d'aujourd'hui il laisse ~40 mm, et aucune transformation 2D ne le corrige
(mesure du 01/09 : translation 52 mm, similitude 42 mm, affine complete 36 mm).

Deux choses apprises en comparant avec `panda-3cam_azure` de NVlabs, et qui
dictent la methode ci-dessous :

1. **Le cadrage d'entrainement doit etre celui de l'inference.** Verifie par
   l'absurde : capturer en 1600x1200 et recadrer autour du robot donne une image
   plus grande et plus nette (le robot passe de 29 % a 53 % du cadre, comme chez
   NVlabs) et l'erreur DOUBLE, 61 -> 143 px. Le reseau est verrouille sur le
   cadrage qu'il a vu. On capture donc en 640x480 direct, arducam a sa place,
   exposition 75 : exactement ce que verra le pick.

2. **Ils filment des trajectoires, pas des poses isolees.** Chez NVlabs l'ecart
   entre images consecutives vaut 0,23 deg (mediane) et le robot BOUGE pendant
   la capture (10 deg/s, 1 image sur 170 a l'arret) : 6394 images en 10
   trajectoires continues. Dans `real_3cam` l'ecart median est de 94,3 deg —
   410 fois plus — et aucune pose n'est a moins de 1 deg de la precedente.

Mais on ne filme PAS en mouvement : NVlabs enregistre l'etat articulaire
synchronise a la trame, alors qu'ici les angles arrivent par requete-reponse
TCP. A 10 deg/s, 100 ms de latence font 1 deg d'erreur d'ETIQUETTE. On garde
donc l'arret a chaque prise — les etiquettes restent exactes — mais avec des
pas petits, pour retrouver la densite de trajectoire.

Une pose dont les keypoints tombent hors de la fenetre reseau (`shrink-and-crop`
640x480 -> 400x400 ne garde que x dans [80, 560]) n'apprend rien : elle est
ecartee AVANT d'etre jouee.

Usage (venv_dream) :
    # ce que ca donnerait, SANS bouger le bras
    python scripts/capture_trajectoires.py --simuler

    # capture reelle — LE BRAS BOUGE LONGTEMPS
    python scripts/capture_trajectoires.py --nom real_montage_0901
    # reprise apres interruption : relancer la meme commande
"""
import argparse
import csv
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

RACINE = Path(__file__).resolve().parents[1]
CALIB = RACINE / 'training' / 'calibration'
DONNEES = RACINE / 'training' / 'dream' / 'dream_data'

sys.path.insert(0, str(RACINE / 'training' / 'dream'))
sys.path.insert(0, str(RACINE / 'mycobot_gateway' / 'mycobot_gateway' / 'vision'))

from mycobot_fk import forward_kinematics, KEYPOINT_NAMES     # noqa: E402
import camera_registry as registre                            # noqa: E402

# Fenetre reellement vue par le reseau apres `shrink-and-crop`.
FENETRE_X, MARGE_PX = (80, 560), 30
PAS_DEG = 2.5           # ecart vise entre deux images consecutives
PAS_MAX_APPROCHE = 30.0  # pour REJOINDRE un depart de trajectoire, on decoupe
TOL_IMMOBILE = 0.35     # deg : deux lectures sous ce seuil = bras arrete

CAMERAS = [
    {'nom': 'arducam', 'v4l2': 'Arducam', 'exposition': 75,
     'extr': 'arducam_extrinsic_pick'},
    {'nom': 'svpro', 'v4l2': '5MP', 'exposition': None,
     'extr': 'svpro_extrinsic_servo'},
]

# Les trajectoires ne sont pas ecrites a la main : on balaie. Autour de
# plusieurs configurations de base (allonges et hauteurs differentes), chaque
# trajectoire fait varier UN ou DEUX joints d'un bout a l'autre de leur plage.
# On obtient une couverture dense et continue, comme les 10 trajectoires de
# NVlabs, au lieu de sauts de 94 deg entre poses independantes.
BASES = [
    [30, -118.7, 82.8, -102.6, -17.8, 49.8],
    [30, -128.7, 76.5, -53.4, -3.7, 21.8],
    [30, -120.0, 90.0, -60.5, 10.3, 6.2],
    [30, -110.0, 70.0, -80.0, -10.0, 30.0],
]
# (joint, min, max) — plages tenant le bras sur la planche et dans la fenetre
# reseau. Les poses qui en sortent quand meme sont ecartees par `visible`.
BALAYAGES = [
    (0, 8, 58),        # J1 : azimut, conserve l'inclinaison de l'outil
    (1, -134, -108),   # J2 : hauteur
    (2, 68, 96),       # J3 : allonge
    (3, -104, -50),    # J4 : poignet
    (4, -42, 22),      # J5 : inclinaison outil
    (5, -10, 92),      # J6 : rotation outil, invisible en FK mais vue en image
]


def construit_trajectoires():
    """Un aller simple par (base, joint balaye), a plusieurs azimuts."""
    chaines = []
    for base in BASES:
        for azimut in (12, 30, 48):
            for joint, lo, hi in BALAYAGES:
                if joint == 0:
                    a = list(base); a[0] = lo
                    b = list(base); b[0] = hi
                else:
                    a = list(base); a[0] = azimut; a[joint] = lo
                    b = list(base); b[0] = azimut; b[joint] = hi
                chaines.append([a, b])
    return chaines


TRAJECTOIRES = construit_trajectoires()

def echantillonne(sommets, pas=PAS_DEG):
    """Poses le long des segments, espacees d'au plus `pas` sur le joint le
    plus rapide. C'est ce qui remplace la densite de la video de NVlabs."""
    poses = []
    for a, b in zip(sommets[:-1], sommets[1:]):
        a, b = np.array(a, float), np.array(b, float)
        n = max(1, int(np.ceil(np.max(np.abs(b - a)) / pas)))
        for i in range(n + 1):
            q = a + (b - a) * (i / n)
            if not poses or np.max(np.abs(q - poses[-1])) > 1e-6:
                poses.append(q)
    return poses


def modele(cfg):
    d = yaml.safe_load((CALIB / f"{cfg['extr']}.yaml").read_text())
    T = np.array(d['T_cam_world'], float)
    K, dist = registre.load_intrinsics(d['intrinsics_stem'])
    return cv2.Rodrigues(T[:3, :3])[0], T[:3, 3], K, dist


def visible(q, mdl):
    """Les 7 keypoints tombent-ils dans la fenetre reseau, avec de la marge ?"""
    pos, _ = forward_kinematics(np.radians(q))
    obj = np.array([pos[n] for n in KEYPOINT_NAMES], float)
    p = cv2.projectPoints(obj, *mdl)[0].reshape(-1, 2)
    x, y = p[:, 0], p[:, 1]
    return bool(np.all((FENETRE_X[0] + MARGE_PX <= x) & (x < FENETRE_X[1] - MARGE_PX))
                and np.all((0 <= y) & (y < 480)))


def index_v4l2(motif):
    sortie = subprocess.run(['v4l2-ctl', '--list-devices'],
                            capture_output=True, text=True).stdout
    for bloc in sortie.split('\n\n'):
        if motif.lower() in bloc.split('\n')[0].lower():
            for ligne in bloc.split('\n')[1:]:
                ligne = ligne.strip()
                if ligne.startswith('/dev/video'):
                    return int(ligne.removeprefix('/dev/video'))
    raise SystemExit(f'camera introuvable : {motif}')


class Camera:
    """640x480, exposition reimposee : l'arducam la perd au rebranchement."""

    def __init__(self, cfg):
        self.nom = cfg['nom']
        self.index = index_v4l2(cfg['v4l2'])
        self.exposition = cfg['exposition']
        self.cap = cv2.VideoCapture(self.index, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        if not self.cap.isOpened():
            raise SystemExit(f'{self.nom} : /dev/video{self.index} inaccessible')
        self._expose()

    def _expose(self):
        if self.exposition is not None:
            subprocess.run(['v4l2-ctl', '-d', f'/dev/video{self.index}',
                            '-c', 'auto_exposure=1',
                            '-c', f'exposure_time_absolute={self.exposition}'],
                           capture_output=True)

    def lit(self):
        # Vider le tampon V4L2 : sinon on enregistre une trame ANTERIEURE, prise
        # pendant le deplacement, donc floue — 7/7 detectes tombait a 2/7.
        for _ in range(6):
            self.cap.read()
        ok, img = self.cap.read()
        return img if ok else None

    def ferme(self):
        self.cap.release()


def immobile(pont, timeout=8.0):
    """Attend deux lectures consecutives sous TOL_IMMOBILE et rend les angles
    MESURES — jamais la consigne : c'est l'etiquette du jeu de donnees."""
    t0, precedent = time.time(), None
    while time.time() - t0 < timeout:
        q = np.array(pont.get_angles(), float)
        if precedent is not None and np.max(np.abs(q - precedent)) < TOL_IMMOBILE:
            return q
        precedent = q
        time.sleep(0.35)
    return np.array(pont.get_angles(), float)


def rejoint(pont, cible, vitesse=25):
    """Rejoint une pose eloignee par paliers — un ordre unique depuis loin fait
    partir toutes les articulations a fond en meme temps."""
    depart = np.array(pont.get_angles(), float)
    cible = np.array(cible, float)
    n = max(1, int(np.ceil(np.max(np.abs(cible - depart)) / PAS_MAX_APPROCHE)))
    for i in range(1, n + 1):
        etape = depart + (cible - depart) * (i / n)
        pont.send({'action': 'send_angles',
                   'angles': [round(float(v), 2) for v in etape], 'speed': vitesse})
        time.sleep(1.6)
    return immobile(pont)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--nom', default='real_montage', help='dossier sous dream_data/')
    ap.add_argument('--pas', type=float, default=PAS_DEG)
    ap.add_argument('--simuler', action='store_true',
                    help='compter les poses retenues sans bouger le bras')
    ap.add_argument('--host', default='10.10.0.224')
    ap.add_argument('--vitesse', type=int, default=25)
    args = ap.parse_args()

    mdl = modele(CAMERAS[0])          # visibilite jugee sur l'arducam
    plan, aveugles = [], 0
    for t, sommets in enumerate(TRAJECTOIRES):
        for q in echantillonne(sommets, args.pas):
            if visible(q, mdl):
                plan.append((t, q))
            else:
                aveugles += 1

    print(f'{len(TRAJECTOIRES)} trajectoires, pas {args.pas} deg')
    print(f'{len(plan)} poses retenues, {aveugles} ecartees hors fenetre reseau')
    ecarts = [float(np.max(np.abs(b - a)))
              for (ta, a), (tb, b) in zip(plan[:-1], plan[1:]) if ta == tb]
    if ecarts:
        print(f'ecart median entre images consecutives : {np.median(ecarts):.2f} deg '
              f'(NVlabs 0,23 ; real_3cam 94,3)')
    print(f'images produites : {len(plan) * len(CAMERAS)} '
          f'({len(plan)} par camera, {len(CAMERAS)} cameras)')
    print(f'duree estimee : ~{len(plan) * 2.6 / 60:.0f} min')
    if args.simuler:
        print('\n--simuler : rien n a bouge, rien n a ete ecrit.')
        return

    racine = DONNEES / args.nom
    for cam in CAMERAS:
        (racine / 'images' / cam['nom']).mkdir(parents=True, exist_ok=True)
    labels = racine / 'labels.csv'
    deja = 0
    if labels.is_file():
        with labels.open() as f:
            deja = len({l['index'] for l in csv.DictReader(f)})
        print(f'\nreprise : {deja} poses deja enregistrees')

    from pick_and_place_real import Bridge
    pont = Bridge(args.host)
    cams = [Camera(c) for c in CAMERAS]
    entetes = (['index'] + [f'j{k}_rad' for k in range(1, 7)]
               + [f'j{k}_deg' for k in range(1, 7)] + ['camera', 'image_path'])
    neuf = not labels.is_file()
    debut, traj_courante = time.time(), None

    with labels.open('a', newline='') as f:
        ecrivain = csv.writer(f)
        if neuf:
            ecrivain.writerow(entetes)
        for i, (t, cible) in enumerate(plan):
            if i < deja:
                continue
            if t != traj_courante:                 # nouveau depart : on y va doux
                q = rejoint(pont, cible, args.vitesse)
                traj_courante = t
            else:
                pont.send({'action': 'send_angles',
                           'angles': [round(float(v), 2) for v in cible],
                           'speed': args.vitesse})
                q = immobile(pont)
            if np.max(np.abs(q - np.array(cible))) > 6.0:
                print(f'  [{i}] le bras n a pas suivi (ecart '
                      f'{np.max(np.abs(q - np.array(cible))):.1f} deg) — pose ignoree')
                continue
            for cam in cams:
                img = cam.lit()
                if img is None:
                    print(f'  [{i}] {cam.nom} : image non lue, ignoree')
                    continue
                chemin = f'images/{cam.nom}/{i:06d}.png'
                cv2.imwrite(str(racine / chemin), img)
                ecrivain.writerow([i] + [f'{v:.4f}' for v in np.radians(q)]
                                  + [f'{v:.2f}' for v in q] + [cam.nom, chemin])
            f.flush()
            if i % 20 == 0:
                fait = i - deja + 1
                reste = (len(plan) - i) * (time.time() - debut) / max(fait, 1)
                print(f'  {i + 1}/{len(plan)}  q={np.round(q, 1).tolist()}  '
                      f'reste ~{reste / 60:.0f} min')

    for cam in cams:
        cam.ferme()
    print(f'\ntermine — {racine}')
    print('etiquettes = angles MESURES bras arrete, jamais la consigne.')


if __name__ == '__main__':
    main()
