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

Ce script n'ecrit QUE dans `training/dream/captures/<nom>/`, un dossier neuf.
Il ne touche ni `dream_data/` ni `capture_real_3cam.py`.

Usage (venv_dream) :
    # ce que ca donnerait, SANS bouger le bras
    python scripts/capture_trajectoires.py --simuler

    # capture reelle — LE BRAS BOUGE LONGTEMPS
    python scripts/capture_trajectoires.py --nom real_montage_0901
    # reprise apres interruption : relancer la meme commande
"""
import argparse
import csv
import json
import math
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

RACINE = Path(__file__).resolve().parents[1]
CALIB = RACINE / 'training' / 'calibration'
# Dossier NEUF, deliberement hors de `dream_data/` : les jeux qui y vivent
# (real_3cam, les mix, les *_ndds) sont acquis et ne doivent pas etre
# melanges avec une capture faite sur un AUTRE montage de camera.
CAPTURES = RACINE / 'training' / 'dream' / 'captures'

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
    {'nom': 'arducam', 'v4l2': 'Arducam', 'exposition': 75, 'focus': None,
     'calib': 'cam_3', 'extr': 'arducam_extrinsic_pick'},
    # focus 90 : le plateau net de la SVPRO (cf. capture_real_3cam.py), sinon
    # l'autofocus derive vers la zone catastrophiquement floue du milieu.
    {'nom': 'svpro', 'v4l2': '5MP', 'exposition': None, 'focus': 90,
     'calib': 'cam_2', 'extr': 'svpro_extrinsic_servo'},
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

# ---------------------------------------------------------------------------
# Securite de pose et reglages camera — logique reprise de
# `training/capture_real_3cam.py` (non modifie, non importe : ce script reste
# autonome). Les constantes viennent de la geometrie mesuree du 320 Pi.
# ---------------------------------------------------------------------------
_BASE_H, _L_UPPER, _L_FORE, _L_FORE_Z = 162.0, 136.35, 120.5, 82.0
_L_WRIST, _L_EE = 84.0, 66.35
# La pince Pro montee sur la bride descend ~110 mm SOUS link6. Sans ce terme,
# des poses jugees « sures » enfoncent les doigts dans la table : link6 passe a
# 60 mm mais les doigts sont 110 mm plus bas. Mettre 0.0 si la pince est retiree.
_L_GRIPPER = 110.0
_TABLE_Z_MIN, _BASE_R_MIN = 60.0, 90.0


def _points_cles(j2, j3, j4):
    a2, a3 = math.radians(j2), math.radians(j2 + j3)
    a4 = math.radians(j2 + j3 + j4)
    z_coude = _BASE_H + _L_UPPER * math.cos(a2)
    r_coude = _L_UPPER * math.sin(a2)
    z_poignet = z_coude + _L_FORE * math.cos(a3) - _L_FORE_Z * math.sin(a3)
    r_poignet = r_coude + _L_FORE * math.sin(a3) + _L_FORE_Z * math.cos(a3)
    l = _L_WRIST + _L_EE + _L_GRIPPER
    return [(z_coude, abs(r_coude)), (z_poignet, abs(r_poignet)),
            (z_poignet + l * math.cos(a4), abs(r_poignet + l * math.sin(a4)))]


def pose_sure(angles_deg):
    """Aucun point cle sous la table, ni dans le volume de la base."""
    for z, r in _points_cles(angles_deg[1], angles_deg[2], angles_deg[3]):
        if z < _TABLE_Z_MIN or (z < _BASE_H and r < _BASE_R_MIN):
            return False
    return True


def ecrit_camera_settings(cam, dossier, w, h):
    """`_camera_settings.json` NDDS dans le dossier images/<cam>/.

    Sans ce fichier la conversion NDDS ne peut pas lire la capture : c'est lui
    qui porte les intrinseques, remises a l'echelle de la trame reellement
    capturee (fx, cx avec la largeur ; fy, cy avec la hauteur)."""
    K, dist = registre.load_intrinsics(cam['calib'], w, h)
    if K is None:
        return False
    reglages = {'camera_settings': [{
        'name': cam['nom'],
        'intrinsic_settings': {'fx': float(K[0, 0]), 'fy': float(K[1, 1]),
                               'cx': float(K[0, 2]), 'cy': float(K[1, 2]),
                               's': float(K[0, 1])},
        'captured_image_size': {'width': w, 'height': h},
        'dist_coeffs': [float(c) for c in np.asarray(dist).ravel()],
    }]}
    chemin = Path(dossier) / 'images' / cam['nom'] / '_camera_settings.json'
    chemin.parent.mkdir(parents=True, exist_ok=True)
    chemin.write_text(json.dumps(reglages, indent=2))
    return True


def identite_physique(index):
    """Identifiant du CAPTEUR : deux /dev/videoN peuvent etre la meme camera."""
    sortie = subprocess.run(['udevadm', 'info', '-q', 'property', '-n',
                             f'/dev/video{index}'], capture_output=True, text=True).stdout
    for cle in ('ID_SERIAL_SHORT=', 'ID_SERIAL=', 'ID_PATH='):
        for ligne in sortie.splitlines():
            if ligne.startswith(cle):
                return ligne.split('=', 1)[1]
    return f'video{index}'



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
        self.focus = cfg.get('focus')
        self.cap = cv2.VideoCapture(self.index, cv2.CAP_V4L2)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        if not self.cap.isOpened():
            raise SystemExit(f'{self.nom} : /dev/video{self.index} inaccessible')
        self._regle()

    def _regle(self):
        """Exposition et mise au point FIGEES — sinon la camera derive d'une
        pose a l'autre et le jeu de donnees melange plusieurs rendus."""
        dev = f'/dev/video{self.index}'
        if self.exposition is not None:
            # `auto_exposure=1` (manuel) d'ABORD, sinon exposure_time_absolute
            # est ignore. gain et brightness sont epingles aussi : c'est ce qui
            # evite les trames noires aleatoires (cf. capture_real_3cam.py).
            subprocess.run(['v4l2-ctl', '-d', dev, '--set-ctrl', 'auto_exposure=1'],
                           capture_output=True)
            subprocess.run(['v4l2-ctl', '-d', dev, '--set-ctrl',
                            f'exposure_time_absolute={self.exposition},'
                            'gain=0,brightness=0'], capture_output=True)
        if self.focus is not None:
            # La SVPRO a un vrai objectif a focale variable : l'autofocus
            # continu POMPE entre les poses et la nettete s'effondre au milieu
            # de la plage. On le coupe et on epingle.
            subprocess.run(['v4l2-ctl', '-d', dev, '--set-ctrl',
                            'focus_automatic_continuous=0'], capture_output=True)
            subprocess.run(['v4l2-ctl', '-d', dev, '--set-ctrl',
                            f'focus_absolute={self.focus},sharpness=0,contrast=1'],
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
    ap.add_argument('--nom', default='real_montage',
                    help='dossier sous training/dream/captures/ (jamais dream_data)')
    ap.add_argument('--pas', type=float, default=PAS_DEG)
    ap.add_argument('--simuler', action='store_true',
                    help='compter les poses retenues sans bouger le bras')
    ap.add_argument('--host', default='10.10.0.224')
    ap.add_argument('--vitesse', type=int, default=25)
    args = ap.parse_args()

    mdl = modele(CAMERAS[0])          # visibilite jugee sur l'arducam
    plan, aveugles, dangereuses = [], 0, 0
    for t, sommets in enumerate(TRAJECTOIRES):
        for q in echantillonne(sommets, args.pas):
            if not pose_sure(q):
                dangereuses += 1          # doigts sous la table, ou dans la base
            elif visible(q, mdl):
                plan.append((t, q))
            else:
                aveugles += 1

    print(f'{len(TRAJECTOIRES)} trajectoires, pas {args.pas} deg')
    print(f'{len(plan)} poses retenues, {aveugles} ecartees hors fenetre reseau, '
          f'{dangereuses} ecartees comme DANGEREUSES (pince sous la table ou base)')
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

    racine = (CAPTURES / args.nom).resolve()
    if CAPTURES.resolve() not in racine.parents:
        raise SystemExit(f'refus : {racine} est hors de {CAPTURES}')
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
    vues = {}
    for cam, cfg in zip(cams, CAMERAS):
        ident = identite_physique(cam.index)
        if ident in vues:
            raise SystemExit(f'{cam.nom} et {vues[ident]} sont la MEME camera '
                             f'physique ({ident}) — images en double')
        vues[ident] = cam.nom
        if not ecrit_camera_settings(cfg, racine, 640, 480):
            raise SystemExit(f'{cam.nom} : intrinseque {cfg["calib"]} introuvable, '
                             'le jeu serait inutilisable pour la conversion NDDS')
        print(f'  {cam.nom} : /dev/video{cam.index}, _camera_settings.json ecrit')
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
    print('dream_data/ n a pas ete touche.')
    print('etiquettes = angles MESURES bras arrete, jamais la consigne.')


if __name__ == '__main__':
    main()
