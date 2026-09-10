#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Petite capture multi-poses pour la demo MARKERLESS de DREAM.

But. DREAM doit retrouver seul la pose des cameras, sans marqueur : le robot
lui-meme sert de mire (FK des encodeurs -> 7 points 3D, DREAM -> leurs 2D). Les
4 ArUco ne servent alors QUE de juge independant, jamais d'entree.

Pourquoi plusieurs poses et pas une seule. Un PnP mono-vue sur une seule
configuration du bras est degenere : les 7 keypoints d'une pose sont presque
coplanaires et la rotation admet plusieurs branches qui reprojettent aussi bien.
Mesure de juillet : 27-30 deg d'erreur d'orientation a une pose. En empilant une
vingtaine de configurations differentes les points remplissent un volume et la
branche devient unique.

Ce script ne definit AUCUNE nouvelle regle de securite : il importe telles
quelles celles de `capture_trajectoires.py` (garde au sol de la pointe, garde
des liens mobiles, volume de la base, approche par paliers, angles MESURES
bras arrete). Il n'ecrit que dans `training/dream/captures/<nom>/`.

Deux differences avec `capture_trajectoires.py`, voulues :
  - visibilite exigee sur LES DEUX cameras, pas seulement l'arducam. Une pose
    vue d'un seul cote n'apprend rien a l'autre extrinseque ;
  - poses choisies pour leur DIVERSITE (echantillonnage du plus eloigne) et non
    pour leur densite : ici on veut du volume, pas de la trajectoire.

Usage (venv_dream) :
    python scripts/capture_markerless.py --simuler          # rien ne bouge
    python scripts/capture_markerless.py --nom markerless_0902
"""
import argparse
import csv
import sys
import time
from pathlib import Path

import cv2
import numpy as np

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE / 'scripts'))

import capture_trajectoires as ct                            # noqa: E402
from mycobot_fk import forward_kinematics, KEYPOINT_NAMES    # noqa: E402

# L'extrinseque SVPRO de `capture_trajectoires.CAMERAS` (`_servo`, 24/08) est
# perimee : la camera a ete inclinee le 01/09 pour faire entrer le marqueur 25.
# On juge la visibilite sur l'ajustement 4 marqueurs du 02/09 (RMS 1,14 px).
EXTRINSEQUES = {'arducam': 'arducam_extrinsic_pick',
                'svpro': 'svpro_extrinsic_4marqueurs'}
N_POSES = 24


def plan_diversifie(n, pas):
    """Poses sures et vues des DEUX cameras, puis les `n` plus dispersees."""
    modeles = {}
    for cfg in ct.CAMERAS:
        cfg = dict(cfg, extr=EXTRINSEQUES[cfg['nom']])
        modeles[cfg['nom']] = ct.modele(cfg)

    pool, hors_champ, dangereuses = [], 0, 0
    for sommets in ct.TRAJECTOIRES:
        for q in ct.echantillonne(sommets, pas):
            if not ct.pose_sure(q):
                dangereuses += 1
            elif all(ct.visible(q, m) for m in modeles.values()):
                pool.append(q)
            else:
                hors_champ += 1
    print(f'{len(pool)} poses sures et vues des 2 cameras '
          f'({hors_champ} hors champ, {dangereuses} dangereuses)')
    if len(pool) < n:
        raise SystemExit('pas assez de poses exploitables')

    # Echantillonnage du plus eloigne : on part de la pose mediane et on ajoute
    # a chaque tour celle qui est la plus loin de tout ce qui est deja retenu.
    P = np.array(pool, float)
    choisis = [int(np.argmin(np.linalg.norm(P - P.mean(0), axis=1)))]
    d = np.max(np.abs(P - P[choisis[0]]), axis=1)
    while len(choisis) < n:
        i = int(np.argmax(d))
        choisis.append(i)
        d = np.minimum(d, np.max(np.abs(P - P[i]), axis=1))
    return [P[i] for i in choisis]


PAS_TRAJET = 40          # subdivisions verifiees entre deux poses


def trajet_sur(a, b, n=PAS_TRAJET):
    """`pose_sure` protege une pose IMMOBILE, pas le chemin qui y mene.

    Mesure du 01/09 : entre deux poses toutes deux au-dessus de la garde, le
    bras est descendu a 125 mm alors que 145 etaient exiges — l'interpolation
    articulaire n'est pas une droite cartesienne. On verifie donc chaque
    segment, et pas seulement ses extremites.
    """
    a, b = np.asarray(a, float), np.asarray(b, float)
    return all(ct.pose_sure(a + (b - a) * (i / n)) for i in range(n + 1))


def ordonne(poses, depart):
    """Chaine du plus proche voisin dont le TRAJET est sur.

    Quand aucun voisin restant n'est joignable directement, on passe par la
    pose de transit — celle dont la pointe est la plus haute — plutot que de
    forcer un segment qui racle.
    """
    poses = [np.asarray(p, float) for p in poses]
    transit = max(poses, key=ct.hauteur_pointe_mm)
    reste, chaine, courant = list(poses), [], np.asarray(depart, float)
    while reste:
        ordre_proximite = sorted(range(len(reste)),
                                 key=lambda i: np.max(np.abs(reste[i] - courant)))
        suivant = next((i for i in ordre_proximite
                        if trajet_sur(courant, reste[i])), None)
        if suivant is None:
            if not trajet_sur(courant, transit):
                raise SystemExit('aucun trajet sur depuis la pose courante — arret')
            chaine.append(transit)
            courant = transit
            continue
        courant = reste.pop(suivant)
        chaine.append(courant)
    return chaine


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--nom', default='markerless_0902')
    ap.add_argument('--poses', type=int, default=N_POSES)
    ap.add_argument('--pas', type=float, default=6.0)
    ap.add_argument('--simuler', action='store_true')
    ap.add_argument('--host', default='10.10.0.224')
    ap.add_argument('--vitesse', type=int, default=25)
    args = ap.parse_args()

    poses = plan_diversifie(args.poses, args.pas)
    P = np.array(poses)
    print(f'{len(poses)} poses retenues — amplitude par joint (deg) : '
          f'{np.round(P.max(0) - P.min(0), 0).tolist()}')
    print(f'pointe la plus basse : {min(ct.hauteur_pointe_mm(q) for q in poses):.0f} mm '
          f'(garde exigee {ct.GARDE_POINTE_MM:.0f})')
    if args.simuler:
        print('\n--simuler : rien n a bouge, rien n a ete ecrit.')
        return

    racine = (ct.CAPTURES / args.nom).resolve()
    if ct.CAPTURES.resolve() not in racine.parents:
        raise SystemExit(f'refus : {racine} est hors de {ct.CAPTURES}')
    if racine.exists():
        raise SystemExit(f'{racine} existe deja — choisir un autre --nom')

    from pick_and_place_real import Bridge
    pont = Bridge(args.host)
    ordre = ordonne(poses, pont.get_angles())

    cams, vues = [], {}
    for cfg in ct.CAMERAS:
        cam = ct.Camera(cfg)
        ident = ct.identite_physique(cam.index)
        if ident in vues:
            raise SystemExit(f'{cam.nom} et {vues[ident]} sont la MEME camera physique')
        vues[ident] = cam.nom
        (racine / 'images' / cam.nom).mkdir(parents=True, exist_ok=True)
        if not ct.ecrit_camera_settings(cfg, racine, 640, 480):
            raise SystemExit(f'{cam.nom} : intrinseque introuvable')
        print(f'  {cam.nom} : /dev/video{cam.index}')
        cams.append(cam)

    entetes = (['index'] + [f'j{k}_rad' for k in range(1, 7)]
               + [f'j{k}_deg' for k in range(1, 7)] + ['camera', 'image_path'])
    rates = 0
    with (racine / 'labels.csv').open('w', newline='') as f:
        ecrivain = csv.writer(f)
        ecrivain.writerow(entetes)
        for i, cible in enumerate(ordre):
            q = ct.rejoint(pont, cible, args.vitesse)
            ecart = float(np.max(np.abs(q - cible)))
            if ecart > 6.0:
                rates += 1
                print(f'  [{i:02d}] le bras n a pas suivi ({ecart:.1f} deg) — ignoree')
                if rates >= 3:
                    raise SystemExit('3 poses ratees — arret, verifier les servos')
                continue
            rates = 0
            for cam in cams:
                img = cam.lit()
                if img is None:
                    print(f'  [{i:02d}] {cam.nom} : image non lue')
                    continue
                chemin = f'images/{cam.nom}/{i:06d}.png'
                cv2.imwrite(str(racine / chemin), img)
                ecrivain.writerow([i] + [f'{v:.4f}' for v in np.radians(q)]
                                  + [f'{v:.2f}' for v in q] + [cam.nom, chemin])
            f.flush()
            print(f'  [{i:02d}] q={np.round(q, 1).tolist()}')

    for cam in cams:
        cam.ferme()
    print(f'\ntermine — {racine}')


if __name__ == '__main__':
    main()
