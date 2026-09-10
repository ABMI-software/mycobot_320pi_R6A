#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Capture haute pour la demo markerless — batie sur les poses DEJA VALIDEES.

But : montrer que DREAM localise la camera SANS marqueur, les 4 ArUco ne
servant que de juge independant. Il faut donc des images ou DREAM detecte bien.
Or le reseau n'a jamais vu la zone basse : le collecteur synthetique rejette
toute pose sous 130 mm (`synthetic_data_collector_v3.py`, TABLE_CLEARANCE), et
les 1102 poses du pick vivent entre 72,5 et 114,5 mm — 100 % hors domaine.

CE SCRIPT NE FABRIQUE PAS DE POSES. Il part de celles de
`scripts/pick_place_positions.json`, enregistrees et jouees par l'operateur, et
ne s'en ecarte que par petits balayages. Les trois retenues sont les seules qui
soient a la fois hautes, sures, vues par la camera et pince vers le bas :

    place_approach                J2  -84,6   162 mm   pince 17,1 deg
    handover_approach             J2  -86,3   162 mm   pince 13,9 deg
    handover_previous_reference   J2 -101,5   135 mm   pince 10,0 deg

Trois erreurs commises avant d'en arriver la, toutes corrigees ici :

1. Des poses tirees au HASARD sur toute la plage articulaire, ordonnees pour
   « maximiser la diversite » : 166 deg entre deux prises. Le bras n'a pas
   suivi et le pont est tombe. On ne s'ecarte plus d'une pose connue autrement
   que par petits pas.
2. J2 releve de +22 deg SANS compenser J4 : la pince passait de 12 a 27-38 deg
   de la verticale, et a 91 deg sur le robot — a l'horizontale. La direction de
   l'outil est desormais un critere de rejet, pas une consequence.
3. La visibilite exigee sur les DEUX cameras. `capture_trajectoires.py` la juge
   sur la seule arducam ; exiger les deux ecartait des poses de travail valides
   (`observation_clear`, `pick_approach_previous_reference`).

J6 ne fait partie d'aucun balayage : il ne deplace aucun keypoint du schema a 7
points (il tourne autour de son propre axe) mais il fait pivoter la PINCE — a
-170 elle pointe vers le haut, a +3 vers le bas. Il est donc amene une fois, en
tout debut, seul et verifie, puis laisse tranquille.

Usage :
    python3 scripts/capture_poses_hautes.py --simuler
    python3 scripts/capture_poses_hautes.py
"""
import argparse
import csv
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE / 'scripts'))
sys.path.insert(0, str(RACINE / 'training' / 'dream'))

from mycobot_fk import forward_kinematics
import capture_trajectoires as ct

GARDE_HAUTE_MM = 140.0      # 130 = debut du domaine synthetique, +10 pour l affaissement
PINCE_MAX_DEG = 25.0        # ecart tolere a la verticale descendante

# Balayages doux AUTOUR de chaque pose validee. J5 bouge peu : il incline
# l'outil et sort vite du critere de pince. J6 est absent, voir l'en-tete.
DELTAS = [(0, -30, 30),     # J1 azimut
          (1, -8, 8),       # J2 hauteur
          (2, -12, 12),     # J3 allonge
          (3, -12, 12),     # J4 poignet
          (4, -15, 15)]     # J5 inclinaison outil

# Capsules reprises de `synthetic_data_collector_v3.py` (recopiees et non
# importees : ce module exige rclpy, absent du venv_dream). `ct.pose_sure` ne
# verifie QUE le sol et le volume de la base — rien n'y empechait un lien de
# rentrer dans un autre.
LINK_RADII = [0.055, 0.045, 0.040, 0.036, 0.045]
SELF_MARGIN = 0.040         # 40 mm : la marge reelle vaut deja 40-64 mm partout
SELF_PAIRS = [(0, 2), (0, 3), (0, 4), (1, 3), (1, 4)]


def _dist_segments(p1, p2, p3, p4):
    u, v, w = p2 - p1, p4 - p3, p1 - p3
    a, b, c = u @ u, u @ v, v @ v
    d, e = u @ w, v @ w
    den = a * c - b * b
    sc, tc = ((0.0, (e / c if c > 1e-12 else 0.0)) if den < 1e-12
              else ((b * e - c * d) / den, (a * e - b * d) / den))
    sc, tc = min(max(sc, 0.0), 1.0), min(max(tc, 0.0), 1.0)
    return float(np.linalg.norm(w + u * sc - v * tc))


def sans_auto_collision(q):
    _, T = forward_kinematics(np.radians(q))
    P = [t[:3, 3] for t in T]
    skel = [P[0], P[1], P[3], P[4], P[5], P[6]]
    segs = [(skel[i], skel[i + 1]) for i in range(5)]
    return all(_dist_segments(*segs[i], *segs[j])
               >= LINK_RADII[i] + LINK_RADII[j] + SELF_MARGIN
               for i, j in SELF_PAIRS)


def hauteur_bras_mm(q):
    """Hauteur minimale du bras, colonne de base exclue — MEME definition que le
    filtre du collecteur synthetique, sinon la comparaison n'aurait pas de sens."""
    _, T = forward_kinematics(np.radians(q))
    P = np.array([t[:3, 3] for t in T]) * 1000.0
    return min((a + (b - a) * t)[2]
               for a, b in zip(P[1:-1], P[2:]) for t in np.linspace(0, 1, 6))


def angle_pince_deg(q):
    """Ecart entre l'axe de la pince et la verticale descendante."""
    _, T = forward_kinematics(np.radians(q))
    axe = T[6][:3, :3] @ ct._AXE_PINCE
    return float(np.degrees(np.arccos(np.clip(-axe[2] / np.linalg.norm(axe), -1, 1))))


def bases():
    d = json.loads((RACINE / 'scripts' / 'pick_place_positions.json').read_text())
    return [d['place_approach'], d['handover_approach'],
            d['handover_previous_reference']]


def construit(pas, mdl):
    plan, rejets = [], {'sure': 0, 'bas': 0, 'pince': 0, 'vue': 0}
    for t, b in enumerate(bases()):
        for j, lo, hi in DELTAS:
            a, c = list(b), list(b)
            a[j] += lo
            c[j] += hi
            for q in ct.echantillonne([a, c], pas):
                if not (ct.pose_sure(q) and sans_auto_collision(q)):
                    rejets['sure'] += 1
                elif hauteur_bras_mm(q) < GARDE_HAUTE_MM:
                    rejets['bas'] += 1
                elif angle_pince_deg(q) > PINCE_MAX_DEG:
                    rejets['pince'] += 1
                elif not ct.visible(q, mdl):
                    rejets['vue'] += 1
                else:
                    plan.append((t, q))
    return plan, rejets


def aligne_j6(pont, cible_j6):
    """Amene J6 SEUL, par petits pas, et verifie qu'il a suivi.

    J6 etait a -170 quand les poses le voulaient a +3 : 173 deg d'un coup. La
    capture precedente a echoue trois fois de suite dessus. On l'amene donc a
    part, avant tout le reste, et on renonce s'il ne bouge pas — plutot que de
    le decouvrir pose apres pose.
    """
    q = ct.immobile(pont)
    print(f'  J6 mesure {q[5]:.1f}, vise {cible_j6:.1f}')
    while abs(q[5] - cible_j6) > 3.0:
        pas = np.clip(cible_j6 - q[5], -20.0, 20.0)
        vise = list(q)
        vise[5] = q[5] + pas
        pont.send({'action': 'send_angles',
                   'angles': [round(float(v), 2) for v in vise], 'speed': 20})
        time.sleep(1.6)
        neuf = ct.immobile(pont)
        if abs(neuf[5] - q[5]) < 0.5 * abs(pas):
            raise SystemExit(f'J6 ne suit pas : {q[5]:.1f} -> {neuf[5]:.1f} '
                             f'pour {pas:+.0f} deg commandes. Arret.')
        q = neuf
        print(f'    J6 a {q[5]:.1f}')
    return q


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--nom', default='demo_markerless_haut')
    ap.add_argument('--pas', type=float, default=2.5)
    ap.add_argument('--simuler', action='store_true')
    ap.add_argument('--host', default='10.10.0.224')
    ap.add_argument('--vitesse', type=int, default=20)
    args = ap.parse_args()

    mdl = ct.modele(ct.CAMERAS[0])          # arducam SEULE
    plan, rejets = construit(args.pas, mdl)
    if not plan:
        raise SystemExit('aucune pose exploitable')
    h = [hauteur_bras_mm(q) for _, q in plan]
    pin = [angle_pince_deg(q) for _, q in plan]
    ecarts = [float(np.max(np.abs(np.array(b) - np.array(a))))
              for (ta, a), (tb, b) in zip(plan[:-1], plan[1:]) if ta == tb]

    print(f'{len(plan)} poses retenues autour de 3 poses validees, pas {args.pas} deg')
    print(f'  ecartees : {rejets}')
    print(f'  hauteur bras {min(h):.0f}-{max(h):.0f} mm   '
          f'pince {min(pin):.0f}-{max(pin):.0f} deg de la verticale')
    print(f'  ecart median entre images : {np.median(ecarts):.2f} deg')
    print(f'  images produites : {len(plan) * len(ct.CAMERAS)}   '
          f'duree ~{len(plan) * 2.6 / 60:.0f} min')
    if args.simuler:
        print('\n--simuler : rien n a bouge, rien n a ete ecrit.')
        return

    racine = (ct.CAPTURES / args.nom).resolve()
    if ct.CAPTURES.resolve() not in racine.parents:
        raise SystemExit(f'refus : {racine} est hors de {ct.CAPTURES}')
    for cam in ct.CAMERAS:
        (racine / 'images' / cam['nom']).mkdir(parents=True, exist_ok=True)

    from pick_and_place_real import Bridge
    pont = Bridge(args.host)
    # Retendre les servos AVANT tout ordre : un joint relache accepte la
    # commande sans bouger (mesure : J5 a 76,7 au lieu de -18, J6 a -134 au lieu
    # de 50), ce qui faisait echouer la tolerance sur chaque pose.
    print('power_on :', pont.send({'action': 'power_on'}))
    time.sleep(2.0)
    aligne_j6(pont, plan[0][1][5])

    cams = [ct.Camera(c) for c in ct.CAMERAS]
    for cam, cfg in zip(cams, ct.CAMERAS):
        if not ct.ecrit_camera_settings(cfg, racine, 640, 480):
            raise SystemExit(f'{cam.nom} : intrinseque {cfg["calib"]} introuvable')
        print(f'  {cam.nom} : /dev/video{cam.index}')

    entetes = (['index'] + [f'j{k}_rad' for k in range(1, 7)]
               + [f'j{k}_deg' for k in range(1, 7)] + ['camera', 'image_path'])
    labels = racine / 'labels.csv'
    neuf = not labels.is_file()
    debut, traj, rates = time.time(), None, 0
    with labels.open('a', newline='') as f:
        ecrivain = csv.writer(f)
        if neuf:
            ecrivain.writerow(entetes)
        for i, (t, cible) in enumerate(plan):
            if t != traj:
                q = ct.rejoint(pont, cible, args.vitesse)
                traj = t
            else:
                pont.send({'action': 'send_angles',
                           'angles': [round(float(v), 2) for v in cible],
                           'speed': args.vitesse})
                q = ct.immobile(pont)
            ecart = float(np.max(np.abs(q - np.array(cible))))
            if ecart > 6.0:
                rates += 1
                pire = int(np.argmax(np.abs(q - np.array(cible))))
                print(f'  [{i}] pas suivi — J{pire + 1} a {ecart:.0f} deg '
                      f'({rates}e d affilee)')
                if rates >= 3:
                    print(f'\nARRET : trois echecs d affilee, mesure '
                          f'{np.round(q, 1).tolist()}')
                    break
                continue
            rates = 0
            for cam in cams:
                img = cam.lit()
                if img is None:
                    continue
                chemin = f'images/{cam.nom}/{i:06d}.png'
                cv2.imwrite(str(racine / chemin), img)
                ecrivain.writerow([i] + [f'{v:.4f}' for v in np.radians(q)]
                                  + [f'{v:.2f}' for v in q] + [cam.nom, chemin])
            f.flush()
            if i % 20 == 0:
                reste = (len(plan) - i) * (time.time() - debut) / max(i + 1, 1)
                print(f'  {i + 1}/{len(plan)}  bras {hauteur_bras_mm(q):.0f} mm  '
                      f'pince {angle_pince_deg(q):.0f} deg  reste ~{reste / 60:.0f} min')

    for cam in cams:
        cam.ferme()
    print(f'\ntermine — {racine}')


if __name__ == '__main__':
    main()
