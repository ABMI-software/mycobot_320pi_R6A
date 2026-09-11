#!/usr/bin/env python3
"""Calibration extrinseque automatique de l'arducam, pilotable depuis une IHM.

Tourne sous le venv (`.venv/bin/python`) : la detection ArUco exige OpenCV 5,
le dashboard tourne lui sur le Python systeme en OpenCV 4.6. C'est pourquoi ce
script est un PROCESSUS separe et non une fonction — meme decoupage que
`_verifie_extrinseque` dans le dashboard.

Deux modes :

    --controle    mesure, sans rien ecrire, de combien les marqueurs se sont
                  ecartes de la reference. C'est ce chiffre qui dit s'il faut
                  recalibrer.

    --reference   releve la position ACTUELLE des quatre marqueurs a travers
                  l'extrinseque de production, et l'ecrit dans
                  `planche_actuelle.yaml`. A faire pendant que l'extrinseque
                  est encore juste.

    (defaut)      recalcule la pose camera contre `planche_actuelle.yaml`,
                  valide, sauvegarde l'ancienne et ecrit la nouvelle.

Pourquoi une reference distincte de `workspace_markers.yaml`. La planche a
bouge le 10/09/2026 — rotation -1,750 deg, translation (18,8 ; -6,5) mm,
mouvement rigide, residu 0,39 mm. `workspace_markers.yaml` decrit donc la
planche d'AVANT. Recalibrer contre lui reinjecterait ces 19 mm dans une
extrinseque qui, elle, est juste : la camera n'a pas bouge (le trepied du fond
n'a derive que de 0,2 px). Ce script ne lit donc jamais `workspace_markers.yaml`
et ne l'ecrit jamais non plus — il reste la reference historique, versionnee.

Ce qui n'est PAS touche, et ne doit pas l'etre :
  - l'intrinseque de l'arducam (`cam_3.meta.json`), lue telle quelle ;
  - `marker_size_mm`, qui vaut 50 et le reste (le cote mesure a l'image est
    tire vers l'interieur par l'obliquite, correlation -0,920 ; voir l'essai L).

Sortie sur stdout, une ligne par evenement, pour que l'IHM suive sans parser
du texte libre :

    ETAPE|<k>|<total>|<libelle>
    INFO|<texte>
    ERREUR|<texte>
    RESULTAT|<json>
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import yaml

RACINE = Path(__file__).resolve().parents[1]
CALIB = RACINE / 'training' / 'calibration'
sys.path.insert(0, str(RACINE / 'scripts'))
sys.path.insert(0, str(RACINE / 'mycobot_gateway' / 'mycobot_gateway' / 'vision'))

import calibrer_extrinseque_4aruco as base                              # noqa: E402
import camera_registry as registre                                      # noqa: E402

EXTRINSEQUE = CALIB / 'arducam_extrinsic_pick.yaml'
REFERENCE = CALIB / 'planche_actuelle.yaml'
LARGEUR, HAUTEUR = registre.CAPTURE_W, registre.CAPTURE_H

# Reglages tentes dans l'ordre tant que les quatre marqueurs ne sortent pas
# tous. Le premier est celui du dashboard : calibrer dans les conditions
# d'exploitation vaut mieux que dans des conditions de laboratoire. Le (45, 60)
# est celui qui a donne 4/4 le 10/09.
REGLAGES = [(75, 0), (45, 60), (60, 30), (100, 0), (30, 80)]

RMS_MAX_PX = 1.5          # au-dela, l'ajustement n'est pas sain
LEAVE_ONE_OUT_MAX_MM = 5.0   # au-dela, on n'ecrit pas : mieux vaut l'ancienne
SAUT_CAMERA_MAX_MM = 150.0   # au-dela, ce n'est pas un deplacement, c'est une erreur


def dis(canal, *morceaux):
    print('|'.join([canal] + [str(m) for m in morceaux]), flush=True)


def etape(k, total, libelle):
    dis('ETAPE', k, total, libelle)


def index_arducam():
    specs = registre.detect_cameras(['arducam'], probe_capture=True)
    if not specs:
        raise SystemExit('arducam absente de v4l2-ctl')
    return specs[0].v4l2_index


def regle(index, exposition, gain):
    """Exposition ET gain, dans cet ordre, manuel d'abord.

    Le gain n'est pas cosmetique : apres un rebranchement USB la meme valeur
    d'exposition ne rend plus la meme luminance, et les scripts qui ne posent
    que l'exposition laissent le gain a 0 — l'image noircit et ArUco ne trouve
    plus rien. Piege consigne dans l'extrinseque elle-meme.
    """
    dev = f'/dev/video{index}'
    for ctrl in ('auto_exposure=1',
                 f'exposure_time_absolute={exposition}',
                 f'gain={gain}', 'brightness=0'):
        subprocess.run(['v4l2-ctl', '-d', dev, '--set-ctrl', ctrl],
                       capture_output=True, timeout=5)


def capture(index, n):
    cap = cv2.VideoCapture(index)
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, LARGEUR)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HAUTEUR)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
    for _ in range(10):
        cap.read()
    trames = []
    for _ in range(n * 4):
        ok, img = cap.read()
        if ok:
            trames.append(img)
        if len(trames) >= n:
            break
    cap.release()
    if not trames:
        raise SystemExit(f'aucune image depuis /dev/video{index}')
    return trames


def vus_partout(par_trame, attendus):
    partout = set(par_trame[0])
    for p in par_trame[1:]:
        partout &= set(p)
    return sorted(i for i in attendus if i in partout)


def acquiert(index, attendus, trames_n, etape0, total):
    """(trames, detections, reglage, luminance) — balaye jusqu'a voir les 4.

    Un marqueur intermittent biaiserait l'ajustement entre les images qui l'ont
    et celles qui ne l'ont pas : on exige de le voir sur TOUTES les trames.
    """
    dernier = None
    for k, (expo, gain) in enumerate(REGLAGES):
        etape(etape0, total, f'acquisition — exposition {expo}, gain {gain}')
        regle(index, expo, gain)
        time.sleep(0.4)
        trames = capture(index, trames_n)
        luminance = float(np.mean([cv2.cvtColor(t, cv2.COLOR_BGR2GRAY).mean()
                                   for t in trames]))
        par_trame = [base.detecte(t) for t in trames]
        utiles = vus_partout(par_trame, attendus)
        dis('INFO', f'{len(utiles)}/{len(attendus)} marqueurs — luminance '
                    f'{luminance:.0f} — expo {expo} gain {gain}')
        dernier = (trames, par_trame, utiles, (expo, gain), luminance)
        if len(utiles) == len(attendus):
            return dernier
    return dernier


def ajuste(par_trame, utiles, monde, K, dist):
    """Pose camera sur les 16 coins, plus le diagnostic.

    Le cote du marqueur est MESURE sur l'image et non lu a 50 mm, et c'est
    volontaire : la detection tire les coins d'un petit carre vers l'interieur,
    donc un modele 3D a 50 mm ne correspondrait pas aux coins detectes. Les
    deux biais se compensent et le CENTRE reste juste a 0,11-0,42 mm — c'est
    exactement la methode qui a produit l'extrinseque de production.
    """
    centres = {i: np.mean([p[i] for p in par_trame], axis=0).mean(axis=0)
               for i in utiles}
    T0, _, _ = base.pose(np.array([monde[i] for i in utiles]),
                         np.array([centres[i] for i in utiles]), K, dist)
    obj, img, appartenance, cotes = [], [], [], {}
    for i in utiles:
        quad = np.mean([p[i] for p in par_trame], axis=0)
        plan = np.array([base.vers_plan(uv, K, dist, T0, 0.0)[:2] for uv in quad])
        yaw = float(np.arctan2(*(plan[1] - plan[0])[::-1]))
        cote = float(np.mean([np.linalg.norm(plan[k] - plan[(k + 1) % 4])
                              for k in range(4)]))
        cotes[i] = cote * 1000.0
        for k, c3 in enumerate(base.coins_3d(monde[i], cote, yaw)):
            obj.append(c3)
            img.append(quad[k])
            appartenance.append(i)
    obj, img = np.array(obj), np.array(img)
    appartenance = np.array(appartenance)
    T, rvec, tvec = base.pose(obj, img, K, dist)
    proj = cv2.projectPoints(obj, rvec, tvec, K, dist)[0].reshape(-1, 2)
    rms = float(np.sqrt((np.linalg.norm(proj - img, axis=1) ** 2).mean()))
    return T, rms, centres, cotes, (obj, img, appartenance)


def leave_one_out(obj, img, appartenance, utiles, centres, monde, K, dist):
    """Pire erreur sur un marqueur EXCLU de l'ajustement.

    Un ajustement qui reproduit ses propres points ne mesure rien : il est
    circulaire. Seule la prediction d'un point neuf dit la justesse.
    """
    pires = {}
    for h in utiles:
        m = appartenance != h
        Th, _, _ = base.pose(obj[m], img[m], K, dist)
        vu = base.vers_plan(centres[h], K, dist, Th, 0.0)[:2] * 1000.0
        pires[h] = float(np.linalg.norm(vu - monde[h][:2] * 1000.0))
    return pires


def position_camera(T):
    return (-T[:3, :3].T @ T[:3, 3]) * 1000.0


def controle(index, trames_n):
    """De combien les marqueurs ont-ils bouge depuis la reference ?

    Ne mesure PAS la justesse de l'extrinseque : un ecart ici veut dire que
    les marqueurs ont bouge, ou que la camera a bouge, et rien dans cette
    mesure ne distingue les deux. Le seul controle qui tranche est la
    projection du squelette du robot sur son image (11/09).
    """
    total = 3
    etape(1, total, 'lecture de la reference')
    fichier = REFERENCE if REFERENCE.exists() else CALIB / 'workspace_markers.yaml'
    ref = yaml.safe_load(fichier.read_text())
    monde = {int(k): np.array(v, float) for k, v in ref['markers'].items()}
    d = yaml.safe_load(EXTRINSEQUE.read_text())
    T = np.array(d['T_cam_world'], float)
    K, dist = registre.load_intrinsics(d['intrinsics_stem'])

    trames, par_trame, utiles, reglage, luminance = acquiert(
        index, sorted(monde), trames_n, 2, total)
    if not utiles:
        raise SystemExit('aucun marqueur vu — degager le bras, ecarter les cables')

    etape(3, total, 'comparaison')
    ecarts = {}
    for i in utiles:
        centre = np.mean([p[i] for p in par_trame], axis=0).mean(axis=0)
        vu = base.vers_plan(centre, K, dist, T, 0.0)[:2] * 1000.0
        ecarts[i] = float(np.linalg.norm(vu - monde[i][:2]))
        dis('INFO', f'id {i} : {ecarts[i]:5.2f} mm de sa position de reference')
    pire = max(ecarts.values())
    moyen = float(np.mean(list(ecarts.values())))
    dis('INFO', f'reference {fichier.name} — ecart moyen {moyen:.2f} mm, '
                f'pire {pire:.2f} mm')
    dis('RESULTAT', json.dumps({
        'mode': 'controle', 'reference': fichier.name,
        'ecart_moyen_mm': round(moyen, 2), 'ecart_max_mm': round(pire, 2),
        'marqueurs_vus': [int(i) for i in utiles],
        'par_marqueur': {str(i): round(ecarts[i], 2) for i in utiles},
        'luminance': round(luminance, 1)}))
    return 0


def ecrit_reference(index, trames_n):
    total = 4
    etape(1, total, 'lecture de l\'extrinseque de production')
    d = yaml.safe_load(EXTRINSEQUE.read_text())
    T = np.array(d['T_cam_world'], float)
    K, dist = registre.load_intrinsics(d['intrinsics_stem'])
    if K is None:
        raise SystemExit(f"intrinseque {d['intrinsics_stem']} introuvable")
    connus = [int(i) for i in d.get('markers_used', [19, 23, 25, 26])]

    trames, par_trame, utiles, reglage, luminance = acquiert(
        index, connus, trames_n, 2, total)
    if len(utiles) < len(connus):
        raise SystemExit(f'{len(utiles)}/{len(connus)} marqueurs seulement — '
                         'degager le bras et ECARTER LES CABLES NOIRS des '
                         'bordures (un cable qui touche la bordure fusionne '
                         'avec elle et le marqueur devient indetectable)')

    etape(3, total, 'projection des centres sur le plan de travail')
    positions = {}
    for i in utiles:
        centre = np.mean([p[i] for p in par_trame], axis=0).mean(axis=0)
        xy = base.vers_plan(centre, K, dist, T, 0.0)[:2] * 1000.0
        positions[i] = [round(float(xy[0]), 2), round(float(xy[1]), 2), 0.0]

    etape(4, total, 'ecriture de la reference')
    REFERENCE.write_text(yaml.safe_dump({
        'source': (f'positions RELEVEES le {datetime.now():%d/%m/%Y %H:%M} a '
                   f'travers {EXTRINSEQUE.name}, {len(trames)} trames, '
                   f'exposition {reglage[0]} gain {reglage[1]}.'),
        'pourquoi': (
            'workspace_markers.yaml decrit la planche AVANT son deplacement du '
            '10/09/2026 (rotation -1,750 deg, translation 18,8 / -6,5 mm). '
            'Recalibrer contre lui reinjecterait ces 19 mm. Ce fichier-ci dit '
            'ou la planche est REELLEMENT, vue par une extrinseque encore '
            'juste. Il est la reference des recalibrations automatiques.'),
        'a_refaire_si': (
            'la planche bouge. PAS si la camera bouge — c\'est justement le cas '
            'que la recalibration automatique traite.'),
        'frame_id': 'base_link',
        'units': 'mm',
        'marker_size_mm': 50.0,
        'marker_size_NE_PAS_CORRIGER': (
            'Les marqueurs font 50 mm. Le cote mesure a l\'image vaut ~48,7 mm '
            'par biais de detection (correlation -0,920 avec l\'obliquite) ; '
            'les distances entre centres sont justes a -0,044 %.'),
        'markers': positions,
    }, sort_keys=False, allow_unicode=True))
    dis('INFO', f'reference ecrite -> {REFERENCE.name}')
    dis('RESULTAT', json.dumps({
        'mode': 'reference', 'marqueurs': {str(k): v for k, v in positions.items()},
        'luminance': round(luminance, 1)}))


def recalibre(index, trames_n, force):
    total = 6
    etape(1, total, 'lecture de la reference planche')
    if not REFERENCE.exists():
        raise SystemExit(f'{REFERENCE.name} absent — lancer d\'abord '
                         '--reference pendant que l\'extrinseque est juste')
    ref = yaml.safe_load(REFERENCE.read_text())
    monde = {int(k): np.array(v, float) / 1000.0 for k, v in ref['markers'].items()}
    ancienne = yaml.safe_load(EXTRINSEQUE.read_text())
    stem = ancienne['intrinsics_stem']
    K, dist = registre.load_intrinsics(stem)
    if K is None:
        raise SystemExit(f'intrinseque {stem} introuvable')
    T_avant = np.array(ancienne['T_cam_world'], float)

    trames, par_trame, utiles, reglage, luminance = acquiert(
        index, sorted(monde), trames_n, 2, total)
    if len(utiles) < 4:
        raise SystemExit(f'{len(utiles)}/4 marqueurs vus sur toutes les trames — '
                         'degager le bras et ecarter les cables noirs des '
                         'bordures. Sans 4 marqueurs il n\'y a pas de '
                         'validation possible : on ne recalibre pas.')

    etape(3, total, 'ajustement de la pose sur 16 coins')
    T, rms, centres, cotes, brut = ajuste(par_trame, utiles, monde, K, dist)

    etape(4, total, 'validation leave-one-out')
    pires = leave_one_out(*brut, utiles, centres, monde, K, dist)
    pire = max(pires.values())
    for i in sorted(pires):
        dis('INFO', f'id {i} exclu : {pires[i]:.2f} mm')

    cam, cam_avant = position_camera(T), position_camera(T_avant)
    deplacement = float(np.linalg.norm(cam - cam_avant))
    dis('INFO', f'RMS {rms:.3f} px — pire point neuf {pire:.2f} mm — '
                f'camera deplacee de {deplacement:.1f} mm')

    etape(5, total, 'controles avant ecriture')
    refus = []
    if rms > RMS_MAX_PX:
        refus.append(f'RMS {rms:.2f} px > {RMS_MAX_PX} px')
    if pire > LEAVE_ONE_OUT_MAX_MM:
        refus.append(f'pire point neuf {pire:.2f} mm > {LEAVE_ONE_OUT_MAX_MM} mm')
    if deplacement > SAUT_CAMERA_MAX_MM:
        refus.append(f'camera deplacee de {deplacement:.0f} mm > '
                     f'{SAUT_CAMERA_MAX_MM:.0f} mm')
    if refus and not force:
        dis('ERREUR', 'non ecrite — ' + ' ; '.join(refus) +
            '. L\'ancienne extrinseque est conservee.')
        dis('ERREUR', 'Cause la plus frequente : les feuilles portant les '
                      'marqueurs ont bouge depuis le releve de reference. '
                      'Les fixer, refaire --reference, puis recommencer.')
        dis('RESULTAT', json.dumps({'mode': 'recalibration', 'ecrite': False,
                                    'rms_px': round(rms, 3),
                                    'pire_mm': round(pire, 2),
                                    'deplacement_mm': round(deplacement, 1),
                                    'refus': refus}))
        return 2

    etape(6, total, 'sauvegarde et ecriture')
    horodatage = f'{datetime.now():%d%m_%H%M}'
    secours = EXTRINSEQUE.with_suffix(f'.avant_{horodatage}.yaml')
    secours.write_text(EXTRINSEQUE.read_text())
    nouvelle = dict(ancienne)
    nouvelle.update({
        'source': (f'calibration_extrinseque_auto.py, 16 coins des '
                   f'{len(utiles)} marqueurs, {len(trames)} trames, exposition '
                   f'{reglage[0]} gain {reglage[1]}, '
                   f'{datetime.now():%d/%m/%Y %H:%M}. Reference : '
                   f'{REFERENCE.name} (positions reelles de la planche).'),
        'camera_index': int(index),
        'intrinsics_stem': stem,
        'reprojection_rms_px': round(rms, 4),
        'marker_size_mesure_mm': round(float(np.mean(list(cotes.values()))), 3),
        'marker_size_yaml_mm': 50.0,
        'markers_used': [int(i) for i in utiles],
        'resolution': [LARGEUR, HAUTEUR],
        'T_cam_world': [[float(x) for x in r] for r in T],
        'validation_leave_one_out_mm': {int(i): round(pires[i], 2) for i in utiles},
        'deplacement_constate': (
            f'position precedente ({cam_avant[0]:.1f}, {cam_avant[1]:.1f}, '
            f'{cam_avant[2]:.1f}) mm ; nouvelle ({cam[0]:.1f}, {cam[1]:.1f}, '
            f'{cam[2]:.1f}) mm ; deplacement {deplacement:.1f} mm.'),
        'sauvegarde': secours.name,
    })
    EXTRINSEQUE.write_text(yaml.safe_dump(nouvelle, sort_keys=False,
                                          allow_unicode=True))
    dis('INFO', f'ecrite -> {EXTRINSEQUE.name} (ancienne : {secours.name})')
    dis('RESULTAT', json.dumps({'mode': 'recalibration', 'ecrite': True,
                                'rms_px': round(rms, 3),
                                'pire_mm': round(pire, 2),
                                'deplacement_mm': round(deplacement, 1),
                                'marqueurs': [int(i) for i in utiles],
                                'luminance': round(luminance, 1),
                                'sauvegarde': secours.name}))
    return 0


def main():
    a = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument('--reference', action='store_true',
                   help='relever la position actuelle de la planche')
    a.add_argument('--controle', action='store_true',
                   help='mesurer l ecart aux marqueurs sans rien ecrire')
    a.add_argument('--frames', type=int, default=5)
    a.add_argument('--force', action='store_true',
                   help='ecrire meme si la validation echoue (a eviter)')
    args = a.parse_args()

    debut = time.time()
    index = index_arducam()
    dis('INFO', f'arducam sur /dev/video{index}')
    code = 0
    if args.controle:
        code = controle(index, args.frames)
    elif args.reference:
        ecrit_reference(index, args.frames)
    else:
        code = recalibre(index, args.frames, args.force)
    dis('DUREE', round(time.time() - debut, 1))
    return code


if __name__ == '__main__':
    sys.exit(main())
