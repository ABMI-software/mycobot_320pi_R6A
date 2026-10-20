#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Veille d'entrainement DREAM — trois tests par checkpoint, une ligne chacun.

Surveille un dossier de checkpoints et evalue chaque `.pth` des qu'il apparait.
N'ecrit ni ne modifie rien dans le dossier d'entrainement : le script
d'entrainement n'a pas a etre touche.

POURQUOI TROIS TESTS ET PAS UN. Le defaut qu'on corrige est que le reseau
memorise la position des keypoints proximaux au lieu de les localiser : mesure
du 02/09, `base` vaut 211,7 / 342,9 px sur l'arducam avec sigma <= 0,02 px, a
deux jours d'intervalle. Un modele qui memorise obtient un score PARFAIT sur le
montage ou il a ete affine. Le suivre seul ne prouve donc rien.

  montage   — non-regression. Dit qu'on n'a pas casse ce qui marchait.
              Ne peut PAS dire qu'on a corrige quoi que ce soit.
  decalage  — le test qui tranche. On translate l'image de N px : la detection
              doit suivre de N px. Mesure du 02/09 sur le modele actuel :
              elle suit 0 %. Passer de 0 % a ~100 % est le signal que la
              memorisation est morte, independamment du reste.
  vue tenue — generalisation. Un secteur de points de vue jamais vu a
              l'entrainement. C'est ce qui manquait aux validations
              precedentes : les trames ecartees etaient separees en espace
              ARTICULAIRE mais venaient du meme point de vue, donc aucune ne
              pouvait detecter la memorisation du cadrage.

Usage (venv_dream) :
    python training/dream/veille_entrainement.py \
        --checkpoints training/dream/checkpoints_dream/<run> \
        --montage training/dream/captures/real_montage_0901_arducam_ndds \
        --vue-tenue training/dream/dream_data/<jeu>_vue_tenue \
        --csv training/dream/output/veille_<run>.csv
"""
import argparse
import csv
import glob
import math
import os
import sys
import time

import cv2
import numpy as np
from PIL import Image as PILImage

sys.path.insert(0, '/tmp/DREAM')
import dream  # noqa: E402

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
from mycobot_fk import KEYPOINT_NAMES  # noqa: E402

SENTINELLE = -900.0
# `mycobot320_base` -> `base` : le prefixe est le meme partout et mange la
# largeur de la ligne, ou seul le suffixe distingue les keypoints.
COURTS = [n.replace('mycobot320_', '') for n in KEYPOINT_NAMES]


def _trames(dossier, n):
    """Echantillonnage REGULIER, pas les n premieres.

    Les captures sont ordonnees par trajectoire : prendre le debut ne montrerait
    qu'un coin de l'espace articulaire.
    """
    js = sorted(glob.glob(os.path.join(dossier, '??????.json')))
    if not js:
        return []
    if len(js) <= n:
        return js
    pas = len(js) / n
    return [js[int(i * pas)] for i in range(n)]


def _detecte(reseau, image_pil):
    """Rend une liste de (u, v) en coordonnees IMAGE BRUTE, ou None."""
    res = reseau.keypoints_from_image(image_pil, image_preprocessing_override=None,
                                      debug=False)
    sortie = []
    for kp in res['detected_keypoints']:
        if kp is None or len(kp) < 2 or kp[0] is None:
            sortie.append(None)
            continue
        u, v = float(kp[0]), float(kp[1])
        sortie.append(None if (u < SENTINELLE or v < SENTINELLE) else (u, v))
    return sortie


def _verite(chemin_json):
    import json
    with open(chemin_json) as f:
        d = json.load(f)
    objets = d['objects'][0]
    return [tuple(p) for p in objets['keypoints_projected'][:len(KEYPOINT_NAMES)]] \
        if 'keypoints_projected' in objets \
        else [tuple(k['projected_location']) for k in objets['keypoints']]


def test_detection(reseau, dossier, n):
    """Taux de detection et erreur pixel mediane contre la verite terrain."""
    par_kp = [[] for _ in KEYPOINT_NAMES]
    detectes = []
    for jf in _trames(dossier, n):
        img = jf.replace('.json', '.rgb.png')
        if not os.path.exists(img):
            continue
        kps = _detecte(reseau, PILImage.open(img).convert('RGB'))
        vt = _verite(jf)
        nd = 0
        for i, kp in enumerate(kps[:len(vt)]):
            if kp is None:
                continue
            nd += 1
            par_kp[i].append(math.hypot(kp[0] - vt[i][0], kp[1] - vt[i][1]))
        detectes.append(nd)
    if not detectes:
        return None
    medianes = [float(np.median(e)) if e else float('nan') for e in par_kp]
    toutes = [e for liste in par_kp for e in liste]
    return (float(np.mean(detectes)),
            float(np.median(toutes)) if toutes else float('nan'),
            len(detectes), medianes)


def test_decalage(reseau, dossier, n, dx, dy):
    """Translate l'image de (dx, dy) : la detection doit suivre de (dx, dy).

    Le ratio est la projection du deplacement mesure sur le decalage applique,
    donc 1,0 = suit parfaitement, 0,0 = position memorisee, insensible a
    l'image. On ne compte un keypoint que s'il est detecte dans LES DEUX
    images — un keypoint perdu par la translation ne dit rien sur la
    memorisation.

    Bord replique plutot que noir : une bande noire est un signal que le reseau
    n'a jamais vu a l'entrainement, et l'echec viendrait alors du bord et non de
    la memorisation. L'interieur, ou sont les keypoints, est exact.
    """
    s = np.array([dx, dy], float)
    s2 = float(s @ s)
    M = np.float32([[1, 0, dx], [0, 1, dy]])
    ratios = [[] for _ in KEYPOINT_NAMES]
    for jf in _trames(dossier, n):
        chemin = jf.replace('.json', '.rgb.png')
        if not os.path.exists(chemin):
            continue
        pil = PILImage.open(chemin).convert('RGB')
        brut = np.array(pil)
        h, w = brut.shape[:2]
        decale = cv2.warpAffine(brut, M, (w, h), borderMode=cv2.BORDER_REPLICATE)
        a = _detecte(reseau, pil)
        b = _detecte(reseau, PILImage.fromarray(decale))
        for i in range(min(len(a), len(b), len(KEYPOINT_NAMES))):
            if a[i] is None or b[i] is None:
                continue
            d = np.array([b[i][0] - a[i][0], b[i][1] - a[i][1]], float)
            ratios[i].append(float(d @ s) / s2)
    medianes = [float(np.median(r)) if r else float('nan') for r in ratios]
    appariees = [len(r) for r in ratios]
    valides = [m for m in medianes if not math.isnan(m)]
    return medianes, appariees, (float(np.median(valides)) if valides else float('nan'))


def evalue(chemin_pth, args):
    config = os.path.splitext(chemin_pth)[0] + '.yaml'
    if not os.path.exists(config):
        return None
    reseau = dream.create_network_from_config_file(config, chemin_pth)
    reseau.enable_evaluation()

    lignes = {}
    for dossier in args.montage:
        lignes['montage:' + os.path.basename(dossier)] = test_detection(
            reseau, dossier, args.n)
    if args.vue_tenue:
        lignes['vue_tenue'] = test_detection(reseau, args.vue_tenue, args.n)
    source = args.montage[0] if args.montage else args.vue_tenue
    decalage = test_decalage(reseau, source, args.n_decalage,
                             args.decalage, 0) if source else None
    del reseau
    import torch
    torch.cuda.empty_cache()
    return lignes, decalage


def formate(nom, lignes, decalage, args):
    bouts = [f'{nom:>14s}']
    for cle, r in lignes.items():
        if r is None:
            bouts.append(f'{cle} AUCUNE TRAME')
            continue
        moy, err, nb, medianes = r
        detail = ' '.join(f'{c}={m:.1f}' if not math.isnan(m) else f'{c}=--'
                          for c, m in zip(COURTS, medianes))
        bouts.append(f'{cle} {moy:.2f}/{len(KEYPOINT_NAMES)} kp  err {err:5.1f} px '
                     f'(n={nb})  [{detail}]')
    if decalage is not None:
        medianes, appariees, globale = decalage
        detail = ' '.join(
            f'{c}={m*100:.0f}%' if not math.isnan(m) else f'{c}=--'
            for c, m in zip(COURTS, medianes))
        bouts.append(f'decalage {args.decalage}px suivi {globale*100:.0f}%  [{detail}]')
    return '  |  '.join(bouts)


def main():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument('--checkpoints', required=True,
                   help='dossier surveille, tout *.pth avec son .yaml frere')
    p.add_argument('--montage', default='', help='dossiers NDDS, separes par des virgules')
    p.add_argument('--vue-tenue', default=None, help='dossier NDDS du secteur tenu a l ecart')
    p.add_argument('--n', type=int, default=120, help='trames par test de detection')
    p.add_argument('--n-decalage', type=int, default=60,
                   help='trames du test de decalage (deux passes chacune)')
    p.add_argument('--decalage', type=int, default=30, help='translation en px')
    p.add_argument('--csv', default=None)
    p.add_argument('--intervalle', type=float, default=60.0)
    p.add_argument('--une-fois', action='store_true',
                   help='evalue ce qui est present et sort')
    args = p.parse_args()
    args.montage = [d for d in args.montage.split(',') if d]

    if args.csv:
        os.makedirs(os.path.dirname(os.path.abspath(args.csv)), exist_ok=True)
        if not os.path.exists(args.csv):
            with open(args.csv, 'w', newline='') as f:
                csv.writer(f).writerow(
                    ['horodatage', 'checkpoint', 'jeu', 'kp_moyen', 'err_px_mediane',
                     'n_trames', 'suivi_decalage'] +
                    [f'suivi_{c}' for c in COURTS] +
                    [f'err_{c}' for c in COURTS])

    vus = {}
    while True:
        for pth in sorted(glob.glob(os.path.join(args.checkpoints, '*.pth'))):
            cle = (pth, os.path.getmtime(pth))
            if cle in vus:
                continue
            vus[cle] = True
            # Un checkpoint en cours d'ecriture est illisible : on laisse
            # `torch.load` echouer une fois et on le reprendra au tour suivant,
            # sa mtime aura change.
            try:
                res = evalue(pth, args)
            except Exception as exc:                       # noqa: BLE001
                print(f'{os.path.basename(pth)} : illisible ({exc})', flush=True)
                del vus[cle]
                continue
            if res is None:
                continue
            lignes, decalage = res
            nom = os.path.basename(pth).replace('.pth', '')
            print(formate(nom, lignes, decalage, args), flush=True)
            if args.csv:
                globale = decalage[2] if decalage else float('nan')
                par_kp = decalage[0] if decalage else [float('nan')] * len(KEYPOINT_NAMES)
                with open(args.csv, 'a', newline='') as f:
                    w = csv.writer(f)
                    for jeu, r in lignes.items():
                        nan = float('nan')
                        moy, err, nb, med = r if r else (nan, nan, 0,
                                                         [nan] * len(KEYPOINT_NAMES))
                        w.writerow([time.strftime('%Y-%m-%d %H:%M:%S'), nom, jeu,
                                    round(moy, 3), round(err, 3), nb,
                                    round(globale, 4)] +
                                   [round(v, 4) for v in par_kp] +
                                   [round(v, 2) for v in med])
        if args.une_fois:
            return
        time.sleep(args.intervalle)


if __name__ == '__main__':
    main()
