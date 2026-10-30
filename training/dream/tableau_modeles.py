#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Tableau comparatif complet : chaque modele sur chaque jeu.

Reunit ce qu'`evaluate_dream.py` mesure (taux de detection, erreur pixel) et ce
qu'il ne mesure pas :

  - le taux de detection PAR KEYPOINT, format historique du projet
    (base/link1/link2 = 100 %, link4 = 89,8 %, ...) ;
  - le SUIVI AU DECALAGE — on translate l'image de N px, la detection doit
    suivre de N px. Aucune verite terrain n'intervient, donc ce chiffre est
    insensible aux erreurs de calibration ; c'est le seul qui distingue un
    reseau qui LOCALISE d'un reseau qui recite une position apprise. Mesure du
    03/09 : les modeles anterieurs suivent 0 % sur `base`.

Usage (venv_dream) :
    python training/dream/tableau_modeles.py
    python training/dream/tableau_modeles.py --n 400 --modeles a,b --jeux x,y
"""
import argparse
import glob
import json
import math
import os
import sys

import cv2
import numpy as np
from PIL import Image as PILImage

sys.path.insert(0, '/tmp/DREAM')
import dream                                                       # noqa: E402

ICI = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, ICI)
from mycobot_fk import KEYPOINT_NAMES                              # noqa: E402

COURTS = [n.replace('mycobot320_', '') for n in KEYPOINT_NAMES]
SENTINELLE = -900.0

MODELES = ['vgg_ultimate_v4_e50', 'vgg_ultimate_v4_mix_ft_e30',
           'vgg_montage0901_ft_e30', 'vgg_v5_geo_ft_e30']
JEUX = [
    ('real_3cam_val',   'dream_data/real_3cam_val_ndds'),
    ('montage arducam', 'captures/real_montage_0901_arducam_ndds'),
    ('montage svpro',   'captures/real_montage_0901_svpro_ndds'),
    ('vue tenue',       'dream_data/synth_cam_libre_vue_tenue'),
]


def _chemin_modele(nom):
    for base in ('checkpoints_dream', 'output/checkpoints_dream'):
        p = os.path.join(ICI, base, nom, 'best_network.pth')
        if os.path.exists(p):
            return p
    return None


def _trames(dossier, n):
    js = sorted(glob.glob(os.path.join(dossier, '??????.json')))
    if len(js) <= n:
        return js
    pas = len(js) / n
    return [js[int(i * pas)] for i in range(n)]


def _detecte(reseau, img):
    out = []
    for kp in reseau.keypoints_from_image(img)['detected_keypoints']:
        if kp is None or len(kp) < 2 or kp[0] is None:
            out.append(None); continue
        u, v = float(kp[0]), float(kp[1])
        out.append(None if (u < SENTINELLE or v < SENTINELLE) else (u, v))
    return out


def _verite(chemin):
    o = json.load(open(chemin))['objects'][0]
    return [tuple(k['projected_location']) for k in o['keypoints']]


def mesure(reseau, dossier, n, dx):
    vus = np.zeros(7); err = [[] for _ in range(7)]
    ratios = [[] for _ in range(7)]
    M = np.float32([[1, 0, dx], [0, 1, 0]])
    trames = _trames(dossier, n)
    for i, jf in enumerate(trames):
        chemin = jf.replace('.json', '.rgb.png')
        if not os.path.exists(chemin):
            continue
        pil = PILImage.open(chemin).convert('RGB')
        a = _detecte(reseau, pil)
        vt = _verite(jf)
        for k in range(7):
            if a[k] is None:
                continue
            vus[k] += 1
            err[k].append(math.hypot(a[k][0] - vt[k][0], a[k][1] - vt[k][1]))
        # le decalage coute une passe de plus : on l'applique sur une trame sur
        # deux, sa dispersion est faible et 200 mesures suffisent largement
        if i % 2:
            continue
        brut = np.array(pil)
        h, w = brut.shape[:2]
        b = _detecte(reseau, PILImage.fromarray(
            cv2.warpAffine(brut, M, (w, h), borderMode=cv2.BORDER_REPLICATE)))
        for k in range(7):
            if a[k] is None or b[k] is None:
                continue
            ratios[k].append((b[k][0] - a[k][0]) / dx)
    n_tr = len(trames)
    return {
        'det': 100 * vus / n_tr,
        'err': [float(np.median(e)) if e else float('nan') for e in err],
        'suivi': [100 * float(np.median(r)) if r else float('nan') for r in ratios],
        'n': n_tr,
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument('--n', type=int, default=300)
    p.add_argument('--decalage', type=int, default=30)
    p.add_argument('--modeles', default=','.join(MODELES))
    p.add_argument('--jeux', default='')
    a = p.parse_args()
    modeles = [m for m in a.modeles.split(',') if m]
    jeux = JEUX if not a.jeux else [(j, j) for j in a.jeux.split(',') if j]

    res = {}
    for m in modeles:
        poids = _chemin_modele(m)
        if poids is None:
            print(f'  {m} : introuvable, ignore'); continue
        reseau = dream.create_network_from_config_file(
            poids.replace('.pth', '.yaml'), poids)
        reseau.enable_evaluation()
        for etq, d in jeux:
            chemin = d if os.path.isabs(d) else os.path.join(ICI, d)
            if not os.path.isdir(chemin):
                continue
            res[(m, etq)] = mesure(reseau, chemin, a.n, a.decalage)
        del reseau
        import torch; torch.cuda.empty_cache()

    larg = max(len(m) for m in modeles) + 2
    for etq, _ in jeux:
        lignes = [(m, res[(m, etq)]) for m in modeles if (m, etq) in res]
        if not lignes:
            continue
        print(f'\n### {etq}   ({lignes[0][1]["n"]} trames, decalage {a.decalage} px)')
        print(f'  {"modele":{larg}s} {"detect.":>8s} {"err px":>7s} {"suivi":>6s}   '
              + ' '.join(f'{c:>6s}' for c in COURTS))
        for m, r in lignes:
            det = r['det'].mean()
            e = np.nanmedian(r['err'])
            s = np.nanmedian(r['suivi'])
            print(f'  {m:{larg}s} {det:7.1f}% {e:7.2f} {s:5.0f}%   '
                  + ' '.join(f'{v:5.0f}%' for v in r['det']))


if __name__ == '__main__':
    main()
