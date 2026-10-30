#!/usr/bin/env python3
"""Pieces et bacs vus par l'arducam, en mm repere base, dans results/tri_fluide/derniere_vue.json.

    .venv/bin/python scripts/yolo26_localise.py

yolo26 donne la boite en 2D ; l'extrinseque arducam donne le rayon de son
centre, coupe au plan de la mi-hauteur de la piece (dossier de fabrication,
`tri_couleur.OBJET_PAR_COULEUR`) ou au rebord du bac. Lecture seule : aucun
mouvement, rien d'ecrit hors de l'image de controle.
"""
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml
from ultralytics import YOLO

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE / 'scripts'))
import calibration_extrinseque_auto as cea  # noqa: E402
import calibrer_extrinseque_4aruco as base  # noqa: E402
import tri_couleur as tc  # noqa: E402
import yolo26_service  # noqa: E402

SEUIL = 0.5
# Les bacs sont aussi rendus sous ce seuil, marques `faible` : `tri_fluide` ne les
# accepte que pres d'une position deja vue sure (un bac ne bouge pas).
SEUIL_BAC_FAIBLE = 0.10
# yolo26 hesite d'une image a l'autre sur le bac jaune vide (0,13 puis < 0,10,
# 13/10) : on juge plusieurs images et on garde la meilleure detection.
IMAGES = 5
IMAGE_CONTROLE = RACINE / 'results' / 'tri_fluide' / 'derniere_vue_arducam.png'
# `calibration_extrinseque_auto` detourne la sortie standard : le resultat passe par un fichier.
SORTIE = RACINE / 'results' / 'tri_fluide' / 'derniere_vue.json'


def hauteurs_mm():
    z = {f'bac_{c}': tc.BAC[2] for c in tc.TEINTES}
    z.update({tc.NOM_PIECE[c]: poses[0][2] / 2.0 for c, (_, poses) in tc.OBJET_PAR_COULEUR.items()})
    return z


def main():
    ext = yaml.safe_load(cea.EXTRINSEQUE.read_text())
    T = np.array(ext['T_cam_world'], float)
    K, dist = cea.registre.load_intrinsics(ext['intrinsics_stem'])
    index = cea.index_arducam()
    cea.regle(index, 75, 0)
    images = cea.capture(index, IMAGES)
    image = images[-1]
    IMAGE_CONTROLE.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(IMAGE_CONTROLE), image)

    z = hauteurs_mm()
    vus = {}
    faibles = {}
    modele = YOLO(str(yolo26_service.POIDS_DEFAUT))
    for r in (r for im in images for r in modele(im, conf=SEUIL_BAC_FAIBLE, verbose=False)):
        for b, c, k in zip(r.boxes.xyxy.cpu().numpy(), r.boxes.conf.cpu().numpy(),
                           r.boxes.cls.cpu().numpy()):
            nom = r.names[int(k)]
            if nom not in z or (c < SEUIL and not nom.startswith('bac_')):
                continue
            uv = ((b[0] + b[2]) / 2.0, (b[1] + b[3]) / 2.0)
            p = base.vers_plan(uv, K, dist, T, z[nom] / 1000.0)[:2] * 1000.0
            vu = {'xy': [round(float(p[0]), 1), round(float(p[1]), 1)],
                  'conf': round(float(c), 3), 'z_mm': z[nom]}
            cible = vus if c >= SEUIL else faibles.setdefault(nom, [])
            if c >= SEUIL:
                if nom not in vus or vus[nom]['conf'] < c:
                    vus[nom] = vu
            else:
                cible.append(vu)
    SORTIE.write_text(json.dumps({'surs': vus, 'faibles': faibles}))


if __name__ == '__main__':
    main()
