#!/usr/bin/env python3
"""Detecteur yolo26 ENTRAINE (8 pieces peintes) pour le dashboard, sous `.venv/bin/python`.

Meme protocole que `yolo_service.py`, meme raison d'etre : le dashboard tourne
en Python systeme sans torch, le modele tourne ici, les deux se parlent par un
tube.

  entree : une ligne JSON {"h": ..., "w": ..., "camera": "arducam"|"svpro"}
           puis h*w*3 octets BGR bruts
  sortie : une ligne JSON [{"classe": "<une des 8>", "conf": ..., "boite": [x0, y0, x1, y1]}]
Une entree vide termine le service.

Ce qui differe de `yolo_service.py`, et pourquoi :

* **un seul modele, entraine, qui NOMME la piece.** YOLOE devinait un nom parmi
  4 585 et la teinte devait le corriger ; ici la classe fait foi (`tri_couleur`
  § `pieces`, regime « modele ENTRAINE »).
* **pas de masque.** Le modele est un detecteur de boites : le contour est
  decoupe cote dashboard dans le masque de la COULEUR de la classe, par
  `tri_couleur.contour_ou_boite` — la fonction qui porte deja les mesures.
* **seuil 0,10, pas 0,20.** L'echelle de confiance d'un modele entraine n'est
  pas celle de YOLOE : mesure du 18/09, les 8 pieces sortent 6 fois sur 6 mais
  le cube rouge a 0,13. A 0,20 il disparaissait (`tri_couleur.SEUIL_MODELE_ENTRAINE`).
* **yolo26 est SANS NMS** : ses quasi-doublons sont retires par recouvrement,
  avec le meme seuil que `tri_couleur.RECOUVREMENT_DOUBLON`.

Le champ `camera` n'est pas lu par le modele — il est journalise pour que le
service dise sur quelle vue il travaille quand on le lance a la main.
"""
import json
import os
import sys
from pathlib import Path

# Le protocole passe par stdout : tout ce que les bibliotheques ecrivent part sur stderr.
SORTIE = sys.stdout
sys.stdout = sys.stderr

import numpy as np  # noqa: E402

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE / 'scripts'))

import tri_couleur as tc  # noqa: E402

# `pieces_v5` : 74 images, 44 arducam et 30 SVPRO, dispositions variees, contre
# 13 + 4 pour `pieces_v3`. Mesure du 22/09 sur des trames fraiches, les deux
# modeles trouvant les 8 classes a chaque fois — c'est la confiance MINIMALE qui
# separe, celle du `cube_rouge` qui s'effacait :
#
#              mediane        minimum
#   arducam   0,84 -> 0,92   0,14 -> 0,60
#   SVPRO     0,91 -> 0,98   0,29 -> 0,64
#
# Le mAP des deux runs ne vaut rien : `train` et `val` pointent le meme dossier.
# Les runs `v4*` reprenaient pieces_v3 avec d'autres augmentations : a recette
# egale l'ecart du seul tirage (0,186) ecrase celui de la recette (0,035), donc
# le levier etait bien le jeu d'images.
#
# `pieces_v6c_gazebo` (01/10) : v5 affine sur les 74 reelles x10 + 1 036 images
# Gazebo des 4 cameras DREAM. Gazebo, 10 graines de test : 283/283 sur les 4
# cameras (v5 : 196/283), bac_jaune top 10/10 (v5 : 4/10). Test reel neuf
# (training/yolo/test_reel_0110) : mAP50 0,990 (v5 : 0,967). Les reelles x5
# (v6b) ne suffisaient pas : le bac rouge reel qui deborde de la planche etait
# coupe a la limite du bois. Detail : docs/PROTOCOLE_YOLO_GAZEBO.md.
POIDS_DEFAUT = (RACINE / 'runs' / 'detect' / 'training' / 'yolo' / 'runs'
                / 'pieces_v6c_gazebo_yolo26s' / 'weights' / 'best.pt')
TAILLE = 640
# Coins tronques a l entier : -0,5 px en moyenne sur le centre (mesure 01/10 :
# -0,47 / -0,51 px, +-0,2). Le banc reel est cale avec ; la simulation demande
# les coins exacts (tri_yolo.launch.py).
BOITES_PRECISES = os.environ.get('YOLO26_BOITES_PRECISES') == '1'


def detecte(modele, image, seuil):
    """[(nom, (x0, y0, x1, y1), conf)] sans doublon, la plus sure d'abord."""
    r = modele.predict(image, conf=seuil, imgsz=TAILLE, verbose=False)[0]
    gardes = []
    for bt in sorted(r.boxes, key=lambda b: -float(b.conf)):
        boite = tuple(float(v) if BOITES_PRECISES else int(v) for v in bt.xyxy[0])
        if any(tc.recouvre(boite, g[1]) > tc.RECOUVREMENT_DOUBLON for g in gardes):
            continue
        gardes.append((modele.names[int(bt.cls)], boite, float(bt.conf)))
    return gardes


def main():
    poids = Path(sys.argv[1]) if len(sys.argv) > 1 else POIDS_DEFAUT
    seuil = float(sys.argv[2]) if len(sys.argv) > 2 else tc.SEUIL_MODELE_ENTRAINE
    if not poids.exists():
        print(json.dumps({'erreur': f'poids introuvables : {poids}'}), file=SORTIE, flush=True)
        return
    from ultralytics import YOLO
    modele = YOLO(str(poids))
    noms = set(modele.names.values())
    if not noms <= set(tc.CLASSES_PIECES):
        print(json.dumps({'erreur': f'classes inattendues : {sorted(noms)}'}),
              file=SORTIE, flush=True)
        return
    # La premiere inference compile et alloue : plusieurs secondes, a payer avant « pret ».
    modele.predict(np.zeros((480, 640, 3), np.uint8), imgsz=TAILLE, verbose=False)
    print(json.dumps({'pret': True, 'poids': poids.name, 'chemin': str(poids.resolve()), 'seuil': seuil,
                      'classes': list(modele.names.values())}), file=SORTIE, flush=True)

    entree = sys.stdin.buffer
    while True:
        ligne = entree.readline()
        if not ligne:
            return
        forme = json.loads(ligne)
        taille = forme['h'] * forme['w'] * 3
        octets = entree.read(taille)
        if len(octets) < taille:
            return
        image = np.frombuffer(octets, np.uint8).reshape(forme['h'], forme['w'], 3)
        print(json.dumps([{'classe': nom, 'conf': conf, 'boite': list(boite)}
                          for nom, boite, conf in detecte(modele, image, seuil)]),
              file=SORTIE, flush=True)


if __name__ == '__main__':
    main()
