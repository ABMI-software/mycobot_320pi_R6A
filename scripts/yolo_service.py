#!/usr/bin/env python3
"""Detecteur YOLOE-26 persistant pour pick_dashboard, a lancer sous `.venv/bin/python`.

Le dashboard tourne en Python systeme, sans torch : meme principe que
`aruco_service.py`, un processus ouvert qui recoit les images par un tube.

Protocole, une image = un aller-retour :
  entree : une ligne JSON {"h": ..., "w": ...} puis h*w*3 octets BGR bruts
  sortie : une ligne JSON [{"classe": "balle"|"cylindre"|"objet", "nom", "conf", "contour": [[u, v], ...]}]
Une entree vide termine le service.

Trois modeles, mesures le 15/09/2026 (voir `yolo_capture.py`) :
- balle : 26s par le nom (`tennis ball`), 12/12 photos libres ; 0 dans la pince ;
- cylindre : 26l par le nom (`tape roll` / `bottle cap`), 5/5 vues arducam. Le nom
  que donne le mode sans consigne ne suffit pas : `adhesive tape` 3 fois sur 5,
  puis `beeper`, puis `opal` — le meme nom que la balle ;
- objet : 26l sans consigne, image agrandie a 960 px. A 640 px les petits objets
  vus de dessus sortaient a 0,52-0,66, a 960 px a 0,84-0,95, pour 32 ms.
Aucun filtre ici : planche, bras, balle et cartons sont juges par le dashboard,
qui a les angles lus et l'extrinseque.
"""
import contextlib
import json
import sys

# Le protocole passe par stdout : tout ce que les bibliotheques ecrivent part sur stderr.
SORTIE = sys.stdout
sys.stdout = sys.stderr

import numpy as np  # noqa: E402
from ultralytics import YOLOE  # noqa: E402

from yolo_capture import INVITES, MODELE_OBJETS, POIDS, SEUIL_OBJET  # noqa: E402

TAILLE_OBJETS = 960


def contour(resultat, i):
    polygone = resultat.masks.xy[i]
    if len(polygone) >= 3:
        return np.round(polygone).astype(int).tolist()
    x0, y0, x1, y1 = (int(v) for v in resultat.boxes.xyxy[i])
    return [[x0, y0], [x1, y0], [x1, y1], [x0, y1]]


def par_le_nom(classe):
    fichier, seuil, noms = INVITES[classe]
    modele = YOLOE(str(POIDS / fichier))
    with contextlib.chdir(POIDS):
        modele.set_classes(list(noms))
    return classe, modele, seuil, noms


def main():
    nommes = [par_le_nom('balle'), par_le_nom('cylindre')]
    tout = YOLOE(str(POIDS / MODELE_OBJETS))
    # La premiere inference compile et alloue : plusieurs secondes, a payer avant « pret ».
    vide = np.zeros((480, 640, 3), np.uint8)
    for _, modele, _, _ in nommes:
        modele.predict(vide, verbose=False)
    tout.predict(vide, imgsz=TAILLE_OBJETS, verbose=False)
    print(json.dumps({'pret': True}), file=SORTIE, flush=True)

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
        rendu = []
        for classe, modele, seuil, noms in nommes:
            r = modele.predict(image, conf=seuil, verbose=False)[0]
            rendu += [{'classe': classe, 'nom': noms[int(b.cls)], 'conf': float(b.conf),
                       'contour': contour(r, i)} for i, b in enumerate(r.boxes)]
        r = tout.predict(image, conf=SEUIL_OBJET, imgsz=TAILLE_OBJETS, verbose=False)[0]
        rendu += [{'classe': 'objet', 'nom': r.names[int(b.cls)], 'conf': float(b.conf),
                   'contour': contour(r, i)} for i, b in enumerate(r.boxes)]
        print(json.dumps(rendu), file=SORTIE, flush=True)


if __name__ == '__main__':
    main()
