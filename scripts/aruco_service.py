#!/usr/bin/env python3
"""Detecteur ArUco persistant, a lancer sous `.venv/bin/python`.

`cv2.aruco` fait SEGFAULT l'OpenCV 4.6 du systeme, or le tableau de bord tourne
en Python systeme. Relancer un interpreteur par image coute ~1 s de demarrage
(numpy + cv2) pour 3 ms de detection ; on garde donc un processus ouvert et on
lui passe les images par un tube.

Protocole, une image = un aller-retour :
  entree  : une ligne JSON {"h": ..., "w": ...} puis h*w*3 octets BGR bruts
  sortie  : une ligne JSON {"<id>": [[u,v] x4], ...}
Une entree vide termine le service.
"""
import json
import sys

import cv2
import numpy as np

DICTIONNAIRE = cv2.aruco.DICT_4X4_50


def main():
    detecteur = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(DICTIONNAIRE),
        cv2.aruco.DetectorParameters())
    entree, sortie = sys.stdin.buffer, sys.stdout
    print(json.dumps({'pret': True}), flush=True)
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
        coins, ids, _ = detecteur.detectMarkers(
            cv2.cvtColor(image, cv2.COLOR_BGR2GRAY))
        vus = {} if ids is None else {
            str(int(i)): c.reshape(4, 2).tolist()
            for c, i in zip(coins, ids.flatten())}
        print(json.dumps(vus), file=sortie, flush=True)


if __name__ == '__main__':
    main()
