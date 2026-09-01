#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Fenetre de reglage de la SVPRO — inclinez jusqu'a voir les 4 marqueurs.

Le marqueur 25 sort par le BAS du champ. Aucune correction logicielle n'existe :
`tilt_absolute` est expose mais inerte (0,1 px pour 4 deg commandes),
`zoom_absolute` est deja au minimum, et les six resolutions du capteur
(640x480 a 2592x1944) partagent exactement le meme champ (correlation
0,997-0,999). Le reglage est mecanique.

Les marqueurs de la planche ne bougent JAMAIS : on regle toujours la camera.

Ce que montre la fenetre :
  - chaque marqueur detecte, entoure et nomme ;
  - la BANDE VERTE en bas = ou doit se trouver le bord bas du marqueur le plus
    bas (marge 20-30 px). Au-dessus c'est trop juste, en dessous il sort ;
  - la marge courante, en gros, coloree : rouge trop juste, vert bon.

Usage — python SYSTEME, pas venv_dream (dont l'OpenCV est headless) :
    /usr/bin/python3 scripts/svpro_regler_4_marqueurs.py
Fermer avec la touche q, ou Ctrl+C.
"""
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

RACINE = Path(__file__).resolve().parents[1]
MARQUEURS = yaml.safe_load(
    (RACINE / 'training' / 'calibration' / 'workspace_markers.yaml').read_text())['markers']
ATTENDUS = sorted(MARQUEURS)
MARGE_MIN, MARGE_BONNE = 12.0, 20.0
ECHELLE = 1.6
ROUGE, ORANGE, VERT, BLANC, NOIR = ((60,60,235), (0,165,255), (60,220,60),
                                    (255,255,255), (0,0,0))
FONT = cv2.FONT_HERSHEY_SIMPLEX


def index_svpro():
    sortie = subprocess.run(['v4l2-ctl', '--list-devices'],
                            capture_output=True, text=True).stdout
    for bloc in sortie.split('\n\n'):
        if '5mp' in bloc.split('\n')[0].lower():
            for ligne in bloc.split('\n')[1:]:
                if ligne.strip().startswith('/dev/video'):
                    return int(ligne.strip().removeprefix('/dev/video'))
    raise SystemExit('SVPRO introuvable')


def texte(img, s, xy, couleur, echelle=0.6, epais=2):
    x, y = xy
    for dx, dy in ((-2,0),(2,0),(0,-2),(0,2)):
        cv2.putText(img, s, (x+dx, y+dy), FONT, echelle, NOIR, epais, cv2.LINE_AA)
    cv2.putText(img, s, xy, FONT, echelle, couleur, epais, cv2.LINE_AA)


def main():
    idx = index_svpro()
    dev = f'/dev/video{idx}'
    cap = cv2.VideoCapture(idx, cv2.CAP_V4L2)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
    # focus 40 : mesure du 01/09 apres inclinaison, nettete 417 contre 211 a
    # 90, et 4/4 marqueurs au lieu de 2/4. Autofocus continu coupe.
    for c in ('focus_automatic_continuous=0', 'focus_absolute=40',
              'sharpness=0', 'contrast=1'):
        subprocess.run(['v4l2-ctl', '-d', dev, '--set-ctrl', c], capture_output=True)

    # L'OpenCV du venv_dream est HEADLESS (pas de GTK/Qt) : cette fenetre doit
    # tourner sous le python systeme, cv2 4.6 + Qt5. Son API aruco est
    # l'ancienne (pas de ArucoDetector), d'ou les deux chemins.
    dico = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
    if hasattr(cv2.aruco, 'ArucoDetector'):
        par = cv2.aruco.DetectorParameters()
        par.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        moteur = cv2.aruco.ArucoDetector(dico, par)
        detecte = lambda g: moteur.detectMarkers(g)
    else:
        par = cv2.aruco.DetectorParameters_create()
        par.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        detecte = lambda g: cv2.aruco.detectMarkers(g, dico, parameters=par)

    fenetre = 'SVPRO — inclinez jusqu a 4/4, marge basse 20-30 px  (q pour fermer)'
    cv2.namedWindow(fenetre, cv2.WINDOW_AUTOSIZE)
    stable = 0
    while True:
        ok, img = cap.read()
        if not ok:
            continue
        h, w = img.shape[:2]
        c, ids, _ = detecte(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY))
        vus, bas = {}, 0.0
        if ids is not None:
            for q, m in zip(c, ids.flatten()):
                if int(m) in MARQUEURS:
                    vus[int(m)] = q.reshape(4, 2)
                    bas = max(bas, q.reshape(4, 2)[:, 1].max())
        marge = h - bas if vus else 0.0
        complet = len(vus) == 4
        stable = stable + 1 if (complet and marge >= MARGE_BONNE) else 0
        couleur = (VERT if marge >= MARGE_BONNE else
                   ORANGE if marge >= MARGE_MIN else ROUGE)

        v = cv2.resize(img, None, fx=ECHELLE, fy=ECHELLE)
        H, W = v.shape[:2]
        # bande cible : ou doit tomber le bord bas du marqueur le plus bas
        y1 = int((h - MARGE_BONNE) * ECHELLE)
        y2 = int((h - 30.0) * ECHELLE)
        bande = v.copy()
        cv2.rectangle(bande, (0, y2), (W, y1), (60, 200, 60), -1)
        v = cv2.addWeighted(bande, 0.28, v, 0.72, 0)
        cv2.line(v, (0, y1), (W, y1), VERT, 1, cv2.LINE_AA)
        cv2.line(v, (0, y2), (W, y2), VERT, 1, cv2.LINE_AA)
        texte(v, 'zone visee du bord bas', (8, y2 - 8), VERT, 0.5, 1)

        for m, q in vus.items():
            p = (q * ECHELLE).astype(int)
            cv2.polylines(v, [p], True, VERT, 2, cv2.LINE_AA)
            texte(v, str(m), tuple(p.mean(axis=0).astype(int) + [-12, 6]), VERT, 0.7)
        for m in ATTENDUS:
            if m not in vus:
                texte(v, f'{m} NON VU', (12, 150 + 30 * ATTENDUS.index(m)), ROUGE, 0.7)
        if vus:
            yb = int(bas * ECHELLE)
            cv2.line(v, (0, yb), (W, yb), couleur, 2, cv2.LINE_AA)

        bandeau = np.full((78, W, 3), 28, np.uint8)
        texte(bandeau, f'{len(vus)}/4 marqueurs', (12, 32),
              VERT if complet else ROUGE, 0.9, 2)
        texte(bandeau, f'marge basse {marge:5.1f} px', (250, 32), couleur, 0.9, 2)
        if not complet:
            conseil = 'INCLINEZ VERS LE BAS'
        elif marge < MARGE_MIN:
            conseil = 'TROP JUSTE — encore un peu vers le bas'
        elif marge < MARGE_BONNE:
            conseil = 'presque — un cheveu de plus'
        elif marge > 45:
            conseil = 'trop bas — remontez un peu'
        else:
            conseil = f'BON — stable depuis {stable} lectures, SERREZ'
        texte(bandeau, conseil, (12, 62), couleur, 0.7, 2)
        cv2.imshow(fenetre, np.vstack([bandeau, v]))
        if cv2.waitKey(30) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
