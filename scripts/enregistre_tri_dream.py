#!/usr/bin/env python3
"""Tri reel des pieces (tri_fluide, inchange) + enregistrement pour DREAM.

    /usr/bin/python3 scripts/enregistre_tri_dream.py            # tri complet, enregistre
    /usr/bin/python3 scripts/enregistre_tri_dream.py --sans-tri # bras immobile, N secondes

Pendant que `tri_fluide` trie, deux fils enregistrent : les images arducam et
SVPRO (640x480, ~5 Hz chacune) et les codeurs (~8 Hz). DREAM n'est PAS lance
ici : `analyse_tri_dream.py` (venv_dream) le passe hors ligne sur les images.

Le pont de la Pi est mono-client : les codeurs sont lus sur la MEME connexion
que le tri, sous un verrou. L'arducam est rendue pendant chaque `localise`, qui
l'ouvre pour yolo26.
"""
import argparse
import json
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE / 'scripts'))
import calibration_extrinseque_auto as cea  # noqa: E402
import pick_fsm as fsm  # noqa: E402
import tri_fluide  # noqa: E402

PERIODE_IMAGE_S = 0.2
PERIODE_CODEURS_S = 0.12
EXPOSITION_ARDUCAM = 75


def index_v4l2(motif):
    sortie = subprocess.run(['v4l2-ctl', '--list-devices'], capture_output=True, text=True).stdout
    carte = None
    for ligne in sortie.splitlines():
        if not ligne.startswith('\t'):
            carte = ligne
        elif carte and motif in carte and '/dev/video' in ligne:
            return int(ligne.strip().replace('/dev/video', ''))
    raise SystemExit(f'camera {motif} absente')


class Camera(threading.Thread):
    def __init__(self, nom, motif, dossier, exposition=None):
        super().__init__(daemon=True)
        self.nom, self.motif, self.exposition = nom, motif, exposition
        self.dossier = dossier / nom
        self.dossier.mkdir(parents=True)
        self.actif = threading.Event()
        self.libre = threading.Event()
        self.fin = threading.Event()
        self.actif.set()
        self.journal = []

    def _ouvre(self):
        index = index_v4l2(self.motif)
        if self.exposition is not None:
            cea.regle(index, self.exposition, 0)
        cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        return cap

    def run(self):
        while not self.fin.is_set():
            self.actif.wait()
            if self.fin.is_set():
                break
            cap = self._ouvre()
            self.libre.clear()
            prochain = time.monotonic()
            while self.actif.is_set() and not self.fin.is_set():
                ok, image = cap.read()
                t = time.monotonic()
                if ok and t >= prochain:
                    nom = f'{len(self.journal):05d}.jpg'
                    cv2.imwrite(str(self.dossier / nom), image, [cv2.IMWRITE_JPEG_QUALITY, 95])
                    self.journal.append({'t': t, 'image': nom})
                    prochain = t + PERIODE_IMAGE_S
            cap.release()
            self.libre.set()

    def pause(self):
        self.actif.clear()
        self.libre.wait(10)

    def reprend(self):
        self.actif.set()

    def arrete(self):
        self.fin.set()
        self.actif.set()
        self.join(10)


class Codeurs(threading.Thread):
    def __init__(self, pont, verrou):
        super().__init__(daemon=True)
        self.pont, self.verrou = pont, verrou
        self.fin = threading.Event()
        self.journal = []

    def run(self):
        while not self.fin.is_set():
            with self.verrou:
                rep = self.pont._envoie_brut('get_angles')
            t = time.monotonic()
            if 'ANGLES' in rep.upper():
                try:
                    q = json.loads(rep.split(':', 1)[-1].strip())
                    if len(q) == 6:
                        self.journal.append({'t': t, 'q': q})
                except (json.JSONDecodeError, ValueError):
                    pass
            time.sleep(PERIODE_CODEURS_S)


def partage_le_pont(verrou):
    """Le tri et le fil codeurs parlent sur la meme connexion, chacun son tour."""
    brut = fsm.Pont.envoie

    def envoie(self, action, **kw):
        with verrou:
            return brut(self, action, **kw)

    fsm.Pont._envoie_brut = brut
    fsm.Pont.envoie = envoie


def main():
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument('--sans-tri', type=float, metavar='S',
                   help='ne pas trier : enregistrer S secondes bras immobile')
    args = a.parse_args()

    dossier = RACINE / 'results' / 'dream_reel' / datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
    dossier.mkdir(parents=True)
    verrou = threading.RLock()
    partage_le_pont(verrou)

    cams = [Camera('arducam', 'Arducam', dossier, EXPOSITION_ARDUCAM),
            Camera('svpro', '5MP', dossier)]
    arducam = cams[0]

    creer_pont = fsm.Pont.__init__
    codeurs = []

    def init_pont(self, *k, **kw):
        creer_pont(self, *k, **kw)
        c = Codeurs(self, verrou)
        c.start()
        codeurs.append(c)

    fsm.Pont.__init__ = init_pont

    localise = tri_fluide.localise

    def localise_sans_arducam():
        arducam.pause()
        try:
            return localise()
        finally:
            arducam.reprend()

    tri_fluide.localise = localise_sans_arducam

    for c in cams:
        c.start()
    print('enregistrement dans', dossier)
    try:
        if args.sans_tri:
            fsm.Pont()
            time.sleep(args.sans_tri)
        else:
            sys.argv = [sys.argv[0], '--toutes']
            tri_fluide.main()
    finally:
        for c in codeurs:
            c.fin.set()
            c.join(5)
        for c in cams:
            c.arrete()
        (dossier / 'session.json').write_text(json.dumps({
            'codeurs': [e for c in codeurs for e in c.journal],
            'cameras': {c.nom: c.journal for c in cams},
            'tri': not args.sans_tri,
        }))
        print('images :', {c.nom: len(c.journal) for c in cams},
              ' codeurs :', sum(len(c.journal) for c in codeurs))
        print('ecrit', dossier)


if __name__ == '__main__':
    main()
