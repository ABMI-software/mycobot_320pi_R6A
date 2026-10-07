#!/usr/bin/env python3
"""YOLOE-26 dans pick_dashboard, sans toucher a `pick_dashboard.py`.

    /usr/bin/python3 scripts/lancer_pick_dashboard.py --yolo [--inventaire robot=4 scotch=2]

`Vision.objets` de l'ARDUCAM ne vient plus des heuristiques couleur/forme mais de
`yolo_service.py` (.venv, GPU). Le cylindre, reconnu par son nom, devient
`scotch` (petit carton) ; tout autre objet pose sur la planche devient `robot`
(grand carton), avec les hauteurs de prise et le couple du robot — les plus
prudents pour un objet inconnu.

La SVPRO garde les heuristiques : elle ne nomme rien, ses taches heritent du nom
de l'objet arducam le plus proche.

La balle : le detecteur jaune D'ABORD, YOLO quand il ne voit rien (surexposition,
couleur alteree). Le centre de la silhouette YOLO tombe a 0,3-1,4 px du centre
jaune (0,6-3,2 mm, 15/09) : proche, mais la carte de correction a appris sur le
jaune, et `_detecte_balle` refuse des echantillons ecartes de plus de 3 mm —
alterner les deux a chaque image ferait sauter la cible. Dans la pince, seul le
jaune la voit (YOLOE : 0 de confiance).

Le jugement reste celui du dashboard, avec ses propres fonctions : planche,
marqueurs detectes, silhouette du bras depuis les angles lus, part de jaune,
marqueurs des cartons, cotes en mm a HAUTEUR_OBJET, point le plus epais.

YOLO tourne en tache de fond : 59-76 ms par image, plus que les 60 ms du
rafraichissement du dashboard. Chaque appel soumet l'image courante et juge le
dernier resultat termine, sur SA propre image : une image de retard, sans effet
sur des objets poses.
"""
from __future__ import annotations

import atexit
import json
import subprocess
import threading
import time
from pathlib import Path

import cv2
import numpy as np

RACINE = Path(__file__).resolve().parents[1]
SERVICE = RACINE / 'scripts' / 'yolo_service.py'
VENV = RACINE / '.venv' / 'bin' / 'python'

CLASSE_DASHBOARD = {'cylindre': 'scotch', 'objet': 'robot'}
PART_PLATEAU_MIN = 0.5
# Le marqueur 19 sortait `keycard` 0,91, entierement dans son carre DETECTE (0 %
# hors marqueurs), et passait le masque de planche : celui-ci efface les
# marqueurs a leur position projetee par l'extrinseque, decalee (14/09). Les
# vrais objets sont a 100 % hors marqueurs (15/09).
PART_HORS_MARQUEURS_MIN = 0.5
# La balle est jaune a 90 % ; le cylindre noir et vert a 13-32 %, son vert tombe
# dans HSV_BALLE. Le seuil 0,25 de `Vision.objets` le jetait (15/09).
PART_JAUNE_BALLE = 0.6
# Meme regle que `Vision.objets` : un objet pose contre le bras touche son masque.
PART_BRAS_MAX = 0.4
RECOUVREMENT_MAX = 0.5
# Choisi, non mesure : un carton fait ~70 x 110 mm (petit) a ~100 x 126 mm (grand).
# Rien a moins de ca de son marqueur n'est traite en objet a trier.
RAYON_CARTON_MM = 130.0
# `_detecte_objet` lit la liste d'objets de la DERNIERE image traitee, sans
# attendre. YOLO ayant une image de retard, le bras a peine degage serait juge
# sur une image ou il masquait encore la planche : objet « invisible depuis
# toutes les poses », donc oublie. Le fil du robot attend un resultat pris APRES
# son appel, puis deux rafraichissements du dashboard (60 ms chacun).
PATIENCE_FRAICHE_S = 1.0
ATTENTE_RAFRAICHI_S = 0.12
# La balle YOLO vient de l'image PRECEDENTE : au-dela, le bras a pu la couvrir depuis.
FRAICHEUR_BALLE_S = 0.5
PART_PLANCHE_BALLE_MIN = 0.5


class ServiceYOLO:
    """`yolo_service.py` en tache de fond : `soumet` ne bloque jamais."""

    def __init__(self, commande=None):
        self.proc = subprocess.Popen(commande or [str(VENV), str(SERVICE)],
                                     stdin=subprocess.PIPE, stdout=subprocess.PIPE,
                                     cwd=str(RACINE / 'scripts'))
        if not self.proc.stdout.readline():
            raise RuntimeError('yolo_service.py n a pas demarre — poids sous weights/yoloe/ ?')
        self.arrete = False
        self._verrou = threading.Lock()
        self._reveil = threading.Event()
        self._a_traiter = None
        self._dernier = None
        threading.Thread(target=self._boucle, daemon=True).start()

    def soumet(self, image):
        with self._verrou:
            self._a_traiter = (image.copy(), time.time())
        self._reveil.set()

    def dernier(self):
        """(image, detections, instant de l'image) du dernier aller-retour termine, ou None."""
        with self._verrou:
            return self._dernier

    def _boucle(self):
        while not self.arrete:
            self._reveil.wait()
            with self._verrou:
                soumise, self._a_traiter = self._a_traiter, None
                self._reveil.clear()
            if soumise is None:
                continue
            image, instant = soumise
            detections = self._aller_retour(image)
            with self._verrou:
                if detections is None:
                    self.arrete = True
                else:
                    self._dernier = (image, detections, instant)

    def _aller_retour(self, image):
        h, w = image.shape[:2]
        try:
            self.proc.stdin.write((json.dumps({'h': h, 'w': w}) + '\n').encode())
            self.proc.stdin.write(np.ascontiguousarray(image).tobytes())
            self.proc.stdin.flush()
            ligne = self.proc.stdout.readline()
        except (BrokenPipeError, OSError, ValueError):
            return None
        return json.loads(ligne) if ligne else None

    def ferme(self):
        self.arrete = True
        self._reveil.set()
        if self.proc.poll() is None:
            self.proc.stdin.close()
            self.proc.wait(timeout=5)


def juge(vision, image, detections, module, angles=None, marqueurs=None):
    """[(classe dashboard, xy base mm, contour)] — ce que rendait `Vision.objets`."""
    forme = image.shape[:2]
    plateau = vision.masque_plateau(forme)
    silhouette = vision.masque_bras(forme, angles) if angles is not None else None
    jaune = cv2.inRange(cv2.cvtColor(image, cv2.COLOR_BGR2HSV), *module.HSV_BALLE)
    cartons = [np.asarray(xy, float) for xy, _, _ in vision.cartons_marques(image, marqueurs).values()]
    gardes, pleins = [], []
    for d in sorted(detections, key=lambda d: (d['classe'] != 'cylindre', -d['conf'])):
        contour = np.asarray(d['contour'], np.int32).reshape(-1, 1, 2)
        _, grand = vision._cotes_mm(cv2.convexHull(contour), module.HAUTEUR_OBJET)
        if grand > module.COTE_ROBOT_MM[1]:
            continue
        plein = np.zeros(forme, np.uint8)
        cv2.fillPoly(plein, [contour], 255)
        aire = np.count_nonzero(plein)
        if (aire < module.AIRE_OBJET_MIN
                or np.count_nonzero(cv2.bitwise_and(plein, plateau)) < PART_PLATEAU_MIN * aire
                or np.count_nonzero(vision.sans_marqueurs(plein, marqueurs)) < PART_HORS_MARQUEURS_MIN * aire
                or np.count_nonzero(cv2.bitwise_and(plein, jaune)) > PART_JAUNE_BALLE * aire):
            continue
        if silhouette is not None and \
                np.count_nonzero(cv2.bitwise_and(plein, silhouette)) > PART_BRAS_MAX * aire:
            continue
        if any(np.count_nonzero(cv2.bitwise_and(plein, p))
               > RECOUVREMENT_MAX * min(aire, np.count_nonzero(p)) for p in pleins):
            continue
        uv = vision.point_le_plus_epais(plein)
        if uv is None:
            continue
        xy = np.asarray(vision.vers_base(uv, module.HAUTEUR_OBJET)[:2], float)
        if any(np.hypot(*(xy - c)) < RAYON_CARTON_MM for c in cartons):
            continue
        pleins.append(plein)
        gardes.append((CLASSE_DASHBOARD[d['classe']], xy, contour))
    return gardes


def balle_yolo(vision, dernier, module):
    """(xy base mm, (u, v, rayon)) comme `Vision.balle`, depuis la silhouette YOLO, ou None."""
    if dernier is None or time.time() - dernier[2] > FRAICHEUR_BALLE_S:
        return None
    image, detections, _ = dernier
    planche = vision.masque_planche(image.shape[:2])
    for d in sorted((d for d in detections if d['classe'] == 'balle'), key=lambda d: -d['conf']):
        contour = np.asarray(d['contour'], np.float32).reshape(-1, 1, 2)
        plein = np.zeros(image.shape[:2], np.uint8)
        cv2.fillPoly(plein, [contour.astype(np.int32)], 255)
        aire = np.count_nonzero(plein)
        if aire == 0 or np.count_nonzero(cv2.bitwise_and(plein, planche)) < PART_PLANCHE_BALLE_MIN * aire:
            continue
        (u, v), rayon = cv2.minEnclosingCircle(contour)
        return (np.asarray(vision.vers_base((u, v), module.HAUTEUR_CENTRE_BALLE)[:2], float),
                (float(u), float(v), float(rayon)))
    return None


def attend_image_fraiche(service, depuis, patience=PATIENCE_FRAICHE_S):
    """Vrai des qu'un resultat YOLO porte sur une image prise apres `depuis`."""
    limite = time.time() + patience
    while time.time() < limite and not service.arrete:
        vu = service.dernier()
        if vu is not None and vu[2] >= depuis:
            time.sleep(ATTENTE_RAFRAICHI_S)
            return True
        time.sleep(0.03)
    return False


def branche(module, service=None, inventaire=None):
    """Remplace `module.Vision.objets` pour l'arducam. A appeler AVANT
    `correction_vision.branche`, pour que la carte corrige ce que YOLO rend."""
    service = service or ServiceYOLO()
    atexit.register(service.ferme)
    origine = module.Vision.objets
    en_panne = []
    # Les marqueurs de la PLANCHE ne bougent jamais : leur dernier carre vu reste
    # valable quand une main les cache. Le 19 a moitie sous la main n'etait plus
    # detecte et ressortait `robot` (15/09). Ceux des cartons bougent : pas retenus.
    planche = {}

    def objets(self, image, angles=None, marqueurs=None):
        if en_panne or not self.source.startswith('arducam'):
            return origine(self, image, angles, marqueurs)
        service.soumet(image)
        if service.arrete:
            en_panne.append(True)
            print('yolo_service.py arrete — retour au detecteur couleur/forme', flush=True)
            return origine(self, image, angles, marqueurs)
        planche.update({i: c for i, c in (marqueurs or {}).items() if i in module._MARQUEURS})
        vu = service.dernier()
        if vu is None:
            return []
        image_vue, detections, _ = vu
        return juge(self, image_vue, [d for d in detections if d['classe'] != 'balle'],
                    module, angles, {**planche, **(marqueurs or {})})

    module.Vision.objets = objets

    origine_balle = module.Vision.balle

    def balle(self, image):
        vu = origine_balle(self, image)
        if vu is not None or en_panne or not self.source.startswith('arducam'):
            return vu
        return balle_yolo(self, service.dernier(), module)

    module.Vision.balle = balle

    # `ctx.detecteur` est lie a la construction de la fenetre : brancher AVANT `Fenetre()`.
    origine_detecte = module.Fenetre._detecte_objet

    def _detecte_objet(self, *a, **kw):
        # Seulement pour CHOISIR une cible (degagement, ou aucune classe en cours) :
        # `cible_a_bouge` interroge aussi pendant l'approche, ou une image de retard
        # ne coute rien et 0,25 s a chaque controle couterait. Et jamais depuis le fil
        # graphique : c'est lui qui soumet les images, l'attendre le bloquerait.
        choix = kw.get('exige_dessus') or not self.ctx.classe_objet
        if choix and not en_panne and threading.current_thread() is not threading.main_thread():
            attend_image_fraiche(service, time.time())
        return origine_detecte(self, *a, **kw)

    module.Fenetre._detecte_objet = _detecte_objet
    if inventaire:
        module.INVENTAIRE.update(inventaire)
    return service
