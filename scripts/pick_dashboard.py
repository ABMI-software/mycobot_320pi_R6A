#!/usr/bin/env python3
"""Tableau de bord du pick-and-place vision-guide.

Trois panneaux : les deux cameras en direct a gauche, le graphe de la machine a
etats au centre (point rouge clignotant sur l'etat courant), les mesures et le
journal a droite.

Deux modes :
* MANUEL — un clic execute UNE etape, on voit le point rouge sauter ;
* AUTOMATIQUE — la machine boucle seule, detection -> saisie -> depot -> detection.

Les mouvements robot durent plusieurs secondes : ils tournent dans un fil separe,
sinon l'interface se fige et les cameras s'arretent.

    conda deactivate
    /usr/bin/python3 scripts/pick_dashboard.py

Prerequis : `gripper_bridge.py` tourne sur la Pi, et aucun autre client TCP n'est
connecte (le pont est mono-client et bloquant).
"""
from __future__ import annotations

import re
import subprocess
import sys
import threading
import time
from collections import deque
from pathlib import Path

import cv2
import numpy as np
import yaml
from PyQt5.QtCore import QPointF, QRectF, Qt, QThread, QTimer, pyqtSignal
from PyQt5.QtGui import (QBrush, QColor, QFont, QImage, QPainter, QPainterPath,
                         QPen, QPixmap, QPolygonF)
from PyQt5.QtWidgets import (QApplication, QComboBox, QFormLayout, QGroupBox,
                             QHBoxLayout, QLabel, QMainWindow, QPushButton,
                             QTextEdit, QVBoxLayout, QWidget)

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE / 'scripts'))
sys.path.insert(0, str(RACINE / 'mycobot_gateway' / 'mycobot_gateway' / 'vision'))
import pick_fsm as fsm                                                  # noqa: E402
import camera_registry as registre                                      # noqa: E402

CALIB = RACINE / 'training' / 'calibration'
SECOURS = {'arducam': 0, 'svpro': 2}   # si v4l2-ctl n'enumere rien
HSV_BALLE = ((25, 90, 90), (45, 255, 255))
# Le carton brun se separe de la planche par la TEINTE et la LUMINOSITE, pas par
# la saturation (mesure sur image : carton H~11 S~161 V~95 ; planche H~19 S~160
# V~200). La saturation est la meme, c'est la teinte et le fait qu'il soit deux
# fois moins lumineux qui le trahissent.
HSV_CARTON = ((5, 90, 45), (16, 235, 145))
HAUTEUR_CENTRE_BALLE = 35.0
HAUTEUR_CARTON = 60.0     # rebord du carton
FENETRE_DETECTION = 1.2   # s — age maximal d'une detection reutilisable

# Emprise du plateau, deduite des marqueurs pour ne pas dériver de leur fichier.
# Ils sont en retrait des bords : d'où la marge.
_XY_MARQUEURS = np.array(
    [v[:2] for v in yaml.safe_load(
        (CALIB / 'workspace_markers.yaml').read_text())['markers'].values()], float)
PLATEAU = ((_XY_MARQUEURS[:, 0].min() - 40.0, _XY_MARQUEURS[:, 0].max() + 40.0),
           (_XY_MARQUEURS[:, 1].min() - 40.0, _XY_MARQUEURS[:, 1].max() + 40.0))

# Disposition du graphe, en coordonnees normalisees. La boucle principale fait le
# tour, ECHEC est au centre : toutes les sorties d'erreur y convergent.
NOEUDS = {
    'ATTENTE': (0.06, 0.52), 'DEGAGEMENT': (0.10, 0.28), 'DETECTION': (0.30, 0.10),
    'APPROCHE': (0.54, 0.06), 'RECALAGE': (0.78, 0.14), 'DESCENTE': (0.92, 0.36),
    'SAISIE': (0.92, 0.64), 'REMONTEE': (0.78, 0.86), 'TRANSFERT': (0.54, 0.94),
    'LARGAGE': (0.30, 0.90), 'RETRAIT': (0.06, 0.72), 'ECHEC': (0.50, 0.50),
}
def lit_controles(index):
    """{nom: valeur} des controles d'exposition lus sur le peripherique."""
    sortie = subprocess.run(['v4l2-ctl', '-d', f'/dev/video{index}', '--get-ctrl',
                             'auto_exposure,exposure_time_absolute'],
                            capture_output=True, text=True, timeout=5).stdout
    return {m.group(1): int(m.group(2))
            for m in (re.match(r'\s*(\w+):\s*(-?\d+)', l) for l in sortie.splitlines()) if m}


def regle_exposition(index, exposition):
    """Recette de `camera_publisher.set_manual_exposure`, sans dependance ROS.

    Passer en manuel AVANT de poser le temps d'exposition : dans l'autre ordre
    le driver ignore la consigne sans rien dire. `exposition < 0` = la camera
    tourne en auto (cas SVPRO), et on force l'auto pour effacer un reglage
    sombre reste coince dans le peripherique.
    """
    dev = f'/dev/video{index}'
    reglages = ([('auto_exposure', '1'),
                 ('exposure_time_absolute,gain,brightness', f'{exposition},0,0')]
                if exposition >= 0 else
                [('auto_exposure', '3'), ('gain,brightness', '100,0')])
    for controles, valeurs in reglages:
        ctrl = ','.join(f'{c}={v}' for c, v in zip(controles.split(','), valeurs.split(',')))
        subprocess.run(['v4l2-ctl', '-d', dev, '--set-ctrl', ctrl],
                       capture_output=True, timeout=5)


BLEU = QColor(62, 110, 190)
BLEU_CLAIR = QColor(120, 165, 225)
GRIS = QColor(170, 175, 185)
ROUGE = QColor(225, 45, 45)


# --------------------------------------------------------------------------- #
#  Vision
# --------------------------------------------------------------------------- #

class Vision:
    """Detection de la balle et projection dans le repere base.

    L'extrinseque est calibree en 640x480 : capturer dans CE mode, sinon les
    intrinseques ne s'appliquent pas.
    """

    def __init__(self, stem='arducam_extrinsic_pick'):
        fichier = CALIB / f'{stem}.yaml'
        if not fichier.exists():
            fichier = CALIB / 'arducam_extrinsic_servo.yaml'
        d = yaml.safe_load(fichier.read_text())
        self.T = np.array(d['T_cam_world'], float)
        intr = np.load(CALIB / f"{d['intrinsics_stem']}.npz")
        self.K = np.array(intr['mtx'], float)
        self.dist = np.array(intr['dist'], float)
        self.source = fichier.name

    def vers_base(self, uv, z_mm):
        """Pixel -> point du plan horizontal Z=z_mm dans le repere base (mm)."""
        p = cv2.undistortPoints(np.array([[uv]], float), self.K, self.dist).reshape(2)
        R, t = self.T[:3, :3], self.T[:3, 3]
        centre = -R.T @ t
        direction = R.T @ np.array([p[0], p[1], 1.0])
        s = (z_mm / 1000.0 - centre[2]) / direction[2]
        return (centre + s * direction) * 1000.0

    def vers_pixel(self, p_mm):
        rvec, _ = cv2.Rodrigues(self.T[:3, :3])
        uv, _ = cv2.projectPoints(np.asarray(p_mm, float).reshape(1, 3) / 1000.0,
                                  rvec, self.T[:3, 3], self.K, self.dist)
        return uv.reshape(2)

    def balle(self, image):
        """(x, y) base en mm, plus la tache image, ou None si douteux."""
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        masque = cv2.morphologyEx(cv2.inRange(hsv, *HSV_BALLE), cv2.MORPH_CLOSE,
                                  np.ones((5, 5), np.uint8))
        contours, _ = cv2.findContours(masque, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if not contours:
            return None
        c = max(contours, key=cv2.contourArea)
        aire = cv2.contourArea(c)
        (u, v), rayon = cv2.minEnclosingCircle(c)
        if aire < 150 or aire / (np.pi * rayon ** 2) < 0.6:
            return None                       # le bouton d'arret d'urgence est jaune aussi
        p = self.vers_base((u, v), HAUTEUR_CENTRE_BALLE)
        return p[:2], (u, v, rayon)

    def carton(self, image):
        """(x, y) base du centre du carton et son contour image, ou None.

        On ne garde que la plus grande tache brune TOMBANT SUR LE PLATEAU : le
        sol et les objets hors planche en produisent de plus grosses encore
        (mesure : deux fausses taches a 505 et 511 mm de portee).
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        masque = cv2.inRange(hsv, *HSV_CARTON)
        masque = cv2.morphologyEx(masque, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        masque = cv2.morphologyEx(masque, cv2.MORPH_CLOSE, np.ones((13, 13), np.uint8))
        contours, _ = cv2.findContours(masque, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in sorted(contours, key=cv2.contourArea, reverse=True):
            if cv2.contourArea(c) < 1200:
                break
            moments = cv2.moments(c)
            uv = (moments['m10'] / moments['m00'], moments['m01'] / moments['m00'])
            p = self.vers_base(uv, HAUTEUR_CARTON)
            if (PLATEAU[0][0] <= p[0] <= PLATEAU[0][1]
                    and PLATEAU[1][0] <= p[1] <= PLATEAU[1][1]):
                return p[:2], c
        return None


# --------------------------------------------------------------------------- #
#  Graphe de la machine a etats
# --------------------------------------------------------------------------- #

class GrapheEtats(QWidget):
    """Noeuds ronds, fleches etiquetees, point rouge clignotant sur l'etat actif."""

    def __init__(self):
        super().__init__()
        self.etat = 'ATTENTE'
        self.clignote = True
        self.setMinimumSize(560, 460)
        minuteur = QTimer(self)
        minuteur.timeout.connect(self._bascule)
        minuteur.start(450)
        self._minuteur = minuteur

    def _bascule(self):
        self.clignote = not self.clignote
        self.update()

    def _centre(self, nom):
        x, y = NOEUDS[nom]
        marge = 62
        return QPointF(marge + x * (self.width() - 2 * marge),
                       marge + y * (self.height() - 2 * marge))

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        p.fillRect(self.rect(), QColor(252, 252, 253))
        rayon = 34

        for depart, arrivee, condition, action in fsm.TRANSITIONS:
            actif = depart == self.etat
            a, b = self._centre(depart), self._centre(arrivee)
            vecteur = b - a
            longueur = float(np.hypot(vecteur.x(), vecteur.y())) or 1.0
            unite = QPointF(vecteur.x() / longueur, vecteur.y() / longueur)
            depuis = a + unite * rayon
            vers = b - unite * rayon
            milieu = (depuis + vers) / 2.0
            normale = QPointF(-unite.y(), unite.x())
            courbe = milieu + normale * (longueur * 0.13)

            p.setPen(QPen(BLEU if actif else GRIS, 2.4 if actif else 1.2))
            chemin = QPainterPath(depuis)
            chemin.quadTo(courbe, vers)
            p.drawPath(chemin)

            tangente = vers - courbe
            n = float(np.hypot(tangente.x(), tangente.y())) or 1.0
            tangente = QPointF(tangente.x() / n, tangente.y() / n)
            perp = QPointF(-tangente.y(), tangente.x())
            p.setBrush(QBrush(BLEU if actif else GRIS))
            p.drawPolygon(QPolygonF([vers, vers - tangente * 11 + perp * 4.5,
                                     vers - tangente * 11 - perp * 4.5]))

            if actif and (condition or action):
                p.setFont(QFont('Sans', 7))
                p.setPen(QPen(BLEU.darker(120)))
                texte = f'{condition} / {action}' if action else condition
                p.drawText(QRectF(courbe.x() - 78, courbe.y() - 12, 156, 24),
                           Qt.AlignCenter | Qt.TextWordWrap, texte)

        for nom in fsm.ETATS:
            c = self._centre(nom)
            actif = nom == self.etat
            p.setPen(QPen(BLEU.darker(140), 1.5))
            p.setBrush(QBrush(BLEU if actif else BLEU_CLAIR))
            p.drawEllipse(c, rayon, rayon)
            p.setPen(QPen(Qt.white if actif else QColor(30, 40, 60)))
            p.setFont(QFont('Sans', 7, QFont.Bold))
            p.drawText(QRectF(c.x() - rayon, c.y() - 11, 2 * rayon, 22),
                       Qt.AlignCenter | Qt.TextWordWrap, nom)
            if actif and self.clignote:
                p.setPen(QPen(ROUGE.darker(130), 1.5))
                p.setBrush(QBrush(ROUGE))
                p.drawEllipse(c + QPointF(rayon * 0.72, -rayon * 0.72), 8, 8)

    def montre(self, etat):
        self.etat = etat
        self.clignote = True
        self.update()


# --------------------------------------------------------------------------- #
#  Fil d'execution du robot
# --------------------------------------------------------------------------- #

class Ouvrier(QThread):
    """Execute des pas de la machine hors du fil graphique."""

    avance = pyqtSignal(str)
    fini = pyqtSignal()

    def __init__(self, machine):
        super().__init__()
        self.machine = machine
        self.continu = False
        self._stop = False

    def demande_arret(self):
        self._stop = True

    def run(self):
        self._stop = False
        while True:
            self.avance.emit(self.machine.pas())
            if self._stop or not self.continu:
                break
            if self.machine.etat in ('ATTENTE', 'ECHEC') and not self.machine.ctx.mode_auto:
                break
        self.fini.emit()


# --------------------------------------------------------------------------- #
#  Fenetre
# --------------------------------------------------------------------------- #

class Fenetre(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('MyCobot 320 Pi — pick-and-place vision-guide')
        self.vision = Vision()
        self.images = {}
        self.captures = {}
        self.ctx = fsm.Contexte()
        self.ctx.detecteur = self._detecte_balle
        self.machine = fsm.MachineEtats(self.ctx)
        self.ouvrier = None
        self._n_journal = 0
        self._detections = deque(maxlen=40)
        self._verrou = threading.Lock()
        self.derives = {}
        self.cameras = self._detecte_cameras()

        self._construit()
        self._ouvre_cameras()
        self.rafraichi = QTimer(self)
        self.rafraichi.timeout.connect(self._tick)
        self.rafraichi.start(60)
        self._connecte()
        # L'exposition se pose une fois le flux etabli : un controle v4l2 ecrit
        # avant les premieres images est silencieusement annule au demarrage du
        # flux. Le rafraichissement fait office de rodage.
        QTimer.singleShot(2500, self._regle_expositions)

    # -- construction ------------------------------------------------------- #

    def _construit(self):
        central = QWidget()
        colonnes = QHBoxLayout(central)

        gauche = QVBoxLayout()
        self.vues = {}
        for nom in self.cameras:
            boite = QGroupBox(nom)
            interieur = QVBoxLayout(boite)
            vue = QLabel('en attente de flux…')
            vue.setMinimumSize(400, 300)
            vue.setAlignment(Qt.AlignCenter)
            vue.setStyleSheet('background:#111; color:#888;')
            interieur.addWidget(vue)
            self.vues[nom] = vue
            gauche.addWidget(boite)
        colonnes.addLayout(gauche, 3)

        milieu = QVBoxLayout()
        self.graphe = GrapheEtats()
        milieu.addWidget(self.graphe, 1)

        commandes = QGroupBox('conduite')
        ligne = QHBoxLayout(commandes)
        self.mode = QComboBox()
        self.mode.addItems(['manuel', 'automatique'])
        self.mode.currentTextChanged.connect(self._change_mode)
        self.bouton_pas = QPushButton('étape suivante ▶')
        self.bouton_pas.clicked.connect(self._un_pas)
        self.bouton_auto = QPushButton('démarrer la boucle')
        self.bouton_auto.clicked.connect(self._bascule_auto)
        self.bouton_auto.setEnabled(False)
        arret = QPushButton('⊘ STOP')
        arret.setStyleSheet('background:#c62828; color:white; font-weight:bold;')
        arret.clicked.connect(self._stop)
        self.etiquette_marche = QLabel('prêt')
        self.etiquette_marche.setStyleSheet('color:#555;')
        for w in (QLabel('mode'), self.mode, self.bouton_pas, self.bouton_auto, arret,
                  self.etiquette_marche):
            ligne.addWidget(w)
        milieu.addWidget(commandes)
        colonnes.addLayout(milieu, 4)

        droite = QVBoxLayout()

        # Affichage continu, independant de la machine a etats : on doit voir ou
        # est la balle AVANT de lancer quoi que ce soit.
        boite_live = QGroupBox('balle vue en direct')
        form_live = QFormLayout(boite_live)
        self.live_position = QLabel('—')
        self.live_portee = QLabel('—')
        self.live_verdict = QLabel('—')
        self.live_expo = QLabel('réglage en cours…')
        self.live_position.setStyleSheet('font-family:monospace; font-weight:bold;')
        self.live_portee.setStyleSheet('font-family:monospace;')
        self.live_expo.setStyleSheet('font-family:monospace; color:#888;')
        form_live.addRow('position base', self.live_position)
        form_live.addRow('portée', self.live_portee)
        form_live.addRow('verdict', self.live_verdict)
        form_live.addRow('exposition', self.live_expo)
        droite.addWidget(boite_live)

        boite_mesures = QGroupBox('mesures')
        self.mesures = QFormLayout(boite_mesures)
        self.champs = {}
        droite.addWidget(boite_mesures)

        boite_cible = QGroupBox('cibles')
        form_cible = QFormLayout(boite_cible)
        self.champ_carton = QLabel('non défini')
        bouton_auto_carton = QPushButton('détecter le carton (couleur)')
        bouton_auto_carton.clicked.connect(self._detecte_carton)
        bouton_carton = QPushButton('définir le carton = position balle')
        bouton_carton.clicked.connect(self._definit_carton)
        form_cible.addRow('carton', self.champ_carton)
        form_cible.addRow(bouton_auto_carton)
        form_cible.addRow(bouton_carton)
        bouton_extr = QPushButton('vérifier l’extrinsèque (ArUco)')
        bouton_extr.clicked.connect(self._verifie_extrinseque)
        form_cible.addRow(bouton_extr)
        self.champ_extr = QLabel(self.vision.source)
        form_cible.addRow('source', self.champ_extr)
        droite.addWidget(boite_cible)

        boite_journal = QGroupBox('journal')
        v = QVBoxLayout(boite_journal)
        self.journal = QTextEdit()
        self.journal.setReadOnly(True)
        self.journal.setStyleSheet('font-family:monospace; font-size:11px;')
        v.addWidget(self.journal)
        droite.addWidget(boite_journal, 1)
        colonnes.addLayout(droite, 3)

        self.setCentralWidget(central)
        self.resize(1600, 900)
        self.statusBar().showMessage('déconnecté — le premier pas ouvre le pont TCP')

    # -- cameras ------------------------------------------------------------ #

    def _detecte_cameras(self):
        """Memes cameras et memes reglages que le dashboard de validation DREAM.

        Le registre les identifie par leur nom V4L2 et porte leur exposition
        calibree — l'ordre des `/dev/video` n'est pas stable d'un branchement a
        l'autre, le figer ici finirait par pointer la mauvaise camera.
        """
        # `probe_capture=False` : la sonde du registre ouvre la camera pour
        # verifier qu'elle capture, et on la rouvre juste apres — l'UVC ne rend
        # pas la bande passante assez vite, la seconde ouverture echoue sur
        # « Failed to allocate required memory ». On ouvre une seule fois.
        trouvees = {s.name: (s.v4l2_index, s.manual_exposure)
                    for s in registre.detect_cameras(probe_capture=False)}
        for nom, index in SECOURS.items():
            trouvees.setdefault(nom, (index, registre.KNOWN_BY_NAME[nom].manual_exposure))
        return trouvees

    def _ouvre_cameras(self, essais=5):
        """Ouvre chaque camera, en reessayant : l'ouverture est capricieuse.

        Une ouverture trop proche de la precedente echoue — le peripherique
        s'ouvre mais ne delivre aucune image. Il faut donc valider par une
        LECTURE, pas par `isOpened()`, et laisser au bus le temps de se liberer.
        """
        for nom, (index, _) in self.cameras.items():
            self.captures[nom] = None
            for essai in range(essais):
                cap = cv2.VideoCapture(index)
                cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
                cap.set(cv2.CAP_PROP_FRAME_WIDTH, registre.CAPTURE_W)
                cap.set(cv2.CAP_PROP_FRAME_HEIGHT, registre.CAPTURE_H)
                cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
                if cap.isOpened() and cap.read()[0]:
                    self.captures[nom] = cap
                    break
                cap.release()
                time.sleep(1.2)
            if self.captures[nom] is None:
                self.ctx.note(f'{nom} /dev/video{index} — muette apres {essais} essais')

    def _regle_expositions(self):
        for nom, (index, exposition) in self.cameras.items():
            if self.captures.get(nom) is None:
                continue
            regle_exposition(index, exposition)
            self.ctx.note(f'{nom} /dev/video{index} — exposition '
                          + (f'manuelle {exposition}' if exposition >= 0 else 'auto'))
        self.surveille = QTimer(self)
        self.surveille.timeout.connect(self._surveille_expositions)
        self.surveille.start(4000)

    def _surveille_expositions(self):
        """L'arducam repasse en auto toute seule — le driver relache le reglage
        manuel (constate : `auto_exposure` revenu a 3, exposition a 157 au lieu
        de 75). L'image scintille alors et la balle jaune disparait sous la
        surexposition. On relit le peripherique et on repose des que ca bouge.
        """
        for nom, (index, exposition) in self.cameras.items():
            if self.captures.get(nom) is None or exposition < 0:
                continue
            etat = lit_controles(index)
            tenue = (etat.get('auto_exposure') == 1
                     and etat.get('exposure_time_absolute') == exposition)
            if nom == 'arducam':
                self.live_expo.setText(f'manuelle {exposition}' if tenue
                                       else f'AUTO ({etat.get("exposure_time_absolute")})')
                self.live_expo.setStyleSheet('font-family:monospace; '
                                             + ('color:#2e7d32;' if tenue
                                                else 'color:#c62828; font-weight:bold;'))
            if tenue:
                continue
            regle_exposition(index, exposition)
            self.derives[nom] = self.derives.get(nom, 0) + 1
            self.ctx.note(f'{nom} — exposition relachee par le driver '
                          f'(auto={etat.get("auto_exposure")}, '
                          f'temps={etat.get("exposure_time_absolute")}), remise a '
                          f'{exposition} — {self.derives[nom]}e fois')

    def _tick(self):
        for nom, cap in self.captures.items():
            if cap is None:
                continue
            ok, image = cap.read()
            if not ok:
                continue
            self.images[nom] = image
            affichee = image.copy()
            if nom == 'arducam':
                trouve = self.vision.balle(image)
                with self._verrou:
                    self._detections.append(
                        (time.time(), None if trouve is None else np.asarray(trouve[0], float)))
                self._maj_live(trouve)
                if trouve is not None:
                    xy, (u, v, r) = trouve
                    cv2.circle(affichee, (int(u), int(v)), int(r) + 3, (0, 0, 255), 2)
                    cv2.putText(affichee, f'{xy[0]:.0f},{xy[1]:.0f}',
                                (int(u) - 34, int(v) - int(r) - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 255), 1)
                carton = self.vision.carton(image)
                if carton is not None:
                    cv2.drawContours(affichee, [carton[1]], -1, (255, 150, 0), 2)
                if self.ctx.carton_xy is not None:
                    uv = self.vision.vers_pixel([self.ctx.carton_xy[0],
                                                 self.ctx.carton_xy[1], 60.0])
                    cv2.drawMarker(affichee, tuple(uv.astype(int)), (255, 0, 255),
                                   cv2.MARKER_TILTED_CROSS, 18, 2)
            self._affiche(nom, affichee)
        self._vide_journal()

    def _maj_live(self, trouve):
        """Position de la balle en continu, avec le verdict d'atteignabilite."""
        if trouve is None:
            self.live_position.setText('—')
            self.live_portee.setText('—')
            self.live_verdict.setText('balle non vue')
            self.live_verdict.setStyleSheet('color:#888;')
            return
        xy, _ = trouve
        portee = float(np.hypot(*xy))
        self.live_position.setText(f'X {xy[0]:7.1f}   Y {xy[1]:7.1f}  mm')
        self.live_portee.setText(f'{portee:.1f} mm')
        if portee > fsm.PORTEE_MAX:
            self.live_verdict.setText(f'HORS ENVELOPPE — rapprocher de '
                                      f'{portee - fsm.PORTEE_MAX:.0f} mm')
            self.live_verdict.setStyleSheet('color:#c62828; font-weight:bold;')
        else:
            self.live_verdict.setText('atteignable')
            self.live_verdict.setStyleSheet('color:#2e7d32; font-weight:bold;')

    def _affiche(self, nom, image):
        rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        h, w, _ = rgb.shape
        qimg = QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888)
        vue = self.vues[nom]
        vue.setPixmap(QPixmap.fromImage(qimg).scaled(vue.width(), vue.height(),
                                                     Qt.KeepAspectRatio,
                                                     Qt.SmoothTransformation))

    # -- machine ------------------------------------------------------------ #

    def _detecte_balle(self, echantillons=3, patience=3.0):
        """Moyenne des dernieres detections DEJA produites par le fil graphique.

        Appele depuis le fil du robot. Y relire la camera pendant que le
        rafraichissement la lit aussi fige la capture GStreamer sans jamais
        rendre la main : l'interface reste vivante, l'etape ne se termine plus.
        Un seul fil touche `VideoCapture`. `patience=0` = ce qu'on a sous la
        main, pour un appel depuis le fil graphique (qui ne peut pas s'attendre
        lui-meme).
        """
        limite = time.time() + patience
        while True:
            maintenant = time.time()
            with self._verrou:
                vues = [xy for horodatage, xy in self._detections
                        if xy is not None and maintenant - horodatage < FENETRE_DETECTION]
            if len(vues) >= echantillons:
                break
            if maintenant >= limite:
                self.ctx.note(f'balle vue sur {len(vues)} image(s) seulement '
                              f'en {patience:.0f} s — cible refusee')
                return None
            time.sleep(0.05)
        vues = vues[-echantillons:]
        moyenne = np.mean(vues, axis=0)
        dispersion = max(float(np.linalg.norm(v - moyenne)) for v in vues)
        if dispersion > 3.0:
            self.ctx.note(f'detection instable ({dispersion:.1f} mm) — cible refusee')
            return None
        return moyenne

    def _assure_pont(self):
        if self.ctx.pont is None:
            self.ctx.pont = fsm.Pont()
            self.statusBar().showMessage(f'connecté à {fsm.PI[0]}:{fsm.PI[1]}')

    def _connecte(self):
        """Le pont s'ouvre au demarrage : l'etat TCP doit etre visible avant de
        commander quoi que ce soit, pas decouvert au premier clic."""
        try:
            self._assure_pont()
        except OSError as erreur:
            self.statusBar().showMessage(
                f'pont injoignable ({erreur}) — gripper_bridge.py tourne-t-il sur '
                f'{fsm.PI[0]} ? un autre client TCP le retient ?')

    def _lance(self, continu):
        if self.ouvrier is not None and self.ouvrier.isRunning():
            return
        try:
            self._assure_pont()
        except OSError as erreur:
            self.statusBar().showMessage(f'pont injoignable : {erreur}')
            return
        self.ouvrier = Ouvrier(self.machine)
        self.ouvrier.continu = continu
        self.ouvrier.avance.connect(self.graphe.montre)
        self.ouvrier.avance.connect(self._montre_mesures)
        self.ouvrier.avance.connect(self._montre_marche)
        self.ouvrier.fini.connect(self._fin_de_course)
        self.bouton_pas.setEnabled(False)
        self._marche(f'⏳ {self.machine.etat} en cours…')
        self.ouvrier.start()

    def _marche(self, texte, occupe=True):
        self.etiquette_marche.setText(texte)
        self.etiquette_marche.setStyleSheet(
            'color:#ef6c00; font-weight:bold;' if occupe else 'color:#555;')

    def _montre_marche(self, etat):
        if self.ouvrier is not None and self.ouvrier.isRunning():
            self._marche(f'⏳ {etat} en cours…')

    def _un_pas(self):
        self._lance(continu=False)

    def _bascule_auto(self):
        if self.ouvrier is not None and self.ouvrier.isRunning():
            self.ouvrier.continu = False
            self.ouvrier.demande_arret()
            self.bouton_auto.setText('démarrer la boucle')
        else:
            self.bouton_auto.setText('arrêter la boucle')
            self._lance(continu=True)

    def _change_mode(self, texte):
        auto = texte == 'automatique'
        self.ctx.mode_auto = auto
        self.bouton_auto.setEnabled(auto)
        self.bouton_pas.setEnabled(not auto or self.ouvrier is None
                                   or not self.ouvrier.isRunning())

    def _stop(self):
        if self.ouvrier is not None:
            self.ouvrier.continu = False
            self.ouvrier.demande_arret()
        self.bouton_auto.setText('démarrer la boucle')
        self.statusBar().showMessage('arrêt demandé — le mouvement en cours se termine')

    def _fin_de_course(self):
        self.bouton_pas.setEnabled(True)
        self.bouton_auto.setText('démarrer la boucle')
        self._marche(f'prêt — {self.machine.etat}', occupe=False)

    def _montre_mesures(self, _):
        for cle, valeur in self.ctx.resultats.items():
            if cle not in self.champs:
                self.champs[cle] = QLabel()
                self.mesures.addRow(cle, self.champs[cle])
            self.champs[cle].setText(str(valeur))

    def _vide_journal(self):
        if len(self.ctx.journal) == self._n_journal:
            return
        for ligne in self.ctx.journal[self._n_journal:]:
            self.journal.append(ligne)
        self._n_journal = len(self.ctx.journal)

    # -- cibles ------------------------------------------------------------- #

    def _detecte_carton(self):
        image = self.images.get('arducam')
        if image is None:
            return
        trouve = self.vision.carton(image)
        if trouve is None:
            self.statusBar().showMessage('carton non détecté sur le plateau')
            return
        xy, _ = trouve
        self.ctx.carton_xy = np.asarray(xy, float)
        portee = float(np.hypot(*xy))
        self.champ_carton.setText(f'({xy[0]:.1f}, {xy[1]:.1f}) mm — détecté')
        self.statusBar().showMessage(
            f'carton détecté à {portee:.0f} mm'
            + ('' if portee <= fsm.PORTEE_MAX
               else f' — HORS ENVELOPPE, le largage échouera ({fsm.PORTEE_MAX:.0f} mm max)'))

    def _definit_carton(self):
        """Le carton se designe en y posant la balle : elle sert de mire."""
        vu = self._detecte_balle(patience=0.0)
        if vu is None:
            self.statusBar().showMessage('balle non vue — impossible de définir le carton')
            return
        self.ctx.carton_xy = np.asarray(vu, float)
        self.champ_carton.setText(f'({vu[0]:.1f}, {vu[1]:.1f}) mm')

    def _verifie_extrinseque(self):
        """ArUco exige OpenCV 5 : on passe par le venv, sur une image fichier."""
        image = self.images.get('arducam')
        if image is None:
            return
        chemin = Path('/tmp/pick_dashboard_arducam.png')
        cv2.imwrite(str(chemin), image)
        venv = RACINE / '.venv' / 'bin' / 'python'
        if not venv.exists():
            self.statusBar().showMessage('venv absent — vérification ArUco indisponible')
            return
        sortie = subprocess.run([str(venv), str(RACINE / 'scripts' / 'aruco_check.py'),
                                 str(chemin)], capture_output=True, text=True, timeout=60)
        self.statusBar().showMessage(sortie.stdout.strip().replace('\n', ' | ')
                                     or sortie.stderr.strip()[:200])

    def closeEvent(self, evenement):
        for cap in self.captures.values():
            if cap is not None:
                cap.release()
        if self.ctx.pont is not None:
            self.ctx.pont.ferme()
        evenement.accept()


def main():
    application = QApplication(sys.argv)
    fenetre = Fenetre()
    fenetre.show()
    sys.exit(application.exec_())


if __name__ == '__main__':
    main()
