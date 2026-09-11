#!/usr/bin/env python3
"""La fenetre « faut-il recalibrer ? » posee au lancement du pick.

Module a part pour une raison precise : `pick_dashboard` l'importe, et il ne
doit surtout pas importer `pick_dashboard` en retour. Tout ce qui touche au
dashboard lui-meme reste dans `lancer_pick_dashboard.py`.

    import calibration_dialogue
    calibration_dialogue.demande()      # rend la main quand l utilisateur a tranche
    fenetre = Fenetre()                 # APRES, pour qu elle lise la nouvelle extrinseque

La calibration tourne dans un processus separe sous `.venv` : ArUco exige
OpenCV 5 alors que le dashboard tourne en 4.6.
"""
from __future__ import annotations

import json
import sys
from datetime import datetime
from pathlib import Path

import numpy as np
import yaml
from PyQt5.QtCore import QProcess, Qt, QTimer
from PyQt5.QtGui import QColor, QFont, QPainter, QPen
from PyQt5.QtWidgets import (QDialog, QHBoxLayout, QLabel, QPushButton,
                             QTextEdit, QVBoxLayout, QWidget)

RACINE = Path(__file__).resolve().parents[1]
CALIB = RACINE / 'training' / 'calibration'
VENV = RACINE / '.venv' / 'bin' / 'python'
OUVRIER = RACINE / 'scripts' / 'calibration_extrinseque_auto.py'
EXTRINSEQUE = CALIB / 'arducam_extrinsic_pick.yaml'
REFERENCE = CALIB / 'planche_actuelle.yaml'
DUREE = RACINE / 'scripts' / 'calibration_duree.json'
DUREE_DEFAUT = 20.0

BLEU = QColor(62, 110, 190)
VERT = QColor(46, 110, 79)
ROUGE = QColor(166, 43, 31)
GRIS = QColor(200, 205, 214)


def duree_attendue():
    try:
        return float(json.loads(DUREE.read_text())['secondes'])
    except (OSError, json.JSONDecodeError, KeyError, TypeError, ValueError):
        return DUREE_DEFAUT


def memorise_duree(secondes):
    DUREE.write_text(json.dumps({'secondes': round(float(secondes), 1),
                                 'date': f'{datetime.now():%Y-%m-%d %H:%M}'}))


def etat_extrinseque():
    """(texte affichable, position camera) de l'extrinseque en place."""
    try:
        d = yaml.safe_load(EXTRINSEQUE.read_text())
    except OSError:
        return 'aucune extrinsèque en place', None
    T = np.array(d['T_cam_world'], float)
    cam = (-T[:3, :3].T @ T[:3, 3]) * 1000.0
    source = str(d.get('source', ''))
    quand = next((m for m in source.split(', ') if '/' in m and len(m) <= 24), '')
    return (f'caméra en ({cam[0]:.0f}, {cam[1]:.0f}, {cam[2]:.0f}) mm  ·  '
            f'RMS {d.get("reprojection_rms_px", "?")} px'
            + (f'  ·  {quand}' if quand else '')), cam


class Jauge(QWidget):
    """Le petit rectangle a cote du compte a rebours."""

    def __init__(self):
        super().__init__()
        self.fraction = 0.0
        self.couleur = BLEU
        self.setFixedSize(190, 26)

    def pose(self, fraction, couleur=None):
        self.fraction = max(0.0, min(1.0, float(fraction)))
        if couleur is not None:
            self.couleur = couleur
        self.update()

    def paintEvent(self, _):
        p = QPainter(self)
        p.setRenderHint(QPainter.Antialiasing)
        cadre = self.rect().adjusted(1, 1, -2, -2)
        p.setPen(QPen(GRIS, 1.4))
        p.setBrush(QColor(246, 248, 251))
        p.drawRoundedRect(cadre, 3, 3)
        if self.fraction > 0:
            plein = cadre.adjusted(2, 2, -2, -2)
            plein.setWidth(int(plein.width() * self.fraction))
            p.setPen(Qt.NoPen)
            p.setBrush(self.couleur)
            p.drawRoundedRect(plein, 2, 2)


class Dialogue(QDialog):
    """Question, puis deroulement, puis verdict."""

    def __init__(self):
        super().__init__()
        self.setWindowTitle('Calibration extrinsèque')
        self.setMinimumWidth(640)
        self.resultat = None
        self.proc = None
        self.restant = 0.0
        self.attendu = duree_attendue()

        v = QVBoxLayout(self)
        self.titre = QLabel('Voulez-vous faire la calibration extrinsèque ?')
        self.titre.setStyleSheet('font-size:16px; font-weight:bold;')
        v.addWidget(self.titre)

        etat, _ = etat_extrinseque()
        self.sous_titre = QLabel(
            f'En place : {etat}.\n'
            f'À faire si la caméra a bougé. Dure environ '
            f'{self.attendu:.0f} s, le robot ne bouge pas.')
        self.sous_titre.setStyleSheet('color:#555;')
        v.addWidget(self.sous_titre)

        ligne = QHBoxLayout()
        self.compteur = QLabel('')
        self.compteur.setFont(QFont('monospace', 22, QFont.Bold))
        self.compteur.setFixedWidth(70)
        self.jauge = Jauge()
        self.etape = QLabel('')
        self.etape.setStyleSheet('color:#333;')
        ligne.addWidget(self.compteur)
        ligne.addWidget(self.jauge)
        ligne.addWidget(self.etape, 1)
        self.bloc_progression = QWidget()
        self.bloc_progression.setLayout(ligne)
        self.bloc_progression.setVisible(False)
        v.addWidget(self.bloc_progression)

        self.journal = QTextEdit()
        self.journal.setReadOnly(True)
        self.journal.setStyleSheet('font-family:monospace; font-size:11px;')
        self.journal.setFixedHeight(150)
        self.journal.setVisible(False)
        v.addWidget(self.journal)

        boutons = QHBoxLayout()
        self.bouton_oui = QPushButton('Oui — calibrer')
        self.bouton_oui.setDefault(True)
        self.bouton_non = QPushButton('Non — ouvrir le dashboard')
        self.bouton_ref = QPushButton('Relever la planche')
        self.bouton_ref.setToolTip(
            'À ne faire que si la PLANCHE a bougé et que l’extrinsèque est '
            'encore juste : relève la position réelle des 4 marqueurs pour '
            'servir de référence aux recalibrations.')
        self.bouton_oui.clicked.connect(lambda: self.lance([]))
        self.bouton_ref.clicked.connect(lambda: self.lance(['--reference']))
        self.bouton_non.clicked.connect(self.accept)
        boutons.addWidget(self.bouton_oui)
        boutons.addWidget(self.bouton_non)
        boutons.addStretch(1)
        boutons.addWidget(self.bouton_ref)
        v.addLayout(boutons)

        self.horloge = QTimer(self)
        self.horloge.timeout.connect(self._tic)

        self.controle = None
        self.tampon_controle = ''
        self.etat = etat
        if REFERENCE.exists() and VENV.exists():
            for b in (self.bouton_oui, self.bouton_ref):
                b.setEnabled(False)
            QTimer.singleShot(0, self._controle)

        if not REFERENCE.exists():
            self.bouton_oui.setEnabled(False)
            self.sous_titre.setText(
                f'En place : {etat}.\n'
                f'⚠ {REFERENCE.name} manque : la recalibration n’a pas de '
                f'référence de planche. Cliquer « Relever la planche » '
                f'pendant que l’extrinsèque est juste.')

    # ------------------------------------------------------------------ #

    def _controle(self):
        """Mesure l'ecart aux marqueurs avant que l'utilisateur ne decide.

        Un ecart dit que les marqueurs ont bouge OU que la camera a bouge ;
        il ne distingue pas les deux. Le seul controle qui tranche est la
        projection du squelette du robot sur son image.
        """
        self.sous_titre.setText(f'En place : {self.etat}.\n'
                                'Contrôle des marqueurs en cours…')
        # La camera ne se partage pas : tant que le controle la tient, lancer
        # une calibration la ferait echouer sur « can't open camera by index ».
        for b in (self.bouton_oui, self.bouton_ref):
            b.setEnabled(False)
        self.bouton_oui.setText('Oui — calibrer  (contrôle en cours…)')
        self.controle = QProcess(self)
        self.controle.setProcessChannelMode(QProcess.MergedChannels)
        self.controle.readyReadStandardOutput.connect(self._controle_sortie)
        self.controle.finished.connect(self._controle_fini)
        self.controle.start(str(VENV), [str(OUVRIER), '--controle', '--frames', '3'])

    def _controle_sortie(self):
        self.tampon_controle += bytes(
            self.controle.readAllStandardOutput()).decode('utf-8', 'replace')

    def _controle_fini(self, code, _statut):
        self.bouton_oui.setText('Oui — calibrer')
        self.bouton_oui.setEnabled(REFERENCE.exists())
        self.bouton_ref.setEnabled(True)
        mesure = None
        for ligne in self.tampon_controle.splitlines():
            if ligne.startswith('RESULTAT|'):
                mesure = json.loads(ligne.split('|', 1)[1])
        if code != 0 or mesure is None:
            motifs = [l.split('|', 1)[1] for l in self.tampon_controle.splitlines()
                      if l.startswith('ERREUR|')]
            self.sous_titre.setText(
                f'En place : {self.etat}.\n'
                'Contrôle impossible — '
                + (motifs[-1] if motifs else 'marqueurs non vus (bras devant, '
                   'ou câble noir contre une bordure).'))
            return
        pire, moyen = mesure['ecart_max_mm'], mesure['ecart_moyen_mm']
        detail = '  '.join(f'{i}:{e:.1f}' for i, e in
                           sorted(mesure['par_marqueur'].items()))
        verdict = ('rien à faire' if pire < 2.0 else
                   'recalibration conseillée' if pire < 20.0 else
                   'recalibration nécessaire')
        self.sous_titre.setText(
            f'En place : {self.etat}.\n'
            f'Marqueurs à {moyen:.2f} mm de la référence en moyenne, '
            f'{pire:.2f} mm au pire ({detail}) → {verdict}.\n'
            f'Dure environ {self.attendu:.0f} s, le robot ne bouge pas.')
        if pire >= 2.0:
            self.bouton_oui.setStyleSheet('font-weight:bold;')

    def lance(self, arguments):
        if not VENV.exists():
            self.ecrit('venv absent — ArUco indisponible', ROUGE)
            return
        for b in (self.bouton_oui, self.bouton_non, self.bouton_ref):
            b.setEnabled(False)
        self.titre.setText('Calibration en cours — ne pas bouger la caméra')
        self.bloc_progression.setVisible(True)
        self.journal.setVisible(True)
        self.restant = self.attendu
        self.compteur.setText(f'{self.restant:.0f}')
        self.horloge.start(1000)

        self.proc = QProcess(self)
        self.proc.setProcessChannelMode(QProcess.MergedChannels)
        self.proc.readyReadStandardOutput.connect(self._sortie)
        self.proc.finished.connect(self._fini)
        self.proc.start(str(VENV), [str(OUVRIER)] + arguments)

    def _tic(self):
        self.restant -= 1.0
        if self.restant > 0:
            self.compteur.setText(f'{self.restant:.0f}')
        else:
            self.compteur.setText('…')
            self.compteur.setStyleSheet('color:#888;')

    def ecrit(self, texte, couleur=None):
        teinte = couleur.name() if couleur is not None else '#222'
        self.journal.append(f'<span style="color:{teinte}">{texte}</span>')

    def _sortie(self):
        for brute in bytes(self.proc.readAllStandardOutput()).decode(
                'utf-8', 'replace').splitlines():
            ligne = brute.rstrip()
            if not ligne:
                continue
            if (ligne.startswith(('[ WARN', '[WARN', '[ERROR', '[ INFO',
                                  '[video4linux2'))
                    or 'obsensor' in ligne):
                continue        # bavardage d OpenCV pendant le sondage V4L2
            champs = ligne.split('|')
            canal = champs[0]
            if canal == 'ETAPE' and len(champs) >= 4:
                k, total, libelle = int(champs[1]), int(champs[2]), champs[3]
                self.jauge.pose(k / total)
                self.etape.setText(f'étape {k}/{total} — {libelle}')
            elif canal == 'INFO':
                self.ecrit(champs[1])
            elif canal == 'ERREUR':
                self.ecrit(champs[1], ROUGE)
            elif canal == 'RESULTAT':
                self.resultat = json.loads(champs[1])
            elif canal == 'DUREE':
                memorise_duree(float(champs[1]))
            else:
                self.ecrit(ligne, QColor(120, 120, 120))

    def _fini(self, code, _statut):
        self.horloge.stop()
        self.compteur.setText('')
        r = self.resultat or {}
        if code == 0 and r.get('mode') == 'reference':
            self.jauge.pose(1.0, VERT)
            self.titre.setText('Planche relevée — référence écrite')
            self.bouton_oui.setEnabled(True)
        elif code == 0 and r.get('ecrite'):
            self.jauge.pose(1.0, VERT)
            self.titre.setText(
                f'Calibrée — pire point neuf {r["pire_mm"]:.2f} mm, '
                f'RMS {r["rms_px"]:.2f} px, caméra déplacée de '
                f'{r["deplacement_mm"]:.0f} mm')
        elif r.get('mode') == 'recalibration' and r.get('ecrite') is False:
            self.jauge.pose(1.0, ROUGE)
            self.titre.setText('Calibration refusée — l’ancienne est conservée')
        else:
            self.jauge.pose(1.0, ROUGE)
            self.titre.setText('Calibration non aboutie — rien n’a été écrit')
        self.bouton_non.setEnabled(True)
        self.bouton_non.setText('Ouvrir le dashboard')
        self.bouton_non.setDefault(True)
        self.etape.setText('')


def demande(sauter=False):
    """Pose la question et rend le resultat, ou None si on a saute l etape.

    Exige qu'une QApplication existe deja : la fenetre du dashboard est
    construite apres, dans la meme application.
    """
    if sauter or not VENV.exists():
        return None
    fenetre = Dialogue()
    fenetre.exec_()
    return fenetre.resultat
