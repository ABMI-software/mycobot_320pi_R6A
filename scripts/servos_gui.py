#!/usr/bin/env python3
"""Deux boutons : ON fixe les servos, OFF les relache.

Sert a demarrer une seance. On allume la Pi, on lance cette fenetre, on met le
bras a la main ou on veut (OFF), on le fige (ON), puis on lance le tableau de
bord.

La connexion N'EST PAS gardee ouverte. Le pont de la Pi est mono-client et
bloquant : une fenetre qui garde sa socket empeche `pick_dashboard.py` de se
connecter ensuite, sans message d'erreur explicite. On ouvre donc le temps
d'une commande, et on referme.
"""
import json
import socket
import sys
import threading
from pathlib import Path

from PyQt5.QtCore import Qt, QTimer, pyqtSignal
from PyQt5.QtWidgets import (QApplication, QHBoxLayout, QLabel, QPushButton,
                             QVBoxLayout, QWidget)

sys.path.insert(0, str(Path(__file__).resolve().parent))
import pick_fsm as fsm

DIAMETRE = 130
PERIODE_MS = 3000
DELAI_SONDE = 0.5


def interroge(hote, port, action, delai):
    """Une commande, une reponse, socket refermee. Rend None si rien ne repond.

    On n'utilise PAS `fsm.Pont` pour sonder : son constructeur balaie le
    sous-reseau quand l'IP par defaut ne repond pas (jusqu'a ~5 s). Appele
    toutes les 3 s depuis le fil graphique, il figerait la fenetre pendant que
    la Pi demarre — exactement le moment ou cette fenetre sert.
    """
    try:
        with socket.create_connection((hote, port), timeout=delai) as sock:
            sock.settimeout(delai)
            sock.sendall((json.dumps({'action': action}) + '\n').encode())
            return sock.recv(4096).decode(errors='replace').strip()
    except OSError:
        return None

STYLE = """
QPushButton {{
    background: {fond};
    color: white;
    font-size: 30px;
    font-weight: bold;
    border: 5px solid {bord};
    border-radius: {rayon}px;
}}
QPushButton:hover  {{ background: {survol}; }}
QPushButton:pressed{{ background: {bord}; }}
QPushButton:disabled {{ background: #9e9e9e; border-color: #757575; }}
"""


def rond(texte, fond, bord, survol):
    bouton = QPushButton(texte)
    bouton.setFixedSize(DIAMETRE, DIAMETRE)
    bouton.setCursor(Qt.PointingHandCursor)
    bouton.setStyleSheet(STYLE.format(fond=fond, bord=bord, survol=survol,
                                      rayon=DIAMETRE // 2))
    return bouton


class Servos(QWidget):
    pi_trouvee = pyqtSignal(str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle('MyCobot — servos')
        self.setMinimumWidth(430)

        self.on = rond('ON', '#2e7d32', '#1b5e20', '#388e3c')
        self.off = rond('OFF', '#c62828', '#8e0000', '#d32f2f')
        self.on.clicked.connect(lambda: self._commande('power_on'))
        self.off.clicked.connect(lambda: self._commande('power_off'))

        ronds = QHBoxLayout()
        ronds.addStretch(1)
        for bouton, sous in ((self.on, 'servos fixés'), (self.off, 'servos relâchés')):
            colonne = QVBoxLayout()
            colonne.addWidget(bouton, alignment=Qt.AlignHCenter)
            etiquette = QLabel(sous)
            etiquette.setAlignment(Qt.AlignHCenter)
            etiquette.setStyleSheet('color:#555; font-size:12px;')
            colonne.addWidget(etiquette)
            ronds.addLayout(colonne)
            ronds.addStretch(1)

        self.lien = QLabel('connexion…')
        self.lien.setAlignment(Qt.AlignHCenter)
        self.lien.setStyleSheet('font-weight:bold;')
        self.pose = QLabel('—')
        self.pose.setAlignment(Qt.AlignHCenter)
        self.pose.setStyleSheet('font-family:monospace; font-size:11px; color:#666;')
        avertissement = QLabel('OFF : le bras devient mou — tiens-le avant de cliquer.')
        avertissement.setAlignment(Qt.AlignHCenter)
        avertissement.setStyleSheet('color:#c62828; font-size:11px;')

        page = QVBoxLayout(self)
        page.addWidget(self.lien)
        page.addLayout(ronds)
        page.addWidget(avertissement)
        page.addWidget(self.pose)

        self.recherche = False
        self.pi_trouvee.connect(self._adopte)
        self._actifs(False)
        self._sonde()
        self.horloge = QTimer(self)
        self.horloge.timeout.connect(self._sonde)
        self.horloge.start(PERIODE_MS)

    def _actifs(self, oui):
        self.on.setEnabled(oui)
        self.off.setEnabled(oui)

    def _etat(self, texte, couleur):
        self.lien.setText(texte)
        self.lien.setStyleSheet(f'font-weight:bold; color:{couleur};')

    def _sonde(self):
        """Une socket courte : la pose si le pont repond, sinon pourquoi non."""
        reponse = interroge(fsm.PI[0], fsm.PORT_PI, 'get_angles', DELAI_SONDE)
        if reponse is None:
            self._etat(f'Pi injoignable ({fsm.PI[0]})', '#c62828')
            self._actifs(False)
            self._cherche_une_fois()
            return
        if 'ANGLES' not in reponse.upper():
            # La socket s'ouvre mais la reponse n'est pas la notre : signature
            # d'un autre client deja connecte, le pont etant mono-client.
            self._etat('pont occupé par un autre programme', '#ef6c00')
            self._actifs(False)
            return
        self._etat(f'connecté à {fsm.PI[0]}:{fsm.PORT_PI}', '#2e7d32')
        try:
            q = json.loads(reponse.split(':', 1)[-1].strip())
            self.pose.setText('  '.join(f'J{i + 1} {v:+7.2f}' for i, v in enumerate(q)))
        except (json.JSONDecodeError, ValueError, TypeError):
            self.pose.setText(reponse)
        self._actifs(True)

    def _cherche_une_fois(self):
        """Balaye le sous-reseau, dans un fil, et une seule fois par lancement."""
        if self.recherche:
            return
        self.recherche = True
        self._etat('recherche de la Pi sur le réseau…', '#ef6c00')

        def travail():
            self.pi_trouvee.emit(fsm.trouve_la_pi() or '')

        threading.Thread(target=travail, daemon=True).start()

    def _adopte(self, hote):
        if hote:
            fsm.PI = (hote, fsm.PORT_PI)
            self._sonde()

    def _commande(self, action):
        self._actifs(False)
        try:
            pont = fsm.Pont(timeout=4.0)
        except OSError:
            self._etat('Pi injoignable — commande non envoyée', '#c62828')
            return
        try:
            reponse = pont.envoie(action)
        finally:
            pont.ferme()
        fixe = action == 'power_on'
        self._etat(('servos FIXÉS — ' if fixe else 'servos RELÂCHÉS — ') + reponse,
                   '#2e7d32' if fixe else '#c62828')
        QTimer.singleShot(400, self._sonde)


def main():
    application = QApplication(sys.argv)
    fenetre = Servos()
    fenetre.show()
    sys.exit(application.exec_())


if __name__ == '__main__':
    main()
