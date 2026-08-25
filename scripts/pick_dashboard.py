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

import json
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
# Teinte du carton — SANS plafond de luminosité. Un seuil de V absolu ne sépare
# pas le carton de la planche : mesuré le 24/08, carton H14 S171 V60 et planche
# H15 S187 V84, les deux dans l'ancienne fenêtre. Le détecteur prenait alors la
# planche entière pour un carton (tache de 30 000 px², centroïde à 300 mm de la
# vraie boîte). La couleur ne sert donc plus qu'à qualifier l'ENTOURAGE.
HSV_CARTON = ((3, 70, 0), (26, 255, 255))
ECART_SOMBRE = 12         # niveaux de V sous le fond local pour être un creux
CONTRASTE_MIN = 8.0       # V de l'anneau moins V du creux
ANNEAU_BRUN_MIN = 0.60    # fraction brune du pourtour du creux
# Le petit carton est NOIR a l'exterieur : son pourtour n'est pas brun mais
# sombre. Exiger le brun seul le rendait invisible.
ANNEAU_SOMBRE_MIN = 0.55
# Ouvertures REELLEMENT mesurees le 24/08 : 96x104 et 80x137 mm. La fourchette
# precedente (55 a 330 mm) laissait passer n'importe quelle grande zone sombre —
# c'est ainsi qu'un quadrilatere en travers de la planche est devenu le "grand
# carton" a (205, -16), au milieu de la table.
# Le grand cote monte a 202 mm sur le carton pose au BORD de la planche : loin de
# l'axe de la camera, son ouverture se projette allongee. Mesure du 24/08.
# Porte a 260 le 25/08 : a 210, le carton de gauche — 4400 px, anneau brun a
# 0,79, contraste 40, un carton parfait — etait jete pour 10 mm de trop
# (164x220 mm). Or ces 220 mm sont mesures a la hauteur SUPPOSEE du rebord ;
# a la hauteur reelle il n'en fait que 214. Le gabarit ne doit pas etre plus
# serre que l'incertitude sur le plan ou on le mesure. Sa vraie tache est
# d'ecarter la planche entiere (450 mm) et les petits objets, pas d'arbitrer
# au centimetre.
# Plancher abaisse a 45 mm le 25/08 : le petit carton mesure 115 x 70 mm hors
# tout, donc une ouverture d'une soixantaine de millimetres au petit cote — le
# plancher precedent, 60, le rejetait tout juste.
COTE_CARTON_MM = (45.0, 260.0)
# Rien de ce qui est a moins de ca du centre de la base n'est un carton : c'est
# le robot lui-meme. Sans ce garde-fou, le bras au repos etait detecte comme un
# creux de 70 x 164 mm a 57 mm de la base et prenait le nom de "petit carton" —
# c'est le "petit carton au milieu de la table" qui ne bougeait pas quand on
# deplacait le vrai (constate le 25/08). Le masque cinematique ne suffit pas :
# il exige les angles, donc le pont vers la Pi, et sans lui il ne masque rien.
# La zone de largage commence de toute facon a 200 mm.
RAYON_BASE_MIN = 200.0    # mm
# Et rien AU-DELA de la portee de largage n'est un carton utilisable : on ne
# saurait de toute facon pas y deposer. C'est ce qui manquait le 25/08 quand un
# fantome a (505,6 ; -34,2) — hors planche, 507 mm — est passe tous les filtres,
# a ete SUIVI, puis ECRIT sur le disque comme designation du grand carton. La
# continuite imposait ensuite ce faux point contre la regle de taille, qui elle
# donnait le bon resultat. Doit rester egal a fsm.PORTEE_CARTON_MAX.
RAYON_CARTON_MAX = 460.0  # mm
# Un objet a moins de ca du bord INTERIEUR de l'ouverture est considere depose.
# Negatif = on accepte un peu au-dela du bord : un objet appuye contre la paroi
# est dans la boite, meme si son centre projete tombe un cheveu dehors.
MARGE_DEPOSE = 15.0       # mm
# Part minimale du creux que doit garder son coeur sombre pour etre cru. Sous ce
# seuil, le seuillage a coupe dans l'ouverture elle-meme au lieu de la separer
# de l'ombre de la paroi.
PART_COEUR_MIN = 0.30
# Une ouverture est un rectangle : elle remplit sa boite englobante. Une ombre
# qui serpente, non.
REMPLISSAGE_CARTON_MIN = 0.60
# Un carton ne peut etre localise que si le bras est a plus de ca. Les cartons
# ne bougent pas tout seuls : mieux vaut garder la derniere position sure que
# d'en adopter une pendant que le bras encombre la vue.
DEGAGEMENT_CARTON = 200.0
AIRE_CREUX_MIN = 250      # px
# Les cartons sont poses au bord de la planche et debordent : a 35 mm de marge
# leur ouverture etait coupee par le masque et devenait informe. A 100 mm les
# deux sont vus entiers, sans laisser entrer le clavier ni la souris.
MARGE_PLATEAU = 100.0      # mm — les marqueurs sont en retrait des bords
COTE_MARQUEUR = 90.0      # mm — carré noir + bordure blanche, à masquer
PART_PLATEAU_MAX = 0.35   # au-delà, la tache brune EST la planche
HAUTEUR_CENTRE_BALLE = 35.0
# Mesure du 25/08 par triangulation des deux vues sur l'ouverture du grand
# carton : 82,9 mm, avec 11,8 mm d'ecart entre les deux rayons. La valeur
# precedente, 60, etait supposee. L'ecart n'est pas anodin : c'est le plan sur
# lequel se projette toute la geometrie des cartons, et entre Z=0 et Z=100 le
# centre projete d'un carton se deplace de 50 mm.
HAUTEUR_CARTON = 83.0     # rebord du carton, mesure
HAUTEUR_OBJET = 12.0      # mi-hauteur d'un rouleau de scotch couche
# Demi-epaisseur des segments du bras PLUS l'ombre qu'ils portent sur la
# planche. C'est cette ombre qui se faisait prendre pour l'ouverture du carton :
# creux sombre, entoure de brun, elle passait tous les tests (mesure du 24/08,
# carton annonce a 166 mm alors qu'il est a 449).
RAYON_BRAS = 80.0

# --- tri par categorie (24/08) ------------------------------------------------
# Les deux scotchs sont des ANNEAUX : un trou au milieu, que rien d'autre sur la
# planche ne possede. La couleur ne les separe pas de facon fiable — a
# l'exposition 75 le scotch bleu se lit V=48, presque noir (mesure du 24/08) —
# la geometrie, si. Le robot imprime est sombre et allonge ; la balle est un
# disque jaune plein.
# --------------------------------------------------------------------------- #
#  Mesures de reference — scene du 25/08/2026
# --------------------------------------------------------------------------- #
# Tout ce qui suit a ete MESURE sur le robot reel, et c'est ce qui justifie
# chacun des gabarits ci-dessous. Sans cette table, chaque seuil redevient un
# nombre magique et se refait resserrer par le premier qui passe.
#
# Objets, vus de dessus par l'arducam, projetes a HAUTEUR_OBJET :
#
#   scotch blanc   39,7 x 41,4 mm   trou 105 px   V142 S 68   -> petit carton
#   scotch bleu    35,1 x 36,8 mm   trou  43 px   V 65 S101   -> petit carton
#   petit robot    71 x 109 mm ramasse, 79 x 146 pattes etalees, V 39 S 58
#                  ventre 41 mm de large (rayon inscrit 20,3 mm)  -> grand
#   balle          disque jaune, detecteur dedie
#
#   Au pied a coulisse l'operateur donne 72,8 mm pour le blanc et 55 pour le
#   bleu, soit pres du double de ce que voit la camera. L'ecart n'est pas
#   explique ; les deux jeux tiennent dans le gabarit, qui est dimensionne sur
#   le plus grand des deux.
#
# Cartons, ouverture mesuree sur le COEUR SOMBRE du creux, a HAUTEUR_CARTON :
#
#   grand   113 x 125 mm  ~115 cm2  (+-7 sur 25 images)
#   petit    67 x 115 mm  ~ 74 cm2  (+-1 sur 25 images)
#
#   Soit 41 cm2 d'ecart pour +-7 de bruit : SIX FOIS le bruit, le gabarit les
#   separe donc tout seul. Ce n'etait pas le cas avant la correction du coeur
#   sombre — contours gonfles par l'ombre des parois, ils donnaient 160x214 et
#   144x205 mm, 16 % d'ecart pour 18 % de bruit, indiscernables.
#
#   Le rebord des deux est a 82,9 mm (triangulation des deux vues, 11,8 mm
#   d'ecart entre les rayons), et non 60 comme suppose jusqu'au 25/08.
#
# ATTENTION : un objet DEPOSE deforme le creux de sa boite — le scotch blanc a
# fait passer le petit carton de 74x127 a 83x172 mm. La separation ci-dessus ne
# vaut que boites vides ; c'est pourquoi l'identite est aussi tenue par la
# continuite et par la designation.

# Diametre exterieur de l'anneau. Plafond porte de 70 a 90 mm le 25/08 : le
# rouleau blanc mesure 72,8 mm au pied a coulisse, donc il etait rejete par le
# gabarit avant meme d'etre classe — un seul des deux scotchs etait detecte.
COTE_SCOTCH_MM = (28.0, 90.0)
# Un blob COMPACT sous cette taille ne peut etre qu'un scotch : le robot fait au
# moins 60 mm, une ouverture de carton au moins 45, la balle a son detecteur.
# C'est le repli quand le trou de l'anneau ne se forme pas — le rouleau bleu
# fait 32 x 38 mm, soit 16 x 19 px, et son trou passe de 43 px a rien du tout
# selon l'eclairage (mesure du 25/08). Compact, donc PETIT COTE compris lui
# aussi dans la fourchette : un morceau de cable fait 34 x 271 mm et sort.
COTE_SCOTCH_COMPACT = 60.0
AIRE_TROU_MIN = 18
MARGE_MASQUE_MARQUEUR = 1.6
# Ecart de luminosite au bois au-dela duquel un pixel est etranger. Le rouleau
# BLANC n'a que 39 unites d'ecart (V148 contre V109 pour le bois) : a 55, seul
# le coeur de son anneau passait, le bord fondait dans la planche et le rouleau
# se cassait en fragments de 16 x 33 mm — sous le gabarit, donc invisible une
# image sur cinq. Balayage du 25/08, 10 images, les deux rouleaux :
#
#   seuil 55 -> blanc  8/10     seuil 45 -> 10/10     seuil 35 -> bleu 4/10
#
# A 35 le veinage du bois entre dans le masque et fabrique de faux objets. 45
# laisse dix unites de marge de chaque cote.
ECART_VALEUR_BOIS = 45                # px — le plus petit trou mesure fait 28 px
# Plus grand cote. Le plancher a 60 mm separe le robot des SCOTCHS : mesure du
# 24/08, le robot fait 82x134 mm et les rouleaux 33x39 et 38x43. Un rouleau dont
# le trou n'est pas vu (il est sombre, V=48) tombait sinon dans la categorie
# robot — et partait vers le mauvais carton.
# Plafond porte de 140 a 200 mm le 25/08 : le petit robot a des membres
# articules, et son encombrement depend de la pose ou on le trouve — 71x109 mm
# ramasse, 79x146 pattes etalees. A 140 il etait rejete pour 6 mm. Ce qui le
# separe d'une OUVERTURE de carton reste sa largeur (LARGEUR_ROBOT_MAX), et
# desormais aussi le fait qu'un objet trouve DANS un carton est classe depose.
COTE_ROBOT_MM = (60.0, 200.0)
# Plus petit cote : c'est lui qui separe le robot d'une OUVERTURE de carton, qui
# est sombre elle aussi. Mesure du 24/08 : robot 82x134 mm, grand carton
# 127x140 mm. Sans cette borne, le carton se faisait ramasser comme un objet.
LARGEUR_ROBOT_MAX = 105.0
RAYON_OBJET_CARTON = 60.0         # mm — un creux si proche d'un objet EST cet objet
# Distance au-dela de laquelle une tache vue par la camera d'appui ne peut plus
# etre la meme chose que ce que l'arducam a identifie. Large : les deux vues se
# reperent a ~30 mm l'une de l'autre, et l'appui doit tolerer ca sans confondre
# deux objets voisins.
APPARIEMENT_MAX = 90.0
# Une fois l'objet choisi, on ne le lache plus des yeux : une detection de la
# meme categorie plus loin que ca N'EST PAS lui. Sans cette porte, le bras qui
# masque le scotch faisait sauter la cible sur l'autre rouleau — 311 mm d'un
# coup, mesure le 24/08 — et la machine repartait a zero.
PORTE_SUIVI_OBJET = 80.0
# C'est la ROBE qui separe les deux cartons, pas l'aire de leur ouverture : le
# GRAND est le carton brun, le PETIT est noir a l'exterieur. Mesure du 24/08 sur
# l'anneau de 25 px autour de l'ouverture : brun S174 V87, 2 % de pixels sous
# V=60 ; noir S148 V52, 59 %.
#
# Le sens de cette regle a ete inverse une fois, sur une reponse de l'operateur,
# puis remis d'aplomb par trois mesures concordantes :
#   * sa consigne d'origine — « le petit box a l'exterieur est noir, le grand
#     est marron » ;
#   * les ouvertures, une fois le masque du plateau elargi : 138x202 mm pour le
#     brun contre 62x113 mm pour le noir, soit quatre fois plus grand ;
#   * l'essai reel — le robot, dirige vers 'grand', a atterri dans le carton que
#     l'operateur appelle le petit.
NOIR_EXTERIEUR_MIN = 0.25
VALEUR_SOMBRE = 60                # V median en-deca duquel un objet est "noir"
# Sombre ne suffit pas : le bois a l'ombre et son veinage descendent sous V=60 et
# se faisaient prendre pour le robot — quatre faux positifs sur une planche vide,
# mesure du 24/08. Ce qui separe vraiment, c'est la SATURATION : le robot imprime
# est noir desature (S=44), le bois reste brun sature meme dans l'ombre (S=170).
# Descendu de 90 a 70 le 25/08. A 90, l'INTERIEUR du petit carton — sombre et
# peu sature, S=89, un point sous le seuil — etait classe "robot" : 72,6 x 115,6
# mm, pile son ouverture. L'ouverture disparaissait alors de la liste des
# cartons (un creux colle a un objet n'est pas retenu), le bras descendait dans
# la boite et se refermait sur du vide. Les valeurs mesurees sur le vrai robot
# sont S=33, 44 et 58 selon l'eclairage : 70 les garde toutes et ecarte le
# carton.
SATURATION_NOIRE_MAX = 70
AIRE_OBJET_MIN = 90               # px
# Destination de chaque categorie. Le petit carton est noir a l'exterieur, le
# grand est brun — c'est l'anneau autour de l'ouverture qui les separe.
DESTINATION = {'scotch': 'petit', 'balle': 'grand', 'robot': 'grand'}
# Categories qu'on saisit par leur endroit le plus EPAIS et non par le centroide
# de leur enveloppe. Le centroide suit les membres qui depassent : sur le petit
# robot la pince se refermait sur un bras. Le scotch, lui, garde le centroide —
# c'est le centre de son anneau.
PRISE_PAR_EPAISSEUR = {'robot'}
COULEUR_CARTON = {'grand': (255, 150, 0), 'petit': (0, 165, 255)}
COULEUR_OBJET = {'scotch': (0, 255, 0), 'robot': (255, 0, 255), 'balle': (0, 0, 255)}
FENETRE_DETECTION = 1.2   # s — age maximal d'une detection reutilisable
ECARTEMENT_MAX = 25.0     # mm — au-dela, les deux caméras ne voient pas le même objet
LISSAGE_CARTON = 0.45     # poids d'une image neuve dans la position suivie
SAUT_CARTON = 60.0        # mm — au-dela, ce n'est plus du tremblement
# Deux images concordantes suffisent — 0,2 s a 10 im/s. Six etaient necessaires
# tant que le bras et son ombre fabriquaient des sauts de 50 a 170 mm ; depuis
# que sa silhouette est retiree de l'image par la cinematique, il n'en reste
# qu'un cas : une detection aberrante isolee, et une seule image ne confirme
# rien. Deplacer le carton a la main se voit donc immediatement.
CONFIRMATIONS_CARTON = 2
PEREMPTION_CARTON = 4.0   # s — au-delà, la position suivie n'est plus fraîche
ECART_TAILLE_MAX = 0.45   # une ouverture ne change pas de taille : au-delà, autre objet
MARGE_TRACE_CARTON = 15.0 # mm — le creux sous-estime l'ouverture, le tracé la rattrape

# Emprise du plateau, deduite des marqueurs pour ne pas dériver de leur fichier.
# Ils sont en retrait des bords : d'où la marge.
_MARQUEURS = yaml.safe_load(
    (CALIB / 'workspace_markers.yaml').read_text())['markers']
_XY_MARQUEURS = np.array([v[:2] for v in _MARQUEURS.values()], float)
# Tour du plateau dans l'ordre : loin-gauche, loin-droite, proche-droite,
# proche-gauche — un ordre quelconque donnerait un polygone croisé.
_TOUR_PLATEAU = (25, 26, 23, 19)
PLATEAU = ((_XY_MARQUEURS[:, 0].min() - 40.0, _XY_MARQUEURS[:, 0].max() + 40.0),
           (_XY_MARQUEURS[:, 1].min() - 40.0, _XY_MARQUEURS[:, 1].max() + 40.0))

# Disposition du graphe, en coordonnees normalisees. La boucle principale fait le
# tour, ECHEC est au centre : toutes les sorties d'erreur y convergent.
NOEUDS = {
    'ATTENTE': (0.06, 0.52), 'DEGAGEMENT': (0.10, 0.28), 'DETECTION': (0.30, 0.10),
    'APPROCHE': (0.54, 0.06), 'RECALAGE': (0.78, 0.14), 'DESCENTE': (0.92, 0.36),
    'SAISIE': (0.92, 0.64), 'REMONTEE': (0.80, 0.88), 'TRANSFERT': (0.54, 0.94),
    'LARGAGE': (0.30, 0.90), 'RETRAIT': (0.06, 0.72), 'ECHEC': (0.50, 0.44),
    'RECHERCHE_CARTON': (0.68, 0.70), 'ECHEC_PORTANT': (0.38, 0.70),
}
def lit_controles(index):
    """{nom: valeur} des controles d'exposition lus sur le peripherique."""
    sortie = subprocess.run(['v4l2-ctl', '-d', f'/dev/video{index}', '--get-ctrl',
                             'auto_exposure,exposure_time_absolute'],
                            capture_output=True, text=True, timeout=5).stdout
    return {m.group(1): int(m.group(2))
            for m in (re.match(r'\s*(\w+):\s*(-?\d+)', l) for l in sortie.splitlines()) if m}


def sous_le_bras(xy, angles):
    """Une ouverture "vue" pres du bras est le bras ou son ombre.

    Trois fois de suite le 24/08, la machine a vise un carton fantome au MILIEU
    DE LA TABLE — (211, 1), (205, -16), puis (212, 6) pris sur le fait a 140 mm
    du bras, 18 089 mm2 — et y a lache l'objet, pendant que le suivi affichait le
    vrai carton a (364, 161). L'ombre du bras est la seule source de detections a
    la fois CONCORDANTES et fausses : elle le suit image apres image, donc elle
    se confirme aussi bien qu'un vrai deplacement, et aucun filtre de forme ou de
    taille ne l'arrete — celle-la passait les deux.

    La distance se mesure au bras ENTIER, pas a sa pointe : ce sont ses segments
    qui portent l'ombre, et ils couvrent bien plus que leur bout. Les cartons ne
    bougent pas tout seuls : garder la derniere position sure vaut toujours mieux
    que d'en adopter une pendant que le bras encombre la vue.
    """
    if angles is None:
        return False
    return fsm.distance_au_bras(xy, np.asarray(angles, float)) < DEGAGEMENT_CARTON


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


def nom_par_aire(aire):
    """Nom d'un carton d'apres sa SEULE aire d'ouverture, ou None si c'est trop
    juste pour trancher.

    C'est le cas qui faisait tout basculer : un seul carton visible, l'autre
    masque par le bras. Le code se rabattait alors sur "le plus grand des
    restants est le grand", donc le PETIT carton vu seul devenait le grand — et
    la continuite figeait l'erreur pour toute la seance.

    La frontiere est la moyenne GEOMETRIQUE des deux aires attendues, pas leur
    moyenne arithmetique : l'erreur de mesure d'une aire est relative, pas
    absolue, et la moyenne geometrique est le point equidistant des deux au sens
    du rapport.
    """
    frontiere = float(np.sqrt(AIRE_CARTON_ATTENDUE['grand']
                              * AIRE_CARTON_ATTENDUE['petit']))
    if abs(aire - frontiere) < BANDE_MORTE_AIRE * frontiere:
        return None
    return 'grand' if aire > frontiere else 'petit'


def plausible(xy):
    """Cette position peut-elle etre celle d'un carton ?

    Un garde-fou a l'ECRITURE, pas seulement a la detection. Le 25/08 un
    fantome a (505,6 ; -34,2) a ete suivi puis grave sur le disque comme
    designation du grand carton : la continuite l'a ensuite impose a chaque
    image contre la regle de taille, qui donnait le bon resultat. Une
    designation fausse est pire qu'une designation absente — elle survit aux
    relances.
    """
    if xy is None:
        return False
    return RAYON_BASE_MIN <= float(np.hypot(*np.asarray(xy, float))) <= RAYON_CARTON_MAX


class VueCliquable(QLabel):
    """Flux camera qui rend le pixel CLIQUE, dans le repere de l'image.

    Le pixmap est mis a l'echelle en gardant les proportions : il est donc
    centre, avec des bandes noires. Sans corriger ce centrage, le clic tombe a
    cote.
    """

    clique = pyqtSignal(str, float, float)

    def __init__(self, nom):
        super().__init__('en attente de flux…')
        self.nom = nom
        self.taille_image = None

    def mousePressEvent(self, evenement):
        pixmap = self.pixmap()
        if pixmap is None or self.taille_image is None:
            return
        x = evenement.x() - (self.width() - pixmap.width()) / 2.0
        y = evenement.y() - (self.height() - pixmap.height()) / 2.0
        if not (0 <= x < pixmap.width() and 0 <= y < pixmap.height()):
            return
        largeur, hauteur = self.taille_image
        self.clique.emit(self.nom, x * largeur / pixmap.width(),
                         y * hauteur / pixmap.height())


# --------------------------------------------------------------------------- #
#  Marqueurs des cartons
# --------------------------------------------------------------------------- #

MARQUEUR_CARTON = {10: 'grand', 11: 'petit'}
# Deux cartons peuvent avoir la MEME ouverture — mesure du 25/08 : 160x214 et
# 144x205 mm, soit 5 % d'ecart, sous le bruit de la detection. Aucun gabarit ne
# les separe alors, et l'etiquette bascule d'une image a l'autre. Ce qui les
# separe, c'est qu'ils ne sont pas au meme endroit : un carton deja nomme garde
# son nom tant qu'il reste pres de la ou on l'a vu.
CONTINUITE_CARTON = 150.0      # mm
# Ecart relatif d'aire au-dela duquel les deux ouvertures se separent d'elles-
# memes, sans avoir besoin ni de la continuite ni d'une designation. Mesure du
# 25/08, les deux cartons vides et le bras degage : 126 +-3 cm2 contre 77 +-1,
# soit 39 % d'ecart pour 3 % de bruit. A 25 % on est encore loin du bruit et
# largement sous l'ecart reel.
ECART_TAILLE_DECISIF = 0.25
# Aire d'ouverture ATTENDUE de chaque carton, en mm2 — mesuree le 25/08, bras
# degage, boites vides, sur 15 images : 126 +-3 cm2 et 77 +-1 cm2.
#
# Elle sert au cas qui faisait tout basculer : UN SEUL carton visible, l'autre
# masque par le bras. Sans elle, le code se rabattait sur "le plus grand des
# restants est le grand" — donc le PETIT carton vu seul devenait le grand, et la
# continuite figeait ensuite l'erreur pour toute la seance. Un carton seul doit
# etre MESURE, pas suppose.
#
# Si les boites changent, ces deux valeurs sont a re-mesurer : le banc
# scratchpad/stabilite_cartons.py les sort en une commande.
AIRE_CARTON_ATTENDUE = {'grand': 12600.0, 'petit': 7700.0}
# Bande morte autour de la moyenne geometrique des deux, en deca de laquelle un
# carton seul n'est pas assez tranche pour se nommer par sa seule aire.
BANDE_MORTE_AIRE = 0.15
DESIGNATION_CARTONS = RACINE / 'scripts' / 'cartons_designes.json'
COTE_MARQUEUR_CARTON = 30.0    # mm — carre noir, bordure blanche exclue
PORTE_MARQUEUR_CARTON = 220.0  # mm — distance max marqueur <-> ouverture
# Un carre plan ne rend sa PROFONDEUR qu'a Z * bruit_coin / cote_px pres. La
# camera est a ~1 m, un coin se pointe a ~0,3 px : il faut donc ~30 px de cote
# pour connaitre la hauteur a 10 mm. Les marqueurs colles le 25/08 font 30 mm,
# soit ~15 px a 2,01 mm/px — leur profondeur est bruitee de ~20 mm, plus que
# l'ecart qu'on cherche a mesurer. On ne leur demande donc que le NOM, et la
# hauteur reste HAUTEUR_CARTON. Reimprimes plus grands, ils repassent ce seuil
# et redonnent la hauteur sans qu'on touche a quoi que ce soit.
COTE_MARQUEUR_PX_MIN = 30.0
# Un marqueur colle sur un rabat EST une tache etrangere sombre et compacte de
# 30 mm : le detecteur d'objets a classe id 11 comme un rouleau de scotch a
# (381, -195) le 25/08, et le bras serait alle pincer l'autocollant. Les
# marqueurs de la planche sont deja retires par `masque_plateau` — leurs places
# sont fixes — mais ceux des cartons bougent avec les boites, donc on les
# retire image par image, a partir du quadrilatere detecte. Facteur mesure sur
# la scene du 25/08 : l'autocollant entier (papier blanc compris) s'etend a
# 1,30-1,33 fois le rayon du carre noir ; 1,6 laisse la marge.
# Le marqueur ne donne sa hauteur au solveur que s'il est PLAUSIBLEMENT sur le
# rebord. Colle sur un rabat rabattu a plat sur la table — le seul endroit
# horizontal qu'offrent certains cartons — il est a Z=0, et prendre cette
# hauteur pour celle du rebord decalerait le centre de l'ouverture de 50 mm.
# En deca de ce seuil, on ne retient du marqueur que le NOM.
REBORD_MARQUEUR_MIN = 40.0     # mm


class Marqueurs:
    """Detection ArUco deportee dans un interpreteur qui en est capable.

    `cv2.aruco` fait segfaulter l'OpenCV 4.6 du systeme, ou tourne ce tableau de
    bord. Un processus du venv reste ouvert et recoit les images par un tube :
    4,5 ms par image, contre ~1 s si on relancait un interpreteur a chaque fois.
    Le service absent, tout ce qui suit retombe silencieusement sur la
    geometrie seule — c'est un supplement d'information, pas une dependance.
    """

    def __init__(self):
        self.proc = None
        venv = RACINE / '.venv' / 'bin' / 'python'
        service = RACINE / 'scripts' / 'aruco_service.py'
        if not (venv.exists() and service.exists()):
            return
        try:
            self.proc = subprocess.Popen(
                [str(venv), str(service)], stdin=subprocess.PIPE,
                stdout=subprocess.PIPE, stderr=subprocess.DEVNULL)
            self.proc.stdout.readline()
        except OSError:
            self.proc = None

    def coins(self, image):
        """{id: coins 4x2} — {} si le service est absent ou muet."""
        if self.proc is None or self.proc.poll() is not None:
            return {}
        h, w = image.shape[:2]
        try:
            self.proc.stdin.write((json.dumps({'h': h, 'w': w}) + '\n').encode())
            self.proc.stdin.write(np.ascontiguousarray(image).tobytes())
            self.proc.stdin.flush()
            ligne = self.proc.stdout.readline()
        except (BrokenPipeError, OSError):
            self.proc = None
            return {}
        if not ligne:
            self.proc = None
            return {}
        return {int(k): np.array(v, float) for k, v in json.loads(ligne).items()}

    def ferme(self):
        if self.proc is not None and self.proc.poll() is None:
            self.proc.stdin.close()
            self.proc.wait(timeout=2)
        self.proc = None


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
        # Par le registre, PAS par le .npz brut : la SVPRO est calibree en
        # 800x600 et lue en 640x480. Avec la matrice brute, les marqueurs se
        # reprojettent a 200 mm de leur position — mesure du 24/08.
        self.K, self.dist = registre.load_intrinsics(d['intrinsics_stem'])
        self.source = fichier.name
        self._plateau = None

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

    def pose_marqueur(self, coins):
        """(xy base mm, z mm, cote mesure mm) d'un marqueur suppose HORIZONTAL.

        Colle a plat sur le rabat du carton, son plan EST celui du rebord : sa
        hauteur donne donc a la fois la garde au largage et le plan sur lequel
        projeter l'ouverture. IPPE_SQUARE est fait pour ce cas — quatre points
        coplanaires dont on connait l'ordre.
        """
        cote = COTE_MARQUEUR_CARTON / 1000.0
        modele = np.array([[-cote / 2, cote / 2, 0.0], [cote / 2, cote / 2, 0.0],
                           [cote / 2, -cote / 2, 0.0], [-cote / 2, -cote / 2, 0.0]])
        ok, rvec, tvec = cv2.solvePnP(modele, np.asarray(coins, float), self.K,
                                      self.dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            return None
        R, t = self.T[:3, :3], self.T[:3, 3]
        monde = (R.T @ (tvec.reshape(3) - t)) * 1000.0
        z = float(np.clip(monde[2], 0.0, 200.0))
        # Le CENTRE vient du rayon, pas de la translation PnP : la profondeur
        # d'une cible plane est bruitee, sa direction ne l'est pas. On garde
        # donc de PnP la seule hauteur, et on redescend le rayon sur ce plan-la.
        # Trop petit, le marqueur ne dit meme plus sa hauteur (cf.
        # COTE_MARQUEUR_PX_MIN) : il ne reste que le nom, qui lui est exact.
        quad = np.asarray(coins, float).reshape(4, 2)
        cote_px = float(np.mean([np.linalg.norm(quad[i] - quad[(i + 1) % 4])
                                 for i in range(4)]))
        centre = quad.mean(axis=0)
        return (self.vers_base(centre, z)[:2],
                z if cote_px >= COTE_MARQUEUR_PX_MIN else None, cote_px)

    def cartons_marques(self, image, marqueurs):
        """{classe: (xy du marqueur, z du rebord)} vu par les marqueurs colles.

        Les heuristiques de robe et de gabarit ont bascule d'une image a l'autre
        des que les deux cartons changeaient de place (24-25/08) : l'ouverture
        du lointain se mesure a la hauteur supposee du rebord, et cette hauteur
        etait fausse de 23 mm. Un marqueur ne se trompe pas de carton.
        """
        rendus = {}
        for ident, coins in (marqueurs or {}).items():
            classe = MARQUEUR_CARTON.get(ident)
            if classe is None:
                continue
            pose = self.pose_marqueur(coins)
            if pose is not None:
                xy, z = pose[:2]
                rendus[classe] = (xy, z if z is not None and z >= REBORD_MARQUEUR_MIN
                                  else None)
        return rendus

    def quad_plateau(self):
        """Polygone image de la planche, deduit des marqueurs et elargi."""
        xy = np.array([_MARQUEURS[i][:2] for i in _TOUR_PLATEAU], float)
        centre = xy.mean(axis=0)
        rayons = np.linalg.norm(xy - centre, axis=1)[:, None]
        elargi = centre + (xy - centre) * (1.0 + MARGE_PLATEAU / rayons)
        return np.array([self.vers_pixel([q[0], q[1], 0.0]) for q in elargi], np.int32)

    def masque_plateau(self, forme):
        """Planche seule, marqueurs exclus. Camera et extrinseque fixes : une
        seule fois. Sans ce masque, le clavier, la moquette et le pied de
        lampe fournissent des taches sombres plus grandes que le carton."""
        if self._plateau is not None and self._plateau.shape == forme:
            return self._plateau
        masque = np.zeros(forme, np.uint8)
        cv2.fillConvexPoly(masque, self.quad_plateau(), 255)
        marqueurs = np.zeros(forme, np.uint8)
        demi = COTE_MARQUEUR / 2.0
        for q in _MARQUEURS.values():
            coins = [[q[0] + dx, q[1] + dy, 0.0]
                     for dx, dy in ((-demi, -demi), (demi, -demi), (demi, demi), (-demi, demi))]
            cv2.fillConvexPoly(marqueurs,
                               np.array([self.vers_pixel(c) for c in coins], np.int32), 255)
        self._plateau = cv2.bitwise_and(masque, cv2.bitwise_not(marqueurs))
        return self._plateau

    def masque_bras(self, forme, angles):
        """Silhouette du bras dans l'image, deduite de la cinematique directe.

        Le bras est le seul objet mobile de la scene et sa position est CONNUE
        — inutile de la deviner sur l'image. On projette la chaine articulaire
        et on l'epaissit de son ombre.
        """
        positions, _ = fsm.forward_kinematics(np.radians(np.asarray(angles, float)))
        chaine = [np.asarray(v, float) * 1000.0 for v in positions.values()]
        chaine.append(fsm.pointe(np.asarray(angles, float)))
        masque = np.zeros(forme, np.uint8)
        precedent = None
        for point in chaine:
            uv = self.vers_pixel(point)
            rayon = int(max(10.0, np.linalg.norm(
                self.vers_pixel(point + np.array([RAYON_BRAS, 0.0, 0.0])) - uv)))
            cv2.circle(masque, tuple(uv.astype(int)), rayon, 255, -1)
            if precedent is not None:
                cv2.line(masque, tuple(precedent.astype(int)),
                         tuple(uv.astype(int)), 255, 2 * rayon)
            precedent = uv
        return masque

    @staticmethod
    def _ne_du_decoupage(contour, silhouette):
        """Le contour longe-t-il le bord du bras decoupe ?

        Retirer le bras de l'image cree un creux neuf le long de sa silhouette :
        son ombre, privee de la piece blanche, devient une tache sombre cerclee
        de brun, de la taille d'une ouverture (mesure du 24/08 : 184 mm annonces
        au lieu de 438). Mais rejeter tout ce qui TOUCHE le bras elargi jette
        aussi les vrais objets poses a cote — le petit carton et le robot
        disparaissaient. Ce qui distingue l'artefact, c'est qu'il EPOUSE le
        bord : on mesure donc la part de son contour collee a la silhouette.
        """
        if silhouette is None:
            return False
        proche = cv2.dilate(silhouette, np.ones((13, 13), np.uint8))
        points = contour.reshape(-1, 2)
        colles = sum(1 for u, v in points if proche[int(v), int(u)] > 0)
        return colles > 0.5 * len(points)

    def zone_utile(self, forme, angles=None):
        """(planche sans les marqueurs ni le bras, silhouette du bras nue).

        Le second masque n'est pas decoratif : DECOUPER le bras cree un creux
        neuf le long de son contour — son ombre, privee de sa silhouette
        blanche, devient une tache sombre cerclee de brun de la taille d'une
        ouverture. Mesure du 24/08 : elle l'emportait sur le vrai carton
        (184 mm annonces au lieu de 438). Tout candidat qui touche ce bord est
        donc ne du decoupage, pas de la scene.
        """
        plateau = self.masque_plateau(forme)
        if angles is None:
            return plateau, None
        bras = self.masque_bras(forme, angles)
        return cv2.bitwise_and(plateau, cv2.bitwise_not(bras)), bras

    def point_le_plus_epais(self, plein):
        """Pixel le plus ENFONCE dans la tache : (u, v).

        Le centroide de l'enveloppe convexe suit les membres qui depassent : sur
        le petit robot, la pince se refermait sur un bras au lieu du torse
        (constate le 25/08). Le point le plus eloigne du bord, lui, est par
        construction l'endroit le plus epais — le torse — donc celui ou la pince
        a le plus de matiere a serrer.

        On ne prend pas l'argmax brut, qui tient a un pixel : on moyenne tout ce
        qui est a plus de 85 % de l'epaisseur maximale, ce qui recentre le point
        dans la zone epaisse au lieu de le coller a son sommet.
        """
        distance = cv2.distanceTransform(plein, cv2.DIST_L2, 5)
        sommet = float(distance.max())
        if sommet <= 0.0:
            return None
        v, u = np.nonzero(distance >= 0.85 * sommet)
        return float(u.mean()), float(v.mean())

    @staticmethod
    def sans_marqueurs(masque, marqueurs):
        """Le meme masque, les autocollants des cartons effaces."""
        if not marqueurs:
            return masque
        net = masque.copy()
        for coins in marqueurs.values():
            quad = np.asarray(coins, float).reshape(4, 2)
            centre = quad.mean(axis=0)
            elargi = centre + (quad - centre) * MARGE_MASQUE_MARQUEUR
            cv2.fillConvexPoly(net, elargi.astype(np.int32), 0)
        return net

    def objets(self, image, angles=None, marqueurs=None):
        """Objets a trier poses sur la planche : [(classe, xy_base, contour)].

        Trois signatures, mesurees le 24/08 sur la scene reelle :

        * `scotch` — ANNEAU, ou a defaut BLOB COMPACT de moins de
          `COTE_SCOTCH_COMPACT`. Le trou reste le meilleur indice, mais il ne se
          forme pas toujours : le rouleau bleu fait 16 x 19 px a l'image et son
          trou passe de 43 px a rien selon l'eclairage. A cette taille-la,
          aucune autre categorie n'est possible — le robot fait au moins 60 mm,
          une ouverture de carton au moins 45, la balle a son propre detecteur.
        * `robot` — tache SOMBRE et allongee (V median < 60), 25 a 140 mm.
        * `balle` — disque jaune plein, rendu par `balle()` qui la connait deja.

        Le bras est retire de l'image avant tout, sinon sa silhouette et son
        ombre fournissent des taches sombres de la bonne taille. Les marqueurs
        des cartons le sont aussi : un carre noir de 30 mm sur son papier blanc
        a exactement la signature d'un rouleau de scotch.
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        teinte, saturation, valeur = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
        plateau = self.masque_plateau(valeur.shape)
        silhouette = (self.masque_bras(valeur.shape, angles)
                      if angles is not None else None)
        bois = (int(np.median(teinte[plateau > 0])),
                int(np.median(saturation[plateau > 0])),
                int(np.median(valeur[plateau > 0])))
        ecart_teinte = np.minimum(np.abs(teinte.astype(int) - bois[0]),
                                  180 - np.abs(teinte.astype(int) - bois[0]))
        brut = (((ecart_teinte > 12)
                 | (np.abs(saturation.astype(int) - bois[1]) > 55)
                 | (np.abs(valeur.astype(int) - bois[2]) > ECART_VALEUR_BOIS))
                & (plateau > 0)).astype(np.uint8) * 255
        etranger = cv2.morphologyEx(brut, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        # Puis on RECOLLE l'anneau. C'est l'ouverture qui le casse, pas la
        # fermeture qui le bouche — mesure du 25/08 sur les deux rouleaux :
        #
        #                        aire      trou
        #   masque brut       206/236 px  54/98 px
        #   apres OPEN 3x3    148/132 px   0/0     <- l'anneau est rompu
        #   OPEN puis CLOSE   206/231 px  51/96 px
        #
        # Rompu, l'anneau perd son trou ET un tiers de son aire : il ne tenait
        # plus que par la regle du blob compact, et deux arcs separes seraient
        # passes chacun sous AIRE_OBJET_MIN. Referme, il retrouve sa signature
        # entiere avec deux fois la marge.
        #
        # Le noyau vaut 5 et pas 3 : un 3x3 ne recolle qu'une cassure d'UN
        # pixel, et l'anneau se rompt plus large des qu'il s'eloigne du centre
        # de l'image. Il n'a plus a menager le trou, qui se mesure desormais sur
        # `brut` (cf. `creux_enferme`) — la fermeture ne sert qu'a recoller. On
        # garde malgre tout le plus petit noyau qui suffise : a 5 elle ne relie
        # que ce qui est distant de 8 mm, a 7 de 12 mm, et deux objets poses
        # cote a cote finiraient par n'en faire qu'un.
        etranger = cv2.morphologyEx(etranger, cv2.MORPH_CLOSE,
                                    np.ones((5, 5), np.uint8))
        etranger = self.sans_marqueurs(etranger, marqueurs)

        contours, hierarchie = cv2.findContours(etranger, cv2.RETR_CCOMP,
                                                cv2.CHAIN_APPROX_SIMPLE)
        trouves = []
        for i, contour in enumerate(contours):
            if hierarchie[0][i][3] != -1:            # c'est un trou, pas un objet
                continue
            if cv2.contourArea(contour) < AIRE_OBJET_MIN:
                continue
            plein = np.zeros(valeur.shape, np.uint8)
            cv2.drawContours(plein, [contour], -1, 255, -1)
            # Ici on ne peut pas rejeter au SIMPLE CONTACT du bras comme pour le
            # carton : un objet pose a cote de lui touche son masque elargi et
            # disparaitrait. Mesure du 24/08 : le robot imprime, a 30 mm du
            # bras, etait rejete a chaque image. On ne rejette donc que ce qui
            # est majoritairement le bras.
            if angles is not None:
                dedans = np.count_nonzero(cv2.bitwise_and(plein, silhouette))
                if dedans > 0.4 * np.count_nonzero(plein):
                    continue
            troue = self.creux_enferme(plein, brut) >= AIRE_TROU_MIN
            # A HAUTEUR_OBJET, pas au plan du rebord : un objet pose sur la
            # planche mesure a 83 mm se lit 7 % trop grand, et le rouleau blanc
            # passait ainsi par-dessus le plafond du gabarit scotch.
            petit, grand = self._cotes_mm(cv2.convexHull(contour), HAUTEUR_OBJET)
            sombre = (int(np.median(valeur[plein > 0])) < VALEUR_SOMBRE
                      and int(np.median(saturation[plein > 0])) < SATURATION_NOIRE_MAX)
            if troue and COTE_SCOTCH_MM[0] <= petit and grand <= COTE_SCOTCH_MM[1]:
                classe = 'scotch'
            elif COTE_SCOTCH_MM[0] <= petit and grand <= COTE_SCOTCH_COMPACT:
                classe = 'scotch'
            elif (sombre and COTE_ROBOT_MM[0] <= grand <= COTE_ROBOT_MM[1]
                  and petit <= LARGEUR_ROBOT_MAX):
                classe = 'robot'
            else:
                continue
            moments = cv2.moments(cv2.convexHull(contour))
            uv = (moments['m10'] / moments['m00'], moments['m01'] / moments['m00'])
            if classe in PRISE_PAR_EPAISSEUR:
                # Le scotch garde le centroide : c'est le centre de son anneau,
                # et la pince doit se refermer dessus, pas sur la bande.
                epais = self.point_le_plus_epais(plein)
                if epais is not None:
                    uv = epais
            trouves.append((classe, self.vers_base(uv, HAUTEUR_OBJET)[:2], contour))
        return trouves

    @staticmethod
    def creux_enferme(plein, brut):
        """Plus grande poche de fond enfermee dans un contour, en pixels.

        Mesuree sur le masque AVANT les morphologies, pas sur les contours-fils
        d'apres : la fermeture qui recolle l'anneau retrecit son trou, et la
        signature du scotch ne doit pas dependre du reglage qui le repare. Le
        contour rempli est erode d'un pixel pour que son propre liseré ne
        compte pas comme du fond.
        """
        dedans = cv2.erode(plein, np.ones((3, 3), np.uint8))
        creux = cv2.bitwise_and(dedans, cv2.bitwise_not(brut))
        nombre, _, stats, _ = cv2.connectedComponentsWithStats(creux, 8)
        return max((stats[j, cv2.CC_STAT_AREA] for j in range(1, nombre)),
                   default=0)

    def polygone_base(self, contour, z=None):
        """Ouverture du carton en mm dans le repere base, a hauteur de rebord.

        Le largage n'a pas a viser le centre : n'importe quel point de
        l'ouverture convient, et le bord proche est bien plus accessible que le
        centre (mesure du 24/08 : sommets de 290 a 427 mm de portee pour un
        centre a 363 mm, hors d'atteinte).
        """
        return np.array([self.vers_base(p, z if z is not None else HAUTEUR_CARTON)[:2]
                         for p in cv2.convexHull(contour).reshape(-1, 2)], float)

    def _cotes_mm(self, contour, z=None):
        """Petit et grand cote reels du contour, projete a hauteur de rebord.

        `z` est la hauteur du plan de projection. La laisser a la constante
        alors que le rebord mesure est a 83 mm gonfle l'ouverture du carton
        lointain de pres de moitie — assez pour lui faire voler la place du
        grand (mesure du 25/08 : 119x234 mm vue par la SVPRO contre 150x204
        par l'arducam, le meme carton).
        """
        coins = cv2.boxPoints(cv2.minAreaRect(contour))
        base = np.array([self.vers_base(c, z if z is not None else HAUTEUR_CARTON)[:2]
                         for c in coins])
        cotes = (np.linalg.norm(base[1] - base[0]), np.linalg.norm(base[2] - base[1]))
        return min(cotes), max(cotes)

    def carton(self, image, dernier=None, angles=None):
        """(x, y) base du centre du carton et son contour image, ou None.

        Le carton et la planche ont la meme teinte : la couleur seule ne les
        separe pas (voir HSV_CARTON). Ce qui distingue le carton, c'est que son
        OUVERTURE est un creux plus sombre que le bois, entoure de brun sur
        tout son pourtour. On cherche donc un ecart LOCAL de luminosite, puis
        on qualifie l'entourage. La couleur ne sert qu'en repli.

        `dernier` (position deja connue, mm) departage deux candidats a egalite
        — utile quand le bras projette une ombre au bord du plateau.
        """
        trouve = self._carton_par_creux(image, dernier, angles)
        return trouve if trouve is not None else self._carton_par_couleur(image, angles)

    def points_interessants(self, image, angles=None, marqueurs=None):
        """Tout ce qui n'est pas le fond, SANS le classer : [(xy base, contour)].

        C'est ce que la camera d'appui doit rendre. Lui faire classifier seule
        depuis sa vue oblique la faisait contredire l'arducam — le plateau
        etiquete "petit carton", la boite noire etiquetee "robot" (constate le
        24/08). Elle ne nomme donc plus rien : elle fournit des positions, et
        c'est l'arducam qui dit ce que chacune est.
        """
        points = [(xy, contour) for _, xy, contour
                  in self.objets(image, angles, marqueurs)]
        points += [(xy, contour) for _, xy, contour, _, _
                   in self._creux_candidats(image, angles)]
        return points

    def cartons(self, image, angles=None, objets=(), marqueurs=None,
                connus=None):
        """Les cartons vus : [(classe, xy, contour, hauteur du rebord)].

        Le nom se decide par trois sources, de la plus sure a la moins sure :

        1. le MARQUEUR colle sur le rabat — il donne le nom sans ambiguite, et
           la hauteur du rebord avec ;
        2. la CONTINUITE — un carton deja nomme garde son nom tant qu'il reste
           pres de la ou on l'a vu ;
        3. la TAILLE RELATIVE, quand les deux ouvertures sont vues et different
           d'au moins `ECART_TAILLE_DECISIF` — 126 +-3 cm2 contre 77 +-1 sur
           boites VIDES, six fois le bruit ;
        4. l'AIRE ABSOLUE, pour un carton vu SEUL, comparee aux deux aires
           mesurees (`AIRE_CARTON_ATTENDUE`). Sans elle, un carton seul se
           voyait attribuer "grand" par defaut : le PETIT vu seul devenait le
           grand ;
        5. la ROBE puis le plus grand des restants, en dernier recours.

        Pourquoi la continuite passe AVANT la taille : une boite PLEINE ne se
        mesure plus. Mesure du 25/08, le grand carton avec la balle et un scotch
        dedans tombe a 59 cm2 contre 126 vide, sous le petit reste a 71 —
        l'aire s'inverse. Elle ne vaut donc que pour NOMMER la premiere fois,
        boites vides ; ensuite c'est la position qui tient l'identite. Un
        marqueur colle affranchit de tout cela.

        Les objets a trier sont ecartes : le robot imprime est une tache sombre
        de 82x134 mm cerclee de bois brun, donc un candidat parfait. Passe en
        premier par l'aire, il volait la place du vrai grand carton (mesure du
        24/08). Un objet deja identifie n'est pas un carton, point.
        """
        centres = [np.asarray(xy, float) for _, xy, _ in objets]
        vus = [v for v in self._creux_candidats(image, angles)
               if all(np.linalg.norm(v[1] - centre) > RAYON_OBJET_CARTON
                      for centre in centres)
               and not sous_le_bras(v[1], angles)]
        rendus, restants = [], list(vus)
        for classe, (xy_marqueur, z) in self.cartons_marques(image, marqueurs).items():
            if not restants:
                break
            creux = min(restants, key=lambda v: float(np.linalg.norm(v[1] - xy_marqueur)))
            if float(np.linalg.norm(creux[1] - xy_marqueur)) > PORTE_MARQUEUR_CARTON:
                continue
            restants.remove(creux)
            # L'ouverture avait ete projetee a la hauteur SUPPOSEE du rebord ;
            # quand le marqueur est sur le rebord il donne la vraie, et on refait
            # la projection avec. Pose a plat sur la table, il ne dit que le nom.
            hauteur = HAUTEUR_CARTON if z is None else z
            rendus.append((classe, self.vers_base(creux[4], hauteur)[:2],
                           creux[2], hauteur))
        manquantes = [c for c in ('grand', 'petit')
                      if c not in {r[0] for r in rendus}]
        # LA CONTINUITE D'ABORD, des lors qu'il y a un nom a conserver. Ce n'est
        # pas un choix de confort : une boite PLEINE ne se mesure plus. Mesure du
        # 25/08, le grand carton avec la balle et un scotch dedans tombe a
        # 59 cm2 contre 126 vide, sous le petit reste a 71 — l'aire s'inverse.
        # Elle n'est fiable que sur une boite vide, donc au moment ou on nomme
        # pour la premiere fois. Apres, c'est la position qui tient l'identite.
        for classe in list(manquantes):
            connu = (connus or {}).get(classe)
            if connu is None or not restants:
                continue
            creux = min(restants,
                        key=lambda v: float(np.linalg.norm(v[1] - np.asarray(connu, float))))
            if float(np.linalg.norm(creux[1] - np.asarray(connu, float))) > CONTINUITE_CARTON:
                continue
            restants.remove(creux)
            manquantes.remove(classe)
            rendus.append((classe,) + creux[1:3] + (HAUTEUR_CARTON,))
        if len(manquantes) == 2 and len(restants) >= 2:
            # LA TAILLE D'ABORD, quand elle tranche franchement. La continuite
            # etait passee avant, et c'etait une faute : une premiere image ratee
            # — un seul carton visible, l'autre masque par le bras — verrouillait
            # une etiquette fausse que plus rien ne corrigeait, et le scotch
            # partait dans le grand carton (constate le 25/08).
            restants.sort(key=lambda v: -v[0])
            gros, maigre = restants[0], restants[1]
            if (gros[0] - maigre[0]) / max(gros[0], 1.0) >= ECART_TAILLE_DECISIF:
                restants.remove(gros)
                restants.remove(maigre)
                manquantes.clear()
                rendus += [('grand',) + gros[1:3] + (HAUTEUR_CARTON,),
                           ('petit',) + maigre[1:3] + (HAUTEUR_CARTON,)]
        if len(restants) == 1 and manquantes:
            # Un carton vu SEUL — le bras masque regulierement l'autre. Son aire
            # absolue le nomme. Restreint a ce cas : des que deux ouvertures sont
            # visibles, c'est leur ecart RELATIF qui tranche (plus haut), et si
            # cet ecart est trop faible c'est a la continuite de decider, pas a
            # une frontiere absolue qui les mettrait toutes deux du meme cote.
            classe = nom_par_aire(restants[0][0])
            if classe in manquantes:
                creux = restants.pop()
                manquantes.remove(classe)
                rendus.append((classe,) + creux[1:3] + (HAUTEUR_CARTON,))
        if manquantes and restants:
            sombres = [v for v in restants if v[3] >= NOIR_EXTERIEUR_MIN]
            bruns = [v for v in restants if v[3] < NOIR_EXTERIEUR_MIN]
            devine = []
            if bruns and 'grand' in manquantes:
                devine.append(('grand',) + max(bruns, key=lambda v: v[0])[1:3]
                              + (HAUTEUR_CARTON,))
            if sombres and 'petit' in manquantes:
                devine.append(('petit',) + max(sombres, key=lambda v: v[0])[1:3]
                              + (HAUTEUR_CARTON,))
            if len(devine) < len(manquantes) and len(restants) >= len(manquantes):
                # Les deux robes se ressemblent (ombre, contre-jour) : on retombe
                # sur le gabarit, moins sur, plutot que de perdre un carton.
                restants.sort(key=lambda v: -v[0])
                devine = [(c,) + v[1:3] + (HAUTEUR_CARTON,)
                          for c, v in zip(sorted(manquantes), restants)]
                devine.sort(key=lambda r: r[0] != 'grand')
            rendus += devine
        return rendus

    def _carton_par_creux(self, image, dernier=None, angles=None):
        candidats = [(aire, xy, c) for aire, xy, c, _, _
                     in self._creux_candidats(image, angles)]
        if not candidats:
            return None
        if dernier is not None:
            dernier = np.asarray(dernier, float)
            candidats.sort(key=lambda t: -t[0] * (0.25 + float(
                np.exp(-np.linalg.norm(t[1] - dernier) / 120.0))))
        else:
            candidats.sort(key=lambda t: -t[0])
        return candidats[0][1], candidats[0][2]

    def _coeur_sombre(self, contour, valeur):
        """Le creux debarrasse de l'ombre de la paroi exterieure, ou None.

        Une paroi de carton a l'ombre est sombre elle aussi : elle se colle a
        l'ouverture et le contour les avale toutes les deux. Le petit carton,
        115 x 70 mm au metre ruban, etait ainsi mesure 105 x 203 mm — et le
        point de largage se choisit sur ce polygone, donc au-dessus de la paroi
        plutot que dans la boite.

        L'interieur de la boite est franchement plus sombre que la paroi
        eclairee de biais : un seuil d'Otsu a l'interieur du seul creux les
        separe. Rendu du coeur seulement s'il reste une ouverture credible,
        sinon None — mieux vaut un contour trop large qu'un contour coupe en
        deux.
        """
        plein = np.zeros(valeur.shape, np.uint8)
        cv2.drawContours(plein, [contour], -1, 255, -1)
        seuil, _ = cv2.threshold(valeur[plein > 0], 0, 255,
                                 cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        # Le seuil strict colle a la mesure au metre ruban (67 x 115 mm contre
        # 115 x 70 reels) ; le seuil large y ajoute 16 mm. Mais sur une ouverture
        # a deux niveaux francs, Otsu tombe pile sur le niveau sombre et le seuil
        # strict ne rend rien : on repasse alors au large plutot que de perdre
        # le coeur.
        garde = None
        for comparaison in (np.less, np.less_equal):
            coeur = (comparaison(valeur, seuil) & (plein > 0)).astype(np.uint8) * 255
            coeur = cv2.morphologyEx(coeur, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
            contours, _ = cv2.findContours(coeur, cv2.RETR_EXTERNAL,
                                           cv2.CHAIN_APPROX_SIMPLE)
            if contours:
                garde = max(contours, key=cv2.contourArea)
                break
        if garde is None:
            return None
        aire = cv2.contourArea(garde)
        if aire < PART_COEUR_MIN * max(cv2.contourArea(contour), 1.0):
            return None
        enveloppe = cv2.convexHull(garde)
        petit, grand = self._cotes_mm(enveloppe)
        rect = cv2.minAreaRect(enveloppe)
        aire_rect = rect[1][0] * rect[1][1]
        if not (COTE_CARTON_MM[0] <= petit and grand <= COTE_CARTON_MM[1]):
            return None
        if not aire_rect or cv2.contourArea(enveloppe) / aire_rect < REMPLISSAGE_CARTON_MIN:
            return None
        return garde

    def _creux_candidats(self, image, angles=None):
        """[(aire mm2, centre base, contour, part noire, centre pixel)] — tous
        creux qui ressemblent a une ouverture de carton.

        L'anneau autour du creux doit etre BRUN ou SOMBRE : brun pour le grand
        carton, sombre pour le petit, qui est noir a l'exterieur. Exiger le brun
        seul — ce que faisait la version mono-carton — rendait le petit carton
        invisible.
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        teinte, saturation, valeur = hsv[:, :, 0], hsv[:, :, 1], hsv[:, :, 2]
        plateau, bord_bras = self.zone_utile(valeur.shape, angles)
        brun = cv2.inRange(hsv, *HSV_CARTON)

        fond = cv2.medianBlur(valeur, 61)
        creux = ((valeur.astype(int) < fond.astype(int) - ECART_SOMBRE)
                 & (plateau > 0)).astype(np.uint8) * 255
        creux = cv2.morphologyEx(creux, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        creux = cv2.morphologyEx(creux, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))

        contours, _ = cv2.findContours(creux, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        candidats = []
        for c in contours:
            aire = cv2.contourArea(c)
            if aire < AIRE_CREUX_MIN:
                continue
            # Tout se juge sur l'ENVELOPPE CONVEXE : le creux epouse l'interieur
            # eclaire d'un seul cote, sa forme brute est un croissant qui ne
            # remplit jamais sa boite. C'est son enveloppe qui recouvre
            # l'ouverture, et c'est elle qui doit etre rectangulaire.
            enveloppe = cv2.convexHull(c)
            petit, grand = self._cotes_mm(enveloppe)
            if not (COTE_CARTON_MM[0] <= petit and grand <= COTE_CARTON_MM[1]):
                continue
            rect = cv2.minAreaRect(enveloppe)
            aire_rect = rect[1][0] * rect[1][1]
            if (not aire_rect
                    or cv2.contourArea(enveloppe) / aire_rect < REMPLISSAGE_CARTON_MIN):
                continue
            if self._ne_du_decoupage(c, bord_bras):
                continue
            plein = np.zeros(valeur.shape, np.uint8)
            cv2.drawContours(plein, [c], -1, 255, -1)
            anneau = cv2.subtract(cv2.dilate(plein, np.ones((19, 19), np.uint8)),
                                  cv2.dilate(plein, np.ones((5, 5), np.uint8)))
            pourtour = int(np.count_nonzero(anneau))
            if pourtour == 0:
                continue
            # Anneau brun : ce qui elimine les marqueurs (bordure blanche) et
            # l'ombre du bras (silhouette blanche a cote du creux).
            part_brune = np.count_nonzero(cv2.bitwise_and(brun, anneau)) / pourtour
            part_sombre = (np.count_nonzero(valeur[anneau > 0] < VALEUR_SOMBRE)
                           / pourtour)
            contraste = float(np.median(valeur[anneau > 0]) - np.median(valeur[plein > 0]))
            if part_brune < ANNEAU_BRUN_MIN and part_sombre < ANNEAU_SOMBRE_MIN:
                continue
            if part_brune >= ANNEAU_BRUN_MIN and contraste < CONTRASTE_MIN:
                continue
            coeur = self._coeur_sombre(c, valeur)
            if coeur is not None:
                c, enveloppe = coeur, cv2.convexHull(coeur)
                petit, grand = self._cotes_mm(enveloppe)
            moments = cv2.moments(enveloppe)
            uv = (moments['m10'] / moments['m00'], moments['m01'] / moments['m00'])
            centre_base = self.vers_base(uv, HAUTEUR_CARTON)[:2]
            if not (RAYON_BASE_MIN <= float(np.hypot(*centre_base)) <= RAYON_CARTON_MAX):
                continue
            robe = cv2.subtract(cv2.dilate(plein, np.ones((25, 25), np.uint8)),
                                cv2.dilate(plein, np.ones((5, 5), np.uint8)))
            part_noire = (np.count_nonzero(valeur[robe > 0] < VALEUR_SOMBRE)
                          / max(1, int(np.count_nonzero(robe))))
            candidats.append((petit * grand, centre_base, c, part_noire,
                              np.asarray(uv, float)))
        return candidats

    def _carton_par_couleur(self, image, angles=None):
        """Repli : plus grande tache brune du plateau, la planche exclue.

        Une tache qui couvre une large part de la planche EST la planche — la
        retenir donne un centroide a 300 mm du vrai carton, et un largage dans
        le vide. Mieux vaut ne rien rendre.
        """
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        masque = cv2.bitwise_and(cv2.inRange(hsv, *HSV_CARTON),
                                 self.zone_utile(hsv.shape[:2], angles)[0])
        masque = cv2.morphologyEx(masque, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))
        masque = cv2.morphologyEx(masque, cv2.MORPH_CLOSE, np.ones((13, 13), np.uint8))
        aire_plateau = float(cv2.contourArea(self.quad_plateau())) or 1.0
        contours, _ = cv2.findContours(masque, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for c in sorted(contours, key=cv2.contourArea, reverse=True):
            aire = cv2.contourArea(c)
            if aire < 1200:
                break
            if aire / aire_plateau > PART_PLATEAU_MAX:
                continue
            petit, grand = self._cotes_mm(c)
            if not (COTE_CARTON_MM[0] <= petit and grand <= COTE_CARTON_MM[1]):
                continue
            moments = cv2.moments(c)
            uv = (moments['m10'] / moments['m00'], moments['m01'] / moments['m00'])
            return self.vers_base(uv, HAUTEUR_CARTON)[:2], c
        return None


class SuiviCarton:
    """Suit le carton d'une image a l'autre : lisse ce qui tremble, bascule
    quand il bouge vraiment.

    Detecte image par image, le centre de l'ouverture tremble de quelques mm —
    l'ombre bouge, pas le carton — et saute de 50 a 170 mm quand le bras passe
    au-dessus (mesure du 24/08). Un simple lissage suivrait ces sauts avec du
    retard ; un simple seuil de rejet ne verrait jamais un carton reellement
    deplace. On garde donc les deux : lissage tant que la detection reste
    proche, bascule franche quand une position NOUVELLE se confirme sur
    plusieurs images de suite. La taille reelle de l'ouverture sert de garde —
    un carton ne change pas de dimensions, une ombre si.
    """

    def __init__(self):
        self.centre = None
        self.taille = None
        self.polygone = None
        self.rebord = None             # hauteur mesuree du rebord, mm
        self.vu_le = 0.0
        self._candidat = None
        self._confirmations = 0
        self.deplacements = 0          # incremente a chaque bascule reelle

    def _adopte(self, centre, taille, polygone, maintenant, rebord=None):
        if self.centre is not None:
            self.deplacements += 1
        self.centre, self.taille, self.polygone = centre, taille, polygone
        self.rebord = rebord if rebord is not None else self.rebord
        self.vu_le = maintenant
        self._candidat, self._confirmations = None, 0

    def maj(self, centre, taille, polygone, maintenant=None, rebord=None):
        maintenant = time.time() if maintenant is None else maintenant
        if centre is None:
            return
        perime = maintenant - self.vu_le > PEREMPTION_CARTON
        if self.centre is None or perime:
            self._adopte(centre, taille, polygone, maintenant, rebord)
            return
        change_de_taille = (self.taille is not None and taille is not None
                            and abs(taille - self.taille) / self.taille > ECART_TAILLE_MAX)
        if float(np.linalg.norm(centre - self.centre)) <= SAUT_CARTON and not change_de_taille:
            a = LISSAGE_CARTON
            self.centre = (1 - a) * self.centre + a * centre
            self.taille = (1 - a) * self.taille + a * taille if taille else self.taille
            self.polygone, self.vu_le = polygone, maintenant
            self.rebord = rebord if rebord is not None else self.rebord
            self._candidat, self._confirmations = None, 0
            return
        # Position franchement differente : elle doit se confirmer avant d'etre crue.
        if (self._candidat is not None
                and float(np.linalg.norm(centre - self._candidat)) < SAUT_CARTON):
            self._confirmations += 1
            self._candidat = centre
        else:
            self._candidat, self._confirmations = centre, 1
        if self._confirmations >= CONFIRMATIONS_CARTON:
            self._adopte(centre, taille, polygone, maintenant, rebord)

    def position(self, maintenant=None):
        """(centre, polygone, rebord) si la position est fraiche, sinon None."""
        maintenant = time.time() if maintenant is None else maintenant
        if self.centre is None or maintenant - self.vu_le > PEREMPTION_CARTON:
            return None
        return self.centre.copy(), self.polygone, self.rebord


def rayon(vision, uv):
    """(origine mm, direction unitaire) du rayon optique dans le repere base."""
    p = cv2.undistortPoints(np.array([[uv]], float), vision.K, vision.dist).reshape(2)
    R, t = vision.T[:3, :3], vision.T[:3, 3]
    direction = R.T @ np.array([p[0], p[1], 1.0])
    return -R.T @ t * 1000.0, direction / np.linalg.norm(direction)


def triangule(vision_a, uv_a, vision_b, uv_b):
    """(point milieu en mm, ecartement des rayons) ou (None, None).

    Deux vues suppriment l'hypothese de hauteur : la vue de dessus seule doit
    SUPPOSER le centre de la balle a 35 mm, et toute erreur sur cette hauteur
    devient une erreur XY d'autant plus grande que le rayon est oblique — donc
    aux bords du plateau. L'ecartement des deux rayons est la mesure de
    confiance : deux detections qui ne portent pas sur le meme objet ne se
    croisent pas.
    """
    o_a, d_a = rayon(vision_a, uv_a)
    o_b, d_b = rayon(vision_b, uv_b)
    ecart = o_a - o_b
    aa, ab, bb = d_a @ d_a, d_a @ d_b, d_b @ d_b
    da, db = d_a @ ecart, d_b @ ecart
    denominateur = aa * bb - ab * ab
    if abs(denominateur) < 1e-9:                  # rayons paralleles
        return None, None
    sa = (ab * db - bb * da) / denominateur
    sb = (aa * db - ab * da) / denominateur
    pa, pb = o_a + sa * d_a, o_b + sb * d_b
    return (pa + pb) / 2.0, float(np.linalg.norm(pa - pb))


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
                       Qt.AlignCenter | Qt.TextWordWrap, nom.replace('_', ' '))
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
            precedent = self.machine.etat
            self.avance.emit(self.machine.pas())
            if self._stop or not self.continu:
                break
            # ECHEC_PORTANT arrete la boucle MEME en automatique : le bras
            # tient l'objet et rien ne changera sans intervention. Boucler
            # dessus ferait tourner le bras avec la balle en main.
            if self.machine.etat == 'ECHEC_PORTANT':
                break
            # La machine ne progresse plus : rien ne changera sans intervention,
            # et boucler dessus ne ferait que remuer le bras pour rien.
            if self.machine.etat == precedent:
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
        # La SVPRO regarde de cote : elle voit la balle quand le bras la cache a
        # la vue de dessus, et les deux ensemble donnent sa hauteur.
        self.vision_svpro = (Vision('svpro_extrinsic_servo')
                             if (CALIB / 'svpro_extrinsic_servo.yaml').exists() else None)
        self.images = {}
        self.captures = {}
        self.ctx = fsm.Contexte()
        self.ctx.detecteur = self._detecte_objet
        self.ctx.detecteur_carton = self._detecte_carton_live
        self.machine = fsm.MachineEtats(self.ctx)
        self.ouvrier = None
        self._n_journal = 0
        self._detections = deque(maxlen=40)
        # Un suivi PAR carton : l'operateur peut deplacer l'un ou l'autre, et
        # chacun doit etre rattrape sans que la position du voisin s'en mele.
        self.suivi_cartons = {classe: SuiviCarton() for classe in ('grand', 'petit')}
        self.marqueurs = Marqueurs()
        self._cartons_marques = []
        self._ouvertures = []
        self._designation = self._designation_memorisee()
        self._designation_faite = bool(self._designation)
        self._deplacements_carton = {}
        self._detections_svpro = deque(maxlen=40)
        self._carton_svpro = {}
        self._objets_vus = []
        self._objets_svpro = []
        # Ecart SVPRO -> arducam appris PAR CATEGORIE : il depend de la
        # hauteur de l'objet, que la vue oblique doit supposer. Un rouleau
        # couche et une balle de 66 mm ne se decalent pas pareil.
        self._decalage_objet = {}
        # Ecart systematique SVPRO -> arducam sur le carton, appris en marche.
        # Les deux la voient a 29 mm l'une de l'autre (mesure du 24/08) : la vue
        # oblique suppose la hauteur du rebord et se trompe dessus. Sans ce
        # recalage, chaque relais SVPRO passait pour un DEPLACEMENT du carton et
        # jetait la pose deja resolue — 1,8 s de solveur a chaque fois.
        self._decalage_svpro = {}
        self._source_balle = '—'
        self._camera_balle = ''
        self._decalage_balle = None
        self._debut_etape = time.time()
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
            vue = VueCliquable(nom)
            vue.setMinimumSize(400, 300)
            vue.setAlignment(Qt.AlignCenter)
            vue.setStyleSheet('background:#111; color:#888;')
            vue.setToolTip('clic sur un carton : il devient le GRAND, '
                           "l'autre devient le petit")
            vue.clique.connect(self._designe_grand)
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
        self.live_source = QLabel('—')
        self.live_source.setStyleSheet('font-family:monospace; font-size:11px; color:#555;')
        form_live.addRow('source', self.live_source)
        form_live.addRow('exposition', self.live_expo)
        droite.addWidget(boite_live)

        boite_mesures = QGroupBox('mesures')
        self.mesures = QFormLayout(boite_mesures)
        self.champs = {}
        droite.addWidget(boite_mesures)

        boite_cible = QGroupBox('cibles')
        form_cible = QFormLayout(boite_cible)
        self.champ_carton = QLabel('non défini')
        memoire = fsm.carton_memorise(self.ctx)
        if memoire is not None:
            self.ctx.carton_xy = memoire
            self.champ_carton.setText(f'({memoire[0]:.1f}, {memoire[1]:.1f}) mm — mémoire')
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
        # Position du bras pour masquer sa silhouette : la DERNIERE lue par la
        # machine a etats, jamais une lecture a nous — le pont est mono-client
        # et bloquant, l'interroger d'ici couperait le dialogue en cours.
        angles = None if self.ctx.pont is None else self.ctx.pont.derniers_angles
        for nom, cap in self.captures.items():
            if cap is None:
                continue
            ok, image = cap.read()
            if not ok:
                continue
            self.images[nom] = image
            affichee = image.copy()
            if nom == 'svpro' and self.vision_svpro is not None:
                vue = self.vision_svpro.balle(image)
                with self._verrou:
                    self._detections_svpro.append(
                        (time.time(), None if vue is None else np.asarray(vue[1][:2], float)))
                if vue is not None:
                    u, v, r = vue[1]
                    cv2.circle(affichee, (int(u), int(v)), int(r) + 3, (0, 0, 255), 2)
                # La SVPRO voit le carton de biais : quand le bras le masque en
                # vue de dessus, elle le voit encore. Elle ASSISTE l'arducam,
                # elle ne la remplace pas — la projection obliques est plus
                # sensible a l'erreur de hauteur de rebord.
                points = self.vision_svpro.points_interessants(
                    image, angles=angles, marqueurs=self.marqueurs.coins(image))
                # La SVPRO ne nomme rien : chaque tache qu'elle voit est
                # rapprochee de ce que l'ARDUCAM a deja identifie, et n'herite
                # d'un nom que par cette proximite. Les deux vues affichent
                # alors la meme chose, ce qui n'etait pas le cas quand chacune
                # classait de son cote.
                nommes = self._nomme_par_arducam(points)
                self._objets_svpro = [(classe, xy) for classe, xy, _ in nommes
                                      if classe in DESTINATION]
                self._carton_svpro = {
                    classe: (xy, self.vision_svpro.polygone_base(contour),
                             float(np.prod(self.vision_svpro._cotes_mm(contour))))
                    for classe, xy, contour in nommes if classe in COULEUR_CARTON}
                for classe, xy, contour in nommes:
                    couleur = COULEUR_OBJET.get(classe) or COULEUR_CARTON[classe]
                    cv2.polylines(affichee, [cv2.convexHull(contour)], True, couleur, 2)
                    rect = cv2.boundingRect(contour)
                    cv2.putText(affichee, classe, (rect[0], rect[1] - 5),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, couleur, 1)
            if nom == 'arducam':
                trouve = self.vision.balle(image)
                with self._verrou:
                    self._detections.append(
                        (time.time(), None if trouve is None else np.asarray(trouve[0], float),
                         None if trouve is None else np.asarray(trouve[1][:2], float)))
                self._maj_live(trouve)
                if trouve is not None:
                    xy, (u, v, r) = trouve
                    cv2.circle(affichee, (int(u), int(v)), int(r) + 3, (0, 0, 255), 2)
                    cv2.putText(affichee, f'{xy[0]:.0f},{xy[1]:.0f}',
                                (int(u) - 34, int(v) - int(r) - 8),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (0, 0, 255), 1)
                marqueurs = self.marqueurs.coins(image)
                objets = self.vision.objets(image, angles=angles,
                                            marqueurs=marqueurs)
                with self._verrou:
                    connus = dict(self._designation)
                    connus.update({c: s.centre for c, s in self.suivi_cartons.items()
                                   if s.centre is not None})
                vus = {classe: (xy, contour, z) for classe, xy, contour, z
                       in self.vision.cartons(image, angles=angles, objets=objets,
                                              marqueurs=marqueurs, connus=connus)}
                # L'UNION de ce qu'on voit a l'instant et de ce que le suivi
                # tient encore. Se limiter a l'image courante laissait la balle
                # deja deposee redevenir une cible des que le bras passait
                # au-dessus de sa boite : plus de carton vu, donc plus
                # d'ouverture, donc plus rien pour la declarer deposee, et le
                # cycle repartait la chercher au fond du carton (25/08).
                self._ouvertures = [self.vision.polygone_base(contour, z)
                                    for _, contour, z in vus.values()]
                with self._verrou:
                    self._ouvertures += [suivi.polygone
                                         for suivi in self.suivi_cartons.values()
                                         if suivi.polygone is not None]
                objets = [o for o in objets if not self._depose(o[1])]
                if trouve is not None and self._depose(trouve[0]):
                    trouve = None
                with self._verrou:
                    self._objets_vus = [(classe, xy) for classe, xy, _ in objets]
                    if trouve is not None:
                        self._objets_vus.append(('balle', np.asarray(trouve[0], float)))
                    self._apprend_biais_objets()
                with self._verrou:
                    self._cartons_marques = sorted(
                        c for c, i in ((v, k) for k, v in MARQUEUR_CARTON.items())
                        if i in marqueurs)
                for classe, suivi in self.suivi_cartons.items():
                    releve = None
                    if classe in vus:
                        xy, contour, z = vus[classe]
                        petit, grand = self.vision._cotes_mm(contour, z)
                        releve = (np.asarray(xy, float),
                                  self.vision.polygone_base(contour, z),
                                  petit * grand, z)
                        appui = self._carton_svpro.get(classe)
                        if appui is not None:
                            # Un decalage PAR CARTON, pas un seul pour les deux.
                            # Mesure du 25/08 : la SVPRO tombe a 14 mm de
                            # l'arducam sur le grand carton et a 60 mm sur le
                            # petit, qu'elle voit par la tranche. Une moyenne
                            # des deux est fausse pour les deux.
                            ecart = releve[0] - appui[0]
                            ancien = self._decalage_svpro.get(classe)
                            self._decalage_svpro[classe] = (
                                ecart if ancien is None else 0.9 * ancien + 0.1 * ecart)
                    elif (self._carton_svpro.get(classe) is not None
                          and self._decalage_svpro.get(classe) is not None):
                        # L'arducam est aveugle sur ce carton : la SVPRO prend le
                        # relais, remise dans le repere de l'arducam. Sans
                        # decalage appris on ne prend rien — une position
                        # decalee de 29 mm vaut moins que la derniere bonne, que
                        # le suivi tient 4 s.
                        appui = self._carton_svpro[classe]
                        decalage = self._decalage_svpro[classe]
                        releve = (appui[0] + decalage, appui[1] + decalage, appui[2])
                    with self._verrou:
                        if releve is not None:
                            suivi.maj(releve[0], releve[2], releve[1],
                                      rebord=releve[3] if len(releve) > 3 else None)
                self._dessine_objets(affichee, objets)
                self._dessine_cartons(affichee)
                self._maj_carton(self._suivi_vise())
            self._affiche(nom, affichee)
        self._vide_journal()
        if self.ouvrier is not None and self.ouvrier.isRunning():
            self._marche(f'⏳ {self.machine.etat} — {time.time() - self._debut_etape:.0f} s')

    def _suivi_vise(self):
        """Le suivi du carton ou l'objet en cours doit aller."""
        return self.suivi_cartons[self.ctx.carton_vise].position()

    def _dessine_objets(self, image, objets):
        for classe, xy, contour in objets:
            cv2.polylines(image, [cv2.convexHull(contour)], True, COULEUR_OBJET[classe], 2)
            rect = cv2.boundingRect(contour)
            cv2.putText(image, f'{classe} -> {DESTINATION[classe]}',
                        (rect[0], rect[1] - 5), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                        COULEUR_OBJET[classe], 1)

    def _dessine_cartons(self, image):
        for classe, suivi in self.suivi_cartons.items():
            with self._verrou:
                vu = suivi.position()
            if vu is None:
                continue
            vise = (self.ctx.carton_xy if (self.ctx.carton_xy is not None
                                           and self.ctx.carton_vise == classe) else None)
            self._dessine_carton(image, vu, COULEUR_CARTON[classe], classe, vise)

    def _dessine_carton(self, image, suivi, couleur, etiquette, vise=None):
        """Rectangle propre sur la boite, et UNE seule croix : le point vise.

        Le contour brut du creux est dentele et plus petit que l'ouverture — on
        dessine donc son rectangle englobant, elargi de la marge que le creux
        sous-estime. La croix marque le point de largage retenu par la machine
        quand il existe, sinon le centre suivi : en afficher deux, l'un pour la
        detection et l'autre pour la cible, faisait croire a deux cartons.
        """
        if suivi is not None and suivi[1] is not None and len(suivi[1]) >= 3:
            rect = cv2.minAreaRect(np.asarray(suivi[1], np.float32).reshape(-1, 1, 2))
            rect = (rect[0], (rect[1][0] + 2 * MARGE_TRACE_CARTON,
                              rect[1][1] + 2 * MARGE_TRACE_CARTON), rect[2])
            coins = np.array([self.vision.vers_pixel([c[0], c[1], HAUTEUR_CARTON])
                              for c in cv2.boxPoints(rect)], np.int32)
            cv2.polylines(image, [coins], True, couleur, 2)
        if vise is not None and suivi[1] is not None and len(suivi[1]) >= 3:
            contour = np.asarray(suivi[1], np.float32).reshape(-1, 1, 2)
            dedans = cv2.pointPolygonTest(contour, (float(vise[0]), float(vise[1])), True)
            if dedans < -MARGE_TRACE_CARTON:
                vise = None       # cible perimee : ne pas la montrer comme valide
        point = vise if vise is not None else suivi[0]
        uv = self.vision.vers_pixel([point[0], point[1], HAUTEUR_CARTON])
        if vise is not None:
            cv2.drawMarker(image, tuple(uv.astype(int)), (255, 0, 255),
                           cv2.MARKER_TILTED_CROSS, 18, 2)
        cv2.putText(image, f'{etiquette} {point[0]:.0f},{point[1]:.0f}',
                    (int(uv[0]) - 44, int(uv[1]) - 16),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, couleur, 1)

    def _depose(self, xy):
        """L'objet est-il DEJA dans un carton ? Alors il n'est plus une cible.

        Sans ca la balle deposee etait redetectee au fond de la boite et le
        cycle repartait la chercher — mesure du 25/08 : 35,4 mm a l'interieur de
        l'ouverture du grand carton, et toujours annoncee comme cible.
        """
        point = (float(xy[0]), float(xy[1]))
        return any(cv2.pointPolygonTest(
            np.asarray(ouverture, np.float32).reshape(-1, 1, 2), point, True) > -MARGE_DEPOSE
            for ouverture in self._ouvertures)

    def _designe_grand(self, camera, u, v):
        """Un clic sur un carton le declare GRAND, l'autre devient le petit.

        Aucune mesure d'image ne separe deux cartons de meme gabarit : 16 %
        d'ecart entre les deux ouvertures, 18 % de bruit sur la meme d'une image
        a l'autre (25/08). L'operateur, lui, sait lequel est lequel. Une fois
        designe, c'est la CONTINUITE qui tient le nom — on peut deplacer les
        cartons a la main sans redesigner.
        """
        vision = self.vision if camera == 'arducam' else self.vision_svpro
        if vision is None:
            return
        vise = vision.vers_base((u, v), HAUTEUR_CARTON)[:2]
        with self._verrou:
            distances = sorted(
                (float(np.linalg.norm(s.centre - vise)), classe)
                for classe, s in self.suivi_cartons.items() if s.centre is not None)
        if not distances or distances[0][0] > CONTINUITE_CARTON:
            self.statusBar().showMessage(
                f'aucun carton suivi près de ({vise[0]:.0f}, {vise[1]:.0f}) mm')
            return
        if distances[0][1] != 'grand':
            with self._verrou:
                self.suivi_cartons['grand'], self.suivi_cartons['petit'] = (
                    self.suivi_cartons['petit'], self.suivi_cartons['grand'])
            self.ctx.carton_resolu = None
            self.ctx.carton_xy = None
            self.ctx.R_carton = None
        self._enregistre_designation(clic=True)
        centre = self.suivi_cartons['grand'].centre
        self.statusBar().showMessage(
            f'GRAND carton = celui en ({centre[0]:.0f}, {centre[1]:.0f}) mm — '
            f"l'autre est le petit")
        self.ctx.note(f'carton grand designe a la main en '
                      f'({centre[0]:.0f}, {centre[1]:.0f}) mm')

    def _enregistre_designation(self, clic=False):
        """Garde la designation d'une seance a l'autre.

        Sans elle, chaque relance du tableau de bord repart sur un tirage a pile
        ou face tant que les deux cartons n'ont pas ete separes par un clic.

        Seul un CLIC cree la designation ; ensuite elle suit les cartons qui
        bougent. Laisser le detecteur l'ecrire tout seul l'a remplie de
        n'importe quoi des le premier essai : bras non connecte, donc pas
        d'angles, donc pas de masque, et l'ombre du bras enregistree comme
        "petit carton" a (54, -18) — au pied du robot.
        """
        if not (clic or self._designation_faite):
            return
        self._designation_faite = True
        with self._verrou:
            positions = {classe: s.centre.tolist()
                         for classe, s in self.suivi_cartons.items()
                         if s.centre is not None and plausible(s.centre)}
        if len(positions) != 2:
            return
        # Ecrire a chaque image userait le disque pour rien ; ne pas reecrire du
        # tout laisserait une designation perimee des que les cartons bougent.
        bouge = any(classe not in self._designation
                    or float(np.linalg.norm(np.asarray(xy) - self._designation[classe])) > 30.0
                    for classe, xy in positions.items())
        if not bouge:
            return
        self._designation = {c: np.asarray(xy, float) for c, xy in positions.items()}
        DESIGNATION_CARTONS.write_text(json.dumps(positions))

    def _designation_memorisee(self):
        if not DESIGNATION_CARTONS.exists():
            return {}
        try:
            memoire = json.loads(DESIGNATION_CARTONS.read_text())
        except json.JSONDecodeError:
            return {}
        return {classe: np.asarray(xy, float) for classe, xy in memoire.items()
                if classe in COULEUR_CARTON and plausible(xy)}

    def _maj_carton(self, suivi):
        """Le carton s'affiche et se met a jour tout seul — aucun clic requis.

        Et quand le suivi declare un VRAI deplacement, la pose resolue par la
        machine est jetee : c'est le seul evenement qui doit la perimer, la
        position detectee tremblant de quelques mm en permanence.
        """
        self._enregistre_designation()
        bouges = {c: s.deplacements for c, s in self.suivi_cartons.items()}
        if bouges != self._deplacements_carton:
            bougeants = [c for c, n in bouges.items()
                         if n != self._deplacements_carton.get(c)]
            self._deplacements_carton = bouges
            self.ctx.carton_resolu = None
            if self.ctx.carton_vise in bougeants:
                # Le POINT DE LARGAGE deja choisi appartient a l'ancienne
                # position : le garder affichait une croix hors du rectangle
                # (constate le 24/08) et aurait fait lacher l'objet a cote.
                self.ctx.carton_xy = None
                self.ctx.R_carton = None
            self.ctx.note(f'carton {" et ".join(bougeants)} deplace — '
                          f'point de largage a recalculer')
        if suivi is None:
            self.champ_carton.setText('non vu')
            return
        centre = suivi[0]
        portee = float(np.hypot(*centre))
        self.champ_carton.setText(f'({centre[0]:.1f}, {centre[1]:.1f}) mm — suivi, '
                                  f'{portee:.0f} mm')

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
        self.live_source.setText(self._source_balle)
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
        vue.taille_image = (w, h)
        vue.setPixmap(QPixmap.fromImage(qimg).scaled(vue.width(), vue.height(),
                                                     Qt.KeepAspectRatio,
                                                     Qt.SmoothTransformation))

    # -- machine ------------------------------------------------------------ #

    def _echantillons(self, file, echantillons, patience, colonne=1):
        """Les `echantillons` dernieres detections fraiches de `file`, ou None.

        Appele depuis le fil du robot. Y relire la camera pendant que le
        rafraichissement la lit aussi fige la capture GStreamer sans jamais
        rendre la main : l'interface reste vivante, l'etape ne se termine plus.
        Un seul fil touche `VideoCapture`. `patience=0` = ce qu'on a sous la
        main, pour un appel depuis le fil graphique.
        """
        limite = time.time() + patience
        while True:
            maintenant = time.time()
            with self._verrou:
                vues = [enr[colonne] for enr in file
                        if enr[colonne] is not None
                        and maintenant - enr[0] < FENETRE_DETECTION]
            if len(vues) >= echantillons:
                return vues[-echantillons:]
            if maintenant >= limite:
                return None
            time.sleep(0.05)

    def _detecte_objet(self, echantillons=3, patience=3.0, exige_dessus=False):
        """Position de l'objet a trier, et choix de sa destination.

        Un cycle deja engage garde SON objet : rechoisir a chaque appel ferait
        changer de cible en cours de descente. Sinon on prend le plus proche du
        robot parmi ceux qui sont dans l'enveloppe — le plus proche est celui
        que le bras atteint le plus vite et le plus surement.
        """
        if self.ctx.classe_objet == 'balle':
            return self._detecte_balle(echantillons, patience, exige_dessus)
        if self.ctx.classe_objet in DESTINATION:
            return self._position_objet(self.ctx.classe_objet, self.ctx.balle_xy)

        candidats = []
        with self._verrou:
            vus = {c for c, _ in self._objets_vus}
            for classe, xy in self._objets_vus:
                candidats.append((float(np.hypot(*xy)), classe, np.asarray(xy, float)))
            for classe in set(DESTINATION) - vus:
                for xy in self._relais_svpro(classe):
                    candidats.append((float(np.hypot(*xy)), classe, xy))
        balle = self._detecte_balle(echantillons, patience, exige_dessus)
        if balle is not None:
            candidats.append((float(np.hypot(*balle)), 'balle', balle))
        # Le filtre "deja depose" se pose ICI, au point de CHOIX, et pas a la
        # source : `_detecte_balle` interroge la camera directement et le relais
        # SVPRO aussi, tous deux court-circuitant la liste filtree. C'est par la
        # que la balle deposee redevenait la cible, cycle apres cycle (25/08).
        deposes = [c for c in candidats if self._depose(c[2])]
        candidats = [c for c in candidats if not self._depose(c[2])]
        if deposes:
            self.ctx.note(f'{len(deposes)} objet(s) deja dans un carton, ignores : '
                          + ', '.join(sorted({c[1] for c in deposes})))
        atteignables = [c for c in candidats if c[0] <= fsm.PORTEE_MAX]
        if not atteignables:
            if candidats:
                self.ctx.note(f'{len(candidats)} objet(s) vus, tous hors enveloppe '
                              f'(le plus proche a {min(candidats)[0]:.0f} mm)')
            return None
        _, classe, xy = min(atteignables)
        self.ctx.classe_objet = classe
        self.ctx.carton_vise = DESTINATION[classe]
        self.ctx.resultats['objet'] = (f'{classe} a ({xy[0]:.0f}, {xy[1]:.0f}) '
                                       f'-> carton {DESTINATION[classe]}')
        self.ctx.note(f'objet retenu : {classe} a {np.hypot(*xy):.0f} mm '
                      f'-> carton {DESTINATION[classe]}')
        return xy

    def _nomme_par_arducam(self, points):
        """[(classe, xy, contour)] — les taches de la SVPRO nommees par l'arducam.

        Une tache sans equivalent cote arducam n'est pas nommee, donc pas
        rendue : la camera d'appui ne peut pas inventer un objet ni un carton.
        """
        with self._verrou:
            connus = [(classe, np.asarray(xy, float))
                      for classe, xy in self._objets_vus]
        for classe, suivi in self.suivi_cartons.items():
            vu = suivi.position()
            if vu is not None:
                connus.append((classe, vu[0]))
        nommes = []
        for xy, contour in points:
            if not connus:
                break
            classe, cible = min(connus, key=lambda t: float(np.linalg.norm(t[1] - xy)))
            if float(np.linalg.norm(cible - xy)) <= APPARIEMENT_MAX:
                nommes.append((classe, xy, contour))
        return nommes

    def _apprend_biais_objets(self):
        """Ecart SVPRO -> arducam, par categorie, appris quand les deux voient.

        Les objets d'une meme categorie sont apparies au plus proche : il y a
        deux scotchs sur la planche, et les confondre ferait apprendre un
        decalage qui n'est celui d'aucun des deux.
        """
        for classe, xy in self._objets_vus:
            memes = [autre for c, autre in self._objets_svpro if c == classe]
            if not memes:
                continue
            plus_proche = min(memes, key=lambda a: float(np.linalg.norm(a - xy)))
            if float(np.linalg.norm(plus_proche - xy)) > 80.0:
                continue                     # ce n'est pas le meme exemplaire
            ecart = xy - plus_proche
            ancien = self._decalage_objet.get(classe)
            self._decalage_objet[classe] = (ecart if ancien is None
                                            else 0.8 * ancien + 0.2 * ecart)

    def _relais_svpro(self, classe):
        """Objets de cette categorie vus par la seule SVPRO, remis dans le
        repere de l'arducam. Sans biais appris on ne rend rien : une position
        decalee vaut moins que pas de position du tout."""
        decalage = self._decalage_objet.get(classe)
        if decalage is None:
            return []
        return [xy + decalage for c, xy in self._objets_svpro if c == classe]

    def _position_objet(self, classe, precedent=None):
        """Position fraiche de l'objet de cette classe, le plus proche du dernier
        point connu quand il y en a plusieurs (deux scotchs sur la planche)."""
        with self._verrou:
            memes = [np.asarray(xy, float) for c, xy in self._objets_vus if c == classe]
            if not memes:
                memes = self._relais_svpro(classe)
                if memes:
                    self.ctx.note(f'{classe} invisible de dessus — repris de la SVPRO')
        if not memes:
            self.ctx.note(f'{classe} vu par aucune camera')
            return None
        if precedent is None:
            return min(memes, key=lambda xy: float(np.hypot(*xy)))
        plus_proche = min(memes, key=lambda xy: float(np.linalg.norm(xy - precedent)))
        ecart = float(np.linalg.norm(plus_proche - precedent))
        if ecart > PORTE_SUIVI_OBJET:
            self.ctx.note(f'{classe} le plus proche a {ecart:.0f} mm de la cible — '
                          f'ce n est pas le meme exemplaire, cible gardee')
            return precedent
        return plus_proche

    def _detecte_balle(self, echantillons=3, patience=3.0, exige_dessus=False):
        try:
            return self._detecte_balle_reel(echantillons, patience, exige_dessus)
        finally:
            self.ctx.source_balle = self._camera_balle

    def _detecte_balle_reel(self, echantillons=3, patience=3.0, exige_dessus=False):
        """Position de la balle en mm dans le repere base, ou None.

        Trois sources, par ordre de qualite :
        1. FUSION — les deux cameras la voient : les rayons se croisent et
           donnent aussi sa hauteur, sans rien supposer ;
        2. arducam seule — le rayon coupe le plan Z = HAUTEUR_CENTRE_BALLE ;
        3. SVPRO seule — meme calcul de cote, ce qui garde la balle quand le
           bras la masque a la vue de dessus (le journal le montre a chaque
           recalage : « balle vue sur 0 image(s) »).
        """
        dessus = self._echantillons(self._detections, echantillons, patience)
        cote = self._echantillons(self._detections_svpro, echantillons, 0.0)

        if dessus is not None:
            moyenne = np.mean(dessus, axis=0)
            if cote is not None and self.vision_svpro is not None:
                vue_cote = np.mean([self.vision_svpro.vers_base(uv, HAUTEUR_CENTRE_BALLE)[:2]
                                    for uv in cote], axis=0)
                ecart = moyenne - vue_cote
                self._decalage_balle = (ecart if self._decalage_balle is None
                                        else 0.8 * self._decalage_balle + 0.2 * ecart)
            dispersion = max(float(np.linalg.norm(v - moyenne)) for v in dessus)
            if dispersion > 3.0:
                self.ctx.note(f'detection instable ({dispersion:.1f} mm) — cible refusee')
                return None
            if cote is not None and self.vision_svpro is not None:
                pixels = self._echantillons(self._detections, echantillons, 0.0, colonne=2)
                if pixels is not None:
                    uv = np.mean(pixels, axis=0)
                    point, ecartement = triangule(self.vision, uv,
                                                  self.vision_svpro, np.mean(cote, axis=0))
                    if point is not None and ecartement <= ECARTEMENT_MAX:
                        # L'arducam reste la source du XY : elle regarde presque
                        # a la verticale, c'est la meilleure vue pour X et Y. La
                        # SVPRO ne sert qu'a remplacer la HAUTEUR supposee par
                        # une hauteur mesuree, sur laquelle on recoupe le rayon.
                        recale = self.vision.vers_base(uv, point[2])[:2]
                        self._camera_balle = 'arducam+svpro'
                        self._source_balle = (
                            f'arducam + SVPRO — Z mesure {point[2]:.0f} mm au lieu de '
                            f'{HAUTEUR_CENTRE_BALLE:.0f}, correction '
                            f'{np.linalg.norm(recale - moyenne):.1f} mm, rayons a '
                            f'{ecartement:.1f} mm')
                        return recale
                    if point is not None:
                        self.ctx.note(f'appui SVPRO refuse — rayons ecartes de '
                                      f'{ecartement:.0f} mm, les deux cameras ne '
                                      f'regardent pas le meme objet')
            self._camera_balle = 'arducam'
            self._source_balle = 'arducam seule — hauteur supposee'
            return moyenne

        if exige_dessus:
            self._camera_balle = ''
            return None                    # le degagement veut la vue de dessus, pas un repli

        if cote is not None and self.vision_svpro is not None:
            vues = [self.vision_svpro.vers_base(uv, HAUTEUR_CENTRE_BALLE)[:2] for uv in cote]
            moyenne = np.mean(vues, axis=0)
            if max(float(np.linalg.norm(v - moyenne)) for v in vues) > 6.0:
                return None
            if self._decalage_balle is None:
                self.ctx.note('balle invisible de dessus et biais SVPRO pas encore '
                              'appris — cible refusee')
                self._camera_balle = ''
                return None
            # La vue laterale suppose la hauteur du centre de la balle et se
            # trompe dessus : 29 mm de biais mesures le 24/08, assez pour faire
            # sortir de l'enveloppe une balle a 401 mm. On la remet dans le
            # repere de l'arducam avant de s'en servir.
            self._camera_balle = 'svpro'
            self._source_balle = 'SVPRO recalee sur l arducam'
            self.ctx.note(f'balle invisible de dessus — SVPRO recalee de '
                          f'{np.linalg.norm(self._decalage_balle):.0f} mm')
            return moyenne + self._decalage_balle

        self.ctx.note(f'balle vue par aucune camera en {patience:.0f} s — cible refusee')
        return None

    def _detecte_carton_live(self, echantillons=3, patience=2.0, dispersion_max=18.0):
        """Position suivie du carton, ou None si elle n'est plus fraiche.

        Ne moyenne plus les dernieres images : c'est `SuiviCarton` qui lisse, et
        lui sait distinguer un tremblement d'un vrai deplacement. La moyenne
        glissante, elle, refusait la cible des que le bras passait au-dessus
        (« carton instable (22,4 mm) — position refusee »).
        """
        limite = time.time() + patience
        while True:
            suivi = self._suivi_vise()
            if suivi is not None or time.time() >= limite:
                break
            time.sleep(0.05)
        if suivi is None:
            self.ctx.note(f'carton {self.ctx.carton_vise} non suivi depuis plus de '
                          f'{PEREMPTION_CARTON:.0f} s')
            return None
        return suivi

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
        self._debut_etape = time.time()
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
        trouve = self._detecte_carton_live(patience=0.0)
        if trouve is None:
            self.statusBar().showMessage('carton non détecté sur le plateau')
            return
        xy, self.ctx.carton_polygone = trouve
        self.ctx.carton_xy = np.asarray(xy, float)
        self.ctx.R_carton = None              # le roulis se rejuge sur la nouvelle position
        self.ctx.carton_resolu = None
        portee = float(np.hypot(*xy))
        self.champ_carton.setText(f'({xy[0]:.1f}, {xy[1]:.1f}) mm — détecté')
        fsm.memorise_carton(self.ctx.carton_xy, self.ctx.roulis_appris)
        self.statusBar().showMessage(
            f'carton détecté à {portee:.0f} mm'
            + ('' if portee <= fsm.PORTEE_CARTON_MAX
               else f' — HORS ATTEINTE, le rapprocher '
                    f'({fsm.PORTEE_CARTON_MAX:.0f} mm au plus)'))

    def _definit_carton(self):
        """Le carton se designe en y posant la balle : elle sert de mire."""
        vu = self._detecte_balle(patience=0.0)
        if vu is None:
            self.statusBar().showMessage('balle non vue — impossible de définir le carton')
            return
        self.ctx.carton_xy = np.asarray(vu, float)
        self.ctx.R_carton = None
        self.ctx.carton_resolu = None
        fsm.memorise_carton(self.ctx.carton_xy, self.ctx.roulis_appris)
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
