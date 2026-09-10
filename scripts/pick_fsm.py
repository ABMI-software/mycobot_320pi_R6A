#!/usr/bin/env python3
"""Machine a etats du pick-and-place vision-guide — moteur, sans interface.

Le tableau de bord (`pick_dashboard.py`) l'anime, mais elle tourne aussi seule
en tete-a-tete avec le robot. Chaque etat fait UNE chose, publie ce qu'il a
mesure dans `resultats`, et rend l'etat suivant.

Regles de sécurité tenues ici, pas dans l'appelant :

* toute trajectoire est simulee et sa garde au sol verifiee AVANT envoi — un
  mouvement commande sans ce controle a plante le bras dans la table le 20/08 ;
* le seuil de garde ne peut pas etre plus exigeant que l'etat de depart, sinon
  on refuse aussi les remontees quand la pince est deja en position basse ;
* `get_pro_gripper_status == 2` est la seule confirmation de prise valable.

Voir `docs/PICK_AND_PLACE_BOUCLE_FERMEE.md` pour le pourquoi de chaque constante.
"""
from __future__ import annotations

import json
import os
import socket
import sys
import time
import traceback
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE / 'scripts'))
sys.path.insert(0, str(RACINE / 'training' / 'dream'))
from diff_ik import solve_pose, orientation_error_deg                   # noqa: E402
from diff_ik import PRACTICAL_JOINT_LIMITS_DEG as LIMITES               # noqa: E402
from mycobot_fk import forward_kinematics                               # noqa: E402

PORT_PI = 5005
# L'adresse de la Pi est distribuee en DHCP : elle a change de .221 a .224 entre
# deux journees (25/08). La figer dans le code obligeait a editer un fichier
# source a chaque bail. On la prend donc dans l'environnement, et a defaut on la
# CHERCHE sur le sous-reseau — voir `trouve_la_pi`.
PI = (os.environ.get('MYCOBOT_PI', '10.10.0.224'), PORT_PI)


def _repond(hote, port=PORT_PI, delai=0.6):
    """Ce pont-la parle-t-il vraiment ?

    Accepter la connexion ne suffit pas : le pont est mono-client et bloquant,
    et un client residuel le fige dans un etat ou le port reste OUVERT alors que
    plus aucune reponse n'arrive (constate le 25/08). Le seul test valable est
    donc une question a laquelle il doit repondre.
    """
    try:
        with socket.create_connection((hote, port), timeout=delai) as sock:
            sock.settimeout(delai)
            sock.sendall(b'{"action": "get_angles"}\n')
            return b'ANGLES' in sock.recv(4096).upper()
    except OSError:
        return False


def trouve_la_pi(delai=0.6):
    """Cherche un pont qui REPOND sur le sous-reseau local, ou None."""
    import ipaddress
    import subprocess
    from concurrent.futures import ThreadPoolExecutor

    sortie = subprocess.run(['ip', '-4', '-brief', 'addr'],
                            capture_output=True, text=True).stdout
    reseaux = []
    for ligne in sortie.splitlines():
        for mot in ligne.split()[2:]:
            if '/' in mot and not mot.startswith('127.'):
                reseaux.append(ipaddress.ip_interface(mot).network)
    for reseau in reseaux:
        if reseau.num_addresses > 512:
            continue
        hotes = [str(h) for h in reseau.hosts()]
        with ThreadPoolExecutor(max_workers=64) as pool:
            for hote, ok in zip(hotes, pool.map(lambda h: _repond(h, delai=delai), hotes)):
                if ok:
                    return hote
    return None
TOOL = np.array(json.loads((RACINE / 'scripts' / 'tool_offset.json').read_text())
                ['tool_offset_mm'], float)

# Pose de prise validee le 20/08 : elle fixe l'orientation de reference et son
# azimut. Toute cible reprend cette orientation, tournee de l'ecart d'azimut.
Q_REFERENCE = np.array([45.43, -86.57, -96.76, 135.52, 1.84, -43.33])
AZIMUT_REFERENCE = 7.2

# --------------------------------------------------------------------------- #
#  Recalage du deport d'outil, 08/09/2026 — TOUTES les hauteurs de ce fichier
#  ont ete decalees de +16,9 mm EN MEME TEMPS que `tool_offset.json` a ete
#  raccourci de 17,33 mm. Les deux vont ENSEMBLE : appliquer l'un sans l'autre
#  deplace la hauteur physique des doigts et casse toutes les prises reglees.
#
#  Mesure : sept lectures au reglet, bras immobile, contre la prediction du
#  modele. Le point d'outil se trouvait 16,9 mm SOUS le bout des doigts pince
#  fermee — ecart-type 1,0 mm sur trois azimuts couvrant 59 deg, pente
#  0,014 mm/deg. La DIRECTION du deport est donc bonne, c'est sa LONGUEUR qui
#  etait fausse : cela leve le doute inscrit dans `a_revalider` depuis le 20/08.
#
#  L'echelle verticale etait deja juste : descente commandee 44,6 mm, realisee
#  45,0. L'erreur etait un decalage, pas une dilatation.
#
#  CE QUE LE PATCH PRESERVE, ET CE QU'IL CHANGE — verifie, pas suppose :
#
#  * En Z l'invariant tient exactement. Avant, commander Z=15 mettait les
#    doigts a 31,9 ; desormais on commande 31,9 et ils y sont.
#  * En XY la cible se DEPLACE de 3,7 mm. L'axe outil n'est pas parfaitement
#    vertical ([-0,02, -0,31, -0,95] dans le monde), donc reculer de 17,33 mm
#    le long de cet axe vaut 16,5 mm en Z ET 5,4 mm de lateral. Les constantes
#    ci-dessous ne corrigent que le Z. Le reliquat lateral n'est pas un effet
#    de bord : l'ancien modele, a qui on demandait (280, 30), plaçait les
#    doigts en (277,8 ; 32,9). Ce patch les met en (280, 30).
#
#  Ce n'est donc PAS un patch neutre : c'est aussi une correction de 3,7 mm en
#  XY, dans le sens de la verite mais jamais validee par une prise.
#
#  A surveiller : sur les cibles de largage, l'IK bascule de branche entre
#  l'ancien et le nouveau reglage (jusqu'a 64 deg d'ecart articulaire pour le
#  meme point atteint). Le point final est bon, la POSTURE pour y aller change.
#
#  Ce qui s'ameliore : le modele dit enfin la verite. `garde_au_sol` et
#  `PLANCHER` protegent la vraie extremite des doigts, et non un point fictif
#  17 mm plus bas qui ne correspondait a aucune piece de la pince.
#
#  NON VALIDE PAR UNE SAISIE REELLE. Les deux cycles autonomes du 20/08 l'ont
#  ete avec l'ancien couple (deport long + hauteurs basses). Refaire une saisie
#  de balle avant de faire confiance a ce reglage.
# --------------------------------------------------------------------------- #
Z_PRISE = 11.9
# Hauteur de POINTE a la prise, par categorie. La balle fait 66 mm : elle tient
# les doigts ecartes, la pointe peut viser sous le plan de la table. Un rouleau
# de scotch couche ne fait que ~22 mm et le robot imprime ~20 : viser -5 ferait
# taper les doigts sur la planche AVANT qu'ils se referment — la pince se ferme
# alors a vide. On vise donc leur mi-hauteur.
# Hauteur de pointe a la prise, par categorie et par REGIME (outil vertical,
# outil couche). Mesure du 24/08 sur le vrai robot : recalage a 0,50 mm,
# descente a 1,30 mm, et pourtant « rien saisi » a chaque fois. La position
# etait juste — c'est la hauteur qui ne l'etait pas, et pour deux raisons
# distinctes :
#   * verticalement, viser -5 mm comme la balle fait taper les doigts sur la
#     planche avant qu'ils se referment : la balle fait 66 mm et les tient
#     ecartes, un rouleau couche 22 mm et le robot imprime ~20 ;
#   * couche (au-dela de 355 mm), l'outil visait Z=25 — la mi-hauteur de la
#     BALLE. Sur un objet plat de 22 mm, la pince se refermait entierement
#     au-dessus de lui, dans le vide. C'est ce cas qui a fait echouer les
#     saisies du scotch a 377 mm.
# On vise donc la mi-hauteur de CHAQUE objet, dans les deux regimes.
# La balle entre a +15, ce qui commande +5 une fois `BIAIS_Z_PRISE` applique.
# Les -5 historiques avaient ete regles AVANT que ce biais soit mesure : ils
# l'embarquaient deja en douce, et le garder tel quel revenait a l'appliquer
# deux fois. Balayage du 26/08 sur la balle, meme boucle fermee, l'angle auquel
# la pince CALE etant le vrai juge — a hauteur egale il ne bouge pas, et il
# saute des que les doigts butent sur autre chose que la balle :
#
#   z commande   modele atteint   resultat   angle de calage
#      +10,0         +11,3         saisi           53
#       +5,0          +6,3         saisi           53
#        0,0          +0,7         saisi           53
#       -5,0          -4,2         saisi           52
#      -10,0          -8,4         saisi           56
#      -15,0         -12,7         saisi           74   <- la PLANCHE, pas la balle
#
# Toutes « reussissent », mais a -15 la pince cale a 74 au lieu de 53 : les
# doigts sont arretes par la planche. On se place a +5, au milieu du palier a
# angle constant, 20 mm au-dessus de la hauteur qui degrade. Les deux objets
# plats (rouleau, robot), eux, restent a leur mi-hauteur geometrique.
# Second terme (outil couche) du rouleau : balayage du 26/08 a 347-357 mm, meme
# boucle fermee, l'angle de calage de la pince faisant juge — commande +3, +9 et
# +15 saisissent toutes les trois, calage 29-30 a chaque fois. On se pose au
# milieu de ce palier, soit z_prise 19 une fois BIAIS_Z_PRISE applique.
# Premier terme du rouleau (outil vertical) : -4 et non +2. Balayage du 26/08 a
# 271-280 mm, meme boucle fermee, l'angle de calage de la pince faisant juge —
# il ne bouge pas de toute la plage, donc les doigts serrent le ROULEAU et
# jamais la planche :
#
#   z commande   modele atteint   resultat   angle de calage
#       -8,0         -6,8          saisi           25
#      -12,0        -11,6          saisi           24
#      -16,0        -14,4          saisi           25
#      -20,0        -18,3          saisi           25
#
# Ce palier a ete lu comme une invitation a descendre au milieu (-14 commande)
# et c'etait FAUX : essaye le 26/08 sur un rouleau a 312 mm, les trois hauteurs
# -14, -18 et -20 se referment sur du VIDE, angle 20, et les doigts finissent
# par toucher la planche. Le palier ci-dessus a ete mesure a 271-280 mm et ne
# transfere pas a 312 : le biais du modele croit avec l'allonge (5,9 mm a 206,
# 11,9 a 327, 17,9 a 370), donc a allonge plus grande la meme consigne descend
# reellement plus bas. On revient a +2, la valeur qui a saisi tout au long de la
# journee sur toute la bande d'allonge, et on laisse les reprises de `_saisie`
# chercher 4 mm plus bas quand c'est necessaire.
Z_PRISE_PAR_CLASSE = {'balle': (31.9, 41.9), 'scotch': (18.9, 35.9),
                      'robot': (18.9, 26.9)}
# Et si ca rate quand meme, on ne refait PAS le meme geste : chaque nouvel essai
# descend d'un cran. Trois tentatives identiques donnent trois echecs identiques.
PAS_DESCENTE_ESSAI = 4.0
# Le modele place la pointe plus BAS qu'elle n'est reellement : viser la
# mi-hauteur d'un objet fait refermer les doigts au-dessus de lui. Balayage du
# 26/08 sur un rouleau (mi-hauteur Z=2), boucle fermee, meme objet, XY converge
# a chaque fois :
#
#   z commande   modele atteint   resultat        angle de calage
#      +2,0          +4,1         rien saisi           20
#      -4,0          -0,7         SAISI                24
#     -10,0          -7,9         SAISI                25
#     -16,0         -14,4         SAISI                25
#
# L'essai qui ECHOUE est celui dont le XY est le meilleur (1,89 mm contre 1,92
# et 2,19 pour deux reussites) : c'est la hauteur, seule. L'angle de calage a
# 24-25 sur les trois reussites confirme que les doigts serrent le rouleau.
#
# Ce biais n'est pas une propriete de l'objet mais du modele du bras, et c'est
# pourquoi la balle se saisissait quand meme : une sphere se laisse prendre par
# en dessous, un rouleau plat non. On vise donc 10 mm sous la mi-hauteur — le
# milieu de la plage mesuree, 6 mm de marge de chaque cote.
#
# Verifie ensuite par le chemin complet de la machine, 7 saisies sur 7 :
# rouleau 3/3 a 271-303 mm, petit robot 2/2 a 187-221 mm, balle 2/2. Une seule
# constante suffit donc sur toute cette bande, alors que le biais de saisie
# croit avec l'allonge (5,9 mm a 206, 11,9 a 327, 17,9 a 370 — signature d'une
# flexion) : au-dela de 303 mm elle reste a confirmer.
#
# Et elle n'est mesuree qu'en regime OUTIL VERTICAL. Au-dela de 355 mm la
# machine couche l'outil ; une erreur verticale ne s'y projette plus de la meme
# facon, et le second terme de `Z_PRISE_PAR_CLASSE` n'a pas ete rejuge.
BIAIS_Z_PRISE = -10.0
# Garde SOUS LES DOIGTS. `pointe()` rend le BOUT des doigts, pas le milieu de
# la pince : a la pose enseignee du 18/08 ou les doigts touchent la planche,
# elle vaut Z = -0,23 mm. Une consigne negative les enfonce donc dans le bois.
#
# C'est exactement ce que faisaient les objets PLATS. `Z_PRISE_PAR_CLASSE` donne
# 2 mm au scotch comme a la figurine, `BIAIS_Z_PRISE` retire 10 : la consigne
# tombait a -8, et l'ancien plancher valait -8 lui aussi, donc rien ne
# l'arretait. Les doigts s'appuyaient sur la planche et se refermaient EN
# RACLANT — symptome rapporte le 28/08. La balle, elle, visait +5 et n'a jamais
# frotte : la panne ne touchait que les objets plats.
#
# Historique conserve, car il explique pourquoi -8 avait ete choisi :
#
#   * approfondir NE SERT PAS — le 26/08 a 312 mm d'allonge, -14, -18 et -20 se
#     referment tous sur du VIDE (angle 20) et les doigts touchent la planche ;
#   * le bras arrive 6 a 25 mm SOUS sa consigne selon l'allonge (affaissement
#     mesure le 27/08) — c'est `converge` qui le rattrape, pas le plancher.
#
# La prise se faisait donc en APPUI : les doigts butaient sur le bois avant de
# se fermer. Cela tenait le rouleau, et c'est pourquoi le fichier avertissait
# que remonter ce plancher pouvait le faire lacher. On accepte desormais ce
# risque : ne pas racler la planche prime, quitte a laisser quelques dixiemes
# de millimetre entre les doigts et l'objet. Si un rouleau glisse, le levier
# reste le COUPLE (`COUPLE_PINCE`), jamais la profondeur.
GARDE_PLANCHE = 19.4
Z_PRISE_MIN = GARDE_PLANCHE
# Saut articulaire maximal tolere entre deux paliers de descente.
#
# 25 deg etait trop serre et refusait des descentes parfaitement saines : le
# rouleau blanc a 312 mm exigeait 39 deg sur son DERNIER palier, refuse trois
# fois d'affilee. Mesure du 26/08 sur cette cible exacte, en faisant varier la
# taille du palier — le saut est toujours sur la derniere marche, et le residu
# de pose y vaut 0,02 mm, donc la pose est bel et bien atteignable : c'est le
# poignet qui se reconfigure progressivement pres du sol, pas une branche qui
# change.
#
#   pas 35 mm (4 marches) : 34,6 deg      pas 15 mm ( 8 marches) : 24,0 deg
#   pas 25 mm (5 marches) : 30,9 deg      pas 10 mm (12 marches) : 18,9 deg
#   pas 20 mm (6 marches) : 28,0 deg
#
# Un vrai changement de branche, lui, coute 180 a 320 deg (balayage des colonnes
# du 26/08). Les deux familles sont a un ordre de grandeur l'une de l'autre : 60
# passe entre elles, avec 1,7x de marge au-dessus du plus gros saut legitime.
SAUT_PALIER_MAX = 60.0
# Hauteur d'un palier de descente. Plus long, l'interpolation articulaire
# s'ecarte trop de la verticale entre les deux bouts.
# 15 mm et non 35 : plus la marche est courte, moins le poignet a a se
# reconfigurer d'un palier au suivant (34,6 deg a 35 mm, 24,0 a 15 — mesure
# ci-dessous), et plus la pointe suit la verticale. Les paliers sont enchaines,
# ils ne coutent donc pas de temps.
PAS_PALIER_DESCENTE = 15.0
# Borne de la correction laterale accumulee pendant une descente. Au-dela, ce
# n'est plus l'affaissement du bras qu'on rattrape mais une cible fausse, et
# reinjecter plus fort ne fait que pousser les servos dans la butee (incident du
# 26/08). La derive mesuree utile vaut quelques millimetres.
DERIVE_DESCENTE_MAX = 25.0
# La descente se fait plus lentement que les transits : c'est le seul moment ou
# la pince arrive au contact, et une approche lente laisse le temps de couper.
VITESSE_DESCENTE = 15
Z_SURVOL = 126.9        # au-dessus du sommet de la balle (~71 mm)
Z_TRANSFERT = 186.9
Z_LARGAGE = 116.9       # plancher, quand la hauteur du rebord n'est pas mesuree
GARDE_LARGAGE = 41.9    # mm au-dessus du rebord MESURE, quand on le connait
# Affaissement de la pointe en pose de largage, mesure le 27/08 (voir
# `affaissement_largage`). Sans compensation la garde ci-dessus est mangee des
# 410 mm de portee et nulle a 470.
PENTE_AFFAISSEMENT_LARGAGE = 0.0943
ORIGINE_AFFAISSEMENT_LARGAGE = -20.43
GARDE_MIN = 41.9
PLANCHER = -3.1        # mm — aucune pose legitime sous la planche
CHUTE_MAX = 220.0       # mm — descente verticale maximale en UN seul ordre
ETAPES_MAX = 12         # decoupage maximal d'un grand deplacement
ESSAIS_MAX = 3
# Un objet dont les essais de saisie sont EPUISES est mis de cote, sinon le
# cycle suivant reprend le plus proche — c'est-a-dire le meme — et la machine
# tourne a vide indefiniment. Constate le 25/08 : un faux scotch a 252 mm, puis
# un vrai rouleau hors de portee de la pince, tous deux repris cycle apres
# cycle pendant que deux autres objets attendaient.
RAYON_OUBLI = 45.0        # mm — un objet a moins de ca d'un oubli EST cet oubli
DUREE_OUBLI = 180.0       # s — au-dela, on retente : la scene a pu changer
VITESSE = 25
# Delai d'engagement d'un ordre, mesure a 0,27-0,30 s sur huit mouvements.
DEMARRAGE_MAX = 0.6
LECTURES_CALMES = 6      # lectures d'affilee sous 0,05 deg — 0,18 s de silence
REPOS_MESURE = 0.3       # s — derive residuelle nulle au-dela, mesuree
# Mesure du 24/08, roulis libre, outil vertical, cible a hauteur de table ET
# survol a 110 : 330, 340 et 350 mm se resolvent (roulis +30, +30, +60), 360 non.
# La valeur precedente, 335, refusait des balles parfaitement atteignables — une
# a 343,6 mm a ete refusee alors qu'elle se resout avec un roulis de +30.
# Incliner l'outil n'ajoute rien ici : teste de +10 a +30 deg, aucune solution.
PORTEE_MAX = 430.0
# Et un PLANCHER d'allonge. Le 26/08 la pince est venue buter contre le J3 sur
# une cible a 151 mm : l'IK donnait une solution, le bras ne pouvait pas la
# tenir, l'ecart de recalage est reste bloque a 33-36 mm en CROISSANT, et la
# boucle a reinjecte l'ecart jusqu'a faire tomber le pont de la Pi.
#
# Cette borne est EMPIRIQUE, pas un modele de collision : `forward_kinematics`
# rend les centres d'articulation, sans l'encombrement des liens ni la largeur
# de la pince, et cette pose-la s'y lit a 152 mm de marge — le modele ne voit
# donc pas la collision. Ce qu'on a, ce sont les allonges reellement executees :
# 152 bloque, 178 et 189 saisissent proprement. On se place entre les deux.
# Le vrai garde-fou reste l'arret de reinjection dans `converge`, qui protege
# quelle que soit la cause — collision, butee ou obstacle.
PORTEE_MIN = 170.0
# L'outil TENU VERTICAL plafonne la : au-dela on le couche, et le scan vertical
# n'a plus lieu d'etre tente (huit roulis qui echouent coutent 40 s).
#
# Piste essayee et FERMEE le 24/08 : cette limite avait ete mesuree en exigeant
# aussi Z_TRANSFERT=170 au-dessus du point de prise, contrainte qui n'a pas lieu
# d'etre sur un objet pose a plat. En ne demandant que le survol et la prise, la
# limite ne bouge pas d'un millimetre — 350 mm passe, 360 non, dans les deux cas.
# C'est bien une limite mecanique. Ne pas re-tenter.
PORTEE_VERTICALE_MAX = 355.0
# L'outil fait 110 mm : couche, la bride n'a plus besoin d'aller aussi loin que
# la pointe. Mesure du 24/08, survol 110 ET prise resolus par la meme
# orientation : 360 mm a -15 deg, 380 et 400 a -30, 420 a -30, 440 a -45.
INCLINAISONS = [0.0, -15.0, -30.0, -45.0]
# Inclinaison minimale exigee par l'allonge — mesuree, survol ET prise resolus.
#
# La premiere borne n'est PAS PORTEE_VERTICALE_MAX (355). Atteindre la prise et
# atteindre le survol ne dit rien de ce qu'il y a ENTRE les deux, et c'est la
# que ca se joue. Balayage du 26/08, colonne verticale complete de Z=110 au ras
# de la table, par pas de 15 mm, pour chaque roulis (saut articulaire maximal
# d'un pas au suivant) :
#
#   portee | roulis qui descendent droit | ce que font les autres
#      240 | +0 +30 +60 +90 +120         | -30 : trou, saut 197 deg
#      270 | +0 +30 +60                  | +90 : 243 deg, +120 : 319 deg
#      300 | +0 +30 +90                  | +60 : 235 deg
#      320 | +30 +60                     | +90 : 246 deg
#      335 | AUCUN                       | +30 : 199 deg, +60 : 194 deg
#      350 | AUCUN                       | +60 : 208 deg
#      355 | AUCUN                       | +60 : 203 deg
#
# A partir de 335 mm la colonne verticale est trouee pour TOUS les roulis : le
# bras ne peut rejoindre la prise qu'en changeant de branche, ~200 deg de
# poignet a quelques centimetres de la planche. C'est ce balayage qui raclait la
# table. Couche, la meme cible descend droit des -15 deg. On arrete donc le
# vertical a 325, entre le dernier propre (320) et le premier troue (335).
# La bascule vertical -> couche est passee de 325 a 320 le 27/08. 325 avait ete
# pose « entre le dernier propre (320) et le premier troue (335) » — un milieu
# raisonne, pas mesure. Le rouleau blanc a 324 mm l'a mis en defaut : la machine
# choisissait l'outil vertical et la descente etait refusee, « palier Z=-8 exige
# 190 deg — changement de branche refuse ». Balayage geometrique du meme jour,
# descente jusqu'a Z=-8 :
#
#   portee |  300  310  315  320  324  326  330  335  340
#   droit  |  ok   ok   ok   ok   NON  NON  NON  NON  NON
#   -15    |  ok   ok   ok   ok   ok   ok   ok   ok   ok
#
# 320 est donc la derniere valeur VERIFIEE, et l'outil couche resout tout le
# reste avec le roulis nominal.
INCLINAISONS_PAR_PORTEE = ((320.0, 0.0), (370.0, -15.0), (425.0, -30.0),
                           (1e9, -45.0))
# Incline, la pointe vise le CENTRE de la balle et non le ras de la table : les
# doigts l'abordent de biais, descendre au ras la pousserait.
Z_PRISE_INCLINE = 41.9
# Le largage est PLUS exigeant que la prise : la pointe doit etre loin ET haut
# (transfert a 170 puis lacher a 100), alors que la prise se fait au ras de la
# table. Outil tenu vertical il plafonne a 355 mm — c'est ce qui bloquait TOUT
# le cycle le 24/08 : carton mesure a 449 mm, quatre points de largage refuses,
# balle jamais saisie faute de savoir ou la deposer.
#
# Mesure du 24/08 a l'azimut du carton, transfert ET largage resolus par la
# meme orientation (roulis retenu entre parentheses) :
#
#   portee |   0     -15    -30    -45    -60
#      360 |   .    (+60)  (+0)   (+0)   (+0)
#      380 |   .     .     (+0)   (+0)   (+0)
#      400 |   .     .     (+0)   (+0)   (+0)
#      420 |   .     .      .     (+0)   (+0)
#      440 |   .     .      .     (+0)   (+0)
#      460 |   .     .      .      .     (+0)
#
# Coucher l'outil porte donc la depose de 355 a 460 mm. Abaisser en plus le
# transfert a 130 mm a ete mesure : ca ne gagne qu'une bande (420 passe a -30
# au lieu de -45) et ca rapproche la trajectoire du rebord — ecarte.
PORTEE_CARTON_MAX = 460.0
INCLINAISONS_CARTON = [0.0, -15.0, -30.0, -45.0, -60.0, -75.0]
# (allonge maximale, inclinaison minimale exigee)
INCLINAISONS_CARTON_PAR_PORTEE = ((355.0, 0.0), (365.0, -15.0), (410.0, -30.0),
                                  (450.0, -45.0), (1e9, -60.0))
ESSAIS_CARTON = 2       # tours de balayage avant de renoncer, objet en main
MARGE_LARGAGE = 38.0    # mm — recul des parois : la balle a 32,1 mm de rayon (mesure)
MARGE_LARGAGE_MIN = 15.0  # plancher quand l'ouverture ne peut pas offrir mieux
LARGAGES_TESTES = 3     # points d'ouverture essayes avant de renoncer
# Au-dela, le MILIEU d'une ouverture n'est plus un point de largage raisonnable
# meme quand l'IK le resout : c'est la zone ou le bras n'arrive plus. On y vise
# le bord proche de la meme ouverture, qui depose tout aussi bien dedans.
PORTEE_LARGAGE_CONFORT = 400.0
MEMOIRE_CARTON = RACINE / 'scripts' / 'carton_position.json'
MEMOIRE_AFFAISSEMENT = RACINE / 'scripts' / 'affaissement.json'
AFFAISSEMENT_MAX = 6.0  # deg — borne de l'ecart articulaire reinjecte au depart
# Le carton ne bouge pas entre le debut d'un cycle et la depose : re-resoudre sa
# pose a chaque etat coutait 4 a 15 s pour un resultat identique.
# La pose resolue n'est perimee que par un VRAI deplacement du carton, signale
# par le suivi (six images concordantes). Le centre detecte, lui, tremble de
# quelques mm en permanence : le re-resoudre la-dessus coutait 4 a 25 s par
# cycle pour un resultat identique.
TOLERANCE_CARTON_RESOLU = 30.0
# Un point de largage doit rester DANS l'ouverture qu'il visait : au-dela d'un
# carton d'ecart avec la position suivie, il designe autre chose.
DERIVE_CARTON_MAX = 150.0
# Ecart tolere entre le point de largage COMMANDE et la pointe REELLE au moment
# d'ouvrir la pince. Le 26/08 la machine a commande (401, 204) a 450 mm, le bras
# s'est immobilise a (373, -52) — 256 mm plus loin, entre les deux boites — et la
# pince s'est ouverte quand meme : le petit robot est tombe a cote du petit
# carton. Rien ne verifiait l'ARRIVEE, seulement que l'ordre avait ete accepte.
# 45 mm laisse passer le biais de modele (18 mm mesure a 370 mm de portee) et
# arrete tout le reste. Une ouverture de carton fait 88 mm de cote (petit) a
# 112 mm (grand) : un objet lache a 45 mm du point vise, choisi avec 15 a 38 mm
# de recul des parois, tombe encore dedans.
ECART_LARGAGE_MAX = 45.0

# Pour une sphere, l'orientation des doigts dans le plan horizontal est sans
# importance : ce roulis est un degre de liberte gratuit qui porte la portee
# utile de 300 a 330 mm. Balayage du plus proche du nominal au plus eloigne.
ROULIS = [0, 30, -30, 60, -60, 90, -90, 120]

ETAT_PINCE = {0: 'en mouvement', 1: 'rien saisi', 2: 'objet saisi', 3: 'objet lache'}

SEUIL_DEPLACEMENT = 8.0   # mm — au-dela, la balle a bouge : on refait la cible
# Le bras masque ce qu'il survole : une detection de carton proche du bras est
# celle de l'ombre ou du bras lui-meme. Mesure du 24/08 : elle sautait de 55, 76
# puis 171 mm d'un pas a l'autre, carton immobile. La distance se mesure au bras
# ENTIER (voir `distance_au_bras`) : ce sont ses segments qui portent l'ombre,
# pas seulement sa pointe.
RAYON_MASQUAGE = 150.0

POSE_OBSERVATION = np.array([-27.50, -61.61, -41.30, 89.56, 0.43, -79.62])

# Le bras s'ecarte en tournant sur J1 jusqu'a ce que la camera revoie la balle.
# La pose d'observation d'abord, puis de plus en plus loin, des deux cotes.
BALAYAGE_J1 = [-27.5, -55.0, -85.0, 15.0, 45.0, -115.0]


# --------------------------------------------------------------------------- #
#  Pont TCP vers la Pi
# --------------------------------------------------------------------------- #

class Pont:
    """Client du `gripper_bridge.py` de la Pi. Protocole texte, une ligne."""

    def __init__(self, timeout=8.0):
        global PI
        self.timeout = timeout
        if not _repond(*PI):
            trouvee = trouve_la_pi()
            if trouvee is not None and trouvee != PI[0]:
                print(f'pont introuvable en {PI[0]} — trouve en {trouvee}')
                PI = (trouvee, PORT_PI)
        self.sock = socket.create_connection(PI, timeout=timeout)
        self.sock.settimeout(timeout)
        self.buf = b''
        # La detection du carton doit masquer le bras. Elle ne peut pas
        # interroger le pont elle-meme : il est mono-client et bloquant, et la
        # machine a etats s'en sert deja. On lui laisse donc la derniere pose lue.
        self.derniers_angles = None

    def _vide(self):
        """Jette le tampon avant d'emettre.

        Une commande qui repond sur deux lignes desynchronise le dialogue : la
        lecture suivante recupere la reponse PRECEDENTE. C'est ce qui a fait lire
        une pose perimee a Z=585 mm puis commander un plongeon de 590 mm.
        """
        self.buf = b''
        self.sock.setblocking(False)
        try:
            while self.sock.recv(4096):
                pass
        except (BlockingIOError, OSError):
            pass
        finally:
            self.sock.setblocking(True)
            self.sock.settimeout(self.timeout)

    def envoie(self, action, **kw):
        self._vide()
        self.sock.sendall((json.dumps({'action': action, **kw}) + '\n').encode())
        while b'\n' not in self.buf:
            bloc = self.sock.recv(4096)
            if not bloc:
                raise ConnectionError('pont ferme')
            self.buf += bloc
        ligne, self.buf = self.buf.split(b'\n', 1)
        return ligne.decode().strip()

    def angles(self):
        """N'accepte qu'une reponse etiquetee ANGLES et dans le domaine."""
        derniere = None
        for _ in range(3):
            derniere = self.envoie('get_angles')
            if 'ANGLES' not in derniere.upper():
                time.sleep(0.3)
                continue
            try:
                q = np.asarray(json.loads(derniere.split(':', 1)[-1].strip()), float)
            except (json.JSONDecodeError, ValueError, TypeError):
                time.sleep(0.3)
                continue
            if q.size == 6 and np.all(np.isfinite(q)) and np.all(np.abs(q) <= 200.0):
                self.derniers_angles = q
                return q
            time.sleep(0.3)
        raise RuntimeError(f'get_angles illisible : {derniere!r}')

    def statut_pince(self):
        rep = self.envoie('get_pro_gripper_status')
        chiffres = ''.join(c for c in rep if c.isdigit())
        return int(chiffres[0]) if chiffres else -1

    def angle_pince(self):
        rep = self.envoie('get_pro_gripper_angle')
        chiffres = ''.join(c for c in rep if c.isdigit() or c == '-')
        return int(chiffres) if chiffres.lstrip('-').isdigit() else -1

    def ferme(self):
        self.sock.close()


# --------------------------------------------------------------------------- #
#  Geometrie
# --------------------------------------------------------------------------- #

def pose_bride(q_deg):
    positions, transformations = forward_kinematics(np.radians(np.asarray(q_deg, float)))
    return np.asarray(positions['mycobot320_link6']) * 1000.0, transformations[6][:3, :3]


def distance_au_bras(xy, q_deg):
    """Distance en XY entre un point du plan et le bras tout entier (mm).

    La pointe seule ne suffit pas : ce sont les SEGMENTS du bras qui portent
    l'ombre prise pour une ouverture de carton, et ils couvrent beaucoup plus
    que leur extremite.
    """
    positions, _ = forward_kinematics(np.radians(np.asarray(q_deg, float)))
    chaine = [np.asarray(v, float)[:2] * 1000.0 for v in positions.values()]
    chaine.append(pointe(q_deg)[:2])
    xy = np.asarray(xy, float)
    proche = float('inf')
    for a, b in zip(chaine, chaine[1:]):
        ab = b - a
        longueur = float(ab @ ab)
        t = 0.0 if longueur == 0 else float(np.clip((xy - a) @ ab / longueur, 0.0, 1.0))
        proche = min(proche, float(np.linalg.norm(xy - (a + t * ab))))
    return proche


def pointe(q_deg):
    p, R = pose_bride(q_deg)
    return p + R @ TOOL


# --------------------------------------------------------------------------- #
#  Anticollision par capsules
# --------------------------------------------------------------------------- #
# Chaque lien est un SEGMENT entoure d'un rayon ; deux liens se touchent quand
# la distance entre leurs segments passe sous la somme des rayons plus une
# marge. Les rayons ne sont pas repris d'un schema de robot generique : ils
# sortent des maillages de CE bras (`mycobot_description/urdf/320_pi/*.dae`,
# unites mm), demi-section la plus large de chaque lien.
RAYONS_CAPSULE = {'colonne': 58.0, 'bras': 48.0, 'avant_bras': 43.0,
                  'poignet': 44.0, 'bride': 29.0, 'pince': 45.0}
MARGE_CAPSULE = 5.0
# Paires qu'on NE TESTE PAS, et pourquoi. Mesure du 27/08, 4000 poses tirees
# dans les butees :
#
#   * poignet/pince — marge de -27,7 a -27,6 mm, soit 0,1 mm d'amplitude. Elle
#     ne depend d'AUCUNE articulation : la bride ne fait que 19 mm d'epaisseur,
#     poignet et pince sont un seul bloc mecanique. C'est une constante
#     geometrique, pas une collision.
#   * avant_bras/bride et avant_bras/pince — une capsule de 45 mm autour de la
#     pince couvre une fourche qui est surtout du vide. Resultat : -8,0 mm sur
#     POSE_OBSERVATION et -6,7 mm sur une prise a 300 mm, deux poses que ce
#     robot tient tous les jours. Tant que le volume reel de la pince n'est pas
#     mesure, ces paires refuseraient le travail au lieu de le proteger.
#
# Restent sept paires, toutes verifiees positives sur l'enveloppe de travail.
PAIRES_CAPSULE_EXCLUES = {('poignet', 'pince'), ('avant_bras', 'bride'),
                          ('avant_bras', 'pince')}


def capsules(q_deg):
    """[(nom, extremite, extremite)] — le bras en six segments, pince comprise."""
    p, R = pose_bride(q_deg)
    origines, _ = forward_kinematics(np.radians(np.asarray(q_deg, float)))
    o = lambda k: np.asarray(origines[f'mycobot320_{k}'], float) * 1000.0
    return [('colonne', o('base'), o('link1')),
            ('bras', o('link1'), o('link3')),
            ('avant_bras', o('link3'), o('link4')),
            ('poignet', o('link4'), o('link5')),
            ('bride', o('link5'), o('link6')),
            ('pince', o('link6'), p + R @ TOOL)]


def _distance_segments(a0, a1, b0, b1):
    u, v, w = a1 - a0, b1 - b0, a0 - b0
    a, b, c, d, e = u @ u, u @ v, v @ v, u @ w, v @ w
    den = a * c - b * b
    t = (e / c if c > 1e-9 else 0.0) if den < 1e-9 else 0.0
    s = 0.0 if den < 1e-9 else np.clip((b * e - c * d) / den, 0.0, 1.0)
    if den >= 1e-9:
        s = np.clip((b * e - c * d) / den, 0.0, 1.0)
        t = np.clip((a * e - b * d) / den, 0.0, 1.0)
    if a > 1e-9:
        s = float(np.clip((b * t - d) / a, 0.0, 1.0))
    if c > 1e-9:
        t = float(np.clip((b * s + e) / c, 0.0, 1.0))
    return float(np.linalg.norm(w + s * u - t * v))


def marge_collision(q_deg):
    """(marge la plus faible en mm, paire concernee). Negative = contact.

    La paire `colonne/avant_bras` est celle qui compte : elle passe sous zero a
    ~143 mm d'allonge, ce qui retrouve tout seul l'incident du 26/08 — pince
    coincee contre J3, cible a 151 mm — et le plancher `PORTEE_MIN` pose
    empiriquement apres coup.
    """
    seg = capsules(q_deg)
    pire, ou = float('inf'), ''
    for i in range(len(seg)):
        for j in range(i + 2, len(seg)):        # deux liens VOISINS se touchent
            if (seg[i][0], seg[j][0]) in PAIRES_CAPSULE_EXCLUES:
                continue
            marge = (_distance_segments(seg[i][1], seg[i][2], seg[j][1], seg[j][2])
                     - RAYONS_CAPSULE[seg[i][0]] - RAYONS_CAPSULE[seg[j][0]]
                     - MARGE_CAPSULE)
            if marge < pire:
                pire, ou = marge, f'{seg[i][0]}/{seg[j][0]}'
    return pire, ou


def garde_au_sol(q_deg):
    """Point le plus bas de tout le bras, pointe comprise (mm)."""
    p, R = pose_bride(q_deg)
    positions, _ = forward_kinematics(np.radians(np.asarray(q_deg, float)))
    return min(float((p + R @ TOOL)[2]),
               min(v[2] * 1000.0 for k, v in positions.items() if 'link' in k))


def _rz(degres):
    k = np.radians(degres)
    return np.array([[np.cos(k), -np.sin(k), 0.0],
                     [np.sin(k), np.cos(k), 0.0],
                     [0.0, 0.0, 1.0]])


_, R_REFERENCE = pose_bride(Q_REFERENCE)


def orientation(p_xy, roulis=0.0):
    azimut = float(np.degrees(np.arctan2(p_xy[1], p_xy[0])))
    return _rz(azimut - AZIMUT_REFERENCE + roulis) @ R_REFERENCE


# Amorces : eventail synthetique LARGE **plus les poses reellement atteintes**.
# Enchainer les amorces produit des faux "hors d'atteinte" en cascade ; un
# eventail purement synthetique rate le bon bassin et declare inatteignable un
# point que le robot a physiquement atteint. Les deux erreurs ont ete commises.
_AMORCES_MESUREES = [
    Q_REFERENCE,
    np.array([1.04, -70.77, -23.42, 86.76, 10.61, -83.82]),
    np.array([1.04, -80.13, -27.88, 100.58, 10.61, -83.83]),
    np.array([45.42, -13.24, -112.27, 77.93, 1.84, -43.56]),
    POSE_OBSERVATION,
]
_AMORCES_SYNTHETIQUES = [np.array(a, float) for a in (
    (0, -90, -60, 120, 0, 0), (0, -60, -40, 90, 0, 0), (0, -110, -80, 150, 0, 0),
    (0, -75, -100, 130, 0, 0), (0, -45, -20, 60, 0, 0), (0, -95, -30, 100, 0, 0),
    (0, -120, -50, 140, 0, 0), (0, -30, -70, 80, 0, 0), (0, -85, -25, 95, 0, 0),
    (0, -70, -110, 140, 0, 0), (0, -100, -95, 145, 0, 0), (0, -55, -85, 110, 0, 0),
)]


def resout_ik(p_cible, R, tol_mm=1.0, tol_deg=1.0, coude_haut=True,
              amorces_max=None, iterations=150):
    """Meilleure solution parmi les amorces, ou None.

    Sortie anticipee des qu'une amorce donne une solution nettement bonne :
    balayer les 22 amorces jusqu'au bout coute des secondes, et les etats de
    largage en enchainent plusieurs. Le seuil de sortie est trois fois plus
    severe que le seuil d'acceptation, pour ne s'arreter que sur une solution
    qui ne demande aucun arbitrage.

    L'ORDRE des amorces compte autant que leur nombre. Mesure du 26/08 sur une
    cible de largage (362, 152), amorce par amorce :

        amorces 0-3 (poses mesurees telles quelles)  216 ms chacune, residu ~100 mm
        amorce  4   (la meme, J1 recale sur l'azimut)  45 ms, residu 0,000 mm

    Une amorce dont le J1 pointe ailleurs que la cible ne converge pas, et paie
    le budget d'iterations en entier : 865 des 910 ms d'un appel partaient la,
    sans jamais changer la solution retenue. Les amorces recalees passent donc
    devant, les brutes restent en dernier recours.

    Le budget est descendu de 400 a 150 iterations : mesure sur la meme cible,
    150 donne exactement le meme resultat que 400 (meme premiere amorce bonne,
    15 amorces valides sur 22) et divise par 2,7 le cout d'une cible SANS
    solution, ou le balayage va forcement jusqu'au bout. A 60 la qualite tombe
    (12 amorces valides), donc la marge est gardee.
    """
    azimut = float(np.degrees(np.arctan2(p_cible[1], p_cible[0])))
    amorces = []
    for q in _AMORCES_MESUREES + _AMORCES_SYNTHETIQUES:
        s = q.copy()
        s[0] = azimut
        amorces.append(s)
    amorces += list(_AMORCES_MESUREES)
    meilleure = None
    for amorce in amorces[:amorces_max]:
        q = solve_pose(amorce, np.asarray(p_cible) - R @ TOOL, R,
                       rot_weight=400.0, max_joint_step_deg=3.0, iterations=iterations)
        if coude_haut and q[2] > 0:
            continue
        if not np.all((q >= LIMITES[:, 0]) & (q <= LIMITES[:, 1])):
            continue
        p, Rq = pose_bride(q)
        residu = float(np.linalg.norm(p + Rq @ TOOL - np.asarray(p_cible)))
        ecart = orientation_error_deg(q, R)
        if meilleure is None or residu + ecart < meilleure[1] + meilleure[2]:
            meilleure = (q, residu, ecart)
        if meilleure[1] < tol_mm / 3.0 and meilleure[2] < tol_deg / 3.0:
            break
    if meilleure is None or meilleure[1] > tol_mm or meilleure[2] > tol_deg:
        return None
    return meilleure


def incline(R, azimut_deg, theta_deg):
    """Couche l'outil de theta autour de l'axe tangentiel a l'allonge."""
    if abs(theta_deg) < 1e-6:
        return R
    a = np.radians(azimut_deg)
    u = np.array([-np.sin(a), np.cos(a), 0.0])
    M, _ = cv2.Rodrigues(u * np.radians(theta_deg))
    return M @ R


def colonne_continue(xy, R, z_bas, z_haut=None):
    """La descente tient-elle sur UNE SEULE branche IK, de bout en bout ?

    Verifier que le survol et la prise se resolvent ne suffit pas : entre les
    deux, l'orientation retenue peut n'avoir aucune solution. Le bras rejoint
    alors la prise en changeant de branche — ~200 deg de poignet a quelques
    centimetres de la planche — et c'est ce balayage qui raclait la table.
    """
    z_haut = Z_SURVOL if z_haut is None else z_haut
    depart = resout_ik(np.array([xy[0], xy[1], z_haut]), R)
    if depart is None:
        return False
    q = depart[0]
    # EXACTEMENT la discretisation de `descend_par_paliers`, bord bas compris.
    # Par pas fixe depuis le haut, le dernier cran tombait avant l'arrivee : la
    # colonne etait declaree bonne jusqu'a -10 pendant que la descente visait
    # -14, et c'est dans ces 4 mm-la que le rouleau a 312 mm demandait 196 deg
    # de saut de branche (26/08). On ne valide plus autre chose que ce qui sera
    # execute.
    marches = max(1, int(np.ceil((z_haut - z_bas) / PAS_PALIER_DESCENTE)))
    for i in range(1, marches + 1):
        z = z_haut + (z_bas - z_haut) * i / marches
        cible = np.array([xy[0], xy[1], float(z)])
        q_z = solve_pose(q, cible - R @ TOOL, R, rot_weight=400.0,
                         max_joint_step_deg=3.0, iterations=150)
        if (float(np.linalg.norm(pointe(q_z) - cible)) > 2.0
                or float(np.abs(q_z - q).max()) > SAUT_PALIER_MAX):
            return False
        q = q_z
    return True


def choisit_pose_prise(ctx, xy):
    """(roulis, inclinaison, R, hauteur de prise) pour saisir en xy, ou None.

    L'outil vertical est essaye d'abord : c'est le regime valide, la pince
    enveloppe la balle par le dessus. Il plafonne a PORTEE_VERTICALE_MAX. Au-dela
    on le couche progressivement, ce qui porte l'atteignabilite de 355 a 440 mm.
    La combinaison qui a marche pour cette bande d'allonge est reessayee en
    premier — un couple qui echoue coute 5 s, celui qui marche 40 ms.
    """
    azimut = float(np.degrees(np.arctan2(xy[1], xy[0])))
    portee = float(np.hypot(*xy))
    # On part de l'inclinaison MINIMALE que l'allonge exige, au lieu de balayer
    # depuis la verticale : a 384 mm, essayer 0 puis -15 avant -30 coutait 41 s
    # pour un resultat connu d'avance.
    depart = next(t for limite, t in INCLINAISONS_PAR_PORTEE if portee <= limite)
    ordre_theta = [t for t in INCLINAISONS if t <= depart] or [INCLINAISONS[-1]]
    couples = [(t, r) for t in ordre_theta for r in ROULIS]
    cle = cle_roulis('balle', xy)
    # Un couple qui a referme la pince a vide passe en DERNIER, il n'est pas
    # supprime : quand ils ont tous echoue il faut bien en reproposer un.
    rates = ctx.prises_ratees.get(cle, [])
    couples = ([c for c in couples if c not in rates]
               + [c for c in couples if c in rates])
    memo = ctx.prise_apprise.get(cle)
    if memo in couples and memo not in rates:
        couples.insert(0, couples.pop(couples.index(memo)))
    # Le repli generique doit se VOIR. Le 09/09 un rouleau a ete vise avec
    # `classe_objet` vide — vide par les sorties d'echec de `_detecte`, jamais
    # rearme par l'appelant : la machine a pris (11,9 ; 41,9) au lieu des
    # (18,9 ; 35,9) du scotch, a commande 32 mm au lieu de 26, et s'est refermee
    # quatre fois sur du vide 23 mm au-dessus du rouleau, en le poussant de 382
    # a 401 mm d'allonge. Le journal ne disait que « prise a Z=42 » : rien qui
    # laisse deviner que la classe manquait.
    if ctx.classe_objet not in Z_PRISE_PAR_CLASSE:
        ctx.note(f'classe "{ctx.classe_objet}" inconnue — hauteurs de prise '
                 f'GENERIQUES ({Z_PRISE}, {Z_PRISE_INCLINE}), pas celles de l objet')
    debout, couche = Z_PRISE_PAR_CLASSE.get(ctx.classe_objet,
                                            (Z_PRISE, Z_PRISE_INCLINE))
    for theta, roulis in couples:
        z_prise = debout if theta == 0.0 else couche
        hauteurs = ([Z_TRANSFERT, Z_SURVOL, z_prise] if theta == 0.0
                    else [Z_SURVOL, z_prise])
        R = incline(orientation(xy, roulis), azimut, theta)
        # La colonne se verifie jusqu'a la hauteur REELLEMENT commandee, pas
        # jusqu'a `z_prise` : `_descente` vise `z_prise + BIAIS_Z_PRISE`, soit
        # 10 mm plus bas. Ces 10 mm-la n'etaient pas controles, et c'est
        # exactement dans ce dernier palier que le rouleau a 319 mm demandait
        # 200 deg de saut de branche (26/08) — refuse a l'execution, donc trois
        # essais perdus, alors qu'une autre orientation descendait droit.
        z_bas = max(Z_PRISE_MIN, z_prise + BIAIS_Z_PRISE)
        if (all(resout_ik(np.array([xy[0], xy[1], z]), R) is not None for z in hauteurs)
                and colonne_continue(xy, R, z_bas)):
            ctx.prise_apprise[cle] = (theta, roulis)
            # La profondeur VALIDEE voyage avec la pose : les reprises de
            # `_saisie` descendent d'un cran a chaque essai a vide, et sans
            # cette borne elles sortaient de la colonne verifiee — troisieme
            # essai refuse pour 194 deg de saut de branche, les deux premiers
            # etant passes (26/08). Valider plus bas d'emblee n'est pas la
            # reponse : a 312 mm cela ecarte le roulis +30, qui saisit, au
            # profit du +60, qui attrape le rouleau par le bord.
            ctx.z_valide = z_bas
            return roulis, theta, R, z_prise
    return None


def cle_roulis(nom, xy):
    """Le bon roulis depend surtout de l'ALLONGE, pas de l'azimut.

    Un roulis appris de pres ne vaut rien de loin : mesure du 24/08, celui
    retenu a 257 mm echoue a 357 mm et coute 5 s avant qu'on trouve le bon. On
    le range donc par bande d'allonge de 25 mm.
    """
    return f'{nom}@{int(np.hypot(*np.asarray(xy, float)) // 25) * 25}'


def roulis_retenu(ctx, nom, xy):
    """Roulis retenu pour cette allonge, a defaut celui de l'allonge voisine."""
    cle = cle_roulis(nom, xy)
    if cle in ctx.roulis_appris:
        return ctx.roulis_appris[cle]
    portee = float(np.hypot(*np.asarray(xy, float)))
    voisins = [(abs(float(c.split('@')[1]) - portee), angle)
               for c, angle in ctx.roulis_appris.items() if c.startswith(nom + '@')]
    return min(voisins)[1] if voisins else None


def choisit_roulis(p_xy, hauteurs, prefere=None):
    """Roulis le plus proche du nominal resolvant TOUTES les hauteurs voulues.

    Scan allege d'abord : un roulis qui echoue doit epuiser toutes les amorces,
    ce qui coute 4,9 s contre 15 ms pour un succes. Ici on ne cherche qu'a
    SELECTIONNER — pour une sphere n'importe quel roulis qui passe convient.

    Mais si le scan allege ne trouve RIEN, on refait le tour au complet avant
    de renoncer : mesure a (320, 60), l'allege ne voyait aucun roulis la ou le
    complet en trouvait un. Un faux negatif est sans consequence tant qu'il
    reste un autre roulis ; quand ils tombent tous, il fait refuser une balle
    parfaitement atteignable.
    """
    # Au-dela de ~310 mm le scan allege ne trouve jamais rien (mesure a 325 et
    # 330 mm, la ou le complet trouve +30) : lui epargner un tour pour rien.
    reglages = [{}] if float(np.hypot(*p_xy)) > 310.0 else [{'amorces_max': 8,
                                                             'iterations': 150}, {}]
    # Le roulis qui a marche au coup precedent d'abord. Un roulis qui ECHOUE
    # doit epuiser les 22 amorces, soit 5 s ; celui qui reussit repond en 40 ms.
    # Commencer par le bon fait tomber le choix de 5,24 s a 0,04 s (mesure du
    # 24/08) — et le carton comme la balle bougent peu d'un cycle a l'autre.
    ordre = ROULIS if prefere is None else [prefere] + [a for a in ROULIS if a != prefere]
    for reglage in reglages:
        for angle in ordre:
            R = orientation(p_xy, angle)
            if all(resout_ik(np.array([p_xy[0], p_xy[1], z]), R, **reglage) is not None
                   for z in hauteurs):
                return angle, R
    return None, None


def autotest_ik():
    """Chaque pose reellement atteinte doit etre retrouvee depuis sa cartesienne.

    Un solveur local peut declarer inatteignable un point deja atteint ; sans ce
    controle on conclut a tort a une limite mecanique.
    """
    for q in _AMORCES_MESUREES:
        p, R = pose_bride(q)
        if resout_ik(p + R @ TOOL, R) is None:
            return False
    return True


# --------------------------------------------------------------------------- #
#  Mouvements
# --------------------------------------------------------------------------- #

@dataclass
class Contexte:
    """Etat partage entre les etats, et vitrine pour l'interface."""
    pont: object = None
    balle_xy: np.ndarray = None
    carton_xy: np.ndarray = None
    carton_polygone: np.ndarray = None    # ouverture en mm, repere base
    z_rebord: float = None                # hauteur mesuree du rebord, mm
    couple_pince: int = None              # dernier couple envoye a la pince
    roulis_balle: float = 0.0
    roulis_carton: float = 0.0
    roulis_appris: dict = field(default_factory=dict)   # {'balle': deg, 'carton': deg}
    carton_resolu: tuple = None       # (centre resolu, point de largage, roulis, R)
    prise_apprise: dict = field(default_factory=dict)  # bande d'allonge -> (inclinaison, roulis)
    # Couples (inclinaison, roulis) qui ont referme la pince A VIDE a cette
    # bande d'allonge. Mesure du 27/08 sur le rouleau bleu a 389 mm : le
    # roulis 0 le POUSSE de 6 mm au lieu de le saisir, le roulis +30 le tient
    # (statut 2, angle 26, garde jusqu'a Z=170). Les deux sont geometriquement
    # valides — seule la prise reelle les separe, et sans cette memoire les
    # trois essais rejouaient le meme roulis rate.
    prises_ratees: dict = field(default_factory=dict)
    inclinaison_balle: float = 0.0
    z_prise: float = Z_PRISE
    R_balle: np.ndarray = None
    R_carton: np.ndarray = None
    correction: np.ndarray = field(default_factory=lambda: np.zeros(6))
    biais_descente: np.ndarray = field(default_factory=lambda: np.zeros(3))
    # Le meme biais, mais garde d'un OBJET A L'AUTRE. La derive laterale de la
    # descente est une propriete du BRAS (l'affaissement sous la gravite), pas de
    # l'objet vise : la remettre a zero a chaque cible faisait repayer a chacune
    # la passe de mesure — descendre, remonter, redescendre, quatre fois par tri.
    biais_appris: np.ndarray = field(default_factory=lambda: np.zeros(3))
    essais: dict = field(default_factory=dict)
    resultats: dict = field(default_factory=dict)
    journal: list = field(default_factory=list)
    mode_auto: bool = False
    echecs: int = 0                   # echecs d'affilee, remis a zero par une depose
    chrono: dict = field(default_factory=dict)   # secondes passees par etat, cycle courant
    derniere_consigne: object = None  # angles du dernier ordre, bras peut-etre encore en route
    en_main: str = ''                 # classe de l'objet tenu, '' si la pince est vide
    pince_fermee: bool = False        # une fermeture a ete commandee et pas encore annulee
    z_valide: float = None            # profondeur jusqu'a laquelle la colonne est verifiee
    garde_cible: bool = False         # un degagement de reprise : la cible ne change pas
    largages: list = field(default_factory=list)   # xy des lachers effectues
    largages_rates: list = field(default_factory=list)  # xy commandes jamais atteints
    cartons_marques: set = field(default_factory=set)  # classes identifiees par leur marqueur
    dernier_cycle: float = 0.0        # duree du dernier cycle complet, en secondes
    debut_cycle: float = 0.0
    detecteur: object = None          # callable(patience=...) -> (x, y) ou None
    source_balle: str = ''            # camera qui a fourni la derniere detection
    source_cible: str = ''            # celle qui a fourni la cible en cours
    classe_objet: str = ''            # 'balle', 'scotch' ou 'robot' — objet en cours
    carton_vise: str = 'grand'        # destination de cet objet
    detecteur_carton: object = None   # idem, pour le carton
    oublies: list = field(default_factory=list)   # [(xy, instant)] — saisies epuisees
    deposes: dict = field(default_factory=dict)  # {classe: nombre reellement largue}

    def note(self, texte):
        self.journal.append(texte)
        del self.journal[:-200]
        # ... et sur la sortie standard. Le journal ne vivait que dans la
        # fenetre Qt : apres un cycle rate, la seule trace disponible etait une
        # capture d'ecran tronquee. Redirige vers un fichier, il devient
        # relisible en entier, ce qui coute une ligne et fait gagner un cycle
        # d'essais a chaque diagnostic.
        print(texte, flush=True)

    def essai(self, etat):
        self.essais[etat] = self.essais.get(etat, 0) + 1
        return self.essais[etat]

    def oublie(self, xy):
        """Mettre de cote une position ou la saisie a echoue pour de bon."""
        if xy is None:
            return
        self.oublies.append((np.asarray(xy, float), time.time()))
        del self.oublies[:-8]

    def est_oublie(self, xy):
        """Cette position est-elle celle d'un objet deja abandonne ?"""
        maintenant = time.time()
        self.oublies = [(p, t) for p, t in self.oublies
                        if maintenant - t < DUREE_OUBLI]
        return any(float(np.linalg.norm(np.asarray(xy, float) - p)) < RAYON_OUBLI
                   for p, _ in self.oublies)

    def repart_a_zero(self):
        """Nouvelle cible : les compteurs d'essais ne valent plus.

        Le BIAIS, lui, repart de ce qui a deja converge. Il decrit la derive du
        bras, pas la cible : le remettre a zero faisait recommencer a chaque
        objet la passe « descendre, mesurer, remonter, redescendre ». S'il se
        trouve faux a la nouvelle position, la passe suivante le corrige comme
        avant — on ne perd rien, on part seulement d'un point deja mesure.
        """
        self.essais.clear()
        self.biais_descente = self.biais_appris.copy()


PINCE_OUVERTE_MIN = 90        # angle lu au-dela duquel la pince est deja ouverte
ANGLE_PINCE_FERMEE = 20       # consigne de fermeture envoyee a la saisie
# A VIDE la pince atteint exactement la consigne — mesure du 26/08, 20 tout
# rond, et 100 ouverte. Sur un objet elle CALE plus haut : 24-25 sur un rouleau
# de pres, 29-30 de loin, 52-53 sur la balle, 74 quand les doigts butent sur la
# planche. Trois degres suffisent donc a separer le vide du tenu.
MARGE_ANGLE_TENUE = 3
# La pince repond parfois n'IMPORTE QUOI pendant que le bras bouge. Mesure du
# 27/08, remontee en trois paliers, 12 lectures : deux statuts « 6 » (aucun sens
# pour cette pince, qui ne rend que 0-3) et un angle « 65535 » — le -1 du
# registre lu en 16 bits non signes. Une seule de ces reponses suffisait a
# declarer l'objet lache : c'est ainsi que le rouleau blanc est parti en vol,
# `monte_par_paliers` ayant rendu « perdu » sur un rouleau parfaitement tenu,
# apres quoi la pince s'est ouverte en l'air.
STATUTS_PINCE = (0, 1, 2, 3)
ANGLE_PINCE_MAX = 100         # pince grande ouverte ; au-dela la lecture est du bruit
LECTURES_PINCE_MAX = 3


def _temoins_pince(ctx):
    """(statut, angle) de la pince, ou None pour un temoin illisible."""
    statut = ctx.pont.statut_pince()
    angle = ctx.pont.angle_pince()
    return (statut if statut in STATUTS_PINCE else None,
            angle if 0 <= angle <= ANGLE_PINCE_MAX else None)


def porte_objet(ctx):
    """La pince tient-elle l'objet ? Statut 2, ou l'ANGLE s'il dit le contraire.

    Tant que c'est vrai, aucun retour au ramassage n'est permis. Le cycle a
    tourne des heures la balle en main parce qu'un carton introuvable renvoyait
    vers ECHEC, donc vers ATTENTE, donc vers un nouveau DEGAGEMENT : le bras
    repartait chercher une balle qu'il tenait deja.

    Le statut seul ne suffit pas : le 26/08 il a annonce « objet lache » a la
    remontee alors que le petit robot etait bel et bien dans les doigts. La
    machine est repartie chercher un rouleau EN LE TENANT, et l'a depose dans le
    petit carton sous le nom « scotch ». L'angle est un temoin independant et il
    ne s'est pas trompe. On le lit dans une BANDE, pas au-dessus d'un seuil :
    ouverte la pince lit 100, ce qui passerait tout aussi bien un simple seuil.

    Une reponse ILLISIBLE n'est pas une reponse : on relit. Et si les deux
    temoins restent illisibles au bout de trois tours, on repond « tenu ». Des
    deux erreurs possibles celle-la est la moins chere : croire tenir ce qu'on
    ne tient pas fait perdre un cycle, croire avoir lache ce qu'on tient fait
    ouvrir la pince en plein vol.
    """
    if ctx.pont is None:
        return False
    for _ in range(LECTURES_PINCE_MAX):
        statut, angle = _temoins_pince(ctx)
        if statut == 2:
            return True
        # L'angle ne temoigne que si l'on a REELLEMENT commande une fermeture.
        # Au reveil apres l'incident du 26/08 la pince lisait 73 avec le statut
        # « rien saisi » : un etat residuel, ni ouvert ni referme sur quoi que
        # ce soit, que la seule lecture de l'angle aurait pris pour une prise.
        if not ctx.pince_fermee:
            return False
        if angle is not None:
            return ANGLE_PINCE_FERMEE + MARGE_ANGLE_TENUE <= angle < PINCE_OUVERTE_MIN
    ctx.note('  pince illisible trois fois — on considere l objet TENU')
    return True


def memorise_carton(classe, xy, roulis_appris=None):
    """Retient le ROULIS appris — jamais la position des cartons.

    Les positions ne sont volontairement PLUS persistees. Les boites se
    deplacent entre deux seances, et une position ecrite sur disque survit a ce
    deplacement : le 26/08 « grand » valait encore (428, -97), du cote du petit
    et a 320 mm du vrai grand, et la balle est allee s'y poser — sur la planche,
    entre les deux boites. Une position fausse et durable coute plus cher que
    tout ce qu'elle fait gagner.

    Pendant une seance les boites ne bougent pas, et le suivi camera
    (`SuiviCarton`) tient leur position en memoire vive : c'est la bonne portee
    pour cette information. Le roulis appris, lui, est une propriete du BRAS et
    non de la scene — il reste persiste, et fait gagner 10 s au premier cycle.
    """
    try:
        d = json.loads(MEMOIRE_CARTON.read_text())
    except (OSError, json.JSONDecodeError):
        d = {}
    MEMOIRE_CARTON.write_text(json.dumps(
        {'roulis_appris': roulis_appris or d.get('roulis_appris') or {}}))


def memorise_affaissement(correction):
    """Retient l'ecart articulaire qui compense l'affaissement.

    Il est reproductible (~+1,9 deg sur J2) : le redecouvrir a chaque cycle
    coute deux passes de convergence, soit deux mouvements. Mesure du 24/08 :
    passe 1 a 10,79 mm, passe 2 a 4,15, passe 3 a 0,38 — les deux premieres ne
    font que retrouver ce qu'on savait deja.
    """
    borne = np.clip(np.asarray(correction, float), -AFFAISSEMENT_MAX, AFFAISSEMENT_MAX)
    MEMOIRE_AFFAISSEMENT.write_text(json.dumps({'correction_deg': borne.tolist()}))


def affaissement_memorise():
    if not MEMOIRE_AFFAISSEMENT.exists():
        return np.zeros(6)
    try:
        v = np.asarray(json.loads(MEMOIRE_AFFAISSEMENT.read_text())['correction_deg'], float)
    except (json.JSONDecodeError, KeyError, TypeError, ValueError):
        return np.zeros(6)
    return np.clip(v, -AFFAISSEMENT_MAX, AFFAISSEMENT_MAX) if v.size == 6 else np.zeros(6)


def carton_memorise(ctx=None, classe=None):
    """Recharge le ROULIS appris. Ne rend JAMAIS de position de carton.

    Les positions ne sont plus persistees (voir `memorise_carton`) : les boites
    se deplacent entre deux seances et une position ecrite sur disque survit a
    ce deplacement. Le 26/08 « grand » valait encore (428, -97), du cote du
    petit, a 320 mm du vrai grand — la balle est allee s'y poser. Pendant une
    seance les boites ne bougent pas et le suivi camera tient leur position en
    memoire vive : c'est la bonne portee pour cette information.

    Le roulis appris, lui, est une propriete du BRAS et non de la scene. Sans
    lui, le premier cycle d'une seance repaie le choix complet (14,5 s contre
    4,1 s ensuite).
    """
    if not MEMOIRE_CARTON.exists():
        return None
    try:
        d = json.loads(MEMOIRE_CARTON.read_text())
    except (OSError, json.JSONDecodeError):
        return None
    if ctx is not None:
        for cible, angle in (d.get('roulis_appris') or {}).items():
            ctx.roulis_appris.setdefault(cible, angle)
    return None


def points_de_largage(xy, polygone, pas=6.0):
    """Points de largage possibles, du PLUS PROCHE DU MILIEU au plus excentre.

    On vise le milieu du carton. Quand le milieu est hors d'atteinte, on ne
    renonce pas au carton pour autant : tout point de l'ouverture depose la
    balle dedans, et on prend le plus central que le bras sache atteindre.
    Mesure du 24/08 : centre a 363 mm, inatteignable a TOUTES les hauteurs de
    largage essayees (100 a 160 mm — la limite est horizontale, pas verticale),
    alors que le bord proche de la meme ouverture est a 290 mm.

    Les candidats reculent des parois pour que la balle ne rebondisse pas sur un
    rebord. Si l'ouverture ne peut pas offrir cette marge, on garde son point le
    plus interieur.
    """
    xy = np.asarray(xy, float)
    if polygone is None or len(polygone) < 3:
        return [xy]
    contour = np.asarray(polygone, np.float32).reshape(-1, 1, 2)
    grille = [np.array([x, y], float)
              for x in np.arange(polygone[:, 0].min(), polygone[:, 0].max() + pas, pas)
              for y in np.arange(polygone[:, 1].min(), polygone[:, 1].max() + pas, pas)]
    marges = [(cv2.pointPolygonTest(contour, (float(p[0]), float(p[1])), True), p)
              for p in grille]
    # Exiger la marge ideale ne laisse que le centre geometrique — justement le
    # point le plus lointain, et sur ce carton le seul hors d'atteinte. On ne
    # demande donc jamais plus que les trois quarts de ce que l'ouverture offre.
    marge_max = max(marge for marge, _ in marges)
    exigee = max(MARGE_LARGAGE_MIN, min(MARGE_LARGAGE, 0.75 * marge_max))
    dedans = [p for marge, p in marges if marge >= exigee]
    if not dedans:
        return [max(marges, key=lambda t: t[0])[1]]
    # VISER LE CENTRE tant qu'il est confortable, LE BORD PROCHE au-dela. Au
    # centre l'objet tombe au milieu de l'ouverture, c'est le meilleur point —
    # mais seulement si le bras y va. Le 26/08 le grand carton etait a 450 mm :
    # l'IK resolvait son centre, le bras s'immobilisait 256 mm avant, et la pince
    # s'ouvrait au-dessus de la planche. Les candidats gardent tous `exigee` de
    # recul des parois, donc le bord proche depose DEDANS lui aussi : quand la
    # boite est loin, le prendre d'emblee epargne un aller-retour d'echec.
    if float(np.hypot(*xy)) > PORTEE_LARGAGE_CONFORT:
        dedans.sort(key=lambda p: float(np.hypot(*p)))
    else:
        dedans.sort(key=lambda p: float(np.linalg.norm(p - xy)))
    # Deux candidats voisins echouent ou reussissent ensemble : les espacer.
    choisis = []
    for p in dedans:
        if all(np.linalg.norm(p - q) > 20.0 for q in choisis):
            choisis.append(p)
        if len(choisis) == LARGAGES_TESTES:
            break
    # Dernier recours : le point de l'ouverture le PLUS PROCHE du robot. Les
    # candidats ci-dessus partent du milieu ; quand le carton est pose loin ils
    # sont tous hors d'atteinte alors que son bord proche, lui, se resout. Un
    # largage excentre depose quand meme la balle dedans — c'est le but.
    bord = min(dedans, key=lambda p: float(np.hypot(*p)))
    if all(np.linalg.norm(bord - q) > 20.0 for q in choisis):
        choisis.append(bord)
    return choisis


def choisit_pose_largage(ctx, xy):
    """(roulis, inclinaison, R) pour larguer en xy, ou (None, None, None).

    Meme methode que la prise, appliquee au depot : l'outil tenu vertical
    plafonne, couche il porte beaucoup plus loin. Sans ca un carton pose a
    449 mm bloquait TOUT le cycle — le bras refusait meme de saisir la balle,
    faute de savoir ou la deposer (journal du 24/08 : quatre points de largage
    a 425-473 mm, tous refuses, `carton ni vu ni memorise a portee`).
    """
    xy = np.asarray(xy, float)
    portee = float(np.hypot(*xy))
    azimut = float(np.degrees(np.arctan2(xy[1], xy[0])))
    hauteurs = [Z_TRANSFERT, z_largage_commande(ctx, xy)]
    depart = next(t for limite, t in INCLINAISONS_CARTON_PAR_PORTEE if portee <= limite)
    if depart == 0.0:
        roulis, R = choisit_roulis(xy, hauteurs,
                                   prefere=roulis_retenu(ctx, 'carton', xy))
        if R is not None:
            return roulis, 0.0, R
    memo = ctx.prise_apprise.get(cle_roulis('carton', xy))
    inclinaisons = [t for t in INCLINAISONS_CARTON if t <= depart and t != 0.0]
    # La memoire reordonnait les roulis mais pas les inclinaisons : on repartait
    # donc du haut de la liste et on re-parcourait tout ce qui avait deja echoue.
    # Mesure du 26/08 : resoudre un carton neuf coute 1,81 s, soit 64 a 96 appels
    # d'IK a 13 ms. Remettre l'inclinaison retenue en tete ramene une resolution
    # deja connue a deux appels.
    if memo is not None and memo[0] in inclinaisons:
        inclinaisons = [memo[0]] + [t for t in inclinaisons if t != memo[0]]
    for theta in inclinaisons:
        ordre = ROULIS
        if memo is not None and memo[0] == theta:
            ordre = [memo[1]] + [r for r in ROULIS if r != memo[1]]
        for roulis in ordre:
            R = incline(orientation(xy, roulis), azimut, theta)
            if all(resout_ik(np.array([xy[0], xy[1], z]), R) is not None
                   for z in hauteurs):
                ctx.prise_apprise[cle_roulis('carton', xy)] = (theta, roulis)
                return roulis, theta, R
    return None, None, None


def z_largage(ctx):
    """Hauteur de lacher VOULUE : au-dessus du rebord REEL, pas d'un suppose.

    Le rebord du grand carton a ete mesure a 82,9 mm par triangulation des deux
    vues le 25/08, contre 60 mm supposes — soit 17 mm de garde au lieu de 40,
    et rien du tout pour un carton plus haut. La hauteur vient desormais du
    marqueur colle sur le rabat, quand il est vu.

    C'est la hauteur VOULUE, pas celle a commander : voir `z_largage_commande`.
    """
    rebord = getattr(ctx, 'z_rebord', None)
    return Z_LARGAGE if rebord is None else max(Z_LARGAGE, rebord + GARDE_LARGAGE)


def affaissement_largage(portee):
    """De combien la pointe arrive SOUS la consigne, en pose de largage (mm).

    Mesure du 27/08 sur le vrai bras, azimut +27, consigne Z=108 :

        portee (mm) | 340   380   410   430   450   470
        manque (mm) | 11,9  14,8  19,7  19,1  20,4  25,3

    Droite de regression, residus <= 1,6 mm. Ce n'est pas une erreur de modele :
    `send_angles` n'atteint pas sa consigne sous le couple de gravite, et la
    cinematique directe des angles LUS le dit fidelement. Le meme phenomene vaut
    `BIAIS_Z_PRISE` a la prise, ou `converge` le rattrape passe par passe — au
    largage il n'y a pas de convergence, la vitesse compte.

    Enjeu : a 470 mm la pointe arrivait a 82,7 mm pour un rebord mesure a
    82,9 — la garde de 25 mm entierement mangee, l'objet lache AU RAS du bord.
    """
    return max(0.0, PENTE_AFFAISSEMENT_LARGAGE * float(portee)
               + ORIGINE_AFFAISSEMENT_LARGAGE)


def z_largage_commande(ctx, xy):
    """Hauteur a COMMANDER pour que la pointe arrive a `z_largage`.

    Verifie le 27/08 a l'azimut -18, c'est-a-dire hors des points ayant servi a
    la regression : consignes 119,6 / 125,3 / 130,0 mm a 340 / 400 / 450 mm de
    portee, pointe atteinte a 108,0 / 107,1 / 109,0 pour 108 voulus.
    """
    return z_largage(ctx) + affaissement_largage(np.hypot(*np.asarray(xy, float)))


def carton_atteignable(ctx, xy, polygone=None):
    """(point de largage, roulis, R) resolvant Z_TRANSFERT PUIS Z_LARGAGE.

    Le point rendu n'est pas forcement celui demande : c'est le point de
    l'ouverture, le plus proche du robot, que l'IK sait atteindre.
    """
    if ctx.carton_resolu is not None:
        cle, cible, roulis, R = ctx.carton_resolu
        if float(np.linalg.norm(np.asarray(xy, float) - cle)) <= TOLERANCE_CARTON_RESOLU:
            return cible, roulis, R
    for cible in points_de_largage(xy, polygone):
        portee = float(np.hypot(*cible))
        # Un point que le bras n'a PAS SU ATTEINDRE ne se represente pas : l'IK
        # le resout (c'est bien pour ca qu'il avait ete retenu), seul le robot
        # sait qu'il n'y va pas. Sans cette memoire, ECHEC_PORTANT reproposait
        # le meme point indefiniment. Ecarte, il laisse la place au candidat
        # suivant — le bord PROCHE de la meme ouverture, qui lui se rejoint.
        if any(float(np.linalg.norm(cible - rate)) < 20.0
               for rate in ctx.largages_rates):
            continue
        if portee > PORTEE_CARTON_MAX:
            ctx.note(f'point de largage a {portee:.0f} mm — au-dela de '
                     f'{PORTEE_CARTON_MAX:.0f} mm aucun roulis ne resout')
            continue
        roulis, theta, R = choisit_pose_largage(ctx, cible)
        if R is not None:
            ctx.roulis_appris[cle_roulis('carton', cible)] = roulis
            ctx.resultats['inclinaison largage'] = (
                'verticale' if theta == 0.0 else f'{theta:+.0f} deg')
            ecart = float(np.linalg.norm(cible - np.asarray(xy, float)))
            if ecart > 1.0:
                ctx.note(f'milieu du carton a {np.hypot(*xy):.0f} mm inatteignable — '
                         f'largage a {ecart:.0f} mm du milieu, en '
                         f'({cible[0]:.0f}, {cible[1]:.0f}) a {portee:.0f} mm')
            ctx.carton_resolu = (np.asarray(xy, float), cible, roulis, R)
            return cible, roulis, R
    return None, None, None


def detecte_carton(ctx, patience=1.5):
    """Cherche le carton et le retient s'il est atteignable.

    Rend 'vu', 'hors atteinte' ou 'invisible' — les trois appellent des suites
    differentes : se replacer, demander de rapprocher le carton, ou balayer.
    """
    if ctx.detecteur_carton is None:
        return 'invisible'
    vu = ctx.detecteur_carton(patience=patience)
    if vu is None:
        return 'invisible'
    xy, polygone = np.asarray(vu[0], float), vu[1]
    ctx.z_rebord = vu[2] if len(vu) > 2 else None
    # Un carton identifie par son MARQUEUR echappe a ce garde-fou : une ombre
    # n'a pas de code ArUco. La regle a ete ecrite quand les cartons etaient
    # reconnus a la couleur, ou l'ombre du bras passait effectivement pour une
    # ouverture. Depuis, elle jetait la bonne detection des que le bras — qui
    # vient justement de saisir au-dessus de la planche — passait a moins de
    # 150 mm, et la machine partait balayer six poses J1 pour retrouver un
    # carton qu'elle avait sous les yeux.
    ecart_bras = distance_au_bras(xy, ctx.pont.angles())
    if ecart_bras < RAYON_MASQUAGE and ctx.carton_vise not in ctx.cartons_marques:
        ctx.note(f'carton "vu" a {ecart_bras:.0f} mm du bras, sans marqueur pour '
                 f'le confirmer — le bras ou son ombre, detection ignoree')
        return 'invisible'
    cible, roulis, R = carton_atteignable(ctx, xy, polygone)
    if R is None:
        ctx.resultats['carton'] = (f'({xy[0]:.0f}, {xy[1]:.0f}) mm — HORS ATTEINTE '
                                   f'({np.hypot(*xy):.0f} mm)')
        return 'hors atteinte'
    ctx.carton_xy, ctx.carton_polygone = cible, polygone
    ctx.roulis_carton, ctx.R_carton = roulis, R
    ctx.resultats['carton'] = (f'({cible[0]:.1f}, {cible[1]:.1f}) mm — vu, '
                               f'{np.hypot(*cible):.0f} mm')
    memorise_carton(ctx.carton_vise, cible, ctx.roulis_appris)
    return 'vu'


def carton_pret(ctx):
    """Le carton est-il localise ET atteignable ? A juger AVANT de saisir.

    Le bras s'est retrouve a tourner la balle en main parce que la depose
    n'etait jugee qu'apres la prise. Un cycle ne commence donc que si la
    depose est possible : vue directe, ou derniere position connue.
    """
    if detecte_carton(ctx, patience=1.0) == 'vu':
        return True
    memoire = carton_memorise(ctx)
    if memoire is None:
        return False
    ctx.z_rebord = None                   # la memoire ne retient pas la hauteur
    cible, roulis, R = carton_atteignable(ctx, memoire)
    if R is None:
        return False
    ctx.carton_xy, ctx.carton_polygone = cible, None
    ctx.roulis_carton, ctx.R_carton = roulis, R
    ctx.resultats['carton'] = f'({cible[0]:.1f}, {cible[1]:.1f}) mm — memoire'
    return True


def va_vers(ctx, q_cible, vitesse=VITESSE, nom='', patience=9.0, stabilise=True,
            enchaine=False):
    """Deplacement valide. Trois controles, tous nes d'un incident reel.

    Un seuil de garde ABSOLU ne convient pas : la pointe doit atteindre
    `Z_PRISE`, bien plus bas que `GARDE_MIN`, sinon aucune saisie ne passe
    jamais. Ce qu'il faut interdire n'est pas d'etre bas, c'est de tomber :

    * `PLANCHER` — sous la planche, aucune pose n'est legitime ;
    * `CHUTE_MAX` — le plongeon de 590 mm du 20/08 venait d'une pose lue
      perimee ; une descente utile fait ~115 mm, au-dela c'est une erreur de
      lecture, pas une intention ;
    * pas de creux en chemin sous le plus bas des deux bouts.
    """
    q0 = ctx.pont.angles()
    # Le bras peut etre ENCORE EN ROUTE vers la consigne precedente : dans ce cas
    # le chemin reel n'est pas le segment depuis la position lue, mais l'enveloppe
    # entre cette position, la consigne encore active et la nouvelle cible. On
    # verifie les deux, sinon l'enchainement ferait passer un creux inapercu.
    origines = [q0]
    if (ctx.derniere_consigne is not None
            and float(np.abs(ctx.derniere_consigne - q0).max()) > 0.5):
        origines.append(np.asarray(ctx.derniere_consigne, float))
    depart, arrivee = min(garde_au_sol(o) for o in origines), garde_au_sol(q_cible)
    if arrivee < PLANCHER:
        ctx.note(f'{nom} REFUSE — cible a {arrivee:.1f} mm, sous le plancher {PLANCHER:.0f}')
        return None
    # La chute se mesure sur la POINTE, pas sur la garde : la garde est le point
    # le plus bas de tout le bras, et un coude reste bas meme bras dresse — elle
    # ne bouge quasiment pas quand la pointe plonge de 590 mm.
    # Reference de la chute : la CONSIGNE encore active quand le bras est en vol,
    # sa position lue sinon. Le garde-fou vise une pose lue PERIMEE (le plongeon
    # de 590 mm du 20/08) ; une consigne, elle, est ecrite par nous et ne peut pas
    # l'etre. Mesurer depuis la position lue pendant un enchainement refusait le
    # trajet entier : le bras a 0,3 s de retard, et l'ecart au palier suivant
    # passait pour un plongeon.
    reference = origines[-1]
    chute = float(pointe(reference)[2] - pointe(q_cible)[2])
    if chute > CHUTE_MAX:
        ctx.note(f'{nom} REFUSE — la pointe plongerait de {chute:.0f} mm en un seul ordre')
        return None
    seuil = min(GARDE_MIN, depart - 2.0, arrivee - 2.0)
    minimum = min(garde_au_sol(o * (1 - t) + q_cible * t)
                  for o in origines for t in np.linspace(0, 1, 61))
    if minimum < seuil:
        ctx.note(f'{nom} REFUSE — creux a {minimum:.1f} mm sous le seuil {seuil:.1f}')
        return None
    # LE BRAS CONTRE LUI-MEME, sur tout le trajet et pas seulement a l'arrivee.
    # 0,22 ms le test, 17 sur le chemin : le cout est nul devant un mouvement.
    # Verifie le 27/08 sur 120 poses de travail (180 a 460 mm, trois azimuts,
    # trois hauteurs, trois roulis) : AUCUNE refusee, marge la plus faible
    # +2,2 mm. Le garde-fou ne coute donc rien au travail normal.
    for o in origines:
        for t in np.linspace(0, 1, 17):
            marge, paire = marge_collision(o * (1 - t) + q_cible * t)
            if marge < 0.0:
                ctx.note(f'{nom} REFUSE — {paire} a {marge:.1f} mm : le bras se '
                         f'toucherait lui-meme')
                return None
    ctx.pont.envoie('send_angles', angles=[round(float(v), 2) for v in q_cible],
                    speed=vitesse)
    ctx.derniere_consigne = np.asarray(q_cible, float)
    # TRANSIT : on rend la main des que le mouvement est ENGAGE, sans attendre
    # l'arret. Mesure du 26/08 : un ordre demarre en 0,30 s et le bras s'immobilise
    # en 1,2 a 1,9 s — identique a vitesse 25 et a vitesse 80, donc le temps ne
    # vient pas du robot. Attendre l'arret complet puis dormir une seconde de plus
    # coutait ~2 s par ordre, et un cycle en compte une douzaine : c'est la, et
    # nulle part ailleurs, que naissait le rythme saccade « une etape, une pause ».
    # Le bras enchaine desormais les transits en un seul geste continu. On ne
    # l'utilise QUE pour les deplacements dont personne ne mesure la pose
    # d'arrivee : toute etape qui sert de reference (recalage, descente, saisie)
    # attend toujours l'immobilite.
    if enchaine:
        debut = time.time()
        while time.time() - debut < DEMARRAGE_MAX:
            time.sleep(0.05)
            if float(np.abs(ctx.pont.angles() - q0).max()) > 0.5:
                break
        return ctx.pont.angles()
    # Arrivee = le bras ne bouge PLUS, et non le bras qui atteint sa consigne :
    # l'affaissement laisse un ecart permanent d'environ 1,9 deg sur J2, donc un
    # test sur la consigne ne serait JAMAIS satisfait.
    #
    # L'immobilite se mesure sur plusieurs lectures d'affilee, pas sur deux. Deux
    # lectures espacees de 0,15 s declarent l'arret des que le bras avance a moins
    # de 1,3 deg/s — c'est-a-dire en pleine deceleration. Mesure du 26/08 : la
    # main etait rendue vers 0,75 s alors que le bras s'immobilise entre 0,87 et
    # 1,32 s, et c'est la seconde de `stabilise` qui masquait l'ecart. On lisait
    # donc parfois la pose d'un bras encore en mouvement.
    if float(np.abs(q_cible - q0).max()) > 0.05:
        debut, calmes, precedent = time.time(), 0, q0
        while time.time() - debut < patience:
            q = ctx.pont.angles()                     # 30 ms d'aller-retour
            calmes = calmes + 1 if float(np.abs(q - precedent).max()) < 0.05 else 0
            precedent = q
            # On n'exige PAS d'avoir vu le bras partir : une passe de convergence
            # le deplace de moins d'un degre et le depart n'est jamais constate —
            # RECALAGE montait alors a 37 s pour quatre passes. Le delai minimal
            # couvre l'engagement de l'ordre, mesure a 0,30 s.
            if calmes >= LECTURES_CALMES and time.time() - debut > DEMARRAGE_MAX:
                break
    if stabilise:
        # Ce qui reste a attendre une fois le bras VRAIMENT arrete. Mesure du
        # 26/08 sur cinq mouvements (J1, J2, J3) : la pose derive encore de 0,09
        # deg au pire a l'instant de l'arret, et de 0,000 deg des +0,3 s. La
        # seconde d'origine n'attendait donc pas l'affaissement, elle rattrapait
        # une detection d'arret trop hative.
        time.sleep(REPOS_MESURE)
    return ctx.pont.angles()


# Progres RELATIF exige d'une passe a l'autre. En absolu (1 mm) le garde-fou
# coupait des convergences qui marchaient : 4,43 -> 3,48 mm, soit 0,95 mm, est un
# gain de 21 % et une passe de plus passait sous la tolerance. L'emballement, lui,
# se reconnait a ce qu'il ne gagne presque RIEN sur un ecart enorme : 34,25 ->
# 33,62 -> 33,48 -> 33,22, donc 1,8 %, 0,4 % puis 0,8 %. Les deux familles sont a
# un ordre de grandeur l'une de l'autre ; 5 % passe entre elles.
PROGRES_CONVERGENCE = 0.05

# Le test de progres ci-dessus n'a de sens que LOIN de la cible. Sous ce seuil on
# est au plancher de bruit du servo (ecart-type mesure le 08/09 : 0,25 mm en X,
# 0,40 en Y, 0,60 en Z) et exiger 5 % de progres y demande moins que sa propre
# repetabilite : la condition ne peut plus etre satisfaite, la boucle declare la
# pose « hors d'atteinte » et rend la main EN GARDANT LE PIRE de ses passages.
# Mesure du 08/09, 9 convergences sur trois portees : 4 abandons, dont
# 4,53 -> 1,03 -> 0,89 -> 1,56 qui rendait 1,56 apres etre passe par 0,89, et
# 0,92 -> 0,93 tue au deuxieme passage pour 0,01 mm de recul. En laissant la
# boucle finir : 0,310 mm au lieu de 0,816 de residu de convergence.
# NE PAS comparer ce chiffre aux +-0,5 mm du constructeur : cette valeur-la est
# une REPETABILITE (repeated positioning precision), c'est-a-dire la dispersion
# du bras revenant plusieurs fois au meme point. Ce qu'on mesure ici est un
# ecart a une cible ABSOLUE, ||FK(q_lu) - P_cible||, qui agrege la lecture
# articulaire, le modele FK, les offsets, la definition du TCP, le settling des
# servos et les changements de repere. Deux metriques differentes : la seconde
# ne peut ni valider ni invalider la premiere.
# L'abandon coutait aussi un essai : `_recalage` repart en APPROCHE sur ok=False.
SEUIL_GARDE_PROGRES = 3.0


def converge(ctx, p_cible, R, passes=4, tol=2.5):
    """Compense l'affaissement en reinjectant l'ecart articulaire mesure.

    `send_angles` n'atteint pas la consigne : ~+1.9 deg sur J2, ce qui deplace la
    pointe ET fait pivoter l'outil. Rend (q, correction, converge).

    `tol` vaut le seuil qui compte reellement en aval (3 mm) et non 1 mm : sur
    une balle de 66 mm, la 3e passe gagnait 1,7 mm pour 3 s (journal du 24/08,
    11,62 -> 2,09 -> 0,34 mm). La convergence reste declaree a `< 3.0`.
    """
    q_mesure = ctx.pont.angles()
    # Depart CHAUD : on repart de l'affaissement deja mesure au lieu de le
    # redecouvrir. Sans lui, la premiere passe commande la solution IK brute et
    # arrive 10 mm trop bas — deux passes pour rien, a chaque cycle.
    q_commande = q_mesure + affaissement_memorise()
    ecart, precedent = float('inf'), float('inf')
    for i in range(passes):
        cible = resout_ik(p_cible, R)
        if cible is None:
            ctx.note('cible devenue insoluble pendant la convergence')
            return q_mesure, np.zeros(6), False
        q_commande = np.clip(q_commande + (cible[0] - q_mesure),
                             LIMITES[:, 0], LIMITES[:, 1])
        suivant = va_vers(ctx, q_commande, nom=f'convergence {i + 1}')
        if suivant is None:
            return q_mesure, np.zeros(6), False
        q_mesure = suivant
        ecart = float(np.linalg.norm(pointe(q_mesure) - p_cible))
        ctx.note(f'  convergence passe {i + 1} : ecart {ecart:.2f} mm')
        if ecart < tol:
            break
        # ARRET SEC des que l'ecart cesse de se refermer. Cette boucle REINJECTE
        # l'ecart dans la consigne a chaque passe ; sur une pose que le bras ne
        # peut pas atteindre, il ne se referme jamais et la consigne s'emballe,
        # poussant les servos dans la butee un peu plus fort a chaque tour.
        # Mesure du 26/08, cible a 151 mm (blob du robot fusionne avec un cable,
        # centroide tire pres de la base) : 12 passes, ecart jamais sous 33 mm et
        # CROISSANT d'un essai a l'autre (34,2 -> 36,0 -> 36,8), jusqu'a ce que
        # le pont de la Pi cesse de repondre. Une consigne qui ne rapproche pas
        # de la cible ne doit pas etre renvoyee plus fort.
        if (ecart > SEUIL_GARDE_PROGRES
                and ecart > precedent * (1.0 - PROGRES_CONVERGENCE)):
            ctx.note(f'  convergence sans progres ({precedent:.1f} -> {ecart:.1f} mm, '
                     f'{100.0 * (precedent - ecart) / max(precedent, 1e-6):.0f} %) '
                     f'— pose hors d atteinte, on cesse de reinjecter')
            return q_mesure, np.zeros(6), False
        precedent = ecart
    correction = q_commande - q_mesure
    if ecart < 3.0:
        memorise_affaissement(correction)
    return q_mesure, correction, ecart < 3.0


def descend_par_paliers(ctx, p_cible, R, correction, nom='descente'):
    """Descend en marches courtes, chacune amorcee sur la pose COURANTE.

    `monte_par_paliers` existait deja pour la montee, et pour cette raison
    exacte : une solution IK cherchee globalement peut etre articulairement
    loin de la pose actuelle, et l'interpolation du robot passe alors SOUS la
    planche. La descente, elle, sautait encore d'un seul ordre de Z=110 au ras
    de la table — 107 mm en une fois, avec une orientation qui change en route.
    Les doigts raclaient. Le garde-fou de `va_vers` ne le voyait pas : il
    mesure la pointe de l'outil et les origines de liens, pas le bout des
    doigts, qui descend plus bas des qu'on couche l'outil.
    """
    q_mesure = ctx.pont.angles()
    depart = float(pointe(q_mesure)[2])
    marches = (1 if depart <= p_cible[2]
               else max(1, int(np.ceil((depart - p_cible[2]) / PAS_PALIER_DESCENTE))))
    # La chaine se poursuit sur le PLAN, pas sur la mesure : les paliers
    # intermediaires sont enchaines, et `va_vers` rend alors la pose du DEBUT du
    # mouvement. Amorcer le palier suivant dessus faisait voir un saut de 27 deg
    # a chaque descente — le retard de l'enchainement, pas un changement de
    # branche — et le garde-fou refusait tout, y compris a 177 mm d'allonge.
    q_plan = q_mesure
    # CORRECTION EN VOL. La descente est purement VERTICALE : d'un palier a
    # l'autre seul Z change, la cible XY ne bouge pas. L'ecart XY lu pendant la
    # descente est donc la derive laterale elle-meme, sans penalite de retard —
    # le retard de l'enchainement ne porte que sur Z. On la corrige palier par
    # palier au lieu de remonter a Z_SURVOL pour recommencer : le bras ne
    # s'arrete jamais, et il arrive en bas deja recale.
    derive = np.asarray(ctx.biais_descente[:2], float).copy()
    for i in range(1, marches + 1):
        z = depart + (p_cible[2] - depart) * i / marches
        cible = np.array([p_cible[0] + derive[0], p_cible[1] + derive[1], z])
        q_cible = solve_pose(q_plan, cible - R @ TOOL, R, rot_weight=400.0,
                             max_joint_step_deg=3.0, iterations=300)
        if float(np.linalg.norm(pointe(q_cible) - cible)) > 2.0:
            sol = resout_ik(cible, R)
            if sol is None:
                ctx.note(f'  palier Z={z:.0f} sans solution — descente interrompue')
                return None
            q_cible = sol[0]
        saut = float(np.abs(q_cible - q_plan).max())
        if saut > SAUT_PALIER_MAX:
            ctx.note(f'  palier Z={z:.0f} exige {saut:.0f} deg — changement de branche refuse')
            return None
        dernier = i == marches
        q_mesure = va_vers(ctx, np.clip(q_cible + correction, LIMITES[:, 0], LIMITES[:, 1]),
                           nom=f'{nom} Z={z:.0f}', vitesse=VITESSE_DESCENTE,
                           stabilise=dernier, enchaine=not dernier)
        if q_mesure is None:
            return None
        q_plan = q_cible
        reste = p_cible[:2] - pointe(q_mesure)[:2]
        derive = np.clip(derive + reste, -DERIVE_DESCENTE_MAX, DERIVE_DESCENTE_MAX)
    ctx.biais_descente[:2] = derive
    return q_mesure


# Ecart lateral en dessous duquel la descente est jugee BONNE — donc digne
# d'etre gardee comme point de depart pour l'objet suivant. Ce n'est plus un
# veto : on saisit de toute facon, c'est la pince qui tranche. La balle fait
# 66 mm de diametre, le ventre du petit robot 41 mm de large ; le rouleau reste
# le plus exigeant, il se prend par sa moitie et pas par son quart.
TOL_XY_PRISE = {'balle': 6.0, 'robot': 4.0, 'scotch': 3.0}
TOL_XY_PRISE_DEFAUT = 2.5


def descente_verticale(ctx, p_cible, R, correction, tol_xy=None):
    """UNE descente, d'un trait, en se recalant sans jamais s'arreter.

    L'ancienne version descendait, mesurait en bas, REMONTAIT a Z_SURVOL et
    redescendait — jusqu'a trois fois. C'est ce va-et-vient qu'on voyait sur le
    robot, et il coutait une descente entiere par objet parce que le biais
    repartait de zero a chaque cible.

    Ce qui l'a remplace : la descente est verticale, donc l'ecart XY lu en cours
    de route EST la derive laterale, sans retard — le retard de l'enchainement
    ne porte que sur Z. `descend_par_paliers` la corrige palier par palier, et
    on arrive en bas deja recale.

    Et on saisit. Un ecart residuel n'annule plus la prise : la pince a de la
    largeur, l'objet a un volume, et c'est la PINCE qui dit si c'est pris. Si
    elle se referme sur du vide, l'ecart mesure ici est deja dans
    `ctx.biais_descente` — l'essai suivant part de la, apres avoir ecarte le
    bras pour revoir l'objet.

    Rend (pose mesuree, descente MENEE A BIEN). Le second terme ne juge pas la
    precision : il ne vaut False que si un palier a ete refuse, c'est-a-dire si
    le bras n'est pas descendu.
    """
    if tol_xy is None:
        tol_xy = TOL_XY_PRISE.get(ctx.classe_objet, TOL_XY_PRISE_DEFAUT)
    q_mesure = descend_par_paliers(ctx, p_cible, R, correction, nom='descente')
    if q_mesure is None:
        return None, False
    ecart_xy = float(np.linalg.norm(p_cible[:2] - pointe(q_mesure)[:2]))
    ctx.resultats['descente XY'] = f'{ecart_xy:.2f} mm'
    ctx.note(f'  descente d un trait : ecart XY {ecart_xy:.2f} mm '
             f'(tolere {tol_xy:.1f}) — on saisit')
    if ecart_xy < tol_xy:
        # Ce qui a marche sert de point de depart au prochain objet : la derive
        # decrit le bras, pas la cible.
        ctx.biais_appris = ctx.biais_descente.copy()
    return q_mesure, True


def va_vers_par_etapes(ctx, q_cible, nom='', vitesse=VITESSE, stabilise=True):
    """Rejoint une pose lointaine en plusieurs ordres, chacun sous CHUTE_MAX.

    `va_vers` refuse un plongeon de plus de CHUTE_MAX en un seul ordre — garde
    nee du plongeon de 590 mm du 20/08, et qui reste. Mais le bras au repos est
    dresse, pointe a ~518 mm : rejoindre la hauteur de travail est alors une
    descente legitime de plus de 400 mm, que le garde-fou refusait, d'ou un
    cycle qui bouclait sur ATTENTE -> DEGAGEMENT -> DETECTION -> refus.

    Le decoupage se fait dans l'espace ARTICULAIRE, pas en hauteurs successives
    au-dessus de la cible : au-dessus de la balle, a Z=340, l'outil ne peut pas
    etre tenu vertical, l'IK n'a aucune solution. Les etapes interpolees, elles,
    sont atteignables par construction — c'est le chemin que le robot suivrait
    de toute facon, simplement verifie et commande par morceaux.
    """
    q0 = ctx.pont.angles()
    # Le nombre d'etapes ne se deduit PAS de la chute totale : l'interpolation
    # articulaire n'est pas monotone en hauteur — mesure sur une approche depuis
    # le bras dresse, la pointe monte d'abord a 562 mm avant de plonger. On
    # augmente donc le decoupage jusqu'a ce que CHAQUE etape passe sous la
    # limite, en regardant le profil reel.
    etapes = 1
    while etapes < ETAPES_MAX:
        hauteurs = [float(pointe(q0 + (q_cible - q0) * (i / etapes))[2])
                    for i in range(etapes + 1)]
        if max(a - b for a, b in zip(hauteurs, hauteurs[1:])) <= CHUTE_MAX - 20.0:
            break
        etapes += 1
    else:
        ctx.note(f'{nom} REFUSE — aucun decoupage en {ETAPES_MAX} etapes ne tient '
                 f'sous la chute maximale')
        return None
    q = None
    for i in range(1, etapes + 1):
        q = va_vers(ctx, q0 + (q_cible - q0) * (i / etapes), vitesse=vitesse,
                    nom=nom if etapes == 1 else f'{nom} etape {i}/{etapes}',
                    stabilise=stabilise and i == etapes,
                    enchaine=i < etapes)
        if q is None:
            return None
    return q


def monte_par_paliers(ctx, R, hauteurs=(50.0, 110.0, Z_TRANSFERT)):
    """Remontee verticale amorcee sur la pose courante, palier par palier.

    Une solution IK choisie loin de la configuration actuelle fait plonger
    l'interpolation articulaire sous la planche.
    """
    q = ctx.pont.angles()
    tp = pointe(q)
    for z in hauteurs:
        cible = np.array([tp[0], tp[1], z])
        q_cible = solve_pose(q, cible - R @ TOOL, R, rot_weight=400.0,
                             max_joint_step_deg=3.0, iterations=300)
        p, Rq = pose_bride(q_cible)
        if np.linalg.norm(p + Rq @ TOOL - cible) > 2.0:
            ctx.note(f'  palier Z={z:.0f} sans solution proche, arret de la montee')
            break
        dernier = z == hauteurs[-1]
        suivant = va_vers(ctx, q_cible, nom=f'palier Z={z:.0f}',
                          stabilise=dernier, enchaine=not dernier)
        if suivant is None:
            break
        q = suivant
        if not porte_objet(ctx):
            return q, False
    return q, True


# --------------------------------------------------------------------------- #
#  Les etats
# --------------------------------------------------------------------------- #

ETATS = ['ATTENTE', 'DEGAGEMENT', 'DETECTION', 'APPROCHE', 'RECALAGE', 'DESCENTE',
         'SAISIE', 'REMONTEE', 'RECHERCHE_CARTON', 'TRANSFERT', 'LARGAGE', 'RETRAIT',
         'ECHEC', 'ECHEC_PORTANT']

# Etats du ramassage : interdits des que la pince tient l'objet. La garde de
# `MachineEtats.pas` les detourne vers RECHERCHE_CARTON.
ETATS_RAMASSAGE = ('ATTENTE', 'DEGAGEMENT', 'DETECTION', 'APPROCHE', 'RECALAGE',
                   'DESCENTE', 'SAISIE')

# (depart, arrivee, condition, action) — pour le graphe du tableau de bord.
TRANSITIONS = [
    ('ATTENTE', 'DEGAGEMENT', 'demarrer', ''),
    ('DEGAGEMENT', 'DETECTION', 'balle visible', ''),
    ('DEGAGEMENT', 'ATTENTE', 'invisible partout', 'refus'),
    ('DETECTION', 'APPROCHE', 'balle vue & portee OK', 'roulis choisi'),
    ('DETECTION', 'ATTENTE', 'hors enveloppe', 'refus'),
    ('DETECTION', 'ATTENTE', 'carton absent', 'cycle non demarre'),
    ('APPROCHE', 'RECALAGE', 'arrive a 110 mm', ''),
    ('APPROCHE', 'DETECTION', 'balle deplacee', 'cible refaite'),
    ('RECALAGE', 'DETECTION', 'balle deplacee', 'cible refaite'),
    ('RECALAGE', 'DESCENTE', 'ecart < 1 mm', 'affaissement appris'),
    ('RECALAGE', 'APPROCHE', 'ne converge pas', 'nouvel essai'),
    ('RECALAGE', 'ECHEC', 'essais epuises', ''),
    ('DESCENTE', 'SAISIE', 'ecart XY < 2.5 mm', ''),
    ('DESCENTE', 'RECALAGE', 'trop excentre', 'nouvel essai, biais garde'),
    ('DESCENTE', 'ECHEC', 'essais epuises', 'pas de fermeture'),
    ('SAISIE', 'REMONTEE', 'statut == 2', ''),
    ('SAISIE', 'RECALAGE', 'rien saisi', 'pince rouverte, nouvel essai'),
    ('SAISIE', 'RETRAIT', 'essais epuises', ''),
    ('REMONTEE', 'RECHERCHE_CARTON', 'prise tenue', 'on revoit le carton'),
    ('REMONTEE', 'DEGAGEMENT', 'objet lache', 'on refait la saisie'),
    ('TRANSFERT', 'LARGAGE', 'au-dessus du carton', ''),
    ('TRANSFERT', 'RECHERCHE_CARTON', 'carton inconnu', ''),
    ('TRANSFERT', 'DEGAGEMENT', 'objet lache', 'on refait la saisie'),
    ('RECHERCHE_CARTON', 'TRANSFERT', 'carton vu', 'position retenue'),
    ('RECHERCHE_CARTON', 'ECHEC_PORTANT', 'introuvable', 'objet garde en main'),
    ('RECHERCHE_CARTON', 'DEGAGEMENT', 'objet lache', 'on refait la saisie'),
    ('LARGAGE', 'RETRAIT', '', 'pince ouverte'),
    ('LARGAGE', 'RECHERCHE_CARTON', 'cible redefinie', 'on la rejuge'),
    ('LARGAGE', 'ECHEC_PORTANT', 'descente refusee', 'objet garde en main'),
    ('ECHEC_PORTANT', 'TRANSFERT', 'carton retrouve', ''),
    ('ECHEC_PORTANT', 'ATTENTE', 'main videe', ''),
    ('RETRAIT', 'DEGAGEMENT', 'mode auto', 'boucle'),
    ('RETRAIT', 'ATTENTE', 'mode manuel', ''),
    ('ECHEC', 'ATTENTE', 'acquitte', ''),
    ('ECHEC', 'ECHEC', 'echecs repetes', 'boucle arretee'),
    ('ECHEC', 'ECHEC_PORTANT', 'objet en main', ''),
]


def cible_a_bouge(ctx, patience=0.5):
    """La balle a-t-elle change de place depuis la mesure en cours ?

    Ne PAS la voir ne signifie pas qu'elle a bouge — le bras l'occulte des
    qu'il s'approche, c'est la situation normale en fin d'approche. Seule une
    detection franche a une position differente compte ; l'absence de detection
    laisse la cible inchangee.

    Une cible qui bouge est une cible NEUVE : les compteurs d'essai et le biais
    de descente appris sur l'ancienne position ne valent plus rien.
    """
    if ctx.detecteur is None or ctx.balle_xy is None:
        return False
    vu = ctx.detecteur(patience=patience)
    if vu is None:
        return False
    xy = np.asarray(vu[:2], float)
    ecart = float(np.linalg.norm(xy - ctx.balle_xy))
    if ecart < SEUIL_DEPLACEMENT:
        return False
    # Deux cameras ne s'accordent pas au millimetre. Quand le bras masque la vue
    # de dessus, la cible passe a la SVPRO et l'ecart de reperage — 10 mm mesures
    # le 24/08 — se lisait comme un deplacement de la balle : retour a DETECTION,
    # 11 s perdues, balle parfaitement immobile. On adopte la nouvelle position
    # sans relancer le cycle : ce n'est pas la balle qui a bouge, c'est l'oeil.
    if ctx.source_balle != ctx.source_cible:
        ctx.note(f'ecart de {ecart:.0f} mm au changement de camera, pas un '
                 f'deplacement — cible ajustee')
        ctx.balle_xy, ctx.source_cible = xy, ctx.source_balle
        return False
    ctx.note(f'la balle a bouge de {ecart:.0f} mm '
             f'({ctx.balle_xy[0]:.0f},{ctx.balle_xy[1]:.0f}) -> ({xy[0]:.0f},{xy[1]:.0f})')
    ctx.balle_xy, ctx.source_cible = xy, ctx.source_balle
    ctx.repart_a_zero()
    ctx.resultats['cible'] = f'{ctx.classe_objet or "objet"} ({xy[0]:.1f}, {xy[1]:.1f}) mm — suivie'
    return True


def dessus(ctx, **kw):
    """Detection par la vue de dessus, ou None si seule la SVPRO voit la balle.

    `exige_dessus` n'existe que sur le detecteur du tableau de bord ; un
    detecteur d'essai ne le connait pas et repond comme d'habitude.
    """
    try:
        return ctx.detecteur(exige_dessus=True, **kw)
    except TypeError:
        return ctx.detecteur(**kw)


# Garde visee par le relevage. Viser JUSTE au-dessus de GARDE_MIN ne suffit
# pas : le controle de chemin de `va_vers` prend pour seuil `depart - 2 mm`,
# donc un depart a 22 mm refuse encore tout trajet qui effleure 19,7 — mesure du
# 25/08, a 0,3 mm pres. On degage franchement, une fois.
GARDE_DEGAGEE = 76.9


def degage_du_sol(ctx):
    """Relever le bras s'il est en appui, par un chemin qui ne descend jamais.

    Une pose dont la garde est deja negative bloque TOUT : depuis le contact,
    n'importe quel trajet interpole creuse encore, donc `va_vers` refuse, donc
    la machine boucle sur ECHEC sans que rien ne la releve. Constate le 25/08 :
    le bras avait fini une manoeuvre pointe a -4,6 mm, et trois lancements de
    suite se sont arretes sur « creux a -13.9 mm sous le seuil -6.6 ».

    On cherche le plus petit relevage sur J2 puis J3 dont le chemin ENTIER reste
    au-dessus du depart — un mouvement qui ne fait que monter ne peut pas
    aggraver un appui.
    """
    depart = garde_au_sol(ctx.pont.angles())
    if depart >= GARDE_MIN:
        return True
    # En PLUSIEURS passes : depuis un appui franc, aucun relevage d'un seul
    # tenant n'atteint la garde visee — mesure du 25/08, J2 +20 deg ne monte que
    # de -4,6 a 30,1 mm. Chaque passe repart d'une pose plus haute, donc d'un
    # seuil de chemin plus permissif.
    for _ in range(4):
        q0 = ctx.pont.angles()
        courante = garde_au_sol(q0)
        if courante >= GARDE_DEGAGEE:
            return True
        meilleure = None
        for j in (1, 2):
            for delta in np.arange(2.0, 30.5, 2.0):
                q = q0.copy()
                q[j] += delta
                arrivee = garde_au_sol(q)
                if arrivee <= courante:
                    continue
                creux = min(garde_au_sol(q0 * (1 - t) + q * t)
                            for t in np.linspace(0, 1, 61))
                if creux < min(GARDE_MIN, courante - 2.0, arrivee - 2.0):
                    continue
                if meilleure is None or arrivee > meilleure[2]:
                    meilleure = (j, delta, arrivee, q)
        if meilleure is None:
            break
        j, delta, arrivee, q = meilleure
        ctx.note(f'bras en appui (garde {courante:.1f} mm) — relevage J{j + 1} '
                 f'de {delta:+.0f} deg vers {arrivee:.1f} mm')
        if va_vers(ctx, q, nom='RELEVAGE') is None:
            break
    if garde_au_sol(ctx.pont.angles()) >= GARDE_MIN:
        return True
    ctx.note(f'bras en appui (garde {depart:.1f} mm) et AUCUN relevage possible '
             f'sur J2 ni J3 — intervenir a la main')
    return False


def _degagement(ctx):
    """Ecarte le bras jusqu'a ce que la VUE DE DESSUS revoie la balle.

    L'arducam regarde de dessus : des que le bras s'approche, il s'interpose et
    se cache la balle a lui-meme. Un cycle qui enchaine sans degager mesure une
    balle a moitie occultee — ou n'en trouve plus du tout et conclut a tort
    qu'il n'y en a pas.

    Se contenter de la SVPRO ici enferme le cycle : le bras reste plante devant
    l'arducam, le degagement se croit inutile, et la mesure de biais de la vue
    laterale fait sortir la balle de l'enveloppe. Journal du 24/08 : sept tours
    ATTENTE -> DEGAGEMENT -> DETECTION a « balle a 430 mm — hors enveloppe »,
    balle immobile et parfaitement atteignable. Le degagement EXIGE donc la vue
    de dessus, et lui seul.
    """
    # L'objet en cours peut avoir disparu (deja depose, ou pousse) : sans cet
    # oubli, le bras balaye indefiniment a la recherche d'une balle qui est deja
    # dans le carton, alors que deux scotchs et un robot attendent.
    #
    # Mais JAMAIS quand la pince tient encore quelque chose. La destination se
    # decide sur ce qui est DANS la pince ; oublier la classe ici la faisait
    # recalculer depuis un objet reste sur la planche, et le 26/08 les deux
    # rouleaux sont partis dans le GRAND carton sous les noms « balle » puis
    # « robot ». Le statut de la pince prime sur le drapeau : `porte_objet` a
    # rendu un faux negatif a la remontee, `en_main` avait donc deja ete efface
    # alors que le rouleau etait bel et bien tenu — la suite du journal le dit
    # elle-meme (« la pince tient l objet, on va le deposer »).
    if ctx.garde_cible:
        # Degagement de REPRISE : on s'ecarte pour revoir le meme objet, pas
        # pour en choisir un autre. Sans ca la classe serait oubliee et la
        # machine repartirait sur la cible la plus proche, en perdant le
        # compteur d'essais qui fait descendre d'un cran.
        ctx.garde_cible = False
    elif ctx.en_main or porte_objet(ctx):
        ctx.en_main = ctx.en_main or ctx.classe_objet
    else:
        ctx.classe_objet = ''
    # Avant tout : si le bras est reste en appui, aucun mouvement ne passera.
    degage_du_sol(ctx)
    if ctx.detecteur is not None and dessus(ctx, patience=0.5) is not None:
        ctx.resultats['degagement'] = 'inutile — objet deja visible de dessus'
        return 'DETECTION'
    ordre = list(BALAYAGE_J1)
    # Au tout premier cycle, aucune cible n'est encore posee : on demande une
    # position, ne serait-ce que par la camera d'appui, plutot que de balayer
    # dans l'ordre du fichier. Mesure du 24/08 : sans ca le bras a essaye cinq
    # poses avant la bonne — 20 s sur un cycle de 99.
    if ctx.balle_xy is None and ctx.detecteur is not None:
        vu = ctx.detecteur(patience=0.5)
        if vu is not None:
            ctx.balle_xy = np.asarray(vu[:2], float)
    if ctx.balle_xy is not None:
        # Azimut connu (boucle automatique) : commencer par le plus loin de la
        # balle. Un J1 proche de son azimut place le bras juste au-dessus
        # d'elle — c'est exactement la position qui l'occulte.
        azimut = float(np.degrees(np.arctan2(ctx.balle_xy[1], ctx.balle_xy[0])))
        ordre.sort(key=lambda j1: -abs(((j1 - azimut + 180.0) % 360.0) - 180.0))
    for j1 in ordre:
        pose = POSE_OBSERVATION.copy()
        pose[0] = j1
        if va_vers_par_etapes(ctx, pose, nom=f'degagement J1={j1:+.0f}',
                              stabilise=False) is None:
            continue
        if ctx.detecteur is None or dessus(ctx) is not None:
            ctx.resultats['degagement'] = f'J1 = {j1:+.0f} deg'
            return 'DETECTION'
        ctx.note(f'  balle invisible de dessus depuis J1={j1:+.0f}, on ecarte davantage')
    ctx.resultats['degagement'] = 'invisible depuis toutes les poses'
    # Et on l'OUBLIE, sans quoi ATTENTE le rechoisit et le meme degagement
    # recommence a l'identique : le 26/08 la boucle a tourne indefiniment,
    # 21,6 s par tour, sur une cible que plus aucune pose ne voyait, pendant que
    # trois autres objets attendaient sur la planche.
    if ctx.balle_xy is not None and not ctx.en_main:
        ctx.oublie(ctx.balle_xy)
        ctx.note(f'{ctx.classe_objet or "objet"} invisible depuis toutes les poses '
                 f'— oublie, on passe au suivant')
        ctx.classe_objet, ctx.balle_xy = '', None
    else:
        ctx.note('objet invisible depuis toutes les poses de degagement')
    return 'ATTENTE'


def ouvre_pince(ctx, patience=3.0, force=False):
    """Ouvrir la pince, et ne rien attendre si elle l'est deja.

    Mesure du 26/08 : un ordre a la pince met 2,16 s a rendre un statut decide,
    et cette duree ne depend pas de l'angle demande — la ouvrir alors qu'elle
    est deja ouverte coute donc 2,16 s pour rien, une fois par cycle. La lecture
    de l'angle, elle, coute 24 ms.

    `force` sert au LARGAGE, et n'est pas une precaution de principe : la pince
    Pro cale sur l'objet a un angle qui n'est pas celui commande (52 mesure pour
    une consigne de 20). Un objet assez large la laisse donc au-dessus du seuil
    d'ouverture tout en etant fermement tenu, et se fier a l'angle reviendrait a
    ne jamais le relacher.
    """
    if not force and ctx.pont.angle_pince() >= PINCE_OUVERTE_MIN:
        ctx.pince_fermee = False
        return
    ctx.pince_fermee = False
    ctx.pont.envoie('pro_gripper_open')
    debut = time.time()
    while time.time() - debut < patience:
        time.sleep(0.1)
        if ctx.pont.statut_pince() in (1, 2, 3):
            return


def _detecte(ctx):
    ouvre_pince(ctx)
    # Le bras vient de se degager : c'est le meilleur moment pour localiser le
    # carton, sans occlusion. Mais on n'en fait PAS une condition de depart :
    # attendre de le voir bloquait tout le cycle (journal du 24/08, la balle
    # n'etait jamais saisie). On saisit d'abord, on cherche le carton ensuite —
    # l'invariant qui compte est ailleurs : une fois l'objet en main, on ne
    # recommence jamais la saisie, on va a RECHERCHE_CARTON.
    if not carton_pret(ctx):
        ctx.note('carton pas encore localise — on saisit quand meme, '
                 'la recherche se fera objet en main')
    vu = ctx.detecteur() if ctx.detecteur else None
    if vu is None:
        ctx.resultats['cible'] = 'aucune'
        ctx.note('balle non detectee')
        return 'ATTENTE'
    xy = np.asarray(vu[:2], float)
    portee = float(np.hypot(*xy))
    ctx.resultats['cible'] = f'{ctx.classe_objet or "objet"} ({xy[0]:.1f}, {xy[1]:.1f}) mm'
    ctx.resultats['portee cible'] = f'{portee:.1f} mm'
    if portee < PORTEE_MIN:
        ctx.resultats['verdict'] = f'TROP PRES (< {PORTEE_MIN:.0f} mm) — le bras se replie sur lui-meme'
        ctx.note(f'cible a {portee:.0f} mm, sous le plancher de {PORTEE_MIN:.0f} mm '
                 f'— la pince vient buter contre le bras')
        ctx.oublie(ctx.balle_xy)
        ctx.classe_objet, ctx.balle_xy = '', None
        return 'ECHEC'
    if portee > PORTEE_MAX:
        ctx.resultats['verdict'] = f'HORS ENVELOPPE (> {PORTEE_MAX:.0f} mm)'
        ctx.note(f'balle a {portee:.0f} mm — hors enveloppe, cible refusee')
        return 'ATTENTE'
    pose = choisit_pose_prise(ctx, xy)
    if pose is None:
        ctx.resultats['verdict'] = 'aucune orientation ne resout'
        return 'ATTENTE'
    roulis, inclinaison, R, z_prise = pose
    ctx.balle_xy, ctx.roulis_balle, ctx.R_balle = xy, roulis, R
    ctx.inclinaison_balle, ctx.z_prise = inclinaison, z_prise
    ctx.source_cible = ctx.source_balle
    ctx.repart_a_zero()
    ctx.resultats['roulis cible'] = f'{roulis:+.0f} deg'
    ctx.resultats['inclinaison outil'] = (
        'verticale' if inclinaison == 0.0
        else f'{inclinaison:+.0f} deg — prise a Z={z_prise:.0f}')
    ctx.resultats['verdict'] = 'atteignable'
    return 'APPROCHE'


def _approche(ctx):
    if cible_a_bouge(ctx):
        return 'DETECTION'                # roulis et enveloppe a rejuger
    cible = resout_ik(np.array([ctx.balle_xy[0], ctx.balle_xy[1], Z_SURVOL]), ctx.R_balle)
    if cible is None or va_vers_par_etapes(ctx, cible[0], nom='approche',
                                           stabilise=False) is None:
        return 'ECHEC'
    return 'RECALAGE'


def _recalage(ctx):
    if cible_a_bouge(ctx):
        return 'DETECTION'
    cible = np.array([ctx.balle_xy[0], ctx.balle_xy[1], Z_SURVOL])
    q, correction, ok = converge(ctx, cible, ctx.R_balle)
    ctx.correction = correction
    ctx.resultats['recalage XY'] = f'{np.linalg.norm(pointe(q) - cible):.2f} mm'
    if ok:
        return 'DESCENTE'
    n = ctx.essai('RECALAGE')
    if n < ESSAIS_MAX:
        ctx.note(f'recalage insuffisant, essai {n + 1}/{ESSAIS_MAX} — on se replace')
        return 'APPROCHE'
    return 'ECHEC'


# Ecart vertical au-dela duquel on recale les doigts avant de fermer. En deca,
# reprendre couterait un mouvement pour rien : la pince a de la course.
TOLERANCE_Z_PRISE = 2.0

# Plancher de la pointe REELLEMENT ATTEINTE, et course maximale que le calage
# s'autorise sur la consigne.
#
# La garde de 2,5 mm etait appliquee a la CONSIGNE, alors qu'elle protege la
# PLANCHE — et entre les deux il y a l'affaissement. Sur le rouleau du 07/09 une
# consigne de 2,5 atterrissait a 5,8 mm : l'ecart valait -4,8, donc
# `max(GARDE_PLANCHE, 2,5 - 4,8)` rendait 2,5, la consigne INCHANGEE. Les trois
# passes renvoyaient la meme valeur et le journal le dit mot pour mot :
# « doigts a 7,3 mm au lieu de 2,5 — on vise 2,5 (passe 1) », puis 5,8 -> 2,5,
# puis 5,8 -> 2,5. La boucle tournait a vide et la pince fermait au-dessus.
#
# La borne porte donc desormais sur la hauteur MESUREE, qu'on relit apres chaque
# palier, et la consigne est libre de passer sous la garde. Mesures du 07/09 sur
# le rouleau : pointe a -4,7 mm elle racle la planche, a -3,1 elle tient sans
# racler, a -2,4 elle ferme a vide.
PLANCHER_POINTE = 13.9
COURSE_CALAGE_MAX = 12.0   # mm sous la hauteur voulue — l'affaissement mesure
                           # plafonne a 8 mm, au-dela c'est un emballement


def cale_les_doigts(ctx, q_mesure, cible, passes=3):
    """Amene les doigts a la hauteur voulue en REINJECTANT l'ecart mesure.

    En Z la descente n'est pas asservie : `descend_par_paliers` ne corrige que
    la derive laterale. L'ecart vertical est libre, il est GRAND, et son SIGNE
    depend de la pose — les deux ont ete mesures le 28/08 :

      * a 305 mm d'allonge, consigne 2,5 -> 3,36 / 6,07 / 5,97 mm atteints,
        soit jusqu'a 3,5 mm TROP HAUT ;
      * sur la visee du tag a 284 mm, consigne 14 -> 6,68, consigne 8 -> 0,59,
        consigne 2,5 -> -4,86, soit 7,4 mm TROP BAS, de facon reproductible.

    Une premiere version se contentait de RECOMMANDER la hauteur voulue. C'est
    sans effet quand le bras la rate deja : reappliquee sur le cas a -4,86, elle
    a rendu -9,04 mm, c'est-a-dire pire. Une consigne qui n'a pas marche ne
    marche pas mieux la seconde fois.

    On corrige donc comme `converge` : on mesure, on reinjecte l'ecart dans la
    consigne, on recommence. La consigne n'est jamais poussee SOUS la garde —
    quand le bras arrive trop bas, la correction la fait monter, ce qui est le
    sens sur ; quand il arrive trop haut, elle la fait descendre, et c'est la
    que la borne compte.

    `cible` est celle de l'OBJET : `descend_par_paliers` y ajoute lui-meme le
    biais lateral, et lui passer la position atteinte le compterait deux fois.
    """
    # Plancher MESURE et non plancher de consigne : voir PLANCHER_POINTE.
    voulu = max(PLANCHER_POINTE, float(cible[2]))
    commande = voulu
    for i in range(passes):
        z = float(pointe(q_mesure)[2])
        ecart = voulu - z
        if abs(ecart) <= TOLERANCE_Z_PRISE:
            ctx.resultats['garde doigts'] = f'{z:.1f} mm'
            return q_mesure
        if z <= PLANCHER_POINTE:
            ctx.note(f'  doigts a {z:.1f} mm — plancher {PLANCHER_POINTE:.1f} '
                     f'atteint, on ferme ici')
            ctx.resultats['garde doigts'] = f'{z:.1f} mm (plancher)'
            return q_mesure
        commande = max(voulu - COURSE_CALAGE_MAX, commande + ecart)
        ctx.note(f'  doigts a {z:.1f} mm au lieu de {voulu:.1f} — on vise '
                 f'{commande:.1f} (passe {i + 1})')
        suivant = descend_par_paliers(ctx, np.array([cible[0], cible[1], commande]),
                                      ctx.R_balle, ctx.correction, nom='calage Z')
        if suivant is None:
            ctx.note('  calage refuse — on ferme la ou on est')
            return q_mesure
        q_mesure = suivant
    ctx.resultats['garde doigts'] = f'{float(pointe(q_mesure)[2]):.1f} mm (recale)'
    return q_mesure


def _descente(ctx):
    baisse = PAS_DESCENTE_ESSAI * ctx.essais.get('SAISIE', 0)
    if baisse:
        ctx.note(f'essai precedent a vide — on descend {baisse:.0f} mm plus bas')
    plancher = Z_PRISE_MIN if ctx.z_valide is None else max(Z_PRISE_MIN, ctx.z_valide)
    cible = np.array([ctx.balle_xy[0], ctx.balle_xy[1],
                      max(plancher, ctx.z_prise + BIAIS_Z_PRISE - baisse)])
    q, ok = descente_verticale(ctx, cible, ctx.R_balle, ctx.correction)
    if q is not None:
        ctx.resultats['descente'] = f'{np.linalg.norm(pointe(q)[:2] - cible[:2]):.2f} mm'
    if ok:
        # La descente grossiere reste bornee a Z_PRISE_MIN : elle ne mesure rien
        # et l'affaissement change de SIGNE selon la pose (07/09 : consigne -2,9
        # atterrit a +3,4 a un endroit, consigne +2 a -6 a un autre). Le calage,
        # lui, relit la hauteur apres chaque palier et peut donc viser la vraie
        # hauteur de l'objet, jusqu'a PLANCHER_POINTE. Sans ca la borne de 2,5 mm
        # ecrasait la cible d'un rouleau plat et la pince fermait au-dessus :
        # 07/09, « garde doigts 3,4 mm » puis « ferme a vide », alors que la
        # prise tient a -3 mm.
        cible_calage = np.array([cible[0], cible[1],
                                 max(PLANCHER_POINTE,
                                     ctx.z_prise + BIAIS_Z_PRISE - baisse)])
        q = cale_les_doigts(ctx, q, cible_calage)
        return 'SAISIE'
    n = ctx.essai('DESCENTE')
    if n < ESSAIS_MAX:
        ctx.resultats['biais appris'] = f'{ctx.biais_descente[:2].round(1)} mm'
        ctx.note(f'descente ratee, essai {n + 1}/{ESSAIS_MAX} — reprise au recalage, '
                 f'biais garde {ctx.biais_descente[:2].round(1)} mm')
        return 'RECALAGE'
    return 'ECHEC'


# Couple de serrage par categorie (100 a 300 chez le bridge). Viser un angle
# plus bas ne serre PAS davantage — la pince cale sur l'objet a l'angle que
# l'objet impose : le seul levier est le couple.
#
# Le petit robot y a ete monte a 250 le 25/08 puis REDESCENDU au defaut : c'est
# une piece imprimee en 3D, a maillons fins, que le serrage casserait. Son
# lachage pendant la remontee ne vient pas d'un manque de serrage mais de
# l'endroit ou la pince se refermait — sur un membre, et trop bas. Cela se
# corrige par le point de prise et sa hauteur, pas par la force.
# Le ROULEAU se serre plus fort. Mesure du 27/08 sur le rouleau bleu a 218 mm :
# la prise reussit — statut 2, angle 24, exactement la signature « rouleau tenu
# de pres » — puis l'objet GLISSE pendant la remontee. Les six decalages
# lateraux essayes ensuite (16 et 22 mm dans les quatre directions) echouent
# tous : le probleme n'est donc pas de viser a cote, c'est que la pince ne
# retient pas ce qu'elle a deja. Le couple est le seul levier — viser un angle
# plus bas ne serre pas davantage, la pince cale sur l'objet.
#
# Le PETIT ROBOT reste au defaut, deliberement : piece imprimee en 3D a maillons
# fins, montee a 250 le 25/08 puis redescendue le meme jour. Son lachage a lui ne
# vient pas du serrage mais de l'endroit ou la pince se referme.
COUPLE_PINCE = {'scotch': 250}
COUPLE_PINCE_DEFAUT = 150
ATTENTE_PINCE_MAX = 2.4   # s — au-dela, la pince a un probleme, pas un objet dur


def _saisie(ctx):
    couple = COUPLE_PINCE.get(ctx.classe_objet, COUPLE_PINCE_DEFAUT)
    if couple != ctx.couple_pince:
        ctx.pont.envoie('set_pro_gripper_torque', torque=couple)
        ctx.couple_pince = couple
        time.sleep(0.3)
    ctx.pont.envoie('pro_gripper_angle', angle=20)
    ctx.pince_fermee = True
    # On attend un statut DECIDE plutot qu'une duree forfaitaire : la pince
    # annonce 0 (« en mouvement ») tant qu'elle serre, et 1/2/3 des qu'elle a
    # conclu. Attendre 2,2 s a tous les coups, c'est attendre le pire cas.
    # On interroge SOUVENT plutot que par tranches de 0,4 s : un aller-retour TCP
    # coute 30 ms, et la pince conclut entre 0,6 et 1,6 s selon l'objet. Attendre
    # par quarts de seconde, c'est ajouter jusqu'a 0,4 s d'immobilite a chaque
    # prise pour rien — quatre objets, quatre fois.
    statut = 0
    limite = time.time() + ATTENTE_PINCE_MAX
    while time.time() < limite:
        time.sleep(0.12)
        statut = ctx.pont.statut_pince()
        if statut in (1, 2, 3):
            break
    angle = ctx.pont.angle_pince()
    ctx.resultats['pince'] = f'{ETAT_PINCE.get(statut, "?")} (angle {angle})'
    cle = cle_roulis('balle', ctx.balle_xy)
    couple_pose = ctx.prise_apprise.get(cle)
    # LE STATUT SEUL NE SUFFIT PAS DANS CE SENS-LA NON PLUS. Mesure du 28/08 sur
    # la figurine imprimee, roulis 0 : statut 2 — « objet saisi » — avec un angle
    # de 22, soit deux degres au-dessus de la pince vide, et la figurine POUSSEE
    # de 18,3 mm. Rien n'etait tenu. Passer en REMONTEE sur ce statut fait perdre
    # le cycle ET empeche d'inscrire le roulis rate : la machine rejoue
    # indefiniment l'angle qui pousse l'objet au lieu de le prendre.
    vide = 0 <= angle < ANGLE_PINCE_FERMEE + MARGE_ANGLE_TENUE
    if statut == 2 and not vide:
        ctx.en_main = ctx.classe_objet
        ctx.prises_ratees.pop(cle, None)
        return 'REMONTEE'
    if statut == 2:
        ctx.note(f'  statut « saisi » DEMENTI par l angle {angle} — pince vide')
    if couple_pose is not None:
        rates = ctx.prises_ratees.setdefault(cle, [])
        if couple_pose not in rates:
            rates.append(couple_pose)
        ctx.prise_apprise.pop(cle, None)
        ctx.note(f'  inclinaison {couple_pose[0]:+.0f} roulis {couple_pose[1]:+.0f} '
                 f'a ferme a vide — on changera d angle au prochain essai')
    n = ctx.essai('SAISIE')
    if n < ESSAIS_MAX:
        ctx.pince_fermee = False
        ctx.pont.envoie('pro_gripper_open')
        time.sleep(2.0)
        # On ECARTE le bras avant de reessayer, au lieu de recaler sur place.
        # Doigts au-dessus de l'objet, le bras le cache a l'arducam : le journal
        # du 26/08 affiche « [scotch non vu] » avant chaque reprise, et la
        # descente suivante repart donc sur la MEME position memorisee — trois
        # essais identiques donnent trois echecs identiques. Reculer coute
        # quelques secondes et rend une position FRAICHE, mesuree sur l'objet
        # tel qu'il est maintenant, y compris s'il a ete pousse par l'essai rate.
        ctx.garde_cible = True
        ctx.note(f'rien saisi, essai {n + 1}/{ESSAIS_MAX} — on ecarte le bras '
                 f'pour revoir l objet, puis on recommence')
        return 'DEGAGEMENT'
    ctx.oublie(ctx.balle_xy)
    ctx.note(f'{ESSAIS_MAX} saisies a vide — objet mis de cote, on passe au suivant')
    ctx.pince_fermee = False
    ctx.pont.envoie('pro_gripper_open')
    return 'RETRAIT'


def _remontee(ctx):
    q, tenue = monte_par_paliers(ctx, ctx.R_balle)
    ctx.resultats['hauteur'] = f'{pointe(q)[2]:.1f} mm'
    if tenue:
        # On ne fait PAS confiance a la position relevee avant la saisie : le
        # carton a pu etre deplace pendant le cycle. Bras en l'air, objet en
        # main, on le cherche a nouveau — c'est la seule facon d'etre adaptatif.
        ctx.R_carton = None
        return 'RECHERCHE_CARTON'
    ctx.en_main = ''
    # ON ROUVRE LA PINCE. Sans ca la machine se contredit elle-meme d'une
    # seconde a l'autre : la remontee conclut « objet lache », puis la garde de
    # `pas()` relit la pince, la trouve « pleine », interdit le ramassage et
    # renvoie deposer — journal du 27/08, deux lignes de suite :
    #
    #     objet lache pendant la remontee — on refait la saisie
    #     DEGAGEMENT interdit — la pince tient l objet, on va le deposer
    #
    # Le bras est alors parti larguer du VIDE au-dessus du carton, et
    # l'inventaire a coche la balle qui n'avait jamais quitte la planche. La
    # cause est physique : quand l'objet glisse des doigts, la pince adaptative
    # RESTE a l'angle ou elle s'etait fermee, et son statut bat entre deux
    # valeurs. Rouvrir remet l'angle a 100 et le statut a plat : les deux
    # temoins disent alors la meme chose, et c'est la verite.
    ctx.pince_fermee = False
    ctx.pont.envoie('pro_gripper_open')
    ctx.note('objet lache pendant la remontee — pince rouverte, on refait la saisie')
    return 'DEGAGEMENT'


def _recherche_carton(ctx):
    """Trouver le carton SANS jamais relacher ni reprendre l'objet.

    Le bras porte la balle : il ne redescend pas, il ne redetecte pas la balle,
    il s'ecarte en hauteur jusqu'a ce que la camera revoie le carton. A defaut,
    la derniere position connue fait foi. Si elle non plus n'est pas
    atteignable, l'objet reste en main et on demande une intervention — jamais
    un retour au ramassage.
    """
    if not porte_objet(ctx):
        ctx.note('plus rien en pince — le ramassage peut reprendre')
        return 'DEGAGEMENT'
    etat = detecte_carton(ctx)
    if etat == 'vu':
        return 'TRANSFERT'
    if etat == 'hors atteinte':
        return 'ECHEC_PORTANT'
    # Le BALAYAGE d'abord, la memoire seulement en dernier recours. L'ordre
    # inverse a ete essaye le 26/08 — aller droit a la position connue au lieu
    # de promener le bras — et REVERSE : la memoire peut etre fausse, et l'etre
    # durablement. Constate le meme jour, `carton_position.json` portait
    # « grand » a (428, -97), du cote du PETIT, a 320 mm du vrai grand ; la
    # balle est allee s'y poser, sur la planche entre les deux boites. Regarder
    # coute quelques secondes, croire une memoire fausse coute un objet a terre.
    if ctx.essai('RECHERCHE_CARTON') <= ESSAIS_CARTON:
        for j1 in BALAYAGE_J1:
            pose = POSE_OBSERVATION.copy()
            pose[0] = j1
            if va_vers_par_etapes(ctx, pose, nom=f'recherche carton J1={j1:+.0f}',
                                  stabilise=False) is None:
                continue
            if not porte_objet(ctx):
                return 'DEGAGEMENT'
            etat = detecte_carton(ctx)
            if etat == 'vu':
                ctx.resultats['recherche carton'] = f'vu depuis J1 = {j1:+.0f} deg'
                return 'TRANSFERT'
            if etat == 'hors atteinte':
                return 'ECHEC_PORTANT'
    memoire = carton_memorise(ctx)
    if memoire is not None:
        cible, roulis, R = carton_atteignable(ctx, memoire)
        if R is not None:
            ctx.carton_xy, ctx.roulis_carton, ctx.R_carton = cible, roulis, R
            ctx.note(f'carton non vu — on vise sa derniere position connue '
                     f'({cible[0]:.0f}, {cible[1]:.0f})')
            ctx.resultats['carton'] = f'({cible[0]:.1f}, {cible[1]:.1f}) mm — memoire'
            return 'TRANSFERT'

    return 'ECHEC_PORTANT'


def _echec_portant(ctx):
    """Objet en main, carton introuvable : on TIENT et on attend.

    Etat volontairement stable : chaque relance ne refait QUE la detection du
    carton. Des que la main est vide (l'operateur a repris la balle), le cycle
    normal redevient possible.
    """
    if not porte_objet(ctx):
        ctx.resultats['verdict'] = 'main vide — pret a repartir'
        return 'ATTENTE'
    etat = detecte_carton(ctx, patience=2.5)
    if etat == 'vu':
        ctx.resultats['verdict'] = 'carton retrouve'
        return 'TRANSFERT'
    ctx.resultats['verdict'] = (
        'OBJET EN MAIN — carton '
        + ('hors d atteinte, le rapprocher' if etat == 'hors atteinte' else 'introuvable')
        + ' puis relancer')
    ctx.note(ctx.resultats['verdict'])
    return 'ECHEC_PORTANT'


def _echec(ctx):
    """Sortie d'erreur generique — sauf si la pince tient encore l'objet.

    S'arrete au bout de `ESSAIS_MAX` echecs d'affilee : en automatique, une
    cause qui ne se resout pas toute seule (une cible que l'IK refuse) faisait
    tourner la boucle sans fin sur ATTENTE -> DEGAGEMENT -> DETECTION -> refus.
    Le compteur repart a chaque depose reussie.
    """
    if porte_objet(ctx):
        return 'ECHEC_PORTANT'
    ctx.echecs += 1
    if ctx.echecs >= ESSAIS_MAX:
        ctx.resultats['verdict'] = (f'{ctx.echecs} echecs d affilee — boucle arretee, '
                                    f'voir le journal')
        ctx.note(f'{ctx.echecs} echecs d affilee sans progres — arret de la boucle')
        return 'ECHEC'
    return 'ATTENTE'


def _transfert(ctx):
    if not porte_objet(ctx):
        ctx.en_main = ''
        ctx.note('prise perdue avant le transfert — on refait la saisie')
        return 'DEGAGEMENT'
    if ctx.carton_xy is None:
        return 'RECHERCHE_CARTON'
    if ctx.R_carton is None:                  # carton designe a la main
        cible, roulis, R = carton_atteignable(ctx, ctx.carton_xy, ctx.carton_polygone)
        if R is None:
            return 'ECHEC_PORTANT'
        ctx.carton_xy, ctx.roulis_carton, ctx.R_carton = cible, roulis, R
    # Dernier controle possible SANS masquer le carton : le bras est encore haut
    # et loin. Le 24/08 la machine a lache la balle a (209, -1), au milieu de la
    # table, sur un carton fantome, pendant que le suivi affichait le vrai a
    # (364, 161). Un point de largage qui a derive de plus d'un carton n'est plus
    # le bon : on refait la recherche plutot que de lacher dans le vide.
    if ctx.detecteur_carton is not None:
        # patience NULLE : le carton est deja suivi en continu par le fil camera,
        # sa position est donc disponible tout de suite. Attendre de nouvelles
        # images ici, c'est attendre apres avoir vu — une fois le carton detecte,
        # on depose.
        vu = ctx.detecteur_carton(patience=0.0)
        if vu is not None:
            derive = float(np.linalg.norm(np.asarray(vu[0], float) - ctx.carton_xy))
            if derive > DERIVE_CARTON_MAX:
                # On NE REPART PAS en recherche : une fois au-dessus du carton on
                # depose, point. On se recale sur la position suivie et on
                # continue — un tour de boucle de plus coute plus cher qu'un
                # largage decale de quelques centimetres dans l'ouverture.
                ctx.note(f'point de largage a {derive:.0f} mm du carton suivi — '
                         f'recale sur le carton, sans refaire de tour')
                cible, roulis, R = carton_atteignable(
                    ctx, np.asarray(vu[0], float), vu[1] if len(vu) > 1 else None)
                if R is None:
                    return 'ECHEC_PORTANT'
                ctx.carton_xy, ctx.roulis_carton, ctx.R_carton = cible, roulis, R
    ctx.resultats['roulis carton'] = f'{ctx.roulis_carton:+.0f} deg'
    cible = resout_ik(np.array([ctx.carton_xy[0], ctx.carton_xy[1], Z_TRANSFERT]),
                      ctx.R_carton)
    if cible is None or va_vers(ctx, cible[0], nom='transfert', stabilise=False) is None:
        ctx.note('transfert refuse vers le carton connu — on le cherche a nouveau')
        ctx.R_carton = None
        return 'RECHERCHE_CARTON'
    return 'LARGAGE' if porte_objet(ctx) else 'DEGAGEMENT'


def _largage(ctx):
    """Le bras est au-dessus du carton : on largue, point.

    Un controle de derniere seconde a ete essaye — redetecter le carton juste
    avant de lacher pour verifier qu'il n'a pas bouge — et RETIRE : a cet
    instant precis le bras est justement au-dessus du carton et le masque, la
    detection saute de 55 a 171 mm, et le cycle repartait en boucle sans jamais
    deposer. Le carton deplace se rattrape a la RECHERCHE, bras degage, pas ici.
    """
    if ctx.R_carton is None:              # l'operateur a redefini la cible en route
        return 'RECHERCHE_CARTON'
    cible = resout_ik(np.array([ctx.carton_xy[0], ctx.carton_xy[1],
                                z_largage_commande(ctx, ctx.carton_xy)]),
                      ctx.R_carton)
    if cible is None or va_vers(ctx, cible[0], nom='largage', stabilise=False) is None:
        ctx.note('descente de largage refusee — objet garde en main')
        return 'ECHEC_PORTANT'
    tp = pointe(ctx.pont.angles())
    # ON N'OUVRE QUE SI ON EST ARRIVE. `va_vers` rend la main quand le bras ne
    # bouge plus, ce qui n'est pas la meme chose qu'etre a la cible : un ordre
    # accepte puis borne par les butees immobilise le bras en chemin. Le 26/08 la
    # pince s'est ouverte a 256 mm du carton vise et l'objet est tombe sur la
    # planche. Un objet garde en main se redepose ; un objet lache a cote se
    # ramasse a la main.
    ecart = float(np.linalg.norm(np.asarray(tp[:2]) - np.asarray(ctx.carton_xy, float)))
    if ecart > ECART_LARGAGE_MAX:
        ctx.note(f'largage ANNULE — pointe a {ecart:.0f} mm du point vise '
                 f'({tp[0]:.0f}, {tp[1]:.0f}) au lieu de '
                 f'({ctx.carton_xy[0]:.0f}, {ctx.carton_xy[1]:.0f}) : objet garde en main')
        ctx.largages_rates.append(np.asarray(ctx.carton_xy, float))
        del ctx.largages_rates[:-6]
        ctx.carton_resolu, ctx.R_carton = None, None
        ctx.resultats['largage'] = f'ANNULE — {ecart:.0f} mm hors du carton'
        return 'ECHEC_PORTANT'
    ouvre_pince(ctx, force=True)
    ctx.resultats['largage'] = f'({tp[0]:.1f}, {tp[1]:.1f}) a Z={tp[2]:.1f}'
    ctx.echecs = 0
    # On COCHE l'objet. Le test geometrique « est-il dans l'ouverture ? » ne
    # tient que tant que le carton reste visible : le bras qui revient le masque,
    # l'ouverture disparait, et la balle deja deposee redevient une cible — le
    # cycle repartait la chercher au fond de la boite. Un largage reussi est un
    # fait acquis, il ne se redetecte pas.
    # On ne coche QUE ce qu'on a porte. `ctx.en_main` est la memoire de la
    # machine : mise a la saisie confirmee, effacee des qu'une prise se perd.
    # S'y fier plutot qu'a `ctx.classe_objet` empeche de cocher un objet reste
    # sur la planche parce que la pince a menti (27/08, la balle cochee sans
    # avoir jamais ete soulevee).
    if ctx.classe_objet and ctx.en_main:
        ctx.deposes[ctx.classe_objet] = ctx.deposes.get(ctx.classe_objet, 0) + 1
    ctx.en_main = ''
    # Et on retient l'ENDROIT du lacher. Le comptage par categorie ne suffit pas
    # quand le meme objet a ete vu sous deux noms : le 26/08 la balle sortait a
    # la fois de son detecteur et de la liste des objets, a 21 mm d'ecart. Le
    # cycle cochait l'un des deux, l'autre restait « a faire », et le bras
    # repartait le chercher AU FOND DU CARTON ou il se refermait sur du vide.
    # Un point ou l'on vient de lacher quelque chose n'est plus une cible,
    # quel que soit le nom qu'on donne a ce qu'on y voit.
    ctx.largages.append(np.asarray(tp[:2], float))
    del ctx.largages[:-12]
    return 'RETRAIT'


def _retrait(ctx):
    q = ctx.pont.angles()
    tp = pointe(q)
    R = ctx.R_carton if ctx.R_carton is not None else ctx.R_balle
    if R is not None:
        haut = resout_ik(np.array([tp[0], tp[1], 175.0]), R)
        if haut is not None:
            va_vers(ctx, haut[0], nom='remontee de retrait', stabilise=False,
                    enchaine=True)
    va_vers_par_etapes(ctx, POSE_OBSERVATION, nom='retour observation', stabilise=False)
    # L'objet vient d'etre depose : la cible SUIVIE n'existe plus. Sans cet
    # effacement, le suivi la garde — il rend la derniere position connue des
    # que plus aucun exemplaire n'est vu pres d'elle — et le cycle suivant
    # repart vers l'endroit VIDE d'ou l'objet a ete enleve, y redescend et s'y
    # referme sur rien. C'est le defaut que l'utilisateur signale depuis une
    # semaine. Un nouveau cycle rechoisit toujours de zero.
    ctx.classe_objet, ctx.balle_xy = '', None
    ctx.repart_a_zero()
    return 'DEGAGEMENT' if ctx.mode_auto else 'ATTENTE'


ACTIONS = {
    'ATTENTE': lambda ctx: 'DEGAGEMENT',
    'DEGAGEMENT': _degagement,
    'DETECTION': _detecte,
    'APPROCHE': _approche,
    'RECALAGE': _recalage,
    'DESCENTE': _descente,
    'SAISIE': _saisie,
    'REMONTEE': _remontee,
    'RECHERCHE_CARTON': _recherche_carton,
    'TRANSFERT': _transfert,
    'LARGAGE': _largage,
    'RETRAIT': _retrait,
    'ECHEC': _echec,
    'ECHEC_PORTANT': _echec_portant,
}


def resume_chrono(ctx):
    """Duree du cycle et les trois etats qui l'ont le plus coute."""
    total = time.time() - ctx.debut_cycle
    pires = sorted(ctx.chrono.items(), key=lambda t: -t[1])[:3]
    return f'{total:.0f} s — ' + ', '.join(f'{nom} {duree:.0f}s' for nom, duree in pires)


class MachineEtats:
    def __init__(self, ctx):
        self.ctx = ctx
        self.etat = 'ATTENTE'

    def pas(self):
        """Execute l'etat courant et passe au suivant. Rend le nouvel etat.

        Garde unique et non contournable : tant que la pince tient l'objet,
        aucun etat de ramassage ne s'execute. Toutes les sorties d'erreur
        passent par ATTENTE puis DEGAGEMENT, donc c'est ici — et non dans
        chaque etat — qu'il faut arreter le retour au debut.
        """
        if self.etat in ETATS_RAMASSAGE and porte_objet(self.ctx):
            self.ctx.note(f'{self.etat} interdit — la pince tient l objet, '
                          f'on va le deposer')
            self.etat = 'RECHERCHE_CARTON'
        self.ctx.note(f'--- {self.etat} ---')
        if not self.ctx.debut_cycle:          # remis a zero par le RETRAIT precedent
            self.ctx.debut_cycle = time.time()
        self.ctx.resultats['cycle en cours'] = (
            f'{time.time() - self.ctx.debut_cycle:.0f} s')
        depart = time.time()
        try:
            suivant = ACTIONS[self.etat](self.ctx)
        except Exception as erreur:                       # le robot doit s'arreter net
            trace = traceback.extract_tb(erreur.__traceback__)[-1]
            self.ctx.note(f'ERREUR dans {self.etat} : {erreur} '
                          f'[{trace.name} ligne {trace.lineno} : {trace.line}]')
            self.etat = 'ECHEC'
            return self.etat
        finally:
            self.ctx.chrono[self.etat] = (self.ctx.chrono.get(self.etat, 0.0)
                                          + time.time() - depart)
        self.ctx.note(f'    {self.etat} : {time.time() - depart:.1f} s')
        if self.etat == 'RETRAIT' and self.ctx.debut_cycle:
            self.ctx.dernier_cycle = time.time() - self.ctx.debut_cycle
            self.ctx.resultats['dernier cycle'] = f'{self.ctx.dernier_cycle:.0f} s'
            self.ctx.resultats['cycle'] = resume_chrono(self.ctx)
            self.ctx.note(f'CYCLE COMPLET : {self.ctx.resultats["cycle"]}')
            self.ctx.chrono, self.ctx.debut_cycle = {}, 0.0
        self.etat = suivant
        return self.etat


if __name__ == '__main__':
    print('auto-test IK :', 'OK' if autotest_ik() else 'ECHEC — solveur non fiable')
    print(f'deport outil : {TOOL.round(2)}  longueur {np.linalg.norm(TOOL):.2f} mm')
    print(f'{len(ETATS)} etats, {len(TRANSITIONS)} transitions')
