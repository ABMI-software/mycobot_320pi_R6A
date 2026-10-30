#!/usr/bin/env python3
"""Carte de correction vision -> realite, qui survit a un deplacement de camera.

Ce que ca corrige. L'extrinseque place la camera ; elle ne corrige pas ce qui
reste APRES. Et ce reste a deux origines qui ne vivent pas aussi longtemps :

  - la CAMERA : ce que les marqueurs coplanaires laissent indetermine — surtout
    la hauteur (la balle est a 32 mm, les marqueurs a Z=0), un peu de
    translation et de rotation. Refaire l'extrinseque change tout ca.
  - le ROBOT : deport d'outil, affaissement, modele du bras, ecart entre le
    centre vu de la balle et l'endroit ou les doigts la prennent. Deplacer la
    camera n'y change rien.

Une carte unique melange les deux : apres une recalibration elle applique a la
nouvelle camera les defauts de l'ancienne. D'ou deux couches.

    ecart(p) = camera_k(p) + robot(p) + residu(p)

  camera_k  4 parametres PAR EXTRINSEQUE k (empreinte du fichier) : translation
            (2), rotation autour de PIVOT, et hauteur — un objet plus haut que
            suppose se projette plus loin du nadir de la camera, d'un facteur
            (p - C) / (C_z - z) connu des qu'on connait le centre C.
  robot     affine (6 parametres), commune a toutes les extrinseques.
  residu    Shepard (IDW) sur ce que les deux premieres n'expliquent pas,
            commun lui aussi : le local, suppose venir du bras.

Nouvelle extrinseque => camera_k repart de ZERO, robot et residu sont gardes.
Deux ou trois echantillons suffisent alors a reajuster les 4 parametres.

Le partage entre couches n'est pas donne par une seule position de camera :
une translation constante peut venir de l'une comme de l'autre. Il est tranche
par des a priori (ecarts-types `SIGMA_*`) : les marqueurs fixent bien la
translation et la rotation dans le plan, mal la hauteur. Chaque nouvelle
position de camera le rend mesure plutot que suppose — et `--valider` le
mesure : il predit chaque extrinseque avec les AUTRES seulement, c'est-a-dire
exactement ce qu'on obtient juste apres une recalibration.

D'ou vient « reel ». Une prise confirmee NE LE DIT PAS : la descente est
asservie sur la cible, la pointe arrive donc sur la cible, et la pince
adaptative recentre la balle en se fermant — l'erreur est effacee au moment
meme ou on voudrait la lire. Apprendre (cible, pointe) ferait recopier a la
carte ses propres corrections.

Le geste qui la lit : RELACHER SUR PLACE. La pince fermee a centre la balle sous
la pointe ; on rouvre sans soulever, on degage, et la camera relit la balle.
(vue apres relache, pointe a la fermeture) est un vrai couple.

Le TEST DE SAISIE l'applique au lancement : apres la question de calibration,
« Faire le test de saisie ? ». Sur un OUI, rien ne bouge seul : au premier cycle
lance (« demarrer la boucle »), la premiere balle saisie est relachee sur place
et relue, et l'ecart reel - vision de cette extrinseque entre dans la carte. En
mode automatique la boucle reprend ensuite la balle, corrigee, et les autres
objets profitent de la meme correction. Il ouvre
la pince : il ne s'arme que sur ce OUI.

    /usr/bin/python3 scripts/pick_dashboard.py

La TOURNEE DES 4 COINS fait la meme chose en quatre points fixes de la zone de
prise, pour couvrir toute la zone (pas les marqueurs : 25 et 26 sont hors
d'atteinte).

    /usr/bin/python3 scripts/lancer_pick_dashboard.py --sans-question --tournee

A la main, dashboard ferme (le pont est mono-client), pince amenee au-dessus du
centre de la balle :

    python3 scripts/correction_vision.py --ajouter --vision 352 48 \
                                         --vision-corrigee --reel-robot
    python3 scripts/correction_vision.py --liste
    python3 scripts/correction_vision.py --valider
    python3 scripts/correction_vision.py --carte
"""
from __future__ import annotations

import argparse
import hashlib
import json
import sys
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np
import yaml

RACINE = Path(__file__).resolve().parents[1]
FICHIER = RACINE / 'scripts' / 'correction_vision.json'
CALIB = RACINE / 'training' / 'calibration'
EXTRINSEQUE = CALIB / 'arducam_extrinsic_pick.yaml'

PUISSANCE = 2.0        # k de Shepard. 1 = tres lisse, 3 = presque du plus-proche-voisin
RAYON_MM = 250.0       # au-dela de tout echantillon, pas de correction du tout
ECART_MAX_MM = 25.0    # borne : au-dela l'echantillon est faux, pas la vision
COINCIDENCE_MM = 0.5   # en deca, on est SUR l'echantillon
# Un nouvel echantillon REMPLACE ceux qu'il recouvre dans la MEME extrinseque,
# pour la meme classe d'objet.
# Les feuilles de la planche glissent (7 mm en 40 min le 11/09) : a un meme
# endroit la mesure la plus recente est la plus vraie. Ceux d'une AUTRE
# extrinseque restent : ils portent la couche robot.
VOISINAGE_MM = 15.0
INCONNUE = 'inconnue'  # echantillons anterieurs a l'empreinte d'extrinseque

PIVOT = np.array([300.0, 0.0])   # centre de la zone de prise, mm
# A priori, en mm (rotation en rad). Le bruit est la dispersion de detection
# mesuree (sigma 0,5 mm) plus le suivi de la pointe.
SIGMA_BRUIT = 1.5
# Translation et rotation : la recalibration les fixe (0,59 mm en leave-one-out
# le 09/09). Hauteur : les marqueurs coplanaires ne la voient pas.
SIGMA_CAMERA = np.array([2.0, 2.0, np.radians(0.3), 15.0])
# Robot : decalage constant de quelques mm, gradient en mm par 100 mm.
SIGMA_ROBOT = np.array([8.0, 8.0, 2.0, 2.0, 2.0, 2.0])

# Relecture apres relache : la balle doit etre immobile et vue de dessus.
REPOS_BALLE_S = 0.8
VUES_RELECTURE = 8
PATIENCE_RELECTURE_S = 6.0
DISPERSION_RELECTURE_MM = 3.0
PINCE_OUVERTE_PATIENCE_S = 4.0


def epoque_de(chemin: Path) -> dict:
    """Empreinte d'une extrinseque et centre optique de sa camera (mm, base)."""
    chemin = Path(chemin)
    brut = chemin.read_bytes()
    T = np.array(yaml.safe_load(brut)['T_cam_world'], float)
    centre = -T[:3, :3].T @ T[:3, 3] * 1000.0
    return {'id': hashlib.sha1(brut).hexdigest()[:12], 'fichier': chemin.name,
            'camera_mm': [round(float(c), 1) for c in centre]}


def _defaut():
    return {'puissance': PUISSANCE, 'rayon_mm': RAYON_MM,
            'ecart_max_mm': ECART_MAX_MM, 'epoques': {}, 'echantillons': []}


def charge(fichier: Path = FICHIER) -> dict:
    try:
        d = json.loads(fichier.read_text())
    except (OSError, json.JSONDecodeError):
        return _defaut()
    base = _defaut()
    base.update(d)
    for e in base['echantillons']:
        e.setdefault('epoque', INCONNUE)
        e.setdefault('z_mm', None)
        e.setdefault('classe', 'balle')
    return base


def enregistre(d: dict, fichier: Path = FICHIER):
    fichier.write_text(json.dumps(d, indent=2, ensure_ascii=False))


def ajoute(vision_xy, reel_xy, note='', epoque=None, z_mm=None, mode='',
           fichier: Path = FICHIER, classe='balle'):
    d = charge(fichier)
    ident = INCONNUE if epoque is None else epoque['id']
    if epoque is not None:
        d['epoques'].setdefault(ident, {'fichier': epoque['fichier'],
                                        'camera_mm': epoque['camera_mm'],
                                        'depuis': f'{datetime.now():%Y-%m-%d %H:%M}'})
    d['echantillons'] = [
        e for e in d['echantillons']
        if e['epoque'] != ident or e['classe'] != classe
        or np.hypot(e['vision'][0] - vision_xy[0], e['vision'][1] - vision_xy[1])
        > VOISINAGE_MM]
    d['echantillons'].append({
        'vision': [round(float(vision_xy[0]), 2), round(float(vision_xy[1]), 2)],
        'reel': [round(float(reel_xy[0]), 2), round(float(reel_xy[1]), 2)],
        'ecart': [round(float(reel_xy[0] - vision_xy[0]), 2),
                  round(float(reel_xy[1] - vision_xy[1]), 2)],
        'date': f'{datetime.now():%Y-%m-%d %H:%M}',
        'epoque': ident,
        'z_mm': None if z_mm is None else round(float(z_mm), 2),
        'mode': mode,
        'classe': classe,
        'note': note,
    })
    enregistre(d, fichier)
    return d


def _colonnes(p, camera_mm, z_mm):
    """Sensibilites de l'ecart en p : (2x4 camera, 2x6 robot)."""
    v = p - PIVOT
    cam = np.zeros((2, 4))
    cam[0, 0] = cam[1, 1] = 1.0
    cam[:, 2] = (-v[1], v[0])
    if camera_mm is not None and z_mm is not None:
        c = np.asarray(camera_mm, float)
        cam[:, 3] = -(p - c[:2]) / (c[2] - z_mm)
    d = v / 100.0
    rob = np.array([[1.0, 0.0, d[0], d[1], 0.0, 0.0],
                    [0.0, 1.0, 0.0, 0.0, d[0], d[1]]])
    return cam, rob


def _idw(p, points, valeurs, puissance, rayon):
    dist = np.linalg.norm(points - p, axis=1)
    colle = dist < COINCIDENCE_MM
    if colle.any():
        return valeurs[colle].mean(axis=0)
    proche = dist <= rayon
    w = 1.0 / dist[proche] ** puissance
    return (w[:, None] * valeurs[proche]).sum(axis=0) / w.sum()


class Modele:
    """Les deux couches ajustees sur un jeu d'echantillons.

    Un seul moindres carres regularise : les parametres camera de chaque
    extrinseque et la couche robot commune sont resolus ENSEMBLE, les a priori
    tranchent ce que les donnees ne separent pas. Le residu part au Shepard.
    """

    def __init__(self, echantillons, epoques):
        self.epoques = epoques
        n = len(echantillons)
        self.points = np.array([e['vision'] for e in echantillons], float).reshape(n, 2)
        self.deltas = np.array([[e['reel'][0] - e['vision'][0],
                                 e['reel'][1] - e['vision'][1]]
                                for e in echantillons], float).reshape(n, 2)
        self.ids = [e['epoque'] for e in echantillons]
        self.z = [e['z_mm'] for e in echantillons]
        self.camera = {}
        self.robot = np.zeros(6)
        self.residus = self.deltas.copy()
        if not n:
            return
        uniques = sorted(set(self.ids))
        rang = {k: i for i, k in enumerate(uniques)}
        m = 4 * len(uniques)
        A = np.zeros((2 * n, m + 6))
        for i in range(n):
            cam, rob = _colonnes(self.points[i], self._centre(self.ids[i]), self.z[i])
            k = rang[self.ids[i]]
            A[2 * i:2 * i + 2, 4 * k:4 * k + 4] = cam
            A[2 * i:2 * i + 2, m:] = rob
        sigmas = np.concatenate([np.tile(SIGMA_CAMERA, len(uniques)), SIGMA_ROBOT])
        x = np.linalg.solve(A.T @ A + np.diag((SIGMA_BRUIT / sigmas) ** 2),
                            A.T @ self.deltas.ravel())
        self.camera = {k: x[4 * rang[k]:4 * rang[k] + 4] for k in uniques}
        self.robot = x[m:]
        self.residus = self.deltas - (A @ x).reshape(n, 2)

    def _centre(self, ident):
        return self.epoques.get(ident, {}).get('camera_mm')

    def ecart(self, xy, epoque_id, camera_mm, z_mm, puissance, rayon, borne):
        p = np.asarray(xy, float)
        if not len(self.points) or not (np.linalg.norm(self.points - p, axis=1)
                                        <= rayon).any():
            return np.zeros(2)
        cam, rob = _colonnes(p, camera_mm, z_mm)
        d = (cam @ self.camera.get(epoque_id, np.zeros(4)) + rob @ self.robot
             + _idw(p, self.points, self.residus, puissance, rayon))
        norme = float(np.linalg.norm(d))
        return d * (borne / norme) if norme > borne else d


class Carte:
    """Carte chargee une fois, interrogee a chaque detection.

    `epoque` est l'extrinseque sous laquelle la vision projette EN CE MOMENT ;
    sa couche camera est celle qu'on applique, zero si elle est neuve.
    """

    def __init__(self, fichier: Path = FICHIER, extrinseque: Path = None, z_mm=None):
        self.fichier = fichier
        self.source = fichier.name
        self.z_mm = z_mm
        self.tournee = None
        self.armement = None
        self.epoque = None
        self._cle = None
        self._vus = deque(maxlen=400)
        self.recharge()
        if extrinseque is not None:
            self.regle_extrinseque(extrinseque)

    def recharge(self):
        d = charge(self.fichier)
        self.puissance = float(d['puissance'])
        self.rayon = float(d['rayon_mm'])
        self.borne = float(d['ecart_max_mm'])
        self.echantillons = d['echantillons']
        self.epoques = d['epoques']
        # Remplace d'un bloc : la vision l'interroge depuis le fil de l'IHM
        # pendant que la machine a etats apprend dans le sien.
        self.modele = Modele(self.echantillons, self.epoques)

    def regle_extrinseque(self, chemin: Path):
        chemin = Path(chemin)
        cle = (str(chemin), chemin.stat().st_mtime_ns)
        if cle == self._cle:
            return
        self._cle = cle
        precedente = self.epoque
        self.epoque = epoque_de(chemin)
        if precedente is None or precedente['id'] != self.epoque['id']:
            print(f'carte : extrinseque {self.epoque["id"]} ({chemin.name}) — '
                  f'{self.dans_epoque()} echantillon(s) propres, '
                  f'{len(self) - self.dans_epoque()} gardes pour la couche robot',
                  flush=True)

    def apprend(self, vision_xy, reel_xy, note='', mode='', classe='balle'):
        ajoute(vision_xy, reel_xy, note, epoque=self.epoque,
               z_mm=self.z_mm if classe == 'balle' else None,
               mode=mode, fichier=self.fichier, classe=classe)
        self.recharge()

    def __len__(self):
        return len(self.echantillons)

    def dans_epoque(self):
        ident = None if self.epoque is None else self.epoque['id']
        return sum(e['epoque'] == ident for e in self.echantillons)

    def ecart(self, xy, hauteur_supposee=True):
        """`hauteur_supposee` : la cible vient d'un rayon coupe au plan du centre
        de la balle. Faux quand la fusion des deux cameras a MESURE la hauteur :
        le terme de hauteur de la couche camera ne s'applique pas."""
        ep = self.epoque
        return self.modele.ecart(xy, None if ep is None else ep['id'],
                                 None if ep is None else ep['camera_mm'],
                                 self.z_mm if hauteur_supposee else None,
                                 self.puissance, self.rayon, self.borne)

    def corrige(self, xy, hauteur_supposee=True):
        return np.asarray(xy, float) + self.ecart(xy, hauteur_supposee)

    def brut(self, xy_corrige):
        """Inverse de `corrige` : le XY que la vision a VU avant la carte.

        Point fixe : la carte varie lentement, quelques iterations suffisent.
        """
        y = np.asarray(xy_corrige, float)
        x = y.copy()
        for _ in range(8):
            x = y - self.ecart(x)
        return x

    def vu(self, classe, xy_brut):
        self._vus.append((time.time(), classe, np.asarray(xy_brut, float)))

    def vues_depuis(self, instant, classe='balle', pres_de=None, rayon=None):
        """XY bruts de cette classe vus depuis `instant` ; a moins de `rayon`
        de `pres_de` quand il y a plusieurs exemplaires (deux scotchs)."""
        return [xy for t, c, xy in list(self._vus)
                if t >= instant and c == classe
                and (pres_de is None or float(np.linalg.norm(xy - pres_de)) <= rayon)]

    def _predit_sans(self, garde, i):
        e = self.echantillons[i]
        m = Modele([self.echantillons[j] for j in garde], self.epoques)
        return m.ecart(e['vision'], e['epoque'], m._centre(e['epoque']), e['z_mm'],
                       self.puissance, self.rayon, self.borne)

    def valide(self):
        """Leave-one-out : chaque echantillon predit par les autres.

        Rend (erreurs par echantillon, erreur sans correction). Si la premiere
        n'est pas nettement plus basse que la seconde, la carte n'apporte rien.
        """
        n = len(self)
        erreurs = [float(np.linalg.norm(self.modele.deltas[i] - self._predit_sans(
            [j for j in range(n) if j != i], i))) for i in range(n)]
        return np.array(erreurs), np.linalg.norm(self.modele.deltas, axis=1)

    def valide_transfert(self):
        """Chaque extrinseque predite par les AUTRES seulement.

        C'est la situation juste apres une recalibration : couche camera a zero,
        couche robot apprise ailleurs. Rend {epoque: (erreurs, sans correction)}.
        """
        n = len(self)
        resultat = {}
        for ident in sorted(set(self.modele.ids)):
            autres = [j for j in range(n) if self.modele.ids[j] != ident]
            if not autres:
                continue
            siens = [i for i in range(n) if self.modele.ids[i] == ident]
            erreurs = [float(np.linalg.norm(self.modele.deltas[i]
                                            - self._predit_sans(autres, i)))
                       for i in siens]
            resultat[ident] = (np.array(erreurs),
                               np.linalg.norm(self.modele.deltas[siens], axis=1))
        return resultat


# Coins de la tournee, repere base (mm). PAS les marqueurs de planche : 25 et 26
# sont a 643 et 609 mm de la base, hors d'atteinte (PORTEE_MAX 430), le bras ne
# peut pas y mesurer son ecart. Un rectangle DANS la zone de prise, des deux
# cotes de l'axe, aux deux allonges ou l'on saisit.
COINS_TOURNEE = ((210.0, 150.0), (210.0, -150.0), (350.0, 130.0), (350.0, -130.0))
RAYON_COIN_MM = 40.0   # balle posee a moins de ca d'un coin = ce coin


class Tournee:
    """Les coins qui n'ont pas encore d'echantillon pour l'extrinseque courante.

    Une nouvelle extrinseque les vide tous : c'est la tournee qui recommence.
    """

    def __init__(self, carte, coins=COINS_TOURNEE, rayon=RAYON_COIN_MM):
        self.carte = carte
        self.coins = [np.asarray(c, float) for c in coins]
        self.rayon = rayon

    def rempli(self, coin):
        ident = None if self.carte.epoque is None else self.carte.epoque['id']
        return any(e['epoque'] == ident
                   and float(np.hypot(*(np.asarray(e['reel'], float) - coin))) <= self.rayon
                   for e in self.carte.echantillons)

    def manquants(self):
        return [c for c in self.coins if not self.rempli(c)]

    def coin_proche(self, xy):
        for c in self.manquants():
            if float(np.hypot(*(np.asarray(xy, float) - c))) <= self.rayon:
                return c
        return None


def dessine_tournee(image, vision, tournee, z_mm):
    """Cercle orange = coin a faire, vert = coin appris. Numerote de 1 a 4."""
    import cv2

    manquants = tournee.manquants()
    for i, coin in enumerate(tournee.coins, 1):
        u, v = vision.vers_pixel(np.array([coin[0], coin[1], z_mm]))
        fait = not any(coin is m for m in manquants)
        couleur = (60, 170, 60) if fait else (0, 140, 255)
        cv2.circle(image, (int(u), int(v)), 16, couleur, 2)
        cv2.putText(image, f'{i} ok' if fait else f'{i}', (int(u) + 18, int(v) + 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, couleur, 1)


CLASSES_TEST = ('balle', 'scotch', 'robot')


class TestSaisie:
    """Le test pose au lancement : UNE saisie de chaque objet, ou qu'il soit.

    Balle, scotch, robot : la premiere prise confirmee de chaque classe est
    relachee sur place et relue. Une fois toutes faites, le pick and place
    tourne normalement avec la correction a jour.
    """

    def __init__(self, classes=CLASSES_TEST):
        self.restant = list(classes)

    def arme_pour(self, xy, classe):
        return np.asarray(xy, float) if classe in self.restant else None

    def fait(self, classe):
        self.restant.remove(classe)

    def bilan(self):
        return ('TEST DE SAISIE TERMINE — correction a jour, le pick and place '
                'l applique' if not self.restant else
                f'reste a tester : {", ".join(self.restant)}')


class ArmementTournee:
    def __init__(self, tournee):
        self.tournee = tournee

    def arme_pour(self, xy, classe):
        return self.tournee.coin_proche(xy) if classe == 'balle' else None

    def fait(self, classe):
        pass

    def bilan(self):
        reste = len(self.tournee.manquants())
        return (f'reste {reste} coin(s) : poser la balle sur le cercle orange et '
                f'relancer un cycle' if reste else
                'TOURNEE TERMINEE, carte a jour pour cette extrinseque')


def branche(classe_vision, fichier: Path = FICHIER, coins=None, calib: Path = CALIB,
            test=None):
    """Branche la carte sur le dashboard dont `classe_vision` est la Vision.

    Deux points d'accroche, sans toucher a `pick_dashboard.py` :

      - `Vision.balle` de l'ARDUCAM rend toujours le XY BRUT, et le memorise :
        c'est ce qu'on relit apres une relache, dans le repere ou la carte
        apprend. L'extrinseque y est relue a chaque trame (sur sa date, sans
        cout) : une recalibration bascule la couche camera d'elle-meme.
      - `Fenetre._detecte_balle` rend la cible CORRIGEE, quelle que soit la
        camera qui l'a fournie. Avant, seule l'arducam seule etait corrigee :
        des que la SVPRO voyait aussi la balle, la fusion repartait du pixel
        brut et le bras descendait sans correction.
      - `Vision.objets` de l'arducam rend les AUTRES objets (scotch, robot)
        corriges, pour qu'ils profitent de ce que la balle a appris. Le relais
        SVPRO des objets s'apprend sur ces positions : il suit.

    Le geste « relacher sur place » s'arme par `coins` (tournee des 4 coins) ou
    par `test` (la prochaine balle saisie, une fois). `test=None` reprend la
    reponse donnee a la question du lancement. Rien ne demarre seul : le test
    se fait au premier cycle que l'operateur lance, comme d'habitude. Sans l'un ni l'autre, rien
    n'ouvre la pince hors du cycle normal.

    Rend la carte.
    """
    import pick_fsm

    module = sys.modules[classe_vision.__module__]
    carte = Carte(fichier, z_mm=module.HAUTEUR_CENTRE_BALLE)
    origine_balle = classe_vision.balle

    def balle(self, image):
        vu = origine_balle(self, image)
        if vu is not None and self.source.startswith('arducam'):
            carte.regle_extrinseque(calib / self.source)
            carte.vu('balle', vu[0])
        return vu

    classe_vision.balle = balle

    origine_objets = classe_vision.objets

    def objets(self, image, *a, **kw):
        trouves = origine_objets(self, image, *a, **kw)
        if not self.source.startswith('arducam'):
            return trouves
        for classe, xy, _ in trouves:
            carte.vu(classe, xy)
        # Sans le terme de hauteur : il est appris sur le centre de la balle,
        # rien ne dit que la hauteur supposee d'un rouleau se trompe pareil.
        # Robot (outil, affaissement) et placement camera, eux, sont communs.
        return [(classe, carte.corrige(xy, hauteur_supposee=False), contour)
                for classe, xy, contour in trouves]

    classe_vision.objets = objets

    origine_detecte = module.Fenetre._detecte_balle

    def _detecte_balle(self, *a, **kw):
        xy = origine_detecte(self, *a, **kw)
        if xy is None:
            return None
        return carte.corrige(xy, hauteur_supposee=self._camera_balle != 'arducam+svpro')

    module.Fenetre._detecte_balle = _detecte_balle

    if test is None:
        import calibration_dialogue
        test = calibration_dialogue.TEST_SAISIE
    if coins is not None:
        carte.tournee = Tournee(carte, coins)
        carte.armement = ArmementTournee(carte.tournee)
    elif test:
        carte.armement = TestSaisie()
    if carte.armement is not None:
        pick_fsm.ACTIONS['SAISIE'] = apprend_par_relache(
            carte, pick_fsm.ACTIONS['SAISIE'], carte.armement)
        pick_fsm.TRANSITIONS.append(('SAISIE', 'ATTENTE', 'test / coin',
                                     'relachee sur place, relue'))
    return carte


def _attend_pince_ouverte(ctx):
    import pick_fsm

    limite = time.time() + PINCE_OUVERTE_PATIENCE_S
    while time.time() < limite:
        time.sleep(0.15)
        if ctx.pont.angle_pince() >= pick_fsm.PINCE_OUVERTE_MIN:
            return True
    return False


def apprend_par_relache(carte, saisie, armement):
    """Balle prise la ou l'armement le demande : relacher sur place, la relire.

    La cible est le centre de la balle donne par la detection pixel, corrige
    par la carte. Apres la prise, la pince a centre la balle sous la pointe :
    la relire donne sa position REELLE, et (vue brute, pointe) est l'ecart que
    la carte apprend.

    Le geste ne s'arme pas — l'objet reste en main, le cycle continue — la ou
    l'armement ne le demande pas (hors coin, classe deja testee).

    L'objet est relu la ou la pince l'a pose, par la meme detection que celle
    qui le vise : `Vision.balle` pour la balle, `Vision.objets` pour les
    autres (le plus proche de la pointe, s'il y en a deux).

    Une fois la balle relachee, le cycle S'ARRETE (ATTENTE), qu'on ait appris
    ou non : jamais deux fermetures enchainees sans l'operateur. Rien n'est
    appris :
      - si la balle n'est pas relue de dessus, immobile, dans le delai ;
      - si l'extrinseque a change entre la prise et la relecture ;
      - si l'ecart depasse la borne.
    """
    import pick_fsm

    def saisie_relache(ctx):
        cible = None if ctx.balle_xy is None else np.asarray(ctx.balle_xy, float)
        classe = ctx.classe_objet
        lieu = None if cible is None else armement.arme_pour(cible, classe)
        suivant = saisie(ctx)
        if suivant != 'REMONTEE' or lieu is None or carte.epoque is None:
            return suivant
        q = ctx.pont.angles()
        reel = pick_fsm.pointe(q)[:2]
        epoque = carte.epoque['id']
        ctx.note(f'apprentissage : {classe} tenu en ({lieu[0]:.0f}, {lieu[1]:.0f}), pointe '
                 f'en ({reel[0]:.1f}, {reel[1]:.1f}) — relache SUR PLACE pour la relire')

        ctx.pont.envoie('pro_gripper_open')
        ctx.pince_fermee = False
        if not _attend_pince_ouverte(ctx):
            ctx.note('apprentissage : la pince ne s est pas ouverte — on ne bouge pas')
            return 'ECHEC'
        ctx.en_main = ''
        haut = pick_fsm.resout_ik(np.array([reel[0], reel[1], pick_fsm.Z_SURVOL]),
                                  ctx.R_balle)
        if haut is None or pick_fsm.va_vers(ctx, haut[0], nom='apprentissage : remontee a vide',
                                            stabilise=False) is None:
            ctx.note('apprentissage : remontee refusee — rien appris')
            return _arrete(ctx)
        if pick_fsm.va_vers_par_etapes(ctx, pick_fsm.POSE_OBSERVATION,
                                       nom='apprentissage : vue degagee') is None:
            ctx.note('apprentissage : pose d observation refusee — rien appris')
            return _arrete(ctx)

        debut = time.time() + REPOS_BALLE_S
        limite = debut + PATIENCE_RELECTURE_S
        vus = []
        while time.time() < limite:
            time.sleep(0.1)
            vus = carte.vues_depuis(debut, classe, pres_de=reel, rayon=carte.borne)
            if len(vus) >= VUES_RELECTURE:
                break
        if len(vus) < VUES_RELECTURE:
            ctx.note(f'apprentissage : {classe} relu {len(vus)} fois sur {VUES_RELECTURE} '
                     f'— rien appris, relancer un cycle')
            return _arrete(ctx)
        vus = np.array(vus[-VUES_RELECTURE:])
        apres = vus.mean(axis=0)
        dispersion = float(np.linalg.norm(vus - apres, axis=1).max())
        if dispersion > DISPERSION_RELECTURE_MM:
            ctx.note(f'apprentissage : {classe} instable a la relecture ({dispersion:.1f} mm) '
                     f'— elle roule ? rien appris')
            return _arrete(ctx)
        if carte.epoque['id'] != epoque:
            ctx.note('apprentissage : extrinseque changee pendant le geste — rien appris')
            return _arrete(ctx)
        ecart = reel - apres
        if float(np.linalg.norm(ecart)) > carte.borne:
            ctx.note(f'apprentissage : ecart {np.linalg.norm(ecart):.1f} mm au-dela de '
                     f'la borne {carte.borne:.0f} — echantillon refuse')
            return _arrete(ctx)
        vise = float(np.linalg.norm(apres - cible))
        carte.apprend(apres, reel, mode='relache', classe=classe,
                      note=f'{classe} en ({lieu[0]:.0f}, {lieu[1]:.0f}), vise a {vise:.1f} mm '
                           f'de sa position relue')
        armement.fait(classe)
        ctx.resultats['ecart vision/reel'] = f'({ecart[0]:+.1f}, {ecart[1]:+.1f}) mm'
        ctx.note(f'apprentissage : {classe} ecart reel - vision ({ecart[0]:+.1f}, {ecart[1]:+.1f}) mm, '
                 f'la cible visee etait a {vise:.1f} mm de la balle relue — '
                 f'{armement.bilan()}')
        return _arrete(ctx)

    return saisie_relache


def _arrete(ctx):
    ctx.classe_objet, ctx.balle_xy = '', None
    ctx.repart_a_zero()
    return 'ATTENTE'


def main():
    a = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument('--ajouter', action='store_true')
    a.add_argument('--vision', nargs=2, type=float, metavar=('X', 'Y'))
    a.add_argument('--reel', nargs=2, type=float, metavar=('X', 'Y'))
    a.add_argument('--reel-robot', action='store_true',
                   help='reel = pointe des angles lus au pont (aucun mouvement) ; '
                        'dashboard FERME, le pont est mono-client')
    a.add_argument('--vision-corrigee', action='store_true',
                   help='--vision est le XY affiche par le dashboard, deja corrige '
                        'par la carte : on le ramene au brut')
    a.add_argument('--extrinseque', type=Path, default=EXTRINSEQUE,
                   help='extrinseque sous laquelle --vision a ete mesure')
    a.add_argument('--note', default='')
    a.add_argument('--liste', action='store_true')
    a.add_argument('--valider', action='store_true')
    a.add_argument('--carte', action='store_true')
    a.add_argument('--ou', nargs=2, type=float, metavar=('X', 'Y'),
                   help='afficher la correction en ce point')
    args = a.parse_args()

    sys.path.insert(0, str(RACINE / 'scripts'))
    from pick_dashboard import HAUTEUR_CENTRE_BALLE

    carte = Carte(extrinseque=args.extrinseque, z_mm=HAUTEUR_CENTRE_BALLE)

    if args.ajouter:
        if not (args.vision and (args.reel or args.reel_robot)):
            raise SystemExit('--ajouter exige --vision X Y et --reel X Y (ou --reel-robot)')
        vision = np.asarray(args.vision, float)
        if args.vision_corrigee:
            vision = carte.brut(vision)
        reel = args.reel
        if args.reel_robot:
            import pick_fsm
            pont = pick_fsm.Pont()
            try:
                q = pont.angles()
            finally:
                pont.ferme()
            reel = pick_fsm.pointe(q)[:2]
            print(f'angles lus {np.round(q, 1).tolist()} -> pointe '
                  f'({reel[0]:.1f}, {reel[1]:.1f}) mm')
        carte.apprend(vision, reel, args.note, mode='manuel')
        e = carte.echantillons[-1]
        print(f"echantillon {len(carte)} : vision {e['vision']} -> "
              f"reel {e['reel']}, ecart ({e['ecart'][0]:+.2f}, {e['ecart'][1]:+.2f}) mm")
        return

    if args.liste or not (args.valider or args.carte or args.ou):
        print(f'{len(carte)} echantillon(s) — puissance {carte.puissance}, '
              f'rayon {carte.rayon:.0f} mm, borne {carte.borne:.0f} mm')
        print(f'extrinseque courante {carte.epoque["id"]} ({carte.epoque["fichier"]}), '
              f'camera en {carte.epoque["camera_mm"]} mm')
        for ident in sorted(set(carte.modele.ids)):
            c = carte.modele.camera[ident]
            info = carte.epoques.get(ident, {})
            marque = ' <- courante' if ident == carte.epoque['id'] else ''
            print(f'  extrinseque {ident}{marque}  {info.get("fichier", "?")} '
                  f'depuis {info.get("depuis", "?")}  camera : translation '
                  f'({c[0]:+.2f}, {c[1]:+.2f}) mm, rotation {np.degrees(c[2]):+.2f} deg, '
                  f'hauteur {c[3]:+.1f} mm')
        r = carte.modele.robot
        print(f'  robot : decalage ({r[0]:+.2f}, {r[1]:+.2f}) mm, gradient '
              f'[{r[2]:+.2f} {r[3]:+.2f} ; {r[4]:+.2f} {r[5]:+.2f}] mm/100 mm')
        for i, e in enumerate(carte.echantillons, 1):
            print(f"  {i:2d}  vision ({e['vision'][0]:7.1f}, {e['vision'][1]:7.1f}) "
                  f"ecart ({e['ecart'][0]:+6.2f}, {e['ecart'][1]:+6.2f}) mm  "
                  f"{e['epoque']}  {e['date']}  {e.get('mode', '')}  {e['note']}")
        if not len(carte):
            print('  aucun — la correction est neutre tant qu il n y en a pas')

    if args.ou:
        d = carte.ecart(args.ou)
        p = carte.corrige(args.ou)
        print(f'en ({args.ou[0]:.1f}, {args.ou[1]:.1f}) : correction '
              f'({d[0]:+.2f}, {d[1]:+.2f}) mm -> ({p[0]:.1f}, {p[1]:.1f})')

    if args.valider:
        if len(carte) < 3:
            raise SystemExit('au moins 3 echantillons pour valider')
        err, brut = carte.valide()
        print(f'\nleave-one-out sur {len(carte)} echantillons :')
        for i, (e, b) in enumerate(zip(err, brut), 1):
            print(f'  {i:2d}  sans correction {b:5.2f} mm  ->  predit par les '
                  f'autres {e:5.2f} mm')
        print(f'  moyenne {brut.mean():.2f} mm -> {err.mean():.2f} mm  '
              f'| pire {brut.max():.2f} -> {err.max():.2f}')
        if err.mean() >= brut.mean():
            print('  -> la carte n APPORTE RIEN sur ces points : les ecarts ne '
                  'sont pas spatialement coherents, ou il en manque.')
        transfert = carte.valide_transfert()
        if not transfert:
            print('\ntransfert entre extrinseques : une seule extrinseque, rien a '
                  'mesurer — le partage camera/robot repose sur les a priori.')
        for ident, (e, b) in transfert.items():
            print(f'\nextrinseque {ident} predite par les AUTRES seulement '
                  f'({len(e)} echantillon(s)) — situation juste apres recalibration :')
            print(f'  moyenne {b.mean():.2f} mm -> {e.mean():.2f} mm  '
                  f'| pire {b.max():.2f} -> {e.max():.2f}')
            if e.mean() >= b.mean():
                print('  -> la couche robot NE SE TRANSFERE PAS : repartir de zero '
                      'apres recalibration vaudrait mieux.')

    if args.carte:
        print('\ncorrection interpolee, pas de 100 mm (norme en mm) :')
        for y in range(200, -201, -100):
            ligne = []
            for x in range(100, 551, 100):
                n = float(np.linalg.norm(carte.ecart((x, y))))
                ligne.append('   .  ' if n == 0 else f'{n:5.2f} ')
            print(f'  Y={y:+5d}  ' + ''.join(ligne))
        print('        X=' + ''.join(f'{x:6d}' for x in range(100, 551, 100)))


if __name__ == '__main__':
    main()
