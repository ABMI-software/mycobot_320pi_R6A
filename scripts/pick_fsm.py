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

PI = ('10.10.0.221', 5005)
TOOL = np.array(json.loads((RACINE / 'scripts' / 'tool_offset.json').read_text())
                ['tool_offset_mm'], float)

# Pose de prise validee le 20/08 : elle fixe l'orientation de reference et son
# azimut. Toute cible reprend cette orientation, tournee de l'ecart d'azimut.
Q_REFERENCE = np.array([45.43, -86.57, -96.76, 135.52, 1.84, -43.33])
AZIMUT_REFERENCE = 7.2

Z_PRISE = -5.0
Z_SURVOL = 110.0        # au-dessus du sommet de la balle (~71 mm)
Z_TRANSFERT = 170.0
Z_LARGAGE = 100.0       # rebord du carton ~60 mm : la balle entre avant le lacher
GARDE_MIN = 25.0
PLANCHER = -20.0        # mm — aucune pose legitime sous la planche
CHUTE_MAX = 220.0       # mm — descente verticale maximale en UN seul ordre
ETAPES_MAX = 12         # decoupage maximal d'un grand deplacement
ESSAIS_MAX = 3
VITESSE = 25
# Mesure du 24/08, roulis libre, outil vertical, cible a hauteur de table ET
# survol a 110 : 330, 340 et 350 mm se resolvent (roulis +30, +30, +60), 360 non.
# La valeur precedente, 335, refusait des balles parfaitement atteignables — une
# a 343,6 mm a ete refusee alors qu'elle se resout avec un roulis de +30.
# Incliner l'outil n'ajoute rien ici : teste de +10 a +30 deg, aucune solution.
PORTEE_MAX = 355.0
# Le largage se fait plus haut que la prise, donc un peu plus loin : mesure a
# l'azimut du carton, 355 mm passe (roulis +60), 365 mm ne passe plus. Ce
# pre-filtre evite les 40 s que coute un `choisit_roulis` qui echoue.
PORTEE_CARTON_MAX = 360.0
ESSAIS_CARTON = 2       # tours de balayage avant de renoncer, objet en main
MARGE_LARGAGE = 38.0    # mm — recul des parois : la balle fait 33 mm de rayon
MARGE_LARGAGE_MIN = 15.0  # plancher quand l'ouverture ne peut pas offrir mieux
LARGAGES_TESTES = 3     # points d'ouverture essayes avant de renoncer
MEMOIRE_CARTON = RACINE / 'scripts' / 'carton_position.json'
MEMOIRE_AFFAISSEMENT = RACINE / 'scripts' / 'affaissement.json'
AFFAISSEMENT_MAX = 6.0  # deg — borne de l'ecart articulaire reinjecte au depart
# Le carton ne bouge pas entre le debut d'un cycle et la depose : re-resoudre sa
# pose a chaque etat coutait 4 a 15 s pour un resultat identique.
TOLERANCE_CARTON_RESOLU = 12.0

# Pour une sphere, l'orientation des doigts dans le plan horizontal est sans
# importance : ce roulis est un degre de liberte gratuit qui porte la portee
# utile de 300 a 330 mm. Balayage du plus proche du nominal au plus eloigne.
ROULIS = [0, 30, -30, 60, -60, 90, -90, 120]

ETAT_PINCE = {0: 'en mouvement', 1: 'rien saisi', 2: 'objet saisi', 3: 'objet lache'}

SEUIL_DEPLACEMENT = 8.0   # mm — au-dela, la balle a bouge : on refait la cible
# Le bras masque ce qu'il survole : une detection de carton dont le centre tombe
# sous la pointe est celle de l'ombre ou du bras lui-meme. Mesure du 24/08 :
# elle sautait de 55, 76 puis 171 mm d'un pas a l'autre, carton immobile.
RAYON_MASQUAGE = 120.0

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
        self.timeout = timeout
        self.sock = socket.create_connection(PI, timeout=timeout)
        self.sock.settimeout(timeout)
        self.buf = b''

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
                return q
            time.sleep(0.3)
        raise RuntimeError(f'get_angles illisible : {derniere!r}')

    def statut_pince(self):
        rep = self.envoie('get_pro_gripper_status')
        chiffres = ''.join(c for c in rep if c.isdigit())
        return int(chiffres[0]) if chiffres else -1

    def ferme(self):
        self.sock.close()


# --------------------------------------------------------------------------- #
#  Geometrie
# --------------------------------------------------------------------------- #

def pose_bride(q_deg):
    positions, transformations = forward_kinematics(np.radians(np.asarray(q_deg, float)))
    return np.asarray(positions['mycobot320_link6']) * 1000.0, transformations[6][:3, :3]


def pointe(q_deg):
    p, R = pose_bride(q_deg)
    return p + R @ TOOL


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
              amorces_max=None, iterations=400):
    """Meilleure solution parmi les amorces, ou None.

    Sortie anticipee des qu'une amorce donne une solution nettement bonne :
    balayer les 22 amorces jusqu'au bout coute 2,8 s, et `choisit_roulis` en
    enchaine 24 — 17 s d'attente pour une seule etape. Le seuil de sortie est
    trois fois plus severe que le seuil d'acceptation, pour ne s'arreter que
    sur une solution qui ne demande aucun arbitrage.
    """
    azimut = float(np.degrees(np.arctan2(p_cible[1], p_cible[0])))
    amorces = list(_AMORCES_MESUREES)
    for q in _AMORCES_MESUREES + _AMORCES_SYNTHETIQUES:
        s = q.copy()
        s[0] = azimut
        amorces.append(s)
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
    roulis_balle: float = 0.0
    roulis_carton: float = 0.0
    roulis_appris: dict = field(default_factory=dict)   # {'balle': deg, 'carton': deg}
    carton_resolu: tuple = None       # (centre resolu, point de largage, roulis, R)
    R_balle: np.ndarray = None
    R_carton: np.ndarray = None
    correction: np.ndarray = field(default_factory=lambda: np.zeros(6))
    biais_descente: np.ndarray = field(default_factory=lambda: np.zeros(3))
    essais: dict = field(default_factory=dict)
    resultats: dict = field(default_factory=dict)
    journal: list = field(default_factory=list)
    mode_auto: bool = False
    echecs: int = 0                   # echecs d'affilee, remis a zero par une depose
    chrono: dict = field(default_factory=dict)   # secondes passees par etat, cycle courant
    debut_cycle: float = 0.0
    detecteur: object = None          # callable(patience=...) -> (x, y) ou None
    detecteur_carton: object = None   # idem, pour le carton

    def note(self, texte):
        self.journal.append(texte)
        del self.journal[:-200]

    def essai(self, etat):
        self.essais[etat] = self.essais.get(etat, 0) + 1
        return self.essais[etat]

    def repart_a_zero(self):
        """Nouvelle cible : le biais et les compteurs appris ne valent plus."""
        self.essais.clear()
        self.biais_descente = np.zeros(3)


def porte_objet(ctx):
    """La pince tient-elle l'objet ? Le statut 2 en est le seul juge.

    Tant que c'est vrai, aucun retour au ramassage n'est permis. Le cycle a
    tourne des heures la balle en main parce qu'un carton introuvable renvoyait
    vers ECHEC, donc vers ATTENTE, donc vers un nouveau DEGAGEMENT : le bras
    repartait chercher une balle qu'il tenait deja.
    """
    return ctx.pont is not None and ctx.pont.statut_pince() == 2


def memorise_carton(xy, roulis_appris=None):
    MEMOIRE_CARTON.write_text(json.dumps(
        {'carton_xy_mm': [float(xy[0]), float(xy[1])],
         'roulis_appris': roulis_appris or {}}))


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


def carton_memorise(ctx=None):
    """Derniere position vue, d'une seance a l'autre. Le carton bouge peu.

    Recharge aussi le roulis qui marchait : sans lui, le premier cycle d'une
    seance repaie le choix complet (mesure : 14,5 s contre 4,1 s ensuite).
    """
    if not MEMOIRE_CARTON.exists():
        return None
    try:
        d = json.loads(MEMOIRE_CARTON.read_text())
        xy = np.asarray(d['carton_xy_mm'], float)
    except (json.JSONDecodeError, KeyError, TypeError, ValueError):
        return None
    if ctx is not None:
        for cible, angle in (d.get('roulis_appris') or {}).items():
            ctx.roulis_appris.setdefault(cible, angle)
    return xy


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
    dedans.sort(key=lambda p: float(np.linalg.norm(p - xy)))
    # Deux candidats voisins echouent ou reussissent ensemble : les espacer.
    choisis = []
    for p in dedans:
        if all(np.linalg.norm(p - q) > 20.0 for q in choisis):
            choisis.append(p)
        if len(choisis) == LARGAGES_TESTES:
            break
    return choisis


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
        if portee > PORTEE_CARTON_MAX:
            ctx.note(f'point de largage a {portee:.0f} mm — au-dela de '
                     f'{PORTEE_CARTON_MAX:.0f} mm aucun roulis ne resout')
            continue
        roulis, R = choisit_roulis(cible, [Z_TRANSFERT, Z_LARGAGE],
                                   prefere=roulis_retenu(ctx, 'carton', cible))
        if R is not None:
            ctx.roulis_appris[cle_roulis('carton', cible)] = roulis
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
    pointe_xy = pointe(ctx.pont.angles())[:2]
    if float(np.linalg.norm(xy - pointe_xy)) < RAYON_MASQUAGE:
        ctx.note(f'carton "vu" a {np.linalg.norm(xy - pointe_xy):.0f} mm sous la '
                 f'pointe — le bras ou son ombre, detection ignoree')
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
    memorise_carton(cible, ctx.roulis_appris)
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
    cible, roulis, R = carton_atteignable(ctx, memoire)
    if R is None:
        return False
    ctx.carton_xy, ctx.carton_polygone = cible, None
    ctx.roulis_carton, ctx.R_carton = roulis, R
    ctx.resultats['carton'] = f'({cible[0]:.1f}, {cible[1]:.1f}) mm — memoire'
    return True


def va_vers(ctx, q_cible, vitesse=VITESSE, nom='', patience=60, stabilise=True):
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
    depart, arrivee = garde_au_sol(q0), garde_au_sol(q_cible)
    if arrivee < PLANCHER:
        ctx.note(f'{nom} REFUSE — cible a {arrivee:.1f} mm, sous le plancher {PLANCHER:.0f}')
        return None
    # La chute se mesure sur la POINTE, pas sur la garde : la garde est le point
    # le plus bas de tout le bras, et un coude reste bas meme bras dresse — elle
    # ne bouge quasiment pas quand la pointe plonge de 590 mm.
    chute = float(pointe(q0)[2] - pointe(q_cible)[2])
    if chute > CHUTE_MAX:
        ctx.note(f'{nom} REFUSE — la pointe plongerait de {chute:.0f} mm en un seul ordre')
        return None
    seuil = min(GARDE_MIN, depart - 2.0, arrivee - 2.0)
    minimum = min(garde_au_sol(q0 * (1 - t) + q_cible * t) for t in np.linspace(0, 1, 61))
    if minimum < seuil:
        ctx.note(f'{nom} REFUSE — creux a {minimum:.1f} mm sous le seuil {seuil:.1f}')
        return None
    ctx.pont.envoie('send_angles', angles=[round(float(v), 2) for v in q_cible],
                    speed=vitesse)
    # Arrivee = le bras ne bouge PLUS, et non le bras qui atteint sa consigne.
    # L'affaissement laisse un ecart permanent d'environ 1,9 deg sur J2,
    # superieur au seuil : le test sur la consigne n'etait donc jamais satisfait
    # et l'attente allait au bout de sa patience a CHAQUE mouvement — 22,7 s
    # mesurees le 24/08, identiques a vitesse 25 et a vitesse 50, ce qui prouve
    # que le temps ne venait pas du robot.
    if float(np.abs(q_cible - q0).max()) > 0.5:
        precedent, parti = q0, False
        for _ in range(patience):
            time.sleep(0.15)
            q = ctx.pont.angles()
            parti = parti or float(np.abs(q - q0).max()) > 0.5
            immobile = float(np.abs(q - precedent).max()) < 0.2
            if float(np.abs(q - q_cible).max()) < 1.2 or (parti and immobile):
                break
            precedent = q
    if stabilise:
        # L'affaissement doit s'etablir avant qu'on MESURE la pose. Un simple
        # transit — se degager, transferer, se retirer — ne mesure rien : cette
        # seconde y est perdue, et il y a une demi-douzaine de transits par cycle.
        time.sleep(1.0)
    return ctx.pont.angles()


def converge(ctx, p_cible, R, passes=4, tol=1.0):
    """Compense l'affaissement en reinjectant l'ecart articulaire mesure.

    `send_angles` n'atteint pas la consigne : ~+1.9 deg sur J2, ce qui deplace la
    pointe ET fait pivoter l'outil. Rend (q, correction, converge).
    """
    q_mesure = ctx.pont.angles()
    # Depart CHAUD : on repart de l'affaissement deja mesure au lieu de le
    # redecouvrir. Sans lui, la premiere passe commande la solution IK brute et
    # arrive 10 mm trop bas — deux passes pour rien, a chaque cycle.
    q_commande = q_mesure + affaissement_memorise()
    ecart = float('inf')
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
    correction = q_commande - q_mesure
    if ecart < 3.0:
        memorise_affaissement(correction)
    return q_mesure, correction, ecart < 3.0


def descente_verticale(ctx, p_cible, R, correction, passes=3, tol_xy=2.5):
    """Descend droit, mesure en bas, REMONTE pour corriger, redescend.

    Corriger lateralement doigts en bas pousse l'objet (constate : balle
    deplacee de 33 mm). Tout recalage se fait donc en hauteur, cible decalee.

    Le biais appris vit dans le contexte : un nouvel essai apres echec reprend
    ou le precedent s'est arrete, il ne repart pas de zero.
    """
    biais = ctx.biais_descente.copy()
    q_mesure, ecart_xy = None, float('inf')
    for i in range(passes):
        cible = resout_ik(p_cible + biais, R)
        if cible is None:
            ctx.note('cible decalee insoluble')
            return q_mesure, False
        q_mesure = va_vers(ctx, np.clip(cible[0] + correction, LIMITES[:, 0], LIMITES[:, 1]),
                           nom=f'descente {i + 1}')
        if q_mesure is None:
            return None, False
        tp = pointe(q_mesure)
        reste = p_cible - tp
        ecart_xy = float(np.linalg.norm(reste[:2]))
        ctx.note(f'  descente passe {i + 1} : ecart XY {ecart_xy:.2f} mm')
        if ecart_xy < tol_xy:
            return q_mesure, True
        biais[:2] += reste[:2]            # retenu meme au dernier essai : il sert au suivant
        ctx.biais_descente = biais.copy()
        if i < passes - 1:
            haut = resout_ik(np.array([tp[0], tp[1], Z_SURVOL]), R)
            if haut is None:
                return q_mesure, False
            va_vers(ctx, np.clip(haut[0] + correction, LIMITES[:, 0], LIMITES[:, 1]),
                    nom='remontee de recalage')
    return q_mesure, ecart_xy < tol_xy


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
                    stabilise=stabilise and i == etapes)
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
        suivant = va_vers(ctx, q_cible, nom=f'palier Z={z:.0f}')
        if suivant is None:
            break
        q = suivant
        if ctx.pont.statut_pince() != 2:
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
    ctx.note(f'la balle a bouge de {ecart:.0f} mm '
             f'({ctx.balle_xy[0]:.0f},{ctx.balle_xy[1]:.0f}) -> ({xy[0]:.0f},{xy[1]:.0f})')
    ctx.balle_xy = xy
    ctx.repart_a_zero()
    ctx.resultats['balle'] = f'({xy[0]:.1f}, {xy[1]:.1f}) mm — suivie'
    return True


def _degagement(ctx):
    """Ecarte le bras jusqu'a ce que la camera revoie la balle.

    L'arducam regarde de dessus : des que le bras s'approche, il s'interpose et
    se cache la balle a lui-meme. Un cycle qui enchaine sans degager mesure une
    balle a moitie occultee — ou n'en trouve plus du tout et conclut a tort
    qu'il n'y en a pas.
    """
    if ctx.detecteur is not None and ctx.detecteur(patience=0.5) is not None:
        ctx.resultats['degagement'] = 'inutile — balle deja visible'
        return 'DETECTION'
    ordre = list(BALAYAGE_J1)
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
        if ctx.detecteur is None or ctx.detecteur() is not None:
            ctx.resultats['degagement'] = f'J1 = {j1:+.0f} deg'
            return 'DETECTION'
        ctx.note(f'  balle invisible depuis J1={j1:+.0f}, on ecarte davantage')
    ctx.resultats['degagement'] = 'invisible depuis toutes les poses'
    ctx.note('balle invisible depuis toutes les poses de degagement')
    return 'ATTENTE'


def _detecte(ctx):
    ctx.pont.envoie('pro_gripper_open')
    time.sleep(2.0)
    # Le bras vient de se degager : c'est le seul moment du cycle ou la camera
    # voit le carton sans obstacle. On le localise MAINTENANT, avant de saisir.
    if not carton_pret(ctx):
        ctx.resultats['verdict'] = 'carton inconnu ou hors d atteinte — cycle non demarre'
        ctx.note('carton ni vu ni memorise a portee — on ne saisit pas la balle')
        return 'ATTENTE'
    vu = ctx.detecteur() if ctx.detecteur else None
    if vu is None:
        ctx.resultats['balle'] = 'non detectee'
        ctx.note('balle non detectee')
        return 'ATTENTE'
    xy = np.asarray(vu[:2], float)
    portee = float(np.hypot(*xy))
    ctx.resultats['balle'] = f'({xy[0]:.1f}, {xy[1]:.1f}) mm'
    ctx.resultats['portee balle'] = f'{portee:.1f} mm'
    if portee > PORTEE_MAX:
        ctx.resultats['verdict'] = f'HORS ENVELOPPE (> {PORTEE_MAX:.0f} mm)'
        ctx.note(f'balle a {portee:.0f} mm — hors enveloppe, cible refusee')
        return 'ATTENTE'
    roulis, R = choisit_roulis(xy, [Z_TRANSFERT, Z_SURVOL, Z_PRISE],
                               prefere=roulis_retenu(ctx, 'balle', xy))
    if R is None:
        ctx.resultats['verdict'] = 'aucun roulis ne resout'
        return 'ATTENTE'
    ctx.roulis_appris[cle_roulis('balle', xy)] = roulis
    ctx.balle_xy, ctx.roulis_balle, ctx.R_balle = xy, roulis, R
    ctx.repart_a_zero()
    ctx.resultats['roulis balle'] = f'{roulis:+.0f} deg'
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


def _descente(ctx):
    cible = np.array([ctx.balle_xy[0], ctx.balle_xy[1], Z_PRISE])
    q, ok = descente_verticale(ctx, cible, ctx.R_balle, ctx.correction)
    if q is not None:
        ctx.resultats['descente'] = f'{np.linalg.norm(pointe(q)[:2] - cible[:2]):.2f} mm'
    if ok:
        return 'SAISIE'
    n = ctx.essai('DESCENTE')
    if n < ESSAIS_MAX:
        ctx.resultats['biais appris'] = f'{ctx.biais_descente[:2].round(1)} mm'
        ctx.note(f'descente ratee, essai {n + 1}/{ESSAIS_MAX} — reprise au recalage, '
                 f'biais garde {ctx.biais_descente[:2].round(1)} mm')
        return 'RECALAGE'
    return 'ECHEC'


def _saisie(ctx):
    ctx.pont.envoie('pro_gripper_angle', angle=20)
    # On attend un statut DECIDE plutot qu'une duree forfaitaire : la pince
    # annonce 0 (« en mouvement ») tant qu'elle serre, et 1/2/3 des qu'elle a
    # conclu. Attendre 2,2 s a tous les coups, c'est attendre le pire cas.
    statut = 0
    for _ in range(6):
        time.sleep(0.4)
        statut = ctx.pont.statut_pince()
        if statut in (1, 2, 3):
            break
    angle = ctx.pont.envoie('get_pro_gripper_angle').split(':')[-1].strip()
    ctx.resultats['pince'] = f'{ETAT_PINCE.get(statut, "?")} (angle {angle})'
    if statut == 2:
        return 'REMONTEE'
    n = ctx.essai('SAISIE')
    if n < ESSAIS_MAX:
        ctx.pont.envoie('pro_gripper_open')
        time.sleep(2.0)
        ctx.note(f'rien saisi, essai {n + 1}/{ESSAIS_MAX} — on remonte et on recale')
        return 'RECALAGE'
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
    ctx.note('objet lache pendant la remontee — on refait la saisie')
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
        ctx.note('prise perdue avant le transfert — on refait la saisie')
        return 'DEGAGEMENT'
    if ctx.carton_xy is None:
        return 'RECHERCHE_CARTON'
    if ctx.R_carton is None:                  # carton designe a la main
        cible, roulis, R = carton_atteignable(ctx, ctx.carton_xy, ctx.carton_polygone)
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
    cible = resout_ik(np.array([ctx.carton_xy[0], ctx.carton_xy[1], Z_LARGAGE]),
                      ctx.R_carton)
    if cible is None or va_vers(ctx, cible[0], nom='largage', stabilise=False) is None:
        ctx.note('descente de largage refusee — objet garde en main')
        return 'ECHEC_PORTANT'
    tp = pointe(ctx.pont.angles())
    ctx.pont.envoie('pro_gripper_open')
    time.sleep(2.5)
    ctx.resultats['largage'] = f'({tp[0]:.1f}, {tp[1]:.1f}) a Z={tp[2]:.1f}'
    ctx.echecs = 0
    return 'RETRAIT'


def _retrait(ctx):
    q = ctx.pont.angles()
    tp = pointe(q)
    R = ctx.R_carton if ctx.R_carton is not None else ctx.R_balle
    if R is not None:
        haut = resout_ik(np.array([tp[0], tp[1], 175.0]), R)
        if haut is not None:
            va_vers(ctx, haut[0], nom='remontee de retrait', stabilise=False)
    va_vers_par_etapes(ctx, POSE_OBSERVATION, nom='retour observation', stabilise=False)
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
            self.ctx.resultats['cycle'] = resume_chrono(self.ctx)
            self.ctx.note(f'CYCLE COMPLET : {self.ctx.resultats["cycle"]}')
            self.ctx.chrono, self.ctx.debut_cycle = {}, 0.0
        self.etat = suivant
        return self.etat


if __name__ == '__main__':
    print('auto-test IK :', 'OK' if autotest_ik() else 'ECHEC — solveur non fiable')
    print(f'deport outil : {TOOL.round(2)}  longueur {np.linalg.norm(TOOL):.2f} mm')
    print(f'{len(ETATS)} etats, {len(TRANSITIONS)} transitions')
