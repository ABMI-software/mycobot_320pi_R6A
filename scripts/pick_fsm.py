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
from dataclasses import dataclass, field
from pathlib import Path

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
ESSAIS_MAX = 3
VITESSE = 25
PORTEE_MAX = 335.0      # au-dela, aucun roulis ne resout a hauteur de table

# Pour une sphere, l'orientation des doigts dans le plan horizontal est sans
# importance : ce roulis est un degre de liberte gratuit qui porte la portee
# utile de 300 a 330 mm. Balayage du plus proche du nominal au plus eloigne.
ROULIS = [0, 30, -30, 60, -60, 90, -90, 120]

ETAT_PINCE = {0: 'en mouvement', 1: 'rien saisi', 2: 'objet saisi', 3: 'objet lache'}

SEUIL_DEPLACEMENT = 8.0   # mm — au-dela, la balle a bouge : on refait la cible

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


def choisit_roulis(p_xy, hauteurs):
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
    for reglage in reglages:
        for angle in ROULIS:
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
    roulis_balle: float = 0.0
    roulis_carton: float = 0.0
    R_balle: np.ndarray = None
    R_carton: np.ndarray = None
    correction: np.ndarray = field(default_factory=lambda: np.zeros(6))
    biais_descente: np.ndarray = field(default_factory=lambda: np.zeros(3))
    essais: dict = field(default_factory=dict)
    resultats: dict = field(default_factory=dict)
    journal: list = field(default_factory=list)
    mode_auto: bool = False
    detecteur: object = None          # callable(patience=...) -> (x, y) ou None

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


def va_vers(ctx, q_cible, vitesse=VITESSE, nom='', patience=60):
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
    for _ in range(patience):
        time.sleep(0.35)
        if np.abs(ctx.pont.angles() - q_cible).max() < 1.2:
            break
    time.sleep(1.0)                       # l'affaissement doit s'etablir avant mesure
    return ctx.pont.angles()


def converge(ctx, p_cible, R, passes=4, tol=1.0):
    """Compense l'affaissement en reinjectant l'ecart articulaire mesure.

    `send_angles` n'atteint pas la consigne : ~+1.9 deg sur J2, ce qui deplace la
    pointe ET fait pivoter l'outil. Rend (q, correction, converge).
    """
    q_mesure = ctx.pont.angles()
    q_commande = q_mesure.copy()
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
    return q_mesure, q_commande - q_mesure, ecart < 3.0


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


def monte_par_paliers(ctx, R, hauteurs=(30.0, 80.0, 130.0, Z_TRANSFERT)):
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
         'SAISIE', 'REMONTEE', 'TRANSFERT', 'LARGAGE', 'RETRAIT', 'ECHEC']

# (depart, arrivee, condition, action) — pour le graphe du tableau de bord.
TRANSITIONS = [
    ('ATTENTE', 'DEGAGEMENT', 'demarrer', ''),
    ('DEGAGEMENT', 'DETECTION', 'balle visible', ''),
    ('DEGAGEMENT', 'ATTENTE', 'invisible partout', 'refus'),
    ('DETECTION', 'APPROCHE', 'balle vue & portee OK', 'roulis choisi'),
    ('DETECTION', 'ATTENTE', 'hors enveloppe', 'refus'),
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
    ('REMONTEE', 'TRANSFERT', 'prise tenue', ''),
    ('REMONTEE', 'ECHEC', 'prise perdue', ''),
    ('TRANSFERT', 'LARGAGE', 'au-dessus du carton', ''),
    ('LARGAGE', 'RETRAIT', '', 'pince ouverte'),
    ('RETRAIT', 'DEGAGEMENT', 'mode auto', 'boucle'),
    ('RETRAIT', 'ATTENTE', 'mode manuel', ''),
    ('ECHEC', 'ATTENTE', 'acquitte', ''),
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
        if va_vers(ctx, pose, nom=f'degagement J1={j1:+.0f}') is None:
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
    roulis, R = choisit_roulis(xy, [Z_TRANSFERT, Z_SURVOL, Z_PRISE])
    if R is None:
        ctx.resultats['verdict'] = 'aucun roulis ne resout'
        return 'ATTENTE'
    ctx.balle_xy, ctx.roulis_balle, ctx.R_balle = xy, roulis, R
    ctx.repart_a_zero()
    ctx.resultats['roulis balle'] = f'{roulis:+.0f} deg'
    ctx.resultats['verdict'] = 'atteignable'
    return 'APPROCHE'


def _approche(ctx):
    if cible_a_bouge(ctx):
        return 'DETECTION'                # roulis et enveloppe a rejuger
    cible = resout_ik(np.array([ctx.balle_xy[0], ctx.balle_xy[1], Z_SURVOL]), ctx.R_balle)
    if cible is None or va_vers(ctx, cible[0], nom='approche') is None:
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
    time.sleep(2.2)
    statut = ctx.pont.statut_pince()
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
    return 'TRANSFERT' if tenue else 'ECHEC'


def _transfert(ctx):
    if ctx.carton_xy is None:
        ctx.note('aucun carton defini')
        return 'ECHEC'
    roulis, R = choisit_roulis(ctx.carton_xy, [Z_TRANSFERT, Z_LARGAGE])
    if R is None:
        ctx.resultats['verdict'] = 'carton hors d atteinte'
        return 'ECHEC'
    ctx.roulis_carton, ctx.R_carton = roulis, R
    ctx.resultats['roulis carton'] = f'{roulis:+.0f} deg'
    cible = resout_ik(np.array([ctx.carton_xy[0], ctx.carton_xy[1], Z_TRANSFERT]), R)
    if cible is None or va_vers(ctx, cible[0], nom='transfert') is None:
        return 'ECHEC'
    return 'LARGAGE' if ctx.pont.statut_pince() == 2 else 'ECHEC'


def _largage(ctx):
    cible = resout_ik(np.array([ctx.carton_xy[0], ctx.carton_xy[1], Z_LARGAGE]),
                      ctx.R_carton)
    if cible is None or va_vers(ctx, cible[0], nom='largage') is None:
        return 'ECHEC'
    tp = pointe(ctx.pont.angles())
    ctx.pont.envoie('pro_gripper_open')
    time.sleep(2.5)
    ctx.resultats['largage'] = f'({tp[0]:.1f}, {tp[1]:.1f}) a Z={tp[2]:.1f}'
    return 'RETRAIT'


def _retrait(ctx):
    q = ctx.pont.angles()
    tp = pointe(q)
    R = ctx.R_carton if ctx.R_carton is not None else ctx.R_balle
    if R is not None:
        haut = resout_ik(np.array([tp[0], tp[1], 175.0]), R)
        if haut is not None:
            va_vers(ctx, haut[0], nom='remontee de retrait')
    va_vers(ctx, POSE_OBSERVATION, nom='retour observation')
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
    'TRANSFERT': _transfert,
    'LARGAGE': _largage,
    'RETRAIT': _retrait,
    'ECHEC': lambda ctx: 'ATTENTE',
}


class MachineEtats:
    def __init__(self, ctx):
        self.ctx = ctx
        self.etat = 'ATTENTE'

    def pas(self):
        """Execute l'etat courant et passe au suivant. Rend le nouvel etat."""
        self.ctx.note(f'--- {self.etat} ---')
        try:
            suivant = ACTIONS[self.etat](self.ctx)
        except Exception as erreur:                       # le robot doit s'arreter net
            self.ctx.note(f'ERREUR dans {self.etat} : {erreur}')
            self.etat = 'ECHEC'
            return self.etat
        self.etat = suivant
        return self.etat


if __name__ == '__main__':
    print('auto-test IK :', 'OK' if autotest_ik() else 'ECHEC — solveur non fiable')
    print(f'deport outil : {TOOL.round(2)}  longueur {np.linalg.norm(TOOL):.2f} mm')
    print(f'{len(ETATS)} etats, {len(TRANSITIONS)} transitions')
