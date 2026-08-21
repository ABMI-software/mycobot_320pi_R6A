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
VITESSE = 25
PORTEE_MAX = 335.0      # au-dela, aucun roulis ne resout a hauteur de table

# Pour une sphere, l'orientation des doigts dans le plan horizontal est sans
# importance : ce roulis est un degre de liberte gratuit qui porte la portee
# utile de 300 a 330 mm. Balayage du plus proche du nominal au plus eloigne.
ROULIS = [0, 30, -30, 60, -60, 90, -90, 120]

ETAT_PINCE = {0: 'en mouvement', 1: 'rien saisi', 2: 'objet saisi', 3: 'objet lache'}

POSE_OBSERVATION = np.array([-27.50, -61.61, -41.30, 89.56, 0.43, -79.62])


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


def resout_ik(p_cible, R, tol_mm=1.0, tol_deg=1.0, coude_haut=True):
    """Meilleure solution parmi toutes les amorces, ou None."""
    azimut = float(np.degrees(np.arctan2(p_cible[1], p_cible[0])))
    amorces = list(_AMORCES_MESUREES)
    for q in _AMORCES_MESUREES + _AMORCES_SYNTHETIQUES:
        s = q.copy()
        s[0] = azimut
        amorces.append(s)
    meilleure = None
    for amorce in amorces:
        q = solve_pose(amorce, np.asarray(p_cible) - R @ TOOL, R,
                       rot_weight=400.0, max_joint_step_deg=3.0, iterations=400)
        if coude_haut and q[2] > 0:
            continue
        if not np.all((q >= LIMITES[:, 0]) & (q <= LIMITES[:, 1])):
            continue
        p, Rq = pose_bride(q)
        residu = float(np.linalg.norm(p + Rq @ TOOL - np.asarray(p_cible)))
        ecart = orientation_error_deg(q, R)
        if meilleure is None or residu + ecart < meilleure[1] + meilleure[2]:
            meilleure = (q, residu, ecart)
    if meilleure is None or meilleure[1] > tol_mm or meilleure[2] > tol_deg:
        return None
    return meilleure


def choisit_roulis(p_xy, hauteurs):
    """Roulis le plus proche du nominal resolvant TOUTES les hauteurs voulues."""
    for angle in ROULIS:
        R = orientation(p_xy, angle)
        if all(resout_ik(np.array([p_xy[0], p_xy[1], z]), R) is not None for z in hauteurs):
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
    resultats: dict = field(default_factory=dict)
    journal: list = field(default_factory=list)
    mode_auto: bool = False
    detecteur: object = None          # callable -> (x, y) ou None

    def note(self, texte):
        self.journal.append(texte)
        del self.journal[:-200]


def va_vers(ctx, q_cible, vitesse=VITESSE, nom='', patience=25):
    """Deplacement valide : la trajectoire ne doit jamais descendre en chemin."""
    q0 = ctx.pont.angles()
    minimum = min(garde_au_sol(q0 * (1 - t) + q_cible * t) for t in np.linspace(0, 1, 61))
    seuil = min(GARDE_MIN, garde_au_sol(q0) - 2.0)
    if minimum < seuil:
        ctx.note(f'{nom} REFUSE — garde {minimum:.1f} mm sous le seuil {seuil:.1f}')
        return None
    ctx.pont.envoie('send_angles', angles=[round(float(v), 2) for v in q_cible],
                    speed=vitesse)
    for _ in range(patience):
        time.sleep(0.8)
        if np.abs(ctx.pont.angles() - q_cible).max() < 1.2:
            break
    time.sleep(1.0)
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
    """
    biais = np.zeros(3)
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
        if i < passes - 1:
            biais[:2] += reste[:2]
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

ETATS = ['ATTENTE', 'DETECTION', 'APPROCHE', 'RECALAGE', 'DESCENTE',
         'SAISIE', 'REMONTEE', 'TRANSFERT', 'LARGAGE', 'RETRAIT', 'ECHEC']

# (depart, arrivee, condition, action) — pour le graphe du tableau de bord.
TRANSITIONS = [
    ('ATTENTE', 'DETECTION', 'demarrer', 'pince ouverte'),
    ('DETECTION', 'APPROCHE', 'balle vue & portee OK', 'roulis choisi'),
    ('DETECTION', 'ATTENTE', 'hors enveloppe', 'refus'),
    ('APPROCHE', 'RECALAGE', 'arrive a 110 mm', ''),
    ('RECALAGE', 'DESCENTE', 'ecart < 1 mm', 'affaissement appris'),
    ('RECALAGE', 'ECHEC', 'ne converge pas', ''),
    ('DESCENTE', 'SAISIE', 'ecart XY < 2.5 mm', ''),
    ('DESCENTE', 'ECHEC', 'trop excentre', 'pas de fermeture'),
    ('SAISIE', 'REMONTEE', 'statut == 2', ''),
    ('SAISIE', 'RETRAIT', 'statut != 2', 'rien saisi'),
    ('REMONTEE', 'TRANSFERT', 'prise tenue', ''),
    ('REMONTEE', 'ECHEC', 'prise perdue', ''),
    ('TRANSFERT', 'LARGAGE', 'au-dessus du carton', ''),
    ('LARGAGE', 'RETRAIT', '', 'pince ouverte'),
    ('RETRAIT', 'DETECTION', 'mode auto', 'boucle'),
    ('RETRAIT', 'ATTENTE', 'mode manuel', ''),
    ('ECHEC', 'ATTENTE', 'acquitte', ''),
]


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
    ctx.resultats['roulis balle'] = f'{roulis:+.0f} deg'
    ctx.resultats['verdict'] = 'atteignable'
    return 'APPROCHE'


def _approche(ctx):
    cible = resout_ik(np.array([ctx.balle_xy[0], ctx.balle_xy[1], Z_SURVOL]), ctx.R_balle)
    if cible is None or va_vers(ctx, cible[0], nom='approche') is None:
        return 'ECHEC'
    return 'RECALAGE'


def _recalage(ctx):
    cible = np.array([ctx.balle_xy[0], ctx.balle_xy[1], Z_SURVOL])
    q, correction, ok = converge(ctx, cible, ctx.R_balle)
    ctx.correction = correction
    ctx.resultats['recalage XY'] = f'{np.linalg.norm(pointe(q) - cible):.2f} mm'
    return 'DESCENTE' if ok else 'ECHEC'


def _descente(ctx):
    cible = np.array([ctx.balle_xy[0], ctx.balle_xy[1], Z_PRISE])
    q, ok = descente_verticale(ctx, cible, ctx.R_balle, ctx.correction)
    if q is not None:
        ctx.resultats['descente'] = f'{np.linalg.norm(pointe(q)[:2] - cible[:2]):.2f} mm'
    return 'SAISIE' if ok else 'ECHEC'


def _saisie(ctx):
    ctx.pont.envoie('pro_gripper_angle', angle=20)
    time.sleep(2.2)
    statut = ctx.pont.statut_pince()
    angle = ctx.pont.envoie('get_pro_gripper_angle').split(':')[-1].strip()
    ctx.resultats['pince'] = f'{ETAT_PINCE.get(statut, "?")} (angle {angle})'
    return 'REMONTEE' if statut == 2 else 'RETRAIT'


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
    return 'DETECTION' if ctx.mode_auto else 'ATTENTE'


ACTIONS = {
    'ATTENTE': lambda ctx: 'DETECTION',
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
