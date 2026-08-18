#!/usr/bin/env python3
"""Superviseur de sûreté — arrête ou ralentit avant que la commande ne parte (§12).

Le superviseur est consulté à CHAQUE période, avant l'envoi. Il ne corrige rien :
il répond OK, RALENTIR ou ARRÊT avec les motifs. C'est délibéré — un superviseur
qui « rattrape » une commande douteuse masque la cause et laisse le système
tourner dans un état qu'on ne comprend plus.

Les neuf conditions du §12 sont couvertes :

    objet perdu par les deux caméras au-delà du délai    -> LOST
    image trop ancienne                                  -> STALE
    saut impossible de la position estimée               -> JUMP
    erreur de reprojection excessive                     -> REPROJECTION
    vitesse de l'objet au-delà de la poursuite           -> TARGET_TOO_FAST
    proximité d'une limite articulaire                   -> JOINT_LIMIT
    risque de collision                                  -> WORKSPACE / TABLE
    divergence de l'erreur                               -> DIVERGENCE
    incohérence commande / mouvement mesuré              -> NOT_FOLLOWING

La dernière est celle qu'on oublie : si le robot répond OK au bridge mais ne
bouge pas (IK firmware qui refuse en silence — cf. l'entête de diff_ik), rien
d'autre ne le détecte. On compare le déplacement réellement mesuré à celui
commandé sur une fenêtre glissante.
"""
from __future__ import annotations

from dataclasses import dataclass, field
from enum import Enum

import numpy as np

# Limites PRATIQUES du MyCobot 320 Pi. L'URDF est plus permissif que le firmware
# réel (notamment J2 : le Pi refuse au-delà de ±137°). On garde 2° de marge au
# niveau du solveur en bornant J2 à ±135° ; le superviseur ajoute encore ses
# marges de ralentissement/arrêt autour de ce domaine déclaré.
JOINT_LIMITS_DEG = np.array([
    (-168.0, 168.0), (-135.0, 135.0), (-150.0, 150.0),
    (-145.0, 145.0), (-165.0, 165.0), (-180.0, 180.0),
], dtype=np.float64)


class Verdict(Enum):
    OK = 'OK'
    SLOW = 'RALENTIR'
    STOP = 'ARRET'


@dataclass(frozen=True)
class SafetyLimits:
    max_lost_time: float = 0.40          # s sans mesure valide avant arrêt (§8.3)
    max_image_age: float = 0.25          # s — image périmée (§11.3)
    max_position_jump: float = 0.30      # m entre deux estimations successives
    max_reprojection_px: float = 8.0
    max_target_speed: float = 0.60       # m/s — au-delà, hors domaine de poursuite
    joint_margin_deg: float = 6.0        # ° avant butée : ralentir
    joint_hard_margin_deg: float = 2.0   # ° avant butée : arrêt
    workspace_min: tuple = (0.05, -0.32, -0.01)   # m, repère base
    workspace_max: tuple = (0.55, 0.32, 0.45)
    table_z: float = 0.0                 # m — plan table
    # Marge réelle validée le 18/08 : à la pose précédente la pince touchait la
    # table malgré la hauteur prédite par la FK/TCP. Conserver au moins 26 mm
    # tant que le TCP vertical n'a pas été recalibré.
    min_clearance: float = 0.026         # m au-dessus de la table hors saisie
    follow_tolerance: float = 0.60       # fraction du pas commandé réellement faite
    follow_grace_s: float = 0.50         # délai mécanique + retour encodeurs (5 Hz)
    follow_window: int = 5               # périodes consécutives avant de conclure


@dataclass
class SafetyState:
    """Ce que le superviseur observe. Tout est optionnel sauf l'horodatage."""
    now: float
    time_since_measurement: float = 0.0
    image_age: float = 0.0
    position_jump: float = 0.0
    reprojection_px: float = 0.0
    target_speed: float = 0.0
    joint_angles_deg: np.ndarray | None = None
    ee_position: np.ndarray | None = None
    commanded_position: np.ndarray | None = None
    diverging: bool = False
    allow_table_contact: bool = False    # True en DESCEND/GRASP : la pince descend


@dataclass
class SafetyReport:
    verdict: Verdict
    reasons: tuple[str, ...] = ()
    speed_scale: float = 1.0             # facteur à appliquer à la commande

    @property
    def stop(self) -> bool:
        return self.verdict is Verdict.STOP


class SafetySupervisor:
    """Évalue l'état avant chaque envoi. Sans mémoire, sauf pour le suivi de commande."""

    def __init__(self, limits: SafetyLimits | None = None):
        self.limits = limits or SafetyLimits()
        self._pending: list[tuple[np.ndarray, np.ndarray, float]] = []
        self._not_following = 0

    def reset(self):
        self._pending.clear()
        self._not_following = 0

    def register_command(self, ee_before, target, now: float = 0.0):
        """Réserve un envoi à vérifier ; False tant que le précédent est en vol."""
        # La commande tourne à 10 Hz mais les encodeurs réels à 5 Hz. Remplacer
        # la référence à chaque envoi revient à poursuivre une cible mouvante :
        # au moment où l'encodeur confirme l'ordre N, on le compare déjà à N+1
        # et on déclenche un faux NOT_FOLLOWING. Une seule commande témoin reste
        # donc en attente jusqu'à ce que son mouvement soit réellement observé.
        if self._pending:
            return False
        self._pending.append((np.asarray(ee_before, dtype=np.float64).copy(),
                              np.asarray(target, dtype=np.float64).copy(), float(now)))
        return True

    def _check_following(self, ee_now, now: float) -> bool:
        """False si le robot n'a pas suivi la dernière commande enregistrée.

        Compare le déplacement réalisé à celui demandé, projeté sur la direction
        demandée : un mouvement latéral parasite ne compte pas comme du suivi.
        """
        if not self._pending or ee_now is None:
            return True
        start, target, sent_at = self._pending[-1]
        commanded = target - start
        norm = float(np.linalg.norm(commanded))
        if norm < 1e-6:
            return True
        achieved = float((np.asarray(ee_now, dtype=np.float64) - start) @ commanded) / norm
        if achieved >= self.limits.follow_tolerance * norm:
            self._not_following = 0
            self._pending.pop(0)
        elif float(now) - sent_at < self.limits.follow_grace_s:
            # Le MyCobot accuse typiquement 200–400 ms entre send_angles et le
            # retour encodeur correspondant. Ce délai n'est pas un échec.
            return True
        else:
            self._not_following += 1
        return self._not_following < self.limits.follow_window

    def check(self, state: SafetyState) -> SafetyReport:
        lim = self.limits
        stop_reasons, slow_reasons = [], []

        if state.time_since_measurement > lim.max_lost_time:
            stop_reasons.append(
                f'LOST cible perdue depuis {state.time_since_measurement:.2f}s '
                f'(> {lim.max_lost_time:.2f}s)')
        if state.image_age > lim.max_image_age:
            stop_reasons.append(
                f'STALE image vieille de {state.image_age:.2f}s '
                f'(> {lim.max_image_age:.2f}s)')
        if state.position_jump > lim.max_position_jump:
            stop_reasons.append(
                f'JUMP saut de {state.position_jump * 1000:.0f}mm '
                f'(> {lim.max_position_jump * 1000:.0f}mm)')
        if state.reprojection_px > lim.max_reprojection_px:
            stop_reasons.append(
                f'REPROJECTION {state.reprojection_px:.1f}px '
                f'(> {lim.max_reprojection_px:.1f}px)')
        if state.target_speed > lim.max_target_speed:
            stop_reasons.append(
                f'TARGET_TOO_FAST {state.target_speed:.2f}m/s '
                f'(> {lim.max_target_speed:.2f}m/s)')
        if state.diverging:
            stop_reasons.append('DIVERGENCE ‖e‖ croît au lieu de décroître')

        if state.joint_angles_deg is not None:
            q = np.asarray(state.joint_angles_deg, dtype=np.float64)
            margin = np.minimum(q - JOINT_LIMITS_DEG[:, 0], JOINT_LIMITS_DEG[:, 1] - q)
            worst = int(np.argmin(margin))
            if margin[worst] < lim.joint_hard_margin_deg:
                stop_reasons.append(
                    f'JOINT_LIMIT J{worst + 1} à {margin[worst]:.1f}° de la butée')
            elif margin[worst] < lim.joint_margin_deg:
                slow_reasons.append(
                    f'JOINT_LIMIT J{worst + 1} à {margin[worst]:.1f}° de la butée')

        target = state.commanded_position
        if target is not None:
            t = np.asarray(target, dtype=np.float64)
            lo = np.asarray(lim.workspace_min, dtype=np.float64)
            hi = np.asarray(lim.workspace_max, dtype=np.float64)
            if np.any(t < lo) or np.any(t > hi):
                stop_reasons.append(
                    f'WORKSPACE cible ({t[0]:.3f},{t[1]:.3f},{t[2]:.3f}) hors gabarit')
            elif not state.allow_table_contact and t[2] < lim.table_z + lim.min_clearance:
                stop_reasons.append(
                    f'TABLE cible à z={t[2] * 1000:.0f}mm hors phase de descente')

        if not self._check_following(state.ee_position, state.now):
            stop_reasons.append(
                f'NOT_FOLLOWING {self._not_following} commandes sans mouvement — '
                f'bridge muet ou servos relâchés ?')

        if stop_reasons:
            return SafetyReport(Verdict.STOP, tuple(stop_reasons), 0.0)
        if slow_reasons:
            return SafetyReport(Verdict.SLOW, tuple(slow_reasons), 0.35)
        return SafetyReport(Verdict.OK, (), 1.0)
