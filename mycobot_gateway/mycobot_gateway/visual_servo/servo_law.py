#!/usr/bin/env python3
"""Loi de commande PBVS : V_EE = V_O − K·e, saturée (§8.3, §9.1).

L'erreur cartésienne est e(t) = P_EE(t) − P_d(t), où P_EE vient des codeurs et
de la cinématique directe, et P_d de la cible mesurée par les caméras. La
commande combine deux termes :

    V_O   anticipation — sans elle, la pince poursuit un objet mobile avec un
          retard permanent proportionnel à sa vitesse (erreur de traînage)
    −K·e  rappel proportionnel vers la cible

Le pilote du MyCobot n'accepte que des positions : la vitesse est donc convertie
en un petit incrément cartésien par période, ce que le §8.3 autorise
explicitement (« ou de petits incréments cartésiens si le pilote du robot
n'accepte que des positions »). Chaque appel produit UN incrément — la boucle
fermée vient de ce que l'appel suivant repart d'une mesure fraîche, jamais d'une
trajectoire pré-calculée.

Deux saturations, pas une : la vitesse (limites du robot) et l'incrément par
période (une période perdue ne doit pas se traduire par un bond). Sans la
seconde, un hoquet du détecteur produit un dt de 0,5 s et un saut de plusieurs
centimètres.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class ServoGains:
    """Gains et limites. Départ volontairement lent — cf. protocole 🐢 Safe start."""
    kp: tuple[float, float, float] = (1.2, 1.2, 0.9)   # 1/s, par axe base
    feedforward: float = 1.0             # 0 = pas d'anticipation, 1 = complète
    max_speed: float = 0.08              # m/s — plafond de la norme de V_EE
    max_step: float = 0.012              # m — déplacement max par période
    deadband: float = 0.0015             # m — sous ce seuil, on n'envoie rien
    approach_kp_scale: float = 0.45      # facteur sur kp en phase fine


@dataclass
class ServoCommand:
    velocity: np.ndarray                 # (3,) m/s demandée
    step: np.ndarray                     # (3,) m à appliquer cette période
    target: np.ndarray                   # (3,) m pose cartésienne à commander
    error_norm: float                    # m
    saturated: bool
    idle: bool                           # True = dans la bande morte, rien à envoyer


def pregrasp_pose(object_position, height: float) -> np.ndarray:
    """P_d = P_objet + [0, 0, h] — la pose de pré-grasp au-dessus de l'objet (§8.2)."""
    p = np.asarray(object_position, dtype=np.float64).copy()
    p[2] += float(height)
    return p


def compute_command(ee_position, desired_position, object_velocity, dt: float,
                    gains: ServoGains, fine: bool = False) -> ServoCommand:
    """Un pas de la loi §8.3.

    Args:
        ee_position: P_EE (3,) m — codeurs + FK, PAS la dernière consigne. Fermer
            la boucle sur la consigne au lieu de la mesure rendrait le système
            ouvert sans que rien ne le signale.
        desired_position: P_d (3,) m — cible déjà compensée en latence.
        object_velocity: V_O (2,) ou (3,) m/s — anticipation ; zéros si immobile.
        dt: période écoulée depuis le dernier envoi (s).
        fine: True en phase FINE_SERVO — gains réduits, pas plus court.
    """
    ee = np.asarray(ee_position, dtype=np.float64)
    target = np.asarray(desired_position, dtype=np.float64)
    v_obj = np.zeros(3)
    v_obj[:len(np.atleast_1d(object_velocity))] = np.asarray(
        object_velocity, dtype=np.float64).ravel()[:3]

    error = ee - target                                    # e = P_EE − P_d
    error_norm = float(np.linalg.norm(error))

    kp = np.asarray(gains.kp, dtype=np.float64)
    if fine:
        kp = kp * gains.approach_kp_scale
    velocity = gains.feedforward * v_obj - kp * error      # V_EE = V_O − K·e

    speed = float(np.linalg.norm(velocity))
    saturated = speed > gains.max_speed
    if saturated and speed > 1e-9:
        velocity = velocity * (gains.max_speed / speed)

    step = velocity * max(dt, 0.0)
    step_norm = float(np.linalg.norm(step))
    max_step = gains.max_step * (0.5 if fine else 1.0)
    if step_norm > max_step and step_norm > 1e-12:
        step = step * (max_step / step_norm)
        saturated = True

    idle = error_norm < gains.deadband
    return ServoCommand(
        velocity=velocity,
        step=np.zeros(3) if idle else step,
        target=ee if idle else ee + step,
        error_norm=error_norm,
        saturated=saturated,
        idle=idle,
    )


class ConvergenceMonitor:
    """Suit ‖e(t)‖ : convergence stable, ou divergence à signaler (§12, §14).

    Deux questions distinctes :
      - `converged` : l'erreur est-elle sous le seuil depuis assez d'images ?
        La temporisation est ce qui empêche une saisie déclenchée par une seule
        mesure bruitée (§12, dernier paragraphe).
      - `diverging`  : l'erreur augmente-t-elle au lieu de diminuer ? C'est le
        symptôme d'un signe faux, d'une extrinsèque périmée ou d'un gain trop
        fort — et ça doit arrêter le robot, pas le laisser insister.
    """

    def __init__(self, threshold: float = 0.006, required_frames: int = 5,
                 history: int = 12, divergence_ratio: float = 1.35):
        self.threshold = threshold
        self.required_frames = required_frames
        self.divergence_ratio = divergence_ratio
        self._history: list[float] = []
        self._history_len = history
        self._below = 0

    def reset(self):
        self._history.clear()
        self._below = 0

    def push(self, error_norm: float):
        self._history.append(float(error_norm))
        if len(self._history) > self._history_len:
            self._history.pop(0)
        self._below = self._below + 1 if error_norm < self.threshold else 0

    @property
    def converged(self) -> bool:
        return self._below >= self.required_frames

    @property
    def diverging(self) -> bool:
        """True si l'erreur a nettement crû par rapport à son minimum récent."""
        if len(self._history) < self._history_len:
            return False
        best = min(self._history)
        if best < self.threshold:
            return False
        return self._history[-1] > best * self.divergence_ratio

    @property
    def last(self) -> float:
        return self._history[-1] if self._history else float('inf')
