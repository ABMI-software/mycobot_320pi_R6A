#!/usr/bin/env python3
"""Suivi de l'objet : Kalman [X, Y, VX, VY] + compensation de latence (§8.1-8.2).

Modèle à vitesse constante sur le plan table. L'état porte la vitesse parce que
la commande en a besoin deux fois : comme terme d'anticipation V_O dans la loi
§8.3, et pour prédire où sera l'objet quand la commande arrivera réellement.

**La prédiction n'est pas une mesure.** Pendant une occultation, `predict` seul
continue de produire une pose — mais c'est de l'extrapolation, pas une boucle
fermée sur une observation actuelle (§8.3, dernier paragraphe). `age` dit depuis
combien de temps la dernière vraie mesure date ; le superviseur s'en sert pour
arrêter le robot. Ne jamais lire la pose sans regarder l'âge.

Le rejet des mesures aberrantes est une distance de Mahalanobis sur l'innovation :
une détection très loin de la trajectoire attendue est refusée (§7.2), mais si
elle se répète elle finit par être acceptée — sinon un objet réellement déplacé
d'un coup verrouillerait le filtre sur une position morte.
"""
from __future__ import annotations

from dataclasses import dataclass

import numpy as np


@dataclass(frozen=True)
class TrackerConfig:
    """Réglages du filtre. Les variances sont en unités SI au carré."""
    process_accel_std: float = 0.35     # m/s² — accélération non modélisée admise
    measurement_std: float = 0.006      # m — bruit d'une détection nominale (6 mm)
    initial_pos_std: float = 0.05       # m — incertitude à la première mesure
    initial_vel_std: float = 0.20       # m/s
    gate_mahalanobis: float = 3.5       # au-delà : mesure suspecte
    gate_max_rejects: int = 4           # rejets consécutifs -> réinitialisation
    max_speed: float = 0.60             # m/s — au-delà, la cible est intenable


class ObjectTracker:
    """Filtre de Kalman à vitesse constante sur (X, Y) dans le repère base.

    Z n'est pas filtré : l'objet est sur la table, sa hauteur est une constante
    connue (plan table) et non une grandeur mesurée bruitée. La filtrer
    ajouterait un état non observable par une caméra monoculaire au-dessus.
    """

    def __init__(self, config: TrackerConfig | None = None):
        self.cfg = config or TrackerConfig()
        self.x = np.zeros(4)                  # [X, Y, VX, VY]
        self.P = np.eye(4)
        self.initialized = False
        self.last_measurement_time = None     # horodatage de la dernière VRAIE mesure
        self.last_predict_time = None
        self._consecutive_rejects = 0
        self.z_plane = 0.0

    @property
    def position(self) -> np.ndarray:
        return self.x[:2].copy()

    @property
    def velocity(self) -> np.ndarray:
        return self.x[2:].copy()

    @property
    def speed(self) -> float:
        return float(np.linalg.norm(self.x[2:]))

    def age(self, now: float) -> float:
        """Secondes écoulées depuis la dernière mesure réellement observée."""
        if self.last_measurement_time is None:
            return float('inf')
        return max(0.0, now - self.last_measurement_time)

    def reset(self):
        self.initialized = False
        self.last_measurement_time = None
        self.last_predict_time = None
        self._consecutive_rejects = 0

    def _initialize(self, xy: np.ndarray, now: float):
        self.x = np.array([xy[0], xy[1], 0.0, 0.0])
        self.P = np.diag([self.cfg.initial_pos_std ** 2] * 2
                         + [self.cfg.initial_vel_std ** 2] * 2)
        self.initialized = True
        self.last_measurement_time = now
        self.last_predict_time = now
        self._consecutive_rejects = 0

    def predict(self, now: float):
        """Avance l'état jusqu'à `now`. Sans effet tant qu'aucune mesure n'a initialisé."""
        if not self.initialized:
            return
        dt = now - self.last_predict_time
        if dt <= 0.0:
            return
        F = np.array([[1, 0, dt, 0], [0, 1, 0, dt], [0, 0, 1, 0], [0, 0, 0, 1]],
                     dtype=np.float64)
        # Bruit de processus à accélération blanche : la forme classique en dt⁴/4,
        # dt³/2, dt². Un bruit constant sous-estimerait l'incertitude après une
        # longue occultation, et le gate rejetterait la mesure de réacquisition.
        q = self.cfg.process_accel_std ** 2
        d4, d3, d2 = dt ** 4 / 4.0, dt ** 3 / 2.0, dt ** 2
        Q = q * np.array([[d4, 0, d3, 0], [0, d4, 0, d3],
                          [d3, 0, d2, 0], [0, d3, 0, d2]], dtype=np.float64)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + Q
        self.last_predict_time = now

    def update(self, xy, now: float, confidence: float = 1.0) -> bool:
        """Intègre une mesure (x, y) en mètres. Retourne False si elle a été rejetée.

        `confidence` ∈ ]0,1] vient de la fusion : elle gonfle la covariance de
        mesure d'une détection douteuse au lieu de la jeter, ce qui laisse le
        filtre en tirer le peu d'information qu'elle porte.
        """
        xy = np.asarray(xy, dtype=np.float64)[:2]
        if not np.all(np.isfinite(xy)):
            return False
        if not self.initialized:
            self._initialize(xy, now)
            return True

        self.predict(now)
        conf = float(np.clip(confidence, 1e-3, 1.0))
        R = np.eye(2) * (self.cfg.measurement_std / conf) ** 2
        H = np.array([[1.0, 0, 0, 0], [0, 1.0, 0, 0]])
        innovation = xy - H @ self.x
        S = H @ self.P @ H.T + R

        distance = float(np.sqrt(innovation @ np.linalg.solve(S, innovation)))
        if distance > self.cfg.gate_mahalanobis:
            self._consecutive_rejects += 1
            if self._consecutive_rejects >= self.cfg.gate_max_rejects:
                # L'objet a réellement sauté (déplacé à la main, ou la première
                # série de mesures était fausse) : repartir de la mesure plutôt
                # que défendre indéfiniment un état devenu faux.
                self._initialize(xy, now)
                return True
            return False

        self._consecutive_rejects = 0
        K = self.P @ H.T @ np.linalg.inv(S)
        self.x = self.x + K @ innovation
        self.P = (np.eye(4) - K @ H) @ self.P
        self.last_measurement_time = now
        return True

    def predict_position(self, horizon: float) -> np.ndarray:
        """Position (x, y) attendue dans `horizon` secondes — compensation §8.2.

        P_prédit = P_O + V_O·τ. À appeler avec la latence totale mesurée
        (exposition + inférence + ROS2 + calcul + réponse mécanique), pas une
        constante devinée : une τ fausse décale la cible proportionnellement à
        la vitesse de l'objet.
        """
        return self.x[:2] + self.x[2:] * float(horizon)

    def is_trackable(self) -> bool:
        """False si l'objet va plus vite que ce que la poursuite admet (§12)."""
        return self.speed <= self.cfg.max_speed


class TargetLock:
    """Verrouillage d'une cible supposée immobile (§7.1).

    Accumule 5 à 10 détections, écarte les aberrantes par MAD, puis fige une
    médiane. Le verrou sert à traverser une occultation courte de la caméra
    supérieure pendant l'approche verticale : le robot continue vers la pose
    figée grâce aux codeurs.

    **Uniquement valable si l'objet ne bouge pas.** Sur une cible mobile, cette
    classe fabrique une position morte vers laquelle le robot fonce — c'est
    exactement le défaut de boucle ouverte du §2.1. La machine à états ne doit
    l'utiliser que sur le chemin « objet immobile ».
    """

    def __init__(self, n_samples: int = 8, mad_threshold: float = 3.0):
        self.n_samples = n_samples
        self.mad_threshold = mad_threshold
        self._samples: list[np.ndarray] = []
        self.locked: np.ndarray | None = None

    def reset(self):
        self._samples.clear()
        self.locked = None

    @property
    def progress(self) -> tuple[int, int]:
        return len(self._samples), self.n_samples

    def add(self, position) -> np.ndarray | None:
        """Ajoute un échantillon ; retourne la pose verrouillée dès qu'elle l'est."""
        p = np.asarray(position, dtype=np.float64)
        if not np.all(np.isfinite(p)):
            return self.locked
        self._samples.append(p.copy())
        if len(self._samples) < self.n_samples:
            return None

        pts = np.array(self._samples)
        median = np.median(pts, axis=0)
        deviation = np.linalg.norm(pts - median, axis=1)
        mad = np.median(deviation)
        # MAD nulle = échantillons identiques (source figée) : tout est inlier.
        keep = pts if mad <= 1e-9 else pts[deviation <= self.mad_threshold * mad]
        if len(keep) < 3:
            self._samples.pop(0)
            return None
        self.locked = np.median(keep, axis=0)
        return self.locked

    def dispersion(self) -> float:
        """Écart-type des échantillons (m) — critère de répétabilité du §14."""
        if len(self._samples) < 2:
            return float('inf')
        pts = np.array(self._samples)
        return float(np.linalg.norm(pts.std(axis=0)))
