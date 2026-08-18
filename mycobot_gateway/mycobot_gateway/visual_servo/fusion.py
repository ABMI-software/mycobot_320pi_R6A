#!/usr/bin/env python3
"""Fusion multi-caméras : chaque vue donne ᴮT_O, on en tire une mesure (§7.2).

Chaque caméra produit indépendamment une estimation dans `base_link` :

    ᴮT_O⁽ⁱ⁾ = ᴮT_Cᵢ · ᶜⁱT_O

Les estimations ne se valent pas — une balle vue de biais à 1,2 m par la SVPRO
est bien moins sûre que la même vue à 1 m d'aplomb par l'arducam. La confiance
agrège les cinq facteurs du §7.2 (score du détecteur, erreur de reprojection,
taille/netteté dans l'image, écart à la dernière mesure valide, ancienneté de
l'image) en un scalaire ∈ [0,1], et une mesure invalide reçoit un poids nul.

Table de sélection (§7.2) :

    arducam seule valide           -> mesure supérieure
    arducam occultée, svpro valide -> mesure latérale
    les deux valides               -> moyenne pondérée par la confiance
    aucune                         -> None ; l'appelant arrête après le délai

La pondération est en 1/σ² : fusionner deux vues ne peut pas donner un résultat
pire que la meilleure des deux, et une vue douteuse ne tire presque rien.
"""
from __future__ import annotations

from dataclasses import dataclass, field

import numpy as np


@dataclass
class CameraMeasurement:
    """Une observation de l'objet par une caméra, déjà exprimée dans base_link."""
    camera: str
    position: np.ndarray                 # (3,) m, repère base
    stamp: float                         # horodatage de l'IMAGE, pas de sa réception
    detector_score: float = 1.0          # ∈[0,1] confiance du détecteur
    reprojection_px: float = 0.0         # erreur de reprojection de la pose objet
    pixel_area: float = 0.0              # px² occupés par l'objet
    sharpness: float = 1.0               # ∈[0,1] netteté (1 = nette)
    valid: bool = True
    confidence: float = field(default=0.0, init=False)


@dataclass(frozen=True)
class FusionConfig:
    max_age: float = 0.25                # s — au-delà, l'image est périmée (§11.3)
    max_reprojection_px: float = 6.0     # au-delà, la pose objet est fausse
    min_pixel_area: float = 40.0         # px² — en dessous, le centroïde est du bruit
    good_pixel_area: float = 400.0       # px² — au-delà, la taille n'aide plus
    max_jump: float = 0.25               # m — écart admis à la dernière mesure valide
    min_confidence: float = 0.05         # en dessous : poids nul
    base_std: float = 0.006              # m — σ d'une mesure de confiance 1
    max_camera_disagreement: float = 0.012  # m — au-delà, ne jamais moyenner


def _age_factor(age: float, max_age: float) -> float:
    """1 pour une image fraîche, décroît linéairement, 0 quand elle est périmée."""
    if age < 0.0:
        return 0.0
    return float(np.clip(1.0 - age / max_age, 0.0, 1.0))


def _reprojection_factor(px: float, limit: float) -> float:
    return float(np.clip(1.0 - px / limit, 0.0, 1.0))


def _area_factor(area: float, min_area: float, good_area: float) -> float:
    if area <= min_area:
        return 0.0
    return float(np.clip((area - min_area) / (good_area - min_area), 0.0, 1.0))


def _jump_factor(position, reference, max_jump: float) -> float:
    """Pénalise une mesure loin de la dernière valide, sans la rejeter d'emblée.

    §7.2 : « Une nouvelle détection très éloignée de la trajectoire attendue ne
    doit pas être acceptée immédiatement. » Le facteur descend à 0 au-delà de
    `max_jump` ; le gate de Kalman fait ensuite le rejet ferme, et finit par
    accepter si la mesure se confirme.
    """
    if reference is None:
        return 1.0
    d = float(np.linalg.norm(np.asarray(position)[:2] - np.asarray(reference)[:2]))
    return float(np.clip(1.0 - d / max_jump, 0.0, 1.0))


def score_measurement(m: CameraMeasurement, now: float, reference, cfg: FusionConfig) -> float:
    """Confiance ∈[0,1] d'une mesure. 0 = inutilisable, poids nul dans la fusion.

    Produit des facteurs, pas moyenne : un seul critère éliminatoire (image
    périmée, objet minuscule, reprojection absurde) doit suffire à annuler la
    mesure. Une moyenne laisserait quatre bons facteurs sauver un mauvais.
    """
    if not m.valid:
        return 0.0
    if not np.all(np.isfinite(m.position)):
        return 0.0
    factors = (
        float(np.clip(m.detector_score, 0.0, 1.0)),
        _age_factor(now - m.stamp, cfg.max_age),
        _reprojection_factor(m.reprojection_px, cfg.max_reprojection_px),
        _area_factor(m.pixel_area, cfg.min_pixel_area, cfg.good_pixel_area),
        float(np.clip(m.sharpness, 0.0, 1.0)),
        _jump_factor(m.position, reference, cfg.max_jump),
    )
    confidence = float(np.prod(factors))
    return confidence if confidence >= cfg.min_confidence else 0.0


@dataclass
class FusionResult:
    position: np.ndarray                  # (3,) m, repère base
    confidence: float                     # ∈]0,1] confiance résultante
    contributing: tuple[str, ...]         # caméras qui ont réellement pesé
    mode: str                             # 'FUSION' | 'MONO' | 'PERDU'
    per_camera: dict[str, float]          # confiance par caméra, diagnostic


class MultiCameraFusion:
    """Combine les mesures d'une même frame et mémorise la dernière valide."""

    def __init__(self, primary: str = 'arducam', config: FusionConfig | None = None):
        self.cfg = config or FusionConfig()
        self.primary = primary
        self.last_valid: np.ndarray | None = None
        self.last_valid_time: float | None = None

    def reset(self):
        self.last_valid = None
        self.last_valid_time = None

    def fuse(self, measurements, now: float) -> FusionResult | None:
        """Mesure fusionnée, ou None si aucune caméra n'est exploitable.

        `now` doit être la même horloge que les `stamp` des mesures — un mélange
        horloge ROS / horloge murale rend toutes les images « périmées ».
        """
        scored = []
        per_camera = {}
        for m in measurements:
            m.confidence = score_measurement(m, now, self.last_valid, self.cfg)
            per_camera[m.camera] = m.confidence
            if m.confidence > 0.0:
                scored.append(m)

        if not scored:
            return FusionResult(
                position=np.full(3, np.nan), confidence=0.0, contributing=(),
                mode='PERDU', per_camera=per_camera)

        # Deux extrinsèques peuvent être individuellement propres en reprojection
        # tout en présentant un biais relatif sur la table. Une moyenne entre deux
        # points distants de plusieurs centimètres ne correspond alors à aucune
        # observation réelle. L'Arducam est la référence XY ; la caméra latérale
        # sert de repli lorsqu'elle est occultée.
        primary = next((m for m in scored if m.camera == self.primary), None)
        if primary is not None and any(
                np.linalg.norm(np.asarray(m.position) - primary.position) >
                self.cfg.max_camera_disagreement
                for m in scored if m is not primary):
            scored = [primary]

        weights = np.array([m.confidence ** 2 for m in scored])  # ∝ 1/σ²
        positions = np.array([np.asarray(m.position, dtype=np.float64) for m in scored])
        fused = (positions * weights[:, None]).sum(axis=0) / weights.sum()

        # σ² de la combinaison pondérée = 1/Σ(1/σᵢ²) : la confiance résultante
        # dépasse celle de la meilleure vue, ce qui est bien l'intérêt de fusionner.
        combined = float(np.sqrt(weights.sum()))
        confidence = float(np.clip(combined, 0.0, 1.0))

        self.last_valid = fused
        self.last_valid_time = now
        names = tuple(m.camera for m in scored)
        return FusionResult(
            position=fused, confidence=confidence, contributing=names,
            mode='FUSION' if len(scored) > 1 else 'MONO', per_camera=per_camera)

    def time_since_valid(self, now: float) -> float:
        if self.last_valid_time is None:
            return float('inf')
        return max(0.0, now - self.last_valid_time)
