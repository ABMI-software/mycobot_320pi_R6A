#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Auto-détection des caméras USB connues + fabrication d'un CameraSpec par
caméra (index V4L2, intrinsèques calibrées rescalées à la résolution de
capture, exposition manuelle, topics ROS2 dédiés).

But : rendre la chaîne DREAM *flexible* — brancher 1 ou 2 caméras et laisser
le launch/dashboard s'adapter tout seul, sans édition manuelle. La fusion
n'améliore la détection que si CHAQUE caméra a des intrinsèques calibrées
(PnP par vue), donc seules les caméras calibrées ci-dessous participent.

Périmètre volontairement limité aux caméras V4L2/USB. L'Astra (oni_grabber,
pas de nœud /dev/video, pas d'intrinsèque PnP) est hors périmètre — voir
CLAUDE.md 2026-07-13 (fusion Astra construite puis retirée : blocage physique
de détection, pas un bug logiciel).

Identité : on lit `v4l2-ctl --list-devices` et on associe une caméra connue
à sa carte par sous-chaîne du nom (ex. "Arducam", "SVPRO"). Le PREMIER
`/dev/videoN` capturable de cette carte est retenu (les caméras UVC exposent
souvent plusieurs nœuds : vidéo + métadonnées).
"""

from __future__ import annotations

import json
import re
import subprocess
from dataclasses import dataclass, field
from pathlib import Path
from typing import List, Optional

import numpy as np

# training/calibration relatif à ce fichier :
# mycobot_gateway/mycobot_gateway/vision/camera_registry.py → repo root = 3 parents up
_REPO_ROOT = Path(__file__).resolve().parents[3]
_CALIBRATION_DIR = _REPO_ROOT / 'training' / 'calibration'

# Résolution de capture commune (camera_publisher). Les intrinsèques calibrées
# à une autre taille (SVPRO à 800x600) sont rescalées vers celle-ci.
CAPTURE_W, CAPTURE_H = 640, 480


@dataclass(frozen=True)
class KnownCamera:
    """Descripteur statique d'une caméra reconnue (identité + calibration +
    topics). Les topics de l'arducam gardent les noms *legacy* (/camera/…,
    /dream/…) pour que la chaîne mono-caméra historique et les docs existantes
    continuent de marcher à l'identique."""
    name: str
    match: List[str]              # sous-chaînes (minuscules) du nom de carte V4L2
    calib_stem: str               # base des fichiers training/calibration/<stem>.meta.json
    manual_exposure: int          # 75 = fixe (arducam) ; -1 = auto/normale
    manual_focus: int             # 90 = verrou focus SVPRO (3cam.py) ; -1 = désactivé
    image_topic: str
    keypoints_topic: str
    status_topic: str
    output_prefix: str            # préfixe des topics publiés par dream_inference


# Caméras calibrées connues. Ordre = priorité (la 1re détectée est la
# caméra "primaire" du dashboard : vue principale + overlay encodeur).
KNOWN_CAMERAS: List[KnownCamera] = [
    KnownCamera(
        name='arducam',
        match=['arducam'],
        calib_stem='cam_3',        # cam_3 = l'intrinsèque arducam réellement
                                   # utilisée par le dashboard (fx≈496, 640x480),
                                   # PAS cam_0. Ne pas changer sans recalibrer.
        manual_exposure=75,        # cf. capture_real_3cam.set_arducam_exposure
        manual_focus=-1,           # focale fixe : pas de verrou focus
        image_topic='/camera/image_raw',
        keypoints_topic='/dream/keypoints',
        status_topic='/dream/status',
        output_prefix='/dream',
    ),
    KnownCamera(
        name='svpro',
        match=['svpro', '5mp'],    # la SVPRO s'énumère « 5MP USB Camera » en V4L2
                                   # (pas « SVPRO ») ; l'arducam est « ..._8mp »
                                   # donc '5mp' les distingue sans ambiguïté
        calib_stem='cam_2',        # SVPRO calibrée à 800x600 → rescalée à 640x480
        manual_exposure=-1,        # AUCUN réglage d'exposition/luminosité : la
        manual_focus=-1,           # SVPRO tourne 100% par défaut (« elle est
                                   # propre »). Ni expo, ni focus/contrast v4l2.
        image_topic='/camera_svpro/image_raw',
        keypoints_topic='/dream_svpro/keypoints',
        status_topic='/dream_svpro/status',
        output_prefix='/dream_svpro',
    ),
]

KNOWN_BY_NAME = {c.name: c for c in KNOWN_CAMERAS}


@dataclass
class CameraSpec:
    """Caméra effectivement détectée + prête à lancer."""
    name: str
    v4l2_index: int
    manual_exposure: int
    manual_focus: int
    image_topic: str
    keypoints_topic: str
    status_topic: str
    output_prefix: str
    K: np.ndarray = field(default=None)          # (3,3) rescalée à la capture
    dist: np.ndarray = field(default=None)        # (N,) coeffs de distorsion
    calib_ok: bool = False


def load_intrinsics(calib_stem: str,
                    target_w: int = CAPTURE_W,
                    target_h: int = CAPTURE_H):
    """(K 3x3, dist) rescalées à target_w×target_h depuis
    training/calibration/<stem>.meta.json, ou (None, None) si absent.

    fx,cx scalent avec la largeur ; fy,cy avec la hauteur (même règle que
    capture_real_3cam.write_ndds_camera_settings)."""
    meta_path = _CALIBRATION_DIR / f'{calib_stem}.meta.json'
    if not meta_path.is_file():
        return None, None
    meta = json.loads(meta_path.read_text())
    r = meta['results']
    fx, fy = float(r['fx']), float(r['fy'])
    cx, cy = float(r['cx']), float(r['cy'])
    calib_w, calib_h = (int(meta['resolution'][0]), int(meta['resolution'][1])) \
        if 'resolution' in meta else (target_w, target_h)
    if (calib_w, calib_h) != (target_w, target_h):
        sx, sy = target_w / calib_w, target_h / calib_h
        fx, cx = fx * sx, cx * sx
        fy, cy = fy * sy, cy * sy
    K = np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64)
    dist = np.array(r.get('dist_coeffs', []), dtype=np.float64)
    return K, dist


def _list_v4l2_devices() -> dict:
    """{nom_de_carte_minuscule: [index_video, ...]} via `v4l2-ctl --list-devices`.

    Sortie type :
        Arducam_8mp: USB Camera (usb-...):
            /dev/video0
            /dev/video1
    """
    try:
        out = subprocess.run(['v4l2-ctl', '--list-devices'],
                             capture_output=True, text=True, timeout=5).stdout
    except (FileNotFoundError, subprocess.SubprocessError):
        return {}

    devices: dict = {}
    current = None
    for line in out.splitlines():
        if not line.strip():
            continue
        if not line.startswith(('\t', ' ')):
            current = line.strip().lower()
            devices[current] = []
        elif current is not None:
            m = re.search(r'/dev/video(\d+)', line)
            if m:
                devices[current].append(int(m.group(1)))
    return devices


def _captures(index: int) -> bool:
    """True si /dev/video<index> délivre réellement une frame (les nœuds méta
    d'une caméra UVC s'ouvrent mais ne lisent rien)."""
    try:
        import cv2
    except ImportError:
        return True  # pas d'OpenCV ici : on fait confiance à l'énumération
    cap = cv2.VideoCapture(index)
    ok = False
    if cap.isOpened():
        ok, _ = cap.read()
    cap.release()
    return bool(ok)


def detect_cameras(names: Optional[List[str]] = None,
                   probe_capture: bool = True) -> List[CameraSpec]:
    """Caméras connues effectivement branchées, dans l'ordre de KNOWN_CAMERAS.

    names : restreint la recherche à ces noms (ex. ['arducam']) ; None = toutes.
    probe_capture : ouvre chaque /dev/video candidat pour confirmer qu'il
        capture (met False dans un launch pour éviter d'ouvrir la caméra deux
        fois — le camera_publisher l'ouvrira ensuite)."""
    wanted = [KNOWN_BY_NAME[n] for n in names] if names else KNOWN_CAMERAS
    devices = _list_v4l2_devices()
    specs: List[CameraSpec] = []

    for cam in wanted:
        card_indices = None
        for card_name, indices in devices.items():
            if any(tok in card_name for tok in cam.match) and indices:
                card_indices = indices
                break
        if not card_indices:
            continue

        chosen = None
        for idx in card_indices:
            if not probe_capture or _captures(idx):
                chosen = idx
                break
        if chosen is None:
            # Aucun nœud sondable : la caméra est probablement déjà OUVERTE
            # (camera_publisher en cours) — occupée ≠ absente. On retient son
            # premier /dev/video ; camera_publisher.find_camera fera le tri.
            chosen = card_indices[0]

        K, dist = load_intrinsics(cam.calib_stem)
        specs.append(CameraSpec(
            name=cam.name,
            v4l2_index=chosen,
            manual_exposure=cam.manual_exposure,
            manual_focus=cam.manual_focus,
            image_topic=cam.image_topic,
            keypoints_topic=cam.keypoints_topic,
            status_topic=cam.status_topic,
            output_prefix=cam.output_prefix,
            K=K, dist=dist, calib_ok=K is not None,
        ))
    return specs


def _main():
    """`python3 camera_registry.py` — diagnostic autonome des caméras vues."""
    specs = detect_cameras()
    if not specs:
        print('Aucune caméra connue détectée (arducam / svpro).')
        return
    for s in specs:
        k = f'fx={s.K[0,0]:.1f} cx={s.K[0,2]:.1f}' if s.calib_ok else 'PAS D\'INTRINSÈQUE'
        print(f'✅ {s.name:8s} /dev/video{s.v4l2_index}  expo={s.manual_exposure:>3}  '
              f'{k}  → {s.keypoints_topic}')
    if len(specs) >= 2:
        print(f'\nFusion possible : {len(specs)} vues calibrées.')
    else:
        print('\nMono-caméra (1 vue).')


if __name__ == '__main__':
    _main()
