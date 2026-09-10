#!/usr/bin/env python3
"""Calibration extrinsèque caméra→base par les 4 ArUco fixes — 16 coins, validée.

Remplace `calibrate_arducam_markers.py` pour l'asservissement visuel. Ce qui
change, et pourquoi (cf. § 5.3 du dossier asservissement visuel) :

  - **16 coins au lieu de 4 centres.** Quatre centres, c'est le minimum absolu
    d'un PnP planaire : le système est quasi-déterminé, le résidu tombe à ~0 et
    ne mesure donc plus rien. `svpro_top_extrinsic_markers.yaml` en est
    l'illustration — RMS 2.7e-08 px sur 3 marqueurs, c'est-à-dire un solveur qui
    interpole exactement 3 points, pas une calibration de qualité 3e-08 px.
    Seize coins donnent 32 équations pour 6 inconnues : le RMS redevient une
    mesure honnête et l'orientation dans le plan devient observable.
  - **RANSAC puis RefineLM** au lieu d'un solvePnP sec : un coin mal détecté
    (reflet, marqueur écorné) ne tire plus toute la pose.
  - **Validation leave-one-marker-out** : la pose est réajustée sur 3 marqueurs
    et l'erreur est mesurée sur le 4e, qui n'a pas servi à l'estimation. C'est
    la seule façon de distinguer « le modèle colle à ses propres points » de
    « le modèle prédit un point neuf ». L'erreur est reportée en px ET en mm sur
    le plan table — le mm est ce qui décide si la pince attrape ou rate.

L'ordre des coins n'est PAS supposé : `cv2.aruco` les renvoie dans l'ordre de
l'image (qui dépend de l'orientation du marqueur et du point de vue), alors que
le modèle 3D les génère dans le repère base. Les deux sont appariés par
projection via une pose initiale issue des centres, puis plus proche voisin —
un marqueur tourné de 90° dans le plan est donc apparié correctement sans
intervention.

Usage (venv_dream) :
    # arducam, capture live
    python calibrate_camera_base_extrinsic.py --camera arducam

    # svpro sur une photo déjà prise
    python calibrate_camera_base_extrinsic.py --camera svpro --image svpro.png
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
_REPO = HERE.parents[1]
sys.path.insert(0, str(_REPO / 'mycobot_gateway'))

from mycobot_gateway.vision.camera_registry import (  # noqa: E402
    KNOWN_BY_NAME, detect_cameras, load_intrinsics)

# Ni le stem intrinsèque ni l'index V4L2 ne sont écrits en dur : les deux
# viennent de camera_registry, qui est la source qu'utilisera l'asservissement.
# L'index change d'un branchement à l'autre (arducam vue en video3 et non video0
# le 18/08) et calibrer avec une autre intrinsèque que celle du runtime déplace
# la cible de plusieurs centimètres (cf. note cam_0/cam_3 dans
# pick_and_place_vision_live.py).
CAMERAS = {
    'arducam': {'out': 'arducam_extrinsic_servo.yaml'},
    'svpro': {'out': 'svpro_extrinsic_servo.yaml'},
}
CAPTURE_W, CAPTURE_H = 640, 480


def resolve_camera(name: str, forced_index: int | None):
    """(index V4L2, stem intrinsèque, exposition) — auto-détecté sauf index forcé."""
    known = KNOWN_BY_NAME[name]
    if forced_index is not None:
        return forced_index, known.calib_stem, known.manual_exposure
    specs = detect_cameras(names=[name], probe_capture=False)
    if not specs:
        raise SystemExit(
            f'caméra « {name} » non détectée par v4l2-ctl — branchée ? '
            f'Sinon force l\'index avec --index.')
    return specs[0].v4l2_index, known.calib_stem, known.manual_exposure


def load_markers(yaml_path: Path) -> tuple[dict, float, str, dict]:
    """{id: (x,y,z) base_link en MÈTRES}, taille (m), frame_id, {id: yaw_deg}.

    Le YAML se lit en millimètres (`units: mm`, `marker_size_mm`) parce que
    c'est l'unité dans laquelle les mesures sont prises au mètre ruban ; tout
    est converti en mètres ici, une fois, car la géométrie en aval (OpenCV, FK,
    repère base) travaille en mètres. `units: m` reste accepté pour les anciens
    fichiers.

    `marker_yaw_deg` est optionnel : absent, l'orientation de chaque marqueur
    est déduite de l'image. Un marqueur posé de travers n'a donc rien à mesurer.
    """
    d = yaml.safe_load(Path(yaml_path).read_text())
    scale = 0.001 if str(d.get('units', 'm')).lower() == 'mm' else 1.0
    centers = {int(k): np.asarray(v, dtype=np.float64) * scale
               for k, v in d['markers'].items()}
    if 'marker_size_mm' in d:
        size = float(d['marker_size_mm']) * 0.001
    else:
        size = float(d['marker_size_m'])
    yaws = {int(k): float(v) for k, v in (d.get('marker_yaw_deg') or {}).items()}
    return centers, size, d.get('frame_id', 'base_link'), yaws


def measure_marker_size(img_corners, K, dist, T_world_cam, plane_z) -> float:
    """Côté du marqueur MESURÉ sur l'image (m), via ses coins rétroprojetés.

    Non circulaire : la pose passée en argument vient des CENTRES des marqueurs,
    et un centre ne dépend pas de la taille du marqueur. On peut donc s'en servir
    pour mesurer cette taille, puis la réinjecter dans les coins 3D.

    Sert d'abord à trancher : `workspace_markers.yaml` a longtemps porté à la
    fois `marker_size_m: 0.080` et un commentaire « marker_size = 50 mm ». Tant
    qu'on n'utilisait que les centres l'ambiguïté était sans effet ; avec les 16
    coins elle décale chaque coin de la moitié de l'erreur.
    """
    plane = [pixel_to_plane(c, K, dist, T_world_cam, plane_z)[:2] for c in img_corners]
    edges = [float(np.linalg.norm(plane[(i + 1) % 4] - plane[i])) for i in range(4)]
    return float(np.mean(edges))


def marker_corners_3d(center: np.ndarray, size: float, yaw_deg: float) -> np.ndarray:
    """Les 4 coins d'un marqueur posé à plat, dans le repère base (m).

    Ordre : (-,+), (+,+), (+,-), (-,-) en (dx,dy) — sans importance, l'appariement
    avec l'image se fait par projection. `yaw_deg` = rotation du marqueur autour
    de Z ; 0 = bords alignés sur les axes base.
    """
    h = size / 2.0
    offsets = np.array([[-h, +h], [+h, +h], [+h, -h], [-h, -h]], dtype=np.float64)
    c, s = np.cos(np.radians(yaw_deg)), np.sin(np.radians(yaw_deg))
    R = np.array([[c, -s], [s, c]], dtype=np.float64)
    rotated = offsets @ R.T
    corners = np.tile(center, (4, 1))
    corners[:, :2] += rotated
    return corners


def detect_markers(bgr: np.ndarray) -> dict[int, np.ndarray]:
    """{id: (4,2) coins image}, raffinés au sous-pixel.

    CLAHE + fenêtres adaptatives larges : les marqueurs vus de loin par la SVPRO
    sont petits et peu contrastés.
    """
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
    gray = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8)).apply(gray)
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 53
    params.adaptiveThreshWinSizeStep = 4
    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000), params)
    corners, ids, _ = detector.detectMarkers(gray)
    if ids is None:
        return {}
    # DICT_4X4_1000 a une faible distance de Hamming : du bois, un câble ou un
    # vêtement se décodent régulièrement en marqueurs fantômes (ids 190 et 379
    # observés le 18/08). Filtrés par identifiant plus loin — sauf si un fantôme
    # tombe sur un ID RÉEL, auquel cas le dict silencieusement en garderait un
    # seul, choisi par l'ordre de détection. On écarte donc tout ID vu deux fois.
    flat = [int(i) for i in ids.flatten()]
    duplicates = {i for i in flat if flat.count(i) > 1}
    if duplicates:
        print(f'  ⚠ identifiant(s) détecté(s) plusieurs fois, écarté(s) : '
              f'{sorted(duplicates)}')
    return {int(i): c.reshape(4, 2).astype(np.float64)
            for c, i in zip(corners, flat) if i not in duplicates}


def estimate_marker_yaw(img_corners, K, dist, T_world_cam, plane_z) -> float:
    """Orientation d'un marqueur dans le plan, déduite de l'image (degrés).

    Les marqueurs sont posés à la main : exiger leur yaw mesuré au rapporteur
    serait une source d'erreur de plus. On rétroprojette les coins détectés sur
    le plan du marqueur, et l'arête coin0→coin1 donne directement l'orientation.

    `cv2.aruco` ordonne toujours les coins dans le repère PROPRE du marqueur
    (coin haut-gauche en premier, puis sens horaire vu de face) : l'arête 0→1
    est donc la même arête physique quelle que soit la rotation, et c'est ce qui
    rend cette mesure fiable. `marker_corners_3d` place cette même arête le long
    de +X à yaw=0.
    """
    p0 = pixel_to_plane(img_corners[0], K, dist, T_world_cam, plane_z)
    p1 = pixel_to_plane(img_corners[1], K, dist, T_world_cam, plane_z)
    edge = p1[:2] - p0[:2]
    return float(np.degrees(np.arctan2(edge[1], edge[0])))


def _pose_from_centers(obj_centers, img_centers, K, dist):
    """Pose grossière à partir des centres — sert uniquement à apparier les coins."""
    flag = cv2.SOLVEPNP_IPPE if len(obj_centers) >= 4 else cv2.SOLVEPNP_SQPNP
    ok, rvec, tvec = cv2.solvePnP(obj_centers, img_centers, K, dist, flags=flag)
    if not ok:
        raise SystemExit('solvePnP sur les centres a échoué : marqueurs mal détectés ?')
    return rvec, tvec


def match_corners(obj_corners, img_corners, rvec, tvec, K, dist):
    """Réordonne `img_corners` (ordre image) pour suivre `obj_corners` (ordre base).

    Chaque coin 3D est projeté avec la pose grossière ; on lui attribue le coin
    image le plus proche, en interdisant les doublons. Sur un marqueur de 80 mm
    les 4 coins sont séparés de dizaines de pixels — l'appariement est sans
    ambiguïté même avec une pose initiale médiocre.
    """
    proj, _ = cv2.projectPoints(obj_corners, rvec, tvec, K, dist)
    proj = proj.reshape(-1, 2)
    order, taken = [], set()
    for p in proj:
        d = np.linalg.norm(img_corners - p, axis=1)
        for idx in np.argsort(d):
            if idx not in taken:
                order.append(int(idx))
                taken.add(int(idx))
                break
    return img_corners[order]


# Seuil RANSAC volontairement large : ici RANSAC ne sert qu'à écarter les fautes
# GROSSIÈRES (détection fantôme, marqueur à moitié occulté). Un seuil serré est
# un piège — mesuré le 18/08 : avec le marqueur 19 mal positionné de 11 mm, ses
# coins reprojetaient à ~5,5 px, juste au bord d'un seuil à 4 px. Selon le bruit
# de l'image ils passaient inliers ou outliers, et la pose sautait de plusieurs
# CENTIMÈTRES d'une capture à l'autre, sans que le RMS le laisse voir. Un
# marqueur mal mesuré doit apparaître dans le tableau des résidus en mm pour
# être corrigé, pas disparaître silencieusement du calcul.
RANSAC_REPROJ_PX = 12.0


def fit_pose(obj_pts, img_pts, K, dist, use_ransac=True):
    """(rvec, tvec, inliers) — RANSAC puis raffinement Levenberg-Marquardt."""
    if use_ransac and len(obj_pts) >= 6:
        ok, rvec, tvec, inliers = cv2.solvePnPRansac(
            obj_pts, img_pts, K, dist, flags=cv2.SOLVEPNP_IPPE,
            reprojectionError=RANSAC_REPROJ_PX, confidence=0.999,
            iterationsCount=500)
        if not ok:
            raise SystemExit('solvePnPRansac a échoué.')
        inliers = np.arange(len(obj_pts)) if inliers is None else inliers.ravel()
    else:
        ok, rvec, tvec = cv2.solvePnP(obj_pts, img_pts, K, dist,
                                      flags=cv2.SOLVEPNP_IPPE)
        if not ok:
            raise SystemExit('solvePnP a échoué.')
        inliers = np.arange(len(obj_pts))
    rvec, tvec = cv2.solvePnPRefineLM(obj_pts[inliers], img_pts[inliers],
                                      K, dist, rvec, tvec)
    return rvec, tvec, inliers


def reproj_errors(obj_pts, img_pts, rvec, tvec, K, dist) -> np.ndarray:
    proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, K, dist)
    return np.linalg.norm(proj.reshape(-1, 2) - img_pts, axis=1)


def to_matrices(rvec, tvec) -> tuple[np.ndarray, np.ndarray]:
    """(T_cam_world, T_world_cam) — base→caméra et son inverse."""
    R, _ = cv2.Rodrigues(rvec)
    T_cam_world = np.eye(4)
    T_cam_world[:3, :3] = R
    T_cam_world[:3, 3] = tvec.ravel()
    return T_cam_world, np.linalg.inv(T_cam_world)


def pixel_to_plane(uv, K, dist, T_world_cam, plane_z) -> np.ndarray:
    """Pixel → point du plan z=plane_z (repère base). Miroir de object_localizer."""
    undist = cv2.undistortPoints(
        np.asarray(uv, dtype=np.float64).reshape(1, 1, 2), K, dist).reshape(2)
    d_cam = np.array([undist[0], undist[1], 1.0])
    origin = T_world_cam[:3, 3]
    d_world = T_world_cam[:3, :3] @ d_cam
    s = (plane_z - origin[2]) / d_world[2]
    return origin + s * d_world


def validate_leave_one_out(per_marker, K, dist, plane_z):
    """Erreur sur un marqueur EXCLU de l'estimation — px et mm dans le plan table.

    C'est le seul chiffre qui dit quelque chose sur un point neuf : le RMS global
    mesure à quel point la pose colle aux points qui l'ont produite.
    """
    ids = sorted(per_marker)
    rows = []
    for held in ids:
        train = [i for i in ids if i != held]
        if len(train) < 3:
            continue
        obj = np.vstack([per_marker[i]['obj'] for i in train])
        img = np.vstack([per_marker[i]['img'] for i in train])
        rvec, tvec, _ = fit_pose(obj, img, K, dist, use_ransac=False)
        err_px = reproj_errors(per_marker[held]['obj'], per_marker[held]['img'],
                               rvec, tvec, K, dist)
        _, T_world_cam = to_matrices(rvec, tvec)
        centre_img = per_marker[held]['img'].mean(axis=0)
        predicted = pixel_to_plane(centre_img, K, dist, T_world_cam, plane_z)
        truth = per_marker[held]['obj'].mean(axis=0)
        rows.append({
            'marker': held,
            'reproj_px': float(np.sqrt((err_px ** 2).mean())),
            'plane_error_mm': float(np.linalg.norm(predicted[:2] - truth[:2]) * 1000.0),
        })
    return rows


def capture_frame(camera: str, index: int, exposure: int) -> np.ndarray:
    cap = cv2.VideoCapture(index, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise SystemExit(f'/dev/video{index} illisible — caméra prise par un autre '
                         f'process (camera_publisher ?) ou index faux.')
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAPTURE_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAPTURE_H)
    if exposure > 0:
        cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
        cap.set(cv2.CAP_PROP_EXPOSURE, exposure)
    frame = None
    for _ in range(10):
        ok, frame = cap.read()
        if not ok:
            frame = None
    cap.release()
    if frame is None:
        raise SystemExit(f'aucune frame lue sur {camera}')
    return frame


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--camera', choices=sorted(CAMERAS), required=True)
    ap.add_argument('--markers', default=str(HERE / 'workspace_markers.yaml'))
    ap.add_argument('--image', help='photo existante ; sinon capture live')
    ap.add_argument('--index', type=int, help='index V4L2 (défaut : celui de la caméra)')
    ap.add_argument('--out', help='YAML de sortie (défaut : celui de la caméra)')
    ap.add_argument('--table-z', type=float, default=0.0,
                    help='hauteur du plan table dans le repère base (m)')
    ap.add_argument('--exclude', type=int, nargs='*', default=[],
                    help='ID(s) à écarter — pour tester une mesure suspecte')
    ap.add_argument('--frames', type=int, default=10,
                    help='images cumulées dans un seul ajustement (caméra fixe)')
    args = ap.parse_args()

    cfg = CAMERAS[args.camera]
    out_path = Path(args.out) if args.out else HERE / cfg['out']

    index, calib_stem, exposure = resolve_camera(args.camera, args.index)
    K, dist = load_intrinsics(calib_stem, CAPTURE_W, CAPTURE_H)
    if K is None:
        raise SystemExit(f'intrinsèque {calib_stem} introuvable — calibre '
                         f'la caméra avant son extrinsèque.')
    print(f'Caméra {args.camera} : /dev/video{index}, intrinsèque {calib_stem}')
    centers_world, size, frame_id, yaws = load_markers(Path(args.markers))

    # Plusieurs images cumulées dans UN ajustement : la caméra est fixe, chaque
    # image apporte un tirage indépendant du bruit de localisation des coins. La
    # dispersion de la pose décroît alors en 1/√N, alors qu'une image unique
    # laisse la PnP planaire osciller de plusieurs centimètres (mesuré).
    if args.image:
        frames = [cv2.imread(args.image)]
        if frames[0] is None:
            raise SystemExit(f'image introuvable : {args.image}')
    else:
        frames = [capture_frame(args.camera, index, exposure)
                  for _ in range(max(1, args.frames))]
    bgr = frames[0]
    if bgr.shape[1] != CAPTURE_W or bgr.shape[0] != CAPTURE_H:
        raise SystemExit(f'image {bgr.shape[1]}x{bgr.shape[0]} ≠ intrinsèque '
                         f'{CAPTURE_W}x{CAPTURE_H} — recalibre ou recapture.')

    per_frame = [detect_markers(f) for f in frames]
    detected = per_frame[0]
    # Un marqueur n'est retenu que s'il est vu sur TOUTES les images : un
    # marqueur intermittent introduirait un biais entre les images qui l'ont et
    # celles qui ne l'ont pas.
    seen_everywhere = set(per_frame[0])
    for d in per_frame[1:]:
        seen_everywhere &= set(d)
    usable = sorted(i for i in centers_world
                    if i in seen_everywhere and i not in args.exclude)
    if len(frames) > 1:
        print(f'Images cumulées     : {len(frames)}')
    if args.exclude:
        print(f'Marqueurs écartés   : {sorted(args.exclude)}')
    print(f'Marqueurs connus    : {sorted(centers_world)}')
    print(f'Marqueurs détectés  : {sorted(detected)}')
    print(f'Utilisables         : {usable}  ({len(usable)}/{len(centers_world)})')
    if len(usable) < 3:
        raise SystemExit('Au moins 3 marqueurs connus doivent être visibles.')
    if len(usable) < 4:
        print('⚠ 3 marqueurs seulement : la validation leave-one-out est impossible '
              '(il resterait 2 marqueurs pour réajuster). Résultat non validé.')

    obj_centers = np.array([centers_world[i] for i in usable])
    img_centers = np.array([detected[i].mean(axis=0) for i in usable])
    rvec0, tvec0 = _pose_from_centers(obj_centers, img_centers, K, dist)
    _, T_world_cam0 = to_matrices(rvec0, tvec0)

    print('\nMarqueurs — orientation et taille mesurées sur l\'image :')
    resolved_yaws, measured_sizes = {}, []
    per_marker = {}
    for i in usable:
        plane_z = float(centers_world[i][2])
        measured_yaw = estimate_marker_yaw(detected[i], K, dist, T_world_cam0, plane_z)
        measured_sizes.append(
            measure_marker_size(detected[i], K, dist, T_world_cam0, plane_z))
        yaw = yaws[i] if i in yaws else measured_yaw
        source = 'YAML' if i in yaws else 'image'
        print(f'  id {i:2d} : yaw {yaw:+7.1f}° ({source})   '
              f'côté mesuré {measured_sizes[-1] * 1000:5.1f} mm')
        resolved_yaws[i] = yaw
        obj = marker_corners_3d(centers_world[i], size, yaw)
        # Les mêmes 4 coins 3D, appariés une fois par image : N fois plus de
        # contraintes 2D pour les 6 mêmes inconnues.
        img = np.vstack([match_corners(obj, d[i], rvec0, tvec0, K, dist)
                         for d in per_frame])
        per_marker[i] = {'obj': np.tile(obj, (len(per_frame), 1)), 'img': img}

    size_mm = float(np.mean(measured_sizes)) * 1000.0
    spread_mm = float(np.std(measured_sizes)) * 1000.0
    print(f'  Côté moyen mesuré : {size_mm:.1f} mm (±{spread_mm:.1f}) '
          f'vs {size * 1000:.1f} mm dans le YAML')
    if abs(size_mm - size * 1000.0) > 5.0:
        print(f'  ⚠ ÉCART DE {abs(size_mm - size * 1000.0):.0f} mm — corrige '
              f'`marker_size_mm` dans {Path(args.markers).name} et relance. '
              f'Avec les 16 coins, cet écart décale chaque coin de '
              f'{abs(size_mm - size * 1000.0) / 2:.0f} mm.')

    obj_pts = np.vstack([per_marker[i]['obj'] for i in usable])
    img_pts = np.vstack([per_marker[i]['img'] for i in usable])
    rvec, tvec, inliers = fit_pose(obj_pts, img_pts, K, dist)
    err = reproj_errors(obj_pts, img_pts, rvec, tvec, K, dist)
    rms = float(np.sqrt((err ** 2).mean()))

    print(f'\nAjustement sur {len(obj_pts)} coins '
          f'({len(inliers)} inliers RANSAC) :')
    stride = 4 * len(per_frame)
    for n, i in enumerate(usable):
        e = err[n * stride:(n + 1) * stride]
        print(f'  id {i:2d} : reproj moyenne {e.mean():5.2f} px  '
              f'(min {e.min():5.2f}, max {e.max():5.2f}) sur {len(e)} coins')
    print(f'  RMS global : {rms:5.2f} px  '
          f"({'OK' if rms < 2.0 else 'ÉLEVÉ — vérifie les mesures et les yaw'})")

    T_cam_world, T_world_cam = to_matrices(rvec, tvec)
    cam_pos = T_world_cam[:3, 3]
    print(f'\nCaméra dans {frame_id} : X={cam_pos[0]:+.3f} Y={cam_pos[1]:+.3f} '
          f'Z={cam_pos[2]:+.3f} m')

    # Où la caméra CROIT que chaque marqueur se trouve, contre la mesure au ruban.
    # C'est le résidu du §5.3 exprimé dans l'unité qui décide de la saisie. Un
    # écart isolé sur un seul marqueur désigne une mesure à refaire ; un écart
    # partagé par tous désigne l'intrinsèque ou le plan table.
    print('\nPosition vue par la caméra vs mesurée au ruban (mm) :')
    residuals = {}
    for i in usable:
        seen = pixel_to_plane(per_marker[i]['img'].mean(axis=0), K, dist,
                              T_world_cam, float(centers_world[i][2]))
        delta = (seen[:2] - centers_world[i][:2]) * 1000.0
        residuals[i] = float(np.linalg.norm(delta))
        print(f'  id {i:2d} : mesuré ({centers_world[i][0] * 1000:6.1f},'
              f'{centers_world[i][1] * 1000:7.1f})  vu ({seen[0] * 1000:6.1f},'
              f'{seen[1] * 1000:7.1f})  écart ΔX={delta[0]:+6.1f} ΔY={delta[1]:+6.1f} '
              f'|{residuals[i]:5.1f}|')
    worst_id = max(residuals, key=residuals.get)
    others = [v for k, v in residuals.items() if k != worst_id]
    if others and residuals[worst_id] > 2.5 * max(others):
        print(f'  ⚠ le marqueur {worst_id} se détache nettement des autres '
              f'({residuals[worst_id]:.1f} mm contre {max(others):.1f} mm au pire) : '
              f'sa position mesurée est probablement à refaire.')

    validation = validate_leave_one_out(per_marker, K, dist, args.table_z)
    if validation:
        print('\nValidation leave-one-marker-out (le marqueur testé est EXCLU du fit) :')
        for row in validation:
            print(f"  id {row['marker']:2d} exclu : reproj {row['reproj_px']:5.2f} px  "
                  f"| erreur plan table {row['plane_error_mm']:6.1f} mm")
        worst = max(r['plane_error_mm'] for r in validation)
        print(f'  Pire erreur sur point neuf : {worst:.1f} mm — à comparer à '
              f"l'ouverture de la pince avant de l'utiliser en saisie.")

    payload = {
        'source': 'calibrate_camera_base_extrinsic (16 coins, RANSAC+LM, validé LOO)',
        'camera': args.camera,
        'intrinsics_stem': calib_stem,
        'resolution': [CAPTURE_W, CAPTURE_H],
        'frame_id': frame_id,
        'markers_used': usable,
        'marker_size_mm': size * 1000.0,
        'marker_size_measured_mm': size_mm,
        'marker_yaw_deg': {int(k): float(v) for k, v in resolved_yaws.items()},
        'n_points': int(len(obj_pts)),
        'n_inliers': int(len(inliers)),
        'rms_reproj_px': rms,
        'table_z_m': args.table_z,
        'validation_leave_one_out': validation,
        'T_cam_world': T_cam_world.tolist(),
        'T_world_cam': T_world_cam.tolist(),
        'camera_position_base_m': cam_pos.tolist(),
    }
    out_path.write_text(yaml.safe_dump(payload, sort_keys=False))
    print(f'\nÉcrit -> {out_path}')

    proj, _ = cv2.projectPoints(obj_pts, rvec, tvec, K, dist)
    for (u, v), (pu, pv) in zip(img_pts, proj.reshape(-1, 2)):
        cv2.circle(bgr, (int(u), int(v)), 5, (0, 0, 255), 1)
        cv2.drawMarker(bgr, (int(pu), int(pv)), (0, 255, 0), cv2.MARKER_CROSS, 10, 1)
    viz = out_path.with_suffix('.reproj.png')
    cv2.imwrite(str(viz), bgr)
    print(f'Contrôle visuel (rouge=détecté, vert=reprojeté) -> {viz}')


if __name__ == '__main__':
    main()
