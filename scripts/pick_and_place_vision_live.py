#!/usr/bin/env python3
"""Pick-and-place vision-guidé LIVE — glue perception (moitié B).

À lancer avec le .venv (torch + roslibpy) :
  .venv/bin/python scripts/pick_and_place_vision_live.py --pi-host 10.10.0.224 --dry-run

Chaîne (markerless, auto-adaptatif) :
  caméras (cv2) ──► YOLO ──► pixel objet (u,v) par vue          ┐
  rosbridge /dream/keypoints, /dream_svpro/keypoints ──► kp 2D  ├─► MultiViewLocalizer
  rosbridge /joint_states ──► angles encodeurs ──► FK           ┘        │
                                                     (x,y,z) base ◄──────┘
                                                          │
                         IK multi-branches PC + contrôle XYZ (send_angles)

Prérequis EN PARALLÈLE :
  - ros2 launch mycobot_gateway dream_multicam.launch.py     (keypoints + joint_states)
  - ros2 launch rosbridge_server rosbridge_websocket_launch.xml
  - bridge Pi (get_angles/get_coords, send_angles, pince)
"""
import argparse
import subprocess
import base64
import json
import sys
import time
from pathlib import Path

import numpy as np

_REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_REPO / 'scripts'))
sys.path.insert(0, str(_REPO / 'mycobot_gateway'))
sys.path.insert(0, str(_REPO / 'mycobot_gateway' / 'mycobot_gateway' / 'vision'))
sys.path.insert(0, str(_REPO / 'training' / 'dream'))

import cv2                                                              # noqa: E402
import roslibpy                                                        # noqa: E402
import yaml                                                            # noqa: E402

import multiview_localizer as mv                                       # noqa: E402
from object_localizer import pixel_to_base, load_extrinsic             # noqa: E402
from mycobot_gateway.vision.camera_registry import (                   # noqa: E402
    load_intrinsics, detect_cameras)
import pick_and_place_vision as ctrl                                   # noqa: E402
import live_aruco_geometry as aruco_geometry                           # noqa: E402
from joint_ik_control import JointIKController                         # noqa: E402

# caméra -> topics DREAM/image + stem intrinsèque. On s'abonne aux IMAGES que le
# launch multicam publie déjà (le device V4L2 est déjà pris par camera_publisher,
# un seul process peut l'ouvrir) -> et YOLO voit la MÊME frame que DREAM.
# 'extrinsic' : YAML DREAM self-cal figé (caméra FIXE) — plus stable que le live
# 1-frame. None = pas d'extrinsèque figée (svpro) -> live obligatoire.
_CAL = _REPO / 'training' / 'calibration'
_PICK_CAL = _REPO / 'scripts' / 'pick_adaptive_calibration.json'
CAMERAS = {
    'arducam': {'kp_topic': '/dream/keypoints',       'img_topic': '/camera/image_raw', 'device': None,
                # cam_3, pas cam_0 : c'est l'intrinsèque que camera_registry et le
                # dashboard utilisent réellement. cam_0 (fx=526 vs 496, cy=226 vs
                # 248) déplace la cible de 41 mm près du robot à 73 mm au fond de
                # la planche — une erreur qui VARIE avec la position, donc
                # incorrigeable par l'offset XY constant.
                'calib': 'cam_3',
                'extrinsic': str(_CAL / 'arducam_extrinsic_dream_v4.yaml'),   # DREAM self-cal
                # Calibration géométrique directe par les quatre ArUco fixes.
                'extrinsic_markers': str(_CAL / 'arducam_extrinsic_markers.yaml')},
    'svpro':   {'kp_topic': '/dream_svpro/keypoints', 'img_topic': '/camera_svpro/image_raw', 'device': 3,
                'calib': 'cam_2', 'extrinsic': None, 'extrinsic_markers': None},
}
N_KP = 7  # modèle DREAM courant (7 keypoints)


class LiveState:
    """Dernières valeurs reçues via rosbridge (thread roslibpy)."""

    def __init__(self):
        self.kp = {name: None for name in CAMERAS}     # (kp_2d (N,2), valid (N,))
        self.frame = {name: None for name in CAMERAS}  # dernière image BGR (H,W,3)
        self.joint_q = None

    def on_keypoints(self, name):
        def cb(msg):
            arr = np.array(msg['data'], dtype=np.float64)
            if arr.size < N_KP * 3:
                return
            arr = arr[:N_KP * 3].reshape(N_KP, 3)
            self.kp[name] = (arr[:, :2], arr[:, 2] > 0.5)
        return cb

    def on_image(self, name):
        def cb(msg):
            buf = base64.b64decode(msg['data'])
            h, w = msg['height'], msg['width']
            self.frame[name] = np.frombuffer(buf, np.uint8).reshape(h, w, -1)  # bgr8
        return cb

    def on_joint_states(self, msg):
        pos = msg.get('position') or []
        if len(pos) >= 6:
            self.joint_q = [float(a) for a in pos[:6]]


class RosbridgeRobot:
    """Commande le robot via bridge_tour (/to_robot -> Pi, /from_robot <- Pi).

    UNE seule connexion Pi (celle de bridge_tour) partagée -> pas de 2ᵉ socket qui
    entre en conflit avec le dashboard. ⚠️ Ferme le dashboard pendant le pick : s'il
    interroge le robot en continu (get_angles), ses réponses se mélangent aux nôtres
    sur /from_robot. Interface compatible avec pick_and_place_vision.run (send/grip/
    gripper_status).
    """

    def __init__(self, ros):
        import json
        self._json = json
        self.pub = roslibpy.Topic(ros, '/to_robot', 'std_msgs/String')
        self.pub.advertise()
        self._resp = None
        roslibpy.Topic(ros, '/from_robot', 'std_msgs/String').subscribe(self._on_resp)
        self._last_grip = 0.0

    def _on_resp(self, msg):
        self._resp = msg['data']

    def send(self, obj, timeout=8.0):
        self._resp = None
        self.pub.publish({'data': self._json.dumps(obj)})
        t0 = time.time()
        while self._resp is None and time.time() - t0 < timeout:
            time.sleep(0.02)
        if self._resp is None:
            raise TimeoutError('pas de réponse sur /from_robot (bridge_tour/dashboard ?)')
        return self._resp

    def grip(self, obj):
        dt = time.time() - self._last_grip
        if dt < 1.6:
            time.sleep(1.6 - dt)
        r = self.send(obj)
        self._last_grip = time.time()
        return r

    def gripper_status(self):
        import re
        r = self.grip({'action': 'get_pro_gripper_status', 'gripper_id': 14})
        m = re.search(r'-?\d+', r.split('PRO_GRIPPER_STATUS:', 1)[-1])
        return int(m.group()) if m else None


def detect_color(frame, hsv_lo, hsv_hi, name=None, save=False, min_area=200,
                 max_area=2500, min_circularity=0.30, border=25):
    """Détecte le plus gros blob dans la plage HSV -> (u,v du centroïde, score) ou (None,0).

    Bien plus robuste que YOLO pour un objet de couleur franche (balle jaune-vert).
    score = aire normalisée (proxy de confiance). `border` : ignore les blobs dont le
    centroïde tombe à moins de `border` px d'un bord de l'image (rejette les objets
    jaunes parasites collés au cadre, ex. un outil au bord — la cible est au centre).
    """
    import cv2 as _cv
    h, w = frame.shape[:2]
    hsv = _cv.cvtColor(frame, _cv.COLOR_BGR2HSV)
    mask = _cv.inRange(hsv, np.array(hsv_lo), np.array(hsv_hi))
    # Petit OPEN contre le bruit, puis CLOSE plus large pour réunir les morceaux
    # d'une balle partiellement masquée par la pince/câbles.
    # Les noyaux sont mis à l'échelle de la résolution : figés à 3/9 (calibrés en
    # 640x480), ils laissent en 1600x1200 un masque déchiqueté dont le périmètre
    # fait chuter la circularité à 0.34 — sous le seuil dès qu'une ombre passe,
    # d'où des « balle non détectée » aléatoires. Mis à l'échelle : 0.75.
    _s = max(1.0, w / 640.0)
    _open, _close = int(3 * _s) | 1, int(9 * _s) | 1
    mask = _cv.morphologyEx(mask, _cv.MORPH_OPEN, np.ones((_open, _open), np.uint8))
    mask = _cv.morphologyEx(mask, _cv.MORPH_CLOSE, np.ones((_close, _close), np.uint8))
    cnts, _ = _cv.findContours(mask, _cv.RETR_EXTERNAL, _cv.CHAIN_APPROX_SIMPLE)
    best, best_area = None, 0.0
    for c in cnts:
        a = _cv.contourArea(c)
        perimeter = _cv.arcLength(c, True)
        circularity = 4.0 * np.pi * a / (perimeter * perimeter) if perimeter > 0 else 0.0
        if (a > best_area and min_area <= a <= max_area and
                circularity >= min_circularity):
            M = _cv.moments(c)
            if M['m00'] > 0:
                cx, cy = M['m10'] / M['m00'], M['m01'] / M['m00']
                if cx < border or cx > w - border or cy < border or cy > h - border:
                    continue
                best = (cx, cy)
                best_area = a
    if name is not None:
        print(f'    {name}: blob couleur aire={best_area:.0f}px')
    if save:
        vis = frame.copy()
        if best is not None:
            _cv.circle(vis, (int(best[0]), int(best[1])), 8, (0, 0, 255), 2)
        _cv.imwrite(str(_REPO / f'scratch_pp_{name}.jpg'), vis)
    conf = min(best_area / 2000.0, 1.0) if best else 0.0
    return best, conf


def detect_object(model, frame, want, name=None, save=False):
    """YOLO -> (u,v) du centre de la bbox la plus confiante (classe voulue) ou None.

    Log TOUTES les détections (diagnostic : voir ce que YOLO reconnaît réellement).
    `save` écrit une frame annotée scratch_pp_<name>.jpg.
    """
    res = model(frame, conf=0.25, verbose=False)[0]
    best, best_conf = None, 0.0
    seen = []
    annotated = frame.copy() if save else None
    for box in res.boxes:
        cls = int(box.cls[0])
        conf = float(box.conf[0])
        x1, y1, x2, y2 = box.xyxy[0].tolist()
        u, v = (x1 + x2) / 2.0, (y1 + y2) / 2.0
        seen.append(f'{model.names[cls]}:{conf:.2f}')
        if save:
            import cv2 as _cv
            _cv.rectangle(annotated, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            _cv.putText(annotated, f'{model.names[cls]} {conf:.2f}', (int(x1), int(y1) - 5),
                        _cv.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        if (want is None or cls in want) and conf > best_conf:
            best, best_conf = (u, v), conf
    if name is not None:
        print(f'    {name}: YOLO voit [{", ".join(seen) or "rien"}]')
    if save and annotated is not None:
        import cv2 as _cv
        out = _REPO / f'scratch_pp_{name}.jpg'
        _cv.imwrite(str(out), annotated)
        print(f'      → frame annotée : {out}')
    return best, best_conf


def camera_extrinsic(name, state, obs, K, mode, live_geometry=None):
    """T_world_cam : ArUco live/YAML, DREAM figé, ou DREAM live."""
    if mode == 'markers-live':
        if live_geometry is None:
            raise ValueError(f'{name}: calibration ArUco live indisponible')
        frame = state.frame.get(name)
        if frame is None:
            raise ValueError(f'{name}: aucune image pour détecter les ArUco')
        centres = aruco_geometry.detect_marker_centres(
            frame,
            dictionary_name=live_geometry.camera.dictionary_name,
        )
        pose = aruco_geometry.estimate_live_extrinsic(
            centres,
            live_geometry.workspace,
            live_geometry.camera,
        )
        p = pose.camera_position_world_m
        print(f'    ✅ {name}: extrinsèque ArUco LIVE — IDs={list(pose.marker_ids)}, '
              f'RMS={pose.rms_reprojection_px:.2f}px, '
              f'caméra=({p[0]*1000:.0f},{p[1]*1000:.0f},{p[2]*1000:.0f})mm base')
        return pose.T_world_cam
    if mode == 'markers' and CAMERAS[name].get('extrinsic_markers'):
        T = load_extrinsic(CAMERAS[name]['extrinsic_markers'])
        print(f'    ℹ️ {name}: extrinsèque ArUco TABLE (précise)')
        return T
    if mode == 'static' and CAMERAS[name].get('extrinsic'):
        T = load_extrinsic(CAMERAS[name]['extrinsic'])
        print(f'    ℹ️ {name}: extrinsèque STATIQUE DREAM (YAML)')
        return T
    T = mv.extrinsic_from_dream(state.joint_q, obs['kp_2d'], obs['valid'], K)
    cam_pos = T[:3, 3]
    print(f'    ℹ️ {name}: extrinsèque LIVE — caméra à '
          f'({cam_pos[0]*1000:.0f},{cam_pos[1]*1000:.0f},{cam_pos[2]*1000:.0f})mm base, '
          f'{int(np.asarray(obs["valid"]).sum())}/{len(obs["valid"])} kp DREAM')
    return T


def localize(state, intr, obj_px, table_z, cameras, mode, live_geometries=None):
    """(x,y,z) base : triangulation si ≥2 vues avec extrinsèque, sinon mono plan-table."""
    extr, rays = {}, []
    for name in cameras:
        if obj_px.get(name) is None:
            continue
        if mode not in ('markers', 'markers-live') and state.kp[name] is None:
            continue                              # extrinsèque DREAM -> keypoints requis
        kp_2d, valid = state.kp[name] if state.kp[name] is not None else (None, None)
        obs = {'kp_2d': kp_2d, 'valid': valid, 'obj_px': obj_px[name]}
        try:
            T = camera_extrinsic(
                name, state, obs, intr[name], mode,
                None if live_geometries is None else live_geometries.get(name))
        except (ValueError, aruco_geometry.GeometryError) as e:
            print(f'    ⚠️ {name}: extrinsèque échoue ({e})')
            continue
        extr[name] = (T, obj_px[name])
        u, v = obj_px[name]
        rays.append(mv.pixel_ray(u, v, intr[name], T))

    if len(rays) >= 2:
        return mv.triangulate(rays), 'triangulation'
    # mono : une seule vue exploitable -> intersection plan table
    for name, (T, (u, v)) in extr.items():
        try:
            if mode == 'markers-live' and live_geometries is not None:
                bundle = live_geometries[name]
                return aruco_geometry.deproject_pixel_to_base(
                    (u, v), bundle.camera, T, table_z), \
                    f'mono {name} (ArUco live + plan-table)'
            return pixel_to_base(u, v, intr[name], T, table_z), f'mono {name} (plan-table)'
        except (ValueError, aruco_geometry.GeometryError) as e:
            print(f'    ⚠️ {name}: déprojection échoue ({e})')
    raise RuntimeError('aucune vue exploitable')


def load_workspace_polygon(path):
    """Retourne l'enveloppe convexe XY des marqueurs fixes de la table."""
    data = yaml.safe_load(Path(path).read_text())
    points = np.asarray([v[:2] for v in data['markers'].values()], dtype=np.float64)
    if len(points) < 3:
        raise ValueError('au moins 3 marqueurs sont requis pour définir la zone de table')
    return cv2.convexHull(points.astype(np.float32)).reshape(-1, 2)


def inside_workspace(xyz, polygon, margin_m):
    """Test zone ArUco, avec marge positive vers l'intérieur de la table."""
    distance = cv2.pointPolygonTest(
        polygon.astype(np.float32), (float(xyz[0]), float(xyz[1])), True)
    return distance >= margin_m, float(distance)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--pi-host', default='10.10.0.224')
    ap.add_argument('--ros-host', default='localhost')
    ap.add_argument('--ros-port', type=int, default=9090)
    ap.add_argument('--weights', default='yolov8n.pt')
    ap.add_argument('--classes', nargs='*', default=None,
                    help='classes COCO à saisir (ex: cup bottle "sports ball")')
    ap.add_argument('--table-z', type=float, default=0.0, help='hauteur de la table (m, base)')
    ap.add_argument('--object-height', type=float, default=0.067,
                    help='hauteur de l\'objet en m ; balle de tennis = 0.067')
    ap.add_argument('--max-xy', type=float, default=0.55,
                    help='limite de sécurité absolue X/Y en mètres (défaut: 0.55)')
    ap.add_argument('--max-reach', type=float, default=0.44,
                    help='portée radiale XY maximale du robot, en m (défaut validé: 0.44)')
    ap.add_argument('--place', nargs=3, type=float, default=[0.20, 0.15, 0.05])
    ap.add_argument('--task', choices=['place', 'handover'], default='place',
                    help='place = dépose sur table ; handover = remise en main à une pose apprise')
    ap.add_argument('--positions', default=str(_REPO / 'scripts' / 'pick_place_positions.json'),
                    help='JSON contenant handover_approach et handover pour --task handover')
    ap.add_argument('--speed', type=int, default=None,
                    help='vitesse ; défaut chargé depuis la calibration adaptative')
    ap.add_argument('--grasp-angle', type=int, default=None,
                    help='fermeture pince 0-100 adaptée à l\'objet')
    ap.add_argument('--open-angle', type=int, default=100)
    ap.add_argument('--mode', type=int, default=1, choices=[0, 1])
    ap.add_argument('--auto', action='store_true')
    ap.add_argument('--keep-ori', action='store_true',
                    help='option historique ; l’IK PC conserve maintenant l’orientation de la '
                         'prise manuelle validée')
    ap.add_argument('--approach-ori', nargs=3, type=float, default=None,
                    metavar=('RX', 'RY', 'RZ'), help='orientation d\'approche imposée (deg).')
    ap.add_argument('--pick-calibration', default=str(_PICK_CAL),
                    help='calibration vision→commande apprise par démonstration')
    ap.add_argument('--no-pick-offset', action='store_true',
                    help='désactive seulement la correction XY apprise')
    ap.add_argument('--pick-offset-x', type=float, default=None,
                    help='surcharge correction X en m')
    ap.add_argument('--pick-offset-y', type=float, default=None,
                    help='surcharge correction Y en m')
    ap.add_argument('--flange-above', type=float, default=None,
                    help='distance bride→centre objet en mm')
    ap.add_argument('--approach-height', type=float, default=None,
                    help='hauteur d\'approche au-dessus du pick en mm')
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--save', action='store_true', help='écrit les frames YOLO annotées (diagnostic)')
    ap.add_argument('--samples', type=int, default=6,
                    help='frames échantillonnées par caméra (détection robuste)')
    ap.add_argument('--sample-spread-px', type=float, default=18.0,
                    help='dispersion pixel maximale des détections retenues (défaut: 18)')
    ap.add_argument('--workspace-markers',
                    default=str(_CAL / 'workspace_markers.yaml'),
                    help='positions des 4 ArUco délimitant la table')
    ap.add_argument('--workspace-margin', type=float, default=0.025,
                    help='marge intérieure de sécurité par rapport aux ArUco, en m')
    ap.add_argument('--adaptive', action='store_true',
                    help='re-détecte et corrige l\'approche tant que l\'objet bouge')
    ap.add_argument('--alignment-check', action='store_true',
                    help='descend pince ouverte, capture une image, puis remonte sans saisir')
    ap.add_argument('--adaptive-tolerance', type=float, default=0.015,
                    help='déplacement XY toléré avant correction, en m (défaut: 0.015)')
    ap.add_argument('--max-replans', type=int, default=4,
                    help='nombre maximal de corrections visuelles avant abandon')
    ap.add_argument('--servo-loop', action='store_true',
                    help='enchaîne observation→corrections→pick→place sur plusieurs cycles')
    ap.add_argument('--cycles', type=int, default=1,
                    help='nombre de cycles avec --servo-loop (1 à 100)')
    ap.add_argument('--miss-retries', type=int, default=3,
                    help='observations sans balle tolérées entre deux cycles')
    ap.add_argument('--controller-xy-limit', type=float, default=0.350,
                    help='limite absolue X/Y acceptée par le contrôleur, en m')
    ap.add_argument('--xyz-tolerance-mm', type=float, default=5.0,
                    help='erreur XYZ réelle maximale avant arrêt/refus de descente')
    ap.add_argument('--descent-step-mm', type=float, default=2.0,
                    help='pas vertical IK pendant une descente (défaut: 2 mm)')
    ap.add_argument('--observation-pose', default='observation_clear',
                    help='pose articulaire JSON qui initialise l’IK et dégage la balle')
    ap.add_argument('--detector', choices=['yolo', 'color'], default='color',
                    help='yolo (COCO) ou color (blob HSV, robuste pour balle jaune-vert)')
    ap.add_argument('--cameras', nargs='+', default=['arducam'], choices=list(CAMERAS),
                    help='caméras à utiliser. 1 seule -> mono plan-table (utile si une '
                         'extrinsèque DREAM est faible). 2 -> triangulation.')
    ap.add_argument('--extrinsic',
                    choices=['markers-live', 'markers', 'static', 'live'],
                    default='markers-live',
                    help='markers-live = recalcule caméra→base avec les 4 ArUco visibles avant '
                         'chaque mesure ; markers = ancien YAML ArUco figé ; static = DREAM '
                         'figé ; live = PnP DREAM.')
    ap.add_argument('--marker-extrinsic', default=None,
                    help='fichier YAML extrinsèque ArUco à utiliser (défaut: calibration standard)')
    ap.add_argument('--hsv-lo', nargs=3, type=int, default=[20, 170, 70],
                    metavar=('H', 'S', 'V'), help='seuil HSV bas (détecteur color)')
    ap.add_argument('--hsv-hi', nargs=3, type=int, default=[35, 255, 255],
                    metavar=('H', 'S', 'V'), help='seuil HSV haut (détecteur color)')
    ap.add_argument('--color-min-circularity', type=float, default=0.12,
                    help='circularité minimale du blob ; 0.12 tolère une balle partiellement cachée')
    ap.add_argument('--camera-source', choices=['rosbridge', 'v4l2'], default='rosbridge',
                    help='rosbridge = images du launch multicam (défaut) ; v4l2 = caméra en '
                         'direct si le launch est arrêté.')
    ap.add_argument('--exposure', type=int, default=75,
                    help='exposition V4L2 manuelle (défaut validé: 75)')
    ap.add_argument('--svpro-focus', type=int, default=90,
                    help='focus manuel SVPro (défaut validé par capture_real_3cam: 90)')
    ap.add_argument('--robot-via', choices=['rosbridge', 'socket'], default='rosbridge',
                    help='rosbridge = commandes robot via /to_robot (bridge_tour, UNE seule '
                         'connexion Pi — pas de conflit avec le dashboard) ; socket = TCP direct.')
    args = ap.parse_args()

    pick_cal_path = Path(args.pick_calibration).expanduser().resolve()
    if not pick_cal_path.is_file():
        raise SystemExit(f'calibration adaptative introuvable : {pick_cal_path}')
    pick_cal = json.loads(pick_cal_path.read_text())
    learned_offset = pick_cal.get('pick_offset_xy_m', [0.0, 0.0])
    if args.no_pick_offset:
        learned_offset = [0.0, 0.0]
    pick_offset = np.array([
        learned_offset[0] if args.pick_offset_x is None else args.pick_offset_x,
        learned_offset[1] if args.pick_offset_y is None else args.pick_offset_y,
    ], dtype=np.float64)
    args.flange_above = (float(pick_cal.get('flange_above_object_mm', 110.0))
                         if args.flange_above is None else args.flange_above)
    args.approach_height = (float(pick_cal.get('approach_height_mm', 80.0))
                            if args.approach_height is None else args.approach_height)
    args.approach_ori = (list(pick_cal.get('safe_orientation_deg', [-11.7, -74.7, 141.8]))
                         if args.approach_ori is None else args.approach_ori)
    args.speed = int(pick_cal.get('speed', 8)) if args.speed is None else args.speed
    args.grasp_angle = (int(pick_cal.get('grasp_angle', 20))
                        if args.grasp_angle is None else args.grasp_angle)

    if not 0.0 <= args.object_height <= 0.25:
        raise SystemExit('--object-height doit être compris entre 0 et 0.25 m')
    if not 0 <= args.grasp_angle <= 100 or not 0 <= args.open_angle <= 100:
        raise SystemExit('--grasp-angle et --open-angle doivent être compris entre 0 et 100')
    if not 0.05 <= args.color_min_circularity <= 1.0:
        raise SystemExit('--color-min-circularity doit être dans [0.05, 1]')
    if not 1 <= args.cycles <= 100 or not 0 <= args.miss_retries <= 20:
        raise SystemExit('--cycles doit être dans [1,100] et --miss-retries dans [0,20]')
    if not 1.0 <= args.xyz_tolerance_mm <= 20.0:
        raise SystemExit('--xyz-tolerance-mm doit être dans [1,20]')
    if not 0.5 <= args.descent_step_mm <= 10.0:
        raise SystemExit('--descent-step-mm doit être dans [0.5,10]')
    if args.servo_loop:
        # L'asservissement demandé corrige dès 5 mm par défaut. Une surcharge
        # CLI plus stricte reste possible.
        if args.adaptive_tolerance == 0.015:
            args.adaptive_tolerance = 0.005
        args.adaptive = True
        args.auto = True

    if args.marker_extrinsic:
        marker_path = Path(args.marker_extrinsic).expanduser().resolve()
        if not marker_path.is_file():
            raise SystemExit(f'extrinsèque ArUco introuvable: {marker_path}')
        for camera_name in args.cameras:
            CAMERAS[camera_name]['extrinsic_markers'] = str(marker_path)

    handover_angles = None
    positions_path = Path(args.positions).expanduser().resolve()
    positions = json.loads(positions_path.read_text()) if positions_path.is_file() else {}
    observation_angles = positions.get(args.observation_pose)
    if args.servo_loop:
        if (not isinstance(observation_angles, list) or len(observation_angles) != 6 or
                not all(isinstance(v, (int, float)) and np.isfinite(v)
                        for v in observation_angles)):
            raise SystemExit(
                f'pose d’observation {args.observation_pose!r} absente/invalide dans {positions_path}')
    if args.task == 'handover':
        missing = [name for name in ('handover_approach', 'handover') if name not in positions]
        if missing:
            raise SystemExit(
                f'poses de remise manquantes dans {positions_path}: {missing}. '
                'Apprends-les avec pick_and_place_real.py --capture handover_approach puis handover.')
        handover_angles = (positions['handover_approach'], positions['handover'])
        for name, angles in zip(('handover_approach', 'handover'), handover_angles):
            if not isinstance(angles, list) or len(angles) != 6 or not all(
                    isinstance(v, (int, float)) and np.isfinite(v) for v in angles):
                raise SystemExit(f'pose {name} invalide dans {positions_path}')

    if args.detector == 'yolo' and not args.classes:
        raise SystemExit(
            '--detector yolo exige --classes (ex: --classes bottle cup "sports ball"). '
            'Cela empêche YOLO de choisir un objet parasite.')

    # intrinsèques par caméra (640x480)
    intr = {}
    for name, c in CAMERAS.items():
        K, _ = load_intrinsics(c['calib'], 640, 480)
        if K is None:
            raise SystemExit(f'intrinsèque {c["calib"]} ({name}) introuvable')
        intr[name] = K

    # Source des images : V4L2 direct (AUTONOME) ou rosbridge (launch multicam)
    state = LiveState()
    ros, caps = None, {}
    if args.camera_source == 'rosbridge':
        ros = roslibpy.Ros(host=args.ros_host, port=args.ros_port)
        ros.run()
        if not ros.is_connected:
            raise SystemExit(f'rosbridge ws://{args.ros_host}:{args.ros_port} injoignable')
        for name, c in CAMERAS.items():
            roslibpy.Topic(ros, c['kp_topic'], 'std_msgs/Float64MultiArray').subscribe(
                state.on_keypoints(name))
            roslibpy.Topic(ros, c['img_topic'], 'sensor_msgs/Image').subscribe(state.on_image(name))
        roslibpy.Topic(ros, '/joint_states', 'sensor_msgs/JointState').subscribe(state.on_joint_states)
        print(f'  ✅ rosbridge {args.ros_host}:{args.ros_port} — DREAM + images + joint_states')
    else:  # v4l2 : caméra en direct, autonome (pas de dashboard/DREAM -> pas de contention)
        if args.extrinsic not in ('markers', 'markers-live'):
            raise SystemExit('--camera-source v4l2 exige --extrinsic markers-live ou markers.')
        for spec in detect_cameras(names=list(args.cameras)):
            CAMERAS[spec.name]['device'] = spec.v4l2_index
        for name in args.cameras:
            if CAMERAS[name]['device'] is None:
                raise SystemExit(
                    f'{name} non détectée par v4l2-ctl — vérifie le branchement '
                    '(`python3 mycobot_gateway/mycobot_gateway/vision/camera_registry.py`)')
            dev = f'/dev/video{CAMERAS[name]["device"]}'
            if name == 'arducam':
                subprocess.run(
                    ['v4l2-ctl', '-d', dev, '-c', 'auto_exposure=1',
                     '-c', f'exposure_time_absolute={args.exposure},gain=0,brightness=0'],
                    check=False,
                )
            else:
                subprocess.run(
                    ['v4l2-ctl', '-d', dev, '-c', 'focus_automatic_continuous=0',
                     '-c', f'focus_absolute={args.svpro_focus},sharpness=0,contrast=1'],
                    check=False,
                )
            # L'Arducam démarre en 1600x1200 MJPEG. Sur certains pilotes UVC,
            # ouvrir d'abord puis changer le format échoue (OpenCV l'annonce à
            # tort comme « caméra occupée »). Fournir les paramètres à
            # VideoCapture négocie MJPEG/640x480 dès VIDIOC_S_FMT.
            capture_params = [
                cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'),
                cv2.CAP_PROP_FRAME_WIDTH, 640,
                cv2.CAP_PROP_FRAME_HEIGHT, 480,
            ]
            cap = cv2.VideoCapture(
                CAMERAS[name]['device'], cv2.CAP_V4L2, capture_params)
            if not cap.isOpened():
                raise SystemExit(f'{name}: /dev/video{CAMERAS[name]["device"]} occupé — arrête le '
                                 'launch multicam/dashboard pour libérer la caméra en mode v4l2.')
            caps[name] = cap
        print('  ✅ caméras V4L2 (mode AUTONOME — sans DREAM ni dashboard)')

    model, want = None, None
    if args.detector == 'yolo':
        from ultralytics import YOLO
        model = YOLO(args.weights)
        if args.classes:
            want = {i for i, n in model.names.items() if n in args.classes}

    def detect(frame, name=None, save=False):
        if args.detector == 'color':
            return detect_color(
                frame, args.hsv_lo, args.hsv_hi, name=name, save=save,
                min_circularity=args.color_min_circularity)
        return detect_object(model, frame, want, name=name, save=save)

    def get_frame(name):
        if caps:
            ok, f = caps[name].read()
            return f if ok else None
        return state.frame[name]

    if ros is not None:  # attendre les 1ers messages (rosbridge)
        need_dream = args.extrinsic not in ('markers', 'markers-live')
        print('  ⏳ attente images' + (' + DREAM' if need_dream else '') + '…')
        t0 = time.time()
        while (all(state.frame[n] is None for n in args.cameras)
               or (need_dream and (state.joint_q is None
                                   or all(state.kp[n] is None for n in args.cameras)))):
            time.sleep(0.2)
            if time.time() - t0 > 15:
                raise SystemExit('pas de données — le launch multicam tourne ?')

    workspace = load_workspace_polygon(args.workspace_markers)
    live_geometries = {}
    if args.extrinsic == 'markers-live':
        for name in args.cameras:
            calib_stem = CAMERAS[name]['calib']
            npz_path = _CAL / f'{calib_stem}.npz'
            meta_path = _CAL / f'{calib_stem}.meta.json'
            try:
                live_geometries[name] = aruco_geometry.load_geometry(
                    npz_path, meta_path, args.workspace_markers)
            except aruco_geometry.GeometryError as exc:
                raise SystemExit(f'calibration ArUco live {name} invalide: {exc}') from exc
        print('  ✅ calibration ArUco LIVE armée — les 4 tags devront rester visibles')

    def observe_target(required=True):
        """Mesure robuste multi-frame puis valide la cible dans la zone ArUco."""
        obj_px = {}
        for name in args.cameras:
            detections, confidences, last_frame = [], [], None
            for _ in range(args.samples):
                frame = get_frame(name)
                if frame is not None:
                    last_frame = frame.copy()
                    px, conf = detect(frame)
                    if px is not None:
                        detections.append(px); confidences.append(conf)
                time.sleep(0.2)
            if last_frame is not None:
                # La pose ArUco et le pixel objet doivent provenir du même flux.
                # C'est également nécessaire en V4L2, où LiveState n'est pas
                # alimenté par rosbridge.
                state.frame[name] = last_frame
                detect(last_frame, name=name, save=args.save)
            if detections:
                pts = np.asarray(detections, dtype=np.float64)
                centre = np.median(pts, axis=0)
                spread = float(np.max(np.linalg.norm(pts - centre, axis=1)))
                obj_px[name] = tuple(centre) if spread <= args.sample_spread_px else None
                print(f'    {name}: {len(pts)}/{args.samples} détections, '
                      f'médiane={tuple(np.round(centre, 1))}, dispersion={spread:.1f}px'
                      + ('' if obj_px[name] else '  ❌ instable'))
            else:
                obj_px[name] = None
                print(f'    {name}: cible absente')

        if all(v is None for v in obj_px.values()):
            if required:
                raise RuntimeError('cible absente ou instable sur toutes les vues')
            return None
        object_center_z = args.table_z + args.object_height / 2.0
        xyz_now, method = localize(
            state, intr, obj_px, object_center_z, args.cameras, args.extrinsic,
            live_geometries)
        if (not np.all(np.isfinite(xyz_now)) or
                abs(float(xyz_now[0])) > args.max_xy or
                abs(float(xyz_now[1])) > args.max_xy):
            raise RuntimeError(f'cible hors limite absolue ±{args.max_xy*1000:.0f} mm')
        reach = float(np.linalg.norm(np.asarray(xyz_now)[:2]))
        if reach > args.max_reach:
            raise RuntimeError(
                f'cible hors portée radiale ({reach*1000:.0f} mm > '
                f'{args.max_reach*1000:.0f} mm)')
        valid, edge_distance = inside_workspace(
            xyz_now, workspace, args.workspace_margin)
        if not valid:
            raise RuntimeError(
                f'cible hors zone ArUco sûre (distance bord={edge_distance*1000:.0f} mm, '
                f'marge requise={args.workspace_margin*1000:.0f} mm)')
        print(f'\n  🎯 objet ({method}) : base=({xyz_now[0]*1000:.0f}, '
              f'{xyz_now[1]*1000:.0f}, {xyz_now[2]*1000:.0f}) mm | '
              f'bord table={edge_distance*1000:.0f} mm\n')
        return np.asarray(xyz_now, dtype=np.float64)

    def command_target():
        """Localisation brute sûre + correction vision→pince apprise."""
        raw = observe_target()
        corrected = raw.copy()
        corrected[:2] += pick_offset
        reach = float(np.linalg.norm(corrected[:2]))
        if (abs(float(corrected[0])) > args.max_xy or
                abs(float(corrected[1])) > args.max_xy or reach > args.max_reach):
            raise RuntimeError(
                f'cible corrigée hors portée : ({corrected[0]*1000:.0f}, '
                f'{corrected[1]*1000:.0f}) mm')
        if (abs(float(corrected[0])) > args.controller_xy_limit or
                abs(float(corrected[1])) > args.controller_xy_limit):
            raise RuntimeError(
                f'cible refusée par limite contrôleur ±{args.controller_xy_limit*1000:.0f} mm : '
                f'({corrected[0]*1000:.1f}, {corrected[1]*1000:.1f}) mm')
        print(f'  🧠 correction démonstration XY = '
              f'({pick_offset[0]*1000:+.0f}, {pick_offset[1]*1000:+.0f}) mm '
              f'→ commande ({corrected[0]*1000:.0f}, {corrected[1]*1000:.0f}) mm')
        return corrected

    if args.dry_run:
        bridge = None
    elif args.robot_via == 'rosbridge':
        if ros is None:
            ros = roslibpy.Ros(host=args.ros_host, port=args.ros_port)
            ros.run()
            if not ros.is_connected:
                raise SystemExit(
                    f'rosbridge ws://{args.ros_host}:{args.ros_port} injoignable pour le robot')
        bridge = RosbridgeRobot(ros)
        print('  🤖 robot via /to_robot (bridge_tour) — ferme le dashboard pour éviter le cross-talk')
    else:
        bridge = ctrl.Bridge(args.pi_host)
        print(f'  🤖 robot via socket direct {args.pi_host}:5005')

    manual_angles = pick_cal.get('successful_pick_angles_deg')
    manual_coords = pick_cal.get('successful_pick_coords_mm_deg')
    if (not isinstance(manual_angles, list) or len(manual_angles) != 6 or
            not isinstance(manual_coords, list) or len(manual_coords) < 3):
        raise SystemExit(
            'calibration incomplète : successful_pick_angles_deg et '
            'successful_pick_coords_mm_deg sont obligatoires pour l’IK PC')
    joint_controller = JointIKController(
        bridge, manual_angles, manual_coords[:3], speed=args.speed,
        xyz_tolerance_mm=args.xyz_tolerance_mm,
        descent_step_mm=args.descent_step_mm, dry_run=args.dry_run)
    print('  🧭 IK PC activée : orientation de la PRISE manuelle validée ; '
          'observation_clear reste uniquement une pose caméra')

    def acquire_target(retries=0):
        """Observe la balle, avec attente bornée entre deux cycles."""
        last_error = None
        for attempt in range(retries + 1):
            try:
                return command_target()
            except RuntimeError as exc:
                last_error = exc
                if attempt < retries:
                    print(f'  ⏳ cible indisponible ({exc}) — nouvelle observation '
                          f'{attempt + 2}/{retries + 1}')
                    time.sleep(1.0)
        raise RuntimeError(str(last_error))

    try:
        if args.servo_loop:
            completed = 0
            for cycle in range(1, args.cycles + 1):
                print(f'\n{"=" * 18} CYCLE ASSERVI {cycle}/{args.cycles} {"=" * 18}')
                # Initialisation articulaire obligatoire : empêche pymycobot de
                # choisir la branche IK J5≈+90° observée depuis home.
                ctrl.move_angles(
                    bridge, observation_angles, args.speed, args.dry_run)
                try:
                    xyz = acquire_target(args.miss_retries)
                except RuntimeError as exc:
                    print(f'  🛑 aucune cible après {args.miss_retries + 1} observations : {exc}')
                    break
                success = ctrl.run(
                    bridge, [float(v) for v in xyz], args.place,
                    args.speed, args.mode, args.dry_run, True,
                    keep_ori=args.keep_ori, ori=args.approach_ori,
                    relocalize=command_target,
                    adaptive_tolerance_m=args.adaptive_tolerance,
                    max_replans=args.max_replans,
                    grasp_angle=args.grasp_angle, open_angle=args.open_angle,
                    flange_above=args.flange_above,
                    approach_h=args.approach_height,
                    joint_controller=joint_controller)
                if not success:
                    print('  🛑 cycle interrompu : prise ou validation non confirmée')
                    break
                completed += 1
            print(f'\n  ✅ boucle terminée : {completed}/{args.cycles} cycle(s) réussi(s)')
            return

        try:
            xyz = acquire_target()
        except RuntimeError as exc:
            raise SystemExit(f'🛑 {exc} — aucun mouvement') from exc

        if args.alignment_check:
            orientation = list(args.approach_ori)
            approach, pick = ctrl.plan(
                xyz, orientation, args.flange_above, args.approach_height)
            print('\n  🔎 TEST ALIGNEMENT — aucune fermeture de pince, aucune dépose')
            ctrl.grip(bridge, args.open_angle, args.dry_run)
            joint_controller.move_pose(approach[:3], context='approche alignement')
            try:
                fresh = command_target()
            except RuntimeError as exc:
                # Pendant ce diagnostic uniquement, la pince peut masquer un
                # tag à l'approche. La pose caméra et la cible ont déjà été
                # validées juste avant le mouvement ; on conserve donc cette
                # mesure au lieu de fermer la pince ou de poursuivre un pick.
                fresh = np.asarray(xyz, dtype=np.float64)
                print(f'  ⚠️ relecture masquée à l\'approche ({exc})')
                print('  ↪ position initiale validée conservée pour la capture sans saisie')
            approach, pick = ctrl.plan(
                fresh, orientation, args.flange_above, args.approach_height)
            descended = False
            try:
                joint_controller.descend_vertical(pick[:3])
                descended = True
                last = None
                for _ in range(8):
                    frame = get_frame(args.cameras[0])
                    if frame is not None:
                        last = frame.copy()
                    time.sleep(0.12)
                if last is None:
                    raise RuntimeError('aucune image pendant le contrôle descendu')
                output = _REPO / 'scratch_alignment_descended.jpg'
                cv2.imwrite(str(output), last)
                print(f'  📷 capture pince descendue : {output}')
            finally:
                if descended:
                    joint_controller.move_pose(approach[:3], context='retrait alignement')
            print('  ✅ contrôle terminé — pince remontée, aucune saisie effectuée')
            return

        def confirm_handover():
            answer = input(
                '\n🤲 Place ta main OUVERTE sous l\'objet, sans toucher la pince. '
                'Tape O puis ENTER pour libérer (autre touche = conserver) : ')
            return answer.strip().lower() in ('o', 'oui', 'y', 'yes')

        ctrl.run(bridge, [float(v) for v in xyz], args.place,
                 args.speed, args.mode, args.dry_run, args.auto,
                 keep_ori=args.keep_ori, ori=args.approach_ori,
                 relocalize=command_target if args.adaptive else None,
                 adaptive_tolerance_m=args.adaptive_tolerance,
                 max_replans=args.max_replans,
                 handover_angles=handover_angles,
                 release_ready=confirm_handover if args.task == 'handover' else None,
                 grasp_angle=args.grasp_angle, open_angle=args.open_angle,
                 flange_above=args.flange_above,
                 approach_h=args.approach_height,
                 joint_controller=joint_controller)
    except RuntimeError as exc:
        raise SystemExit(f'🛑 {exc}') from exc

    if ros is not None:
        ros.terminate()
    for cap in caps.values():
        cap.release()


if __name__ == '__main__':
    main()
