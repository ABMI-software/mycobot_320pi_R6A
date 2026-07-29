#!/usr/bin/env python3
"""Pick-and-place vision-guidé LIVE — glue perception (moitié B).

À lancer avec le .venv (torch + roslibpy) :
  .venv/bin/python scripts/pick_and_place_vision_live.py --pi-host 10.10.0.221 --dry-run

Chaîne (markerless, auto-adaptatif) :
  caméras (cv2) ──► YOLO ──► pixel objet (u,v) par vue          ┐
  rosbridge /dream/keypoints, /dream_svpro/keypoints ──► kp 2D  ├─► MultiViewLocalizer
  rosbridge /joint_states ──► angles encodeurs ──► FK           ┘        │
                                                     (x,y,z) base ◄──────┘
                                                          │
                                       pick_and_place_vision.run (send_coords)

Prérequis EN PARALLÈLE :
  - ros2 launch mycobot_gateway dream_multicam.launch.py     (keypoints + joint_states)
  - ros2 launch rosbridge_server rosbridge_websocket_launch.xml
  - bridge Pi (send_coords / pince)
"""
import argparse
import base64
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

import multiview_localizer as mv                                       # noqa: E402
from object_localizer import pixel_to_base, load_extrinsic             # noqa: E402
from mycobot_gateway.vision.camera_registry import load_intrinsics     # noqa: E402
import pick_and_place_vision as ctrl                                   # noqa: E402

# caméra -> topics DREAM/image + stem intrinsèque. On s'abonne aux IMAGES que le
# launch multicam publie déjà (le device V4L2 est déjà pris par camera_publisher,
# un seul process peut l'ouvrir) -> et YOLO voit la MÊME frame que DREAM.
# 'extrinsic' : YAML DREAM self-cal figé (caméra FIXE) — plus stable que le live
# 1-frame. None = pas d'extrinsèque figée (svpro) -> live obligatoire.
_CAL = _REPO / 'training' / 'calibration'
CAMERAS = {
    'arducam': {'kp_topic': '/dream/keypoints',       'img_topic': '/camera/image_raw', 'device': 0,
                'calib': 'cam_3',
                'extrinsic': str(_CAL / 'arducam_extrinsic_dream_v4.yaml'),   # DREAM self-cal
                'extrinsic_markers': str(_CAL / 'arducam_extrinsic_markers.yaml')},  # ArUco table (précis)
    'svpro':   {'kp_topic': '/dream_svpro/keypoints', 'img_topic': '/camera_svpro/image_raw', 'device': 2,
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


def detect_color(frame, hsv_lo, hsv_hi, name=None, save=False, min_area=300):
    """Détecte le plus gros blob dans la plage HSV -> (u,v du centroïde, score) ou (None,0).

    Bien plus robuste que YOLO pour un objet de couleur franche (balle jaune-vert).
    score = aire normalisée (proxy de confiance).
    """
    import cv2 as _cv
    hsv = _cv.cvtColor(frame, _cv.COLOR_BGR2HSV)
    mask = _cv.inRange(hsv, np.array(hsv_lo), np.array(hsv_hi))
    mask = _cv.morphologyEx(mask, _cv.MORPH_OPEN, np.ones((5, 5), np.uint8))
    cnts, _ = _cv.findContours(mask, _cv.RETR_EXTERNAL, _cv.CHAIN_APPROX_SIMPLE)
    best, best_area = None, 0.0
    for c in cnts:
        a = _cv.contourArea(c)
        if a > best_area and a >= min_area:
            M = _cv.moments(c)
            if M['m00'] > 0:
                best = (M['m10'] / M['m00'], M['m01'] / M['m00'])
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


def camera_extrinsic(name, state, obs, K, mode):
    """T_world_cam : ArUco table (précis), DREAM self-cal figé, ou live 1-frame PnP."""
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


def localize(state, intr, obj_px, table_z, cameras, mode):
    """(x,y,z) base : triangulation si ≥2 vues avec extrinsèque, sinon mono plan-table."""
    extr, rays = {}, []
    for name in cameras:
        if obj_px.get(name) is None:
            continue
        if mode != 'markers' and state.kp[name] is None:
            continue                              # extrinsèque DREAM -> keypoints requis
        kp_2d, valid = state.kp[name] if state.kp[name] is not None else (None, None)
        obs = {'kp_2d': kp_2d, 'valid': valid, 'obj_px': obj_px[name]}
        try:
            T = camera_extrinsic(name, state, obs, intr[name], mode)
        except ValueError as e:
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
            return pixel_to_base(u, v, intr[name], T, table_z), f'mono {name} (plan-table)'
        except ValueError as e:
            print(f'    ⚠️ {name}: déprojection échoue ({e})')
    raise RuntimeError('aucune vue exploitable')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--pi-host', default='10.10.0.221')
    ap.add_argument('--ros-host', default='localhost')
    ap.add_argument('--ros-port', type=int, default=9090)
    ap.add_argument('--weights', default='yolov8n.pt')
    ap.add_argument('--classes', nargs='*', default=None,
                    help='classes COCO à saisir (ex: cup bottle "sports ball")')
    ap.add_argument('--table-z', type=float, default=0.0, help='plan table (m, base) pour le repli mono')
    ap.add_argument('--place', nargs=3, type=float, default=[0.20, 0.15, 0.05])
    ap.add_argument('--speed', type=int, default=40)
    ap.add_argument('--mode', type=int, default=1, choices=[0, 1])
    ap.add_argument('--auto', action='store_true')
    ap.add_argument('--keep-ori', action='store_true',
                    help='garde l\'orientation actuelle du bras (atteignable) au lieu de forcer '
                         'TOP_DOWN_ORI — utile si send_coords ne bouge pas (pose top-down injoignable).')
    ap.add_argument('--approach-ori', nargs=3, type=float, default=None,
                    metavar=('RX', 'RY', 'RZ'), help='orientation d\'approche imposée (deg).')
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--save', action='store_true', help='écrit les frames YOLO annotées (diagnostic)')
    ap.add_argument('--samples', type=int, default=6,
                    help='frames échantillonnées par caméra (détection robuste)')
    ap.add_argument('--detector', choices=['yolo', 'color'], default='yolo',
                    help='yolo (COCO) ou color (blob HSV, robuste pour balle jaune-vert)')
    ap.add_argument('--cameras', nargs='+', default=list(CAMERAS), choices=list(CAMERAS),
                    help='caméras à utiliser. 1 seule -> mono plan-table (utile si une '
                         'extrinsèque DREAM est faible). 2 -> triangulation.')
    ap.add_argument('--extrinsic', choices=['markers', 'static', 'live'], default='markers',
                    help='markers = ArUco table (précis, 0.7px) ; static = DREAM self-cal ; '
                         'live = recalcul PnP chaque frame (si la caméra bouge).')
    ap.add_argument('--hsv-lo', nargs=3, type=int, default=[25, 60, 60],
                    metavar=('H', 'S', 'V'), help='seuil HSV bas (détecteur color)')
    ap.add_argument('--hsv-hi', nargs=3, type=int, default=[45, 255, 255],
                    metavar=('H', 'S', 'V'), help='seuil HSV haut (détecteur color)')
    ap.add_argument('--camera-source', choices=['rosbridge', 'v4l2'], default='rosbridge',
                    help='rosbridge = images du launch multicam (défaut) ; v4l2 = caméra en '
                         'direct si le launch est arrêté.')
    ap.add_argument('--robot-via', choices=['rosbridge', 'socket'], default='rosbridge',
                    help='rosbridge = commandes robot via /to_robot (bridge_tour, UNE seule '
                         'connexion Pi — pas de conflit avec le dashboard) ; socket = TCP direct.')
    args = ap.parse_args()

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
        if args.extrinsic != 'markers':
            raise SystemExit('--camera-source v4l2 exige --extrinsic markers (pas de DREAM live).')
        for name in args.cameras:
            cap = cv2.VideoCapture(CAMERAS[name]['device'], cv2.CAP_V4L2)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640); cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
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
            return detect_color(frame, args.hsv_lo, args.hsv_hi, name=name, save=save)
        return detect_object(model, frame, want, name=name, save=save)

    def get_frame(name):
        if caps:
            ok, f = caps[name].read()
            return f if ok else None
        return state.frame[name]

    if ros is not None:  # attendre les 1ers messages (rosbridge)
        need_dream = args.extrinsic != 'markers'
        print('  ⏳ attente images' + (' + DREAM' if need_dream else '') + '…')
        t0 = time.time()
        while (all(state.frame[n] is None for n in args.cameras)
               or (need_dream and (state.joint_q is None
                                   or all(state.kp[n] is None for n in args.cameras)))):
            time.sleep(0.2)
            if time.time() - t0 > 15:
                raise SystemExit('pas de données — le launch multicam tourne ?')

    # Détection robuste : on échantillonne plusieurs frames par caméra et on garde
    # la détection la plus confiante (la détection YOLO d'un petit objet varie
    # frame-à-frame — 1 seule frame rate souvent la cible dans une des vues).
    obj_px = {}
    for name in args.cameras:
        best_px, best_conf, last_frame = None, 0.0, None
        for _ in range(args.samples):
            frame = get_frame(name)
            if frame is not None:
                last_frame = frame
                px, conf = detect(frame)
                if px is not None and conf > best_conf:
                    best_px, best_conf = px, conf
            time.sleep(0.2)
        # 1 log récap + frame annotée sur la dernière image
        if last_frame is not None:
            detect(last_frame, name=name, save=args.save)
        obj_px[name] = best_px
        print(f'    {name}: objet cible px = {best_px}  (conf max {best_conf:.2f})')

    if all(v is None for v in obj_px.values()):
        raise SystemExit('YOLO n\'a rien détecté sur aucune vue.')

    xyz, method = localize(state, intr, obj_px, args.table_z, args.cameras, args.extrinsic)
    print(f'\n  🎯 objet localisé ({method}) : base = '
          f'({xyz[0]*1000:.0f}, {xyz[1]*1000:.0f}, {xyz[2]*1000:.0f}) mm\n')

    if args.dry_run:
        bridge = None
    elif args.robot_via == 'rosbridge':
        if ros is None:
            raise SystemExit('--robot-via rosbridge exige --camera-source rosbridge (rosbridge actif).')
        bridge = RosbridgeRobot(ros)
        print('  🤖 robot via /to_robot (bridge_tour) — ferme le dashboard pour éviter le cross-talk')
    else:
        bridge = ctrl.Bridge(args.pi_host)
        print(f'  🤖 robot via socket direct {args.pi_host}:5005')

    ctrl.run(bridge, [float(v) for v in xyz], args.place,
             args.speed, args.mode, args.dry_run, args.auto,
             keep_ori=args.keep_ori, ori=args.approach_ori)

    if ros is not None:
        ros.terminate()
    for cap in caps.values():
        cap.release()


if __name__ == '__main__':
    main()
