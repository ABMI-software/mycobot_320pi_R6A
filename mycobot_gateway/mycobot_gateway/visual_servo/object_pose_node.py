#!/usr/bin/env python3
"""Détection + estimation de pose de l'objet, une instance par caméra (§11.1).

Regroupe `object_detector_node` et `pose_estimator_node` du §11.1 dans un seul
processus : ils échangent un pixel et une image, et les séparer coûterait une
sérialisation d'image par période pour rien (§11.3).

Deux détecteurs, tous deux du §4.2 :

  `color`  seuillage HSV + plus gros blob circulaire. Pas de profondeur propre :
           la position vient de l'intersection du rayon avec le plan table. C'est
           ce qui marche aujourd'hui sur la balle du dépôt.
  `aruco`  marqueur collé sur l'objet, solvePnP sur ses 4 coins -> pose métrique
           complète, profondeur incluse, avec une vraie erreur de reprojection.
           À préférer pour valider la géométrie (§13 Étape 1) : c'est le seul
           mode où l'erreur reportée mesure quelque chose.

⚠ Le marqueur objet ne doit PAS porter un des identifiants des quatre ArUco fixes
de calibration — ils seraient confondus.

Publie, dans le repère base :
    <prefix>/object_pose    geometry_msgs/PoseStamped
    <prefix>/detection      std_msgs/String (JSON : facteurs de confiance §7.2)

L'horodatage republié est celui de l'IMAGE, jamais l'heure courante : c'est lui
qui permet à la fusion de rejeter une donnée périmée et au contrôleur de
compenser la latence (§8.2).
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import Image
from std_msgs.msg import String
from tf2_ros import StaticTransformBroadcaster

_PKG_DIR = Path(__file__).resolve().parent
_REPO_ROOT = _PKG_DIR.parents[2]
if str(_REPO_ROOT / 'mycobot_gateway') not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT / 'mycobot_gateway'))

from mycobot_gateway.vision.camera_registry import load_intrinsics  # noqa: E402

_CALIBRATION_DIR = _REPO_ROOT / 'training' / 'calibration'

# Profil capteur : profondeur 1 et best-effort, pour ne jamais traiter une image
# en retard (§11.3 « profondeur de file égale à 1 »).
SENSOR_QOS = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1,
    durability=QoSDurabilityPolicy.VOLATILE,
)


def detect_color_blob(bgr, hsv_lo, hsv_hi, min_area, max_area, min_circularity,
                      border) -> dict | None:
    """Plus gros blob de la plage HSV -> {u, v, area, circularity} ou None.

    Les noyaux morphologiques suivent la résolution : figés, ils déchiquettent le
    masque en haute résolution et la circularité passe sous le seuil dès qu'une
    ombre traverse (constat repris de pick_and_place_vision_live.detect_color).
    """
    h, w = bgr.shape[:2]
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, np.asarray(hsv_lo, np.uint8), np.asarray(hsv_hi, np.uint8))
    scale = max(1.0, w / 640.0)
    k_open, k_close = int(3 * scale) | 1, int(9 * scale) | 1
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, np.ones((k_open, k_open), np.uint8))
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((k_close, k_close), np.uint8))
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    best, best_area = None, 0.0
    for c in contours:
        area = cv2.contourArea(c)
        if not (min_area <= area <= max_area) or area <= best_area:
            continue
        perimeter = cv2.arcLength(c, True)
        circularity = 4.0 * np.pi * area / (perimeter ** 2) if perimeter > 0 else 0.0
        if circularity < min_circularity:
            continue
        m = cv2.moments(c)
        if m['m00'] <= 0:
            continue
        u, v = m['m10'] / m['m00'], m['m01'] / m['m00']
        if not (border <= u <= w - border and border <= v <= h - border):
            continue
        best, best_area = {'u': u, 'v': v, 'area': float(area),
                           'circularity': float(circularity)}, area
    return best


def detect_object_aruco(bgr, marker_id: int) -> dict | None:
    """Marqueur objet -> {u, v, corners, area} ou None."""
    params = cv2.aruco.DetectorParameters()
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    detector = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000), params)
    corners, ids, _ = detector.detectMarkers(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY))
    if ids is None:
        return None
    for c, i in zip(corners, ids.flatten()):
        if int(i) != marker_id:
            continue
        pts = c.reshape(4, 2).astype(np.float64)
        centre = pts.mean(axis=0)
        return {'u': float(centre[0]), 'v': float(centre[1]), 'corners': pts,
                'area': float(cv2.contourArea(pts.astype(np.float32)))}
    return None


def sharpness_score(bgr, u, v, radius=24) -> float:
    """Netteté locale ∈[0,1] — variance du laplacien autour de l'objet.

    Une cible floue (bougé, mise au point perdue) donne un centroïde qui dérive
    de plusieurs pixels ; le §7.2 en fait un facteur de confiance à part entière.
    Normalisation à 300 : au-delà l'image est franchement nette, inutile de
    récompenser davantage.
    """
    h, w = bgr.shape[:2]
    x0, x1 = max(0, int(u - radius)), min(w, int(u + radius))
    y0, y1 = max(0, int(v - radius)), min(h, int(v + radius))
    if x1 - x0 < 4 or y1 - y0 < 4:
        return 0.0
    roi = cv2.cvtColor(bgr[y0:y1, x0:x1], cv2.COLOR_BGR2GRAY)
    return float(np.clip(cv2.Laplacian(roi, cv2.CV_64F).var() / 300.0, 0.0, 1.0))


class ObjectPoseNode(Node):
    """Une caméra -> une pose objet dans base_link, avec ses facteurs de confiance."""

    def __init__(self):
        super().__init__('object_pose_node')
        self.declare_parameter('camera', 'arducam')
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('calib_stem', 'cam_3')
        self.declare_parameter('extrinsic', '')
        self.declare_parameter('output_prefix', '/visual_servo/arducam')
        self.declare_parameter('detector', 'color')
        self.declare_parameter('table_z', 0.0)
        # Diamètre de l'objet : le rayon visuel est coupé à la hauteur de son
        # CENTRE, pas à celle de la table. Défaut = la balle du dépôt, mesurée
        # le 18/08 par les deux caméras indépendamment (71,7 et 71,4 mm). Une
        # valeur fausse décale la position calculée de la moitié de l'erreur —
        # les 20 mm d'origine décalaient de 26 mm.
        self.declare_parameter('object_height', 0.071)
        self.declare_parameter('hsv_low', [25, 80, 80])
        self.declare_parameter('hsv_high', [45, 255, 255])
        self.declare_parameter('min_area', 200.0)
        self.declare_parameter('max_area', 2500.0)
        self.declare_parameter('min_circularity', 0.30)
        self.declare_parameter('border_px', 25)
        self.declare_parameter('aruco_id', 40)
        self.declare_parameter('aruco_size', 0.030)
        self.declare_parameter('publish_debug_image', False)
        # Capture directe : -1 = s'abonner au topic image (mode historique,
        # partagé avec la chaîne DREAM) ; >= 0 = ouvrir soi-même /dev/videoN.
        #
        # Une Image 640x480 bgr8 pèse 921 ko. La faire transiter par le
        # middleware pour un seul consommateur plafonne la perception à 1-8 Hz
        # de façon erratique (mesuré le 18/08) — sous les 10 Hz du §11.3, et
        # surtout sous la fenêtre de fraîcheur de la fusion, qui déclare alors
        # la cible perdue alors que les détections sont bonnes. Le §11.3
        # recommande justement la communication intra-processus : ici le nœud
        # est le seul consommateur, donc autant lire la caméra directement et
        # ne rien sérialiser du tout.
        self.declare_parameter('camera_device', -1)
        self.declare_parameter('capture_rate', 15.0)
        self.declare_parameter('manual_exposure', -1)

        p = self.get_parameter
        self.camera = p('camera').value
        self.detector = p('detector').value
        self.table_z = float(p('table_z').value)
        self.object_height = float(p('object_height').value)
        prefix = p('output_prefix').value

        self.K, self.dist = load_intrinsics(p('calib_stem').value)
        if self.K is None:
            raise SystemExit(f"intrinsèque « {p('calib_stem').value} » introuvable — "
                             f'calibre la caméra avant de lancer l\'asservissement.')
        if self.dist is None or self.dist.size == 0:
            self.dist = np.zeros(5)

        extrinsic_path = p('extrinsic').value or str(
            _CALIBRATION_DIR / f'{self.camera}_extrinsic_servo.yaml')
        self.T_world_cam, self.extrinsic_meta = self._load_extrinsic(extrinsic_path)

        self.bridge = CvBridge()
        self.pub_pose = self.create_publisher(PoseStamped, f'{prefix}/object_pose', 1)
        self.pub_detection = self.create_publisher(String, f'{prefix}/detection', 1)
        self.pub_debug = (self.create_publisher(Image, f'{prefix}/debug_image', 1)
                          if p('publish_debug_image').value else None)

        device = int(p('camera_device').value)
        self.capture = None
        if device >= 0:
            self.capture = self._open_device(device, int(p('manual_exposure').value))
            rate = float(p('capture_rate').value)
            self.create_timer(1.0 / rate, self.on_capture)
            self.get_logger().info(
                f'[{self.camera}] capture directe /dev/video{device} à {rate:.0f}Hz '
                f'(aucune image ne transite par le middleware)')
        else:
            self.create_subscription(Image, p('image_topic').value, self.on_image,
                                     SENSOR_QOS)

        self._tf_static = StaticTransformBroadcaster(self)
        self._broadcast_camera_frame()

        rms = self.extrinsic_meta.get('rms_reproj_px')
        rms_text = f'{rms:.2f} px' if rms is not None else 'non reporté'
        self.get_logger().info(
            f'[{self.camera}] détecteur={self.detector} '
            f'extrinsèque={Path(extrinsic_path).name} (RMS {rms_text})')
        self._warn_if_extrinsic_unvalidated()

    def _load_extrinsic(self, path: str):
        f = Path(path)
        if not f.is_file():
            raise SystemExit(
                f'extrinsèque introuvable : {f}\n'
                f'Lance d\'abord :  python training/calibration/'
                f'calibrate_camera_base_extrinsic.py --camera {self.camera}')
        data = yaml.safe_load(f.read_text())
        T = np.asarray(data['T_world_cam'], dtype=np.float64)
        if T.shape != (4, 4):
            raise SystemExit(f'T_world_cam doit être 4x4 dans {f}, reçu {T.shape}')
        return T, data

    def _warn_if_extrinsic_unvalidated(self):
        """Une extrinsèque sans validation leave-one-out n'est pas mesurée (§5.3)."""
        loo = self.extrinsic_meta.get('validation_leave_one_out')
        if not loo:
            self.get_logger().warn(
                f'[{self.camera}] extrinsèque sans validation leave-one-out : son RMS '
                f'ne dit rien sur un point neuf. Recalibre avec '
                f'calibrate_camera_base_extrinsic.py avant toute saisie.')
            return
        worst = max(r['plane_error_mm'] for r in loo)
        level = self.get_logger().info if worst < 10.0 else self.get_logger().warn
        level(f'[{self.camera}] erreur sur point neuf (leave-one-out) : {worst:.1f} mm')

    def _broadcast_camera_frame(self):
        """base_link -> camera_<nom> (§11.2) — statique tant que le support ne bouge pas."""
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = 'base_link'
        t.child_frame_id = f'camera_{self.camera}'
        t.transform.translation.x = float(self.T_world_cam[0, 3])
        t.transform.translation.y = float(self.T_world_cam[1, 3])
        t.transform.translation.z = float(self.T_world_cam[2, 3])
        qx, qy, qz, qw = _quaternion_from_matrix(self.T_world_cam[:3, :3])
        t.transform.rotation.x, t.transform.rotation.y = qx, qy
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self._tf_static.sendTransform(t)

    def _ray_to_plane(self, u, v, plane_z) -> np.ndarray:
        """Pixel -> point du plan z=plane_z, repère base (§4.2, dernière option)."""
        undistorted = cv2.undistortPoints(
            np.array([[[u, v]]], dtype=np.float64), self.K, self.dist).reshape(2)
        d_cam = np.array([undistorted[0], undistorted[1], 1.0])
        origin = self.T_world_cam[:3, 3]
        d_world = self.T_world_cam[:3, :3] @ d_cam
        if abs(d_world[2]) < 1e-9:
            raise ValueError('rayon parallèle au plan table')
        s = (plane_z - origin[2]) / d_world[2]
        if s <= 0:
            raise ValueError('intersection derrière la caméra — extrinsèque fausse ?')
        return origin + s * d_world

    def _pose_from_aruco(self, corners, size) -> tuple[np.ndarray, float]:
        """(position base, erreur de reprojection px) d'un marqueur collé sur l'objet."""
        h = size / 2.0
        obj = np.array([[-h, h, 0.0], [h, h, 0.0], [h, -h, 0.0], [-h, -h, 0.0]])
        ok, rvec, tvec = cv2.solvePnP(obj, corners, self.K, self.dist,
                                      flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            raise ValueError('solvePnP marqueur objet a échoué')
        rvec, tvec = cv2.solvePnPRefineLM(obj, corners, self.K, self.dist, rvec, tvec)
        proj, _ = cv2.projectPoints(obj, rvec, tvec, self.K, self.dist)
        reproj = float(np.sqrt((np.linalg.norm(
            proj.reshape(-1, 2) - corners, axis=1) ** 2).mean()))
        in_base = self.T_world_cam @ np.append(tvec.ravel(), 1.0)
        return in_base[:3], reproj

    def _open_device(self, device: int, exposure: int):
        """Ouvre la caméra en V4L2/MJPG. MJPG est indispensable : en YUYV deux
        caméras sur le même contrôleur USB se partagent mal la bande passante."""
        capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if not capture.isOpened():
            raise SystemExit(f'/dev/video{device} illisible — déjà ouvert par un '
                             f'autre process (camera_publisher ?) ou index faux.')
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        capture.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if exposure >= 0:
            # Même ordonnancement que camera_publisher : un contrôle v4l2 posé
            # avant la première lecture est réinitialisé au démarrage du flux.
            for _ in range(5):
                capture.read()
            capture.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)
            capture.set(cv2.CAP_PROP_EXPOSURE, exposure)
        for _ in range(5):
            capture.read()
        return capture

    def on_capture(self):
        ok, frame = self.capture.read()
        if not ok:
            return
        self.process_frame(frame, self.get_clock().now())

    def on_image(self, msg: Image):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self.process_frame(frame, msg.header.stamp, from_message=True)

    def process_frame(self, frame, stamp_source, from_message: bool = False):
        """Détecte l'objet et publie sa pose. `stamp_source` horodate l'IMAGE."""
        stamp_msg = stamp_source if from_message else stamp_source.to_msg()
        stamp = stamp_msg.sec + stamp_msg.nanosec * 1e-9
        detection, position, reproj = None, None, 0.0
        error = ''

        try:
            if self.detector == 'aruco':
                detection = detect_object_aruco(
                    frame, int(self.get_parameter('aruco_id').value))
                if detection is not None:
                    position, reproj = self._pose_from_aruco(
                        detection['corners'],
                        float(self.get_parameter('aruco_size').value))
            else:
                g = self.get_parameter
                detection = detect_color_blob(
                    frame, g('hsv_low').value, g('hsv_high').value,
                    float(g('min_area').value), float(g('max_area').value),
                    float(g('min_circularity').value), int(g('border_px').value))
                if detection is not None:
                    # Le blob est le CENTRE de la balle : le rayon est coupé à la
                    # hauteur de son centre, pas à celle de la table, sinon la
                    # position dérive vers la caméra du rayon de l'objet.
                    position = self._ray_to_plane(
                        detection['u'], detection['v'],
                        self.table_z + self.object_height / 2.0)
        except ValueError as exc:
            error = str(exc)

        valid = position is not None
        payload = {
            'camera': self.camera,
            'stamp': stamp,
            'valid': valid,
            'detector': self.detector,
            'error': error,
        }
        if detection is not None:
            payload.update({
                'pixel': [detection['u'], detection['v']],
                'pixel_area': detection['area'],
                'sharpness': sharpness_score(frame, detection['u'], detection['v']),
                'reprojection_px': reproj,
                # Le seuillage couleur n'a pas de score probabiliste : la
                # circularité en tient lieu (1 = disque parfait). En aruco, la
                # détection est binaire — un marqueur décodé l'est ou ne l'est pas.
                'detector_score': float(detection.get('circularity', 1.0)),
            })
        if valid:
            payload['position'] = [float(x) for x in position]

        self.pub_detection.publish(String(data=json.dumps(payload)))
        if valid:
            pose = PoseStamped()
            pose.header.stamp = stamp_msg
            pose.header.frame_id = 'base_link'
            pose.pose.position.x = float(position[0])
            pose.pose.position.y = float(position[1])
            pose.pose.position.z = float(position[2])
            pose.pose.orientation.w = 1.0
            self.pub_pose.publish(pose)

        if self.pub_debug is not None:
            self._publish_debug(frame, detection, stamp_msg)

    def _publish_debug(self, frame, detection, stamp_msg):
        if detection is not None:
            cv2.circle(frame, (int(detection['u']), int(detection['v'])), 8,
                       (0, 255, 0), 2)
        out = self.bridge.cv2_to_imgmsg(frame, 'bgr8')
        out.header.stamp = stamp_msg
        out.header.frame_id = f'camera_{self.camera}'
        self.pub_debug.publish(out)

    def destroy_node(self):
        if self.capture is not None:
            self.capture.release()
        super().destroy_node()


def _quaternion_from_matrix(R) -> tuple[float, float, float, float]:
    """(x, y, z, w) depuis une rotation 3x3 — Shepperd, branche du plus grand terme."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0.0:
        s = 0.5 / np.sqrt(trace + 1.0)
        return ((R[2, 1] - R[1, 2]) * s, (R[0, 2] - R[2, 0]) * s,
                (R[1, 0] - R[0, 1]) * s, 0.25 / s)
    i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = 2.0 * np.sqrt(1.0 + R[i, i] - R[j, j] - R[k, k])
    q = [0.0, 0.0, 0.0]
    q[i] = 0.25 * s
    q[j] = (R[j, i] + R[i, j]) / s
    q[k] = (R[k, i] + R[i, k]) / s
    return q[0], q[1], q[2], (R[k, j] - R[j, k]) / s


def main(args=None):
    rclpy.init(args=args)
    node = ObjectPoseNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
