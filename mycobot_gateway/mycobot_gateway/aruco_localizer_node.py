#!/usr/bin/env python3
"""ArUco workspace localizer — mode robot réel.

Pipeline
--------
1. Souscrit à /camera/image_raw (sensor_msgs/Image).
2. Charge la matrice K et les distorsions depuis cam_0.npz (calibration réelle).
3. Détecte 4 marqueurs ArUco workspace (IDs 0-3, DICT_4X4_1000, taille 50 mm).
   Positions connues dans le repère base du robot (z = 0 = surface table) :
       ID 0 : ( 0.150, -0.150, 0.0 )  [avant-gauche]
       ID 1 : ( 0.150,  0.150, 0.0 )  [avant-droit]
       ID 2 : ( 0.280, -0.150, 0.0 )  [arrière-gauche]
       ID 3 : ( 0.280,  0.150, 0.0 )  [arrière-droit]
   → solvePnP → T_cam→base.
4. Détecte le marqueur objet (ID 10, 40 mm) → transforme en repère base.
5. Publie :
       /aruco/object_pose     (geometry_msgs/PoseStamped)  — objet en repère base
       /aruco/workspace_valid (std_msgs/Bool)               — ≥ 2 marqueurs vus
       /aruco/debug_image     (sensor_msgs/Image)           — frame annotée

Paramètres ROS 2
----------------
  calib_file      : chemin vers cam_0.npz        (défaut : repo-relative)
  ws_marker_size  : côté marqueur workspace en m  (défaut : 0.050)
  obj_marker_size : côté marqueur objet en m       (défaut : 0.040)
  obj_marker_id   : ID ArUco de l'objet            (défaut : 10)
  camera_topic    : topic image                    (défaut : /camera/image_raw)

Usage
-----
  ros2 run mycobot_gateway aruco_localizer
  ros2 run mycobot_gateway aruco_localizer --ros-args \
      -p calib_file:=/path/to/cam_0.npz \
      -p camera_topic:=/cam_0/image_raw
"""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

# ── Chemin par défaut du fichier de calibration ───────────────────────────────
# __file__ : .../mycobot_gateway/mycobot_gateway/aruco_localizer_node.py
# parents[3] : racine du dépôt
_REPO_ROOT = Path(__file__).resolve().parents[3]
_DEFAULT_CALIB = str(_REPO_ROOT / "training" / "calibration" / "cam_0.npz")

# ── Positions des marqueurs workspace dans le repère base (mètres) ────────────
WORKSPACE_MARKER_POSITIONS: dict[int, np.ndarray] = {
    0: np.array([ 0.150, -0.150, 0.0], dtype=np.float64),
    1: np.array([ 0.150,  0.150, 0.0], dtype=np.float64),
    2: np.array([ 0.280, -0.150, 0.0], dtype=np.float64),
    3: np.array([ 0.280,  0.150, 0.0], dtype=np.float64),
}


# ── Utilitaires géométriques ─────────────────────────────────────────────────

def _marker_corners_3d(half: float) -> np.ndarray:
    """4 coins d'un marqueur plat dans son propre repère (z = 0).
    Ordre OpenCV : haut-gauche, haut-droit, bas-droit, bas-gauche.
    """
    return np.array([
        [-half,  half, 0.0],
        [ half,  half, 0.0],
        [ half, -half, 0.0],
        [-half, -half, 0.0],
    ], dtype=np.float64)


def _rotation_matrix_to_quaternion(R: np.ndarray) -> Quaternion:
    """Conversion matrice 3×3 → geometry_msgs/Quaternion (Shepperd stable)."""
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    q = Quaternion()
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        q.w = 0.25 / s
        q.x = (R[2, 1] - R[1, 2]) * s
        q.y = (R[0, 2] - R[2, 0]) * s
        q.z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        q.w = (R[2, 1] - R[1, 2]) / s
        q.x = 0.25 * s
        q.y = (R[0, 1] + R[1, 0]) / s
        q.z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        q.w = (R[0, 2] - R[2, 0]) / s
        q.x = (R[0, 1] + R[1, 0]) / s
        q.y = 0.25 * s
        q.z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        q.w = (R[1, 0] - R[0, 1]) / s
        q.x = (R[0, 2] + R[2, 0]) / s
        q.y = (R[1, 2] + R[2, 1]) / s
        q.z = 0.25 * s
    return q


# ── Nœud principal ────────────────────────────────────────────────────────────

class ArucoLocalizerNode(Node):
    """Localisation ArUco pour le robot réel."""

    def __init__(self) -> None:
        super().__init__("aruco_localizer")

        # ── paramètres ──
        self.declare_parameter("calib_file",      _DEFAULT_CALIB)
        self.declare_parameter("ws_marker_size",  0.050)
        self.declare_parameter("obj_marker_size", 0.040)
        self.declare_parameter("obj_marker_id",   10)
        self.declare_parameter("camera_topic",    "/camera/image_raw")

        calib_file   = self.get_parameter("calib_file").value
        ws_size      = float(self.get_parameter("ws_marker_size").value)
        self._obj_sz = float(self.get_parameter("obj_marker_size").value)
        self._obj_id = int(self.get_parameter("obj_marker_id").value)
        cam_topic    = self.get_parameter("camera_topic").value

        # ── calibration ──
        if not Path(calib_file).exists():
            self.get_logger().error(f"Fichier calibration introuvable : {calib_file}")
            raise FileNotFoundError(calib_file)
        data = np.load(calib_file)
        self._K: np.ndarray = data["mtx"].astype(np.float64)
        self._D: np.ndarray = data["dist"].astype(np.float64)
        self.get_logger().info(
            f"Calibration chargée : fx={self._K[0,0]:.1f}  fy={self._K[1,1]:.1f}"
            f"  cx={self._K[0,2]:.1f}  cy={self._K[1,2]:.1f}"
        )

        # ── détecteur ArUco (API moderne OpenCV ≥ 4.7) ──
        aruco_dict   = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
        aruco_params = cv2.aruco.DetectorParameters()
        self._detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)

        # ── 4 coins 3D de chaque marqueur workspace dans le repère base ──
        half_ws = ws_size / 2.0
        # Les coins sont décalés par rapport au centre du marqueur
        self._ws_corners_base: dict[int, np.ndarray] = {
            mid: (_marker_corners_3d(half_ws) + center)
            for mid, center in WORKSPACE_MARKER_POSITIONS.items()
        }

        # ── transformée cam→base (mise à jour à chaque frame) ──
        self._T_cam_to_base: np.ndarray | None = None  # homogène 4×4

        # ── ROS I/O ──
        self._bridge = CvBridge()
        self._sub   = self.create_subscription(Image, cam_topic, self._image_cb, 5)
        self._pub_pose  = self.create_publisher(PoseStamped, "/aruco/object_pose",     5)
        self._pub_valid = self.create_publisher(Bool,         "/aruco/workspace_valid", 5)
        self._pub_debug = self.create_publisher(Image,        "/aruco/debug_image",     2)

        self.get_logger().info(
            f"Localizer ArUco prêt  |  topic caméra : {cam_topic}"
            f"  |  ID objet : {self._obj_id}"
        )

    # ──────────────────────────────────────────────────────────────────────────

    def _image_cb(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as exc:
            self.get_logger().warning(f"cv_bridge : {exc}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self._detector.detectMarkers(gray)

        valid = False
        if ids is not None:
            ids_flat = ids.flatten().tolist()
            valid = self._update_workspace_frame(corners, ids_flat)
            if valid and self._T_cam_to_base is not None:
                self._try_publish_object(corners, ids_flat, msg.header.stamp)

        self._pub_valid.publish(Bool(data=valid))

        # image de débogage annotée
        debug = frame.copy()
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(debug, corners, ids)
            # annotation des marqueurs workspace trouvés
            ids_flat_vis = ids.flatten().tolist()
            for mid in WORKSPACE_MARKER_POSITIONS:
                if mid in ids_flat_vis:
                    idx = ids_flat_vis.index(mid)
                    c = corners[idx][0].astype(int)
                    cv2.putText(debug, f"WS{mid}", tuple(c[0]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        status_color = (0, 255, 0) if valid else (0, 0, 255)
        cv2.putText(debug, "WORKSPACE OK" if valid else "WORKSPACE MANQUANT",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)
        self._pub_debug.publish(self._bridge.cv2_to_imgmsg(debug, "bgr8"))

    # ──────────────────────────────────────────────────────────────────────────

    def _update_workspace_frame(self, corners, ids_flat: list) -> bool:
        """solvePnP sur tous les marqueurs workspace visibles → T_cam→base."""
        obj_pts: list[np.ndarray] = []
        img_pts: list[np.ndarray] = []

        for i, mid in enumerate(ids_flat):
            if mid not in self._ws_corners_base:
                continue
            obj_pts.append(self._ws_corners_base[mid])          # 4×3 repère base
            img_pts.append(corners[i][0].astype(np.float64))    # 4×2 image

        if len(obj_pts) < 2:   # besoin d'au moins 2 marqueurs (8 points)
            return False

        obj_all = np.concatenate(obj_pts, axis=0)   # (4N)×3
        img_all = np.concatenate(img_pts, axis=0)   # (4N)×2

        ok, rvec, tvec = cv2.solvePnP(
            obj_all, img_all, self._K, self._D,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )
        if not ok:
            return False

        # solvePnP donne : p_cam = R @ p_base + t
        # Donc T_cam_from_base[4×4] transforme un point base → cam
        R, _ = cv2.Rodrigues(rvec)
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3,  3] = tvec.flatten()
        self._T_cam_to_base = T   # T : p_cam = T @ [p_base; 1]
        return True

    # ──────────────────────────────────────────────────────────────────────────

    def _try_publish_object(self, corners, ids_flat: list, stamp) -> None:
        """Détecte le marqueur objet et publie sa pose dans le repère base."""
        if self._obj_id not in ids_flat:
            return

        idx = ids_flat.index(self._obj_id)
        img_obj  = corners[idx][0].astype(np.float64)
        half_obj = self._obj_sz / 2.0
        obj_3d   = _marker_corners_3d(half_obj)

        ok, rvec_obj, tvec_obj = cv2.solvePnP(
            obj_3d, img_obj, self._K, self._D,
            flags=cv2.SOLVEPNP_IPPE_SQUARE,
        )
        if not ok:
            return

        # T_obj_cam : pose de l'objet dans le repère caméra
        R_obj, _ = cv2.Rodrigues(rvec_obj)
        T_obj_cam = np.eye(4, dtype=np.float64)
        T_obj_cam[:3, :3] = R_obj
        T_obj_cam[:3,  3] = tvec_obj.flatten()

        # T_base_from_cam = inv(T_cam_from_base)
        T_base_from_cam = np.linalg.inv(self._T_cam_to_base)
        T_obj_base = T_base_from_cam @ T_obj_cam

        pose = PoseStamped()
        pose.header.stamp    = stamp
        pose.header.frame_id = "base_link"
        pose.pose.position.x = float(T_obj_base[0, 3])
        pose.pose.position.y = float(T_obj_base[1, 3])
        pose.pose.position.z = float(T_obj_base[2, 3])
        pose.pose.orientation = _rotation_matrix_to_quaternion(T_obj_base[:3, :3])

        self._pub_pose.publish(pose)
        self.get_logger().debug(
            f"Objet détecté  x={T_obj_base[0,3]:.3f}  "
            f"y={T_obj_base[1,3]:.3f}  z={T_obj_base[2,3]:.3f}"
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = ArucoLocalizerNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
