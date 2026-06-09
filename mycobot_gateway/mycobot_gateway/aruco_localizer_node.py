#!/usr/bin/env python3
"""ArUco workspace localizer — mode robot réel.

Pipeline
--------
1. Souscrit à /camera/image_raw (sensor_msgs/Image).
2. Charge la matrice K et les distorsions depuis cam_0.npz (calibration réelle).
3. Détecte les 4 marqueurs ArUco workspace (DICT_4X4_1000, taille 25 mm) dont
   les positions dans le repère base sont chargées depuis
   training/calibration/workspace_markers.yaml (même source que la calibration
   extrinsèque §3.2, indexées par ID ArUco réel) :
       ID 19 : ( 0.065, -0.241, 0.0 )  [proche-droite du robot]
       ID 25 : ( 0.365, -0.241, 0.0 )  [loin-droite   du robot]
       ID 23 : ( 0.065,  0.241, 0.0 )  [proche-gauche du robot]
       ID 26 : ( 0.365,  0.241, 0.0 )  [loin-gauche   du robot]
   → solvePnP → T_cam→base.
4. Détecte le marqueur objet (ID 10, 25 mm) → transforme en repère base.
5. Publie :
       /aruco/object_pose     (geometry_msgs/PoseStamped)  — objet en repère base
       /aruco/workspace_valid (std_msgs/Bool)               — ≥ 2 marqueurs vus
       /aruco/debug_image     (sensor_msgs/Image)           — frame annotée

Paramètres ROS 2
----------------
  calib_file      : chemin vers cam_0.npz        (défaut : repo-relative)
  markers_yaml    : positions marqueurs sol YAML  (défaut : workspace_markers.yaml)
  ws_marker_size  : côté marqueur workspace en m  (défaut : 0.025)
  obj_marker_size : côté marqueur objet en m       (défaut : 0.025)
  obj_marker_id   : ID ArUco de l'objet            (défaut : 10)
  camera_topic    : topic image                    (défaut : /camera/image_raw)
    camera_height_m : hauteur caméra attendue (base→cam, m) (défaut : 0.57)

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
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, Quaternion
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, Int32MultiArray


_ARUCO_DICT_BY_NAME: dict[str, int] = {}
for _name in [
    "DICT_4X4_50",
    "DICT_4X4_100",
    "DICT_4X4_250",
    "DICT_4X4_1000",
    "DICT_5X5_50",
    "DICT_5X5_100",
    "DICT_5X5_250",
    "DICT_5X5_1000",
    "DICT_6X6_50",
    "DICT_6X6_100",
    "DICT_6X6_250",
    "DICT_6X6_1000",
    "DICT_7X7_50",
    "DICT_7X7_100",
    "DICT_7X7_250",
    "DICT_7X7_1000",
    "DICT_APRILTAG_16h5",
    "DICT_APRILTAG_25h9",
    "DICT_APRILTAG_36h10",
    "DICT_APRILTAG_36h11",
]:
    if hasattr(cv2.aruco, _name):
        _ARUCO_DICT_BY_NAME[_name] = getattr(cv2.aruco, _name)

# ── Chemin par défaut du fichier de calibration ───────────────────────────────
def _resolve_default_calib() -> str:
    here = Path(__file__).resolve()
    # Works when running from source, build/, or install/ symlinked layouts.
    for p in [here, *here.parents]:
        cand = p / "training" / "calibration" / "cam_0.npz"
        if cand.exists():
            return str(cand)
    # Stable workspace fallback used on this machine.
    return "/home/genji/ros_jazzy/src/mycobot_R6A/training/calibration/cam_0.npz"


_DEFAULT_CALIB = _resolve_default_calib()


def _resolve_default_markers_yaml() -> str:
    here = Path(__file__).resolve()
    for p in [here, *here.parents]:
        cand = p / "training" / "calibration" / "workspace_markers.yaml"
        if cand.exists():
            return str(cand)
    return "/home/genji/ros_jazzy/src/mycobot_R6A/training/calibration/workspace_markers.yaml"


_DEFAULT_MARKERS_YAML = _resolve_default_markers_yaml()

# ── Positions des marqueurs workspace dans le repère base (mètres) ────────────
# Repère base_link (X=avant, Y=gauche REP-103, Z=haut). Clé = ID ArUco RÉEL.
# Source de vérité : training/calibration/workspace_markers.yaml (mêmes valeurs
# que la calibration extrinsèque §3.2). Le fallback ci-dessous n'est utilisé que
# si le YAML est introuvable.
# NB : l'étiquetage gauche/droite initial était inversé en miroir ; Y a été
#      corrigé le 9 juin 2026 (validé : caméra à z=+0.37 m, reproj < 0.6 px).
#   19 = proche-droite   23 = proche-gauche
#   25 = loin-droite     26 = loin-gauche
_FALLBACK_WORKSPACE_POSITIONS: dict[int, np.ndarray] = {
    19: np.array([0.065, -0.241, 0.0], dtype=np.float64),  # proche-droite
    25: np.array([0.365, -0.241, 0.0], dtype=np.float64),  # loin-droite
    23: np.array([0.065,  0.241, 0.0], dtype=np.float64),  # proche-gauche
    26: np.array([0.365,  0.241, 0.0], dtype=np.float64),  # loin-gauche
}


def _load_workspace_positions(yaml_path: str) -> tuple[dict[int, np.ndarray], float | None, str]:
    """Charge les centres des marqueurs sol depuis workspace_markers.yaml.

    Retourne (positions_par_id, marker_size_m_ou_None, source).
    Repli sur _FALLBACK_WORKSPACE_POSITIONS si le fichier est absent/illisible.
    """
    try:
        if yaml_path and Path(yaml_path).exists():
            with open(yaml_path, "r") as fh:
                data = yaml.safe_load(fh) or {}
            raw = data.get("markers", {}) or {}
            positions = {
                int(mid): np.asarray(xyz, dtype=np.float64)
                for mid, xyz in raw.items()
            }
            size = data.get("marker_size_m")
            if positions:
                return positions, (float(size) if size is not None else None), yaml_path
    except Exception:
        pass
    return dict(_FALLBACK_WORKSPACE_POSITIONS), None, "(fallback codé en dur)"


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
        self.declare_parameter("markers_yaml",    _DEFAULT_MARKERS_YAML)
        self.declare_parameter("ws_marker_size",  0.025)
        self.declare_parameter("obj_marker_size", 0.025)
        self.declare_parameter("obj_marker_id",   10)
        self.declare_parameter("ws_marker_ids",   [0, 1, 2, 3])
        self.declare_parameter("auto_workspace_ids", True)
        self.declare_parameter("auto_object_from_center", True)
        self.declare_parameter("aruco_dict_name", "DICT_4X4_1000")
        self.declare_parameter("aruco_try_multiple_dicts", False)
        self.declare_parameter(
            "aruco_dict_candidates",
            ["DICT_4X4_1000", "DICT_5X5_1000", "DICT_6X6_1000", "DICT_7X7_1000", "DICT_APRILTAG_36h11"],
        )
        self.declare_parameter("aruco_detect_scale", 1.6)
        self.declare_parameter("camera_topic",    "/camera/image_raw")
        self.declare_parameter("camera_height_m", 0.57)

        calib_file   = self.get_parameter("calib_file").value
        if not calib_file:
            calib_file = _DEFAULT_CALIB
        markers_yaml = self.get_parameter("markers_yaml").value
        if not markers_yaml:
            markers_yaml = _DEFAULT_MARKERS_YAML
        ws_size      = float(self.get_parameter("ws_marker_size").value)
        self._obj_sz = float(self.get_parameter("obj_marker_size").value)
        self._obj_id = int(self.get_parameter("obj_marker_id").value)
        self._ws_marker_ids = [int(x) for x in self.get_parameter("ws_marker_ids").value]
        self._auto_ws_ids = bool(self.get_parameter("auto_workspace_ids").value)
        self._auto_obj_center = bool(self.get_parameter("auto_object_from_center").value)
        dict_name = str(self.get_parameter("aruco_dict_name").value)
        self._aruco_try_multiple_dicts = bool(self.get_parameter("aruco_try_multiple_dicts").value)
        dict_candidates = [str(x) for x in self.get_parameter("aruco_dict_candidates").value]
        self._aruco_detect_scale = float(self.get_parameter("aruco_detect_scale").value)
        cam_topic    = self.get_parameter("camera_topic").value
        self._camera_height_m = float(self.get_parameter("camera_height_m").value)

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

        # ── détecteur ArUco (compat OpenCV ancien + récent) ──
        self._dict_name = dict_name if dict_name in _ARUCO_DICT_BY_NAME else "DICT_4X4_1000"
        self._aruco_dict = cv2.aruco.getPredefinedDictionary(_ARUCO_DICT_BY_NAME[self._dict_name])
        self._aruco_dict_candidates = [
            name for name in dict_candidates if name in _ARUCO_DICT_BY_NAME
        ]
        if not self._aruco_dict_candidates:
            self._aruco_dict_candidates = [self._dict_name]
        if self._dict_name not in self._aruco_dict_candidates:
            self._aruco_dict_candidates.insert(0, self._dict_name)

        self._use_new_aruco_api = hasattr(cv2.aruco, "ArucoDetector")
        if self._use_new_aruco_api:
            self._aruco_params = cv2.aruco.DetectorParameters()
        else:
            self._aruco_params = cv2.aruco.DetectorParameters_create()
        # More permissive settings for real camera noise / distant tags.
        self._aruco_params.adaptiveThreshWinSizeMin = 3
        self._aruco_params.adaptiveThreshWinSizeMax = 61
        self._aruco_params.adaptiveThreshWinSizeStep = 4
        self._aruco_params.minMarkerPerimeterRate = 0.01
        self._aruco_params.maxMarkerPerimeterRate = 4.0
        if hasattr(cv2.aruco, "CORNER_REFINE_SUBPIX"):
            self._aruco_params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

        self._active_detect_dict = self._dict_name

        # ── positions des marqueurs sol (chargées depuis workspace_markers.yaml) ──
        self._ws_positions, yaml_size, ws_src = _load_workspace_positions(markers_yaml)
        # Taille marqueur : priorité au YAML s'il la fournit, sinon paramètre.
        if yaml_size is not None:
            ws_size = yaml_size
        self.get_logger().info(
            f"Positions marqueurs sol : {sorted(self._ws_positions.keys())}  "
            f"(source : {ws_src}, taille {ws_size*1000:.0f} mm)"
        )

        # ── 4 coins 3D de chaque marqueur workspace dans le repère base ──
        half_ws = ws_size / 2.0
        # Les coins sont décalés par rapport au centre du marqueur.
        # Clé = ID ArUco RÉEL (plus de slots abstraits → mapping direct).
        self._ws_corners_base: dict[int, np.ndarray] = {
            mid: (_marker_corners_3d(half_ws) + center)
            for mid, center in self._ws_positions.items()
        }

        # ── transformée cam→base (mise à jour à chaque frame) ──
        self._T_cam_to_base: np.ndarray | None = None  # homogène 4×4
        self._active_ws_ids: list[int] = []

        # ── ROS I/O ──
        self._bridge = CvBridge()
        self._sub   = self.create_subscription(Image, cam_topic, self._image_cb, 5)
        self._pub_pose  = self.create_publisher(PoseStamped, "/aruco/object_pose",     5)
        self._pub_valid = self.create_publisher(Bool,         "/aruco/workspace_valid", 5)
        self._pub_debug = self.create_publisher(Image,        "/aruco/debug_image",     2)
        self._pub_ids   = self.create_publisher(Int32MultiArray, "/aruco/detected_ids", 5)
        self._last_ids: list[int] = []

        self.get_logger().info(
            f"Localizer ArUco prêt  |  topic caméra : {cam_topic}"
            f"  |  ID objet : {self._obj_id}"
            f"  |  dict primaire : {self._dict_name}"
            f"  |  multi-dict : {self._aruco_try_multiple_dicts}"
            f"  |  detect-scale : {self._aruco_detect_scale:.2f}"
            f"  |  hauteur caméra attendue : {self._camera_height_m:.3f} m"
        )

    # ──────────────────────────────────────────────────────────────────────────

    def _image_cb(self, msg: Image) -> None:
        try:
            frame = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        except Exception as exc:
            self.get_logger().warning(f"cv_bridge : {exc}")
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids = self._detect_markers(gray)

        valid = False
        ids_flat: list[int] = []
        if ids is not None:
            ids_flat = ids.flatten().tolist()
            valid = self._update_workspace_frame(corners, ids_flat)
            if valid and self._T_cam_to_base is not None:
                self._try_publish_object(corners, ids_flat, msg.header.stamp)

        # Publish all detected IDs for quick field diagnostics.
        self._pub_ids.publish(Int32MultiArray(data=[int(x) for x in ids_flat]))
        if ids_flat != self._last_ids:
            self._last_ids = ids_flat
            self.get_logger().info(f"IDs détectés: {ids_flat}")

        self._pub_valid.publish(Bool(data=valid))

        # image de débogage annotée
        debug = frame.copy()
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(debug, corners, ids)
            # annotation des marqueurs workspace trouvés
            ids_flat_vis = ids.flatten().tolist()
            for mid in self._ws_positions:
                if mid in ids_flat_vis:
                    idx = ids_flat_vis.index(mid)
                    c = corners[idx][0].astype(int)
                    cv2.putText(debug, f"WS{mid}", tuple(c[0]),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        status_color = (0, 255, 0) if valid else (0, 0, 255)
        cv2.putText(debug, "WORKSPACE OK" if valid else "WORKSPACE MANQUANT",
                    (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)
        cv2.putText(debug, f"DICT: {self._active_detect_dict}",
                    (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 200, 0), 2)
        self._pub_debug.publish(self._bridge.cv2_to_imgmsg(debug, "bgr8"))

    def _detect_markers(self, gray: np.ndarray):
        def _run_with_dict(dict_name: str):
            d = cv2.aruco.getPredefinedDictionary(_ARUCO_DICT_BY_NAME[dict_name])
            scaled = gray
            inv_scale = 1.0
            if self._aruco_detect_scale > 1.01:
                scaled = cv2.resize(
                    gray,
                    dsize=None,
                    fx=self._aruco_detect_scale,
                    fy=self._aruco_detect_scale,
                    interpolation=cv2.INTER_LINEAR,
                )
                inv_scale = 1.0 / self._aruco_detect_scale
            if self._use_new_aruco_api:
                detector = cv2.aruco.ArucoDetector(d, self._aruco_params)
                c, i, _ = detector.detectMarkers(scaled)
            else:
                c, i, _ = cv2.aruco.detectMarkers(scaled, d, parameters=self._aruco_params)

            if inv_scale != 1.0 and c is not None:
                c = [(corner * inv_scale).astype(np.float32) for corner in c]
            return c, i

        if not self._aruco_try_multiple_dicts:
            self._active_detect_dict = self._dict_name
            return _run_with_dict(self._dict_name)

        best = None  # (score, dict_name, corners, ids)
        for name in self._aruco_dict_candidates:
            c, i = _run_with_dict(name)
            count = 0 if i is None else int(len(i))
            includes_obj = False
            if i is not None:
                includes_obj = self._obj_id in [int(x) for x in i.flatten().tolist()]
            score = count + (0.25 if includes_obj else 0.0)
            if best is None or score > best[0]:
                best = (score, name, c, i)

        if best is None:
            self._active_detect_dict = self._dict_name
            return [], None

        self._active_detect_dict = best[1]
        return best[2], best[3]

    # ──────────────────────────────────────────────────────────────────────────

    def _update_workspace_frame(self, corners, ids_flat: list) -> bool:
        """solvePnP sur tous les marqueurs workspace visibles → T_cam→base.

        Les positions 3D sont indexées par l'ID ArUco RÉEL (chargées depuis
        workspace_markers.yaml), donc on associe directement chaque ID détecté à
        sa position connue — aucune recherche de permutation slot↔ID n'est
        nécessaire.
        """

        def _solve_from_ids(marker_ids: list[int]):
            id_to_index = {int(mid): i for i, mid in enumerate(ids_flat)}
            obj_pts = []
            img_pts = []
            for marker_id in marker_ids:
                idx = id_to_index[marker_id]
                obj_pts.append(self._ws_corners_base[marker_id])
                img_pts.append(corners[idx][0].astype(np.float64))
            obj_all = np.concatenate(obj_pts, axis=0)
            img_all = np.concatenate(img_pts, axis=0)

            def _reproj(rv, tv):
                proj, _ = cv2.projectPoints(obj_all, rv, tv, self._K, self._D)
                return float(np.mean(np.linalg.norm(proj.reshape(-1, 2) - img_all, axis=1)))

            def _cam_z(rv, tv):
                R_tmp, _ = cv2.Rodrigues(rv)
                return float((-R_tmp.T @ tv.flatten())[2])

            # Pour exactement 4 marqueurs coplanaires, IPPE retourne 2 solutions
            # (l'objet et son miroir). On utilise les centres pour satisfaire la
            # contrainte, on garde la solution physiquement valide (caméra
            # au-dessus du plan, cam_z > 0), puis on raffine avec tous les coins.
            centers_3d = np.array([p.mean(axis=0) for p in obj_pts], dtype=np.float64)
            centers_2d = np.array([p.mean(axis=0) for p in img_pts], dtype=np.float64)
            candidates = []
            if len(marker_ids) == 4:
                try:
                    retval, rvecs, tvecs, _ = cv2.solvePnPGeneric(
                        centers_3d, centers_2d, self._K, self._D,
                        flags=cv2.SOLVEPNP_IPPE,
                    )
                    for i in range(retval):
                        rv, tv = rvecs[i], tvecs[i]
                        ok2, rv2, tv2 = cv2.solvePnP(
                            obj_all, img_all, self._K, self._D,
                            useExtrinsicGuess=True, rvec=rv.copy(), tvec=tv.copy(),
                            flags=cv2.SOLVEPNP_ITERATIVE,
                        )
                        if ok2:
                            rv, tv = rv2, tv2
                        candidates.append((rv, tv))
                except cv2.error:
                    pass
            # Toujours proposer une solution ITERATIVE (gère aussi le cas >4).
            ok, rvec, tvec = cv2.solvePnP(
                obj_all, img_all, self._K, self._D,
                flags=cv2.SOLVEPNP_ITERATIVE,
            )
            if ok:
                candidates.append((rvec, tvec))
            if not candidates:
                return None

            # Sélection : on préfère une caméra au-dessus du plan dont la hauteur
            # estimée colle à l'attendu, puis la reprojection minimale.
            best = None  # (score, rv, tv, reproj)
            for rv, tv in candidates:
                reproj = _reproj(rv, tv)
                cam_h = _cam_z(rv, tv)
                score = reproj + abs(cam_h - self._camera_height_m) * 100.0
                if cam_h <= 0.0:
                    score += 10000.0  # rejette la pose miroir (sous le plan)
                if best is None or score < best[0]:
                    best = (score, rv, tv, reproj)
            _, rv, tv, reproj = best
            return rv, tv, reproj

        # Marqueurs sol visibles dont on connaît la position (hors objet).
        present_ids = [
            int(m) for m in ids_flat
            if int(m) in self._ws_positions and int(m) != self._obj_id
        ]
        # IPPE exige 4 points coplanaires pour lever l'ambiguïté miroir.
        if len(present_ids) < 4:
            self._active_ws_ids = []
            return False

        solved = _solve_from_ids(present_ids)
        if solved is None:
            self._active_ws_ids = []
            return False
        rvec, tvec, _ = solved
        used_ids = list(present_ids)

        # solvePnP donne : p_cam = R @ p_base + t
        # Donc T_cam_from_base[4×4] transforme un point base → cam
        R, _ = cv2.Rodrigues(rvec)
        T = np.eye(4, dtype=np.float64)
        T[:3, :3] = R
        T[:3,  3] = tvec.flatten()
        self._T_cam_to_base = T   # T : p_cam = T @ [p_base; 1]

        # Contrôle de cohérence : hauteur caméra estimée depuis la pose PnP.
        # camera_pos_in_base = -R^T * t
        cam_pos_base = -R.T @ tvec.flatten()
        cam_h_est = float(cam_pos_base[2])
        if abs(cam_h_est - self._camera_height_m) > 0.12:
            self.get_logger().warning(
                "Hauteur caméra estimée incohérente: "
                f"{cam_h_est:.3f} m (attendu {self._camera_height_m:.3f} m)"
            )
        self._active_ws_ids = used_ids
        return True

    # ──────────────────────────────────────────────────────────────────────────

    def _try_publish_object(self, corners, ids_flat: list, stamp) -> None:
        """Détecte le marqueur objet et publie sa pose dans le repère base."""
        marker_id = None
        if self._obj_id in ids_flat:
            marker_id = self._obj_id
        elif self._auto_obj_center and self._active_ws_ids:
            id_to_index = {int(mid): i for i, mid in enumerate(ids_flat)}
            ws_centers = []
            for mid in self._active_ws_ids:
                if mid in id_to_index:
                    ws_centers.append(corners[id_to_index[mid]][0].mean(axis=0))
            if ws_centers:
                ws_center = np.mean(np.array(ws_centers), axis=0)
                best = None
                for mid in ids_flat:
                    mid = int(mid)
                    if mid in self._active_ws_ids:
                        continue
                    if mid not in id_to_index:
                        continue
                    c = corners[id_to_index[mid]][0].mean(axis=0)
                    dist = float(np.linalg.norm(c - ws_center))
                    if best is None or dist < best[0]:
                        best = (dist, mid)
                if best is not None:
                    marker_id = best[1]

        if marker_id is None:
            return

        idx = ids_flat.index(marker_id)
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
