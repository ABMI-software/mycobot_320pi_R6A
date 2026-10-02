#!/usr/bin/env python3
"""§3.3 — Calibration Hand-Eye (T_base_camera) — méthode Tsai-Lenz (AX=XB).

Configuration du banc : caméra FIXE + marqueur ArUco solidaire de l'effecteur
(eye-to-hand). On déplace le bras en ≥ N poses (avec rotations), et pour chaque
pose on enregistre :
   • T_base_ee   = cinématique directe depuis /joint_states (FK URDF)
   • T_cam_marker = pose du marqueur estimée par la caméra (solvePnP)
On résout X = T_base_camera via cv2.calibrateHandEye (TSAI + ANDREFF).

⚠ IMPORTANT — montage du marqueur :
   Le marqueur de calibration (ID 20, 30 mm par défaut) DOIT être fixé
   rigidement sur la bride / l'effecteur du robot (il se déplace AVEC le bras).
   C'est la condition d'observabilité du eye-to-hand. Si la caméra était montée
   sur le bras, utiliser mode:=eye_in_hand.

Capture interactive (taper la touche puis Entrée dans ce terminal) :
   c → capturer la pose courante      u → annuler la dernière capture
   l → lister le nombre de captures   s → résoudre + sauvegarder
   a → balayage AUTO (robot piloté)   x → arrêter le balayage
   q → quitter sans sauvegarder

Balayage automatique ('a')
--------------------------
Le robot est piloté par commande (publie send_angles sur /to_robot). Amène
d'abord le bras à une pose où le marqueur ID 20 est VISIBLE, puis tape 'a' :
le nœud lit la pose courante (/joint_states), génère des variations du poignet
(j4,j5,j6) autour d'elle, les envoie une par une, attend la stabilisation, et
capture automatiquement quand le marqueur est vu. Les poses où il n'est pas
visible sont ignorées. Termine avec 's'.

Usage
-----
  ros2 run mycobot_gateway calibrate_hand_eye --ros-args \\
      -p marker_id:=20 -p marker_size:=0.03 -p mode:=eye_to_hand
"""

from __future__ import annotations

import itertools
import json
import os
import random
import select
import sys
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image, JointState
from std_msgs.msg import String

_DREAM_DIR_ALT = "/home/genji/ros_jazzy/src/mycobot_R6A/training/dream"
_DREAM_DIR = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "..", "..", "..", "..", "training", "dream",
))
for _p in [_DREAM_DIR, _DREAM_DIR_ALT]:
    if os.path.isdir(_p) and _p not in sys.path:
        sys.path.insert(0, _p)

from mycobot_fk import forward_kinematics  # type: ignore  # noqa: E402

JOINT_NAMES = [
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
]

# Limites articulaires conservatrices (degrés) — MyCobot 320 Pi.
_JOINT_LIMITS_DEG = [
    (-168.0, 168.0),
    (-135.0, 135.0),
    (-150.0, 150.0),
    (-145.0, 145.0),
    (-165.0, 165.0),
    (-180.0, 180.0),
]

_HANDEYE_METHODS = {
    "TSAI": cv2.CALIB_HAND_EYE_TSAI,
    "ANDREFF": cv2.CALIB_HAND_EYE_ANDREFF,
    "PARK": cv2.CALIB_HAND_EYE_PARK,
    "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
    "DANIILIDIS": cv2.CALIB_HAND_EYE_DANIILIDIS,
}


def _resolve_default(rel: str) -> str:
    here = Path(__file__).resolve()
    for p in [here, *here.parents]:
        cand = p / "training" / "calibration" / rel
        if cand.exists():
            return str(cand)
    return f"/home/genji/ros_jazzy/src/mycobot_R6A/training/calibration/{rel}"


def _aruco_dict(name: str) -> int:
    return getattr(cv2.aruco, name if hasattr(cv2.aruco, name) else "DICT_4X4_1000")


def _marker_corners_3d(half: float) -> np.ndarray:
    return np.array([
        [-half,  half, 0.0],
        [ half,  half, 0.0],
        [ half, -half, 0.0],
        [-half, -half, 0.0],
    ], dtype=np.float64)


def _invert(T: np.ndarray) -> np.ndarray:
    R = T[:3, :3]
    t = T[:3, 3]
    Ti = np.eye(4)
    Ti[:3, :3] = R.T
    Ti[:3, 3] = -R.T @ t
    return Ti


class CalibrateHandEyeNode(Node):
    def __init__(self) -> None:
        super().__init__("calibrate_hand_eye")

        self.declare_parameter("calib_file",   _resolve_default("cam_0.npz"))
        self.declare_parameter("output",       _resolve_default("hand_eye_calibration.yaml"))
        self.declare_parameter("camera_topic", "/camera/image_raw")
        self.declare_parameter("marker_id",    20)
        self.declare_parameter("marker_size",  0.03)
        self.declare_parameter("aruco_dict_name", "DICT_4X4_1000")
        self.declare_parameter("aruco_detect_scale", 1.6)
        self.declare_parameter("mode",         "eye_to_hand")  # ou eye_in_hand
        self.declare_parameter("min_samples",  20)
        # Balayage automatique de poses (robot piloté par commande)
        self.declare_parameter("auto_speed",     30)
        self.declare_parameter("auto_settle_s",  3.0)
        self.declare_parameter("auto_max_poses", 30)

        calib_file   = str(self.get_parameter("calib_file").value)
        self._out    = str(self.get_parameter("output").value)
        cam_topic    = str(self.get_parameter("camera_topic").value)
        self._mid    = int(self.get_parameter("marker_id").value)
        self._half   = float(self.get_parameter("marker_size").value) / 2.0
        self._scale  = float(self.get_parameter("aruco_detect_scale").value)
        self._mode   = str(self.get_parameter("mode").value)
        self._min_n  = int(self.get_parameter("min_samples").value)
        self._auto_speed  = int(self.get_parameter("auto_speed").value)
        self._auto_settle = float(self.get_parameter("auto_settle_s").value)
        self._auto_max    = int(self.get_parameter("auto_max_poses").value)

        data = np.load(calib_file)
        self._K = data["mtx"].astype(np.float64)
        self._D = data["dist"].astype(np.float64)

        d = cv2.aruco.getPredefinedDictionary(
            _aruco_dict(str(self.get_parameter("aruco_dict_name").value)))
        self._use_new = hasattr(cv2.aruco, "ArucoDetector")
        params = (cv2.aruco.DetectorParameters() if self._use_new
                  else cv2.aruco.DetectorParameters_create())
        params.adaptiveThreshWinSizeMin = 3
        params.adaptiveThreshWinSizeMax = 61
        params.adaptiveThreshWinSizeStep = 4
        params.minMarkerPerimeterRate = 0.01
        params.maxMarkerPerimeterRate = 4.0
        if hasattr(cv2.aruco, "CORNER_REFINE_SUBPIX"):
            params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self._dict = d
        self._params = params

        self._bridge = CvBridge()
        self._T_base_ee: np.ndarray | None = None      # FK courante
        self._T_cam_marker: np.ndarray | None = None    # pose marqueur courante
        self._marker_seen = False

        # échantillons collectés
        self._samples: list[tuple[np.ndarray, np.ndarray]] = []  # (T_base_ee, T_cam_marker)

        # état du balayage automatique
        self._cur_deg: list[float] | None = None   # angles courants (degrés)
        self._sweep: dict | None = None            # état machine du balayage
        self._prev_seen: bool = False              # pour log de visibilité

        self._to_robot_pub = self.create_publisher(String, "/to_robot", 10)
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self.create_subscription(Image, cam_topic, self._image_cb, 5)
        self.create_timer(0.1, self._poll_stdin)
        self.create_timer(0.2, self._sweep_step)
        self.create_timer(0.5, self._visibility_heartbeat)

        self.get_logger().info(
            f"Hand-Eye prêt — mode={self._mode}  marqueur ID={self._mid} "
            f"({self._half*2*1000:.0f} mm)  min_samples={self._min_n}")
        self._print_menu()

    # ── callbacks ───────────────────────────────────────────────────────────

    def _js_cb(self, msg: JointState) -> None:
        name_to_pos = dict(zip(msg.name, msg.position))
        try:
            angles = np.array([name_to_pos[j] for j in JOINT_NAMES], dtype=np.float64)
        except KeyError:
            return
        _, transforms = forward_kinematics(angles)
        self._T_base_ee = transforms[-1]
        self._cur_deg = np.degrees(angles).tolist()

    def _detect(self, gray: np.ndarray):
        scaled, inv = gray, 1.0
        if self._scale > 1.01:
            scaled = cv2.resize(gray, None, fx=self._scale, fy=self._scale,
                                interpolation=cv2.INTER_LINEAR)
            inv = 1.0 / self._scale
        if self._use_new:
            det = cv2.aruco.ArucoDetector(self._dict, self._params)
            corners, ids, _ = det.detectMarkers(scaled)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                scaled, self._dict, parameters=self._params)
        if inv != 1.0 and corners:
            corners = [(c * inv).astype(np.float32) for c in corners]
        return corners, ids

    def _image_cb(self, msg: Image) -> None:
        frame = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids = self._detect(gray)
        self._marker_seen = False
        if ids is None:
            self._T_cam_marker = None
            return
        ids_flat = [int(x) for x in ids.flatten().tolist()]
        if self._mid not in ids_flat:
            self._T_cam_marker = None
            return
        idx = ids_flat.index(self._mid)
        img = corners[idx][0].astype(np.float64)
        ok, rvec, tvec = cv2.solvePnP(
            _marker_corners_3d(self._half), img, self._K, self._D,
            flags=cv2.SOLVEPNP_IPPE_SQUARE)
        if not ok:
            self._T_cam_marker = None
            return
        R, _ = cv2.Rodrigues(rvec)
        T = np.eye(4)
        T[:3, :3] = R
        T[:3, 3] = tvec.flatten()
        self._T_cam_marker = T
        self._marker_seen = True

    # ── interface clavier ─────────────────────────────────────────────────────

    def _visibility_heartbeat(self) -> None:
        """Log sur transition de visibilité du marqueur (hors balayage)."""
        if self._sweep is not None:
            return
        if self._marker_seen and not self._prev_seen:
            t = self._T_cam_marker[:3, 3] if self._T_cam_marker is not None else (0, 0, 0)
            self.get_logger().info(
                f"👁  Marqueur ID {self._mid} VISIBLE "
                f"[{t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}] m — prêt pour 'a' ou 'c'.")
        elif not self._marker_seen and self._prev_seen:
            self.get_logger().info(f"   Marqueur ID {self._mid} hors champ.")
        self._prev_seen = self._marker_seen

    def _print_menu(self) -> None:
        print("\n[c]apturer  [u]ndo  [l]ister  [a]uto-balayage  [x]stop-balayage  "
              "[s]olve+save  [q]uit", flush=True)

    def _poll_stdin(self) -> None:
        r, _, _ = select.select([sys.stdin], [], [], 0.0)
        if not r:
            return
        line = sys.stdin.readline().strip().lower()
        if not line:
            return
        cmd = line[0]
        if cmd == "c":
            self._capture()
        elif cmd == "u":
            self._undo()
        elif cmd == "l":
            self._list()
        elif cmd == "a":
            self._start_sweep()
        elif cmd == "x":
            self._stop_sweep()
        elif cmd == "s":
            self._solve_and_save()
        elif cmd == "q":
            self.get_logger().info("Abandon sans sauvegarde.")
            rclpy.shutdown()
        else:
            self._print_menu()

    def _capture(self) -> None:
        if self._T_base_ee is None:
            self.get_logger().warning("Pas de /joint_states encore reçu. Capture ignorée.")
            return
        if self._T_cam_marker is None or not self._marker_seen:
            self.get_logger().warning(
                f"Marqueur ID {self._mid} non visible. Repositionne et réessaie.")
            return
        self._samples.append((self._T_base_ee.copy(), self._T_cam_marker.copy()))
        t = self._T_cam_marker[:3, 3]
        self.get_logger().info(
            f"Capture #{len(self._samples)} OK — marqueur@cam "
            f"[{t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}] m")
        self._print_menu()

    def _undo(self) -> None:
        if self._samples:
            self._samples.pop()
            self.get_logger().info(f"Dernière capture annulée. Reste {len(self._samples)}.")
        else:
            self.get_logger().info("Aucune capture à annuler.")

    def _list(self) -> None:
        self.get_logger().info(
            f"{len(self._samples)} captures (min requis : {self._min_n}).")

    # ── balayage automatique de poses ─────────────────────────────────────────

    @staticmethod
    def _clamp(idx: int, val: float) -> float:
        lo, hi = _JOINT_LIMITS_DEG[idx]
        return max(lo, min(hi, val))

    def _build_sweep_poses(self, base: list[float]) -> list[list[float]]:
        """Génère des poses en perturbant le poignet (j4,j5,j6) autour de `base`.

        Garde la position du marqueur à peu près face caméra tout en variant son
        orientation (condition d'observabilité du hand-eye). Les poses où le
        marqueur n'est finalement pas visible seront simplement ignorées.
        """
        d4s = [0.0, -25.0, 25.0, -45.0, 45.0]
        d5s = [0.0, -20.0, 20.0]
        d6s = [0.0, -40.0, 40.0, -80.0, 80.0]
        combos = list(itertools.product(d4s, d5s, d6s))
        base_combo = (0.0, 0.0, 0.0)
        combos.remove(base_combo)
        random.Random(42).shuffle(combos)
        combos = [base_combo] + combos  # la pose de base (déjà visible) en premier

        poses: list[list[float]] = []
        seen: set = set()
        for d4, d5, d6 in combos:
            p = list(base)
            p[3] = self._clamp(3, base[3] + d4)
            p[4] = self._clamp(4, base[4] + d5)
            p[5] = self._clamp(5, base[5] + d6)
            key = tuple(round(v, 1) for v in p)
            if key in seen:
                continue
            seen.add(key)
            poses.append(p)
            if len(poses) >= self._auto_max:
                break
        return poses

    def _start_sweep(self) -> None:
        if self._sweep is not None:
            self.get_logger().warning("Balayage déjà en cours. 'x' pour l'arrêter.")
            return
        if self._cur_deg is None:
            self.get_logger().warning(
                "Pas d'angles courants (/joint_states). Impossible de démarrer le balayage.")
            return
        if not self._marker_seen:
            self.get_logger().warning(
                f"Marqueur ID {self._mid} non visible à la pose courante. "
                "Amène d'abord le marqueur face caméra, puis 'a'.")
            return
        base = list(self._cur_deg)
        poses = self._build_sweep_poses(base)
        self._sweep = {"poses": poses, "idx": 0, "phase": "move", "t0": 0.0}
        self.get_logger().info(
            f"▶ Balayage auto démarré : {len(poses)} poses "
            f"(vitesse {self._auto_speed}, stabilisation {self._auto_settle:.1f}s). "
            "'x' pour arrêter.")

    def _stop_sweep(self) -> None:
        if self._sweep is None:
            self.get_logger().info("Aucun balayage en cours.")
            return
        self._sweep = None
        self.get_logger().info(
            f"⏹ Balayage arrêté. {len(self._samples)} captures au total.")
        self._print_menu()

    def _send_angles(self, pose_deg: list[float]) -> None:
        cmd = {
            "action": "send_angles",
            "angles": [round(float(a), 2) for a in pose_deg],
            "speed": self._auto_speed,
        }
        msg = String()
        msg.data = json.dumps(cmd)
        self._to_robot_pub.publish(msg)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _sweep_step(self) -> None:
        sw = self._sweep
        if sw is None:
            return
        poses = sw["poses"]
        idx = sw["idx"]
        phase = sw["phase"]

        if phase == "move":
            self._send_angles(poses[idx])
            sw["t0"] = self._now_s()
            sw["phase"] = "settle"
            self.get_logger().info(f"→ Pose {idx + 1}/{len(poses)} envoyée.")
        elif phase == "settle":
            if self._now_s() - sw["t0"] >= self._auto_settle:
                sw["phase"] = "capture"
        elif phase == "capture":
            if self._marker_seen and self._T_cam_marker is not None and self._T_base_ee is not None:
                self._samples.append((self._T_base_ee.copy(), self._T_cam_marker.copy()))
                t = self._T_cam_marker[:3, 3]
                self.get_logger().info(
                    f"   ✓ Capture #{len(self._samples)} (pose {idx + 1}) — "
                    f"marqueur@cam [{t[0]:.3f}, {t[1]:.3f}, {t[2]:.3f}] m")
            else:
                self.get_logger().warning(
                    f"   ✗ Pose {idx + 1} : marqueur non visible, ignorée.")
            sw["idx"] += 1
            if sw["idx"] >= len(poses):
                self._sweep = None
                self.get_logger().info(
                    f"✅ Balayage terminé : {len(self._samples)} captures sur "
                    f"{len(poses)} poses (min requis {self._min_n}). "
                    "Appuie 's' pour résoudre + sauvegarder.")
                self._print_menu()
            else:
                sw["phase"] = "move"

    # ── résolution ────────────────────────────────────────────────────────────

    def _solve_and_save(self) -> None:
        n = len(self._samples)
        if n < max(3, self._min_n):
            self.get_logger().warning(
                f"Seulement {n} captures (min {self._min_n}). Continue à bouger le bras.")
            return

        # Eye-to-hand : on alimente calibrateHandEye avec base→gripper (inverse FK)
        # → le résultat est camera→base = T_base_camera.
        # Eye-in-hand : on alimente gripper→base directement → résultat camera→gripper.
        R_g2b, t_g2b, R_t2c, t_t2c = [], [], [], []
        for T_base_ee, T_cam_marker in self._samples:
            if self._mode == "eye_to_hand":
                T_robot = _invert(T_base_ee)   # base→ee
            else:
                T_robot = T_base_ee            # ee→base (eye_in_hand)
            R_g2b.append(T_robot[:3, :3])
            t_g2b.append(T_robot[:3, 3].reshape(3, 1))
            R_t2c.append(T_cam_marker[:3, :3])
            t_t2c.append(T_cam_marker[:3, 3].reshape(3, 1))

        results = {}
        for name in ("TSAI", "ANDREFF", "PARK"):
            try:
                R_x, t_x = cv2.calibrateHandEye(
                    R_g2b, t_g2b, R_t2c, t_t2c, method=_HANDEYE_METHODS[name])
            except cv2.error as exc:
                self.get_logger().warning(f"{name} a échoué : {exc}")
                continue
            X = np.eye(4)
            X[:3, :3] = R_x
            X[:3, 3] = t_x.flatten()
            resid = self._residual_mm(X)
            results[name] = (X, resid)
            self.get_logger().info(
                f"{name:8s} → résidu invariant {resid:.2f} mm  "
                f"t=[{X[0,3]:.3f}, {X[1,3]:.3f}, {X[2,3]:.3f}] m")

        if not results:
            self.get_logger().error("Toutes les méthodes ont échoué.")
            return

        best_name = min(results, key=lambda k: results[k][1])
        X_best, resid_best = results[best_name]
        ok = resid_best <= 2.0
        verdict = "OK (≤ 2 mm)" if ok else "HORS TOLÉRANCE (> 2 mm)"
        (self.get_logger().info if ok else self.get_logger().warning)(
            f"VERDICT hand-eye : {verdict}  | meilleure méthode : {best_name}")

        label = "T_base_camera" if self._mode == "eye_to_hand" else "T_ee_camera"
        out = {
            "mode": self._mode,
            "marker_id": self._mid,
            "marker_size_m": round(self._half * 2, 4),
            "num_samples": n,
            "best_method": best_name,
            "residual_mm": round(float(resid_best), 3),
            "methods_residual_mm": {k: round(float(v[1]), 3) for k, v in results.items()},
            label: [[round(float(v), 6) for v in row] for row in X_best],
            "translation_m": [round(float(v), 5) for v in X_best[:3, 3]],
            "rotation_matrix": [[round(float(v), 6) for v in row] for row in X_best[:3, :3]],
        }
        with open(self._out, "w") as fh:
            yaml.safe_dump(out, fh, sort_keys=False)
        self.get_logger().info(f"{label} sauvegardé → {self._out}")
        rclpy.shutdown()

    def _residual_mm(self, X: np.ndarray) -> float:
        """Résidu d'invariance AX=XB.

        eye_to_hand : X = T_base_cam. La pose du marqueur dans le repère EE doit
        être constante : T_ee_marker = inv(T_base_ee) @ X @ T_cam_marker.
        eye_in_hand : X = T_ee_cam. La pose du marqueur dans le repère base doit
        être constante : T_base_marker = T_base_ee @ X @ T_cam_marker.
        On mesure l'écart-type des translations sur tous les échantillons.
        """
        pts = []
        for T_base_ee, T_cam_marker in self._samples:
            if self._mode == "eye_to_hand":
                T_inv = _invert(T_base_ee) @ X @ T_cam_marker
            else:
                T_inv = T_base_ee @ X @ T_cam_marker
            pts.append(T_inv[:3, 3])
        pts = np.array(pts)
        # dispersion 3D (norme de l'écart-type par axe) en mm
        return float(np.linalg.norm(pts.std(axis=0)) * 1000.0)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CalibrateHandEyeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
