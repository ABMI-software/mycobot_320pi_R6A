#!/usr/bin/env python3
"""§3.2 — Calibration extrinsèque : repère monde via marqueurs sol.

Estime la transformation T_cam_world (caméra ↔ repère monde fixe défini par les
marqueurs sol) par solvePnP multi-points, valide la reprojection et la redondance,
sauvegarde le résultat en YAML et publie (optionnellement) world → camera sur /tf.

Méthode (conforme au protocole) :
  1. Charge les positions mesurées des marqueurs sol depuis workspace_markers.yaml
     (centres 3D dans le repère monde = repère base, ±1 mm).
  2. Accumule N frames, moyenne les coins 2D détectés des marqueurs connus.
  3. solvePnP (centres connus ↔ détections ArUco) → T_cam_world.
  4. Vérifie erreur de reprojection < reproj_max_px (défaut 1.5 px).
  5. Test de redondance : masque chaque marqueur, recalcule, vérifie écart de
     position caméra < redundancy_max_mm (défaut 1 mm).
  6. Sauvegarde T_cam_world (4×4), R, t et l'inverse dans camera_extrinsic.yaml.
  7. Publie en continu world → camera (static TF) si publish_tf=True.

Usage
-----
  ros2 run mycobot_gateway calibrate_extrinsic --ros-args \\
      -p markers_yaml:=/path/workspace_markers.yaml \\
      -p camera_topic:=/camera/image_raw \\
      -p num_frames:=30 \\
      -p publish_tf:=true
"""

from __future__ import annotations

from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from sensor_msgs.msg import Image
from tf2_ros import StaticTransformBroadcaster


def _resolve_default(rel: str) -> str:
    here = Path(__file__).resolve()
    for p in [here, *here.parents]:
        cand = p / "training" / "calibration" / rel
        if cand.exists():
            return str(cand)
    return f"/home/genji/ros_jazzy/src/mycobot_R6A/training/calibration/{rel}"


def _aruco_dict(name: str) -> int:
    return getattr(cv2.aruco, name if hasattr(cv2.aruco, name) else "DICT_4X4_1000")


def _rot_to_quat(R: np.ndarray) -> tuple[float, float, float, float]:
    """Matrice 3×3 → quaternion (x, y, z, w)."""
    t = np.trace(R)
    if t > 0:
        s = 0.5 / np.sqrt(t + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return float(x), float(y), float(z), float(w)


class CalibrateExtrinsicNode(Node):
    def __init__(self) -> None:
        super().__init__("calibrate_extrinsic")

        self.declare_parameter("calib_file",       _resolve_default("cam_0.npz"))
        self.declare_parameter("markers_yaml",     _resolve_default("workspace_markers.yaml"))
        self.declare_parameter("output",           _resolve_default("camera_extrinsic.yaml"))
        self.declare_parameter("camera_topic",     "/camera/image_raw")
        self.declare_parameter("aruco_dict_name",  "DICT_4X4_1000")
        self.declare_parameter("aruco_detect_scale", 1.6)
        self.declare_parameter("num_frames",       30)
        self.declare_parameter("reproj_max_px",    1.5)
        self.declare_parameter("redundancy_max_mm", 1.0)
        self.declare_parameter("publish_tf",       True)

        calib_file   = str(self.get_parameter("calib_file").value)
        markers_yaml = str(self.get_parameter("markers_yaml").value)
        self._out    = str(self.get_parameter("output").value)
        cam_topic    = str(self.get_parameter("camera_topic").value)
        self._scale  = float(self.get_parameter("aruco_detect_scale").value)
        self._n_want = int(self.get_parameter("num_frames").value)
        self._reproj_max = float(self.get_parameter("reproj_max_px").value)
        self._redund_max = float(self.get_parameter("redundancy_max_mm").value)
        self._publish_tf = bool(self.get_parameter("publish_tf").value)

        # ── intrinsèques ──
        data = np.load(calib_file)
        self._K = data["mtx"].astype(np.float64)
        self._D = data["dist"].astype(np.float64)

        # ── positions mesurées des marqueurs sol ──
        with open(markers_yaml, "r") as fh:
            spec = yaml.safe_load(fh)
        self._frame_id = str(spec.get("frame_id", "base_link"))
        self._markers: dict[int, np.ndarray] = {
            int(mid): np.asarray(xyz, dtype=np.float64)
            for mid, xyz in spec["markers"].items()
        }
        self.get_logger().info(
            f"{len(self._markers)} marqueurs sol chargés : "
            f"{sorted(self._markers)} (repère '{self._frame_id}')"
        )

        # ── détecteur ArUco ──
        d = cv2.aruco.getPredefinedDictionary(
            _aruco_dict(str(self.get_parameter("aruco_dict_name").value))
        )
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

        # ── accumulation des centres 2D par ID ──
        self._acc: dict[int, list[np.ndarray]] = {mid: [] for mid in self._markers}
        self._frames = 0
        self._done = False

        self._bridge = CvBridge()
        self._static_tf = StaticTransformBroadcaster(self) if self._publish_tf else None
        self.create_subscription(Image, cam_topic, self._image_cb, 5)
        self.get_logger().info(
            f"Acquisition de {self._n_want} frames sur {cam_topic} …"
        )

    # ──────────────────────────────────────────────────────────────────────────

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
        if self._done:
            return
        frame = self._bridge.imgmsg_to_cv2(msg, "bgr8")
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids = self._detect(gray)
        if ids is not None:
            ids_flat = [int(x) for x in ids.flatten().tolist()]
            for mid in self._markers:
                if mid in ids_flat:
                    idx = ids_flat.index(mid)
                    self._acc[mid].append(corners[idx][0].mean(axis=0))
        self._frames += 1
        if self._frames >= self._n_want:
            self._done = True
            self._solve()

    # ──────────────────────────────────────────────────────────────────────────

    def _solve_pnp(self, ids_used: list[int], verbose: bool = False):
        """solvePnP sur les centres des marqueurs `ids_used`. → (R, t, reproj).

        Les marqueurs sol sont coplanaires (z=0) → ambiguïté miroir : deux poses
        proper-rotation reprojettent de façon quasi identique, l'une avec la
        caméra au-dessus du plan (physique), l'autre en-dessous (miroir). On
        énumère toutes les solutions candidates (IPPE + ITERATIVE) et on retient
        impérativement une caméra au-dessus du plan (cam_z > 0).
        """
        obj = np.array([self._markers[m] for m in ids_used], dtype=np.float64)
        img = np.array([np.mean(self._acc[m], axis=0) for m in ids_used],
                       dtype=np.float64)

        def reproj(rv, tv):
            proj, _ = cv2.projectPoints(obj, rv, tv, self._K, self._D)
            return float(np.mean(np.linalg.norm(
                proj.reshape(-1, 2) - img, axis=1)))

        def cam_z(rv, tv):
            R, _ = cv2.Rodrigues(rv)
            return float((-R.T @ tv.flatten())[2])

        candidates = []  # (rv, tv, reproj, cam_z, source)
        # 4 points coplanaires → IPPE renvoie explicitement les 2 solutions.
        if len(ids_used) >= 4:
            try:
                n, rvs, tvs, _ = cv2.solvePnPGeneric(
                    obj, img, self._K, self._D, flags=cv2.SOLVEPNP_IPPE)
                for i in range(n):
                    rv, tv = rvs[i], tvs[i]
                    candidates.append(
                        (rv, tv, reproj(rv, tv), cam_z(rv, tv), f"IPPE{i}"))
            except cv2.error as exc:
                if verbose:
                    self.get_logger().warning(f"IPPE a échoué : {exc}")
        # Candidat ITERATIVE supplémentaire (utile si < 4 points ou IPPE muet).
        try:
            ok, rv, tv = cv2.solvePnP(obj, img, self._K, self._D,
                                      flags=cv2.SOLVEPNP_ITERATIVE)
            if ok:
                candidates.append(
                    (rv, tv, reproj(rv, tv), cam_z(rv, tv), "ITER"))
        except cv2.error:
            pass

        if not candidates:
            return None
        if verbose:
            for rv, tv, e, cz, src in candidates:
                self.get_logger().info(
                    f"  candidat {src}: reproj={e:.3f} px  cam_z={cz:+.3f} m")

        # Préférer une caméra AU-DESSUS du plan ; départage par reprojection.
        above = [c for c in candidates if c[3] > 0.0]
        pool = above if above else candidates
        best = min(pool, key=lambda c: c[2])
        return best[0], best[1], best[2]

    def _cam_pos_world(self, rv, tv) -> np.ndarray:
        R, _ = cv2.Rodrigues(rv)
        return -R.T @ tv.flatten()

    def _solve(self) -> None:
        seen = [m for m in self._markers if len(self._acc[m]) > 0]
        self.get_logger().info(
            f"Frames acquises : {self._frames}  |  marqueurs vus : {sorted(seen)} "
            f"(détections par ID : { {m: len(self._acc[m]) for m in seen} })"
        )
        if len(seen) < 4:
            self.get_logger().error(
                f"Seulement {len(seen)} marqueurs sol visibles (4 requis). "
                "Vérifie le champ de vision et l'éclairage. Abandon."
            )
            rclpy.shutdown()
            return

        sol = self._solve_pnp(seen, verbose=True)
        if sol is None:
            self.get_logger().error("solvePnP a échoué. Abandon.")
            rclpy.shutdown()
            return
        rv, tv, reproj = sol
        R, _ = cv2.Rodrigues(rv)
        cam_pos = self._cam_pos_world(rv, tv)

        # T_cam_world : p_cam = T @ [p_world; 1]   (world→camera)
        T_cam_world = np.eye(4)
        T_cam_world[:3, :3] = R
        T_cam_world[:3, 3] = tv.flatten()
        T_world_cam = np.linalg.inv(T_cam_world)  # camera→world (pose caméra)

        # ── stabilité ──
        # Test de redondance (masquer un marqueur) : nécessite ≥ 5 marqueurs car
        # le PnP planaire a besoin de 4 points. Avec 4 marqueurs (cas du banc),
        # on rapporte à la place le résidu de reprojection PAR marqueur, qui
        # mesure la cohérence géométrique de la pose retenue.
        drifts_mm = []
        if len(seen) >= 5:
            for drop in seen:
                subset = [m for m in seen if m != drop]
                s2 = self._solve_pnp(subset)
                if s2 is None:
                    continue
                pos2 = self._cam_pos_world(s2[0], s2[1])
                drifts_mm.append(
                    (drop, float(np.linalg.norm(pos2 - cam_pos) * 1000.0)))

        # Résidu de reprojection par marqueur (toujours calculable).
        per_marker_px = []
        for m in seen:
            proj, _ = cv2.projectPoints(
                self._markers[m].reshape(1, 3), rv, tv, self._K, self._D)
            meas = np.mean(self._acc[m], axis=0)
            per_marker_px.append(
                (m, float(np.linalg.norm(proj.reshape(2) - meas))))

        # ── verdict ──
        self.get_logger().info(
            f"Reprojection : {reproj:.3f} px (cible < {self._reproj_max} px)")
        self.get_logger().info(
            f"Position caméra (repère {self._frame_id}) : "
            f"[{cam_pos[0]:.4f}, {cam_pos[1]:.4f}, {cam_pos[2]:.4f}] m")
        for mid, e in per_marker_px:
            self.get_logger().info(f"  résidu reproj ID {mid} : {e:.3f} px")
        if drifts_mm:
            worst = max(d for _, d in drifts_mm)
            for mid, d in drifts_mm:
                self.get_logger().info(
                    f"  redondance sans ID {mid} : écart {d:.2f} mm")
            self.get_logger().info(
                f"Écart de redondance max : {worst:.2f} mm "
                f"(cible < {self._redund_max} mm)")
        else:
            worst = float("nan")
            self.get_logger().info(
                "Test de redondance ignoré (< 5 marqueurs) — "
                "voir résidus de reprojection par marqueur ci-dessus.")

        # La caméra DOIT être au-dessus du plan des marqueurs (z > 0).
        above_plane = cam_pos[2] > 0.0
        if not above_plane:
            self.get_logger().error(
                f"Caméra estimée SOUS le plan des marqueurs (z={cam_pos[2]:.3f} m) "
                "— solution miroir non physique. Vérifie les positions mesurées "
                "dans workspace_markers.yaml et l'ordre des axes du repère.")
        reproj_ok = reproj < self._reproj_max
        redund_ok = (not drifts_mm) or worst < self._redund_max
        all_ok = reproj_ok and redund_ok and above_plane
        verdict = "OK" if all_ok else "HORS TOLÉRANCE"
        (self.get_logger().info if all_ok else self.get_logger().warning)(
            f"VERDICT extrinsèque : {verdict}")

        # ── sauvegarde ──
        out = {
            "frame_id": self._frame_id,
            "camera_frame": "camera",
            "markers_used": sorted(seen),
            "physically_valid": bool(above_plane),
            "reproj_error_px": round(reproj, 4),
            "per_marker_reproj_px": {int(m): round(e, 4) for m, e in per_marker_px},
            "redundancy_max_mm": (None if not drifts_mm else round(worst, 3)),
            "camera_position_world_m": [round(float(v), 5) for v in cam_pos],
            "T_cam_world": [[round(float(v), 6) for v in row] for row in T_cam_world],
            "T_world_cam": [[round(float(v), 6) for v in row] for row in T_world_cam],
        }
        with open(self._out, "w") as fh:
            yaml.safe_dump(out, fh, sort_keys=False)
        self.get_logger().info(f"T_cam_world sauvegardé → {self._out}")

        # ── TF statique world → camera (uniquement si pose physique) ──
        if self._static_tf is not None and above_plane:
            x, y, z = T_world_cam[:3, 3]
            qx, qy, qz, qw = _rot_to_quat(T_world_cam[:3, :3])
            tf = TransformStamped()
            tf.header.stamp = self.get_clock().now().to_msg()
            tf.header.frame_id = self._frame_id
            tf.child_frame_id = "camera"
            tf.transform.translation.x = float(x)
            tf.transform.translation.y = float(y)
            tf.transform.translation.z = float(z)
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self._static_tf.sendTransform(tf)
            self.get_logger().info(
                f"TF statique {self._frame_id} → camera publiée. Ctrl-C pour quitter.")
        else:
            rclpy.shutdown()


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CalibrateExtrinsicNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
