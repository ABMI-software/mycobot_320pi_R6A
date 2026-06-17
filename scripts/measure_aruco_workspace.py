#!/usr/bin/env python3
"""Mesure les positions 3D des marqueurs ArUco dans le repère caméra.

Lance en standalone (pas besoin de ROS) : estime la pose de chaque marqueur
détecté via solvePnP individuel, calcule les distances réelles entre marqueurs,
et imprime le layout pour comparaison avec les positions hardcodées.

Usage
-----
  # Depuis une image ROS (nécessite ROS2 sourcé)
  python3 scripts/measure_aruco_workspace.py --ros-topic /camera/image_raw \\
      --calib training/calibration/cam_0.npz --marker-size 0.025

  # Depuis la caméra directement (OpenCV VideoCapture)
  python3 scripts/measure_aruco_workspace.py --device 0 \\
      --calib training/calibration/cam_0.npz --marker-size 0.025

Sortie : positions XYZ de chaque marqueur en repère caméra + distances pairwise.
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np


def _load_calib(path: str):
    data = np.load(path)
    K = data["mtx"].astype(np.float64)
    D = data["dist"].astype(np.float64)
    print(f"Calibration : fx={K[0,0]:.1f}  fy={K[1,1]:.1f}  cx={K[0,2]:.1f}  cy={K[1,2]:.1f}")
    return K, D


def _marker_corners_3d(half: float) -> np.ndarray:
    return np.array([
        [-half,  half, 0.0],
        [ half,  half, 0.0],
        [ half, -half, 0.0],
        [-half, -half, 0.0],
    ], dtype=np.float64)


def _estimate_positions(frame, K, D, marker_size: float, aruco_dict_id: int):
    """Retourne dict[marker_id] -> xyz_in_camera_frame (centre du marqueur)."""
    dictionary = cv2.aruco.getPredefinedDictionary(aruco_dict_id)
    use_new_api = hasattr(cv2.aruco, "ArucoDetector")

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    if use_new_api:
        params = cv2.aruco.DetectorParameters()
        params.adaptiveThreshWinSizeMin = 3
        params.adaptiveThreshWinSizeMax = 61
        params.minMarkerPerimeterRate = 0.01
        detector = cv2.aruco.ArucoDetector(dictionary, params)
        corners, ids, _ = detector.detectMarkers(gray)
    else:
        params = cv2.aruco.DetectorParameters_create()
        params.adaptiveThreshWinSizeMin = 3
        params.adaptiveThreshWinSizeMax = 61
        params.minMarkerPerimeterRate = 0.01
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)

    if ids is None:
        return {}, frame

    half = marker_size / 2.0
    obj_pts = _marker_corners_3d(half)
    positions = {}

    debug = frame.copy()
    cv2.aruco.drawDetectedMarkers(debug, corners, ids)

    for i, mid in enumerate(ids.flatten().tolist()):
        img_pts = corners[i][0].astype(np.float64)
        ok, rvec, tvec = cv2.solvePnP(
            obj_pts, img_pts, K, D, flags=cv2.SOLVEPNP_IPPE_SQUARE
        )
        if not ok:
            continue
        # tvec = position du centre du marqueur dans le repère caméra
        pos = tvec.flatten()
        positions[mid] = pos

        label = f"ID{mid} ({pos[0]:.3f},{pos[1]:.3f},{pos[2]:.3f})"
        c = corners[i][0].astype(int)
        cv2.putText(debug, label, tuple(c[0] + np.array([0, -10])),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, (0, 255, 255), 1)

    return positions, debug


def _print_report(positions: dict, expected_ids: list[int]):
    if not positions:
        print("  Aucun marqueur détecté.")
        return

    print(f"\n  Marqueurs détectés : {sorted(positions.keys())}")
    print(f"  {'ID':>6}  {'X_cam':>8}  {'Y_cam':>8}  {'Z_cam':>8}  (m, repère caméra)")
    print("  " + "-" * 45)
    for mid in sorted(positions.keys()):
        p = positions[mid]
        tag = " ← OBJET?" if mid not in expected_ids else ""
        print(f"  {mid:>6}  {p[0]:>8.3f}  {p[1]:>8.3f}  {p[2]:>8.3f}{tag}")

    ids = sorted(positions.keys())
    if len(ids) >= 2:
        print("\n  Distances pairwise dans le repère caméra (3D euclidien) :")
        for i in range(len(ids)):
            for j in range(i + 1, len(ids)):
                a, b = ids[i], ids[j]
                d = np.linalg.norm(positions[a] - positions[b])
                print(f"    ID{a} ↔ ID{b} : {d*100:.1f} cm")

    ws_ids = [mid for mid in ids if mid in expected_ids]
    if len(ws_ids) == 4:
        pts = np.array([positions[m] for m in ws_ids])
        center = pts.mean(axis=0)
        print(f"\n  Centre workspace (cam) : {np.round(center, 3)}")
        w = np.linalg.norm(positions[ws_ids[0]] - positions[ws_ids[1]])
        h = np.linalg.norm(positions[ws_ids[0]] - positions[ws_ids[2]])
        print(f"  Dimensions estimées    : {w*100:.1f} cm × {h*100:.1f} cm")
        print()
        print("  Positions hardcodées actuelles (repère base_link) :")
        print("    slot 0 : X=0.250 m, Y=+0.215 m  (proche-gauche)")
        print("    slot 1 : X=0.500 m, Y=+0.215 m  (loin-gauche)")
        print("    slot 2 : X=0.250 m, Y=-0.215 m  (proche-droite)")
        print("    slot 3 : X=0.500 m, Y=-0.215 m  (loin-droite)")
        print("    → largeur attendue : 43.0 cm   profondeur attendue : 25.0 cm")


def _from_device(device_id: int, K, D, marker_size: float, aruco_dict_id: int,
                 n_frames: int = 10):
    cap = cv2.VideoCapture(device_id)
    if not cap.isOpened():
        print(f"Impossible d'ouvrir /dev/video{device_id}", file=sys.stderr)
        sys.exit(1)

    print(f"Lecture caméra /dev/video{device_id} — {n_frames} frames…")
    all_pos: dict[int, list[np.ndarray]] = {}
    debug_frame = None

    for _ in range(n_frames):
        ret, frame = cap.read()
        if not ret:
            break
        positions, debug_frame = _estimate_positions(frame, K, D, marker_size, aruco_dict_id)
        for mid, pos in positions.items():
            all_pos.setdefault(mid, []).append(pos)

    cap.release()

    # Moyenne sur les frames pour stabiliser
    averaged = {mid: np.mean(np.stack(pts), axis=0) for mid, pts in all_pos.items()}

    if debug_frame is not None:
        out = Path("aruco_measure_debug.jpg")
        cv2.imwrite(str(out), debug_frame)
        print(f"\nImage de debug sauvegardée : {out.resolve()}")

    return averaged


def _from_ros_topic(topic: str, K, D, marker_size: float, aruco_dict_id: int,
                    n_frames: int = 30):
    try:
        import rclpy
        from rclpy.node import Node
        from sensor_msgs.msg import Image
        from cv_bridge import CvBridge
    except ImportError:
        print("rclpy ou cv_bridge non disponible — source ROS2 d'abord.", file=sys.stderr)
        sys.exit(1)

    rclpy.init()
    bridge = CvBridge()
    collected: list[dict] = []
    debug_frame = [None]

    class _Collector(Node):
        def __init__(self):
            super().__init__("aruco_measure_ws")
            self.create_subscription(Image, topic, self._cb, 5)

        def _cb(self, msg):
            if len(collected) >= n_frames:
                return
            try:
                frame = bridge.imgmsg_to_cv2(msg, "bgr8")
            except Exception:
                return
            positions, debug = _estimate_positions(frame, K, D, marker_size, aruco_dict_id)
            if positions:
                collected.append(positions)
                debug_frame[0] = debug
                print(f"  frame {len(collected)}/{n_frames}  IDs: {sorted(positions.keys())}")

    node = _Collector()
    print(f"Écoute {topic} — attente de {n_frames} frames avec détections…")
    deadline = time.time() + 20.0
    while len(collected) < n_frames and time.time() < deadline:
        rclpy.spin_once(node, timeout_sec=0.1)

    node.destroy_node()
    rclpy.shutdown()

    if not collected:
        print("Aucun marqueur détecté sur le topic.", file=sys.stderr)
        sys.exit(1)

    all_pos: dict[int, list[np.ndarray]] = {}
    for frame_positions in collected:
        for mid, pos in frame_positions.items():
            all_pos.setdefault(mid, []).append(pos)

    averaged = {mid: np.mean(np.stack(pts), axis=0) for mid, pts in all_pos.items()}

    if debug_frame[0] is not None:
        out = Path("aruco_measure_debug.jpg")
        cv2.imwrite(str(out), debug_frame[0])
        print(f"\nImage de debug sauvegardée : {out.resolve()}")

    return averaged


def main():
    parser = argparse.ArgumentParser(description="Mesure workspace ArUco")
    src = parser.add_mutually_exclusive_group(required=True)
    src.add_argument("--device", type=int, metavar="N",
                     help="Index caméra OpenCV (/dev/videoN)")
    src.add_argument("--ros-topic", metavar="TOPIC",
                     help="Topic ROS image (ex. /camera/image_raw)")
    parser.add_argument("--calib", required=True, help="Chemin vers cam_0.npz")
    parser.add_argument("--marker-size", type=float, default=0.025,
                        help="Côté marqueur en m (défaut 0.025)")
    parser.add_argument("--dict", default="DICT_4X4_1000",
                        help="Dictionnaire ArUco (défaut DICT_4X4_1000)")
    parser.add_argument("--ws-ids", nargs="+", type=int, default=[19, 23, 25, 26],
                        help="IDs attendus pour le workspace (défaut 19 23 25 26)")
    parser.add_argument("--frames", type=int, default=30,
                        help="Nombre de frames à moyenner (défaut 30)")
    args = parser.parse_args()

    aruco_dict_id = getattr(cv2.aruco, args.dict, cv2.aruco.DICT_4X4_1000)
    K, D = _load_calib(args.calib)

    if args.device is not None:
        positions = _from_device(args.device, K, D, args.marker_size, aruco_dict_id,
                                 n_frames=args.frames)
    else:
        positions = _from_ros_topic(args.ros_topic, K, D, args.marker_size, aruco_dict_id,
                                    n_frames=args.frames)

    print("\n" + "=" * 55)
    print("  RÉSULTATS — positions marqueurs dans repère caméra")
    print("=" * 55)
    _print_report(positions, expected_ids=args.ws_ids)


if __name__ == "__main__":
    main()
