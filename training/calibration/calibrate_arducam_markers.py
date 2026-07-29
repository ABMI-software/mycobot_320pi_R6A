#!/usr/bin/env python3
"""Calibration extrinsèque arducam à partir des 4 ArUco fixes (PnP indépendant).

Contrairement à self_calibrate_arducam (robot-comme-cible = circulaire, absorbe
le biais de DREAM), ici la vérité vient des marqueurs : positions base connues
(workspace_markers.yaml) + centres détectés dans l'image -> solvePnP -> extrinsèque
INDÉPENDANT de DREAM. C'est lui qui permet un overlay FK honnête.

Chaque marqueur pris à plat (Z=0) ; on utilise le centre (robuste à la rotation
dans le plan). solvePnP planaire (IPPE). Erreur de reprojection = qualité.

Sortie : arducam_extrinsic_markers.yaml (même format que *_selfcal.yaml :
T_cam_world = monde(base)->caméra, directement utilisable par le dashboard).

Usage (venv_dream) :
    # sur une photo déjà prise (les 4 marqueurs visibles, bras peu importe)
    python calibrate_arducam_markers.py --image aruco_check.png

    # capture live depuis l'arducam
    python calibrate_arducam_markers.py --index 0 --exposure 75
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

HERE = Path(__file__).resolve().parent


def load_markers(yaml_path):
    d = yaml.safe_load(Path(yaml_path).read_text())
    pts = {int(k): np.array(v, dtype=np.float64) for k, v in d["markers"].items()}
    return pts, float(d["marker_size_m"])


def detect_centers(bgr):
    det = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000),
        cv2.aruco.DetectorParameters())
    corners, ids, _ = det.detectMarkers(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY))
    centers = {}
    if ids is not None:
        for c, i in zip(corners, ids.flatten()):
            centers[int(i)] = c.reshape(4, 2).mean(axis=0)
    return centers, corners, ids


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--markers", default=str(HERE / "workspace_markers.yaml"))
    ap.add_argument("--intrinsics-npz", default=str(HERE / "cam_0.npz"))
    ap.add_argument("--out", default=str(HERE / "arducam_extrinsic_markers.yaml"))
    ap.add_argument("--image", help="photo existante ; sinon capture live")
    ap.add_argument("--index", type=int, default=0)
    ap.add_argument("--exposure", type=int, default=75)
    args = ap.parse_args()

    npz = np.load(args.intrinsics_npz)
    K, dist = npz["mtx"].astype(np.float64), npz["dist"].astype(np.float64)
    world, _size = load_markers(args.markers)

    if args.image:
        bgr = cv2.imread(args.image)
        if bgr is None:
            raise SystemExit(f"image introuvable : {args.image}")
    else:
        sys.path.insert(0, str(HERE.parent))
        from capture_real_3cam import LocalCamera
        cam = LocalCamera(args.index, "arducam", exposure=args.exposure)
        rgb = None
        for _ in range(5):
            rgb = cam.capture(flush=3)
            if rgb is not None:
                break
        cam.close()
        if rgb is None:
            raise SystemExit("pas de frame arducam")
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

    centers, corners, ids = detect_centers(bgr)
    usable = [i for i in world if i in centers]
    print(f"Marqueurs attendus : {sorted(world)}")
    print(f"Marqueurs détectés : {sorted(centers)}")
    print(f"Utilisables (connus + vus) : {sorted(usable)}  ({len(usable)}/4)")
    if len(usable) < 4:
        raise SystemExit("Il faut les 4 marqueurs connus visibles pour un PnP fiable.")

    obj = np.array([world[i] for i in usable], dtype=np.float64)
    img = np.array([centers[i] for i in usable], dtype=np.float64)
    ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist, flags=cv2.SOLVEPNP_IPPE)
    if not ok:
        raise SystemExit("solvePnP a échoué.")

    proj, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    proj = proj.reshape(-1, 2)
    err = np.linalg.norm(proj - img, axis=1)
    print("\nErreur de reprojection par marqueur :")
    for i, e in zip(usable, err):
        print(f"  id {i:2d} : {e:5.2f} px")
    rms = float(np.sqrt((err ** 2).mean()))
    print(f"  RMS   : {rms:5.2f} px  ({'OK' if rms < 3 else 'ÉLEVÉ — vérifie les mesures'})")

    R, _ = cv2.Rodrigues(rvec)
    T_cam_world = np.eye(4); T_cam_world[:3, :3] = R; T_cam_world[:3, 3] = tvec.ravel()
    T_world_cam = np.linalg.inv(T_cam_world)
    cam_pos = T_world_cam[:3, 3]
    print(f"\nCaméra dans le repère base : "
          f"X={cam_pos[0]:+.3f}  Y={cam_pos[1]:+.3f}  Z={cam_pos[2]:+.3f} m")

    out = {
        "source": "calibrate_arducam_markers (4 ArUco fixes, PnP independant)",
        "rms_reproj_px": rms,
        "markers_used": sorted(usable),
        "T_cam_world": T_cam_world.tolist(),
        "T_world_cam": T_world_cam.tolist(),
        "camera_position_base_m": cam_pos.tolist(),
    }
    Path(args.out).write_text(yaml.safe_dump(out, sort_keys=False))
    print(f"\nÉcrit -> {args.out}")

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(bgr, corners, ids)
    for (u, v), (pu, pv) in zip(img, proj):
        cv2.circle(bgr, (int(u), int(v)), 6, (0, 0, 255), 2)      # détecté = rouge
        cv2.drawMarker(bgr, (int(pu), int(pv)), (0, 255, 0),
                       cv2.MARKER_CROSS, 12, 2)                     # reprojeté = vert
    viz = HERE / "arducam_calib_reproj.png"
    cv2.imwrite(str(viz), bgr)
    print(f"Contrôle visuel (rouge=détecté, vert=reprojeté) -> {viz}")


if __name__ == "__main__":
    main()
