#!/usr/bin/env python3
"""Preview live de l'arducam (vue haut) avec les réglages image du 3-cam.

Ouvre l'arducam en V4L2 et applique l'amélioration image de capture_real_3cam
(set_arducam_exposure : exposure=75, gain=0, brightness=0 — rendu « session4 »,
mean≈86), pour que le preview ait le MÊME rendu que les images d'entraînement.

Bonus placement : reprojette les 4 marqueurs (positions base connues) via
l'extrinsèque calibré camera_extrinsic.yaml + les intrinsèques cam_0.npz. Si
l'arducam est bien à sa position d'origine (0.69,0,0.37 m), les cibles VERTES
(reprojection) tombent sur les marqueurs réels détectés (contours cyan). L'écart
en pixels affiché = à quel point le placement colle à la calibration.

Touches : q/Échap quitter · s snapshot. Headless → écrit arducam_preview.png.

Usage (venv_dream) :
    source ~/ros_jazzy/venv_dream/bin/activate
    python arducam_preview.py --index 0 --exposure 75
"""
import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))            # training/ pour capture_real_3cam
sys.path.insert(0, str(HERE.parent / "dream"))  # pour project_keypoints / FK
from capture_real_3cam import LocalCamera, RobotBridge   # noqa: E402


def load_camera_model():
    """K, dist, rvec, tvec depuis la calibration arducam (intrinsèque + extrinsèque)."""
    npz = np.load(HERE / "cam_0.npz")
    K, dist = npz["mtx"].astype(np.float64), npz["dist"].astype(np.float64)
    ext = yaml.safe_load((HERE / "camera_extrinsic.yaml").read_text())
    T_cam_world = np.array(ext["T_cam_world"], dtype=np.float64)
    rvec, _ = cv2.Rodrigues(T_cam_world[:3, :3])
    tvec = T_cam_world[:3, 3].reshape(3, 1)
    return K, dist, rvec, tvec


def load_reprojection():
    """rvec, tvec, K, dist, {id:(u,v)_attendu} depuis la calibration arducam."""
    npz = np.load(HERE / "cam_0.npz")
    K, dist = npz["mtx"].astype(np.float64), npz["dist"].astype(np.float64)
    ext = yaml.safe_load((HERE / "camera_extrinsic.yaml").read_text())
    T_cam_world = np.array(ext["T_cam_world"], dtype=np.float64)
    rvec, _ = cv2.Rodrigues(T_cam_world[:3, :3])
    tvec = T_cam_world[:3, 3].reshape(3, 1)
    mk = yaml.safe_load((HERE / "workspace_markers.yaml").read_text())["markers"]
    world = {int(k): np.array(v, dtype=np.float64) for k, v in mk.items()}
    proj = {}
    for mid, xyz in world.items():
        uv, _ = cv2.projectPoints(xyz.reshape(1, 3), rvec, tvec, K, dist)
        proj[mid] = uv.reshape(2)
    return proj


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--index", type=int, default=0, help="index V4L2 de l'arducam")
    ap.add_argument("--exposure", type=int, default=75, help="exposure_time_absolute (réglage 3-cam)")
    ap.add_argument("--robot", action="store_true",
                    help="projeter le squelette FK du bras (vérif placement SANS marqueurs)")
    ap.add_argument("--pi-host", default="10.10.0.224")
    args = ap.parse_args()

    try:
        expected = load_reprojection()
    except Exception as e:
        print(f"(reprojection marqueurs indispo : {e})")
        expected = {}

    bridge = project_kp = cam_model = None
    if args.robot:
        from estimate_angles_from_keypoints import project_keypoints as project_kp  # noqa
        cam_model = load_camera_model()
        bridge = RobotBridge(args.pi_host); bridge.connect()

    cam = LocalCamera(args.index, "arducam", exposure=args.exposure)
    det = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000),
        cv2.aruco.DetectorParameters())
    print("Preview arducam — q/Échap quitter, s snapshot.")
    headless = False

    while True:
        rgb = cam.capture(flush=2)
        if rgb is None:
            print("… pas de frame"); time.sleep(0.2); continue
        bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)

        corners, ids, _ = det.detectMarkers(cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY))
        seen = {}
        if ids is not None:
            cv2.aruco.drawDetectedMarkers(bgr, corners, ids)
            for c, mid in zip(corners, ids.flatten()):
                seen[int(mid)] = c.reshape(4, 2).mean(0)

        # cibles reprojetées (vertes) + écart px si le marqueur est aussi détecté
        errs = []
        for mid, uv in expected.items():
            u, v = int(round(uv[0])), int(round(uv[1]))
            cv2.drawMarker(bgr, (u, v), (0, 255, 0), cv2.MARKER_TILTED_CROSS, 16, 2)
            cv2.putText(bgr, str(mid), (u + 8, v), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            if mid in seen:
                e = float(np.linalg.norm(seen[mid] - uv)); errs.append(e)
                cv2.line(bgr, (u, v), tuple(seen[mid].astype(int)), (0, 200, 255), 1)

        # squelette FK projeté depuis les encodeurs live (vérif SANS marqueurs)
        if bridge is not None:
            enc = bridge.get_angles()
            if enc and len(enc) == 6:
                q = np.radians(enc)
                kp = project_kp(q, *cam_model)          # (7,2)
                pts = [tuple(np.round(p).astype(int)) for p in kp]
                for a, b in zip(pts[:-1], pts[1:]):
                    cv2.line(bgr, a, b, (255, 0, 255), 2)
                for i, p in enumerate(pts):
                    cv2.circle(bgr, p, 5, (255, 0, 255), -1)
                    cv2.circle(bgr, p, 6, (255, 255, 255), 1)
                cv2.putText(bgr, "magenta = squelette FK (doit coller au bras reel)",
                            (10, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 255), 2)

        if expected:
            if errs:
                msg = f"placement: ecart moyen {np.mean(errs):.0f}px  ({len(errs)}/4 marqueurs)"
                col = (0, 255, 0) if np.mean(errs) < 8 else (0, 165, 255)
            else:
                msg = "placement: 0/4 marqueurs (normal si pas poses) — utilise --robot"
                col = (200, 200, 200)
            cv2.putText(bgr, msg, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)
            cv2.putText(bgr, "vert=reprojection calibree  cyan=detecte", (10, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)

        if headless:
            cv2.imwrite(str(HERE / "arducam_preview.png"), bgr); time.sleep(0.3); continue
        try:
            cv2.imshow("arducam preview (vue haut)", bgr)
            k = cv2.waitKey(30) & 0xFF
            if k in (ord("q"), 27):
                break
            if k == ord("s"):
                cv2.imwrite(str(HERE / "arducam_preview.png"), bgr)
                print(f"snapshot -> {HERE/'arducam_preview.png'}")
        except cv2.error:
            headless = True
            print("Pas d'affichage GUI -> j'écris arducam_preview.png en boucle.")

    cam.close()
    if bridge is not None:
        bridge.close()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
