#!/usr/bin/env python3
"""Capture hand-eye eye-to-hand : marqueur ArUco sur J6 + FK, sur N poses.

À chaque pose : encodeurs -> FK -> T_base_gripper (base->J6) ; image arducam ->
détection du marqueur J6 (ID fixe) -> solvePnP (taille connue) -> T_cam_marker.
On enregistre les deux. solve_handeye.py en déduit base->caméra (cv2 hand-eye).

Le bras bouge : ping obligatoire, vitesse basse, go_home au début et à la fin.
On ne garde que les poses où le marqueur est bien vu (sinon on saute). Il faut de
la DIVERSITÉ d'orientation du marqueur -> les poses aléatoires font tourner j4/j5/j6.

Usage (venv_dream) :
    python capture_handeye_j6.py --pi-host 10.10.0.224 --marker-id 30 \\
        --marker-mm 40 --target 20 --preview
"""
import argparse
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent))          # training/
from capture_real_3cam import LocalCamera, RobotBridge, random_joint_angles  # noqa: E402
from mycobot_fk import forward_kinematics  # noqa: E402


def wait_until_settled(bridge, tol_deg=0.5, poll=0.4, timeout=8.0):
    """Attend que le bras soit immobile (2 lectures consécutives < tol) et renvoie
    les angles stabilisés. None si ça ne se stabilise pas -> pose ignorée."""
    prev = None; t0 = time.time()
    while time.time() - t0 < timeout:
        time.sleep(poll)
        enc = bridge.get_angles()
        if not enc or len(enc) != 6:
            continue
        if prev is not None and max(abs(a - b) for a, b in zip(enc, prev)) < tol_deg:
            return enc
        prev = enc
    return prev


def flange_faces_camera(enc, min_up):
    """Vrai si la normale du marqueur J6 (axe z local du flasque) pointe vers le
    haut (vers la caméra) ET si le flasque est dans une zone visible au-dessus de
    la table. Garde de la marge d'inclinaison -> diversité d'orientation."""
    _, T = forward_kinematics(np.radians(enc))
    R, p = T[6][:3, :3], T[6][:3, 3]
    up = R[2, 2] > min_up
    in_view = 0.05 < float(np.hypot(p[0], p[1])) < 0.40 and 0.10 < p[2] < 0.55
    return up and in_view


def sample_visible_pose(min_up, limit_fraction, tries=500):
    for _ in range(tries):
        q = random_joint_angles(limit_fraction=limit_fraction)
        if q != [0.0] * 6 and flange_faces_camera(q, min_up):
            return q
    return None


def detect_marker_pose(bgr, marker_id, size_m, K, dist):
    det = cv2.aruco.ArucoDetector(
        cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000),
        cv2.aruco.DetectorParameters())
    corners, ids, _ = det.detectMarkers(cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY))
    if ids is None or marker_id not in ids.flatten():
        return None, None, corners, ids
    c = corners[list(ids.flatten()).index(marker_id)].reshape(4, 2)
    s = size_m
    objp = np.array([[-s/2, s/2, 0], [s/2, s/2, 0],
                     [s/2, -s/2, 0], [-s/2, -s/2, 0]], np.float64)
    ok, rvec, tvec = cv2.solvePnP(objp, c, K, dist, flags=cv2.SOLVEPNP_IPPE_SQUARE)
    if not ok:
        return None, None, corners, ids
    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4); T[:3, :3] = R; T[:3, 3] = tvec.ravel()
    return T, c.mean(axis=0), corners, ids


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--pi-host", default="10.10.0.224")
    ap.add_argument("--index", type=int, default=0)
    ap.add_argument("--exposure", type=int, default=75)
    ap.add_argument("--marker-id", type=int, default=30)
    ap.add_argument("--marker-mm", type=float, default=40.0)
    ap.add_argument("--intrinsics-npz", default=str(HERE.parent / "calibration" / "cam_0.npz"))
    ap.add_argument("--out", default=str(HERE.parent / "calibration" / "handeye_data.npz"))
    ap.add_argument("--target", type=int, default=20, help="poses valides visées")
    ap.add_argument("--attempts", type=int, default=70, help="essais max")
    ap.add_argument("--limit-fraction", type=float, default=0.6)
    ap.add_argument("--min-up", type=float, default=0.5,
                    help="composante Z mini de la normale marqueur (0.5=incliné à 60°, "
                         "1=pile vers le haut). Baisse si trop peu de poses trouvées.")
    ap.add_argument("--speed", type=int, default=25)
    ap.add_argument("--settle", type=float, default=8.0,
                    help="timeout (s) d'attente d'immobilisation avant lecture+photo")
    ap.add_argument("--preview", action="store_true")
    args = ap.parse_args()

    if subprocess.run(["ping", "-c", "1", "-W", "2", args.pi_host],
                      stdout=subprocess.DEVNULL).returncode != 0:
        raise SystemExit(f"Robot {args.pi_host} injoignable — abandon.")

    npz = np.load(args.intrinsics_npz)
    K, dist = npz["mtx"].astype(np.float64), npz["dist"].astype(np.float64)

    cam = LocalCamera(args.index, "arducam", exposure=args.exposure)
    bridge = RobotBridge(args.pi_host); bridge.connect()
    print("power_on…"); bridge.send_command({"action": "power_on"}); time.sleep(1)
    print("Retour home…"); bridge.go_home(); time.sleep(3)

    def is_novel(enc):
        return all(max(abs(a - b) for a, b in zip(enc, k)) > 5.0 for k in angle_list)

    T_bg_list, T_cm_list, angle_list = [], [], []
    tries = 0
    prev_enc, frozen = None, 0
    try:
        while len(T_bg_list) < args.target and tries < args.attempts:
            tries += 1
            q = sample_visible_pose(args.min_up, args.limit_fraction)
            if q is None:
                print(f"  [{tries}] aucune pose visible trouvée (baisse --min-up)"); continue
            bridge.send_angles(q, speed=args.speed)
            enc = wait_until_settled(bridge, timeout=args.settle)
            if not enc or len(enc) != 6:
                print(f"  [{tries}] pas d'encodeurs, saut"); continue
            if prev_enc is not None and max(abs(a - b) for a, b in zip(enc, prev_enc)) < 1.0:
                frozen += 1
                if frozen >= 6:
                    raise SystemExit(
                        "\n⛔ Le robot NE BOUGE PAS (6 poses identiques d'affilée).\n"
                        "   Cause probable : servos non alimentés ou joint en butée/alarme.\n"
                        "   Réarme le robot (power_off → recentre j1 à la main → power_on/"
                        "redémarre le bridge) avant de relancer.")
            else:
                frozen = 0
            prev_enc = enc
            rgb = cam.capture(flush=3)
            if rgb is None:
                print(f"  [{tries}] pas d'image, saut"); continue
            bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
            T_cm, center, corners, ids = detect_marker_pose(
                bgr, args.marker_id, args.marker_mm / 1000.0, K, dist)
            seen = T_cm is not None and is_novel(enc)
            if seen:
                _, transforms = forward_kinematics(np.radians(enc))
                T_bg_list.append(transforms[6]); T_cm_list.append(T_cm)
                angle_list.append(enc)
            status = "✅ gardée" if seen else (
                "— déjà vue (pose peu différente)" if T_cm is not None else "— marqueur non vu")
            print(f"  [{tries}] {status}   ({len(T_bg_list)}/{args.target})")
            if args.preview:
                vis = bgr.copy()
                if ids is not None:
                    cv2.aruco.drawDetectedMarkers(vis, corners, ids)
                if seen:
                    cv2.circle(vis, tuple(center.astype(int)), 8, (0, 255, 0), 2)
                cv2.putText(vis, f"gardees {len(T_bg_list)}/{args.target}", (10, 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                            (0, 255, 0) if seen else (0, 0, 255), 2)
                try:
                    cv2.imshow("hand-eye capture (q=stop)", vis)
                    if (cv2.waitKey(1) & 0xFF) == ord("q"):
                        break
                except cv2.error:
                    cv2.imwrite(str(HERE.parent / "calibration" / "handeye_preview.png"), vis)
    finally:
        print("\nRetour home…"); bridge.go_home(); time.sleep(2)
        cam.close(); bridge.close(); cv2.destroyAllWindows()

    n = len(T_bg_list)
    if n < 8:
        raise SystemExit(f"Seulement {n} poses valides — il en faut ≥8 (idéal ≥15). "
                         "Rapproche/oriente le marqueur vers la caméra et relance.")
    np.savez(args.out,
             T_base_gripper=np.array(T_bg_list),
             T_cam_marker=np.array(T_cm_list),
             angles_deg=np.array(angle_list),
             marker_id=args.marker_id, marker_mm=args.marker_mm)
    print(f"\n{n} poses valides -> {args.out}\nLance maintenant : python solve_handeye.py")


if __name__ == "__main__":
    main()
