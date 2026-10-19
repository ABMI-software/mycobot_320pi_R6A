#!/usr/bin/env python3
"""Résout l'extrinsèque eye-to-hand (base->caméra) depuis handeye_data.npz.

Données : par pose, T_base_gripper (FK) et T_cam_marker (PnP du marqueur J6).

Méthode = FULL-POSE robuste. Contrainte, pour chaque pose i :
    X @ T_cam_marker[i] = T_base_gripper[i] @ Y
avec X = T_base_cam (ce qu'on veut) et Y = T_gripper_marker constants. On résout
X et Y (12 paramètres) par moindres carrés robustes (scipy), puis on REJETTE
itérativement les poses aberrantes — surtout les poses « flippées » (ambiguïté de
pose du marqueur plan : solvePnP choisit parfois la mauvaise des 2 solutions).
Résidu de pose = translation (mm) + rotation (deg).

Sortie : arducam_extrinsic_handeye.yaml (T_cam_world = base->cam, format dashboard).

Usage (venv_dream) :
    python solve_handeye.py
"""
import argparse
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.optimize import least_squares

HERE = Path(__file__).resolve().parent


def to_T(rv, tv):
    T = np.eye(4); T[:3, :3] = cv2.Rodrigues(np.asarray(rv, float))[0]; T[:3, 3] = tv
    return T


def inv(T):
    R, t = T[:3, :3], T[:3, 3]
    Ti = np.eye(4); Ti[:3, :3] = R.T; Ti[:3, 3] = -R.T @ t
    return Ti


def from_T(T):
    return cv2.Rodrigues(T[:3, :3])[0].ravel(), T[:3, 3].copy()


def fit(Tbg, Tcm, idx):
    def resid(p):
        X = to_T(p[0:3], p[3:6]); Y = to_T(p[6:9], p[9:12]); r = []
        for i in idx:
            A = X @ Tcm[i]; B = Tbg[i] @ Y
            r.extend((A[:3, 3] - B[:3, 3]).tolist())
            r.extend(cv2.Rodrigues(A[:3, :3].T @ B[:3, :3])[0].ravel().tolist())
        return r

    Y0 = np.eye(4); X0 = Tbg[idx[0]] @ Y0 @ inv(Tcm[idx[0]])
    rX, tX = from_T(X0); rY, tY = from_T(Y0)
    s = least_squares(resid, np.concatenate([rX, tX, rY, tY]),
                      loss="soft_l1", f_scale=0.01, max_nfev=30000)
    return s.x


def pose_err(p, Tbg, Tcm):
    """(translation_mm, rotation_deg) par pose."""
    X = to_T(p[0:3], p[3:6]); Y = to_T(p[6:9], p[9:12]); e = []
    for i in range(len(Tbg)):
        A = X @ Tcm[i]; B = Tbg[i] @ Y
        et = np.linalg.norm(A[:3, 3] - B[:3, 3]) * 1000
        er = np.degrees(np.linalg.norm(cv2.Rodrigues(A[:3, :3].T @ B[:3, :3])[0]))
        e.append((et, er))
    return np.array(e)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default=str(HERE.parent / "calibration" / "handeye_data.npz"))
    ap.add_argument("--intrinsics-npz", default=str(HERE.parent / "calibration" / "cam_0.npz"))
    ap.add_argument("--out", default=str(HERE.parent / "calibration" / "arducam_extrinsic_handeye.yaml"))
    ap.add_argument("--reject-px", type=float, default=10.0,
                    help="seuil de rejet des poses aberrantes (px)")
    args = ap.parse_args()

    d = np.load(args.data)
    Tbg, Tcm = d["T_base_gripper"], d["T_cam_marker"]
    n = len(Tbg)
    npz = np.load(args.intrinsics_npz)
    K, dist = npz["mtx"].astype(np.float64), npz["dist"].astype(np.float64)
    print(f"{n} poses chargées.")

    keep = list(range(n)); p = None
    for it in range(6):
        p = fit(Tbg, Tcm, keep)
        e = pose_err(p, Tbg, Tcm)
        score = e[:, 0] + 5.0 * e[:, 1]                 # mm + 5 mm/deg
        thr = max(20.0, 2.0 * np.median(score[keep]))
        new_keep = [i for i in range(n) if score[i] <= thr]
        print(f"  itér {it}: {len(keep)} poses -> médiane {np.median(e[keep,0]):.1f} mm / "
              f"{np.median(e[keep,1]):.1f}°, rejet > {thr:.0f} -> {len(new_keep)} poses")
        if new_keep == keep or len(new_keep) < 8:
            break
        keep = new_keep

    e = pose_err(p, Tbg, Tcm)
    X = to_T(p[0:3], p[3:6]); Y = to_T(p[6:9], p[9:12])   # X = T_base_cam
    T_cam_base = inv(X); cam_pos = X[:3, 3]
    med_mm = float(np.median(e[keep, 0])); med_deg = float(np.median(e[keep, 1]))
    print(f"\n{len(keep)}/{n} poses retenues (rejetées = poses flippées)")
    print(f"Résidu : médiane {med_mm:.1f} mm / {med_deg:.1f}°")
    print(f"Offset marqueur/flasque : {np.linalg.norm(Y[:3,3])*1000:.0f} mm")
    print(f"Caméra dans le repère base : X={cam_pos[0]:+.3f} Y={cam_pos[1]:+.3f} Z={cam_pos[2]:+.3f} m")
    verdict = "OK" if med_mm < 8 else ("CORRECT (limité par la précision du MyCobot)"
                                       if med_mm < 15 else "ÉLEVÉ — recapture avec plus de poses")
    print(f"Verdict : {verdict}")

    out = {
        "source": "solve_handeye (eye-to-hand full-pose robuste, marqueur J6)",
        "n_poses_total": int(n), "n_poses_kept": int(len(keep)),
        "median_err_mm": med_mm, "median_err_deg": med_deg,
        "gripper_marker_offset_mm": float(np.linalg.norm(Y[:3, 3]) * 1000),
        "T_cam_world": T_cam_base.tolist(),
        "T_world_cam": X.tolist(),
        "camera_position_base_m": cam_pos.tolist(),
    }
    Path(args.out).write_text(yaml.safe_dump(out, sort_keys=False))
    print(f"\nÉcrit -> {args.out}")
    if med_mm < 15:
        print("Branche-le sur le dashboard : --extrinsic ../calibration/arducam_extrinsic_handeye.yaml")


if __name__ == "__main__":
    main()
