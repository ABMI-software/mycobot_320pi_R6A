#!/usr/bin/env python3
"""Estime la pose caméra→robot par DREAM, IMAGE PAR IMAGE (mode DREAM natif).

Pour chaque image : DREAM détecte les keypoints 2D, FK(encodeurs) donne les 3D en
repère base, solvePnP sort T_cam_world -> position + orientation de la caméra dans
le repère base. Comme la caméra est FIXE, toutes les images doivent donner ~la même
pose : la dispersion (écart-type) = la stabilité de l'estimation markerless.

Usage (venv_dream) :
    python estimate_pose_dream.py \\
        --dataset dream_data/real_3cam_session6 \\
        --weights checkpoints_dream/vgg_ultimate_v4_mix_ft_e30/best_network.pth
"""
import argparse
import csv
import sys
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

sys.path.insert(0, "/tmp/DREAM")
sys.path.insert(0, str(Path(__file__).resolve().parent))
import dream  # noqa: E402
from convert_to_ndds import load_real_intrinsics  # noqa: E402
from mycobot_fk import forward_kinematics, KEYPOINT_NAMES  # noqa: E402

KP = ["base", "link1", "link2", "link3", "link4", "link5", "link6"]


def fk_obj(q):
    pos, _ = forward_kinematics(q)
    return np.array([pos[n] for n in KEYPOINT_NAMES], dtype=np.float64)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset", required=True)
    ap.add_argument("--weights", required=True)
    ap.add_argument("--camera", default="arducam")
    ap.add_argument("--keypoints", default="base,link1,link2,link3,link4")
    args = ap.parse_args()

    kp_keep = [KP.index(n.strip()) for n in args.keypoints.split(",") if n.strip()]
    ds = Path(args.dataset)
    K = load_real_intrinsics(args.camera)
    dist = np.load(Path(__file__).resolve().parents[1] / "calibration" / "cam_0.npz")["dist"].astype(np.float64)
    net = dream.create_network_from_config_file(
        str(Path(args.weights).with_suffix(".yaml")), args.weights)
    net.enable_evaluation()

    rows = [r for r in csv.DictReader(open(ds / "labels.csv"))
            if r.get("camera", args.camera) == args.camera
            and (ds / r["image_path"]).exists()]

    positions, eulers = [], []
    for r in rows:
        rgb = np.array(Image.open(ds / r["image_path"]).convert("RGB"))
        det = np.array(net.keypoints_from_image(Image.fromarray(rgb))["detected_keypoints"], float)
        q = np.radians([float(r[f"j{j}_deg"]) for j in range(1, 7)])
        obj = fk_obj(q)
        o2, d2 = [], []
        for k in kp_keep:
            if det[k, 0] > -900 and det[k, 1] > -900:
                o2.append(obj[k]); d2.append(det[k])
        if len(o2) < 4:
            continue
        ok, rvec, tvec = cv2.solvePnP(np.array(o2), np.array(d2), K, dist,
                                      flags=cv2.SOLVEPNP_EPNP)
        if not ok:
            continue
        rvec, tvec = cv2.solvePnPRefineLM(np.array(o2), np.array(d2), K, dist, rvec, tvec)
        R, _ = cv2.Rodrigues(rvec)
        cam_pos = (-R.T @ tvec).ravel()                     # caméra dans repère base
        euler = cv2.RQDecomp3x3(R.T)[0]                      # roll,pitch,yaw (deg)
        positions.append(cam_pos); eulers.append(euler)

    positions = np.array(positions); eulers = np.array(eulers)
    n = len(positions)
    print(f"\n{n}/{len(rows)} images ont donné une pose (chacune INDÉPENDAMMENT).")
    if n == 0:
        return
    mp, sp = positions.mean(0), positions.std(0)
    me, se = eulers.mean(0), eulers.std(0)
    print("\n=== Pose caméra→robot estimée par DREAM (repère base, sans marqueur) ===")
    print(f"  position X = {mp[0]*1000:7.1f} ± {sp[0]*1000:4.1f} mm")
    print(f"  position Y = {mp[1]*1000:7.1f} ± {sp[1]*1000:4.1f} mm")
    print(f"  position Z = {mp[2]*1000:7.1f} ± {sp[2]*1000:4.1f} mm")
    print(f"  orient. roll  = {me[0]:6.1f} ± {se[0]:.1f} °")
    print(f"  orient. pitch = {me[1]:6.1f} ± {se[1]:.1f} °")
    print(f"  orient. yaw   = {me[2]:6.1f} ± {se[2]:.1f} °")
    print(f"\n  Stabilité (dispersion inter-images) : "
          f"±{np.linalg.norm(sp)*1000:.0f} mm / ±{np.linalg.norm(se):.1f}° "
          f"→ {'excellente' if np.linalg.norm(sp)<0.03 else 'correcte' if np.linalg.norm(sp)<0.08 else 'bruitée'}")


if __name__ == "__main__":
    main()
