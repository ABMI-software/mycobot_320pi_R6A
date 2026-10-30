#!/usr/bin/env python3
"""Décompose l'erreur d'angle : plancher SOLVEUR (keypoints parfaits) vs DREAM réel.

Sur les mêmes poses (mêmes encodeurs, même extrinsèque) on compare, par joint :
  - PLANCHER : FK(encodeurs) projetée + bruit 1 px → estimate_angles  (qualité pure
    du solveur, si la détection était parfaite),
  - DREAM    : détection réelle de DREAM → estimate_angles.

L'écart entre les deux barres = ce que coûte la détection 2D de DREAM. Montre que la
math est bonne (~1-3°) et que le reste vient de la vision.

Usage (venv_dream) :
    python decompose_angle_error.py \\
        --dataset dream_data/real_3cam_session6 \\
        --weights checkpoints_dream/vgg_ultimate_v4_mix_ft_e30/best_network.pth \\
        --extrinsic ../calibration/arducam_extrinsic_dream_v4.yaml
"""
import argparse
import csv
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml
from PIL import Image
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, "/tmp/DREAM")
sys.path.insert(0, str(Path(__file__).resolve().parent))
import dream  # noqa: E402
from convert_to_ndds import load_real_intrinsics  # noqa: E402
from estimate_angles_from_keypoints import estimate_angles, project_keypoints  # noqa: E402
from visual_servoing_dashboard import dream_keypoints  # noqa: E402


def wrap(x):
    return (x + 180.0) % 360.0 - 180.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset", required=True)
    ap.add_argument("--weights", required=True)
    ap.add_argument("--extrinsic", required=True)
    ap.add_argument("--camera", default="arducam")
    ap.add_argument("--noise-px", type=float, default=1.0)
    ap.add_argument("--max-rms-px", type=float, default=5.0)
    ap.add_argument("--out", default="../calibration/angle_error_decomp.png")
    args = ap.parse_args()

    ds = Path(args.dataset)
    K = load_real_intrinsics(args.camera)
    dist = np.load(Path(__file__).resolve().parents[1] / "calibration" / "cam_0.npz")["dist"].astype(np.float64)
    ext = yaml.safe_load(Path(args.extrinsic).read_text())
    T = np.array(ext["T_cam_world"], float)
    rvec, _ = cv2.Rodrigues(T[:3, :3]); tvec = T[:3, 3].reshape(3, 1)
    net = dream.create_network_from_config_file(
        str(Path(args.weights).with_suffix(".yaml")), args.weights)
    net.enable_evaluation()

    rows = [r for r in csv.DictReader(open(ds / "labels.csv"))
            if r.get("camera", args.camera) == args.camera
            and (ds / r["image_path"]).exists()]
    rng = np.random.default_rng(0)

    floor_err, dream_err = [], []
    for r in rows:
        q_true = np.radians([float(r[f"j{j}_deg"]) for j in range(1, 7)])
        # plancher solveur : keypoints parfaits + bruit
        proj = project_keypoints(q_true, K, dist, rvec, tvec)
        noisy = proj + rng.normal(0, args.noise_px, proj.shape)
        try:
            qf, rf, _ = estimate_angles(noisy, K, dist, rvec, tvec, n_restarts=8)
            if rf <= args.max_rms_px:
                floor_err.append(np.abs(wrap(np.degrees(qf) - np.degrees(q_true))))
        except ValueError:
            pass
        # DREAM réel
        kps = dream_keypoints(net, Image.open(ds / r["image_path"]).convert("RGB"))
        if np.isnan(kps).all(axis=1).sum() > 4:
            continue
        try:
            qd, rd, _ = estimate_angles(kps, K, dist, rvec, tvec, n_restarts=8)
            if rd <= args.max_rms_px:
                dream_err.append(np.abs(wrap(np.degrees(qd) - np.degrees(q_true))))
        except ValueError:
            pass

    floor = np.median(np.array(floor_err), axis=0)
    drea = np.median(np.array(dream_err), axis=0)
    print(f"n plancher={len(floor_err)}  n dream={len(dream_err)}")
    print("joint   plancher(1px)   DREAM réel")
    for j in range(6):
        print(f"  j{j+1}      {floor[j]:6.1f}°       {drea[j]:6.1f}°")

    x = np.arange(6); w = 0.38
    fig, ax = plt.subplots(figsize=(9, 5))
    ax.bar(x - w/2, floor, w, label=f"plancher solveur (keypoints parfaits, {args.noise_px:.0f}px)",
           color="#4363d8")
    ax.bar(x + w/2, drea, w, label="DREAM réel (détection)", color="#f58231")
    for j in range(6):
        ax.text(x[j]-w/2, floor[j]+0.5, f"{floor[j]:.0f}", ha="center", fontsize=8)
        ax.text(x[j]+w/2, drea[j]+0.5, f"{drea[j]:.0f}", ha="center", fontsize=8)
    ax.set_xticks(x); ax.set_xticklabels([f"j{j+1}" for j in range(6)])
    ax.set_ylabel("erreur angulaire médiane (°)")
    ax.set_title("Erreur d'angle : la math (~1-3°) vs le coût de détection DREAM")
    ax.legend()
    ax.annotate("j6 sur-axe\n(non observable)", (5, drea[5]), (4.3, drea[5]*0.6),
                fontsize=8, color="gray", arrowprops=dict(arrowstyle="->", color="gray"))
    fig.tight_layout(); fig.savefig(args.out, dpi=130)
    print(f"✅ {args.out}")


if __name__ == "__main__":
    main()
