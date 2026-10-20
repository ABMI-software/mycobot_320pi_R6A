#!/usr/bin/env python3
"""Slide de conclusion : le schéma DREAM réalisé sur le vrai MyCobot -> T_C^R.

Rangée 1 (pipeline) : image arducam -> DREAM keypoints -> (FK + intrinsèques) -> T_C^R.
Rangée 2 (preuve)   : squelette FK reprojeté via T_C^R posé sur le bras.

Usage (venv_dream) :
    python build_conclusion_slide.py \\
        --dataset dream_data/real_3cam_session6 --index 2524 \\
        --weights checkpoints_dream/vgg_ultimate_v4_mix_ft_e30/best_network.pth \\
        --extrinsic ../calibration/arducam_extrinsic_dream_v4.yaml \\
        --out ../calibration/conclusion_TCR.png
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
from matplotlib.gridspec import GridSpec

sys.path.insert(0, "/tmp/DREAM")
sys.path.insert(0, str(Path(__file__).resolve().parent))
import dream  # noqa: E402
from convert_to_ndds import load_real_intrinsics  # noqa: E402
from estimate_angles_from_keypoints import project_keypoints  # noqa: E402
from visual_servoing_dashboard import dream_keypoints  # noqa: E402

GREEN, ORANGE = "#3cb44b", "#f58231"


def euler_deg(R):
    sy = np.hypot(R[0, 0], R[1, 0])
    return (np.degrees(np.arctan2(R[2, 1], R[2, 2])),
            np.degrees(np.arctan2(-R[2, 0], sy)),
            np.degrees(np.arctan2(R[1, 0], R[0, 0])))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset", required=True)
    ap.add_argument("--index", required=True)
    ap.add_argument("--weights", required=True)
    ap.add_argument("--extrinsic", required=True)
    ap.add_argument("--camera", default="arducam")
    ap.add_argument("--out", default="../calibration/conclusion_TCR.png")
    args = ap.parse_args()

    ds = Path(args.dataset)
    K = load_real_intrinsics(args.camera)
    dist = np.load(Path(__file__).resolve().parents[1] / "calibration" / "cam_0.npz")["dist"].astype(np.float64)
    ext = yaml.safe_load(Path(args.extrinsic).read_text())
    T = np.array(ext["T_cam_world"], float)          # base->cam (pour projeter)
    Twc = np.array(ext["T_world_cam"], float)        # cam dans repère base (pour affichage)
    p = np.array(ext["camera_position_base_m"], float) * 1000
    roll, pitch, yaw = euler_deg(Twc[:3, :3])
    rvec, _ = cv2.Rodrigues(T[:3, :3]); tvec = T[:3, 3].reshape(3, 1)

    net = dream.create_network_from_config_file(
        str(Path(args.weights).with_suffix(".yaml")), args.weights)
    net.enable_evaluation()

    row = next(r for r in csv.DictReader(open(ds / "labels.csv"))
               if r.get("camera", args.camera) == args.camera and r["index"] == str(args.index))
    rgb = np.array(Image.open(ds / row["image_path"]).convert("RGB"))
    det = dream_keypoints(net, Image.fromarray(rgb))
    q = np.radians([float(row[f"j{j}_deg"]) for j in range(1, 7)])
    fk = project_keypoints(q, K, dist, rvec, tvec)

    fig = plt.figure(figsize=(15, 8.2))
    gs = GridSpec(2, 4, figure=fig, height_ratios=[1, 1.25], hspace=0.28, wspace=0.22)

    # ① image
    ax = fig.add_subplot(gs[0, 0]); ax.imshow(rgb)
    ax.set_title("① Image arducam\n640×480×3", fontsize=11); ax.axis("off")
    # ② DREAM keypoints
    ax = fig.add_subplot(gs[0, 1]); ax.imshow(rgb)
    vis = (det[:, 0] > -900) & (det[:, 1] > -900)
    ax.scatter(det[vis, 0], det[vis, 1], c=ORANGE, s=30, edgecolors="white", linewidths=0.5)
    ax.set_title("② DREAM → keypoints 2D\ndétection 3,3 px", fontsize=11); ax.axis("off")
    # ③ entrées PnP
    ax = fig.add_subplot(gs[0, 2]); ax.axis("off")
    ax.text(0.5, 0.5,
            "③ Entrées PnP\n\n• FK(encodeurs)\n   → keypoints 3D\n\n"
            "• intrinsèques cam_0.npz\n   fx, fy, cx, cy + distorsion\n\n(déjà calibrées, ChArUco)",
            ha="center", va="center", fontsize=11,
            bbox=dict(boxstyle="round", fc="#eef3ff", ec="#4363d8"))
    # ④ T_C^R
    ax = fig.add_subplot(gs[0, 3]); ax.axis("off")
    ax.text(0.5, 0.5,
            "④ PnP → $T_C^R$\nSANS MARQUEUR\n\n"
            f"position (base) :\n X = {p[0]:.0f} mm\n Y = {p[1]:.0f} mm\n Z = {p[2]:.0f} mm\n\n"
            f"orientation :\n roll {roll:.0f}°  pitch {pitch:.0f}°  yaw {yaw:.0f}°\n\n"
            "reproj médian : 3,3 px",
            ha="center", va="center", fontsize=11,
            bbox=dict(boxstyle="round", fc="#eaf7ea", ec=GREEN))

    for xf in (0.255, 0.505, 0.755):
        fig.text(xf, 0.70, "→", fontsize=26, ha="center", va="center", color="#444")

    # rangée 2 : preuve
    ax = fig.add_subplot(gs[1, :])
    ax.imshow(rgb)
    for a, b in [(0, 1), (1, 2), (2, 3), (3, 4)]:
        ax.plot([fk[a, 0], fk[b, 0]], [fk[a, 1], fk[b, 1]], "-", color=GREEN, lw=2.5)
    ax.scatter(fk[:5, 0], fk[:5, 1], c=GREEN, s=55, edgecolors="white", linewidths=0.8,
               label="FK reprojeté via $T_C^R$ (vérité géométrique)", zorder=3)
    ax.scatter(det[vis, 0], det[vis, 1], c=ORANGE, s=28, edgecolors="white", linewidths=0.5,
               label="détection DREAM", zorder=4)
    ax.set_title("Preuve : le squelette reprojeté via le $T_C^R$ estimé se pose sur le bras "
                 "(caméra fixe, sans marqueur)", fontsize=12)
    ax.axis("off"); ax.legend(loc="lower right", fontsize=10)

    fig.suptitle("DREAM sur MyCobot 320 Pi (réel, session non vue) : image → keypoints → PnP → "
                 "pose caméra-robot $T_C^R$, sans marqueur", fontsize=13, y=0.98)
    fig.savefig(args.out, dpi=130, bbox_inches="tight")
    print(f"✅ {args.out}")


if __name__ == "__main__":
    main()
