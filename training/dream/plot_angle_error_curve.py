#!/usr/bin/env python3
"""Courbe d'écart par joint — angles estimés (caméra IA) vs encodeurs réels.

Le livrable d'évaluation eye-to-hand : pour un dataset astra RGB-D + encodeurs
(capture_astra_rgbd.py), rejoue DREAM sur chaque image couleur, remonte aux
angles articulaires via la profondeur (estimate_angles_3d), et compare aux
encodeurs enregistrés. Trace l'écart |estimé − encodeur| par joint j1..j6.

Attendu (cf. self-tests) : j1–j4 précis, j5 faible, j6 non observable (le
keypoint link6 est sur l'axe de j6 — aucune caméra ne le récupère).

Usage (venv_dream) :
    source ~/ros_jazzy/venv_dream/bin/activate
    python plot_angle_error_curve.py \\
        --dataset dream_data/real_astra_rgbd \\
        --weights ../checkpoints_dream/vgg_ultimate_v4_mix_ft_e30/best_network.pth \\
        --extrinsic ../calibration/astra_extrinsic.yaml \\
        --intrinsics-npz ../calibration/cam_astra.npz \\
        --out dream_data/real_astra_rgbd/angle_error_curve.png
"""
import argparse
import csv
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import torch
from PIL import Image

sys.path.insert(0, "/tmp/DREAM")   # lib DREAM vendored (comme evaluate_dream.py)
import dream
from estimate_angles_from_keypoints import (
    deproject_to_base, estimate_angles_3d, load_extrinsic)

N_KP = 7


def dream_keypoints(net, pil_image):
    """(7,2) keypoints en coords image brutes ; NaN si non détecté (sentinel < -900)."""
    with torch.no_grad():
        res = net.keypoints_from_image(pil_image, image_preprocessing_override=None, debug=False)
    det = res["detected_keypoints"]
    out = np.full((N_KP, 2), np.nan)
    for k in range(N_KP):
        if k < len(det) and det[k] is not None and det[k][0] is not None:
            u, v = float(det[k][0]), float(det[k][1])
            if u > -900 and v > -900:
                out[k] = (u, v)
    return out


def wrap_deg(x):
    return (x + 180.0) % 360.0 - 180.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset", required=True)
    ap.add_argument("--weights", required=True)
    ap.add_argument("--extrinsic", required=True)
    ap.add_argument("--intrinsics-npz", required=True)
    ap.add_argument("--out", default=None)
    ap.add_argument("--max-rms-mm", type=float, default=25.0,
                    help="rejette les solves 3D dont le résidu dépasse ce seuil")
    args = ap.parse_args()

    ds = Path(args.dataset)
    K = np.load(args.intrinsics_npz)["mtx"].astype(np.float64)
    T_world_cam = load_extrinsic(args.extrinsic)
    net = dream.create_network_from_config_file(
        str(Path(args.weights).with_suffix(".yaml")), args.weights)

    rows = list(csv.DictReader(open(ds / "labels.csv")))
    print(f"{len(rows)} poses | K fx={K[0,0]:.1f} | extrinsèque {Path(args.extrinsic).name}")

    errors = []      # (6,) écart signé par pose retenue
    enc_all = []     # encodeur j1 (pour trier la courbe façon "balayage")
    kept = rejected = 0
    for r in rows:
        idx = int(r["index"])
        enc = np.array([float(r[f"j{j}_deg"]) for j in range(1, 7)])
        cpath = ds / "images" / f"{idx:06d}.color.png"
        dpath = ds / "images" / f"{idx:06d}.depth.npy"
        if not (cpath.exists() and dpath.exists()):
            continue
        kps = dream_keypoints(net, Image.open(cpath).convert("RGB"))
        depth = np.load(dpath).astype(np.float64)
        base_pts = deproject_to_base(kps, depth, K, T_world_cam)
        if (~np.isnan(base_pts).any(axis=1)).sum() < 3:
            rejected += 1; continue
        q_est, rms_mm, _ = estimate_angles_3d(base_pts)
        if rms_mm > args.max_rms_mm:
            rejected += 1; continue
        errors.append(wrap_deg(np.degrees(q_est) - enc))
        enc_all.append(enc[0])
        kept += 1

    if kept == 0:
        raise SystemExit("Aucune pose exploitable — vérifie grabber/extrinsèque/keypoints.")
    E = np.abs(np.array(errors))            # écart absolu (poses × 6)
    med = np.median(E, axis=0)
    print(f"\nRetenues {kept} / rejetées {rejected}")
    print("Écart médian par joint (deg) : " +
          "  ".join(f"j{j+1}={med[j]:.1f}" for j in range(6)))

    fig, axes = plt.subplots(2, 3, figsize=(14, 8))
    fig.suptitle("Écart par joint — angles estimés (caméra IA) vs encodeurs réels", fontsize=15)
    for j, ax in enumerate(axes.flat):
        curve = np.sort(E[:, j])
        ax.plot(np.arange(len(curve)), curve, color="#4363d8", lw=1.6)
        ax.axhline(med[j], color="red", ls="--", lw=1.0, label=f"médiane={med[j]:.1f}°")
        blind = " — NON OBSERVABLE (sur-axe)" if j == 5 else (" (faible)" if j == 4 else "")
        ax.set_title(f"j{j+1}{blind}")
        ax.set_xlabel("poses (triées par écart)")
        ax.set_ylabel("|écart| (deg)")
        ax.legend(loc="upper left", fontsize=8)
        ax.grid(alpha=0.3)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    out = args.out or str(ds / "angle_error_curve.png")
    fig.savefig(out, dpi=120)
    print(f"✅ {out}")


if __name__ == "__main__":
    main()
