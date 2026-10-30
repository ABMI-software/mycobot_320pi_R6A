#!/usr/bin/env python3
"""Superpose les keypoints PRÉDITS par DREAM sur de vraies images (sans GT).

Sur données réelles on n'a pas de vérité-terrain fiable → la seule façon honnête de
juger un modèle est visuelle : on dessine les 7 keypoints prédits (+ squelette) sur
l'image réelle et on regarde s'ils tombent sur le robot.

Usage:
    python dream/overlay_predictions.py \
        --weights dream/vgg_mix_20k_50k_e50/best_network.pth \
        --data dream_data/real_astra_ndds --n 8 \
        --out dream_data/real_3cam/overlay_astra.png
"""
import os, sys, glob, argparse
import numpy as np
import torch
from PIL import Image as PILImage
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, os.path.dirname(__file__))
import dream
from mycobot_fk import KEYPOINT_NAMES

SKELETON = [(0, 1), (1, 2), (2, 3), (3, 4), (4, 5), (5, 6)]
COLORS = ["#e6194B", "#f58231", "#ffe119", "#3cb44b", "#42d4f4", "#4363d8", "#911eb4"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", "-w", required=True)
    ap.add_argument("--data", "-d", required=True)
    ap.add_argument("--n", type=int, default=8)
    ap.add_argument("--out", "-o", required=True)
    args = ap.parse_args()

    cfg = os.path.splitext(args.weights)[0] + ".yaml"
    net = dream.create_network_from_config_file(cfg, args.weights)
    net.enable_evaluation()

    imgs = sorted(glob.glob(os.path.join(args.data, "??????.rgb.png")))
    if not imgs:
        print("❌ aucune image .rgb.png dans", args.data); return
    step = max(1, len(imgs) // args.n)
    imgs = imgs[::step][:args.n]

    cols = 2
    rows = (len(imgs) + cols - 1) // cols
    fig, axes = plt.subplots(rows, cols, figsize=(cols * 5.5, rows * 4.2))
    axes = np.array(axes).reshape(-1)

    for ax, ip in zip(axes, imgs):
        image = PILImage.open(ip).convert("RGB")
        with torch.no_grad():
            res = net.keypoints_from_image(image, debug=False)
        kps = res["detected_keypoints"]
        ax.imshow(image)
        pts = []
        for ki in range(len(KEYPOINT_NAMES)):
            d = kps[ki] if ki < len(kps) else None
            if d is None or len(d) < 2 or d[0] is None or float(d[0]) < -900:
                pts.append(None); continue
            x, y = float(d[0]), float(d[1])
            pts.append((x, y))
            ax.scatter([x], [y], c=COLORS[ki], s=90, edgecolors="black",
                       linewidths=0.9, zorder=3)
            ax.annotate(KEYPOINT_NAMES[ki].replace("mycobot320_", ""), (x, y),
                        color="white", fontsize=8, weight="bold",
                        xytext=(4, 4), textcoords="offset points", zorder=4)
        for a, b in SKELETON:
            if pts[a] and pts[b]:
                ax.plot([pts[a][0], pts[b][0]], [pts[a][1], pts[b][1]],
                        c="white", lw=1.5, zorder=2, alpha=0.8)
        n_det = sum(1 for p in pts if p)
        ax.set_title(f"{os.path.basename(ip)[:6]}  ({n_det}/7 keypoints)", fontsize=8)
        ax.axis("off")

    for ax in axes[len(imgs):]:
        ax.axis("off")

    fig.suptitle(f"Keypoints prédits sur RÉEL — {os.path.basename(args.data)}\n"
                 "base=rouge link1=orange link2=jaune link3=vert link4=cyan link5=bleu link6=violet",
                 fontsize=11)
    fig.tight_layout(rect=[0, 0, 1, 0.94])
    fig.savefig(args.out, dpi=110)
    print(f"✅ {len(imgs)} images → {args.out}")


if __name__ == "__main__":
    main()
