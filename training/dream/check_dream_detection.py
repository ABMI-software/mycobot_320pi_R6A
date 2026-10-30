#!/usr/bin/env python3
"""Pré-check détection DREAM sur un dataset — ZÉRO calibration.

Fait tourner DREAM sur toutes les images couleur d'un dataset et rapporte, par
keypoint, le taux de détection (%) + le pic de belief map médian (confiance).
Répond à « ce point de vue est-il exploitable ? » avant toute calibration :
  < ~70%  -> hors domaine, calibrer/estimer serait inutile
  > ~85%  -> le modèle voit le bras -> on peut calibrer + tracer la courbe

Usage (venv_dream) :
    python check_dream_detection.py \\
        --dataset dream_data/real_arducam \\
        --weights checkpoints_dream/vgg_ultimate_v4_mix_ft_e30/best_network.pth
"""
import argparse
import sys
from pathlib import Path

import numpy as np
from PIL import Image

sys.path.insert(0, "/tmp/DREAM")
import dream  # noqa: E402

KP = ["base", "link1", "link2", "link3", "link4", "link5", "link6"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset", required=True, help="dossier contenant images/*.color.png")
    ap.add_argument("--weights", required=True)
    ap.add_argument("--glob", default="images/*.color.png")
    args = ap.parse_args()

    net = dream.create_network_from_config_file(
        str(Path(args.weights).with_suffix(".yaml")), args.weights)
    net.enable_evaluation()

    imgs = sorted(Path(args.dataset).glob(args.glob))
    n = len(imgs)
    if n == 0:
        raise SystemExit(f"Aucune image ({args.glob}) dans {args.dataset}")
    print(f"{n} images — inference DREAM…\n")

    det = np.zeros(7, int)
    peaks = [[] for _ in range(7)]
    for i, p in enumerate(imgs):
        res = net.keypoints_from_image(Image.open(p).convert("RGB"), debug=True)
        kps = np.array(res["detected_keypoints"], dtype=float)
        bmaps = res["belief_maps"]
        for k in range(7):
            if kps[k, 0] > -900 and kps[k, 1] > -900:
                det[k] += 1
                peaks[k].append(float(bmaps[k].max()))
        print(f"\r  {i+1}/{n}", end="", flush=True)
    print("\n")

    print(f"{'keypoint':10s}  {'détection':>10s}  {'pic belief (méd)':>18s}")
    print("-" * 42)
    for k in range(7):
        pk = np.median(peaks[k]) if peaks[k] else 0.0
        print(f"{KP[k]:10s}  {100*det[k]/n:8.1f} %  {pk:16.2f}")
    overall = 100.0 * det.sum() / (7 * n)
    print("-" * 42)
    print(f"{'GLOBAL':10s}  {overall:8.1f} %\n")
    if overall < 70:
        print("  ❌ < 70% — hors domaine. Rapprocher du montage d'entraînement.")
    elif overall < 85:
        print("  ⚠️ 70–85% — limite. Courbe possible mais trous en distal (link5/6).")
    else:
        print("  ✅ > 85% — le modèle voit le bras. On continue (self-calib + courbe).")


if __name__ == "__main__":
    main()
