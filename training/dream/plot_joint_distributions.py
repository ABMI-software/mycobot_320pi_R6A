#!/usr/bin/env python3
"""Histogrammes de distribution des angles articulaires (j1..j6) d'un dataset de poses.

Lit un labels.csv (colonnes j1_deg..j6_deg) et trace une grille 2x3 d'histogrammes,
avec la moyenne de chaque articulation. Déduplique par index de pose : le CSV 3-cam
répète chaque pose une fois par caméra, on ne veut compter chaque angle qu'une fois.

Usage:
    python dream/plot_joint_distributions.py \
        --csv dream_data/real_3cam/labels.csv \
        --out dream_data/real_3cam/joint_distributions.png
"""
import argparse
import csv
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt


def load(csv_path):
    rows = {}
    with open(csv_path) as f:
        for r in csv.DictReader(f):
            idx = r.get("index")
            if idx is None or idx in rows:        # dédoublonnage par pose
                continue
            rows[idx] = [float(r[f"j{j}_deg"]) for j in range(1, 7)]
    poses = list(rows.values())
    return [[p[j] for p in poses] for j in range(6)], len(poses)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--csv", required=True)
    ap.add_argument("--out", default="joint_distributions.png")
    ap.add_argument("--bins", type=int, default=50)
    ap.add_argument("--overlay-csv", default=None,
                    help="2e dataset a superposer (mode coverage overlay)")
    ap.add_argument("--label", default="dataset A")
    ap.add_argument("--overlay-label", default="dataset B")
    ap.add_argument("--title", default=None, help="titre (single mode)")
    args = ap.parse_args()

    joints, n = load(args.csv)

    if args.overlay_csv is None:
        # --- mode single (joint_distributions_SYNTH.png) ---
        fig, axes = plt.subplots(2, 3, figsize=(13.5, 7.5))
        title = args.title or "Joint Angle Distributions (degrees)"
        fig.suptitle(title, fontsize=15)
        for j, ax in enumerate(axes.flat):
            data = joints[j]
            mean = sum(data) / len(data)
            w = [100.0 / len(data)] * len(data)   # % des poses -> somme = 100 %
            ax.hist(data, bins=args.bins, weights=w, color="#5B9BD5",
                    edgecolor="white", linewidth=0.3)
            ax.axvline(mean, color="red", linestyle="--", linewidth=1.2,
                       label=f"mean={mean:.1f}°")
            ax.set_title(f"j{j+1}_deg")
            ax.set_xlabel("degrees")
            ax.set_ylabel("% des poses")
            ax.legend(loc="upper right", fontsize=8)
        fig.tight_layout(rect=[0, 0, 1, 0.96])
        fig.savefig(args.out, dpi=120)
        print(f"✅ {n} poses uniques → {args.out}")
        return

    # --- mode overlay (joint_coverage_overlay.png) ---
    joints_b, nb = load(args.overlay_csv)
    wa = 100.0 / n      # chaque courbe normalisee a 100 % de SON dataset
    wb = 100.0 / nb     # -> comparable malgre n different, jamais > 100 %
    fig, axes = plt.subplots(2, 3, figsize=(13.5, 7.5))
    title = args.title or "Joint Coverage Overlay (degrees, % des poses par dataset)"
    fig.suptitle(title, fontsize=15)
    for j, ax in enumerate(axes.flat):
        ax.hist(joints[j], bins=args.bins, weights=[wa] * n, alpha=0.55,
                color="#4363d8", label=f"{args.label} (n={n})")
        ax.hist(joints_b[j], bins=args.bins, weights=[wb] * nb, alpha=0.55,
                color="#e6194B", label=f"{args.overlay_label} (n={nb})")
        ax.set_title(f"j{j+1}_deg")
        ax.set_xlabel("degrees")
        ax.set_ylabel("% des poses")
        if j == 0:
            ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout(rect=[0, 0, 1, 0.96])
    fig.savefig(args.out, dpi=120)
    print(f"✅ overlay {args.label} n={n} | {args.overlay_label} n={nb} → {args.out}")


if __name__ == "__main__":
    main()
