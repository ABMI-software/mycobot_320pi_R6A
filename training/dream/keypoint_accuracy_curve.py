#!/usr/bin/env python3
"""Courbe 1 — précision des keypoints DREAM (SANS calibration).

Évalue le modèle seul : erreur en pixels entre les keypoints détectés par DREAM
et la vérité terrain (projected_location des JSON NDDS). N'utilise QUE les labels
déjà présents dans le dataset — aucun extrinsèque, aucune calibration.

Produit, par dataset :
  - taux de détection + erreur médiane (px) par keypoint
  - courbe PCK : % de keypoints corrects en fonction du seuil (px)

C'est « la courbe avant de calibrer » : elle dit si le modèle est bon, avant
d'investir dans la calibration (qui, elle, sert à la courbe des angles en degrés).

Usage (venv_dream) :
    python keypoint_accuracy_curve.py \\
        --weights checkpoints_dream/vgg_ultimate_v4_mix_ft_e30/best_network.pth \\
        --datasets dream_data/synthetic_50k_ndds:synthetique \\
                   dream_data/real_3cam_ndds:real_3cam \\
        --sample 400 --out dream_data/keypoint_accuracy_curve.png
"""
import argparse
import json
import sys
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
from PIL import Image

sys.path.insert(0, "/tmp/DREAM")
import dream  # noqa: E402

KP = ["base", "link1", "link2", "link3", "link4", "link5", "link6"]


def gt_keypoints(json_path):
    kps = json.loads(Path(json_path).read_text())["objects"][0]["keypoints"]
    return np.array([k["projected_location"] for k in kps], dtype=float)


def eval_dataset(net, ds, sample, offset=0, every=1):
    jsons = sorted(Path(ds).glob("*.json"))[offset::every]   # sélection caméra (mod-N)
    if len(jsons) > sample:
        jsons = jsons[:: len(jsons) // sample][:sample]
    errs = [[] for _ in range(7)]        # erreurs px des keypoints détectés
    det = np.zeros(7, int); n = 0
    for jf in jsons:
        img = jf.with_suffix(".rgb.png")
        if not img.exists():
            continue
        gt = gt_keypoints(jf)
        pred = np.array(net.keypoints_from_image(Image.open(img).convert("RGB"))["detected_keypoints"], float)
        for k in range(7):
            if pred[k, 0] > -900 and pred[k, 1] > -900:
                det[k] += 1
                errs[k].append(float(np.linalg.norm(pred[k] - gt[k])))
        n += 1
        print(f"\r  {Path(ds).name}: {n}/{len(jsons)}", end="", flush=True)
    print()
    return errs, det, n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--weights", required=True)
    ap.add_argument("--datasets", nargs="+", required=True, help="chemin:label")
    ap.add_argument("--sample", type=int, default=400)
    ap.add_argument("--out", default="dream_data/keypoint_accuracy_curve.png")
    args = ap.parse_args()

    net = dream.create_network_from_config_file(
        str(Path(args.weights).with_suffix(".yaml")), args.weights)
    net.enable_evaluation()

    thresholds = np.linspace(0, 30, 61)     # 0..30 px
    results = {}
    for spec in args.datasets:
        parts = spec.split(":")
        ds = parts[0]
        label = parts[1] if len(parts) > 1 else Path(ds).name
        offset = int(parts[2]) if len(parts) > 2 else 0
        every = int(parts[3]) if len(parts) > 3 else 1
        errs, det, n = eval_dataset(net, ds, args.sample, offset, every)
        alle = np.concatenate([np.array(e) for e in errs if e]) if any(errs) else np.array([])
        pck = [100.0 * (alle < t).mean() if alle.size else 0.0 for t in thresholds]
        results[label] = {"errs": errs, "det": det, "n": n, "pck": pck}
        print(f"\n== {label} ({n} images) ==")
        print(f"{'keypoint':8s} {'détect.':>8s} {'err.méd(px)':>12s}")
        for k in range(7):
            e = np.median(errs[k]) if errs[k] else float("nan")
            print(f"{KP[k]:8s} {100*det[k]/n:6.0f}% {e:12.1f}")
        print(f"{'GLOBAL':8s} {100*det.sum()/(7*n):6.0f}% {np.median(alle) if alle.size else float('nan'):12.1f}")

    # --- figure : PCK (gauche) + erreur par keypoint (droite) ---
    fig, (axl, axr) = plt.subplots(1, 2, figsize=(15, 6))
    for label, r in results.items():
        axl.plot(thresholds, r["pck"], lw=2, label=f"{label} (n={r['n']})")
    axl.set_title("Courbe PCK — précision des keypoints DREAM (sans calibration)")
    axl.set_xlabel("seuil (px)"); axl.set_ylabel("% keypoints corrects")
    axl.axvline(5, color="gray", ls=":", lw=1); axl.text(5.3, 5, "5px≈7mm", fontsize=8, color="gray")
    axl.grid(alpha=0.3); axl.legend(); axl.set_ylim(0, 100)

    x = np.arange(7); w = 0.8 / max(1, len(results))
    for i, (label, r) in enumerate(results.items()):
        med = [np.median(r["errs"][k]) if r["errs"][k] else 0 for k in range(7)]
        axr.bar(x + i * w, med, w, label=label)
    axr.set_title("Erreur médiane par keypoint (px)")
    axr.set_xticks(x + w * (len(results) - 1) / 2); axr.set_xticklabels(KP, rotation=30)
    axr.set_ylabel("erreur médiane (px)"); axr.grid(alpha=0.3, axis="y"); axr.legend()

    fig.tight_layout()
    fig.savefig(args.out, dpi=120)
    print(f"\n✅ {args.out}")


if __name__ == "__main__":
    main()
