#!/usr/bin/env python3
"""Preprocess synthetic images in mixed_3k_10k to reduce sim-to-real gap.

Étape 1 du pipeline v2 :
  - GaussianBlur léger sur les images synthétiques (sigma=0.5-1.0)
  - Color transfer : applique les stats de couleur du réel sur le synth
  - Image standardization : normalise avec les stats réelles

Seules les images synthétiques (frames 003000 → 012999) sont modifiées.
Les images réelles (frames 000000 → 002999) ne sont pas touchées.

Usage:
  cd training/
  python dream/preprocess_synth.py
"""

import os
import json
import numpy as np
from PIL import Image
from scipy.ndimage import gaussian_filter
from tqdm import tqdm

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
MIXED_DIR  = os.path.join(SCRIPT_DIR, "dream_data", "mixed_3k_10k")
REAL_DIR   = os.path.join(SCRIPT_DIR, "dream_data", "real")

N_REAL = 3000   # les 3K premières frames sont réelles
SIGMA  = 0.7    # GaussianBlur sigma (0.5 = léger, 1.0 = plus fort)


def compute_color_stats(image_dir, n_frames=500, offset=0):
    """Calcule mean et std des canaux RGB sur un échantillon de frames."""
    frames = sorted([
        f for f in os.listdir(image_dir)
        if f.endswith(".rgb.png") and not f.startswith("_")
    ])[offset:offset + n_frames]

    means = []
    stds  = []

    for fname in tqdm(frames, desc="  Calcul stats couleur"):
        img = np.array(Image.open(os.path.join(image_dir, fname)).convert("RGB"), dtype=np.float32) / 255.0
        means.append(img.mean(axis=(0, 1)))
        stds.append(img.std(axis=(0, 1)))

    mean = np.mean(means, axis=0)
    std  = np.mean(stds,  axis=0)
    return mean, std


def color_transfer(img_np, src_mean, src_std, tgt_mean, tgt_std):
    """Transfère le style de couleur de src vers tgt."""
    img = img_np.astype(np.float32) / 255.0
    # Normaliser avec stats source puis remettre avec stats cible
    img = (img - src_mean) / (src_std + 1e-6)
    img = img * tgt_std + tgt_mean
    img = np.clip(img * 255.0, 0, 255).astype(np.uint8)
    return img


def main():
    print(f"\n{'='*60}")
    print(f"PREPROCESS SYNTH — Réduction sim-to-real gap")
    print(f"  Mixed dir : {MIXED_DIR}")
    print(f"  Real dir  : {REAL_DIR}")
    print(f"  Sigma     : {SIGMA}")
    print(f"{'='*60}\n")

    if not os.path.exists(MIXED_DIR):
        print(f"❌ Dossier manquant : {MIXED_DIR}")
        print(f"   Lance d'abord : python dream/create_mixed_dataset.py")
        return

    # Récupérer les frames synthétiques (offset N_REAL)
    all_frames = sorted([
        f.replace(".rgb.png", "")
        for f in os.listdir(MIXED_DIR)
        if f.endswith(".rgb.png") and not f.startswith("_")
    ])

    synth_frames = [f for f in all_frames if int(f) >= N_REAL]
    real_frames  = [f for f in all_frames if int(f) < N_REAL]

    print(f"Frames dans mixed_3k_10k :")
    print(f"  Réelles     : {len(real_frames)} (non modifiées)")
    print(f"  Synthétiques: {len(synth_frames)} (à preprocesser)")

    # ---- Étape 1 : Calcul des stats de couleur ----
    print(f"\n📊 Calcul des stats de couleur réelles (500 frames)...")
    real_mean, real_std = compute_color_stats(MIXED_DIR, n_frames=500, offset=0)
    print(f"  Real mean : R={real_mean[0]:.3f} G={real_mean[1]:.3f} B={real_mean[2]:.3f}")
    print(f"  Real std  : R={real_std[0]:.3f} G={real_std[1]:.3f} B={real_std[2]:.3f}")

    print(f"\n📊 Calcul des stats de couleur synthétiques (500 frames)...")
    synth_mean, synth_std = compute_color_stats(MIXED_DIR, n_frames=500, offset=N_REAL)
    print(f"  Synth mean: R={synth_mean[0]:.3f} G={synth_mean[1]:.3f} B={synth_mean[2]:.3f}")
    print(f"  Synth std : R={synth_std[0]:.3f} G={synth_std[1]:.3f} B={synth_std[2]:.3f}")

    # ---- Étape 2 : Preprocessing des images synthétiques ----
    print(f"\n🔧 Preprocessing {len(synth_frames)} images synthétiques...")
    print(f"  → GaussianBlur (sigma={SIGMA})")
    print(f"  → Color transfer synth → réel")

    modified = 0
    for fname in tqdm(synth_frames, desc="  Preprocessing"):
        img_path = os.path.join(MIXED_DIR, fname + ".rgb.png")

        if not os.path.exists(img_path):
            continue

        # Charger image
        img = np.array(Image.open(img_path).convert("RGB"))

        # Color transfer
        img = color_transfer(img, synth_mean, synth_std, real_mean, real_std)

        # GaussianBlur léger
        img_blurred = np.zeros_like(img)
        for c in range(3):
            img_blurred[:, :, c] = gaussian_filter(img[:, :, c].astype(np.float32), sigma=SIGMA)
        img = np.clip(img_blurred, 0, 255).astype(np.uint8)

        # Sauvegarder (écrase l'original)
        Image.fromarray(img).save(img_path)
        modified += 1

    print(f"\n{'='*60}")
    print(f"✅ TERMINÉ !")
    print(f"  {modified} images synthétiques preprocessées")
    print(f"  {len(real_frames)} images réelles non modifiées")
    print(f"{'='*60}")
    print(f"\nProchaine étape :")
    print(f"  python dream/train_dream_mixed_aug_optimized_v2.py \\")
    print(f"    --data dream/dream_data/mixed_3k_10k \\")
    print(f"    --epochs 50 --batch-size 8 --workers 8")


if __name__ == "__main__":
    main()