#!/usr/bin/env python3
"""Preprocess synthetic images v2 — plus net et plus clair.

Améliorations vs v1 :
  - GaussianBlur réduit (sigma=0.3 au lieu de 0.7)
  - Sharpening après color transfer
  - Légère augmentation du contraste (+15%)
  - Color transfer identique (mean/std RGB)

Output : dream_data/mixed_3k_10k_v2 (nouveau dossier, mixed_3k_10k non touché)

Usage:
  cd training/
  python dream/preprocess_synth_v2.py
"""

import os
import shutil
import numpy as np
from PIL import Image, ImageFilter, ImageEnhance
from scipy.ndimage import gaussian_filter
from tqdm import tqdm

SCRIPT_DIR  = os.path.dirname(os.path.abspath(__file__))
INPUT_DIR   = os.path.join(SCRIPT_DIR, "dream_data", "mixed_3k_10k")   # source
OUTPUT_DIR  = os.path.join(SCRIPT_DIR, "dream_data", "mixed_3k_10k_v2") # destination
REAL_DIR    = os.path.join(SCRIPT_DIR, "dream_data", "real")

N_REAL      = 3000
SIGMA       = 0.3    # réduit vs v1 (0.7) → moins de flou
CONTRAST    = 1.15   # +15% contraste → plus claire
SHARPEN     = True   # sharpening PIL


def compute_color_stats(image_dir, n_frames=500, offset=0):
    frames = sorted([
        f for f in os.listdir(image_dir)
        if f.endswith(".rgb.png") and not f.startswith("_")
    ])[offset:offset + n_frames]

    means, stds = [], []
    for fname in tqdm(frames, desc="  Calcul stats couleur"):
        img = np.array(Image.open(os.path.join(image_dir, fname)).convert("RGB"),
                       dtype=np.float32) / 255.0
        means.append(img.mean(axis=(0, 1)))
        stds.append(img.std(axis=(0, 1)))

    return np.mean(means, axis=0), np.mean(stds, axis=0)


def color_transfer(img_np, src_mean, src_std, tgt_mean, tgt_std):
    img = img_np.astype(np.float32) / 255.0
    img = (img - src_mean) / (src_std + 1e-6)
    img = img * tgt_std + tgt_mean
    return np.clip(img * 255.0, 0, 255).astype(np.uint8)


def main():
    print(f"\n{'='*60}")
    print(f"PREPROCESS SYNTH v2 — Plus net et plus clair")
    print(f"  Input   : {INPUT_DIR}")
    print(f"  Output  : {OUTPUT_DIR}")
    print(f"  Sigma   : {SIGMA} (v1=0.7)")
    print(f"  Contrast: {CONTRAST}x")
    print(f"  Sharpen : {SHARPEN}")
    print(f"{'='*60}\n")

    if not os.path.exists(INPUT_DIR):
        print(f"❌ Dossier manquant : {INPUT_DIR}")
        return

    if os.path.exists(OUTPUT_DIR):
        print(f"⚠️  {OUTPUT_DIR} existe déjà !")
        answer = input("Écraser ? (o/n) : ")
        if answer.lower() != "o":
            print("Annulé.")
            return
        shutil.rmtree(OUTPUT_DIR)

    os.makedirs(OUTPUT_DIR, exist_ok=True)

    # Copier settings
    for f in ["_camera_settings.json", "_object_settings.json"]:
        src = os.path.join(INPUT_DIR, f)
        if os.path.exists(src):
            shutil.copy2(src, os.path.join(OUTPUT_DIR, f))

    # Récupérer toutes les frames
    all_frames = sorted([
        f.replace(".rgb.png", "")
        for f in os.listdir(INPUT_DIR)
        if f.endswith(".rgb.png") and not f.startswith("_")
    ])

    real_frames  = [f for f in all_frames if int(f) < N_REAL]
    synth_frames = [f for f in all_frames if int(f) >= N_REAL]

    print(f"Frames : {len(real_frames)} réelles + {len(synth_frames)} synthétiques\n")

    # ---- Étape 1 : Copier frames réelles sans modification ----
    print(f"📋 Copie des {len(real_frames)} frames réelles...")
    for fname in tqdm(real_frames, desc="  Copie réel"):
        for ext in [".rgb.png", ".json"]:
            src = os.path.join(INPUT_DIR, fname + ext)
            dst = os.path.join(OUTPUT_DIR, fname + ext)
            if os.path.exists(src):
                shutil.copy2(src, dst)

    # ---- Étape 2 : Calcul stats couleur ----
    print(f"\n📊 Calcul stats couleur réelles...")
    real_mean, real_std = compute_color_stats(INPUT_DIR, n_frames=500, offset=0)
    print(f"  Real mean : R={real_mean[0]:.3f} G={real_mean[1]:.3f} B={real_mean[2]:.3f}")

    print(f"\n📊 Calcul stats couleur synthétiques...")
    synth_mean, synth_std = compute_color_stats(INPUT_DIR, n_frames=500, offset=N_REAL)
    print(f"  Synth mean: R={synth_mean[0]:.3f} G={synth_mean[1]:.3f} B={synth_mean[2]:.3f}")

    # ---- Étape 3 : Preprocessing frames synthétiques ----
    print(f"\n🔧 Preprocessing {len(synth_frames)} images synthétiques...")

    for fname in tqdm(synth_frames, desc="  Preprocessing"):
        img_path  = os.path.join(INPUT_DIR, fname + ".rgb.png")
        json_path = os.path.join(INPUT_DIR, fname + ".json")

        if not os.path.exists(img_path):
            continue

        # Charger
        img = np.array(Image.open(img_path).convert("RGB"))

        # Color transfer
        img = color_transfer(img, synth_mean, synth_std, real_mean, real_std)

        # GaussianBlur léger (réduit vs v1)
        if SIGMA > 0:
            img_blurred = np.zeros_like(img)
            for c in range(3):
                img_blurred[:, :, c] = gaussian_filter(
                    img[:, :, c].astype(np.float32), sigma=SIGMA)
            img = np.clip(img_blurred, 0, 255).astype(np.uint8)

        # Sharpening
        img_pil = Image.fromarray(img)
        if SHARPEN:
            img_pil = img_pil.filter(ImageFilter.SHARPEN)

        # Contraste
        img_pil = ImageEnhance.Contrast(img_pil).enhance(CONTRAST)

        # Sauvegarder
        img_pil.save(os.path.join(OUTPUT_DIR, fname + ".rgb.png"))

        # Copier JSON
        if os.path.exists(json_path):
            shutil.copy2(json_path, os.path.join(OUTPUT_DIR, fname + ".json"))

    print(f"\n{'='*60}")
    print(f"✅ TERMINÉ !")
    print(f"  Output : {OUTPUT_DIR}")
    print(f"{'='*60}")
    print(f"\nProchaine étape :")
    print(f"  python dream/train_dream_mixed_aug_optimized_v6.py \\")
    print(f"    --data dream/dream_data/mixed_3k_10k_v2 \\")
    print(f"    --epochs 5 --batch-size 8 --workers 8 \\")
    print(f"    --lr 0.0001 --patience 5 \\")
    print(f"    --aug-brightness 0.4 --aug-contrast 0.4 \\")
    print(f"    --aug-hue 10 --aug-sat 20 --aug-noise 30 \\")
    print(f"    --output checkpoints_dream/vgg_3k10k_v2_noise30")


if __name__ == "__main__":
    main()