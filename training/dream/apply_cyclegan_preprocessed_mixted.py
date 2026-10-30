#!/usr/bin/env python3
"""Applique CycleGAN G_S2R sur les images synthétiques DÉJÀ preprocessées de mixed_3k_10k.

Pipeline :
  synth brut → preprocess_synth (color transfer + blur) → CycleGAN → mixed_3k_10k_cyclegan

Les images réelles (frames 000000 → 002999) sont copiées telles quelles.
Les images synthétiques (frames 003000 → 012999) sont traduites par CycleGAN.
Les fichiers JSON (annotations keypoints) sont copiés sans modification.

Usage:
  cd training/
  python dream/apply_cyclegan_on_mixed.py \
      --weights dream/cyclegan_checkpoints_v3/G_S2R_epoch2.pth \
      --input   dream/dream_data/mixed_3k_10k \
      --output  dream/dream_data/mixed_3k_10k_cyclegan \
      --n-real  3000
"""

import os
import sys
import json
import shutil
import argparse

import numpy as np
from PIL import Image
import torch
import torch.nn as nn
import torchvision.transforms as transforms
from tqdm import tqdm

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


# ============================================================
# Générateur CycleGAN (même architecture que train_cyclegan.py)
# ============================================================

class ResidualBlock(nn.Module):
    def __init__(self, channels):
        super().__init__()
        self.block = nn.Sequential(
            nn.ReflectionPad2d(1),
            nn.Conv2d(channels, channels, 3),
            nn.InstanceNorm2d(channels),
            nn.ReLU(inplace=True),
            nn.ReflectionPad2d(1),
            nn.Conv2d(channels, channels, 3),
            nn.InstanceNorm2d(channels),
        )

    def forward(self, x):
        return x + self.block(x)


class Generator(nn.Module):
    def __init__(self, n_residual=9):
        super().__init__()
        model = [
            nn.ReflectionPad2d(3),
            nn.Conv2d(3, 64, 7),
            nn.InstanceNorm2d(64),
            nn.ReLU(inplace=True),
            nn.Conv2d(64, 128, 3, stride=2, padding=1),
            nn.InstanceNorm2d(128),
            nn.ReLU(inplace=True),
            nn.Conv2d(128, 256, 3, stride=2, padding=1),
            nn.InstanceNorm2d(256),
            nn.ReLU(inplace=True),
        ]
        for _ in range(n_residual):
            model.append(ResidualBlock(256))
        model += [
            nn.ConvTranspose2d(256, 128, 3, stride=2, padding=1, output_padding=1),
            nn.InstanceNorm2d(128),
            nn.ReLU(inplace=True),
            nn.ConvTranspose2d(128, 64, 3, stride=2, padding=1, output_padding=1),
            nn.InstanceNorm2d(64),
            nn.ReLU(inplace=True),
            nn.ReflectionPad2d(3),
            nn.Conv2d(64, 3, 7),
            nn.Tanh(),
        ]
        self.model = nn.Sequential(*model)

    def forward(self, x):
        return self.model(x)


def main():
    parser = argparse.ArgumentParser(
        description="Applique CycleGAN sur images synth preprocessées de mixed_3k_10k"
    )
    parser.add_argument("--weights",  "-w", required=True, help="G_S2R weights (.pth)")
    parser.add_argument("--input",    "-i", required=True, help="Dossier mixed_3k_10k (source)")
    parser.add_argument("--output",   "-o", required=True, help="Dossier output (ex: mixed_3k_10k_cyclegan)")
    parser.add_argument("--n-real",   type=int, default=3000,
                        help="Nombre de frames réelles (non modifiées) (default: 3000)")
    parser.add_argument("--img-size", type=int, default=640,
                        help="Taille image pour CycleGAN (default: 256)")
    args = parser.parse_args()

    print(f"\n{'='*60}")
    print(f"APPLY CYCLEGAN ON MIXED_3K_10K")
    print(f"  Weights  : {args.weights}")
    print(f"  Input    : {args.input}")
    print(f"  Output   : {args.output}")
    print(f"  N real   : {args.n_real} (copiées sans modification)")
    print(f"  Img size : {args.img_size}")
    print(f"{'='*60}\n")

    # Vérifications
    if not os.path.exists(args.input):
        print(f"❌ Dossier manquant : {args.input}")
        sys.exit(1)
    if not os.path.exists(args.weights):
        print(f"❌ Weights manquants : {args.weights}")
        sys.exit(1)

    # Créer dossier output
    os.makedirs(args.output, exist_ok=True)

    # Copier les fichiers settings
    for f in ["_camera_settings.json", "_object_settings.json"]:
        src = os.path.join(args.input, f)
        if os.path.exists(src):
            shutil.copy2(src, os.path.join(args.output, f))
            print(f"  ✅ Copié : {f}")

    # Récupérer toutes les frames
    all_frames = sorted([
        f.replace(".rgb.png", "")
        for f in os.listdir(args.input)
        if f.endswith(".rgb.png") and not f.startswith("_")
    ])

    real_frames  = [f for f in all_frames if int(f) < args.n_real]
    synth_frames = [f for f in all_frames if int(f) >= args.n_real]

    print(f"Frames détectées :")
    print(f"  Réelles     : {len(real_frames)} → copie directe")
    print(f"  Synthétiques: {len(synth_frames)} → CycleGAN\n")

    # ---- Étape 1 : Copier les frames réelles ----
    print(f"📋 Copie des {len(real_frames)} frames réelles...")
    for fname in tqdm(real_frames, desc="  Copie réel"):
        for ext in [".rgb.png", ".json"]:
            src = os.path.join(args.input, fname + ext)
            dst = os.path.join(args.output, fname + ext)
            if os.path.exists(src):
                shutil.copy2(src, dst)

    # ---- Étape 2 : Charger CycleGAN ----
    print(f"\n🔧 Chargement du générateur G_S2R...")
    G_S2R = Generator().cuda()
    G_S2R.load_state_dict(torch.load(args.weights, map_location="cuda"))
    G_S2R.eval()
    print(f"  ✅ Générateur chargé")

    transform_in = transforms.Compose([
        transforms.Resize((args.img_size, args.img_size)),
        transforms.ToTensor(),
        transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5)),
    ])

    # ---- Étape 3 : Appliquer CycleGAN sur frames synthétiques ----
    print(f"\n🔄 Application CycleGAN sur {len(synth_frames)} frames synthétiques...")
    n_ok = 0
    n_fail = 0

    with torch.no_grad():
        for fname in tqdm(synth_frames, desc="  CycleGAN synth"):
            img_path  = os.path.join(args.input, fname + ".rgb.png")
            json_path = os.path.join(args.input, fname + ".json")

            if not os.path.exists(img_path):
                n_fail += 1
                continue

            # Charger image preprocessée (color transfer déjà appliqué)
            img_orig = Image.open(img_path).convert("RGB")
            orig_w, orig_h = img_orig.size

            # Appliquer CycleGAN
            img_tensor = transform_in(img_orig).unsqueeze(0).cuda()
            fake_real  = G_S2R(img_tensor)

            # Dénormaliser et redimensionner à la résolution originale
            img_np = fake_real[0].cpu().numpy().transpose(1, 2, 0)
            img_np = ((img_np * 0.5 + 0.5) * 255).clip(0, 255).astype(np.uint8)
            img_translated = Image.fromarray(img_np).resize((orig_w, orig_h), Image.BILINEAR)

            # Sauvegarder image traduite
            img_translated.save(os.path.join(args.output, fname + ".rgb.png"))

            # Copier JSON (annotations GT inchangées)
            if os.path.exists(json_path):
                shutil.copy2(json_path, os.path.join(args.output, fname + ".json"))

            n_ok += 1

    print(f"\n{'='*60}")
    print(f"✅ TERMINÉ !")
    print(f"  Frames réelles copiées  : {len(real_frames)}")
    print(f"  Frames synth traduites  : {n_ok}")
    print(f"  Échecs                  : {n_fail}")
    print(f"  Output                  : {args.output}")
    print(f"{'='*60}")
    print(f"\nProchaine étape — entraîner DREAM :")
    print(f"  python dream/train_dream_mixed_aug_optimized_v6.py \\")
    print(f"    --data {args.output} \\")
    print(f"    --epochs 5 --batch-size 8 --workers 8 \\")
    print(f"    --lr 0.0001 --patience 5 \\")
    print(f"    --aug-brightness 0.4 --aug-contrast 0.4 \\")
    print(f"    --aug-hue 10 --aug-sat 20 --aug-noise 30 \\")
    print(f"    --output checkpoints_dream/vgg_3k10k_cyclegan_noise30")


if __name__ == "__main__":
    main()