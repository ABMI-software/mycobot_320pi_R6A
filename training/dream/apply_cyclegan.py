#!/usr/bin/env python3
"""Applique le générateur CycleGAN G_S2R pour traduire les images synthétiques en style réel.

Usage:
  python dream/apply_cyclegan.py \
      --weights dream/cyclegan_checkpoints/G_S2R_epoch50.pth \
      --data    dream/dream_data/synthetic \
      --output  dream/dream_data/synth_real_style \
      --max-samples 10000

Après :
  python dream/train_dream_mixed_aug_optimized_v2.py \
      --data dream/dream_data/synth_real_style \
      --epochs 50 --batch-size 8 --workers 8 \
      --output checkpoints_dream/vgg_cyclegan_e50
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
    parser = argparse.ArgumentParser(description="Applique CycleGAN G_S2R sur images synthétiques")
    parser.add_argument("--weights",     "-w", required=True, help="G_S2R weights (.pth)")
    parser.add_argument("--data",        "-d", required=True, help="Dossier images synthétiques")
    parser.add_argument("--output",      "-o", required=True, help="Dossier output images traduites")
    parser.add_argument("--max-samples", "-n", type=int, default=10000,
                        help="Nombre max d'images à traduire (default: 10000)")
    parser.add_argument("--img-size",    type=int, default=640,
                        help="Taille image pour CycleGAN (default: 640)")
    args = parser.parse_args()

    print(f"\n{'='*60}")
    print(f"APPLY CYCLEGAN G_S2R")
    print(f"  Weights : {args.weights}")
    print(f"  Data    : {args.data}")
    print(f"  Output  : {args.output}")
    print(f"  Samples : {args.max_samples}")
    print(f"{'='*60}\n")

    # Charger le générateur
    print(f"Chargement du générateur G_S2R...")
    G_S2R = Generator().cuda()
    G_S2R.load_state_dict(torch.load(args.weights, map_location="cuda"))
    G_S2R.eval()
    print(f"  ✅ Générateur chargé")

    # Transforms
    transform_in = transforms.Compose([
        transforms.Resize((args.img_size, args.img_size)),
        transforms.ToTensor(),
        transforms.Normalize((0.5, 0.5, 0.5), (0.5, 0.5, 0.5)),
    ])

    # Créer dossier output
    os.makedirs(args.output, exist_ok=True)

    # Copier les fichiers settings
    for f in ["_camera_settings.json", "_object_settings.json"]:
        src = os.path.join(args.data, f)
        if os.path.exists(src):
            shutil.copy2(src, os.path.join(args.output, f))
            print(f"  ✅ Copié : {f}")

    # Trouver toutes les frames
    json_files = sorted([
        f.replace(".json", "")
        for f in os.listdir(args.data)
        if f.endswith(".json") and not f.startswith("_")
    ])[:args.max_samples]

    print(f"\nTraduction de {len(json_files)} images synth → style réel...")

    n_ok = 0
    n_fail = 0

    with torch.no_grad():
        for frame_id in tqdm(json_files, desc="Applying G_S2R"):
            img_path  = os.path.join(args.data, f"{frame_id}.rgb.png")
            json_path = os.path.join(args.data, f"{frame_id}.json")

            if not os.path.exists(img_path) or not os.path.exists(json_path):
                n_fail += 1
                continue

            # Charger image originale (pour récupérer la résolution)
            img_orig = Image.open(img_path).convert("RGB")
            orig_w, orig_h = img_orig.size

            # Traduire avec CycleGAN
            img_tensor = transform_in(img_orig).unsqueeze(0).cuda()
            fake_real  = G_S2R(img_tensor)

            # Dénormaliser et redimensionner à la résolution originale
            img_np = fake_real[0].cpu().numpy().transpose(1, 2, 0)
            img_np = ((img_np * 0.5 + 0.5) * 255).clip(0, 255).astype(np.uint8)
            img_translated = Image.fromarray(img_np).resize((orig_w, orig_h), Image.BILINEAR)

            # Sauvegarder image traduite
            out_img = os.path.join(args.output, f"{frame_id}.rgb.png")
            img_translated.save(out_img)

            # Copier le JSON (annotations GT inchangées — positions keypoints conservées)
            out_json = os.path.join(args.output, f"{frame_id}.json")
            shutil.copy2(json_path, out_json)

            n_ok += 1

    print(f"\n{'='*60}")
    print(f"✅ TERMINÉ !")
    print(f"  Images traduites : {n_ok}")
    print(f"  Échecs           : {n_fail}")
    print(f"  Output           : {args.output}")
    print(f"{'='*60}")
    print(f"\nProchaine étape — réentraîner DREAM :")
    print(f"  python dream/train_dream_mixed_aug_optimized_v2.py \\")
    print(f"    --data {args.output} \\")
    print(f"    --epochs 50 --batch-size 8 --workers 8 \\")
    print(f"    --output checkpoints_dream/vgg_cyclegan_e50")


if __name__ == "__main__":
    main()