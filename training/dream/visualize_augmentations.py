#!/usr/bin/env python3
"""Visualise l'effet de chaque augmentation sur image réelle et synthétique."""

import cv2
import numpy as np
import matplotlib.pyplot as plt
import albumentations as albu
import argparse

def apply_aug(img, transform):
    return transform(image=img)["image"]

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--real",   required=True)
    parser.add_argument("--synth",  required=True)
    parser.add_argument("--output", default="/tmp/augmentation_effects.png")
    args = parser.parse_args()

    real  = cv2.cvtColor(cv2.imread(args.real),  cv2.COLOR_BGR2RGB)
    synth = cv2.cvtColor(cv2.imread(args.synth), cv2.COLOR_BGR2RGB)

    configs = {
        "Original"   : albu.NoOp(),
        "Brightness" : albu.RandomBrightnessContrast(brightness_limit=0.4, contrast_limit=0.4, p=1),
        "Hue/Sat"    : albu.HueSaturationValue(hue_shift_limit=10, sat_shift_limit=20, p=1),
        "Noise=20"   : albu.GaussNoise(std_range=(20/255, 40/255), p=1),
        "Noise=30"   : albu.GaussNoise(std_range=(30/255, 60/255), p=1),
        "Tout"       : albu.Compose([
            albu.RandomBrightnessContrast(brightness_limit=0.4, contrast_limit=0.4, p=1),
            albu.HueSaturationValue(hue_shift_limit=10, sat_shift_limit=20, p=1),
            albu.GaussNoise(std_range=(30/255, 60/255), p=1),
        ]),
    }

    n_cols = len(configs)
    fig, axes = plt.subplots(2, n_cols, figsize=(n_cols * 3, 7))
    fig.suptitle("Effet des augmentations — Réel (haut) vs Synthétique (bas)", fontsize=13)

    for col, (name, aug) in enumerate(configs.items()):
        axes[0, col].imshow(apply_aug(real,  aug))
        axes[0, col].set_title(name, fontsize=9)
        axes[0, col].axis("off")

        axes[1, col].imshow(apply_aug(synth, aug))
        axes[1, col].axis("off")

    axes[0, 0].set_ylabel("Réel", fontsize=11)
    axes[1, 0].set_ylabel("Synthétique", fontsize=11)

    plt.tight_layout()
    plt.savefig(args.output, dpi=150, bbox_inches="tight")
    print(f"Sauvegardé : {args.output}")

if __name__ == "__main__":
    main()