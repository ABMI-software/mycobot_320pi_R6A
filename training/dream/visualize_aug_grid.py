#!/usr/bin/env python3
"""Visualise l'effet de chaque paramètre à différentes valeurs."""

import cv2
import numpy as np
import matplotlib.pyplot as plt
import albumentations as albu
import argparse

def apply_aug(img, transform):
    return transform(image=img.copy())["image"]

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--image", required=True)
    parser.add_argument("--output", default="/tmp/aug_grid.png")
    args = parser.parse_args()

    img = cv2.cvtColor(cv2.imread(args.image), cv2.COLOR_BGR2RGB)

    params = {
        "Brightness": [
            ("0.1", albu.RandomBrightnessContrast(brightness_limit=(0.1,0.1), contrast_limit=0, p=1)),
            ("0.2", albu.RandomBrightnessContrast(brightness_limit=(0.2,0.2), contrast_limit=0, p=1)),
            ("0.3", albu.RandomBrightnessContrast(brightness_limit=(0.3,0.3), contrast_limit=0, p=1)),
            ("0.4 ✅", albu.RandomBrightnessContrast(brightness_limit=(0.4,0.4), contrast_limit=0, p=1)),
            ("0.5", albu.RandomBrightnessContrast(brightness_limit=(0.5,0.5), contrast_limit=0, p=1)),
        ],
        "Contrast": [
            ("0.1", albu.RandomBrightnessContrast(brightness_limit=0, contrast_limit=(0.1,0.1), p=1)),
            ("0.2", albu.RandomBrightnessContrast(brightness_limit=0, contrast_limit=(0.2,0.2), p=1)),
            ("0.3", albu.RandomBrightnessContrast(brightness_limit=0, contrast_limit=(0.3,0.3), p=1)),
            ("0.4 ✅", albu.RandomBrightnessContrast(brightness_limit=0, contrast_limit=(0.4,0.4), p=1)),
            ("0.5", albu.RandomBrightnessContrast(brightness_limit=0, contrast_limit=(0.5,0.5), p=1)),
        ],
        "Hue": [
            ("hue=2", albu.HueSaturationValue(hue_shift_limit=2, sat_shift_limit=0, val_shift_limit=0, p=1)),
            ("hue=5", albu.HueSaturationValue(hue_shift_limit=5, sat_shift_limit=0, val_shift_limit=0, p=1)),
            ("hue=10 ✅", albu.HueSaturationValue(hue_shift_limit=10, sat_shift_limit=0, val_shift_limit=0, p=1)),
            ("hue=20", albu.HueSaturationValue(hue_shift_limit=20, sat_shift_limit=0, val_shift_limit=0, p=1)),
            ("hue=30", albu.HueSaturationValue(hue_shift_limit=30, sat_shift_limit=0, val_shift_limit=0, p=1)),
        ],
        "Saturation": [
            ("sat=5", albu.HueSaturationValue(hue_shift_limit=0, sat_shift_limit=5, val_shift_limit=0, p=1)),
            ("sat=10", albu.HueSaturationValue(hue_shift_limit=0, sat_shift_limit=10, val_shift_limit=0, p=1)),
            ("sat=20 ✅", albu.HueSaturationValue(hue_shift_limit=0, sat_shift_limit=20, val_shift_limit=0, p=1)),
            ("sat=30", albu.HueSaturationValue(hue_shift_limit=0, sat_shift_limit=30, val_shift_limit=0, p=1)),
            ("sat=40", albu.HueSaturationValue(hue_shift_limit=0, sat_shift_limit=40, val_shift_limit=0, p=1)),
        ],
        "Noise": [
            ("noise=10", albu.GaussNoise(std_range=(10/255, 20/255), p=1)),
            ("noise=20", albu.GaussNoise(std_range=(20/255, 40/255), p=1)),
            ("noise=30 ✅", albu.GaussNoise(std_range=(30/255, 60/255), p=1)),
            ("noise=50", albu.GaussNoise(std_range=(50/255, 100/255), p=1)),
            ("noise=80", albu.GaussNoise(std_range=(80/255, 160/255), p=1)),
        ],
    }

    n_rows = len(params)
    n_cols = 6
    fig, axes = plt.subplots(n_rows, n_cols, figsize=(n_cols * 3, n_rows * 3))
    fig.suptitle("Effet de chaque paramètre d'augmentation (vert = valeur retenue)", fontsize=13)

    for row, (param_name, configs) in enumerate(params.items()):
        axes[row, 0].imshow(img)
        axes[row, 0].set_title("Original", fontsize=8)
        axes[row, 0].set_ylabel(param_name, fontsize=10, fontweight='bold')
        axes[row, 0].axis("off")

        for col, (label, aug) in enumerate(configs):
            axes[row, col + 1].imshow(apply_aug(img, aug))
            color = "green" if "✅" in label else "black"
            axes[row, col + 1].set_title(label, fontsize=8, color=color)
            axes[row, col + 1].axis("off")

    plt.tight_layout()
    plt.savefig(args.output, dpi=120, bbox_inches="tight")
    print(f"Sauvegardé : {args.output}")

if __name__ == "__main__":
    main()