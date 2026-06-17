#!/usr/bin/env python3
"""Enhanced DREAM training with aggressive augmentation for sim-to-real transfer.

Patches DREAM's albumentations pipeline with stronger transforms to
close the visual domain gap between Gazebo synthetic data and real camera images.

Usage:
  python train_dream_augmented.py \
      --data /tmp/dream_data/synthetic \
      --arch vgg \
      --epochs 25 \
      --batch-size 32 \
      --lr 0.0001 \
      --output checkpoints_dream/vgg_augmented_e25
"""

import sys
import os

# Monkey-patch DREAM's dataset to use aggressive augmentation
# This must be done BEFORE importing dream
import albumentations as albu

# Save original __getitem__ to patch
_original_getitem = None


def _patch_augmentation():
    """Patch ManipulatorNDDSDataset to use stronger augmentation."""
    sys.path.insert(0, "/tmp/DREAM")
    import dream.datasets as dream_datasets
    from PIL import Image as PILImage
    import numpy as np

    # Store original
    original_getitem = dream_datasets.ManipulatorNDDSDataset.__getitem__

    def patched_getitem(self, idx):
        """Override __getitem__ to use stronger augmentation."""
        # Temporarily override augment_data behavior
        if self.augment_data:
            # Save original augment_data, set False to skip DREAM's weak aug
            self.augment_data = False
            sample = original_getitem(self, idx)
            self.augment_data = True

            # Now apply our stronger augmentation on the network input image
            # We need to work with image_rgb_input tensor
            # Convert back to numpy for albumetations
            img_tensor = sample["image_rgb_input"]
            # Undo normalization: img = tensor * stdev + mean
            # DREAM normalizes with mean=0.5, stdev=0.5, so: img = tensor * 0.5 + 0.5
            img_np = img_tensor.permute(1, 2, 0).numpy() * 0.5 + 0.5
            img_np = (img_np * 255).clip(0, 255).astype(np.uint8)

            # Strong augmentation pipeline
            strong_aug = albu.Compose([
                albu.GaussNoise(p=0.5),
                albu.RandomBrightnessContrast(
                    brightness_limit=0.3,
                    contrast_limit=0.3,
                    brightness_by_max=False,
                    p=0.7,
                ),
                albu.HueSaturationValue(
                    hue_shift_limit=20,
                    sat_shift_limit=30,
                    val_shift_limit=20,
                    p=0.5,
                ),
                albu.OneOf([
                    albu.GaussianBlur(blur_limit=(3, 7)),
                    albu.MotionBlur(blur_limit=(3, 7)),
                    albu.MedianBlur(blur_limit=(3, 5)),
                ], p=0.3),
                albu.CLAHE(clip_limit=4.0, p=0.2),
                albu.RandomGamma(gamma_limit=(70, 130), p=0.3),
                albu.ImageCompression(quality_range=(50, 95), p=0.3),
                albu.CoarseDropout(
                    num_holes_range=(1, 4),
                    hole_height_range=(10, 50),
                    hole_width_range=(10, 50),
                    fill="random",
                    p=0.3,
                ),
                albu.ShiftScaleRotate(
                    shift_limit=0.05,
                    scale_limit=0.1,
                    rotate_limit=15,
                    p=0.5,
                ),
            ], p=1.0)

            augmented = strong_aug(image=img_np)
            img_aug = augmented["image"]

            # Re-normalize back to tensor format
            import torch
            img_aug_f = img_aug.astype(np.float32) / 255.0
            img_aug_f = (img_aug_f - 0.5) / 0.5
            sample["image_rgb_input"] = torch.from_numpy(
                img_aug_f.transpose(2, 0, 1)
            ).float()

            return sample
        else:
            return original_getitem(self, idx)

    dream_datasets.ManipulatorNDDSDataset.__getitem__ = patched_getitem
    print("[PATCH] Augmentation pipeline replaced with strong sim-to-real transforms")


def main():
    import argparse

    parser = argparse.ArgumentParser(
        description="Train DREAM with enhanced augmentation"
    )
    parser.add_argument("--data", "-d", required=True)
    parser.add_argument("--arch", default="vgg", choices=["vgg", "resnet"])
    parser.add_argument("--epochs", "-e", type=int, default=25)
    parser.add_argument("--batch-size", "-b", type=int, default=32)
    parser.add_argument("--lr", type=float, default=0.0001)
    parser.add_argument("--workers", "-w", type=int, default=8)
    parser.add_argument("--gpu", "-g", type=int, default=0)
    parser.add_argument("--output", "-o", default=None)
    parser.add_argument("--training-fraction", "-t", type=float, default=0.8)
    args = parser.parse_args()

    # Apply augmentation patch
    _patch_augmentation()

    # Now import and run DREAM training
    DREAM_DIR = "/tmp/DREAM"
    SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
    MANIP_CONFIG = os.path.join(SCRIPT_DIR, "manip_configs", "mycobot320.yaml")

    arch_map = {
        "vgg": os.path.join(DREAM_DIR, "arch_configs", "dream_vgg_q.yaml"),
        "resnet": os.path.join(DREAM_DIR, "arch_configs", "dream_resnet_h.yaml"),
    }
    arch_config = arch_map[args.arch]

    if args.output is None:
        args.output = os.path.join(
            SCRIPT_DIR, "checkpoints_dream",
            f"{args.arch}_augmented_e{args.epochs}"
        )

    os.makedirs(args.output, exist_ok=True)

    # Build the command to run DREAM's train_network
    train_script = os.path.join(DREAM_DIR, "scripts", "train_network.py")

    # We can't just run the script since we need the monkey-patch active
    # Instead, import and run the main function
    sys.path.insert(0, os.path.join(DREAM_DIR, "scripts"))

    # Simulate command line args for train_network
    sys.argv = [
        "train_network.py",
        "-i", args.data,
        "-t", str(args.training_fraction),
        "-m", MANIP_CONFIG,
        "-ar", arch_config,
        "-e", str(args.epochs),
        "-lr", str(args.lr),
        "-b", str(args.batch_size),
        "-w", str(args.workers),
        "-o", args.output,
        "-g", str(args.gpu),
        "--force-overwrite",
    ]

    print(f"Training {args.arch} with enhanced augmentation")
    print(f"  Data: {args.data}")
    print(f"  Output: {args.output}")
    print(f"  Epochs: {args.epochs}, BS={args.batch_size}, LR={args.lr}")

    # Import and run
    import importlib
    train_module = importlib.import_module("train_network")

    # DREAM's train_network uses argparse, so we just call it
    # The sys.argv is already set
    import runpy
    runpy.run_path(train_script, run_name="__main__")


if __name__ == "__main__":
    main()
