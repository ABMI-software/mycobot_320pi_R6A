#!/usr/bin/env python3
"""Train DREAM keypoint detector for MyCobot 320 Pi.

This script wraps DREAM's train_network.py with our project-specific defaults.

Usage (quick start):
  # Train ResNet on synthetic data (recommended first step)
  python train_dream.py --data /tmp/dream_data/synthetic --arch resnet

  # Train VGG on synthetic data
  python train_dream.py --data /tmp/dream_data/synthetic --arch vgg

  # Resume training
  python train_dream.py --data /tmp/dream_data/synthetic --arch resnet --resume

  # Custom parameters
  python train_dream.py --data /tmp/dream_data/synthetic --arch resnet \\
      --epochs 50 --batch-size 64 --lr 0.0001
"""

import argparse
import os
import sys
import subprocess


SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)

# DREAM paths
DREAM_DIR = "/tmp/DREAM"
TRAIN_SCRIPT = os.path.join(DREAM_DIR, "scripts", "train_network.py")

# Our config paths
MANIP_CONFIG = os.path.join(SCRIPT_DIR, "manip_configs", "mycobot320.yaml")

# Architecture configs
ARCH_CONFIGS = {
    "resnet": os.path.join(DREAM_DIR, "arch_configs", "dream_resnet_h.yaml"),
    "vgg": os.path.join(DREAM_DIR, "arch_configs", "dream_vgg_q.yaml"),
}

# Default output dirs
OUTPUT_BASE = os.path.join(SCRIPT_DIR, "checkpoints_dream")


def main():
    parser = argparse.ArgumentParser(
        description="Train DREAM keypoint detector for MyCobot 320 Pi",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--data", "-d", required=True,
        help="Path to NDDS-format training data directory",
    )
    parser.add_argument(
        "--arch", "-a", choices=["resnet", "vgg"], default="resnet",
        help="Architecture: resnet (ResNet + hourglass) or vgg",
    )
    parser.add_argument(
        "--epochs", "-e", type=int, default=25,
        help="Number of training epochs",
    )
    parser.add_argument(
        "--batch-size", "-b", type=int, default=128,
        help="Batch size",
    )
    parser.add_argument(
        "--lr", type=float, default=0.00015,
        help="Learning rate",
    )
    parser.add_argument(
        "--train-frac", "-t", type=float, default=0.8,
        help="Fraction of data used for training (rest is validation)",
    )
    parser.add_argument(
        "--workers", "-w", type=int, default=8,
        help="Number of data loading workers",
    )
    parser.add_argument(
        "--output", "-o", type=str, default=None,
        help="Output directory (default: auto-generated)",
    )
    parser.add_argument(
        "--resume", action="store_true",
        help="Resume training from latest checkpoint",
    )
    parser.add_argument(
        "--gpu", type=str, default="0",
        help="GPU ID(s) to use",
    )
    args = parser.parse_args()

    # Validate paths
    assert os.path.isdir(args.data), f"Data directory not found: {args.data}"
    assert os.path.isfile(TRAIN_SCRIPT), (
        f"DREAM train script not found: {TRAIN_SCRIPT}\n"
        f"Please clone DREAM: git clone https://github.com/NVlabs/DREAM.git /tmp/DREAM"
    )
    assert os.path.isfile(MANIP_CONFIG), f"Manipulator config not found: {MANIP_CONFIG}"

    arch_config = ARCH_CONFIGS[args.arch]
    assert os.path.isfile(arch_config), f"Architecture config not found: {arch_config}"

    # Auto output dir
    if args.output is None:
        data_name = os.path.basename(args.data.rstrip("/"))
        args.output = os.path.join(
            OUTPUT_BASE, f"{args.arch}_{data_name}_e{args.epochs}"
        )

    os.makedirs(args.output, exist_ok=True)

    # Build command
    cmd = [
        sys.executable,
        TRAIN_SCRIPT,
        "-i", args.data,
        "-t", str(args.train_frac),
        "-m", MANIP_CONFIG,
        "-ar", arch_config,
        "-e", str(args.epochs),
        "-lr", str(args.lr),
        "-b", str(args.batch_size),
        "-w", str(args.workers),
        "-o", args.output,
        "-g", args.gpu,
        "--force-overwrite",
    ]

    if args.resume:
        cmd.append("--resume-training")

    # Print summary
    print("=" * 70)
    print("  DREAM Training — MyCobot 320 Pi")
    print("=" * 70)
    print(f"  Data:         {args.data}")
    print(f"  Architecture: {args.arch}")
    print(f"  Config:       {arch_config}")
    print(f"  Manipulator:  {MANIP_CONFIG}")
    print(f"  Output:       {args.output}")
    print(f"  Epochs:       {args.epochs}")
    print(f"  Batch size:   {args.batch_size}")
    print(f"  Learning rate:{args.lr}")
    print(f"  Train frac:   {args.train_frac}")
    print(f"  Workers:      {args.workers}")
    print(f"  GPU:          {args.gpu}")
    print(f"  Resume:       {args.resume}")
    print("=" * 70)
    print()
    print(f"Command: {' '.join(cmd)}")
    print()

    # Set CUDA device
    os.environ["CUDA_VISIBLE_DEVICES"] = args.gpu

    # Run training
    result = subprocess.run(cmd, cwd=DREAM_DIR)
    sys.exit(result.returncode)


if __name__ == "__main__":
    main()
