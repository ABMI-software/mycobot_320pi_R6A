#!/usr/bin/env python3
"""Verify and visualise a synthetic dataset.

Generates:
  1. A grid montage of N random sample images with joint angles overlaid
  2. Console statistics (angle distribution, file sizes, duplicates)
  3. A histogram of joint angle distributions

Usage:
  python3 scripts/verify_dataset.py /tmp/mycobot_synth_dataset
  python3 scripts/verify_dataset.py /tmp/mycobot_synth_dataset --grid 6 --out /tmp/verify_report
"""

import argparse
import csv
import os
import random
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont


def load_labels(dataset_dir: str):
    """Load labels.csv and return list of dicts."""
    csv_path = os.path.join(dataset_dir, 'labels.csv')
    if not os.path.exists(csv_path):
        print(f"❌ labels.csv not found in {dataset_dir}")
        sys.exit(1)
    rows = []
    with open(csv_path, 'r') as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def print_statistics(labels, dataset_dir):
    """Print dataset statistics."""
    img_dir = os.path.join(dataset_dir, 'images')
    n = len(labels)
    print(f"\n{'='*60}")
    print(f"📊 DATASET STATISTICS")
    print(f"{'='*60}")
    print(f"  Total samples   : {n}")

    # Image file stats
    sizes = []
    missing = 0
    for row in labels:
        img_path = os.path.join(dataset_dir, row['image_path'])
        if os.path.exists(img_path):
            sizes.append(os.path.getsize(img_path))
        else:
            missing += 1

    if sizes:
        print(f"  Image files OK  : {len(sizes)}")
        print(f"  Missing images  : {missing}")
        print(f"  Image size range: {min(sizes)/1024:.1f} KB — {max(sizes)/1024:.1f} KB")
        print(f"  Total disk usage: {sum(sizes)/1024/1024:.1f} MB")

    # Check first image dimensions
    first_img_path = os.path.join(dataset_dir, labels[0]['image_path'])
    if os.path.exists(first_img_path):
        img = Image.open(first_img_path)
        print(f"  Image dimensions: {img.size[0]}x{img.size[1]} ({img.mode})")

    # Joint angle statistics
    joint_cols_deg = ['j1_deg', 'j2_deg', 'j3_deg', 'j4_deg', 'j5_deg', 'j6_deg']
    print(f"\n  {'Joint':<8} {'Min°':>8} {'Max°':>8} {'Mean°':>8} {'Std°':>8}")
    print(f"  {'-'*40}")
    all_angles = []
    for jc in joint_cols_deg:
        vals = [float(row[jc]) for row in labels]
        all_angles.append(vals)
        print(f"  {jc:<8} {min(vals):>8.1f} {max(vals):>8.1f} {np.mean(vals):>8.1f} {np.std(vals):>8.1f}")

    # Check for duplicate angle sets (potential issue)
    angle_sets = set()
    dupes = 0
    for row in labels:
        key = tuple(round(float(row[jc]), 1) for jc in joint_cols_deg)
        if key in angle_sets:
            dupes += 1
        angle_sets.add(key)
    print(f"\n  Duplicate poses (within 0.1°): {dupes}")
    print(f"{'='*60}\n")

    return all_angles, joint_cols_deg


def create_montage(labels, dataset_dir, grid_size=4, out_path=None):
    """Create a grid montage of random samples with angles overlaid."""
    n_images = grid_size * grid_size
    indices = random.sample(range(len(labels)), min(n_images, len(labels)))

    # Load images
    images = []
    for idx in indices:
        row = labels[idx]
        img_path = os.path.join(dataset_dir, row['image_path'])
        if os.path.exists(img_path):
            img = Image.open(img_path).convert('RGB')
            # Draw joint angles on the image
            draw = ImageDraw.Draw(img)
            angles_text = f"#{row['index']}: [{row['j1_deg']}, {row['j2_deg']}, {row['j3_deg']}, {row['j4_deg']}, {row['j5_deg']}, {row['j6_deg']}]"
            # Draw text with background
            draw.rectangle([0, 0, img.width, 20], fill='black')
            try:
                font = ImageFont.truetype("/usr/share/fonts/truetype/dejavu/DejaVuSansMono.ttf", 11)
            except (IOError, OSError):
                font = ImageFont.load_default()
            draw.text((4, 4), angles_text, fill='lime', font=font)
            images.append(img)

    if not images:
        print("❌ No images found!")
        return

    # Create grid
    w, h = images[0].size
    thumb_w, thumb_h = w // 2, h // 2  # half-size thumbnails
    grid_w = grid_size * thumb_w
    grid_h = grid_size * thumb_h
    montage = Image.new('RGB', (grid_w, grid_h), color='black')

    for i, img in enumerate(images):
        row_i = i // grid_size
        col_i = i % grid_size
        thumb = img.resize((thumb_w, thumb_h), Image.LANCZOS)
        montage.paste(thumb, (col_i * thumb_w, row_i * thumb_h))

    if out_path is None:
        out_path = os.path.join(dataset_dir, 'montage.png')
    montage.save(out_path)
    print(f"🖼️  Montage saved to {out_path} ({grid_w}x{grid_h})")
    return out_path


def create_histogram(all_angles, joint_names, out_path):
    """Create joint angle distribution histograms."""
    try:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(2, 3, figsize=(14, 8))
        fig.suptitle('Joint Angle Distributions (degrees)', fontsize=14)

        for i, (angles, name) in enumerate(zip(all_angles, joint_names)):
            ax = axes[i // 3][i % 3]
            ax.hist(angles, bins=50, color='steelblue', edgecolor='white', alpha=0.8)
            ax.set_title(name)
            ax.set_xlabel('degrees')
            ax.set_ylabel('count')
            ax.axvline(np.mean(angles), color='red', linestyle='--', linewidth=1, label=f'mean={np.mean(angles):.1f}°')
            ax.legend(fontsize=8)

        plt.tight_layout()
        plt.savefig(out_path, dpi=150)
        plt.close()
        print(f"📈 Histogram saved to {out_path}")
    except ImportError:
        print("⚠️  matplotlib not available — skipping histogram")


def main():
    parser = argparse.ArgumentParser(description='Verify synthetic dataset')
    parser.add_argument('dataset_dir', help='Path to the dataset directory')
    parser.add_argument('--grid', type=int, default=4, help='Grid size for montage (NxN)')
    parser.add_argument('--out', default=None, help='Output directory for report files')
    args = parser.parse_args()

    dataset_dir = args.dataset_dir
    out_dir = args.out or dataset_dir

    if not os.path.isdir(dataset_dir):
        print(f"❌ Directory not found: {dataset_dir}")
        sys.exit(1)

    labels = load_labels(dataset_dir)
    if not labels:
        print("❌ No data rows in labels.csv")
        sys.exit(1)

    # Statistics
    all_angles, joint_names = print_statistics(labels, dataset_dir)

    # Montage
    montage_path = os.path.join(out_dir, 'montage.png')
    create_montage(labels, dataset_dir, grid_size=args.grid, out_path=montage_path)

    # Histogram
    hist_path = os.path.join(out_dir, 'angle_distributions.png')
    create_histogram(all_angles, joint_names, hist_path)

    print(f"\n✅ Verification complete! Check:")
    print(f"   {montage_path}")
    print(f"   {hist_path}")


if __name__ == '__main__':
    main()
