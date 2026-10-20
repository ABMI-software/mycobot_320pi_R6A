#!/usr/bin/env python3
"""Evaluate DREAM model avec Test-Time Augmentation (TTA).

Principe : pour chaque image, applique N augmentations photométriques,
détecte les keypoints pour chaque version via keypoints_from_image(),
puis moyenne les positions détectées (ignore les non-détectés).

Un keypoint est considéré détecté si au moins 1 augmentation le détecte.
La position finale est la moyenne des positions détectées.

Augmentations utilisées :
  1. Original (sans aug)
  2. Brightness +15%
  3. Brightness -15%
  4. Contrast +15%
  5. Contrast -15%
  6. Bruit gaussien léger

Usage:
  python dream/evaluate_dream_tta.py \\
      --weights checkpoints_dream/grid_bhs/br04_h05_s10_n30/best_network.pth \\
      --data dream/dream_data/real_test \\
      --split all \\
      --max-samples 1000 \\
      --n-aug 6
"""

import argparse
import json
import math
import os
import sys
import glob

import numpy as np
from PIL import Image as PILImage, ImageEnhance
import torch
from tqdm import tqdm

sys.path.insert(0, "/tmp/DREAM")
import dream

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
from mycobot_fk import KEYPOINT_NAMES

TRAIN_FRAC = 0.80
VAL_FRAC   = 0.10


def load_camera_intrinsics(data_dir):
    path = os.path.join(data_dir, "_camera_settings.json")
    with open(path) as f:
        settings = json.load(f)
    cs   = settings["camera_settings"][0]
    intr = cs["intrinsic_settings"]
    K    = np.array([
        [intr["fx"], intr.get("s", 0), intr["cx"]],
        [0, intr["fy"], intr["cy"]],
        [0, 0, 1],
    ], dtype=np.float64)
    w = cs["captured_image_size"]["width"]
    h = cs["captured_image_size"]["height"]
    return K, w, h


def load_gt_keypoints(json_path):
    with open(json_path) as f:
        data = json.load(f)
    projections = []
    for kp in data["objects"][0]["keypoints"]:
        projections.append(kp["projected_location"])
    return projections


def apply_tta_augmentations(image_pil, n_aug=6):
    """Génère N versions augmentées d'une image PIL."""
    augmented = [image_pil]  # 1. Original
    if n_aug >= 2:
        augmented.append(ImageEnhance.Brightness(image_pil).enhance(1.15))
    if n_aug >= 3:
        augmented.append(ImageEnhance.Brightness(image_pil).enhance(0.85))
    if n_aug >= 4:
        augmented.append(ImageEnhance.Contrast(image_pil).enhance(1.15))
    if n_aug >= 5:
        augmented.append(ImageEnhance.Contrast(image_pil).enhance(0.85))
    if n_aug >= 6:
        img_np = np.array(image_pil).astype(np.float32)
        noise  = np.random.normal(0, 8, img_np.shape).astype(np.float32)
        img_noisy = np.clip(img_np + noise, 0, 255).astype(np.uint8)
        augmented.append(PILImage.fromarray(img_noisy))
    return augmented[:n_aug]


def is_valid_kp(kp):
    """Vérifie si un keypoint est détecté (pas None, pas -999)."""
    if kp is None:
        return False
    if hasattr(kp, '__len__') and len(kp) >= 2:
        if kp[0] is None or kp[1] is None:
            return False
        if float(kp[0]) < -900 or float(kp[1]) < -900:
            return False
    return True


def evaluate_tta(args):
    config_path = os.path.splitext(args.weights)[0] + ".yaml"
    print(f"Loading model: {args.weights}")
    dream_net = dream.create_network_from_config_file(config_path, args.weights)
    dream_net.enable_evaluation()

    print(f"  Architecture: {dream_net.architecture_type}")
    print(f"  Keypoints: {dream_net.n_keypoints}")
    print(f"  TTA: {args.n_aug} augmentations (moyenne des positions)")

    K, img_w, img_h = load_camera_intrinsics(args.data)
    print(f"  Camera: fx={K[0,0]:.1f} fy={K[1,1]:.1f} cx={K[0,2]:.1f} cy={K[1,2]:.1f}  {img_w}x{img_h}")

    json_files = sorted(glob.glob(os.path.join(args.data, "??????.json")))
    n_total    = len(json_files)
    print(f"\nFound {n_total} frames in {args.data}")

    n_train = int(n_total * TRAIN_FRAC)
    n_val   = int(n_total * VAL_FRAC)

    if args.split == "val":
        json_files = json_files[n_train:n_train + n_val]
        print(f"Using val split   : {len(json_files)} frames")
    elif args.split == "test":
        json_files = json_files[n_train + n_val:]
        print(f"Using test split  : {len(json_files)} frames")
    elif args.split == "train":
        json_files = json_files[:n_train]
        print(f"Using train split : {len(json_files)} frames")
    else:
        print(f"Using all frames  : {len(json_files)} frames")

    if args.max_samples and len(json_files) > args.max_samples:
        step    = len(json_files) / args.max_samples
        indices = [int(i * step) for i in range(args.max_samples)]
        json_files = [json_files[i] for i in indices]
        print(f"Subsampled to {len(json_files)} frames")

    np.random.seed(42)

    all_errors   = []
    all_detected = []
    n_frames     = len(json_files)
    n_kp         = len(KEYPOINT_NAMES)

    for jf in tqdm(json_files, desc="TTA Evaluation"):
        frame_id = os.path.basename(jf).replace(".json", "")
        img_path = os.path.join(args.data, f"{frame_id}.rgb.png")

        if not os.path.exists(img_path):
            continue

        gt_proj = load_gt_keypoints(jf)
        image   = PILImage.open(img_path).convert("RGB")

        # ---- TTA : collecter les positions détectées pour chaque aug ----
        augmented_images = apply_tta_augmentations(image, args.n_aug)

        # kp_positions[ki] = liste des (u, v) détectés pour ce keypoint
        kp_positions = [[] for _ in range(n_kp)]

        with torch.no_grad():
            for aug_img in augmented_images:
                try:
                    result = dream_net.keypoints_from_image(aug_img, debug=False)
                    detected = result["detected_keypoints"]
                    for ki in range(n_kp):
                        kp = detected[ki] if ki < len(detected) else None
                        if is_valid_kp(kp):
                            kp_positions[ki].append((float(kp[0]), float(kp[1])))
                except Exception:
                    continue

        # ---- Moyenne des positions détectées ----
        frame_errors = []
        n_det = 0

        for ki in range(n_kp):
            gt = gt_proj[ki] if ki < len(gt_proj) else None

            if len(kp_positions[ki]) == 0 or gt is None:
                frame_errors.append(float('nan'))
                continue

            # Moyenne des positions détectées
            u_avg = np.mean([p[0] for p in kp_positions[ki]])
            v_avg = np.mean([p[1] for p in kp_positions[ki]])

            n_det += 1
            du  = u_avg - float(gt[0])
            dv  = v_avg - float(gt[1])
            err = math.sqrt(du * du + dv * dv)
            frame_errors.append(err)

        all_errors.append(frame_errors)
        all_detected.append(n_det)

    # ---- Résultats ----
    print(f"\n{'='*60}")
    print(f"EVALUATION TTA RESULTS ({n_frames} frames) — split={args.split} — {args.n_aug} augs")
    print(f"{'='*60}")

    total_kps      = n_frames * n_kp
    total_detected = sum(all_detected)
    det_rate       = 100.0 * total_detected / total_kps if total_kps > 0 else 0
    print(f"\nDetection rate: {total_detected}/{total_kps} ({det_rate:.1f}%)")

    all_errors_np = np.array(all_errors)
    print(f"\nPer-keypoint L2 pixel error (raw image coords):")
    print(f"  {'Keypoint':<20s} {'Mean':>8s} {'Median':>8s} {'Std':>8s} {'Max':>8s} {'Det%':>6s}")
    print(f"  {'-'*20} {'-'*8} {'-'*8} {'-'*8} {'-'*8} {'-'*6}")

    for ki, name in enumerate(KEYPOINT_NAMES):
        col   = all_errors_np[:, ki]
        valid = col[~np.isnan(col)]
        if len(valid) > 0:
            short   = name.split("_")[-1]
            det_pct = 100.0 * len(valid) / len(col)
            print(f"  {short:<20s} {np.mean(valid):8.2f} {np.median(valid):8.2f} {np.std(valid):8.2f} {np.max(valid):8.2f} {det_pct:5.1f}%")
        else:
            print(f"  {name.split('_')[-1]:<20s}  {'N/A':>8s}")

    all_valid = all_errors_np[~np.isnan(all_errors_np)]
    if len(all_valid) > 0:
        print(f"\n  {'OVERALL':<20s} {np.mean(all_valid):8.2f} {np.median(all_valid):8.2f} {np.std(all_valid):8.2f} {np.max(all_valid):8.2f}")

        thresholds = [2, 5, 10, 20, 50]
        print(f"\n  Accuracy at thresholds:")
        for th in thresholds:
            pct = 100.0 * np.sum(all_valid < th) / len(all_valid)
            print(f"    < {th:3d} px: {pct:6.1f}%")

    per_frame_mean = [np.nanmean(e) for e in all_errors if not all(math.isnan(x) for x in e)]
    if per_frame_mean:
        print(f"\n  Per-frame mean error: {np.mean(per_frame_mean):.2f} ± {np.std(per_frame_mean):.2f} px")
        print(f"  Best frame:  {np.min(per_frame_mean):.2f} px")
        print(f"  Worst frame: {np.max(per_frame_mean):.2f} px")

    print(f"\n{'='*60}")
    print(f"Baseline sans TTA : 52.9%")
    print(f"Avec TTA ({args.n_aug} augs) : {det_rate:.1f}%")
    diff = det_rate - 52.9
    print(f"Différence        : {diff:+.1f}%")
    print(f"{'='*60}")


def main():
    parser = argparse.ArgumentParser(description="Evaluate DREAM avec TTA — moyenne des positions")
    parser.add_argument("--weights",     "-w", required=True)
    parser.add_argument("--data",        "-d", required=True)
    parser.add_argument("--split",       "-s", default="all",
                        choices=["train", "val", "test", "all"])
    parser.add_argument("--max-samples", "-n", type=int, default=1000)
    parser.add_argument("--n-aug",             type=int, default=6,
                        help="Nombre d'augmentations TTA (default: 6)")
    args = parser.parse_args()
    evaluate_tta(args)


if __name__ == "__main__":
    main()