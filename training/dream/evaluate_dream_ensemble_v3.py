#!/usr/bin/env python3
"""Evaluate DREAM Ensemble + TTA + Vote + Filtrage outliers (max-spread).

Nouveauté vs v2 : --max-spread filtre les détections trop éloignées
de la médiane du groupe → réduit mean/median sans perdre trop de détections.

Usage:
  python dream/evaluate_dream_ensemble_v3.py \\
      --weights checkpoints_dream/grid_bhs/br04_h05_s10_n30/best_network.pth \\
                checkpoints_dream/grid_seeds/seed_7/best_network.pth \\
                checkpoints_dream/grid_seeds/seed_256/best_network.pth \\
      --data dream/dream_data/real_test \\
      --split all --max-samples 1000 --n-aug 12 \\
      --min-votes 2 --max-spread 50
"""

import argparse
import json
import math
import os
import sys
import glob

import numpy as np
from PIL import Image as PILImage, ImageEnhance, ImageFilter
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
    return [kp["projected_location"] for kp in data["objects"][0]["keypoints"]]


def gamma_correction(image_pil, gamma):
    img_np = np.array(image_pil).astype(np.float32) / 255.0
    return PILImage.fromarray((np.power(img_np, gamma) * 255).clip(0, 255).astype(np.uint8))


def apply_tta_augmentations(image_pil, n_aug=12):
    all_augs = [image_pil]
    if n_aug >= 2:  all_augs.append(ImageEnhance.Brightness(image_pil).enhance(1.15))
    if n_aug >= 3:  all_augs.append(ImageEnhance.Brightness(image_pil).enhance(0.85))
    if n_aug >= 4:  all_augs.append(ImageEnhance.Contrast(image_pil).enhance(1.15))
    if n_aug >= 5:  all_augs.append(ImageEnhance.Contrast(image_pil).enhance(0.85))
    if n_aug >= 6:
        img_np = np.array(image_pil).astype(np.float32)
        noise  = np.random.normal(0, 8, img_np.shape).astype(np.float32)
        all_augs.append(PILImage.fromarray(np.clip(img_np + noise, 0, 255).astype(np.uint8)))
    if n_aug >= 7:  all_augs.append(ImageEnhance.Color(image_pil).enhance(1.20))
    if n_aug >= 8:  all_augs.append(ImageEnhance.Color(image_pil).enhance(0.80))
    if n_aug >= 9:  all_augs.append(gamma_correction(image_pil, 1.2))
    if n_aug >= 10: all_augs.append(gamma_correction(image_pil, 0.8))
    if n_aug >= 11: all_augs.append(image_pil.filter(ImageFilter.SHARPEN))
    if n_aug >= 12: all_augs.append(image_pil.filter(ImageFilter.GaussianBlur(radius=1)))
    return all_augs[:n_aug]


def is_valid_kp(kp):
    if kp is None:
        return False
    if hasattr(kp, '__len__') and len(kp) >= 2:
        if kp[0] is None or kp[1] is None:
            return False
        if float(kp[0]) < -900 or float(kp[1]) < -900:
            return False
    return True


def filter_outliers(positions, max_spread):
    """Filtre les positions trop éloignées de la médiane du groupe.
    
    Calcule la médiane (u_med, v_med) et rejette les positions
    dont la distance à la médiane dépasse max_spread pixels.
    """
    if len(positions) <= 1:
        return positions

    us = np.array([p[0] for p in positions])
    vs = np.array([p[1] for p in positions])

    u_med = np.median(us)
    v_med = np.median(vs)

    filtered = []
    for u, v in positions:
        dist = math.sqrt((u - u_med)**2 + (v - v_med)**2)
        if dist <= max_spread:
            filtered.append((u, v))

    # Si tout est filtré → garder au moins le point le plus proche de la médiane
    if len(filtered) == 0:
        dists = [math.sqrt((u - u_med)**2 + (v - v_med)**2) for u, v in positions]
        best  = positions[np.argmin(dists)]
        filtered = [best]

    return filtered


def evaluate_ensemble(args):
    models = []
    for w in args.weights:
        config_path = os.path.splitext(w)[0] + ".yaml"
        print(f"Loading model: {w}")
        net = dream.create_network_from_config_file(config_path, w)
        net.enable_evaluation()
        models.append(net)

    total_preds = len(models) * args.n_aug
    print(f"\n  Modèles chargés          : {len(models)}")
    print(f"  TTA par modèle           : {args.n_aug} augmentations")
    print(f"  Total prédictions/image  : {total_preds}")
    print(f"  Min votes                : {args.min_votes}/{total_preds}")
    print(f"  Max spread (filtrage)    : {args.max_spread} px")

    K, img_w, img_h = load_camera_intrinsics(args.data)
    print(f"  Camera: {img_w}x{img_h}")

    json_files = sorted(glob.glob(os.path.join(args.data, "??????.json")))
    n_total    = len(json_files)
    print(f"\nFound {n_total} frames in {args.data}")

    n_train = int(n_total * TRAIN_FRAC)
    n_val   = int(n_total * VAL_FRAC)

    if args.split == "val":
        json_files = json_files[n_train:n_train + n_val]
    elif args.split == "test":
        json_files = json_files[n_train + n_val:]
    elif args.split == "train":
        json_files = json_files[:n_train]
    print(f"Using {args.split} : {len(json_files)} frames")

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

    desc = f"Ensemble ({len(models)}M×{args.n_aug}A, votes≥{args.min_votes}, spread≤{args.max_spread}px)"
    for jf in tqdm(json_files, desc=desc):
        frame_id = os.path.basename(jf).replace(".json", "")
        img_path = os.path.join(args.data, f"{frame_id}.rgb.png")

        if not os.path.exists(img_path):
            continue

        gt_proj = load_gt_keypoints(jf)
        image   = PILImage.open(img_path).convert("RGB")

        augmented_images = apply_tta_augmentations(image, args.n_aug)
        kp_positions = [[] for _ in range(n_kp)]

        with torch.no_grad():
            for net in models:
                for aug_img in augmented_images:
                    try:
                        result   = net.keypoints_from_image(aug_img, debug=False)
                        detected = result["detected_keypoints"]
                        for ki in range(n_kp):
                            kp = detected[ki] if ki < len(detected) else None
                            if is_valid_kp(kp):
                                kp_positions[ki].append((float(kp[0]), float(kp[1])))
                    except Exception:
                        continue

        frame_errors = []
        n_det = 0

        for ki in range(n_kp):
            gt = gt_proj[ki] if ki < len(gt_proj) else None

            if len(kp_positions[ki]) < args.min_votes or gt is None:
                frame_errors.append(float('nan'))
                continue

            # Filtrage outliers
            filtered = filter_outliers(kp_positions[ki], args.max_spread)

            if len(filtered) < args.min_votes:
                frame_errors.append(float('nan'))
                continue

            u_avg = np.mean([p[0] for p in filtered])
            v_avg = np.mean([p[1] for p in filtered])

            n_det += 1
            du  = u_avg - float(gt[0])
            dv  = v_avg - float(gt[1])
            err = math.sqrt(du * du + dv * dv)
            frame_errors.append(err)

        all_errors.append(frame_errors)
        all_detected.append(n_det)

    # ---- Résultats ----
    print(f"\n{'='*60}")
    print(f"ENSEMBLE v3 ({n_frames} frames) — {len(models)}M×{args.n_aug}A — votes≥{args.min_votes} — spread≤{args.max_spread}px")
    print(f"{'='*60}")

    total_kps      = n_frames * n_kp
    total_detected = sum(all_detected)
    det_rate       = 100.0 * total_detected / total_kps if total_kps > 0 else 0
    print(f"\nDetection rate: {total_detected}/{total_kps} ({det_rate:.1f}%)")

    all_errors_np = np.array(all_errors)
    print(f"\nPer-keypoint L2 pixel error:")
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
    print(f"Baseline                       : 52.9%  médiane 2.82px")
    print(f"TTA 12 augs                    : 58.8%  médiane 5.59px")
    print(f"Ensemble 3M×12A votes≥1        : 62.5%  médiane 14.18px")
    print(f"Ce run                         : {det_rate:.1f}%  médiane {np.median(all_valid):.2f}px")
    print(f"{'='*60}")


def main():
    parser = argparse.ArgumentParser(description="Evaluate DREAM Ensemble + TTA + filtrage outliers")
    parser.add_argument("--weights",     "-w", required=True, nargs="+")
    parser.add_argument("--data",        "-d", required=True)
    parser.add_argument("--split",       "-s", default="all",
                        choices=["train", "val", "test", "all"])
    parser.add_argument("--max-samples", "-n", type=int, default=1000)
    parser.add_argument("--n-aug",             type=int, default=12)
    parser.add_argument("--min-votes",         type=int, default=1,
                        help="Nb min détections pour valider keypoint (default: 1)")
    parser.add_argument("--max-spread",        type=float, default=50.0,
                        help="Distance max en px pour filtrer outliers (default: 50px)")
    args = parser.parse_args()
    evaluate_ensemble(args)


if __name__ == "__main__":
    main()