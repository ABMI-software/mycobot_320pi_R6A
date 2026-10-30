#!/usr/bin/env python3
"""Evaluate DREAM avec Ensemble + TTA + Multi-scale + Vote majoritaire.

Usage:
  python dream/evaluate_dream_ensemble.py \\
      --weights checkpoints_dream/grid_bhs/br04_h05_s10_n30/best_network.pth \\
                checkpoints_dream/grid_seeds/seed_7/best_network.pth \\
      --data dream/dream_data/real_test \\
      --split all --max-samples 1000 --n-aug 12 --min-votes 2 \\
      --scales 0.75 1.0 1.25
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


def resize_image(image_pil, scale):
    """Redimensionne une image par un facteur scale."""
    w, h = image_pil.size
    new_w = int(w * scale)
    new_h = int(h * scale)
    return image_pil.resize((new_w, new_h), PILImage.BILINEAR)


def rescale_keypoints(kps, scale, orig_w, orig_h):
    """Convertit les keypoints détectés sur image scalée vers coordonnées originales."""
    rescaled = []
    for kp in kps:
        if kp is None:
            rescaled.append(None)
            continue
        if hasattr(kp, '__len__') and len(kp) >= 2:
            if kp[0] is None or kp[1] is None or float(kp[0]) < -900:
                rescaled.append(None)
                continue
            # Diviser par scale pour revenir aux coordonnées originales
            rescaled.append((float(kp[0]) / scale, float(kp[1]) / scale))
        else:
            rescaled.append(None)
    return rescaled


def is_valid_kp(kp):
    if kp is None:
        return False
    if hasattr(kp, '__len__') and len(kp) >= 2:
        if kp[0] is None or kp[1] is None:
            return False
        if float(kp[0]) < -900 or float(kp[1]) < -900:
            return False
    return True


def evaluate_ensemble(args):
    # Charger tous les modèles
    models = []
    for w in args.weights:
        config_path = os.path.splitext(w)[0] + ".yaml"
        print(f"Loading model: {w}")
        net = dream.create_network_from_config_file(config_path, w)
        net.enable_evaluation()
        models.append(net)

    total_preds = len(models) * args.n_aug * len(args.scales)
    print(f"\n  Modèles chargés          : {len(models)}")
    print(f"  TTA par modèle           : {args.n_aug} augmentations")
    print(f"  Scales                   : {args.scales}")
    print(f"  Total prédictions/image  : {total_preds}")
    print(f"  Min votes pour détection : {args.min_votes}/{total_preds}")

    K, img_w, img_h = load_camera_intrinsics(args.data)
    print(f"  Camera: fx={K[0,0]:.1f} fy={K[1,1]:.1f} cx={K[0,2]:.1f} cy={K[1,2]:.1f}  {img_w}x{img_h}")

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
    print(f"Using {args.split} split : {len(json_files)} frames")

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

    desc = f"Ensemble ({len(models)}M × {args.n_aug}A × {len(args.scales)}S, votes≥{args.min_votes})"
    for jf in tqdm(json_files, desc=desc):
        frame_id = os.path.basename(jf).replace(".json", "")
        img_path = os.path.join(args.data, f"{frame_id}.rgb.png")

        if not os.path.exists(img_path):
            continue

        gt_proj = load_gt_keypoints(jf)
        image   = PILImage.open(img_path).convert("RGB")

        kp_positions = [[] for _ in range(n_kp)]

        with torch.no_grad():
            for scale in args.scales:
                # Redimensionner l'image
                if scale == 1.0:
                    scaled_image = image
                else:
                    scaled_image = resize_image(image, scale)

                augmented_images = apply_tta_augmentations(scaled_image, args.n_aug)

                for net in models:
                    for aug_img in augmented_images:
                        try:
                            result   = net.keypoints_from_image(aug_img, debug=False)
                            detected = result["detected_keypoints"]

                            # Rescaler les keypoints vers coordonnées originales
                            if scale != 1.0:
                                detected = rescale_keypoints(detected, scale, img_w, img_h)

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
    print(f"ENSEMBLE RESULTS ({n_frames} frames) — {len(models)}M × {args.n_aug}A × {len(args.scales)}S — votes≥{args.min_votes}")
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
    print(f"Baseline sans TTA              : 52.9%")
    print(f"TTA seul (12 augs)             : 58.8%")
    print(f"Ensemble 3M × 12A (votes≥1)   : 62.5%")
    print(f"Ce run                         : {det_rate:.1f}%")
    print(f"{'='*60}")


def main():
    parser = argparse.ArgumentParser(description="Evaluate DREAM Ensemble + TTA + Multi-scale")
    parser.add_argument("--weights",     "-w", required=True, nargs="+")
    parser.add_argument("--data",        "-d", required=True)
    parser.add_argument("--split",       "-s", default="all",
                        choices=["train", "val", "test", "all"])
    parser.add_argument("--max-samples", "-n", type=int, default=1000)
    parser.add_argument("--n-aug",             type=int, default=12,
                        help="Nombre d'augmentations TTA (default: 12)")
    parser.add_argument("--min-votes",         type=int, default=1,
                        help="Nb min détections pour valider keypoint (default: 1)")
    parser.add_argument("--scales",            type=float, nargs="+", default=[1.0],
                        help="Scales multi-resolution ex: 0.75 1.0 1.25 (default: 1.0)")
    args = parser.parse_args()
    evaluate_ensemble(args)


if __name__ == "__main__":
    main()