#!/usr/bin/env python3
"""Evaluate a trained DREAM model — v2 avec support split test (80/10/10).

Nouveautés vs evaluate_dream.py :
  - Split 'test' ajouté (10% dernières frames — jamais vues pendant training)
  - Split basé sur 80/10/10 au lieu de 80/20
  - Affichage du split utilisé clairement

Usage:
  # Sur val (10%) — même split que pendant training
  python dream/evaluate_dream_v2.py \
      --weights checkpoints_dream/vgg_cyclegan_e50/best_network.pth \
      --data dream/dream_data/mixed_cyclegan \
      --split val

  # Sur test (10%) — jamais vu pendant training
  python dream/evaluate_dream_v2.py \
      --weights checkpoints_dream/vgg_cyclegan_e50/best_network.pth \
      --data dream/dream_data/mixed_cyclegan \
      --split test

  # Sur real_test (1K frames réelles — test final)
  python dream/evaluate_dream_v2.py \
      --weights checkpoints_dream/vgg_cyclegan_e50/best_network.pth \
      --data dream/dream_data/real_test \
      --split all
"""

import argparse
import json
import math
import os
import sys
import glob

import cv2
import numpy as np
from PIL import Image as PILImage
import torch

sys.path.insert(0, "/tmp/DREAM")
import dream
from dream.image_proc import preprocess_image, convert_keypoints_to_netin_from_raw

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
from mycobot_fk import KEYPOINT_NAMES

# Split fractions — doivent correspondre à train_dream_mixed_aug_optimized_v3.py
TRAIN_FRAC = 0.80
VAL_FRAC   = 0.10
# TEST_FRAC  = 0.10


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
    projections  = []
    positions_3d = []
    for kp in data["objects"][0]["keypoints"]:
        projections.append(kp["projected_location"])
        positions_3d.append(kp["location"])
    return projections, positions_3d


def evaluate(args):
    config_path = os.path.splitext(args.weights)[0] + ".yaml"
    print(f"Loading model: {args.weights}")
    dream_net = dream.create_network_from_config_file(config_path, args.weights)
    dream_net.enable_evaluation()
    print(f"  Architecture: {dream_net.architecture_type}")
    print(f"  Keypoints: {dream_net.n_keypoints}")
    print(f"  Net input:  {dream_net.trained_net_input_resolution()}")
    print(f"  Net output: {dream_net.net_output_resolution_from_input_resolution(dream_net.trained_net_input_resolution())}")
    print(f"  Image preproc: {dream_net.image_preprocessing()}")

    K, img_w, img_h = load_camera_intrinsics(args.data)
    print(f"  Camera: fx={K[0,0]:.1f} fy={K[1,1]:.1f} cx={K[0,2]:.1f} cy={K[1,2]:.1f}  {img_w}x{img_h}")

    json_files = sorted(glob.glob(os.path.join(args.data, "??????.json")))
    n_total    = len(json_files)
    print(f"\nFound {n_total} frames in {args.data}")

    # ---- Splits 80/10/10 ----
    n_train = int(n_total * TRAIN_FRAC)
    n_val   = int(n_total * VAL_FRAC)
    n_test  = n_total - n_train - n_val

    if args.split == "val":
        json_files = json_files[n_train:n_train + n_val]
        print(f"Using val split   : frames {n_train}-{n_train+n_val} ({len(json_files)} frames) — 10%")
    elif args.split == "test":
        json_files = json_files[n_train + n_val:]
        print(f"Using test split  : frames {n_train+n_val}-{n_total} ({len(json_files)} frames) — 10% jamais vues ✅")
    elif args.split == "train":
        json_files = json_files[:n_train]
        print(f"Using train split : frames 0-{n_train} ({len(json_files)} frames) — 80%")
    else:
        print(f"Using all frames  : {len(json_files)} frames")

    if args.max_samples and len(json_files) > args.max_samples:
        step    = len(json_files) / args.max_samples
        indices = [int(i * step) for i in range(args.max_samples)]
        json_files = [json_files[i] for i in indices]
        print(f"Subsampled to {len(json_files)} frames")

    if args.viz_dir:
        os.makedirs(args.viz_dir, exist_ok=True)

    all_errors    = []
    all_detected  = []
    n_frames      = len(json_files)

    for idx, jf in enumerate(json_files):
        frame_id = os.path.basename(jf).replace(".json", "")
        img_path = os.path.join(args.data, f"{frame_id}.rgb.png")

        if not os.path.exists(img_path):
            continue

        gt_proj, gt_3d = load_gt_keypoints(jf)
        image           = PILImage.open(img_path).convert("RGB")

        with torch.no_grad():
            result = dream_net.keypoints_from_image(image, debug=False)

        detected_kp = result["detected_keypoints"]

        frame_errors = []
        n_det = 0
        for kp_idx in range(len(KEYPOINT_NAMES)):
            det = detected_kp[kp_idx] if kp_idx < len(detected_kp) else None
            gt  = gt_proj[kp_idx]     if kp_idx < len(gt_proj)    else None

            if det is None or (hasattr(det, '__len__') and len(det) >= 2 and det[0] is None):
                frame_errors.append(float('nan'))
                continue

            du_val, dv_val = float(det[0]), float(det[1])
            if du_val < -900 or dv_val < -900:
                frame_errors.append(float('nan'))
                continue

            n_det += 1
            du  = du_val - float(gt[0])
            dv  = dv_val - float(gt[1])
            err = math.sqrt(du * du + dv * dv)
            frame_errors.append(err)

        all_errors.append(frame_errors)
        all_detected.append(n_det)

        if (idx + 1) % 100 == 0:
            print(f"  Processed {idx + 1}/{n_frames} frames...")

    # ---- Résultats ----
    print(f"\n{'='*60}")
    print(f"EVALUATION RESULTS ({n_frames} frames) — split={args.split}")
    print(f"{'='*60}")

    total_kps      = n_frames * len(KEYPOINT_NAMES)
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
        print(f"  Best frame: {np.min(per_frame_mean):.2f} px")
        print(f"  Worst frame: {np.max(per_frame_mean):.2f} px")

    print(f"\n{'='*60}")


def main():
    parser = argparse.ArgumentParser(description="Evaluate DREAM model v2 — support split test 80/10/10")
    parser.add_argument("--weights", "-w", required=True)
    parser.add_argument("--data",    "-d", required=True)
    parser.add_argument("--split",   "-s", default="val",
                        choices=["train", "val", "test", "all"],
                        help="Split : train(80%) | val(10%) | test(10%) | all")
    parser.add_argument("--max-samples", "-n", type=int, default=500)
    parser.add_argument("--viz-dir",     default=None)
    parser.add_argument("--max-viz",     type=int, default=50)
    parser.add_argument("--visualize",   action="store_true")
    args = parser.parse_args()

    if args.visualize and not args.viz_dir:
        args.viz_dir = "/tmp/dream_eval_viz"

    evaluate(args)


if __name__ == "__main__":
    main()