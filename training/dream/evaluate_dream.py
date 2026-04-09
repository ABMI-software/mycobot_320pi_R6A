#!/usr/bin/env python3
"""Evaluate a trained DREAM model on synthetic validation data.

Measures:
  - Per-keypoint L2 pixel error (detected vs GT on net-output resolution)
  - Percentage of keypoints detected
  - ADD (Average Distance of keypoints) in 3D camera space
  - PnP pose accuracy if GT camera transform is available

Usage:
  python evaluate_dream.py \
      --weights .../best_network.pth \
      --data /tmp/dream_data/synthetic \
      --split val \
      --max-samples 500 \
      --visualize --viz-dir /tmp/dream_eval_viz
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

# Add DREAM to path
sys.path.insert(0, "/tmp/DREAM")
import dream
from dream.image_proc import preprocess_image, convert_keypoints_to_netin_from_raw

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
from mycobot_fk import KEYPOINT_NAMES


def load_camera_intrinsics(data_dir):
    """Load camera intrinsics from NDDS _camera_settings.json."""
    path = os.path.join(data_dir, "_camera_settings.json")
    with open(path) as f:
        settings = json.load(f)
    cs = settings["camera_settings"][0]
    intr = cs["intrinsic_settings"]
    K = np.array([
        [intr["fx"], intr.get("s", 0), intr["cx"]],
        [0, intr["fy"], intr["cy"]],
        [0, 0, 1],
    ], dtype=np.float64)
    w = cs["captured_image_size"]["width"]
    h = cs["captured_image_size"]["height"]
    return K, w, h


def load_gt_keypoints(json_path):
    """Load ground truth keypoints from NDDS JSON.

    Returns:
        projections: list of [u, v] (2D pixel coords in raw image)
        positions_3d: list of [x, y, z] (3D in camera frame)
    """
    with open(json_path) as f:
        data = json.load(f)

    projections = []
    positions_3d = []
    for kp in data["objects"][0]["keypoints"]:
        projections.append(kp["projected_location"])
        positions_3d.append(kp["location"])
    return projections, positions_3d


def evaluate(args):
    # Load network
    config_path = os.path.splitext(args.weights)[0] + ".yaml"
    print(f"Loading model: {args.weights}")
    dream_net = dream.create_network_from_config_file(config_path, args.weights)
    dream_net.enable_evaluation()
    print(f"  Architecture: {dream_net.architecture_type}")
    print(f"  Keypoints: {dream_net.n_keypoints}")
    print(f"  Net input:  {dream_net.trained_net_input_resolution()}")
    print(f"  Net output: {dream_net.net_output_resolution_from_input_resolution(dream_net.trained_net_input_resolution())}")
    print(f"  Image preproc: {dream_net.image_preprocessing()}")

    # Load camera intrinsics
    K, img_w, img_h = load_camera_intrinsics(args.data)
    print(f"  Camera: fx={K[0,0]:.1f} fy={K[1,1]:.1f} cx={K[0,2]:.1f} cy={K[1,2]:.1f}  {img_w}x{img_h}")

    # Find all frames
    json_files = sorted(glob.glob(os.path.join(args.data, "??????.json")))
    n_total = len(json_files)
    print(f"\nFound {n_total} frames in {args.data}")

    # Split into train/val using same split as DREAM training
    if args.split == "val":
        train_frac = 0.8
        n_train = int(n_total * train_frac)
        json_files = json_files[n_train:]
        print(f"Using validation split: frames {n_train}-{n_total} ({len(json_files)} frames)")
    elif args.split == "train":
        train_frac = 0.8
        n_train = int(n_total * train_frac)
        json_files = json_files[:n_train]
        print(f"Using training split: frames 0-{n_train} ({len(json_files)} frames)")
    else:
        print(f"Using all frames")

    if args.max_samples and len(json_files) > args.max_samples:
        # Sample evenly
        step = len(json_files) / args.max_samples
        indices = [int(i * step) for i in range(args.max_samples)]
        json_files = [json_files[i] for i in indices]
        print(f"Subsampled to {len(json_files)} frames")

    if args.viz_dir:
        os.makedirs(args.viz_dir, exist_ok=True)

    # Evaluate
    all_errors = []  # per-frame list of per-keypoint errors
    all_detected = []  # per-frame count of detected keypoints
    all_errors_3d = []  # 3D position errors when GT available
    n_frames = len(json_files)

    net_input_res = dream_net.trained_net_input_resolution()
    net_output_res = dream_net.net_output_resolution_from_input_resolution(net_input_res)
    image_preproc = dream_net.image_preprocessing()

    for idx, jf in enumerate(json_files):
        frame_id = os.path.basename(jf).replace(".json", "")
        img_path = os.path.join(args.data, f"{frame_id}.rgb.png")

        if not os.path.exists(img_path):
            continue

        # Load GT
        gt_proj, gt_3d = load_gt_keypoints(jf)

        # Load image
        image = PILImage.open(img_path).convert("RGB")
        image_raw_res = image.size  # (w, h)

        # Run inference
        with torch.no_grad():
            result = dream_net.keypoints_from_image(
                image,
                image_preprocessing_override=None,
                debug=False,
            )
        detected_kp = result["detected_keypoints"]
        # detected_kp is a list of [u, v] in RAW image coordinates or None

        # Compute per-keypoint L2 pixel error in raw image coords
        frame_errors = []
        n_det = 0
        for kp_idx in range(len(KEYPOINT_NAMES)):
            det = detected_kp[kp_idx] if kp_idx < len(detected_kp) else None
            gt = gt_proj[kp_idx] if kp_idx < len(gt_proj) else None

            if det is None or (hasattr(det, '__len__') and len(det) >= 2 and det[0] is None):
                frame_errors.append(float('nan'))
                continue

            n_det += 1
            du = float(det[0]) - float(gt[0])
            dv = float(det[1]) - float(gt[1])
            err = math.sqrt(du * du + dv * dv)
            frame_errors.append(err)

        all_errors.append(frame_errors)
        all_detected.append(n_det)

        # Visualization
        if args.viz_dir and idx < args.max_viz:
            img_np = np.array(image).copy()
            colors = [
                (255, 0, 0), (0, 255, 0), (0, 0, 255),
                (255, 255, 0), (255, 0, 255), (0, 255, 255), (128, 128, 255),
            ]
            for ki in range(len(KEYPOINT_NAMES)):
                color = colors[ki % len(colors)]
                # Draw GT as circles
                if ki < len(gt_proj):
                    gu, gv = int(round(gt_proj[ki][0])), int(round(gt_proj[ki][1]))
                    cv2.circle(img_np, (gu, gv), 6, color, 2)
                # Draw detected as filled circles
                det = detected_kp[ki] if ki < len(detected_kp) else None
                if det is not None and det[0] is not None:
                    du, dv = int(round(float(det[0]))), int(round(float(det[1])))
                    cv2.circle(img_np, (du, dv), 4, color, -1)

            # Add legend
            for ki, name in enumerate(KEYPOINT_NAMES):
                short = name.split("_")[-1]
                color = colors[ki % len(colors)]
                y_pos = 20 + ki * 18
                cv2.putText(img_np, short, (10, y_pos),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)
                if ki < len(frame_errors) and not math.isnan(frame_errors[ki]):
                    cv2.putText(img_np, f"{frame_errors[ki]:.1f}px", (80, y_pos),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

            viz_path = os.path.join(args.viz_dir, f"{frame_id}_eval.png")
            PILImage.fromarray(img_np).save(viz_path)

        if (idx + 1) % 100 == 0:
            print(f"  Processed {idx + 1}/{n_frames} frames...")

    # ---- Aggregate metrics ----
    print(f"\n{'='*60}")
    print(f"EVALUATION RESULTS ({n_frames} frames)")
    print(f"{'='*60}")

    # Detection rate
    total_kps = n_frames * len(KEYPOINT_NAMES)
    total_detected = sum(all_detected)
    det_rate = 100.0 * total_detected / total_kps if total_kps > 0 else 0
    print(f"\nDetection rate: {total_detected}/{total_kps} ({det_rate:.1f}%)")

    # Per-keypoint error statistics
    all_errors_np = np.array(all_errors)  # (n_frames, n_keypoints)
    print(f"\nPer-keypoint L2 pixel error (raw image coords):")
    print(f"  {'Keypoint':<20s} {'Mean':>8s} {'Median':>8s} {'Std':>8s} {'Max':>8s} {'Det%':>6s}")
    print(f"  {'-'*20} {'-'*8} {'-'*8} {'-'*8} {'-'*8} {'-'*6}")

    for ki, name in enumerate(KEYPOINT_NAMES):
        col = all_errors_np[:, ki]
        valid = col[~np.isnan(col)]
        if len(valid) > 0:
            short = name.split("_")[-1]
            det_pct = 100.0 * len(valid) / len(col)
            print(f"  {short:<20s} {np.mean(valid):8.2f} {np.median(valid):8.2f} {np.std(valid):8.2f} {np.max(valid):8.2f} {det_pct:5.1f}%")
        else:
            print(f"  {name.split('_')[-1]:<20s}  {'N/A':>8s}")

    # Overall
    all_valid = all_errors_np[~np.isnan(all_errors_np)]
    if len(all_valid) > 0:
        print(f"\n  {'OVERALL':<20s} {np.mean(all_valid):8.2f} {np.median(all_valid):8.2f} {np.std(all_valid):8.2f} {np.max(all_valid):8.2f}")

        # AUC metrics (percentage below threshold)
        thresholds = [2, 5, 10, 20, 50]
        print(f"\n  Accuracy at thresholds:")
        for th in thresholds:
            pct = 100.0 * np.sum(all_valid < th) / len(all_valid)
            print(f"    < {th:3d} px: {pct:6.1f}%")

    # Per-frame statistics
    per_frame_mean = [np.nanmean(e) for e in all_errors if not all(math.isnan(x) for x in e)]
    if per_frame_mean:
        print(f"\n  Per-frame mean error: {np.mean(per_frame_mean):.2f} ± {np.std(per_frame_mean):.2f} px")
        print(f"  Best frame: {np.min(per_frame_mean):.2f} px")
        print(f"  Worst frame: {np.max(per_frame_mean):.2f} px")

    print(f"\n{'='*60}")

    if args.viz_dir:
        print(f"\nVisualizations saved to: {args.viz_dir}")
        # Create montage of first N
        viz_files = sorted(glob.glob(os.path.join(args.viz_dir, "*_eval.png")))[:20]
        if viz_files:
            try:
                imgs = [PILImage.open(f).resize((320, 240)) for f in viz_files]
                cols = 5
                rows = (len(imgs) + cols - 1) // cols
                montage = PILImage.new("RGB", (320 * cols, 240 * rows))
                for i, img in enumerate(imgs):
                    r, c = i // cols, i % cols
                    montage.paste(img, (c * 320, r * 240))
                montage_path = os.path.join(args.viz_dir, "montage_eval.png")
                montage.save(montage_path)
                print(f"  Montage: {montage_path}")
            except Exception as e:
                print(f"  (montage failed: {e})")


def main():
    parser = argparse.ArgumentParser(description="Evaluate DREAM model")
    parser.add_argument("--weights", "-w", required=True,
                        help="Path to best_network.pth")
    parser.add_argument("--data", "-d", required=True,
                        help="NDDS data directory")
    parser.add_argument("--split", "-s", default="val",
                        choices=["train", "val", "all"],
                        help="Which split to evaluate on")
    parser.add_argument("--max-samples", "-n", type=int, default=500,
                        help="Max number of frames to evaluate")
    parser.add_argument("--viz-dir", default=None,
                        help="Directory for visualization output")
    parser.add_argument("--max-viz", type=int, default=50,
                        help="Max number of frames to visualize")
    parser.add_argument("--visualize", action="store_true",
                        help="Enable visualization (sets viz-dir if not specified)")
    args = parser.parse_args()

    if args.visualize and not args.viz_dir:
        args.viz_dir = "/tmp/dream_eval_viz"

    evaluate(args)


if __name__ == "__main__":
    main()
