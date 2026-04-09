#!/usr/bin/env python3
"""DREAM inference for MyCobot 320 Pi — detect keypoints + solve PnP for pose.

Given a trained DREAM model and an input image, this script:
1. Detects 2D keypoint locations (joint positions) via belief maps
2. Solves PnP to recover camera-to-robot pose
3. (Optional) Computes joint angles if ground truth is provided for comparison

Usage:
  # Single image inference
  python infer_dream.py \\
      --model /path/to/best_network.pth \\
      --image /path/to/image.png

  # Batch inference on a directory
  python infer_dream.py \\
      --model /path/to/best_network.pth \\
      --image-dir /tmp/dream_data/synthetic \\
      --max-images 100

  # Evaluate on NDDS dataset with ground truth
  python infer_dream.py \\
      --model /path/to/best_network.pth \\
      --image-dir /tmp/dream_data/synthetic \\
      --evaluate \\
      --max-images 500
"""

import argparse
import json
import math
import os
import sys

import cv2
import numpy as np
from PIL import Image as PILImage
from ruamel.yaml import YAML
import torch

# Add DREAM to path
sys.path.insert(0, "/tmp/DREAM")
import dream

# Add our FK module
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
from mycobot_fk import (
    KEYPOINT_NAMES,
    GAZEBO_INTRINSICS,
    forward_kinematics,
)


def load_dream_network(model_path, config_path=None):
    """Load a trained DREAM network."""
    if config_path is None:
        config_path = os.path.splitext(model_path)[0] + ".yaml"

    assert os.path.exists(config_path), f"Config not found: {config_path}"
    assert os.path.exists(model_path), f"Model not found: {model_path}"

    parser = YAML(typ="safe")
    with open(config_path, "r") as f:
        network_config = parser.load(f)

    # Force CPU-friendly config
    network_config["training"]["platform"]["gpu_ids"] = [0]

    dream_network = dream.create_network_from_config_data(network_config)
    dream_network.model.load_state_dict(
        torch.load(model_path, map_location="cuda:0")
    )
    dream_network.enable_evaluation()
    return dream_network


def detect_keypoints(dream_network, image_path):
    """Detect 2D keypoints in an image using the trained DREAM network.

    Returns:
        detected_kp: list of [u, v] or [None, None] for each keypoint
        belief_maps: tensor of belief maps
    """
    image = PILImage.open(image_path).convert("RGB")

    result = dream_network.keypoints_from_image(
        image,
        image_preprocessing_override=None,
        debug=True,
    )

    detected_kp = result["detected_keypoints"]
    belief_maps = result.get("belief_maps", None)

    return detected_kp, belief_maps, image


def solve_pose_pnp(detected_kp_2d, canonical_kp_3d, camera_K):
    """Solve PnP to find camera-to-robot transform from detected 2D keypoints.

    Parameters
    ----------
    detected_kp_2d : list of [u, v] or [None, None]
        Detected 2D keypoint pixel coordinates.
    canonical_kp_3d : list of [x, y, z]
        3D keypoint positions in robot base frame (at home pose).
    camera_K : np.ndarray (3×3)
        Camera intrinsic matrix.

    Returns
    -------
    success : bool
    rvec : rotation vector (Rodrigues)
    tvec : translation vector
    """
    # Filter valid keypoints
    pts_3d = []
    pts_2d = []
    for kp_2d, kp_3d in zip(detected_kp_2d, canonical_kp_3d):
        if kp_2d is None or len(kp_2d) < 2:
            continue
        if kp_2d[0] is None or kp_2d[1] is None:
            continue
        if math.isnan(kp_2d[0]) or math.isnan(kp_2d[1]):
            continue
        pts_2d.append(kp_2d)
        pts_3d.append(kp_3d)

    if len(pts_3d) < 4:
        return False, None, None

    pts_3d = np.array(pts_3d, dtype=np.float64)
    pts_2d = np.array(pts_2d, dtype=np.float64)

    success, rvec, tvec = cv2.solvePnP(
        pts_3d, pts_2d, camera_K, None, flags=cv2.SOLVEPNP_EPNP
    )

    if success:
        # Refine with iterative method
        success, rvec, tvec = cv2.solvePnP(
            pts_3d, pts_2d, camera_K, None,
            rvec=rvec, tvec=tvec,
            useExtrinsicGuess=True,
            flags=cv2.SOLVEPNP_ITERATIVE,
        )

    return success, rvec, tvec


def get_canonical_keypoints_3d():
    """Get 3D keypoint positions at the home pose (all joints = 0)."""
    positions, _ = forward_kinematics([0.0] * 6)
    canonical_3d = []
    for kp_name in KEYPOINT_NAMES:
        canonical_3d.append(list(positions[kp_name]))
    return canonical_3d


def keypoint_error(detected_kp, gt_kp):
    """Compute per-keypoint L2 pixel error."""
    errors = []
    for det, gt in zip(detected_kp, gt_kp):
        if det is None or gt is None:
            errors.append(float('nan'))
            continue
        if det[0] is None or det[1] is None:
            errors.append(float('nan'))
            continue
        dx = det[0] - gt[0]
        dy = det[1] - gt[1]
        errors.append(math.sqrt(dx * dx + dy * dy))
    return errors


def visualize_keypoints(image, detected_kp, gt_kp=None, output_path=None):
    """Draw detected (and optionally GT) keypoints on an image."""
    img = np.array(image).copy()
    colors_det = [
        (255, 0, 0), (0, 255, 0), (0, 0, 255),
        (255, 255, 0), (255, 0, 255), (0, 255, 255), (128, 128, 255),
    ]

    for i, (kp, name) in enumerate(zip(detected_kp, KEYPOINT_NAMES)):
        color = colors_det[i % len(colors_det)]
        if kp is not None and kp[0] is not None and not math.isnan(kp[0]):
            u, v = int(round(kp[0])), int(round(kp[1]))
            cv2.circle(img, (u, v), 5, color, -1)
            cv2.putText(img, name.split("_")[-1], (u + 7, v - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

    if gt_kp is not None:
        for i, kp in enumerate(gt_kp):
            if kp is not None and not math.isnan(kp[0]):
                u, v = int(round(kp[0])), int(round(kp[1]))
                cv2.circle(img, (u, v), 5, (0, 255, 0), 2)  # green ring for GT

    result = PILImage.fromarray(img)
    if output_path:
        result.save(output_path)
    return result


def evaluate_on_ndds(dream_network, data_dir, camera_K, max_images=None):
    """Evaluate a DREAM model on an NDDS-format dataset."""
    found_data = dream.utilities.find_ndds_data_in_dir(data_dir)
    data_list = found_data[0]

    if data_list is None or len(data_list) == 0:
        print("No NDDS data found in", data_dir)
        return

    if max_images:
        data_list = data_list[:max_images]

    canonical_3d = get_canonical_keypoints_3d()
    all_kp_errors = []
    pnp_successes = 0
    n_evaluated = 0

    manip_name = dream_network.manipulator_name
    kp_names = dream_network.keypoint_names

    for datum in data_list:
        image_path = datum["image_paths"]["rgb"]
        data_path = datum["data_path"]

        if not os.path.exists(image_path):
            continue

        # Detect keypoints
        detected_kp, _, image = detect_keypoints(dream_network, image_path)

        # Load ground truth
        gt_keypoints = dream.utilities.load_keypoints(data_path, manip_name, kp_names)
        gt_projs = gt_keypoints["projections"]

        # Keypoint pixel errors
        errors = keypoint_error(detected_kp, gt_projs)
        all_kp_errors.append(errors)

        # PnP
        success, _, _ = solve_pose_pnp(detected_kp, canonical_3d, camera_K)
        if success:
            pnp_successes += 1

        n_evaluated += 1
        if n_evaluated % 100 == 0:
            print(f"  Evaluated {n_evaluated}/{len(data_list)} images...")

    # Summary statistics
    all_errors = np.array(all_kp_errors)
    valid_mask = ~np.isnan(all_errors)

    print(f"\n{'=' * 60}")
    print(f"  DREAM Evaluation Results ({n_evaluated} images)")
    print(f"{'=' * 60}")

    print(f"\n  Per-keypoint mean pixel error:")
    for i, kp_name in enumerate(kp_names):
        kp_errs = all_errors[:, i][valid_mask[:, i]]
        if len(kp_errs) > 0:
            print(f"    {kp_name:30s}: {np.mean(kp_errs):6.2f} px  "
                  f"(median {np.median(kp_errs):6.2f}, std {np.std(kp_errs):6.2f})")
        else:
            print(f"    {kp_name:30s}: N/A")

    overall_valid = all_errors[valid_mask]
    if len(overall_valid) > 0:
        print(f"\n  Overall mean pixel error: {np.mean(overall_valid):.2f} px")
        print(f"  Overall median pixel error: {np.median(overall_valid):.2f} px")

    print(f"\n  PnP success rate: {pnp_successes}/{n_evaluated} "
          f"({100 * pnp_successes / max(n_evaluated, 1):.1f}%)")
    print(f"{'=' * 60}")

    return {
        "mean_pixel_error": float(np.mean(overall_valid)) if len(overall_valid) > 0 else None,
        "median_pixel_error": float(np.median(overall_valid)) if len(overall_valid) > 0 else None,
        "pnp_success_rate": pnp_successes / max(n_evaluated, 1),
        "n_evaluated": n_evaluated,
    }


def main():
    parser = argparse.ArgumentParser(
        description="DREAM inference for MyCobot 320 Pi",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--model", "-m", required=True,
        help="Path to trained DREAM model weights (.pth)",
    )
    parser.add_argument(
        "--config", "-c", default=None,
        help="Path to network config (.yaml). If None, inferred from model path.",
    )
    parser.add_argument(
        "--image", "-i", default=None,
        help="Path to a single image for inference",
    )
    parser.add_argument(
        "--image-dir", "-d", default=None,
        help="Path to NDDS dataset directory for batch evaluation",
    )
    parser.add_argument(
        "--evaluate", "-e", action="store_true",
        help="Run full evaluation with ground truth comparison",
    )
    parser.add_argument(
        "--max-images", type=int, default=None,
        help="Maximum number of images to evaluate",
    )
    parser.add_argument(
        "--output-dir", "-o", default=None,
        help="Directory to save visualization outputs",
    )
    parser.add_argument(
        "--camera-fx", type=float, default=None,
        help="Camera focal length (default: Gazebo synthetic camera)",
    )
    args = parser.parse_args()

    # Load network
    print("Loading DREAM network...")
    dream_net = load_dream_network(args.model, args.config)
    print(f"  Manipulator: {dream_net.manipulator_name}")
    print(f"  Keypoints: {dream_net.keypoint_names}")
    print(f"  Architecture: {dream_net.network_config['architecture']['type']}")
    print()

    # Camera intrinsics
    if args.camera_fx:
        camera_K = np.array([
            [args.camera_fx, 0, 320.0],
            [0, args.camera_fx, 240.0],
            [0, 0, 1.0],
        ])
    else:
        camera_K = GAZEBO_INTRINSICS

    if args.image:
        # Single image inference
        print(f"Running inference on: {args.image}")
        detected_kp, belief_maps, image = detect_keypoints(dream_net, args.image)

        print("\nDetected keypoints:")
        for kp, name in zip(detected_kp, KEYPOINT_NAMES):
            if kp is not None and kp[0] is not None:
                print(f"  {name}: ({kp[0]:.1f}, {kp[1]:.1f})")
            else:
                print(f"  {name}: NOT DETECTED")

        # PnP
        canonical_3d = get_canonical_keypoints_3d()
        success, rvec, tvec = solve_pose_pnp(detected_kp, canonical_3d, camera_K)
        if success:
            print(f"\nPnP solved successfully:")
            print(f"  Translation: [{tvec[0][0]:.4f}, {tvec[1][0]:.4f}, {tvec[2][0]:.4f}]")
            print(f"  Rotation (Rodrigues): [{rvec[0][0]:.4f}, {rvec[1][0]:.4f}, {rvec[2][0]:.4f}]")
        else:
            print("\nPnP failed — not enough valid keypoints")

        # Save visualization
        if args.output_dir:
            os.makedirs(args.output_dir, exist_ok=True)
            out_path = os.path.join(args.output_dir, "inference_result.png")
            visualize_keypoints(image, detected_kp, output_path=out_path)
            print(f"\nVisualization saved to: {out_path}")

    elif args.image_dir:
        if args.evaluate:
            evaluate_on_ndds(dream_net, args.image_dir, camera_K, args.max_images)
        else:
            # Quick batch inference without GT
            found_data = dream.utilities.find_ndds_data_in_dir(args.image_dir)
            data_list = found_data[0]
            if args.max_images:
                data_list = data_list[:args.max_images]

            canonical_3d = get_canonical_keypoints_3d()
            for datum in data_list[:5]:  # Just show first 5
                image_path = datum["image_paths"]["rgb"]
                detected_kp, _, _ = detect_keypoints(dream_net, image_path)
                print(f"\n{os.path.basename(image_path)}:")
                for kp, name in zip(detected_kp, KEYPOINT_NAMES):
                    if kp is not None and kp[0] is not None:
                        print(f"  {name}: ({kp[0]:.1f}, {kp[1]:.1f})")
    else:
        print("Error: Provide either --image or --image-dir")
        sys.exit(1)


if __name__ == "__main__":
    main()
