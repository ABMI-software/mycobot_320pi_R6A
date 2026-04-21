#!/usr/bin/env python3
"""Convert our synthetic/real datasets to DREAM NDDS format.

NDDS format expected by DREAM:
  <output_dir>/
    _camera_settings.json      — camera intrinsics
    _object_settings.json      — (optional) object info
    000000.json                — keypoint annotations per frame
    000000.rgb.png             — RGB image
    000001.json
    000001.rgb.png
    ...

Each frame JSON has:
  objects:
    - class: "mycobot320"
      keypoints:
        - name: "mycobot320_base"
          location: [x, y, z]           # 3D position in camera frame
          projected_location: [u, v]    # 2D pixel coordinates

Usage:
  # Convert synthetic dataset (all 4 cameras → single NDDS dir)
  python convert_to_ndds.py \\
      --input /tmp/mycobot_synth_v2 \\
      --output /tmp/dream_data/synthetic \\
      --source synth \\
      --cameras front right left top

  # Convert real dataset (cam0 only)
  python convert_to_ndds.py \\
      --input /tmp/real_dataset \\
      --output /tmp/dream_data/real \\
      --source real \\
      --cameras cam0
"""

import argparse
import csv
import json
import math
import os
import shutil
from pathlib import Path

import numpy as np

# Add parent to path for FK import
import sys
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mycobot_fk import (
    KEYPOINT_NAMES,
    GAZEBO_INTRINSICS,
    forward_kinematics,
    keypoints_in_camera_frame,
    project_keypoints,
    get_camera_transform,
)


# ---- Real camera parameters (Elephant Robotics Pi camera) ----
# Default intrinsics for 640×480 USB camera (approximate, no calibration)
# Users should replace with calibrated values for best accuracy
REAL_CAMERA_INTRINSICS = np.array([
    [610.0,   0.0, 320.0],
    [  0.0, 610.0, 240.0],
    [  0.0,   0.0,   1.0],
], dtype=np.float64)

# Real camera extrinsics (approximate — needs calibration for production)
# cam0: front-facing camera looking at the robot from ~0.5m
# For now we use a placeholder that will be refined via PnP during inference
REAL_CAMERA_TRANSFORMS = {
    "cam0": {
        "xyz": (0.5, 0.0, 0.3),
        "rpy": (0.0, 0.2, math.pi),
    },
    "cam3": {
        "xyz": (0.0, 0.5, 0.3),
        "rpy": (0.0, 0.2, -math.pi / 2),
    },
}


def _real_camera_transform(cam_name):
    """Build world→optical for a real camera (approximate)."""
    from mycobot_fk import _T, _rpy
    cam = REAL_CAMERA_TRANSFORMS[cam_name]
    x, y, z = cam["xyz"]
    r, p, ya = cam["rpy"]
    T_world_link = _T(x, y, z) @ _rpy(r, p, ya)
    T_link_optical = np.array([
        [ 0,  0,  1, 0],
        [-1,  0,  0, 0],
        [ 0, -1,  0, 0],
        [ 0,  0,  0, 1],
    ], dtype=np.float64)
    return T_world_link @ T_link_optical


def write_camera_settings(output_dir, camera_K, width=640, height=480):
    """Write _camera_settings.json in NDDS format."""
    settings = {
        "camera_settings": [
            {
                "name": "mycobot_camera",
                "intrinsic_settings": {
                    "fx": float(camera_K[0, 0]),
                    "fy": float(camera_K[1, 1]),
                    "cx": float(camera_K[0, 2]),
                    "cy": float(camera_K[1, 2]),
                    "s": 0.0,
                },
                "captured_image_size": {
                    "width": width,
                    "height": height,
                },
            }
        ]
    }
    path = os.path.join(output_dir, "_camera_settings.json")
    with open(path, "w") as f:
        json.dump(settings, f, indent=2)
    return path


def write_frame_json(output_dir, frame_idx, positions_wrt_cam, projections):
    """Write per-frame NDDS JSON with keypoint annotations."""
    keypoints = []
    for i, kp_name in enumerate(KEYPOINT_NAMES):
        keypoints.append({
            "name": kp_name,
            "location": positions_wrt_cam[i],
            "projected_location": projections[i],
        })

    data = {
        "objects": [
            {
                "class": "mycobot320",
                "keypoints": keypoints,
            }
        ]
    }
    path = os.path.join(output_dir, f"{frame_idx:06d}.json")
    with open(path, "w") as f:
        json.dump(data, f, indent=2)
    return path


def copy_image_as_ndds(src_path, output_dir, frame_idx):
    """Copy/symlink image to NDDS naming convention: XXXXXX.rgb.png."""
    dst_path = os.path.join(output_dir, f"{frame_idx:06d}.rgb.png")
    shutil.copy2(src_path, dst_path)
    return dst_path


def load_labels(csv_path):
    """Load labels.csv → list of dicts with joint angles and camera/image info."""
    rows = []
    with open(csv_path, "r") as f:
        reader = csv.DictReader(f)
        for row in reader:
            rows.append(row)
    return rows


def convert_synthetic(input_dir, output_dir, cameras):
    """Convert our Gazebo synthetic dataset to NDDS format."""
    csv_path = os.path.join(input_dir, "labels.csv")
    rows = load_labels(csv_path)

    os.makedirs(output_dir, exist_ok=True)

    # Write camera settings (all Gazebo cameras share the same intrinsics)
    write_camera_settings(output_dir, GAZEBO_INTRINSICS)

    frame_idx = 0
    n_skipped = 0

    for row in rows:
        cam_name = row["camera"]
        if cam_name not in cameras:
            continue

        # Parse joint angles (radians)
        joints = [float(row[f"j{i}_rad"]) for i in range(1, 7)]

        # Get camera transform
        T_cam = get_camera_transform(cam_name)

        # Compute 3D keypoint positions in camera frame
        kp_cam = keypoints_in_camera_frame(joints, T_cam)

        # Project to 2D
        projs = project_keypoints(kp_cam, GAZEBO_INTRINSICS)

        # Check all keypoints are in frame (no NaN, within image bounds)
        all_visible = True
        for proj in projs:
            if math.isnan(proj[0]) or math.isnan(proj[1]):
                all_visible = False
                break
            if proj[0] < -50 or proj[0] > 690 or proj[1] < -50 or proj[1] > 530:
                all_visible = False
                break

        if not all_visible:
            n_skipped += 1
            continue

        # Copy image
        img_path = os.path.join(input_dir, row["image_path"])
        if not os.path.exists(img_path):
            n_skipped += 1
            continue

        copy_image_as_ndds(img_path, output_dir, frame_idx)
        write_frame_json(output_dir, frame_idx, kp_cam, projs)

        frame_idx += 1
        if frame_idx % 1000 == 0:
            print(f"  Converted {frame_idx} frames...")

    print(f"  Total: {frame_idx} frames written, {n_skipped} skipped")
    return frame_idx


def convert_real(input_dir, output_dir, cameras):
    """Convert our real dataset to NDDS format.

    NOTE: Real camera extrinsics are approximate. The keypoint 2D projections
    will NOT be pixel-accurate. This data should only be used for fine-tuning
    or evaluation, with the understanding that labels may be noisy.
    For proper real-data training, we need either:
    1. Manual keypoint annotation
    2. Calibrated camera extrinsics
    3. A sim-to-real transfer approach (train on synth, test on real)
    """
    csv_path = os.path.join(input_dir, "labels.csv")
    rows = load_labels(csv_path)

    os.makedirs(output_dir, exist_ok=True)

    # Use real camera intrinsics (approximate)
    write_camera_settings(output_dir, REAL_CAMERA_INTRINSICS)

    frame_idx = 0
    n_skipped = 0

    for row in rows:
        cam_name = row["camera"]
        if cam_name not in cameras:
            continue

        joints = [float(row[f"j{i}_rad"]) for i in range(1, 7)]

        # Get approximate camera transform
        try:
            T_cam = _real_camera_transform(cam_name)
        except KeyError:
            n_skipped += 1
            continue

        kp_cam = keypoints_in_camera_frame(joints, T_cam)
        projs = project_keypoints(kp_cam, REAL_CAMERA_INTRINSICS)

        # For real data, we still copy the image even if projections are off
        img_path = os.path.join(input_dir, row["image_path"])
        if not os.path.exists(img_path):
            n_skipped += 1
            continue

        copy_image_as_ndds(img_path, output_dir, frame_idx)
        write_frame_json(output_dir, frame_idx, kp_cam, projs)

        frame_idx += 1
        if frame_idx % 500 == 0:
            print(f"  Converted {frame_idx} frames...")

    print(f"  Total: {frame_idx} frames written, {n_skipped} skipped")
    return frame_idx


def main():
    parser = argparse.ArgumentParser(
        description="Convert MyCobot datasets to DREAM NDDS format"
    )
    parser.add_argument(
        "--input", "-i", required=True,
        help="Input dataset directory (containing labels.csv + images/)",
    )
    parser.add_argument(
        "--output", "-o", required=True,
        help="Output NDDS directory",
    )
    parser.add_argument(
        "--source", "-s", required=True, choices=["synth", "real"],
        help="Dataset source type",
    )
    parser.add_argument(
        "--cameras", "-c", nargs="+",
        default=None,
        help="Camera names to include (default: all)",
    )
    args = parser.parse_args()

    print(f"Converting {args.source} dataset: {args.input} → {args.output}")

    if args.source == "synth":
        cameras = args.cameras or ["front", "right", "left", "top"]
        print(f"  Cameras: {cameras}")
        convert_synthetic(args.input, args.output, cameras)
    else:
        cameras = args.cameras or ["cam0", "cam3"]
        print(f"  Cameras: {cameras}")
        convert_real(args.input, args.output, cameras)

    print("Done!")


if __name__ == "__main__":
    main()
