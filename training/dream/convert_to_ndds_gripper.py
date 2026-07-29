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
from mycobot_fk_gripper import (
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
#
# ⚠️ ALL entries below come from the same quick MANUAL recalage method
# (2026-07-03): a handful of frames per camera were visually annotated
# (base + link6 pixel positions) and solved with cv2.solvePnP (seeded from
# the previous rough placeholders to stay in the physically-plausible
# solution branch). Mean reprojection error ~43-66 px (real_3cam rig) /
# ~53-55 px (cam0/cam3) — far better than the original "not even on the
# robot" placeholders, but still approximate (2 landmark points per pose,
# hand-picked pixel coordinates, no proper hand-eye calibration rig).
# Re-run with a real ChArUco-on-flange hand-eye calibration for
# production-grade accuracy. Intrinsics, by contrast, ARE calibrated — see
# CALIBRATION_MAP below (astra, cam0, cam3 excepted).
REAL_CAMERA_TRANSFORMS = {
    # --- cam0/cam3 rig (manual PnP recalage, 2026-07-03 — still approximate) ---
    "cam0": {"xyz": (0.2437, 0.7009, 0.0041), "rpy": (-0.1418, -0.2095, -1.9188)},
    "cam3": {"xyz": (0.0902, 1.0449, -0.2080), "rpy": (-0.2796, -0.2834, -1.6911)},
    # NOTE: training/calibration/camera_extrinsic.yaml (§3.2, ros_jazzy repo,
    # 0.59px reproj) was tried here for arducam on 2026-07-03 and made results
    # WORSE (base 84.9px -> 123.5px) — that calibration is for the pick-and-place
    # rig's fixed overhead arducam mount, a different physical position than the
    # one used to capture real_3cam. Same camera/intrinsics, different mount ->
    # extrinsic does not transfer. Reverted to the manual PnP recalage below.
    "arducam": {"xyz": (-0.3170, -0.6978, 1.0660), "rpy": (0.6339, 0.9941, 0.9094)},
    # --- svpro/astra (manual PnP recalage, 2026-07-03 — still approximate) ---
    "svpro":   {"xyz": (0.5493, 1.3796, 0.3110), "rpy": (0.0449, 0.2318, -2.1443)},
    "astra":   {"xyz": (-1.0331, -0.3908, 0.4162), "rpy": (0.5049, 0.2185, 0.3181)},
}

# Calibrated intrinsics per real camera, from training/calibration/<stem>.npz
# (key 'mtx'). astra is NOT calibrated → falls back to REAL_CAMERA_INTRINSICS.
CALIBRATION_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "calibration")
CALIBRATION_MAP = {"arducam": "cam_0", "svpro": "cam_2", "cam0": "cam_0", "cam3": "cam_3"}


def load_real_intrinsics(cam_name):
    """Calibrated 3x3 K for a real camera, or the default if uncalibrated."""
    stem = CALIBRATION_MAP.get(cam_name)
    if stem:
        npz_path = os.path.join(CALIBRATION_DIR, f"{stem}.npz")
        if os.path.exists(npz_path):
            return np.load(npz_path)["mtx"].astype(np.float64)
    return REAL_CAMERA_INTRINSICS


def _real_camera_transform(cam_name):
    """Build world→optical for a real camera (approximate)."""
    from mycobot_fk_gripper import _T, _rpy
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


def write_frame_json(output_dir, frame_idx, positions_wrt_cam, projections,
                     gripper_absent=False):
    """Write per-frame NDDS JSON with keypoint annotations.

    If ``gripper_absent`` is True (dataset captured WITHOUT the gripper mounted,
    e.g. the 6000 old real images), the gripper_tip keypoint is written with an
    out-of-frame projected_location [-1, -1]. DREAM's create_belief_map then
    produces an all-zero target for that channel — the network is supervised to
    NOT peak there, which is the correct ground truth for gripper-less frames
    (the black gripper's visual presence on the with-gripper frames lets the
    network learn to disambiguate). Keeps J1-J6 supervision intact.
    """
    keypoints = []
    for i, kp_name in enumerate(KEYPOINT_NAMES):
        proj = projections[i]
        if gripper_absent and kp_name == "mycobot320_gripper_tip":
            proj = [-1.0, -1.0]
        keypoints.append({
            "name": kp_name,
            "location": positions_wrt_cam[i],
            "projected_location": proj,
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


def convert_real(input_dir, output_dir, cameras, max_frames=None,
                 gripper_absent=False):
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

    # Calibrated intrinsics per camera (astra falls back to default).
    intrinsics = {c: load_real_intrinsics(c) for c in cameras}
    for c in cameras:
        K = intrinsics[c]
        tag = "calibrée" if c in CALIBRATION_MAP else "NON calibrée (défaut)"
        print(f"  {c}: intrinsèques {tag} — fx={K[0,0]:.1f} fy={K[1,1]:.1f} "
              f"cx={K[0,2]:.1f} cy={K[1,2]:.1f}")

    # One NDDS dir = one _camera_settings.json → use the primary camera's K.
    write_camera_settings(output_dir, intrinsics[cameras[0]])
    if len(cameras) > 1:
        print(f"  ⚠️  {len(cameras)} caméras dans une seule sortie NDDS : "
              f"_camera_settings.json prend {cameras[0]}. Pour des intrinsèques "
              f"correctes, convertis UNE caméra par run (--cameras arducam).")

    frame_idx = 0
    n_skipped = 0

    for row in rows:
        if max_frames is not None and frame_idx >= max_frames:
            break

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
        projs = project_keypoints(kp_cam, intrinsics[cam_name])

        # For real data, we still copy the image even if projections are off
        img_path = os.path.join(input_dir, row["image_path"])
        if not os.path.exists(img_path):
            n_skipped += 1
            continue

        copy_image_as_ndds(img_path, output_dir, frame_idx)
        write_frame_json(output_dir, frame_idx, kp_cam, projs,
                         gripper_absent=gripper_absent)

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
    parser.add_argument(
        "--max-frames", "-n", type=int, default=None,
        help="Max frames to write (real only). Default: all.",
    )
    parser.add_argument(
        "--gripper-absent", action="store_true",
        help="Real dataset captured WITHOUT gripper (e.g. the 6000 old real "
             "images): write gripper_tip out-of-frame so it supervises J1-J6 "
             "only, not the 8th keypoint.",
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
        if args.gripper_absent:
            print("  ⚠️  gripper-absent: gripper_tip écrit hors-cadre (-1,-1) "
                  "→ supervise J1-J6 seulement.")
        convert_real(args.input, args.output, cameras, max_frames=args.max_frames,
                     gripper_absent=args.gripper_absent)

    print("Done!")


if __name__ == "__main__":
    main()
