#!/usr/bin/env python3
"""Merge a multi-camera REAL dataset (e.g. real_3cam) into a single NDDS folder,
guaranteeing every frame keeps ITS OWN camera's calibrated intrinsics/extrinsics.

Why this script exists (and not merge_and_convert.py):
  merge_and_convert.py's --ndds flag calls convert_synthetic(), which applies
  the Gazebo synthetic intrinsics (fx=493.8) and synthetic camera transforms
  (front/right/left/top) to every image it is given. Feeding it real camera
  names (arducam/svpro/astra) would silently overwrite their calibrated 2D
  keypoint ground truth with wrong synthetic-camera math.

This script instead reuses convert_to_ndds.py's convert_real() building blocks
per-camera (load_real_intrinsics, _real_camera_transform, project_keypoints),
so each frame's projected_location is computed with the RIGHT camera model,
then merges all cameras into one continuously-numbered NDDS folder (frames
stay pose-adjacent: arducam, svpro, astra for pose 0, then pose 1, ...).

NDDS format keeps ONE _camera_settings.json per dataset directory (verified:
DREAM's train/eval loaders — see evaluate_dream.py:load_camera_intrinsics —
read it once for the whole folder, print it, but do NOT use it to compute
detection rate / pixel error; those come purely from each frame's own
projected_location, already correct per-camera). Since one file cannot
represent 3 different physical cameras, this script ALSO writes a companion
`_camera_settings_per_frame.json` (frame_idx -> camera name + true K) so the
per-frame intrinsics are always fully traceable, and prints an explicit
warning about which camera's K ended up in the cosmetic global file.

Usage:
  python merge_and_convert_real.py \\
      --input dream_data/real_3cam \\
      --output dream_data/real_3cam_ndds \\
      --cameras arducam svpro astra
"""

import argparse
import json
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from convert_to_ndds import (
    CALIBRATION_MAP,
    load_labels,
    load_real_intrinsics,
    _real_camera_transform,
    keypoints_in_camera_frame,
    project_keypoints,
    write_camera_settings,
    write_frame_json,
    copy_image_as_ndds,
)


def merge_and_convert_real(input_dir, output_dir, cameras, primary_camera=None, max_frames=None):
    csv_path = os.path.join(input_dir, "labels.csv")
    rows = load_labels(csv_path)

    os.makedirs(output_dir, exist_ok=True)

    intrinsics = {c: load_real_intrinsics(c) for c in cameras}
    print(f"  Cameras: {cameras}")
    for c in cameras:
        K = intrinsics[c]
        tag = "calibrée" if c in CALIBRATION_MAP else "NON calibrée (défaut)"
        print(f"  {c}: intrinsèques {tag} — fx={K[0,0]:.1f} fy={K[1,1]:.1f} "
              f"cx={K[0,2]:.1f} cy={K[1,2]:.1f}")

    primary = primary_camera or cameras[0]
    write_camera_settings(output_dir, intrinsics[primary])
    print(f"  ⚠️  _camera_settings.json (fichier global, cosmétique) = intrinsèques "
          f"'{primary}'. Non utilisé par evaluate_dream.py pour le taux de "
          f"détection / l'erreur pixel (vérifié : ces métriques viennent des "
          f"'projected_location' déjà correctes par frame). Utile seulement si un "
          f"futur calcul PnP/3D lit ce fichier global — dans ce cas, se référer à "
          f"_camera_settings_per_frame.json ci-dessous plutôt qu'à ce fichier.")

    per_frame_manifest = {}
    frame_idx = 0
    n_skipped = 0

    for row in rows:
        if max_frames is not None and frame_idx >= max_frames:
            break

        cam_name = row["camera"]
        if cam_name not in cameras:
            continue

        joints = [float(row[f"j{i}_rad"]) for i in range(1, 7)]

        try:
            T_cam = _real_camera_transform(cam_name)
        except KeyError:
            n_skipped += 1
            continue

        K = intrinsics[cam_name]
        kp_cam = keypoints_in_camera_frame(joints, T_cam)
        projs = project_keypoints(kp_cam, K)

        img_path = os.path.join(input_dir, row["image_path"])
        if not os.path.exists(img_path):
            n_skipped += 1
            continue

        copy_image_as_ndds(img_path, output_dir, frame_idx)
        write_frame_json(output_dir, frame_idx, kp_cam, projs)
        per_frame_manifest[f"{frame_idx:06d}"] = {
            "camera": cam_name,
            "fx": float(K[0, 0]), "fy": float(K[1, 1]),
            "cx": float(K[0, 2]), "cy": float(K[1, 2]),
            "calibrated": cam_name in CALIBRATION_MAP,
        }

        frame_idx += 1
        if frame_idx % 500 == 0:
            print(f"  Converted {frame_idx} frames...")

    manifest_path = os.path.join(output_dir, "_camera_settings_per_frame.json")
    with open(manifest_path, "w") as f:
        json.dump(per_frame_manifest, f, indent=1)

    print(f"  Total: {frame_idx} frames written, {n_skipped} skipped")
    print(f"  Per-frame intrinsics manifest: {manifest_path}")
    return frame_idx


def main():
    parser = argparse.ArgumentParser(
        description="Merge a multi-camera real dataset into one NDDS folder "
                     "with guaranteed per-camera-correct intrinsics."
    )
    parser.add_argument("--input", "-i", required=True,
                         help="Input dataset directory (labels.csv + images/<camera>/)")
    parser.add_argument("--output", "-o", required=True,
                         help="Output NDDS directory")
    parser.add_argument("--cameras", "-c", nargs="+", default=["arducam", "svpro", "astra"],
                         help="Camera names to include, in the order to prefer for "
                              "the cosmetic global _camera_settings.json")
    parser.add_argument("--primary-camera", default=None,
                         help="Which camera's K to write in the global "
                              "_camera_settings.json (default: first in --cameras)")
    parser.add_argument("--max-frames", "-n", type=int, default=None)
    args = parser.parse_args()

    print(f"Converting real dataset: {args.input} -> {args.output}")
    merge_and_convert_real(args.input, args.output, args.cameras,
                            args.primary_camera, args.max_frames)
    print("Done!")


if __name__ == "__main__":
    main()
