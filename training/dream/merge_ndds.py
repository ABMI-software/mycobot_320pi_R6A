#!/usr/bin/env python3
"""Concatenate multiple already-converted NDDS directories into one, renumbering
frames continuously. Each source frame keeps its own correct 2D/3D annotations
(computed with the right camera model at conversion time) — this script only
copies and renumbers, it never recomputes projections.

Usage:
  python merge_ndds.py \\
      --inputs dream_data/synthetic_50k_ndds dream_data/real_3cam_train_x5_ndds \\
      --output dream_data/mix_synth50k_real3camx5_ndds
"""

import argparse
import os
import shutil


def merge_ndds(input_dirs, output_dir):
    os.makedirs(output_dir, exist_ok=True)

    primary_settings = None
    frame_idx = 0
    for input_dir in input_dirs:
        settings_path = os.path.join(input_dir, "_camera_settings.json")
        if primary_settings is None and os.path.exists(settings_path):
            primary_settings = settings_path

        json_files = sorted(
            f for f in os.listdir(input_dir)
            if f.endswith(".json") and not f.startswith("_")
        )
        print(f"  {input_dir}: {len(json_files)} frames")

        for jf in json_files:
            stem = jf[:-5]
            src_json = os.path.join(input_dir, jf)
            src_rgb = os.path.join(input_dir, f"{stem}.rgb.png")

            dst_stem = f"{frame_idx:06d}"
            shutil.copy2(src_json, os.path.join(output_dir, f"{dst_stem}.json"))
            shutil.copy2(src_rgb, os.path.join(output_dir, f"{dst_stem}.rgb.png"))

            frame_idx += 1
            if frame_idx % 5000 == 0:
                print(f"  Merged {frame_idx} frames...")

    if primary_settings:
        shutil.copy2(primary_settings, os.path.join(output_dir, "_camera_settings.json"))

    print(f"Total merged: {frame_idx} frames -> {output_dir}")
    return frame_idx


def main():
    parser = argparse.ArgumentParser(description="Merge NDDS directories (post-conversion)")
    parser.add_argument("--inputs", "-i", nargs="+", required=True)
    parser.add_argument("--output", "-o", required=True)
    args = parser.parse_args()

    merge_ndds(args.inputs, args.output)
    print("Done!")


if __name__ == "__main__":
    main()
