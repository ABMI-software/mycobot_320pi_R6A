#!/usr/bin/env python3
"""Merge multiple synthetic datasets and convert to DREAM NDDS format.

This script:
1. Merges the existing 20K dataset with the new 7.5K dataset
2. Re-indexes all samples contiguously
3. Converts the merged dataset to NDDS format for DREAM training

Usage:
  python merge_and_convert.py \
      --datasets /path/to/dataset1 /path/to/dataset2 \
      --merged /tmp/merged_synth \
      --ndds /tmp/dream_data/synthetic_50k \
      --cameras front right left top
"""

import argparse
import csv
import os
import shutil
from pathlib import Path


def merge_datasets(dataset_dirs, merged_dir, cameras=None):
    """Merge multiple synthetic datasets into one.

    Each dataset has:
      labels.csv  (header: index, j1_rad..j6_rad, j1_deg..j6_deg, camera, image_path)
      images/{camera}/{XXXXXX}.png

    The merged dataset renumbers all samples contiguously.
    """
    os.makedirs(merged_dir, exist_ok=True)
    merged_csv = os.path.join(merged_dir, "labels.csv")

    # Discover all camera dirs from first dataset if not specified
    if cameras is None:
        first_img_dir = os.path.join(dataset_dirs[0], "images")
        cameras = sorted([
            d for d in os.listdir(first_img_dir)
            if os.path.isdir(os.path.join(first_img_dir, d))
        ])
    print(f"  Cameras: {cameras}")

    for cam in cameras:
        os.makedirs(os.path.join(merged_dir, "images", cam), exist_ok=True)

    global_idx = 0
    total_rows = 0

    with open(merged_csv, "w", newline="") as out_f:
        writer = csv.writer(out_f)
        header_written = False

        for ds_dir in dataset_dirs:
            csv_path = os.path.join(ds_dir, "labels.csv")
            if not os.path.exists(csv_path):
                print(f"  ⚠ Skipping {ds_dir}: no labels.csv")
                continue

            print(f"  Processing: {ds_dir}")
            with open(csv_path, "r") as in_f:
                reader = csv.DictReader(in_f)

                if not header_written:
                    writer.writerow(reader.fieldnames)
                    header_written = True

                # Group rows by original index (one pose → multiple camera rows)
                prev_orig_idx = None
                for row in reader:
                    cam_name = row["camera"]
                    if cam_name not in cameras:
                        continue

                    orig_idx = int(row["index"])
                    if orig_idx != prev_orig_idx:
                        if prev_orig_idx is not None:
                            global_idx += 1
                        prev_orig_idx = orig_idx

                    # Build new image path
                    new_filename = f"{global_idx:06d}.png"
                    new_img_rel = f"images/{cam_name}/{new_filename}"

                    # Copy image
                    src_img = os.path.join(ds_dir, row["image_path"])
                    dst_img = os.path.join(merged_dir, new_img_rel)
                    if os.path.exists(src_img):
                        shutil.copy2(src_img, dst_img)
                    else:
                        print(f"    ⚠ Missing: {src_img}")
                        continue

                    # Write updated row
                    row["index"] = global_idx
                    row["image_path"] = new_img_rel
                    writer.writerow([row[f] for f in reader.fieldnames])
                    total_rows += 1

                # Advance global_idx for last group
                if prev_orig_idx is not None:
                    global_idx += 1

            print(f"    → {global_idx} poses so far")

    print(f"\n  ✅ Merged: {global_idx} total poses, {total_rows} rows")
    print(f"  ✅ Output: {merged_dir}")
    return global_idx, total_rows


def main():
    parser = argparse.ArgumentParser(
        description="Merge synthetic datasets and optionally convert to NDDS"
    )
    parser.add_argument(
        "--datasets", "-d", nargs="+", required=True,
        help="Input dataset directories to merge",
    )
    parser.add_argument(
        "--merged", "-m", required=True,
        help="Output merged dataset directory",
    )
    parser.add_argument(
        "--ndds", "-n", default=None,
        help="If provided, also convert to NDDS format at this path",
    )
    parser.add_argument(
        "--cameras", "-c", nargs="+", default=None,
        help="Camera names to include (default: all found)",
    )
    args = parser.parse_args()

    print(f"=== Merging {len(args.datasets)} datasets ===")
    n_poses, n_rows = merge_datasets(args.datasets, args.merged, args.cameras)

    if args.ndds:
        print(f"\n=== Converting merged dataset to NDDS ===")
        from convert_to_ndds import convert_synthetic
        cameras = args.cameras or ["front", "right", "left", "top"]
        convert_synthetic(args.merged, args.ndds, cameras)
        print(f"  ✅ NDDS output: {args.ndds}")

    print(f"\nDone! {n_poses} poses, {n_rows} total image entries.")


if __name__ == "__main__":
    main()
