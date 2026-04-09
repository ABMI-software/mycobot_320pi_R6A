#!/usr/bin/env python3
"""Visualize keypoint annotations overlaid on images — sanity check for NDDS data.

Usage:
  python visualize_ndds.py --data /tmp/dream_data/synthetic --num 10 --output /tmp/dream_viz
"""

import argparse
import math
import os
import sys

import cv2
import numpy as np
from PIL import Image as PILImage

sys.path.insert(0, "/tmp/DREAM")
import dream

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
from mycobot_fk import KEYPOINT_NAMES


COLORS = [
    (255, 0, 0),    # red — base
    (0, 200, 0),    # green — link1
    (0, 0, 255),    # blue — link2
    (255, 255, 0),  # yellow — link3
    (255, 0, 255),  # magenta — link4
    (0, 255, 255),  # cyan — link5
    (255, 128, 0),  # orange — link6 (end-effector)
]


def main():
    parser = argparse.ArgumentParser(description="Visualize NDDS keypoint annotations")
    parser.add_argument("--data", "-d", required=True, help="NDDS data directory")
    parser.add_argument("--num", "-n", type=int, default=10, help="Number of images to visualize")
    parser.add_argument("--output", "-o", default="/tmp/dream_viz", help="Output directory")
    parser.add_argument("--skip", type=int, default=0, help="Skip first N frames")
    args = parser.parse_args()

    os.makedirs(args.output, exist_ok=True)

    found_data = dream.utilities.find_ndds_data_in_dir(args.data)
    data_list = found_data[0]
    if data_list is None:
        print("No NDDS data found in", args.data)
        return

    manip_name = "mycobot320"
    kp_names = KEYPOINT_NAMES

    data_subset = data_list[args.skip: args.skip + args.num]

    for idx, datum in enumerate(data_subset):
        image_path = datum["image_paths"]["rgb"]
        data_path = datum["data_path"]

        # Load image
        img = cv2.imread(image_path)
        if img is None:
            print(f"  Could not read: {image_path}")
            continue

        # Load keypoints
        try:
            keypoints = dream.utilities.load_keypoints(data_path, manip_name, kp_names)
        except Exception as e:
            print(f"  Error loading keypoints for {data_path}: {e}")
            continue

        projs = keypoints["projections"]

        # Draw keypoints
        for i, (kp, name) in enumerate(zip(projs, kp_names)):
            u, v = int(round(kp[0])), int(round(kp[1]))
            color = COLORS[i % len(COLORS)]

            # Draw filled circle
            cv2.circle(img, (u, v), 6, color, -1)
            cv2.circle(img, (u, v), 6, (255, 255, 255), 1)

            # Label
            label = name.replace("mycobot320_", "")
            cv2.putText(img, label, (u + 8, v - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 2)
            cv2.putText(img, label, (u + 8, v - 3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 1)

        # Draw skeleton lines (connect adjacent keypoints)
        for i in range(len(projs) - 1):
            u1, v1 = int(round(projs[i][0])), int(round(projs[i][1]))
            u2, v2 = int(round(projs[i + 1][0])), int(round(projs[i + 1][1]))
            cv2.line(img, (u1, v1), (u2, v2), (200, 200, 200), 1)

        # Save
        out_path = os.path.join(args.output, f"viz_{idx:04d}.png")
        cv2.imwrite(out_path, img)

    print(f"Saved {len(data_subset)} visualizations to {args.output}/")


if __name__ == "__main__":
    main()
