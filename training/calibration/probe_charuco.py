#!/usr/bin/env python3
"""Headless single-shot ChArUco detection probe.

Captures one frame from the given camera, runs ArUco + ChArUco detection
with the same parameters as calibrate_camera.py, prints the diagnostic
counters to stdout, and saves an annotated PNG. Use this to verify the
board is actually being detected before spinning up the calibration loop.

Usage:
    python training/calibration/probe_charuco.py --camera 0 \\
        --squares-x 4 --squares-y 3 \\
        --square-length-m 0.030 --marker-length-m 0.024 \\
        --output /tmp/probe_cam_0.png
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser()
    p.add_argument("--camera", type=int, required=True)
    p.add_argument("--dictionary", default="DICT_4X4_1000")
    p.add_argument("--squares-x", type=int, default=4)
    p.add_argument("--squares-y", type=int, default=3)
    p.add_argument("--square-length-m", type=float, default=0.030)
    p.add_argument("--marker-length-m", type=float, default=0.024)
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--warmup-frames", type=int, default=10,
                   help="Drop the first N frames so auto-exposure has time to settle.")
    p.add_argument("--output", type=Path, default=Path("/tmp/probe_charuco.png"))
    return p.parse_args()


def main() -> int:
    args = parse_args()

    dictionary = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, args.dictionary))

    if hasattr(cv2.aruco, "CharucoBoard"):
        board = cv2.aruco.CharucoBoard(
            (args.squares_x, args.squares_y),
            args.square_length_m, args.marker_length_m,
            dictionary,
        )
    else:
        board = cv2.aruco.CharucoBoard_create(
            args.squares_x, args.squares_y,
            args.square_length_m, args.marker_length_m,
            dictionary,
        )

    parameters = cv2.aruco.DetectorParameters_create()

    cap = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
    if not cap.isOpened():
        print(f"Could not open camera {args.camera}", file=sys.stderr)
        return 1
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)
    cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

    for _ in range(args.warmup_frames):
        cap.read()
        time.sleep(0.05)

    ok, frame = cap.read()
    cap.release()
    if not ok:
        print("Frame read failed", file=sys.stderr)
        return 1

    h, w = frame.shape[:2]
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    sharpness = float(cv2.Laplacian(gray, cv2.CV_64F).var())
    mean = float(np.mean(gray))
    p99 = float(np.percentile(gray, 99))

    marker_corners, marker_ids, rejected = cv2.aruco.detectMarkers(
        gray, dictionary, parameters=parameters,
    )

    n_markers = 0 if marker_ids is None else len(marker_ids)
    n_charuco = 0
    charuco_corners = None
    charuco_ids = None
    if n_markers > 0:
        try:
            n_charuco_result, charuco_corners, charuco_ids = cv2.aruco.interpolateCornersCharuco(
                marker_corners, marker_ids, gray, board,
            )
            n_charuco = int(n_charuco_result) if n_charuco_result is not None else 0
        except Exception as exc:
            print(f"interpolateCornersCharuco failed: {exc}", file=sys.stderr)

    annotated = frame.copy()
    if n_markers > 0:
        cv2.aruco.drawDetectedMarkers(annotated, marker_corners, marker_ids)
    if charuco_ids is not None and len(charuco_ids) > 0:
        cv2.aruco.drawDetectedCornersCharuco(
            annotated, charuco_corners, charuco_ids, (0, 255, 0),
        )
    overlay = (
        f"markers={n_markers}  charuco={n_charuco}  sharp={sharpness:.0f}  "
        f"mean={mean:.0f}  p99={p99:.0f}  res={w}x{h}"
    )
    cv2.putText(annotated, overlay, (10, 30), cv2.FONT_HERSHEY_SIMPLEX,
                0.55, (0, 255, 255), 2)
    cv2.putText(annotated,
                f"rejected_candidates={len(rejected) if rejected is not None else 0}",
                (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.output), annotated)

    print()
    print("=" * 60)
    print("ChArUco probe — single frame")
    print("=" * 60)
    print(f"camera idx:           {args.camera}")
    print(f"resolution:           {w}x{h}")
    print(f"frame mean gray:      {mean:.1f}    (target ~110-140)")
    print(f"frame p99:            {p99:.1f}    (close to 255 = saturated)")
    print(f"sharpness (Laplacian):{sharpness:.1f}   (need >= 80 to be accepted)")
    print(f"detected ArUco markers: {n_markers}")
    if marker_ids is not None:
        print(f"  marker ids:         {marker_ids.flatten().tolist()}")
    print(f"rejected ArUco candidates: {len(rejected) if rejected is not None else 0}")
    print(f"interpolated ChArUco corners: {n_charuco}")
    print()
    print(f"annotated image saved to: {args.output}")
    print("Open it to visually verify the detector sees the board.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
