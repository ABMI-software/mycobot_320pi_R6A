#!/usr/bin/env python3
"""Generate a printable ChArUco board PNG using the SAME parameters as
calibrate_camera.py defaults. Print at exact size (no "fit to page" scaling)
and glue to a flat rigid surface (foam-board minimum).

Usage:
    python training/calibration/generate_board.py
    python training/calibration/generate_board.py --output /tmp/board.png \\
           --squares-x 6 --squares-y 9 \\
           --square-length-m 0.030 --marker-length-m 0.024 \\
           --dpi 300

Defaults match the calibrator: 6x9 squares, 30 mm / 24 mm markers,
DICT_4X4_1000. At 300 DPI the printed size is ~178 mm x 267 mm — fits on
A4 with margin.
"""
from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


def parse_args() -> argparse.Namespace:
    here = Path(__file__).resolve().parent
    p = argparse.ArgumentParser()
    p.add_argument("--output", type=Path, default=here / "charuco_board.png")
    p.add_argument("--dictionary", default="DICT_4X4_1000")
    p.add_argument("--squares-x", type=int, default=6)
    p.add_argument("--squares-y", type=int, default=9)
    p.add_argument("--square-length-m", type=float, default=0.030)
    p.add_argument("--marker-length-m", type=float, default=0.024)
    p.add_argument("--dpi", type=int, default=300, help="Print resolution. 300 DPI ≈ photo print quality.")
    p.add_argument("--margin-px", type=int, default=20)
    p.add_argument("--border-bits", type=int, default=1)
    return p.parse_args()


def main() -> int:
    args = parse_args()

    try:
        dict_id = getattr(cv2.aruco, args.dictionary)
    except AttributeError:
        raise SystemExit(f"Unknown ArUco dictionary: {args.dictionary}")

    dictionary = cv2.aruco.getPredefinedDictionary(dict_id)

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

    # Pixel size that matches the requested DPI + physical square length.
    inches_per_meter = 39.3700787
    px_per_square = int(round(args.square_length_m * inches_per_meter * args.dpi))
    img_w = px_per_square * args.squares_x
    img_h = px_per_square * args.squares_y

    if hasattr(board, "generateImage"):
        img = board.generateImage((img_w, img_h), marginSize=args.margin_px,
                                  borderBits=args.border_bits)
    else:
        img = board.draw((img_w, img_h), marginSize=args.margin_px,
                         borderBits=args.border_bits)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.output), img)

    physical_w_mm = args.squares_x * args.square_length_m * 1000
    physical_h_mm = args.squares_y * args.square_length_m * 1000

    print(f"Board generated: {args.output}")
    print(f"  Pixels:        {img_w} x {img_h}  (at {args.dpi} DPI)")
    print(f"  Print size:    {physical_w_mm:.1f} mm x {physical_h_mm:.1f} mm")
    print(f"  Squares:       {args.squares_x} x {args.squares_y} of {args.square_length_m*1000:.1f} mm")
    print(f"  Markers:       {args.marker_length_m*1000:.1f} mm — dictionary {args.dictionary}")
    print()
    print("Print at 100% (NO 'fit to page'). Verify with calipers that one square = "
          f"{args.square_length_m*1000:.1f} mm before calibrating.")
    print("Glue to foam-board or hardboard. Bent paper ruins calibration.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
