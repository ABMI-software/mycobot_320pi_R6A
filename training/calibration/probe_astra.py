#!/usr/bin/env python3
"""Astra RGB diagnostic probe: capture one frame, try detection in 4 modes,
report counts, save annotated PNGs to /tmp/probe_astra_*.png.

Modes:
  1. raw_bgr           — frame as returned, treated as BGR
  2. raw_bgr + CLAHE   — same, with CLAHE before detection
  3. swap_rb           — swap red/blue channels (in case oni_grabber returns RGB)
  4. swap_rb + CLAHE   — both

Run with the Astra board in view, well-lit. Should clarify whether the
issue is channel order, low contrast, or a more fundamental sensor problem.
"""
from __future__ import annotations

import sys
from pathlib import Path

import cv2
import numpy as np

REPO = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO / "teleop"))
from orbbec_capture import open_orbbec  # type: ignore


def annotate(gray, frame_for_draw, label, dictionary, board, parameters,
             out_path: Path) -> tuple[int, int]:
    mc, mi, rej = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)
    n_markers = 0 if mi is None else len(mi)
    n_charuco = 0
    cc = ci = None
    if n_markers > 0:
        try:
            n_res, cc, ci = cv2.aruco.interpolateCornersCharuco(mc, mi, gray, board)
            n_charuco = int(n_res) if n_res is not None else 0
        except Exception as exc:
            print(f"[{label}] interpolate failed: {exc}")

    out = frame_for_draw.copy()
    if n_markers > 0:
        cv2.aruco.drawDetectedMarkers(out, mc, mi)
    if ci is not None and len(ci) > 0:
        cv2.aruco.drawDetectedCornersCharuco(out, cc, ci, (0, 255, 0))
    cv2.putText(out,
                f"{label}: markers={n_markers}  charuco={n_charuco}  rej={len(rej) if rej is not None else 0}",
                (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

    sharp = float(cv2.Laplacian(gray, cv2.CV_64F).var())
    cv2.putText(out, f"sharpness={sharp:.0f}  mean={gray.mean():.0f}",
                (10, 55), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 1)
    cv2.imwrite(str(out_path), out)

    ids_list = mi.flatten().tolist() if mi is not None else []
    print(f"[{label:18s}] markers={n_markers:3d}  charuco={n_charuco:3d}  "
          f"sharp={sharp:6.0f}  mean={gray.mean():5.1f}  ids={ids_list[:8]}{'...' if len(ids_list) > 8 else ''}")
    return n_markers, n_charuco


def main() -> int:
    import time

    print("Opening Astra...")
    cap = open_orbbec(auto_spawn=True, open_timeout=10.0)
    if not cap.isOpened():
        print("Astra failed to open", file=sys.stderr)
        return 1

    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
    parameters = cv2.aruco.DetectorParameters_create()

    # Capture for 15 s, keep the frame with the most detected markers.
    duration_s = 15.0
    print(f"Hold the ChArUco board in front of the Astra for {duration_s:.0f}s.")
    print("Move it slowly. The probe keeps the frame with the most markers detected.")
    print()
    deadline = time.time() + duration_s
    best = (-1, None)
    n_frames = 0
    while time.time() < deadline:
        ok, frame = cap.read()
        if not ok:
            continue
        n_frames += 1
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        _, mi, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)
        n = 0 if mi is None else len(mi)
        if n > best[0]:
            best = (n, frame.copy())
            print(f"  t={time.time()-(deadline-duration_s):4.1f}s  best now {n} markers")
    cap.release()

    n_best, frame = best
    if frame is None:
        print("No frames captured", file=sys.stderr)
        return 1

    print(f"\nScanned {n_frames} frames; best has {n_best} markers.")
    print(f"Frame shape: {frame.shape}, dtype: {frame.dtype}")
    cv2.imwrite("/tmp/probe_astra_raw.png", frame)
    print("Best raw frame saved: /tmp/probe_astra_raw.png")
    print()

    board = cv2.aruco.CharucoBoard_create(6, 9, 0.030, 0.024, dictionary)
    clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))

    # Mode 1: treat as BGR, no CLAHE
    gray_bgr = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    annotate(gray_bgr, frame, "raw_bgr", dictionary, board, parameters,
             Path("/tmp/probe_astra_1_bgr.png"))

    # Mode 2: BGR + CLAHE
    gray_bgr_clahe = clahe.apply(gray_bgr)
    annotate(gray_bgr_clahe, frame, "raw_bgr+clahe", dictionary, board, parameters,
             Path("/tmp/probe_astra_2_bgr_clahe.png"))

    # Mode 3: swap R<->B (in case oni_grabber writes RGB)
    swapped = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)  # swap R<->B
    gray_swap = cv2.cvtColor(swapped, cv2.COLOR_BGR2GRAY)
    annotate(gray_swap, swapped, "swap_rb", dictionary, board, parameters,
             Path("/tmp/probe_astra_3_swap.png"))

    # Mode 4: swapped + CLAHE
    gray_swap_clahe = clahe.apply(gray_swap)
    annotate(gray_swap_clahe, swapped, "swap_rb+clahe", dictionary, board, parameters,
             Path("/tmp/probe_astra_4_swap_clahe.png"))

    print()
    print("Inspect the 4 annotated images to see which mode finds the most markers:")
    for name in ["1_bgr", "2_bgr_clahe", "3_swap", "4_swap_clahe"]:
        print(f"  /tmp/probe_astra_{name}.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
