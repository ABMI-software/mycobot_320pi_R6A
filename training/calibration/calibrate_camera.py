#!/usr/bin/env python3
"""Intrinsic calibration for the MyCobot 320 Pi cameras (Arducam ×2 + Astra RGB).

Combines:
  - Capture-side settings freeze (v4l2-ctl recipes + OpenCV property setters
    for exposure / WB / focus / gain) so the calibration is captured under the
    same conditions DREAM will see at inference time.
  - ChArUco-board calibration with quality gating (sharpness + coverage grid +
    diversity timer + per-view reprojection-error rejection).

Per-camera workflow:

    # cam_0 (Arducam #1)
    python training/calibration/calibrate_camera.py --camera 0 --name cam_0

    # cam_3 (Arducam #2 — typically /dev/video2 once plugged after cam_0)
    python training/calibration/calibrate_camera.py --camera 2 --name cam_3

    # Astra RGB (typically /dev/video4 — the depth stream is separate, not handled here)
    python training/calibration/calibrate_camera.py --camera 4 --name astra_rgb \\
           --width 640 --height 480

Each run writes:
    training/calibration/<name>.npz             — mtx, dist, rvecs, tvecs
    training/calibration/<name>.meta.json        — recipe + RMS + sample count
    training/calibration/<name>.snapshot.png     — last accepted frame (for record)

Use the resulting K vs. the values baked in /tmp/dream_data/real_cam0/_camera_settings.json
(currently fx=fy=610, cx=320, cy=240) to decide whether the existing GT projections
need to be regenerated before the v2 dataset capture.

Controls during capture:
    SPACE  manually accept the current valid frame (only useful with --manual)
    C      run calibration on accepted frames + save
    R      clear the accepted-frames buffer
    P      print live camera properties to terminal
    I      toggle on-screen overlay (sharpness / coverage / status)
    Q/ESC  quit without saving
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional, Sequence, Tuple

import cv2
import numpy as np


# --------------------------- Backend / FourCC --------------------------- #

DEFAULT_BACKEND = {
    "any": cv2.CAP_ANY,
    "v4l2": getattr(cv2, "CAP_V4L2", cv2.CAP_ANY),
}

DEFAULT_FOURCC = {
    "YUY2": cv2.VideoWriter_fourcc(*"YUY2"),
    "MJPG": cv2.VideoWriter_fourcc(*"MJPG"),
}


# --------------------------- Camera presets ----------------------------- #
# Known-good v4l2 recipes for the project cameras. Apply these *before*
# OpenCV opens the device so the camera is in a deterministic state.
# Kept here (not in v4l2-ctl shell calls) so the resulting calibration is
# reproducible from the script alone.

CAMERA_PRESETS = {
    "arducam": dict(
        auto_exposure="off",
        exposure=80.0,
        gain=0.0,
        auto_wb="off",
        wb_temperature=4600.0,
        autofocus="off",
        focus=50.0,
        sharpness=None,
    ),
    "astra_rgb": dict(
        # Astra exposes fewer controls via UVC; leave AE on, WB fixed.
        auto_exposure="on",
        exposure=None,
        gain=None,
        auto_wb="off",
        wb_temperature=4600.0,
        autofocus="off",
        focus=None,
        sharpness=None,
    ),
}


# --------------------------- Utility setters ---------------------------- #

def _set_if_requested(cap: cv2.VideoCapture, prop_id: int, value, label: str) -> None:
    if value is None:
        return
    success = cap.set(prop_id, value)
    actual = cap.get(prop_id)
    print(f"  {label:18s} req={value!r:<8} ok={success!s:<5} read={actual}")


def _apply_camera_settings(cap: cv2.VideoCapture, args: argparse.Namespace) -> None:
    print("Applying camera settings:")
    _set_if_requested(cap, cv2.CAP_PROP_FRAME_WIDTH, args.width, "frame_width")
    _set_if_requested(cap, cv2.CAP_PROP_FRAME_HEIGHT, args.height, "frame_height")
    _set_if_requested(cap, cv2.CAP_PROP_BUFFERSIZE, 1, "buffersize")
    if args.fourcc:
        _set_if_requested(cap, cv2.CAP_PROP_FOURCC, DEFAULT_FOURCC[args.fourcc], "fourcc")

    if args.auto_exposure is not None:
        requested = 0.75 if args.auto_exposure == "on" else 0.25
        _set_if_requested(cap, cv2.CAP_PROP_AUTO_EXPOSURE, requested, "auto_exposure")

    if args.auto_wb is not None:
        requested = 1.0 if args.auto_wb == "on" else 0.0
        _set_if_requested(cap, cv2.CAP_PROP_AUTO_WB, requested, "auto_white_balance")

    if args.autofocus is not None and hasattr(cv2, "CAP_PROP_AUTOFOCUS"):
        requested = 1.0 if args.autofocus == "on" else 0.0
        _set_if_requested(cap, cv2.CAP_PROP_AUTOFOCUS, requested, "autofocus")

    if args.focus is not None and hasattr(cv2, "CAP_PROP_FOCUS"):
        _set_if_requested(cap, cv2.CAP_PROP_FOCUS, args.focus, "focus")

    _set_if_requested(cap, cv2.CAP_PROP_EXPOSURE, args.exposure, "exposure")
    _set_if_requested(cap, cv2.CAP_PROP_GAIN, args.gain, "gain")
    _set_if_requested(cap, cv2.CAP_PROP_BRIGHTNESS, args.brightness, "brightness")
    _set_if_requested(cap, cv2.CAP_PROP_CONTRAST, args.contrast, "contrast")
    _set_if_requested(cap, cv2.CAP_PROP_SATURATION, args.saturation, "saturation")
    _set_if_requested(cap, cv2.CAP_PROP_SHARPNESS, args.sharpness, "sharpness")
    _set_if_requested(cap, cv2.CAP_PROP_WB_TEMPERATURE, args.wb_temperature, "wb_temperature")


def _print_camera_properties(cap: cv2.VideoCapture) -> None:
    props = [
        ("frame_width", cv2.CAP_PROP_FRAME_WIDTH),
        ("frame_height", cv2.CAP_PROP_FRAME_HEIGHT),
        ("fps", cv2.CAP_PROP_FPS),
        ("auto_exposure", cv2.CAP_PROP_AUTO_EXPOSURE),
        ("exposure", cv2.CAP_PROP_EXPOSURE),
        ("gain", cv2.CAP_PROP_GAIN),
        ("brightness", cv2.CAP_PROP_BRIGHTNESS),
        ("contrast", cv2.CAP_PROP_CONTRAST),
        ("saturation", cv2.CAP_PROP_SATURATION),
        ("sharpness", cv2.CAP_PROP_SHARPNESS),
        ("auto_wb", cv2.CAP_PROP_AUTO_WB),
        ("wb_temperature", cv2.CAP_PROP_WB_TEMPERATURE),
    ]
    if hasattr(cv2, "CAP_PROP_AUTOFOCUS"):
        props.append(("autofocus", cv2.CAP_PROP_AUTOFOCUS))
    if hasattr(cv2, "CAP_PROP_FOCUS"):
        props.append(("focus", cv2.CAP_PROP_FOCUS))
    print("\nLive camera properties:")
    for label, prop_id in props:
        print(f"  {label:16s} {cap.get(prop_id)}")
    print()


# --------------------------- ArUco / ChArUco ---------------------------- #

def _create_detector_parameters():
    # Prefer the legacy `_create()` factory when available — on OpenCV 4.6
    # the new `DetectorParameters()` constructor returns an object whose
    # `cornerRefinementMethod` accessor segfaults (known regression).
    if hasattr(cv2.aruco, "DetectorParameters_create"):
        parameters = cv2.aruco.DetectorParameters_create()
    else:
        parameters = cv2.aruco.DetectorParameters()

    if hasattr(cv2.aruco, "CORNER_REFINE_SUBPIX"):
        parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    for attr, val in [
        ("cornerRefinementWinSize", 5),
        ("cornerRefinementMaxIterations", 50),
        ("cornerRefinementMinAccuracy", 0.01),
        ("adaptiveThreshWinSizeMin", 5),
        ("adaptiveThreshWinSizeMax", 31),
        ("adaptiveThreshWinSizeStep", 4),
        ("minMarkerPerimeterRate", 0.02),
        ("maxMarkerPerimeterRate", 4.0),
        ("minCornerDistanceRate", 0.03),
        ("minDistanceToBorder", 3),
    ]:
        if hasattr(parameters, attr):
            setattr(parameters, attr, val)
    return parameters


def _build_charuco_board(squares_x, squares_y, square_length_m, marker_length_m, dictionary):
    # Prefer the legacy `_create()` factory — on OpenCV 4.6 the new
    # `CharucoBoard((sx, sy), ...)` constructor returns an object that
    # crashes `interpolateCornersCharuco` (segfault, same family of
    # regressions as DetectorParameters()).
    if hasattr(cv2.aruco, "CharucoBoard_create"):
        return cv2.aruco.CharucoBoard_create(
            squares_x, squares_y, square_length_m, marker_length_m, dictionary
        )
    return cv2.aruco.CharucoBoard(
        (squares_x, squares_y), square_length_m, marker_length_m, dictionary
    )


def _detect_markers(gray, dictionary, parameters):
    if hasattr(cv2.aruco, "ArucoDetector"):
        detector = cv2.aruco.ArucoDetector(dictionary, parameters)
        return detector.detectMarkers(gray)
    return cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)


def _interpolate_charuco(marker_corners, marker_ids, gray, board):
    try:
        return cv2.aruco.interpolateCornersCharuco(
            markerCorners=marker_corners, markerIds=marker_ids, image=gray, board=board
        )
    except TypeError:
        return cv2.aruco.interpolateCornersCharuco(marker_corners, marker_ids, gray, board)


def _sharpness_score(gray: np.ndarray) -> float:
    return float(cv2.Laplacian(gray, cv2.CV_64F).var())


def _coverage_cell(points, frame_w, frame_h, grid_cols, grid_rows):
    center = points.reshape(-1, 2).mean(axis=0)
    col = min(grid_cols - 1, max(0, int(center[0] / max(1, frame_w) * grid_cols)))
    row = min(grid_rows - 1, max(0, int(center[1] / max(1, frame_h) * grid_rows)))
    return col, row


# --------------------------- Sample dataclass --------------------------- #

@dataclass
class CalibrationSample:
    charuco_corners: np.ndarray
    charuco_ids: np.ndarray
    image_size: Tuple[int, int]
    sharpness: float
    marker_count: int
    charuco_count: int
    coverage_cell: Tuple[int, int]
    capture_time: float
    frame_bgr: np.ndarray  # kept so we can dump the last accepted snapshot


_CLAHE_INSTANCE = None  # cached so we don't recreate the engine every frame


def _maybe_clahe(gray: np.ndarray, enabled: bool) -> np.ndarray:
    """Optional CLAHE preprocessing. Useful for low-contrast / noisy sensors
    like the Orbbec Astra RGB stream where ArUco's adaptive thresholding
    struggles with partial markers."""
    if not enabled:
        return gray
    global _CLAHE_INSTANCE
    if _CLAHE_INSTANCE is None:
        _CLAHE_INSTANCE = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
    return _CLAHE_INSTANCE.apply(gray)


def detect_charuco_sample(
    frame, dictionary, board, parameters,
    min_markers, min_charuco, grid_cols, grid_rows,
    *, clahe: bool = False,
):
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    gray = _maybe_clahe(gray, clahe)
    marker_corners, marker_ids, _ = _detect_markers(gray, dictionary, parameters)
    preview = frame.copy()
    if marker_ids is not None and len(marker_ids) > 0:
        cv2.aruco.drawDetectedMarkers(preview, marker_corners, marker_ids)

    if marker_ids is None or len(marker_ids) < min_markers:
        return None, preview

    _, charuco_corners, charuco_ids = _interpolate_charuco(
        marker_corners, marker_ids, gray, board
    )
    if charuco_corners is not None and charuco_ids is not None and len(charuco_ids) > 0:
        cv2.aruco.drawDetectedCornersCharuco(preview, charuco_corners, charuco_ids, (0, 255, 0))

    if charuco_ids is None or len(charuco_ids) < min_charuco:
        return None, preview

    h, w = gray.shape[:2]
    sample = CalibrationSample(
        charuco_corners=charuco_corners.copy(),
        charuco_ids=charuco_ids.copy(),
        image_size=(w, h),
        sharpness=_sharpness_score(gray),
        marker_count=int(len(marker_ids)),
        charuco_count=int(len(charuco_ids)),
        coverage_cell=_coverage_cell(charuco_corners, w, h, grid_cols, grid_rows),
        capture_time=time.time(),
        frame_bgr=frame.copy(),
    )
    return sample, preview


def sample_is_diverse(sample, accepted, min_time_delta_s, max_per_cell):
    cell_count = sum(1 for s in accepted if s.coverage_cell == sample.coverage_cell)
    if cell_count >= max_per_cell:
        return False
    if not accepted:
        return True
    if sample.capture_time - accepted[-1].capture_time < min_time_delta_s:
        return False
    return True


# --------------------------- Calibration -------------------------------- #

def calibrate_charuco(samples, board, image_size):
    all_corners = [s.charuco_corners for s in samples]
    all_ids = [s.charuco_ids for s in samples]

    flags = (
        cv2.CALIB_RATIONAL_MODEL
        | cv2.CALIB_FIX_S1_S2_S3_S4
        | cv2.CALIB_FIX_TAUX_TAUY
    )
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 200, 1e-9)

    extended_fn = getattr(cv2.aruco, "calibrateCameraCharucoExtended", None)
    if extended_fn is not None:
        (rms, mtx, dist, rvecs, tvecs,
         std_intrinsics, std_extrinsics, per_view_errors) = extended_fn(
            charucoCorners=all_corners, charucoIds=all_ids,
            board=board, imageSize=image_size,
            cameraMatrix=None, distCoeffs=None,
            flags=flags, criteria=criteria,
        )
        return {
            "rms": float(rms), "mtx": mtx, "dist": dist,
            "rvecs": rvecs, "tvecs": tvecs,
            "std_intrinsics": std_intrinsics,
            "std_extrinsics": std_extrinsics,
            "per_view_errors": np.asarray(per_view_errors).reshape(-1),
        }

    rms, mtx, dist, rvecs, tvecs = cv2.aruco.calibrateCameraCharuco(
        charucoCorners=all_corners, charucoIds=all_ids,
        board=board, imageSize=image_size,
        cameraMatrix=None, distCoeffs=None,
        flags=flags, criteria=criteria,
    )
    return {
        "rms": float(rms), "mtx": mtx, "dist": dist,
        "rvecs": rvecs, "tvecs": tvecs,
        "std_intrinsics": None, "std_extrinsics": None,
        "per_view_errors": None,
    }


def reject_outliers(samples, calibration, max_view_error_px, trim_fraction):
    per_view = calibration.get("per_view_errors")
    if per_view is None or len(samples) < 8:
        return list(samples)

    indexed = list(enumerate(zip(samples, per_view)))
    kept = [item for item in indexed if float(item[1][1]) <= max_view_error_px]
    if len(kept) < max(8, int(len(samples) * 0.6)):
        kept = indexed

    trimmed = int(len(kept) * trim_fraction)
    if trimmed > 0 and len(kept) - trimmed >= 8:
        kept = sorted(kept, key=lambda x: float(x[1][1]))[:-trimmed]

    keep_idx = {idx for idx, _ in kept}
    return [s for i, s in enumerate(samples) if i in keep_idx]


# --------------------------- I/O ---------------------------------------- #

def save_calibration(out_dir, name, calibration, samples, args):
    out_dir = Path(out_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    npz_path = out_dir / f"{name}.npz"
    meta_path = out_dir / f"{name}.meta.json"
    snap_path = out_dir / f"{name}.snapshot.png"

    np.savez(
        npz_path,
        mtx=calibration["mtx"],
        dist=calibration["dist"],
        rvecs=np.asarray(calibration["rvecs"], dtype=np.float64),
        tvecs=np.asarray(calibration["tvecs"], dtype=np.float64),
    )

    mtx = calibration["mtx"]
    dist = calibration["dist"]
    metadata = {
        "name": name,
        "created_at_epoch_s": time.time(),
        "camera_index": args.camera,
        "preset": args.preset,
        "resolution": [args.width, args.height],
        "dictionary": args.dictionary,
        "board": {
            "squares_x": args.squares_x,
            "squares_y": args.squares_y,
            "square_length_m": args.square_length_m,
            "marker_length_m": args.marker_length_m,
        },
        "capture": {
            "min_markers": args.min_markers,
            "min_charuco": args.min_charuco,
            "min_sharpness": args.min_sharpness,
            "grid_cols": args.grid_cols,
            "grid_rows": args.grid_rows,
            "max_per_cell": args.max_per_cell,
        },
        "results": {
            "rms_reprojection_error_px": calibration["rms"],
            "accepted_views": len(samples),
            "fx": float(mtx[0, 0]),
            "fy": float(mtx[1, 1]),
            "cx": float(mtx[0, 2]),
            "cy": float(mtx[1, 2]),
            "dist_coeffs": [float(x) for x in np.asarray(dist).reshape(-1)],
        },
        "settings_applied": {
            "auto_exposure": args.auto_exposure,
            "exposure": args.exposure,
            "gain": args.gain,
            "auto_wb": args.auto_wb,
            "wb_temperature": args.wb_temperature,
            "autofocus": args.autofocus,
            "focus": args.focus,
            "brightness": args.brightness,
            "contrast": args.contrast,
            "saturation": args.saturation,
            "sharpness": args.sharpness,
            "fourcc": args.fourcc,
        },
    }
    meta_path.write_text(json.dumps(metadata, indent=2), encoding="utf-8")

    if samples:
        cv2.imwrite(str(snap_path), samples[-1].frame_bgr)

    return npz_path, meta_path, snap_path


# --------------------------- CLI ---------------------------------------- #

def parse_args() -> argparse.Namespace:
    repo_root = Path(__file__).resolve().parents[2]
    default_out = repo_root / "training" / "calibration"

    p = argparse.ArgumentParser(description="ChArUco intrinsic calibration for MyCobot project cameras.")

    # camera identity / capture
    p.add_argument("--camera", type=int, default=-1,
                   help="OpenCV camera index (e.g. 0, 2). Required for --source v4l2; ignored for astra.")
    p.add_argument("--name", required=True, help="Output basename: cam_0, cam_3, astra_rgb, etc.")
    p.add_argument("--source", choices=["v4l2", "astra"], default="v4l2",
                   help="Capture source. 'v4l2' = standard /dev/videoN UVC. "
                        "'astra' = Orbbec Astra via teleop/orbbec_capture.py (OpenNI2 + oni_grabber).")
    p.add_argument("--preset", choices=sorted(CAMERA_PRESETS.keys()) + ["none"],
                   default=None,
                   help="Apply a known camera-control preset before capture (default: arducam for "
                        "names starting with 'cam_', astra_rgb for names starting with 'astra').")
    p.add_argument("--backend", choices=sorted(DEFAULT_BACKEND.keys()), default="v4l2")
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fourcc", choices=sorted(DEFAULT_FOURCC.keys()), default=None)

    # board
    p.add_argument("--dictionary", default="DICT_4X4_1000")
    p.add_argument("--squares-x", type=int, default=6)
    p.add_argument("--squares-y", type=int, default=9)
    p.add_argument("--square-length-m", type=float, default=0.030)
    p.add_argument("--marker-length-m", type=float, default=0.024)

    # quality gating
    p.add_argument("--target-samples", type=int, default=35)
    p.add_argument("--min-markers", type=int, default=6)
    p.add_argument("--min-charuco", type=int, default=12)
    p.add_argument("--min-sharpness", type=float, default=80.0)
    p.add_argument("--grid-cols", type=int, default=4)
    p.add_argument("--grid-rows", type=int, default=3)
    p.add_argument("--max-per-cell", type=int, default=6)
    p.add_argument("--capture-interval-s", type=float, default=0.7)
    p.add_argument("--max-view-error-px", type=float, default=1.2)
    p.add_argument("--trim-fraction", type=float, default=0.1)
    p.add_argument("--clahe", action="store_true",
                   help="Apply CLAHE (contrast-limited adaptive histogram equalization) "
                        "to the grayscale image before ArUco detection. Helps low-contrast "
                        "/ noisy sensors (Orbbec Astra RGB).")

    # output
    p.add_argument("--output-dir", type=Path, default=default_out)
    p.add_argument("--manual", action="store_true",
                   help="Capture only on SPACE. Default mode auto-captures.")

    # camera control overrides
    p.add_argument("--auto-exposure", choices=["on", "off"], default=None)
    p.add_argument("--exposure", type=float, default=None)
    p.add_argument("--gain", type=float, default=None)
    p.add_argument("--brightness", type=float, default=None)
    p.add_argument("--contrast", type=float, default=None)
    p.add_argument("--saturation", type=float, default=None)
    p.add_argument("--sharpness", type=float, default=None)
    p.add_argument("--auto-wb", choices=["on", "off"], default=None)
    p.add_argument("--wb-temperature", type=float, default=None)
    p.add_argument("--autofocus", choices=["on", "off"], default=None)
    p.add_argument("--focus", type=float, default=None)

    args = p.parse_args()
    _apply_preset(args)
    return args


def _apply_preset(args: argparse.Namespace) -> None:
    """Resolve --preset, defaulting from --name. Explicit CLI flags always win."""
    if args.preset is None:
        if args.name.startswith("cam_"):
            args.preset = "arducam"
        elif args.name.startswith("astra"):
            args.preset = "astra_rgb"
        else:
            args.preset = "none"

    if args.preset == "none":
        return
    preset = CAMERA_PRESETS[args.preset]
    for key, val in preset.items():
        if getattr(args, key) is None:
            setattr(args, key, val)


# --------------------------- Main loop ---------------------------------- #

def main() -> int:
    args = parse_args()

    if not hasattr(cv2, "aruco"):
        print("OpenCV was built without the aruco module.", file=sys.stderr)
        return 1

    try:
        dictionary_id = getattr(cv2.aruco, args.dictionary)
    except AttributeError:
        print(f"Unknown ArUco dictionary: {args.dictionary}", file=sys.stderr)
        return 1

    dictionary = cv2.aruco.getPredefinedDictionary(dictionary_id)
    board = _build_charuco_board(
        args.squares_x, args.squares_y,
        args.square_length_m, args.marker_length_m,
        dictionary,
    )
    parameters = _create_detector_parameters()

    if args.source == "astra":
        # Use the project's OrbbecCapture wrapper. It auto-spawns oni_grabber
        # and exposes a cv2.VideoCapture-compatible interface.
        repo_root = Path(__file__).resolve().parents[2]
        teleop_path = repo_root / "teleop"
        if str(teleop_path) not in sys.path:
            sys.path.insert(0, str(teleop_path))
        try:
            from orbbec_capture import open_orbbec  # type: ignore
        except ImportError as exc:
            print(f"Could not import orbbec_capture from {teleop_path}: {exc}",
                  file=sys.stderr)
            return 1
        try:
            cap = open_orbbec(auto_spawn=True, open_timeout=10.0)
        except Exception as exc:
            print(f"open_orbbec failed: {exc}", file=sys.stderr)
            return 1
        if not cap.isOpened():
            print("OrbbecCapture failed to open.", file=sys.stderr)
            return 1
        # OrbbecCapture exposes width/height via .get(); refresh args for metadata.
        ow = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        oh = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        if ow > 0 and oh > 0:
            args.width, args.height = ow, oh
        print(f"Astra capture opened — native resolution {ow}x{oh}")
    else:
        if args.camera < 0:
            print("--camera <index> is required for --source v4l2.", file=sys.stderr)
            return 1
        cap = cv2.VideoCapture(args.camera, DEFAULT_BACKEND[args.backend])
        if not cap.isOpened():
            print(f"Could not open camera index {args.camera}.", file=sys.stderr)
            return 1
        _apply_camera_settings(cap, args)
        _print_camera_properties(cap)

    accepted: List[CalibrationSample] = []
    last_auto_capture = 0.0
    show_overlay = True
    auto_solved = False  # set True after auto-save fires so we don't loop

    def _solve_and_save(samples: List[CalibrationSample]) -> bool:
        """Run calibration + outlier rejection + save. Returns True on success."""
        if len(samples) < max(12, args.target_samples // 2):
            print(f"Not enough views: {len(samples)} collected, need >= {max(12, args.target_samples // 2)}.")
            return False
        image_size = samples[0].image_size
        calibration = calibrate_charuco(samples, board, image_size)
        filtered = reject_outliers(
            samples, calibration,
            max_view_error_px=args.max_view_error_px,
            trim_fraction=args.trim_fraction,
        )
        if len(filtered) != len(samples):
            print(f"Outlier rejection: {len(samples)} -> {len(filtered)} views, re-solving.")
            calibration = calibrate_charuco(filtered, board, image_size)
            samples[:] = filtered

        npz, meta, snap = save_calibration(args.output_dir, args.name, calibration, samples, args)
        mtx = calibration["mtx"]
        print()
        print("=" * 60)
        print(f"Calibration saved for '{args.name}'.")
        print(f"  Output:    {npz}")
        print(f"  Metadata:  {meta}")
        print(f"  Snapshot:  {snap}")
        print(f"  Views:     {len(samples)}")
        print(f"  RMS px:    {calibration['rms']:.4f}")
        print(f"  fx={mtx[0,0]:.2f}  fy={mtx[1,1]:.2f}  cx={mtx[0,2]:.2f}  cy={mtx[1,2]:.2f}")
        print("  dist:", np.asarray(calibration["dist"]).reshape(-1))
        print("=" * 60)
        return True

    print()
    print(f"=== Calibrating '{args.name}' (cv index {args.camera}, preset {args.preset}) ===")
    print("Move the ChArUco board across the entire frame, vary distance + tilt.")
    print(f"Auto-save will fire automatically once {args.target_samples} views are collected.")
    print("Controls: SPACE accept | C calibrate+save | R reset | P props | I overlay | Q quit")
    print()

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Camera frame read failed.", file=sys.stderr)
                return 1

            sample, preview = detect_charuco_sample(
                frame=frame,
                dictionary=dictionary, board=board, parameters=parameters,
                min_markers=args.min_markers, min_charuco=args.min_charuco,
                grid_cols=args.grid_cols, grid_rows=args.grid_rows,
                clahe=args.clahe,
            )

            now = time.time()
            status = "No valid ChArUco board detected"
            can_accept = False

            if sample is not None:
                diverse = sample_is_diverse(
                    sample, accepted,
                    min_time_delta_s=args.capture_interval_s,
                    max_per_cell=args.max_per_cell,
                )
                can_accept = sample.sharpness >= args.min_sharpness and diverse

                status = (
                    f"markers={sample.marker_count} charuco={sample.charuco_count} "
                    f"sharpness={sample.sharpness:.0f} cell={sample.coverage_cell}"
                )
                if sample.sharpness < args.min_sharpness:
                    status += " | rejected: blurry"
                elif not diverse:
                    status += " | rejected: low diversity"
                else:
                    status += " | ready"

                if (not args.manual and can_accept
                        and now - last_auto_capture >= args.capture_interval_s):
                    accepted.append(sample)
                    last_auto_capture = now
                    print(f"[auto] view {len(accepted)}/{args.target_samples} "
                          f"cell={sample.coverage_cell} sharp={sample.sharpness:.0f}")

            # Auto-save once target is reached (no keypress needed).
            if not auto_solved and len(accepted) >= args.target_samples:
                print(f"\nTarget {args.target_samples} reached — auto-saving calibration.")
                if _solve_and_save(accepted):
                    auto_solved = True
                    return 0

            if show_overlay:
                cv2.putText(
                    preview,
                    f"{args.name}  accepted: {len(accepted)} / {args.target_samples}",
                    (20, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 255, 0) if len(accepted) >= args.target_samples else (0, 200, 200), 2,
                )
                cv2.putText(
                    preview, status, (20, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                    (0, 255, 255) if can_accept else (0, 128, 255), 2,
                )
                cv2.putText(
                    preview, "SPACE accept | C calibrate | R reset | Q quit",
                    (20, preview.shape[0] - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 2,
                )

            cv2.imshow(f"Calibrate {args.name}", preview)
            key = cv2.waitKey(1) & 0xFF

            if key in (27, ord("q"), ord("Q")):
                if not auto_solved and len(accepted) >= max(12, args.target_samples // 2):
                    print(f"Quit pressed with {len(accepted)} views — saving before exit.")
                    _solve_and_save(accepted)
                else:
                    print("Quit without saving.")
                return 0

            if key in (ord("r"), ord("R")):
                accepted.clear()
                last_auto_capture = 0.0
                print("Accepted views cleared.")

            if key in (ord("i"), ord("I")):
                show_overlay = not show_overlay

            if key in (ord("p"), ord("P")):
                _print_camera_properties(cap)

            if key == ord(" "):
                if sample is None:
                    print("No valid ChArUco detection in current frame.")
                elif sample.sharpness < args.min_sharpness:
                    print(f"Rejected: sharpness {sample.sharpness:.1f} < {args.min_sharpness:.1f}.")
                elif not sample_is_diverse(sample, accepted,
                                           args.capture_interval_s, args.max_per_cell):
                    print("Rejected: too similar to recent samples (move the board).")
                else:
                    accepted.append(sample)
                    last_auto_capture = now
                    print(f"Accepted view {len(accepted)} (sharpness {sample.sharpness:.1f}, cell {sample.coverage_cell}).")

            if key in (ord("c"), ord("C")):
                if _solve_and_save(accepted):
                    return 0

    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    raise SystemExit(main())
