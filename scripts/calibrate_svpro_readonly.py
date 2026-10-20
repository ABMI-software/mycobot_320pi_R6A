#!/usr/bin/env python3
"""Calibrate SVPRO from saved images only; never opens a camera or a robot.

The output is a candidate, never a production calibration. Four distinct table
markers must be detected in EVERY supplied frame. The leave-one-marker-out
check is conditional on marker yaw/size estimated using the initial full fit;
it is a consistency diagnostic, not independent ground truth. Calibration on a
plane does not validate localization at other heights.

Example:
  .venv/bin/python scripts/calibrate_svpro_readonly.py \
    --images 'docs/session/svpro_raw_*.png' --out /tmp/svpro_candidate.yaml
"""
from __future__ import annotations

import argparse
import glob
import json
from pathlib import Path

import cv2
import numpy as np
import yaml

REPO = Path(__file__).resolve().parents[1]
CALIB = REPO / 'training/calibration'
IDS = (19, 23, 25, 26)


def intrinsics(resolution, stem=CALIB / 'cam_2'):
    meta = json.loads(stem.with_suffix('.meta.json').read_text())
    with np.load(stem.with_suffix('.npz'), allow_pickle=False) as data:
        K = np.asarray(data['mtx'], dtype=float).copy()
        dist = np.asarray(data['dist'], dtype=float).reshape(-1)
    native = tuple(map(int, meta['resolution']))
    if K.shape != (3, 3) or not np.isfinite(K).all() or not np.isfinite(dist).all():
        raise ValueError('invalid camera intrinsics')
    if K[0, 0] <= 0 or K[1, 1] <= 0 or dist.size not in (4, 5, 8, 12, 14):
        raise ValueError('invalid focal length or distortion coefficient count')
    K[0] *= resolution[0] / native[0]
    K[1] *= resolution[1] / native[1]
    return K, dist, native


def load_markers(path):
    spec = yaml.safe_load(Path(path).read_text())
    units = spec.get('units')
    if units not in ('mm', 'm'):
        raise ValueError('marker units must be explicitly mm or m')
    if spec.get('frame_id') != 'base_link':
        raise ValueError('marker frame must be base_link')
    scale = 0.001 if units == 'mm' else 1.0
    points = {int(i): np.asarray(p, dtype=float) * scale
              for i, p in spec['markers'].items()}
    if any(i not in points for i in IDS):
        raise ValueError(f'all marker IDs {IDS} are required')
    points = {i: points[i] for i in IDS}
    array = np.asarray(list(points.values()))
    if array.shape != (4, 3) or not np.isfinite(array).all():
        raise ValueError('invalid marker coordinates')
    if np.ptp(array[:, 2]) > 1e-6:
        raise ValueError('markers must lie on one horizontal plane')
    if np.linalg.matrix_rank(array[:, :2] - array[:, :2].mean(axis=0)) < 2:
        raise ValueError('marker centres are collinear')
    return points


def detect(image, mode='gray', scale=1.0):
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError('detect-scale must be positive and finite')
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    if mode == 'clahe':
        gray = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8)).apply(gray)
    elif mode != 'gray':
        raise ValueError('detect-mode must be gray or clahe')
    if scale != 1.0:
        gray = cv2.resize(gray, None, fx=scale, fy=scale, interpolation=cv2.INTER_CUBIC)
    dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
    legacy = hasattr(cv2.aruco, 'DetectorParameters_create')
    params = (cv2.aruco.DetectorParameters_create() if legacy
              else cv2.aruco.DetectorParameters())
    params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
    params.adaptiveThreshWinSizeMin = 3
    params.adaptiveThreshWinSizeMax = 53
    params.adaptiveThreshWinSizeStep = 4
    if legacy:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
    else:
        corners, ids, _ = cv2.aruco.ArucoDetector(dictionary, params).detectMarkers(gray)
    if ids is None:
        return {}, []
    ids = list(map(int, ids.ravel()))
    duplicates = sorted(i for i in set(ids) if ids.count(i) > 1)
    return {i: np.asarray(q, float).reshape(4, 2) / scale
            for i, q in zip(ids, corners) if i in IDS and i not in duplicates}, duplicates


def projective_center(quad, K, dist):
    """Intersect diagonals after undistortion, then return a raw image pixel."""
    q = cv2.undistortPoints(np.asarray(quad, float).reshape(4, 1, 2), K, dist).reshape(4, 2)
    h = np.column_stack((q, np.ones(4)))
    intersection = np.cross(np.cross(h[0], h[2]), np.cross(h[1], h[3]))
    if abs(intersection[2]) < 1e-12:
        raise ValueError('degenerate marker diagonal intersection')
    normalised = intersection / intersection[2]
    uv, _ = cv2.projectPoints(normalised.reshape(1, 3), np.zeros(3), np.zeros(3), K, dist)
    return uv.reshape(2)


def project(points, T, K, dist):
    rv = cv2.Rodrigues(T[:3, :3])[0]
    return cv2.projectPoints(np.asarray(points, float), rv, T[:3, 3], K, dist)[0].reshape(-1, 2)


def pose(obj, pixels, K, dist):
    obj, pixels = np.asarray(obj, float), np.asarray(pixels, float)
    result = cv2.solvePnPGeneric(obj, pixels, K, dist, flags=cv2.SOLVEPNP_IPPE)
    candidates = []
    for rv, tv in zip(result[1], result[2]):
        rv, tv = cv2.solvePnPRefineLM(obj, pixels, K, dist, rv, tv)
        T = np.eye(4)
        T[:3, :3] = cv2.Rodrigues(rv)[0]
        T[:3, 3] = tv.ravel()
        errors = np.linalg.norm(project(obj, T, K, dist) - pixels, axis=1)
        physical = not physical_reasons(T, obj)
        candidates.append((not physical, float(np.mean(errors ** 2)), T))
    if not candidates:
        raise ValueError('PnP returned no solution')
    return min(candidates, key=lambda item: item[:2])[2]


def physical_reasons(T, points):
    if T.shape != (4, 4) or not np.isfinite(T).all():
        return ['camera matrix is not finite 4x4']
    R = T[:3, :3]
    reasons = []
    if not np.allclose(R.T @ R, np.eye(3), atol=1e-6) or abs(np.linalg.det(R) - 1) > 1e-6:
        reasons.append('camera rotation is not proper orthonormal')
    camera = -R.T @ T[:3, 3]
    if camera[2] <= float(np.mean(points[:, 2])):
        reasons.append('camera is not above the marker plane')
    if np.min((points @ R.T + T[:3, 3])[:, 2]) <= 0:
        reasons.append('some marker points are behind the camera')
    return reasons


def to_plane(uv, T, K, dist, plane_z):
    n = cv2.undistortPoints(np.asarray(uv, float).reshape(1, 1, 2), K, dist).reshape(2)
    C = -T[:3, :3].T @ T[:3, 3]
    ray = T[:3, :3].T @ np.r_[n, 1.0]
    if abs(ray[2]) < 1e-10:
        raise ValueError('projection ray parallel to marker plane')
    distance = (plane_z - C[2]) / ray[2]
    if distance <= 0:
        raise ValueError('marker-plane intersection behind camera')
    return C + distance * ray


def fit(detections, markers, K, dist):
    """Fit 16 corners/frame with inferred square sizes and yaw; report conditional LOO."""
    centres = {i: np.mean([projective_center(d[i], K, dist) for d in detections], axis=0)
               for i in IDS}
    T0 = pose([markers[i] for i in IDS], [centres[i] for i in IDS], K, dist)
    geometry, measured = {}, {}
    for i in IDS:
        quad = np.mean([d[i] for d in detections], axis=0)
        plane = np.asarray([to_plane(p, T0, K, dist, markers[i][2]) for p in quad])
        edge = plane[1, :2] - plane[0, :2]
        ex = edge / np.linalg.norm(edge)
        ey = np.array([-ex[1], ex[0]])
        side = float(np.mean(np.linalg.norm(np.roll(plane, -1, axis=0) - plane, axis=1)))
        offsets = np.array([[-1, 1], [1, 1], [1, -1], [-1, -1]]) * side / 2
        xyz = np.tile(markers[i], (4, 1))
        xyz[:, :2] += offsets @ np.vstack((ex, ey))
        geometry[i] = xyz
        measured[i] = {'size_mm': side * 1000, 'yaw_deg': float(np.degrees(np.arctan2(ex[1], ex[0])))}
    obj = np.concatenate([geometry[i] for d in detections for i in IDS])
    img = np.concatenate([d[i] for d in detections for i in IDS])
    membership = np.tile(np.repeat(IDS, 4), len(detections))
    T = pose(obj, img, K, dist)
    errors = np.linalg.norm(project(obj, T, K, dist) - img, axis=1)
    loo = []
    for i in IDS:
        keep = membership != i
        held = pose(obj[keep], img[keep], K, dist)
        vu = to_plane(centres[i], held, K, dist, markers[i][2])
        reproj = np.linalg.norm(project(obj[~keep], held, K, dist) - img[~keep], axis=1)
        loo.append({'marker': i, 'plane_error_mm': float(np.linalg.norm(vu[:2] - markers[i][:2]) * 1000),
                    'reproj_px': float(np.sqrt(np.mean(reproj ** 2)))})
    dispersion = {i: float(np.sqrt(np.mean(np.sum((np.asarray([projective_center(d[i], K, dist)
                  for d in detections]) - centres[i]) ** 2, axis=1)))) for i in IDS}
    return {'T_cam_world': T.tolist(), 'T_world_cam': np.linalg.inv(T).tolist(),
            'camera_position_base_m': np.linalg.inv(T)[:3, 3].tolist(),
            'rms_reproj_px': float(np.sqrt(np.mean(errors ** 2))),
            'validation_leave_one_out': loo, 'marker_geometry_estimated': measured,
            'centre_interframe_rms_px': dispersion}, physical_reasons(T, obj)


def expand_images(patterns):
    paths = []
    for pattern in patterns:
        matches = sorted(glob.glob(pattern))
        if not matches:
            raise ValueError(f'no image matches {pattern}')
        for match in matches:
            path = Path(match).resolve()
            if path not in paths:
                paths.append(path)
    return paths


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--images', nargs='+', required=True, help='saved images or quoted glob patterns')
    parser.add_argument('--markers', type=Path, default=CALIB / 'planche_actuelle.yaml')
    parser.add_argument('--out', type=Path, required=True, help='candidate YAML, outside training/calibration')
    parser.add_argument('--overlay', type=Path, help='annotated first image')
    parser.add_argument('--preprocess', '--detect-mode', dest='detect_mode',
                        choices=('gray', 'clahe'), default='clahe',
                        help='CLAHE at scale 2 decoded all four tags on the 29/09 saved sequence')
    parser.add_argument('--detect-scale', type=float, default=2.0)
    parser.add_argument('--rms-max-px', type=float, default=1.5)
    parser.add_argument('--loo-max-mm', type=float, default=5.0)
    parser.add_argument('--reference-base-confirmed', action='store_true',
                        help='human explicitly confirmed marker positions in the current robot base')
    args = parser.parse_args(argv)
    if CALIB.resolve() in args.out.resolve().parents:
        parser.error('candidate output must be outside the production calibration directory')
    if any(not np.isfinite(v) or v <= 0 for v in (args.detect_scale, args.rms_max_px, args.loo_max_mm)):
        parser.error('scale and quality thresholds must be positive finite values')
    try:
        paths = expand_images(args.images)
        if args.out.resolve() in paths or (args.overlay and args.overlay.resolve() in paths):
            raise ValueError('output may not overwrite an input image')
        if args.out.resolve() == args.markers.resolve():
            raise ValueError('output may not overwrite marker reference')
        if args.overlay and (CALIB.resolve() in args.overlay.resolve().parents or
                             args.overlay.resolve() == args.out.resolve()):
            raise ValueError('overlay must be distinct from YAML and production calibration')
        markers = load_markers(args.markers)
        images = [cv2.imread(str(p)) for p in paths]
        if any(im is None for im in images):
            raise ValueError('unreadable input image')
        shapes = {im.shape[:2] for im in images}
        if len(shapes) != 1:
            raise ValueError('all images must have the same resolution')
        h, w = images[0].shape[:2]
        K, dist, native = intrinsics((w, h))
        detections, counts, reasons = [], {i: 0 for i in IDS}, []
        per_frame = []
        for path, image in zip(paths, images):
            found, duplicates = detect(image, args.detect_mode, args.detect_scale)
            detections.append(found)
            missing = sorted(set(IDS) - set(found))
            per_frame.append({'path': str(path), 'detected_ids': sorted(found),
                              'missing_ids': missing, 'duplicate_ids_rejected': duplicates})
            for i in found:
                counts[i] += 1
            if missing:
                reasons.append(f'{path.name}: missing or duplicated required IDs {missing}')
        payload = {'camera': 'svpro', 'intrinsics_stem': 'cam_2', 'resolution': [w, h],
                   'intrinsics_native_resolution': list(native), 'frame_id': 'base_link',
                   'units': 'm', 'K': K.tolist(), 'dist': dist.tolist(),
                   'reference_file': str(args.markers.resolve()),
                   'reference_base_confirmed': bool(args.reference_base_confirmed),
                   'markers_used': list(IDS), 'marker_detection_counts': counts,
                   'capture': {'images': [str(p) for p in paths], 'frame_count': len(paths),
                               'detection_mode': args.detect_mode, 'detection_scale': args.detect_scale,
                               'per_frame': per_frame},
                   'validation': {'thresholds': {'rms_max_px': args.rms_max_px, 'loo_max_mm': args.loo_max_mm},
                                  'loo_is_independent_ground_truth': False,
                                  'limitations': 'LOO conditional on yaw/size inferred using all marker centres. '
                                  'No independent ground truth and no validation outside marker plane. '
                                  'Passing geometry does not confirm the reference in the current robot base.'}}
        if not reasons:
            try:
                fitted, physical = fit(detections, markers, K, dist)
                payload.update(fitted)
                reasons.extend(physical)
                rms = fitted['rms_reproj_px']
                errors = [row['plane_error_mm'] for row in fitted['validation_leave_one_out']]
                if not np.isfinite(rms) or not np.isfinite(errors).all():
                    reasons.append('quality metrics contain non-finite values')
                if rms > args.rms_max_px:
                    reasons.append(f'RMS {rms:.3f} px exceeds {args.rms_max_px:.3f}')
                worst = max(errors)
                if worst > args.loo_max_mm:
                    reasons.append(f'conditional LOO {worst:.3f} mm exceeds {args.loo_max_mm:.3f}')
            except (ValueError, cv2.error, np.linalg.LinAlgError) as exc:
                reasons.append(f'pose or projection failed: {exc}')
        payload['validation'].update(status='failed' if reasons else 'passed', reasons=reasons)
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(yaml.safe_dump(payload, sort_keys=False, allow_unicode=True))
        if args.overlay:
            overlay = images[0].copy()
            for i, q in detections[0].items():
                cv2.polylines(overlay, [q.astype(np.int32)], True, (0, 220, 0), 2)
                uv = projective_center(q, K, dist)
                cv2.putText(overlay, str(i), tuple(uv.astype(int)), cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 220, 0), 1)
            if 'T_cam_world' in payload:
                projected = project(np.array([markers[i] for i in IDS]), np.array(payload['T_cam_world']), K, dist)
                for uv in projected:
                    cv2.drawMarker(overlay, tuple(np.round(uv).astype(int)), (255, 255, 0), cv2.MARKER_CROSS, 12, 1)
            args.overlay.parent.mkdir(parents=True, exist_ok=True)
            if not cv2.imwrite(str(args.overlay), overlay):
                raise ValueError('unable to write overlay')
        print(json.dumps({'candidate': str(args.out), 'validation': payload['validation'],
                          'reference_base_confirmed': payload['reference_base_confirmed'],
                          'marker_detection_counts': counts,
                          'rms_reproj_px': payload.get('rms_reproj_px'),
                          'validation_leave_one_out': payload.get('validation_leave_one_out')}, ensure_ascii=False))
        return 2 if reasons else 0
    except (OSError, ValueError, KeyError, cv2.error, np.linalg.LinAlgError) as exc:
        parser.exit(2, f'calibration failed: {exc}\n')


if __name__ == '__main__':
    raise SystemExit(main())
