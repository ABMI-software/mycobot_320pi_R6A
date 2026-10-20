#!/usr/bin/env python3
"""YOLO26 sur image réelle, projection base puis comparaison GT indépendante.

Aucun accès caméra, bridge ou robot. Deux commandes séparées : predict, compare.
Z est imposé par un fichier de hauteurs indépendant, jamais mesuré par YOLO.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
from pathlib import Path

import cv2
import numpy as np
import yaml

ROOT = Path(__file__).resolve().parents[1]
CALIB_DIR = ROOT / 'training' / 'calibration'
DEFAULT_WEIGHTS = ROOT / 'runs/detect/training/yolo/runs/pieces_v5_yolo26s/weights/best.pt'
POINT_FIELDS = ['object_id', 'class_name', 'x_mm', 'y_mm', 'z_mm', 'point_definition']
PRED_FIELDS = ['detection_id', *POINT_FIELDS, 'confidence', 'u_px', 'v_px',
               'x_min', 'y_min', 'x_max', 'y_max', 'status', 'reason', 'mode',
               'ray_gap_mm', 'ray_angle_deg', 'reprojection_a_px', 'reprojection_b_px',
               'provisional_reference']
ERROR_FIELDS = ['object_id', 'detection_id', 'gt_class', 'pred_class', 'class_correct',
                'point_definition', 'association', 'dx_mm', 'dy_mm', 'dz_mm',
                'error_xy_mm', 'error_xyz_mm']
MODE = 'bbox_center_on_known_plane_z_not_measured'


def finite(value, name):
    if isinstance(value, bool):
        raise ValueError(f'{name}: nombre attendu')
    number = float(value)
    if not np.isfinite(number):
        raise ValueError(f'{name}: valeur non finie')
    return number


def fingerprint(path):
    path = Path(path).resolve()
    return {'path': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest()}


def write_csv(path, rows, fields):
    with Path(path).open('w', newline='') as stream:
        writer = csv.DictWriter(stream, fieldnames=fields)
        writer.writeheader()
        writer.writerows(rows)


def write_json(path, data):
    Path(path).write_text(json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False) + '\n')


def load_geometry(path, width, height, max_loo_mm=5.0):
    """Calibration SVPRO validée LOO; translations en mètres, K rescalée."""
    data = yaml.safe_load(Path(path).read_text())
    camera = data.get('camera')
    if camera not in ('svpro', 'arducam') or data.get('frame_id') != 'base_link':
        raise ValueError('calibration attendue: camera svpro/arducam, frame_id=base_link')
    if data.get('validation', {}).get('status') != 'passed':
        raise ValueError('calibration refusée: validation.status doit être passed')
    if list(data.get('resolution', [])) != [width, height]:
        raise ValueError('résolution image différente de la calibration extrinsèque')
    loo = data.get('validation_leave_one_out', [])
    if not isinstance(loo, list) or len(loo) < 4:
        raise ValueError('calibration sans validation leave-one-out des 4 marqueurs')
    errors = [finite(item['plane_error_mm'], 'plane_error_mm') for item in loo]
    if min(errors) < 0 or max(errors) > finite(max_loo_mm, 'max_loo_mm'):
        raise ValueError(f'calibration refusée: LOO max {max(errors):.2f} mm > {max_loo_mm:g} mm')
    T = np.asarray(data['T_cam_world'], dtype=float)
    if T.shape != (4, 4) or not np.isfinite(T).all():
        raise ValueError('T_cam_world doit être une matrice 4x4 finie')
    R = T[:3, :3]
    if (not np.allclose(T[3], [0, 0, 0, 1], atol=1e-6)
            or not np.allclose(R.T @ R, np.eye(3), atol=1e-5)
            or not np.isclose(np.linalg.det(R), 1.0, atol=1e-5)):
        raise ValueError('T_cam_world ne décrit pas une transformation rigide')
    stem = data.get('intrinsics_stem')
    if stem != {'svpro': 'cam_2', 'arducam': 'cam_3'}[camera]:
        raise ValueError('intrinsics_stem incompatible avec la caméra')
    intrinsics_path = CALIB_DIR / f'{stem}.meta.json'
    meta = json.loads(intrinsics_path.read_text())
    iw, ih = np.asarray(meta['resolution'], float)
    if not np.isfinite([iw, ih]).all() or min(iw, ih) <= 0:
        raise ValueError('résolution intrinsèque invalide')
    if not np.isclose(width / height, iw / ih):
        raise ValueError('ratio image différent de la calibration intrinsèque')
    p = meta['results']
    sx, sy = width / iw, height / ih
    K = np.array([[p['fx'] * sx, 0, p['cx'] * sx],
                  [0, p['fy'] * sy, p['cy'] * sy], [0, 0, 1]], float)
    dist = np.asarray(p.get('dist_coeffs', []), float)
    if not np.isfinite(K).all() or min(K[0, 0], K[1, 1]) <= 0 or not np.isfinite(dist).all():
        raise ValueError('intrinsèques invalides')
    return K, dist, T, {'camera': camera, 'intrinsics': fingerprint(intrinsics_path),
                         'provisional_reference': data.get('reference_base_confirmed') is not True,
                         'loo_max_mm': max(errors), 'loo_limit_mm': max_loo_mm,
                         'validation_scope': 'calibration_on_table_plane_only'}


def project_on_plane(uv, z_mm, K, dist, T_cam_world):
    """Centre boîte dédistordu -> intersection rayon/plan base, résultat mm."""
    uv = np.asarray(uv, float)
    if uv.shape != (2,) or not np.isfinite(uv).all():
        raise ValueError('pixel invalide')
    z_mm = finite(z_mm, 'hauteur')
    normalized = cv2.undistortPoints(uv.reshape(1, 1, 2), K, dist).reshape(2)
    R, t = T_cam_world[:3, :3], T_cam_world[:3, 3]
    origin = -R.T @ t
    direction = R.T @ np.r_[normalized, 1.0]
    if not np.isfinite(direction).all() or abs(direction[2]) / np.linalg.norm(direction) < 1e-6:
        raise ValueError('rayon parallèle ou trop proche de l’horizon')
    scale = (z_mm / 1000 - origin[2]) / direction[2]
    if not np.isfinite(scale) or scale <= 0:
        raise ValueError('intersection derrière la caméra')
    point = (origin + scale * direction) * 1000
    if not np.isfinite(point).all():
        raise ValueError('intersection non finie')
    return point


def detect(image, weights, confidence):
    """Même suppression par recouvrement que le service réel, sans ses imports."""
    if not Path(weights).is_file():
        raise ValueError(f'poids locaux absents: {weights}')
    from ultralytics import YOLO
    model = YOLO(str(weights))
    result = model.predict(image, conf=confidence, imgsz=640, verbose=False)[0]
    detections = []
    for box in sorted(result.boxes, key=lambda b: -float(b.conf.item())):
        xyxy = np.asarray(box.xyxy[0].cpu(), float).astype(int).astype(float)
        if not np.isfinite(xyxy).all() or np.any(xyxy[2:] <= xyxy[:2]):
            continue
        duplicate = False
        for previous in detections:
            other = previous['box']
            intersection = np.prod(np.maximum(0, np.minimum(xyxy[2:], other[2:])
                                                - np.maximum(xyxy[:2], other[:2])))
            denominator = min(np.prod(xyxy[2:] - xyxy[:2]), np.prod(other[2:] - other[:2]))
            if intersection / denominator > 0.5:
                duplicate = True
                break
        if not duplicate:
            detections.append({'class_name': model.names[int(box.cls.item())],
                               'confidence': float(box.conf.item()), 'box': xyxy})
    return detections


def predict(args):
    image = cv2.imread(str(args.image))
    if image is None:
        raise ValueError(f'image illisible: {args.image}')
    height, width = image.shape[:2]
    K, dist, T, calibration_info = load_geometry(args.calibration, width, height, args.max_loo_mm)
    heights_data = json.loads(args.heights.read_text())
    if not isinstance(heights_data, dict) or not heights_data:
        raise ValueError('heights doit être un objet JSON classe -> Z du point en mm')
    heights = {name: finite(value, f'hauteur {name}') for name, value in heights_data.items()}
    identities = {} if args.ids is None else json.loads(args.ids.read_text())
    if not isinstance(identities, dict) or any(not isinstance(v, str) or not v.strip()
                                             for v in identities.values()):
        raise ValueError('ids doit associer detection_id à object_id non vide')
    if len(set(identities.values())) != len(identities):
        raise ValueError('object_id répété dans le fichier ids')
    rows, overlay = [], image.copy()
    for index, detection in enumerate(detect(image, args.weights, args.confidence), 1):
        a, b, c, d = detection['box']
        uv = ((a + c) / 2, (b + d) / 2)
        ident, name = f'det{index:04d}', detection['class_name']
        row = {key: '' for key in PRED_FIELDS}
        row.update(detection_id=ident, object_id=identities.get(ident, ''), class_name=name,
                   confidence=detection['confidence'], point_definition=args.point_definition,
                   u_px=uv[0], v_px=uv[1], x_min=a, y_min=b, x_max=c, y_max=d,
                   mode=MODE, status='ok', provisional_reference=calibration_info['provisional_reference'])
        try:
            if name not in heights:
                raise ValueError('hauteur indépendante absente pour cette classe')
            xyz = project_on_plane(uv, heights[name], K, dist, T)
            row.update(zip(['x_mm', 'y_mm', 'z_mm'], xyz.tolist()))
        except ValueError as exc:
            row.update(status='rejected', reason=str(exc))
        rows.append(row)
        color = (30, 210, 30) if row['status'] == 'ok' else (0, 100, 255)
        cv2.rectangle(overlay, (int(a), int(b)), (int(c), int(d)), color, 2)
        cv2.putText(overlay, f'{ident} {name} {row["status"]}', (int(a), max(14, int(b) - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
    unknown_ids = set(identities) - {row['detection_id'] for row in rows}
    if unknown_ids:
        raise ValueError(f'detection_id inconnus: {sorted(unknown_ids)}')
    args.out_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.out_dir / 'predictions.csv', rows, PRED_FIELDS)
    if not cv2.imwrite(str(args.out_dir / 'predictions.png'), overlay):
        raise ValueError('échec écriture aperçu')
    write_json(args.out_dir / 'prediction_run.json', {
        'mode': MODE, 'frame_id': 'base_link', 'units': 'mm',
        'point_definition': args.point_definition, 'pixel_estimator': 'bbox_center',
        'z_measured': False, 'image': fingerprint(args.image),
        'calibration': fingerprint(args.calibration), 'heights': fingerprint(args.heights),
        'weights': fingerprint(args.weights), 'ids': fingerprint(args.ids) if args.ids else None,
        'confidence_threshold': args.confidence, 'detections': len(rows),
        'localized': sum(row['status'] == 'ok' for row in rows), **calibration_info})
    print(f'{len(rows)} détections -> {args.out_dir / "predictions.csv"}; Z imposé, non mesuré')


def camera_ray(uv, K, dist, T):
    normalized = cv2.undistortPoints(np.asarray(uv, float).reshape(1, 1, 2), K, dist).reshape(2)
    R, t = T[:3, :3], T[:3, 3]
    direction = R.T @ np.r_[normalized, 1.0]
    direction /= np.linalg.norm(direction)
    return -R.T @ t, direction


def triangulate_points(uv_a, geometry_a, uv_b, geometry_b, min_angle_deg=2.0, max_gap_mm=20.0):
    """Milieu du segment entre deux rayons; aucun plan Z ni GT."""
    origin_a, direction_a = camera_ray(uv_a, *geometry_a)
    origin_b, direction_b = camera_ray(uv_b, *geometry_b)
    if not np.isfinite([origin_a, direction_a, origin_b, direction_b]).all():
        raise ValueError('rayons non finis')
    angle = np.degrees(np.arccos(np.clip(abs(np.dot(direction_a, direction_b)), 0, 1)))
    if angle < min_angle_deg:
        raise ValueError(f'rayons quasi parallèles: {angle:.3f} deg')
    distances = np.linalg.lstsq(np.column_stack([direction_a, -direction_b]),
                                origin_b - origin_a, rcond=None)[0]
    if not np.isfinite(distances).all() or np.any(distances <= 0):
        raise ValueError('intersection derrière une caméra')
    point_a = origin_a + distances[0] * direction_a
    point_b = origin_b + distances[1] * direction_b
    gap = float(np.linalg.norm(point_a - point_b) * 1000)
    if gap > max_gap_mm:
        raise ValueError(f'écart des rayons trop grand: {gap:.2f} mm')
    xyz_m = (point_a + point_b) / 2
    errors = []
    for uv, (K, dist, T) in [(uv_a, geometry_a), (uv_b, geometry_b)]:
        if (T[:3, :3] @ xyz_m + T[:3, 3])[2] <= 0:
            raise ValueError('point reconstruit derrière une caméra')
        rvec, _ = cv2.Rodrigues(T[:3, :3])
        pixels, _ = cv2.projectPoints(xyz_m.reshape(1, 3), rvec, T[:3, 3], K, dist)
        errors.append(float(np.linalg.norm(pixels.reshape(2) - uv)))
    return xyz_m * 1000, {'ray_gap_mm': gap, 'ray_angle_deg': float(angle),
                         'reprojection_a_px': errors[0], 'reprojection_b_px': errors[1]}


def triangulate_images(args):
    images, geometries, metadata, observations = [], [], [], []
    for image_path, calibration in [(args.image_a, args.calibration_a),
                                    (args.image_b, args.calibration_b)]:
        image = cv2.imread(str(image_path))
        if image is None:
            raise ValueError(f'image illisible: {image_path}')
        K, dist, T, info = load_geometry(calibration, image.shape[1], image.shape[0], args.max_loo_mm)
        images.append(image)
        geometries.append((K, dist, T))
        metadata.append({**info, 'image': fingerprint(image_path), 'calibration': fingerprint(calibration)})
        detections = detect(image, args.weights, args.confidence)
        counts = {}
        for detection in detections:
            name = detection['class_name']
            counts[name] = counts.get(name, 0) + 1
        duplicates = [name for name, count in counts.items() if count > 1]
        if duplicates:
            raise ValueError(f'classes ambiguës dans une vue: {duplicates}; une instance par classe requise')
        observations.append({detection['class_name']: detection for detection in detections})
    if metadata[0]['camera'] == metadata[1]['camera']:
        raise ValueError('deux caméras distinctes requises')
    rows = []
    provisional = any(item['provisional_reference'] for item in metadata)
    for index, name in enumerate(sorted(set(observations[0]) | set(observations[1])), 1):
        row = {key: '' for key in PRED_FIELDS}
        row.update(detection_id=f'det{index:04d}', class_name=name, point_definition=args.point_definition,
                   mode='two_view_bbox_centres_correspond', status='ok', provisional_reference=provisional)
        try:
            if any(name not in observation for observation in observations):
                raise ValueError('classe absente de l’une des deux vues')
            pair = [observation[name] for observation in observations]
            uv = [(d['box'][:2] + d['box'][2:]) / 2 for d in pair]
            row.update(confidence=min(d['confidence'] for d in pair), u_px=uv[0][0], v_px=uv[0][1])
            row.update(zip(['x_min', 'y_min', 'x_max', 'y_max'], pair[0]['box']))
            xyz, diagnostics = triangulate_points(uv[0], geometries[0], uv[1], geometries[1],
                                                   args.min_ray_angle_deg, args.max_ray_gap_mm)
            row.update(diagnostics)
            if max(diagnostics['reprojection_a_px'], diagnostics['reprojection_b_px']) > args.max_reprojection_px:
                raise ValueError('erreur de reprojection trop grande')
            row.update(zip(['x_mm', 'y_mm', 'z_mm'], xyz.tolist()))
        except ValueError as exc:
            row.update(status='rejected', reason=str(exc))
        rows.append(row)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.out_dir / 'predictions.csv', rows, PRED_FIELDS)
    for index, (image, observation) in enumerate(zip(images, observations)):
        overlay = image.copy()
        for row in rows:
            detection = observation.get(row['class_name'])
            if detection is None:
                continue
            a, b, c, d = map(int, detection['box'])
            color = (30, 210, 30) if row['status'] == 'ok' else (0, 100, 255)
            cv2.rectangle(overlay, (a, b), (c, d), color, 2)
            cv2.putText(overlay, f'{row["detection_id"]} {row["class_name"]}', (a, max(14, b - 5)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)
        if not cv2.imwrite(str(args.out_dir / f'view_{index + 1}.png'), overlay):
            raise ValueError('échec écriture aperçu')
    write_json(args.out_dir / 'prediction_run.json', {
        'mode': 'two_view_bbox_centres_correspond', 'frame_id': 'base_link', 'units': 'mm',
        'point_definition': args.point_definition, 'z_measured': True,
        'provisional_reference': provisional, 'cameras': metadata,
        'weights': fingerprint(args.weights), 'confidence_threshold': args.confidence,
        'min_ray_angle_deg': args.min_ray_angle_deg, 'max_ray_gap_mm': args.max_ray_gap_mm,
        'max_reprojection_px': args.max_reprojection_px,
        'detections': len(rows), 'localized': sum(row['status'] == 'ok' for row in rows),
        'assumptions': ['static_scene_between_images', 'bbox_centres_correspond'],
        'limitation': 'Les centres de boîtes de deux vues ne sont pas nécessairement le même '
                      'point physique. La triangulation peut être biaisée même avec un faible résidu.'})
    print(f'{sum(row["status"] == "ok" for row in rows)}/{len(rows)} classes triangulées -> '
          f'{args.out_dir / "predictions.csv"}')


def read_points(path, ground_truth=False):
    with Path(path).open(newline='') as stream:
        reader = csv.DictReader(stream)
        if not set(POINT_FIELDS) <= set(reader.fieldnames or []):
            raise ValueError(f'{path}: colonnes requises {POINT_FIELDS}')
        rows = list(reader)
    seen = set()
    for row in rows:
        for field in ['object_id', 'class_name', 'point_definition']:
            row[field] = row[field].strip()
        if not row['class_name'] or not row['point_definition'] or (ground_truth and not row['object_id']):
            raise ValueError(f'{path}: identifiant, classe ou point_definition manquant')
        ident = row['object_id']
        if ident and ident in seen:
            raise ValueError(f'{path}: object_id répété: {ident}')
        if ident:
            seen.add(ident)
        if ground_truth or row.get('status', 'ok') == 'ok':
            for axis in ['x_mm', 'y_mm', 'z_mm']:
                row[axis] = finite(row[axis], axis)
    return rows


def associate(predictions, references):
    """ID explicite prioritaire; sinon classe unique. Aucune distance utilisée."""
    matches, used_p, used_g = [], set(), set()
    reference_ids = {row['object_id']: i for i, row in enumerate(references)}
    for i, prediction in enumerate(predictions):
        ident = prediction.get('object_id', '')
        if ident and ident in reference_ids:
            j = reference_ids[ident]
            if j in used_g:
                raise ValueError(f'object_id ambigu: {ident}')
            matches.append((i, j, 'object_id'))
            used_p.add(i)
            used_g.add(j)
    for i, prediction in enumerate(predictions):
        if i in used_p or prediction.get('object_id', ''):
            continue
        name = prediction['class_name']
        candidates = [j for j, r in enumerate(references) if r['class_name'] == name]
        same_predictions = [p for p in predictions if p['class_name'] == name]
        if len(candidates) > 1 or (candidates and len(same_predictions) > 1):
            raise ValueError(f'association ambiguë pour {name}: fournir des object_id explicites')
        if candidates and candidates[0] not in used_g:
            j = candidates[0]
            matches.append((i, j, 'unique_class'))
            used_p.add(i)
            used_g.add(j)
    return matches, sorted(set(range(len(predictions))) - used_p), sorted(set(range(len(references))) - used_g)


def compare_points(predictions, references):
    valid = [row for row in predictions if row.get('status', 'ok') == 'ok']
    matches, extra, missing = associate(valid, references)
    rows = []
    for i, j, method in matches:
        prediction, reference = valid[i], references[j]
        if prediction['point_definition'] != reference['point_definition']:
            raise ValueError(f'point_definition incompatible pour {reference["object_id"]}')
        delta = np.array([finite(prediction[a], a) - finite(reference[a], a)
                          for a in ['x_mm', 'y_mm', 'z_mm']])
        rows.append({'object_id': reference['object_id'],
                     'detection_id': prediction.get('detection_id', ''),
                     'gt_class': reference['class_name'], 'pred_class': prediction['class_name'],
                     'class_correct': prediction['class_name'] == reference['class_name'],
                     'point_definition': reference['point_definition'], 'association': method,
                     'dx_mm': delta[0], 'dy_mm': delta[1], 'dz_mm': delta[2],
                     'error_xy_mm': np.linalg.norm(delta[:2]), 'error_xyz_mm': np.linalg.norm(delta)})
    stats = {}
    for metric in ['dx_mm', 'dy_mm', 'dz_mm', 'error_xy_mm', 'error_xyz_mm']:
        values = np.array([row[metric] for row in rows])
        stats[metric] = ({'mean': float(values.mean()), 'rmse': float(np.sqrt(np.mean(values ** 2))),
                          'p95': float(np.percentile(values, 95)),
                          'p95_absolute': float(np.percentile(abs(values), 95))} if len(values) else None)
    summary = {'frame_id': 'base_link', 'units': 'mm', 'reference_count': len(references),
               'prediction_count': len(predictions), 'valid_prediction_count': len(valid),
               'rejected_prediction_count': len(predictions) - len(valid),
               'matched': len(rows), 'correct_class': sum(row['class_correct'] for row in rows),
               'coverage': len(rows) / len(references) if references else None,
               'missing_count': len(missing), 'false_positive_count': len(extra),
               'missing_ids': [references[j]['object_id'] for j in missing],
               'extra_predictions': [{'object_id': valid[i].get('object_id', ''),
                                      'detection_id': valid[i].get('detection_id', ''),
                                      'class_name': valid[i]['class_name']} for i in extra],
               'metrics_all_matched': stats,
               'bias_xyz_mm': [stats[key]['mean'] for key in ['dx_mm', 'dy_mm', 'dz_mm']] if rows else None,
               'provisional_reference': any(str(row.get('provisional_reference', '')).lower() == 'true'
                                            for row in predictions),
               'note': 'Erreurs prédiction moins référence. XY et XYZ incluent calibration, '
                       'centre boîte et méthode 3D; Z de projection est imposé, non mesuré. '
                       'La GT doit être indépendante de la calibration et des hauteurs.'}
    return rows, summary


def compare(args):
    predictions = read_points(args.predictions)
    references = read_points(args.ground_truth, ground_truth=True)
    if not references:
        raise ValueError('vérité terrain vide')
    rows, summary = compare_points(predictions, references)
    summary.update(predictions=fingerprint(args.predictions), ground_truth=fingerprint(args.ground_truth),
                   ground_truth_source=args.ground_truth_source)
    args.out_dir.mkdir(parents=True, exist_ok=True)
    write_csv(args.out_dir / 'errors.csv', rows, ERROR_FIELDS)
    write_json(args.out_dir / 'comparison.json', summary)
    print(f'{summary["matched"]}/{summary["reference_count"]} appariés; '
          f'{summary["missing_count"]} manquants; {summary["false_positive_count"]} en trop')


def parser():
    ap = argparse.ArgumentParser(description=__doc__)
    commands = ap.add_subparsers(dest='command', required=True)
    prediction = commands.add_parser('predict', help='image réelle -> XYZ, sans lecture de GT')
    for name in ['image', 'calibration', 'heights', 'out-dir']:
        prediction.add_argument(f'--{name}', type=Path, required=True)
    prediction.add_argument('--weights', type=Path, default=DEFAULT_WEIGHTS)
    prediction.add_argument('--ids', type=Path, help='JSON detection_id -> object_id indépendant')
    prediction.add_argument('--point-definition', default='object_center')
    prediction.add_argument('--confidence', type=float, default=0.10)
    prediction.add_argument('--max-loo-mm', type=float, default=5.0)
    prediction.set_defaults(run=predict)
    triangulation = commands.add_parser('triangulate', help='deux images statiques -> XYZ sans hauteur imposée')
    for name in ['image-a', 'calibration-a', 'image-b', 'calibration-b', 'out-dir']:
        triangulation.add_argument(f'--{name}', type=Path, required=True)
    triangulation.add_argument('--weights', type=Path, default=DEFAULT_WEIGHTS)
    triangulation.add_argument('--point-definition', default='object_center')
    triangulation.add_argument('--confidence', type=float, default=0.10)
    triangulation.add_argument('--max-loo-mm', type=float, default=5.0)
    triangulation.add_argument('--min-ray-angle-deg', type=float, default=2.0)
    triangulation.add_argument('--max-ray-gap-mm', type=float, default=20.0)
    triangulation.add_argument('--max-reprojection-px', type=float, default=10.0)
    triangulation.set_defaults(run=triangulate_images)
    comparison = commands.add_parser('compare', help='CSV prédictions figées vs référence indépendante')
    for name in ['predictions', 'ground-truth', 'out-dir']:
        comparison.add_argument(f'--{name}', type=Path, required=True)
    comparison.add_argument('--ground-truth-source', required=True,
                            help='méthode indépendante, date et incertitude de la mesure physique')
    comparison.set_defaults(run=compare)
    return ap


def main():
    ap = parser()
    args = ap.parse_args()
    if args.command in ('predict', 'triangulate') and (not 0 < args.confidence <= 1 or not args.point_definition.strip()):
        ap.error('confidence doit être dans ]0,1] et point-definition non vide')
    if args.command == 'triangulate' and not (0 < args.min_ray_angle_deg < 90
            and 0 < args.max_ray_gap_mm < float('inf') and 0 < args.max_reprojection_px < float('inf')):
        ap.error('seuils de triangulation invalides')
    try:
        args.run(args)
    except (ValueError, OSError, KeyError, TypeError) as exc:
        ap.error(str(exc))


if __name__ == '__main__':
    main()
