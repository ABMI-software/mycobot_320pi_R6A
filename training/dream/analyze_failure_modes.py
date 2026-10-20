#!/usr/bin/env python3
"""Failure-mode analysis on the EXISTING real_3cam cache (angle_solver_
validation_cache.npz, 2500 real DREAM detections, GT encoder q) — no new
data collection, no retraining. Answers, per the 2026-07-15 request:

  1. Keypoint quality: per-keypoint pixel error (oracle pose = built from
     encoder+FK, so this isolates DREAM's own detection error, nothing else).
  2. Does keypoint error predict joint-angle error? (correlation, per joint)
  3. Which POSE REGIONS fail worst, even under oracle (best-case, perfectly-
     known pose) conditions — same methodology as angle_error_diagnosis.py,
     bucketed by joint angle instead of pooled.
  4. Geometric ambiguity prevalence: multi-start clustering (like
     diagnose_local_minima.py) on a stratified sample of real frames, using
     the joint+pose FREE solver (what the live dashboard actually runs) —
     how often does a real frame admit multiple near-equal-error minima, and
     does that correlate with keypoint quality or pose region?

Never touches the encoder as a solve TARGET — q_true is used only to (a)
build the oracle pose reference (a one-time, offline, non-live use, same as
angle_error_diagnosis.py) and (b) as a REPORTING ground truth to measure
error against, exactly as in the live dashboard's own MAE panel.
"""
import json
import os
import sys

import cv2
import numpy as np
import pandas as pd

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

from dream_angle_solver import (
    solve_joint_angles_two_pass, solve_joint_angles_and_pose,
    JOINT_LOWER, JOINT_UPPER,
)
from mycobot_fk import forward_kinematics, KEYPOINT_NAMES

_REPO = os.path.dirname(os.path.dirname(_HERE))
CALIB_PATH = os.path.join(_REPO, 'training', 'calibration', 'cam_3.meta.json')
CACHE_PATH = os.path.join(_HERE, 'angle_solver_validation_cache.npz')

REG_WEIGHT = 1.5
JOINT_NAMES = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
KP_LABELS = [n.replace('mycobot320_', '') for n in KEYPOINT_NAMES]
N_OBSERVING_KP = {0: 5, 1: 4, 2: 3, 3: 2, 4: 1, 5: 0}


def load_intrinsics():
    meta = json.loads(open(CALIB_PATH).read())
    r = meta['results']
    K = np.array([[r['fx'], 0.0, r['cx']], [0.0, r['fy'], r['cy']], [0.0, 0.0, 1.0]])
    dist = np.array(r['dist_coeffs'], dtype=np.float64)
    return K, dist


def build_session_anchor(kp_2d_list, kp_valid_list, q_true_list, camera_K, camera_dist):
    rvecs, tvecs = [], []
    for kp_2d, kp_valid, q in zip(kp_2d_list, kp_valid_list, q_true_list):
        if kp_valid.sum() < 5:
            continue
        positions, _ = forward_kinematics(q)
        pts_3d = np.array([positions[n] for n in KEYPOINT_NAMES], dtype=np.float64)
        idx = np.where(kp_valid)[0]
        ok, rvec, tvec = cv2.solvePnP(pts_3d[idx], kp_2d[idx], camera_K, camera_dist, flags=cv2.SOLVEPNP_EPNP)
        if not ok:
            continue
        rvec, tvec = cv2.solvePnPRefineLM(pts_3d[idx], kp_2d[idx], camera_K, camera_dist, rvec, tvec)
        rvecs.append(rvec.flatten())
        tvecs.append(tvec.flatten())
    if len(rvecs) < 5:
        return None
    return {'rvec': np.median(np.array(rvecs), axis=0), 'tvec': np.median(np.array(tvecs), axis=0)}


def cluster_by_q(results, thresh_deg=15.0):
    clusters = []
    for r in results:
        placed = False
        for members, rep in clusters:
            if np.degrees(np.abs(r['q'] - rep)).max() < thresh_deg:
                members.append(r)
                placed = True
                break
        if not placed:
            clusters.append(([r], r['q']))
    return clusters


def multi_start_n_minima(kp_2d, kp_valid, camera_K, n_random=8, seed=0):
    rng = np.random.default_rng(seed)
    seeds = [np.zeros(6)]
    for j1 in np.radians([-135, -90, -45, 45, 90, 135]):
        c = np.zeros(6)
        c[0] = j1
        seeds.append(c)
    for _ in range(n_random):
        seeds.append(rng.uniform(JOINT_LOWER + 0.05, JOINT_UPPER - 0.05))

    results = []
    for q0 in seeds:
        res = solve_joint_angles_and_pose(
            kp_2d, kp_valid, camera_K, q_init=q0, rvec_init=None, tvec_init=None,
            reg_weight=REG_WEIGHT, pose_reg_weight=5.0)
        if res is not None and not res['solver_failed']:
            results.append(res)
    clusters = cluster_by_q(results)
    clusters.sort(key=lambda c: min(r['rms_reproj_px'] for r in c[0]))
    best_rms = clusters[0][0][0]['rms_reproj_px'] if clusters else float('nan')
    # "competing" minima: within 2x the best rms — a real ambiguity, not just
    # a numerically-worse also-ran the solver would never actually settle on.
    competing = sum(1 for m, _ in clusters if min(r['rms_reproj_px'] for r in m) < 2.0 * best_rms)
    return len(clusters), competing, best_rms


def main():
    if not os.path.isfile(CACHE_PATH):
        print(f'No cache at {CACHE_PATH}')
        return
    data = np.load(CACHE_PATH)
    camera_K, camera_dist = load_intrinsics()
    sessions = data['session']

    per_frame_rows = []

    for session in np.unique(sessions):
        mask = sessions == session
        kp_2d_s, kp_valid_s, q_true_s, is_anchor_s = (
            data['kp_2d'][mask], data['kp_valid'][mask], data['q_true'][mask], data['is_anchor'][mask])

        anchor = build_session_anchor(kp_2d_s[is_anchor_s], kp_valid_s[is_anchor_s], q_true_s[is_anchor_s],
                                       camera_K, camera_dist)
        if anchor is None:
            continue

        held = ~is_anchor_s
        for kp_2d, kp_valid, q_true in zip(kp_2d_s[held], kp_valid_s[held], q_true_s[held]):
            if kp_valid.sum() < 4:
                continue

            # (1) oracle pixel error per keypoint — pure DREAM detection
            # quality, GT pose known, GT q known.
            positions, _ = forward_kinematics(q_true)
            pts_3d = np.array([positions[n] for n in KEYPOINT_NAMES])
            proj, _ = cv2.projectPoints(pts_3d, anchor['rvec'], anchor['tvec'], camera_K, camera_dist)
            proj = proj.reshape(-1, 2)
            px_err = np.linalg.norm(proj - kp_2d, axis=1)
            px_err_valid = np.where(kp_valid, px_err, np.nan)

            # (2)/(3) oracle angle error (isolates keypoint/geometry error
            # from warm-start, same as angle_error_diagnosis.py)
            res_o = solve_joint_angles_two_pass(kp_2d, kp_valid, camera_K, anchor['rvec'], anchor['tvec'],
                                                 q_init=q_true, reg_weight=REG_WEIGHT)
            if res_o is None or res_o['solver_failed']:
                continue
            ang_err = np.degrees(np.abs(res_o['q'] - q_true))

            row = {
                'session': int(session),
                'n_valid_kp': int(kp_valid.sum()),
                'mae_deg': float(ang_err.mean()),
                'max_joint_err_deg': float(ang_err.max()),
                'worst_joint': JOINT_NAMES[int(np.argmax(ang_err))],
            }
            for j in range(6):
                row[f'{JOINT_NAMES[j]}_err_deg'] = float(ang_err[j])
                row[f'{JOINT_NAMES[j]}_true_deg'] = float(np.degrees(q_true[j]))
            for k in range(7):
                row[f'{KP_LABELS[k]}_px_err'] = float(px_err_valid[k]) if not np.isnan(px_err_valid[k]) else None
            per_frame_rows.append(row)

    df = pd.DataFrame(per_frame_rows)
    pd.set_option('display.width', 160)
    pd.set_option('display.float_format', lambda x: f'{x:.2f}')

    print(f'{len(df)} frames analysées (oracle pose, GT encodeur)\n')

    # ── Part A: per-keypoint detection quality ──
    print('=== A. Qualité de détection par keypoint (px, oracle pose) ===')
    kp_stats = []
    for k in KP_LABELS:
        col = df[f'{k}_px_err'].dropna()
        kp_stats.append({'keypoint': k, 'n': len(col), 'median_px': col.median(),
                          'mean_px': col.mean(), 'p90_px': col.quantile(0.9)})
    print(pd.DataFrame(kp_stats).to_string(index=False))

    # ── Part B: keypoint error -> angle error correlation, per joint ──
    print('\n=== B. Corrélation erreur keypoint -> erreur angle (Pearson r) ===')
    corr_rows = []
    for j in range(6):
        jn = JOINT_NAMES[j]
        observing = N_OBSERVING_KP[j]
        # pick the keypoint most directly downstream of this joint (index j+1
        # in KEYPOINT_NAMES corresponds to link_j, the nearest observer)
        kp_col = f'{KP_LABELS[min(j + 1, 6)]}_px_err'
        sub = df[[kp_col, f'{jn}_err_deg']].dropna()
        r = sub[kp_col].corr(sub[f'{jn}_err_deg']) if len(sub) > 5 else float('nan')
        corr_rows.append({'joint': jn, 'n_kp_observing': observing, 'nearest_kp': kp_col,
                           'pearson_r': r, 'n_samples': len(sub)})
    print(pd.DataFrame(corr_rows).to_string(index=False))

    # ── Part C: worst pose regions (oracle MAE bucketed by joint angle) ──
    print('\n=== C. Zones de pose les plus difficiles (MAE oracle par tranche de 30°, par joint) ===')
    for j in range(6):
        jn = JOINT_NAMES[j]
        bins = np.arange(-150, 181, 30)
        df['_bucket'] = pd.cut(df[f'{jn}_true_deg'], bins)
        g = df.groupby('_bucket', observed=True)[f'{jn}_err_deg'].agg(['mean', 'count'])
        g = g[g['count'] >= 5]
        if len(g):
            worst = g['mean'].idxmax()
            print(f'  {jn}: pire tranche = {worst}, MAE={g.loc[worst, "mean"]:.2f}° (n={int(g.loc[worst, "count"])}) '
                  f'| MAE global={df[f"{jn}_err_deg"].mean():.2f}°')
    df.drop(columns=['_bucket'], inplace=True)

    # ── Part D: geometric ambiguity prevalence (stratified sample) ──
    print('\n=== D. Prévalence de l\'ambiguïté géométrique (échantillon stratifié, pose libre) ===')
    rng = np.random.default_rng(0)
    sample_idx = rng.choice(len(df), size=min(60, len(df)), replace=False)
    amb_rows = []
    all_rows_by_session_frame = []  # rebuild kp_2d/kp_valid lookup
    idx_counter = 0
    frame_lookup = []
    for session in np.unique(sessions):
        mask = sessions == session
        kp_2d_s, kp_valid_s, q_true_s, is_anchor_s = (
            data['kp_2d'][mask], data['kp_valid'][mask], data['q_true'][mask], data['is_anchor'][mask])
        held = ~is_anchor_s
        for kp_2d, kp_valid, q_true in zip(kp_2d_s[held], kp_valid_s[held], q_true_s[held]):
            if kp_valid.sum() < 4:
                continue
            frame_lookup.append((kp_2d, kp_valid, q_true))

    for i in sample_idx:
        if i >= len(frame_lookup):
            continue
        kp_2d, kp_valid, q_true = frame_lookup[i]
        n_min, n_competing, best_rms = multi_start_n_minima(kp_2d, kp_valid, camera_K)
        amb_rows.append({
            'n_valid_kp': int(kp_valid.sum()),
            'n_minima': n_min,
            'n_competing_minima': n_competing,
            'best_rms_px': best_rms,
        })
    amb_df = pd.DataFrame(amb_rows)
    print(f'{len(amb_df)} frames testées (13 départs chacune)')
    print(f'  minima concurrents (rms < 2x meilleur) — moyenne : {amb_df["n_competing_minima"].mean():.2f}, '
          f'>1 minimum concurrent sur {(amb_df["n_competing_minima"] > 1).mean() * 100:.0f}% des frames')
    print(f'  corrélation n_valid_kp vs n_competing_minima : r={amb_df["n_valid_kp"].corr(amb_df["n_competing_minima"]):.2f}')

    out_path = os.path.join(_HERE, 'failure_mode_analysis.csv')
    df.to_csv(out_path, index=False)
    amb_df.to_csv(os.path.join(_HERE, 'failure_mode_ambiguity.csv'), index=False)
    print(f'\nSaved {out_path} + failure_mode_ambiguity.csv')


if __name__ == '__main__':
    main()
