#!/usr/bin/env python3
"""Comparative test: linear vs. robust (Huber / soft_l1) loss in
solve_joint_angles_fixed_pose, on the same cached real_3cam frames used by
sweep_reg_weight.py.

Motivation: live testing on the real robot found a single bad DREAM
detection (link5) dragging the whole per-frame pose fit — every OTHER
keypoint's apparent error dropped from ~14px to ~5px once link5 alone was
excluded from a fresh fit (see the 2026-07-13 investigation notes in
CHANGELOG.md / training/dream/README.md). Plain least-squares treats every
residual quadratically, so one outlier point can dominate the sum-of-squares
and contaminate every joint's estimate, not just the one that keypoint
nominally informs. This script checks whether switching the LOSS function
(not reg_weight, not filtering — both left untouched per the investigation's
instruction) actually fixes that contamination, before anything is changed
in the live dashboard.

reg_weight is held FIXED at 1.5 (its existing default) throughout — this
script isolates the loss function's effect only.

Usage (venv_dream):
    source ~/ros_jazzy/venv_dream/bin/activate
    python3 compare_robust_loss.py
"""
import os
import sys
import time

import cv2
import numpy as np
import pandas as pd

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

from dream_angle_solver import solve_joint_angles_fixed_pose, solve_joint_angles_two_pass
from mycobot_fk import forward_kinematics, KEYPOINT_NAMES

_REPO = os.path.dirname(os.path.dirname(_HERE))
CALIB_PATH = os.path.join(_REPO, 'training', 'calibration', 'cam_3.meta.json')
CACHE_PATH = os.path.join(_HERE, 'angle_solver_validation_cache.npz')

REG_WEIGHT = 1.5   # fixed — not what this script tests
F_SCALE_PX = 10.0  # outlier threshold, see dream_angle_solver docstring

# (label, loss, use_two_pass)
CONFIGS = [
    ('linear (actuel)', 'linear', False),
    ('huber', 'huber', False),
    ('soft_l1', 'soft_l1', False),
    ('linear + rejet 2-passes', 'linear', True),
]


def load_intrinsics():
    import json
    meta = json.loads(open(CALIB_PATH).read())
    r = meta['results']
    K = np.array([[r['fx'], 0.0, r['cx']],
                  [0.0, r['fy'], r['cy']],
                  [0.0, 0.0, 1.0]], dtype=np.float64)
    dist = np.array(r['dist_coeffs'], dtype=np.float64)
    return K, dist


def build_session_anchor(kp_2d_list, kp_valid_list, q_true_list, camera_K, camera_dist):
    """Unchanged from sweep_reg_weight.py — same single-pose-per-session
    median anchor the existing cache was built against, so results are
    directly comparable to the reg_weight sweep."""
    rvecs, tvecs = [], []
    for kp_2d, kp_valid, q in zip(kp_2d_list, kp_valid_list, q_true_list):
        if kp_valid.sum() < 5:
            continue
        positions, _ = forward_kinematics(q)
        pts_3d = np.array([positions[n] for n in KEYPOINT_NAMES], dtype=np.float64)
        idx = np.where(kp_valid)[0]
        ok, rvec, tvec = cv2.solvePnP(
            pts_3d[idx], kp_2d[idx], camera_K, camera_dist, flags=cv2.SOLVEPNP_EPNP)
        if not ok:
            continue
        rvec, tvec = cv2.solvePnPRefineLM(pts_3d[idx], kp_2d[idx], camera_K, camera_dist, rvec, tvec)
        rvecs.append(rvec.flatten())
        tvecs.append(tvec.flatten())
    if len(rvecs) < 5:
        return None
    return {'rvec': np.median(np.array(rvecs), axis=0),
            'tvec': np.median(np.array(tvecs), axis=0)}


def run_for_config(data, camera_K, camera_dist, loss, two_pass):
    sessions = data['session']
    all_err = []          # angular error (deg) vs encoder truth, shape (N,6)
    all_reproj = []        # px
    all_latency = []       # s
    all_drift = []          # frame-to-frame |dq| (deg), chained in dataset order
    all_n_outliers = []      # per-solve outlier count (pass-1 flag)
    all_n_rejected = []       # per-solve keypoints actually dropped for pass 2
    n_limit_hit = 0
    n_total = 0

    solve_fn = solve_joint_angles_two_pass if two_pass else solve_joint_angles_fixed_pose

    for session in np.unique(sessions):
        mask = sessions == session
        kp_2d_s = data['kp_2d'][mask]
        kp_valid_s = data['kp_valid'][mask]
        q_true_s = data['q_true'][mask]
        is_anchor_s = data['is_anchor'][mask]

        anchor = build_session_anchor(
            kp_2d_s[is_anchor_s], kp_valid_s[is_anchor_s], q_true_s[is_anchor_s],
            camera_K, camera_dist)
        if anchor is None:
            continue

        held = ~is_anchor_s
        prev_q = None
        for kp_2d, kp_valid, q_true in zip(kp_2d_s[held], kp_valid_s[held], q_true_s[held]):
            if kp_valid.sum() < 4:
                continue
            q_init = prev_q if prev_q is not None else q_true
            t0 = time.perf_counter()
            res = solve_fn(
                kp_2d, kp_valid, camera_K, anchor['rvec'], anchor['tvec'],
                q_init=q_init, reg_weight=REG_WEIGHT, loss=loss, f_scale=F_SCALE_PX)
            latency = time.perf_counter() - t0
            if res is None:
                continue
            n_total += 1
            all_latency.append(latency)
            all_reproj.append(res['rms_reproj_px'])
            all_n_outliers.append(res['n_outliers'])
            all_n_rejected.append(res.get('n_rejected_pass2', 0))
            if res['solver_failed']:
                n_limit_hit += 1
                continue
            err_deg = np.degrees(np.abs(res['q'] - q_true))
            all_err.append(err_deg)
            if prev_q is not None:
                all_drift.append(np.degrees(np.abs(res['q'] - prev_q)))
            prev_q = res['q']

    err_arr = np.array(all_err) if all_err else np.zeros((0, 6))
    drift_arr = np.array(all_drift) if all_drift else np.zeros((0, 6))

    return {
        'n_total': n_total,
        'n_limit_hit': n_limit_hit,
        'pct_limit_hit': 100.0 * n_limit_hit / n_total if n_total else float('nan'),
        'rms_angle_j1_j6': float(np.sqrt(np.mean(err_arr ** 2))) if len(err_arr) else float('nan'),
        'rms_angle_j1_j5': float(np.sqrt(np.mean(err_arr[:, :5] ** 2))) if len(err_arr) else float('nan'),
        'mean_reproj_px': float(np.mean(all_reproj)) if all_reproj else float('nan'),
        'mean_latency_ms': float(np.mean(all_latency) * 1000) if all_latency else float('nan'),
        'mean_drift_deg': float(np.mean(drift_arr)) if len(drift_arr) else float('nan'),
        'mean_n_outliers_per_frame': float(np.mean(all_n_outliers)) if all_n_outliers else float('nan'),
        'mean_n_rejected_pass2': float(np.mean(all_n_rejected)) if all_n_rejected else float('nan'),
        'n_solved_ok': len(err_arr),
    }


def main():
    if not os.path.isfile(CACHE_PATH):
        print(f'No cache at {CACHE_PATH} — run validate_angle_solver_real.py first.')
        return
    data = np.load(CACHE_PATH)
    camera_K, camera_dist = load_intrinsics()

    rows = []
    for label, loss, two_pass in CONFIGS:
        print(f'Running {label}...')
        stats = run_for_config(data, camera_K, camera_dist, loss, two_pass)
        stats['config'] = label
        rows.append(stats)

    df = pd.DataFrame(rows).set_index('config')
    cols = ['rms_angle_j1_j6', 'rms_angle_j1_j5', 'mean_reproj_px',
            'mean_n_outliers_per_frame', 'mean_n_rejected_pass2',
            'pct_limit_hit', 'mean_drift_deg', 'mean_latency_ms', 'n_solved_ok']
    df = df[cols]
    pd.set_option('display.width', 200)
    pd.set_option('display.float_format', lambda x: f'{x:.3f}')
    print()
    print(df.to_string())

    out_csv = os.path.join(_HERE, 'robust_loss_comparison.csv')
    df.to_csv(out_csv)
    print(f'\nSaved to {out_csv}')
    print(f'\n(reg_weight fixed at {REG_WEIGHT}, f_scale={F_SCALE_PX}px — not swept here)')


if __name__ == '__main__':
    main()
