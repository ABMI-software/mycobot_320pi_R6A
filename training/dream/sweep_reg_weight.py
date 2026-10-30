#!/usr/bin/env python3
"""Comparative sweep of solve_joint_angles_fixed_pose's reg_weight.

Live testing showed the solver running to a joint's mechanical limit (J1
pinned at 167.9° = its exact upper bound) with reg_weight=1.5 — a sign the
regularization is too weak to hold poorly-observed joints near a sane value.
Rather than guess a bigger number, this replays the cached real_3cam
detections (built by validate_angle_solver_real.py — no GPU inference needed)
through the solver at several reg_weight values and reports, per weight:

  - angular RMS (J1-J6 and J1-J5) against real encoder ground truth
  - reprojection RMS (px)
  - number of joint_limit_hit occurrences (the failure mode this sweep exists
    to fix)
  - solve latency
  - frame-to-frame drift (how much q changes between consecutive poses in a
    session, CHAINED the same way the live dashboard warm-starts from the
    previous frame's own estimate) — the failure mode on the other side: too
    much regularization just makes DREAM parrot q_init, drift collapses to
    ~0 and the curve stops being an independent measurement.

real_3cam poses are i.i.d. random configs, not a video trajectory, so
"frame-to-frame drift" here is a proxy (chained in dataset order) rather than
true temporal drift — still a valid signal for "does the estimate jump
wildly or go artificially static", which is what matters for picking a
weight.
"""
import os
import sys
import time

import cv2
import numpy as np
import pandas as pd

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

from dream_angle_solver import solve_joint_angles_fixed_pose
from mycobot_fk import forward_kinematics, KEYPOINT_NAMES

_REPO = os.path.dirname(os.path.dirname(_HERE))
CALIB_PATH = os.path.join(_REPO, 'training', 'calibration', 'cam_3.meta.json')
CACHE_PATH = os.path.join(_HERE, 'angle_solver_validation_cache.npz')

REG_WEIGHTS = [1.5, 3.0, 5.0, 8.0]
ANCHOR_N = 30


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


def run_sweep_for_weight(data, camera_K, camera_dist, reg_weight):
    sessions = data['session']
    all_err = []       # per-solve angular error (deg), shape (N,6)
    all_reproj = []     # px
    all_latency = []    # s
    all_drift = []       # per-joint |q_t - q_{t-1}| (deg), shape (N-1 per session,6)
    n_limit_hit = 0
    n_total = 0

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
            q_init = prev_q if prev_q is not None else q_true  # first-frame fallback ~ encoder
            t0 = time.perf_counter()
            res = solve_joint_angles_fixed_pose(
                kp_2d, kp_valid, camera_K, anchor['rvec'], anchor['tvec'],
                q_init=q_init, reg_weight=reg_weight)
            latency = time.perf_counter() - t0
            if res is None:
                continue
            n_total += 1
            all_latency.append(latency)
            all_reproj.append(res['rms_reproj_px'])
            if res['solver_failed']:
                n_limit_hit += 1
                # Same policy as the live dashboard: don't chain a failed
                # solve into the next warm start.
                continue
            err_deg = np.degrees(np.abs(res['q'] - q_true))
            all_err.append(err_deg)
            if prev_q is not None:
                all_drift.append(np.degrees(np.abs(res['q'] - prev_q)))
            prev_q = res['q']

    err_arr = np.array(all_err) if all_err else np.zeros((0, 6))
    drift_arr = np.array(all_drift) if all_drift else np.zeros((0, 6))

    return {
        'reg_weight': reg_weight,
        'n_total': n_total,
        'n_limit_hit': n_limit_hit,
        'pct_limit_hit': 100.0 * n_limit_hit / n_total if n_total else float('nan'),
        'rms_angle_j1_j6': float(np.sqrt(np.mean(err_arr ** 2))) if len(err_arr) else float('nan'),
        'rms_angle_j1_j5': float(np.sqrt(np.mean(err_arr[:, :5] ** 2))) if len(err_arr) else float('nan'),
        'mean_reproj_px': float(np.mean(all_reproj)) if all_reproj else float('nan'),
        'mean_latency_ms': float(np.mean(all_latency) * 1000) if all_latency else float('nan'),
        'mean_drift_deg': float(np.mean(drift_arr)) if len(drift_arr) else float('nan'),
        'n_solved_ok': len(err_arr),
    }


def main():
    if not os.path.isfile(CACHE_PATH):
        print(f'No cache at {CACHE_PATH} — run validate_angle_solver_real.py first.')
        return
    data = np.load(CACHE_PATH)
    camera_K, camera_dist = load_intrinsics()

    rows = []
    for w in REG_WEIGHTS:
        print(f'Running reg_weight={w}...')
        rows.append(run_sweep_for_weight(data, camera_K, camera_dist, w))

    df = pd.DataFrame(rows)
    pd.set_option('display.width', 160)
    pd.set_option('display.float_format', lambda x: f'{x:.2f}')
    print()
    print(df.to_string(index=False))

    out_csv = os.path.join(_HERE, 'reg_weight_sweep.csv')
    df.to_csv(out_csv, index=False)
    print(f'\nSaved to {out_csv}')


if __name__ == '__main__':
    main()
