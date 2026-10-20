#!/usr/bin/env python3
"""Per-joint angular error diagnosis against the 0.5-0.9° target (José's spec).

Replays the cached real_3cam detections through the CURRENT best solver
config (solve_joint_angles_two_pass, reg_weight=1.5 default — see
compare_robust_loss.py, 2026-07-13) and reports, PER JOINT:

  - mean |DREAM - encodeur| (deg)
  - RMS (deg)
  - max error (deg)
  - % frames with error < 1°
  - % frames with error < 2°

Uses a WELL-CONSTRUCTED multi-frame session anchor (build_session_anchor,
same as sweep_reg_weight.py / compare_robust_loss.py) rather than a
degenerate single-frame one, so the numbers below isolate solver/keypoint
error from the known anchor-degeneracy confound (that confound is a live-
dashboard-only issue since the pose-diversity requirement was removed there
on 2026-07-13 at user request — this offline diagnostic is unaffected
because it always used the multi-frame anchor).
"""
import os
import sys
import time

import cv2
import numpy as np
import pandas as pd

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)

from dream_angle_solver import solve_joint_angles_two_pass
from mycobot_fk import forward_kinematics, KEYPOINT_NAMES

_REPO = os.path.dirname(os.path.dirname(_HERE))
CALIB_PATH = os.path.join(_REPO, 'training', 'calibration', 'cam_3.meta.json')
CACHE_PATH = os.path.join(_HERE, 'angle_solver_validation_cache.npz')

REG_WEIGHT = 1.5
JOINT_NAMES = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']

# Which KEYPOINT_NAMES indices actually carry information about each joint's
# OWN rotation (mirrors JOINT_OBSERVING_KP in dream_validation_dashboard.py).
N_OBSERVING_KP = {0: 5, 1: 4, 2: 3, 3: 2, 4: 1, 5: 0}


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


def _stats_table(err, label):
    rows = []
    for j in range(6):
        e = err[:, j]
        rows.append({
            'mode': label,
            'joint': JOINT_NAMES[j],
            'n_kp_observing': N_OBSERVING_KP[j],
            'mean_deg': float(np.mean(e)),
            'median_deg': float(np.median(e)),
            'rms_deg': float(np.sqrt(np.mean(e ** 2))),
            'max_deg': float(np.max(e)),
            'pct_lt_1deg': float(100.0 * np.mean(e < 1.0)),
            'pct_lt_2deg': float(100.0 * np.mean(e < 2.0)),
        })
    return rows


def main():
    if not os.path.isfile(CACHE_PATH):
        print(f'No cache at {CACHE_PATH} — run validate_angle_solver_real.py first.')
        return
    data = np.load(CACHE_PATH)
    camera_K, camera_dist = load_intrinsics()
    sessions = data['session']

    err_oracle = []   # q_init = q_true (isolates keypoint/geometry error from warm-start)
    err_chained = []  # q_init = previous DREAM solve, artificial order (as the live dashboard warm-starts)

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

            res_o = solve_joint_angles_two_pass(
                kp_2d, kp_valid, camera_K, anchor['rvec'], anchor['tvec'],
                q_init=q_true, reg_weight=REG_WEIGHT)
            if res_o is not None and not res_o['solver_failed']:
                err_oracle.append(np.degrees(np.abs(res_o['q'] - q_true)))

            q_init = prev_q if prev_q is not None else q_true
            res_c = solve_joint_angles_two_pass(
                kp_2d, kp_valid, camera_K, anchor['rvec'], anchor['tvec'],
                q_init=q_init, reg_weight=REG_WEIGHT)
            if res_c is None or res_c['solver_failed']:
                prev_q = None
                continue
            err_chained.append(np.degrees(np.abs(res_c['q'] - q_true)))
            prev_q = res_c['q']

    eo = np.array(err_oracle)
    ec = np.array(err_chained)
    print(f'N oracle-init solved OK  : {len(eo)}')
    print(f'N chained-init solved OK : {len(ec)}\n')

    rows = _stats_table(eo, 'oracle_init') + _stats_table(ec, 'chained_init')
    df = pd.DataFrame(rows)
    pd.set_option('display.width', 160)
    pd.set_option('display.float_format', lambda x: f'{x:.2f}')
    print(df.to_string(index=False))

    out_csv = os.path.join(_HERE, 'angle_error_diagnosis.csv')
    df.to_csv(out_csv, index=False)
    print(f'\nSaved to {out_csv}')


if __name__ == '__main__':
    main()
