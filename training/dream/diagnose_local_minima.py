#!/usr/bin/env python3
"""Multi-start diagnostic for solve_joint_angles_and_pose — checks whether
several distinct joint configurations reproject to near-identical pixel
error (the classic signature of a gauge-ambiguous / rank-deficient fit),
using scipy's own least_squares diagnostics (cost, nfev, optimality, and the
Jacobian's condition number) rather than guesswork.

Answers, for one or more real captured keypoint sets:
  1. Do multiple q's converge to similar reprojection error from different
     q_init (joint+pose jointly free, as the live dashboard runs it)?
  2. Does fixing the camera pose (instead of estimating it jointly) remove
     that multiplicity?

Usage: python3 diagnose_local_minima.py
Reads training/dream/diagnose_local_minima_frame.json (one captured frame:
kp_2d, kp_valid, camera_K, encoder_q_deg) written alongside this script.
"""
import json
import sys
from pathlib import Path

import numpy as np
import cv2

sys.path.insert(0, str(Path(__file__).resolve().parent))
from mycobot_fk import forward_kinematics, KEYPOINT_NAMES
from dream_angle_solver import (
    solve_joint_angles_and_pose, solve_joint_angles_fixed_pose,
    JOINT_LOWER, JOINT_UPPER,
)

FRAME_PATH = Path(__file__).resolve().parent / 'diagnose_local_minima_frame.json'

N_RANDOM_INIT = 12
CLUSTER_DEG = 15.0  # q's within this per-joint max-abs-diff count as "same minimum"


def _candidate_inits(seed=0):
    rng = np.random.default_rng(seed)
    seeds = [np.zeros(6)]
    for j1 in np.radians([-135, -90, -45, 45, 90, 135]):
        c = np.zeros(6)
        c[0] = j1
        seeds.append(c)
    for _ in range(N_RANDOM_INIT):
        seeds.append(rng.uniform(JOINT_LOWER + 0.05, JOINT_UPPER - 0.05))
    return seeds


def _cluster(results):
    """Group results whose q's are within CLUSTER_DEG (max per-joint) of an
    existing cluster's first member. Returns list of (members, rep_q)."""
    clusters = []
    for r in results:
        placed = False
        for members, rep_q in clusters:
            if np.degrees(np.abs(r['q'] - rep_q)).max() < CLUSTER_DEG:
                members.append(r)
                placed = True
                break
        if not placed:
            clusters.append(([r], r['q']))
    return clusters


def run_joint_pose_free(kp_2d, kp_valid, camera_K):
    print('\n=== Pose CAMERA LIBRE (jointe avec les angles) — comme le dashboard live ===')
    results = []
    for q0 in _candidate_inits():
        res = solve_joint_angles_and_pose(
            kp_2d, kp_valid, camera_K,
            q_init=q0, rvec_init=None, tvec_init=None,
            reg_weight=0.0, pose_reg_weight=0.0,  # no warm-start prior — pure multi-start
        )
        if res is None:
            continue
        results.append(res)

    clusters = _cluster(results)
    clusters.sort(key=lambda c: min(r['rms_reproj_px'] for r in c[0]))

    print(f'{len(results)} solves, {len(clusters)} minimum/a distinct(s) (seuil cluster = {CLUSTER_DEG}° max/joint)\n')
    for i, (members, rep_q) in enumerate(clusters):
        best = min(members, key=lambda r: r['rms_reproj_px'])
        print(f'--- Minimum #{i+1} ({len(members)}/{len(results)} départs y convergent) ---')
        print(f'  q (deg)          : {np.degrees(best["q"]).round(1).tolist()}')
        print(f'  rms_reproj_px    : {best["rms_reproj_px"]:.3f}')
        print(f'  cost (0.5*sum^2) : {best["cost"]:.4f}')
        print(f'  nfev             : {best["nfev"]}')
        print(f'  optimality       : {best["optimality"]:.2e}')
        print(f'  jac_cond         : {best["jac_cond"]:.3e}')
        print(f'  joint_limit_hit  : {best["joint_limit_hit"]}')
    return clusters


def run_fixed_pose(kp_2d, kp_valid, camera_K, encoder_q_rad):
    """Fixed-pose control: derive ONE reference camera pose from the encoder
    angles (FK) + a solvePnP fit — legitimate for a one-off OFFLINE
    diagnostic (never done in the live independent-validation path) — then
    multi-start q alone with that pose held fixed."""
    print('\n=== Pose CAMERA FIXEE (dérivée encodeur+FK, pour ce diagnostic seulement) ===')
    positions, _ = forward_kinematics(encoder_q_rad)
    pts_3d = np.array([positions[n] for n in KEYPOINT_NAMES], dtype=np.float64)
    idx = [i for i in range(7) if kp_valid[i]]
    ok, rvec, tvec = cv2.solvePnP(pts_3d[idx], np.asarray(kp_2d)[idx], camera_K, None,
                                   flags=cv2.SOLVEPNP_EPNP)
    if not ok:
        print('  solvePnP a échoué sur la pose de référence — abandon du test pose-fixée.')
        return []
    rvec = np.asarray(rvec, dtype=np.float64).flatten()
    tvec = np.asarray(tvec, dtype=np.float64).flatten()

    results = []
    for q0 in _candidate_inits():
        res = solve_joint_angles_fixed_pose(
            kp_2d, kp_valid, camera_K, rvec, tvec,
            q_init=q0, reg_weight=0.0,
        )
        if res is None:
            continue
        results.append(res)

    clusters = _cluster(results)
    clusters.sort(key=lambda c: min(r['rms_reproj_px'] for r in c[0]))
    print(f'{len(results)} solves, {len(clusters)} minimum/a distinct(s)\n')
    for i, (members, rep_q) in enumerate(clusters):
        best = min(members, key=lambda r: r['rms_reproj_px'])
        print(f'--- Minimum #{i+1} ({len(members)}/{len(results)} départs y convergent) ---')
        print(f'  q (deg)       : {np.degrees(best["q"]).round(1).tolist()}')
        print(f'  rms_reproj_px : {best["rms_reproj_px"]:.3f}')
    return clusters


def main():
    if not FRAME_PATH.exists():
        print(f'Missing {FRAME_PATH} — capture one first.', file=sys.stderr)
        sys.exit(1)
    data = json.loads(FRAME_PATH.read_text())
    kp_2d = np.array(data['kp_2d'], dtype=np.float64)
    kp_valid = np.array(data['kp_valid'], dtype=bool)
    camera_K = np.array(data['camera_K'], dtype=np.float64)
    encoder_q_rad = np.radians(np.array(data['encoder_q_deg'], dtype=np.float64))

    print(f'Frame: {int(kp_valid.sum())}/7 keypoints valides, encoder q(deg)={np.degrees(encoder_q_rad).round(2).tolist()}')

    free_clusters = run_joint_pose_free(kp_2d, kp_valid, camera_K)
    fixed_clusters = run_fixed_pose(kp_2d, kp_valid, camera_K, encoder_q_rad)

    print('\n=== Conclusion ===')
    print(f'Pose libre  : {len(free_clusters)} minimum/a distinct(s)')
    print(f'Pose fixée  : {len(fixed_clusters)} minimum/a distinct(s)')


if __name__ == '__main__':
    main()
