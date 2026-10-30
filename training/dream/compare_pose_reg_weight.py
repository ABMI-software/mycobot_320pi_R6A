#!/usr/bin/env python3
"""Replay a captured live sequence (/dream/keypoints + /joint_states) through
the SAME warm-start + hard-pose-bound + cold-restart machinery the dashboard
runs live, sweeping pose_reg_weight and the hard per-frame pose-delta bound
on/off, to compare (2026-07-15 request):

  - number of distinct minima accessible per frame (static multi-start,
    reused from diagnose_local_minima.py, on the FIRST frame of the sequence)
  - Jacobian conditioning (jac_cond) of the accepted solve, per frame
  - MAE/RMS angular error vs encoder, pooled over the whole sequence
  - temporal stability (frame-to-frame |delta q|, mean/max, over ACCEPTED
    solves only)
  - cold-restart / pose-rupture frequency (WarmStartMonitor +
    solve_joint_angles_and_pose_bounded)
  - latency (solver wall-clock time per call)

Independence constraint preserved throughout: MAE/RMS vs encoder is used only
to REPORT accuracy, never to steer which minimum the solver accepts (that
selection is always by reprojection error / internal solver signals only,
exactly as in the live dashboard).

Usage: python3 compare_pose_reg_weight.py [sequence.json]
"""
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from dream_angle_solver import (
    solve_joint_angles_and_pose_bounded, solve_joint_angles_and_pose_multi_init,
    JOINT_LOWER, JOINT_UPPER,
)

SEQ_PATH = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(__file__).resolve().parent.parent.parent / 'compare_seq.json'
# fallback: scratch capture path passed explicitly in this session

MAX_CAMERA_ROT_RATE_DEG_S = 10.0
MAX_CAMERA_TRANS_RATE_M_S = 0.03
POSE_BOUND_MAX_DT_S = 2.0

# Same trigger thresholds as WarmStartMonitor in dream_validation_dashboard.py
_RMS_SPIKE_MULT = 3.0
_RMS_SPIKE_MIN_PX = 8.0
_MAX_PLAUSIBLE_JUMP_DEG = 30.0
_LOW_CONF_MIN_KP = 5
_LOW_CONF_STREAK_LIMIT = 5
_PASS_DISAGREEMENT_DEG = 15.0


class WarmStartMonitor:
    """Verbatim copy of dream_validation_dashboard.WarmStartMonitor's logic
    (duplicated here rather than imported, to keep this script importable
    with only dream_angle_solver's light dependencies — the dashboard module
    pulls in PyQt5/rclpy at import time)."""

    def __init__(self):
        self.rms_history = []
        self.prev_q = None
        self.low_conf_streak = 0
        self._last_streak_kp_seq = None

    def should_restart(self, res, n_valid, kp_seq, first_q=None):
        reasons = []
        if self.rms_history:
            baseline = float(np.median(self.rms_history[-10:]))
            if res['rms_reproj_px'] > max(_RMS_SPIKE_MULT * baseline, baseline + _RMS_SPIKE_MIN_PX):
                reasons.append('rms_spike')
        if res['solver_failed']:
            reasons.append('joint_limit')
        if self.prev_q is not None:
            jump_deg = float(np.degrees(np.abs(res['q'] - self.prev_q)).max())
            if jump_deg > _MAX_PLAUSIBLE_JUMP_DEG:
                reasons.append('angle_jump')
        if kp_seq != self._last_streak_kp_seq:
            self._last_streak_kp_seq = kp_seq
            self.low_conf_streak = self.low_conf_streak + 1 if n_valid < _LOW_CONF_MIN_KP else 0
        if self.low_conf_streak >= _LOW_CONF_STREAK_LIMIT:
            reasons.append('low_confidence_streak')
        if first_q is not None:
            disagree_deg = float(np.degrees(np.abs(res['q'] - first_q)).max())
            if disagree_deg > _PASS_DISAGREEMENT_DEG:
                reasons.append('pass_disagreement')
        return (len(reasons) > 0, '+'.join(reasons))

    def accept(self, res):
        self.rms_history.append(res['rms_reproj_px'])
        self.prev_q = res['q'].copy()


def replay(samples, camera_K, pose_reg_weight, use_hard_bound):
    monitor = WarmStartMonitor()
    prev_q = prev_rvec = prev_tvec = None
    prev_pose_time = None
    n_solves = n_cold_restarts = n_pose_ruptures = n_pose_bound_hit = 0
    jac_conds, solver_ms_list, err_deg_list, accepted_qs, accepted_ts = [], [], [], [], []

    t_seq0 = samples[0]['t_recv']
    for i, s in enumerate(samples):
        kp_2d = np.array(s['kp_2d'])
        kp_valid = np.array(s['kp_valid'])
        if kp_valid.sum() < 4 or s['encoder_q_rad'] is None:
            continue

        t_now = s['t_recv']
        max_drot = max_dtrans = None
        if use_hard_bound and prev_pose_time is not None:
            dt = t_now - prev_pose_time
            if 0.0 < dt <= POSE_BOUND_MAX_DT_S:
                max_drot = np.radians(MAX_CAMERA_ROT_RATE_DEG_S) * dt
                max_dtrans = MAX_CAMERA_TRANS_RATE_M_S * dt

        t0 = time.time()
        res = solve_joint_angles_and_pose_bounded(
            kp_2d, kp_valid, camera_K,
            q_init=prev_q if prev_q is not None else np.array(s['encoder_q_rad']),
            rvec_init=prev_rvec, tvec_init=prev_tvec,
            pose_reg_weight=pose_reg_weight,
            max_drot_rad=max_drot, max_dtrans_m=max_dtrans,
            use_two_pass=True,
        )
        n_valid = int(kp_valid.sum())
        need_restart, _ = monitor.should_restart(res, n_valid=n_valid, kp_seq=i, first_q=res.get('pass1_q'))
        if need_restart:
            cand = solve_joint_angles_and_pose_multi_init(kp_2d, kp_valid, camera_K, use_two_pass=True)
            current_rms = res['rms_reproj_px'] if not res['solver_failed'] else float('inf')
            if cand is not None and cand['rms_reproj_px'] < current_rms:
                res = cand
                n_cold_restarts += 1
        t1 = time.time()
        n_solves += 1
        solver_ms_list.append((t1 - t0) * 1000.0)
        if res.get('pose_rupture'):
            n_pose_ruptures += 1
        if res.get('pose_bound_hit'):
            n_pose_bound_hit += 1

        if res['solver_failed']:
            continue

        jac_conds.append(res['jac_cond'])
        monitor.accept(res)
        prev_q, prev_rvec, prev_tvec = res['q'], res['rvec'], res['tvec']
        prev_pose_time = t_now

        enc_deg = np.degrees(np.array(s['encoder_q_rad']))
        dream_deg = np.degrees(res['q'])
        err_deg_list.append(np.abs(dream_deg - enc_deg))
        accepted_qs.append(dream_deg)
        accepted_ts.append(t_now - t_seq0)

    err = np.array(err_deg_list) if err_deg_list else np.zeros((0, 6))
    qs = np.array(accepted_qs) if accepted_qs else np.zeros((0, 6))
    jumps = np.abs(np.diff(qs, axis=0)).max(axis=1) if len(qs) > 1 else np.array([0.0])

    return {
        'pose_reg_weight': pose_reg_weight,
        'hard_bound': use_hard_bound,
        'n_solves': n_solves,
        'n_accepted': len(err_deg_list),
        'n_cold_restarts': n_cold_restarts,
        'n_pose_ruptures': n_pose_ruptures,
        'n_pose_bound_hit': n_pose_bound_hit,
        'restart_rate_pct': 100.0 * n_cold_restarts / max(n_solves, 1),
        'rupture_rate_pct': 100.0 * n_pose_ruptures / max(n_solves, 1),
        'bound_hit_rate_pct': 100.0 * n_pose_bound_hit / max(n_solves, 1),
        'mae_total_deg': float(err.mean()) if len(err) else float('nan'),
        'rms_total_deg': float(np.sqrt((err ** 2).mean())) if len(err) else float('nan'),
        'jac_cond_median': float(np.median(jac_conds)) if jac_conds else float('nan'),
        'jac_cond_max': float(np.max(jac_conds)) if jac_conds else float('nan'),
        'jump_deg_mean': float(jumps.mean()),
        'jump_deg_max': float(jumps.max()),
        'solver_ms_median': float(np.median(solver_ms_list)) if solver_ms_list else float('nan'),
        'solver_ms_max': float(np.max(solver_ms_list)) if solver_ms_list else float('nan'),
    }


def main():
    data = json.loads(SEQ_PATH.read_text())
    camera_K = np.array([
        [496.307502700341, 0.0, 313.37381079861257],
        [0.0, 494.142967106682, 248.00566176810372],
        [0.0, 0.0, 1.0],
    ])

    print(f'{len(data)} captured frames, camera_K = Arducam calibration\n')

    sweep_weights = [0.0, 2.0, 5.0, 10.0, 20.0, 50.0]
    rows = []
    for hard_bound in (False, True):
        for w in sweep_weights:
            r = replay(data, camera_K, pose_reg_weight=w, use_hard_bound=hard_bound)
            rows.append(r)

    header = (f'{"bound":>6} {"reg_w":>6} {"n_ok":>5} {"MAE°":>7} {"RMS°":>7} '
              f'{"jac_med":>10} {"jac_max":>10} {"jump_mean°":>11} {"jump_max°":>10} '
              f'{"restart%":>9} {"rupture%":>9} {"bound_hit%":>11} {"solve_ms":>9}')
    print(header)
    print('-' * len(header))
    for r in rows:
        print(f'{str(r["hard_bound"]):>6} {r["pose_reg_weight"]:>6.1f} {r["n_accepted"]:>5} '
              f'{r["mae_total_deg"]:>7.2f} {r["rms_total_deg"]:>7.2f} '
              f'{r["jac_cond_median"]:>10.2e} {r["jac_cond_max"]:>10.2e} '
              f'{r["jump_deg_mean"]:>11.2f} {r["jump_deg_max"]:>10.2f} '
              f'{r["restart_rate_pct"]:>8.1f}% {r["rupture_rate_pct"]:>8.1f}% '
              f'{r["bound_hit_rate_pct"]:>10.1f}% '
              f'{r["solver_ms_median"]:>8.2f}')

    out_path = Path(__file__).resolve().parent / 'pose_reg_weight_sweep_results.json'
    out_path.write_text(json.dumps(rows, indent=2))
    print(f'\nFull results written to {out_path}')


if __name__ == '__main__':
    main()
