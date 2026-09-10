#!/usr/bin/env python3
"""Articulated pose solver — recover joint angles from DREAM 2D keypoints.

dream_inference_node's solvePnP assumes the joint angles are already known
(from encoders) and only solves for the 6-DoF camera pose. That's fine for
overlaying encoder-reconstructed joints on the image, but it cannot produce
an independent "DREAM angle estimate" to compare against the encoders — the
encoder angles are baked into its 3D points.

This module solves for BOTH the 6 joint angles and the 6-DoF camera pose at
once, from the DREAM 2D keypoint detections and the camera intrinsic matrix
alone. No pre-calibrated camera->robot extrinsic is used or needed — a fixed
extrinsic goes stale the moment the camera is physically moved (as happened
with the arducam hand-eye calibration between capture sessions).
"""

import numpy as np
import cv2
from scipy.optimize import least_squares

from mycobot_fk import forward_kinematics, KEYPOINT_NAMES

# From mycobot_pro_320_pi_gazebo_nogripper.urdf <limit> tags, joints 1..6.
JOINT_LOWER = np.array([-2.93, -2.35, -2.53, -2.53, -2.93, -3.14])
JOINT_UPPER = np.array([2.93, 2.35, 2.53, 2.53, 2.93, 3.14])

_MIN_KEYPOINTS = 4

# How close (rad) to a joint's mechanical limit counts as "the optimizer ran
# to the boundary" rather than "the true angle happens to be near its limit".
# ~0.57° — tight enough that a genuine near-limit pose won't false-positive,
# loose enough to catch trf actually pinning against the bound.
_JOINT_LIMIT_EPS = 0.01


def _project(q, rvec, tvec, camera_K):
    positions, _ = forward_kinematics(q)
    pts_3d = np.array([positions[name] for name in KEYPOINT_NAMES], dtype=np.float64)
    pts_2d, _ = cv2.projectPoints(pts_3d, rvec, tvec, camera_K, None)
    return pts_2d.reshape(-1, 2)


def solve_joint_angles_and_pose(
    kp_2d, kp_valid, camera_K,
    q_init=None, rvec_init=None, tvec_init=None,
    reg_weight=1.5, pose_reg_weight=5.0, loss='linear', f_scale=10.0,
    max_drot_rad=None, max_dtrans_m=None,
):
    """Jointly recover 6 joint angles (rad) + camera pose from 2D keypoints,
    fresh every frame — no persisted or session-frozen extrinsic. This is the
    "camera mobile" case: a fixed extrinsic goes stale the moment the camera
    is physically moved, so pose is re-estimated alongside the angles on
    every call, warm-started from the PREVIOUS frame's own solution for
    temporal continuity (the joint q+pose fit is otherwise gauge-ambiguous —
    see module docstring — warm-starting keeps consecutive frames near the
    same local minimum instead of jumping between equally-valid solutions).

    Parameters
    ----------
    kp_2d : (7, 2) array-like of pixel coordinates, one row per KEYPOINT_NAMES entry.
    kp_valid : (7,) array-like of bool — which detections to trust.
    camera_K : (3, 3) camera intrinsic matrix.
    q_init, rvec_init, tvec_init : optional warm-start (e.g. previous frame's
        solution) to keep the solver fast and avoid local minima jumps.
    reg_weight, loss, f_scale : same meaning as in
        solve_joint_angles_fixed_pose — a soft prior pulling q back towards
        q_init, which is what keeps a structurally-unobserved joint (J6) held
        near its previous estimate instead of drifting on a flat cost
        surface, and a robust loss to keep one bad keypoint from
        contaminating the whole fit.
    pose_reg_weight : soft prior pulling (rvec, tvec) back towards
        (rvec_init, tvec_init), same units/mechanism as reg_weight. Without
        this, empirically (see training/dream/dream_angle_solver.py commit
        history, 2026-07-15), J1 loses observability too, not just J6: a
        near top-down camera view makes "rotate the base joint" and "roll
        the camera about the same axis" produce nearly identical
        reprojections, and with q penalized by reg_weight but pose
        completely free, the optimizer prefers to explain real J1 motion as
        a pose drift instead — a synthetic warm-start test showed a genuine
        1° J1 move getting almost entirely absorbed into rvec instead
        (0.003° tracked) at pose_reg_weight=0, dropping to ~0.02° residual
        error at pose_reg_weight=20. This assumes the camera moves much less
        frame-to-frame than the arm typically does; a genuine camera bump
        will still be tracked, just damped, rather than the pose being
        completely free every frame.
    max_drot_rad, max_dtrans_m : optional HARD per-frame bound on how far
        (rvec, tvec) may move from (rvec_init, tvec_init) — a literal
        least_squares box constraint, not just a soft prior. Only applied
        when rvec_init/tvec_init were actually supplied (a real warm start);
        a cold/fresh solve (rvec_init=None) stays pose-unconstrained, so
        this never degrades into a permanently fixed extrinsic. Approximates
        a geodesic bound on the rotation/translation delta with a per-axis
        box (each of rvec's 3 components independently within
        rvec_init[i] +/- max_drot_rad) — not an exact bound on total
        rotation magnitude (a corner of the box can be up to sqrt(3) times
        max_drot_rad from center), but consistent with how every other bound
        in this solver (JOINT_LOWER/UPPER, the +/-pi/+/-5 pose box) is
        already expressed, and simple enough to reason about. Added
        2026-07-15 after diagnose_local_minima.py showed jac_cond ~1e17-1e18
        (numerically singular) and 19/19 multi-starts landing on 19 distinct
        minima with pose left fully free — pose_reg_weight alone (a soft
        prior) wasn't enough to stop the fit from wandering along that
        near-flat direction. See solve_joint_angles_and_pose_bounded for the
        rupture-detection wrapper that relaxes this bound (instead of
        silently degrading tracking) when the camera genuinely moved more
        than the bound allows in one frame.

    Returns
    -------
    dict with q (rad), rvec, tvec, rms_reproj_px, success, n_valid,
    joint_limit_hit, solver_failed, outlier_px_mask, n_outliers, plus raw
    scipy.optimize.least_squares diagnostics (cost, nfev, optimality,
    jac, jac_cond — the last is np.linalg.cond(jac), a large value flagging
    a near-singular/rank-deficient Jacobian, i.e. a direction in parameter
    space the pixel residuals barely constrain — the signature of a
    gauge ambiguity like J1-vs-camera-roll, see
    training/dream/diagnose_local_minima.py) — or None if fewer than 4
    valid keypoints were detected.
    """
    kp_valid = list(kp_valid)
    n_valid = sum(kp_valid)
    if n_valid < _MIN_KEYPOINTS:
        return None

    kp_2d = np.asarray(kp_2d, dtype=np.float64)

    if q_init is None:
        q_init = np.zeros(6)
    q_init = np.clip(q_init, JOINT_LOWER + 1e-6, JOINT_UPPER - 1e-6)
    q_prior = q_init.copy()

    # A real warm start (both supplied) is what a per-frame pose-delta bound
    # is meant to constrain — a cold/fresh solve (either missing) has no
    # "previous frame" to be close to, so it must stay pose-unconstrained.
    had_warm_start_pose = rvec_init is not None and tvec_init is not None

    if not had_warm_start_pose:
        positions, _ = forward_kinematics(q_init)
        pts_3d = np.array([positions[n] for n in KEYPOINT_NAMES], dtype=np.float64)
        idx = [i for i in range(7) if kp_valid[i]]
        ok, rvec_init, tvec_init = cv2.solvePnP(
            pts_3d[idx], kp_2d[idx], camera_K, None, flags=cv2.SOLVEPNP_EPNP)
        if not ok:
            rvec_init, tvec_init = np.zeros(3), np.array([0.0, 0.0, 1.0])
        rvec_init = np.asarray(rvec_init, dtype=np.float64).flatten()
        tvec_init = np.asarray(tvec_init, dtype=np.float64).flatten()
    rvec_prior = np.asarray(rvec_init, dtype=np.float64).flatten()
    tvec_prior = np.asarray(tvec_init, dtype=np.float64).flatten()

    x0 = np.concatenate([q_init, rvec_init, tvec_init])
    lower = np.concatenate([JOINT_LOWER, [-np.pi] * 3, [-5.0] * 3])
    upper = np.concatenate([JOINT_UPPER, [np.pi] * 3, [5.0] * 3])

    if had_warm_start_pose and max_drot_rad is not None and max_dtrans_m is not None:
        pose_lower = np.concatenate([rvec_prior - max_drot_rad, tvec_prior - max_dtrans_m])
        pose_upper = np.concatenate([rvec_prior + max_drot_rad, tvec_prior + max_dtrans_m])
        lower[6:12] = np.maximum(lower[6:12], pose_lower)
        upper[6:12] = np.minimum(upper[6:12], pose_upper)

    # least_squares(trf) requires x0 strictly inside (lower, upper).
    x0 = np.clip(x0, lower + 1e-6, upper - 1e-6)

    def residuals(params):
        q, rvec, tvec = params[:6], params[6:9], params[9:12]
        proj = _project(q, rvec, tvec, camera_K)
        diffs = [proj[i] - kp_2d[i] for i in range(len(kp_valid)) if kp_valid[i]]
        pixel_res = np.concatenate(diffs) if diffs else np.zeros(2)
        reg_res = reg_weight * (q - q_prior)
        pose_reg_res = pose_reg_weight * np.concatenate([rvec - rvec_prior, tvec - tvec_prior])
        return np.concatenate([pixel_res, reg_res, pose_reg_res])

    result = least_squares(
        residuals, x0, bounds=(lower, upper),
        method='trf', max_nfev=200,
        loss=loss, f_scale=f_scale,
    )

    q, rvec, tvec = result.x[:6], result.x[6:9], result.x[9:12]
    reproj = _project(q, rvec, tvec, camera_K)
    px_err = np.linalg.norm(reproj - kp_2d, axis=1)
    rms_px = float(np.sqrt(np.mean(px_err[kp_valid] ** 2)))

    outlier_px_mask = [bool(kp_valid[i] and px_err[i] > f_scale) for i in range(len(kp_valid))]

    joint_limit_hit = [
        bool(q[i] <= JOINT_LOWER[i] + _JOINT_LIMIT_EPS or q[i] >= JOINT_UPPER[i] - _JOINT_LIMIT_EPS)
        for i in range(6)
    ]
    solver_failed = any(joint_limit_hit)

    try:
        jac_cond = float(np.linalg.cond(result.jac))
    except np.linalg.LinAlgError:
        jac_cond = float('inf')

    return {
        'q': q,
        'rvec': rvec,
        'tvec': tvec,
        'rms_reproj_px': rms_px,
        'success': bool(result.success),
        'n_valid': n_valid,
        'joint_limit_hit': joint_limit_hit,
        'solver_failed': solver_failed,
        'outlier_px_mask': outlier_px_mask,
        'n_outliers': int(sum(outlier_px_mask)),
        'cost': float(result.cost),
        'nfev': int(result.nfev),
        'optimality': float(result.optimality),
        'jac': result.jac,
        'jac_cond': jac_cond,
    }


def solve_joint_angles_and_pose_two_pass(
    kp_2d, kp_valid, camera_K,
    q_init=None, rvec_init=None, tvec_init=None,
    reg_weight=1.5, pose_reg_weight=5.0, loss='linear', f_scale=10.0,
    max_drot_rad=None, max_dtrans_m=None,
):
    """Two-pass outlier-rejection variant of solve_joint_angles_and_pose: fit
    once, drop any keypoint whose reprojection error then exceeds f_scale,
    and refit without it — only if enough valid keypoints (>= _MIN_KEYPOINTS)
    remain afterwards. Never persists the rejection across calls: kp_valid
    for the NEXT frame's own call is unaffected.

    Same rationale as solve_joint_angles_two_pass (see that docstring), just
    without a pre-known camera pose — pose is re-estimated jointly with the
    angles on both passes. max_drot_rad/max_dtrans_m (see
    solve_joint_angles_and_pose) apply to BOTH passes — pass 2 is bounded
    relative to pass 1's own (already-bounded) pose rather than the original
    rvec_init/tvec_init, so worst case the two passes can compound to
    2x the single-frame bound; in practice pass 2 only drops 1-2 outlier
    keypoints and refits; it doesn't hunt for a new pose.
    """
    first = solve_joint_angles_and_pose(
        kp_2d, kp_valid, camera_K,
        q_init=q_init, rvec_init=rvec_init, tvec_init=tvec_init,
        reg_weight=reg_weight, pose_reg_weight=pose_reg_weight, loss=loss, f_scale=f_scale,
        max_drot_rad=max_drot_rad, max_dtrans_m=max_dtrans_m)
    if first is None or first['n_outliers'] == 0:
        if first is not None:
            first['n_rejected_pass2'] = 0
            first['two_pass_applied'] = False
            first['rejected_kp_idx'] = []
            first['pass1_q'] = first['q']
        return first

    kp_valid2 = [v and not out for v, out in zip(kp_valid, first['outlier_px_mask'])]
    if sum(kp_valid2) < _MIN_KEYPOINTS:
        # Not enough independent points left to refit — keep the first pass.
        first['n_rejected_pass2'] = 0
        first['two_pass_applied'] = False
        first['rejected_kp_idx'] = []
        first['pass1_q'] = first['q']
        return first

    second = solve_joint_angles_and_pose(
        kp_2d, kp_valid2, camera_K,
        q_init=q_init, rvec_init=first['rvec'], tvec_init=first['tvec'],
        reg_weight=reg_weight, pose_reg_weight=pose_reg_weight, loss=loss, f_scale=f_scale,
        max_drot_rad=max_drot_rad, max_dtrans_m=max_dtrans_m)
    second['n_rejected_pass2'] = first['n_outliers']
    second['two_pass_applied'] = True
    second['rejected_kp_idx'] = [i for i, out in enumerate(first['outlier_px_mask']) if out]
    # Exposed so a caller can detect pass1/pass2 disagreement — the two
    # passes converging to very different q despite both rejecting the same
    # outliers is itself a sign of an underconstrained/ambiguous fit, on top
    # of whatever per-pixel reprojection error either pass reports alone.
    second['pass1_q'] = first['q']
    return second


def solve_joint_angles_and_pose_bounded(
    kp_2d, kp_valid, camera_K,
    q_init=None, rvec_init=None, tvec_init=None,
    reg_weight=1.5, pose_reg_weight=5.0, loss='linear', f_scale=10.0,
    max_drot_rad=None, max_dtrans_m=None,
    use_two_pass=True,
    rupture_rms_ratio=2.0, rupture_rms_min_px=5.0,
):
    """Solve with a HARD per-frame bound on the camera pose delta (see
    solve_joint_angles_and_pose), plus rupture detection: if the bound is
    pinned (the fit wants to move the pose further than the bound allows)
    AND relaxing it would fit meaningfully better, that's read as the camera
    having genuinely moved — not the fit merely wandering along the
    near-singular pose direction (see diagnose_local_minima.py) — and the
    POSE (only) is re-initialized fresh via a cold solvePnP-based start,
    exactly like the very first solve of a session. q is left to whatever
    warm start/cold-restart machinery the caller already runs elsewhere
    (WarmStartMonitor) — this function only ever resets pose.

    "Meaningfully better" = rupture_rms_ratio x lower reprojection RMS, or
    rupture_rms_min_px px lower in absolute terms, whichever is looser.

    Returns the accepted result dict with two extra keys: 'pose_bound_hit'
    (bool — the hard bound was pinned on at least one axis) and
    'pose_rupture' (bool — judged a genuine camera move, pose reinitialized).
    None if even a fresh solve fails (fewer than _MIN_KEYPOINTS valid).
    """
    solve_fn = solve_joint_angles_and_pose_two_pass if use_two_pass else solve_joint_angles_and_pose

    bound_active = (
        max_drot_rad is not None and max_dtrans_m is not None
        and rvec_init is not None and tvec_init is not None
    )
    bounded_res = solve_fn(
        kp_2d, kp_valid, camera_K,
        q_init=q_init, rvec_init=rvec_init, tvec_init=tvec_init,
        reg_weight=reg_weight, pose_reg_weight=pose_reg_weight, loss=loss, f_scale=f_scale,
        max_drot_rad=max_drot_rad if bound_active else None,
        max_dtrans_m=max_dtrans_m if bound_active else None,
    )
    if bounded_res is None:
        return None
    bounded_res['pose_bound_hit'] = False
    bounded_res['pose_rupture'] = False
    if not bound_active:
        return bounded_res

    rvec_prior = np.asarray(rvec_init, dtype=np.float64).flatten()
    tvec_prior = np.asarray(tvec_init, dtype=np.float64).flatten()
    drot = np.abs(bounded_res['rvec'] - rvec_prior)
    dtrans = np.abs(bounded_res['tvec'] - tvec_prior)
    pose_bound_hit = bool(np.any(drot >= max_drot_rad - 1e-6) or np.any(dtrans >= max_dtrans_m - 1e-6))
    bounded_res['pose_bound_hit'] = pose_bound_hit
    if not pose_bound_hit:
        return bounded_res

    unbounded_res = solve_fn(
        kp_2d, kp_valid, camera_K,
        q_init=q_init, rvec_init=rvec_init, tvec_init=tvec_init,
        reg_weight=reg_weight, pose_reg_weight=pose_reg_weight, loss=loss, f_scale=f_scale)
    if unbounded_res is None:
        return bounded_res

    meaningfully_better = (
        unbounded_res['rms_reproj_px'] * rupture_rms_ratio < bounded_res['rms_reproj_px']
        or bounded_res['rms_reproj_px'] - unbounded_res['rms_reproj_px'] > rupture_rms_min_px
    )
    if not meaningfully_better:
        return bounded_res

    reinit_res = solve_fn(
        kp_2d, kp_valid, camera_K,
        q_init=q_init, rvec_init=None, tvec_init=None,
        reg_weight=reg_weight, pose_reg_weight=0.0, loss=loss, f_scale=f_scale)
    if reinit_res is None:
        bounded_res['pose_rupture'] = True
        return bounded_res
    reinit_res['pose_bound_hit'] = False
    reinit_res['pose_rupture'] = True
    return reinit_res


def _cold_restart_candidates(n_random=4, seed=None):
    """Seed q's for a cold-restart search — deliberately independent of any
    encoder reading (see dream_validation_dashboard's WarmStartMonitor: using
    the encoder to pick or seed the restart would make DREAM's estimate stop
    being an independent validation signal). Zero pose plus a spread of J1
    hypotheses (the documented J1/camera-roll gauge ambiguity, see
    solve_joint_angles_and_pose's docstring, is the most likely thing to trap
    a warm start) plus a few uniform-random draws across the full joint
    envelope for general coverage.
    """
    rng = np.random.default_rng(seed)
    candidates = [np.zeros(6)]
    for j1 in (-1.57, -0.79, 0.79, 1.57):  # +-90 deg, +-45 deg
        c = np.zeros(6)
        c[0] = j1
        candidates.append(c)
    for _ in range(n_random):
        candidates.append(rng.uniform(JOINT_LOWER + 0.05, JOINT_UPPER - 0.05))
    return candidates


def solve_joint_angles_and_pose_multi_init(
    kp_2d, kp_valid, camera_K, candidates_q=None,
    reg_weight=1.5, pose_reg_weight=5.0, loss='linear', f_scale=10.0,
    use_two_pass=True,
):
    """Cold-restart search: solve independently from several candidate q0's
    (each with its own fresh pose fit, NOT warm-started from any previous
    frame) and keep whichever converges to the lowest reprojection error —
    i.e. the best GEOMETRIC self-consistency, the only signal available that
    doesn't compromise DREAM's independence from the encoders.

    Intended for the case where the normal warm-started solve is suspected
    to be stuck in a persistent-but-wrong local minimum (see
    WarmStartMonitor.should_restart in dream_validation_dashboard.py) —
    warm-starting keeps drifting back to the same wrong branch every frame,
    so escaping it requires seeds that don't depend on that branch at all.

    Returns the winning solve's result dict (with 'n_candidates_tried' and
    'cold_restart'=True added), or None if every candidate failed (fewer
    than _MIN_KEYPOINTS valid, or every fit hit a joint limit).
    """
    if candidates_q is None:
        candidates_q = _cold_restart_candidates()

    solve_fn = solve_joint_angles_and_pose_two_pass if use_two_pass else solve_joint_angles_and_pose

    best = None
    n_tried = 0
    for q0 in candidates_q:
        res = solve_fn(
            kp_2d, kp_valid, camera_K,
            q_init=q0, rvec_init=None, tvec_init=None,
            reg_weight=reg_weight, pose_reg_weight=pose_reg_weight, loss=loss, f_scale=f_scale)
        if res is None:
            continue
        n_tried += 1
        if res['solver_failed']:
            continue
        if best is None or res['rms_reproj_px'] < best['rms_reproj_px']:
            best = res

    if best is None:
        return None
    best['n_candidates_tried'] = n_tried
    best['cold_restart'] = True
    return best


def solve_joint_angles_multiview(
    views, q_init=None, poses_init=None,
    reg_weight=1.5, pose_reg_weight=5.0, loss='linear', f_scale=10.0,
):
    """Recover the SIX joint angles shared across N calibrated camera views,
    plus a 6-DoF pose per view, in a single least_squares — the multi-camera
    fusion the monocular solver can't do alone.

    The physical setup: the arm has ONE true joint configuration q; each
    camera sees it from its own (unknown, re-estimated) pose. Fusing views
    adds independent pixel constraints on that shared q, which is what lifts
    the monocular branch/observability ambiguities (a distal joint poorly
    observed in one view is usually well observed in another). No persisted
    extrinsic between cameras is used or needed — each view's pose is solved
    fresh every frame, exactly like the monocular
    solve_joint_angles_and_pose, just with q tied across all of them.

    Parameters
    ----------
    views : list of dicts, one per camera, each with
        'kp_2d'   : (7, 2) pixel detections,
        'kp_valid': (7,) bool,
        'camera_K': (3, 3) intrinsics (already rescaled to the capture size),
        'camera_dist' (optional) : distortion coeffs, or None.
        A view with fewer than _MIN_KEYPOINTS valid detections is dropped from
        the fusion (it can't seed its own pose); the fit proceeds on the rest.
    q_init : shared (6,) warm start (previous frame's fused q, or encoder seed).
    poses_init : optional list of (rvec, tvec) per KEPT view for warm-starting
        each view's pose; missing/None entries get a fresh per-view solvePnP.
    reg_weight, pose_reg_weight, loss, f_scale : same meaning as
        solve_joint_angles_and_pose (reg_weight may be a scalar or a per-joint
        (6,) vector, e.g. the consistency-mode distal prior).

    Returns dict with q (rad), view_rvecs/view_tvecs (lists, one per KEPT
    view), per_view_rms_px, rms_reproj_px (RMS over all kept views' pixel
    residuals), n_views_used, n_valid (total valid kps fused), success,
    joint_limit_hit, solver_failed, kept_view_idx (indices into the input
    `views` that were actually used) — or None if no view had >= 4 valid kps.
    """
    kept = []
    for i, v in enumerate(views):
        val = list(v['kp_valid'])
        if sum(val) >= _MIN_KEYPOINTS:
            kept.append(i)
    if not kept:
        return None

    if q_init is None:
        q_init = np.zeros(6)
    q_init = np.clip(np.asarray(q_init, dtype=np.float64), JOINT_LOWER + 1e-6, JOINT_UPPER - 1e-6)
    q_prior = q_init.copy()

    # Per-view pose warm start: use the supplied one, else a fresh solvePnP at
    # q_init against that view's own valid detections.
    rvec_priors, tvec_priors = [], []
    for slot, i in enumerate(kept):
        v = views[i]
        kp = np.asarray(v['kp_2d'], dtype=np.float64)
        val = list(v['kp_valid'])
        rvec0 = tvec0 = None
        if poses_init is not None and slot < len(poses_init) and poses_init[slot] is not None:
            rvec0, tvec0 = poses_init[slot]
        if rvec0 is None or tvec0 is None:
            positions, _ = forward_kinematics(q_init)
            pts_3d = np.array([positions[n] for n in KEYPOINT_NAMES], dtype=np.float64)
            idx = [k for k in range(7) if val[k]]
            ok, rvec0, tvec0 = cv2.solvePnP(
                pts_3d[idx], kp[idx], v['camera_K'], None, flags=cv2.SOLVEPNP_EPNP)
            if not ok:
                rvec0, tvec0 = np.zeros(3), np.array([0.0, 0.0, 1.0])
        rvec_priors.append(np.asarray(rvec0, dtype=np.float64).flatten())
        tvec_priors.append(np.asarray(tvec0, dtype=np.float64).flatten())

    n = len(kept)
    x0 = np.concatenate([q_init] + [np.concatenate([rvec_priors[s], tvec_priors[s]]) for s in range(n)])
    lower = np.concatenate([JOINT_LOWER] + [np.concatenate([[-np.pi] * 3, [-5.0] * 3])] * n)
    upper = np.concatenate([JOINT_UPPER] + [np.concatenate([[np.pi] * 3, [5.0] * 3])] * n)
    x0 = np.clip(x0, lower + 1e-6, upper - 1e-6)

    def residuals(params):
        q = params[:6]
        res = []
        for slot, i in enumerate(kept):
            base = 6 + slot * 6
            rvec, tvec = params[base:base + 3], params[base + 3:base + 6]
            v = views[i]
            kp = np.asarray(v['kp_2d'], dtype=np.float64)
            val = v['kp_valid']
            proj = _project(q, rvec, tvec, v['camera_K'])
            diffs = [proj[k] - kp[k] for k in range(len(val)) if val[k]]
            if diffs:
                res.append(np.concatenate(diffs))
            res.append(pose_reg_weight * (rvec - rvec_priors[slot]))
            res.append(pose_reg_weight * (tvec - tvec_priors[slot]))
        res.append(reg_weight * (q - q_prior))
        return np.concatenate(res)

    result = least_squares(
        residuals, x0, bounds=(lower, upper),
        method='trf', max_nfev=300, loss=loss, f_scale=f_scale)

    q = result.x[:6]
    view_rvecs, view_tvecs, per_view_rms, all_err, n_valid = [], [], [], [], 0
    for slot, i in enumerate(kept):
        base = 6 + slot * 6
        rvec, tvec = result.x[base:base + 3], result.x[base + 3:base + 6]
        v = views[i]
        kp = np.asarray(v['kp_2d'], dtype=np.float64)
        val = np.asarray(v['kp_valid'], dtype=bool)
        reproj = _project(q, rvec, tvec, v['camera_K'])
        px = np.linalg.norm(reproj - kp, axis=1)
        view_rvecs.append(rvec)
        view_tvecs.append(tvec)
        per_view_rms.append(float(np.sqrt(np.mean(px[val] ** 2))) if val.any() else float('nan'))
        all_err.extend(px[val].tolist())
        n_valid += int(val.sum())

    rms_px = float(np.sqrt(np.mean(np.square(all_err)))) if all_err else float('inf')
    joint_limit_hit = [
        bool(q[j] <= JOINT_LOWER[j] + _JOINT_LIMIT_EPS or q[j] >= JOINT_UPPER[j] - _JOINT_LIMIT_EPS)
        for j in range(6)
    ]

    return {
        'q': q,
        'view_rvecs': view_rvecs,
        'view_tvecs': view_tvecs,
        'per_view_rms_px': per_view_rms,
        'rms_reproj_px': rms_px,
        'n_views_used': n,
        'n_valid': n_valid,
        'success': bool(result.success),
        'joint_limit_hit': joint_limit_hit,
        'solver_failed': any(joint_limit_hit),
        'kept_view_idx': kept,
    }


def solve_joint_angles_fixed_pose(kp_2d, kp_valid, camera_K, rvec, tvec,
                                   q_init=None, reg_weight=1.5,
                                   loss='linear', f_scale=10.0):
    """Recover the 6 joint angles ALONE, given an already-known camera pose.

    Well-posed in the sense of not sharing solve_joint_angles_and_pose's gauge
    ambiguity, but not necessarily well-CONDITIONED: some keypoints carry no
    information about their own joint (a point sitting on a joint's own axis
    doesn't move when that joint rotates — see mycobot320_link6/J6, which no
    keypoint in this 7-point set is sensitive to), and a near top-down camera
    view makes J1 easy to confuse with a camera-yaw error baked into the
    anchor. Without any prior, the optimizer is free to wander arbitrarily far
    along those poorly-observed directions while barely moving the pixel
    reprojection cost.

    `reg_weight` adds a soft penalty (in pixel-equivalent units per radian)
    pulling q back towards q_init. It's intentionally light: for a
    well-observed joint the pixel residuals dominate and the prior barely
    matters, but for an unobserved one (J6) it's the ONLY term with any
    gradient, so the joint holds at q_init instead of drifting to whatever
    the optimizer's line search happens to try first. Pass q_init as the
    PREVIOUS frame's DREAM estimate (not the encoder) for temporal
    smoothness of the DREAM curve itself; falls back to zero as intended.

    Intended use: solve the pose once per session with the ENCODER angles
    (well-posed 6-DoF PnP, see dream_validation_dashboard.compute_overlay),
    freeze it as a session anchor, then call this every frame with only the
    live DREAM detections — no persisted extrinsic file, refreshed on every
    launch.

    `loss`/`f_scale` control how a single bad keypoint detection affects the
    fit — see compare_robust_loss.py for the comparison that motivated this.
    Plain least-squares (`loss='linear'`, the default — unchanged behaviour)
    treats every residual quadratically, so one noisy point (e.g. a bad
    link5 detection) drags the shared pose fit and contaminates every other
    joint's estimate, not just the one that keypoint nominally informs — a
    single outlier's residual dominates the sum of squares. `loss='huber'`
    (scipy) switches a residual from quadratic to linear cost once it
    exceeds `f_scale` pixels, so an outlier still pulls the fit a little but
    can no longer dominate it; `'soft_l1'` is a smoother version of the same
    idea. `f_scale` is the pixel error at which a residual is considered
    "large" — 10px sits above the well-detected 2-6px band documented
    elsewhere and below the ~15-25px this module's outlier investigation
    found on genuinely bad detections.

    Returns dict with q (rad), rms_reproj_px, success, n_valid, and
    outlier_px_mask/n_outliers (keypoints whose FINAL reprojection error
    exceeds f_scale, regardless of which loss was used — a diagnostic, not
    a hard rejection) — or None if fewer than 4 valid keypoints were detected.
    """
    kp_valid = list(kp_valid)
    n_valid = sum(kp_valid)
    if n_valid < _MIN_KEYPOINTS:
        return None

    kp_2d = np.asarray(kp_2d, dtype=np.float64)
    rvec = np.asarray(rvec, dtype=np.float64).flatten()
    tvec = np.asarray(tvec, dtype=np.float64).flatten()

    q_init = np.zeros(6) if q_init is None else np.clip(q_init, JOINT_LOWER, JOINT_UPPER)
    q_init = np.clip(q_init, JOINT_LOWER + 1e-6, JOINT_UPPER - 1e-6)
    q_prior = q_init.copy()

    def residuals(q):
        proj = _project(q, rvec, tvec, camera_K)
        diffs = [proj[i] - kp_2d[i] for i in range(len(kp_valid)) if kp_valid[i]]
        pixel_res = np.concatenate(diffs)
        reg_res = reg_weight * (q - q_prior)
        return np.concatenate([pixel_res, reg_res])

    result = least_squares(
        residuals, q_init, bounds=(JOINT_LOWER, JOINT_UPPER),
        method='trf', max_nfev=200,
        loss=loss, f_scale=f_scale,
    )

    q = result.x
    reproj = _project(q, rvec, tvec, camera_K)
    px_err = np.linalg.norm(reproj - kp_2d, axis=1)
    rms_px = float(np.sqrt(np.mean(px_err[kp_valid] ** 2)))

    # Diagnostic only (see loss/f_scale docstring above) — computed the same
    # way regardless of which loss actually ran, so 'linear' runs report what
    # WOULD have been flagged for comparison against a robust run.
    outlier_px_mask = [bool(kp_valid[i] and px_err[i] > f_scale) for i in range(len(kp_valid))]

    # A joint pinned within epsilon of its mechanical limit almost always
    # means the optimizer ran to the boundary rather than converged on the
    # true value (see module docstring: poorly-observed joints have near-flat
    # pixel cost, so trf's line search can walk them all the way to a bound
    # looking for any residual improvement). Flag it rather than silently
    # returning a physically-extreme "estimate".
    joint_limit_hit = [
        bool(q[i] <= JOINT_LOWER[i] + _JOINT_LIMIT_EPS or q[i] >= JOINT_UPPER[i] - _JOINT_LIMIT_EPS)
        for i in range(6)
    ]
    solver_failed = any(joint_limit_hit)

    return {
        'q': q,
        'rms_reproj_px': rms_px,
        'success': bool(result.success),
        'n_valid': n_valid,
        'joint_limit_hit': joint_limit_hit,
        'solver_failed': solver_failed,
        'outlier_px_mask': outlier_px_mask,
        'n_outliers': int(sum(outlier_px_mask)),
    }


def solve_joint_angles_two_pass(kp_2d, kp_valid, camera_K, rvec, tvec,
                                 q_init=None, reg_weight=1.5,
                                 loss='linear', f_scale=10.0):
    """Diagnostic variant: fit once, drop any keypoint whose reprojection
    error then exceeds f_scale, and refit without it — only if enough valid
    keypoints (>= _MIN_KEYPOINTS) remain afterwards. Never persists the
    rejection: `kp_valid` for the NEXT frame's own call is unaffected, so a
    keypoint that's briefly bad doesn't stay excluded once it recovers.

    Compared against `loss='huber'`'s soft downweighting in
    compare_robust_loss.py — confirmed better on every metric there (2026-
    07-13: lower reprojection, fewer joint-limit hits, less drift, more
    frames solved). Returns the same dict shape as
    solve_joint_angles_fixed_pose, plus 'n_rejected_pass2',
    'two_pass_applied', and 'rejected_kp_idx' (indices into KEYPOINT_NAMES
    actually dropped for the second pass — empty if none were, either
    because nothing looked like an outlier or because dropping it would have
    left fewer than _MIN_KEYPOINTS independent points to refit with).
    """
    first = solve_joint_angles_fixed_pose(
        kp_2d, kp_valid, camera_K, rvec, tvec,
        q_init=q_init, reg_weight=reg_weight, loss=loss, f_scale=f_scale)
    if first is None or first['n_outliers'] == 0:
        if first is not None:
            first['n_rejected_pass2'] = 0
            first['two_pass_applied'] = False
            first['rejected_kp_idx'] = []
        return first

    kp_valid2 = [v and not out for v, out in zip(kp_valid, first['outlier_px_mask'])]
    if sum(kp_valid2) < _MIN_KEYPOINTS:
        # Not enough independent points left to refit — keep the first pass.
        first['n_rejected_pass2'] = 0
        first['two_pass_applied'] = False
        first['rejected_kp_idx'] = []
        return first

    second = solve_joint_angles_fixed_pose(
        kp_2d, kp_valid2, camera_K, rvec, tvec,
        q_init=q_init, reg_weight=reg_weight, loss=loss, f_scale=f_scale)
    second['n_rejected_pass2'] = first['n_outliers']
    second['two_pass_applied'] = True
    second['rejected_kp_idx'] = [i for i, out in enumerate(first['outlier_px_mask']) if out]
    return second
