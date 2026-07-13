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


def _project(q, rvec, tvec, camera_K):
    positions, _ = forward_kinematics(q)
    pts_3d = np.array([positions[name] for name in KEYPOINT_NAMES], dtype=np.float64)
    pts_2d, _ = cv2.projectPoints(pts_3d, rvec, tvec, camera_K, None)
    return pts_2d.reshape(-1, 2)


def _residuals(params, kp_2d, kp_valid, camera_K):
    q, rvec, tvec = params[:6], params[6:9], params[9:12]
    proj = _project(q, rvec, tvec, camera_K)
    diffs = [proj[i] - kp_2d[i] for i in range(len(kp_valid)) if kp_valid[i]]
    return np.concatenate(diffs) if diffs else np.zeros(2)


def solve_joint_angles_and_pose(
    kp_2d, kp_valid, camera_K,
    q_init=None, rvec_init=None, tvec_init=None,
):
    """Jointly recover 6 joint angles (rad) + camera pose from 2D keypoints.

    Parameters
    ----------
    kp_2d : (7, 2) array-like of pixel coordinates, one row per KEYPOINT_NAMES entry.
    kp_valid : (7,) array-like of bool — which detections to trust.
    camera_K : (3, 3) camera intrinsic matrix.
    q_init, rvec_init, tvec_init : optional warm-start (e.g. previous frame's
        solution) to keep the solver fast and avoid local minima jumps.

    Returns
    -------
    dict with q (rad), rvec, tvec, rms_reproj_px, success, n_valid — or None
    if fewer than 4 valid keypoints were detected.
    """
    kp_valid = list(kp_valid)
    n_valid = sum(kp_valid)
    if n_valid < _MIN_KEYPOINTS:
        return None

    kp_2d = np.asarray(kp_2d, dtype=np.float64)

    if q_init is None:
        q_init = np.zeros(6)
    q_init = np.clip(q_init, JOINT_LOWER, JOINT_UPPER)

    if rvec_init is None or tvec_init is None:
        positions, _ = forward_kinematics(q_init)
        pts_3d = np.array([positions[n] for n in KEYPOINT_NAMES], dtype=np.float64)
        idx = [i for i in range(7) if kp_valid[i]]
        ok, rvec_init, tvec_init = cv2.solvePnP(
            pts_3d[idx], kp_2d[idx], camera_K, None, flags=cv2.SOLVEPNP_EPNP)
        if not ok:
            rvec_init, tvec_init = np.zeros(3), np.array([0.0, 0.0, 1.0])
        rvec_init = np.asarray(rvec_init, dtype=np.float64).flatten()
        tvec_init = np.asarray(tvec_init, dtype=np.float64).flatten()

    x0 = np.concatenate([q_init, rvec_init, tvec_init])
    lower = np.concatenate([JOINT_LOWER, [-np.pi] * 3, [-5.0] * 3])
    upper = np.concatenate([JOINT_UPPER, [np.pi] * 3, [5.0] * 3])
    # least_squares(trf) requires x0 strictly inside (lower, upper).
    x0 = np.clip(x0, lower + 1e-6, upper - 1e-6)

    result = least_squares(
        _residuals, x0, bounds=(lower, upper),
        args=(kp_2d, kp_valid, camera_K),
        method='trf', max_nfev=200,
    )

    q, rvec, tvec = result.x[:6], result.x[6:9], result.x[9:12]
    reproj = _project(q, rvec, tvec, camera_K)
    px_err = np.linalg.norm(reproj - kp_2d, axis=1)
    rms_px = float(np.sqrt(np.mean(px_err[kp_valid] ** 2)))

    return {
        'q': q,
        'rvec': rvec,
        'tvec': tvec,
        'rms_reproj_px': rms_px,
        'success': bool(result.success),
        'n_valid': n_valid,
    }
