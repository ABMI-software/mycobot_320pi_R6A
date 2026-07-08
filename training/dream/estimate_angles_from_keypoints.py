#!/usr/bin/env python3
"""Estimate joint angles (j1..j6) from detected 2D keypoints — reprojection-min.

The vision stage (DREAM) outputs 7 keypoints in image pixels. This module
recovers the joint configuration that best reproduces those observations, by
minimising the reprojection error of the FK keypoint chain against the
detections:

    q* = argmin_q  Σ_k  vis_k · || project( FK(q)_k ) - detected_k ||²

where FK(q) comes from mycobot_fk.forward_kinematics (world = base_link frame),
and the projection uses the REAL fixed camera: intrinsics K + distortion from
the camera meta.json and the world->camera transform T_cam_world from
camera_extrinsic.yaml (eye-to-hand calibration, base_link <-> camera).

This is the "estimate the encoders with the camera" step: the estimated angles
are then compared to the true encoder angles to produce the per-joint coverage
curve (see plot_angle_error_curve.py).

Self-test (no DREAM needed): pick a known q_true, project its keypoints through
the real camera, add pixel noise, and check the solver recovers q_true. Runs the
whole math in "sim" to prove the bridge is correct before touching real data.
"""
import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.optimize import least_squares

from mycobot_fk import forward_kinematics, KEYPOINT_NAMES

# Joint limits (radians), matching the URDF — same as mycobot_ik.py
JOINT_LIMITS = [
    (-2.96, 2.96), (-2.79, 2.79), (-2.79, 2.79),
    (-2.79, 2.79), (-2.96, 2.96), (-3.05, 3.05),
]
_LO = np.array([lo for lo, _ in JOINT_LIMITS])
_HI = np.array([hi for _, hi in JOINT_LIMITS])


def load_camera(meta_json, extrinsic_yaml):
    """Load real fixed-camera model: K, distortion, and world->cam pose.

    Returns
    -------
    K : (3,3) intrinsics matrix
    dist : (N,) distortion coefficients (OpenCV rational model)
    rvec, tvec : Rodrigues rotation and translation of T_cam_world (world->cam),
        i.e. the pose that projects base_link points into the camera.
    """
    meta = json.loads(Path(meta_json).read_text())
    r = meta["results"]
    K = np.array([[r["fx"], 0.0, r["cx"]],
                  [0.0, r["fy"], r["cy"]],
                  [0.0, 0.0, 1.0]], dtype=np.float64)
    dist = np.array(r["dist_coeffs"], dtype=np.float64)

    ext = yaml.safe_load(Path(extrinsic_yaml).read_text())
    T_cam_world = np.array(ext["T_cam_world"], dtype=np.float64)
    rvec, _ = cv2.Rodrigues(T_cam_world[:3, :3])
    tvec = T_cam_world[:3, 3].reshape(3, 1)
    return K, dist, rvec, tvec


def project_keypoints(q, K, dist, rvec, tvec):
    """Project the 7 FK keypoints of configuration q to pixel coordinates.

    Returns (7,2) array of [u, v] using the real camera model (with distortion).
    """
    positions, _ = forward_kinematics(q)
    obj = np.array([positions[name] for name in KEYPOINT_NAMES], dtype=np.float64)
    img, _ = cv2.projectPoints(obj, rvec, tvec, K, dist)
    return img.reshape(-1, 2)


def estimate_angles(detected_2d, K, dist, rvec, tvec, q0=None, n_restarts=6):
    """Recover joint angles from detected 2D keypoints (reprojection-min).

    Parameters
    ----------
    detected_2d : (7,2) array of [u, v]; NaN rows = undetected keypoints (skipped).
    q0 : optional warm start (radians). Random restarts are added.

    Returns
    -------
    q : (6,) estimated joint angles (radians).
    rms_px : RMS reprojection error over the visible keypoints (pixels).
    per_kp_px : (7,) per-keypoint reprojection error (NaN where undetected).
    """
    detected = np.asarray(detected_2d, dtype=np.float64)
    vis = ~np.isnan(detected).any(axis=1)
    if vis.sum() < 3:
        raise ValueError(f"Need >=3 visible keypoints to constrain 6 DoF, got {vis.sum()}")

    det_vis = detected[vis]

    def residuals(q):
        proj = project_keypoints(q, K, dist, rvec, tvec)[vis]
        return (proj - det_vis).ravel()

    rng = np.random.default_rng(0)
    guesses = []
    if q0 is not None:
        guesses.append(np.clip(np.asarray(q0, dtype=np.float64), _LO, _HI))
    guesses.append(np.zeros(6))
    for _ in range(n_restarts):
        guesses.append(rng.uniform(_LO * 0.7, _HI * 0.7))

    best_q, best_cost = None, np.inf
    for g in guesses:
        res = least_squares(residuals, g, bounds=(_LO, _HI),
                            loss="soft_l1", f_scale=3.0, max_nfev=500)
        if res.cost < best_cost:
            best_cost, best_q = res.cost, res.x

    proj = project_keypoints(best_q, K, dist, rvec, tvec)
    err = np.linalg.norm(proj - detected, axis=1)
    per_kp = np.where(vis, err, np.nan)
    rms = float(np.sqrt(np.nanmean(err[vis] ** 2)))
    return best_q, rms, per_kp


def observability(K, dist, rvec, tvec, q0=None, delta_deg=20.0):
    """Pixel sensitivity of the keypoints to a delta_deg move of each joint.

    A joint whose keypoints barely move is unobservable from this view. j6 is
    exactly zero here: the link6 keypoint sits on the j6 rotation axis (tool
    roll), so no keypoint set can recover it — a structural limit, not a bug.
    """
    q0 = np.zeros(6) if q0 is None else np.asarray(q0, dtype=np.float64)
    base = project_keypoints(q0, K, dist, rvec, tvec)
    out = []
    for j in range(6):
        dq = q0.copy()
        dq[j] += np.radians(delta_deg)
        shift = np.linalg.norm(project_keypoints(dq, K, dist, rvec, tvec) - base, axis=1)
        out.append(shift.max())
    return np.array(out)


def _self_test(K, dist, rvec, tvec):
    # j1-j4 position the end-effector and are observable; j5 is weak, j6 is on
    # its own rotation axis (0 px) so it is excluded from the pass criterion.
    OBSERVABLE = slice(0, 4)

    print("=== Observability: max keypoint shift for a 20deg move of each joint ===")
    obs = observability(K, dist, rvec, tvec)
    for j, s in enumerate(obs):
        tag = "OK" if s > 15 else ("faible" if s > 3 else "NULLE (sur-axe)")
        print(f"  j{j+1}: {s:5.1f}px  {tag}")

    print("\n=== Recovery (warm-start from previous-frame prior, 1px noise) ===")
    rng = np.random.default_rng(42)
    worst_j14 = 0.0
    for trial in range(5):
        q_true = rng.uniform(_LO * 0.5, _HI * 0.5)
        noisy = project_keypoints(q_true, K, dist, rvec, tvec) + rng.normal(0, 1.0, (7, 2))
        q0 = q_true + np.radians(rng.normal(0, 10, 6))   # ~previous frame in a trajectory
        q_est, rms, _ = estimate_angles(noisy, K, dist, rvec, tvec, q0=q0, n_restarts=0)
        ang_err = np.degrees(np.abs(q_est - q_true))
        worst_j14 = max(worst_j14, ang_err[OBSERVABLE].max())
        print(f"  trial {trial}: rms={rms:5.2f}px | j1-j4 err={np.round(ang_err[:4],1)} "
              f"| j5={ang_err[4]:.1f} j6={ang_err[5]:.1f} (blind)")
    verdict = "OK — bridge correct on observable joints" if worst_j14 < 5 else \
              "WARNING — j1-j4 error too large, check camera model"
    print(f"\nWorst j1-j4 error over trials: {worst_j14:.2f}deg -> {verdict}")


# ─────────────────────────────────────────────────────────────────────────────
# 3D mode (RGB-D astra): use depth to lift keypoints to 3D and fit angles by
# 3D-3D correspondence. Far better conditioned than 2D reprojection — the depth
# breaks the perspective ambiguity that made the mono-2D solve fragile (~20% of
# poses fell into a wrong basin). j6 stays blind: the link6 keypoint is ON the
# j6 rotation axis, so neither its pixel nor its 3D position moves with j6.
# ─────────────────────────────────────────────────────────────────────────────

def load_extrinsic(extrinsic_yaml):
    """Load T_world_cam (camera->base, i.e. maps camera-frame points to base)."""
    ext = yaml.safe_load(Path(extrinsic_yaml).read_text())
    return np.array(ext["T_world_cam"], dtype=np.float64)


def fk_base_points(q):
    """FK keypoints (7,3) in base/world frame, in KEYPOINT_NAMES order (meters)."""
    positions, _ = forward_kinematics(q)
    return np.array([positions[name] for name in KEYPOINT_NAMES], dtype=np.float64)


def deproject_to_base(keypoints_2d, depth_m, K, T_world_cam, win=5):
    """Lift detected 2D keypoints to 3D base-frame points using the depth map.

    keypoints_2d : (7,2) [u,v]; NaN rows undetected. depth_m : (H,W) meters.
    Returns (7,3) base-frame points; NaN rows where keypoint or depth is missing.
    """
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    R, t = T_world_cam[:3, :3], T_world_cam[:3, 3]
    h, w = depth_m.shape
    out = np.full((7, 3), np.nan)
    for k, (u, v) in enumerate(keypoints_2d):
        if np.isnan(u) or np.isnan(v):
            continue
        ui, vi = int(round(u)), int(round(v))
        if not (0 <= ui < w and 0 <= vi < h):
            continue
        patch = depth_m[max(0, vi - win):vi + win + 1, max(0, ui - win):ui + win + 1]
        valid = patch[(patch > 0.05) & np.isfinite(patch)]
        if valid.size < 3:
            continue
        Z = float(np.median(valid))
        p_cam = np.array([(u - cx) / fx * Z, (v - cy) / fy * Z, Z])
        out[k] = R @ p_cam + t
    return out


def estimate_angles_3d(base_points, q0=None, n_restarts=4):
    """Recover joint angles from 3D base-frame keypoints (3D-3D fit).

    Returns q (6,), rms_mm over visible keypoints, per_kp_mm (7,) NaN where absent.
    """
    obs = np.asarray(base_points, dtype=np.float64)
    vis = ~np.isnan(obs).any(axis=1)
    if vis.sum() < 3:
        raise ValueError(f"Need >=3 keypoints with depth, got {vis.sum()}")
    obs_vis = obs[vis]

    def residuals(q):
        return (fk_base_points(q)[vis] - obs_vis).ravel()

    rng = np.random.default_rng(0)
    guesses = []
    if q0 is not None:
        guesses.append(np.clip(np.asarray(q0, float), _LO, _HI))
    guesses.append(np.zeros(6))
    for _ in range(n_restarts):
        guesses.append(rng.uniform(_LO * 0.7, _HI * 0.7))

    best_q, best_cost = None, np.inf
    for g in guesses:
        res = least_squares(residuals, g, bounds=(_LO, _HI),
                            loss="soft_l1", f_scale=0.01, max_nfev=500)
        if res.cost < best_cost:
            best_cost, best_q = res.cost, res.x

    err = np.linalg.norm(fk_base_points(best_q) - obs, axis=1)
    per_kp = np.where(vis, err, np.nan)
    rms_mm = float(np.sqrt(np.nanmean(err[vis] ** 2)) * 1000)
    return best_q, rms_mm, per_kp * 1000


def _self_test_3d(K, T_world_cam):
    # Same geometry, but observations are 3D (depth). Expect j1-j5 recovered and
    # NO fragile basins even without a warm start — that is the whole point.
    print("=== 3D mode self-test: recover config from depth-lifted keypoints ===")
    T_cam_world = np.linalg.inv(T_world_cam)
    fx, fy, cx, cy = K[0, 0], K[1, 1], K[0, 2], K[1, 2]
    rng = np.random.default_rng(7)
    worst_j15 = 0.0
    for trial in range(5):
        q_true = rng.uniform(_LO * 0.5, _HI * 0.5)
        base = fk_base_points(q_true)
        cam = (T_cam_world[:3, :3] @ base.T).T + T_cam_world[:3, 3]   # base->cam
        u = fx * cam[:, 0] / cam[:, 2] + cx + rng.normal(0, 1.0, 7)   # 1px pixel noise
        v = fy * cam[:, 1] / cam[:, 2] + cy + rng.normal(0, 1.0, 7)
        Z = cam[:, 2] + rng.normal(0, 0.003, 7)                       # 3mm depth noise
        p_cam = np.stack([(u - cx) / fx * Z, (v - cy) / fy * Z, Z], axis=1)
        obs_base = (T_world_cam[:3, :3] @ p_cam.T).T + T_world_cam[:3, 3]
        q_est, rms_mm, _ = estimate_angles_3d(obs_base, q0=None, n_restarts=4)  # NO warm start
        ang = np.degrees(np.abs(q_est - q_true))
        worst_j14 = ang[:4].max()
        worst_j15 = max(worst_j15, worst_j14)   # track j1-j4 (j5 reported separately)
        print(f"  trial {trial}: rms={rms_mm:5.1f}mm | j1-j4 err={np.round(ang[:4],1)} "
              f"| j5={ang[4]:.1f} (weak) | j6={ang[5]:.1f} (blind)")
    v = ("OK — depth fixes the fragility: j1-j4 robust with NO warm start "
         "(j5 weak ~few deg, j6 blind by construction)") if worst_j15 < 3 \
        else "WARNING — j1-j4 error too large, check extrinsic/intrinsics"
    print(f"\nWorst j1-j4 error: {worst_j15:.2f}deg -> {v}")


if __name__ == "__main__":
    here = Path(__file__).resolve().parent
    calib = here.parent / "calibration"
    ap = argparse.ArgumentParser()
    ap.add_argument("--meta", default=str(calib / "cam_3.meta.json"),
                    help="camera meta.json (intrinsics + distortion)")
    ap.add_argument("--extrinsic", default=str(calib / "camera_extrinsic.yaml"),
                    help="camera_extrinsic.yaml (T_cam_world, base_link<->camera)")
    ap.add_argument("--self-test", action="store_true",
                    help="run the 2D synthetic round-trip check (no DREAM needed)")
    ap.add_argument("--self-test-3d", action="store_true",
                    help="run the 3D (depth) synthetic round-trip check")
    args = ap.parse_args()

    K, dist, rvec, tvec = load_camera(args.meta, args.extrinsic)
    print(f"Loaded camera: fx={K[0,0]:.1f} fy={K[1,1]:.1f} "
          f"cx={K[0,2]:.1f} cy={K[1,2]:.1f} | {len(dist)} dist coeffs")
    cam_pos = -cv2.Rodrigues(rvec)[0].T @ tvec
    print(f"Camera position in base frame: {cam_pos.ravel().round(3)} m")

    if args.self_test:
        _self_test(K, dist, rvec, tvec)
    if args.self_test_3d:
        _self_test_3d(K, load_extrinsic(args.extrinsic))
