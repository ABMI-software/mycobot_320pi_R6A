#!/usr/bin/env python3
"""J5 observability test — is link6 alone enough to pin down J5?

JOINT_OBSERVING_KP (mycobot_gateway/dream_validation_dashboard.py) says only
keypoint mycobot320_link6 depends on J5's own rotation — every other
keypoint's position is invariant to it (mycobot_fk.forward_kinematics applies
a joint's rotation AFTER the translation to its own link origin, so it never
moves that link, only the ones downstream; J5 is the second-to-last joint, so
only link6 is downstream of it).

One 2D point constraining one angle is fine in principle, but degenerates
badly if the camera views link6's rotation nearly along its own axis: the 3D
circle link6 traces as J5 sweeps then projects to a small, near-degenerate
arc, and many different J5 values land within a few pixels of each other —
indistinguishable to any detector, however accurate the detector itself is.

This script makes that concrete: hold every joint but J5 and the camera pose
fixed, sweep J5 across its full mechanical range, and plot the reprojection
error of link6 (each hypothesis vs. the true J5) as a function of the
hypothesis. A wide flat valley means J5 is intrinsically weakly observable
from this keypoint at this pose — no amount of solver tuning (regularization,
filtering) fixes that; a narrow sharp valley means the current reg_weight /
detector noise is the real bottleneck, not geometry.

No calibration file involved: the camera pose is a synthetic look-at built
directly from --distance/--azimuth/--elevation, not read from any saved
extrinsic. This is a numerical experiment about the kinematic chain's
geometry, not a claim about where the real Arducam actually is.

Usage (venv_dream):
    source ~/ros_jazzy/venv_dream/bin/activate
    python3 j5_observability_test.py
    python3 j5_observability_test.py --pose -0.3,-91.4,-0.9,-0.4,0.1,-0.3
    python3 j5_observability_test.py --azimuth 90 --elevation 10   # side-on view
"""
import argparse
import json
import math
import os
import sys

import cv2
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
from mycobot_fk import forward_kinematics  # noqa: E402

_REPO = os.path.dirname(os.path.dirname(_HERE))
CALIB_PATH = os.path.join(_REPO, 'training', 'calibration', 'cam_3.meta.json')

# From mycobot_pro_320_pi_gazebo_nogripper.urdf <limit> tags — same table as
# dream_angle_solver.py's JOINT_LOWER/UPPER.
JOINT_LOWER = np.array([-2.93, -2.35, -2.53, -2.53, -2.93, -3.14])
JOINT_UPPER = np.array([2.93, 2.35, 2.53, 2.53, 2.93, 3.14])


def load_intrinsics():
    """Real Arducam K + distortion — intrinsics are a lens property, not an
    extrinsic calibration, so this one's fine to read from the charuco file."""
    meta = json.loads(open(CALIB_PATH).read())
    r = meta['results']
    K = np.array([[r['fx'], 0.0, r['cx']],
                  [0.0, r['fy'], r['cy']],
                  [0.0, 0.0, 1.0]], dtype=np.float64)
    dist = np.array(r['dist_coeffs'], dtype=np.float64)
    return K, dist


def build_camera_pose(target, distance, azimuth_deg, elevation_deg):
    """Synthetic look-at camera pose — built in code, not loaded from a file.

    Places the camera on a sphere of `distance` metres around `target`
    (base/world frame), at the given azimuth/elevation, looking at it.
    Returns (rvec, tvec) in the same world->camera convention cv2.solvePnP /
    cv2.projectPoints use everywhere else in this codebase.
    """
    az, el = math.radians(azimuth_deg), math.radians(elevation_deg)
    target = np.asarray(target, dtype=np.float64)
    cam_pos = target - distance * np.array([
        math.cos(el) * math.cos(az), math.cos(el) * math.sin(az), -math.sin(el),
    ])

    forward = target - cam_pos
    forward /= np.linalg.norm(forward)
    world_up = np.array([0.0, 0.0, 1.0])
    right = np.cross(forward, world_up)
    right /= np.linalg.norm(right)
    up = np.cross(right, forward)

    # OpenCV camera axes: x=right, y=down, z=forward. Rows = world->camera.
    R = np.stack([right, -up, forward], axis=0)
    tvec = (-R @ cam_pos).reshape(3, 1)
    rvec, _ = cv2.Rodrigues(R)
    return rvec, tvec


def project_link6(q, rvec, tvec, K, dist):
    positions, _ = forward_kinematics(q)
    p3d = np.array(positions['mycobot320_link6'], dtype=np.float64).reshape(1, 3)
    p2d, _ = cv2.projectPoints(p3d, rvec, tvec, K, dist)
    return p2d.reshape(2)


def sweep(q_base, rvec, tvec, K, dist, n_steps=361):
    """Sweep J5 over its full mechanical range; return (j5_deg, err_px, xy, p_true).

    err_px[i] is the pixel distance between link6's projection at hypothesis
    j5_deg[i] and its projection at the TRUE J5 (q_base[4]) — everything else
    (other joints, camera pose) held fixed throughout.
    """
    p_true = project_link6(q_base, rvec, tvec, K, dist)

    j5_vals = np.linspace(JOINT_LOWER[4], JOINT_UPPER[4], n_steps)
    xy = np.zeros((n_steps, 2))
    for i, j5 in enumerate(j5_vals):
        q = q_base.copy()
        q[4] = j5
        xy[i] = project_link6(q, rvec, tvec, K, dist)
    err_px = np.linalg.norm(xy - p_true, axis=1)
    return np.degrees(j5_vals), err_px, xy, p_true


def summarize(j5_deg, err_px, j5_true_deg, noise_px):
    """Width (deg) of the region within noise_px of the minimum — the
    practical ambiguity band: any J5 in this band reprojects within typical
    detector noise of the true value, so no detector at that precision could
    tell them apart. Also the local slope at the true value: steep = well
    observed, flat = ambiguous.
    """
    within = j5_deg[err_px <= noise_px]
    band_width = float(within.max() - within.min()) if within.size else 0.0
    i_true = int(np.argmin(np.abs(j5_deg - j5_true_deg)))
    lo, hi = max(0, i_true - 5), min(len(j5_deg), i_true + 6)
    local_slope = float(np.mean(np.abs(np.gradient(err_px[lo:hi], j5_deg[lo:hi]))))
    return band_width, local_slope


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument(
        '--pose', type=str, default='-0.3,-91.4,-0.9,-0.4,0.1,-0.3',
        help="Baseline J1..J6 in degrees (J5 = the 'true' value being tested); "
             "default is the pose from the dashboard's 2026-07-13 anchor capture")
    ap.add_argument('--target', type=str, default='0.25,0.0,0.25',
                     help='Look-at point in base frame (m), x,y,z')
    ap.add_argument('--distance', type=float, default=0.7, help='Camera distance from target (m)')
    ap.add_argument('--azimuth', type=float, default=0.0, help='Camera azimuth around target (deg)')
    ap.add_argument('--elevation', type=float, default=20.0, help='Camera elevation above target (deg)')
    ap.add_argument('--noise-px', type=float, default=5.0,
                     help='Detector noise floor (px) used to size the ambiguity band '
                          '(dashboard: well-detected points sit at 2-6px median)')
    ap.add_argument('--n-steps', type=int, default=361)
    ap.add_argument('--out', type=str, default=os.path.join(_HERE, 'j5_observability_test.png'))
    args = ap.parse_args()

    q_base_deg = np.array([float(x) for x in args.pose.split(',')], dtype=np.float64)
    assert q_base_deg.shape == (6,), '--pose needs exactly 6 comma-separated values'
    q_base = np.radians(q_base_deg)
    j5_true_deg = q_base_deg[4]
    target = tuple(float(x) for x in args.target.split(','))

    K, dist = load_intrinsics()
    rvec, tvec = build_camera_pose(target, args.distance, args.azimuth, args.elevation)

    j5_deg, err_px, xy, p_true = sweep(q_base, rvec, tvec, K, dist, args.n_steps)
    band_width, local_slope = summarize(j5_deg, err_px, j5_true_deg, args.noise_px)

    print(f'Pose caméra (synthétique)   : target={target} distance={args.distance}m '
          f'azimuth={args.azimuth}° elevation={args.elevation}°')
    print(f'Pose de base (°)            : {q_base_deg.tolist()}')
    print(f'J5 vrai                     : {j5_true_deg:.1f}° -> link6 projeté '
          f'à ({p_true[0]:.1f}, {p_true[1]:.1f}) px')
    print(f'Bande d\'ambiguïté (<= {args.noise_px:.0f}px du minimum) : {band_width:.1f}°')
    print(f'Pente locale autour de J5 vrai : {local_slope:.3f} px/°')
    if band_width > 10.0:
        print('=> J5 est FAIBLEMENT observable à cette pose/vue : plusieurs valeurs de J5 '
              'donnent une reprojection de link6 indiscernable du bruit détecteur.')
    else:
        print('=> J5 semble correctement contraint par link6 à cette pose/vue.')

    fig, axes = plt.subplots(1, 2, figsize=(12, 5))

    ax = axes[0]
    ax.plot(j5_deg, err_px, color='#1f77b4', lw=1.8)
    ax.axvline(j5_true_deg, color='#d62728', ls='--', lw=1, label=f'J5 vrai = {j5_true_deg:.1f}°')
    ax.axhline(args.noise_px, color='#888', ls=':', lw=1, label=f'seuil {args.noise_px:.0f}px')
    ax.set_xlabel('Hypothèse J5 (°)')
    ax.set_ylabel('Erreur de reprojection link6 (px)')
    ax.set_title('Sensibilité de link6 à J5\n(autres joints + pose caméra fixes)')
    ax.legend()
    ax.grid(alpha=0.3)

    ax = axes[1]
    sc = ax.scatter(xy[:, 0], xy[:, 1], c=j5_deg, cmap='twilight', s=8)
    ax.scatter([p_true[0]], [p_true[1]], color='red', marker='x', s=80, label='J5 vrai')
    ax.set_xlabel('u (px)')
    ax.set_ylabel('v (px)')
    ax.set_title('Trajectoire projetée de link6 quand J5 varie')
    ax.invert_yaxis()
    ax.legend()
    fig.colorbar(sc, ax=ax, label='J5 (°)')

    fig.tight_layout()
    fig.savefig(args.out, dpi=140)
    print(f'Figure : {args.out}')


if __name__ == '__main__':
    main()
