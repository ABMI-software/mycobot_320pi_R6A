#!/usr/bin/env python3
"""Shared offline IK helpers for Part 6.5 (reachability sweep) and Part 7
(waypoint precompute). Pure kinematics -- numpy only, no rclpy, no
simulator. Deliberately mirrors the seeding/branch-selection approach
already proven in mycobot_gateway/mycobot_gateway/sim_sorting_grasp.py
rather than inventing a new one, but adds an EXPLICIT elbow-up filter
(J3 < 0) as Part 7.1 requires -- the reference node biases toward elbow-up
through its seed choices but does not hard-reject elbow-down solutions the
way this specification asks for.

TOOL_OFFSET is the same measured constant sim_sorting_grasp.py uses
(fingertip offset from link6, in the link6 frame) -- not re-derived from
Part 7.2's gripper_base-frame tf2_echo procedure. That procedure needs a
live Gazebo run; this constant is already proven correct (cross-checked
independently by track_grasp.py during Part 3's grasp test). Re-measuring
a second, differently-framed version of the same physical offset would add
a live-run dependency for no accuracy gain. See doc/GRIPPER_FRAME_NOTE.md.
"""
import math
import sys
from pathlib import Path

import numpy as np

_VENDOR = Path(__file__).resolve().parents[1] / 'vendor'
sys.path.insert(0, str(_VENDOR / 'training_dream'))
sys.path.insert(0, str(_VENDOR / 'scripts'))
from diff_ik import fk_pose, solve_pose  # noqa: E402

TOOL_OFFSET = np.array([-0.001, 0.0078, 0.166])
IK_ITERATIONS = 60

JOINT_LIMITS_DEG = np.array([(-168., 168.), (-135., 135.), (-150., 150.),
                             (-145., 145.), (-165., 165.), (-180., 180.)])


def _wrap180(a):
    return (a + 180.0) % 360.0 - 180.0


def rotation_top_down(phi_rad):
    z = np.array([0.0, 0.0, -1.0])
    x = np.array([math.cos(phi_rad), math.sin(phi_rad), 0.0])
    return np.column_stack([x, np.cross(z, x), z])


def tool_tip(q_deg):
    p_mm, rot = fk_pose(q_deg)
    return p_mm / 1000.0 + rot @ TOOL_OFFSET


def limit_margin(q_deg):
    return float(np.min(np.minimum(q_deg - JOINT_LIMITS_DEG[:, 0],
                                   JOINT_LIMITS_DEG[:, 1] - q_deg)))


def _seeds(tip, q_ref):
    az = math.degrees(math.atan2(tip[1], tip[0]))
    seeds = [
        [_wrap180(az + 22), -27., -58., -4., 90., 0.],
        [_wrap180(az + 22), 0., -92., 3., 90., 0.],
        [_wrap180(az + 22), -16., -94., 20., 90., 97.],
        [_wrap180(az - 22), -27., -58., -4., -90., 0.],
        [_wrap180(az - 200), 3., 90., -3., -90., 0.],
        [_wrap180(az - 160), 3., 90., -3., -90., 0.],
        # extra elbow-up-biased seeds not in the reference node's list --
        # widens coverage for the sweep, which (unlike the reference) has
        # no live q_ref to seed from at every grid point.
        [_wrap180(az), -40., -70., 10., 90., 0.],
        [_wrap180(az), -10., -100., 30., 90., 0.],
        [_wrap180(az + 45), -30., -60., 0., 90., 0.],
        [_wrap180(az - 45), -30., -60., 0., -90., 0.],
    ]
    if q_ref is not None:
        seeds.insert(0, list(q_ref))
    return [np.array(s, dtype=float) for s in seeds]


def solve_tip(tip, phi_deg=None, q_ref=None, require_elbow_up=True,
              n_seeds=12, tol_mm=2.0, tol_deg=3.0, margin_min=3.0,
              early_exit=False):
    """Joint solution reaching `tip` (m, world frame), tool pointing down.
    Returns (q_deg, phi_used) or (None, None). If require_elbow_up, any
    solution with J3 >= 0 is rejected outright (Part 7.1).

    early_exit=True returns the FIRST passing solution instead of searching
    every phi/seed combination for the best one -- for a reachability MAP
    (Part 6.5) only solvability matters, not trajectory quality, and the
    full search is ~2 orders of magnitude too slow for a real sweep grid.
    Waypoint precompute (Part 7.4) needs the thorough search (continuity
    matters there) and does not set this.
    """
    phis = [phi_deg] if phi_deg is not None else list(range(0, 180, 15))
    best = None
    for phi in phis:
        rot = rotation_top_down(math.radians(phi))
        flange_mm = (np.asarray(tip) - rot @ TOOL_OFFSET) * 1000.0
        for seed in _seeds(tip, q_ref)[:n_seeds]:
            q = solve_pose(seed, flange_mm, rot, iterations=IK_ITERATIONS)
            if require_elbow_up and q[2] >= 0:
                continue
            got_mm, got_rot = fk_pose(q)
            if float(np.linalg.norm(got_mm - flange_mm)) > tol_mm:
                continue
            ang = math.degrees(np.arccos(
                np.clip((np.trace(got_rot @ rot.T) - 1) / 2, -1, 1)))
            if ang > tol_deg:
                continue
            margin = limit_margin(q)
            if margin < margin_min:
                continue
            if early_exit:
                return q, phi
            travel = 0.0 if q_ref is None else float(np.max(np.abs(q - q_ref)))
            # When a reference pose exists, prioritise CONTINUITY: margin is
            # already floor-filtered by margin_min above, so a further
            # margin bonus here just lets a far, high-margin solution
            # outscore a near, adequate-margin one -- exactly the bug that
            # produced ~40-80 deg joint jumps between waypoints meant to
            # share a wrist orientation (see solve_column's docstring).
            # Margin only matters as a tiebreaker among similarly-near
            # solutions when there is no reference to stay near.
            score = travel if q_ref is not None else -min(margin, 30.0)
            if best is None or score < best[0]:
                best = (score, q, phi)
    return (None, None) if best is None else (best[1], best[2])


def solve_column(xyz_list, phi_deg=None, q_ref=None, require_elbow_up=True,
                  n_seeds=12):
    """Several waypoints at the SAME wrist orientation (mirrors
    sim_sorting_grasp.py's solve_column exactly, and for the same reason:
    re-searching phi independently at each height lets the solver pick a
    DIFFERENT wrist angle per waypoint, which showed up as a ~40-80 deg
    'branch jump' between every consecutive waypoint pair in this
    project's first precompute attempt -- caught by check_waypoints.py,
    not by inspection. Locking one phi across the whole column is the fix,
    not a lower branch-jump threshold. Tried phis: the caller's own
    phi_deg if fixed, else every 15 deg step."""
    phis = [phi_deg] if phi_deg is not None else list(range(0, 180, 15))
    best = None
    for phi in phis:
        column, prev, ok = [], q_ref, True
        for xyz in xyz_list:
            q, _ = solve_tip(xyz, phi_deg=phi, q_ref=prev,
                              require_elbow_up=require_elbow_up, n_seeds=n_seeds)
            if q is None:
                ok = False
                break
            column.append(q)
            prev = q
        if not ok:
            continue
        travel = max(float(np.max(np.abs(column[0] - q_ref))) if q_ref is not None else 0.0,
                     *([float(np.max(np.abs(b - a))) for a, b in zip(column, column[1:])]
                       or [0.0]))
        if best is None or travel < best[0]:
            best = (travel, phi, column)
    return (None, None) if best is None else (best[2], best[1])
