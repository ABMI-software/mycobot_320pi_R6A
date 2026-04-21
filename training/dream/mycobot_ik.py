#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Numerical Inverse Kinematics for MyCobot 320 Pi.

Uses scipy.optimize to solve IK via the existing FK chain in mycobot_fk.py.
Supports position-only IK (3-DOF target) and full 6-DOF pose IK.

Author: Generated for pick-and-place pipeline
"""

import numpy as np
from scipy.optimize import minimize
from typing import Optional, Tuple, List

# Import our FK module
import sys, os
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from mycobot_fk import forward_kinematics


# Joint limits (radians) matching URDF
JOINT_LIMITS = [
    (-2.96, 2.96),   # joint2_to_joint1
    (-2.79, 2.79),   # joint3_to_joint2
    (-2.79, 2.79),   # joint4_to_joint3
    (-2.79, 2.79),   # joint5_to_joint4
    (-2.96, 2.96),   # joint6_to_joint5
    (-3.05, 3.05),   # joint6output_to_joint6
]


def fk_end_effector(joint_angles: np.ndarray) -> np.ndarray:
    """
    Get end-effector (link6) position from FK.
    
    Returns:
        np.ndarray: [x, y, z] position of link6 in world frame (meters).
    """
    positions, transforms = forward_kinematics(joint_angles)
    ee_pos = positions['mycobot320_link6']
    return np.array(ee_pos)


def fk_full_transform(joint_angles: np.ndarray) -> np.ndarray:
    """
    Get end-effector 4x4 homogeneous transform from FK.
    
    Returns:
        np.ndarray: 4x4 transform matrix of link6 in world frame.
    """
    positions, transforms = forward_kinematics(joint_angles)
    # transforms is a list of 4x4 matrices, last one is link6
    return transforms[-1]


def inverse_kinematics_position(
    target_xyz: np.ndarray,
    q0: Optional[np.ndarray] = None,
    max_iter: int = 500,
    tol: float = 1e-6,
    n_restarts: int = 8,
) -> Tuple[bool, np.ndarray, float]:
    """
    Solve position-only IK: find joint angles that place link6 at target_xyz.
    
    Uses L-BFGS-B with multiple random restarts to avoid local minima.
    
    Args:
        target_xyz: Desired [x, y, z] position of end-effector (meters).
        q0: Initial guess for joint angles (radians). If None, random.
        max_iter: Maximum iterations per optimization.
        tol: Cost function tolerance.
        n_restarts: Number of random restarts to try.
    
    Returns:
        success: True if solution found within tolerance.
        q_solution: Best joint angles found (radians).
        residual: Final position error (meters).
    """
    target = np.array(target_xyz, dtype=np.float64)
    bounds = [(lo, hi) for lo, hi in JOINT_LIMITS]
    
    best_q = None
    best_cost = np.inf
    
    def cost_fn(q):
        ee = fk_end_effector(q)
        return np.sum((ee - target) ** 2)
    
    # Try with user's initial guess first
    guesses = []
    if q0 is not None:
        guesses.append(np.array(q0, dtype=np.float64))
    
    # Add random restarts
    rng = np.random.default_rng(42)
    for _ in range(n_restarts):
        q_rand = np.array([
            rng.uniform(lo * 0.7, hi * 0.7) for lo, hi in JOINT_LIMITS
        ])
        guesses.append(q_rand)
    
    # Also try home pose
    guesses.append(np.zeros(6))
    
    for q_init in guesses:
        try:
            result = minimize(
                cost_fn,
                q_init,
                method='L-BFGS-B',
                bounds=bounds,
                options={'maxiter': max_iter, 'ftol': tol},
            )
            if result.fun < best_cost:
                best_cost = result.fun
                best_q = result.x.copy()
        except Exception:
            continue
    
    if best_q is None:
        return False, np.zeros(6), np.inf
    
    residual = np.sqrt(best_cost)  # Euclidean distance error
    success = residual < 0.005  # 5mm threshold
    
    return success, best_q, residual


def inverse_kinematics_pose(
    target_xyz: np.ndarray,
    target_orientation: np.ndarray,
    q0: Optional[np.ndarray] = None,
    pos_weight: float = 1.0,
    rot_weight: float = 0.1,
    max_iter: int = 500,
    tol: float = 1e-6,
    n_restarts: int = 10,
) -> Tuple[bool, np.ndarray, float]:
    """
    Solve full 6-DOF IK: position + orientation.
    
    Args:
        target_xyz: Desired [x, y, z] position (meters).
        target_orientation: Desired 3x3 rotation matrix.
        q0: Initial guess.
        pos_weight: Weight for position error.
        rot_weight: Weight for orientation error.
        max_iter: Maximum iterations.
        tol: Tolerance.
        n_restarts: Number of random restarts.
    
    Returns:
        success, q_solution, residual
    """
    target_pos = np.array(target_xyz, dtype=np.float64)
    target_rot = np.array(target_orientation, dtype=np.float64)
    bounds = [(lo, hi) for lo, hi in JOINT_LIMITS]
    
    def cost_fn(q):
        T = fk_full_transform(q)
        pos_err = np.sum((T[:3, 3] - target_pos) ** 2)
        # Rotation error: ||R_target^T @ R_current - I||^2
        R_err = target_rot.T @ T[:3, :3]
        rot_err = np.sum((R_err - np.eye(3)) ** 2)
        return pos_weight * pos_err + rot_weight * rot_err
    
    best_q = None
    best_cost = np.inf
    
    guesses = []
    if q0 is not None:
        guesses.append(np.array(q0, dtype=np.float64))
    
    rng = np.random.default_rng(42)
    for _ in range(n_restarts):
        q_rand = np.array([
            rng.uniform(lo * 0.7, hi * 0.7) for lo, hi in JOINT_LIMITS
        ])
        guesses.append(q_rand)
    guesses.append(np.zeros(6))
    
    for q_init in guesses:
        try:
            result = minimize(
                cost_fn,
                q_init,
                method='L-BFGS-B',
                bounds=bounds,
                options={'maxiter': max_iter, 'ftol': tol},
            )
            if result.fun < best_cost:
                best_cost = result.fun
                best_q = result.x.copy()
        except Exception:
            continue
    
    if best_q is None:
        return False, np.zeros(6), np.inf
    
    # Check position error separately
    T = fk_full_transform(best_q)
    pos_residual = np.linalg.norm(T[:3, 3] - target_pos)
    success = pos_residual < 0.005  # 5mm
    
    return success, best_q, pos_residual


def plan_cartesian_waypoints(
    waypoints_xyz: List[np.ndarray],
    q_start: Optional[np.ndarray] = None,
    approach_height: float = 0.05,
) -> Tuple[bool, List[np.ndarray]]:
    """
    Plan a sequence of IK solutions for Cartesian waypoints.
    Uses warm-starting: each IK starts from previous solution.
    
    Args:
        waypoints_xyz: List of [x, y, z] target positions.
        q_start: Starting joint configuration.
        approach_height: Not used here but available for pre-grasp.
    
    Returns:
        success: True if all waypoints are reachable.
        joint_trajectory: List of joint angle arrays.
    """
    trajectory = []
    q_prev = q_start if q_start is not None else np.zeros(6)
    
    for i, wp in enumerate(waypoints_xyz):
        success, q_sol, residual = inverse_kinematics_position(
            wp, q0=q_prev, n_restarts=12
        )
        if not success:
            print(f"[IK] Failed at waypoint {i}: {wp}, residual={residual:.4f}m")
            return False, trajectory
        trajectory.append(q_sol)
        q_prev = q_sol
    
    return True, trajectory


# ── Quick self-test ──────────────────────────────────────────────
if __name__ == '__main__':
    print("=== MyCobot 320 IK Solver Test ===\n")
    
    # Test 1: Home position FK → IK round-trip
    q_home = np.zeros(6)
    ee_home = fk_end_effector(q_home)
    print(f"Home EE position: {ee_home}")
    
    success, q_sol, res = inverse_kinematics_position(ee_home)
    ee_check = fk_end_effector(q_sol)
    print(f"IK solution: {np.round(q_sol, 4)}")
    print(f"IK residual: {res*1000:.2f} mm")
    print(f"FK verify:   {ee_check}")
    print(f"Success: {success}\n")
    
    # Test 2: Target cube position (0.25, 0.10, 0.06) — slightly above table
    target = np.array([0.25, 0.10, 0.06])
    print(f"Target: {target}")
    success, q_sol, res = inverse_kinematics_position(target)
    ee_check = fk_end_effector(q_sol)
    print(f"IK solution: {np.round(np.degrees(q_sol), 1)}°")
    print(f"IK residual: {res*1000:.2f} mm")
    print(f"FK verify:   {np.round(ee_check, 4)}")
    print(f"Success: {success}\n")
    
    # Test 3: Waypoint planning for pick-and-place
    print("=== Waypoint Planning ===")
    waypoints = [
        np.array([0.25, 0.10, 0.15]),   # Above cube
        np.array([0.25, 0.10, 0.06]),   # At cube (grasp)
        np.array([0.25, 0.10, 0.15]),   # Lift
        np.array([-0.20, 0.15, 0.15]),  # Above drop zone
        np.array([-0.20, 0.15, 0.06]),  # At drop zone (release)
    ]
    success, traj = plan_cartesian_waypoints(waypoints)
    print(f"Planning success: {success}")
    for i, (wp, q) in enumerate(zip(waypoints, traj)):
        ee = fk_end_effector(q)
        err = np.linalg.norm(ee - wp) * 1000
        print(f"  WP{i}: target={wp} → err={err:.1f}mm")
