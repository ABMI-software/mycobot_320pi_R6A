#!/usr/bin/env python3
"""Part 9.1-9.3 -- the per-episode checks, unit-testable against synthetic
pose dictionaries, no simulator needed."""
import math


def grasp_held(dz_samples, tol=0.005):
    """Part 9.1. If A8 applies (simulated attachment), this is true by
    construction -- it verifies the object was CARRIED, not that it was
    GRASPED. Say so in every written output; never call it 'grasp verified'
    when simulated_attachment is true (see doc/GRASP_DECISION.md)."""
    return (max(dz_samples) - min(dz_samples)) < tol


def object_rose(bz_samples, baseline_z, min_rise=0.05):
    return (max(bz_samples) - baseline_z) >= min_rise


def placed_in_correct_bin(target, cfg, obj_pose, tol=0.005):
    """Part 9.2. Inside its OWN bin footprint, and below the rim -- not
    resting on it."""
    b = cfg["bins"][cfg["objects"][target]["bin"]]
    dx, dy = obj_pose[0] - b["pose"][0], obj_pose[1] - b["pose"][1]
    inside = math.hypot(dx, dy) <= (b["inner_radius"] - tol)
    settled = obj_pose[2] <= b["rim_z"]
    return inside and settled, {"inside": inside, "below_rim": settled}


def landed_in_wrong_bin(target, cfg, obj_pose):
    """Part 9.2 -- the check the previous acquisition never needed. An
    episode where the object was grasped, carried, and released into the
    WRONG bin passes every check in 9.1 and is wrong anyway; the instruction
    attached to it is false. This is the check that catches it."""
    own = cfg["objects"][target]["bin"]
    for bn, b in cfg["bins"].items():
        if bn == own:
            continue
        if math.hypot(obj_pose[0] - b["pose"][0],
                       obj_pose[1] - b["pose"][1]) <= b["inner_radius"]:
            return bn
    return None


def distractors_undisturbed(target, start_poses, end_poses, tol=0.02):
    """Part 9.3. A warning, not a hard failure -- real demonstrations
    contain incidental contact; what must never be recorded is a
    mislabelled episode (that is landed_in_wrong_bin's job, not this one)."""
    moved = []
    for name, p0 in start_poses.items():
        if name == target:
            continue
        d = math.dist(p0[:2], end_poses[name][:2])
        if d > tol:
            moved.append([name, round(d, 4)])
    return (not moved), moved
