#!/usr/bin/env python3
"""precompute_ik.py -- solve every Cartesian waypoint the sequence needs,
in a moveit_py-ONLY process (no rclpy import at all), and save the joint
solutions to JSON.

Why precompute rather than solve inline in the sequencing script:
constructing moveit_py's MoveItPy in the same process as an explicit
rclpy.init()+Node was found to make MoveItPy's constructor hang
indefinitely partway through its own internal setup -- reproducible
across multiple attempts and a full container restart (ruling out
leftover DDS state from earlier kills). Since every waypoint this task
needs is known ahead of time (block hover/grasp at a given x,y, plate
hover/place), solving them all up front in an isolated process and
handing the sequencing script a flat JSON of joint angles sidesteps the
conflict entirely rather than debugging moveit_py/rclpy internals
further -- this is itself worth flagging as a moveit_py rough edge,
alongside the earlier-documented "no lightweight path for pure FK"
finding from the RLDS conversion work.
"""
import json
import re
import subprocess
import sys

sys.path.insert(0, "/workspace/htgpp")
from ik_helper import build_moveit, ik_for_point  # noqa: E402


def live_block_xyz(name="red_block"):
    """Settled block position from Gazebo. The spawn pose is NOT where the
    block ends up: it drops from z=0.05 and slid ~8 cm in testing, so
    planning against nominal spawn coordinates grasps empty air. `gz model`
    prints then hangs, hence the timeout + partial-stdout handling."""
    try:
        out = subprocess.run(["gz", "model", "-m", name, "-p"], capture_output=True,
                             text=True, timeout=10).stdout
    except subprocess.TimeoutExpired as e:
        out = (e.stdout or b"").decode() if isinstance(e.stdout, bytes) else (e.stdout or "")
    m = re.search(r"\[\s*(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)\s*\]", out)
    if not m:
        raise RuntimeError(f"could not read live pose of {name}: {out!r}")
    return tuple(float(v) for v in m.groups())


def solve_waypoints(model, args, bx, by, bz, start_seed=None):
    px, py, ptz = args.plate_x, args.plate_y, args.plate_top_z
    bx += args.tip_dx
    px += args.tip_dx
    grasp_z = bz + args.tip_offset
    place_z = ptz + 0.02 + 0.005 + args.tip_offset
    HOME_Q = [0.0] * 6
    points = {
        "home": HOME_Q,
        "approach": (bx, by, grasp_z + args.hover),
        "descend":  (bx, by, grasp_z),
        "lift":     (bx, by, grasp_z + args.hover),
        "transport": (px, py, place_z + args.hover),
        "place":    (px, py, place_z),
        "retreat":  (px, py, place_z + args.hover),
    }
    solutions, ok, seed = {}, True, (start_seed or HOME_Q)
    for name, val in points.items():
        if name == "home":
            solutions[name] = val
            continue
        q = ik_for_point(model, *val, seed_q=seed)
        if q is None:
            print(f"  {name:10s} FAILED for {val}")
            ok = False
            continue
        solutions[name] = q
        seed = q
    return solutions, ok


def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--block-x", type=float, default=None)
    parser.add_argument("--block-y", type=float, default=None)
    parser.add_argument("--block-z", type=float, default=0.02)
    parser.add_argument("--plate-x", type=float, default=0.20)
    parser.add_argument("--plate-y", type=float, default=-0.15)
    parser.add_argument("--plate-top-z", type=float, default=0.048)
    parser.add_argument("--tip-offset", type=float, default=0.105,
                        help="gripper_base height above the block centre when the fingers straddle it")
    parser.add_argument("--tip-dx", type=float, default=-0.036,
                        help="x shift of gripper_base so the finger pair closes on the block centre")
    parser.add_argument("--hover", type=float, default=0.08)
    parser.add_argument("--out", default="/workspace/htgpp/waypoints.json")
    parser.add_argument("--poses", default=None,
                        help="batch mode: 'x,y x,y ...' block positions; writes <out-dir>/waypoints_x<x>_y<y>.json each")
    parser.add_argument("--out-dir", default="/workspace/htgpp/waypoints")
    args = parser.parse_args()

    print("Building robot model...")
    moveit_instance = build_moveit()
    model = moveit_instance.get_robot_model()

    if args.poses:
        import os
        os.makedirs(args.out_dir, exist_ok=True)
        all_ok = True
        solved = []  # (bx, by, approach_q) of branch-consistent solutions
        for token in args.poses.split():
            bx, by = (float(v) for v in token.split(","))
            KNOWN_GOOD = [0.367, 0.046, -2.422, 1.146, 1.56, -1.206]
            seed = KNOWN_GOOD
            if solved:
                seed = min(solved, key=lambda t: (t[0] - bx) ** 2 + (t[1] - by) ** 2)[2]
            for attempt in range(6):
                sol, ok = solve_waypoints(model, args, bx, by, args.block_z,
                                          start_seed=seed if attempt % 2 == 0 else KNOWN_GOOD)
                j1 = sol.get("approach", [9])[0]
                j3 = sol.get("descend", [0, 0, 9])[2]
                branch_ok = abs(j1) < 1.2 and j3 < 0
                if ok and branch_ok:
                    break
                print(f"  attempt {attempt}: J1={j1:+.3f} -> retry")
            print(f"block ({bx:.3f},{by:.3f}) ok={ok} approach J1={j1:+.3f} descend J3={j3:+.3f} branch_ok={branch_ok}")
            if ok and branch_ok:
                solved.append((bx, by, sol["approach"]))
            all_ok &= ok and branch_ok
            with open(f"{args.out_dir}/waypoints_x{bx:.3f}_y{by:.3f}.json", "w") as f:
                json.dump(sol, f, indent=2)
        sys.exit(0 if all_ok else 1)

    live = live_block_xyz() if None in (args.block_x, args.block_y) else (None,) * 3
    bx = args.block_x if args.block_x is not None else live[0]
    by = args.block_y if args.block_y is not None else live[1]
    print(f"block target: ({bx:.4f}, {by:.4f}, {args.block_z:.4f})")
    sol, ok = solve_waypoints(model, args, bx, by, args.block_z)
    for k, v in sol.items():
        print(f"{k:10s} {[round(x, 3) for x in v]}")
    with open(args.out, "w") as f:
        json.dump(sol, f, indent=2)
    print(f"{'All waypoints solved' if ok else 'SOME WAYPOINTS FAILED'} -- wrote {args.out}")
    sys.exit(0 if ok else 1)


if __name__ == "__main__":
    main()
