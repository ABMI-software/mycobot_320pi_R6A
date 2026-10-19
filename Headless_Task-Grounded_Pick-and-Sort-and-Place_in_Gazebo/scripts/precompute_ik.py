#!/usr/bin/env python3
"""Part 6.5 (--sweep) and Part 7.4 (--matrix) -- offline, no simulator."""
import argparse
import csv
import json
import sys

import numpy as np
import yaml

from mycobot_ik import solve_column, solve_tip, tool_tip


def cmd_sweep(args):
    cfg = yaml.safe_load(open(args.objects)) if args.objects else None
    if cfg:
        zs = sorted({round(cfg["table_top_z"] + (
            (o["size"][2] / 2 if o["shape"] == "box" else o["length"] / 2)
            + o["grasp_dz"]), 4) for o in cfg["objects"].values()})
        zs.append(0.110)   # common transit height, always checked too
    else:
        zs = [float(z) for z in args.z_list.split(",")]

    xs = np.arange(args.x_range[0], args.x_range[1] + 1e-9, args.x_step)
    ys = np.arange(args.y_range[0], args.y_range[1] + 1e-9, args.y_step)
    with open(args.out, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["x", "y", "z", "solved", "elbow_up", "margin_deg"])
        n = 0
        for z in zs:
            for x in xs:
                for y in ys:
                    q, _phi = solve_tip([x, y, z], require_elbow_up=args.require_elbow_up,
                                         early_exit=True)
                    if q is None:
                        w.writerow([round(x, 4), round(y, 4), z, 0, 0, ""])
                    else:
                        from mycobot_ik import limit_margin
                        w.writerow([round(x, 4), round(y, 4), z, 1, 1,
                                    round(limit_margin(q), 2)])
                    n += 1
    print(f"{n} grid points written to {args.out} (z levels: {zs})")


def cmd_matrix(args):
    cfg = yaml.safe_load(open(args.objects))
    gf = yaml.safe_load(open(args.gripper_frame)) if args.gripper_frame else None
    rows = list(csv.DictReader(open(args.matrix)))
    APPROACH_Z = 0.110
    TRANSIT_Z = 0.110

    out = {}
    for row in rows:
        ep = f'ep{int(row["episode"]):03d}'
        target = row["target"]
        o = cfg["objects"][target]
        half = (o["size"][2] / 2) if o["shape"] == "box" else (o["length"] / 2)
        grasp_z = cfg["table_top_z"] + half + o["grasp_dz"]
        bx, by = cfg["bins"][o["bin"]]["pose"][:2]
        rim_z = cfg["bins"][o["bin"]]["rim_z"]
        release_z = rim_z + cfg["release"]["height_above_bin_rim"]
        ox, oy = float(row[f"{target}_x"]), float(row[f"{target}_y"])
        phi = (o["grasp_yaw"] * 180.0 / np.pi) if o.get("grasp_yaw_fixed") else None

        # Two GROUPS, each solved with ONE shared wrist orientation (mirrors
        # sim_sorting_grasp.py's solve_column) -- solving every waypoint
        # independently let the solver pick a different phi per waypoint,
        # a ~40-80 deg "branch jump" between every pair, caught by
        # check_waypoints.py on the first attempt. See mycobot_ik.solve_column.
        pick_group = [(ox, oy, APPROACH_Z), (ox, oy, grasp_z), (ox, oy, APPROACH_Z)]
        bin_group = [(bx, by, TRANSIT_Z), (bx, by, release_z), (bx, by, APPROACH_Z)]

        pick_seq, pick_phi = solve_column(pick_group, phi_deg=phi, q_ref=None,
                                           n_seeds=args.seeds)
        ok = pick_seq is not None
        # Release yaw constraint (2026-10-04): the closing jaws square a cube to
        # the gripper, so a box lands at the gripper's WORLD yaw at release,
        # mod 90 -- measured 15.2 / -45.0 / -30.0 deg against 105 / 135 / 150 deg
        # releases, exact on all three. With a free release phi a 50 mm cube
        # landed up to 45 deg yawed, 70.7 mm across an 80 mm opening. Boxes
        # therefore release axis-aligned with the bins; the cylinder's yaw is
        # irrelevant and stays free. The solver takes the candidate with the
        # least wrist travel among the landings that are EQUIVALENT for the
        # shape: a square footprint (the cubes) lands the same at 0/90/180/270
        # (offering only 0/90 spun the wrist up to 189 deg in transport); a
        # non-square one only at its grasp phi and phi + 180, since phi + 90
        # swaps its long axis from x to y. The yellow box (50 x 30 mm) has a
        # fixed grasp phi of 90, so it releases at 90 or 270, landing exactly
        # as it was spawned. A non-square box with a free grasp phi has no
        # known landing orientation and is refused.
        if o["shape"] != "box":
            place_phis = [None]
        elif o["size"][0] == o["size"][1]:
            place_phis = [0.0, 90.0, 180.0, 270.0]
        elif o.get("grasp_yaw_fixed"):
            place_phis = [phi, phi + 180.0]
        else:
            raise SystemExit(f"{target}: non-square box with a free grasp yaw -- its landing "
                             f"orientation is not determined; fix grasp_yaw in objects.yaml")
        bin_seq = None
        if ok:
            best = None
            for place_phi in place_phis:
                seq_try, _ = solve_column(bin_group, phi_deg=place_phi, q_ref=pick_seq[-1],
                                          n_seeds=args.seeds)
                if seq_try is None:
                    continue
                travel = max(float(np.max(np.abs(b - a)))
                             for a, b in zip([pick_seq[-1]] + seq_try, seq_try))
                if best is None or travel < best[0]:
                    best = (travel, seq_try)
            bin_seq = best[1] if best else None
            ok = bin_seq is not None
        seq = ([[round(float(v), 4) for v in q] for q in (pick_seq + bin_seq)]
               if ok else None)
        out[ep] = seq
        print(f'{ep} ({target}): {"OK" if ok else "UNSOLVED"}')

    json.dump(out, open(args.out, "w"), indent=1)
    print(f"wrote {len(out)} episode entries to {args.out}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--sweep", action="store_true")
    ap.add_argument("--matrix")
    ap.add_argument("--objects")
    ap.add_argument("--gripper-frame")
    ap.add_argument("--x-range", nargs=2, type=float, default=[0.18, 0.36])
    ap.add_argument("--x-step", type=float, default=0.005)
    ap.add_argument("--y-range", nargs=2, type=float, default=[-0.14, 0.14])
    ap.add_argument("--y-step", type=float, default=0.01)
    ap.add_argument("--z-list", default="0.11")
    ap.add_argument("--z-from-config", action="store_true")
    ap.add_argument("--require-elbow-up", action="store_true", default=True)
    ap.add_argument("--branch-max-step", type=float, default=0.6)
    ap.add_argument("--seeds", type=int, default=12)
    ap.add_argument("--out", required=True)
    args = ap.parse_args()
    if args.sweep and not args.z_from_config:
        args.objects = None
    if args.sweep:
        cmd_sweep(args)
    elif args.matrix:
        cmd_matrix(args)
    else:
        sys.exit("pass --sweep or --matrix")


if __name__ == "__main__":
    main()
