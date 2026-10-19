#!/usr/bin/env python3
"""Place reachability and lift-to-place path clearance, offline (no simulator).

  python3 scripts/check_place_path.py [--episodes N ...]

For each episode: the place waypoints (over-bin, release) must exist in
config/waypoints.json (precompute_ik.py solved them), and along the
joint-space path lift -> over-bin -> release, sampled with the project's own
forward kinematics, the CARRIED object's bottom must clear everything it
passes over: the other three objects (at this episode's positions, their
heights) and every bin's 3 cm walls -- the destination bin excepted only for
the final descent into it. Reports the minimum vertical clearance per
episode and exits 1 if any is below 10 mm.
"""
import argparse
import csv
import json
import math
import sys

import numpy as np
import yaml

sys.path.insert(0, "scripts")
from mycobot_ik import tool_tip  # noqa: E402

MIN_CLEAR = 0.010


def footprint_r(o):
    return math.hypot(o["size"][0], o["size"][1]) / 2 if o["shape"] == "box" else o["radius"]


def height(o):
    return o["size"][2] if o["shape"] == "box" else o["length"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, nargs="*")
    a = ap.parse_args()
    cfg = yaml.safe_load(open("config/objects.yaml"))
    wps = json.load(open("config/waypoints.json"))
    rows = {int(r["episode"]): r for r in csv.DictReader(open("config/episode_matrix.csv"))}
    bad = 0
    for ep in a.episodes or sorted(rows):
        r = rows[ep]; t = r["target"]; o = cfg["objects"][t]
        seq = wps.get(f"ep{ep:03d}")
        if not seq or len(seq) < 5:
            print(f"ep {ep:>2} {t:<15} PLACE UNREACHABLE (no place waypoints)"); bad += 1; continue
        grasp_z = cfg["table_top_z"] + height(o) / 2 + o["grasp_dz"]
        r_obj = footprint_r(o)
        # objects: circles (half-diagonal / radius), conservative
        obstacles = [("circle", n, float(r[n + "_x"]), float(r[n + "_y"]), footprint_r(cfg["objects"][n]),
                      height(cfg["objects"][n])) for n in cfg["objects"] if n != t]
        # bins: their real axis-aligned outer SQUARE, 86 mm since 2026-10-04 (inner_radius
        # 40 mm + 3 mm wall). A circumscribed circle overstates a square's reach by 41%
        # along the diagonal and flagged every blue release as passing over the red bin.
        obstacles += [("square", bn, b["pose"][0], b["pose"][1], b["inner_radius"] + 0.003, b["rim_z"])
                      for bn, b in cfg["bins"].items()]
        worst = (9.0, None)
        legs = [("lift->over_bin", seq[2], seq[3], False), ("over_bin->release", seq[3], seq[4], True)]
        for leg, qa, qb, final in legs:
            for s in np.linspace(0, 1, 41):
                q = [qa[i] + (qb[i] - qa[i]) * s for i in range(6)]
                tip = tool_tip(q)
                bottom = tip[2] - grasp_z
                for kind, n, x, y, rad, h in obstacles:
                    if final and n == o["bin"]:
                        continue
                    over = (math.hypot(tip[0] - x, tip[1] - y) < rad + r_obj if kind == "circle"
                            else abs(tip[0] - x) < rad + r_obj and abs(tip[1] - y) < rad + r_obj)
                    if over:
                        c = bottom - h
                        if c < worst[0]:
                            worst = (c, f"{leg} over {n}")
        ok = worst[0] >= MIN_CLEAR
        bad += not ok
        txt = "nothing passed over" if worst[1] is None else f"{worst[0]*1000:+.1f} mm ({worst[1]})"
        print(f"ep {ep:>2} {t:<15} {r['kind']:<8} start ({float(r['target_x']):.3f},{float(r['target_y']):+.3f}) "
              f"place reachable; min clearance {txt}  {'OK' if ok else 'TOO CLOSE'}")
    sys.exit(1 if bad else 0)


if __name__ == "__main__":
    main()
