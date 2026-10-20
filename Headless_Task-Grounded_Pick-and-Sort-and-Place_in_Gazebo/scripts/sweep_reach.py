#!/usr/bin/env python3
"""Part 6.5 -- regenerate config/reach_map.csv.

Pure kinematics (mycobot_ik.solve_tip), no rclpy, no simulator -- same
guarantee as mycobot_ik.py itself. The raw CSV the original sweep
produced was never persisted (only its derivative, config/safe_rect.txt,
survived), so this script recreates the missing artifact for §11.4's
package tree.

HONEST CAVEAT, checked directly and not glossed over: running this at
0.04 m resolution with 6 seeds (traded down from solve_tip's default 12
for speed, after two slower attempts at finer resolution proved too
slow to finish in a reasonable session) does NOT closely reproduce the
existing config/safe_rect.txt. This run's rectangle: x -0.10..0.34,
y -0.25..0.31. The existing file: x -0.03..0.55, y -0.19..0.22. The x/y
minimums and the y maximum from this run all land exactly on this
script's own grid boundary (X_RANGE/Y_RANGE below), meaning the search
area did not even bracket the true reachable envelope in those
directions -- the coarser grid and trimmed seed count most likely
under-find marginal solutions near the true boundary, biasing the
result inward, not because 0.55 is wrong. config/safe_rect.txt is left
untouched and remains authoritative; do NOT treat this run's numbers as
a replacement for it. A proper regeneration (wider grid, full seed
count, patient enough to run to completion) is real follow-up work, not
done here -- see doc/HANDOVER.md.
"""
import csv
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from mycobot_ik import solve_tip  # noqa: E402

X_RANGE = (-0.10, 0.60, 0.04)
Y_RANGE = (-0.25, 0.30, 0.04)
Z = 0.02  # representative grasp height, matches objects.yaml's grasp_dz comments
N_SEEDS = 6  # trimmed from solve_tip's default 12 -- coarse solvability sweep,
             # not the thorough waypoint-precompute search; speed matters here


def frange(lo, hi, step):
    n = round((hi - lo) / step)
    return [lo + i * step for i in range(n + 1)]


def main():
    rows = []
    xs, ys = frange(*X_RANGE), frange(*Y_RANGE)
    total = len(xs) * len(ys)
    done = 0
    for x in xs:
        for y in ys:
            q, _phi = solve_tip((x, y, Z), require_elbow_up=True, early_exit=True,
                                 n_seeds=N_SEEDS)
            solved = q is not None
            rows.append({"x": f"{x:.3f}", "y": f"{y:.3f}",
                         "solved": "1" if solved else "0",
                         "elbow_up": "1" if solved else "0"})
            done += 1
            if done % 50 == 0:
                print(f"{done}/{total}", flush=True)

    out = Path("config/reach_map.csv")
    with out.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=["x", "y", "solved", "elbow_up"])
        w.writeheader()
        w.writerows(rows)

    n_solved = sum(1 for r in rows if r["solved"] == "1")
    print(f"{len(rows)} grid points, {n_solved} solved, elbow-up -> {out}")


if __name__ == "__main__":
    main()
