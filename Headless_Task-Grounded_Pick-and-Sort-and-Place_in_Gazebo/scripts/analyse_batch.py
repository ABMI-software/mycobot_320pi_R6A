#!/usr/bin/env python3
"""Post-batch analysis: convergence margins and the reachability map.

  python3 scripts/analyse_batch.py [--home /workspace/htgspp]

Reads batch_results.csv (one row per attempt), each episode's grasp_log.csv
(per-pose maximum joint error of the attempt left on disk, i.e. the last
one), config/episode_matrix.csv and config/waypoints.json (lift-pose J2).

Prints:
  1. Per-pose maximum joint error, all episodes on disk; and the PASSING
     episodes whose worst pose converged within 1.3-1.5 deg (scraping the
     1.5 deg tolerance), grouped by object and by lift J2.
  2. The reachability map: every matrix episode by object and lift J2,
     marked PASS (first attempt), RETRY-PASS, FAIL or NOT RUN.
"""
import argparse
import csv
import json
from collections import defaultdict
from pathlib import Path

POSES = ("approach", "descend", "lift", "transport", "release_descend", "retreat", "home_return")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--home", default="/workspace/htgspp")
    a = ap.parse_args()
    H = Path(a.home)
    matrix = {int(r["episode"]): r for r in csv.DictReader(open("config/episode_matrix.csv"))}
    wps = json.load(open("config/waypoints.json"))
    lift_j2 = {e: wps[f"ep{e:03d}"][2][1] for e in matrix}

    attempts = defaultdict(list)
    for r in csv.DictReader(open(H / "batch_results.csv")):
        attempts[int(r["episode"])].append(r)

    def outcome(e):
        rows = attempts.get(e)
        if not rows:
            return "NOT RUN"
        last = rows[-1]
        if last["verdict"] == "PASS":
            return "PASS" if len(rows) == 1 else "RETRY-PASS"
        return "FAIL"

    # ---- 1. convergence margins -----------------------------------------
    worst, per_pose = {}, defaultdict(list)
    for e in matrix:
        f = H / "episodes" / f"ep_{e:03d}" / "grasp_log.csv"
        if not f.exists():
            continue
        errs = {}
        for row in csv.DictReader(open(f)):
            if row["phase"] in POSES and row.get("q_err_deg"):
                errs[row["phase"]] = float(row["q_err_deg"])
                per_pose[row["phase"]].append(float(row["q_err_deg"]))
        if errs:
            worst[e] = max(errs.items(), key=lambda kv: kv[1])

    print("== 1. Per-pose max joint error (deg), last attempt on disk of every episode ==")
    for ph in POSES:
        v = sorted(per_pose[ph])
        if v:
            print(f"  {ph:<16} n={len(v):>2}  min {v[0]:.2f}  median {v[len(v)//2]:.2f}  max {v[-1]:.2f}"
                  f"  >=1.3: {sum(x >= 1.3 for x in v):>2}")
    passing = [e for e in matrix if outcome(e) in ("PASS", "RETRY-PASS") and e in worst]
    scrape = [e for e in passing if 1.3 <= worst[e][1] <= 1.5]
    print(f"\n  PASSING episodes: {len(passing)};  worst pose within 1.3-1.5 deg: {len(scrape)}")
    by_obj, by_j2 = defaultdict(lambda: [0, 0]), defaultdict(lambda: [0, 0])
    for e in passing:
        o, b = matrix[e]["target"], f"{int(lift_j2[e] // 5 * 5)}-{int(lift_j2[e] // 5 * 5) + 5}"
        by_obj[o][0] += 1; by_j2[b][0] += 1
        if e in scrape:
            by_obj[o][1] += 1; by_j2[b][1] += 1
    print("  by object  (scraping / passing):", {k: f"{v[1]}/{v[0]}" for k, v in sorted(by_obj.items())})
    print("  by lift J2 (scraping / passing):", {k: f"{v[1]}/{v[0]}" for k, v in sorted(by_j2.items())})
    print("  scraping episodes:", [(e, matrix[e]["target"], round(lift_j2[e], 1), worst[e]) for e in scrape])

    # ---- 2. reachability map --------------------------------------------
    print("\n== 2. Reachability map: every matrix episode, sorted by object then lift J2 ==")
    print(f"  {'ep':>3} {'object':<15} {'kind':<8} {'split':<8} {'target (x, y)':<16} {'lift J2':>7}  outcome      attempts")
    for e in sorted(matrix, key=lambda e: (matrix[e]["target"], lift_j2[e], e)):
        r = matrix[e]
        print(f"  {e:>3} {r['target']:<15} {r['kind']:<8} {r['split']:<8} "
              f"({float(r['target_x']):.2f}, {float(r['target_y']):+.2f})   {lift_j2[e]:>6.1f}  "
              f"{outcome(e):<12} {len(attempts.get(e, []))}")


if __name__ == "__main__":
    main()
