#!/usr/bin/env python3
"""Part 6.7 -- config/episode_matrix.csv, adapted for 4 distinct objects.

The specification's own template (Part 6.7) illustrates a SINGLE shared
X_NOM/Y_NOM/X_VAR/Y_VAR for all objects, inherited from the previous
acquisition's single-object task. This world has four objects at four
different base positions (config/objects.yaml, read from the merged SDF --
see doc/OBJECTS_MANIFEST.md), so each object gets its OWN nominal position
(its real_table.sdf spawn pose) and its own +-20mm x/y variation band
around that position, rather than one shared point. Distractor placement
(the other three objects, every episode) reuses Part 6.6's layout.py
unchanged -- that part of the scheme is generic and does not depend on how
many distinct base positions exist.
"""
import csv
import math
import random
import sys

import yaml
from layout import STATS, sample_distractors

random.seed(20260923)          # reproducibility is not optional
rng = random.Random(20260923)

cfg = yaml.safe_load(open("config/objects.yaml"))
OBJ = ["red_cube", "blue_cube", "green_cylinder", "yellow_box"]
BASE_XY = {  # each object's nominal start position (x, y)
    # Originally each object's real_table.sdf spawn pose. Since 2026-10-04 every
    # variant must put the LIFT pose at shoulder J2 <= 48.0 deg (validated
    # region, MEASUREMENTS.md "Reachability"): in more folded lift poses the
    # welded object is obstructed and the elbow cannot reach its goal (a
    # simulation collision defect, not gravity sag). blue_cube moved
    # (0.10 -> 0.120) and green_cylinder (0.10 -> 0.135), each to the nearest
    # x where all nine variants solve at <= 46 deg (worst: 45.8, 45.7).
    "red_cube": (0.22, -0.08),
    "blue_cube": (0.120, -0.06),
    "green_cylinder": (0.135, 0.02),
    "yellow_box": (0.10, 0.09),
}
bins = {k: (v["pose"][0], v["pose"][1]) for k, v in cfg["bins"].items()}

# Safe rectangle from Part 6.5 (scripts/summarise_reach.py's output),
# read here rather than hardcoded so a re-run of the sweep automatically
# propagates. Falls back loudly if the sweep hasn't been run yet.
try:
    SAFE = tuple(float(x) for x in open("config/safe_rect.txt").read().split())
except FileNotFoundError:
    sys.exit("config/safe_rect.txt missing — run scripts/summarise_reach.py "
             "(after the Part 6.5 sweep) and write its SAFE line's four "
             "numbers to that file first.")

VAR_M = 0.020   # +-20mm variation band per object, around its own base pose

# The table board (worlds/sorting_table.sdf: 622 x 449 mm centred at
# (0.261, 0.017)). Every object's axis-aligned FOOTPRINT must lie on it --
# the safe rectangle only bounds centres.
BOARD = (0.261 - 0.311, 0.261 + 0.311, 0.017 - 0.2245, 0.017 + 0.2245)


# Robot keep-out: no object centre within 120 mm of the robot's vertical
# axis. The robot base has no collision geometry at table height, so an
# object placed on it interpenetrates it silently: episode 8's green cylinder
# at 14 mm vanished inside the base in the camera image; distractors at 14-72 mm
# were hidden or under the arm. 120 mm is the closest radius verified clear
# and fully visible in the framing check (targets at 117-120 mm). It applies
# to distractors: targets are fixed by the plan and all sit at >= 117 mm.
ROBOT_KEEPOUT = 0.120


# Bin keep-out on the real geometry: every object's footprint (spawned at yaw 0,
# so axis-aligned) at least 10 mm outside every bin's outer square. layout.py's
# bin_keepout_radius is a circle around the bin centre and misses the corners:
# with 86 mm bins it let episode 32's blue cube sit 14 mm inside the blue bin
# (2026-10-04). Applies to targets too: they are fixed by the plan, so a
# conflicting one stops the generator instead of being moved silently.
# BIN_CLEAR = 5 mm is a CHOSEN value, not a derived safety margin: it is fitted
# under red's existing y+20 target, which sits 7.0 mm from the blue bin's wall,
# with one precedent -- at 7.5 mm from that wall (100 mm bins) its episode 12
# grasped and passed on 2026-10-03. Nothing here shows 5 mm is safe in general.
BIN_HALF_OUTER = {k: v["inner_radius"] + 0.003 for k, v in cfg["bins"].items()}
BIN_CLEAR = 0.005


def half_footprint(name):
    o = cfg["objects"][name]
    return ((o["size"][0] / 2, o["size"][1] / 2) if o["shape"] == "box"
            else (o["radius"], o["radius"]))


def fits_board(name, xy):
    hx, hy = half_footprint(name)
    return (BOARD[0] + hx <= xy[0] <= BOARD[1] - hx
            and BOARD[2] + hy <= xy[1] <= BOARD[3] - hy)


def clears_bins(name, xy):
    hx, hy = half_footprint(name)
    return all(abs(xy[0] - bx) >= BIN_HALF_OUTER[bn] + hx + BIN_CLEAR
               or abs(xy[1] - by) >= BIN_HALF_OUTER[bn] + hy + BIN_CLEAR
               for bn, (bx, by) in bins.items())


def fits(name, xy):
    return (fits_board(name, xy) and clears_bins(name, xy)
            and (name == current_target or math.hypot(*xy) >= ROBOT_KEEPOUT))

rows, idx = [], 1
for obj in OBJ:
    others = [o for o in OBJ if o != obj]
    bx0, by0 = BASE_XY[obj]
    x_var = [bx0 - VAR_M, bx0 - VAR_M / 2, bx0 + VAR_M / 2, bx0 + VAR_M]
    y_var = [by0 - VAR_M, by0 - VAR_M / 2, by0 + VAR_M / 2, by0 + VAR_M]
    plan = ([("nominal", bx0, by0)] * 4
            + [("x_var", x, by0) for x in x_var]
            + [("y_var", bx0, y) for y in y_var]
            + [("heldout", bx0, by0),
               ("heldout", x_var[1], by0),
               ("heldout", bx0, y_var[2])])
    current_target = obj
    for kind, x, y in plan:
        layout = sample_distractors(
            obj, (x, y), bins, SAFE,
            cfg["spawn"]["min_object_separation"],
            cfg["spawn"]["bin_keepout_radius"], others, rng, fits=fits)
        row = {"episode": idx, "target": obj, "kind": kind,
               "target_x": x, "target_y": y,
               "camera": "heldout" if kind == "heldout" else "table",
               "split": "heldout" if kind == "heldout" else "train",
               "task_index": OBJ.index(obj)}
        for name, (px, py) in layout.items():
            row[f"{name}_x"], row[f"{name}_y"] = px, py
        rows.append(row)
        idx += 1

with open(sys.argv[1] if len(sys.argv) > 1 else "config/episode_matrix.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0]))
    w.writeheader()
    w.writerows(rows)

print(f"{len(rows)} episodes  "
      f"train={sum(r['split']=='train' for r in rows)}  "
      f"heldout={sum(r['split']=='heldout' for r in rows)}")
for o in OBJ:
    print(f"  {o:16s} {sum(r['target']==o for r in rows)}")
print(f"sampler: {STATS['layouts']} layouts from {STATS['draws']} draws -- acceptance "
      f"{STATS['layouts'] / STATS['draws']:.1%}, mean {STATS['draws'] / STATS['layouts']:.1f} draws per layout")
