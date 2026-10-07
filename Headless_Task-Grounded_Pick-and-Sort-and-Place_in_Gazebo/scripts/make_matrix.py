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
import random
import sys

import yaml
from layout import sample_distractors

random.seed(20260923)          # reproducibility is not optional
rng = random.Random(20260923)

cfg = yaml.safe_load(open("config/objects.yaml"))
OBJ = ["red_cube", "blue_cube", "green_cylinder", "yellow_box"]
BASE_XY = {  # each object's own real_table.sdf spawn pose (x, y)
    "red_cube": (0.22, -0.08),
    "blue_cube": (0.10, -0.06),
    "green_cylinder": (0.10, 0.02),
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
    for kind, x, y in plan:
        layout = sample_distractors(
            obj, (x, y), bins, SAFE,
            cfg["spawn"]["min_object_separation"],
            cfg["spawn"]["bin_keepout_radius"], others, rng)
        row = {"episode": idx, "target": obj, "kind": kind,
               "target_x": x, "target_y": y,
               "camera": "heldout" if kind == "heldout" else "table",
               "split": "heldout" if kind == "heldout" else "train",
               "task_index": OBJ.index(obj)}
        for name, (px, py) in layout.items():
            row[f"{name}_x"], row[f"{name}_y"] = px, py
        rows.append(row)
        idx += 1

with open("config/episode_matrix.csv", "w", newline="") as f:
    w = csv.DictWriter(f, fieldnames=list(rows[0]))
    w.writeheader()
    w.writerows(rows)

print(f"{len(rows)} episodes  "
      f"train={sum(r['split']=='train' for r in rows)}  "
      f"heldout={sum(r['split']=='heldout' for r in rows)}")
for o in OBJ:
    print(f"  {o:16s} {sum(r['target']==o for r in rows)}")
