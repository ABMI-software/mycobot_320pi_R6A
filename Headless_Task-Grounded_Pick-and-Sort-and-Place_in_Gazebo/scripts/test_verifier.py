#!/usr/bin/env python3
"""Part 9.5 -- a verifier that has never rejected anything is not a
verifier. Synthetic pose dictionaries, no simulator.

Heights are REALISTIC settled heights, not placeholders: the bin floor's top
is at z = 0.002 (models/*_bin.sdf: 2 mm floor at z 0.001), so an object
resting in a bin has its centre at 0.002 + its half-extent along the vertical
for the face it rests on. The placement check's height limit depends on the
object's shape (box: size[2]/2, cylinder: length/2), so every SHAPE gets its
own positives -- the cylinder in two poses, as the case most likely to be
computed wrong. Values marked "measured" are the landed poses of the
2026-10-03 smoke episodes.

Positives (expect PASS): each shape in its own bin, upright/flat and lying.
Negatives: each shape resting ON its bin's rim; correct object in the correct
bin but lying on top of a distractor; wrong bin; no lift. Plus a disturbed
distractor, which must PASS with the disturbance flagged.
"""
import copy
import sys

import yaml

sys.path.insert(0, "scripts")
from verify_episode import verify  # noqa: E402

cfg = yaml.safe_load(open("config/objects.yaml"))
FLOOR = 0.002

START = {"red_cube": [0.22, -0.08, 0.02], "blue_cube": [0.10, -0.06, 0.025],
         "green_cylinder": [0.10, 0.02, 0.025], "yellow_box": [0.10, 0.09, 0.02]}


def meta(target, landed, end_overrides=None, bz=(None, 0.10, 0.15, 0.15)):
    m = {
        "episode": 901, "target": target, "task_index": 0, "rc": 0,
        "camera": "train_overhead", "split": "train", "simulated_attachment": True,
        "attach_target_link": "link6", "rtf_observed": 0.3, "wall_seconds": 150,
        "sim_seconds": 20, "host_role": "test", "world_sha256": "test",
        "dz_samples": [0.164, 0.164], "baseline_z": START[target][2],
        "bz_samples": [START[target][2] if b is None else b for b in bz],
        "start_poses": copy.deepcopy(START), "landed_pose": landed,
    }
    m["end_poses"] = copy.deepcopy(START)
    m["end_poses"][target] = landed
    m["end_poses"].update(end_overrides or {})
    return m


def in_bin(target, z, dx=0.0, dy=0.0):
    b = cfg["bins"][cfg["objects"][target]["bin"]]["pose"]
    return [b[0] + dx, b[1] + dy, z]


def rim(target):
    o = cfg["objects"][target]
    half = (o["size"][2] if o["shape"] == "box" else o["length"]) / 2
    return cfg["bins"][o["bin"]]["rim_z"] + half


cases = [
    # --- positives, one block per SHAPE ---------------------------------
    ("cube 40 mm flat (red_cube, measured 0.022)",          meta("red_cube", in_bin("red_cube", 0.022, -0.001, 0.003)), "PASS"),
    ("cube 50 mm flat (blue_cube, 0.002 + 0.025)",          meta("blue_cube", in_bin("blue_cube", FLOOR + 0.025)), "PASS"),
    ("cube 50 mm tilted on a wall (blue, measured 0.0369)", meta("blue_cube", in_bin("blue_cube", 0.0369, 0.002, -0.012)), "PASS"),
    ("cylinder upright (green_cylinder, measured 0.027)",   meta("green_cylinder", in_bin("green_cylinder", 0.027, 0.0005, -0.0003)), "PASS"),
    ("cylinder lying on its side (0.002 + radius 0.022)",   meta("green_cylinder", in_bin("green_cylinder", FLOOR + 0.022)), "PASS"),
    ("box flat on 50x30 face (yellow_box, measured 0.022)", meta("yellow_box", in_bin("yellow_box", 0.022)), "PASS"),
    ("box on its side, 50x40 face (0.002 + 0.015)",         meta("yellow_box", in_bin("yellow_box", FLOOR + 0.015)), "PASS"),
    # --- negatives ------------------------------------------------------
    ("cube 40 mm resting ON the rim",                       meta("red_cube", in_bin("red_cube", rim("red_cube"))), "FAIL"),
    ("cube 50 mm resting ON the rim",                       meta("blue_cube", in_bin("blue_cube", rim("blue_cube"))), "FAIL"),
    ("cylinder upright resting ON the rim",                 meta("green_cylinder", in_bin("green_cylinder", rim("green_cylinder"))), "FAIL"),
    ("box resting ON the rim",                              meta("yellow_box", in_bin("yellow_box", rim("yellow_box"))), "FAIL"),
    # green_cylinder in the green bin, but on top of blue_cube (also in the green bin)
    ("correct object, correct bin, lying ON a distractor",
     meta("green_cylinder", in_bin("green_cylinder", FLOOR + 0.050 + 0.025),
          {"blue_cube": in_bin("green_cylinder", FLOOR + 0.025)}), "FAIL"),
    ("wrong bin (red_cube flat in blue_bin)",               meta("red_cube", in_bin("blue_cube", 0.022)), "WRONG_BIN"),
    ("no lift (object never rose)",                         meta("red_cube", [0.22, -0.08, 0.02], bz=(None, 0.021, 0.02, 0.02)), "FAIL"),
    # --- flagged, not failed --------------------------------------------
    ("disturbed distractor (PASS, flagged)",
     meta("red_cube", in_bin("red_cube", 0.022), {"blue_cube": [0.10, -0.15, 0.025]}), "PASS"),
]

failed = False
for name, m, expected in cases:
    r = verify(m, cfg)
    ok = r["verdict"] == expected
    z = m["landed_pose"][2]
    print(f"{'OK  ' if ok else 'FAIL'} {name:<52} z={z:.4f}  got {r['verdict']:<9} expected {expected}"
          + (f"  distractors_moved={r['distractors_moved']}" if r["distractors_moved"] else ""))
    failed = failed or not ok

sys.exit(1 if failed else 0)
