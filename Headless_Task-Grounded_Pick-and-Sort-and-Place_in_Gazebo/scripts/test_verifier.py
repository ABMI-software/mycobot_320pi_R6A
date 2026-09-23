#!/usr/bin/env python3
"""Part 9.5 -- a verifier that has never rejected anything is not a
verifier. Three deliberate failures, three correct rejections, plus a
sanity check that a genuinely clean episode still passes. Synthetic pose
dictionaries, no simulator."""
import sys

import yaml

sys.path.insert(0, "scripts")
from verify_episode import verify

cfg = yaml.safe_load(open("config/objects.yaml"))

BASE = {
    "episode": 901, "target": "red_cube", "task_index": 0, "rc": 0,
    "camera": "table", "split": "train", "simulated_attachment": True,
    "attach_target_link": "link6", "rtf_observed": 0.14,
    "wall_seconds": 300, "sim_seconds": 42, "host_role": "A",
    "world_sha256": "test",
    "dz_samples": [0.166, 0.166, 0.166, 0.166],
    "bz_samples": [0.02, 0.10, 0.15, 0.15],
    "baseline_z": 0.02,
    "start_poses": {"red_cube": [0.22, -0.08, 0.02],
                    "blue_cube": [0.10, -0.06, 0.025],
                    "green_cylinder": [0.10, 0.02, 0.025],
                    "yellow_box": [0.10, 0.09, 0.02]},
}

cases = []

bad1 = dict(BASE)
bad1["landed_pose"] = [0.24, 0.02, 0.005]        # lands inside blue_bin
bad1["end_poses"] = dict(BASE["start_poses"])
cases.append(("wrong-bin", bad1, "WRONG_BIN"))

bad2 = dict(BASE)
bad2["bz_samples"] = [0.02, 0.021, 0.02, 0.02]   # never rose
bad2["landed_pose"] = [0.22, -0.08, 0.02]
bad2["end_poses"] = dict(BASE["start_poses"])
cases.append(("no-lift", bad2, "FAIL"))

bad3 = dict(BASE)
bad3["landed_pose"] = [0.22, 0.10, 0.005]        # correct
bad3["end_poses"] = dict(BASE["start_poses"])
bad3["end_poses"]["blue_cube"] = [0.10, -0.15, 0.025]  # knocked 0.09m
cases.append(("disturbed-distractor (should still PASS, flagged)", bad3, "PASS"))

good = dict(BASE)
good["landed_pose"] = [0.222, 0.098, 0.005]
good["end_poses"] = dict(BASE["start_poses"])
cases.append(("clean episode", good, "PASS"))

failed = False
for name, meta, expected in cases:
    r = verify(meta, cfg)
    ok = r["verdict"] == expected
    print(f"{'OK  ' if ok else 'FAIL'} {name}: got {r['verdict']}, expected {expected}"
          + (f", distractors_moved={r['distractors_moved']}" if r["distractors_moved"] else ""))
    failed = failed or not ok

sys.exit(1 if failed else 0)
