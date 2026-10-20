#!/usr/bin/env python3
"""Part 9.4 -- combine Part 9.1-9.3's checks into one episode verdict file.

Input: the raw per-episode record run_pick_and_place.py writes (--meta),
containing rc, the lift/transport dz and bz samples, the landed pose, and
the start/end pose snapshot of every object. Output: the verdict JSON,
written next to the input with a '.verdict.json' suffix, and also printed.
Exit 0 only on PASS.
"""
import argparse
import json
import sys

import yaml

from checks import (distractors_undisturbed, grasp_held,
                     landed_in_wrong_bin, object_rose, placed_in_correct_bin)


def verify(meta, cfg):
    target = meta["target"]
    o = cfg["objects"][target]
    result = {
        "episode": meta["episode"], "target": target,
        "task_index": meta["task_index"], "instruction": o["instruction"],
        "expected_bin": o["bin"], "rc": meta["rc"],
        "camera": meta["camera"], "split": meta["split"],
        "simulated_attachment": meta["simulated_attachment"],
        "attach_target_link": meta.get("attach_target_link"),
        "rtf_observed": meta.get("rtf_observed"),
        "wall_seconds": meta.get("wall_seconds"),
        "sim_seconds": meta.get("sim_seconds"),
        "host_role": meta.get("host_role"),
        "world_sha256": meta.get("world_sha256"),
        # port_bag.py (Part 10.5) reads the bag location from HERE, the
        # verdict file, not from grasp_meta.json directly -- it needs the
        # verdict to decide whether to port an episode at all, so it always
        # opens this file first.
        "bag_path": meta.get("bag_path"),
    }

    if meta["rc"] != 0:
        result.update(grasp_held=False, rise_m=0.0, placed_in_correct_bin=False,
                       landed_in_wrong_bin=None, distractors_moved=[],
                       verdict="FAIL")
        return result

    held = grasp_held(meta["dz_samples"])
    rise = max(meta["bz_samples"]) - meta["baseline_z"]
    rose = object_rose(meta["bz_samples"], meta["baseline_z"])
    placed_ok, _detail = placed_in_correct_bin(target, cfg, meta["landed_pose"])
    wrong_bin = landed_in_wrong_bin(target, cfg, meta["landed_pose"])
    _clean, moved = distractors_undisturbed(
        target, meta["start_poses"], meta["end_poses"])

    if not (held and rose):
        verdict = "FAIL"
    elif wrong_bin:
        verdict = "WRONG_BIN"
    elif placed_ok:
        verdict = "PASS"
    else:
        verdict = "FAIL"

    result.update(grasp_held=held, rise_m=round(rise, 4),
                   placed_in_correct_bin=placed_ok,
                   landed_in_wrong_bin=wrong_bin,
                   distractors_moved=moved, verdict=verdict)
    return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("meta_path")
    ap.add_argument("--objects", default="config/objects.yaml")
    args = ap.parse_args()

    meta = json.load(open(args.meta_path))
    cfg = yaml.safe_load(open(args.objects))
    result = verify(meta, cfg)

    out_path = args.meta_path.rsplit(".", 1)[0] + ".verdict.json"
    json.dump(result, open(out_path, "w"), indent=1)
    print(json.dumps(result, indent=1))
    sys.exit(0 if result["verdict"] == "PASS" else 1)


if __name__ == "__main__":
    main()
