#!/usr/bin/env python3
"""Part 8.1 -- spawn all four objects + four bins for one episode, verified.

Every wait that concerns physics is expressed in SIMULATION time, divided
by the measured real-time factor -- not wall-clock -- per Part 8.1's own
warning (and Part 3's own measured lesson: this container's RTF is ~0.14,
not ~1, and a bare wall-clock sleep is nowhere near enough).
"""
import argparse
import csv
import re
import subprocess
import sys
import time

import yaml


def spawn(model_file, name, x, y, z, world):
    subprocess.run(["ros2", "run", "ros_gz_sim", "create",
                    "-world", world, "-file", model_file, "-name", name,
                    "-x", str(x), "-y", str(y), "-z", str(z)],
                   check=True, timeout=90)


def measured_rtf(world):
    out = subprocess.run(
        ["gz", "topic", "-e", "-n", "1", "-t", f"/world/{world}/stats"],
        capture_output=True, text=True, timeout=10).stdout
    m = re.search(r"real_time_factor:\s*([\d.]+)", out)
    return float(m.group(1)) if m else 0.1


def read_pose(world, name):
    out = subprocess.run(
        ["gz", "topic", "-e", "-n", "1", "-t", f"/world/{world}/dynamic_pose/info"],
        capture_output=True, text=True, timeout=10).stdout
    for block in out.split("pose {")[1:]:
        nm = re.search(r'name: "([^"]+)"', block)
        if not (nm and nm.group(1) == name):
            continue
        pos = re.search(
            r"position \{\s*x: ([-\d.e+]+)\s*y: ([-\d.e+]+)\s*z: ([-\d.e+]+)", block)
        if pos:
            return tuple(float(pos.group(i)) for i in (1, 2, 3))
    return None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episode", type=int, required=True)
    ap.add_argument("--models-dir", default="models")
    args = ap.parse_args()

    cfg = yaml.safe_load(open("config/objects.yaml"))
    world = open(cfg["world_path_file"]).read().strip().split("/")[-1].removesuffix(".sdf")
    row = next(r for r in csv.DictReader(open("config/episode_matrix.csv"))
               if int(r["episode"]) == args.episode)

    for bn, b in cfg["bins"].items():
        spawn(f"{args.models_dir}/{bn}.sdf", bn, *b["pose"], world)

    for name, o in cfg["objects"].items():
        half = (o["size"][2] / 2) if o["shape"] == "box" else (o["length"] / 2)
        z = cfg["table_top_z"] + half + 0.0005
        spawn(f"{args.models_dir}/{name}.sdf", name,
              float(row[f"{name}_x"]), float(row[f"{name}_y"]), z, world)

    rtf = measured_rtf(world)
    wait_s = cfg["spawn"]["settle_seconds_sim"] / max(rtf, 0.02)
    time.sleep(wait_s)

    bad = []
    for name in cfg["objects"]:
        p = read_pose(world, name)
        want = (float(row[f"{name}_x"]), float(row[f"{name}_y"]))
        if p is None or abs(p[0] - want[0]) > 0.01 or abs(p[1] - want[1]) > 0.01:
            bad.append((name, want, p[:2] if p else None))
    if bad:
        print("SPAWN DRIFT:", bad)
        sys.exit(1)
    print(f"scene OK (rtf={rtf:.3f}, settled {wait_s:.1f}s wall for "
          f"{cfg['spawn']['settle_seconds_sim']}s sim)")


if __name__ == "__main__":
    main()
