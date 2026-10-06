#!/usr/bin/env python3
"""Placement probe: does an object settle well inside its bin, level, and stay put?

  python3 scripts/bin_probe.py --episodes 20 16 23     (inside the container)

For each episode: its own world, the simulator, its scene, the full pick and
place sequence (no recording). Then, on the settled target:
  a. centre margin: half-opening minus the larger of |dx|, |dy| from the bin
     centre (mm), and footprint margin (mm): the object's footprint rotated by
     its measured yaw -- a 50 mm cube turned 30 deg spans 68.3 mm, not 50;
  b. tilt: angle between the object's z axis and the world's (deg);
  c. drift: position (mm) and tilt (deg) change over a further 3 and 6
     SIMULATED seconds.
Gate: a. footprint margin > 5 mm, b < 3 deg, c < 1 mm and < 0.5 deg. Exit 0
only if every episode passes all three. b is not redundant with a: on the
70 mm probe, episodes 16 and 23 read 5.5 / 5.9 mm of (then axis-aligned)
margin and sat propped on the rim at 28 deg (MEASUREMENTS.md section 9).
"""
import argparse
import csv
import math
import os
import re
import signal
import subprocess
import sys
import time

import rclpy
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from run_pick_and_place import Sequencer  # noqa: E402

SHARE = "/workspace/install/mycobot_description/share/mycobot_description"
WORLD = "sorting_table"


def pose_quat(name):
    out = subprocess.run(["gz", "topic", "-e", "-n", "1", "-t", f"/world/{WORLD}/dynamic_pose/info"],
                         capture_output=True, text=True, timeout=10).stdout
    for block in out.split("pose {")[1:]:
        if f'name: "{name}"' not in block.split("position")[0]:
            continue
        p = re.search(r"position \{\s*x: ([-\d.e+]+)\s*y: ([-\d.e+]+)\s*z: ([-\d.e+]+)", block)
        q = re.search(r"orientation \{([^}]*)\}", block)
        qd = {k: float(v) for k, v in re.findall(r"([xyzw]): ([-\d.e+]+)", q.group(1))} if q else {}
        return [float(p.group(i)) for i in (1, 2, 3)], [qd.get(k, 0.0) for k in "xyzw"] if qd else [0, 0, 0, 1]
    return None, None


def tilt_deg(q):
    x, y, z, w = q
    # z-axis of the rotated frame: third column of the rotation matrix
    zz = 1 - 2 * (x * x + y * y)
    return math.degrees(math.acos(max(-1.0, min(1.0, zz))))


def yaw_deg(q):
    x, y, z, w = q
    return math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, nargs="+", required=True)
    a = ap.parse_args()
    os.environ.update(DISPLAY=":0", LIBGL_ALWAYS_SOFTWARE="1", GALLIUM_DRIVER="llvmpipe",
                      MESA_LOADER_DRIVER_OVERRIDE="")
    cfg = yaml.safe_load(open("config/objects.yaml"))
    rows = {int(r["episode"]): r for r in csv.DictReader(open("config/episode_matrix.csv"))}
    rclpy.init()
    all_ok = True
    for ep in a.episodes:
        t = rows[ep]["target"]; o = cfg["objects"][t]; b = cfg["bins"][o["bin"]]
        subprocess.run(["python3", "scripts/make_episode_world.py", "--episode", str(ep),
                        "--out", f"{SHARE}/worlds/{WORLD}.sdf"], check=True, capture_output=True)
        sim = subprocess.Popen(["ros2", "launch", "mycobot_gateway", "sim_grasp.launch.py", f"world_name:={WORLD}",
                                "headless:=true", "bridge_camera:=false"],
                               stdout=open(f"/tmp/bin_probe_{ep}.log", "w"), stderr=subprocess.STDOUT,
                               start_new_session=True)
        try:
            node = Sequencer(WORLD)
            node.wait_ready()
            subprocess.run(["python3", "scripts/spawn_scene.py", "--episode", str(ep)], check=True, capture_output=True)
            r = subprocess.run(["python3", "scripts/run_pick_and_place.py", "--episode", str(ep), "--target", t,
                                "--log", f"/tmp/bin_probe_{ep}.csv", "--meta", f"/tmp/bin_probe_{ep}.json"],
                               capture_output=True, text=True, timeout=1200)
            node.wait_sim_seconds(2.0)
            p0, q0 = pose_quat(t)
            node.wait_sim_seconds(3.0)
            p3, q3 = pose_quat(t)
            node.wait_sim_seconds(3.0)
            p6, q6 = pose_quat(t)
            node.destroy_node()
        finally:
            os.killpg(sim.pid, signal.SIGINT)
            try:
                sim.wait(timeout=15)
            except subprocess.TimeoutExpired:
                os.killpg(sim.pid, signal.SIGKILL)
            time.sleep(3)
        half_open = b["inner_radius"]
        dx, dy = p6[0] - b["pose"][0], p6[1] - b["pose"][1]
        hx, hy = (o["size"][0] / 2, o["size"][1] / 2) if o["shape"] == "box" else (o["radius"], o["radius"])
        yaw = yaw_deg(q6)
        c, s_ = abs(math.cos(math.radians(yaw))), abs(math.sin(math.radians(yaw)))
        if o["shape"] == "box":
            hx, hy = hx * c + hy * s_, hx * s_ + hy * c
        centre_m = (half_open - max(abs(dx), abs(dy))) * 1000
        foot_m = min(half_open - abs(dx) - hx, half_open - abs(dy) - hy) * 1000
        tilt = tilt_deg(q6)
        drift3, drift6 = math.dist(p0, p3) * 1000, math.dist(p0, p6) * 1000
        dtilt = abs(tilt_deg(q6) - tilt_deg(q0))
        ok_a, ok_b, ok_c = foot_m > 5.0, tilt < 3.0, drift6 < 1.0 and dtilt < 0.5
        all_ok &= ok_a and ok_b and ok_c and r.returncode == 0
        print(f"ep {ep:>2} {t} start ({float(rows[ep]['target_x']):.3f}, {float(rows[ep]['target_y']):+.3f}) rc={r.returncode}\n"
              f"   a. landed at ({p6[0]:.4f}, {p6[1]:.4f}, {p6[2]:.4f}); offset dx {dx*1000:+.1f} dy {dy*1000:+.1f} mm; "
              f"yaw {yaw:+.1f} deg; centre margin {centre_m:.1f} mm, footprint margin {foot_m:.1f} mm -> {'PASS' if ok_a else 'FAIL'}\n"
              f"   b. tilt {tilt:.2f} deg -> {'PASS' if ok_b else 'FAIL'}\n"
              f"   c. drift +3 s {drift3:.2f} mm, +6 s {drift6:.2f} mm, tilt change {dtilt:.2f} deg -> {'PASS' if ok_c else 'FAIL'}",
              flush=True)
    rclpy.try_shutdown()
    print("GATE", "PASS" if all_ok else "FAIL")
    sys.exit(0 if all_ok else 1)


if __name__ == "__main__":
    main()
