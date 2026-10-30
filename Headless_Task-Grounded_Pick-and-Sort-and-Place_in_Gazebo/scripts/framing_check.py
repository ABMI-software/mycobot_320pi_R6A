#!/usr/bin/env python3
"""Camera framing check: is the target in the dataset camera's picture?

  python3 scripts/framing_check.py --episodes 20 24 --out /tmp/framing   (inside the container)

For each episode: generate its own world (its jittered camera, light and
texture), start the simulator with the camera bridged, spawn its scene, then
drive the arm through approach -> grasp -> lift (target welded) -> over the
destination bin, saving the dataset camera's frame (/camera/image_raw) at
start, grasp, lift and over-bin. Writes <out>/ep_NNN_<phase>.png and one
contact sheet per episode, <out>/ep_NNN_sheet.png, labelled.

No motion gate checks framing -- all of them check motion -- so an object
off the frame edge would pass every check and still be useless as data.
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import time

import numpy as np
import rclpy
from PIL import Image, ImageDraw
from sensor_msgs.msg import Image as ImageMsg

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import weld  # noqa: E402
from run_pick_and_place import HOME_Q, Sequencer  # noqa: E402

SHARE = "/workspace/install/mycobot_description/share/mycobot_description"


class Cam:
    def __init__(self, node):
        self.last = None
        node.create_subscription(ImageMsg, "/camera/image_raw", self._cb, 10)

    def _cb(self, m):
        self.last = np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3).copy()


def snap(node, cam, path):
    cam.last = None
    end = time.time() + 30
    while cam.last is None and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    Image.fromarray(cam.last).save(path)
    return cam.last


def run_episode(ep, out):
    import csv
    row = next(r for r in csv.DictReader(open("config/episode_matrix.csv")) if int(r["episode"]) == ep)
    target = row["target"]
    seq = json.load(open("config/waypoints.json"))[f"ep{ep:03d}"]
    subprocess.run(["python3", "scripts/make_episode_world.py", "--episode", str(ep),
                    "--out", f"{SHARE}/worlds/sorting_table.sdf", "--params", f"{out}/ep_{ep:03d}_scene.json"],
                   check=True, capture_output=True)
    sim = subprocess.Popen(["ros2", "launch", "mycobot_gateway", "sim_grasp.launch.py", "world_name:=sorting_table",
                            "headless:=true", "bridge_camera:=true"],
                           stdout=open(f"{out}/ep_{ep:03d}_sim.log", "w"), stderr=subprocess.STDOUT,
                           start_new_session=True)
    try:
        node = Sequencer("sorting_table")
        cam = Cam(node)
        node.wait_ready()
        subprocess.run(["python3", "scripts/spawn_scene.py", "--episode", str(ep)], check=True, capture_output=True)
        frames = {"start": snap(node, cam, f"{out}/ep_{ep:03d}_start.png")}
        node.goto(HOME_Q, 1.0)
        node.goto(seq[0])
        node.goto(seq[1])
        frames["grasp"] = snap(node, cam, f"{out}/ep_{ep:03d}_grasp.png")
        weld.send("attach", target)
        node.goto(seq[2])
        frames["lift"] = snap(node, cam, f"{out}/ep_{ep:03d}_lift.png")
        node.goto(seq[3], sim_duration=3.5)
        frames["over_bin"] = snap(node, cam, f"{out}/ep_{ep:03d}_over_bin.png")
        node.destroy_node()
        return target, row, frames
    finally:
        os.killpg(sim.pid, signal.SIGINT)
        try:
            sim.wait(timeout=15)
        except subprocess.TimeoutExpired:
            os.killpg(sim.pid, signal.SIGKILL)
        time.sleep(3)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, nargs="+", required=True)
    ap.add_argument("--out", default="/tmp/framing")
    a = ap.parse_args()
    os.makedirs(a.out, exist_ok=True)
    os.environ.update(DISPLAY=":0", LIBGL_ALWAYS_SOFTWARE="1", GALLIUM_DRIVER="llvmpipe",
                      MESA_LOADER_DRIVER_OVERRIDE="")
    rclpy.init()
    for ep in a.episodes:
        target, row, frames = run_episode(ep, a.out)
        h, w = next(iter(frames.values())).shape[:2]
        sheet = Image.new("RGB", (w * 4, h + 22), "white")
        d = ImageDraw.Draw(sheet)
        d.text((4, 4), f"ep {ep}  {target}  {row['kind']}  start ({float(row['target_x']):.3f}, "
                       f"{float(row['target_y']):+.3f})  camera: {row['split']}", fill="black")
        for i, (ph, img) in enumerate(frames.items()):
            sheet.paste(Image.fromarray(img), (i * w, 22))
            d.text((i * w + 4, 26), ph, fill="yellow")
        sheet.save(f"{a.out}/ep_{ep:03d}_sheet.png")
        print(f"ep {ep}: {target} sheet -> {a.out}/ep_{ep:03d}_sheet.png", flush=True)
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
