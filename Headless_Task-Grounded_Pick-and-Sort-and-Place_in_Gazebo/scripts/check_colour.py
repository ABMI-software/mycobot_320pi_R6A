#!/usr/bin/env python3
"""Colour guard: is the TARGET object visibly its own colour in the recording?

  python3 scripts/check_colour.py <bag_dir> <episode_dir> [--out colour.json]

Takes the first /camera/image_raw frame of the recording (the arm is still at
HOME), projects the target object's start position into it with the episode's
own camera pose (scene_variation.json) and the camera's horizontal FOV, and
measures the fraction of pixels in a small window around that point whose
colour classifies as the target's colour. Exit 0 only if it is high enough.

Why at the object's own position and not anywhere in the frame: the
same-coloured bin is always in view, so a whole-frame check passes even when
the object itself renders white -- which is exactly what WSL's D3D12 GPU path
does to every dynamic object (MEASUREMENTS.md section 1). That failure still
produces a PASS verdict from the motion checks; this guard is what rejects it.
Also writes the window, enlarged, to <episode_dir>/colour_crop.png.
"""
import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import rosbag2_py
from PIL import Image
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Image as ImageMsg

HFOV = 1.05            # worlds/sorting_table.sdf, table_camera
HALF = 5               # window = 11 x 11 px
MIN_FRACTION = 0.25
COLOUR = {"red_cube": "red", "blue_cube": "blue", "green_cylinder": "green", "yellow_box": "yellow"}


def classify(px):
    r, g, b = (px[..., i].astype(float) for i in range(3))
    return {
        "red": (r > 110) & (g < 0.55 * r) & (b < 0.55 * r),
        "blue": (b > 110) & (r < 0.6 * b) & (g < 0.8 * b),
        "green": (g > 90) & (r < 0.75 * g) & (b < 0.75 * g),
        "yellow": (r > 110) & (g > 100) & (b < 0.55 * np.minimum(r, g)),
    }


def first_frame(bag_dir, topic="/camera/image_raw"):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id=""),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    while reader.has_next():
        t, data, _ = reader.read_next()
        if t == topic:
            m = deserialize_message(data, ImageMsg)
            return np.frombuffer(bytes(m.data), np.uint8).reshape(m.height, m.width, 3)
    return None


def project(p_world, cam_xyz_rpy, w, h):
    x, y, z, roll, pitch, yaw = cam_xyz_rpy
    cr, sr, cp, sp, cy, sy = (math.cos(roll), math.sin(roll), math.cos(pitch),
                              math.sin(pitch), math.cos(yaw), math.sin(yaw))
    R = np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                  [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                  [-sp, cp * sr, cp * cr]])
    pc = R.T @ (np.array(p_world) - np.array([x, y, z]))      # camera: +x forward, +y left, +z up
    if pc[0] <= 0:
        return None
    f = (w / 2) / math.tan(HFOV / 2)
    return (w / 2 - f * pc[1] / pc[0], h / 2 - f * pc[2] / pc[0])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag_dir")
    ap.add_argument("episode_dir")
    ap.add_argument("--out")
    args = ap.parse_args()
    ep = Path(args.episode_dir)
    meta = json.load(open(ep / "grasp_meta.json"))
    var = json.load(open(ep / "scene_variation.json"))
    target = meta["target"]
    want = COLOUR[target]

    img = first_frame(args.bag_dir)
    r = {"target": target, "expected_colour": want, "fraction": 0.0, "ok": False}
    if img is None:
        r["error"] = "no image in the bag"
    else:
        h, w = img.shape[:2]
        uv = project(meta["start_poses"][target], var["camera_pose_xyz_rpy"], w, h)
        if uv is None or not (HALF <= uv[0] < w - HALF and HALF <= uv[1] < h - HALF):
            r["error"] = f"target projects outside the image: {uv}"
        else:
            u, v = int(round(uv[0])), int(round(uv[1]))
            win = img[v - HALF:v + HALF + 1, u - HALF:u + HALF + 1]
            masks = classify(win)
            r.update(pixel=[u, v], fraction=round(float(masks[want].mean()), 3),
                     fractions={k: round(float(m.mean()), 3) for k, m in masks.items()},
                     window_mean_rgb=[int(c) for c in win.reshape(-1, 3).mean(0)])
            r["ok"] = r["fraction"] >= MIN_FRACTION
            Image.fromarray(win).resize((110, 110), Image.NEAREST).save(ep / "colour_crop.png")
    if args.out:
        json.dump(r, open(args.out, "w"), indent=1)
    print(json.dumps(r))
    return 0 if r["ok"] else 1


if __name__ == "__main__":
    sys.exit(main())
