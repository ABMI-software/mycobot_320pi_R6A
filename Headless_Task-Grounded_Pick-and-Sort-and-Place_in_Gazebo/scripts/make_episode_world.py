#!/usr/bin/env python3
"""Per-episode scene variation: writes the world file and the parameters used.

  python3 scripts/make_episode_world.py --episode 7 --out <share>/worlds/sorting_table.sdf \
      --params episodes/ep_007/scene_variation.json
  python3 scripts/make_episode_world.py --nominal --out ...     # no jitter (harness, demos)

Varied per episode, from a seed derived from the episode number (a retry
reproduces the same scene):
  * camera pose -- small position and orientation jitter around one of two
    configurations: "train_overhead" for every train episode, and
    "heldout_oblique", RESERVED for the held-out split and used by no train
    episode. Both render /camera/image_raw, 320x240 @ 30 Hz.
  * light -- sun direction and intensity, fill-light intensity, ambient level.
  * table texture -- one of the variants made by scripts/make_table_textures.sh.
  * distractor placement -- already per-episode in config/episode_matrix.csv;
    copied into the parameters so one file holds everything that varied.
"""
import argparse
import csv
import json
import math
import random
import re
from pathlib import Path

SEED_BASE = 20261003
CAMERA_CONFIGS = {
    # x, y, z, roll, pitch, yaw -- the overhead pose real_table.sdf ships with
    "train_overhead": (0.261, 0.017, 0.90, 0.0, math.pi / 2, 0.0),
    # front-right, looking down at the work area centre (0.20, 0.05, 0.0)
    "heldout_oblique": None,
}
POS_JITTER_M = 0.02
ROT_JITTER_RAD = math.radians(2.0)


def look_at(eye, target):
    dx, dy, dz = (t - e for t, e in zip(target, eye))
    yaw = math.atan2(dy, dx)
    pitch = math.atan2(-dz, math.hypot(dx, dy))
    return (*eye, 0.0, pitch, yaw)


CAMERA_CONFIGS["heldout_oblique"] = look_at((0.78, -0.32, 0.60), (0.20, 0.03, 0.0))


def scene_params(episode, row, textures, nominal):
    rng = random.Random(SEED_BASE + (episode or 0))
    config = "heldout_oblique" if row and row["split"] == "heldout" else "train_overhead"
    base = CAMERA_CONFIGS[config]
    if nominal:
        cam = list(base)
        sun_dir, sun_k, fill_k, amb_k = [-0.5, 0.3, -0.9], 1.0, 1.0, 1.0
        texture = "wood_original.png"
    else:
        cam = [base[i] + rng.uniform(-POS_JITTER_M, POS_JITTER_M) for i in range(3)] + \
              [base[i] + rng.uniform(-ROT_JITTER_RAD, ROT_JITTER_RAD) for i in range(3, 6)]
        sun_dir = [-0.5 + rng.uniform(-0.3, 0.3), 0.3 + rng.uniform(-0.3, 0.3), -0.9]
        sun_k, fill_k, amb_k = rng.uniform(0.6, 1.15), rng.uniform(0.5, 1.3), rng.uniform(0.8, 1.2)
        texture = rng.choice(textures)
    p = {
        "episode": episode, "seed": SEED_BASE + (episode or 0), "nominal": nominal,
        "camera_config": config,
        "camera_pose_xyz_rpy": [round(v, 5) for v in cam],
        "camera": {"topic": "/camera/image_raw", "width": 320, "height": 240, "update_rate_hz": 30},
        "light": {"sun_direction": [round(v, 4) for v in sun_dir], "sun_intensity_scale": round(sun_k, 4),
                  "fill_intensity_scale": round(fill_k, 4), "ambient_scale": round(amb_k, 4)},
        "table_texture": texture,
    }
    if row:
        p["object_xy"] = {k[:-2]: [float(row[k]), float(row[k[:-1] + "y"])]
                          for k in row if k.endswith("_x") and k != "target_x"}
        p["target"] = row["target"]
    return p


def render_world(template, p, textures_dir):
    w = template
    x, y, z, r, pi, ya = p["camera_pose_xyz_rpy"]
    w, n = re.subn(r"(<model name=\"table_camera\">\s*<static>true</static>\s*<pose>)[^<]*(</pose>)",
                   rf"\g<1>{x} {y} {z} {r} {pi} {ya}\g<2>", w)
    assert n == 1, "table_camera pose not found"

    def scale(block_name, rgb_tag, k):
        nonlocal w
        m = re.search(rf"<light type=\"directional\" name=\"{block_name}\">.*?</light>", w, flags=re.S)
        blk = m.group(0)
        vals = re.search(rf"<{rgb_tag}>([^<]*)</{rgb_tag}>", blk).group(1).split()
        new = " ".join(f"{min(1.0, float(v) * k):.4f}" for v in vals[:3]) + f" {vals[3]}"
        w = w.replace(blk, re.sub(rf"<{rgb_tag}>[^<]*</{rgb_tag}>", f"<{rgb_tag}>{new}</{rgb_tag}>", blk, count=1))

    scale("sun", "diffuse", p["light"]["sun_intensity_scale"])
    scale("fill_light", "diffuse", p["light"]["fill_intensity_scale"])
    sx, sy, sz = p["light"]["sun_direction"]
    w = re.sub(r"(<light type=\"directional\" name=\"sun\">.*?<direction>)[^<]*(</direction>)",
               rf"\g<1>{sx} {sy} {sz}\g<2>", w, count=1, flags=re.S)
    k = p["light"]["ambient_scale"]
    w = re.sub(r"<ambient>0.5 0.5 0.5 1</ambient>",
               f"<ambient>{0.5 * k:.4f} {0.5 * k:.4f} {0.5 * k:.4f} 1</ambient>", w, count=1)
    w, n = re.subn(r"<albedo_map>[^<]*wood_albedo.png</albedo_map>",
                   f"<albedo_map>{Path(textures_dir, p['table_texture']).resolve()}</albedo_map>", w)
    assert n == 1, "table albedo_map not found"
    return w


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument("--episode", type=int)
    g.add_argument("--nominal", action="store_true", help="no jitter, original texture, train camera")
    ap.add_argument("--heldout-camera", action="store_true", help="with --nominal: use the reserved camera")
    ap.add_argument("--template", default="worlds/sorting_table.sdf")
    ap.add_argument("--textures-dir", default="textures")
    ap.add_argument("--matrix", default="config/episode_matrix.csv")
    ap.add_argument("--out", required=True)
    ap.add_argument("--params")
    args = ap.parse_args()

    textures = sorted(p.name for p in Path(args.textures_dir).glob("*.png"))
    assert textures, f"no textures in {args.textures_dir} -- run scripts/make_table_textures.sh"
    row = None
    if args.episode is not None:
        row = next(r for r in csv.DictReader(open(args.matrix)) if int(r["episode"]) == args.episode)
    elif args.heldout_camera:
        row = {"split": "heldout", "target": ""}
    p = scene_params(args.episode, row, textures, args.nominal)
    if args.nominal:
        p.pop("object_xy", None); p.pop("target", None)
    Path(args.out).write_text(render_world(Path(args.template).read_text(), p, args.textures_dir))
    if args.params:
        Path(args.params).write_text(json.dumps(p, indent=1))
    print(json.dumps({k: p[k] for k in ("episode", "camera_config", "camera_pose_xyz_rpy", "table_texture")}))


if __name__ == "__main__":
    main()
