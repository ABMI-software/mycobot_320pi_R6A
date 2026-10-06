#!/usr/bin/env python3
"""Part 10.5 -- convert the recorded episodes into LeRobot datasets, with LeRobot's own writer.

  ~/venvs/lerobot044-cpu/bin/python scripts/port_bag.py --split train --out datasets/mycobot_sorting_train
  ~/venvs/lerobot044-cpu/bin/python scripts/port_bag.py --split heldout --out datasets/mycobot_sorting_heldout

Runs in the CPU-only venv of scripts/requirements-lerobot.txt (the one place the
LeRobot version is pinned; this script refuses any other version), not in the
ROS container: the bags are read as MCAP with mcap-ros2-support, no ROS install.

Rewritten 2026-10-05 on LeRobotDataset.create / add_frame / save_episode /
finalize. The previous converter wrote the files by hand and got four things
wrong; each is fixed here:
  - action[t] was a copy of state[t]. Now action[t] = state[t+1] and each
    episode's last frame is dropped. Reason: the recordings hold no controller
    command or desired-state topic, only /joint_states and the camera.
  - timestamps were bag RECEIVE time (wall clock, 625 frames over 106.8 s).
    Now timestamp = frame_index / fps, and each image is paired with the joint
    state nearest to it in HEADER time (simulation time), as the contract says.
  - the camera key differed per split (table / heldout). Now one key,
    observation.images.top, for both: the held-out split differs in camera
    CONFIGURATION, recorded per episode in the sidecar below.
  - no statistics, a hand-made layout, a per-episode index. The writer now
    produces the v3.0 layout, the statistics and the global index.

The 0.4.4 writer stores no custom per-episode fields, so each dataset carries a
sidecar, meta/episodes_extra.jsonl (one line per episode_index): source
episode, split, camera configuration and pose, distractors_moved, simulated
attachment, and the largest image-to-joint-state pairing gap in sim time.
"""
import argparse
import bisect
import csv
import importlib.metadata
import json
import re
import sys
from pathlib import Path

import numpy as np
from mcap_ros2.reader import read_ros2_messages

HERE = Path(__file__).resolve().parent
ARM_JOINTS = ["joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
              "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6"]
STATE_JOINTS = ARM_JOINTS + ["gripper_controller"]
FPS = 30
IMAGE_KEY = "observation.images.top"
IMAGE_TOPIC = "/camera/image_raw"
JOINT_TOPIC = "/joint_states"


def check_pin():
    """The version pinned in requirements-lerobot.txt must be the one installed."""
    pin = re.search(r"^lerobot(?:\[[^\]]*\])?==(\S+)", (HERE / "requirements-lerobot.txt").read_text(), re.M).group(1)
    have = importlib.metadata.version("lerobot")
    if have != pin:
        sys.exit(f"lerobot {have} installed, {pin} pinned in scripts/requirements-lerobot.txt -- install the pin")
    return pin


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def read_bag(bag_dir):
    """(header-time-sorted joint states [(t, 7 positions)], images [(t, HxWx3 uint8 RGB)])."""
    mcap = next(Path(bag_dir).glob("*.mcap"))
    joints, images = [], []
    for m in read_ros2_messages(str(mcap), topics=[JOINT_TOPIC, IMAGE_TOPIC]):
        msg = m.ros_msg
        if m.channel.topic == JOINT_TOPIC:
            pos = dict(zip(msg.name, msg.position))
            joints.append((stamp(msg), np.array([pos[j] for j in STATE_JOINTS], dtype=np.float32)))
        else:
            if msg.encoding not in ("rgb8", "bgr8"):
                raise ValueError(f"{bag_dir}: unexpected image encoding {msg.encoding}")
            img = np.frombuffer(bytes(msg.data), np.uint8).reshape(msg.height, msg.width, 3)
            images.append((stamp(msg), img[:, :, ::-1].copy() if msg.encoding == "bgr8" else img.copy()))
    joints.sort(key=lambda x: x[0])
    images.sort(key=lambda x: x[0])
    return joints, images


def pair(joints, images):
    """Each image with the joint state nearest in header time; returns states and the largest gap (s)."""
    jt = [t for t, _ in joints]
    states, worst = [], 0.0
    for t, _ in images:
        i = bisect.bisect_left(jt, t)
        best = min((k for k in (i - 1, i) if 0 <= k < len(jt)), key=lambda k: abs(jt[k] - t))
        worst = max(worst, abs(jt[best] - t))
        states.append(joints[best][1])
    return states, worst


def clean(ep_dir):
    """Same acceptance as before: final verdict PASS, frames complete, target colour seen, zero re-sends."""
    v = json.loads((ep_dir / "grasp_meta.verdict.json").read_text())
    raw = json.loads((ep_dir / "grasp_meta.json").read_text())
    frames = json.loads((ep_dir / "frames.json").read_text()) if (ep_dir / "frames.json").exists() else {}
    colour = json.loads((ep_dir / "colour.json").read_text()) if (ep_dir / "colour.json").exists() else {}
    problems = [n for n, bad in (("verdict not PASS", v.get("verdict") != "PASS"),
                                 ("frames incomplete", not frames.get("complete")),
                                 ("target colour not visible", not colour.get("ok")),
                                 ("trajectory re-sends", raw.get("trajectory_resends_total") != 0)) if bad]
    return v, problems


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--split", required=True, choices=["train", "heldout"])
    ap.add_argument("--out", required=True, help="dataset root; must not exist yet")
    ap.add_argument("--repo-id", help="default: local/<basename of --out>")
    ap.add_argument("--episodes-dir", default="/workspace/htgspp/episodes")
    ap.add_argument("--bags-dir", help="where the bags are, if not at each verdict's bag_path")
    ap.add_argument("--matrix", default=str(HERE.parent / "config/episode_matrix.csv"))
    a = ap.parse_args()
    pin = check_pin()
    from lerobot.datasets.lerobot_dataset import LeRobotDataset  # after the version check

    wanted = [int(r["episode"]) for r in csv.DictReader(open(a.matrix)) if r["split"] == a.split]
    out = Path(a.out)
    features = {
        "observation.state": {"dtype": "float32", "shape": (len(STATE_JOINTS),), "names": STATE_JOINTS},
        "action": {"dtype": "float32", "shape": (len(STATE_JOINTS),), "names": STATE_JOINTS},
        IMAGE_KEY: {"dtype": "video", "shape": (240, 320, 3), "names": ["height", "width", "channels"]},
    }
    ds = LeRobotDataset.create(repo_id=a.repo_id or f"local/{out.name}", fps=FPS, features=features,
                               root=out, robot_type="mycobot_320_pi", use_videos=True)
    extra, worst_all = [], 0.0
    for src in wanted:
        ep_dir = Path(a.episodes_dir) / f"ep_{src:03d}"
        v, problems = clean(ep_dir)
        if problems:
            print(f"episode {src}: skipped ({', '.join(problems)})")
            continue
        bag = Path(a.bags_dir) / Path(v["bag_path"]).name if a.bags_dir else Path(v["bag_path"])
        joints, images = read_bag(bag)
        states, gap = pair(joints, images)
        worst_all = max(worst_all, gap)
        # action[t] = state[t+1]: no command topic was recorded, so the next achieved
        # state is the action; the last frame has no successor and is dropped.
        for t in range(len(images) - 1):
            ds.add_frame({"observation.state": states[t], "action": states[t + 1],
                          IMAGE_KEY: images[t][1], "task": v["instruction"]})
        ds.save_episode()
        sv = v.get("scene_variation") or {}
        extra.append({"episode_index": len(extra), "source_episode": src, "split": a.split,
                      "camera_config": sv.get("camera_config"), "camera_pose_xyz_rpy": sv.get("camera_pose_xyz_rpy"),
                      "distractors_moved": v.get("distractors_moved") or [],
                      "simulated_attachment": v.get("simulated_attachment"),
                      "frames": len(images) - 1, "max_pair_gap_ms_sim": round(gap * 1000, 3)})
        print(f"episode {src} -> {len(extra) - 1}: {len(images) - 1} frames, pairing gap {gap * 1000:.2f} ms sim",
              flush=True)
    ds.finalize()
    with open(out / "meta" / "episodes_extra.jsonl", "w") as f:
        for e in extra:
            f.write(json.dumps(e) + "\n")
    print(f"\n{len(extra)} episodes, {ds.meta.total_frames} frames -> {out} (lerobot {pin}); "
          f"largest pairing gap {worst_all * 1000:.2f} ms sim")


if __name__ == "__main__":
    main()
