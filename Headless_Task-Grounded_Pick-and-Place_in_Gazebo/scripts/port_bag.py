#!/usr/bin/env python3
"""port_bag.py -- write one bag+meta into the shared LeRobot-layout dataset.

No lerobot/torch dependency (see MEASUREMENTS.md: venv_lerobot's pip install
pulled the full CUDA/triton stack and was repeatedly killed by container
restarts on this 4.9 GB laptop). Writes the documented layout directly with
pyarrow (parquet) and ffmpeg (mp4), matching the shapes the real
LeRobotDataset loader expects:
  meta/info.json, meta/episodes.jsonl, meta/tasks.jsonl
  data/chunk-000/episode_XXXXXX.parquet
  videos/chunk-000/observation.images.front/episode_XXXXXX.mp4

NOT validated against the real `lerobot` package here -- that happens on the
GPU machine later (see dataset README: "Validation split" note).
"""
import argparse
import json
import re
import shutil
import subprocess
import sys
from pathlib import Path

import cv2
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
import rclpy.serialization
import rosbag2_py
from sensor_msgs.msg import CompressedImage, JointState

ARM_JOINTS = ["joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
              "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6"]
STATE_JOINTS = ARM_JOINTS + ["gripper_controller"]
FPS = 30


def read_bag(bag_dir, image_topic):
    so = rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap")
    co = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(so, co)
    joint_states = []   # (t_ns, position dict)
    frames = []          # (t_ns, jpg bytes)
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == "/joint_states":
            msg = rclpy.serialization.deserialize_message(data, JointState)
            joint_states.append((t, dict(zip(msg.name, msg.position))))
        elif topic == image_topic:
            msg = rclpy.serialization.deserialize_message(data, CompressedImage)
            frames.append((t, bytes(msg.data)))
    joint_states.sort(key=lambda x: x[0])
    frames.sort(key=lambda x: x[0])
    return joint_states, frames


def hold_align(joint_states, sample_times):
    """contract align.strategy: hold -- last joint_states at-or-before each sample time."""
    out = []
    i = 0
    last = joint_states[0][1] if joint_states else {}
    for t in sample_times:
        while i < len(joint_states) and joint_states[i][0] <= t:
            last = joint_states[i][1]
            i += 1
        out.append(last)
    return out


def write_video(frames, out_path, size):
    proc = subprocess.Popen(
        ["ffmpeg", "-y", "-f", "rawvideo", "-pix_fmt", "bgr24",
         "-s", f"{size[0]}x{size[1]}", "-r", str(FPS), "-i", "-",
         "-pix_fmt", "yuv420p", "-c:v", "libx264", "-crf", "18", str(out_path)],
        stdin=subprocess.PIPE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    for _, jpg in frames:
        img = cv2.imdecode(np.frombuffer(jpg, dtype=np.uint8), cv2.IMREAD_COLOR)
        img = cv2.resize(img, size)
        proc.stdin.write(img.tobytes())
    proc.stdin.close()
    proc.wait()
    if proc.returncode != 0:
        raise RuntimeError(f"ffmpeg failed for {out_path}")


def port_episode(bag_dir, meta_path, image_topic, episode_index, out_root, camera_key):
    meta = json.loads(Path(meta_path).read_text())
    joint_states, frames = read_bag(bag_dir, image_topic)
    if not frames:
        raise RuntimeError(f"no frames on {image_topic} in {bag_dir}")
    if not joint_states:
        raise RuntimeError(f"no /joint_states in {bag_dir}")

    sample_times = [t for t, _ in frames]
    aligned_states = hold_align(joint_states, sample_times)
    n = len(sample_times)

    state = np.array([[js.get(j, 0.0) for j in STATE_JOINTS] for js in aligned_states], dtype=np.float32)
    action = state.copy()  # contract: action := follower's own achieved state (no leader arm)

    t0 = sample_times[0]
    timestamp = np.array([(t - t0) / 1e9 for t in sample_times], dtype=np.float32)

    ep_dir_data = out_root / "data" / "chunk-000"
    ep_dir_video = out_root / "videos" / "chunk-000" / f"observation.images.{camera_key}"
    ep_dir_data.mkdir(parents=True, exist_ok=True)
    ep_dir_video.mkdir(parents=True, exist_ok=True)

    table = pa.table({
        "observation.state": pa.array(state.tolist(), type=pa.list_(pa.float32(), len(STATE_JOINTS))),
        "action": pa.array(action.tolist(), type=pa.list_(pa.float32(), len(STATE_JOINTS))),
        "timestamp": pa.array(timestamp),
        "frame_index": pa.array(np.arange(n, dtype=np.int64)),
        "episode_index": pa.array(np.full(n, episode_index, dtype=np.int64)),
        "index": pa.array(np.arange(n, dtype=np.int64)),
        "task_index": pa.array(np.zeros(n, dtype=np.int64)),
    })
    parquet_path = ep_dir_data / f"episode_{episode_index:06d}.parquet"
    pq.write_table(table, parquet_path)

    sample = cv2.imdecode(np.frombuffer(frames[0][1], dtype=np.uint8), cv2.IMREAD_COLOR)
    h, w = sample.shape[:2]
    video_path = ep_dir_video / f"episode_{episode_index:06d}.mp4"
    write_video(frames, video_path, (w, h))

    return {
        "episode_index": episode_index,
        "length": n,
        "tasks": [meta["instruction"]],
        "camera": camera_key,
        "simulated_attachment": meta.get("simulated_attachment"),
        "placed_on_plate": meta.get("placed_on_plate"),
        "grasp_held": meta.get("grasp_held"),
        "block_xy_commanded": meta.get("block_xy_commanded"),
        "video_shape": [h, w, 3],
        "parquet_path": str(parquet_path.relative_to(out_root)),
        "video_path": str(video_path.relative_to(out_root)),
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--episodes-dir", default="/workspace/htgpp/episodes")
    p.add_argument("--out", required=True)
    p.add_argument("--indices", required=True, help="space-separated episode indices, e.g. '1 2 3'")
    p.add_argument("--image-topic-front", default="/synth_camera/image/compressed")
    p.add_argument("--image-topic-heldout", default="/synth_camera_right/image/compressed")
    args = p.parse_args()

    out_root = Path(args.out)
    out_root.mkdir(parents=True, exist_ok=True)
    (out_root / "meta").mkdir(exist_ok=True)

    episodes_meta = []
    for idx_str in args.indices.split():
        idx = int(idx_str)
        ep_dir = Path(args.episodes_dir) / f"ep_{idx:03d}"
        meta_path = ep_dir / "grasp_meta.json"
        meta = json.loads(meta_path.read_text())
        camera_key = "front" if meta.get("camera") == "front" else "right_heldout"
        image_topic = args.image_topic_heldout if meta.get("camera") == "heldout" else args.image_topic_front
        bag_dirs = [d for d in (ep_dir / "bag").iterdir() if d.is_dir()]
        if not bag_dirs:
            raise RuntimeError(f"no bag under {ep_dir}/bag")
        bag_dir = bag_dirs[0]
        # output episode_index is the position in THIS dataset (0-based, contiguous)
        out_idx = len(episodes_meta)
        rec = port_episode(bag_dir, meta_path, image_topic, out_idx, out_root, camera_key)
        rec["source_episode"] = idx
        episodes_meta.append(rec)
        print(f"episode {idx} -> out_idx {out_idx}: {rec['length']} frames, camera={camera_key}", flush=True)

    with open(out_root / "meta" / "episodes.jsonl", "w") as f:
        for rec in episodes_meta:
            f.write(json.dumps({"episode_index": rec["episode_index"], "tasks": rec["tasks"],
                                 "length": rec["length"]}) + "\n")

    with open(out_root / "meta" / "tasks.jsonl", "w") as f:
        f.write(json.dumps({"task_index": 0,
                            "task": "pick up the red block and place it on the plate"}) + "\n")

    total_frames = sum(r["length"] for r in episodes_meta)
    info = {
        "codebase_version": "v3.0-manual",
        "robot_type": "mycobot_320_pi",
        "total_episodes": len(episodes_meta),
        "total_frames": total_frames,
        "total_tasks": 1,
        "fps": FPS,
        "features": {
            "observation.state": {"dtype": "float32", "shape": [len(STATE_JOINTS)], "names": STATE_JOINTS},
            "action": {"dtype": "float32", "shape": [len(STATE_JOINTS)], "names": STATE_JOINTS},
            "observation.images.front": {"dtype": "video", "shape": episodes_meta[0]["video_shape"]},
        },
        "note": "Written directly with pyarrow+ffmpeg, NOT via the lerobot package "
                "(no torch on this machine). Not yet loaded/validated with the real "
                "LeRobotDataset class -- do that on the GPU machine before training.",
    }
    with open(out_root / "meta" / "info.json", "w") as f:
        json.dump(info, f, indent=2)

    with open(out_root / "port_manifest.json", "w") as f:
        json.dump(episodes_meta, f, indent=2)

    print(f"\nWrote {len(episodes_meta)} episodes, {total_frames} frames -> {out_root}")


if __name__ == "__main__":
    main()
