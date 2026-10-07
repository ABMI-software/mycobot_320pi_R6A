#!/usr/bin/env python3
"""Part 10.5 -- write recorded episodes into the LeRobot-layout dataset.

Adapted from Headless_Task-Grounded_Pick-and-Place_in_Gazebo/scripts/port_bag.py
(the previous, single-object acquisition's own converter, already proven).
Changes for the four-object sort:
  - Four tasks (from config/tasks.jsonl), task_index taken per-episode from
    each episode's own metadata, not hardcoded to one string.
  - Camera key is "table" or "heldout" (matching this project's contracts'
    observation.images.{table,heldout}), not "front"/"right_heldout".
  - Accepts either CompressedImage or plain Image on the bag's image topic
    -- the table camera (/camera/image_raw) is not guaranteed to have a
    compressed stream the way /synth_camera* empirically did in the
    previous acquisition (see that project's own contract comment).
  - meta field names match THIS project's verify_episode.py output
    (placed_in_correct_bin, landed_in_wrong_bin, grasp_held, verdict),
    not the single-object script's placed_on_plate/block_xy_commanded.

No lerobot/torch dependency -- pyarrow (parquet) + ffmpeg (mp4) only, same
as the source script. NOT validated against the real LeRobotDataset loader
here; see Part 10.6 and the dataset README's "validation deferred" note.
"""
import argparse
import json
import subprocess
from pathlib import Path

import cv2
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq
import rclpy.serialization
import rosbag2_py
from sensor_msgs.msg import CompressedImage, Image, JointState

ARM_JOINTS = ["joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
              "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6"]
STATE_JOINTS = ARM_JOINTS + ["gripper_controller"]
FPS = 30


def read_bag(bag_dir, image_topic):
    so = rosbag2_py.StorageOptions(uri=str(bag_dir), storage_id="mcap")
    co = rosbag2_py.ConverterOptions("", "")
    reader = rosbag2_py.SequentialReader()
    reader.open(so, co)
    topic_types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    is_compressed = "CompressedImage" in topic_types.get(image_topic, "")
    joint_states, frames = [], []
    while reader.has_next():
        topic, data, t = reader.read_next()
        if topic == "/joint_states":
            msg = rclpy.serialization.deserialize_message(data, JointState)
            joint_states.append((t, dict(zip(msg.name, msg.position))))
        elif topic == image_topic:
            if is_compressed:
                msg = rclpy.serialization.deserialize_message(data, CompressedImage)
                frames.append((t, bytes(msg.data), True))
            else:
                msg = rclpy.serialization.deserialize_message(data, Image)
                frames.append((t, (bytes(msg.data), msg.width, msg.height, msg.encoding), False))
    joint_states.sort(key=lambda x: x[0])
    frames.sort(key=lambda x: x[0])
    return joint_states, frames


def decode_frame(frame):
    t, payload, compressed = frame
    if compressed:
        return cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
    data, w, h, enc = payload
    arr = np.frombuffer(data, dtype=np.uint8).reshape(h, w, 3)
    return arr[:, :, ::-1] if enc == "rgb8" else arr


def hold_align(joint_states, sample_times):
    out, i = [], 0
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
    for f in frames:
        img = cv2.resize(decode_frame(f), size)
        proc.stdin.write(img.tobytes())
    proc.stdin.close()
    proc.wait()
    if proc.returncode != 0:
        raise RuntimeError(f"ffmpeg failed for {out_path}")


def port_episode(bag_dir, meta_path, image_topic, out_index, out_root, camera_key):
    meta = json.loads(Path(meta_path).read_text())
    joint_states, frames = read_bag(bag_dir, image_topic)
    if not frames:
        raise RuntimeError(f"no frames on {image_topic} in {bag_dir}")
    if not joint_states:
        raise RuntimeError(f"no /joint_states in {bag_dir}")

    sample_times = [f[0] for f in frames]
    aligned_states = hold_align(joint_states, sample_times)
    n = len(sample_times)

    state = np.array([[js.get(j, 0.0) for j in STATE_JOINTS] for js in aligned_states],
                      dtype=np.float32)
    action = state.copy()   # no leader arm: action := achieved state, same as source contract

    t0 = sample_times[0]
    timestamp = np.array([(t - t0) / 1e9 for t in sample_times], dtype=np.float32)
    task_index = meta.get("task_index", 0)

    ep_dir_data = out_root / "data" / "chunk-000"
    ep_dir_video = out_root / "videos" / "chunk-000" / f"observation.images.{camera_key}"
    ep_dir_data.mkdir(parents=True, exist_ok=True)
    ep_dir_video.mkdir(parents=True, exist_ok=True)

    table = pa.table({
        "observation.state": pa.array(state.tolist(), type=pa.list_(pa.float32(), len(STATE_JOINTS))),
        "action": pa.array(action.tolist(), type=pa.list_(pa.float32(), len(STATE_JOINTS))),
        "timestamp": pa.array(timestamp),
        "frame_index": pa.array(np.arange(n, dtype=np.int64)),
        "episode_index": pa.array(np.full(n, out_index, dtype=np.int64)),
        "index": pa.array(np.arange(n, dtype=np.int64)),
        "task_index": pa.array(np.full(n, task_index, dtype=np.int64)),
    })
    parquet_path = ep_dir_data / f"episode_{out_index:06d}.parquet"
    pq.write_table(table, parquet_path)

    sample = decode_frame(frames[0])
    h, w = sample.shape[:2]
    video_path = ep_dir_video / f"episode_{out_index:06d}.mp4"
    write_video(frames, video_path, (w, h))

    return {
        "episode_index": out_index, "length": n, "task_index": task_index,
        "tasks": [meta["instruction"]] if "instruction" in meta else None,
        "camera": camera_key,
        "simulated_attachment": meta.get("simulated_attachment"),
        "grasp_held": meta.get("grasp_held"),
        "placed_in_correct_bin": meta.get("placed_in_correct_bin"),
        "landed_in_wrong_bin": meta.get("landed_in_wrong_bin"),
        "verdict": meta.get("verdict"),
        "video_shape": [h, w, 3],
        "parquet_path": str(parquet_path.relative_to(out_root)),
        "video_path": str(video_path.relative_to(out_root)),
    }


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--episodes-dir", default="/workspace/htgspp/episodes")
    p.add_argument("--out", required=True)
    p.add_argument("--matrix", default="config/episode_matrix.csv")
    p.add_argument("--split", required=True, choices=["train", "heldout"])
    p.add_argument("--image-topic-table", default="/camera/image_raw")
    p.add_argument("--image-topic-heldout", default="/synth_camera_right/image")
    args = p.parse_args()

    import csv
    wanted = [int(r["episode"]) for r in csv.DictReader(open(args.matrix))
              if r["split"] == args.split]

    out_root = Path(args.out)
    out_root.mkdir(parents=True, exist_ok=True)
    (out_root / "meta").mkdir(exist_ok=True)

    episodes_meta = []
    for idx in wanted:
        ep_dir = Path(args.episodes_dir) / f"ep_{idx:03d}"
        # Read the VERDICT file, not grasp_meta.json directly: verify_episode.py
        # (Part 9.4) writes the verdict to a separate '*.verdict.json' file,
        # not back into grasp_meta.json -- checking meta.get("verdict") on the
        # raw meta file (as an earlier version of this script did) always
        # returns None and silently ports nothing. The verdict file is a
        # superset of grasp_meta.json's useful fields (task_index,
        # instruction, simulated_attachment, bag_path) plus the verdict
        # itself, so it is the only file this script needs to read.
        verdict_path = ep_dir / "grasp_meta.verdict.json"
        if not verdict_path.exists():
            print(f"episode {idx}: no verdict file, skipping (not yet run/verified)")
            continue
        meta = json.loads(verdict_path.read_text())
        if meta.get("verdict") != "PASS":
            print(f"episode {idx}: verdict={meta.get('verdict')}, skipping (not clean)")
            continue
        camera_key = "heldout" if args.split == "heldout" else "table"
        image_topic = args.image_topic_heldout if args.split == "heldout" else args.image_topic_table
        bag_path = meta.get("bag_path")
        if not bag_path or not Path(bag_path).exists():
            print(f"episode {idx}: bag_path {bag_path!r} missing or absent, skipping")
            continue
        out_idx = len(episodes_meta)
        rec = port_episode(Path(bag_path), verdict_path, image_topic, out_idx, out_root, camera_key)
        rec["source_episode"] = idx
        episodes_meta.append(rec)
        print(f"episode {idx} -> out_idx {out_idx}: {rec['length']} frames, camera={camera_key}", flush=True)

    with open(out_root / "meta" / "episodes.jsonl", "w") as f:
        for rec in episodes_meta:
            f.write(json.dumps({"episode_index": rec["episode_index"],
                                "tasks": rec["tasks"], "length": rec["length"]}) + "\n")

    tasks = [json.loads(l) for l in open("config/tasks.jsonl")]
    with open(out_root / "meta" / "tasks.jsonl", "w") as f:
        for t in tasks:
            f.write(json.dumps(t) + "\n")

    total_frames = sum(r["length"] for r in episodes_meta)
    info = {
        "codebase_version": "v3.0-manual",
        "robot_type": "mycobot_320_pi",
        "total_episodes": len(episodes_meta),
        "total_frames": total_frames,
        "total_tasks": len(tasks),
        "fps": FPS,
        "features": {
            "observation.state": {"dtype": "float32", "shape": [len(STATE_JOINTS)], "names": STATE_JOINTS},
            "action": {"dtype": "float32", "shape": [len(STATE_JOINTS)], "names": STATE_JOINTS},
            f"observation.images.{'heldout' if args.split == 'heldout' else 'table'}":
                {"dtype": "video", "shape": episodes_meta[0]["video_shape"] if episodes_meta else None},
        },
        "note": "Written directly with pyarrow+ffmpeg, not via the lerobot package. "
                "Not yet loaded/validated with the real LeRobotDataset class -- "
                "Part 10.6, do that before training.",
    }
    with open(out_root / "meta" / "info.json", "w") as f:
        json.dump(info, f, indent=2)
    with open(out_root / "port_manifest.json", "w") as f:
        json.dump(episodes_meta, f, indent=2)

    print(f"\nWrote {len(episodes_meta)} episodes, {total_frames} frames -> {out_root}")


if __name__ == "__main__":
    main()
