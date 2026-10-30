#!/usr/bin/env python3
"""Frames received vs frames expected for one recorded episode.

  python3 scripts/check_frames.py <bag_dir> [--fps 30] [--out frames.json]

expected = recording span in SIMULATED time x camera rate + 1. The span is
taken from /joint_states header stamps (published continuously, so it covers
the whole recording); images are counted on /camera/image_raw. Also reports
the largest gap between consecutive image stamps, in frame periods. Exit 0
only if every expected frame arrived (received >= expected - 1, allowing for
the recording starting or stopping between two renders).
"""
import argparse
import json
import sys

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


def stamps(bag_dir):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag_dir, storage_id=""),
                rosbag2_py.ConverterOptions("cdr", "cdr"))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    out = {}
    while reader.has_next():
        topic, data, _ = reader.read_next()
        msg = deserialize_message(data, get_message(types[topic]))
        out.setdefault(topic, []).append(msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag_dir")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--image-topic", default="/camera/image_raw")
    ap.add_argument("--out")
    args = ap.parse_args()

    s = stamps(args.bag_dir)
    js, im = sorted(s.get("/joint_states", [])), sorted(s.get(args.image_topic, []))
    span = (js[-1] - js[0]) if len(js) > 1 else 0.0
    expected = int(span * args.fps) + 1
    gaps = [b - a for a, b in zip(im, im[1:])]
    r = {"received": len(im), "expected": expected, "span_sim_s": round(span, 3),
         "image_span_sim_s": round(im[-1] - im[0], 3) if len(im) > 1 else 0.0,
         "max_gap_frames": round(max(gaps) * args.fps, 2) if gaps else None,
         "fps": args.fps, "complete": len(im) >= expected - 1}
    if args.out:
        json.dump(r, open(args.out, "w"), indent=1)
    print(json.dumps(r))
    return 0 if r["complete"] else 1


if __name__ == "__main__":
    sys.exit(main())
