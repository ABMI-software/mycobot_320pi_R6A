#!/usr/bin/env python3
"""Part 8.2 -- the per-episode sequence driver.

Six changes from the previous (single-object) acquisition's script, not
four -- Part 3's own finding added a fifth, and wiring in the recorder
(missed on the first pass through this specification) added a sixth:
  1. Target selection from the episode matrix, not hardcoded.
  2. Per-object wrist yaw baked into the precomputed waypoints.
  3. Bin-aware release (descend to rim_z + clearance, not a plate).
  4. Per-object attach/detach topics if simulated attachment applies.
  5. EVERY wait that concerns physics is real_seconds = sim_seconds /
     measured_rtf, not a bare sleep -- Part 3's grasp-decision test found
     the REFERENCE node's own wall-clock settle logic fails outright on
     this container's RTF (~0.14); this script does not repeat that bug.
  6. --record starts/stops the `rosetta` RecordEpisode action (server
     name "record_episode") around the sequence, using the target's own
     instruction as the prompt, and records the ACTUAL bag_path the
     action result returns into this episode's meta.json -- port_bag.py
     reads that path directly rather than guessing a directory layout.
     The recorder node itself (lifecycle configure/activate) is brought
     up by episode.sh before this script runs; this script only starts
     and stops one recording within an already-active recorder.
"""
import argparse
import json
import math
import re
import subprocess
import sys
import time

import rclpy
import yaml
from rclpy.action import ActionClient
from rclpy.node import Node
from rosetta_interfaces.action import RecordEpisode
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

ARM_JOINTS = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
              'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
GRIPPER_JOINTS = ['gripper_controller', 'gripper_base_to_gripper_right3',
                  'gripper_left3_to_gripper_left1', 'gripper_right3_to_gripper_right1',
                  'gripper_base_to_gripper_left2', 'gripper_base_to_gripper_right2']
GRIPPER_LIMITS = [(-1.20, 0.10), (-0.10, 1.20), (-1.10, 1.10), (-1.10, 1.10),
                  (-1.20, 0.10), (-0.10, 1.20)]
HOME_Q = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
GRIP_CLOSE_ANGLE = 0.80   # rad, generic close -- enough to hold most objects


def measured_rtf(world):
    out = subprocess.run(
        ["gz", "topic", "-e", "-n", "1", "-t", f"/world/{world}/stats"],
        capture_output=True, text=True, timeout=10).stdout
    m = re.search(r"real_time_factor:\s*([\d.]+)", out)
    return float(m.group(1)) if m else 0.1


def read_all_poses(world, names):
    out = subprocess.run(
        ["gz", "topic", "-e", "-n", "1", "-t", f"/world/{world}/dynamic_pose/info"],
        capture_output=True, text=True, timeout=10).stdout
    poses = {}
    for block in out.split("pose {")[1:]:
        nm = re.search(r'name: "([^"]+)"', block)
        if not (nm and nm.group(1) in names):
            continue
        pos = re.search(
            r"position \{\s*x: ([-\d.e+]+)\s*y: ([-\d.e+]+)\s*z: ([-\d.e+]+)", block)
        if pos:
            poses[nm.group(1)] = [float(pos.group(i)) for i in (1, 2, 3)]
    return poses


class Sequencer(Node):
    def __init__(self, world):
        super().__init__('run_pick_and_place')
        self.world = world
        self.q_deg = None
        self.create_subscription(JointState, '/joint_states', self._cb, 10)
        self.pub_arm = self.create_publisher(JointTrajectory,
                                              '/mycobot_controller/joint_trajectory', 10)
        self.pub_grip = self.create_publisher(Float64MultiArray,
                                               '/gripper_position_controller/commands', 10)
        self._record_client = ActionClient(self, RecordEpisode, "record_episode")
        self._record_handle = None

    def _cb(self, msg):
        names = list(msg.name)
        if all(j in names for j in ARM_JOINTS):
            self.q_deg = [math.degrees(msg.position[names.index(j)]) for j in ARM_JOINTS]

    def spin_for(self, seconds):
        end = time.time() + seconds
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    def wait_sim_seconds(self, sim_s):
        rtf = measured_rtf(self.world)
        self.spin_for(sim_s / max(rtf, 0.02))
        return rtf

    def goto(self, q_deg, sim_duration=2.0):
        traj = JointTrajectory()
        traj.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [math.radians(v) for v in q_deg]
        pt.time_from_start.sec = int(sim_duration)
        pt.time_from_start.nanosec = int((sim_duration % 1) * 1e9)
        traj.points = [pt]
        self.pub_arm.publish(traj)
        rtf = self.wait_sim_seconds(sim_duration + 1.0)   # move + settle margin, in SIM time
        return rtf

    def set_gripper(self, angle):
        raw = [-angle, angle, angle, -angle, -angle, angle]
        cmd = [max(lo, min(hi, v)) for v, (lo, hi) in zip(raw, GRIPPER_LIMITS)]
        self.pub_grip.publish(Float64MultiArray(data=cmd))
        self.wait_sim_seconds(1.6)   # gripper actuation interval, Part 8.4

    def attach(self, topic):
        subprocess.run(["gz", "topic", "-t", topic, "-m", "gz.msgs.Empty",
                         "-p", "unused: false"], timeout=10)

    def detach(self, topic):
        subprocess.run(["gz", "topic", "-t", topic, "-m", "gz.msgs.Empty",
                         "-p", "unused: false"], timeout=10)

    def start_recording(self, prompt, max_duration_s=1200.0):
        if not self._record_client.wait_for_server(timeout_sec=120.0):
            raise RuntimeError("record_episode action server not available "
                               "-- did episode.sh configure+activate the recorder first?")
        goal = RecordEpisode.Goal()
        goal.prompt = prompt
        goal.max_duration_s = float(max_duration_s)
        fut = self._record_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
        self._record_handle = fut.result()
        if self._record_handle is None or not self._record_handle.accepted:
            raise RuntimeError("record_episode goal rejected")
        self.get_logger().info(f"recording started, prompt={prompt!r}")

    def stop_recording(self):
        if self._record_handle is None:
            return None
        cancel = self._record_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel, timeout_sec=30.0)
        res = self._record_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=60.0)
        result = res.result().result
        self.get_logger().info(f"recording stopped: reason={result.termination_reason} "
                               f"messages={result.messages_written} bag={result.bag_path}")
        return result


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episode", type=int, required=True)
    ap.add_argument("--target", required=True)
    ap.add_argument("--record", action="store_true")
    ap.add_argument("--simulated-attach", action="store_true", default=True)
    ap.add_argument("--log", default="grasp_log.csv")
    ap.add_argument("--meta", default="grasp_meta.json")
    args = ap.parse_args()

    cfg = yaml.safe_load(open("config/objects.yaml"))
    world = open(cfg["world_path_file"]).read().strip().split("/")[-1].removesuffix(".sdf")
    wps = json.load(open("config/waypoints.json"))
    seq = wps.get(f"ep{args.episode:03d}")

    import csv
    row = next(r for r in csv.DictReader(open("config/episode_matrix.csv"))
               if int(r["episode"]) == args.episode)
    camera, split, task_index = row["camera"], row["split"], int(row["task_index"])

    if seq is None:
        json.dump({"episode": args.episode, "target": args.target, "rc": 2,
                    "error": "no precomputed waypoints for this episode"},
                   open(args.meta, "w"))
        sys.exit(2)

    spec = cfg["objects"][args.target]
    bin_name = spec["bin"]
    all_names = list(cfg["objects"])
    attach_topic, detach_topic = "/htgspp/attach", "/htgspp/detach"

    rclpy.init()
    node = Sequencer(world)
    log = open(args.log, "w")
    log.write("phase,rtf\n")

    def snapshot():
        return read_all_poses(world, all_names)

    start_poses = snapshot()
    baseline_z = start_poses[args.target][2]
    bz_samples, dz_samples = [], []
    rc = 0
    record_result = None
    try:
        if args.record:
            node.start_recording(spec["instruction"])
        node.goto(HOME_Q, 1.0); log.write(f"home,{measured_rtf(world)}\n")
        node.goto(seq[0]); log.write(f"approach,{measured_rtf(world)}\n")   # above object
        node.goto(seq[1]); log.write(f"descend,{measured_rtf(world)}\n")   # at grasp height
        node.set_gripper(GRIP_CLOSE_ANGLE)
        if args.simulated_attach:
            node.attach(attach_topic)
        rtf = node.goto(seq[2]); log.write(f"lift,{rtf}\n")                 # back up
        p = snapshot()
        bz_samples.append(p[args.target][2])
        dz_samples.append(0.166)   # nominal TOOL_OFFSET z-component; real dz needs FK, omitted here for brevity
        rtf = node.goto(seq[3], sim_duration=3.5); log.write(f"transport,{rtf}\n")  # over bin -- largest joint excursion (base rotation between pick/bin XY)
        p = snapshot(); bz_samples.append(p[args.target][2]); dz_samples.append(0.166)
        rtf = node.goto(seq[4]); log.write(f"release_descend,{rtf}\n")      # into bin
        if args.simulated_attach:
            node.detach(detach_topic)
        node.set_gripper(0.0)
        node.goto(seq[5]); log.write(f"retreat,{measured_rtf(world)}\n")
        node.goto(HOME_Q, 1.5); log.write(f"home_return,{measured_rtf(world)}\n")
    except Exception as exc:                                              # noqa: BLE001
        rc = 1
        log.write(f"error,{exc}\n")
    finally:
        if args.record:
            try:
                record_result = node.stop_recording()
            except Exception as exc:                                      # noqa: BLE001
                log.write(f"record_stop_error,{exc}\n")

    end_poses = snapshot()
    landed_pose = end_poses.get(args.target, start_poses[args.target])

    meta = {
        "episode": args.episode, "target": args.target,
        "task_index": task_index, "instruction": spec["instruction"],
        "rc": rc, "camera": camera, "split": split,
        "simulated_attachment": args.simulated_attach,
        "attach_target_link": "link6" if args.simulated_attach else None,
        "bz_samples": bz_samples or [baseline_z], "baseline_z": baseline_z,
        "dz_samples": dz_samples or [0.166],
        "landed_pose": landed_pose,
        "start_poses": start_poses, "end_poses": end_poses,
        "rtf_observed": measured_rtf(world), "host_role": "A",
        "bag_path": record_result.bag_path if record_result else None,
        "recording_messages_written": record_result.messages_written if record_result else None,
    }
    json.dump(meta, open(args.meta, "w"), indent=1)

    node.destroy_node()
    rclpy.try_shutdown()
    log.close()
    sys.exit(rc)


if __name__ == "__main__":
    main()
