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

import weld
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rosetta_interfaces.action import RecordEpisode
from rosgraph_msgs.msg import Clock
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from trajectory_msgs.msg import JointTrajectoryPoint

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


def read_orientation(world, name):
    """Quaternion (x, y, z, w) of one model, from the same topic as the poses."""
    out = subprocess.run(
        ["gz", "topic", "-e", "-n", "1", "-t", f"/world/{world}/dynamic_pose/info"],
        capture_output=True, text=True, timeout=10).stdout
    for block in out.split("pose {")[1:]:
        nm = re.search(r'name: "([^"]+)"', block)
        if not (nm and nm.group(1) == name):
            continue
        q = re.search(r"orientation \{([^}]*)\}", block)
        qd = dict(re.findall(r"([xyzw]): ([-\d.e+]+)", q.group(1))) if q else {}
        return [float(qd.get(k, 0.0)) for k in "xyzw"]   # proto3 text omits zero fields
    return None


class Sequencer(Node):
    def __init__(self, world):
        super().__init__('run_pick_and_place')
        self.world = world
        self.q_deg = None
        self.create_subscription(JointState, '/joint_states', self._cb, 10)
        self.sim_now = None
        # /clock is published RELIABLE by ros_gz_bridge at ~170 Hz (wall);
        # a BEST_EFFORT subscription is compatible and is what the
        # controllers themselves use (checked with `ros2 topic info -v`).
        self.create_subscription(Clock, '/clock', self._on_clock, qos_profile_sensor_data)
        self.last_resends = 0
        self.arm_client = ActionClient(self, FollowJointTrajectory,
                                       '/mycobot_controller/follow_joint_trajectory')
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

    def _on_clock(self, msg):
        self.sim_now = msg.clock.sec + msg.clock.nanosec * 1e-9

    def wait_sim_seconds(self, sim_s):
        """Wait until the SIMULATION clock has advanced by sim_s, read from
        /clock. Dividing by a single real-time-factor sample instead turned a
        1.6 s gripper wait into 32 s when one sample read 0.05 (smoke episode
        1, 2026-10-03). Returns the real-time factor actually achieved."""
        first = time.time() + 30.0
        while self.sim_now is None:
            if time.time() > first:
                raise RuntimeError("no /clock message within 30 s -- is ros_gz_bridge running?")
            rclpy.spin_once(self, timeout_sec=0.05)
        t_sim, t_wall = self.sim_now, time.time()
        deadline = t_wall + sim_s / 0.01          # an RTF below 0.01 counts as stalled
        while self.sim_now - t_sim < sim_s:
            if time.time() > deadline:
                raise RuntimeError(f"simulation clock stalled: advanced {self.sim_now - t_sim:.3f} "
                                   f"of {sim_s} sim s in {time.time() - t_wall:.0f} s wall")
            rclpy.spin_once(self, timeout_sec=0.05)
        return (self.sim_now - t_sim) / max(time.time() - t_wall, 1e-3)

    def wait_ready(self, timeout_s=120.0):
        """Block until /joint_states arrives, the trajectory ACTION server is up,
        and the gripper controller has subscribed to our topic publisher. A
        topic message published before discovery completes is dropped without
        any error -- found live, the first arm command of a fresh node went
        nowhere. The arm therefore goes through FollowJointTrajectory (goals
        are accepted or rejected, then return a result code); the gripper
        stays on its topic, guarded by the subscription count."""
        if not self.arm_client.wait_for_server(timeout_sec=timeout_s):
            raise RuntimeError("follow_joint_trajectory action server not available")
        end = time.time() + timeout_s
        while time.time() < end:
            if self.q_deg is not None and self.pub_grip.get_subscription_count() > 0:
                return
            self.spin_for(0.5)
        raise RuntimeError("no /joint_states, or gripper controller not subscribed")

    def goto(self, q_deg, sim_duration=2.0, tol_deg=1.5, attempts=2):
        # attempts=2: at most ONE re-send per pose. A re-send changes the
        # recorded trajectory, so the count is kept (self.last_resends) and
        # written per phase into grasp_meta.json.
        """Send a goal, check its result, then wait (bounded, in SIM time) for
        the measured joints to converge: SUCCESSFUL from this controller only
        means the trajectory's time ran out when no goal tolerances are set (the
        harness measured the arm 2.5-8.2 deg short at result time). Since
        2026-10-03 controller.yaml sets 1.5 deg goal tolerances with a 1 s
        goal_time, so GOAL_TOLERANCE_VIOLATED (-5) is possible: converge, then
        re-send once if it still has not converged."""
        rtf = measured_rtf(self.world)
        self.last_resends = 0
        for n in range(attempts):
            self.last_resends = n
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = ARM_JOINTS
            pt = JointTrajectoryPoint()
            pt.positions = [math.radians(v) for v in q_deg]
            pt.time_from_start.sec = int(sim_duration)
            pt.time_from_start.nanosec = int((sim_duration % 1) * 1e9)
            goal.trajectory.points = [pt]
            fut = self.arm_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
            handle = fut.result()
            if handle is None or not handle.accepted:
                raise RuntimeError(f"trajectory goal rejected: {q_deg}")
            res = handle.get_result_async()
            rclpy.spin_until_future_complete(
                self, res, timeout_sec=(sim_duration + 5.0) / max(rtf, 0.02))
            if not res.done():
                raise RuntimeError(f"trajectory goal timed out: {q_deg}")
            code = res.result().result.error_code
            # -5 GOAL_TOLERANCE_VIOLATED: not within 1.5 deg after the
            # controller's goal_time -- converge below, then re-send.
            if code not in (0, -5):
                raise RuntimeError(f"trajectory goal failed, error_code={code}: {q_deg}")
            settle_end = time.time() + 3.0 / max(rtf, 0.02)
            while time.time() < settle_end and self.max_err(q_deg) > tol_deg:
                rclpy.spin_once(self, timeout_sec=0.1)
            rtf = measured_rtf(self.world)
            if self.max_err(q_deg) <= tol_deg:
                break
        else:
            # Fail fast: carrying on with an arm that has not reached its pose
            # is how an episode ends up dragging objects (episode 13, attempt 1).
            raise RuntimeError(f"ARM_NOT_CONVERGED: max joint error {self.max_err(q_deg):.2f} deg "
                               f"> {tol_deg} after {attempts} sends, goal {q_deg}")
        return rtf

    def max_err(self, q_deg):
        return max(abs(a - b) for a, b in zip(self.q_deg, q_deg))

    def set_gripper(self, angle):
        raw = [-angle, angle, angle, -angle, -angle, angle]
        cmd = [max(lo, min(hi, v)) for v, (lo, hi) in zip(raw, GRIPPER_LIMITS)]
        self.pub_grip.publish(Float64MultiArray(data=cmd))
        self.wait_sim_seconds(1.6)   # gripper actuation interval, Part 8.4

    def attach(self, name):
        weld.send("attach", name)     # waits for the plugin's subscription; raises if unheard

    def detach(self, name):
        weld.send("detach", name)

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
    # The labelled weld (Part 3 / A8) is the default; --physical-grasp turns it
    # off to test the friction grasp alone, under the same verifier.
    ap.add_argument("--physical-grasp", dest="simulated_attach", action="store_false")
    ap.add_argument("--variation", help="scene_variation.json from make_episode_world.py")
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
    # One topic pair PER OBJECT: with a single shared pair, every object's
    # DetachableJoint answered the same attach message and all four objects
    # were welded to link6 at once, distractors included.
    wall_start = time.time()

    rclpy.init()
    node = Sequencer(world)
    log = open(args.log, "w")
    log.write("phase,t_wall_s,rtf,q_err_deg,target_x,target_y,target_z,link6_x,link6_y,link6_z\n")

    def snapshot():
        return read_all_poses(world, all_names + ["link6"])

    def sample_carry(poses):
        bz_samples.append(poses[args.target][2])
        dz_samples.append(poses["link6"][2] - poses[args.target][2])

    def mark(phase, rtf, q_goal=None):
        p = snapshot()
        err = round(node.max_err(q_goal), 2) if q_goal is not None else ""
        if q_goal is not None:
            resends[phase] = node.last_resends
        t, l6 = p.get(args.target, [""] * 3), p.get("link6", [""] * 3)
        log.write(f"{phase},{time.time() - wall_start:.1f},{rtf:.3f},{err}," + ",".join(f"{v:.4f}" for v in (*t, *l6)) + "\n")
        log.flush()
        return p

    node.wait_ready()
    ready_wait_s = round(time.time() - wall_start, 1)
    start_poses = snapshot()
    baseline_z = start_poses[args.target][2]
    bz_samples, dz_samples = [], []
    resends = {}
    rc = 0
    error = None
    record_result = None
    try:
        if args.record:
            node.start_recording(spec["instruction"])
        mark("home", node.goto(HOME_Q, 1.0), HOME_Q)
        p = mark("approach", node.goto(seq[0]), seq[0])             # above object
        # Postcondition of the release at spawn: once the arm has made its
        # first real motion, NO object may have moved. An object still welded
        # to link6 is dragged by that motion (held-out episode 13, attempt 1:
        # cube ended 0.52 m away, off the table). Abort with a named error.
        moved = {n: round(math.dist(p[n][:2], start_poses[n][:2]), 4) for n in all_names
                 if n in p and math.dist(p[n][:2], start_poses[n][:2]) > 0.005}
        if moved:
            raise RuntimeError(f"OBJECT_MOVED_BEFORE_GRASP: {moved} (m) after the approach motion "
                               "-- an object is still welded to link6 from spawn")
        mark("descend", node.goto(seq[1]), seq[1])                  # at grasp height
        node.set_gripper(GRIP_CLOSE_ANGLE)
        if args.simulated_attach:
            node.attach(args.target)
        mark("grasp", measured_rtf(world))
        p_lift = mark("lift", node.goto(seq[2]), seq[2])            # back up
        sample_carry(p_lift)
        # Two independent signals must agree: the plugin CONFIRMED the weld
        # (weld.send raises otherwise), and the object actually rose. A
        # confirmed weld with an object still on the table is a named failure,
        # never left to grasp_held alone (episode 57: dz "constant" while the
        # box never moved).
        if args.simulated_attach and p_lift[args.target][2] - baseline_z < 0.05:
            raise RuntimeError(f"WELD_CONFIRMED_BUT_NOT_CARRIED: {args.target} rose only "
                               f"{p_lift[args.target][2] - baseline_z:.4f} m at the lift")
        # over bin -- largest joint excursion (base rotation between pick/bin XY)
        sample_carry(mark("transport", node.goto(seq[3], sim_duration=3.5), seq[3]))
        mark("release_descend", node.goto(seq[4]), seq[4])         # into bin
        if args.simulated_attach:
            node.detach(args.target)
        node.set_gripper(0.0)
        mark("open", measured_rtf(world))
        mark("retreat", node.goto(seq[5]), seq[5])
        mark("home_return", node.goto(HOME_Q, 1.5), HOME_Q)
    except Exception as exc:                                              # noqa: BLE001
        rc = 1
        error = f"{type(exc).__name__}: {exc}"
        log.write(f"error,{exc}\n")
    finally:
        if args.record:
            try:
                record_result = node.stop_recording()
            except Exception as exc:                                      # noqa: BLE001
                log.write(f"record_stop_error,{exc}\n")

    end_poses = snapshot()
    landed_pose = end_poses.get(args.target, start_poses[args.target])
    # Landed orientation (added 2026-10-04, after batch f977d7af65dc): the
    # 60/60 fit result was inferred from landed height; yaw and tilt make it
    # a measurement. Yaw is the object's world yaw; tilt is its z axis from
    # vertical.
    lq = read_orientation(world, args.target)
    landed_yaw_deg = landed_tilt_deg = None
    if lq:
        x, y, z, w = lq
        landed_yaw_deg = round(math.degrees(math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))), 2)
        landed_tilt_deg = round(math.degrees(math.acos(max(-1.0, min(1.0, 1 - 2 * (x * x + y * y))))), 2)
    for poses in (start_poses, end_poses):
        poses.pop("link6", None)

    meta = {
        "episode": args.episode, "target": args.target,
        "task_index": task_index, "instruction": spec["instruction"],
        "rc": rc, "error": error, "camera": camera, "split": split,
        "simulated_attachment": args.simulated_attach,
        "attach_target_link": "link6" if args.simulated_attach else None,
        "bz_samples": bz_samples or [baseline_z], "baseline_z": baseline_z,
        "dz_samples": dz_samples,
        "landed_pose": landed_pose,
        "landed_quat_xyzw": lq, "landed_yaw_deg": landed_yaw_deg, "landed_tilt_deg": landed_tilt_deg,
        "start_poses": start_poses, "end_poses": end_poses,
        "rtf_observed": measured_rtf(world), "host_role": "single-host",
        "wall_seconds": round(time.time() - wall_start, 1),
        "ready_wait_s": ready_wait_s,
        "trajectory_resends": resends,
        "trajectory_resends_total": sum(resends.values()),
        "scene_variation": json.load(open(args.variation)) if args.variation else None,
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
