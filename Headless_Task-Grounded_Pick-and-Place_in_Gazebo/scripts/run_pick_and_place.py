#!/usr/bin/env python3
"""run_pick_and_place.py -- the 8-phase headless pick-and-place sequence,
with numeric grasp verification (block pose vs end-effector pose, both
from independent sources) logged throughout, and a camera-frame snapshot
at peak lift.

Architecture note -- why this process never imports moveit_py: an
earlier version built the IK/FK helper into this same process alongside
a plain rclpy Node. Combining rclpy.init()+Node with moveit_py's
MoveItPy in one process was found to make MoveItPy's constructor hang
indefinitely partway through its own internal setup -- reproducible
across multiple attempts AND a full container restart (ruling out
leftover DDS state from earlier kills). Rather than debug moveit_py's
internals further, the architecture is split:
  - precompute_ik.py solves every waypoint in a moveit_py-ONLY process
    (no rclpy at all), writing joint angles to waypoints.json.
  - THIS script is plain rclpy only: sends those precomputed joint
    angles via the standard MoveGroup action (moveit_msgs types, no
    moveit_py import), and gets the live end-effector pose from
    /tf (base->gripper_base, already published by robot_state_publisher)
    via tf2_ros -- no FK computation needed in this process either.

Target frame note: gripper_base, NOT link6 -- see ik_helper.py's
TARGET_LINK comment. link6 to the real fingertips is ~12cm; treating
link6 as the grasp point drove the real gripper underground and caused
a hard physical clamp regardless of command source, independent of
this rclpy/moveit_py issue.

Timing note -- why there are no fixed sleeps: the spec's 200ms command
cooldown and ~1.6s gripper interval come from the REAL robot's hardware
bridge (serial comm latency, TCP rate limiting), which don't
mechanically apply to this sim's controllers. Every arm motion instead
blocks on the actual MoveGroup action result, and the gripper phase
polls /joint_states until the commanded position is actually reached
(or a generous timeout elapses) -- synchronizing on real completion
signals rather than an imported hardware constant.

Gripper convention, corrected here from an earlier (wrong) visual read
during the RLDS conversion work: per controller.yaml's own documented,
measured (31/08) convention, gripper_controller (servo_left) = 0 is
OPEN, negative (toward its -1.20 lower limit) is CLOSED. The command
topic takes all 6 driven joints in one Float64MultiArray:
  [servo_left, servo_right, tip_left, tip_right, bar_left, bar_right]
  Open:  six zeros
  Close: [-a, +a, +a, -a, -a, +a]
"""
import argparse
import csv
import json
import re
import subprocess
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rosetta_interfaces.action import RecordEpisode
from rclpy.node import Node
from rclpy.time import Time
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
from tf2_ros import Buffer, TransformListener
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint

ARM_JOINTS = [
    "joint2_to_joint1", "joint3_to_joint2", "joint4_to_joint3",
    "joint5_to_joint4", "joint6_to_joint5", "joint6output_to_joint6",
]
GRIPPER_JOINT = "gripper_controller"
# Squeeze force is ~P*(target-pos) with P=60 on the servo joints; contact stalls near -0.31,
# so -0.40 squeezes with ~5 Nm instead of ~23 Nm at -0.7.
GRIPPER_CLOSE_MAGNITUDE = 0.40
GRIPPER_OPEN_CMD = [0.0] * 6
GRIPPER_CLOSE_CMD = [-GRIPPER_CLOSE_MAGNITUDE, GRIPPER_CLOSE_MAGNITUDE, GRIPPER_CLOSE_MAGNITUDE,
                     -GRIPPER_CLOSE_MAGNITUDE, -GRIPPER_CLOSE_MAGNITUDE, GRIPPER_CLOSE_MAGNITUDE]
GRIPPER_TOLERANCE = 0.05
PLATE_XY = (0.20, -0.15)
PLATE_RADIUS = 0.06
PLATE_TOP_Z = 0.048
BLOCK_HALF = 0.02


class PickAndPlace(Node):
    def __init__(self, log_path):
        super().__init__("pick_and_place")
        self._client = ActionClient(self, MoveGroup, "/move_action")
        self.latest_joint_state = None
        self.create_subscription(JointState, "/joint_states", self._js_cb, 10)
        self.gripper_pub = self.create_publisher(
            Float64MultiArray, "/gripper_position_controller/commands", 10)
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.log_rows = []
        self.log_path = log_path
        self.t0 = time.time()

    def _js_cb(self, msg):
        self.latest_joint_state = msg

    def wait_for_joint_state(self, timeout=15.0):
        start = time.time()
        while rclpy.ok() and self.latest_joint_state is None and time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
        return self.latest_joint_state is not None

    def wait_for_tf(self, timeout=15.0):
        start = time.time()
        while rclpy.ok() and time.time() - start < timeout:
            rclpy.spin_once(self, timeout_sec=0.5)
            if self.tf_buffer.can_transform("base", "gripper_base", Time()):
                return True
        return False

    # ---------------- arm motion: blocks on the real MoveGroup result ----------------
    def move_to_joints(self, q, label):
        goal = MoveGroup.Goal()
        goal.request.group_name = "arm"
        goal.request.num_planning_attempts = 5
        goal.request.allowed_planning_time = 15.0
        goal.request.max_velocity_scaling_factor = 0.5
        goal.request.max_acceleration_scaling_factor = 0.5
        constraints = Constraints()
        for name, pos in zip(ARM_JOINTS, q):
            jc = JointConstraint(joint_name=name, position=pos,
                                  tolerance_above=0.01, tolerance_below=0.01, weight=1.0)
            constraints.joint_constraints.append(jc)
        goal.request.goal_constraints = [constraints]
        goal.planning_options.plan_only = False

        self._client.wait_for_server()
        done = {"flag": False, "success": False}

        def on_response(future):
            gh = future.result()
            if not gh.accepted:
                self.get_logger().error(f"[{label}] goal rejected")
                done["flag"] = True
                return
            rf = gh.get_result_async()

            def on_result(f2):
                res = f2.result().result
                done["success"] = res.error_code.val == 1
                done["flag"] = True

            rf.add_done_callback(on_result)

        future = self._client.send_goal_async(goal)
        future.add_done_callback(on_response)

        start = time.time()
        while rclpy.ok() and not done["flag"] and time.time() - start < 90.0:
            rclpy.spin_once(self, timeout_sec=0.5)
        self.get_logger().info(f"[{label}] move {'OK' if done['success'] else 'FAILED'}")
        return done["success"]

    def current_arm_positions(self):
        js = self.latest_joint_state
        if js is None:
            return None
        pos_by_name = dict(zip(js.name, js.position))
        try:
            return [pos_by_name[j] for j in ARM_JOINTS]
        except KeyError:
            return None

    def move_to_joints_closed_loop(self, q, label, tol=0.02, max_corrections=4):
        """move_to_joints, then check the ACTUAL resulting joint state and,
        if it's off by more than `tol`, send a corrective move targeting
        q + (q - actual) -- i.e. push further in the direction of the
        residual error, external integral action against a controller
        that was found (empirically, on this sim's large low-reach
        targets) to settle with persistent steady-state error rather than
        reaching the commanded position. Not needed for small moves --
        those were already confirmed exact earlier in this project."""
        target = list(q)
        ok = self.move_to_joints(target, label)
        for i in range(max_corrections):
            rclpy.spin_once(self, timeout_sec=0.3)
            actual = self.current_arm_positions()
            if actual is None:
                break
            error = [q[j] - actual[j] for j in range(6)]
            max_err = max(abs(e) for e in error)
            self.get_logger().info(f"[{label}] correction {i}: max_err={max_err:.4f} rad")
            if max_err <= tol:
                break
            target = [target[j] + error[j] for j in range(6)]
            ok = self.move_to_joints(target, f"{label}_corr{i+1}")
        return ok

    # ---------------- gripper: polls /joint_states for real completion ----------------
    def gripper(self, close, label, timeout=10.0):
        cmd = Float64MultiArray()
        cmd.data = GRIPPER_CLOSE_CMD if close else GRIPPER_OPEN_CMD
        target = -GRIPPER_CLOSE_MAGNITUDE if close else 0.0
        start = time.time()
        last_publish = 0
        history = []
        last_log = 0
        while rclpy.ok() and time.time() - start < timeout:
            if time.time() - last_publish > 0.5:
                self.gripper_pub.publish(cmd)
                last_publish = time.time()
            rclpy.spin_once(self, timeout_sec=0.3)
            js = self.latest_joint_state
            if js and GRIPPER_JOINT in js.name:
                pos = js.position[js.name.index(GRIPPER_JOINT)]
                history.append((time.time(), pos))
                if time.time() - last_log > 1.0:
                    last_log = time.time()
                    self.get_logger().info(f"[{label}] gripper pos {pos:.3f} t={time.time() - start:.1f}s")
                if abs(pos - target) < GRIPPER_TOLERANCE:
                    self.get_logger().info(f"[{label}] gripper reached {pos:.3f} (target {target})")
                    return True
                recent = [p for t, p in history if time.time() - t < 2.0]
                if close and time.time() - start > 3.0 and pos < -0.1 and max(recent) - min(recent) < 0.005:
                    self.get_logger().info(f"[{label}] gripper stalled at {pos:.3f} (object contact, target {target})")
                    return True
        self.get_logger().warning(f"[{label}] gripper did not confirm reaching target within {timeout}s")
        return False

    def start_recording(self, prompt, max_duration_s=1200.0):
        client = ActionClient(self, RecordEpisode, "record_episode")
        if not client.wait_for_server(timeout_sec=120.0):
            raise RuntimeError("record_episode action server not available")
        goal = RecordEpisode.Goal()
        goal.prompt = prompt
        goal.max_duration_s = float(max_duration_s)
        fut = client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
        self._record_handle = fut.result()
        if self._record_handle is None or not self._record_handle.accepted:
            raise RuntimeError("record_episode goal rejected")
        self.get_logger().info(f"recording started, prompt={prompt!r}")

    def stop_recording(self):
        cancel = self._record_handle.cancel_goal_async()
        rclpy.spin_until_future_complete(self, cancel, timeout_sec=30.0)
        res = self._record_handle.get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=60.0)
        result = res.result().result
        self.get_logger().info(f"recording stopped: reason={result.termination_reason} "
                               f"messages={result.messages_written} bag={result.bag_path}")
        return result

    def _gz_publish_empty(self, topic):
        subprocess.run(["gz", "topic", "-t", topic, "-m", "gz.msgs.Empty", "-p", " "],
                       timeout=15, check=True, capture_output=True)

    def attach_block(self):
        self.get_logger().warning("SIMULATED ATTACHMENT: welding block to link6 (NOT a physical grasp)")
        self._gz_publish_empty("/htgpp/attach")

    def detach_block(self):
        self.get_logger().warning("SIMULATED ATTACHMENT: releasing block from link6")
        self._gz_publish_empty("/htgpp/detach")

    def block_pose(self, model_name="red_block"):
        out = subprocess.run(["gz", "model", "-m", model_name, "-p"],
                              capture_output=True, text=True).stdout
        m = re.search(r"\[\s*([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)\s*\]", out)
        return tuple(map(float, m.groups())) if m else None

    def ee_pose(self):
        try:
            t = self.tf_buffer.lookup_transform("base", "gripper_base", Time())
        except Exception as e:
            self.get_logger().warning(f"tf lookup failed: {e}")
            return None
        p = t.transform.translation
        return (p.x, p.y, p.z)

    def snapshot(self, phase):
        rclpy.spin_once(self, timeout_sec=0.2)
        b, e = self.block_pose(), self.ee_pose()
        if b and e:
            row = {
                "t": round(time.time() - self.t0, 2), "phase": phase,
                "bx": b[0], "by": b[1], "bz": b[2],
                "ex": e[0], "ey": e[1], "ez": e[2],
                "dz": round(e[2] - b[2], 4),
            }
            self.log_rows.append(row)
            self.get_logger().info(f"[{phase}] block_z={b[2]:.4f} ee_z={e[2]:.4f} dz={row['dz']:.4f}")
        return b, e

    def track_during(self, phase, seconds, interval=1.0):
        start = time.time()
        while time.time() - start < seconds:
            self.snapshot(phase)
            time.sleep(interval)

    def save_log(self):
        if not self.log_rows:
            return
        with open(self.log_path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(self.log_rows[0].keys()))
            w.writeheader()
            w.writerows(self.log_rows)
        self.get_logger().info(f"grasp log written to {self.log_path}")


def grasp_held(rows, tol=0.008, min_rise=0.05):
    lift = [r for r in rows if r["phase"] in ("lift", "transport")]
    if not lift:
        return False
    ref = lift[0]["dz"]
    constant = all(abs(r["dz"] - ref) < tol for r in lift)
    rows_by_phase = {r["phase"]: r for r in rows}
    start = rows_by_phase.get("descend")
    rose = start is not None and max(r["bz"] for r in lift) - start["bz"] > min_rise
    return constant and rose


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--waypoints", default="/workspace/htgpp/waypoints.json")
    parser.add_argument("--log", default="/workspace/htgpp/grasp_log.csv")
    parser.add_argument("--frame-dir", default="/workspace/htgpp/frames")
    parser.add_argument("--meta", default="/workspace/htgpp/grasp_meta.json")
    parser.add_argument("--record", action="store_true", help="record the episode via rosetta /record_episode")
    parser.add_argument("--prompt", default="pick up the red block and place it on the plate")
    parser.add_argument("--physical-grasp", action="store_true",
                        help="do not use the simulated attachment (friction grasp only)")
    args = parser.parse_args()
    attach = not args.physical_grasp

    with open(args.waypoints) as f:
        wp = json.load(f)

    rclpy.init()
    node = PickAndPlace(args.log)
    if not node.wait_for_joint_state():
        print("FATAL: never received /joint_states", flush=True)
        sys.exit(2)
    if not node.wait_for_tf():
        print("FATAL: never got base->link6 transform", flush=True)
        sys.exit(2)

    m = node.move_to_joints_closed_loop
    ok = True
    bag_path = None
    if args.record:
        node.start_recording(args.prompt)
    if attach:
        node.detach_block()
        node.track_during("detach_init", seconds=2.0, interval=1.0)
    ok &= m(wp["home"], "home"); node.snapshot("home")
    ok &= node.gripper(close=False, label="open")
    ok &= m(wp["approach"], "approach"); node.snapshot("approach")
    ok &= m(wp["descend"], "descend"); node.snapshot("descend")
    node.gripper(close=True, label="grasp")
    if attach:
        node.attach_block()
        node.track_during("attach", seconds=3.0, interval=1.0)
    node.snapshot("grasp")
    ok &= m(wp["lift"], "lift")
    node.track_during("lift", seconds=4.0, interval=1.0)
    subprocess.run(["python3", "/workspace/htgpp/grab_frame.py",
                    f"{args.frame_dir}/lift_peak.png"])
    ok &= m(wp["transport"], "transport")
    node.track_during("transport", seconds=4.0, interval=1.0)
    ok &= m(wp["place"], "place"); node.snapshot("place")
    if attach:
        node.detach_block()
        node.track_during("detach", seconds=3.0, interval=1.0)
    node.gripper(close=False, label="release")
    node.track_during("settle", seconds=4.0, interval=1.0)
    node.snapshot("release")
    final = node.block_pose()
    placed = (final is not None
              and ((final[0] - PLATE_XY[0]) ** 2 + (final[1] - PLATE_XY[1]) ** 2) ** 0.5 < PLATE_RADIUS
              and abs(final[2] - (PLATE_TOP_Z + BLOCK_HALF)) < 0.01)
    ok &= m(wp["retreat"], "retreat"); node.snapshot("retreat")
    ok &= m(wp["home"], "home_final"); node.snapshot("home_final")

    if args.record:
        bag_path = node.stop_recording().bag_path
    node.save_log()
    held = grasp_held(node.log_rows)
    mech = ("SIMULATED ATTACHMENT (gz DetachableJoint), NOT a physical grasp" if attach
            else "physical friction grasp")
    with open(args.meta, "w") as f:
        json.dump({"simulated_attachment": attach, "carry_mechanism": mech,
                   "instruction": "pick up the red block and place it on the plate",
                   "block_final_xyz": final, "placed_on_plate": placed,
                   "bag_path": bag_path, "grasp_held": bool(held)}, f, indent=2)
    print(f"\nCarry mechanism: {mech}")
    print(f"All motions succeeded: {ok}")
    print(f"Block carried (dz constant AND block rose through lift+transport): {held}")
    print(f"Grasp held (dz constant through lift+transport): {held}")
    print(f"Block final pose: {final}  placed on plate: {placed}")

    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if (ok and held and placed) else 1)


if __name__ == "__main__":
    main()
