#!/usr/bin/env python3
"""Minimal reproduction harness: one cube, one trajectory, numbers out.

  python3 scripts/repro_harness.py --standalone            # start + stop its own simulator
  python3 scripts/repro_harness.py                         # use a simulator already running
  python3 scripts/repro_harness.py --standalone --attach   # also weld, lift 5 cm, check it followed

Prints, with wall-clock timestamps for every stage: commanded vs achieved
joints, link6-to-cube distance, and (with --attach) whether the cube rose
with the arm. Exit 0 only if the arm reached the goal (and, with --attach,
the cube followed). Runs in a container shell; respects ROS_DOMAIN_ID and
GZ_PARTITION, so several copies can run side by side.
"""
import argparse
import json
import math
import os
import re
import signal
import subprocess
import sys
import time

import rclpy
from control_msgs.action import FollowJointTrajectory
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectoryPoint

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from run_pick_and_place import ARM_JOINTS, read_all_poses  # noqa: E402
import weld  # noqa: E402
from mycobot_ik import tool_tip  # noqa: E402

WORLD = "sorting_table"
T0 = time.time()


def stamp(msg):
    print(f"[{time.time() - T0:6.1f}s] {msg}", flush=True)


def gz(*args, timeout=15):
    return subprocess.run(["gz", *args], capture_output=True, text=True, timeout=timeout)


def rtf():
    out = gz("topic", "-e", "-n", "1", "-t", f"/world/{WORLD}/stats").stdout
    m = re.search(r"real_time_factor:\s*([\d.]+)", out)
    return float(m.group(1)) if m else float("nan")


class Arm(Node):
    def __init__(self):
        super().__init__("repro_harness")
        self.q = None
        self.create_subscription(JointState, "/joint_states", self._js, 10)
        self.client = ActionClient(self, FollowJointTrajectory,
                                   "/mycobot_controller/follow_joint_trajectory")

    def _js(self, msg):
        if all(j in msg.name for j in ARM_JOINTS):
            self.q = [math.degrees(msg.position[list(msg.name).index(j)]) for j in ARM_JOINTS]

    def wait_joint_states(self, timeout):
        end = time.time() + timeout
        while self.q is None and time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.q is not None

    def settle(self, q_deg, tol_deg=1.5, sim_timeout=3.0):
        """SUCCESSFUL from this controller only means the trajectory's time
        ran out: controller.yaml sets goal_time 0 and no per-joint goal
        tolerances, so the arm can still be degrees short (measured: 2.5-3.9
        deg on J3/J5/J6 at result time). Wait, bounded in SIM time, until
        every joint is within tol_deg. Returns seconds of wall clock waited."""
        t = time.time()
        end = t + sim_timeout / max(rtf(), 0.02)
        while time.time() < end:
            rclpy.spin_once(self, timeout_sec=0.1)
            if max(abs(a - b) for a, b in zip(self.q, q_deg)) <= tol_deg:
                break
        return time.time() - t

    def move(self, q_deg, sim_duration, timeout):
        """FollowJointTrajectory goal: accepted or rejected, then a real result
        code -- nothing is dropped silently the way a topic publish can be."""
        goal = FollowJointTrajectory.Goal()
        goal.trajectory.joint_names = ARM_JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [math.radians(v) for v in q_deg]
        pt.time_from_start.sec = int(sim_duration)
        pt.time_from_start.nanosec = int((sim_duration % 1) * 1e9)
        goal.trajectory.points = [pt]
        fut = self.client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30)
        handle = fut.result()
        if handle is None or not handle.accepted:
            return "REJECTED"
        res = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res, timeout_sec=timeout)
        if not res.done():
            return "TIMEOUT"
        return {0: "SUCCESSFUL", -1: "INVALID_GOAL", -2: "INVALID_JOINTS",
                -3: "OLD_HEADER_TIMESTAMP", -4: "PATH_TOLERANCE_VIOLATED",
                -5: "GOAL_TOLERANCE_VIOLATED"}.get(res.result().result.error_code,
                                                   str(res.result().result.error_code))


def start_sim(camera):
    # Same rendering setup as episode.sh: software OpenGL (the WSL D3D12 path
    # renders dynamic objects white) and DISPLAY=:0 (WSLg).
    os.environ.update(DISPLAY=":0", LIBGL_ALWAYS_SOFTWARE="1", GALLIUM_DRIVER="llvmpipe",
                      MESA_LOADER_DRIVER_OVERRIDE="")
    cmd = ["ros2", "launch", "mycobot_gateway", "sim_grasp.launch.py",
           f"world_name:={WORLD}", "headless:=true", f"bridge_camera:={str(camera).lower()}"]
    log = open(f"/tmp/repro_sim_{os.environ.get('ROS_DOMAIN_ID', '0')}.log", "w")
    return subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)


def stop_sim(proc):
    os.killpg(proc.pid, signal.SIGINT)
    try:
        proc.wait(timeout=15)
    except subprocess.TimeoutExpired:
        os.killpg(proc.pid, signal.SIGKILL)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--standalone", action="store_true", help="start and stop its own simulator")
    ap.add_argument("--camera", action="store_true", help="also bridge /camera/image_raw to ROS")
    ap.add_argument("--attach", action="store_true", help="weld the cube, lift 5 cm, check it followed")
    ap.add_argument("--lift-only", action="store_true",
                    help="lift WITHOUT welding (unloaded), to compare sag with the loaded --attach case")
    ap.add_argument("--episode", type=int, default=1, help="take the cube position and goal from this episode")
    ap.add_argument("--duration", type=float, default=2.0, help="trajectory duration, SIMULATED seconds")
    args = ap.parse_args()

    here = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    os.chdir(here)
    import csv
    row = next(r for r in csv.DictReader(open("config/episode_matrix.csv")) if int(r["episode"]) == args.episode)
    seq = json.load(open("config/waypoints.json"))[f"ep{args.episode:03d}"]
    target = row["target"]
    cx, cy = float(row[f"{target}_x"]), float(row[f"{target}_y"])
    goal_q = seq[1]                                   # the grasp ("descend") pose
    lift_q = seq[2]

    proc = None
    stamp(f"start  domain={os.environ.get('ROS_DOMAIN_ID', '0')} partition={os.environ.get('GZ_PARTITION', '')!r}")
    if args.standalone:
        proc = start_sim(args.camera)
    rclpy.init()
    arm = Arm()
    ok = False
    try:
        if not arm.client.wait_for_server(timeout_sec=120):
            stamp("FAIL: follow_joint_trajectory server never appeared"); return 2
        stamp("trajectory action server up (controller active)")
        if not arm.wait_joint_states(60):
            stamp("FAIL: no /joint_states"); return 2
        stamp(f"joint_states up, rtf={rtf():.3f}")

        model = os.path.abspath(f"models/{target}.sdf")
        r = subprocess.run(["ros2", "run", "ros_gz_sim", "create", "-world", WORLD, "-file", model,
                            "-name", target, "-x", str(cx), "-y", str(cy), "-z", "0.0255"],
                           capture_output=True, text=True, timeout=90)
        weld.send("detach", target)
        p0 = read_all_poses(WORLD, [target, "link6"])
        if target not in p0:
            stamp(f"FAIL: {target} not in the scene after create ({r.stdout.strip()[-80:]})"); return 2
        stamp(f"{target} spawned + weld released at {[round(v, 3) for v in p0[target]]}")

        res = arm.move(goal_q, args.duration, timeout=args.duration / 0.05 + 30)
        err0 = max(abs(a - c) for a, c in zip(arm.q, goal_q))
        waited = arm.settle(goal_q)
        p1 = read_all_poses(WORLD, [target, "link6"])
        err = [round(a - c, 2) for a, c in zip(arm.q, goal_q)]
        d = math.dist(p1["link6"], p1[target])
        stamp(f"goal result={res}  max err at result {err0:.2f} deg, settled after {waited:.1f}s wall  rtf={rtf():.3f}")
        print(f"   commanded {goal_q}\n   achieved  {[round(v, 2) for v in arm.q]}\n   error     {err}"
              f"\n   link6->cube {d:.4f} m  (link6 z {p1['link6'][2]:.4f}, cube z {p1[target][2]:.4f};"
              f" gripper-tip offset is 0.164 m)", flush=True)
        ok = res == "SUCCESSFUL" and max(abs(e) for e in err) <= 1.5

        if ok and (args.attach or args.lift_only):
            if args.attach:
                weld.send("attach", target)
            res2 = arm.move(lift_q, args.duration, timeout=args.duration / 0.05 + 30)
            # tol 0: never stop early -- settle the full window and report the
            # STEADY-STATE error, not the instant it first crossed 1.5 deg.
            arm.settle(lift_q, tol_deg=0.0, sim_timeout=3.0)
            p2 = read_all_poses(WORLD, [target, "link6"])
            rise = p2[target][2] - p1[target][2]
            dz0, dz1 = p1["link6"][2] - p1[target][2], p2["link6"][2] - p2[target][2]
            # Sag at the lift pose: per-joint error, and the tool-tip offset it
            # causes (forward kinematics of achieved vs commanded joints), in mm
            # -- the units the real-arm measurements use (13 mm / 15 mm).
            lerr = [round(a - c, 2) for a, c in zip(arm.q, lift_q)]
            tip_c, tip_a = tool_tip(lift_q), tool_tip(arm.q)
            d = (tip_a - tip_c) * 1000.0
            stamp(f"{'loaded (welded)' if args.attach else 'UNLOADED'} lift result={res2}: "
                  f"object rose {rise:.4f} m, dz {dz0:.4f} -> {dz1:.4f}")
            print(f"   lift commanded {[round(v, 2) for v in lift_q]}\n   lift error     {lerr}"
                  f"\n   tip offset     dx {d[0]:+.1f} dy {d[1]:+.1f} dz {d[2]:+.1f} mm"
                  f"  (|d| {float((d ** 2).sum() ** 0.5):.1f} mm)", flush=True)
            ok = res2 == "SUCCESSFUL" and (rise > 0.05 and abs(dz1 - dz0) < 0.005 if args.attach else True)
    finally:
        arm.destroy_node()
        rclpy.try_shutdown()
        if proc:
            stop_sim(proc)
            stamp("simulator stopped")
    stamp("PASS" if ok else "FAIL")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
