#!/usr/bin/env python3
"""run_demo.py -- the single orchestrator behind pick_and_place_demo.launch.py
and pick_and_place_no_gui_demo.launch.py.

Why a Python orchestrator behind a launch file, rather than a pure
declarative LaunchDescription: the startup sequence has real timing/retry
needs a static launch graph handles poorly -- cold-start controller races,
a settle wait before the block's pose is trustworthy, and the moveit_py/
rclpy process conflict (see ik_helper.py) that forces IK to be solved in a
separate process *before* the live sequencing script ever imports rclpy.
scripts/episode.sh and batch.sh already solved all of this, the hard way,
across 20 recorded episodes -- this script is that same sequence, ported to
Python for clearer error handling, generalised to an arbitrary (not
pre-vetted) block position, and with the recorder made fully optional
(record:=false, the demo default, needs no rosetta install at all).

Usage (normally invoked by the launch files, not directly):
    python3 run_demo.py --gui false --record false --block-x 0.25 --block-y 0.00 \\
        --attach-mode simulated --htgpp-dir /workspace/htgpp
"""
import argparse
import json
import os
import re
import signal
import subprocess
import sys
import time

ARM_JOINTS_MSG = (
    "attach_mode: only 'simulated' is currently supported. A bounded, "
    "three-run physics experiment (see MEASUREMENTS.md §8) could not "
    "produce a rigid-body friction grasp on this arm/gripper -- the block "
    "is carried by a Gazebo DetachableJoint weld instead, labelled "
    "SIMULATED ATTACHMENT everywhere it appears. There is no 'friction' "
    "mode to fall back to yet."
)

KILL_PATTERNS = [
    "[r]osetta.*episode_recorder_node", "[l]ib/rosetta/episode_recorder_node",
    "[l]ib/moveit_ros_move_group/move_group",
    # NOT the generic "ros2 launch" pattern episode.sh used: run_demo.py is
    # itself invoked *as a child of* an outer "ros2 launch pick_and_place_*"
    # process (the launch file's own ExecuteProcess), so that pattern would
    # match and kill its own parent the moment it runs -- confirmed the hard
    # way (LAUNCH_EXIT=143 within the first second of every run). Name the
    # two inner launch files this actually needs to clean up instead.
    "sim_full\\.launch\\.py", "move_group\\.launch\\.py",
    "[g]z sim", "[r]uby .*gz", "[p]arameter_bridge", "[i]mage_bridge",
    "[r]obot_state_publisher",
]


def log(msg):
    print(f"[demo {time.strftime('%H:%M:%S')}] {msg}", flush=True)


def sh(cmd, timeout=None, **kw):
    """subprocess.run, but a timeout returns a failed CompletedProcess (like the
    bash `timeout` command episode.sh relied on) instead of raising -- almost
    every poll in this script is EXPECTED to time out repeatedly before the
    thing it's polling for is ready. Confirmed the hard way: the very first
    `ros2 control list_controllers` call, taken before the sim has even
    started, raised TimeoutExpired and crashed the whole orchestrator."""
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, **kw)
    except subprocess.TimeoutExpired as e:
        stdout = e.stdout.decode() if isinstance(e.stdout, bytes) else (e.stdout or "")
        stderr = e.stderr.decode() if isinstance(e.stderr, bytes) else (e.stderr or "")
        return subprocess.CompletedProcess(cmd, 124, stdout, stderr)


def pkill_stale():
    for pat in KILL_PATTERNS:
        pids = sh(["pgrep", "-f", pat]).stdout.split()
        for pid in pids:
            subprocess.run(["kill", pid], stderr=subprocess.DEVNULL)
    time.sleep(3)
    for pid in sh(["pgrep", "-f", "[g]z sim"]).stdout.split():
        subprocess.run(["kill", "-9", pid], stderr=subprocess.DEVNULL)
    time.sleep(1)


class Demo:
    def __init__(self, args):
        self.args = args
        self.children = []  # Popen handles we own, terminated on exit
        os.makedirs(args.out_dir, exist_ok=True)

    def spawn_bg(self, cmd, logfile):
        f = open(logfile, "w")
        p = subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT)
        self.children.append(p)
        return p

    def cleanup(self, *_):
        log("shutting down demo processes...")
        # SIGINT/SIGTERM to the outer "ros2 launch sim_full.launch.py" Popen
        # handle does NOT reliably cascade to gz sim / move_group: confirmed
        # the hard way -- both were still running, fully alive, after this
        # method reported "process has finished cleanly" for a prior run.
        # ROS2's own launch service needs its *own* signal handling to tear
        # down what it manages; an externally-delivered signal to the
        # wrapper process itself doesn't trigger that. Backstop with the
        # same pkill patterns pkill_stale() uses at the START of a run, so
        # cleanup is guaranteed regardless of ros2 launch's own semantics.
        for p in self.children:
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
        deadline = time.time() + 15
        for p in self.children:
            try:
                p.wait(timeout=max(0, deadline - time.time()))
            except subprocess.TimeoutExpired:
                p.kill()
        pkill_stale()
        left = sh(["pgrep", "-af", "gz sim|move_group|sim_full"]).stdout.strip()
        if left:
            log(f"WARNING: still running after cleanup:\n{left}")
        sys.exit(getattr(self, "exit_code", 0))

    # ---------------------------------------------------------- bring-up
    def wait_controllers(self):
        needed = ["mycobot_controller", "gripper_position_controller", "joint_state_broadcaster"]

        def all_active():
            out = sh(["ros2", "control", "list_controllers"], timeout=25).stdout
            return all(re.search(rf"{c}\b.*active", out) for c in needed)

        for _ in range(60):
            if all_active():
                return True
            if "spawner" in open(f"{self.args.out_dir}/sim.log").read() and \
               "process has died" in open(f"{self.args.out_dir}/sim.log").read():
                break
            time.sleep(5)

        log("controllers not all active yet -- respawning the missing ones sequentially "
            "(known cold-start race: 3 parallel spawners fighting over the controller "
            "manager's lock, see MEASUREMENTS.md)")
        param_file = "/workspace/install/mycobot_description/share/mycobot_description/config/controller.yaml"
        for c in needed:
            out = sh(["ros2", "control", "list_controllers"], timeout=40).stdout
            if re.search(rf"{c}\b.*active", out):
                continue
            cmd = ["ros2", "run", "controller_manager", "spawner", c,
                   "--controller-manager", "/controller_manager",
                   "--controller-manager-timeout", "150", "--switch-timeout", "150"]
            if c != "joint_state_broadcaster":
                cmd += ["--param-file", param_file]
            sh(cmd, timeout=240)
        return all_active()

    def wait_block_settled(self):
        pose = None
        for _ in range(5):
            out = sh(["gz", "model", "-m", "red_block", "-p"], timeout=25).stdout
            m = re.search(r"\[\s*(-?[\d.]+)\s+(-?[\d.]+)\s+(-?[\d.]+)", out)
            if m:
                pose = tuple(map(float, m.groups()))
                break
            time.sleep(10)
        if pose is None:
            return None
        x, y, z = pose
        if not (0.005 < z < 0.05):
            log(f"WARNING: block resting height z={z:.4f} looks wrong (expected ~0.02) -- "
                "it may still be settling, or it fell through/into the ground plane")
        if abs(x - self.args.block_x) > 0.03 or abs(y - self.args.block_y) > 0.03:
            log(f"WARNING: block settled at ({x:.3f},{y:.3f}), {((x-self.args.block_x)**2+(y-self.args.block_y)**2)**0.5*1000:.0f} mm "
                f"from the commanded ({self.args.block_x},{self.args.block_y}) -- IK will target where it actually is")
        return pose

    def run(self):
        a = self.args
        log(f"gui={a.gui} record={a.record} block=({a.block_x},{a.block_y}) attach_mode={a.attach_mode}")

        pkill_stale()

        log(f"launching sim ({'GUI' if a.gui else 'headless'})...")
        self.spawn_bg(
            ["ros2", "launch", "gazebo_to_lerobot_bringup", "sim_full.launch.py",
             f"headless:={'false' if a.gui else 'true'}"],
            f"{a.out_dir}/sim.log")

        if not self.wait_controllers():
            log("FATAL: controllers never reached 'active' -- see sim.log")
            self.exit_code = 1
            return
        log("controllers active. Spawning plate and block...")

        sh(["ros2", "run", "ros_gz_sim", "create", "-world", "empty",
            "-file", f"{a.htgpp_dir}/models/plate.sdf", "-name", "plate",
            "-x", "0.20", "-y", "-0.15", "-z", "0.044"], timeout=90)
        sh(["ros2", "run", "ros_gz_sim", "create", "-world", "empty",
            "-file", f"{a.htgpp_dir}/models/red_block.sdf", "-name", "red_block",
            "-x", str(a.block_x), "-y", str(a.block_y), "-z", "0.0205"], timeout=90)
        time.sleep(20)

        pose = self.wait_block_settled()
        if pose is None:
            log("FATAL: could not read the block's pose after spawning")
            self.exit_code = 1
            return
        log(f"block settled at {pose}")

        log("solving IK for the settled block pose (moveit_py-only process)...")
        wp_path = f"{a.out_dir}/waypoints.json"
        r = sh(["python3", f"{a.htgpp_dir}/precompute_ik.py", "--out", wp_path],
               timeout=180, cwd=a.htgpp_dir)
        print(r.stdout)
        if r.returncode != 0:
            log("FATAL: no reachable elbow-up IK solution for this block position "
                f"({a.block_x}, {a.block_y}) -- try a position closer to (0.25, 0.00); "
                "the reachable envelope on this arm is roughly x in [0.25, 0.31], y in [-0.08, 0.08]")
            self.exit_code = 1
            return

        log("starting move_group...")
        mg_log = f"{a.out_dir}/move_group.log"
        self.spawn_bg(["ros2", "launch", "mycobot_moveit_config", "move_group.launch.py"], mg_log)
        for _ in range(60):
            if os.path.exists(mg_log) and "You can start planning now" in open(mg_log).read():
                break
            time.sleep(3)
        else:
            log("FATAL: move_group never became ready -- see move_group.log")
            self.exit_code = 1
            return

        record_args = []
        if a.record:
            log("record=true: starting the rosetta recorder...")
            contract = f"{a.htgpp_dir}/contracts/mycobot_pick_place.yaml"
            rec_log = f"{a.out_dir}/recorder.log"
            self.spawn_bg(["ros2", "run", "rosetta", "episode_recorder_node", "--ros-args",
                           "-p", f"contract_path:={contract}",
                           "-p", f"bag_base_dir:={a.out_dir}/bag"], rec_log)
            time.sleep(8)
            configured = False
            for _ in range(2):
                r = sh(["ros2", "lifecycle", "set", "--no-daemon", "--spin-time", "15",
                        "/episode_recorder", "configure"], timeout=90)
                if r.returncode == 0:
                    configured = True
                    break
                time.sleep(10)
            if not configured:
                log("FATAL: recorder never configured -- see recorder.log")
                self.exit_code = 1
                return
            r = sh(["ros2", "lifecycle", "set", "--no-daemon", "--spin-time", "15",
                    "/episode_recorder", "activate"], timeout=90)
            if r.returncode != 0:
                log("FATAL: recorder never activated -- see recorder.log")
                self.exit_code = 1
                return
            record_args = ["--record"]

        r = sh(["ros2", "run", "tf2_ros", "tf2_echo", "base", "gripper_base"], timeout=25)
        if "Translation" not in r.stdout:
            log("FATAL: no TF from base to gripper_base")
            self.exit_code = 1
            return

        log("running the pick-and-place sequence...")
        run_log = f"{a.out_dir}/run.log"
        meta_path = f"{a.out_dir}/grasp_meta.json"
        with open(run_log, "w") as f:
            try:
                rc = subprocess.run(
                    ["python3", "run_pick_and_place.py", "--waypoints", wp_path,
                     "--log", f"{a.out_dir}/grasp_log.csv", "--meta", meta_path,
                     "--frame-dir", a.out_dir, *record_args],
                    cwd=a.htgpp_dir, stdout=f, stderr=subprocess.STDOUT, timeout=1200).returncode
            except subprocess.TimeoutExpired:
                log("FATAL: the sequence did not finish within 1200s")
                rc = 124
        print(open(run_log).read()[-2000:])

        try:
            meta = json.load(open(meta_path))
        except Exception:
            meta = {}
        placed = meta.get("placed_on_plate")
        held = meta.get("grasp_held")
        log("=" * 60)
        log(f"RESULT: motions_ok={rc == 0} placed_on_plate={placed} grasp_held={held}")
        log(f"carry mechanism: {meta.get('carry_mechanism', 'SIMULATED ATTACHMENT, not a physical grasp')}")
        log("=" * 60)
        self.exit_code = 0 if (rc == 0 and placed and held) else 1


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--gui", choices=["true", "false"], required=True)
    p.add_argument("--record", choices=["true", "false"], required=True)
    p.add_argument("--block-x", type=float, required=True)
    p.add_argument("--block-y", type=float, required=True)
    p.add_argument("--attach-mode", default="simulated")
    p.add_argument("--htgpp-dir", default="/workspace/htgpp")
    p.add_argument("--out-dir", default="/workspace/htgpp/demo")
    args = p.parse_args()
    args.gui = args.gui == "true"
    args.record = args.record == "true"

    if args.attach_mode != "simulated":
        log(ARM_JOINTS_MSG)
        sys.exit(2)

    demo = Demo(args)
    signal.signal(signal.SIGINT, demo.cleanup)
    signal.signal(signal.SIGTERM, demo.cleanup)
    demo.run()

    if args.gui:
        log("gui=true: leaving Gazebo and move_group running so the result stays visible. "
            "Press Ctrl+C (on the launch process) to shut everything down.")
        signal.pause()
    else:
        demo.cleanup()


if __name__ == "__main__":
    main()
