#!/usr/bin/env python3
"""run_demo.py -- the single orchestrator behind pick_and_sort_and_place_demo.launch.py
and pick_and_sort_and_place_no_gui_demo.launch.py.

Runs ONE episode of the matrix (config/episode_matrix.csv) the way
scripts/episode.sh does in the batch -- same world generation, simulator,
controller bring-up, scene, sequence and verifier -- without the recorder:
the demo is for watching or for a pass/fail check, the batch is the data
path. The episode picks the target, its start position, the three other
objects' positions and the scene variation (camera, light, table texture).

Usage (normally invoked by the launch files, not directly):
    python3 scripts/run_demo.py --gui false --episode 16 --project-dir /workspace/htgspp

Exit code: 0 if the verifier says PASS (target lifted and placed in its own
bin), 1 otherwise, 2 for a bad argument.
"""
import argparse
import csv
import json
import os
import re
import signal
import subprocess
import sys
import threading
import time

SHARE = "/workspace/install/mycobot_description/share/mycobot_description"
WORLD = "sorting_table"
CONTROLLERS = ["joint_state_broadcaster", "mycobot_controller", "gripper_position_controller"]

# NOT a generic "ros2 launch" pattern: this script runs as a child of the
# outer "ros2 launch pick_and_sort_and_place_*" process, which that pattern
# would kill (the pick-and-place demo found this out, exit 143 within a
# second). Name the inner launch file instead.
KILL_PATTERNS = [
    "sim_grasp\\.launch\\.py", "[g]z sim", "[r]un_pick_and_place", "[c]ontroller_manager/spawner",
    "[p]arameter_bridge", "[i]mage_bridge", "[r]obot_state_publisher",
]

SIMULATED_ONLY = (
    "attach_mode: only 'simulated' is supported. The target is carried by a "
    "Gazebo DetachableJoint weld to link6, not a friction grasp (doc/LIMITATIONS.md, "
    "'The grasp mechanism'); there is no other mode to select."
)


def log(msg):
    print(f"[demo {time.strftime('%H:%M:%S')}] {msg}", flush=True)


def sh(cmd, timeout=None, **kw):
    """subprocess.run whose timeout returns a failed result instead of raising:
    most polls below are expected to time out until the thing is ready."""
    try:
        return subprocess.run(cmd, capture_output=True, text=True, timeout=timeout, **kw)
    except subprocess.TimeoutExpired:
        return subprocess.CompletedProcess(cmd, 124, "", "")


def kill_stale():
    for pat in KILL_PATTERNS:
        for pid in sh(["pgrep", "-f", pat]).stdout.split():
            subprocess.run(["kill", "-9", pid], stderr=subprocess.DEVNULL)
    # Fast DDS shared-memory segments outlive their processes (episode.sh).
    subprocess.run("rm -rf /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_*", shell=True)
    # A ROS 2 daemon left from a previous run caches a graph without the new
    # controller manager: list_controllers then reports nothing for minutes
    # (first test of this script: 283 s). episode.sh stops it for the same reason.
    sh(["ros2", "daemon", "stop"], timeout=20)
    time.sleep(2)


def frame_gui_camera():
    """The GUI's default view frames the whole world and leaves the arm
    tiny; move it close to the bench once the GUI service exists."""
    for _ in range(60):
        if "/gui/move_to/pose" in sh(["gz", "service", "-l"], timeout=10).stdout.split():
            break
        time.sleep(2)
    else:
        return
    time.sleep(5)
    sh(["gz", "service", "-s", "/gui/move_to/pose", "--reqtype", "gz.msgs.GUICamera",
        "--reptype", "gz.msgs.Boolean", "--timeout", "5000", "--req",
        "pose: {position: {x: 0.75, y: -0.45, z: 0.55}, "
        "orientation: {w: 0.37847, x: -0.24197, y: 0.10319, z: 0.88745}}"], timeout=15)


class Demo:
    def __init__(self, a):
        self.a = a
        self.out = os.path.join(a.project_dir, "demo")
        os.makedirs(self.out, exist_ok=True)
        self.children = []
        self.exit_code = 1

    def bg(self, cmd, logfile):
        p = subprocess.Popen(cmd, stdout=open(logfile, "w"), stderr=subprocess.STDOUT,
                             cwd=self.a.project_dir)
        self.children.append(p)
        return p

    def cleanup(self, *_):
        log("shutting down the simulator...")
        for p in self.children:
            if p.poll() is None:
                p.send_signal(signal.SIGINT)
        deadline = time.time() + 15
        for p in self.children:
            try:
                p.wait(timeout=max(0.0, deadline - time.time()))
            except subprocess.TimeoutExpired:
                p.kill()
        # SIGINT to the outer ros2 launch does not reliably reach gz sim.
        kill_stale()
        sys.exit(self.exit_code)

    def controllers_active(self):
        out = sh(["ros2", "control", "list_controllers"], timeout=15).stdout
        # exact state column: a substring 'active' also matches 'inactive'
        states = {l.split()[0]: l.split()[-1] for l in out.splitlines() if l.split()}
        return all(states.get(c) == "active" for c in CONTROLLERS)

    def wait_controllers(self):
        for _ in range(30):
            if self.controllers_active():
                return True
            time.sleep(8)
        log("controllers not all active -- respawning the missing ones (cold-start race, "
            "MEASUREMENTS.md section 3)")
        for c in CONTROLLERS:
            sh(["ros2", "run", "controller_manager", "spawner", c,
                "--controller-manager", "/controller_manager",
                "--controller-manager-timeout", "150", "--switch-timeout", "150"], timeout=200)
        return self.controllers_active()

    def run(self):
        a, d = self.a, self.a.project_dir
        row = next((r for r in csv.DictReader(open(f"{d}/config/episode_matrix.csv"))
                    if int(r["episode"]) == a.episode), None)
        if row is None:
            log(f"FATAL: episode {a.episode} is not in config/episode_matrix.csv (1-60)")
            self.exit_code = 2
            return
        target = row["target"]
        log(f"episode {a.episode}: {target} at ({float(row['target_x']):.3f}, "
            f"{float(row['target_y']):+.3f}), {row['kind']}, camera {row['split']}; gui={a.gui}")

        kill_stale()
        r = sh(["python3", "scripts/make_episode_world.py", "--episode", str(a.episode),
                "--out", f"{SHARE}/worlds/{WORLD}.sdf", "--params", f"{self.out}/scene_variation.json"],
               timeout=60, cwd=d)
        if r.returncode != 0:
            log(f"FATAL: world generation failed:\n{r.stdout}{r.stderr}")
            return

        log(f"launching the simulator ({'GUI' if a.gui else 'headless'})...")
        self.bg(["ros2", "launch", "mycobot_gateway", "sim_grasp.launch.py", f"world_name:={WORLD}",
                 f"headless:={'false' if a.gui else 'true'}", "bridge_camera:=false"],
                f"{self.out}/sim.log")
        if a.gui:
            threading.Thread(target=frame_gui_camera, daemon=True).start()
        if not self.wait_controllers():
            log(f"FATAL: controllers never became active -- see {self.out}/sim.log")
            return

        log("controllers active; spawning the four objects and four bins...")
        r = sh(["python3", "scripts/spawn_scene.py", "--episode", str(a.episode)], timeout=600, cwd=d)
        if r.returncode != 0:
            log(f"FATAL: scene spawn failed:\n{r.stdout[-1500:]}")
            return
        log(r.stdout.strip().splitlines()[-1])

        log("running the pick-and-sort-and-place sequence...")
        meta = f"{self.out}/grasp_meta.json"
        with open(f"{self.out}/run.log", "w") as f:
            rc = subprocess.run(
                ["python3", "scripts/run_pick_and_place.py", "--episode", str(a.episode),
                 "--target", target, "--variation", f"{self.out}/scene_variation.json",
                 "--log", f"{self.out}/grasp_log.csv", "--meta", meta],
                cwd=d, stdout=f, stderr=subprocess.STDOUT, timeout=1200).returncode
        v = sh(["python3", "scripts/verify_episode.py", meta], timeout=60, cwd=d)
        print(v.stdout[-1500:], flush=True)
        try:
            verdict = json.load(open(meta.replace(".json", ".verdict.json")))
        except (OSError, json.JSONDecodeError):
            verdict = {}
        log("=" * 60)
        log(f"RESULT: sequence_rc={rc} verdict={verdict.get('verdict')} "
            f"placed_in_correct_bin={verdict.get('placed_in_correct_bin')} "
            f"distractors_moved={verdict.get('distractors_moved') or 'none'}")
        log("carry: SIMULATED ATTACHMENT (DetachableJoint weld), not a physical grasp")
        log("=" * 60)
        self.exit_code = 0 if (rc == 0 and v.returncode == 0 and verdict.get("verdict") == "PASS") else 1


def main():
    p = argparse.ArgumentParser()
    p.add_argument("--gui", choices=["true", "false"], required=True)
    p.add_argument("--episode", type=int, required=True)
    p.add_argument("--attach-mode", default="simulated")
    p.add_argument("--project-dir", default="/workspace/htgspp")
    a = p.parse_args()
    a.gui = a.gui == "true"
    if a.attach_mode != "simulated":
        log(SIMULATED_ONLY)
        sys.exit(2)
    # WSLg display and software OpenGL: an unreachable DISPLAY stalls the
    # simulation; the D3D12 GPU path renders every spawned object white
    # (episode.sh, MEASUREMENTS.md section 1).
    os.environ.update(DISPLAY=":0", LIBGL_ALWAYS_SOFTWARE="1", GALLIUM_DRIVER="llvmpipe",
                      MESA_LOADER_DRIVER_OVERRIDE="")
    demo = Demo(a)
    signal.signal(signal.SIGINT, demo.cleanup)
    signal.signal(signal.SIGTERM, demo.cleanup)
    demo.run()
    if a.gui:
        log("gui=true: Gazebo stays open on the final scene. Ctrl+C the launch to shut down.")
        signal.pause()
    demo.cleanup()


if __name__ == "__main__":
    main()
