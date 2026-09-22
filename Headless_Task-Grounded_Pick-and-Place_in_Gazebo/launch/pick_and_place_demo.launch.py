"""pick_and_place_demo.launch.py -- single entry point to watch the
task-grounded pick-and-place demo run, Gazebo GUI included.

For a reviewer with a PC capable of running the Gazebo viewport (this
project's own laptop cannot -- see MEASUREMENTS.md §1). Everything that
used to require scripts/episode.sh plus a 7-step manual walkthrough (launch
sim, poll controllers, spawn models, wait to settle, solve IK offline,
launch move_group, poll it ready, run the sequence) now happens behind this
one command:

    ros2 launch pick_and_place_demo.launch.py
    ros2 launch pick_and_place_demo.launch.py block_x:=0.28 block_y:=0.05
    ros2 launch pick_and_place_demo.launch.py record:=true

Defaults are chosen for *watching*, not data collection: gui:=true (the
entire point is seeing it), record:=false (no rosetta/bag dependency at
all unless you ask for it). Gazebo and move_group are left running after
the sequence finishes so the final scene stays on screen -- Ctrl+C this
launch to shut everything down.

attach_mode:=simulated is declared as an argument, not buried in a report,
because it is the one thing anyone running this demo needs to know before
drawing conclusions from it: the block is carried by a scripted Gazebo
weld (DetachableJoint), not a physical friction grasp -- a bounded
experiment could not produce one (MEASUREMENTS.md §8). It is the only
value this argument currently accepts; passing anything else is a hard
error, on purpose.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def _find_run_demo(project_root):
    # This project's git layout keeps orchestration scripts under scripts/; the
    # container this was developed/tested in has always been deployed with every
    # *.py flattened directly into the project root instead (docker cp, one file at
    # a time, no subdirs). Accept either rather than hardcoding one.
    for candidate in ("scripts/run_demo.py", "run_demo.py"):
        path = os.path.join(project_root, candidate)
        if os.path.isfile(path):
            return path
    raise FileNotFoundError(
        f"run_demo.py not found under {project_root}/scripts/ or {project_root}/ -- "
        "check this launch file was copied alongside the rest of the project")


def generate_launch_description():
    here = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(here)
    run_demo_path = _find_run_demo(project_root)

    gui_arg = DeclareLaunchArgument(
        "gui", default_value="true",
        description="Show the Gazebo 3D viewport. Default true here: the point of this "
                    "launch file is watching the arm work.")
    record_arg = DeclareLaunchArgument(
        "record", default_value="false",
        description="Also record the episode via the rosetta recorder into a LeRobot-format "
                    "bag. Default false: this launch file is for watching, not data collection "
                    "-- with record:=false, no rosetta install is needed at all.")
    block_x_arg = DeclareLaunchArgument(
        "block_x", default_value="0.25",
        description="Block spawn X (m). Reachable envelope on this arm is roughly "
                    "[0.25, 0.31]; move the object and re-run to try another point.")
    block_y_arg = DeclareLaunchArgument(
        "block_y", default_value="0.00",
        description="Block spawn Y (m). Reachable envelope roughly [-0.08, 0.08].")
    attach_mode_arg = DeclareLaunchArgument(
        "attach_mode", default_value="simulated",
        description="How the block is carried. 'simulated' (the only supported value) means "
                    "a scripted Gazebo DetachableJoint weld, NOT a physical friction grasp -- "
                    "see MEASUREMENTS.md §8-9. Declared here so the limitation is visible "
                    "at the point of use, not just in a report.")

    demo = ExecuteProcess(
        cmd=["python3", run_demo_path,
             "--gui", LaunchConfiguration("gui"),
             "--record", LaunchConfiguration("record"),
             "--block-x", LaunchConfiguration("block_x"),
             "--block-y", LaunchConfiguration("block_y"),
             "--attach-mode", LaunchConfiguration("attach_mode"),
             "--htgpp-dir", project_root,
             "--out-dir", os.path.join(project_root, "demo")],
        output="screen",
        emulate_tty=True,
    )

    return LaunchDescription([
        gui_arg, record_arg, block_x_arg, block_y_arg, attach_mode_arg, demo,
    ])
