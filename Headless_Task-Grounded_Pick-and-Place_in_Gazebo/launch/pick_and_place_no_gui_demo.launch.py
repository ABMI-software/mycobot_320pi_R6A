"""pick_and_place_no_gui_demo.launch.py -- the same single-entry-point demo
as pick_and_place_demo.launch.py, but headless and exiting with a pass/fail
code instead of staying open -- for a laptop that cannot run Gazebo's 3D
viewport usefully (see MEASUREMENTS.md §1) and for automated re-runs.

    ros2 launch pick_and_place_no_gui_demo.launch.py
    ros2 launch pick_and_place_no_gui_demo.launch.py block_x:=0.28 block_y:=0.05
    echo $?   # 0 = placed on the plate and the grasp held, 1 = it didn't

`gui` is intentionally not exposed here: this launch file's entire reason
to exist is running without the viewport. Use pick_and_place_demo.launch.py
(gui:=false is also valid there) if you want the same headless run but with
the standard gui argument present for symmetry/documentation.

See pick_and_place_demo.launch.py's own docstring for what each argument
means -- record, block_x, block_y and attach_mode behave identically here.
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

    record_arg = DeclareLaunchArgument(
        "record", default_value="false",
        description="Also record the episode via the rosetta recorder. Default false.")
    block_x_arg = DeclareLaunchArgument(
        "block_x", default_value="0.25",
        description="Block spawn X (m). Reachable envelope roughly [0.25, 0.31].")
    block_y_arg = DeclareLaunchArgument(
        "block_y", default_value="0.00",
        description="Block spawn Y (m). Reachable envelope roughly [-0.08, 0.08].")
    attach_mode_arg = DeclareLaunchArgument(
        "attach_mode", default_value="simulated",
        description="How the block is carried. 'simulated' (the only supported value) means "
                    "a scripted Gazebo DetachableJoint weld, NOT a physical friction grasp -- "
                    "see MEASUREMENTS.md §8-9.")

    demo = ExecuteProcess(
        cmd=["python3", run_demo_path,
             "--gui", "false",
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
        record_arg, block_x_arg, block_y_arg, attach_mode_arg, demo,
    ])
