"""pick_and_sort_and_place_no_gui_demo.launch.py -- the same one-episode
demo as pick_and_sort_and_place_demo.launch.py, headless, exiting with a
pass/fail code instead of staying open.

    ros2 launch pick_and_sort_and_place_no_gui_demo.launch.py
    ros2 launch pick_and_sort_and_place_no_gui_demo.launch.py episode:=50
    echo $?   # 0 = PASS (lifted and placed in its own bin), non-zero = not

`gui` is not exposed: running without the viewport is this file's reason
to exist. `episode` and `attach_mode` behave as in the GUI launch file.
"""
import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration


def _exit_with(event, _context):
    # ros2 launch returns 0 after a clean Shutdown whatever its processes
    # returned (tested: run_demo.py exit 2 -> launch exit 0). run_demo.py has
    # already torn down everything it started, and this launch runs nothing
    # else, so end the launch process with the demo's own code.
    sys.stdout.flush()
    os._exit(event.returncode)


def generate_launch_description():
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    episode_arg = DeclareLaunchArgument(
        "episode", default_value="1",
        description="Row of config/episode_matrix.csv to run (1-60): 1-15 red cube, 16-30 blue "
                    "cube, 31-45 green cylinder, 46-60 yellow box.")
    attach_mode_arg = DeclareLaunchArgument(
        "attach_mode", default_value="simulated",
        description="How the target is carried. 'simulated' (the only supported value): a "
                    "scripted Gazebo DetachableJoint weld, NOT a physical grasp.")

    demo = ExecuteProcess(
        cmd=["python3", os.path.join(project_root, "scripts", "run_demo.py"),
             "--gui", "false",
             "--episode", LaunchConfiguration("episode"),
             "--attach-mode", LaunchConfiguration("attach_mode"),
             "--project-dir", project_root],
        output="screen",
        emulate_tty=True,
    )
    stop = RegisterEventHandler(OnProcessExit(target_action=demo,
                                              on_exit=_exit_with))
    return LaunchDescription([episode_arg, attach_mode_arg, demo, stop])
