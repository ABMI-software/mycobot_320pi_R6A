"""pick_and_sort_and_place_demo.launch.py -- watch one episode of the
four-object sorting task run, Gazebo GUI included.

    ros2 launch pick_and_sort_and_place_demo.launch.py
    ros2 launch pick_and_sort_and_place_demo.launch.py episode:=16

`episode` is a row of config/episode_matrix.csv (1-60). It fixes the
target (1-15 red cube, 16-30 blue cube, 31-45 green cylinder, 46-60 yellow
box), its start position, the three other objects' positions and the scene
variation (camera, light, table texture), exactly as the batch ran it.
Nothing is recorded: scripts/batch.sh is the data path. Gazebo stays open
on the final scene after the verdict; Ctrl+C this launch to shut down.

attach_mode:=simulated is declared so the one thing anyone watching must
know is visible at the point of use: the target is carried by a scripted
Gazebo weld (DetachableJoint), not a friction grasp. It is the only value
accepted; anything else is a hard error.

On this project's WSL2 laptop the viewport renders in software (llvmpipe)
and the simulation runs at roughly a third of real time (MEASUREMENTS.md
section 1). scripts/run_gui_demo.sh wraps this launch in one command.
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    episode_arg = DeclareLaunchArgument(
        "episode", default_value="1",
        description="Row of config/episode_matrix.csv to run (1-60): 1-15 red cube, 16-30 blue "
                    "cube, 31-45 green cylinder, 46-60 yellow box.")
    gui_arg = DeclareLaunchArgument(
        "gui", default_value="true",
        description="Show the Gazebo viewport. Default true: this launch file is for watching.")
    attach_mode_arg = DeclareLaunchArgument(
        "attach_mode", default_value="simulated",
        description="How the target is carried. 'simulated' (the only supported value): a "
                    "scripted Gazebo DetachableJoint weld, NOT a physical grasp.")

    demo = ExecuteProcess(
        cmd=["python3", os.path.join(project_root, "scripts", "run_demo.py"),
             "--gui", LaunchConfiguration("gui"),
             "--episode", LaunchConfiguration("episode"),
             "--attach-mode", LaunchConfiguration("attach_mode"),
             "--project-dir", project_root],
        output="screen",
        emulate_tty=True,
    )
    return LaunchDescription([episode_arg, gui_arg, attach_mode_arg, demo])
