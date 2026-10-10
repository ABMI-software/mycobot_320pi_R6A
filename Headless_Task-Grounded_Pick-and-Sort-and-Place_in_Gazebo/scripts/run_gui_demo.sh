#!/usr/bin/env bash
# One-command GUI demo on the WSL2 laptop, inside the gazebo_to_lerobot container:
#   docker exec -it gazebo_to_lerobot /workspace/htgspp/scripts/run_gui_demo.sh [episode:=N]
# Extra arguments go straight to ros2 launch. Display and OpenGL settings are
# applied by scripts/run_demo.py itself (DISPLAY=:0, software OpenGL).
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
cd "$(dirname "$(readlink -f "$0")")/../launch"
exec ros2 launch pick_and_sort_and_place_demo.launch.py "$@"
