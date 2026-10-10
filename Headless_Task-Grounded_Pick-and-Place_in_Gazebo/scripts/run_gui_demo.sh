#!/usr/bin/env bash
# One-command GUI demo for a WSL2 laptop, run inside the gazebo_to_lerobot
# container:
#   docker exec -it gazebo_to_lerobot /workspace/htgpp/run_gui_demo.sh [block_x:=.. block_y:=..]
#
# - DISPLAY=:0 (WSLg). The host's DISPLAY may point to an X server that does not
#   answer; Gazebo's camera sensor then blocks on it and the sim clock never
#   advances (controllers never reach 'active').
# - Software OpenGL (llvmpipe). The D3D12 GPU path only offers GL 4.1 without
#   compute shaders, and Gazebo's viewport stays white with it.
# - The GUI camera is moved close to the work area once the GUI is up; the
#   default view (and Move to / Follow on mycobot_320) frames the whole
#   4-camera rig and leaves the arm tiny.
# Extra arguments go straight to ros2 launch.
export DISPLAY=:0
export LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe MESA_LOADER_DRIVER_OVERRIDE=
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash

frame_camera() {
    until gz service -l 2>/dev/null | grep -qx /gui/move_to/pose; do sleep 2; done
    sleep 8
    gz service -s /gui/move_to/pose --reqtype gz.msgs.GUICamera --reptype gz.msgs.Boolean \
        --timeout 5000 \
        --req "pose: {position: {x: 0.65, y: -0.55, z: 0.45}, orientation: {w: 0.37847, x: -0.24197, y: 0.10319, z: 0.88745}}" \
        >/dev/null 2>&1 && echo "[run_gui_demo] GUI camera framed on the arm"
}

cleanup() {
    kill "$framer" 2>/dev/null
    # An interrupted run can leave move_group behind; the next run then fails.
    pkill -INT -f "move_group.launch.py" 2>/dev/null
    pkill -INT -f "gz sim" 2>/dev/null
    sleep 3
    pkill -9 -f "move_group" 2>/dev/null
    pkill -9 -f "gz sim" 2>/dev/null
}

frame_camera &
framer=$!
trap cleanup EXIT

cd /workspace/htgpp/launch
ros2 launch pick_and_place_demo.launch.py "$@"
