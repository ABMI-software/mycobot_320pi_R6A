#!/usr/bin/env bash
# Four-object sort in Osama's tri_yolo scene, with the Gazebo GUI, inside the
# gazebo_to_lerobot container. Positions come from Gazebo ground truth
# (pose_source:=ground_truth), not YOLO: the container has no torch/ultralytics.
#
# From the WSL2 host:
#   docker exec -it gazebo_to_lerobot /workspace/src/scripts/run_sort_gui.sh [seed]
#
# The GUI stays open on the final scene; Ctrl+C stops everything.
set -e
SEED="${1:-1}"
LOG=/workspace/sort_smoke/gui_seed${SEED}_$(date +%H%M%S)
mkdir -p "$LOG"

set +u
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
# WSLg display and software OpenGL: the host's forwarded DISPLAY stalls the sim
# clock, and the GPU path renders runtime-spawned objects white.
export DISPLAY=:0 LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe MESA_LOADER_DRIVER_OVERRIDE=
ros2 daemon stop >/dev/null 2>&1 || true

trap 'kill 0 2>/dev/null' EXIT
ros2 launch mycobot_gateway tri_yolo.launch.py seed:="$SEED" piece_reach:=0.28 \
    headless:=false yolo:=false panel:=false randomize_panel:=false > "$LOG/scene.log" 2>&1 &

echo "seed $SEED -- Gazebo starting, logs in $LOG"
until ros2 control list_controllers 2>/dev/null | grep -q 'gripper_position_controller.*active'; do
    sleep 5
done
echo "controllers active -- sorting (slow: software rendering, ~0.1x real time)"

ros2 run mycobot_gateway sim_sorting_grasp --ros-args -p use_sim_time:=true \
    -p world_name:=tri_yolo -p pose_source:=ground_truth -p max_attempts:=2 \
    -p wall_timeout_scale:=15.0 -p startup_timeout:=600.0 \
    -p csv_path:="$LOG/tri.csv" 2>&1 | tee "$LOG/sort.log" | grep --line-buffered -E '▶|essai|RESULTAT|✔|✘' || true

echo "done -- the GUI stays open on the final scene; Ctrl+C to quit"
wait
