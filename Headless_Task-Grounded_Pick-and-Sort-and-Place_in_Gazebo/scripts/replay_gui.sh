#!/usr/bin/env bash
# Replay a recorded episode in Osama's tri_yolo scene, Gazebo GUI open, from the
# dataset's actions alone (replay_episode.py). Inside the container:
#
#   replay_gui.sh lerobot [seed] [piece]   # reads the LeRobot dataset with LeRobotDataset
#   replay_gui.sh rlds    [seed] [piece]   # reads /workspace/replay/rlds_seed<N>_<piece>.npz,
#                                          # exported by tfds (replay_rlds_gui_host.sh does it)
#
# Defaults: seed 1, cube_rouge. The scene is the seed's fresh layout, so a piece
# sorted later in the original run starts with the earlier pieces still on the table.
# HEADLESS=1 runs without the GUI; DATASET_ROOT / REPO_ID choose another LeRobot dataset.
# The GUI stays open on the final scene; Ctrl+C stops everything.
set -e
SOURCE="${1:?usage: $0 lerobot|rlds [seed] [piece]}"
SEED="${2:-1}"
PIECE="${3:-cube_rouge}"
REPO_ID="${REPO_ID:-local/mycobot_tri_sort}"
DATASET_ROOT="${DATASET_ROOT:-/workspace/tri_datasets/$REPO_ID}"
NPZ=/workspace/replay/${SOURCE}_seed${SEED}_${PIECE}.npz
LOG=/workspace/replay/${SOURCE}_seed${SEED}_${PIECE}_$(date +%H%M%S)
mkdir -p "$LOG"
HERE="$(dirname "$(readlink -f "$0")")"

set +u
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
# TRI_DISPLAY: the X display (":0" under WSLg, set by run_container.sh); software
# OpenGL renders runtime-spawned objects in colour everywhere.
export DISPLAY="${TRI_DISPLAY:-:0}" LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe MESA_LOADER_DRIVER_OVERRIDE=

case "$SOURCE" in
  lerobot) /workspace/venv_lerobot/bin/python -W ignore "$HERE/export_lerobot_episode.py" \
             "$DATASET_ROOT" "$REPO_ID" "$SEED" "$PIECE" "$NPZ" 2>&1 | grep -v '^Svt\|INFO' ;;
  rlds)    [ -f "$NPZ" ] || { echo "$NPZ missing -- run replay_rlds_gui_host.sh from the host"; exit 2; } ;;
  *)       echo "usage: $0 lerobot|rlds [seed] [piece]"; exit 2 ;;
esac

ros2 daemon stop >/dev/null 2>&1 || true
source "$(dirname "$(readlink -f "$0")")/tri_cleanup.sh"
GUI=true; [ "${HEADLESS:-0}" = 1 ] && GUI=false
ros2 launch mycobot_gateway tri_yolo.launch.py seed:="$SEED" piece_reach:=0.28 \
    headless:=$([ $GUI = true ] && echo false || echo true) yolo:=false panel:=false \
    randomize_panel:=false > "$LOG/scene.log" 2>&1 &
echo "seed $SEED scene starting ($SOURCE replay of $PIECE), logs in $LOG"
until ros2 control list_controllers 2>/dev/null | grep -q 'gripper_position_controller.*active'; do
    sleep 5
done
# tri_yolo moves the arm to its observation pose 12 s after launch, as in the
# original run; wait for that before taking over.
until ros2 topic echo --once /validation/gt/objects >/dev/null 2>&1; do sleep 2; done
sleep 20
echo "controllers active -- replaying (slow: software rendering)"
python3 "$HERE/replay_episode.py" "$NPZ" --out "$LOG/result.json" 2>&1 \
    | grep --line-buffered -vE '^\[(INFO|WARN|ERROR)\]|class_loader|at line' | tee "$LOG/replay.log" || true
[ $GUI = true ] && { echo "done -- the GUI stays open on the final scene; Ctrl+C to quit"; wait; }
