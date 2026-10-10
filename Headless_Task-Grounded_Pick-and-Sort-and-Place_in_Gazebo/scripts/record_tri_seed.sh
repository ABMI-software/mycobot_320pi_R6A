#!/usr/bin/env bash
# Record one seed of Osama's tri_yolo sort: one rosetta episode per
# pick-and-place, inside the gazebo_to_lerobot container.
#
#   record_tri_seed.sh <seed> [piece[,piece...]]
#
# Without a piece list, all four are sorted and recorded. Output in
# $TRI_HOME/seed_<NNN>/ : bags/, ep_*.json, logs. Everything is stopped on exit.
set -e
SEED="$1"
ONLY="${2:-}"
[ -n "$SEED" ] || { echo "usage: $0 <seed> [piece,...]"; exit 2; }
TRI_HOME="${TRI_HOME:-/workspace/tri_sort}"
D="$TRI_HOME/seed_$(printf %03d "$SEED")"
[ -e "$D" ] && { echo "$D exists -- never overwrite a recording"; exit 2; }
mkdir -p "$D/bags"
HERE="$(dirname "$(readlink -f "$0")")"
CONTRACT=/workspace/src/contracts/mycobot_tri_sort.yaml

set +u
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
export DISPLAY=:0 LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe MESA_LOADER_DRIVER_OVERRIDE=
ros2 daemon stop >/dev/null 2>&1 || true
trap 'kill 0 2>/dev/null' EXIT
stage() { echo "[$(TZ=Europe/Paris date +%H:%M:%S)] $*" | tee -a "$D/stages.log"; }

ros2 launch mycobot_gateway tri_yolo.launch.py seed:="$SEED" piece_reach:=0.28 \
    headless:=true yolo:=false > "$D/scene.log" 2>&1 &
stage "seed $SEED: scene starting"
until ros2 control list_controllers 2>/dev/null | grep -q 'gripper_position_controller.*active'; do
    sleep 5
done
stage "controllers active"

ros2 run rosetta episode_recorder_node --ros-args -p use_sim_time:=true \
    -p contract_path:="$CONTRACT" -p bag_base_dir:="$D/bags" -p record_all:=false \
    > "$D/recorder.log" 2>&1 &
timeout 120 python3 "$HERE/recorder_lifecycle.py" --timeout 60 > "$D/lifecycle.log" 2>&1 \
    || { stage "FATAL: recorder lifecycle"; cat "$D/lifecycle.log"; exit 1; }
stage "recorder active"

python3 "$HERE/record_tri_sort.py" --seed "$SEED" --out "$D" > "$D/driver.log" 2>&1 &
DRIVER=$!

ONLY_ARGS=()
[ -n "$ONLY" ] && ONLY_ARGS=(-p only:="$ONLY")
stage "sorting ${ONLY:-all four}"
ros2 run mycobot_gateway sim_sorting_grasp --ros-args -p use_sim_time:=true \
    -p world_name:=tri_yolo -p pose_source:=ground_truth -p max_attempts:=2 \
    -p wall_timeout_scale:=15.0 -p startup_timeout:=600.0 "${ONLY_ARGS[@]}" \
    -p csv_path:="$D/tri.csv" > "$D/sort.log" 2>&1 || true

wait "$DRIVER" || true
stage "done: $(ls "$D"/ep_*.json 2>/dev/null | wc -l) episode(s)"
grep -h '"verdict"' "$D"/ep_*.json 2>/dev/null || true
