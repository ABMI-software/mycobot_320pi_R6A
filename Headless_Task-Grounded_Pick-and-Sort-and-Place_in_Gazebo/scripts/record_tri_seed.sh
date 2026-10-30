#!/usr/bin/env bash
# Record one seed of Osama's tri_yolo sort: one rosetta episode per
# pick-and-place, inside the gazebo_to_lerobot container.
#
#   record_tri_seed.sh <seed> [piece[,piece...]]
#   PREPLACE=1 record_tri_seed.sh <seed>     # one piece, 0-3 others already in their bins
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
export DISPLAY="${TRI_DISPLAY:-:0}" LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe MESA_LOADER_DRIVER_OVERRIDE=
ros2 daemon stop >/dev/null 2>&1 || true
source "$(dirname "$(readlink -f "$0")")/tri_cleanup.sh"
stage() { echo "[$(TZ=Europe/Paris date +%H:%M:%S)] $*" | tee -a "$D/stages.log"; }

ros2 launch mycobot_gateway tri_yolo.launch.py seed:="$SEED" piece_reach:=0.28 \
    headless:=true yolo:=false > "$D/scene.log" 2>&1 &
stage "seed $SEED: scene starting"
# "inactive" contains "active": match the state as a whole word.
until ros2 control list_controllers 2>/dev/null \
        | grep -qE 'gripper_position_controller.*[[:space:]]active([[:space:]]|$)'; do
    # tri_scene gives up on ~1 seed in 800 (5515): no scene, no controllers, ever.
    if grep -q "could not sample a valid sorting scene" "$D/scene.log"; then
        stage "FATAL: seed $SEED unsampleable"
        exit 3
    fi
    sleep 5
done
stage "controllers active"

python3 "$HERE/commanded_action_relay.py" > "$D/relay.log" 2>&1 &
until timeout 10 ros2 topic echo --once /tri_sort/commanded_action >/dev/null 2>&1; do sleep 2; done
stage "commanded-action relay publishing"

ros2 run rosetta episode_recorder_node --ros-args -p use_sim_time:=true \
    -p contract_path:="$CONTRACT" -p bag_base_dir:="$D/bags" -p record_all:=false \
    > "$D/recorder.log" 2>&1 &
timeout 120 python3 "$HERE/recorder_lifecycle.py" --timeout 60 > "$D/lifecycle.log" 2>&1 \
    || { stage "FATAL: recorder lifecycle"; cat "$D/lifecycle.log"; exit 1; }
stage "recorder active"

# PREPLACE=1: one episode per layout. preplace_pieces.py picks the target (seed % 4), puts 0-3 of
# the other pieces in their bins (never the target) and checks they rest there; only the target
# is then sorted and recorded.
if [ "${PREPLACE:-0}" = 1 ]; then
    if ! TARGET=$(python3 "$HERE/preplace_pieces.py" --seed "$SEED" --out "$D" 2>> "$D/preplace.log" | tail -1) \
            || [ -z "$TARGET" ]; then
        stage "FATAL: preplace failed: $(tail -1 "$D/preplace.log")"
        exit 1
    fi
    ONLY="$TARGET"
    stage "preplaced $(python3 -c "import json,sys; d=json.load(open(sys.argv[1])); print(d['n_presorted'], sorted(d['presorted']))" "$D/preplaced.json"); target $TARGET"
fi

python3 "$HERE/record_tri_sort.py" --seed "$SEED" --out "$D" > "$D/driver.log" 2>&1 &
DRIVER=$!

ONLY_ARGS=()
[ -n "$ONLY" ] && ONLY_ARGS=(-p only:="$ONLY")
# RANDOM_START=1: each piece starts from a random arm pose (random_start_sorter.py), drawn with
# numpy default_rng(<seed>) and logged with the pieces' displacement in start_poses.jsonl.
SORTER=(ros2 run mycobot_gateway sim_sorting_grasp)
if [ "${RANDOM_START:-0}" = 1 ]; then
    SORTER=(python3 "$HERE/random_start_sorter.py")
    ONLY_ARGS+=(-p random_start_seed:="$SEED" -p random_start_log:="$D/start_poses.jsonl")
fi
stage "sorting ${ONLY:-all four}$([ "${RANDOM_START:-0}" = 1 ] && echo ', random starts')"
"${SORTER[@]}" --ros-args -p use_sim_time:=true \
    -p world_name:=tri_yolo -p pose_source:=ground_truth -p max_attempts:=2 \
    -p wall_timeout_scale:=15.0 -p startup_timeout:=600.0 "${ONLY_ARGS[@]}" \
    -p csv_path:="$D/tri.csv" > "$D/sort.log" 2>&1 || true

wait "$DRIVER" || true
stage "done: $(ls "$D"/ep_*.json 2>/dev/null | wc -l) episode(s)"
grep -h '"verdict"' "$D"/ep_*.json 2>/dev/null || true
