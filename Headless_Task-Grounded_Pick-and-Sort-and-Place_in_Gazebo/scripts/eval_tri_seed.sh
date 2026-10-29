#!/usr/bin/env bash
# Evaluate a SmolVLA checkpoint on one tri_yolo seed, inside the gazebo_to_lerobot container.
#
#   eval_tri_seed.sh <checkpoint/pretrained_model> <seed> <out dir> [eval_smolvla_tri_sort.py args...]
#   PREPLACE=1 eval_tri_seed.sh ... <seed 1001-1400> ...   # the recorded target only, as recorded
#   HEADLESS=false eval_tri_seed.sh ...                     # Gazebo window, left open after the run
#   STEP_LOCKED=1 eval_tri_seed.sh ...                      # step_lock.py: reproducible to the bit
#   STEP_LOCKED=1 RESET_FROM=<run dir> eval_tri_seed.sh ...  # each piece from that run's snapshot
#   VIEWER=1 eval_tri_seed.sh ...                           # headless server + a view-only window
#   RECORD=1 eval_tri_seed.sh ...                           # Gazebo state log in <out>/gzlog
#
# VIEWER=1 is how to watch a run that must match a headless one: the cameras and the physics are
# the headless server's, the window (gz sim -g) only displays them. HEADLESS=false renders the
# cameras through the window's own process, which can change their pixels. RECORD=1 keeps the
# simulation state every 20 ms of sim time; play_eval_gui.sh replays it without recomputing.
#
# Same scene as record_tri_seed.sh (tri_yolo.launch.py, piece_reach 0.28, headless, no YOLO),
# without the recorder. <out dir>/ gets result.json, trace_*.jsonl and the logs. Everything is
# stopped on exit.
set -e
CKPT="$1"; SEED="$2"; OUT="$3"; shift 3 || true
[ -n "$OUT" ] || { echo "usage: $0 <checkpoint> <seed> <out dir> [args...]"; exit 2; }
[ -e "$OUT/result.json" ] && { echo "$OUT/result.json exists -- never overwrite a result"; exit 2; }
mkdir -p "$OUT"
HERE="$(dirname "$(readlink -f "$0")")"

set +u
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
export DISPLAY="${TRI_DISPLAY:-:0}" LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe MESA_LOADER_DRIVER_OVERRIDE=
export HF_HOME="${HF_HOME:-/root/.cache/huggingface}"
ros2 daemon stop >/dev/null 2>&1 || true
source "$HERE/tri_cleanup.sh"
stage() { echo "[$(TZ=Europe/Paris date +%H:%M:%S)] $*" | tee -a "$OUT/stages.log"; }

SCENE=(headless:="${HEADLESS:-true}")
[ "${VIEWER:-0}" = 1 ] && SCENE=(headless:=true gui_client:=true)
[ "${RECORD:-0}" = 1 ] && SCENE+=("gz_extra_args:=--record-path $OUT/gzlog --record-period 0.02")
# Step-locked: the server starts paused with the state_reset system, no wall-timed observation
# move; the same thread count for the policy in every run (batch or window).
if [ "${STEP_LOCKED:-0}" = 1 ]; then
    SCENE+=(run_on_start:=false state_reset:=true observe:=false)
    export GZ_SIM_SYSTEM_PLUGIN_PATH="$HERE/gz_systems${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
    export OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 STEP_LOCK_DIR="$OUT"
fi
ros2 launch mycobot_gateway tri_yolo.launch.py seed:="$SEED" piece_reach:=0.28 \
    "${SCENE[@]}" yolo:=false > "$OUT/scene.log" 2>&1 &
stage "seed $SEED: scene starting ($CKPT)"
# Step-locked: the robot is spawned at simulation time 0 (controller loop and cameras on one time
# grid, step_lock.py); then the simulation only advances in 100-step blocks until the evaluator
# locks it (it touches lock_requested, this stepper answers stepper_stopped).
if [ "${STEP_LOCKED:-0}" = 1 ]; then
    until gz model --list 2>/dev/null | grep -q mycobot_320; do sleep 1; done
    stage "robot spawned at sim time 0; stepping in 100-step blocks until lock"
    # multi_step requests add up: the next block is sent only once /stats shows the previous one
    # done, so nothing is left queued when the evaluator locks. A request can time out unanswered
    # while the policy loads (29/10): if the iterations have not moved after 60 s, the block was
    # not queued and is sent again; the last block always finishes before stepper_stopped.
    iterations() { gz topic -e -n 1 -t /world/tri_yolo/stats 2>/dev/null | awk '/^iterations:/ {print $2}'; }
    ( while [ ! -e "$OUT/lock_requested" ]; do
          start=$(iterations); start=${start:-0}
          gz service -s /world/tri_yolo/control --reqtype gz.msgs.WorldControl --reptype gz.msgs.Boolean \
              --timeout 5000 --req 'multi_step: 100, pause: true' > /dev/null || true
          waited=0
          until now=$(iterations); [ "${now:-0}" -ge $((start + 100)) ]; do
              sleep 0.2; waited=$((waited + 1))
              if [ $waited -ge 300 ] && [ "${now:-0}" = "$start" ]; then break; fi
          done
      done
      touch "$OUT/stepper_stopped" ) &
fi
# "inactive" contains "active": match the state as a whole word.
until ros2 control list_controllers 2>/dev/null \
        | grep -qE 'gripper_position_controller.*[[:space:]]active([[:space:]]|$)'; do
    sleep 5
done
stage "controllers active"

# PREPLACE=1: replay a one-episode-per-layout recording (record_tri_seed.sh PREPLACE=1 RANDOM_START=1).
# preplace_pieces.py is deterministic in the seed, so it puts the same 0-3 pieces in their bins and
# names the same target; the placement is checked against the recording's preplaced.json
# (PREPLACE_REF, <dir>/seed_<seed>/preplaced.json), and only the target is evaluated, from the
# random start the recording drew first (draw 0 of default_rng(seed), --start-pose random).
EXTRA=()
[ "${STEP_LOCKED:-0}" = 1 ] && EXTRA+=(--step-locked)
[ -n "${RESET_FROM:-}" ] && EXTRA+=(--reset-from "$RESET_FROM")
if [ "${PREPLACE:-0}" = 1 ]; then
    TARGET=$(python3 "$HERE/preplace_pieces.py" --seed "$SEED" --out "$OUT" 2>> "$OUT/preplace.log" | tail -1) \
        || { stage "FATAL: preplace failed: $(tail -1 "$OUT/preplace.log")"; exit 1; }
    python3 - "$OUT/preplaced.json" "${PREPLACE_REF:-/workspace/tri_sort_B}/seed_$SEED/preplaced.json" \
            >> "$OUT/stages.log" <<'EOF' || { stage "FATAL: preplacement differs from the recording"; exit 1; }
import json, sys
a, b = (json.load(open(p)) for p in sys.argv[1:3])
d = max(max(abs(x - y) for x, y in zip(a['pieces_after'][k], b['pieces_after'][k])) for k in b['pieces_after'])
ok = a['target'] == b['target'] and sorted(a['presorted']) == sorted(b['presorted']) and d < 0.002
print(f"preplacement vs recording: target {a['target']}, presorted {sorted(a['presorted'])}, max diff {d * 1000:.1f} mm, {'OK' if ok else 'DIFFERENT'}")
sys.exit(0 if ok else 1)
EOF
    stage "preplaced like the recording; target $TARGET"
    EXTRA+=(--only "$TARGET" --start-pose random)
fi

/workspace/venv_lerobot/bin/python -W ignore "$HERE/eval_smolvla_tri_sort.py" \
    --checkpoint "$CKPT" --seed "$SEED" --out "$OUT" "${EXTRA[@]}" "$@" > "$OUT/eval.log" 2>&1 || true
stage "done: $(python3 -c "import json,sys; r=json.load(open(sys.argv[1]))['pieces']; \
print(sum(p.get('placed_after_pick', False) for p in r), '/', len(r), 'sorted')" "$OUT/result.json" 2>/dev/null)"

# With the GUI (HEADLESS=false), the scene stays up after the evaluation, simulation running, until
# the Gazebo window is closed or Ctrl+C: any Gazebo process ending (the window) or the launch
# ending stops everything (tri_cleanup). KEEP_GUI=0 closes it at once, as headless runs do.
if { [ "${HEADLESS:-true}" = false ] || [ "${VIEWER:-0}" = 1 ]; } && [ "${KEEP_GUI:-1}" = 1 ]; then
    stage "Gazebo window left open: close it (or Ctrl+C here) to stop the scene"
    n=$(pgrep -fc "gz sim" || true)
    while kill -0 %1 2>/dev/null && [ "$(pgrep -fc "gz sim" || true)" -ge "$n" ]; do sleep 2; done
    stage "Gazebo window closed: stopping the scene"
fi
