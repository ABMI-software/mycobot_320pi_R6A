#!/usr/bin/env bash
# Evaluate a SmolVLA checkpoint on one tri_yolo seed, inside the gazebo_to_lerobot container.
#
#   eval_tri_seed.sh <checkpoint/pretrained_model> <seed> <out dir> [eval_smolvla_tri_sort.py args...]
#   PREPLACE=1 eval_tri_seed.sh ... <seed 1001-1400> ...   # the recorded target only, as recorded
#   HEADLESS=false eval_tri_seed.sh ...                     # Gazebo window, left open after the run
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

ros2 launch mycobot_gateway tri_yolo.launch.py seed:="$SEED" piece_reach:=0.28 \
    headless:="${HEADLESS:-true}" yolo:=false > "$OUT/scene.log" 2>&1 &
stage "seed $SEED: scene starting ($CKPT)"
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
    EXTRA=(--only "$TARGET" --start-pose random)
fi

/workspace/venv_lerobot/bin/python -W ignore "$HERE/eval_smolvla_tri_sort.py" \
    --checkpoint "$CKPT" --seed "$SEED" --out "$OUT" "${EXTRA[@]}" "$@" > "$OUT/eval.log" 2>&1 || true
stage "done: $(python3 -c "import json,sys; r=json.load(open(sys.argv[1]))['pieces']; \
print(sum(p.get('placed_after_pick', False) for p in r), '/', len(r), 'sorted')" "$OUT/result.json" 2>/dev/null)"

# With the GUI (HEADLESS=false), the scene stays up after the evaluation, simulation running, until
# the Gazebo window is closed or Ctrl+C: any Gazebo process ending (the window) or the launch
# ending stops everything (tri_cleanup). KEEP_GUI=0 closes it at once, as headless runs do.
if [ "${HEADLESS:-true}" = false ] && [ "${KEEP_GUI:-1}" = 1 ]; then
    stage "Gazebo window left open: close it (or Ctrl+C here) to stop the scene"
    n=$(pgrep -fc "gz sim" || true)
    while kill -0 %1 2>/dev/null && [ "$(pgrep -fc "gz sim" || true)" -ge "$n" ]; do sleep 2; done
    stage "Gazebo window closed: stopping the scene"
fi
