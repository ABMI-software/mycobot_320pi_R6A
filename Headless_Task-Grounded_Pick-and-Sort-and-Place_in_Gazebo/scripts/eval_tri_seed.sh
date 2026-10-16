#!/usr/bin/env bash
# Evaluate a SmolVLA checkpoint on one tri_yolo seed, inside the gazebo_to_lerobot container.
#
#   eval_tri_seed.sh <checkpoint/pretrained_model> <seed> <out dir> [eval_smolvla_tri_sort.py args...]
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

/workspace/venv_lerobot/bin/python -W ignore "$HERE/eval_smolvla_tri_sort.py" \
    --checkpoint "$CKPT" --seed "$SEED" --out "$OUT" "$@" > "$OUT/eval.log" 2>&1 || true
stage "done: $(python3 -c "import json,sys; r=json.load(open(sys.argv[1]))['pieces']; \
print(sum(p.get('placed_after_pick', False) for p in r), '/', len(r), 'sorted')" "$OUT/result.json" 2>/dev/null)"
