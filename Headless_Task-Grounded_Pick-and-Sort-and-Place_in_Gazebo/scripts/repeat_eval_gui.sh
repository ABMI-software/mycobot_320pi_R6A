#!/usr/bin/env bash
# Repeat one seed of an evaluation batch as the batch ran it, with a Gazebo window to watch it.
#
#   repeat_eval_gui.sh <original run dir> [piece]   # e.g. /workspace/eval/runB_step_locked/ood_s0017 pave_jaune
#
# Reads the checkpoint, seed, start pose, action steps, pieces and step-locked flag from
# <original run dir>/result.json, PREPLACE from an id_* name, and runs the same eval_tri_seed.sh
# call into a new <original>_gui_tryN next to it, with VIEWER=1: the same headless server as the
# batch (cameras and physics), plus a window that only displays it. For a step-locked original
# (eval_smolvla_tri_sort.py --step-locked) the repeat is step-locked too, every piece starts from
# the original's own snapshot of that piece (RESET_FROM), so one piece can be recomputed alone
# (second argument), and compare_runs.py says, piece by piece, whether it is identical to the
# original or where it first parts. For a
# free-running original (the 27/10 batch) the outcome can differ: Gazebo then runs on wall-clock
# timing (step_lock.py). Once the policy is done, the comparison is shown while the window stays
# open; close the window (or Ctrl+C) to finish. No time limit: it would close the window.
# Inside gazebo_to_lerobot: docker exec -it gazebo_to_lerobot bash <this script> <dir>
set -e
ORIG="${1:?usage: $0 <original run dir>}"
ORIG="${ORIG%/}"
PIECE="${2:-}"
[ -f "$ORIG/result.json" ] || { echo "$ORIG/result.json not found"; exit 2; }
S=/workspace/src/scripts
read -r CKPT SEED START NSTEPS LOCKED <<<"$(python3 -c "import json,sys; r=json.load(open(sys.argv[1])); \
print(r['checkpoint'], r['seed'], r['start_pose'], r['n_action_steps'], int(r.get('step_locked', False)))" "$ORIG/result.json")"
MODE=(env STEP_LOCKED="$LOCKED")
[ "$LOCKED" = 1 ] && MODE+=(RESET_FROM="$ORIG")
ONLY=()
[ -n "$PIECE" ] && ONLY=(--only "$PIECE")
case "$(basename "$ORIG")" in id_*) MODE+=(PREPLACE=1) ;; esac
n=1; while [ -e "${ORIG}_gui_try$n" ]; do n=$((n + 1)); done
OUT="${ORIG}_gui_try$n"
echo "repeating $(basename "$ORIG"): checkpoint $CKPT, seed $SEED, start $START, $NSTEPS actions per chunk, step-locked $LOCKED"
echo "pieces in the original order: $(python3 -c "import json,sys; print(', '.join(p['piece'] for p in json.load(open(sys.argv[1]))['pieces']))" "$ORIG/result.json")"
[ "$LOCKED" = 1 ] || echo "free-running original: this repeat can end differently (see step_lock.py)"
echo "output: $OUT"

export OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1 VIEWER=1
"${MODE[@]}" python3 "$S/reap_children.py" bash "$S/eval_tri_seed.sh" "$CKPT" "$SEED" "$OUT" "${ONLY[@]}" &
RUN=$!
trap 'kill -INT $RUN 2>/dev/null; wait $RUN 2>/dev/null' INT
until grep -qE "done: |FATAL" "$OUT/stages.log" 2>/dev/null || ! kill -0 $RUN 2>/dev/null; do sleep 5; done

set +u
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
/workspace/venv_lerobot/bin/python -W ignore "$S/failure_modes.py" "$OUT" \
    /workspace/tri_datasets/local/mycobot_tri_sort_B_s1001_1401 > "$OUT.modes.log" 2>&1 || true
echo
/workspace/venv_lerobot/bin/python -I "$S/compare_runs.py" "$ORIG" "$OUT" --json "$OUT/compare.json" || true
echo
if kill -0 $RUN 2>/dev/null; then
    echo "The Gazebo window stays open: close it (or Ctrl+C) to stop the scene."
    wait $RUN || true
fi
