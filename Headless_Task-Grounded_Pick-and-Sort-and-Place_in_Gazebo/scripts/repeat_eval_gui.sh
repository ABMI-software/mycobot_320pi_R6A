#!/usr/bin/env bash
# Repeat one seed of an evaluation batch exactly as the batch ran it, with the Gazebo window open.
#
#   repeat_eval_gui.sh <original run dir>      # e.g. /workspace/eval/runB_paper/ood_s0017
#
# Reads the checkpoint, seed, start pose, action steps and pieces from <original run dir>/result.json
# and the mode from its name (id_* = PREPLACE=1, the recorded target), then does what
# /workspace/eval/runB_paper/run.sh did for that seed: the same eval_tri_seed.sh call (same
# environment: OMP 4, offline, under reap_children.py) and failure_modes.py, into
# a new <original>_gui_tryN next to it. Differences: HEADLESS=false (the window renders the scene
# for you) and no time limit (it never cut a batch seed; here it would also close the window).
# Gazebo's contact physics varies from run to run, so the outcome can differ. Once the policy is
# done, this run is shown next to the original while the window stays open; close it to finish.
# Inside gazebo_to_lerobot: docker exec -it gazebo_to_lerobot bash <this script> <dir>
set -e
ORIG="${1:?usage: $0 <original run dir>}"
ORIG="${ORIG%/}"
[ -f "$ORIG/result.json" ] || { echo "$ORIG/result.json not found"; exit 2; }
S=/workspace/src/scripts
read -r CKPT SEED START NSTEPS <<<"$(python3 -c "import json,sys; r=json.load(open(sys.argv[1])); \
print(r['checkpoint'], r['seed'], r['start_pose'], r['n_action_steps'])" "$ORIG/result.json")"
case "$(basename "$ORIG")" in
    id_*)  MODE=(env PREPLACE=1) ;;
    *)     MODE=(env) ;;
esac
n=1; while [ -e "${ORIG}_gui_try$n" ]; do n=$((n + 1)); done
OUT="${ORIG}_gui_try$n"
echo "repeating $(basename "$ORIG"): checkpoint $CKPT, seed $SEED, start $START, $NSTEPS actions per chunk"
echo "pieces in the original order: $(python3 -c "import json,sys; print(', '.join(p['piece'] for p in json.load(open(sys.argv[1]))['pieces']))" "$ORIG/result.json")"
echo "output: $OUT"

export OMP_NUM_THREADS=4 MKL_NUM_THREADS=4 HF_HUB_OFFLINE=1 TRANSFORMERS_OFFLINE=1 HEADLESS=false
"${MODE[@]}" python3 "$S/reap_children.py" bash "$S/eval_tri_seed.sh" "$CKPT" "$SEED" "$OUT" &
RUN=$!
trap 'kill -INT $RUN 2>/dev/null; wait $RUN 2>/dev/null' INT
until grep -qE "done: |FATAL" "$OUT/stages.log" 2>/dev/null || ! kill -0 $RUN 2>/dev/null; do sleep 5; done

set +u
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
/workspace/venv_lerobot/bin/python -W ignore "$S/failure_modes.py" "$OUT" \
    /workspace/tri_datasets/local/mycobot_tri_sort_B_s1001_1401 > "$OUT.modes.log" 2>&1 || true
python3 - "$ORIG" "$OUT" <<'EOF'
import json, sys
from pathlib import Path
print(f"\n{'piece':15s}{'original (' + Path(sys.argv[1]).name + ')':>36s}{'this run':>36s}")
cols = []
for d in map(Path, sys.argv[1:3]):
    res = {p['piece']: p for p in json.loads((d / 'result.json').read_text())['pieces']} if (d / 'result.json').exists() else {}
    modes = {m['piece']: m['mode'] for m in json.loads((d / 'failure_modes.json').read_text())} if (d / 'failure_modes.json').exists() else {}
    cols.append((res, modes))
for piece in cols[0][0]:
    cells = []
    for res, modes in cols:
        p = res.get(piece)
        cells.append('not run' if p is None else
                     f"{'picked' if p.get('picked') else 'not picked'}, {'placed' if p.get('placed') else 'not placed'}"
                     f" ({modes.get(piece, '?')})")
    print(f'{piece:15s}{cells[0]:>36s}{cells[1]:>36s}')
EOF
echo
if kill -0 $RUN 2>/dev/null; then
    echo "The Gazebo window stays open: close it (or Ctrl+C) to stop the scene."
    wait $RUN || true
fi
