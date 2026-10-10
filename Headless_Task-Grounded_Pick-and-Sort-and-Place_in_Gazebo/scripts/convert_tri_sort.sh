#!/usr/bin/env bash
# Convert the recorded tri_yolo episodes whose verdict is OK, inside gazebo_to_lerobot:
#   1. LeRobot dataset with rosetta_port (official LeRobot writer, venv_lerobot)
#   2. per-episode .npy for the RLDS builder (extract_tri_sort.py, MoveIt FK)
#
#   convert_tri_sort.sh [repo_id]          # default local/mycobot_tri_sort
#
# rosetta_port finds bags with rglob, which does not follow symlinks, so the OK
# bags are gathered with hard links (cp -al): no data is copied.
# The RLDS build itself runs in rlds_builder (see build_rlds_tri_sort_host.sh).
set -e
REPO_ID="${1:-local/mycobot_tri_sort}"
TRI_HOME="${TRI_HOME:-/workspace/tri_sort}"
STAGE=/workspace/tri_sort_ok
ROOT=/workspace/tri_datasets
NPY=/workspace/rlds_tri_sort/data/train
HERE="$(dirname "$(readlink -f "$0")")"
set +u
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash

[ -e "$ROOT/$REPO_ID" ] && { echo "$ROOT/$REPO_ID exists -- never overwrite a dataset"; exit 2; }
rm -rf "$STAGE" "$NPY"
mkdir -p "$STAGE" "$NPY"
n=0
for meta in "$TRI_HOME"/seed_[0-9][0-9][0-9]/ep_*.json; do
    ok=$(python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['ok'])" "$meta")
    if [ "$ok" != True ]; then echo "skipped (verdict not OK): $meta"; continue; fi
    bag=$(python3 -c "import json,sys; print(json.load(open(sys.argv[1]))['bag_path'])" "$meta")
    seed=$(basename "$(dirname "$meta")")
    cp -al "$bag" "$STAGE/${seed}_$(basename "$meta" .json)"
    n=$((n + 1))
done
echo "$n OK episodes staged in $STAGE"

/workspace/venv_lerobot/bin/python -W ignore -m rosetta.robots.ros2.offline.port \
    --raw-dir "$STAGE" --contract /workspace/src/contracts/mycobot_tri_sort.yaml \
    --repo-id "$REPO_ID" --root "$ROOT" 2>&1 | grep -E "frames from|Completed|rror"
# rosetta_port keeps no source name: record which bag became which episode (it
# processes the bags sorted by path), for replay_gui.sh <seed> <piece>.
python3 - "$STAGE" "$ROOT/$REPO_ID/meta/tri_sort_sources.json" <<'PY2'
import json, sys
from pathlib import Path
names = sorted(p.parent.name for p in Path(sys.argv[1]).rglob('metadata.yaml'))
Path(sys.argv[2]).write_text(json.dumps([{'episode_index': i, 'source': n, 'seed': int(n[5:8]),
    'piece': n.split('_ep_')[1].rsplit('_a', 1)[0]} for i, n in enumerate(names)], indent=1) + '\n')
print(f'{len(names)} episode sources -> {sys.argv[2]}')
PY2
/workspace/venv_lerobot/bin/python -W ignore "$HERE/check_lerobot_dataset.py" \
    "$ROOT/$REPO_ID" "$REPO_ID" "$ROOT/$(basename "$REPO_ID")_sheet.png" 2>&1 | grep -v "^Svt\|INFO"

python3 "$HERE/rlds/extract_tri_sort.py" --episodes "$TRI_HOME/seed_[0-9][0-9][0-9]/ep_*.json" --out "$NPY" 2>&1 \
    | grep -E "steps|skipped|rror"
echo "RLDS episodes: $(ls "$NPY" | wc -l) in $NPY"
