#!/usr/bin/env bash
# Record Osama's tri_yolo sort for a range of seeds, one rosetta episode per
# pick-and-place (record_tri_seed.sh per seed), inside gazebo_to_lerobot.
#
#   record_tri_batch.sh <first seed> <last seed>      # e.g. 1 10: Osama's 40/40 campaign
#
# A seed already finished ($TRI_HOME/seed_NNN/COMPLETE) is skipped, so a batch
# stopped by a WSL2 restart resumes where it was by running the same command; a
# seed cut off mid-way is moved to seed_NNN.incomplete_<time> and redone.
# Each seed has a 75-minute watchdog; the simulator is torn down between seeds.
# Progress: $TRI_HOME/batch.log (Paris time).
set -u
FIRST="${1:?usage: $0 <first seed> <last seed>}"
LAST="${2:?usage: $0 <first seed> <last seed>}"
TRI_HOME="${TRI_HOME:-/workspace/tri_sort}"
export TRI_HOME
HERE="$(dirname "$(readlink -f "$0")")"
mkdir -p "$TRI_HOME"
LOG="$TRI_HOME/batch.log"
say() { echo "[$(TZ=Europe/Paris date '+%Y-%m-%d %H:%M:%S')] $*" | tee -a "$LOG"; }

teardown() {
    for p in record_tri_seed record_tri_sort sim_sorting_grasp episode_recorder_node \
             commanded_action_relay tri_yolo.launch gazebo_ground_truth tri_scene_randomizer \
             parameter_bridge robot_state_publisher; do
        pkill -INT -f "[${p:0:1}]${p:1}" 2>/dev/null
    done
    sleep 8
    pkill -9 -f "[g]z sim" 2>/dev/null
    pkill -9 -f "[r]uby.*gz" 2>/dev/null
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
    sleep 2
}

say "batch seeds $FIRST-$LAST start, container $(hostname), contract sha $(sha256sum /workspace/src/contracts/mycobot_tri_sort.yaml | cut -c1-12)"
for seed in $(seq "$FIRST" "$LAST"); do
    D="$TRI_HOME/seed_$(printf %03d "$seed")"
    if [ -e "$D/COMPLETE" ]; then
        say "seed $seed: already recorded, skipped"
        continue
    fi
    if [ -e "$D" ]; then
        mv "$D" "$D.incomplete_$(date +%Y%m%d_%H%M%S)"
        say "seed $seed: incomplete earlier run moved aside, redoing"
    fi
    teardown
    say "seed $seed: start"
    timeout -k 30 4500 "$HERE/record_tri_seed.sh" "$seed" > "$TRI_HOME/seed_$(printf %03d "$seed").out" 2>&1
    rc=$?
    teardown
    n_ok=$(grep -l '"ok": true' "$D"/ep_*.json 2>/dev/null | wc -l)
    n_all=$(ls "$D"/ep_*.json 2>/dev/null | wc -l)
    [ $rc = 124 ] && say "seed $seed: WATCHDOG (75 min) fired"
    # record_tri_seed.sh stops its own process group on exit (trap kill 0), so a
    # normal end returns 143, not 0: finished means its own "done:" stage line.
    grep -q "done:" "$D/stages.log" 2>/dev/null && touch "$D/COMPLETE"
    say "seed $seed: done rc=$rc, episodes $n_all, OK $n_ok -- $(grep -h '"piece"\|"verdict"' "$D"/ep_*.json 2>/dev/null | tr -d '\n' | sed 's/  */ /g')"
done
total_ok=$(grep -l '"ok": true' "$TRI_HOME"/seed_[0-9][0-9][0-9]/ep_*.json 2>/dev/null | wc -l)
say "batch end: $total_ok OK episodes over seeds $FIRST-$LAST"
