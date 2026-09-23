#!/usr/bin/env bash
# Part 8/10.3 -- one episode, clean restart, watchdog. Usage: episode.sh <idx>
set -u
if [ -z "${EP_WATCHDOG:-}" ]; then EP_WATCHDOG=1 exec timeout -k 20 1500 bash "$0" "$@"; fi
IDX=$(printf "%03d" "$1")
H=/workspace/htgspp
STAGE="$H/stage_$IDX.log"     # OUTSIDE the episode folder -- a retry that
                               # deletes the folder must not erase this (Part 10.3)
stage() { echo "$(date +%T) $*" >> "$STAGE"; }

E="$H/episodes/ep_$IDX"
rm -rf "$E"; mkdir -p "$E"
TARGET=$(python3 -c "
import csv
row = next(r for r in csv.DictReader(open('config/episode_matrix.csv')) if int(r['episode'])==$1)
print(row['target'])
")
[ -n "$TARGET" ] || { echo "FATAL: episode $1 not in matrix"; exit 3; }
SPLIT=$(python3 -c "
import csv
row = next(r for r in csv.DictReader(open('config/episode_matrix.csv')) if int(r['episode'])==$1)
print(row['split'])
")
CONTRACT="contracts/mycobot_sorting.yaml"
[ "$SPLIT" = "heldout" ] && CONTRACT="contracts/mycobot_sorting_heldout.yaml"

stage "attempt start, target=$TARGET"
# set -u (above) and ROS2's own setup.bash don't mix: setup.bash references
# variables (e.g. AMENT_TRACE_SETUP_FILES) that are unset in a fresh shell,
# and under nounset that's a hard error, before any of THIS script's own
# logic runs -- found live, on the first real run after a container
# restart. Suspend nounset only around the two source lines.
set +u
source /opt/ros/jazzy/setup.bash; source /workspace/install/setup.bash
set -u

for pat in "[r]os2 launch" "[g]z sim" "[r]un_pick_and_place" "[c]ontroller_manager/spawner"; do
  for p in $(pgrep -f "$pat" 2>/dev/null); do kill -9 "$p" 2>/dev/null; done
done
# Fast DDS's shared-memory transport segments outlive the processes that
# created them and this script restarts the sim stack, not the container --
# so leaked segments accumulate across the whole batch, not just within one
# episode. Matches the measured failure shape exactly (RTPS_TRANSPORT_SHM
# port-lock errors ~20 min into a Gazebo instance's uptime, on a container
# whose /dev/shm is Docker's 64 MB default). Clear them every episode.
rm -rf /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
ros2 daemon stop 2>/dev/null
sleep 2

stage "launching sim"
nohup ros2 launch mycobot_gateway real_table.launch.py headless:=true > "$E/sim.log" 2>&1 &
# Only bridge the heldout camera when THIS episode actually needs it --
# bridging + rendering a second camera unconditionally on every episode
# was avoidable extra load on a container that turned out to have very
# little CPU headroom to spare (see the record_all finding below).
if [ "$SPLIT" = "heldout" ]; then
  nohup ros2 run ros_gz_image image_bridge /synth_camera_right/image > "$E/bridge_heldout.log" 2>&1 &
fi

OKC=0
for i in $(seq 1 30); do
  out=$(timeout 15 ros2 control list_controllers 2>/dev/null)
  n_active=$(echo "$out" | awk '{print $NF}' | grep -cx active)
  [ "$n_active" -ge 3 ] && { OKC=1; break; }
  grep -q "process has died" "$E/sim.log" 2>/dev/null && stage "a bringup process died, will retry spawners"
  sleep 8
done
if [ "$OKC" != 1 ]; then
  # cold-start respawn, exact-match state check (Part 3 lesson: a
  # substring check on 'active' matches inside 'inactive' and lies)
  for c in joint_state_broadcaster mycobot_controller gripper_position_controller; do
    state=$(timeout 15 ros2 control list_controllers 2>/dev/null | awk -v n="$c" '$1==n{print $NF}')
    if [ "$state" != "active" ]; then
      timeout 200 ros2 run controller_manager spawner "$c" \
        --controller-manager /controller_manager \
        --controller-manager-timeout 150 --switch-timeout 150 >> "$E/respawn.log" 2>&1
    fi
  done
  out=$(timeout 15 ros2 control list_controllers 2>/dev/null)
  [ "$(echo "$out" | awk '{print $NF}' | grep -cx active)" -ge 3 ] && OKC=1
fi
[ "$OKC" = 1 ] || { echo "FATAL: BRINGUP_FAILED"; tail -5 "$E/respawn.log" 2>/dev/null; exit 1; }
stage "controllers ok"

# Part 10.2 -- recorder bring-up. Non-daemon lifecycle form ONLY (the bare
# `ros2 lifecycle set` form reports "node not found" for a recorder that
# has in fact started, because of a stale daemon cache -- cost the
# previous acquisition ~1h49m on a single episode). Killed and restarted
# fresh every episode, same as the rest of the stack (Part 10.3).
for p in $(pgrep -f "[e]pisode_recorder_node" 2>/dev/null); do kill -9 "$p" 2>/dev/null; done
sleep 1
mkdir -p "$H/bags"
# record_all:=false -- found live, on the very first real recording attempt:
# the default (record_all=true, "like ros2 bag record -a") auto-discovered
# 28 topics on this graph, including high-rate controller introspection
# topics the contract never asked for. Recording all of them pushed this
# already gz-sim-constrained container's CPU past what it could sustain --
# measured real-time factor collapsed to ~0.0006 (a stall, not a slowdown)
# with the recorder itself at 40% CPU on top of gz sim's 147%. Limiting to
# the contract's own two declared topics (observation.images.*, the state/
# action channel) fixes both the load AND, as a side effect, the topic-
# discovery race noted at ep_001 (/camera/image_raw missing from the initial
# auto-discovery snapshot): a direct, named subscription to a contract
# topic does not depend on winning a one-time graph-scan race the way an
# open-ended record_all discovery does.
nohup ros2 run rosetta episode_recorder_node --ros-args \
  -p contract_path:="$CONTRACT" -p bag_base_dir:="$H/bags" -p record_all:=false \
  > "$E/recorder.log" 2>&1 &
sleep 8
CONFIGURED=0
for t in 1 2; do
  if timeout 90 ros2 lifecycle set --no-daemon --spin-time 15 /episode_recorder configure \
      > "$E/lifecycle.log" 2>&1; then CONFIGURED=1; break; fi
  stage "recorder configure attempt $t failed"; sleep 10
done
[ "$CONFIGURED" = 1 ] || { echo "FATAL: RECORDER_CONFIGURE_FAILED"; tail -5 "$E/recorder.log"; exit 1; }
stage "recorder configured"
timeout 90 ros2 lifecycle set --no-daemon --spin-time 15 /episode_recorder activate \
  >> "$E/lifecycle.log" 2>&1 || { echo "FATAL: RECORDER_ACTIVATE_FAILED"; tail -5 "$E/recorder.log"; exit 1; }
stage "recorder active"

stage "running sequence"
python3 scripts/run_pick_and_place.py --episode "$1" --target "$TARGET" --record \
  --log "$E/grasp_log.csv" --meta "$E/grasp_meta.json" > "$E/run.log" 2>&1
RC=$?
stage "sequence finished rc=$RC"

python3 scripts/verify_episode.py "$E/grasp_meta.json" > "$E/verdict.log" 2>&1
VRC=$?
stage "verify rc=$VRC"

tail -5 "$E/verdict.log"
exit $VRC
