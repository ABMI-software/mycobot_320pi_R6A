#!/usr/bin/env bash
# Part 8/10.3 -- one episode, clean restart, watchdog. Usage: episode.sh <idx>
set -u
# 1200 s watchdog. Episodes take 2.5-3.5 min, but across episodes 1-5 of the
# first consecutive run the end-of-episode RTF fell 0.41 -> 0.16 and wall time
# rose 40% (possibly thermal): a healthy but slowed episode must not be killed
# and retried as a false failure.
if [ -z "${EP_WATCHDOG:-}" ]; then EP_WATCHDOG=1 exec timeout -k 20 1200 bash "$0" "$@"; fi
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

# DISPLAY=:0 (WSLg) -- an unreachable host DISPLAY forwarded into the
# container blocks the camera sensor, and with it the whole simulation step
# (controllers never become active). See the pick-and-place report, §24.
export DISPLAY=:0
# Software OpenGL (llvmpipe). On WSL's D3D12 GPU path, every DYNAMIC object
# spawned at runtime renders WHITE in the camera, whatever its material or
# texture (static ones are fine): a colour-sorting dataset cannot be recorded
# that way. llvmpipe renders the colours correctly, and with shadows off it
# is also the faster path here: RTF 0.304 vs 0.144 (MEASUREMENTS.md section 1).
export LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe MESA_LOADER_DRIVER_OVERRIDE=
kill_stack() {
  for pat in "[r]os2 launch" "[g]z sim" "[r]un_pick_and_place" "[c]ontroller_manager/spawner" \
             "[p]arameter_bridge" "[i]mage_bridge" "[r]obot_state_publisher" "[e]pisode_recorder_node"; do
    for p in $(pgrep -f "$pat" 2>/dev/null); do kill -9 "$p" 2>/dev/null; done
  done
}
kill_stack
# Fast DDS's shared-memory transport segments outlive the processes that
# created them and this script restarts the sim stack, not the container --
# so leaked segments accumulate across the whole batch, not just within one
# episode. Matches the measured failure shape exactly (RTPS_TRANSPORT_SHM
# port-lock errors ~20 min into a Gazebo instance's uptime, on a container
# whose /dev/shm is Docker's 64 MB default). Clear them every episode.
rm -rf /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
ros2 daemon stop 2>/dev/null
sleep 2

# Per-episode scene variation (camera pose, light, table texture; distractor
# placement comes from the matrix). Written into the INSTALLED world file,
# because sim_grasp.launch.py loads worlds/<world_name>.sdf from the
# mycobot_description share. The parameters go into the episode metadata.
SHARE=$(ros2 pkg prefix mycobot_description)/share/mycobot_description
python3 scripts/make_episode_world.py --episode "$1" --out "$SHARE/worlds/sorting_table.sdf" \
  --params "$E/scene_variation.json" > "$E/world.log" 2>&1 \
  || { echo "FATAL: WORLD_GENERATION_FAILED"; cat "$E/world.log"; exit 1; }
stage "world generated: $(cat "$E/world.log")"

stage "launching sim"
# sorting_table = real_table's bench with NO objects or bins in it: the world
# that ships real_table.sdf holds a single red_cube/red_bin, so 45 of the 60
# episodes had no target to pick. spawn_scene.py places all eight per episode.
nohup ros2 launch mycobot_gateway sim_grasp.launch.py world_name:=sorting_table \
  headless:=true bridge_camera:=true > "$E/sim.log" 2>&1 &
# Train and held-out episodes both record /camera/image_raw: the held-out
# condition is a reserved CAMERA CONFIGURATION (make_episode_world.py,
# "heldout_oblique"), used by no train episode -- no second sensor or bridge.

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

python3 scripts/spawn_scene.py --episode "$1" > "$E/spawn.log" 2>&1 \
  || { echo "FATAL: SPAWN_FAILED"; tail -3 "$E/spawn.log"; exit 1; }
stage "scene spawned"

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
# Direct lifecycle service calls, waiting on the service itself: the former
# `sleep 8` + two `ros2 lifecycle set --no-daemon --spin-time 15` calls took
# 71 s per episode, ~31 s of it per CLI call, for a service that is up
# ~1 s after the recorder starts (MEASUREMENTS.md section 3).
timeout 120 python3 scripts/recorder_lifecycle.py --timeout 60 > "$E/lifecycle.log" 2>&1 \
  || { echo "FATAL: RECORDER_LIFECYCLE_FAILED"; cat "$E/lifecycle.log"; tail -5 "$E/recorder.log"; exit 1; }
stage "recorder active ($(tr '\n' ' ' < "$E/lifecycle.log"))"

stage "running sequence"
python3 scripts/run_pick_and_place.py --episode "$1" --target "$TARGET" --record \
  --variation "$E/scene_variation.json" \
  --log "$E/grasp_log.csv" --meta "$E/grasp_meta.json" > "$E/run.log" 2>&1
RC=$?
stage "sequence finished rc=$RC"

python3 scripts/verify_episode.py "$E/grasp_meta.json" > "$E/verdict.log" 2>&1
VRC=$?
stage "verify rc=$VRC"

# A recording missing camera frames is not a valid dataset episode, whatever
# the arm did: check received vs expected (30 fps over the recorded SIM span).
BAG=$(python3 -c "import json; print(json.load(open('$E/grasp_meta.json')).get('bag_path') or '')")
if [ -n "$BAG" ]; then
  python3 scripts/check_frames.py "$BAG" --out "$E/frames.json" > "$E/frames.log" 2>&1
  FRC=$?
else
  echo '{"received": 0, "expected": null, "complete": false}' > "$E/frames.json"; FRC=1
fi
stage "frames rc=$FRC $(cat "$E/frames.json" | tr -d '\n ')"

# Colour guard: the target must be visibly its own colour in the recording.
# The WSL GPU path renders dynamic objects white and the motion checks still
# say PASS; an environment variable alone is not enough protection.
if [ -n "$BAG" ]; then
  python3 scripts/check_colour.py "$BAG" "$E" --out "$E/colour.json" > "$E/colour.log" 2>&1
  CRC=$?
else
  echo '{"ok": false, "error": "no bag"}' > "$E/colour.json"; CRC=1
fi
stage "colour rc=$CRC $(cat "$E/colour.json" | tr -d '\n ')"

# One FINAL verdict, written into the verdict file itself (motion + frames +
# colour + no re-sends): port_bag.py converts on that file, so it must never
# say PASS for an episode that failed any check.
python3 scripts/finalize_verdict.py "$E" > "$E/final_verdict.log" 2>&1
VRC=$?
stage "final verdict rc=$VRC $(cat "$E/final_verdict.log" | tr -d '\n')"

tail -5 "$E/verdict.log"
kill_stack
rm -rf /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
stage "teardown done"
exit $VRC
