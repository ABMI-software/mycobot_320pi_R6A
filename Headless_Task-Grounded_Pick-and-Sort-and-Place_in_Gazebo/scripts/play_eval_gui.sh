#!/usr/bin/env bash
# Play back the Gazebo state log of an evaluation run (eval_tri_seed.sh RECORD=1) in a window.
#
#   play_eval_gui.sh <run dir>      # e.g. /workspace/eval/runB_step_locked/ood_s0017
#
# Nothing is simulated: Gazebo replays the recorded states (every 20 ms of sim time), so what the
# window shows is what happened in that run, every time it is played. The playback controls at
# the bottom of the window pause, step and seek. No policy, no ROS. Close the window to finish.
# Inside gazebo_to_lerobot: docker exec -it gazebo_to_lerobot bash <this script> <dir>
set -e
RUN="${1:?usage: $0 <run dir>}"
LOG="${RUN%/}/gzlog"
[ -f "$LOG/state.tlog" ] || { echo "$LOG/state.tlog not found: was the run recorded (RECORD=1)?"; exit 2; }
set +u
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
export DISPLAY="${TRI_DISPLAY:-:0}" LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe MESA_LOADER_DRIVER_OVERRIDE=
DESC=$(ros2 pkg prefix mycobot_description)/share
export GZ_SIM_RESOURCE_PATH="$DESC${GZ_SIM_RESOURCE_PATH:+:$GZ_SIM_RESOURCE_PATH}"
python3 -c "import json,sys; r=json.load(open(sys.argv[1])); \
print('seed', r['seed'], '|', ', '.join(p['piece'] + (' sorted' if p.get('placed_after_pick') else ' not sorted') \
for p in r['pieces']))" "${RUN%/}/result.json" 2>/dev/null || true
exec gz sim --playback "$LOG"
