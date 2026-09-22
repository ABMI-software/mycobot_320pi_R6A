#!/bin/bash
# Usage: episode.sh <idx> <block_x> <block_y> <front|heldout>
# One episode from a clean sim restart. Exit 0 only if the sequence passed
# (all motions ok, block carried, block on plate). SIMULATED ATTACHMENT mode.
if [ -z "$EP_WATCHDOG" ]; then EP_WATCHDOG=1 exec timeout -k 20 1500 bash "$0" "$@"; fi
IDX=$(printf "%03d" "$1"); BX=$2; BY=$3; CAM=$4
STAGE=/workspace/htgpp/stage_$IDX.log
stage() { echo "$(date +%T) $*" >> $STAGE; }
stage "attempt start"
source /opt/ros/jazzy/setup.bash; source /workspace/install/setup.bash
H=/workspace/htgpp; E=$H/episodes/ep_$IDX; rm -rf "$E"; mkdir -p "$E"
CONTRACT=$H/contracts/mycobot_pick_place.yaml
[ "$CAM" = heldout ] && CONTRACT=$H/contracts/mycobot_pick_place_heldout.yaml
WP=$H/waypoints/waypoints_x${BX}_y${BY}.json
[ -f "$WP" ] || { echo "FATAL: missing $WP"; exit 3; }

for pat in "[r]osetta.*episode_recorder_node" "[l]ib/rosetta/episode_recorder_node" "[l]ib/moveit_ros_move_group/move_group" "[b]in/ros2 launch" "[g]z sim" "[r]uby .*gz" "[p]arameter_bridge" "[i]mage_bridge" "[r]obot_state_publisher"; do
  for p in $(pgrep -f "$pat"); do kill $p 2>/dev/null; done
done
sleep 8; for p in $(pgrep -f "[g]z sim"); do kill -9 $p 2>/dev/null; done; sleep 2

stage "launching sim"
nohup ros2 launch gazebo_to_lerobot_bringup sim_full.launch.py > $E/sim.log 2>&1 &
[ "$CAM" = heldout ] && nohup ros2 run ros_gz_image image_bridge /synth_camera_right/image > $E/bridge_right.log 2>&1 &
OKC=0
for i in $(seq 1 60); do
  out=$(timeout 25 ros2 control list_controllers 2>&1)
  if echo "$out" | grep -q "mycobot_controller.*active" && echo "$out" | grep -q "gripper_position_controller.*active" && echo "$out" | grep -q "joint_state_broadcaster.*active"; then OKC=1; break; fi
  grep -q "spawner.*process has died" $E/sim.log && break
  sleep 5
done
if [ "$OKC" != 1 ]; then
  # cold-start: the 3 parallel spawners can time out against a busy controller_manager; respawn the missing ones one at a time
  PF=/workspace/install/mycobot_description/share/mycobot_description/config/controller.yaml
  for c in joint_state_broadcaster mycobot_controller gripper_position_controller; do
    if ! timeout 40 ros2 control list_controllers 2>/dev/null | grep -q "$c.*active"; then
      A=""; [ "$c" != joint_state_broadcaster ] && A="--param-file $PF"
      timeout 240 ros2 run controller_manager spawner $c --controller-manager /controller_manager --controller-manager-timeout 150 --switch-timeout 150 $A >> $E/respawn.log 2>&1
    fi
  done
  out=$(timeout 40 ros2 control list_controllers 2>&1)
  if echo "$out" | grep -q "mycobot_controller.*active" && echo "$out" | grep -q "gripper_position_controller.*active" && echo "$out" | grep -q "joint_state_broadcaster.*active"; then OKC=1; fi
fi
[ "$OKC" = 1 ] || { echo "FATAL: BRINGUP_FAILED"; tail -3 $E/respawn.log 2>/dev/null; exit 1; }

stage "controllers ok, spawning models"
timeout 90 ros2 run ros_gz_sim create -world empty -file $H/models/plate.sdf -name plate -x 0.20 -y -0.15 -z 0.044 >/dev/null 2>&1
timeout 90 ros2 run ros_gz_sim create -world empty -file $H/models/red_block.sdf -name red_block -x $BX -y $BY -z 0.0205 >/dev/null 2>&1
sleep 20
POSE=""
for t in 1 2 3 4 5; do
  POSE=$(timeout 25 gz model -m red_block -p 2>&1 | grep -A1 Pose | tail -1)
  [ -n "$POSE" ] && break
  stage "block pose query $t empty"; sleep 10
done
echo "block settled: $POSE"
python3 - "$POSE" "$BX" "$BY" <<PY || { echo "FATAL: BLOCK_NOT_SETTLED"; exit 1; }
import re,sys
m=re.search(r"\[\s*([-\d.]+)\s+([-\d.]+)\s+([-\d.]+)",sys.argv[1]); x,y,z=map(float,m.groups())
sys.exit(0 if abs(x-float(sys.argv[2]))<0.005 and abs(y-float(sys.argv[3]))<0.005 and abs(z-0.02)<0.003 else 1)
PY

stage "block settled, starting move_group"
nohup ros2 launch mycobot_moveit_config move_group.launch.py > $E/move_group.log 2>&1 &
for i in $(seq 1 60); do grep -q "You can start planning now" $E/move_group.log 2>/dev/null && break; sleep 3; done
nohup ros2 run rosetta episode_recorder_node --ros-args -p contract_path:=$CONTRACT -p bag_base_dir:=$E/bag > $E/recorder.log 2>&1 &
sleep 8
configured=0
for t in 1 2; do
  if timeout 90 ros2 lifecycle set --no-daemon --spin-time 15 /episode_recorder configure > $E/lifecycle.log 2>&1; then configured=1; break; fi
  stage "recorder configure attempt $t failed"; sleep 10
done
[ "$configured" = 1 ] || { echo "FATAL: RECORDER_CONFIGURE_FAILED"; tail -5 $E/recorder.log; exit 1; }
stage "recorder configured"
timeout 90 ros2 lifecycle set --no-daemon --spin-time 15 /episode_recorder activate >> $E/lifecycle.log 2>&1 || { echo "FATAL: RECORDER_ACTIVATE_FAILED"; tail -5 $E/recorder.log; exit 1; }
stage "recorder active"
timeout 25 ros2 run tf2_ros tf2_echo base gripper_base 2>&1 | grep -q Translation || { echo "FATAL: NO_TF"; exit 1; }

stage "starting sequence"
cd $H && timeout 1200 python3 run_pick_and_place.py --record --waypoints "$WP" \
  --log $E/grasp_log.csv --meta $E/grasp_meta.json --frame-dir $E > $E/run.log 2>&1
RC=$?
stage "sequence finished rc=$RC"
python3 - <<PY
import json
p="$E/grasp_meta.json"
try: m=json.load(open(p))
except Exception: m={}
m.update({"episode": $1, "block_xy_commanded": [float("$BX"), float("$BY")], "camera": "$CAM",
          "contract": "$CONTRACT", "run_exit_code": $RC})
json.dump(m, open(p,"w"), indent=2)
PY
tail -6 $E/run.log | cut -c1-160
exit $RC
