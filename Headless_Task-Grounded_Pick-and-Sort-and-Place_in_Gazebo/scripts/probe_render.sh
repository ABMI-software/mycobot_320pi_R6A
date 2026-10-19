#!/usr/bin/env bash
# Render the encoder-probe layouts <first>..<last> (config/probe_protocol.yaml) in one
# tri_yolo scene, inside gazebo_to_lerobot. Resumes: seeds already in <out dir> are skipped.
#
#   probe_render.sh <first> <last> [out dir, default /workspace/probe/render]
set -e
FIRST="$1"; LAST="$2"; OUT="${3:-/workspace/probe/render}"
[ -n "$LAST" ] || { echo "usage: $0 <first> <last> [out dir]"; exit 2; }
mkdir -p "$OUT"
HERE="$(dirname "$(readlink -f "$0")")"

set +u
source /opt/ros/jazzy/setup.bash
source /workspace/install/setup.bash
export DISPLAY="${TRI_DISPLAY:-:0}" LIBGL_ALWAYS_SOFTWARE=1 GALLIUM_DRIVER=llvmpipe MESA_LOADER_DRIVER_OVERRIDE=
ros2 daemon stop >/dev/null 2>&1 || true
source "$HERE/tri_cleanup.sh"
stage() { echo "[$(TZ=Europe/Paris date +%H:%M:%S)] $*" | tee -a "$OUT/stages.log"; }

# The scene's own layout is replaced by the first set_pose_vector; its seed is outside 5001-5800.
ros2 launch mycobot_gateway tri_yolo.launch.py seed:=5000 piece_reach:=0.28 \
    headless:=true yolo:=false > "$OUT/scene.log" 2>&1 &
stage "probe seeds $FIRST-$LAST: scene starting"
until ros2 control list_controllers 2>/dev/null \
        | grep -qE 'gripper_position_controller.*[[:space:]]active([[:space:]]|$)'; do
    sleep 5
done
stage "controllers active"
/workspace/venv_lerobot/bin/python -W ignore "$HERE/probe_render.py" \
    --first "$FIRST" --last "$LAST" --out "$OUT" >> "$OUT/render.log" 2>&1 || true
stage "done: $(ls "$OUT"/*.npz 2>/dev/null | wc -l) layouts in $OUT"
