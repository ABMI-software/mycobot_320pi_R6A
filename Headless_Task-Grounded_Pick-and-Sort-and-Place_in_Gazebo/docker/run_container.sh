#!/usr/bin/env bash
# Start the recording container on the workspace built by setup_workspace.sh,
# then build it (--base-paths src: the venv under /workspace holds CMake
# projects colcon would otherwise try to build). On the HOST, Linux or WSL2 (detected):
#
#   Headless_Task-Grounded_Pick-and-Sort-and-Place_in_Gazebo/docker/run_container.sh [workspace]
#
# Containers: $CONTAINER (default gazebo_to_lerobot) on tri_sort:jazzy-harmonic,
# and rlds_builder (OpenVLA's TensorFlow 2.13 image) on the repository's
# ROS2_to_RLDS_Conversion_OpenVLA folder. After a reboot, `docker start` both.
set -e
HERE="$(cd "$(dirname "$(readlink -f "$0")")" && pwd)"
REPO="$(dirname "$(dirname "$HERE")")"
WS="${1:-$HOME/tri_sort_ws}"
C="${CONTAINER:-gazebo_to_lerobot}"
RC="${RLDS_CONTAINER:-rlds_builder}"

if docker ps -a --format '{{.Names}}' | grep -qx "$C"; then
    echo "container $C already exists: docker start $C (or docker rm -f $C for a fresh one)"; exit 1
fi
# Fast DDS uses /dev/shm; Docker's 64 MB default runs out mid-batch.
ARGS=(-d --name "$C" --shm-size=2g -v "$WS/src:/workspace/src" -v /tmp/.X11-unix:/tmp/.X11-unix)
if grep -qi microsoft /proc/version; then
    # WSLg: the forwarded host DISPLAY stalls the simulation clock; :0 does not.
    ARGS+=(-v /mnt/wslg:/mnt/wslg -v /usr/lib/wsl:/usr/lib/wsl -e TRI_DISPLAY=:0
           -e WAYLAND_DISPLAY=wayland-0 -e XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir)
else
    # Plain Linux X11: let the container's root draw on your display.
    xhost +local:root >/dev/null 2>&1 || echo "xhost not found: the Gazebo GUI may be refused (headless runs are unaffected)"
    ARGS+=(-e TRI_DISPLAY="${DISPLAY:-:0}")
fi
docker run "${ARGS[@]}" tri_sort:jazzy-harmonic

echo "building the workspace inside $C (rosdep, colcon) ..."
docker exec "$C" bash -lc 'source /opt/ros/jazzy/setup.bash && cd /workspace && apt-get update -qq >/dev/null \
    && rosdep install --from-paths src --ignore-src -r -y >/dev/null \
    && colcon build --base-paths src --symlink-install 2>&1 | tail -3'

if ! docker ps -a --format '{{.Names}}' | grep -qx "$RC"; then
    docker run -d --name "$RC" -v "$REPO/ROS2_to_RLDS_Conversion_OpenVLA:/workspace" rlds_builder:py39-tf213 >/dev/null
    echo "started $RC on $REPO/ROS2_to_RLDS_Conversion_OpenVLA"
fi
echo "ready: docker exec -it $C bash"
