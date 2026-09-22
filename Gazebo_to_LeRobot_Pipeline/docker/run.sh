#!/usr/bin/env bash
# First-time bring-up: build the image (if needed) and start one persistent
# container with WSLg GUI/GPU passthrough. Open additional terminals with
# shell.sh, not by re-running this script.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_NAME="gazebo_to_lerobot:jazzy-harmonic"
CONTAINER_NAME="gazebo_to_lerobot"

docker build -t "$IMAGE_NAME" -f "$REPO_ROOT/docker/Dockerfile" "$REPO_ROOT/docker"

if docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
    echo "Container '$CONTAINER_NAME' already exists. Remove it first if you want a clean start:"
    echo "  docker rm -f $CONTAINER_NAME"
    exit 1
fi

docker run -d \
    --name "$CONTAINER_NAME" \
    --device=/dev/dxg \
    -v /usr/lib/wsl:/usr/lib/wsl \
    -e LD_LIBRARY_PATH=/usr/lib/wsl/lib \
    -v /tmp/.X11-unix:/tmp/.X11-unix \
    -v /mnt/wslg:/mnt/wslg \
    -e DISPLAY="${DISPLAY:-:0}" \
    -e WAYLAND_DISPLAY="${WAYLAND_DISPLAY:-wayland-0}" \
    -e XDG_RUNTIME_DIR=/mnt/wslg/runtime-dir \
    -e PULSE_SERVER="${PULSE_SERVER:-}" \
    -e MESA_LOADER_DRIVER_OVERRIDE=d3d12 \
    -e GALLIUM_DRIVER=d3d12 \
    -v "$REPO_ROOT/src:/workspace/src" \
    "$IMAGE_NAME"

echo "Container '$CONTAINER_NAME' started."
echo "Open a shell with: $REPO_ROOT/docker/shell.sh"
echo "First shell should run:"
echo "  cd /workspace && rosdep install --from-paths src --ignore-src -r -y && colcon build --symlink-install"
