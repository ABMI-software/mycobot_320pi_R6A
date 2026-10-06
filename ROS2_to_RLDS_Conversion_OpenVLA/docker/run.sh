#!/usr/bin/env bash
# First-time bring-up: build the image and start one persistent container.
# Open additional terminals with shell.sh, not by re-running this script.
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
IMAGE_NAME="rlds_builder:py39-tf213"
CONTAINER_NAME="rlds_builder"

docker build -t "$IMAGE_NAME" -f "$REPO_ROOT/docker/Dockerfile" "$REPO_ROOT"

if docker ps -a --format '{{.Names}}' | grep -qx "$CONTAINER_NAME"; then
    echo "Container '$CONTAINER_NAME' already exists. Remove it first if you want a clean start:"
    echo "  docker rm -f $CONTAINER_NAME"
    exit 1
fi

docker run -d \
    --name "$CONTAINER_NAME" \
    -v "$REPO_ROOT:/workspace" \
    "$IMAGE_NAME"

echo "Container '$CONTAINER_NAME' started."
echo "Open a shell with: $REPO_ROOT/docker/shell.sh"
