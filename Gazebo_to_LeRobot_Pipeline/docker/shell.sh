#!/usr/bin/env bash
# Every terminal after the first: attach to the already-running container.
set -euo pipefail
docker exec -it gazebo_to_lerobot bash
