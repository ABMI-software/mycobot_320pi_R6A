#!/usr/bin/env bash
# Every terminal after the first: attach to the already-running container.
set -euo pipefail
docker exec -it rlds_builder bash -lc 'conda activate rlds_env && exec bash'
