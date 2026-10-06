#!/usr/bin/env bash
# Part 11.4/11.5 -- run episodes START..END, up to 3 attempts each.
set -u
START="${1:-1}"
END="${2:-60}"
mkdir -p /workspace/htgspp
date +%s > /workspace/htgspp/batch_start_epoch

for i in $(seq "$START" "$END"); do
  ok=0
  for attempt in 1 2 3; do
    echo "=== episode $i, attempt $attempt ==="
    bash scripts/episode.sh "$i" && { ok=1; break; }
    echo "episode $i attempt $attempt FAILED, retrying" >&2
  done
  [ "$ok" = 1 ] || echo "episode $i FAILED all 3 attempts -- see /workspace/htgspp/stage_$(printf '%03d' "$i").log" >&2
done
