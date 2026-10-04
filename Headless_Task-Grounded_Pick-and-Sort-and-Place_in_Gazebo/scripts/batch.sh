#!/usr/bin/env bash
# Part 11.4/11.5 -- run episodes START..END unattended: one attempt plus ONE
# automatic retry each, every attempt's outcome appended to a results file.
# Never stops on a bad episode: episode.sh carries its own watchdog, and its
# first act is to kill whatever a crashed or timed-out attempt left behind.
#   bash scripts/batch.sh [START] [END]
# Results: /workspace/htgspp/batch_results.csv -- one row per ATTEMPT:
#   utc,episode,attempt,exit_code,verdict,target,split,wall_s,frames_received,frames_expected,resends
# resends = trajectory re-sends over the whole episode (at most one per pose):
# a re-send changes the recorded trajectory, so it is tracked per attempt.
set -u
START="${1:-1}"
END="${2:-60}"
# BATCH_HOME lets a TEST of this script write to a scratch directory -- never
# test against the real one (a 2026-10-03 test appended fake rows to it).
H="${BATCH_HOME:-/workspace/htgspp}"
RES="$H/batch_results.csv"
mkdir -p "$H"
date +%s > "$H/batch_start_epoch"
# Which container produced this batch (hostname inside a container = its short ID;
# the name is passed in by whoever starts the batch: docker exec -e CONTAINER_NAME=...).
# code_version: content hash of everything that shapes an episode, so the log
# says which code produced which episode range (nothing is committed mid-batch).
CODE_VERSION=$(find scripts models worlds config contracts -type f \( -name "*.py" -o -name "*.sh" \
  -o -name "*.sdf" -o -name "*.yaml" -o -name "*.csv" -o -name "*.json" -o -name "*.jsonl" \
  -o -name "*.txt" -o -name "WORLD_PATH" \) | sort | xargs sha256sum | sha256sum | cut -c1-12)
echo "=== batch $START..$END start $(date -u +%FT%TZ) container_id=$(hostname) container_name=${CONTAINER_NAME:-unknown} code_version=$CODE_VERSION ==="
[ -f "$RES" ] || echo "utc,episode,attempt,exit_code,verdict,target,split,wall_s,frames_received,frames_expected,resends" > "$RES"

field() {   # field <json> <key> -> value, or empty if the file or key is missing
  python3 -c "import json
try: print(json.load(open('$1')).get('$2', ''))
except Exception: print('')" 2>/dev/null
}

passed=0; failed=0; consecutive_failed=0
# Circuit breaker: 3 consecutive episodes failing ALL their attempts stops the
# batch -- a systematic failure (e.g. one object that never works) must not
# burn the rest of an unattended night.
BREAK_AFTER=3
for i in $(seq "$START" "$END"); do
  ok=0
  for attempt in 1 2; do
    echo "=== episode $i, attempt $attempt  ($(date -u +%FT%TZ)) ==="
    t0=$(date +%s)
    bash scripts/episode.sh "$i"
    rc=$?
    E="$H/episodes/ep_$(printf '%03d' "$i")"
    verdict=$(field "$E/grasp_meta.verdict.json" verdict); verdict=${verdict:-NONE}
    echo "$(date -u +%FT%TZ),$i,$attempt,$rc,$verdict,$(field "$E/grasp_meta.json" target),$(field "$E/grasp_meta.json" split),$(( $(date +%s) - t0 )),$(field "$E/frames.json" received),$(field "$E/frames.json" expected),$(field "$E/grasp_meta.json" trajectory_resends_total)" >> "$RES"
    # Keep this attempt's evidence OUTSIDE the episode folder: the next
    # attempt's episode.sh starts with `rm -rf` on that folder (episode 16's
    # failed attempts lost their re-send counts that way, 2026-10-03).
    A="$H/attempt_meta/ep_$(printf '%03d' "$i")"
    mkdir -p "$A"
    for f in grasp_meta.json grasp_meta.verdict.json frames.json colour.json \
             run.log grasp_log.csv final_verdict.log; do
      [ -f "$E/$f" ] && cp "$E/$f" "$A/${f%.*}.attempt$attempt.${f##*.}"
    done
    if [ "$rc" = 0 ] && [ "$verdict" = PASS ]; then ok=1; break; fi
    echo "episode $i attempt $attempt: rc=$rc verdict=$verdict" >&2
  done
  # ...and back beside the final attempt, as grasp_meta.attempt<N>.json etc.
  cp "$A"/*.attempt[0-9].* "$E/" 2>/dev/null
  if [ "$ok" = 1 ]; then passed=$((passed + 1)); consecutive_failed=0; else
    failed=$((failed + 1)); consecutive_failed=$((consecutive_failed + 1))
    echo "episode $i FAILED both attempts -- see $H/stage_$(printf '%03d' "$i").log" >&2
    if [ "$consecutive_failed" -ge "$BREAK_AFTER" ]; then
      echo "=== CIRCUIT BREAKER: $BREAK_AFTER consecutive episodes failed all attempts (last: $i); batch STOPPED: passed=$passed failed=$failed ==="
      exit 3
    fi
  fi
done
echo "=== batch $START..$END done: passed=$passed failed=$failed  (per attempt: $RES) ==="
