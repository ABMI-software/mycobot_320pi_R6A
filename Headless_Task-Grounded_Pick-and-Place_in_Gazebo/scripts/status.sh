#!/bin/bash
# status.sh -- ground truth about the 20-episode recording batch, read from
# disk only (grasp_meta.json's own exit-code/verdict fields, the bag file's
# presence and size, and file mtimes). Prints nothing from memory.
#
# Runnable from the plain WSL host shell, no attached Claude session needed:
#   bash scripts/status.sh
#
# Depends only on `docker` and `python3` on the HOST (JSON parsing happens
# host-side over `docker exec ... cat`, so the container needs no python3
# json tooling of its own).
set -u
CONTAINER=gazebo_to_lerobot
EPDIR=/workspace/htgpp/episodes
FRESH_MIN=20   # an episode dir touched more recently than this and not yet
               # marked done is IN-PROGRESS; older and undone is FAILED.
               # 20 min is generous: a normal episode takes ~7-12 min end to
               # end at this sim's measured real-time factor (~0.08).

echo "=== status.sh -- read at:"
echo "  UTC:          $(date -u '+%Y-%m-%d %H:%M:%S %Z')"
echo "  Europe/Paris: $(TZ=Europe/Paris date '+%Y-%m-%d %H:%M:%S %Z')"
echo

if ! docker ps --format '{{.Names}}' | grep -qx "$CONTAINER"; then
  echo "FATAL: container '$CONTAINER' is not running (docker ps shows no such name)."
  exit 1
fi

NOW_EPOCH=$(date -u +%s)

echo "=== per-episode status (1-20) ==="
PASSED=0; FAILED=0; MISSING=0; INPROG=0
LATEST_EPOCH=0; LATEST_EP=""
for i in $(seq -w 1 20); do
  EP="ep_0$i"
  META="$EPDIR/$EP/grasp_meta.json"

  EXISTS=$(docker exec "$CONTAINER" bash -c "[ -d $EPDIR/$EP ] && echo 1 || echo 0")
  if [ "$EXISTS" = "0" ]; then
    echo "episode $i: MISSING (no $EP directory)"
    MISSING=$((MISSING+1))
    continue
  fi

  # latest mtime of any file in this episode dir (epoch seconds), read fresh
  MAXMTIME=$(docker exec "$CONTAINER" bash -c \
    "find $EPDIR/$EP -type f -printf '%T@\n' 2>/dev/null | sort -n | tail -1")
  MAXMTIME_INT=${MAXMTIME%.*}
  if [ -n "$MAXMTIME_INT" ] && [ "$MAXMTIME_INT" -gt "$LATEST_EPOCH" ]; then
    LATEST_EPOCH=$MAXMTIME_INT
    LATEST_EP=$i
  fi

  BAG_OK=$(docker exec "$CONTAINER" bash -c \
    "find $EPDIR/$EP/bag -type f -name '*.mcap' -size +0c 2>/dev/null | head -1 | wc -l")

  META_JSON=$(docker exec "$CONTAINER" bash -c "cat '$META' 2>/dev/null")
  if [ -n "$META_JSON" ]; then
    read -r RC PLACED HELD <<<"$(python3 -c "
import json,sys
try:
    m=json.loads(sys.argv[1])
    print(m.get('run_exit_code'), m.get('placed_on_plate'), m.get('grasp_held'))
except Exception:
    print('None None None')
" "$META_JSON")"
  else
    RC=None; PLACED=None; HELD=None
  fi

  if [ "$RC" = "0" ] && [ "$PLACED" = "True" ] && [ "$HELD" = "True" ] && [ "$BAG_OK" = "1" ]; then
    echo "episode $i: PASSED (rc=0, placed_on_plate=True, grasp_held=True, bag present)"
    PASSED=$((PASSED+1))
  elif [ "$RC" != "None" ] && [ "$RC" != "0" ]; then
    echo "episode $i: FAILED (run_exit_code=$RC in grasp_meta.json)"
    FAILED=$((FAILED+1))
  else
    AGE_MIN=$(( (NOW_EPOCH - MAXMTIME_INT) / 60 ))
    if [ -n "$MAXMTIME_INT" ] && [ "$AGE_MIN" -le "$FRESH_MIN" ]; then
      echo "episode $i: IN-PROGRESS (no completion marker yet, last file touched ${AGE_MIN}m ago)"
      INPROG=$((INPROG+1))
    else
      echo "episode $i: FAILED (no completion marker, last touched ${AGE_MIN}m ago, > ${FRESH_MIN}m fresh window -- stalled/abandoned)"
      FAILED=$((FAILED+1))
    fi
  fi
done

echo
echo "=== totals (counted above, not asserted) ==="
echo "  PASSED=$PASSED  FAILED=$FAILED  IN-PROGRESS=$INPROG  MISSING=$MISSING  (sum=$((PASSED+FAILED+INPROG+MISSING)) of 20)"

echo
echo "=== most recently modified episode folder ==="
if [ -n "$LATEST_EP" ]; then
  AGE_MIN=$(( (NOW_EPOCH - LATEST_EPOCH) / 60 ))
  echo "  ep_0$LATEST_EP, ${AGE_MIN} minutes ago"
else
  echo "  none found"
fi

echo
echo "=== container uptime vs wall-clock elapsed (reveals a host suspend) ==="
STARTED_AT=$(docker inspect -f '{{.State.StartedAt}}' "$CONTAINER")
STARTED_EPOCH=$(date -u -d "$STARTED_AT" +%s 2>/dev/null)
if [ -n "$STARTED_EPOCH" ]; then
  WALL_ELAPSED_S=$((NOW_EPOCH - STARTED_EPOCH))
else
  WALL_ELAPSED_S=""
fi

# PID 1's own monotonic age: /proc/uptime's first field is seconds since
# boot on a clock that does NOT advance during host suspend; /proc/1/stat's
# 22nd field is PID1's start time in clock ticks on that same clock. Their
# difference is how long the container's init has genuinely been running,
# unaffected by suspend -- unlike StartedAt vs now, which are both wall-clock
# (RTC/NTP) timestamps that DO jump across a suspend/resume.
read -r PROC_UPTIME _ < <(docker exec "$CONTAINER" cat /proc/uptime)
CLK_TCK=$(docker exec "$CONTAINER" getconf CLK_TCK)
PID1_STARTTICKS=$(docker exec "$CONTAINER" bash -c "awk '{print \$22}' /proc/1/stat")
if [ -n "$PROC_UPTIME" ] && [ -n "$CLK_TCK" ] && [ -n "$PID1_STARTTICKS" ]; then
  MONO_AGE_S=$(python3 -c "print(int(float('$PROC_UPTIME') - float('$PID1_STARTTICKS')/float('$CLK_TCK')))")
else
  MONO_AGE_S=""
fi

fmt_hm() { local s=$1; printf '%dh%02dm' $((s/3600)) $(((s%3600)/60)); }

echo "  container StartedAt (wall clock): $STARTED_AT"
if [ -n "$WALL_ELAPSED_S" ]; then
  echo "  wall-clock elapsed since start:   $(fmt_hm "$WALL_ELAPSED_S") ($WALL_ELAPSED_S s)"
fi
if [ -n "$MONO_AGE_S" ]; then
  echo "  PID1 monotonic uptime (suspend-blind clock): $(fmt_hm "$MONO_AGE_S") ($MONO_AGE_S s)"
fi
if [ -n "$WALL_ELAPSED_S" ] && [ -n "$MONO_AGE_S" ]; then
  GAP_S=$((WALL_ELAPSED_S - MONO_AGE_S))
  if [ "$GAP_S" -gt 60 ]; then
    echo "  GAP: wall-clock elapsed exceeds monotonic uptime by $(fmt_hm "$GAP_S") -- consistent with a host suspend/pause of about that long."
  else
    echo "  GAP: ${GAP_S}s -- no evidence of a suspend since container start."
  fi
fi
