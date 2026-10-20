#!/usr/bin/env bash
# Part 9.6 -- ground truth from the filesystem, never from memory.
set -u
EPDIR="${EPDIR:-/workspace/htgspp/episodes}"

echo "=== status.sh -- read at ==="
echo "  UTC:          $(date -u '+%Y-%m-%d %H:%M:%S UTC')"
echo "  Europe/Paris: $(TZ=Europe/Paris date '+%Y-%m-%d %H:%M:%S %Z')"

echo "=== per-object status ==="
python3 - "$EPDIR" << 'PY'
import glob, json, os, sys
epdir = sys.argv[1]
objs = ["red_cube", "blue_cube", "green_cylinder", "yellow_box"]
rows = {o: {"train": [0, 0], "heldout": [0, 0], "fail": []} for o in objs}
for f in sorted(glob.glob(os.path.join(epdir, "ep_*", "*.verdict.json"))):
    v = json.load(open(f))
    o, split, verdict = v["target"], v["split"], v["verdict"]
    if o not in rows:
        continue
    rows[o][split][1] += 1
    if verdict == "PASS":
        rows[o][split][0] += 1
    else:
        rows[o]["fail"].append((os.path.basename(os.path.dirname(f)), verdict))
totals = {"PASSED": 0, "FAILED": 0, "WRONG_BIN": 0, "MISSING": 0}
for o in objs:
    tr, ho, fails = rows[o]["train"], rows[o]["heldout"], rows[o]["fail"]
    passed = tr[0] + ho[0]
    line = f"{o:16s} {tr[0]}/{tr[1]} train   {ho[0]}/{ho[1]} heldout   "
    line += "OK" if not fails else " ".join(f"{v} ({n})" for n, v in fails)
    print(line)
    totals["PASSED"] += passed
    for n, v in fails:
        totals[v if v in totals else "FAILED"] += 1
missing = 60 - sum(v for k, v in totals.items())
totals["MISSING"] = missing if missing > 0 else 0
print("\n=== totals (counted above, not asserted) ===")
s = "  " + "=".join([])
print(f"  PASSED={totals['PASSED']}  FAILED={totals['FAILED']}  "
      f"WRONG_BIN={totals['WRONG_BIN']}  MISSING={totals['MISSING']}  "
      f"(sum={sum(totals.values())} of 60)")
PY

echo "=== most recently modified episode folder ==="
if [ -d "$EPDIR" ]; then
  ls -dt "$EPDIR"/ep_*/ 2>/dev/null | head -1 | xargs -r -I{} bash -c \
    'echo "  $(basename {}), $(( ($(date +%s) - $(stat -c %Y {})) / 60 )) minutes ago"'
else
  echo "  (episode directory $EPDIR does not exist yet)"
fi

echo "=== container uptime vs wall-clock elapsed (reveals a host suspend) ==="
if [ -f /proc/uptime ] && [ -f "$EPDIR/../batch_start_epoch" ]; then
  start=$(cat "$EPDIR/../batch_start_epoch")
  uptime_now=$(cut -d' ' -f1 /proc/uptime)
  now=$(date +%s)
  wall_elapsed=$((now - start))
  gap=$((wall_elapsed - ${uptime_now%.*}))
  echo "  GAP: ${gap}s $([ "$gap" -gt 60 ] && echo '-- HOST SUSPEND SUSPECTED' || echo '-- no evidence of a suspend')"
else
  echo "  (no batch_start_epoch marker yet -- run batch.sh first)"
fi
