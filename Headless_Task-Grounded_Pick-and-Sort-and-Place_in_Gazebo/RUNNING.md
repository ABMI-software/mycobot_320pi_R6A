# Running the four-object sorting acquisition

**What this proves and doesn't (Part 11.7):** everything that doesn't step
physics is built, self-consistent, and verified on this machine — the
world, the object/bin geometry, the IK/waypoints (`check_waypoints.py`
exits 0), the episode/task/contract configs, and the verifier's
correct-rejection logic (`preflight.sh` passes end to end) — but the
smoke test itself, the one thing that *does* step physics, is not proven
here: recorder wiring reached "recording started" live under Gazebo
exactly once, no episode has ever completed to a PASS/FAIL/WRONG_BIN
verdict, and two distinct, unresolved problems blocked every further
attempt — this machine's ~3.8 GB memory ceiling under `gz sim`, and a
separate, real, root-cause-unidentified hang in the recorder's own
lifecycle service reproduced independently of Gazebo (both detailed in
`doc/SMOKE_TEST.md`) — so the four smoke episodes below are the actual
first test of this pipeline against real physics, not a formality.

## 1. Check the environment

    bash preflight.sh

Expect `PREFLIGHT PASSED`. If not, stop — the failing line names the cause.

## 2. First action on this machine: four smoke episodes, one per object

    bash scripts/episode.sh 1    # red_cube
    bash scripts/episode.sh 2    # whichever episode is blue_cube in config/episode_matrix.csv
    bash scripts/episode.sh 3    # green_cylinder -- do not skip this one
    bash scripts/episode.sh 4    # yellow_box

(Check `config/episode_matrix.csv` for the exact episode indices per
object/split if these four don't match — the point is one of each
object, cylinder included, before the unattended batch.)

**Stop and report back if any of these four fails** — do not proceed to
the full batch on a guess that it'll sort itself out, and do not retry
past a second attempt on the same episode without reporting first. This
is not extra caution for its own sake: on Role A hardware this session,
recorder wiring was confirmed working end to end exactly once, under
Gazebo load — but no single episode was ever driven through to a
completed PASS/FAIL/WRONG_BIN verdict, and a second, independent
resource-exhaustion mode surfaced when testing the recorder in
isolation (a lifecycle-service hang, unrelated to memory — see
`doc/SMOKE_TEST.md`'s 2026-09-23 entries for the full record). Role B's
hardware is expected to be materially less memory-constrained, but that
expectation itself hasn't been tested — these four episodes are that
test. A failure here is exactly the kind of thing worth stopping for.

## 3. Run the batch (~5–8 hours on Role B hardware)

    bash scripts/batch.sh 1 60

Restarts the simulator per episode, up to 3 attempts each, 25-minute
watchdog per attempt (`episode.sh`'s own `timeout -k 20 1500`). Safe to
leave unattended — once the four smoke episodes above have passed.

**IMPORTANT: disable system sleep first** — a suspended host presents as a
stall, not an error. In an elevated Windows PowerShell:

    powercfg /change standby-timeout-ac 0
    powercfg /change hibernate-timeout-ac 0
    powercfg /change monitor-timeout-ac 15

## 4. Check progress or the result — any time, in a separate terminal

    bash scripts/status.sh

### Success looks like

    PASSED=60  FAILED=0  WRONG_BIN=0  MISSING=0  (sum=60 of 60)

and 15 per object across train + heldout.

## If an episode fails three times

`status.sh` names it. Its stage log is at `/workspace/htgspp/stage_<idx>.log`,
**outside** the episode folder, so a retry cannot erase it. Send that file
back.

## Convert to LeRobot format (after the batch, or per-object as episodes finish)

    python3 scripts/port_bag.py --out datasets/mycobot_sorting_train --split train
    python3 scripts/port_bag.py --out datasets/mycobot_sorting_heldout --split heldout

Only ports episodes whose `grasp_meta.json` says `"verdict": "PASS"` —
failed attempts are silently skipped, not counted, matching A9.

## What to return

Either `datasets/` (small, converted) or the raw episodes under
`/workspace/htgspp/episodes/` (~5 GB, regenerable from `batch.sh`).

## If this is the first run on this machine

The graphical Gazebo path has never been exercised by Role A — every
artefact here was built and tested headless (`gz sim -s`). The first run
on Role B's machine should be a short joint session before the unattended
60-episode batch, specifically to catch any display-passthrough surprise
early rather than 4 hours into an unattended run. See Part 11.8 in the
full specification.
