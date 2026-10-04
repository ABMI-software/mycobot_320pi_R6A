# Running the four-object sorting acquisition

**Status (2026-10-04):** run end to end on a single WSL2 laptop, inside the
`gazebo_to_lerobot` container: a 60-episode batch under one code version,
60/60 PASS on the first attempt, converted to both datasets
(`MEASUREMENTS.md` §10, `datasets/README.md`). The history before that
(the smoke-test blockers of 2026-09-23) is in `doc/SMOKE_TEST.md`.

## 0. Watch one episode, or check one pass/fail

Two launch files run a single row of `config/episode_matrix.csv` (1–60:
1–15 red cube, 16–30 blue cube, 31–45 green cylinder, 46–60 yellow box)
exactly as the batch does, without recording:

    cd /workspace/htgspp/launch
    ros2 launch pick_and_sort_and_place_demo.launch.py episode:=16          # Gazebo GUI, stays open; Ctrl+C to stop
    ros2 launch pick_and_sort_and_place_no_gui_demo.launch.py episode:=16   # headless
    echo $?                                                                  # 0 = PASS, 1 = not, 2 = bad argument

On the WSL2 laptop, the GUI in one command from the host:

    docker exec -it gazebo_to_lerobot /workspace/htgspp/scripts/run_gui_demo.sh episode:=16

Measured on that laptop: headless ~100 s per episode (up to ~6 min when the
controllers need respawning), GUI the same plus the viewport, which renders
in software (DISPLAY=:0, llvmpipe; `scripts/run_demo.py` sets both).

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

## 3. Run the batch (~2.7 hours on the WSL2 laptop, 2026-10-04)

    docker exec -d -w /workspace/htgspp -e CONTAINER_NAME=gazebo_to_lerobot gazebo_to_lerobot \
        bash -c 'bash scripts/batch.sh 1 60 >> batch.log 2>&1'

Restarts the simulator per episode, up to 2 attempts each, a 20-minute
watchdog per attempt (`episode.sh`'s `timeout -k 20 1200`), and stops after
3 consecutive episodes fail every attempt (circuit breaker). The start line
of `batch.log` records the container and the code version (a hash of
`scripts`, `models`, `worlds`, `config`, `contracts`). A test of `batch.sh`
itself must set `BATCH_HOME` to a scratch directory, never the real one.

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

## If an episode fails both attempts

`status.sh` names it. Its stage log is at `/workspace/htgspp/stage_<idx>.log`,
**outside** the episode folder, so a retry cannot erase it. Send that file
back.

## Convert to LeRobot format (after the batch, or per-object as episodes finish)

    python3 scripts/port_bag.py --out datasets/mycobot_sorting_train --split train
    python3 scripts/port_bag.py --out datasets/mycobot_sorting_heldout --split heldout

Only ports episodes whose `grasp_meta.verdict.json` says `"verdict": "PASS"`
and whose frames, colour check and re-send count are also clean; anything
else is skipped with its reason printed. An episode that displaced another
object is kept and labelled (`distractors_moved` in `meta/episodes.jsonl`).

## What to return

Either `datasets/` (19 MB converted, 2026-10-04) or the raw recordings under
`/workspace/htgspp/bags/` (8.1 GB, regenerable from `batch.sh`).

## If this is the first run on this machine

The graphical path was exercised on 2026-10-04 (section 0). On WSL2, keep
`DISPLAY=:0` and software OpenGL: the host's forwarded DISPLAY stalls the
simulation clock, and the GPU path renders every spawned object white.
