# Running the four-object sorting acquisition

**Status (2026-10-04):** run end to end on a single WSL2 laptop, inside the
`gazebo_to_lerobot` container: a 60-episode batch under one code version,
60/60 PASS on the first attempt, converted to both datasets
(`MEASUREMENTS.md` §10, `datasets/README.md`). The history before that
(the smoke-test blockers of 2026-09-23) is in `doc/SMOKE_TEST.md`.

**Second acquisition (2026-10-10):** Osama's four-object sort (`tri_yolo`),
recorded as one episode per pick-and-place and converted to LeRobot and RLDS.
Its commands are in the last section, "Recording Osama's four-object sort";
the full account is `doc/FOUR_OBJECT_SORTING_REPORT` addendum §29–43 and
the specification's Appendix D.

## Where each command runs

Every command below runs in a **shell inside the container**, unless it starts
with `docker` or `sudo` (those run on the WSL2 host, your Ubuntu prompt). From
the host:

    sudo sh -c 'nohup dockerd > /tmp/dockerd.log 2>&1 &'   # WSL has no systemd: start Docker by hand
    docker start gazebo_to_lerobot                          # starts the container in the background, gives no shell
    docker exec -it gazebo_to_lerobot bash                  # a shell inside it; the prompt becomes root@<container id>

That shell's `~/.bashrc` loads ROS 2 and the workspace, so nothing needs
sourcing. `exit` leaves it; the container keeps running.

## 0. Watch one episode, or check one pass/fail

Two launch files run a single row of `config/episode_matrix.csv` (1–60:
1–15 red cube, 16–30 blue cube, 31–45 green cylinder, 46–60 yellow box)
exactly as the batch does, without recording. In a container shell:

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

    cd /workspace/htgspp && bash preflight.sh

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
is not extra caution for its own sake: on the memory-constrained machine of 2026-09-23,
recorder wiring was confirmed working end to end exactly once, under
Gazebo load — but no single episode was ever driven through to a
completed PASS/FAIL/WRONG_BIN verdict, and a second, independent
resource-exhaustion mode surfaced when testing the recorder in
isolation (a lifecycle-service hang, unrelated to memory — see
`doc/SMOKE_TEST.md`'s 2026-09-23 entries for the full record). A machine
with more memory was expected to avoid both, but that
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

## Convert to LeRobot format (after the batch)

On the **WSL2 host, not in the container**: the converter uses LeRobot's own
writer in a CPU-only venv, pinned in `scripts/requirements-lerobot.txt`
(install commands at the top of that file). Copy the bags and the episode
metadata out of the container first, then:

    V=~/venvs/lerobot044-cpu/bin/python
    $V scripts/port_bag.py --split train   --out datasets/mycobot_sorting_train \
        --episodes-dir <copy>/episodes --bags-dir <copy>/bags
    $V scripts/port_bag.py --split heldout --out datasets/mycobot_sorting_heldout \
        --episodes-dir <copy>/episodes --bags-dir <copy>/bags

~1 min for held-out, ~4 min for train on the laptop. Only ports episodes whose
`grasp_meta.verdict.json` says `"verdict": "PASS"` and whose frames, colour
check and re-send count are also clean; anything else is skipped with its
reason printed. An episode that displaced another object is kept and labelled
(`distractors_moved` in the sidecar `meta/episodes_extra.jsonl`). Format,
action definition and validation: `datasets/README.md`.

## What to return

Either `datasets/` (112 MB converted, 2026-10-05) or the raw recordings under
`/workspace/htgspp/bags/` (8.1 GB, regenerable from `batch.sh`).

## If this is the first run on this machine

The graphical path was exercised on 2026-10-04 (section 0). On WSL2, keep
`DISPLAY=:0` and software OpenGL: the host's forwarded DISPLAY stalls the
simulation clock, and the GPU path renders every spawned object white.

## Recording Osama's four-object sort (added 2026-10-10)

Osama's scene (`tri_yolo`, team repository `main` `36e2d4d5`, merged into
`feature/sort-episodes`) sorts four objects into four bins per seed with real
gripper contact. Positions come from Gazebo ground truth
(`sim_sorting_grasp -p pose_source:=ground_truth`), not YOLO. Each
pick-and-place is one rosetta episode: cameras `top`, `right`, `left` at
640×480 and 10 Hz, state = 6 joints + gripper, action = what the controllers
command. Report addendum §29–43 explains every choice.

### From a fresh clone, on a bare Linux machine (or WSL2)

Needs only `git` and Docker (and an X display for the Gazebo GUI). Everything
else is built from this repository: `docker/` holds the image
(`tri_sort:jazzy-harmonic`, on top of `Gazebo_to_LeRobot_Pipeline/docker`),
the exact LeRobot environment (`requirements_lerobot.txt`), the rosetta
packages pinned to the commits used (`rosetta.repos`), and two scripts.

    git clone https://github.com/ABMI-software/mycobot_320pi_R6A.git && cd mycobot_320pi_R6A
    Headless_Task-Grounded_Pick-and-Sort-and-Place_in_Gazebo/docker/setup_workspace.sh   # images + ~/tri_sort_ws/src
    Headless_Task-Grounded_Pick-and-Sort-and-Place_in_Gazebo/docker/run_container.sh     # containers + colcon build

`setup_workspace.sh [workspace]` builds three images (`gazebo_to_lerobot`,
`tri_sort`, `rlds_builder`; the first time downloads several GB), clones the
pinned rosetta packages into `~/tri_sort_ws/src`, and copies into it
`mycobot_description` and `mycobot_gateway` (repository root),
`mycobot_moveit_config`, the controller rate of 100 Hz, `sorting_table.sdf`,
and every script and contract below to `scripts/`, `scripts/rlds/`,
`contracts/`, `training/dream/`. Re-run it after a `git pull`.
`run_container.sh [workspace]` starts `gazebo_to_lerobot` (`--shm-size=2g`,
the X display: `TRI_DISPLAY`; `xhost +local:root` on plain Linux) and
`rlds_builder`, then runs `rosdep` and `colcon build` inside. `CONTAINER` and
`RLDS_CONTAINER` change the container names.

The container runs the copies placed in the workspace (`/workspace/src/scripts`,
`scripts/rlds`, `contracts`); the repository copies are under this project's
`scripts/`, `contracts/`, `docker/`, and `ROS2_to_RLDS_Conversion_OpenVLA/extraction/`.

    # WSL2 host, every new session (WSL2 has no systemd; Docker stops with WSL2)
    sudo sh -c 'nohup dockerd > /tmp/dockerd.log 2>&1 &'; sleep 5
    # every new session, Linux or WSL2
    docker start gazebo_to_lerobot rlds_builder

    # watch a sort with the Gazebo GUI (seed, optional piece list)
    docker exec -it gazebo_to_lerobot /workspace/src/scripts/run_sort_gui.sh 1
    docker exec -it gazebo_to_lerobot /workspace/src/scripts/run_sort_gui.sh 1 cube_rouge

    # record one seed, or Osama's ten (about 19 min per seed; resumable)
    docker exec gazebo_to_lerobot bash -c 'TRI_HOME=/workspace/tri_sort /workspace/src/scripts/record_tri_seed.sh 1'
    docker exec -d gazebo_to_lerobot /workspace/src/scripts/record_tri_batch.sh 1 10
    docker exec gazebo_to_lerobot tail -f /workspace/tri_sort/batch.log
    docker exec gazebo_to_lerobot python3 /workspace/src/scripts/summarise_tri_batch.py

    # convert: LeRobot (+ check with LeRobotDataset) and the RLDS .npy, then the RLDS build
    docker exec gazebo_to_lerobot /workspace/src/scripts/convert_tri_sort.sh
    ./scripts/build_rlds_tri_sort_host.sh

    # replay one episode from each dataset in Gazebo (GUI): <seed> <piece>, default 1 cube_rouge;
    # HEADLESS=1 without the window
    docker exec -it gazebo_to_lerobot /workspace/src/scripts/replay_gui.sh lerobot 1 cube_rouge
    ./scripts/replay_rlds_gui_host.sh 1 cube_rouge

    # pieces: cube_rouge, pave_jaune, cylindre_vert, cube_bleu (Osama's sort order)

Expected results (2026-10-10, Osama's seeds 1–10): 40/40 pick-and-places in
their bin, each on its first attempt (seed 10 needed a re-run: its blue cube was
pushed out of reach once; contact physics is not deterministic, so check every
verdict with `summarise_tri_batch.py`), ~19 min per seed, 19.6 GB of bags;
LeRobot 40 episodes / 7 049 frames, RLDS 40 episodes / 7 056 steps.

### Random-start recordings (seeds 21-50 on 14/10, 51-100 from 16/10)

`RANDOM_START=1` starts each piece from a random arm pose instead of the
observation pose, so the joints alone no longer tell where the trajectory goes:

    docker exec gazebo_to_lerobot bash -c 'RANDOM_START=1 /workspace/src/scripts/record_tri_seed.sh 51'
    docker exec -d gazebo_to_lerobot bash -c 'RANDOM_START=1 /workspace/src/scripts/record_tri_batch.sh 52 100'

`random_start_sorter.py` is the scripted sorter with one step added: after the
positions are read at the observation pose, the arm moves to a pose drawn by
`random_start.py` with numpy `default_rng(<seed>)`. Every draw goes to
`seed_NNN/start_poses.jsonl` (seed, draw index, pose, tip, and the largest
displacement of any piece during the move, from Gazebo poses before and after),
and each `ep_*.json` carries the start pose its episode began from
(`random_start`).

**The sampler of seeds 21-50 was lost with the machine (15/10), and seeds
51-100 use a different one.** Its poses fit no simple rule, so the new sampler
draws from them: one of the 120 recorded start poses
(`config/random_start_poses_s021_050.csv`, from `mycobot-tri-sort-160`, zip
sha256 `0738a842…5c65`) plus Gaussian jitter of 0.25 × each joint's spread, kept
inside each joint's recorded range, with the tip at least 13 cm above the board
and every joint 20° from its limits. Compared on 120 drawn vs the 120 recorded
poses, no joint differs significantly (KS p 0.18-0.99) and the lowest tip is
13.2 cm in both. It is **not** identical:

* slightly narrower at the extremes, mostly J2, J4 and J6 (for example J2 from
  -18° instead of -30°, J4 -73/78° instead of -86/83°, J6 up to 122° instead of
  153° in one draw of 120);
* somewhat fewer low starts: tip below 20 cm in 11-14 of 120 draws, against 19 of
  120 recorded (not significant, Fisher p 0.17-0.45).

A dataset mixing seeds 21-50 and 51-100 therefore mixes two start-pose
distributions; compare per seed range if a result depends on the start.

**Held-out seeds, never trained on:** 11-20 (Gazebo evaluation, never recorded),
**91-100** (recorded 17/10 with random starts, open-loop evaluation of every
checkpoint, as `local/mycobot_tri_sort_heldout_s091_100`), 101-120 (reserve).
`convert_tri_sort.sh` skips them and prints each one it leaves out.

Before a batch: Windows sleep off (`powercfg /change standby-timeout-ac 0`,
`powercfg /change hibernate-timeout-ac 0`), keep a WSL2 terminal open, mains
power. "Cannot connect to the Docker daemon" means WSL2 restarted: run the
first two commands again; `record_tri_batch.sh` with the same range resumes.

### One episode per layout, 0-3 pieces already sorted (seeds 1001-1400, from 18/10)

For Run B: about 400 layouts, one episode each. `PREPLACE=1` makes
`record_tri_seed.sh` record only the target `PIECES[seed % 4]` (red cube, yellow
box, green cylinder, blue cube), after `preplace_pieces.py` has put 0-3 of the
other pieces in their bins (never the target; `preplaced.json`). Each
`ep_*.json` carries the Gazebo positions of the four pieces and four bins at the
start and end of the episode. Kept apart from the older seeds, in
`/workspace/tri_sort_B`:

    docker exec -d gazebo_to_lerobot bash -c 'TRI_HOME=/workspace/tri_sort_B RANDOM_START=1 PREPLACE=1 /workspace/src/scripts/record_tri_batch.sh 1001 1050'
    docker exec gazebo_to_lerobot python3 /workspace/src/scripts/verify_preplaced_batch.py /workspace/tri_sort_B 1001 1050
    docker exec gazebo_to_lerobot bash -c 'TRI_HOME=/workspace/tri_sort_B STAGE=/workspace/tri_sort_B_stage TRAIN_SEEDS=/workspace/tri_sort_B_train_seeds SKIP_RLDS=1 /workspace/src/scripts/convert_tri_sort.sh local/mycobot_tri_sort_B_s1001_1050'
    docker exec gazebo_to_lerobot /workspace/venv_lerobot/bin/python /workspace/src/scripts/verify_preplaced_batch.py /workspace/tri_sort_B 1001 1050 --dataset /workspace/tri_datasets/local/mycobot_tri_sort_B_s1001_1050

`--dataset` needs pandas, so it runs in the LeRobot venv; the container's
system python3 has none. Measured on 1001-1050: median 6.2 min per layout
(5.4-6.9) once nothing else ran, 0.48 GB of bags per layout, 50 layouts ->
147 MB dataset (seed 1001: 11.5 min with two other CPU jobs running). Seeds 5001-5800 are the encoder-probe layouts and are never
converted.
