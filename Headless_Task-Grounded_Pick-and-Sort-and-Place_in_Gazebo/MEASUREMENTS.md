# Measurements — four-object sorting, single-host run (2026-10-03)

Every number below was measured on this host, in the `gazebo_to_lerobot`
container, on 2026-10-03. Nothing is estimated. Each section names the command
or script that produced it, so it can be re-measured.

**Host:** Windows laptop, WSL2 (Ubuntu 22.04.3, kernel 6.18.40.1-microsoft-standard-WSL2),
Intel Core i7-11370H (4 cores / 8 threads), 7.8 GB RAM and 2 GB swap visible in
WSL. Docker Engine 24.0.5 inside WSL. Container `gazebo_to_lerobot`
(Ubuntu 24.04, ROS 2 Jazzy, Gazebo Harmonic / gz-sim 8): no CPU or memory limit
(`NanoCpus=0`, `Memory=0`).

**Scene:** `worlds/sorting_table.sdf` (the `real_table` bench with no objects
or bins), robot spawned by `mycobot_gateway/sim_grasp.launch.py`, and for
§1–2 the four bins and four objects of episode 1. All runs are headless
(`gz sim -s --headless-rendering`), with no GUI.

---

## 1. Real-time factor, cameras on vs off

> **Correction (later the same day).** The first version of this section
> concluded "cameras cost nothing measurable". That was wrong: in those runs
> nothing subscribed to the camera topic (`bridge_camera:=false`), and **Gazebo
> only renders a camera that has a subscriber**, so no camera was rendering in
> any of variants A–D. The table below stands as a measurement of *non-rendering*
> configurations only. The rendering measurements that replace its conclusion
> follow it.

`bash scripts/measure_perf.sh 30`: full stack, episode-1 scene, arm idle, RTF
sampled once a second for 30 s (n=27 per row), **camera not subscribed**:

| Variant | RTF mean | min | max |
|---|---|---|---|
| A — table camera 1280×960 @ 10 Hz + robot `synth_camera` 320×240 @ 10 Hz | 0.193 | 0.132 | 0.215 |
| B — table camera sensor removed from the world | 0.196 | 0.107 | 0.217 |
| C — table camera reduced to 320×240 | 0.198 | 0.168 | 0.213 |
| D — table camera removed + `gz-sim-sensors-system` removed from the robot URDF | 0.196 | 0.142 | 0.220 |

**With the camera actually rendering** (`bridge_camera:=true`, so the camera
has a subscriber), 2 ms physics step, table camera 320×240 @ 30 Hz, RTF mean of
10 samples per row:

| Configuration | RTF |
|---|---|
| no subscriber, no scene (camera idle) | 0.412 |
| subscribed, no scene, shadows on, software GL | 0.088 |
| no subscriber, episode-1 scene spawned | 0.393 |
| subscribed + scene, shadows on, software GL | **0.011** |
| subscribed + scene, shadows on, **GPU** (WSL D3D12) | **0.006** |
| subscribed + scene, **shadows off**, GPU | 0.144 |
| subscribed + scene, **shadows off, software GL (llvmpipe)** | **0.304** ← adopted |

- **Shadow rendering was the cost.** With the sun's `cast_shadows` on, a
  rendering 30 Hz camera took the RTF to 0.006–0.011, i.e. ~100 s of wall clock
  per simulated second.
- **On the GPU path, every dynamic object renders white.** Every object spawned
  at runtime that is not `static` (all four sorting objects, and test cubes with
  plain-colour, no-specular, PBR or **texture-mapped** materials) appeared white
  in the camera; static ones (the bins, a static copy of the red cube) were
  correctly coloured. In software (llvmpipe, OpenGL 4.5) all colours are
  correct. A colour-sorting dataset can't be recorded white, so episodes render
  in software, which is also the faster path here once shadows are off.
- Adopted: software OpenGL + sun `cast_shadows false` + 2 ms step + camera
  320×240 @ 30 Hz. Verified on `repro_harness.py --camera --attach`, 2/2 PASS,
  RTF 0.21–0.40.

What costs without rendering, isolated (`gz sim` server, RTF over 15 s):

| Configuration | RTF | Gazebo server main thread |
|---|---|---|
| Bare world (table, markers, camera), no robot, no ROS | **1.000** | 14 % CPU |
| Robot spawned, **without** `gz_ros2_control` | **0.220** | — |
| Full stack (robot + `gz_ros2_control` + 3 controllers), arm idle | **0.206** | **96.8 % CPU** |
| Full stack with `max_step_size` 0.002 instead of 0.001 | **0.413** | — |

The slowdown comes from **simulating the robot itself**: 0.220 without
ros2_control, against 1.000 for the bench alone. `gz_ros2_control` adds about
7 % on top. All 14 of the robot's collision shapes are full DAE triangle meshes
(7 arm links, 7 gripper links; counted in
`mycobot_pro_320_pi_gazebo.urdf`). Halving the number of physics steps per
simulated second doubles the RTF (0.413), which confirms the cost is per
physics step.

> **Root cause of the low RTF, and recommendation for the next iteration
> (not done in this one):** the robot's **14 triangle-mesh collision shapes**.
> Evidence: the bench without the robot runs at RTF 1.000; adding the robot
> without any controller drops it to 0.220; and halving the number of physics
> steps (`max_step_size` 0.001 → 0.002) doubles it to 0.413, so the cost scales
> with physics steps, i.e. with collision checking of those meshes. Replacing
> them with primitive shapes (boxes and cylinders sized to each link, keeping
> the meshes for visuals) is the expected large win. It changes the robot's
> contact geometry, so it needs its own validation (grasp, reach, the harness)
> before any data is recorded with it.

**Adopted for this iteration: `max_step_size` 0.002.** Validated on
`repro_harness.py --standalone --attach`, 3/3 PASS: every joint within 1.36°
of the goal (identical values each run), the welded cube rising 9.0 cm with
`dz` constant to 2 mm, RTF 0.36–0.44.

## 2. CPU usage per core during a run

Same runs as §1, from `/proc/stat` deltas over 30 s (8 logical CPUs):

| Variant | Per-core busy % | Mean |
|---|---|---|
| A | 34, 26, 19, 20, 17, 19, 17, 17 | 21 % |
| B | 35, 29, 18, 20, 14, 20, 18, 20 | 22 % |
| C | 37, 30, 21, 18, 17, 16, 18, 19 | 22 % |
| D | 36, 27, 15, 21, 14, 22, 15, 19 | 21 % |

`ps` shows `gz sim` at 106–110 % CPU. Per thread (`top -H` on the server
process, full stack): **the main thread is at 96.8 %**; the next busiest
threads are at 2.0–2.4 % (one is Fast DDS's shared-memory thread).

**Answer: one thread is pinned at 100 %, and that thread is the simulation
loop.** The per-core view doesn't show it as one hot core because the scheduler
moves the thread between cores. Overall the machine is ~79 % idle: the
simulation is single-thread bound, not CPU-capacity bound. That is why §7
helps.

## 3. Breakdown of one episode, in seconds

`bash scripts/episode.sh 1`: the full episode including the recorder,
2026-10-03 09:50:54, verdict **PASS**. Timestamps from
`/workspace/htgspp/stage_001.log` and the per-phase `t_wall_s` column of
`episodes/ep_001/grasp_log.csv`.

| Stage | Wall s | Share |
|---|---|---|
| Kill previous stack, clear `/dev/shm`, `ros2 daemon stop` | 3 | 1 % |
| Simulator startup → all 3 controllers `active` | 14 | 6 % |
| Spawn 4 bins + 4 objects (8 × `ros_gz_sim create`) + settle 3.0 s sim | 25 | 11 % |
| Recorder start + `configure` (includes a fixed 8 s sleep) | 40 | 17 % |
| Recorder `activate` | 31 | 13 % |
| **Discovery wait** in `run_pick_and_place.py` (`wait_ready`: action server + `/joint_states` + gripper subscriber) | **0.8** | <1 % |
| **Simulated motion**, 10 phases, home → home (~21 s simulated at RTF ≈ 0.18) | **120** | 51 % |
| Verify + teardown (`kill -9` of the stack) | <1 | <1 % |
| **Total** | **234** | |

**The episode takes 3.9 min, not the 7–8 min estimated earlier.** Half of it is
motion. **The recorder lifecycle bring-up is the largest overhead (71 s,
30 %)**: two `ros2 lifecycle set --no-daemon --spin-time 15` CLI calls, plus the
fixed sleep. The "recorder configure hang" recorded in `doc/HANDOVER.md` did not
occur; both transitions succeeded on the first attempt.

Per-phase motion (wall s): home 6.9 · approach 12.2 · descend 12.2 ·
close + attach 10.8 · lift 12.4 · transport 21.9 · release-descend 12.8 ·
open + detach 9.8 · retreat 11.5 · home 9.8.

**Recorder fix (adopted).** The 71 s was the CLI, not the recorder: its
`change_state` service is reachable ~1 s after the process starts, and each
`ros2 lifecycle set --no-daemon --spin-time 15` call took ~31 s.
`scripts/recorder_lifecycle.py` waits on the service itself and calls it
directly: **bring-up 71 s → ~1 s** (configure 0.3–0.9 s, activate < 0.1 s),
on every episode since.

> **Known inefficiency, for the next iteration (not changed in this one):
> controller-spawner race at bring-up.** In batch episode 14 (2026-10-03,
> 434 s against ~150–190 s), the `joint_state_broadcaster` spawner died at
> startup (`Switch controller timed out after 5 seconds!` → `Failed to activate
> controller` → `process has died`). `episode.sh` only falls back to
> sequential respawn after its full 30-poll loop, ~4.5 min later. The fallback
> worked and the recording was clean, so this costs time, not correctness.
> Proposed fix: respawn the missing controllers **as soon as a spawner
> process dies** (the loop already detects it, logging "a bringup process
> died"), instead of waiting out the loop. Deferred because bring-up is the
> most fragile part of the pipeline and the current fallback is proven.

## 4. Camera sensors in the scene vs what the dataset needs

> **Update (2026-10-03, later the same day).** Superseded for the dataset: the
> table camera now renders at **320×240 @ 30 Hz** (frames complete in every
> episode since, `check_frames.py`), and the held-out split no longer uses
> `synth_camera_right`: both splits record `/camera/image_raw`, the held-out
> condition being a reserved camera configuration (`heldout_oblique`,
> `scripts/make_episode_world.py`). The table below is the state measured
> before that change.

| Sensor | Where | Resolution | Rate | `always_on` |
|---|---|---|---|---|
| `table_camera` → `/camera/image_raw` | world (`sorting_table.sdf`) | **1280×960** | 10 Hz | true |
| `synth_camera` → `synth_camera/image` | robot URDF | 320×240 | 10 Hz | true |
| `synth_camera_right` → `synth_camera_right/image` | robot URDF | 320×240 | 10 Hz | false (renders only when subscribed) |
| `synth_camera_left`, `synth_camera_top` | robot URDF | 320×240 | 10 Hz | false |

Rates are in simulated time. The robot's cameras come from the
`Gazebo_to_LeRobot_Pipeline` override URDF, which lowered them from 640×480.

**What the dataset requires** (`contracts/mycobot_sorting.yaml`,
`mycobot_sorting_heldout.yaml`): `fps: 30`; one image stream per split
(`/camera/image_raw` for train, `/synth_camera_right/image` for held-out),
**resized to 224×224**; plus `/joint_states` (6 arm joints + gripper) as both
state and action.

**Measured consequence (episode 1 bag):** 119.3 s, 2,111 messages:
**60 images** plus 2,051 joint states, **213 MB**. About 21 simulated seconds
at 10 Hz should give ~210 images; 60 arrived. Each 1280×960 RGB frame is
3.7 MB, sent `best_effort`, so most are dropped in transport. The contract's
30 fps is not met: the source renders at 10 Hz, and only about 29 % of those
frames reach the bag. Since §1 shows rendering resolution has no RTF cost, a
320×240 table camera (still ≥ 224×224) would cut each frame 16× and should
stop the drops. **Not applied:** it changes the dataset, so it's an owner's
decision.

## 5. Shared memory

```
$ df -h /dev/shm                       (inside the container)
shm   64M  2.7M   62M   5% /dev/shm
$ docker inspect gazebo_to_lerobot --format '{{.HostConfig.ShmSize}}'
67108864                                (= 64 MiB, Docker's default)
```

`docker/run.sh` in `Gazebo_to_LeRobot_Pipeline` does **not** set `--shm-size`
on this checkout; the 2 GB raise described in `doc/HANDOVER.md` is not in the
current file. Peak use observed: **12 MB** of 64 with three full stacks running
at once (§7). `episode.sh` clears leftover Fast DDS segments at the start and
end of every episode.

## 6. Gazebo physics configuration

> **Update (2026-10-03).** `max_step_size` is now **0.002** (§1, adopted), and
> the sun's `cast_shadows` is false. The block below is the configuration as
> first measured.

From `worlds/sorting_table.sdf` (unchanged from `real_table.sdf`):

```xml
<physics name="default_physics" type="ode">
  <max_step_size>0.001</max_step_size>
  <real_time_factor>1.0</real_time_factor>
</physics>
```

- `max_step_size`: **0.001 s** (1,000 physics steps per simulated second).
- `real_time_update_rate`: **not set** (SDF default).
- Solver iterations: **not set**; no `<ode><solver>` block, engine defaults.
- **Engine actually used: DART** (`gz-physics` dartsim plugin).
  `type="ode"` does not select ODE in Gazebo Harmonic: the installed engine
  plugins are dartsim, bullet, bullet-featherstone and tpe, with no ODE.
- Controller loop: `controller.yaml` `update_rate: 100` (override; the team's
  file says 500). `gz_ros2_control` warns that its 0.01 s period is slower
  than the 0.001 s physics step, which is expected. `position_proportional_gain`
  is 0.1 (logged at startup).

## 7. Three episodes concurrently

`scripts/repro_harness.py --standalone --attach`, three copies started at the
same moment, each with its own `ROS_DOMAIN_ID` and `GZ_PARTITION`.

| Run | Instance results | Wall s each | RTF per instance |
|---|---|---|---|
| Alone (reference) | PASS | 36 | 0.19–0.22 |
| 3 concurrent, 1st try (domains 11/12/13) | PASS, PASS, **FAIL** (`no /joint_states` within 60 s) | 53, 47, 76 | 0.11–0.18 |
| 3 concurrent, 2nd try (domains 21/22/23) | PASS, PASS, PASS | 56, 52, 54 | — |

- **Memory:** +2.1 GB for three stacks (used 993 → 3,088 MB; 4.75 GB still available).
- **CPU:** 41 % user overall. **Shared memory:** peak 12 MB.
- **Throughput:** three runs in ~55 s against one in 36 s, **≈ 2× the episodes per hour**.
- **What broke:** one instance in six never received `/joint_states`, though its
  trajectory action server was up: a discovery race during three simultaneous
  startups. A longer `wait_ready` timeout plus one retry should absorb it.

**What would break with `episode.sh` as written** (not run concurrently; read
from the script). It is **not** concurrency-safe:
1. `kill_stack` uses `pkill -f` on global patterns (`ros2 launch`, `gz sim`,
   `episode_recorder_node`…): each episode's restart would kill the other
   episodes' simulators.
2. `rm -rf /dev/shm/fastrtps_*` deletes the other stacks' live Fast DDS segments.
3. `ros2 daemon stop` stops the daemon the other stacks share.
4. The `/episode_recorder` node name and `/workspace/htgspp/bags` are shared:
   safe only because domains separate the nodes and bag folders are
   timestamped.

Running the batch three-wide therefore needs a per-instance process group
(kill only your own children), per-instance shm cleanup, and no daemon stop;
`repro_harness.py` already works this way.

## 8. Reachability: a simulation collision defect in folded lift poses

**Finding.** In the 2026-10-03 60-episode batch (3 code versions, not used
for the dataset), 6 episodes failed both attempts (20, 21, 35, 36, 39, 44),
all at the LIFT, all with the object welded, and only when the lift pose
folds the arm (high shoulder J2): `ARM_NOT_CONVERGED`, joints 6.2–6.5° short
after a re-send. Every episode whose lift pose has J2 ≤ 48.0° passed, for all
four objects:

| Object (mass) | Lift J2 tested | Outcome |
|---|---|---|
| red_cube (50 g) | −15.7 … −1.3° | all pass |
| yellow_box (50 g) | 30.0 … 42.5° | all pass |
| blue_cube (60 g) | 34.2 … 53.7° | pass ≤ 48.0°; 49.7° fails or retry-passes; 53.7° fails |
| green_cylinder (50 g) | 38.9 … 61.4° | pass ≤ 53.4°; 53.6°, 58.3°, 61.4° fail |

**Evidence: steady-state error at the lift pose**, `repro_harness.py
--standalone --episode N --lift-only | --attach`, settling the full window
(tolerance 0), per-joint error and the tool-tip offset it causes (forward
kinematics of achieved vs commanded joints):

| Lift pose | Unloaded | Loaded (object welded) |
|---|---|---|
| ep 16 (J2 45.8°) | 0.0° all joints, 0 mm | 0.0° all joints, 0 mm |
| ep 20 (J2 53.7°) | 0.0° all joints, 0 mm | **J3 −3.37°, J4 +1.13° → 8.2 mm** (dz −7.4 mm) |
| ep 35 (J2 61.4°) | 0.0° all joints, 0 mm | **J3 −5.99° → 16.9 mm** (dz −11.0 mm) |

**Mass test** at ep 20's lift pose, blue_cube mass changed and nothing else:
**1 g → J3 −3.37°**, 60 g (real) → −3.59°, 120 g → −3.63°; tip offset
8.7–9.6 mm throughout.

**Conclusion: this is a simulation collision defect, NOT gravity sag.**
- **Zero error unloaded**, at the very poses that fail loaded. Sag would be
  present unloaded too.
- **The error is on J3 (elbow), not J2.** The real arm's measured sag is a
  ~1.9° permanent offset on **J2**, ~13 mm unloaded / ~15 mm loaded
  (`DEVELOPMENT_SUMMARY.md`): a different joint, present unloaded.
- **Mass-insensitive**: a 120-fold mass change moves the error by 0.26°.
  Gravity would scale with mass.
So in folded lift poses the welded object is physically obstructed, most
likely by the robot's own links (not yet confirmed from contact data). Gains
were not touched (raising them was tried on 28/08 and withdrawn).

**Validated operating region: lift J2 ≤ 48.0°.** Adopted for the dataset by
moving object start positions (`scripts/make_matrix.py`, 2026-10-04):

| Object | Base before | Base after | Worst / second-worst variant lift J2 |
|---|---|---|---|
| red_cube | (0.22, −0.08) | unchanged | −1.3° / −4.9° |
| blue_cube | (0.10, −0.06) | **(0.120, −0.06)** | 45.8° (x −20 mm) / 38.0° |
| green_cylinder | (0.10, +0.02) | **(0.135, +0.02)** | 45.7° (x −20 mm) / 36.8° |
| yellow_box | (0.10, +0.09) | unchanged | 42.5° / 42.4° |

> **How these bases were found, and what they are not.** A full search over
> x and y with 12 IK seeds per variant exceeded the 30-minute time limit
> before reporting anything. The bases above come from an **x-only outward
> search** in 5 mm steps, with **6 IK seeds** per variant, stopping at the
> first x where all nine variants (held-out included) solve at ≤ 46°, then
> **re-verified with 12 seeds**. They are **working positions, not nearest or
> optimal ones**: y was never searched, and for blue and green the margin is
> a single variant (x −20 mm) at 45.7–45.8°, with the rest in the 20s–30s.
> Spawn drift does not move the lift J2 (the lift is a precomputed joint
> waypoint). Target spawn drift measured at the OLD positions (60 episodes):
> median 0.00 mm, max 0.33 mm; re-measured in the new batch, see below.

**The margin is stronger than a threshold.** Both worst cases sit inside
their object's OWN measured passing region, not just under the global 48°
line. Blue's worst variant, (0.100, −0.060) at 45.8°, is exactly episode 16's
pose: it passed in every run, and the harness measured it at 0.0° on every
joint, loaded and unloaded. Green's worst, 45.7°, is 7.7° inside green's own
measured ceiling (53.4°, episode 40, passed).

**The binding variant is the innermost one: the variant closest to the
robot base.** Lift J2 rises monotonically as an object comes inward. For red,
blue and green that is always x −20 mm. **For yellow_box, x −20 mm (42.5°)
and y −20 mm (42.4°) are tied**: yellow sits at y = +0.09, so −20 mm in y
brings it as close to the base axis as −20 mm in x. A future position change
therefore needs only the innermost variant(s) solved to know its ceiling,
not all nine: the shortcut that would have spared the search that timed
out. The outward search above also moved the OUTER variants into territory
no batch had exercised (green up to x 0.155, blue up to 0.140). Their lift
J2 is low (23.9°, 26.5°), so the J2 check cannot flag them; they were
checked separately (camera framing, table and keep-out clearance, place
reachability; below).

**Next iteration (not done in this one)**, next to the collision-mesh item in
§1: identify the obstructing contact from Gazebo contact data at a folded
lift pose (e.g. episode 20's matrix position), then either lift along a less
folded path or stop the welded object colliding with the robot's own links.
That would lift the J2 ≤ 48° restriction. Filed in `doc/LIMITATIONS.md`,
"Open items for the next iteration".

## 9. Bins: 80 mm opening, blue bin moved 10 mm, boxes released square (2026-10-04)

**Bins are axis-aligned squares: never model them as circles.** Three bugs
on one morning came from doing so: the red–blue wall gap from Euclidean centre
distance instead of max(|dx|, |dy|); the place-path check using circumscribed
circles, overstating a bin's reach by 41 % along the diagonal; the distractor
keep-out as a circle around the bin centre, missing the corners.

**Why the bins changed: physical interpenetration of the red and green bin
walls (15 mm) causing tilted landings, plus lost episode yield from false
`WRONG_BIN` labels. Not label corruption.** With the original 100 mm bins
(95 mm opening, 5 mm walls) the red bin's wall stood 2 cm inside the blue bin
and the green bin's wall 15 mm inside the red one. The verifier tests the wrong
bin first, so an object landing in the overlap was labelled `WRONG_BIN`: a
false FAIL, never a false PASS. No reachable layout of four 100 mm bins exists
in this workspace (searched against the IK-reachable region and the board).

**The batch run before this change (2026-10-03, 54/60 PASS) had the overlap.
It is NOT a baseline** for the batch that follows: different bins, a different
blue bin position, different distractor layouts.

| Attempt | Opening / wall / outer | Blue bin | Wall gaps (red–blue, red–green) |
|---|---|---|---|
| original | 95 / 5 / 105 mm | (0.24, 0.02) | −20, −15 mm (overlap) |
| option 1 | 70 / 3 / 76 mm | (0.24, 0.02) | 4.0, 14.0 mm |
| **option 2 (adopted)** | **80 / 3 / 86 mm** | **(0.24, 0.01)** | **4.0, 4.0 mm** |

Every other pair is ≥ 94 mm apart. `objects.yaml` `inner_radius` = 0.040.

**Blue placement probe** (`scripts/bin_probe.py`, episodes 20 / 16 / 23, the
x −20 / nominal / x +20 variants; full pick and place, then the cube's pose
2, 5 and 8 simulated seconds after the arm is home):

| Ep | Opening | Offset from bin centre | Yaw | Footprint margin | Tilt | Drift (6 s) |
|---|---|---|---|---|---|---|
| 20 | 70 mm | −7.5, +8.6 mm | not measured | — | 0.00° | 0.00 mm |
| 16 | 70 mm | +3.0, +4.5 mm | not measured | — | **28.47°** | 0.00 mm |
| 23 | 70 mm | +2.8, +4.1 mm | not measured | — | **27.58°** | 0.00 mm |
| 20 | 80 mm | −3.6, −2.9 mm | +15.2° | 5.7 mm | 0.00° | 0.00 mm |
| 16 | 80 mm | −1.7, −3.2 mm | −45.0° | 1.4 mm | 0.00° | 0.00 mm |
| 23 | 80 mm | −2.5, −2.6 mm | −30.0° | 3.3 mm | 0.00° | 0.00 mm |

The 70 mm opening failed: two of three cubes came to rest propped on the rim
at 28° from level (centre z 43 mm instead of 27 mm), stable, not toppling.
**A margin check alone would have shipped propped cubes:** episodes 16 and 23
read 5.5 and 5.9 mm of footprint margin under the axis-aligned margin the
probe computed then, above the 5 mm gate. Only the tilt condition caught them.

**The 80 mm opening failed the 5 mm footprint gate** (episodes 16 and 23:
1.4 and 3.3 mm), though all three cubes landed flat and still.

**The gate was then relaxed, wrongly, and a batch started on it.** The
footprint threshold was lowered from > 5 mm to > 0 mm after seeing those
numbers, and the 60-episode batch started (code `e719eb5603ac`). It was stopped
after 2 episodes (both PASS) and archived in the container as
`archive_2026-10-04_option2_stopped/`; it is not the dataset. The reasoning
("every footprint is inside, every cube is flat") was coherent and still
wrong: **a margin is not a statement about the probed episode, it is a
statement about the episodes that were NOT probed.** Three probes clearing by
1.4–5.7 mm mean the population of 60 spreads below zero. "This episode passed"
and "this configuration is safe" are different claims; the gate measures the
second. A gate that fails is reported with a proposed fix, never relaxed and
passed through.

**Why the cube lands yawed: the jaws square it to the gripper.** The landed
yaw is not the spawn yaw (0) plus the J1 swing; it is the gripper's WORLD yaw
at release, mod 90°. Predicted from the waypoints, against the 80 mm probe:

| Ep | Gripper yaw at pick → release | Predicted (spawn yaw carried) | Predicted (release yaw mod 90) | Measured |
|---|---|---|---|---|
| 20 | 60° → 105° | +45° | +15° | +15.2° |
| 16 | 165° → 135° | −30° | −45° | −45.0° |
| 23 | 165° → 150° | −15° | −30° | −30.0° |

Closing two jaws on a cube turns it square to them; the weld then holds it at
the gripper's angle. **Consequence, which closes the bin question for good:**
with a free release yaw the worst case is 45°, where a 50 mm cube spans
50·√2 = 70.7 mm. Clearing 5 mm with a 4.5 mm landing offset then needs a
44.9 mm half-opening: a 90 mm opening, 96 mm outer, and four bins at 96 mm
outer have NEGATIVE wall gaps in this layout. No bin geometry could have
worked. Fixing the release yaw was not the better option, it was the only one.

**The fix: boxes are released square to the bins** (`scripts/precompute_ik.py`).
The place column is solved with the gripper's world yaw constrained, among the
landings that are equivalent for the shape, the least wrist travel winning:
0/90/180/270° for the cubes (square footprint); for the yellow box (50 × 30 mm,
not square) only its fixed grasp angle and that + 180° (90/270°), since + 90°
would swap its long axis from x to y; the cylinder stays free. Offering only
0/90° first spun the wrist up to 189.5° in transport; with all four the worst
J6 travel from lift to over-bin is 42.3° (red) and 25.0° (blue).

- Solvability: 60/60, branch check OK, place path clear by ≥ 57.5 mm.
- Wrist limits: worst J6 limit margin over the place leg 5.2° (episode 35,
  green, whose yaw is unconstrained and unchanged); worst over any joint and
  waypoint 3.5° (red episodes 11 and 15, at the pick, unchanged by the fix).
- Cost: the full 60-episode precompute takes 670 s (602 s with free release
  yaw).
- Spawn yaw is 0 for all four objects: the models carry no `<pose>` and
  `spawn_scene.py` passes no yaw. With the jaws squaring the cube, that matters
  less than assumed: the landed yaw follows the release.
- Release yaw chosen (gripper world yaw): red 90° ×11, 180° ×4; blue 90° ×1,
  180° ×14; yellow 90° ×15; green (free) 60° ×4, 75° ×10, 150° ×1. Each box
  lands square to its bin in every episode (`doc/LIMITATIONS.md`).

**Probe with the fix, gated on the original three conditions** (rotated
footprint margin > 5 mm, tilt < 3°, drift < 1 mm and < 0.5°), inner and outer
x variant of each box; green as an ungated spot-check:

| Ep | Object | Offset from bin centre | Yaw | Footprint margin | Tilt | Drift (6 s) |
|---|---|---|---|---|---|---|
| 5 | red, x −20 | +0.1, −0.7 mm | −0.3° | 19.2 mm | 0.00° | 0.00 mm |
| 8 | red, x +20 | −0.2, +2.6 mm | +15.3° | 12.8 mm | 0.00° | 0.00 mm |
| 20 | blue, x −20 | −0.9, −3.4 mm | +0.6° | 11.4 mm | 0.00° | 0.00 mm |
| 23 | blue, x +20 | +0.9, −3.4 mm | −0.0° | 11.6 mm | 0.00° | 0.00 mm |
| 50 | yellow, x −20 | 0.0, 0.0 mm | +0.0° | 15.0 mm | 0.00° | 0.00 mm |
| 53 | yellow, x +20 | 0.0, 0.0 mm | −0.0° | 15.0 mm | 0.00° | 0.00 mm |
| 35 | green, x −20 (not gated) | −0.6, +0.3 mm | +44.9° | 17.4 mm | 0.00° | 0.00 mm |

All six pass. Two readings checked against `run_pick_and_place.py`'s own
per-phase pose log: yellow's 0.0 mm is real (carried 0.9 mm from the bin
centre, dropped straight; its fixed 90° grasp closes across the 30 mm side
without turning it). Red episode 8 landed 15.3° off square: closing the jaws
pushed the cube 2.4 mm and did not fully square it, so the squaring model is
not exact for every grasp. Red is safe at any yaw (a 40 mm cube at 45° spans
56.6 mm: 40 − 28.3 − 2.6 = 9.1 mm); the blue cube, where yaw matters, landed
at 0.6° and 0.0°.

**Distractor keep-out made square** (`scripts/make_matrix.py`,
`clears_bins`). `layout.py`'s `bin_keepout_radius` is a 70 mm circle around
each bin centre and misses the corners: with 86 mm bins it let episode 32's
blue cube sit 14 mm inside the blue bin. Every footprint, targets included,
must now clear every bin's outer square by `BIN_CLEAR` = 5 mm. **5 mm is a
chosen value, not a derived safety margin:** it is fitted under red's existing
y +20 target (0.22, −0.06), 7.0 mm from the blue bin, with one precedent: at
7.5 mm from that wall (100 mm bins) its episode 12 passed on 2026-10-03. Matrix regenerated
(md5 `25189132741ef47d7ed7a921c91f65bf`): targets and splits identical,
distractors re-drawn; sampler acceptance 14.6 % (6.8 draws per layout).
Max lift J2 45.77° (episode 20, unchanged). Place path (`check_place_path.py`, bins as real
squares): all 60 clear, minimum 57.5 mm. Verifier self-test 15/15.

**Weld race: the root cause of held-out episode 13's stuck weld
(2026-10-03).** gz-sim 8's DetachableJoint subscribes to its detach topic as
soon as the model loads but creates the joint a moment later. A release
landing in between is ignored and the object is THEN welded, pinning the arm.
`scripts/weld.py` now waits for the plugin's `attached`/`detached` report and
re-publishes when it does not come: on the base probe it needed **2
publishes, once**.

**Spawn drift** at the new positions, re-measured in the batch (§10): 0.00 mm
median and maximum, for 60 targets and 180 other objects.

**Fit is proven for all 60 episodes by landed height, without landed yaw.**
The batch recorded the landed position, not the orientation, so the
rotated-footprint margin cannot be computed per episode. It does not need to
be: an object too wide for the opening cannot land flat at its expected
height. It props on the rim, and propping raised the centre from 27 mm to
43 mm in the 70 mm probe. All 60 targets landed at exactly their expected
height (red and yellow 22.0 mm, blue and green 27.0 mm, the bin floor plus
half the object), so no episode had a fit problem, whatever its yaw. The
squaring model itself is still not exact (probe episode 8 landed 15.3° off
square); height closes the fit question, not the yaw one. From the next batch
on, landed yaw and tilt are recorded in every episode's metadata
(`run_pick_and_place.py`: `landed_quat_xyzw`, `landed_yaw_deg`,
`landed_tilt_deg`), so the margin becomes a measurement instead of an
inference.

## 10. The 60-episode batch (2026-10-04, code `f977d7af65dc`)

One code version from the first episode to the last, started
2026-10-04T08:37:11Z, finished about 11:20Z, container `c76def923a51`.

| | |
|---|---|
| Passed | **60 / 60**, all on the first attempt (no retries) |
| Trajectory re-sends | 0 in every episode |
| Circuit breaker | did not fire |
| Camera frames | complete in all 60; received exceeds expected by 2–15; largest gap 1.02 frames |
| RTF | 0.100–0.393, median 0.344 (episode 27 lowest) |
| Wall time per episode (`batch_results.csv`) | median 160 s |
| of which the pick-and-place sequence (`wall_seconds`) | 104–144 s, median 122 s |
| Spawn drift | 0.00 mm (60 targets, 180 other objects) |
| Landing offset, max(\|dx\|, \|dy\|) from bin centre | red median 2.5 / max 4.7 mm; blue 3.4 / 5.2; green 0.5 / 2.9; yellow 0.0 / 0.0 |
| Other objects disturbed | 1 episode (12), below |

**Not comparable with the 2026-10-03 batch** (54/60, §9): different bins, blue
bin position, distractor layouts and release yaw. **Speed, measured the same
way on both batches** (medians over the 60 episodes on disk): RTF 0.292 →
0.344, sequence wall time 122 → 122 s, episode wall time 165 → 160 s. There
is no real speed-up: the faster simulation step did not shorten the
sequence. Earlier figures of "0.164 → 0.344" and "~175 → ~124 s" mixed
quantities (the 124 s was the sequence alone, not the episode) and are
withdrawn. The RTF difference itself is most likely host load or thermal
state, not the shorter wrist travel: RTF measures the cost of a simulation
step, which a shorter joint path does not reduce, and RTF on this laptop has
already been seen falling 0.41 → 0.16 across five consecutive episodes of one
run (`scripts/episode.sh`, watchdog comment).

**Episode 12: a disturbed object, kept and labelled.** Target red cube at
(0.22, −0.06); the green cylinder, 82 mm away at (0.286, −0.109), was pushed
22.9 mm, by (+20.6, +10.0) mm, away from the gripper. **Cause, verified from
the recording:** the open gripper jaw, on the last 12 mm of the descent to
grasp height. In the dataset camera's frames the cylinder's centroid holds at
(194.8, 124.1) px while the tool tip comes down from 170 mm to 31 mm, jumps to
(192.9, 118.7) px between frames 165 and 175 as the tip goes from 31 mm to
19 mm, and does not move again; the logged end pose projects to
(192.9, 119.2) px. The fingers are open during the descent (they close to
0.8 rad only at the grasp), and the jaw is the only part of the robot over
the cylinder in those frames. The red cube was shoved 8.9 mm the other way on
the same descent.

The episode is kept: it succeeded at its stated task, the task was never
specified as "without touching anything else", and a labelled collision in a
60-episode smoke test is more honest than a silently removed one. It is kept
**because the flag survives conversion**: `port_bag.py` writes
`distractors_moved` into `meta/episodes.jsonl` for every episode (empty when
nothing moved); episode 12 is training episode 11 there, with
`"source_episode": 12, "distractors_moved": [["green_cylinder", 0.0229]]`.
The verifier's rule (a disturbed object is flagged, not failed) is unchanged.

**Converted** (2026-10-05, LeRobot v3.0 format, lerobot 0.4.4, provisional
pin): `mycobot_sorting_train` 48 episodes, 29 687 frames;
`mycobot_sorting_heldout` 12 episodes, 7 411 frames; 320 × 240 at 30 fps.
One frame fewer per episode than recorded: `action[t] = state[t+1]` (no
command topic was recorded), so each episode's last frame is dropped. Loaded
by the real `LeRobotDataset` and smoke-trained with SmolVLA on CPU; checks
and numbers in `datasets/README.md`. **Correction, 2026-10-05:** this
paragraph first gave 29 735 / 7 423 frames, from a hand-written conversion of
2026-10-04 that did not load and was replaced. Why: action one frame early
(`action[t] = state[t]`), wall-clock timestamps, a different camera key per
split, and no statistics.


## Also observed

- **Zombie processes:** 62 `<defunct>` `ruby` processes after a day of runs.
  The container's PID 1 is `sleep infinity`, which never reaps them. They hold
  no memory, but over a 60-episode batch they accumulate PIDs. `docker run
  --init` (or restarting the container) clears them.
- **Trajectory "success" is not arrival:** the `mycobot_controller` action
  returns `SUCCESSFUL` when the trajectory's time is up, with the arm measured
  **2.5–8.2°** short (no goal tolerances, `goal_time: 0`). It converges about
  1 s later; both the harness and `run_pick_and_place.py` wait for convergence.
