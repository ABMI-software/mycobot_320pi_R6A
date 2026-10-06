# Part 8 smoke test — result: not achieved on this container; the failure mode itself is the finding

## What was attempted

One episode (`ep001`, `red_cube`, `real_table.sdf`, all 8 objects + 4
ArUco markers + table camera + wood-table mesh present, matching the
chosen world exactly): bring up the full launch stack, wait for the three
controllers, spawn the scene, run the 7-waypoint sequence, verify. Per
Part 8's own scope note, this was always going to be at most a partial
attempt at Part 8's real target ("four smoke-test episodes, one per
object, cylinder included") given the time already spent on Parts 1–7
this session — but even the first, single attempt did not complete.

## What happened

Two consecutive bring-up attempts on the same long-running `gazebo_to_lerobot`
container, both following the exact respawn procedure that worked
repeatedly earlier in this session (Part 3, and the Part 6/7 verification
runs) on the *simpler* `pick_and_place_sorting.sdf` world:

1. **Attempt 1**: all three controllers (`joint_state_broadcaster`,
   `mycobot_controller`, `gripper_position_controller`) failed to
   activate within the default 5-second switch timeout — the same
   symptom as earlier, expected to resolve with a manual respawn using a
   longer `--switch-timeout 150`, as it had every previous time this
   session.
2. **Attempt 2 (the respawn itself)**: did not resolve it. Roughly 20
   minutes into this specific Gazebo instance's uptime, the respawn
   commands themselves started failing with:
   ```
   [RTPS_TRANSPORT_SHM Error] Failed init_port fastrtps_port7001: open_and_lock_file failed
   [WARN] Could not contact service /controller_manager/list_controllers
   failed to check service availability: rcl node's context is invalid
   ```
   This is not "the controller is slow to activate" (Part 3's finding).
   This is the DDS transport layer itself failing to reach the
   `controller_manager` service at all. All three spawner processes ended
   as zombies (`[spawner] <defunct>`). Container memory had climbed to
   2.1/3.8 GB used, 394 MB genuinely free (1.8 GB "available" only via
   reclaimable cache) — the CPU-bound `gz sim` process itself had
   accumulated 27m34s of CPU time against ~21 minutes of wall-clock
   uptime (133% CPU, i.e. saturating more than one core continuously).

## What this means, and what it doesn't

**It doesn't reopen the grasp-mechanism question.** No grasp was
attempted in either try — the failure is entirely in scene bring-up,
before `run_pick_and_place.py` ever ran. Part 3's finding (the reference
grasp mechanism is genuine; two live attempts failed for a settle-timing
reason) stands unchanged and unrelated to this.

**It is new, independent, and stronger evidence for A10.** Part 3's
resource-pressure symptom (a 5-second controller-activation timeout,
resolved every time by one respawn) was observed on `pick_and_place_sorting.sdf`
— a simpler world with no camera, no ArUco markers, no textured mesh.
`real_table.sdf` is the chosen world specifically *because* of that
extra content (Part 2's decision criteria: ArUco markers, table camera,
textured table) — and that same extra content is presumably why this
container's resource ceiling is reached faster and more severely here:
not a slow activation this time, but the DDS transport itself failing
after ~20 minutes, in a way a respawn cannot fix because the underlying
service is unreachable, not merely slow.

**The specification's own A10 argument predicted exactly this class of
outcome** for Role A hardware on the *previous* (simpler, single-object)
acquisition. Finding it reproduced, independently, and more severely, on
this richer world, on this same container, is not a surprise — it is
confirmation, gathered rather than assumed.

## What remains unproven

- No episode — not `ep001`, not any of the 60 — has been recorded end to
  end on this hardware. `run_pick_and_place.py`, `spawn_scene.py`, and
  `verify_episode.py` are written, and their logic is exercised
  separately and verified (the attach mechanism live, per Part 3.6; the
  verifier's own correctness, per Part 9.5's synthetic test cases; the
  waypoints, per the offline IK checks in `doc/WAYPOINT_CONTINUITY.md`)
  — but the three have never run together against a live scene.
- Whether `real_table.sdf`'s specific memory/CPU footprint is survivable
  at all on Role A hardware for a *single* episode (rather than failing
  during bring-up before any motion starts) is not established either
  way by this result — bring-up itself did not complete cleanly enough to
  find out.

## Recommendation (original)

Do not keep retrying bring-up against this same long-uptime container
instance — a fresh container/VM restart (not just killing the Gazebo
process, which was already tried and had already been done once earlier
in this exact session before this second failure) is the more likely
fix, and even then, per A10, the actual 60-episode batch belongs on Role B
hardware regardless of whether a single episode can be coaxed through
here. If a single smoke-test episode on Role A hardware is still wanted
as a sanity check before handover, budget a **fresh** container start
immediately before it, not a reuse of a container that has been up for
any significant time — this session's own evidence says that uptime is
exactly the variable that broke it.

---

## 2026-09-23 — recorder wiring confirmed live; container itself then exited

A later pass wired the `rosetta episode_recorder_node` lifecycle
bring-up into `episode.sh` (configure → activate → `RecordEpisode` goal
from `run_pick_and_place.py`), following the fresh-container
recommendation above. Genuine progress and four further, independent
failure modes were both found, in this order:

1. **Recorder wiring itself works.** Against a genuinely fresh
   container (`docker restart`), the lifecycle sequence
   (`configure`/`activate` via `ros2 lifecycle set --no-daemon
   --spin-time 15`) succeeded, and `run_pick_and_place.py`'s
   `RecordEpisode` goal was accepted with a real prompt — "recording
   started" was reached. This is the part of Part 8/10 this project's
   own code had not previously exercised at all; it is no longer an open
   question.
2. **`record_all=true` (the recorder's default) overloads this
   container.** With it on, `gz sim` climbed to 146.7% CPU and
   `episode_recorder_node` itself to ~40% CPU on top, 2.5/3.8 GB memory
   used, and RTF collapsed to ~0.0006 (from a normal ~0.07–0.27) — a
   different mechanism from the DDS-unreachable finding above, but the
   same root cause (A10's resource-ceiling argument). Fixed by passing
   `-p record_all:=false`, which limits recording to the two topics the
   contract actually declares instead of auto-discovering and recording
   all ~28 topics on the graph.
3. **A full episode attempt still hit `FATAL: BRINGUP_FAILED`
   intermittently**, even with the fix above, on a container that by
   then had accumulated some uptime again. One case was a genuine
   reporting artifact — the manual respawn logged "Failed to configure
   controller" for `joint_state_broadcaster` but the final verification
   pass showed all three controllers actually active — not treated as a
   real failure. But this confirms bring-up on `real_table.sdf` remains
   fragile enough that a `FATAL` exit is not on its own proof of a real
   problem; it has to be checked against `ros2 control list_controllers`
   directly.
4. **A background command as simple as `ps aux | grep episode_recorder`
   itself timed out (exit 124)**, with `uptime` showing load average
   6–8 on a container reporting nominal free memory and no CPU-heavy
   processes visible in `ps`. Killing every relevant process (`gz`,
   `ros2`, `episode_recorder`) did **not** bring load average back down
   — it stayed at 7.0–7.8 with zero matching processes running. This is
   a new, more severe, and less explicable symptom than 1–3 above:
   consistent with contention outside the container's own visibility
   (host/WSL2-VM level), not something this project's code can diagnose
   or fix from inside.
5. **The container itself exited (`docker ps` showed `Exited (255)`),
   unprompted, with an empty `docker logs`**, discovered a few minutes
   after the load-average-6–8 finding above, while running nothing more
   demanding than `uptime`/`free -h` for diagnostics. `docker start`
   brought it back up cleanly (load average 0.71, no residual
   processes) — the container's own image/config is fine; something
   killed the running instance from outside anything this session could
   observe. This is the most severe manifestation of resource
   exhaustion found this session, one level up from all the in-container
   symptoms above (a hung command, a collapsed RTF, an unreachable DDS
   port): the container itself did not survive.

**Per the standing instruction not to retry indefinitely against
genuine instability**: after finding #5, the live smoke test (a
complete episode reaching a PASS/FAIL/WRONG_BIN verdict, and the
held-out-camera frame-grab check) was not attempted again this session.
The container was restarted once, confirmed idle and healthy, and left
in that clean state rather than pushed through another live attempt.

### What is now proven vs. still open

- **Proven**: recorder lifecycle bring-up + `RecordEpisode` goal
  acceptance is real, working code, exercised live — this closes the
  "recorder wiring" gap that the original attempt above never even
  reached.
- **Still open, for the same reason as the original attempt**: no
  episode has reached a final verdict on this container, and the
  held-out camera topic (`/synth_camera_right/image`) was not
  live-verified to produce frames — the container's own instability,
  now including a full unprompted exit, made a further attempt this
  session not worthwhile to force.
- `port_bag.py` (Part 8, dataset export) has nothing to act on yet:
  there is no `grasp_meta.verdict.json` with `verdict: PASS` anywhere
  under `episodes/`, because no episode has completed. This is stated
  explicitly rather than silently skipped.

## Recommendation (current)

Unchanged in substance from the original recommendation, now with
stronger evidence behind it: this container is demonstrably not a
reliable place to run the full `gz sim` + controllers + recorder
workload, up to and including staying alive at all. Role B's hardware
remains the correct place to run the actual 60-episode batch and the
held-out-camera check, per A10. All of this project's code involved —
`episode.sh`'s recorder bring-up, `run_pick_and_place.py`'s recording
calls, `port_bag.py`'s verdict-driven export — is written, fixed, and
individually exercised as far as this container allows; what remains is
purely a hardware-capacity question, not an open code question.

---

## 2026-09-23 (continued) — the shm hypothesis, tested and confirmed fixed; a second, deeper ceiling found underneath it

The `RTPS_TRANSPORT_SHM` port-lock failure above has a specific, testable
root cause: Fast DDS's shared-memory transport lives in `/dev/shm`, and
Docker's default `--shm-size` is 64 MB **for the whole container**, not
per process. Checked directly:

```
docker exec gazebo_to_lerobot df -h /dev/shm   # -> 64M, 0 used, 64M avail
docker inspect gazebo_to_lerobot --format '{{.HostConfig.ShmSize}}'  # -> 67108864
```

Confirmed exactly as suspected. Fix applied and verified:

1. **`docker/run.sh`** (in the sibling `Gazebo_to_LeRobot_Pipeline` repo
   that owns this container) now passes `--shm-size=2g`. Recreating the
   container is destructive to anything not on a bind mount — checked
   first: `/workspace/src` is bind-mounted (safe), `/workspace/install`
   (colcon output, rebuildable) and `/workspace/htgspp` (this project's
   mirror, already fully committed at `aed552b`, so trivially
   recoverable) were the only things living solely in the writable
   layer. Recreated, re-copied `htgspp`, rebuilt `install` — `df -h
   /dev/shm` now reads `2.0G`, confirmed.
2. **`scripts/episode.sh`** now clears
   `/dev/shm/fastrtps_*`/`/dev/shm/sem.fastrtps_*` and runs `ros2 daemon
   stop` in its per-episode teardown, alongside the existing process
   kills — because the script restarts the sim stack per episode but
   not the container itself, so leaked segments were accumulating
   across the whole batch, which matches the original ~20-minute-in
   symptom shape exactly.

**This fix is confirmed correct but did not, by itself, get a live test
through** — a second, independent, and more fundamental ceiling was
found immediately underneath it while running the decomposed **3a**
test (recorder-only, no arm motion, the cheapest possible probe of the
recorder-wiring gap):

- Cold bring-up hit the routine, benign 5-second controller-activation
  timeout (the normal, self-resolving case) — one manual respawn and
  all three controllers were active.
- Starting `episode_recorder_node` itself then failed twice in a row
  with the `docker exec` process itself killed (**exit 137 = SIGKILL**,
  no `recorder.log` even created either time) — not a DDS/shm error
  this time; `df -h /dev/shm` at that exact moment showed `4.4M / 2.0G`
  used, ruling the shm fix out as the cause of *this* failure.
- `free -h` immediately before each attempt: **91–108 MiB genuinely
  free**, out of 3.8 GiB total, with `gz sim` alone holding ~950 MB–1
  GB RSS. `grep MemTotal /proc/meminfo` inside the container reads
  **3,910,246 KB total** — this is not a container memory limit
  (`docker inspect`: `Memory=0`, i.e. unlimited) but the **entire WSL2
  VM's total RAM**, which every process on this machine — container or
  not — shares.

**Conclusion**: the shm-size fix was real and is confirmed working, but
it was fixing one symptom of a problem whose actual root is that the
WSL2 VM backing this whole setup has only ~3.8 GB of RAM total, and
`gz sim` alone consumes enough of it that there isn't reliably enough
left over to start a second ROS2 process, let alone run the recorder
and the arm motion together for the full episode length. This is not
fixable from inside the container (no `docker run` flag raises the VM's
own total memory) — it requires either increasing WSL2's memory
allocation (`.wslconfig` → `memory=`, on the Windows side, then a WSL
restart) or running the batch on hardware with more RAM to begin with,
i.e. Role B. Environment left clean and idle (`gz sim` and the ROS2
daemon killed, `free -h` back to 2.9 GiB available) rather than pushed
through a further retry.

---

## 2026-09-23 (continued, 2) — swap is actually active; 3a-minimal (no Gazebo) isolates a second, distinct problem

**Swap check, as requested, before anything else:**

```
free -h            -> Swap: 12Gi total, ~300-450 KiB used, 11Gi free
swapon --show      -> /dev/sdc  partition  12G  308K  -2
cat /proc/swaps     -> /dev/sdc  partition  12582912  308  -2
```

Swap **is** active and correctly sized at 12 GB — the `.wslconfig`
`swap=12GB` setting did apply; this is not a config-never-applied
situation. It also barely moved (a few hundred KiB) across the whole
session, including through the SIGKILL events documented above — the
kernel killed those processes well before meaningfully falling back to
swap, not because swap was unavailable.

**3a-minimal, as specified**: a throwaway `rclpy` publisher
(`smoke_3a/throwaway_pub.py`, deleted after the test, never committed)
publishing `/joint_states` (30 Hz, the contract's 7 selected joints) and
`/camera/image_raw` (a solid-colour frame) with no Gazebo, no
controllers, no arm motion at all — the cheapest possible probe of the
recorder-wiring gap.

- Bring-up was cheap and clean: publisher confirmed at a true 30.0 Hz
  via `ros2 topic hz`, `free -h` steady at ~2.8-2.9 GiB available
  throughout (vs. ~90-110 MiB with `gz sim` running) — **this
  isolates the memory ceiling to `gz sim` specifically**, not to the
  recorder or the ROS graph in general.
- `episode_recorder_node` (`record_all:=false`, same as everywhere
  else) came up, and `configure` succeeded once (logged `Configured:
  robot_type=mycobot_320_pi, topics=2`, `df -h /dev/shm` at that point:
  `2.0G` size, `1.1M` used — the shm fix holds under this load too).
- `activate` reported "Transitioning successful" via the CLI, but the
  node's own log never printed a corresponding line, and two subsequent
  `ros2 lifecycle get` calls (15s and 25s spin-time) both timed out
  (exit 124) against the *same* node while `ros2 node list` answered
  instantly — the DDS graph was healthy, only this one node's lifecycle
  service was unresponsive. `deactivate` also reported "successful" but
  produced no bag file at all (`bags_test/` stayed empty), and the
  process kept climbing in RSS (77→195 MB) with no growth in accumulated
  CPU *time* between two `ps` samples 8s apart — i.e. genuinely idle,
  not busy-looping. Consistent with a stuck/blocked thread, not a
  resource problem.
- **Repeated with a 64×64 frame instead of 640×480**, to rule out image
  throughput as the cause: this time `configure` itself timed out
  (exit 124), even earlier in the sequence, on a node that had been
  running for only a few seconds and had not yet logged anything past
  "Node created (unconfigured)". Memory stayed comfortable (~2.8 GiB
  available) throughout — ruling out load/memory as the cause of this
  specific hang.

**This is a different, still-open problem from the memory ceiling
above**: something in `episode_recorder_node`'s lifecycle-service
handling becomes unresponsive under `--no-daemon` CLI calls, on this
container, independent of image size, independent of Gazebo, and
independent of available memory. It is *not* a repeat of the
already-known "stale daemon cache reports node-not-found" issue
(`--no-daemon` already avoids that) — this is the node itself failing
to answer, confirmed via `ros2 node list` succeeding against the same
graph at the same time.

**This does not contradict the earlier finding that recorder wiring
works** — the very first live attempt this session, under full
`gz sim` load, drove `episode_recorder_node` through configure→activate
and `run_pick_and_place.py`'s `RecordEpisode` goal was accepted
("recording started"). So the wiring is provably not broken end to end.
What 3a-minimal adds is that the *same* lifecycle sequence, run twice in
isolation with no simulator at all, hung twice in different places —
pointing at flakiness in the lifecycle-service path itself, not at a
resource ceiling. Per the standing instruction not to retry indefinitely
against instability, this was not chased further after the second
independent hang; processes were killed, `bags_test`/`smoke_3a` deleted
(nothing but throwaway wiring-test artifacts, no real data), environment
confirmed idle.

**Net effect on gaps 3/4**: gap 3 (a live episode) and gap 4
(bag→dataset conversion) remain unproven end-to-end this session, now
for two independently-confirmed reasons rather than one — the WSL2
memory ceiling with `gz sim` running, and a still-undiagnosed
lifecycle-service hang in `episode_recorder_node` without it. Both are
Role B / follow-up items; neither is a defect in this project's own
`episode.sh`/`run_pick_and_place.py`/`port_bag.py` code, which drove the
one successful "recording started" case correctly.

### 3b/3c — out of scope for this machine

Full episodes with `move_group`-class motion (3b: one episode,
`red_cube`; 3c: the other three, cylinder first) are **not attempted on
this machine**. The memory arithmetic: `gz sim` alone holds ~950
MB-1 GB RSS on a 3.8 GB total VM (per the WSL2 `.wslconfig` `memory=4GB`
setting), leaving well under 3 GB for everything else — controllers,
recorder, DDS, the Python IK/motion process, and headroom for growth
over a run. The single-object project this one derives from already
needed a respawn-and-retry protocol on a *simpler* world; this world
adds a camera, ArUco markers, and a textured mesh on top. Role B
hardware is the correct place for 3b/3c, and for the full 60-episode
batch regardless.

---

## 2026-09-23 (continued, 3) — the four cheap causes, cleared; static-analysis diagnosis of the recorder hang

**Direct answer to "what did the four cheap causes resolve": all four are
clear, and no bag was ever produced.** In order:

1. **Topic names** — exact match, contract to publisher, both topics.
2. **Data flow** — `ros2 topic hz` confirmed a true 30.0 Hz on both
   `/joint_states` and `/camera/image_raw` throughout every attempt.
3. **Bag output directory** — created, writable, confirmed with a direct
   write test.
4. **QoS** — could not be compared the way originally planned, and that
   itself is the finding: `ros2 topic info -v` on both topics, taken
   *while `configure` was hung*, showed **`Subscription count: 0`** on
   both — the recorder never got far enough to create a subscription at
   all, on any attempt. There is no QoS mismatch to find because there
   is nothing yet subscribing to mismatch.

So this is not a test-harness artifact. All four candidate harness-side
causes are cleared, and 3a-minimal never produced a bag in any of the
three attempts made (640×480 image, 64×64 image, and this QoS-focused
one) — the recorder's own log stalls at `Node created (unconfigured)`
every time it hangs, never reaching the `Configured: ...` line that
marks a completed `_setup()`.

### Static-analysis diagnosis (source read directly on the host — no
container, no simulator, nothing executed)

The rosetta package's source is bind-mounted onto the host at
`Gazebo_to_LeRobot_Pipeline/src/action/rosetta/rosetta/robots/ros2/`, so
this was pure reading: `episode_recorder_node.py`,
`rosetta_lifecycle_node.py` (the shared lifecycle base class), and
`nodes/node_utils.py`'s `spin_lifecycle_node` (the `main()` entry point).

**The specific hypothesis — a service call or a future awaited inside
`on_configure` under a single-threaded executor — does not hold.**
`spin_lifecycle_node` constructs a **`MultiThreadedExecutor(num_threads=4)`**,
not a single-threaded one:

```python
executor = MultiThreadedExecutor(num_threads=4)
executor.add_node(node)
...
while True:
    executor.spin_once(timeout_sec=0.1)
```

A second, related mechanism was also checked and also ruled out:
`rclpy`'s `LifecycleNode` serializes its own `~/change_state`/`~/get_state`
services through one internal transition lock, so a classic deadlock
could still occur if `_setup()` (run from inside `on_configure`) tried to
trigger something on the **same** callback group hosting that lock. It
doesn't: every entity `_setup()` creates — the persistent subscriptions
(`_create_sub`, line 627), the `RecordEpisode` action server, the
`_cancel_client`, and all three services — is explicitly constructed with
`callback_group=self._cbg`, a `ReentrantCallbackGroup` distinct from
whatever group hosts the lifecycle transition itself.

Every other call reachable from `on_configure` → `_setup()` was checked
individually and is bounded, not blocking, as authored:

- `Path(contract_path).read_text()` — local file read, already proven
  fast in the one run that completed it.
- `parse_contract`/`iter_specs` (`rosetta/contract/schema.py`,
  `specs.py`) — pure YAML/dataclass parsing, no loops, no I/O, no
  network calls, no subprocess calls.
- `_build_topic_list`, `_dedup_topics` — pure list/dict construction.
- `_discover_topics` (the `get_publishers_info_by_topic`-based
  auto-discovery path) — gated by `record_all`, which is `false` in
  every test here, so this code path never even runs.
- `rosbag2_py.SequentialWriter()` — confirmed constructed only at
  **record start** (well after `_setup()` returns), never at configure.
  Configure never touches the bag writer at all.

**What this does confirm**: the hang is real and is genuinely inside the
`on_configure` transition, not a lost CLI response. Two independent
symptoms agree: the node's own log never advances past "Node created
(unconfigured)", and a **separate** read-only call
(`ros2 lifecycle get`) hung too, against a healthy DDS graph
(`ros2 node list` answered instantly) — consistent with `LifecycleNode`'s
internal transition lock still being held by the stuck `on_configure`,
not with a delivery problem.

**What this does not do: pinpoint a line.** Every individual call on the
configure path is authored as bounded. No code-level defect matching the
hypothesized pattern, or any alternative blocking pattern checked here,
was found by static reading alone. **Honest status: real, reproducible
hang, location narrowed to inside `on_configure`/`_setup()`, root cause
not identified.** Resolving it further needs a live Python stack trace
of the hung process (`py-spy dump`) — attempted this session but blocked
by the container lacking `CAP_SYS_PTRACE`; a container recreation to add
it was in progress across two consecutive session interruptions and was
explicitly not resumed a third time, per the decision to stop all
hardware-bound work for tonight. **This is the concrete next step for
whoever picks this up — a live stack trace, not more source reading —
and it needs the container, so it is not something to attempt from a
constrained workstation.**

### Verdict: harness artifact or real recorder defect?

**Real, as far as this analysis goes** — every harness-side explanation
(QoS, topic names, data flow, bag directory) is cleared, and the
alternative, environment-side explanation this session could reach
(a container capability gap blocking a stack trace) is itself downstream
of trying to diagnose a hang that reproduced on a **freshly recreated**
container, with a **freshly written, minimal** publisher, independent of
`gz sim`, twice, in different ways. It does not look like an artifact of
this session's specific test setup. Whether it is a genuine defect in
`episode_recorder_node`/`RosettaLifecycleNode`, or an upstream
`rclpy`/`rosbag2_py` interaction neither of those files controls, is the
one open question a live stack trace would close.
