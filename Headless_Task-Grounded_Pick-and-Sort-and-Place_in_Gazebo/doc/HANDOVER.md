# Handover — four-object sorting acquisition (Role A → Role B)

Read this first, then `RUNNING.md`. This is the Part 12/13-style honesty
pass: what's actually done, what isn't, and why — matched against the
specification's own Appendix A order-of-work table.

## What Role A completed

Everything in Appendix A that doesn't step physics, per its own "Done
when" column:

| Row | Step | Done when | Status |
|---|---|---|---|
| 1 | Assumption register | Every assumption confirmed/failed in writing | ✅ `doc/ASSUMPTIONS.md` |
| 2 | Target world ID | WORLD_PATH + WORLD_SELECTION.md w/ hash | ✅ |
| 3 | Grasp decision gate | GRASP_DECISION.md w/ CSV evidence | ✅ |
| 4 | Object manifest | `validate_objects.py` exits 0 | ✅ (preflight confirms) |
| 5 | Task definition | 4 instructions, generated | ✅ (preflight confirms) |
| 6 | Reachability + matrix | 60 rows valid, 2nd camera confirmed | ✅ for the matrix itself (unaffected — `episode_matrix.csv` was built from the original sweep's derivative, `safe_rect.txt`, which is untouched); ⚠️ `reach_map.csv` regenerated this pass but at low fidelity — see below |
| 7 | Waypoint precompute | `check_waypoints.py` exits 0 | ✅ — and the "420" figure elsewhere in the spec was the spec's own arithmetic error (HOME counted as a solved waypoint when it's a fixed constant); 360 = 60×6 is correct, confirmed with the spec's author |
| **8** | **Extend spawn/sequence scripts** | **Runs for all four objects** | **⚠️ PARTIAL — see below** |
| 9 | Verification + wrong-bin check | 3 deliberate failures correctly rejected | ✅ `test_verifier.py`: wrong-bin→WRONG_BIN, no-lift→FAIL, disturbed-distractor→PASS-flagged, clean→PASS |
| **10** | **Smoke test, 1/object** | **Four episodes pass end to end, cylinder included** | **❌ NOT DONE** |
| 11 | preflight.sh/RUNNING.md/package | Preflight passes locally | ✅ `PREFLIGHT PASSED`; package matches §11.4 |
| 12 | 60-episode batch | — | Role B's, by design |
| 13 | Convert, README, report | Two datasets, five caveats | ⚠️ Caveats done (`doc/LIMITATIONS.md`); datasets don't exist yet — correctly sequenced *after* Role B returns data per §11.7, not a handover blocker |

**Row 8 is marked partial, not complete, to match row 10.** The code
genuinely supports all four objects (`episode_matrix.csv` has 60 rows
across all four, `run_pick_and_place.py --target` is generic, not
red-cube-specific) — but "runs for all four objects" and row 10's "four
episodes pass end to end" are the same underlying claim at two different
strictness levels, and only the weaker one (the code exists and accepts
any of the four) is actually verified. Marking row 8 done while row 10
is open would overstate what's proven.

## What is outstanding, and why

### Row 10 — the smoke test (the real gap)

No episode — not one, of any object — has ever completed to a
PASS/FAIL/WRONG_BIN verdict on this machine. Two independent,
**Role-A-hardware-specific** reasons, both diagnosed this session, not
guessed at:

**1. Memory.** The WSL2 VM backing this whole setup has a `.wslconfig`
`memory=4GB` cap — confirmed via `/proc/meminfo` (3,910,246 KB total) and
`docker inspect` (no *container* memory limit — this is the VM itself).
`gz sim` alone holds ~950 MB–1 GB RSS once `real_table.sdf` (four
objects, four bins, ArUco markers, a textured table, a camera) is
loaded. That leaves well under 3 GB for controllers, the recorder, DDS,
and the Python IK/motion process, with no headroom for growth over a
run. Confirmed directly: attempts to even *start* a second ROS2 process
alongside a running `gz sim` were SIGKILLed, with `free -h` showing
90–110 MiB genuinely free at the moment of the kill. Swap was checked
and is active and correctly sized (12 GB, per `.wslconfig`'s own
`swap=12GB`) — the kernel killed processes before meaningfully falling
back to it, not because swap was misconfigured or absent.

A real fix was found and applied for the *specific* symptom this first
surfaced as (`RTPS_TRANSPORT_SHM` port-lock failures ~20 minutes into a
`gz sim` run): Docker's default `--shm-size` was 64 MB for the whole
container; raised to 2 GB in `docker/run.sh` (in the sibling
`Gazebo_to_LeRobot_Pipeline` repo), confirmed via `df -h /dev/shm`. This
fix is real and holds, but it doesn't touch the underlying VM memory
ceiling — that's a `.wslconfig` setting on the Windows host, not
something fixable from inside a container.

**2. A separate, real, unresolved recorder-lifecycle hang.** Isolated
from the memory question entirely by testing with a throwaway publisher
faking `/joint_states` + camera data — no `gz sim`, no controllers, no
arm motion, ~2.8–2.9 GiB free throughout. `episode_recorder_node`'s own
`configure` transition still hung, twice, in different ways, with the
node's log stuck at "Node created (unconfigured)" and never reaching
"Configured". All four cheap, harness-side explanations (QoS mismatch,
no data flowing, a topic-name typo, a non-writable bag directory) were
checked directly and are clean — this ruled out the test harness as the
cause. A static read of `episode_recorder_node.py` and
`RosettaLifecycleNode` (the shared lifecycle base, both in the
`rosetta` package) did not find the specific mechanism guessed at going
in (a blocking call under a single-threaded executor — it's actually a
4-threaded `MultiThreadedExecutor`) or any alternative blocking call:
every I/O, lock, and constructor call reachable from `on_configure` is
authored as bounded. Full evidence trail in `doc/SMOKE_TEST.md`'s
2026-09-23 entries. **This needs a live Python stack trace
(`py-spy dump` on the hung process) to actually resolve** — attempted
here, blocked by the container lacking `CAP_SYS_PTRACE`, and not pursued
further once this session moved to stopping all hardware-bound work for
the night. Whoever runs the four smoke episodes on Role B hardware
should expect to hit this too, and a stack trace at that point — on
hardware that isn't also fighting the memory ceiling — would settle
definitively whether it's a `rosetta` package defect or something
upstream in `rclpy`/`rosbag2_py`.

### Row 13 — dataset conversion

Not attempted, and correctly so: `port_bag.py` only ports episodes whose
`grasp_meta.verdict.json` says `PASS`, and none exist yet. Per §11.7,
this step belongs to Role A *after* Role B returns data, not before
handover.

## What Role A needs back from Role B

1. **Run the four smoke episodes first** (`RUNNING.md` step 2) — not the
   full batch. If any of the four fails, stop and send back:
   - The failing episode's `/workspace/htgspp/stage_<idx>.log` (outside
     the episode folder, survives a retry).
   - Whether it failed during bring-up (controllers/Gazebo) or during
     recording (the lifecycle hang above) — that distinction is exactly
     what determines whether this is the memory ceiling, the recorder
     defect, or something new.
   - If it's the recorder hang: a `py-spy dump --pid <episode_recorder
     pid>` taken while it's stuck, if `py-spy` is available or can be
     installed (`pip install py-spy`; needs `CAP_SYS_PTRACE`, which
     Role B's `docker run` should grant explicitly if not already
     present).
2. **Once all four pass**: proceed to the full 60-episode batch per
   `RUNNING.md` step 3, exactly as written.
3. **Whatever comes back** — either `datasets/` or the raw episodes
   under `/workspace/htgspp/episodes/` — goes back to Role A for
   `status.sh` verification, conversion (if not already done), and the
   final report (Appendix A row 13).

## Everything else in the handover package (§11.4)

Matches the specification's file tree with one addition this pass made,
**with an honest caveat on its quality**: `config/reach_map.csv` (Part
6.5's raw reachability-sweep output) had been produced at some point
earlier in this project but never persisted to disk — only its
derivative, `config/safe_rect.txt`, survived, and
`scripts/summarise_reach.py` depends on the raw file existing.

A new `scripts/sweep_reach.py` (pure `numpy`/analytical IK, no
simulator, no ROS2 — the same guarantee `scripts/mycobot_ik.py` already
has) regenerates it, so the file now exists — but **at a resolution and
seed count traded down for speed (0.04 m grid, 6 seeds instead of the
default 12) after two slower, higher-fidelity attempts didn't finish in
a reasonable session**, and the result does **not** closely match the
existing, trusted `safe_rect.txt`: this run found x −0.10..0.34,
y −0.25..0.31, against the existing x −0.03..0.55, y −0.19..0.22. Three
of those four bounds land exactly on this run's own search-grid edge,
meaning the coarse sweep didn't even bracket the true envelope in those
directions. **`safe_rect.txt` was not touched and remains the
authoritative source `episode_matrix.csv` was actually built from** —
this gap is purely about the missing raw-data file being now present
but low-fidelity, not about anything the episode matrix depends on
being wrong. A proper regeneration (wider grid, full seed count, patient
enough to run to completion — this took over 15 minutes even at the
reduced settings on this machine's CPU) is real, still-outstanding
follow-up work.
