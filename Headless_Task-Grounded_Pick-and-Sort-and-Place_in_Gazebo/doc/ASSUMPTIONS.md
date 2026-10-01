# Assumption register (Part 1), annotated with outcomes

Twelve assumptions from the specification, each with its actual verification
outcome as of this pass. Status legend: **CONFIRMED** (verified with
evidence), **PARTIAL** (statically confirmed, runtime step still open),
**PENDING** (not yet reached), **ADOPTED** (a specification decision, not a
locally-verifiable fact).

## A1 — A merged world containing four objects and four colour-matched bins exists

**CONFIRMED.** See `doc/WORLD_SELECTION.md`. `mycobot_description/worlds/real_table.sdf`
satisfies all four criteria in §2.4 (all eight models, all four ArUco
markers, a table camera, a textured table); `pick_and_place_sorting.sdf`
satisfies only the first. The two premises the specification calls
"inconsistent" were not actually in conflict — they described the same file
at two different points in time. The merge of the four objects into
`real_table.sdf` (the gap the spec's own fallback anticipated) was completed
this session, not left open. **Caveat**: uncommitted on disk, see
`WORLD_SELECTION.md`'s hash note.

## A2 — The object and bin set is fixed and colour-matched

**CONFIRMED.** The enumeration in Part 2.3 lists exactly `red_cube`,
`blue_cube`, `green_cylinder`, `yellow_box`, `red_bin`, `blue_bin`,
`green_bin`, `yellow_bin` — no more, no fewer, in the chosen world.

## A3 — All world, model and pipeline files are present on the local filesystem

**CONFIRMED**, with one shape correction. `find -type d (*cube*|*bin*|*cylinder*)`
in Part 2.2 returns nothing — the sorting objects/bins are inline `<model>`
blocks inside the world SDF, not separate `models/<name>/` packages (unlike
the four ArUco markers and the wood table, which are). All named artefacts
are present: the world file, the four ArUco model directories
(`mycobot_description/models/aruco_{19,23,25,26}`), the wood table model
(`mycobot_description/models/wood_table`), `sim_grasp.launch.py`, and the
companion node `sim_sorting_grasp.py` plus its two runtime dependencies
(`scripts/diff_ik.py`, `training/dream/mycobot_fk.py`) — the last two were
not installed in the `gazebo_to_lerobot` container until this session (see
"Container wiring" below); they are present in the source repository.

## A4 — A reference pipeline achieving a physically-verified grasp exists locally

**CONFIRMED as a mechanism; not yet achieved as a completed live grasp on
this hardware.** See `doc/GRASP_DECISION.md` for the full evidence and
verdict.

Summary (full evidence in `doc/GRASP_DECISION.md`): the reference pipeline
(`sim_grasp.launch.py` + `sim_sorting_grasp.py`) contains no attach/weld/
`DetachableJoint` anywhere, confirmed both statically (repo-wide grep, full
571-line code read) and at runtime (no attach/detach topic in the live
graph, on two separate world launches). Two live grasp attempts on this
container both failed to lift `red_cube`, but the diagnosed cause is a
timing bug (`move_to()`'s wall-clock settle logic vs. this container's
measured RTF ≈ 0.143), not the grasp mechanism. Verdict: **A8 remains the
adopted floor here**, with a recorded recommendation that Role B re-run
this same test on faster hardware before finalizing, since the mechanism
itself is likely sound.

## A5 — The demonstrator may use ground-truth object poses

**ADOPTED**, agreed explicitly earlier in this work (see conversation
record) on the same reasoning the specification gives: a VLA is trained on
`instruction + image → action`; it never observes how the demonstrator knew
the object's pose. `sim_sorting_grasp.py`'s own `object_poses()` already
works this way (queries Gazebo's ground truth for both the pick position
and grasp/placement verification). **Not yet verified**: Part 10.1's
specific contract check (`observation.state` contains joint positions and
gripper only, no object poses) — the contracts don't exist yet; this is
correctly sequenced as a Part 10 item, not an A5 blocker.

## A6 — Sorting assignment is by colour

**CONFIRMED**, established earlier this session from two independent
sources: `Presentation24` page 10 ("Tri 4 couleurs sur plateau réel — objet
→ bac de sa couleur") and `sim_sorting_grasp.py`'s own `TARGETS` list, which
pairs each object with its same-colour bin by name with no size-based logic
anywhere.

## A7 — ArUco IDs 10 and 11 are not sorting destinations

**CONFIRMED.**

```
$ grep -o 'aruco_[0-9]*' mycobot_description/worlds/real_table.sdf | sort -u
aruco_19
aruco_23
aruco_25
aruco_26
```

IDs 10/11 belong to a separate, unrelated experiment (`Presentation24` page
7 — a carton-size depth-detection heuristic, "Grand carton"/"Petit carton"),
confirmed earlier this session by reading that slide directly. They do not
appear in this world at all.

## A8 — The simulated attachment remains sanctioned

**CONFIRMED — applies, per Part 3's decision.** Two bounded live grasp
attempts on this container's hardware both failed to lift `red_cube`, for a
diagnosed timing reason (not a mechanism failure — see A4 and
`doc/GRASP_DECISION.md`). Per §3.5's decision table ("cannot be made to
work within one day → keep the weld"), A8 stands for this acquisition, with
a written recommendation to the robotics team (Appendix B item 2) that
Role B re-attempt the same bounded test on faster hardware before treating
this as final, since the mechanism itself is not the suspected cause.

## A9 — Target volume is fifteen clean successful episodes per object

**ADOPTED** as specified (60 total, 15/object, failures re-run not counted).
This is a scheduling decision stated by the specification itself, not a
locally-verifiable fact. Feeds Part 6's matrix (not yet built).

## A10 — The acquisition batch does not run on the constrained workstation

**ADOPTED** as specified, for the reasons given in §11.3 (3.4× RTF ratio,
and the documented memory-exhaustion symptoms from the previous acquisition
run on this same class of hardware). Relevant to Part 11, not yet reached.

## A11 — The runtime environment matches the previous acquisition

**CONFIRMED**, and directly demonstrated rather than merely asserted: this
session wired the current `mycobot_description`/`mycobot_gateway` source
(worlds, models, launch files, and `sim_sorting_grasp`'s two script
dependencies) into the **same** `gazebo_to_lerobot:jazzy-harmonic` container
used for the previous acquisition, and rebuilt it there
(`colcon build --packages-select mycobot_description mycobot_gateway --symlink-install`)
— succeeded, and `from mycobot_gateway.sim_sorting_grasp import SimSortingGrasp, TARGETS`
now imports cleanly inside that container (it did not, before this session,
because `scripts/diff_ik.py` had never been copied in). This is the same
container, not a fresh one, and the fix was additive (nothing existing was
removed or overwritten destructively).

**Second wiring gap found and fixed during Part 3**: the container's
`sim_grasp.launch.py` was also stale (a pre-`real_table` version hardcoding
`WORLD_NAME` at module level, no `world_name`/`bridge_camera`/
`robot_appearance` arguments) — missed in the first wiring pass because an
earlier diff check of mine reported it as identical when it was not. Fixed
the same way (copy + rebuild); verified `real_table.sdf` now genuinely
loads. See `doc/GRASP_DECISION.md` for the full account, including two
self-caught mistakes in my own verification scripts along the way.

**Residual risk, stated per §A11 rather than hidden**: the graphical Gazebo
path has still never been exercised from this side — every verification so
far, including both colcon builds and every live run, was headless/CLI.
Display passthrough to Role B's Windows host is explicitly out of scope for
Role A to test.

## A12 — The 0.63 mm figure is not the benchmark for this work

**CONFIRMED**, and already argued at length earlier in this session:
0.63 mm is essai 7's converged closed-loop *servoing* accuracy after two
correction passes against a known ArUco fiducial — a fundamentally
different quantity from a one-shot, open-loop VLA inference. Verified
directly against `training/calibration/PROTOCOLE_ESSAIS_PRECISION.md` §7
rather than from memory (10.79 mm → 1.29 mm → 0.63 mm, 18/20 trials under
1 mm, dispersion flat throughout — a purely systematic, correctable bias,
not a floor). Part 13's metric set (task success rate, correct-bin rate,
held-out-condition success) replaces it; not yet built (Part 13 comes after
data collection).

## Summary table

| ID | Status | Blocking? |
|---|---|---|
| A1 | CONFIRMED | — |
| A2 | CONFIRMED | — |
| A3 | CONFIRMED (shape note) | — |
| A4 | CONFIRMED (mechanism); live grasp not yet achieved here | Recommend Role B re-test (Appendix B item 2) |
| A5 | ADOPTED — contract check deferred to Part 10 | — |
| A6 | CONFIRMED | — |
| A7 | CONFIRMED | — |
| A8 | CONFIRMED — applies for this acquisition | — |
| A9 | ADOPTED | — |
| A10 | ADOPTED — reinforced by A4's timing finding | — |
| A11 | CONFIRMED (2nd wiring gap found & fixed) | Graphical path untested (stated, not hidden) |
| A12 | CONFIRMED | — |

**Definition of done for Part 1, per the specification**: every assumption
either confirmed, or recorded as failed with its fallback triggered and the
robotics team informed. All twelve are now confirmed or adopted. Nothing
here has failed outright, so no robotics-team referral is triggered by
Part 1 itself — Appendix B's items (including the A4 re-test
recommendation) are separate, forward-looking referrals, not assumption
failures. Parts 1–3 of the specification are complete; see
`doc/WORLD_SELECTION.md` and `doc/GRASP_DECISION.md` for the full evidence
trail.
