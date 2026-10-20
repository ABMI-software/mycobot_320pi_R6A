# Bin repositioning (2026-09-23) — a reach-margin concern confirmed and fixed

## What happened

When `blue_bin`, `green_bin`, and `yellow_bin` were first added to
`real_table.sdf` (extending it from one object to four), their positions
were chosen by hand geometry, at radius 0.34–0.371 m from the robot base —
close to the measured maximum reach (~0.39 m, per `CLAUDE.md`). At the
time, this was flagged explicitly as the least-confident part of that
layout, pending an actual reachability check.

## What the offline IK check found

Building the Part 6.5/7 waypoint infrastructure (`scripts/mycobot_ik.py`,
using the same `diff_ik.py`/`fk_pose`/`solve_pose` the reference grasp node
uses) gave a reliable, fast way to check this for real rather than by hand.
The result was unambiguous:

```
blue_bin     z=0.110  elbow_up=FAIL  any_branch=FAIL
blue_bin     z=0.080  elbow_up=FAIL  any_branch=FAIL
blue_bin     z=0.060  elbow_up=FAIL  any_branch=FAIL
blue_bin     z=0.045  elbow_up=FAIL  any_branch=FAIL
green_bin    (same pattern, all FAIL)
yellow_bin   (same pattern, all FAIL)
```

All three original positions are **genuinely unreachable at any height,
on either kinematic branch** — not a solver coverage gap, not an
elbow-up-only limitation. The concern flagged at the time these bins were
first placed was correct.

## The fix

A small geometric search (`/tmp/search_bins.py`, not checked in — a
one-off) generated candidate positions satisfying, simultaneously: board
bounds, ≥0.10 m separation from every other bin, ≥0.075 m separation from
every pick object, and ≥0.075 m clearance from the two nearby ArUco
markers (19 and 23) — then the surviving candidates were verified
reachable with the same offline IK tool, at both the release height
(0.06 m) and the transit height (0.11 m):

| Bin | Old pose (unreachable) | New pose (verified reachable) | New radius |
|---|---|---|---|
| blue_bin | (0.340, 0.130) — r=0.363m | **(0.240, 0.020)** | 0.241 m |
| green_bin | (0.340, −0.040) — r=0.343m | **(0.180, 0.190)** | 0.262 m |
| yellow_bin | (0.330, −0.170) — r=0.371m | **(0.000, 0.120)** | 0.120 m |

All three new radii sit comfortably below the measured ~0.39 m maximum
reach, and below even `red_bin`'s own already-proven 0.241 m in two of the
three cases.

## Files updated

- `mycobot_320pi_R6A/mycobot_description/worlds/real_table.sdf` (source
  repo, uncommitted) — the three `<pose>` elements, with an inline comment
  pointing back to this file.
- This project's `worlds/real_table.sdf` copy — re-copied from the fixed
  source.
- This project's `models/blue_bin.sdf` / `green_bin.sdf` / `yellow_bin.sdf`
  — only their header comments referenced a world pose; updated for
  accuracy (the model files themselves carry no pose).
- `mycobot_gateway/launch/real_table.launch.py` — the `bin_xy.*` parameters
  passed to `sim_sorting_grasp` for `demo:=true`.
- `config/objects.yaml` — the three bins' `pose` fields.

## Re-verification

```
blue_bin     r=0.241 z=0.06: OK   z=0.11: OK
green_bin    r=0.262 z=0.06: OK   z=0.11: OK
yellow_bin   r=0.120 z=0.06: OK   z=0.11: OK
```

All 40 previously-checked points (4 objects × 9 pick-position variants,
plus `red_bin`) were unaffected by this change and remain reachable.
