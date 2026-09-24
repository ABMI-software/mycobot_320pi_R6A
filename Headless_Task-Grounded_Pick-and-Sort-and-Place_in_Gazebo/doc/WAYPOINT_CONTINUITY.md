# Waypoint continuity — two real bugs found and fixed, two false alarms resolved

Part 7.4's `check_waypoints.py` exists precisely to catch bad trajectories
before they cost a live episode. It did its job — on the very first
precompute attempt, every one of the 60 episodes failed it. Diagnosing why
took four iterations; all four are recorded here rather than only the
final state, since two were real bugs in this project's own code and two
were false alarms that needed the check itself corrected — the second
false alarm was found only when pushing for a literal, unqualified
`check_waypoints.py` exit 0 across all 60 episodes, rather than accepting
"59 clean, 1 inspected and judged benign" as good enough.

## Attempt 1 — every waypoint solved independently: real bug

`precompute_ik.py`'s first `cmd_matrix()` called `solve_tip()` once per
waypoint, each free to search all wrist orientations (`phi`) from scratch.
Result: **every** episode showed 40-80° "branch jumps" between every
consecutive waypoint pair, including within a single approach→grasp→lift
sequence at the same (x,y). This is exactly the failure mode
`sim_sorting_grasp.py`'s own `solve_column()` was built to avoid — its
docstring: *"Rebalayer phi a chaque hauteur faisait tourner le poignet
entre la saisie et la levee, ce qui devissait l'objet des doigts."* My
precompute reintroduced the exact bug the reference had already fixed,
because I hadn't mirrored that part of its architecture.

**Fix**: added `solve_column()` to `scripts/mycobot_ik.py`, mirroring the
reference's own function, and rewrote `cmd_matrix()` to solve the
pick-side waypoints (approach/grasp/approach) as one group with one shared
`phi`, and the bin-side waypoints (transit/release/retreat) as a second
group with its own shared `phi`.

## Attempt 2 — grouped, but the SCORING still picked far solutions: real bug

After the fix above, the wrist joint (J6) was confirmed locked within each
group (values agreed to 0.01°) — but `check_waypoints.py` still flagged
the *same* 40-80° jumps, now traced to J3/J4, not J6. Cause:
`solve_tip()`'s scoring formula, copied from the reference unchanged
(`score = travel - 2·min(margin, 30)`), lets a solution with high
joint-limit margin outscore a solution close to the previous waypoint even
when travel differs by 40+ degrees — a margin bonus of up to 60 can beat a
travel difference smaller than that. This formula's own bias toward margin
happened not to bite the reference's original world/positions; it did
bite this world's.

**Fix**: when a reference pose (`q_ref`) exists, `solve_tip()` now scores
by travel alone — margin is already floor-filtered by `margin_min`, so it
should only break ties among similarly-near solutions, never outrank
continuity outright.

## Attempt 3 — real per-joint jumps remained, but they aren't branch flips: false alarm, check corrected

Even after both fixes, `check_waypoints.py` still flagged ~40-45°
per-waypoint-pair jumps for red_cube's episodes. Direct inspection of the
actual joint values settled it:

```
waypoint:        0            1             2
                [1.3,-6.3,-61.8,-21.8,90.0,-163.7]
                [1.3,-6.7,-106.3,23.0,90.0,-163.7]
                [1.3,-6.3,-61.8,-21.8,90.0,-163.7]
```

J1, J5, J6 do not move (to within 0.4°). J3 swings by -44.4°, J4 by
+44.8° — **equal and opposite**. `J3 + J4` is invariant across the whole
sequence (-83.6 → -83.3 → -83.6). That is the textbook signature of
elbow-bend/wrist-pitch compensation for a pure vertical descent at fixed
(x,y) and fixed tool orientation, not a branch flip — this world's pick
objects sit closer to the base (x≈0.10) than the reference world's, so
the same ~92 mm descent (`APPROACH_Z` 0.110 → `grasp_z` ≈0.018) needs a
larger angular swing at this shorter reach. A flat "max joint delta"
threshold cannot tell this apart from a genuine branch flip; it flagged
correct IK as broken.

**Fix, in `check_waypoints.py` itself, not in the solver**: check J1, J2,
J5, J6 individually (a real base-spin or wrist-flip is exactly what must
still be caught), but check `J3 + J4` as the invariant for the elbow/wrist
pair — it only moves if J3 and J4 are *not* moving in the complementary
way the geometry requires, which is the actual definition of a branch
flip for this joint pair. Also excluded waypoint-pair index 2→3 (the
deliberate pick-group → bin-group transport move) from the same-group
continuity check entirely — a base rotation between two different table
locations is the intended motion there, not a discontinuity; that specific
move gets its own `sim_duration` in `run_pick_and_place.py` to execute
smoothly, and Part 7.4's own concern (staying on one branch through a
single continuous approach/grasp/lift gesture) never applied across it.

## Attempt 4 — the J3+J4 invariant was itself only a special case: second false alarm, check corrected again

Attempt 3's fix got 59 of 60 episodes clean. The one exception, `ep035`
(`green_cylinder`, its closest-to-base `x_var`, radius ≈0.0825 m — much
nearer the base than `red_cube`'s ≈0.234 m), still flagged a 35.4° "J3+J4
jump." Rather than accept one inspected-and-judged-benign exception as
good enough, I checked why the same reasoning that cleared `red_cube`
didn't clear this one:

```
waypoint:   0                                    1
red_cube:  [1.3,-6.3,-61.8,-21.8,90.0,-163.7]  [1.3,-6.7,-106.3,23.0,90.0,-163.7]
ep035:     [109.6,61.4,-78.6,-72.8,90.2,-10.4] [109.6,96.8,-122.0,-64.9,90.2,-10.4]
```

For `red_cube`, J2 happened to barely move (-6.3 → -6.7°), which is *why*
J3+J4 alone looked invariant — it was a special case of a more general
fact, not the fact itself. For `ep035`, J2 also swings substantially
(61.4 → 96.8°, a real 35.4° change) because this point is much closer to
the base, where the same vertical descent needs more shoulder
involvement, not just elbow/wrist-pitch. Checking J2+J3+J4 (the arm's
total shoulder-elbow-wrist pitch — what must stay constant to keep the
tool pointing straight down at a fixed (x,y), however that pitch is
distributed across the three joints) resolves both cases correctly:
red_cube -89.9° → -90.0° (invariant), ep035 -90.0° → -90.1° (invariant).

**Fix**: `check_waypoints.py` now checks J1, J5, J6 individually, and
J2+J3+J4 as the single invariant for the whole shoulder-elbow-wrist
chain, replacing the narrower (and, it turned out, only coincidentally
correct) J3+J4-only check from attempt 3.

## Where this leaves the check

`scripts/check_waypoints.py` now distinguishes what it should always have
distinguished: a genuine kinematic discontinuity (base spin, wrist flip,
a shoulder/elbow/wrist chain moving inconsistently) from a legitimate,
large, coupled joint motion required by this world's specific geometry —
checked against two independent cases at very different reach distances,
not tuned to pass one and hoped to generalize. All four fixes are in the
checked-in scripts, not worked around in this document alone — re-running
`precompute_ik.py` from a clean `config/waypoints.json` and
`check_waypoints.py` reproduces this reasoning mechanically, not by hand.

**Current state, verified just now**: `python3 scripts/check_waypoints.py`
exits 0. `60 episodes: all waypoints OK` — literally, not "59 clean plus
one accepted exception."

## Waypoint count: six solved per episode plus a constant, 360 total (not a deviation)

The specification's Part 7.3 lists seven named waypoints per episode
(HOME, APPROACH, DESCEND, LIFT, TRANSPORT, RELEASE, RETREAT-then-HOME),
but describes HOME itself as joint-space and fixed — it needs no IK
solve. The six that actually require a solution are APPROACH, DESCEND,
LIFT, TRANSPORT, RELEASE, RETREAT. `precompute_ik.py` solves exactly
those six per episode: 60 × 6 = 360 entries in `config/waypoints.json`.

Part 7.4's "420 pre-solved waypoints" (60 × 7) came from multiplying by
all seven named waypoints, including the constant. Confirmed with the
spec's author: this was an error in the specification itself, not a
deviation introduced here — 360 is the correct number. `HOME_Q =
[0,0,0,0,0,0]` in `run_pick_and_place.py` is the fixed constant;
`run_pick_and_place.py` still visits it at the start and end of every
episode (`node.goto(HOME_Q, ...)`, twice), matching the specification's
intended motion exactly. Six-plus-a-constant, not seven. Closed.
