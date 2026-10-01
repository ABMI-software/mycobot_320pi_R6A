# World selection (Part 2 of the specification)

**Chosen world:** `mycobot_description/worlds/real_table.sdf`
(repo: `mycobot_320pi_R6A`, branch `feature/pick-and-place-osama`)

```
config/WORLD_PATH → mycobot_description/worlds/real_table.sdf
```

## Content hash

```
$ sha256sum mycobot_description/worlds/real_table.sdf
f147410111fd876a977ca7813d0bcb66b7510feacf6545aa98b6e1be23037070  mycobot_description/worlds/real_table.sdf
```

(Updated 2026-09-23, Part 6/7: `blue_bin`/`green_bin`/`yellow_bin` were
repositioned after offline IK confirmed their original poses were
unreachable at any height, on any branch — see `doc/BIN_REPOSITIONING.md`.
The world's model *set* (8 objects/bins, 4 ArUco, 1 camera) is unchanged,
only three poses moved, so the decision rule in the table below still
applies unchanged.)

**Caveat, stated per A3/A11:** this hash is of the file as it exists on disk on
`feature/pick-and-place-osama` right now. The world was extended from one
object (`red_cube`/`red_bin`) to all eight (four objects, four bins) in this
same work session, and that change is **uncommitted**. Anyone re-deriving
this hash from a fresh clone of that branch before the commit lands will get
a different value. This is flagged explicitly rather than discovered later.

## Enumeration output, verbatim (Part 2.2 / 2.3)

Run from the repository root (`mycobot_320pi_R6A`), exact commands from the
specification, no edits:

```
$ find . -name "*.sdf" -path "*world*" -printf '%T@ %p\n' | sort -rn | cut -d' ' -f2-
./mycobot_description/worlds/real_table.sdf
./mycobot_gateway/worlds/lab_room.sdf
./mycobot_description/worlds/randomized.sdf
./mycobot_description/worlds/precision_benchmark.sdf
./mycobot_description/worlds/randomized_v2.sdf
./mycobot_description/worlds/pick_and_place_sorting.sdf
./mycobot_description/worlds/pick_and_place.sdf

$ find . -name "sim_grasp.launch.py" -o -name "*gateway*" -type d
./mycobot_gateway
./mycobot_gateway/mycobot_gateway
./mycobot_gateway/launch/sim_grasp.launch.py

$ find . -type d \( -name "*cube*" -o -name "*bin*" -o -name "*cylinder*" \)
(no output — these objects are modelled inline inside world SDFs, not as
their own model directories; see A3 note below)
```

Per-world enumeration (Part 2.3), all seven `*world*.sdf` files found above:

```
==========================================================
WORLD: ./mycobot_description/worlds/pick_and_place_sorting.sdf
-- models declared:
back_wall blue_bin blue_cube green_bin green_cylinder ground_plane red_bin red_cube table yellow_bin yellow_box
-- included uris:
-- aruco ids:
-- camera sensors: 0
-- camera names:
-- textured:       0
-- static bins:    4
==========================================================
WORLD: ./mycobot_description/worlds/randomized.sdf
-- models declared:
back_wall clutter_box_1 clutter_box_2 clutter_cylinder ground_plane side_panel table
-- included uris:
-- aruco ids:
-- camera sensors: 0
-- camera names:
-- textured:       1
-- static bins:    0
==========================================================
WORLD: ./mycobot_description/worlds/real_table.sdf
-- models declared:
blue_bin blue_cube green_bin green_cylinder ground_plane red_bin red_cube table table_camera yellow_bin yellow_box
-- included uris:
package://mycobot_description/models/aruco_19
package://mycobot_description/models/aruco_23
package://mycobot_description/models/aruco_25
package://mycobot_description/models/aruco_26
package://mycobot_description/models/wood_table/meshes/tabletop.dae
-- aruco ids:      19 23 25 26
-- camera sensors: 1
-- camera names:   table_camera
-- textured:       1
-- static bins:    4
==========================================================
WORLD: ./mycobot_description/worlds/randomized_v2.sdf
-- models declared:
back_wall clutter_box_1 clutter_box_2 clutter_box_3 clutter_box_4 clutter_box_5 clutter_box_6 clutter_cylinder_1 clutter_cylinder_2 clutter_cylinder_3 clutter_cylinder_4 clutter_sphere_1 clutter_sphere_2 ground_plane left_wall right_wall table
-- included uris:
-- aruco ids:
-- camera sensors: 0
-- camera names:
-- textured:       0
-- static bins:    0
==========================================================
WORLD: ./mycobot_description/worlds/pick_and_place.sdf
-- models declared:
back_wall drop_zone ground_plane table target_cube
-- included uris:
-- aruco ids:
-- camera sensors: 0
-- camera names:
-- textured:       0
-- static bins:    0
==========================================================
WORLD: ./mycobot_description/worlds/precision_benchmark.sdf
-- models declared:
grid_pt_0 grid_pt_1 grid_pt_2 grid_pt_3 grid_pt_4 grid_pt_5 grid_pt_6 grid_pt_7 grid_pt_8 ground_plane table target_cube ws_marker_0 ws_marker_1 ws_marker_2 ws_marker_3
-- included uris:
-- aruco ids:      4
-- camera sensors: 0
-- camera names:
-- textured:       5
-- static bins:    0
==========================================================
WORLD: ./mycobot_gateway/worlds/lab_room.sdf
-- models declared:
ground_plane
-- included uris:
-- aruco ids:
-- camera sensors: 0
-- camera names:
-- textured:       0
-- static bins:    0
```

A7 literal check, run separately:

```
$ grep -o 'aruco_[0-9]*' mycobot_description/worlds/real_table.sdf | sort -u
aruco_19
aruco_23
aruco_25
aruco_26
```
Confirms A7: only 19/23/25/26 appear in the target world. IDs 10 and 11 (the
carton-size experiment from an earlier presentation) do not appear anywhere
in it — A7 confirmed, no further investigation needed.

## Decision rule (Part 2.4) applied to the two real candidates

| # | Criterion | `pick_and_place_sorting.sdf` | `real_table.sdf` |
|---|---|---|---|
| 1 | Declares all eight models from A2 | ✅ yes | ✅ yes |
| 2 | Contains ArUco markers 19/23/25/26 | ❌ no (0 aruco ids) | ✅ yes |
| 3 | Contains a table camera sensor | ❌ no (0 camera sensors) | ✅ yes (`table_camera`) |
| 4 | Has a textured table, not a bare ground plane | ❌ no | ✅ yes |

**Exactly one world satisfies all four criteria: `real_table.sdf`.** Per
§2.4, A1 is resolved and this part is complete — no escalation to the
robotics team was needed under the "if no world satisfies criterion 1, stop"
or "if only one has all eight objects" branches.

## Why the two circulating premises are not actually in conflict

The specification's own framing of A1 (§1, A1) states two premises "in
circulation" and calls them mutually inconsistent: one naming
`pick_and_place_sorting.sdf` as the four-object target, the other stating
the four objects were added to the `real_table.sdf` line and calling
`pick_and_place_sorting.sdf` "the older generic world."

Both were true statements about different points in time, not a real
contradiction. `pick_and_place_sorting.sdf` genuinely was — and remains —
the only merged, git-committed world with all eight objects (authored by
José Bernardo, 2026-04-23, commits `bff55d0c`/`ab0de957`). `real_table.sdf`
genuinely was, at the start of this work, a single-object (`red_cube`/
`red_bin`) world — matching the screenshots in both of Osama's presentations
(`Presentation24`, pages 3 and 8–9; `Presentation23`, page 25), where
extending it to all four objects is explicitly listed as a **future** step,
not yet done.

The apparent conflict was resolved by doing the "substantial work item" the
specification's own fallback anticipates (§A1 fallback, §2.4's second
branch): the missing `blue_cube`, `green_cylinder`, `yellow_box` and their
three bins were ported into `real_table.sdf` from `pick_and_place_sorting.sdf`
(identical primitives — box/box/cylinder/box, identical bin wall geometry —
so no dimension, mass or friction value was invented; every one was copied
from an existing, already-merged file), repositioned to fit inside
`real_table.sdf`'s actual measured board (622×449 mm) and stay clear of the
four ArUco markers. That work is what produced the "all eight" row for
`real_table.sdf` in the table above — it did not exist when the two premises
were first written down.

**Per §2.4's second branch**, this is recorded as its own work item, done
the same day: no separate referral to the robotics team was required because
the merge was completed rather than left as an open question, but the
robotics team should still be informed that this merge happened, since it
changes the file on a shared branch (see Appendix B item 1 in the main
specification, and `doc/GRASP_DECISION.md` / `CHANGELOG.md` entries once
committed).

## A3 note — objects are inline, not separate model directories

The `find -type d \( -name "*cube*" ... \)` sweep in §2.2 returns nothing,
which could look like an A3 failure. It is not: `red_cube`, `blue_cube`,
`green_cylinder`, `yellow_box` and all four bins are defined **inline** as
`<model>` blocks directly inside `real_table.sdf` (and inside
`pick_and_place_sorting.sdf`), not as separate `models/<name>/model.sdf`
packages the way the four ArUco markers and the wood table are. This matches
the file layout confirmed by the `-- included uris:` line above: only the
ArUco markers and the table mesh are `<include>`d from `models/`; the eight
sorting objects/bins are not. A3 is satisfied — the geometry is real and on
disk — just not in the directory shape the sweep was looking for.
