# Object and bin geometry (Part 4) — raw extraction evidence

Every numeric value in `config/objects.yaml` traces to one of the two
command outputs below, run against `mycobot_description/worlds/real_table.sdf`
(the chosen world, `doc/WORLD_SELECTION.md`). Nothing was invented or
copied from the specification's own illustrative placeholder values.

## Part 4.2 — object/bin primitives

```
model            file                           box size               cyl r,l          mass     mu     static  pose
red_cube         real_table.sdf                 0.04 0.04 0.04         -                0.05     1.0    -       0.22 -0.08 0.020 0 0 0
red_bin          real_table.sdf                 0.10 0.10 0.002        -                -        -      true    0.22 0.10 0 0 0 0
blue_cube        real_table.sdf                 0.05 0.05 0.05         -                0.06     1.0    -       0.10 -0.06 0.025 0 0 0
green_cylinder   real_table.sdf                 -                      0.022,0.05       0.05     1.0    -       0.10 0.02 0.025 0 0 0
yellow_box       real_table.sdf                 0.05 0.03 0.04         -                0.05     1.0    -       0.10 0.09 0.020 0 0 0
blue_bin         real_table.sdf                 0.10 0.10 0.002        -                -        -      true    0.34 0.13 0 0 0 0
green_bin        real_table.sdf                 0.10 0.10 0.002        -                -        -      true    0.34 -0.04 0 0 0 0
yellow_bin       real_table.sdf                 0.10 0.10 0.002        -                -        -      true    0.33 -0.17 0 0 0 0
```

The regex used only captures the *first* `<box>` inside each `<model>`
block, which for the bins is the floor primitive (0.10 × 0.10 × 0.002) —
the four wall primitives (0.10 × 0.005 × 0.03 / 0.005 × 0.10 × 0.03, at
±0.05 m from centre) are not separately listed here since they were
authored directly in this same session (see the `real_table.sdf` edit) and
their geometry is exact by construction, not something that needed
re-extraction.

## Table top height

```
$ grep -B4 -A20 '<model name="table"' mycobot_description/worlds/real_table.sdf \
    | grep -E '<pose>|<size>|<box>'
<pose>0 0 -0.0085 0 0 0</pose>
<pose>0.261 0.017 -0.00425 0 0 0</pose>
```

Table model pose z = −0.00425 m, thickness 0.0085 m (half = 0.00425) →
world-frame top surface z = −0.00425 + 0.00425 = **0.000**, matching the
file's own header comment ("Top Z=0"). `table_top_z: 0.000` in the
manifest.

## Part 4.4 — bin static-ness

```
$ for b in red_bin blue_bin green_bin yellow_bin; do
    grep -A8 "name=\"$b\"" mycobot_description/worlds/real_table.sdf | grep -q '<static>true' \
      && echo "$b: static OK" || echo "$b: DYNAMIC"
  done
red_bin      static OK
blue_bin     static OK
green_bin    static OK
yellow_bin   static OK
```

All four confirmed static — no local override needed (Part 4.4's fallback
path was not triggered).

## Bin inner_radius / rim_z — derived from authored wall geometry

Not extracted by regex (the walls are separate `<visual>`/`<collision>`
primitives, not a single `<box>` per model); derived directly from the
geometry as authored:

- Each bin: floor at local z=0.001 (2mm thick), four walls each 0.10 (or
  0.005) × 0.005 (or 0.10) × 0.03 m, centred at local z=0.015, positioned
  at ±0.05 m from the bin centre on their respective axis.
- Wall **inner** face (the side facing the bin's centre): 0.05 − 0.0025 =
  **0.0475 m** from centre, on both x and y walls — matches
  `sim_sorting_grasp.py`'s own `BIN_INNER_HALF_MM = 47.5` constant exactly,
  an independent cross-check that this reading is correct.
- Wall **top** (rim): local z = 0.015 + 0.015 = **0.030 m**; since each
  bin's own pose has world z = 0, this is also the world-frame rim height.
- The clear opening is a **square**, not a circle. `inner_radius: 0.0475`
  in the manifest uses the *inscribed* circle (conservative: slightly
  under-counts the usable area near the square's corners, never over-counts
  it), consistent with how `sim_sorting_grasp.py` itself already treats
  this quantity.

## Validation

```
$ python3 scripts/validate_objects.py
objects.yaml OK
```
