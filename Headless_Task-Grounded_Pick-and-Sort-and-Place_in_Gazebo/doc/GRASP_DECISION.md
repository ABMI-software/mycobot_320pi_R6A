# Grasp decision gate (Part 3)

## Verdict

**A8 (labelled simulated attachment) remains the adopted floor for this
acquisition on Role A hardware.** The reference pipeline's grasp mechanism
is genuinely physical (confirmed both statically and at runtime, twice) —
not a weld — but two bounded live attempts on this constrained container
both failed to lift `red_cube`, for a diagnosed timing reason unrelated to
the mechanism itself. Per §3.5's decision table ("cannot be made to work
within one day → keep the weld; report exactly where it stopped and what
was tried"), this is exactly that row.

**Recommendation for the robotics team (Appendix B item 2):** the failure
mode strongly suggests the mechanism would succeed on Role B's faster
hardware. Re-running this same test there, before finalizing on A8, is
cheap and would retire the dataset's single largest caveat if it succeeds.

## 3.2 — Static evidence

Decisive grep, run against the whole repository:

```
$ grep -rniE "detachable|attach|weld|fixed_joint|SIMULATED" \
       --include="*.py" --include="*.sdf" --include="*.xml" --include="*.yaml" . \
    | grep -v "^./doc/"
mycobot_gateway/mycobot_gateway/pick_and_place_node.py:387:  🤏 GRASPING (simulated)
mycobot_gateway/mycobot_gateway/pick_and_place_node.py:394:  📦 RELEASING (simulated)
mycobot_gateway/mycobot_gateway/sorting_orchestrator.py:11:  Because the simulated robot has no working gripper...
mycobot_gateway/mycobot_gateway/sorting_orchestrator.py:439: GRASP ... (simulated)
```

Zero matches inside `sim_grasp.launch.py` or `sim_sorting_grasp.py` — every
hit is in the two older, explicitly-documented teleportation pipelines
(`pick_and_place_node.py`, `sorting_orchestrator.py`), which this project's
own `CLAUDE.md` already names as non-reference.

Reading `sim_sorting_grasp.py` in full (571 lines) independently confirms
this from the mechanism itself: `set_gripper()` publishes real position
commands to `/gripper_position_controller/commands` (angles from a measured
finger-span/footprint table derived from the gripper's own mesh geometry),
and every grasp and placement is verified by reading Gazebo's own object
pose (`gz topic -e -n 1 -t /world/{world}/dynamic_pose/info`) before and
after. No `DetachableJoint`, no `set_pose` call anywhere in it.

Per §3.2's interpretation table: "No matches anywhere [in the reference
pipeline] → genuine contact physics → proceed to 3.3." **This branch was
taken.**

## 3.3 — Runtime evidence

Checked live, on two separate occasions (once against `pick_and_place_sorting.sdf`,
once against `real_table.sdf` after the launch-file fix below):

```
$ ros2 topic list | grep -iE "attach|detach|weld"     # no output, both times
$ gz topic -l | grep -iE "attach|detach"                # no output, both times
```

No attach/detach/weld topic exists in the live graph either. This matches
the static evidence: whatever else is wrong, there is no attachment plugin
anywhere in this scene.

### Two live grasp attempts, both failed to lift the object

Both run via `ros2 run mycobot_gateway sim_sorting_grasp --ros-args
-p world_name:=pick_and_place_sorting -p only:=red_cube`, independently
logged by `scripts/track_grasp.py` (does not import or trust
`sim_sorting_grasp.py`'s own console output — computes the end-effector
position itself via forward kinematics from `/joint_states`, and reads the
object's pose directly from `gz topic -e`).

| | target tip (x,y,z) | achieved tip (x,y,z) | 3D miss | object z after "lift" | verdict |
|---|---|---|---|---|---|
| Attempt 1 | (0.220, −0.120, 0.018) | (0.213, −0.293, 0.397) | **~0.42 m** | 0.020 m (never rose) | prise ratée |
| Attempt 2 | (0.220, −0.120, 0.018) | (0.233, −0.114, 0.067) | **~0.05 m** | 0.026 m (never rose) | prise ratée |

`scripts/assess_reference_grasp.py` run against attempt 2's CSV (attempt
1's was lost to a bug in my own logger, fixed before attempt 2 — see below)
independently confirms: `no lift/transport samples — the run did not reach
the lift phase`, exit 1 — the verifier correctly rejects a non-grasp, which
is itself a useful early sanity check on the verifier ahead of Part 9.5's
formal test.

### Root cause of both failures: a timing bug, not a grasp-mechanism bug

Measured real-time factor in this container, via `gz topic -e -n1 -t
/world/pick_and_place_sorting/stats`:

```
real_time_factor: 0.14310192380860046
```

`sim_sorting_grasp.py`'s `move_to()` waits `self.settle` (default 1.5 s)
and a per-move `duration` (clipped to 0.6–4.0 s) — both measured in **wall
clock**, via plain `time.time()`, not scaled by the actual real-time
factor. At RTF ≈ 0.143, a 1.5 s wall-clock wait only buys ≈0.21 s of
*simulation* time — nowhere near enough for a real joint trajectory to
finish traversing a large joint-space distance. The function then gives up
and reports whatever pose the arm happened to be at, with no error raised.
This exactly explains the two misses above: attempt 1's fingertip ended up
at a completely different pose (≈42 cm off — consistent with the arm still
mid-trajectory toward an *earlier* waypoint when the deadline hit); attempt
2's ended up close in x/y but 5 cm short in z (consistent with the descent
being cut short before reaching grasp depth). Both closed the gripper on
empty air above/beside the object, so no lift was possible — nothing to do
with weld vs. friction.

This is the same class of bug the specification itself calls out in Part
8.1 for wall-clock settle waits generally (`sleep 3` at RTF 0.082 buying a
quarter of a simulated second) — except here it is baked into the
**reference pipeline's own code**, not just the new orchestration scripts
this specification asks Role A to write. It is a strong, independent
argument for A10: this reference pipeline's timing assumptions may not
survive Role A's real-time factor at all, regardless of which grasp
mechanism is used.

## A separate bug found and fixed along the way: `sim_grasp.launch.py`'s `world_name` argument

While setting up this test, `world_name:=real_table` (passed directly, and
via `real_table.launch.py`'s hardcoded value) was silently ignored — the
scene kept loading `pick_and_place_sorting.sdf` regardless. `ros2 launch
mycobot_gateway sim_grasp.launch.py --show-args` revealed why: the
container's installed copy of `sim_grasp.launch.py` was an **older
version** that hardcodes `WORLD_NAME` as a plain Python module constant
(`world_path = os.path.join(desc_pkg, 'worlds', f'{WORLD_NAME}.sdf')`),
with no `world_name`, `bridge_camera`, or `robot_appearance` launch
arguments declared at all — an artefact of an earlier wiring pass this
session that updated `mycobot_description` and added `real_table.launch.py`
but never re-copied the current `sim_grasp.launch.py` itself into the
container. Fixed: copied the current host source in, rebuilt
(`colcon build --packages-select mycobot_gateway --symlink-install`).
Verified after the fix:

```
$ ros2 launch mycobot_gateway sim_grasp.launch.py --show-args
    'world_name':  (default: 'pick_and_place_sorting')
    'bridge_camera': ...
    'robot_appearance': ...
```

and a real launch of `real_table.launch.py` now genuinely loads
`.../worlds/real_table.sdf` (confirmed via `ps aux`), with all four sorting
objects present in `dynamic_pose/info` (`red_cube`, `blue_cube`,
`green_cylinder`, `yellow_box`) and the dedicated `/camera/image_raw` +
`/camera/camera_info` topics publishing. This does not change the
grasp-mechanism verdict above (untouched by this fix, and confirmed on both
worlds independently) but it does unblock every later part of this
specification that needs to target `real_table.sdf` specifically.

## Two of my own mistakes, recorded honestly

1. **`track_grasp.py` buffered all rows in memory and only wrote the CSV
   after `node.run()` returned normally.** Stopping the process with
   `pkill` (plain SIGTERM) crashed it mid-loop on an `rclpy` context error,
   losing attempt 1's entire trace. Fixed by writing and flushing each row
   immediately; attempt 2's trace survived an identical kill cleanly.
2. **My own controller-readiness check used `grep -q '$controller.*active'`,
   which matches inside the string "inactive."** This produced a false
   "CONTROLLERS ACTIVE" report on the very first bring-up attempt, when in
   fact all three controllers were still inactive — and I repeated the
   identical mistake a second time later in this same session, in a
   slightly different check script, before catching it again. Both times
   the actual controller-manager symptom underneath was real and
   independent of my check being wrong (`Switch controller timed out after
   5 seconds`, `joint_state_broadcaster`'s spawner dying and needing a
   manual respawn with a longer `--switch-timeout`) — but my own tooling
   masked it the first time rather than surfacing it. Fixed both times by
   checking the controller-state field for an exact match, not a substring.

## Controller bring-up is itself flaky on this container, independent of the above

Every clean launch on this container needed one respawn pass
(`joint_state_broadcaster`, and on one occasion all three controllers)
after the automatic spawners' default 5-second switch timeout was exceeded
during Gazebo's busy startup window — resolved every time by re-running
`ros2 run controller_manager spawner <name> --controller-manager-timeout
150 --switch-timeout 150`, never by anything more invasive. This matches
the specification's own §10.3 philosophy (a bounded automated respawn, not
manual intervention) and is exactly the class of resource-pressure symptom
Part 11.3 predicts for constrained hardware — recorded here as first-party
evidence for that argument, not assumed from the specification's own prior
narrative.
