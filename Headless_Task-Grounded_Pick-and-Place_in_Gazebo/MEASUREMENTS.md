# Measurements established so far (runs 1-18)

Everything below was measured in this sim (Gazebo Harmonic, headless, CPU-only
laptop, ~4.9 GB RAM, `Gazebo_to_LeRobot_Pipeline` container). "Measured" means
read from TF, `gz model -p`, logs or the run CSV, not inferred from the spec.
Confidence is stated per item. The grasp itself is **not** solved yet.

## 1. Real-time factor

- `gz topic -e -t /stats` gave `real_time_factor` = **0.082** (2 sim-seconds in
  ~25 wall-seconds), on top of `gz sim` at ~150 % CPU and 1.6 GB RSS.
- Practical consequence: a 6-8 s sim trajectory takes 60-80 s of wall time. A
  full 10-phase run is ~3 min wall; a fresh-sim cycle is ~5-7 min.
- The spec assumed ~6 %; it is ~8 % here. The spec's 200 ms / 1.6 s constants
  are real-hardware bridge timings and do not apply to the sim.
- One run showed a ~2.9 h wall-clock gap inside a single trajectory (controller
  log: `SceneBroadcaster ... Timed out waiting for state`). Cause not isolated
  (host suspend or a sim stall). Runs must be supervised, not left overnight.

## 2. Fingertip offset below `gripper_base`

- **~0.10 m** (measured once, moderate confidence). With the arm commanded to
  `gripper_base` z = 0.099 the descend stalled; the ground is z = 0, so the
  fingertips are ~0.099 m below `gripper_base` when the gripper points down.
- `link6` is ~0.056 m behind `gripper_base` (static TF `link6 -> gripper_base`
  = (0, -0.007, 0.056), ~90 deg about X). Targeting `link6` as if it were the
  grasp point drives the fingers into the ground.
- Finger frame origins (`base` frame) with `gripper_base` at (0.250, 0.000,
  0.125), gripper open, top-down: `left1` (0.202, -0.008, 0.055), `right1`
  (0.330, -0.011, 0.099), `left2` (0.202, 0.000, 0.120), `right2` (0.289,
  -0.002, 0.148), `left3` (0.238, 0.002, 0.105), `right3` (0.270, 0.001,
  0.117). These are joint origins, not pad surfaces.

## 3. Finger meeting point is ahead of `gripper_base`, and asymmetric

- Fingers close along world **x** in the top-down pose. Measured open-gripper
  origins relative to `gripper_base` x: left1 **-0.048**, right1 **+0.080**.
- Midpoint of those origins is +0.016 m; **empirically the block ended
  ~+0.036 m ahead of `gripper_base`** in two independent runs (run 13: block
  0.291 vs base 0.252; run 14: 0.2546 vs 0.219). Treat +0.036 m as the meeting
  point (moderate confidence, n=2).
- Waypoints use `--tip-dx -0.036` in `precompute_ik.py` to compensate.
- Caveat: the offset direction depends on gripper yaw (J1/J6 IK branch). Run 15
  landed on a different branch (J1 = 2.54 rad) than runs 13/14/16-18 (J1 = 0.37).

## 4. Where the block goes, and what a grasp attempt does to it

- In a fresh sim the block settles exactly at (0.250, 0.000, 0.020), upright.
  The spawn pose in the spec is fine. The block drifts only when the arm pushes
  it; earlier runs left it at (0.304, 0.064), which looked like spawn drift but
  was residue from previous attempts. Reset it before every run.
- Gripper sign: `gripper_controller` 0 = open, -0.7 = closed
  (command `[-0.7, 0.7, 0.7, -0.7, -0.7, 0.7]`).
- Run 15 (tip offset 0.105): block untouched through approach/descend; gripper
  **stalled at -0.313** (object contact). Manual probe of the same pose: on
  close the block **rose 2 mm** (z 0.0223) and moved -0.001 m in x; after the
  lift the block was back at z 0.020 and all gripper joints were at +/-0.7
  (closed on nothing). So: contact, no hold.
- Runs 17 (offset 0.12) and 18 (offset 0.10): block completely untouched.
  Run 17's release reported "reached -0.000" immediately, i.e. the gripper
  never closed during the grasp phase in that sim instance. Not reproduced
  results, so **no conclusion about the best grasp height should be drawn**.
- Finger friction is already mu1 = mu2 = 1.6 in the URDF (block: 1.5).
- `grasp_held()` originally passed when nothing moved (dz trivially constant).
  It now also requires the block to rise >= 5 cm.

## 5. `move_group` "stuck executing" failure

- **Symptom:** every MoveGroup goal returns `error_code = -4` (CONTROL_FAILED)
  within ~0.1-0.3 s; the arm never moves (`ee_z` stays 0.5098, the home value);
  correction retries all fail identically.
- **Log signature** (`ros2 launch mycobot_moveit_config move_group.launch.py`
  output): `Cannot push a new trajectory while another is being executed`
  followed by `Apparently trajectory initialization failed`.
- **Detect:** `grep -c "Cannot push a new trajectory" <move_group log>`, or the
  first move of a run failing in < 1 s.
- **Cause:** not isolated. Both occurrences (runs 10 and 12) started within
  minutes of running `precompute_ik.py`, which builds a `MoveItPy` instance;
  the controller logged "Goal reached" but `move_group` never saw completion.
  Correlation only.
- **Recovery:** kill `move_group` and its launch process by PID (do not
  `pkill -f` from inside the same shell: it matches itself), relaunch, wait
  for `You can start planning now`. Ordering that worked: precompute IK, then
  restart `move_group`, then run.
- Separate config fix, **uncommitted in `Gazebo_to_LeRobot_Pipeline`**:
  `trajectory_execution.allowed_start_tolerance` 0.01 -> 0.2 in
  `move_group.launch.py` (0.01 caused CONTROL_FAILED after the first move).

## 6. TF loss after prolonged sim uptime

- After ~1.5 h of sim uptime (3.4 GB of 4.9 GB RAM in use), `/tf` stopped
  publishing: `robot_state_publisher` alive, `/joint_states` flowing (~10 Hz
  wall), all three controllers `active`, yet
  `tf2_echo base gripper_base` reports `Invalid frame ID "base" ... frame does
  not exist`. `run_pick_and_place.py` then exits with `FATAL: never got
  base->link6 transform` (the message text is stale; it waits for
  `gripper_base`).
- **Detect:** `ros2 run tf2_ros tf2_echo base gripper_base` before each run.
- **Recovery:** full stack restart (kill launch + `gz sim`, relaunch
  `sim_full.launch.py`, respawn plate and block). `robot_state_publisher` alone
  was not restarted, so whether that suffices is untested.
- Rule adopted: every run starts from a clean sim restart.

## 7. Other findings that shaped the tooling

- `MoveItPy` and a plain `rclpy` Node cannot live in one process (constructor
  hang, or the node goes deaf if the instance is garbage collected). IK is
  therefore solved offline in `precompute_ik.py` (moveit_py only) and consumed
  by `run_pick_and_place.py` (rclpy only) via `waypoints.json`.
- IK seeds must be chained waypoint to waypoint; otherwise identical Cartesian
  targets land on different branches.
- Publishing a gripper command once is not enough (publisher/subscriber
  discovery race); it is republished every 0.5 s. A held object stops the
  fingers short of -0.7, so confirmation also accepts "stalled below -0.1".
- Cold start: `joint_state_broadcaster` can stay `unconfigured`; recover with
  `ros2 control set_controller_state joint_state_broadcaster inactive` then
  `active`.

## 8. Step-2 experiment: soft block contact + gentler squeeze (2026-09-21)

Variable under test: block contact `kp=1e6/kd=1.0` -> `kp=5e4/kd=50.0`
(`models/red_block.sdf`) and gripper squeeze effort (close target; squeeze
force ~ P*(target-pos), P=60). Grasp geometry held fixed (`--tip-dx -0.036
--tip-offset 0.105`, same IK branch, J1=0.367). Every run: full sim restart,
block reset, IK precompute, then `move_group` restart, then the sequence.

| Run | Close target | Gripper stall | Block during grasp | Block z at lift | Held |
|-----|--------------|---------------|--------------------|-----------------|------|
| A   | -0.40        | -0.334        | (0.250, 0.000, 0.0200) unchanged | 0.0200 | no |
| B   | -0.55        | -0.375        | unchanged          | 0.0200          | no |
| C   | -0.70        | -0.201        | moved 4 mm in x    | 0.0200          | no |

Reading:
- The disturbance symptom (block popping 2 mm / punted 4 cm at closure, seen
  at -0.7 with the stiff contact) is **not reproduced** with the soft contact:
  the block stays put through approach, descend and closure in A and B.
- Softening the contact and lowering the effort did **not** produce a hold in
  any of the three runs. The stall position is not consistent across runs
  (-0.334, -0.375, -0.201) and does not track the target monotonically, so it
  is not a clean block-width contact signal.
- Two earlier attempts in this window were invalid and are excluded: one where
  the gripper never closed in the restarted sim, and one where my own probe
  had left the gripper closed at -0.40 before the sequence started.
- Bug found and fixed along the way: the sequence never opened the gripper
  before approach (it relied on the sim starting open). It now opens explicitly
  after `home`.
- Conclusion: a physical friction grasp was not achieved. Moving to the
  simulated attachment described in the next section.

## 9. Simulated attachment (step 3) - SIMULATED ATTACHMENT, NOT A PHYSICAL GRASP

- Mechanism: Gazebo `gz-sim-detachable-joint-system` plugin declared in
  `models/red_block.sdf`, welding the block to the robot on a message to
  `/htgpp/attach` and releasing it on `/htgpp/detach`. The script publishes
  those at the grasp pose (after the gripper closes) and at the place pose.
- Gotcha: `gripper_base` is **not a Gazebo link** (the URDF-to-SDF conversion
  lumps it into `link6`; it only exists as a TF frame). The plugin must use
  `child_link = link6`. With `gripper_base` the attach message is accepted
  and nothing happens, with no error.
- Result (clean restart, `--tip-dx -0.036 --tip-offset 0.105`, squeeze -0.40):
  block z 0.0200 -> 0.0985 (lift) -> 0.152 (transport) -> 0.068 (on plate);
  `dz` = ee_z - block_z constant at 0.107-0.108 through lift and transport;
  final block pose (0.196, -0.151, 0.068), inside the plate radius 0.06 and at
  plate top + half block height. All motions succeeded.
- This proves the sequencing, IK, timing and logging, not a friction grasp.
  Every artifact from this mode carries `simulated_attachment: true`
  (`grasp_meta.json`, and the log line `SIMULATED ATTACHMENT`).
- Harness: `restart_sim.sh` now aborts with `BRINGUP_FAILED` when the
  cold-start controller spawners die (observed again this session:
  `joint_state_broadcaster` and `gripper_position_controller` spawners exit 1
  with "Switch controller timed out after 60 seconds").
