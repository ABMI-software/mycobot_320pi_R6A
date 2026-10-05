# Four-object sorting datasets

Two datasets in the **LeRobot v3.0** format, written by `scripts/port_bag.py`
with LeRobot's own writer (`LeRobotDataset.create` / `add_frame` /
`save_episode` / `finalize`) and loaded by the real `LeRobotDataset` class
under **lerobot 0.4.4**. That pin is provisional (the current release, to be
confirmed against the version the team trains with) and lives in one place,
`scripts/requirements-lerobot.txt`; `port_bag.py` refuses any other version.

```
mycobot_sorting_train/      48 episodes, 29 687 frames, camera configuration "train_overhead"
mycobot_sorting_heldout/    12 episodes,  7 411 frames, camera configuration "heldout_oblique"
```

> **Correction, 2026-10-05: the conversion was redone.** This file first gave
> 29 735 / 7 423 frames, from a hand-written conversion of 2026-10-04. Redone
> with LeRobot's own writer, it gives 29 687 / 7 411, one frame fewer per
> episode. Why: the action was one frame early (`action[t]` equal to
> `state[t]`), the timestamps were wall-clock receive time, the camera key
> differed per split, and there were no statistics; that conversion also did
> not load in LeRobot.

| Feature | Type | Content |
|---|---|---|
| `observation.state` | float32 [7] | 6 arm joints + `gripper_controller`, rad, from `/joint_states` |
| `action` | float32 [7] | same names: **the next frame's state**, see below |
| `observation.images.top` | video 240×320×3, AV1 | `/camera/image_raw` at 30 fps; **the same key in both splits** |

**Action definition: `action[t] = observation.state[t+1]`**, and each
episode's last recorded frame is dropped (it has no successor). The reason is
what was recorded: the bags hold only `/joint_states` and the camera, with no
controller command or desired-state topic, so the next achieved state is the
best available action. Future recordings also capture the controller's desired
state (`/mycobot_controller/controller_state`) and the gripper command
(`/gripper_position_controller/commands`) as `adjunct` channels of both
contracts, so a later converter can use the commanded position instead.

**Time and pairing:** `timestamp = frame_index / 30`. Each image is paired with
the joint state nearest to it in header (simulation) time. Once the
joint-state stream (100 Hz) has started, the largest pairing gap is **6 ms**.
Larger gaps occur only on the camera frames recorded before the first
joint-state message: 1 frame in most episodes (4–40 ms), and 4–11 frames in
episodes 17, 20, 42 and 50 (110–334 ms). The arm is at rest there in all 60
episodes (joints change by < 3·10⁻¹³ rad over the first 0.5 s), so those
frames carry the exact state. Per-episode largest gap: `max_pair_gap_ms_sim`
in the sidecar.

**One camera key, two configurations.** Both splits store their image as
`observation.images.top`; the held-out split differs in camera configuration,
not in feature name, so a policy trained on `train` finds its input in
`heldout`. The configuration of each episode is in the sidecar.

**Sidecar `meta/episodes_extra.jsonl`** (one line per `episode_index`; the
0.4.4 writer stores no custom per-episode fields): `source_episode` (row of
`config/episode_matrix.csv`), `split`, `camera_config`, `camera_pose_xyz_rpy`,
`distractors_moved`, `simulated_attachment`, `frames`, `max_pair_gap_ms_sim`.

**Loading:** `LeRobotDataset("local/mycobot_sorting_train", root=...,
video_backend="pyav")` works offline. The default video backend, torchcodec,
needs FFmpeg's shared libraries on the host (`sudo apt install ffmpeg`), which
the WSL2 laptop did not have; with them, the default works too.

Recorded in one batch, 60/60 PASS on the first attempt (`MEASUREMENTS.md`
§10). The sidecar gives, for every episode, `distractors_moved`: the other
objects the demonstration displaced by more than 5 mm, with the distance in
metres. One episode carries a non-empty value: **training episode 11 (source
episode 12)** pushed the green cylinder 22.9 mm with the open gripper jaw on
the descent to the grasp. It succeeded at its task and is kept, labelled, not
removed; filter on this field if your use needs untouched scenes.

Whoever trains on this in three months will read this file and nothing
else. It states, up front:

1. **Grasp mechanism** — recorded per-episode in each episode's own
   metadata (`simulated_attachment: true/false`), not fixed for the whole
   dataset. Part 3's decision: a genuine, unmodified friction grasp exists
   in the reference pipeline (confirmed statically and at runtime), but
   two live attempts on the constrained host failed for a diagnosed timing
   reason unrelated to the mechanism. The labelled `DetachableJoint` weld
   is the adopted floor on that hardware; if faster hardware
   achieves a genuine grasp, some or all episodes may carry
   `simulated_attachment: false` instead. Check the field — don't assume.
   Full account: `doc/GRASP_DECISION.md`.
2. **Ground truth, not perception** — the demonstrator read object poses
   directly from the simulator to decide where to move (A5). The recorded
   observation stream contains only camera images and joint/gripper
   proprioception; ground truth never appears in it (verified,
   `doc/CONTRACT_VERIFICATION.md`). A model trained on this data must learn
   perception and selection from pixels — the dataset shows only the
   solved form of the selection problem, never a demonstrator hesitating
   or recovering from a wrong guess.
3. **Scripted, not teleoperated** — every motion is IK-solved and
   pre-verified for elbow-up branch consistency (Part 7) before recording;
   no human piloted the arm.
4. **Split**: 48 train / 12 held out. The held-out condition is a
   **reserved camera configuration**: front-right and oblique
   (`heldout_oblique`), against the overhead view (`train_overhead`) used by
   every train episode, and by no held-out one. It is not held-out images
   from the training camera. See Part 6.3 for why that distinction matters
   (the project's own markerless pose-estimation work failed for exactly
   the mistake of testing memorisation instead of generalisation).
   Every episode also varies, from a seed derived from its episode number:
   camera pose (±2 cm, ±2° around its configuration), sun direction and
   intensity, fill and ambient light, table texture (8 variants), and
   distractor placement. Each episode's exact values are in its metadata,
   under `scene_variation`.
5. **Rendering limitations.** Episodes are rendered **with cast shadows
   disabled**: rendering the camera at 30 Hz with the sun casting shadows
   took the simulation to ~1 % of real time, which is unusable. Objects
   therefore carry no shadows, and a policy will not see the depth cues
   shadows give. Rendering is done in software (Mesa llvmpipe), because
   WSL's GPU path renders every moving object white; each episode's first
   frame is checked for the target's colour at the target's own position
   (`colour.json`). Camera: 320×240 at 30 Hz, stored as recorded; SmolVLA
   resizes its input to 512×512 with padding.
6. **Task geometry.** Objects: red cube 40 mm, blue cube 50 mm, green
   cylinder Ø44 × 50 mm, yellow box 50 × 30 × 40 mm. **Bins: 80 mm square
   opening**, 3 mm walls 30 mm tall, 86 mm outer; red (0.22, 0.10), blue
   (0.24, 0.01), green (0.18, 0.19), yellow (0.0, 0.12). The opening is a
   task parameter, and a tight one: a 50 mm cube turned 45° spans 70.7 mm.
   The demonstrator therefore releases every box square to its bin (gripper
   world yaw fixed at the place; the cylinder's is free), so place motions end
   in few wrist orientations (`doc/LIMITATIONS.md`). With that, every
   episode of the recorded batch landed flat in its bin (`MEASUREMENTS.md`
   §9–10); a different demonstrator, or a policy, gets no such guarantee
   from a tight opening.
   **Scope: lift shoulder angle J2 ≤ 48°.** Object start positions keep
   every lift pose at J2 ≤ 48.0°, the validated region; objects near the
   robot base are outside this dataset (`doc/LIMITATIONS.md`). The working
   positions were found by an x-only outward search from each object's
   original spawn pose, not a full 2-D search (`MEASUREMENTS.md` §8).
   **Code version** of the recorded batch: `f977d7af65dc` (started
   2026-10-04T08:37:11Z), printed on the batch's start line (`batch.log`),
   hashed over `scripts`, `models`, `worlds`, `config` and `contracts`. The
   committed tree differs from it in four files changed after the batch, none
   of which changes a motion: `port_bag.py` (carries `distractors_moved`),
   `run_pick_and_place.py` (also records landed yaw and tilt), and the new
   demo files `run_demo.py` and `run_gui_demo.sh`. Hashing the committed tree
   therefore gives a different value.
7. **Sixty episodes is a smoke test with real task semantics, not a
   training set.** The community figure for fine-tuning is 100–500
   episodes *per task*; this is four tasks × 15. See `doc/LIMITATIONS.md`
   for the full list of what this does and does not prove, and
   `doc/METRICS.md` for the metric set to actually report against (never
   the servoing figure in `training/calibration/PROTOCOLE_ESSAIS_PRECISION.md` —
   see A12 and `doc/METRICS.md` for why).

## Validation (2026-10-05, lerobot 0.4.4, CPU-only venv)

All checked offline, no Hub access:
- both splits load with `LeRobotDataset`; 48 / 12 episodes, 29 687 / 7 411
  frames;
- on a full episode, `action[t] == observation.state[t+1]` on every row, and
  the last row's action equals the dropped final frame's state in the bag;
- **timestamp to frame:** on 10 rows across both splits, the image the loader
  returns equals the frame decoded directly from the MP4 at that index (max
  difference 0); on the 7 rows with moving pixels, that frame matches the raw
  bag image at t (e.g. 2.2 vs 77.6 / 52.0 for t−1 / t+1);
- statistics exist for state, action and image (`meta/stats.json`); every
  joint's standard deviation ≥ 0.249 rad;
- the image key is `observation.images.top` in both splits.

**SmolVLA smoke-train** (`lerobot/smolvla_base`, CPU, batch size 1, 10 steps,
`--rename_map='{"observation.images.top": "observation.images.camera1"}'`,
since the pretrained policy names its cameras `camera1`–`camera3`): loss
finite at every step (0.117–2.371), ~4.1 s per step, peak RAM 3.5 GB;
100 M of 450 M parameters trainable, i.e. the action expert only, the
vision-language backbone frozen (the 0.4.4 defaults and the checkpoint's own
config: `freeze_vision_encoder=True`, `train_expert_only=True`). This proves
data → model → loss end to end; it is not a training run.

## Storage

8.1 GB of raw bags for the 60 episodes (they stay in the container); the
two converted datasets take 112 MB (86 + 26). The previous, single-object
acquisition's 20 episodes were 1.7 GB of bags. Confirmed ≥800 GB free on both the
host and the container's filesystem before this acquisition started — not
a constraint here. Bags stay out of version control and are regenerable
from `scripts/batch.sh`; only the converted `datasets/` directories need
to travel back if bandwidth is the concern, not the raw bags.
