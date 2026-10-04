# Four-object sorting datasets

Two datasets, written by `scripts/port_bag.py`, LeRobot on-disk layout
(parquet + MP4 + JSON), no PyTorch dependency:

```
mycobot_sorting_train/      48 episodes, 29 735 frames, /camera/image_raw, "train_overhead" camera configuration
mycobot_sorting_heldout/    12 episodes,  7 423 frames, /camera/image_raw, "heldout_oblique" camera configuration
```

Recorded in one batch, 60/60 PASS on the first attempt (`MEASUREMENTS.md`
§10). `meta/episodes.jsonl` gives, for every episode, its `source_episode`
(row of `config/episode_matrix.csv`) and `distractors_moved`: the other
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
   (`colour.json`). Camera: 320×240 at 30 Hz, stored as recorded (the
   contract resizes to 224×224).
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

## Validation is deferred

This on-disk layout matches the documented LeRobot specification (parquet
schema, MP4 chunking, `meta/*.json` shapes) but has **not** been proven to
load by the real `LeRobotDataset` class here — that needs PyTorch, which
this constrained host does not carry (see `doc/LIMITATIONS.md` and the
previous acquisition's own note on losing a night installing a 999 MB
CPU-only PyTorch build that a container restart then destroyed). Load it
with the real class on a machine that has PyTorch before training.

## Storage

~5 GB expected for 60 episodes' raw bags (the previous, single-object
acquisition's 20 episodes were 1.7 GB). Confirmed ≥800 GB free on both the
host and the container's filesystem before this acquisition started — not
a constraint here. Bags stay out of version control and are regenerable
from `scripts/batch.sh`; only the converted `datasets/` directories need
to travel back if bandwidth is the concern, not the raw bags.
