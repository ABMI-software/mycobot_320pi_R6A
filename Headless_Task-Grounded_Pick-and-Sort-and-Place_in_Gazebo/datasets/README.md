# Four-object sorting datasets

Two datasets, written by `scripts/port_bag.py`, LeRobot on-disk layout
(parquet + MP4 + JSON), no PyTorch dependency:

```
mycobot_sorting_train/      48 episodes, table camera (/camera/image_raw)
mycobot_sorting_heldout/    12 episodes, held-out camera (/synth_camera_right/image)
```

Whoever trains on this in three months will read this file and nothing
else. It states, up front:

1. **Grasp mechanism** — recorded per-episode in each episode's own
   metadata (`simulated_attachment: true/false`), not fixed for the whole
   dataset. Part 3's decision: a genuine, unmodified friction grasp exists
   in the reference pipeline (confirmed statically and at runtime), but
   two live attempts on the constrained host failed for a diagnosed timing
   reason unrelated to the mechanism. The labelled `DetachableJoint` weld
   is the adopted floor on that hardware; if Role B's faster hardware
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
   genuinely different camera mount (`synth_camera_right`, world-fixed,
   already part of the robot rig — not the same camera under a different
   name), not held-out images from the training camera. See Part 6.3 for
   why that distinction matters (the project's own markerless
   pose-estimation work failed for exactly the mistake of testing
   memorisation instead of generalisation).
5. **Sixty episodes is a smoke test with real task semantics, not a
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
