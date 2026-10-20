# mycobot pick-and-place datasets

Two LeRobot-layout datasets, produced by `scripts/port_bag.py` from the 20
episodes recorded via `scripts/batch.sh` (see `MEASUREMENTS.md` for the full
recording history and bugs found along the way).

| Dataset | Episodes | Camera | Frames | Purpose |
|---|---|---|---|---|
| `mycobot_pick_place_train` | 1-15 | `/synth_camera` (front) | 2670 | training |
| `mycobot_pick_place_heldout` | 16-20 | `/synth_camera_right` | 927 | held-out validation |

The split is deliberate, not incidental: episodes 16-20 use a camera mount
never seen in the other 15, per the spec's own reasoning (a held-out *camera*
catches a model that memorized mount geometry instead of localizing, which
held-out *images* from the same mount would not).

## SIMULATED ATTACHMENT, not a physical grasp

Every episode's block is carried by a Gazebo `DetachableJoint` weld between
the block and the robot's `link6`, triggered on gripper-close and released on
gripper-open (see `MEASUREMENTS.md` §9). Three clean-restart experiments with
softer block contact and lower squeeze effort did not produce a friction hold
(block never rose). Each episode's `grasp_meta.json` (copied verbatim into
this dataset's `port_manifest.json`, one row per episode) carries
`"simulated_attachment": true`. Any model trained on this data is learning
"approach, close the gripper near the block, carry it to the plate, open" --
the actual load-bearing physics of a grasp are not exercised.

## Validation split: this repo vs. the GPU machine

This repo (a CPU-only, ~4.9 GB RAM laptop) does not have `torch`. Installing
the `lerobot` package here pulled its full CUDA/triton dependency chain and
was twice killed by unrelated container restarts before finishing -- see
`MEASUREMENTS.md`. Rather than fight that, `port_bag.py` writes the documented
LeRobot v3.0 layout directly with `pyarrow` (parquet) and `ffmpeg` (mp4), no
`lerobot`/`torch` import at all:

```
meta/info.json          -- features, fps, episode/frame counts
meta/episodes.jsonl      -- one line per episode: index, task, length
meta/tasks.jsonl          -- task_index -> task string
data/chunk-000/episode_XXXXXX.parquet
videos/chunk-000/observation.images.<front|right_heldout>/episode_XXXXXX.mp4
```

**This has NOT been loaded through the real `LeRobotDataset` class.** That
check -- `LeRobotDataset(repo_id_or_path).__getitem__` returning the expected
tensors, shapes and video decode -- is deferred to the GPU machine, where
`lerobot`/`torch` install cleanly. Until that check runs, treat the on-disk
layout as "written to spec, not proven to load."

## Fields

`observation.state` and `action` are both 7-float vectors: the 6 arm joints
(`joint2_to_joint1` ... `joint6output_to_joint6`) plus `gripper_controller`
(0 = open, -0.4 = closed in these recordings). `action := observation.state`
(no leader arm in this sandbox -- same simplification as the source contract
`Gazebo_to_LeRobot_Pipeline/src/contracts/mycobot_320_pi.yaml`, meaning
"action" is what the arm achieved, not a command issued an instant earlier).

## Regenerating

```bash
python3 scripts/port_bag.py --out <path> --indices "1 2 3 ... 15"
```

Reads `episodes/ep_NNN/bag/*/*.mcap` and `episodes/ep_NNN/grasp_meta.json`
directly (`rosbag2_py`, no `lerobot` import). Source bags (1.7 GB, one per
episode) are not in this dataset dir or in git; they live in the recording
container under `/workspace/htgpp/episodes/`.
