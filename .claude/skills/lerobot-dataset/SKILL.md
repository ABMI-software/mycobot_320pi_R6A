---
name: lerobot-dataset
description: Recording, converting, and publishing episodic manipulation data in the LeRobot / HuggingFace datasets format. Invoke when the user wants to collect VLA training data, convert bag files to episode format, or publish a dataset to the HuggingFace hub.
---

# LeRobot dataset workflow

LeRobot (by HuggingFace) is the de-facto format for episodic manipulation data in 2026. Most fine-tune-ready VLA models (OpenVLA, Octo, π0) accept this format natively.

## The format in 30 seconds

A LeRobot dataset is a directory with:
- `meta/info.json` — dataset-level metadata (robot type, cameras, FPS, action space)
- `meta/episodes.jsonl` — one line per episode (length, instruction, tags)
- `meta/tasks.jsonl` — distinct language instructions used across episodes
- `data/chunk-XXX/episode_YYY.parquet` — per-timestep (`observation.state`, `observation.images.*`, `action`, `timestamp`, `frame_index`, `episode_index`, `task_index`)
- `videos/chunk-XXX/observation.images.<name>/episode_YYY.mp4` — H.264 videos (one per camera per episode)

**Keep the video as video, not as PNG frames.** The format is optimized for fast loading via decord/torchcodec; storing as PNGs bloats the dataset 10×.

## Recording from this project

Our teleop pipeline already produces the right signals:
- `/teleop/camera/image` or `/cameras/<name>/image_raw` — camera
- `/joint_states` — proprioception
- `/mycobot_controller/joint_trajectory` — commanded action
- The operator types a language instruction per episode

### Recording approach

```bash
# In the hand-teleop conda env
conda activate hand-teleop

# Record a single episode with an instruction
python teleop/record_episode.py \
  --instruction "pick the red cube and place it on the plate" \
  --output datasets/mycobot_lerobot/episode_001/ \
  --cameras front,wrist \
  --duration 30
```

(The `record_episode.py` script does not yet exist — it's on the VLA-prep backlog. Subscribe to camera + joint_states + joint_trajectory, write to parquet + mp4.)

### Quality bar — reject any episode that

- Has > 200 ms of camera dropout
- Starts or ends mid-motion (operator must pause at rest pose before/after)
- Contains "oops, redo" corrections (either trim them out or drop the episode)
- Has an instruction that doesn't match what the operator did

## Converting a ROS2 bag → LeRobot

If you have historical rosbags from real sessions:

```bash
python training/data/bag_to_lerobot.py \
  --bag rosbag2_2026_04_22-16_45_12/ \
  --output datasets/mycobot_lerobot_april/ \
  --camera-topic /cameras/front/image_raw \
  --state-topic /joint_states \
  --action-topic /mycobot_controller/joint_trajectory \
  --instruction-file instructions.txt
```

(Also not yet written — on the backlog.)

## Publishing to the HuggingFace hub

Once a dataset is validated:

```bash
pip install huggingface_hub  # in venv_dream or a dedicated env
huggingface-cli login
lerobot-dataset push datasets/mycobot_lerobot \
  --repo-id ABMI/mycobot-pick-and-place-v1 \
  --private
```

Keep datasets **private by default** — robot data from physical sessions may contain operators on camera, lab surroundings, etc.

## Dataset size targets

| Phase | Episodes | Purpose |
|-------|----------|---------|
| Smoke test | 10 | Verify the pipeline end-to-end; don't train a policy on this |
| LoRA fine-tune | 100–500 | First useful fine-tune of OpenVLA / Octo |
| POC-ready | 500–2000 | Reasonable task generalization; still task-specific |
| Production | 10K+ | Mutliple tasks, multi-operator, multi-session |

At ~30 s per episode, 500 episodes ≈ 4 h of recording. Reasonable for a focused session + break.

## Integration with the dashboard

Once the recording scripts exist, add a **🔴 Record episode** button to the teleop dashboard Home tab (ActionButton pattern — see [`teleop/teleop_dashboard.py`](../../../teleop/teleop_dashboard.py)). Clicking pops a modal for the instruction, records for a user-set duration (default 30 s), writes the episode, and toasts the path.

## Never

- Never mix datasets recorded at different FPS / camera resolutions — the VLA trainer will fail silently
- Never publish a dataset before verifying a visual sample (unreadable MP4s are a common bug)
- Never strip the `episode_index` and `task_index` fields to "simplify" — they're load-bearing

## References

- LeRobot docs: https://huggingface.co/docs/lerobot/
- Dataset format spec: https://huggingface.co/docs/lerobot/en/lerobot-dataset-format
- Ported data from the R5A / LeRobot project (reused in our teleop filters) — same format applies
