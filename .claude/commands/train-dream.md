---
description: Run a DREAM training cycle (keypoint detection, VGG-19 backbone)
---

Train a DREAM model for MyCobot keypoint detection. Always in the dedicated `venv_dream`.

## Activate the right env

```bash
conda deactivate   # must be clean
source ~/ros_jazzy/venv_dream/bin/activate
```

## Data preparation

If training on a fresh merge of real + synthetic frames:

```bash
python training/dream/merge_and_convert.py \
  --real /tmp/dream_data/real_cam0 \
  --synth /tmp/dream_data/synthetic_50k \
  --output /tmp/dream_data/mixed_v2 \
  --real-oversample 5
```

`--real-oversample` repeats each real frame N times so the ratio stays tractable when synthetic vastly outweighs real.

## Training (native DREAM script)

```bash
python /tmp/DREAM/scripts/train_network.py \
  -i /tmp/dream_data/mixed_v2 \
  -m /tmp/DREAM/manip_configs/mycobot320.yaml \
  -ar /tmp/DREAM/arch_configs/dream_vgg_q.yaml \
  -e 25 -b 32 -lr 0.0001 \
  -o training/checkpoints_dream/vgg_mixed_v2 -f
```

## Or: the scripted pipeline

```bash
bash scripts/train_pipeline.sh
```

This runs merge + convert + training with known-good parameters.

## Evaluation

On real data:
```bash
python training/dream/evaluate_dream.py \
  --weights training/checkpoints_dream/vgg_mixed_v2/best_network.pth \
  --data /tmp/dream_data/real_cam0 --split all
```

On synthetic (regression check):
```bash
python training/dream/evaluate_dream.py \
  --weights training/checkpoints_dream/vgg_mixed_v2/best_network.pth \
  --data /tmp/dream_data/synthetic_50k --split val
```

## Known-good reference points

- VGG-base (50K synth only): 98.3% detection · 3.15 px median (synth) · 13.2% (real)
- VGG-aug (25 epochs, synth): val = 0.000667 · 96.6% det (synth) · ~26% (real)
- Mixed 18K (10K real ×5 + 8K synth): epoch 1 val = 0.000474 — **real-data eval still pending** as of last checkpoint in `SESSION_RESUME.md`

## Do not

- Train with `σ=4` using the custom `finetune_real.py` — known bug (belief maps collapse, 0% detection). Use the native DREAM script.
- Delete `training/checkpoints_dream/` without copying successful weights first.

**Docs:** [`training/dream/`](../../training/dream/), [`SESSION_RESUME.md`](../../SESSION_RESUME.md), [`CHANGELOG.md`](../../CHANGELOG.md) entries 1.5.0 / 1.6.0 / 1.7.0
