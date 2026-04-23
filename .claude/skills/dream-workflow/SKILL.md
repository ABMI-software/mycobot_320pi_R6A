---
name: dream-workflow
description: Guidance for DREAM keypoint-detection workflows — dataset prep, training choice, sim-to-real gap analysis. Invoke when asked to train, fine-tune, or evaluate a DREAM model, or to reason about why detection is low on real images.
---

# DREAM workflow

The project uses NVlabs DREAM (VGG-19 → 7 belief maps → PnP → pose). Domain gap between synthetic and real is the open problem as of 2026-04-21.

## Cheat sheet — which approach for which symptom

| Symptom | Action |
|---------|--------|
| Fresh start, no real data yet | VGG-base on synthetic (native `train_network.py`) |
| Synthetic OK, real < 30% detection | Mix real + synthetic with oversampling (`merge_and_convert.py --real-oversample 5`), train with native DREAM |
| Belief maps look flat on real images | Evaluate peak intensity — if << synthetic, the VGG didn't learn real-domain features. Consider a heavier augmentation config. |
| Tried `finetune_real.py` and got 0% detection | Known bug — **use native `train_network.py` instead.** The custom script has a sigma/belief-map issue documented in CHANGELOG 1.7.0. |
| Small keypoint distance errors (< 10 px) on synthetic, huge on real | Domain gap — add real-world augmentations (blur, JPEG compression, color jitter). |

## Data paths

- Synthetic (50K): `/tmp/dream_data/synthetic_50k/`
- Synthetic v2 (30K, randomized_v2 world): `/tmp/dream_data/synthetic_50k_v2/`
- Real (4K, cam0 + cam3): `/tmp/dream_data/real_cam0/`
- Mixed (10K real ×5 + 8K synth): `/tmp/dream_data/mixed_real_synth/`

## Current best weights

| Checkpoint | Performance |
|-----------|-------------|
| `vgg_weighted_50k_e50/best_network.pth` | Best on synthetic (98.3% det, 3.15 px) |
| `vgg_mixed_real_synth/best_network.pth` | Mixed training, epoch 1 val=0.000474 — real-data eval still pending |

## Evaluation cheat sheet

```bash
source ~/ros_jazzy/venv_dream/bin/activate

# Real data
python training/dream/evaluate_dream.py \
  --weights <checkpoint.pth> --data /tmp/dream_data/real_cam0 --split all

# Synthetic regression check
python training/dream/evaluate_dream.py \
  --weights <checkpoint.pth> --data /tmp/dream_data/synthetic_50k --split val
```

Look for:
- **% detection** per keypoint (base/link1…link6). base + link1 should be near 100% on real; link6 is the hardest.
- **Median px error** — 1 px ≈ 1.44 mm at the current camera distance.
- **Median mm error** is the physical number that matters for pick-and-place (target: ±5 mm).

## Signal conversion reference

| Keypoint px (median) | mm | Angular error |
|----------------------|----|----|
| 2.8 px | 4.0 mm | ~0.7° |
| 5.6 px | 8.1 mm | ~2.1° |
| 10.1 px | 14.6 mm | ~18.3° |

Pick-and-place requires ±5 mm → first 3 keypoints OK (base→link2), link6 insufficient by ~3×.

## Red flags

- A run that reports `val` improving but `detection rate` dropping → belief maps collapsing (classic DREAM instability). Kill the run, drop LR by 10×, restart.
- Training loss going NaN in the first 2 epochs → LR too high or bad batch. Check dataset shuffling.
- `%det > 90%` on synthetic but `%det < 5%` on real → textures and lighting too different. Add augmentations or switch to the `randomized_v2` world for new data.

## When to spawn the dream-trainer agent

If the user asks to launch a multi-epoch training run in the background and periodically evaluate — delegate to the `dream-trainer` agent so progress can be monitored without polluting the main session.

**Docs:** [`training/dream/`](../../../training/dream/), [`docs/ARCHITECTURE.md`](../../../docs/ARCHITECTURE.md), [`SESSION_RESUME.md`](../../../SESSION_RESUME.md)
