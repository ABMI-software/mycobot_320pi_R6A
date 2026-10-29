---
name: dream-workflow
description: Guidance for DREAM keypoint-detection workflows — dataset prep, training choice, sim-to-real gap analysis, evaluation in the paper's metrics. Invoke when asked to train, fine-tune, or evaluate a DREAM model, or to reason about why detection is low on real images.
---

# DREAM workflow

The project uses NVlabs DREAM (VGG-19 → 7 belief maps → PnP → pose). State on
29/10/2026: in-domain the model reaches 1.7–3 px; synthetic-only does not
transfer to the real bench yet. Work feeding the paper follows
[`article-dream`](../article-dream/SKILL.md) and
[`docs/ARTICLE_DREAM.md`](../../../docs/ARTICLE_DREAM.md).

## Cheat sheet — which approach for which symptom

| Symptom | Action |
|---------|--------|
| Fresh synthetic training | `train_dream_ultimate_v5_geo.py` on a free-camera dataset (`synthetic_data_collector_v3_camera_libre.py`) |
| Synthetic OK, real low | Appearance randomisation and camera diversity first (paper § II.C); real fine-tuning is the upper bound, not the fix |
| A keypoint does not follow an image shift | Memorisation — check the augmentation (v4 and older are broken) and the camera diversity of the data |
| Belief maps flat, MSE low | Collapse: judge on detection and PCK, not on the loss |
| Good on test, bad live | Test holds out poses but not a viewpoint, a day or a scene; or the live pose is out of domain (observation pose, J5 ≈ 2°) |

## Models in service

| Checkpoint | Use | Note |
|---|---|---|
| `vgg_tri_mix_ft_e10` | Gazebo sort scene | 99.7 % / 3.0 px on seeds 2001-2003 |
| `vgg_tri_reel_ft_e10` | Real bench | PCK@2.5/5/10 = 0.760 / 0.941 / 0.959 on `real_tri_test` (same day, same 2 cameras) |
| `vgg_montage0901_ft_e30` | — | Invalid: memorised base |

## Evaluation

```bash
conda deactivate && source ~/ros_jazzy/venv_dream/bin/activate
cd training/dream
# paper metric: undetected keypoints count as wrong
taskset -c 16-27 python eval_papier.py --weights <ckpt>/best_network.pth \
    --data <ndds_dir> --sortie <out_dir>
# then the shift test on real images (no motion; captures both cameras, or --images)
python ../../scripts/dream_test_decalage.py --modele <ckpt_dir_name> --images a.png b.png
```

`evaluate_dream.py` still gives per-keypoint mean/median/max, but its
"% < N px" counts detected keypoints only: never quote it as a PCK.

## Red flags

- `val` improving while detection drops → belief maps collapsing.
- Loss NaN in the first 2 epochs → LR too high or bad shuffle.
- `import dream` fails → you are outside `venv_dream`; the library is `~/DREAM`, installed editable there (the `/tmp/DREAM` paths left in old scripts are dead but harmless inside the venv).

## When to spawn the dream-trainer agent

Any multi-epoch run, ablation or full evaluation sweep: delegate to
`dream-trainer` so the main session stays responsive.

**Docs:** [`training/dream/README.md`](../../../training/dream/README.md), [`docs/ARTICLE_DREAM.md`](../../../docs/ARTICLE_DREAM.md), [`CLAUDE.md`](../../../CLAUDE.md) § DREAM
