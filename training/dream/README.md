# DREAM Keypoint-Based Pose Estimation for MyCobot 320 Pi

This module implements [DREAM](https://github.com/NVlabs/DREAM) (Deep Robot-to-camera
Extrinsics for Articulated Manipulators) for the MyCobot 320 Pi robot arm.

## Overview

Instead of regressing joint angles directly from pixels (which fails when the robot
is small in frame), DREAM detects **keypoint positions** (joint locations) in the image
using belief maps, then solves the camera-to-robot pose via **PnP** (Perspective-n-Point).

**Pipeline:**
```
Image → CNN (ResNet-101 + Hourglass) → 7 Belief Maps → Peak Detection → 2D Keypoints
                                                                              ↓
                        3D Keypoints (from URDF FK) → PnP Solver → Camera Pose + Joint Angles
```

## Architecture

Two architectures were tested:

### VGG-Q (recommended)
- **Backbone:** VGG-19 (pretrained on ImageNet, no BatchNorm)
- **Head:** 6 cascaded refinement stages (DOPE-style)
- **Input:** 400×400 RGB image
- **Output:** 7 belief maps (100×100) — one per keypoint
- **Parameters:** 28.2M
- **Advantage:** Stable training, no BN issues with small batches

### ResNet-H
- **Backbone:** ResNet-101 (pretrained on ImageNet)
- **Head:** Hourglass decoder with deconvolutions
- **Output:** 7 belief maps (208×208)
- **Parameters:** 54.0M
- **Warning:** Unstable validation loss with batch_size < 64 due to BatchNorm

**Common:**
- **Loss:** MSE on belief maps
- **Keypoints:** base, link1–link6 (7 total, matching URDF frames)

## Training Results

### VGG-Q on Synthetic Data (20K frames, 4 cameras, 5000 poses)

| Model | Epochs | Val Loss | Stability |
|-------|--------|----------|-----------|
| VGG (no aug) | 25 | 0.000438 | ✅ Stable |
| VGG (augmented) | 25 | 0.000667 | ✅ Stable |
| ResNet-H | 25 | 0.000305* | ❌ Wildly unstable (BN issue) |

*ResNet best loss was from epoch 1 only; later epochs oscillated 0.0003–96.0 due to BN instability.

### Keypoint Detection Accuracy (VGG-aug, synthetic validation, detected only)

| Keypoint | Detection | Mean | Median | <5px | <10px | <20px |
|----------|-----------|------|--------|------|-------|-------|
| base | 100% | 2.9px | 2.8px | 100% | 100% | 100% |
| link1 | 100% | 2.7px | 2.6px | 100% | 100% | 100% |
| link2 | 100% | 2.7px | 2.6px | 100% | 100% | 100% |
| link3 | 99% | 11.0px | 5.6px | 45% | 74% | 88% |
| link4 | 96% | 19.2px | 6.4px | 39% | 65% | 80% |
| link5 | 95% | 27.4px | 8.8px | 26% | 53% | 70% |
| link6 | 86% | 28.9px | 10.1px | 19% | 50% | 69% |
| **ALL** | **97%** | **13.1px** | **3.1px** | **63%** | **78%** | **87%** |

### Sim-to-Real Transfer

| Model | Synthetic | Real |
|-------|-----------|------|
| VGG synth-only (20K) | 97% det, 3.1px | ~26% det |
| VGG synth-only (50K, ancien) | 98.3% det, 3.15px | 13.2% det, 172px |
| VGG-aug + augmentation | ~97% det | 25.7% det (marginal improvement) |
| vgg_ultimate_v2_e50 | 97.7% det | ~26% det |
| vgg_ultimate_v4_e50 (2026-07-06) | **99.4% det, 2.61px mean** | ≈27% det (transfert bloqué malgré le gain synthétique) |
| **vgg_ultimate_v4_mix_ft_e30 (2026-07-08)** | 99.4% det (pas de régression) | **91.6% det** (1500 frames, 3 caméras, jamais vues) |

Détail complet des deux runs ci-dessous.

---

### Dataset synthétique 50k (v4) — généré 2026-07-02, entraîné 2026-07-02 → 06

12 500 poses × 4 caméras (front/right/left/top) = 50 000 images. Échantillonnage
uniforme sur les limites **pratiques** (pas les limites constructeur — une pose
au bord théorique est presque toujours en auto-collision ou hors-table), avec
filtre géométrique d'anti-collision et un rendu de lumière calé sur les
conditions réelles de capture (bureau, lumière neutre). Détails complets :
[`../SYNTHETIC_50K_V3.md`](../SYNTHETIC_50K_V3.md), rapport source
`RAPPORT_SYNTHETIC_50K_V4.docx`.

**Contrainte de bras robotique — limites constructeur vs limites en pratique**

| Joint | Limite constructeur | Limite en pratique |
|-------|---------------------|---------------------|
| J1 | −165° ~ +165° | **±168°** |
| J2 | −165° ~ +165° | **±135°** |
| J3 | −165° ~ +165° | **±150°** |
| J4 | −165° ~ +165° | **±145°** |
| J5 | −165° ~ +165° | **±165°** |
| J6 | −175° ~ +175° | **±180°** |

**Limites des joints en pratique (échantillonnage uniforme, `synthetic_data_collector_v3.py`)**

| Joint | Rôle | Limite (rad) | Limite (°) |
|-------|------|--------------|------------|
| J1 | Rotation base | ±2.9322 | ±168° |
| J2 | Épaule | ±2.3562 | ±135° |
| J3 | Coude | ±2.6180 | ±150° |
| J4 | Poignet 1 | ±2.5307 | ±145° |
| J5 | Poignet 2 | ±2.8798 | ±165° |
| J6 | Poignet 3 (flange) | ±3.1416 | ±180° |

Deux règles géométriques (FK) filtrent en plus les poses impossibles :
- **R1 — Garde-fou table** : aucun point du bras ne descend sous 13 cm de la
  table (garde l'épaule à 16,2 cm).
- **R2 — Anti-repli base** : les liens distaux ne reviennent pas dans le
  cylindre de base (rayon 5,5 cm).
- **Vérification « Settle »** : une pose n'est enregistrée que si le robot
  l'atteint réellement (tolérance 2°) — élimine les blocages physiques réels
  (voir aussi le filtre capsule ci-dessous, qui élimine les poses repliées
  sur elles-mêmes en amont).

**Anticollision — modèle capsule (segment + rayon) par lien**

Collision ⟺ `d < r_i + r_j + marge`, avec marge de sécurité = 0.005 m.

| Lien | Description | Rayon (m) |
|------|-------------|-----------|
| L0 | Colonne de base | 0.055 |
| L1 | Bras supérieur | 0.045 |
| L2 | Avant-bras | 0.040 |
| L3 | Poignet | 0.036 |
| L4 | Flange | 0.045 |

Taux de rejet sur 20 000 poses uniformes testées : 24,6 % (vs 17,8 % pour
l'ancien filtre — voir [`../SYNTHETIC_50K_V3.md`](../SYNTHETIC_50K_V3.md) §1).

**Éclairage — reproduire un rendu de type « soleil » de bureau**

Randomisé par pose (`_randomize_lights()` dans `synthetic_data_collector_v3.py`),
calé sur les conditions réelles de capture arducam/svpro/astra (bureau
lumineux, lumière neutre-blanche, ombres douces plutôt que dures).

| Nom | Type | Rôle |
|-----|------|------|
| `sun` | directionnelle | Lumière principale façon soleil/plafonnier, projette des ombres (`cast_shadows=true`) |
| `fill_light` | directionnelle | Remplissage, adoucit les ombres dures, pas d'ombre propre |
| `back_light` | directionnelle | Contre-jour (rim light), atténué |
| `warm_point` | ponctuelle | Point chaud proche du robot, position randomisable |
| `cool_point` | ponctuelle | Point froid côté opposé, léger contraste chaud/froid |
| `overhead_spot` | ponctuelle | Spot au plafond, projette des ombres |

**Résultat final — validation synthétique 50k** (`evaluate_dream.py`, split val
40 000–50 000, 2000 frames, 2026-07-06 · meilleure époque **49/50**)

| Keypoint | Mean (px) | Median (px) | Std | Max | Det% |
|----------|-----------|-------------|-----|-----|------|
| base | 3.47 | 3.38 | 0.17 | 3.89 | 100.0% |
| link1 | 3.20 | 3.17 | 0.21 | 3.64 | 100.0% |
| link2 | 3.20 | 3.18 | 0.21 | 3.65 | 100.0% |
| link3 | 1.88 | 1.61 | 2.40 | 51.88 | 99.8% |
| link4 | 2.11 | 1.69 | 4.55 | 97.69 | 100.0% |
| link5 | 2.11 | 1.59 | 5.15 | 127.65 | 99.2% |
| link6 | 2.30 | 1.77 | 5.35 | 112.54 | 97.0% |
| **OVERALL** | **2.61** | **2.78** | **3.46** | 127.65 | **99.4%** (13920/14000) |

Précision par seuil : 37,1% <2px · 98,8% <5px · 99,5% <10px · 99,7% <20px ·
99,9% <50px. Erreur moyenne par frame : 2,62 ± 2,33 px. **Dépasse le record
v2 (97,7 %)**. Détails : [`../VGG_ULTIMATE_V4_50K.md`](../VGG_ULTIMATE_V4_50K.md).

---

### Fine-tune mixte réel (real_3cam ×5 oversampling) — 2026-07-03 → 2026-07-08

Objectif : combler l'écart sim→réel (v4 plafonnait à ≈27 % de détection réel
malgré 99,4 % en synthétique) sans perdre l'acquis synthétique. Méthode :
fine-tune (pas from-scratch) depuis `vgg_ultimate_v4_e50/best_network.pth`.

**Composition du dataset de fine-tune**

- Synthétique : `synthetic_50k_ndds` (50 000 frames, inchangé).
- Réel : `real_3cam` (2500 poses × 3 caméras arducam/svpro/astra = 7500
  images), split **par pose** (pas par image, pour éviter une fuite train/val)
  → 2000 poses train (**6000 images**) / 500 poses val (**1500 images**,
  jamais oversamplées, jamais vues en entraînement).
- Oversampling ×5 du train réel : 6000 → **30 000 images**.
- Fusion NDDS : 50 000 (synth) + 30 000 (réel ×5) = **~80 000 frames** au total.
- `ShiftScaleRotate scale_limit` élargi de 0.1 → **0.3** (couvre l'écart de
  focale mesuré jusqu'à +23,5 % pour l'astra non calibrée vs la caméra
  synthétique).
- Poids par keypoint inchangés `[1,1,1,1,1.5,1.5,6.0]`.
- **30 epochs demandées, meilleure à l'epoch 27**, val_loss 0,000942,
  **13,8 h** d'entraînement (`vgg_ultimate_v4_mix_ft_e30`).

Méthodologie complète, y compris le recalage manuel des extrinsèques caméra
réelles (2026-07-03, préalable indispensable — les anciens placeholders
plaçaient la vérité-terrain 2D à côté du bras) :
[`../FINETUNE_MIX_REAL3CAM_PLAN.md`](../FINETUNE_MIX_REAL3CAM_PLAN.md).

**Évaluation réelle — méthodologie et correction d'un biais d'échantillonnage**

`evaluate_dream.py` en mode par défaut (500 frames, step=3,0) tombait en phase
avec l'ordre des 3 caméras et n'évaluait quasiment que l'arducam → détection
gonflée artificiellement sur les keypoints distaux. Corrigé avec
`--max-samples 1500` (couvre les 500 poses val × 3 caméras en entier).

| Keypoint | Det% (arducam seul, biaisé, 500 frames) | Det% (3 caméras, complet, 1500 frames) | Note |
|----------|------------------------------------------|-------------------------------------------|------|
| base / link1 / link2 | 100% | 100% | — |
| link3 | 96,8% | 97,3% | — |
| link4 | 87,8% | 89,8% | — |
| link5 | 61,6% | 75,7% | svpro/astra meilleurs qu'arducam |
| link6 | 67,0% | 78,4% | svpro/astra meilleurs qu'arducam |
| **OVERALL** | **87,6%** (3066/3500) | **91,6%** (9618/10500) | — |

**Résultats finaux — évaluation complète (1500 frames, 3 caméras, 500 poses
jamais vues, 2026-07-08)**

| Keypoint | Mean (px) | Median (px) | Std | Max | Det% |
|----------|-----------|-------------|-----|-----|------|
| base | 1,59 | 1,59 | 0,58 | 10,93 | 100,0% |
| link1 | 1,41 | 1,56 | 1,01 | 17,74 | 100,0% |
| link2 | 1,41 | 1,56 | 1,01 | 17,74 | 100,0% |
| link3 | 10,00 | 7,22 | 9,53 | 87,05 | 97,3% |
| link4 | 21,14 | 15,90 | 19,07 | 161,09 | 89,8% |
| link5 | 29,20 | 21,85 | 26,40 | 234,99 | 75,7% |
| link6 | 34,44 | 27,44 | 27,24 | 231,21 | 78,4% |
| **OVERALL** | **12,82** | **2,91** | **19,95** | 234,99 | **91,6%** (9618/10500) |

Précision par seuil : 35,1% <2px · 54,4% <5px · 64,8% <10px · 78,6% <20px ·
94,4% <50px. Erreur moyenne par frame : 12,67 ± 9,28 px (meilleure frame
0,83 px, pire frame 100,25 px). Les keypoints distaux (link4/5/6) restent le
point faible relatif mais ont le plus progressé pendant le fine-tune (+27–33%
de MSE) — voir `CHANGELOG.md` [1.13.0].

---

> **Historique (avant le fine-tune mixte du 2026-07-08 ci-dessus, qui a fermé
> l'écart sim→réel à 91,6 %)** — analyse et pistes envisagées quand le
> transfert était encore bloqué à ≈27 % :

**Belief maps are 10× weaker on real images** (peaks 0.02–0.25) vs synthetic (0.5–1.0).
Aggressive augmentation (HueSaturation, GaussianBlur, CLAHE, CoarseDropout) gave only
marginal improvement (22.9% → 25.7%).

**Recommended next steps to close the domain gap:**
1. **Self-supervised labeling** — use robot joint angles + FK + calibrated camera intrinsics
   to auto-generate 2D keypoint GT on real images, then fine-tune
2. **Domain randomization v2** — collect with `randomized_v2.sdf` (6 lights, 12 clutter objects)
3. **Style transfer** (CycleGAN) — translate Gazebo renders to real-camera style

**What NOT to do:**
- Do not fine-tune DREAM manually with custom loss (MSE on near-empty belief maps → all-zeros)
- Do not use `sigma=4` (DREAM native uses `sigma=2`)
- Always use `train_network.py` from NVlabs/DREAM for training

## Joint-Angle Observability from a Single Camera (2026-07-13)

The dashboard's "DREAM angle estimate" (`dream_validation_dashboard.py`,
`estimate_dream_angles()`) recovers joint angles from one monocular view + the
7 DREAM keypoints, against a frozen session pose anchor. Two structural limits
apply, confirmed empirically rather than assumed:

- **J6**: no keypoint depends on it at all (`JOINT_OBSERVING_KP` in the
  dashboard) — its own rotation axis passes through its own keypoint. Always
  flagged unreliable.
- **J5**: only `link6` depends on it — one 2D point constraining one DoF.
  `j5_observability_test.py` sweeps J5 across its full mechanical range with
  every other joint and the camera pose held fixed, and measures how much
  link6 actually moves per degree. At the real camera's mounting angle, an
  ambiguity band of **~15° (front view) to ~23° (top-down)** exists where
  different J5 values reproject within typical detector noise (5px) of each
  other — indistinguishable even to a perfect detector at that pose. Side-on
  views do better (~9°) but are still borderline.

This is a **geometry problem, not a solver-tuning problem**: increasing
`reg_weight` or adding a filter would narrow the *reported* J5 error without
resolving the underlying ambiguity — it masks the issue rather than fixing it.
Left untouched per the 2026-07-13 investigation conclusion.

**Improvement paths (not yet implemented):**
1. A second camera at a different viewing angle — turns the underdetermined
   1-point/1-DoF problem into an overdetermined multi-view one.
2. A stronger geometric prior (temporal smoothness, or a learned pose prior) —
   doesn't add information, but constrains the solver's search within the
   ambiguity band instead of letting it wander.

Evidence: `j5_observability_test.py` + `j5_observability_test*.png` (3
viewpoints tested). Camera pose for the test is a synthetic look-at built
directly in code — no calibration file involved.

Related: the dashboard's session pose anchor was independently found to be
underconstrained when built from a single static startup pose (same root
cause: single-view PnP rotation ambiguity on a near-collinear keypoint chain).
Fixed the same day by requiring pose diversity (≥3 distinct poses, ≥15° spread
on ≥2 joints) before pooling correspondences across the whole accumulation
window into one fit — see `CHANGELOG.md` [1.16.0].

## Files

| File | Description |
|------|-------------|
| `mycobot_fk.py` | Forward kinematics — computes 3D joint positions from angles |
| `mycobot_ik.py` | Inverse kinematics (Jacobian-based numerical solver) |
| `dream_angle_solver.py` | Recovers joint angles from 2D keypoints (fixed-pose and joint+pose variants) |
| `j5_observability_test.py` | J5 observability sensitivity sweep — see section above |
| `convert_to_ndds.py` | Converts our datasets to DREAM's NDDS format |
| `merge_and_convert.py` | Merges real + synthetic datasets with oversampling → NDDS |
| `train_dream.py` | Training wrapper (calls DREAM's train_network.py) |
| `train_dream_augmented.py` | Training with aggressive augmentation for sim-to-real |
| `train_dream_weighted.py` | Training with per-keypoint loss weighting (link6 × 5.0) |
| `evaluate_dream.py` | Comprehensive evaluation with per-keypoint metrics |
| `infer_dream.py` | Inference — keypoint detection + PnP solving |
| `visualize_ndds.py` | Sanity check — overlays keypoint annotations on images |
| `finetune_real.py` | Custom fine-tuning (⚠️ non-functional — see Lessons Learned) |
| `manip_configs/mycobot320.yaml` | Manipulator keypoint configuration |

## Quick Start

### 1. Prerequisites

```bash
# Clone DREAM
git clone https://github.com/NVlabs/DREAM.git /tmp/DREAM
cd /tmp/DREAM && pip install -e . -r requirements.txt
```

### 2. Convert Data to NDDS Format

```bash
# Convert synthetic data (all 4 cameras → 20K frames)
python convert_to_ndds.py \
    --input /tmp/mycobot_synth_v2 \
    --output /tmp/dream_data/synthetic \
    --source synth \
    --cameras front right left top

# Verify with visualization
python visualize_ndds.py --data /tmp/dream_data/synthetic --num 20
```

### 3. Train

```bash
# Train VGG on synthetic data (recommended — stable, no BN issues)
python train_dream.py \
    --data /tmp/dream_data/synthetic \
    --arch vgg \
    --epochs 25 \
    --batch-size 32 \
    --lr 0.0001

# Train VGG with aggressive augmentation (for sim-to-real)
python train_dream_augmented.py \
    --data /tmp/dream_data/synthetic \
    --arch vgg \
    --epochs 25 \
    --batch-size 32 \
    --lr 0.0001

# Output: checkpoints_dream/vgg_*_e25/
```

### 4. Evaluate

```bash
# Evaluate on synthetic validation split
python evaluate_dream.py \
    --weights checkpoints_dream/vgg_augmented_e25/best_network.pth \
    --data /tmp/dream_data/synthetic \
    --split val \
    --max-samples 500 \
    --visualize

# Single image inference with PnP
python infer_dream.py \
    --model checkpoints_dream/vgg_augmented_e25/best_network.pth \
    --image /path/to/image.png
```

## Keypoint Configuration

The 7 keypoints correspond to the MyCobot 320 URDF link frames:

| Keypoint | URDF Frame | Description |
|----------|-----------|-------------|
| `mycobot320_base` | `base` | Robot base (fixed) |
| `mycobot320_link1` | `link1` | After joint 1 (yaw) |
| `mycobot320_link2` | `link2` | After joint 2 |
| `mycobot320_link3` | `link3` | After joint 3 |
| `mycobot320_link4` | `link4` | After joint 4 |
| `mycobot320_link5` | `link5` | After joint 5 |
| `mycobot320_link6` | `link6` | End-effector |

Note: link1 and link2 have the same origin in our URDF (zero offset at joint2→joint3),
so they will always project to the same pixel. This is expected and does not hurt training.

## Camera Intrinsics

**Gazebo (synthetic):**
- Resolution: 640×480
- HFOV: 1.047 rad (60°)
- fx = fy = 554.38 px
- cx = 320.0, cy = 240.0

**Real cameras:** Need calibration. Default assumes fx = fy ≈ 610 px.

## References

- [DREAM Paper (ICRA 2020)](https://arxiv.org/abs/1911.09231)
- [NVlabs/DREAM GitHub](https://github.com/NVlabs/DREAM)
- [NDDS (NVIDIA Deep learning Dataset Synthesizer)](https://github.com/NVIDIA/Dataset_Synthesizer)
