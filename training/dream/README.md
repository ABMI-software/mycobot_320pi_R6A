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

Tested on 2000 real camera images — detection rates are low (0–55%) due to the
large visual domain gap between Gazebo renders and real camera images. Domain
randomization or real-data fine-tuning is needed for production real-robot use.

## Files

| File | Description |
|------|-------------|
| `mycobot_fk.py` | Forward kinematics — computes 3D joint positions from angles |
| `convert_to_ndds.py` | Converts our datasets to DREAM's NDDS format |
| `train_dream.py` | Training wrapper (calls DREAM's train_network.py) |
| `train_dream_augmented.py` | Training with aggressive augmentation for sim-to-real |
| `evaluate_dream.py` | Comprehensive evaluation with per-keypoint metrics |
| `infer_dream.py` | Inference — keypoint detection + PnP solving |
| `visualize_ndds.py` | Sanity check — overlays keypoint annotations on images |
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
