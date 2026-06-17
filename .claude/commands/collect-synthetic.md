---
description: Collect a synthetic dataset in Gazebo (domain-randomized world, 4 cameras)
---

Generate synthetic poses with FK-based anti-collision and domain randomization.

## Steps

```bash
conda deactivate
source /opt/ros/jazzy/setup.bash
source ~/ros_jazzy/src/mycobot_R6A/install/setup.bash

# Collector v3 — world randomized_v2 (6 lights, 12 clutter objects, 3 walls)
ros2 launch mycobot_gateway synthetic_data_v3.launch.py num_samples:=7500
```

## What the launch does

1. Starts Gazebo Harmonic with [`mycobot_description/worlds/randomized_v2.sdf`](../../mycobot_description/worlds/randomized_v2.sdf)
2. Spawns the URDF with `gz_ros2_control`
3. Runs the collector node that:
   - Samples random joint configurations within the **corrected** URDF limits
   - Runs FK, rejects poses where:
     - EE drops below the table (z < 2 cm)
     - Arm passes through its own base column
     - Elbow height is invalid
     - `|j2 + j3| > 3.8 rad` (extreme fold-back)
   - Captures 4 cameras (front / right / left / top) to `/tmp/dream_data/synthetic_..._v2/`
   - Randomizes lighting + materials per sample

Typical rejection rate: **~35%** of candidate poses.

## Monitor progress

```bash
bash scripts/monitor_collection.sh
```

Watches the output dir, reports frames/sec and estimated time to completion.

## Convert to NDDS format (for DREAM)

```bash
source ~/ros_jazzy/venv_dream/bin/activate
python training/dream/convert_to_ndds.py \
  --input /tmp/dream_data/synthetic_50k_v2 \
  --output /tmp/dream_data/synthetic_50k_v2_ndds
```

## Do not

- Collect with the **v1 world** ([`randomized.sdf`](../../mycobot_description/worlds/randomized.sdf)) for new datasets — it's simpler but produces too-clean backgrounds that widen the sim-to-real gap.
- Skip the FK anti-collision — the ~35% rejected poses are physically unrealistic and poison training.

**Docs:** [`docs/SYNTHETIC_DATA.md`](../../docs/SYNTHETIC_DATA.md), [`CHANGELOG.md`](../../CHANGELOG.md) entry 1.8.0
