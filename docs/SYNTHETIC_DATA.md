# 🤖📸 Synthetic Data Pipeline — MyCobot 320 Pi Pose Estimation

## Goal

Generate a labelled dataset of **(image, joint_angles)** pairs inside Gazebo
to train an AI model that predicts the MyCobot 320 Pi pose from a single
camera image in real-time.

## Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                     GAZEBO HARMONIC                          │
│                                                              │
│  ┌────────────┐        ┌──────────────┐                     │
│  │ MyCobot 320│  seen  │ synth_camera │                     │
│  │  (URDF)    │───────▶│  (640×480)   │                     │
│  └─────┬──────┘        └──────┬───────┘                     │
│        │ /joint_states        │ /synth_camera/image          │
└────────┼──────────────────────┼──────────────────────────────┘
         │  ros_gz_bridge       │  ros_gz_image bridge
         ▼                      ▼
┌──────────────────────────────────────────────────────────────┐
│                    ROS2 JAZZY                                │
│                                                              │
│        ┌─────────────────────────────┐                       │
│        │  synthetic_data_collector   │                       │
│        │                             │                       │
│        │  1. Random joint command ───┼──▶ Gz joint controllers│
│        │  2. Wait settle_time        │                       │
│        │  3. Capture image + angles  │                       │
│        │  4. Save to disk            │                       │
│        └─────────────────────────────┘                       │
│                      │                                       │
└──────────────────────┼───────────────────────────────────────┘
                       ▼
              /tmp/mycobot_synth_dataset/
              ├── images/
              │   ├── 000000.png
              │   ├── 000001.png
              │   └── ...
              └── labels.csv
```

## Quick Start

### 1. Build

```bash
cd ~/ros_jazzy/src/mycobot_R6A

# Clean environment
env -i HOME=$HOME PATH="/usr/bin:/bin:/opt/ros/jazzy/bin" DISPLAY=$DISPLAY bash

source /opt/ros/jazzy/setup.bash
colcon build --packages-select mycobot_description mycobot_gateway --symlink-install
source install/setup.bash
```

### 2. Collect data

```bash
# Collect 1000 samples (default)
ros2 launch mycobot_gateway synthetic_data.launch.py

# Collect 500 samples to a custom directory
ros2 launch mycobot_gateway synthetic_data.launch.py \
    num_samples:=500 \
    output_dir:=/home/$USER/datasets/mycobot_pose_v1

# Faster collection (less settle time, noisier)
ros2 launch mycobot_gateway synthetic_data.launch.py \
    num_samples:=2000 settle_time:=0.8
```

### 3. Inspect the output

```bash
ls /tmp/mycobot_synth_dataset/
# images/  labels.csv

head /tmp/mycobot_synth_dataset/labels.csv
# index,j1_rad,j2_rad,j3_rad,j4_rad,j5_rad,j6_rad,j1_deg,j2_deg,...,image_path
# 0,0.3412,-1.2045,...,images/000000.png
```

## Dataset Format

### labels.csv columns

| Column | Description |
|--------|-------------|
| `index` | Sample number (0-based) |
| `j1_rad` … `j6_rad` | Ground-truth joint angles in **radians** |
| `j1_deg` … `j6_deg` | Same angles in **degrees** (convenience) |
| `image_path` | Relative path to the PNG image |

### Joint order

| Index | URDF Joint Name | Description |
|-------|-----------------|-------------|
| j1 | `joint2_to_joint1` | Base rotation |
| j2 | `joint3_to_joint2` | Shoulder |
| j3 | `joint4_to_joint3` | Elbow |
| j4 | `joint5_to_joint4` | Wrist pitch |
| j5 | `joint6_to_joint5` | Wrist roll |
| j6 | `joint6output_to_joint6` | End-effector |

## Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `num_samples` | 1000 | Number of samples to collect |
| `output_dir` | `/tmp/mycobot_synth_dataset` | Where to save images + CSV |
| `settle_time` | 1.5 | Seconds to wait between command and capture |
| `image_topic` | `/synth_camera/image` | Camera topic from Gazebo |
| `joint_limit_fraction` | 0.7 | Use 70% of each joint's range |

## Camera Setup

The camera is mounted at a fixed position in the Gazebo world:
- **Position:** x=0.8m, y=0.0m, z=0.4m (in front and above the robot)
- **Resolution:** 640 × 480 @ 10 Hz
- **Field of view:** 60°

To change the camera position, edit the `<joint name="world_to_camera">` 
origin in `mycobot_pro_320_pi_gazebo.urdf`.

## Next Steps: Training

Once you have collected enough data, you can train a pose estimation model:

```python
import pandas as pd
from PIL import Image
from torch.utils.data import Dataset

class MycobotPoseDataset(Dataset):
    def __init__(self, csv_path, transform=None):
        self.df = pd.read_csv(csv_path)
        self.root = os.path.dirname(csv_path)
        self.transform = transform

    def __len__(self):
        return len(self.df)

    def __getitem__(self, idx):
        row = self.df.iloc[idx]
        img = Image.open(os.path.join(self.root, row['image_path']))
        angles = row[['j1_rad','j2_rad','j3_rad','j4_rad','j5_rad','j6_rad']].values.astype(float)
        if self.transform:
            img = self.transform(img)
        return img, torch.tensor(angles, dtype=torch.float32)
```

Suggested architectures:
- **ResNet-18/34** backbone → 6-neuron regression head
- **EfficientNet-B0** for better accuracy/speed trade-off
- **MobileNetV3** for real-time inference on edge devices

## Domain Randomization (Future)

To improve real-world transfer, consider adding:
- [ ] Random lighting conditions
- [ ] Random background textures
- [ ] Camera position jitter
- [ ] Random table/objects around the robot
- [ ] Noise/blur on captured images
