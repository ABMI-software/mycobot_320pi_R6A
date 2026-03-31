# MyCobot 320 Pi — Pose Estimation Training Pipeline

Train a CNN to predict the 6 joint angles of a MyCobot 320 Pi from a single
camera image.  The model is trained on synthetic data generated in Gazebo
Harmonic (see `docs/SYNTHETIC_DATA.md`).

## Quick Start

```bash
# 1. Install dependencies (conda env recommended)
pip install -r training/requirements.txt

# 2. Train (uses GPU automatically if available)
python3 training/train.py \
    --dataset /tmp/mycobot_synth_dataset \
    --epochs 100 \
    --batch-size 32

# 3. Evaluate on test split
python3 training/train.py \
    --dataset /tmp/mycobot_synth_dataset \
    --evaluate \
    --checkpoint training/checkpoints/best_model.pth

# 4. Predict on a single image
python3 training/predict.py \
    --image /tmp/mycobot_synth_dataset/images/000042.png \
    --checkpoint training/checkpoints/best_model.pth
```

## Architecture

```
Image (640×480 RGB)
       │
       ▼
  ResNet-18 backbone (pretrained ImageNet)
       │
       ▼
  AdaptiveAvgPool2d → flatten (512-d)
       │
       ▼
  FC 512 → 256 → ReLU → Dropout(0.3)
       │
       ▼
  FC 256 → 6  (joint angles in radians)
```

## Dataset Format

The training script expects a dataset directory with:
```
dataset_dir/
├── images/
│   ├── 000000.png
│   ├── 000001.png
│   └── ...
└── labels.csv   # columns: index, j1_rad..j6_rad, j1_deg..j6_deg, image_path
```

## Output

Training produces:
- `training/checkpoints/best_model.pth` — best validation loss
- `training/checkpoints/last_model.pth` — last epoch
- `training/checkpoints/training_log.csv` — per-epoch metrics
- `training/checkpoints/training_curves.png` — loss & MAE plots
