# MyCobot 320 Pi — Pose Estimation Training Pipeline

## Approche Active : DREAM Keypoint Detection

Le pipeline principal utilise **DREAM** (NVlabs) — détection de keypoints par
belief maps (VGG-19) suivie d'une résolution PnP pour la pose 3D.

> ⚠️ L'ancienne approche par **régression directe** (ResNet → angles) est
> conservée dans `train.py` / `model.py` mais **abandonnée** (bloquée à 32° MAE
> sur données réelles).

### Architecture DREAM

```
Image 640×480 RGB
       │
       ▼
  Resize → 400×400
       │
       ▼
  VGG-19 backbone (ImageNet, sans BatchNorm)
       │
       ▼
  6 stages cascadés (style DOPE)
       │
       ▼
  7 belief maps 100×100 (1 par keypoint)
       │
       ▼
  Peak detection (Gaussian filter σ=3, seuil > 0.01)
       │
       ▼
  7 keypoints 2D (pixels)
       │
       ▼
  PnP (Levenberg-Marquardt) + FK 3D → Pose caméra [R|t]
```

### 7 Keypoints

| Keypoint | Frame URDF | Description |
|----------|-----------|-------------|
| `mycobot320_base` | `base` | Base (fixe) |
| `mycobot320_link1` | `link1` | Joint 1 (yaw) |
| `mycobot320_link2` | `link2` | Joint 2 |
| `mycobot320_link3` | `link3` | Joint 3 |
| `mycobot320_link4` | `link4` | Joint 4 |
| `mycobot320_link5` | `link5` | Joint 5 |
| `mycobot320_link6` | `link6` | End-effector |

## Quick Start — DREAM Training

### Prérequis

```bash
# Environnement Python (venv, pas conda — incompatible avec ROS2)
source ~/ros_jazzy/venv_dream/bin/activate

# DREAM doit être installé
# git clone https://github.com/NVlabs/DREAM.git /tmp/DREAM
# cd /tmp/DREAM && pip install -e . -r requirements.txt
```

### Entraîner sur dataset mixte (recommandé)

```bash
python /tmp/DREAM/scripts/train_network.py \
  -i /tmp/dream_data/mixed_real_synth \
  -m /tmp/DREAM/manip_configs/mycobot320.yaml \
  -ar /tmp/DREAM/arch_configs/dream_vgg_q.yaml \
  -e 25 -b 32 -lr 0.0001 \
  -o training/checkpoints_dream/vgg_mixed_real_synth -f
```

### Évaluer un modèle

```bash
# Sur données réelles
python training/dream/evaluate_dream.py \
  --weights training/checkpoints_dream/vgg_mixed_real_synth/best_network.pth \
  --data /tmp/dream_data/real_cam0 --split all

# Sur données synthétiques (validation)
python training/dream/evaluate_dream.py \
  --weights training/checkpoints_dream/vgg_mixed_real_synth/best_network.pth \
  --data /tmp/dream_data/synthetic_50k --split val
```

## Résultats

| Modèle | Dataset entraîn. | Eval synth | Eval réel |
|--------|-------------------|------------|-----------|
| VGG synth-only (20K) | 20K synth | 97% det, 3.1px | ~26% det |
| VGG synth-only (50K) | 50K synth | 98.3% det, 3.15px | 13.2% det, 172px |
| **VGG mixte (18K)** | 10K réel + 8K synth | terminé | **À évaluer** |

## Dataset Format (NDDS)

DREAM attend le format **NDDS** :

```
dataset_dir/
├── _camera_settings.json      # Intrinsèques caméra (fx, fy, cx, cy)
├── 000000.png                 # Image RGB
├── 000000.json                # Annotations keypoints
├── 000001.png
├── 000001.json
└── ...
```

### Conversion depuis nos datasets

```bash
python training/dream/convert_to_ndds.py \
  --input datasets/synthetic_dataset \
  --output /tmp/dream_data/synthetic

python training/dream/convert_to_ndds.py \
  --input datasets/real_dataset \
  --output /tmp/dream_data/real_cam0 --camera cam0
```

## Fichiers du module

| Fichier | Rôle |
|---------|------|
| `dream/evaluate_dream.py` | Évaluation complète avec métriques par keypoint |
| `dream/convert_to_ndds.py` | Conversion dataset → format NDDS |
| `dream/merge_and_convert.py` | Fusion réel + synthétique → NDDS avec oversampling |
| `dream/mycobot_fk.py` | Forward kinematics + projection caméra |
| `dream/mycobot_ik.py` | Inverse kinematics (Jacobien) |
| `dream/infer_dream.py` | Inférence : keypoints + PnP |
| `dream/finetune_real.py` | Fine-tuning expérimental (⚠️ ne fonctionne pas) |
| `model.py` | Legacy: PoseResNet (abandonné) |
| `train.py` | Legacy: régression directe (abandonné) |
| `capture_real.py` | Capture données réelles depuis Pi |

## Leçons apprises

1. **Régression directe ≠ viable** quand le robot est petit (~15% pixels)
2. **DREAM σ=2** pour belief maps — σ=4 écrase les pics et tue la détection
3. **Ne pas fine-tuner DREAM manuellement** — MSE sur grilles quasi-vides → all-zeros
4. **Utiliser `train_network.py` natif** de DREAM
5. **VGG sans BatchNorm** stable ; ResNet+BN explose en batch_size < 64
