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
| VGG synth-only (50K, ancien) | 50K synth | 98.3% det, 3.15px | 13.2% det, 172px |
| vgg_ultimate_v2_e50 | 20K synth | 97.7% det | ~26% det |
| **vgg_ultimate_v4_e50** | 50K synth (intrinsèques corrigées) | **99.4% det, 2.61px** | ≈27% det |
| **vgg_ultimate_v4_mix_ft_e30** | mix **80K** (synth 50K + real 3cam ×5) | ≈99% det | **91.6% det** |

Détails : [`dream/VGG_ULTIMATE_V4_50K.md`](dream/VGG_ULTIMATE_V4_50K.md),
[`dream/FINETUNE_MIX_REAL3CAM_PLAN.md`](dream/FINETUNE_MIX_REAL3CAM_PLAN.md),
holdout réel : [`dream/REAL3CAM_SESSION6_HOLDOUT.md`](dream/REAL3CAM_SESSION6_HOLDOUT.md).

### Quel script pour quel run ? (50K synth → 80K mixte)

| Étape | Script | Données | Départ | Sortie |
|-------|--------|---------|--------|--------|
| **1. Base synthétique (99.4%)** | [`dream/train_dream_ultimate_v4.py`](dream/train_dream_ultimate_v4.py) | `dream_data/synthetic_50k_ndds` (**50K**, split 40K/10K) | **from scratch** | `vgg_ultimate_v4_e50` |
| **2. Fine-tune mixte (91.6% réel)** | [`dream/train_dream_ultimate_v4_mix.py`](dream/train_dream_ultimate_v4_mix.py) | `dream_data/mix_synth50k_real3camx5_ndds` (**80K** = 50K synth + real_3cam ×5 suréchantillonné, split 64K/8K/8K) | `--pretrained …/vgg_ultimate_v4_e50/best_network.pth` | **`vgg_ultimate_v4_mix_ft_e30`** |

```bash
source ~/ros_jazzy/venv_dream/bin/activate
cd training/dream

# 1 — base synthétique 50K (99.4% synth)
python train_dream_ultimate_v4.py \
  --data dream_data/synthetic_50k_ndds \
  --output output/checkpoints_dream/vgg_ultimate_v4_e50 \
  --epochs 50 --batch-size 8 --workers 8 --patience 5

# 2 — fine-tune datamixte 80K (91.6% réel) — départ = base ci-dessus
python train_dream_ultimate_v4_mix.py \
  --data dream_data/mix_synth50k_real3camx5_ndds \
  --pretrained output/checkpoints_dream/vgg_ultimate_v4_e50/best_network.pth \
  --output output/checkpoints_dream/vgg_ultimate_v4_mix_ft_e30 \
  --epochs 30 --batch-size 8 --workers 8
```

> Le « **80K** » = le dataset **mixte** (data-mixte) : 50K synthétiques + les
> captures real_3cam ×5 pour équilibrer réel/synthétique. Le fine-tune part de la
> base 50K (pas from-scratch) pour ne pas perdre l'acquis synthétique (99.4%).

### Contenu d'un checkpoint (`checkpoints_dream/vgg_ultimate_v4_mix_ft_e30/`)

| Fichier | Rôle |
|---------|------|
| `best_network.pth` (~88 Mo) | poids du meilleur epoch (chargé par `dream_inference`) |
| `best_network.yaml` | config : `data_path`, archi VGG, hyperparams, **loss** val (~0.00094 MSE), `pretrained` |
| `epoch_10/20/30.pth`+`.yaml` | snapshots intermédiaires |
| `learning_curve_final.png` | courbe train / val / test |
| `training_log.pkl` | historique par epoch (losses, per-kp val, LR, split 64K/8K/8K, seed) |

⚠️ Le checkpoint stocke la **loss** (MSE), **pas** le « 91.6% » : ce taux de
détection vient de l'**évaluation** a posteriori (`evaluate_dream.py` sur le
holdout réel), voir [`dream/REAL3CAM_SESSION6_HOLDOUT.md`](dream/REAL3CAM_SESSION6_HOLDOUT.md).

### Résultats Grid Search — Weighted Loss (20K synthetic)

| Modèle | Weights | Overall | link6 Det% | Epochs |
|--------|---------|---------|------------|--------|
| vgg_augmented_e25 (baseline) | [1,1,1,1,1,1,1] | 96.8% | 88.2% | 25 |
| vgg_ultimate_e30 | [1,1,1,1,1.5,3.0,5.0] | 97.5% | 91.0% | 28/30 |
| vgg_ultimate_e50 | [1,1,1,1,1.5,3.0,5.0] | 96.9% | 88.4% | 35/50 |
| vgg_ultimate_v2_e30 | [1,1,1,1,1.5,1.5,6.0] | 97.5% | 90.8% | 27/30 |
| **vgg_ultimate_v2_e50**  | **[1,1,1,1,1.5,1.5,6.0]** | **97.7%** | **92.6%** | **20/50** 

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
| `dream/train_dream_weighted.py` | Training pondéré par keypoint (base B3) |
| `dream/train_dream_grid_search.py` | Grid search 64 combinaisons de weights |
| `dream/evaluate_grid.py` | Évaluation automatisée des runs grid search |
| `dream/train_dream_ultimate_v2.py` | Training final w=[1,1,1,1,1.5,1.5,6.0] ⭐ |
| `dream/train_dream_ultimate.py` | Training final w=[1,1,1,1,1.5,3.0,5.0] |
| `dream/merge_ndds.py` | Fusion réel + synthétique déjà en NDDS |
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

## Leçons apprises

1. **Régression directe ≠ viable** quand le robot est petit (~15% pixels)
2. **DREAM σ=2** pour belief maps — σ=4 écrase les pics et tue la détection
3. **Ne pas fine-tuner DREAM manuellement** — MSE sur grilles quasi-vides → all-zeros
4. **Utiliser `train_network.py` natif** de DREAM 
5. **VGG sans BatchNorm** stable ; ResNet+BN explose en batch_size < 64 .
6. **w6 est le levier principal** — en dessous de 4.5 → link6 Det% < 85% systématiquement 
7. **w5=1.5 optimal** — au-delà de 5.0, gain négligeable sur link5 mais overall régresse 
8. **w4=1.5 ou 3.0 suffisant** — au-delà de 5.0, perte de 0.5-1% overall sans gain notable 
9. **Early stopping accélère la convergence** — weights [1,1,1,1,1.5,1.5,6.0] convergent dès epoch 20/50 .
10. **97.7% = nouveau record sur 20K synthétiques** — weights [1,1,1,1,1.5,1.5,6.0] + early stopping ont dépassé le plafond supposé de 97% 
11. **workers=8 + batch_size=8 recommandé** — 8 workers chargent les batches en parallèle pendant que le GPU entraîne, ce qui évite les temps d'attente CPU→GPU et réduit significativement la durée totale d'entraînement 
