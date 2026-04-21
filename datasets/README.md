# 📦 Datasets - MyCobot 320 Pi Pose Estimation

Ce dossier contient les données d'entraînement pour le pipeline de pose estimation CNN.

> ⚠️ **Les images PNG sont stockées via [Git LFS](https://git-lfs.github.com/).**  
> Après `git clone`, exécutez `git lfs pull` pour télécharger les images.

---

## 📁 Structure

```
datasets/
├── README.md                        # Ce fichier
├── real_dataset/                    # Données réelles (Pi Arducam)
│   ├── labels.csv                   # 4001 lignes (header + 2000 poses × 2 vues)
│   └── images/
│       ├── cam0/                    # 2000 images (640×480 PNG)
│       └── cam3/                    # 2000 images (640×480 PNG)
└── synthetic_dataset/               # Données synthétiques (Gazebo)
    ├── labels.csv                   # 20001 lignes (header + 5000 poses × 4 vues)
    └── images/
        ├── front/                   # 5000 images (640×480 PNG)
        ├── left/                    # 5000 images (640×480 PNG)
        ├── right/                   # 5000 images (640×480 PNG)
        └── top/                     # 5000 images (640×480 PNG)
```

## 📊 Résumé

| Dataset | Poses | Caméras | Images | Taille | Resolution |
|---------|-------|---------|--------|--------|------------|
| **Synthétique** | 5000 | 4 (front, left, right, top) | 20,000 | ~8.3 GB | 640×480 PNG |
| **Réel** | 2000 | 2 (cam0, cam3) | 4,000 | ~1.2 GB | 640×480 PNG |

## 📋 Format labels.csv

Chaque ligne contient :
```
camera,image_path,j1,j2,j3,j4,j5,j6
cam0,images/cam0/000000.png,-45.23,12.67,-30.45,5.12,-15.89,22.34
```

- `camera` : identifiant de la caméra
- `image_path` : chemin relatif de l'image
- `j1` à `j6` : angles articulaires en degrés

## 🔬 Notes sur les datasets

### Synthétique (Gazebo) ✅
- Généré avec `synthetic_data_collector_v2.py` dans Gazebo avec domain randomization
- 4 caméras virtuelles positionnées autour du robot
- Éclairage et textures randomisés
- **Meilleur résultat : 12.97° MAE** (multi-view ResNet50)

### Réel (Pi Arducam) ⚠️
- Capturé avec `training/capture_real.py` via TCP bridge (Pi)
- 2 caméras USB Arducam sur le Pi
- **Problème identifié** : le robot est trop petit dans les images (15.4% cam0, 5.7% cam3)
- Dérive d'éclairage naturelle pendant les 2h de capture
- Corrélation pose ↔ pixel quasi-nulle (r ≈ 0.004)
- **Nécessite une recapture** avec exposition fixée et caméras plus proches

## 🚀 Utilisation

```python
from training.dataset import MyCobotMultiViewDataset

# Synthétique (4 vues)
ds = MyCobotMultiViewDataset("datasets/synthetic_dataset", split="train")

# Réel (2 vues)
ds = MyCobotMultiViewDataset("datasets/real_dataset", split="train",
                              cameras=["cam0", "cam3"])
```

Pour l'entraînement :
```bash
# Multi-view synthétique
/home/genji/miniconda/bin/python3 training/train.py \
  --dataset datasets/synthetic_dataset \
  --multi-view --backbone resnet50 --epochs 150

# Multi-view réel
/home/genji/miniconda/bin/python3 training/train.py \
  --dataset datasets/real_dataset \
  --multi-view --views cam0 cam3 --backbone resnet50 --epochs 300
```
