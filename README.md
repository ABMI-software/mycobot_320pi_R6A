# MyCobot 320 Pi — ROS2 Control & Vision-Based Pose Estimation

🤖 **Plateforme complète pour contrôle robotique et estimation de pose par vision CNN**

Ce projet intègre :
- Un **bridge ROS2 TCP** pour contrôler un MyCobot 320 Pi depuis un PC distant
- Une **simulation Gazebo** avec 4 caméras et domain randomization
- Un **pipeline ML complet** : collecte de données → entraînement CNN multi-vues → prédiction de pose
- Des **datasets** synthétiques (Gazebo) et réels (caméras Pi) via Git LFS

> 📋 Pour reprendre le développement, voir [`SESSION_RESUME.md`](SESSION_RESUME.md)  
> 📖 Documentation technique détaillée dans [`DEVELOPMENT_SUMMARY.md`](DEVELOPMENT_SUMMARY.md)

---

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              PC TOUR (10.10.0.115)                          │
│                ROS2 Jazzy / Ubuntu 24.04 / Python 3.12                      │
│                Conda: Python 3.13 / PyTorch 2.6 + CUDA 12.4                │
│                GPU: NVIDIA RTX 4000 Ada (20 GB VRAM)                        │
├─────────────────────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌──────────────┐       │
│  │ simple_gui  │  │slider_control│  │teleop_keyb. │  │ training/    │       │
│  │  (Tkinter)  │  │(joint_states)│  │  (clavier)  │  │ train.py     │       │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  │ predict.py   │       │
│         │                │                │          │ capture_real  │       │
│         └────────────────┴────────────────┘          └──────┬───────┘       │
│                          │                                  │               │
│                   /to_robot (JSON)                   TCP:5005 + 5006        │
│                          ▼                                  ▼               │
│                ┌─────────────────┐                                          │
│                │   bridge_tour   │                                          │
│                └────────┬────────┘                                          │
├─────────────────────────┼───────────────────────────────────────────────────┤
│                   RÉSEAU ETHERNET (10.10.0.x)                              │
├─────────────────────────┼───────────────────────────────────────────────────┤
│                         ▼                                                   │
│           ┌─────────────────┐       ┌─────────────────┐                    │
│           │bridge_pi_simple │       │pi_camera_server │                    │
│           │  TCP:5005       │       │  TCP:5006       │                    │
│           └────────┬────────┘       └────────┬────────┘                    │
│                    ▼                         ▼                             │
│           ┌─────────────────┐       ┌─────────────────┐                    │
│           │    pymycobot    │       │ Arducam USB ×2  │                    │
│           │  /dev/ttyAMA0   │       │  cam0 + cam3    │                    │
│           └────────┬────────┘       └─────────────────┘                    │
│                    ▼                                                       │
│           ┌─────────────────┐                                              │
│           │  MyCobot 320 Pi │                                              │
│           └─────────────────┘                                              │
│                     RASPBERRY PI (10.10.0.225)                             │
└────────────────────────────────────────────────────────────────────────────┘
```

---

## 📦 Packages & Composants

| Composant | Description |
|-----------|-------------|
| `mycobot_gateway/` | Bridge TCP, GUI, contrôles robotiques (ROS2 package) |
| `mycobot_description/` | URDF, meshes 3D, configs RViz, Gazebo simulation |
| `training/` | Pipeline ML : dataset, modèle CNN, entraînement, capture, prédiction |
| `datasets/` | Données synthétiques (Gazebo) et réelles (Pi) — via **Git LFS** |
| `scripts/` | Scripts utilitaires de diagnostic et test |
| `docs/` | Documentation technique complète |

---

## 🚀 Quick Start

### Prérequis

**PC Tour :**
- Ubuntu 24.04, ROS2 Jazzy, Python 3.12
- Conda avec PyTorch 2.6 + CUDA (pour le training)
- GPU NVIDIA (recommandé pour entraînement)

**Raspberry Pi :**
- Ubuntu, pymycobot (`pip3 install pymycobot`)
- Caméras USB Arducam (pour capture réelle)

### Installation

```bash
# Cloner le repo (avec Git LFS pour les datasets)
cd ~/ros_jazzy/src
git clone https://github.com/ABMI-software/mycobot_320pi_R6A.git
cd mycobot_320pi_R6A
git lfs pull   # Télécharge les images des datasets (~9.5 GB)

# Compiler les packages ROS2
cd ~/ros_jazzy
colcon build --packages-select mycobot_gateway mycobot_description --symlink-install
source install/setup.bash
```

### Démarrage du robot

```bash
# Sur le Pi — Terminal 1 : bridge robot
ssh er@10.10.0.225
python3 bridge_pi_simple.py

# Sur le Pi — Terminal 2 : serveur caméras
python3 pi_camera_server.py --cameras 0 3 --names cam0 cam3
```

### Contrôle du robot (PC Tour)

```bash
# ⚠️ Important : désactiver conda avant ROS2
conda deactivate
source /opt/ros/jazzy/setup.bash
source ~/ros_jazzy/src/mycobot_R6A/install/setup.bash

# Modes de contrôle
ros2 launch mycobot_gateway simple_gui.launch.py        # GUI graphique
ros2 launch mycobot_gateway slider_control.launch.py    # Sliders RViz
ros2 launch mycobot_gateway teleop_keyboard.launch.py   # Clavier
ros2 launch mycobot_gateway commander.launch.py         # CLI interactif
ros2 launch mycobot_gateway rviz_sync.launch.py         # Sync robot→RViz
```

---

## 🧠 Pipeline Vision / Pose Estimation

### Vue d'ensemble

```
Simulation Gazebo (4 caméras)  →  Données Synthétiques (5000×4 = 20K images)
                                          ↓
                                 train.py (Multi-view ResNet50)
                                          ↓
                                Modèle synthétique : 12.97° MAE ✅
                                          ↓
Capture réelle (2 caméras Pi)  →  Données Réelles (2000×2 = 4K images)
                                          ↓
                                 train.py (fine-tune / transfer)
                                          ↓
                                🔄 En cours d'amélioration
```

### Résultats d'entraînement

| Configuration | Dataset | MAE |
|---|---|---|
| ResNet18 single-view | Synthétique 1K | 22.6° |
| ResNet50 single-view (front) | Synthétique 5K | 16.5° |
| **ResNet50 multi-view (4 cam)** | **Synthétique 5K** | **12.97°** ✅ |
| Toutes approches | Réel 2K | ~31.7° (en cours) |

### Entraînement

```bash
# Multi-view synthétique (meilleur résultat : 12.97°)
/home/genji/miniconda/bin/python3 training/train.py \
  --dataset datasets/synthetic_dataset \
  --multi-view --backbone resnet50 \
  --epochs 150 --batch-size 16 --lr 1e-4

# Multi-view réel (cam0 + cam3)
/home/genji/miniconda/bin/python3 training/train.py \
  --dataset datasets/real_dataset \
  --multi-view --views cam0 cam3 --backbone resnet50 \
  --lr 1e-3 --epochs 300 --batch-size 32 --freeze-epochs 3
```

### Capture de données réelles

```bash
/home/genji/miniconda/bin/python3 training/capture_real.py \
  --output datasets/real_dataset \
  --num-samples 2000 \
  --pi-host 10.10.0.225 \
  --settle-time 3.0 --speed 25 --limit-fraction 0.5
```

---

## � Datasets

> ⚠️ Les images sont stockées via **Git LFS**. Après `git clone`, exécutez `git lfs pull`.

| Dataset | Poses | Caméras | Images | Taille |
|---------|-------|---------|--------|--------|
| **Synthétique** (`datasets/synthetic_dataset/`) | 5,000 | 4 (front, left, right, top) | 20,000 | ~8.3 GB |
| **Réel** (`datasets/real_dataset/`) | 2,000 | 2 (cam0, cam3) | 4,000 | ~1.2 GB |

Format `labels.csv` :
```
camera,image_path,j1,j2,j3,j4,j5,j6
cam0,images/cam0/000000.png,-45.23,12.67,-30.45,5.12,-15.89,22.34
```

Plus de détails : [`datasets/README.md`](datasets/README.md)

---

## 🎮 Modes de Contrôle

| Mode | Launch file | Description |
|------|-------------|-------------|
| **Simple GUI** | `simple_gui.launch.py` | Interface Tkinter (angles, coords, gripper, LED) |
| **Slider Control** | `slider_control.launch.py` | Joint State Publisher GUI + RViz temps réel |
| **Teleop Keyboard** | `teleop_keyboard.launch.py` | Contrôle clavier (WASD + ZX) |
| **Commander CLI** | `commander.launch.py` | Commandes textuelles interactives |
| **RViz Sync** | `rviz_sync.launch.py` | Synchronisation robot réel → RViz |

---

## 📁 Structure du Projet

```
mycobot_R6A/
├── README.md                       # 👈 Ce fichier
├── SESSION_RESUME.md               # Point de départ sessions dev
├── DEVELOPMENT_SUMMARY.md          # Résumé technique complet
│
├── mycobot_gateway/                # 📦 Package ROS2 — contrôle
│   ├── mycobot_gateway/
│   │   ├── bridge_tour.py          # Client TCP vers Pi
│   │   ├── simple_gui.py           # GUI Tkinter
│   │   ├── slider_control.py       # Contrôle sliders
│   │   └── synthetic_data_collector_v2.py
│   ├── scripts/
│   │   ├── bridge_pi_simple.py     # Script Pi (serveur robot)
│   │   └── pi_camera_server.py     # Script Pi (serveur caméras)
│   └── launch/                     # Fichiers launch ROS2
│
├── mycobot_description/            # 📦 Package ROS2 — URDF/Gazebo
│   ├── urdf/320_pi/                # Modèle 3D + 4 caméras Gazebo
│   └── worlds/randomized.sdf       # Domain randomization
│
├── training/                       # 📦 Pipeline ML/IA
│   ├── model.py                    # PoseResNet + MultiViewPoseResNet
│   ├── dataset.py                  # Datasets single/multi-view
│   ├── train.py                    # Training complet (finetune, multi-view)
│   ├── predict.py                  # Inférence
│   ├── capture_real.py             # Capture réelle avec FK safety
│   └── preview_cameras.py          # Prévisualisation caméras Pi
│
├── datasets/                       # 📦 Données (Git LFS)
│   ├── real_dataset/               # 2000 poses × 2 caméras
│   └── synthetic_dataset/          # 5000 poses × 4 caméras
│
├── scripts/                        # Scripts utilitaires
└── docs/                           # Documentation détaillée
```

---

## 📡 Configuration Réseau

| Machine | IP | Ports |
|---------|-----|-------|
| PC Tour | 10.10.0.115 | — |
| Raspberry Pi | 10.10.0.225 | 5005 (robot) + 5006 (caméras) |

```bash
ros2 launch mycobot_gateway simple_gui.launch.py pi_ip:=<VOTRE_IP_PI>
```

---

## ⚠️ Troubleshooting

### Erreur Python conda/ROS2
```bash
# Toujours désactiver conda avant ROS2
conda deactivate
```

### Connexion TCP échoue
```bash
ping 10.10.0.225
nc -zv 10.10.0.225 5005   # robot bridge
nc -zv 10.10.0.225 5006   # camera server
```

### Git LFS — images manquantes après clone
```bash
git lfs install
git lfs pull
```

---

## 📚 Documentation

| Fichier | Description |
|---------|-------------|
| [`SESSION_RESUME.md`](SESSION_RESUME.md) | Point de départ pour le développement |
| [`DEVELOPMENT_SUMMARY.md`](DEVELOPMENT_SUMMARY.md) | Résumé technique complet |
| [`datasets/README.md`](datasets/README.md) | Documentation des datasets |
| [`docs/QUICKSTART.md`](docs/QUICKSTART.md) | Guide démarrage rapide |
| [`docs/SYNTHETIC_DATA.md`](docs/SYNTHETIC_DATA.md) | Pipeline données synthétiques |
| [`docs/ROBOT_QUICKSTART.md`](docs/ROBOT_QUICKSTART.md) | Procédure robot réel |
| [`training/README.md`](training/README.md) | Documentation pipeline ML |

---

## 📄 License

Apache License 2.0

## 👥 Contributeurs

- ABMI Software Team
