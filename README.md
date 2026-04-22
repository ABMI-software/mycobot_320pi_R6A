# MyCobot 320 Pi — ROS2 Control & Vision-Based Pose Estimation

**Plateforme complète pour contrôle robotique et estimation de pose par vision CNN**

Ce projet intègre :
- Un **bridge ROS2 TCP** pour contrôler un MyCobot 320 Pi depuis un PC distant
- Une **simulation Gazebo Harmonic** avec gripper adaptatif, 4 caméras et domain randomization
- Un **pipeline ML DREAM** : keypoint detection (VGG-19) → belief maps → PnP → pose 3D
- Une **téléopération par la main** (Wilor + Orbbec Astra) avec dashboard de tuning et rapport Excel — adapté du pipeline R5A / LeRobot. **Pipeline validé sur robot physique le 22/04/2026**
- Des **datasets** synthétiques (Gazebo, 50K frames) et réels (caméras Pi, 4K images) via Git LFS

> Pour reprendre le développement, voir [`SESSION_RESUME.md`](SESSION_RESUME.md)
> Documentation technique détaillée dans [`DEVELOPMENT_SUMMARY.md`](DEVELOPMENT_SUMMARY.md)
> Pour la téléopération (pipeline, filtres, dashboard, rapport de perf) : [`docs/TELEOPERATION.md`](docs/TELEOPERATION.md)

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
| `training/` | Pipeline ML : DREAM keypoint detection (VGG-19), legacy ResNet regression |
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

Le projet utilise **deux approches** de pose estimation, la seconde (DREAM) étant l'approche active :

```
═══════════════════════════════════════════════════════════════
  Phase 1 : Régression directe (image → angles)  [ABANDONNÉ]
═══════════════════════════════════════════════════════════════
  ResNet50 multi-view → 12.97° MAE synthétique
  ❌ Bloqué à ~32° MAE sur données réelles (robot trop petit)

═══════════════════════════════════════════════════════════════
  Phase 2 : DREAM Keypoint Detection  [ACTIF]
═══════════════════════════════════════════════════════════════
  Image → VGG-19 → 7 belief maps → keypoints 2D → PnP → pose

  Données synthétiques (50K) : 98.3% détection, 3.15px médiane ✅
  Transfert sim-to-real       : 13.2% détection ❌ (domain gap)
  🔄 Entraînement mixte (10K réel + 8K synth) en cours
```

### Approche DREAM (active)

**DREAM** (NVlabs) détecte les 7 articulations du robot dans l'image via des **belief maps** (cartes de chaleur), puis résout la pose 3D par **PnP**.

```
Image 640×480 → VGG-19 → 6 stages cascadés → 7 belief maps 100×100
                                                      ↓
                                              Peak Detection → 7 keypoints 2D
                                                      ↓
                              3D keypoints (FK) → PnP → Pose caméra [R|t]
```

### Résultats

| Modèle | Dataset entraîn. | Eval synth | Eval réel |
|--------|-----------------|------------|-----------|
| VGG synth-only (20K) | 20K synth | 97% det, 3.1px | ~26% det |
| VGG synth-only (50K) | 50K synth | 98.3% det, 3.15px | 13.2% det, 172px |
| **VGG mixte** | 10K réel + 8K synth | — | **À évaluer** |

### Entraînement DREAM (natif)

```bash
source ~/ros_jazzy/venv_dream/bin/activate
python /tmp/DREAM/scripts/train_network.py \
  -i /tmp/dream_data/mixed_real_synth \
  -m /tmp/DREAM/manip_configs/mycobot320.yaml \
  -ar /tmp/DREAM/arch_configs/dream_vgg_q.yaml \
  -e 25 -b 32 -lr 0.0001 \
  -o training/checkpoints_dream/vgg_mixed_real_synth -f
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
| **Hand Teleop** | `mycobot_teleop.launch.py` | **Téléop par caméra/main** (Wilor + Astra) — voir ci-dessous |

---

## 🖐️ Téléopération par la main

Pipeline complet de pilotage du robot par la main de l'opérateur, adapté du R5A / LeRobot. **Orbbec Astra S** (RGB via OpenNI2 shared-memory) → **Wilor** (hand pose 6-DoF) → mapping relatif → filtres R5A → **rosbridge** → JTC Gazebo + `bridge_tour` vers le Pi réel.

**Outils livrés** ([teleop/](teleop/)) :

| Outil | Rôle |
|-------|------|
| `mycobot_teleop.py` | Script principal — caméra → joints |
| `teleop_dashboard.py` | GUI ttkbootstrap : 4 sliders live (x/y/z gain, tfs), bouton Recalibrate, 3 plots temps réel, tableau stats par joint avec flags OK/JITTERY/UNSTABLE |
| `performance_analyzer.py` | Générateur de rapport Excel — protocole guidé 7 phases → verdict READY / CAUTIOUS / NOT READY + onglets par-joint, par-scénario, raw data |
| `orbbec_capture.py` | Wrapper shared-memory Astra avec auto-spawn `oni_grabber` + watchdog |

**Workflow 5 terminaux** :

```bash
# T1 — rosbridge
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# T2 — Gazebo + controllers
ros2 launch mycobot_gateway mycobot_teleop.launch.py target:=sim

# T3 — teleop (env conda hand-teleop)
conda activate hand-teleop && cd teleop
python3 mycobot_teleop.py --camera astra --ros --use-rosbridge

# T4 — dashboard live tuning
python3 teleop_dashboard.py

# T5 — rapport de performance avant robot réel
python3 performance_analyzer.py --guided
```

**Documentation détaillée** :
- [`docs/TELEOPERATION.md`](docs/TELEOPERATION.md) — pipeline complet, filtres, historique
- [`docs/TELEOP_ARCHITECTURE_VIZ.md`](docs/TELEOP_ARCHITECTURE_VIZ.md) — **visuel détaillé** : de la détection main au mouvement du bras (types, unités, latences)
- [`docs/TELEOP_DASHBOARD.md`](docs/TELEOP_DASHBOARD.md) — manuel utilisateur du dashboard
- [`docs/TELEOP_TUNING.md`](docs/TELEOP_TUNING.md) — référence des paramètres + dépannage
- [`docs/REAL_ROBOT_TEST_PROCEDURE.md`](docs/REAL_ROBOT_TEST_PROCEDURE.md) — procédure de test sur robot physique

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
│   ├── urdf/pro_adaptive_gripper/   # Gripper adaptatif (meshes)
│   └── worlds/
│       ├── randomized.sdf           # Monde de base
│       └── randomized_v2.sdf        # 6 lights + 12 clutter objects
│
├── training/                       # 📦 Pipeline ML/IA
│   ├── train.py                    # Legacy: régression directe ResNet
│   ├── predict.py                  # Legacy: inférence régression
│   ├── capture_real.py             # Capture réelle avec FK safety
│   └── dream/                      # DREAM keypoint detection (actif)
│       ├── evaluate_dream.py       # Évaluation (métriques par keypoint)
│       ├── convert_to_ndds.py      # Conversion dataset → NDDS
│       ├── merge_and_convert.py    # Fusion réel+synth → NDDS
│       ├── mycobot_fk.py           # Forward kinematics + projection
│       ├── infer_dream.py          # Inférence keypoints + PnP
│       └── finetune_real.py        # Fine-tuning expérimental (⚠️)
│
├── datasets/                       # 📦 Données (Git LFS)
│   ├── real_dataset/               # 2000 poses × 2 caméras
│   └── synthetic_dataset/          # 5000 poses × 4 caméras
│
├── teleop/                         # 🖐️ Téléopération par la main (env conda)
│   ├── mycobot_teleop.py           # Script principal : caméra → joints
│   ├── teleop_dashboard.py         # GUI ttkbootstrap live tuning + plots
│   ├── performance_analyzer.py     # Rapport Excel avant robot réel
│   └── orbbec_capture.py           # Wrapper Astra via oni_grabber + shm
│
├── scripts/
│   ├── train_pipeline.sh           # Pipeline merge→NDDS→training automatisé
│   └── monitor_collection.sh       # Suivi collecte en temps réel
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
| [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | Architecture détaillée du système |
| [`datasets/README.md`](datasets/README.md) | Documentation des datasets |
| [`docs/QUICKSTART.md`](docs/QUICKSTART.md) | Guide démarrage rapide |
| [`docs/SYNTHETIC_DATA.md`](docs/SYNTHETIC_DATA.md) | Pipeline données synthétiques |
| [`docs/ROBOT_QUICKSTART.md`](docs/ROBOT_QUICKSTART.md) | Procédure robot réel |
| [`docs/TELEOPERATION.md`](docs/TELEOPERATION.md) | Téléopération par la main (pipeline + filtres) |
| [`docs/TELEOP_ARCHITECTURE_VIZ.md`](docs/TELEOP_ARCHITECTURE_VIZ.md) | Visuel détaillé — détection main → mouvement bras |
| [`docs/TELEOP_DASHBOARD.md`](docs/TELEOP_DASHBOARD.md) | Manuel utilisateur du dashboard de téléop |
| [`docs/TELEOP_TUNING.md`](docs/TELEOP_TUNING.md) | Référence paramètres + dépannage téléop |
| [`docs/REAL_ROBOT_TEST_PROCEDURE.md`](docs/REAL_ROBOT_TEST_PROCEDURE.md) | Procédure + protocole de calibration sécurisé sur robot physique |
| [`training/README.md`](training/README.md) | Documentation pipeline ML |

---

## 📄 License

Apache License 2.0

## 👥 Contributeurs

- ABMI Software Team
