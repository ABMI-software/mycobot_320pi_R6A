# 📋 SESSION RESUME - MyCobot 320 Pi Gateway Bridge# 📋 SESSION RESUME - MyCobot 320 Pi Gateway Bridge



> **Date de dernière mise à jour :** 2 avril 2026  > **Date de dernière mise à jour :** 1 avril 2026  

> **Version :** 1.5.0  > **Version :** 1.4.0  

> **Repository GitHub :** https://github.com/ABMI-software/mycobot_320pi_R6A  > **Repository GitHub :** https://github.com/ABMI-software/mycobot_320pi_R6A  

> **Branche :** `main`> **Branche :** `main` | `feature/gazebo` | `feature/synthetic-data` | `feature/pose-training`  

> **Dernier commit :** (feature/pose-training)

---

---

## 🎯 POINT DE DÉPART - Session Suivante (3 avril 2026)

## 🎯 POINT DE DÉPART - Session Précédente

### Contexte Résumé

### ✅ Ce qui a été accompli

Le projet a un pipeline complet : **Gazebo simulation → données synthétiques → entraînement CNN → capture réelle**. Le modèle multi-view ResNet50 atteint **12.97° MAE sur données synthétiques** (4 caméras, 5000 poses). Cependant, **l'entraînement sur les données réelles (2000 poses, 2 caméras) est bloqué** au niveau du mean-predictor baseline (~32.76°).

| Tâche | Statut | Détails |

### 🔴 Problème Principal à Résoudre|-------|--------|---------|

| Création repo GitHub | ✅ Complété | `ABMI-software/mycobot_320pi_R6A` |

**Les images réelles ne contiennent pas assez de signal visuel exploitable** pour que le CNN apprenne la relation image → pose. Diagnostic complet :| Architecture TCP Tour ↔ Pi | ✅ Validé | Connexion stable |

| bridge_tour (PC) | ✅ Fonctionnel | Client TCP ROS2 |

1. **Corrélation pose ↔ pixel quasi-nulle** (r = 0.004, même entre paires consécutives sans dérive d'éclairage)| bridge_pi_simple (Pi) | ✅ Fonctionnel | Serveur TCP + pymycobot |

2. **Le robot est trop petit dans l'image** : cam0 = 15.4% des pixels varient, cam3 = 5.7% seulement| simple_gui | ✅ Testé | Interface Tkinter |

3. **56% de l'image est du fond statique** qui ne change jamais| slider_control | ✅ Testé | Robot suit les sliders en temps réel |

4. **Dérive d'éclairage massive** pendant les 2h de capture (luminosité moyenne : 140 → 82 → 143)| RViz visualisation | ✅ Corrigé | Config avec Fixed Frame = base |

5. Même entre poses extrêmes (J1 : -83° vs +83°), seulement 9.1% des pixels changent significativement| Commandes JSON | ✅ Validé | send_angles, get_angles, go_home... |



### ✅ Ce qui a été accompli (sessions 1-2 avril 2026)### 🔧 Dernières modifications



| Tâche | Statut | Détails |1. **`slider_control.launch.py`** - Ajout du chemin config RViz

|-------|--------|---------|2. **`bridge_pi_simple.py`** - Ajout commandes texte (get_angles, power_on/off, gripper)

| Capture 2000 échantillons réels | ✅ | 2 caméras (cam0, cam3), 4000 images, 1.2 GB, 0 collisions |

| FK safety dans capture_real.py | ✅ | Calcul FK depuis dimensions URDF réelles, protection table + câbles |---

| Auto-détection vues CSV | ✅ | dataset.py + train.py gèrent `--views cam0 cam3` |

| Chargement checkpoint compatible | ✅ | train.py filtre les couches incompatibles (4→2 vues) |## �️ Architecture du Système

| PerImageNormalize | ✅ testé | Élimine la dérive d'éclairage, mais insuffisant seul |

| Diagnostic dataset complet | ✅ | Corrélation, variance map, uniqueness, off-by-one, SSIM |```

| Test overfit 10 samples | ✅ 0.01° | Prouve que le modèle et le pipeline fonctionnent correctement |┌─────────────────────────────────────────────────────────────────────────────┐

| Ablation preprocessing | 🔄 Partiel | Exp A (Standard) : 31.81°. Exp B-E interrompus |│                              PC TOUR (10.10.0.115)                          │

│                         ROS2 Jazzy / Ubuntu 24.04 / Python 3.12             │

### 📊 Résultats des Tentatives d'Entraînement sur Données Réelles├─────────────────────────────────────────────────────────────────────────────┤

│                                                                             │

| Approche | Best MAE | vs Baseline (32.76°) | Note |│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │

|----------|----------|----------------------|------|│  │ simple_gui  │  │slider_control│  │teleop_keyb. │  │marker_follow│        │

| Multi-view ResNet50, lr=1e-3 | 32.7° | -0.06° | Stagne dès epoch 1 |│  │  (Tkinter)  │  │(joint_states)│  │  (clavier)  │  │   (ArUco)   │        │

| Single-view ResNet18, lr=3e-3 | 32.8° | +0.04° | Aucun progrès |│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘        │

| SmoothL1 + OneCycleLR (minimal) | 31.75° | -1.01° | Overfit epoch 20+ |│         │                │                │                │               │

| PerImageNormalize + SmoothL1 | 31.69° | -1.07° | Overfit epoch 6 |│         └────────────────┴────────────────┴────────────────┘               │

| Standard ImageNet (ablation A) | 31.81° | -0.95° | Early stop epoch 32 |│                                   │                                         │

| **Mean predictor (baseline)** | **32.76°** | **0.00°** | Prédire toujours la moyenne |│                          /to_robot (JSON)                                   │

│                                   ▼                                         │

**→ Toutes les approches ne dépassent la baseline que d'~1° avant d'overfitter.**│                         ┌─────────────────┐                                │

│                         │   bridge_tour   │◄─── /from_robot                │

---│                         │   (TCP Client)  │                                │

│                         └────────┬────────┘                                │

## ❗ PROCHAINES ÉTAPES (Priorité)│                                  │ TCP:5005                                │

├──────────────────────────────────┼──────────────────────────────────────────┤

### 1. 🔴 Résoudre le problème de signal visuel (CRITIQUE)│                                  │                                          │

│                    RÉSEAU ETHERNET (10.10.0.x)                             │

Le problème fondamental : **le robot est trop petit dans les images et les changements de pose sont quasi-invisibles au niveau pixel**. Options :│                                  │                                          │

├──────────────────────────────────┼──────────────────────────────────────────┤

#### Option A : Recapturer avec de meilleures conditions (RECOMMANDÉ)│                                  ▼                                          │

- **Fixer l'exposition caméra** (évite la dérive d'éclairage naturelle)│                         ┌─────────────────┐                                │

- **Rapprocher encore les caméras** (robot doit remplir > 50% du frame)│                         │bridge_pi_simple │                                │

- **Éclairage artificiel constant** (pas de lumière naturelle variable)│                         │  (TCP Server)   │                                │

- Modifier `pi_camera_server.py` pour fixer exposure + white-balance :│                         └────────┬────────┘                                │

```python│                                  │                                          │

cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)   # manual mode│                                  ▼                                          │

cap.set(cv2.CAP_PROP_EXPOSURE, -6)        # fixed exposure│                         ┌─────────────────┐                                │

cap.set(cv2.CAP_PROP_AUTO_WB, 0)          # fixed white balance│                         │    pymycobot    │                                │

```│                         │  /dev/ttyAMA0   │                                │

│                         └────────┬────────┘                                │

#### Option B : Finir l'ablation CLAHE + Crop + Grayscale│                                  │                                          │

```bash│                         ┌────────▼────────┐                                │

# Le script est prêt dans /tmp/experiment_ablation.py│                         │  MyCobot 320 Pi │                                │

# Relancer avec stdbuf pour éviter le problème de buffering :│                         │     (Robot)     │                                │

cd ~/ros_jazzy/src/mycobot_R6A│                         └─────────────────┘                                │

stdbuf -oL /home/genji/miniconda/bin/python3 /tmp/experiment_ablation.py 2>&1 | tee /tmp/ablation_log.txt│                                                                             │

```│                     RASPBERRY PI (10.10.0.218)                             │

Experiments à tester : CLAHE, Crop robot region, CLAHE+Crop, Grayscale+Crop.│                   ROS2 Galactic / Ubuntu 20.04 / Python 3.8                │

└─────────────────────────────────────────────────────────────────────────────┘

#### Option C : Approches alternatives```

- Pré-traiter avec edge detection (Canny/Sobel) pour extraire les contours du robot

- Segmentation (SAM) pour isoler le robot avant régression---

- Prédire uniquement J1-J3 (les joints les plus visibles) au lieu des 6

## 📦 Structure du Projet

### 2. 🟡 Si la recapture est choisie — checklist

- [ ] Modifier `pi_camera_server.py` pour fixer exposition et white-balance```

- [ ] Rapprocher les caméras (robot > 50% du frame)mycobot_R6A/

- [ ] Recapturer 2000+ poses avec éclairage constant├── SESSION_RESUME.md              # 👈 CE FICHIER - Point de départ

- [ ] Vérifier corrélation pose ↔ pixel > 0.3 avant de lancer le training├── DEVELOPMENT_SUMMARY.md         # Résumé technique détaillé

├── INDEX.md                       # Index de documentation

### 3. 🟢 Si le training fonctionne├── README.md                      # README principal

- [ ] Lancer `train.py` complet avec meilleures transforms├── bridge_pi_debug.py             # Script debug pour Pi

- [ ] Fine-tune du modèle synthétique (12.97°) sur données réelles│

- [ ] Commit final et deploy├── mycobot_gateway/               # 📦 Package ROS2 Principal

│   ├── package.xml

---│   ├── setup.py

│   │

## 🏗️ Architecture du Système│   ├── mycobot_gateway/           # Modules Python

│   │   ├── __init__.py

```│   │   ├── bridge_tour.py         # ⭐ Client TCP vers Pi

┌─────────────────────────────────────────────────────────────────────────────┐│   │   ├── robot_commander.py     # Interface CLI

│                              PC TOUR (10.10.0.115)                          ││   │   ├── joint_sync.py          # Sync angles → RViz

│                         ROS2 Jazzy / Ubuntu 24.04 / Python 3.12             ││   │   ├── simple_gui.py          # GUI Tkinter

│                         Conda: Python 3.13 / PyTorch 2.6 + CUDA 12.4       ││   │   ├── slider_control.py      # Contrôle sliders

│                         GPU: NVIDIA RTX 4000 Ada (20 GB VRAM)               ││   │   ├── teleop_keyboard.py     # Contrôle clavier

├─────────────────────────────────────────────────────────────────────────────┤│   │   ├── marker_follower.py     # Suivi ArUco

│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌──────────────┐       ││   │   ├── synthetic_data_collector.py      # Collecte données Gazebo v1

│  │ simple_gui  │  │slider_control│  │teleop_keyb. │  │ training/    │       ││   │   └── synthetic_data_collector_v2.py   # 🆕 Collecte v2 (multi-cam + domain rand)

│  │  (Tkinter)  │  │(joint_states)│  │  (clavier)  │  │ train.py     │       ││   │

│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  │ predict.py   │       ││   ├── scripts/

│         │                │                │          │ capture_real  │       ││   │   ├── bridge_pi_simple.py    # ⭐ Script Pi (serveur TCP)

│         └────────────────┴────────────────┘          └──────┬───────┘       ││   │   ├── bridge_pi_standalone.py

│                                   │                         │              ││   │   ├── synthetic_data_collector  # Wrapper ros2 run v1

│                          /to_robot (JSON)            TCP:5005 + 5006       ││   │   └── synthetic_data_collector_v2  # 🆕 Wrapper ros2 run v2

│                                   ▼                         ▼              ││   │

│                         ┌─────────────────┐                                ││   └── launch/

│                         │   bridge_tour   │                                ││       ├── simple_gui.launch.py

│                         └────────┬────────┘                                ││       ├── slider_control.launch.py   # ⭐ Contrôle temps réel validé

├──────────────────────────────────┼──────────────────────────────────────────┤│       ├── teleop_keyboard.launch.py

│                    RÉSEAU ETHERNET (10.10.0.x)                             ││       ├── rviz_sync.launch.py

├──────────────────────────────────┼──────────────────────────────────────────┤│       ├── marker_follow_full.launch.py

│                                  ▼                                          ││       ├── synthetic_data.launch.py       # Pipeline synthétique v1

│            ┌─────────────────┐       ┌─────────────────┐                   ││       └── synthetic_data_v2.launch.py    # 🆕 Pipeline v2 (4 caméras + randomized.sdf)

│            │bridge_pi_simple │       │pi_camera_server │                   ││

│            │  TCP:5005       │       │  TCP:5006       │                   │├── mycobot_description/           # 📦 Package URDF

│            └────────┬────────┘       └────────┬────────┘                   ││

│                     │                         │                            │├── mycobot_description/           # 📦 Package URDF

│                     ▼                         ▼                            ││   ├── urdf/320_pi/               # Modèle 3D robot

│            ┌─────────────────┐       ┌─────────────────┐                   ││   │   └── mycobot_pro_320_pi_gazebo.urdf  # URDF + 4 caméras + contrôleurs

│            │    pymycobot    │       │ Arducam USB ×2  │                   ││   ├── worlds/

│            │  /dev/ttyAMA0   │       │  cam0 + cam3    │                   ││   │   └── randomized.sdf         # 🆕 Monde Gazebo avec domain randomization

│            └────────┬────────┘       └─────────────────┘                   ││   ├── config/mycobot_320_pi.rviz # Config RViz

│                     ▼                                                      ││   └── launch/

│            ┌─────────────────┐                                             ││       ├── display.launch.py

│            │  MyCobot 320 Pi │                                             ││       └── gazebo_sim.launch.py   # Lancement Gazebo + bridges

│            └─────────────────┘                                             ││

│                                                                             │├── docs/                          # Documentation détaillée

│                     RASPBERRY PI (10.10.0.225)                             ││   ├── SYNTHETIC_DATA.md          # 🆕 Guide pipeline données synthétiques

└─────────────────────────────────────────────────────────────────────────────┘│   └── ...

```│

├── training/                      # 🆕 Pipeline entraînement IA (feature/pose-training)

---│   ├── __init__.py

│   ├── dataset.py                 # MyCobotPoseDataset, MultiView, Merged, normalisation

## 📦 Structure du Projet│   ├── model.py                   # PoseResNet + MultiViewPoseResNet (fusion 4 vues)

│   ├── train.py                   # v2: multi-view, domain rand, finetune, camera filter

```│   ├── predict.py                 # Inférence sur image(s)

mycobot_R6A/│   ├── capture_real.py            # 🆕 Capture images réelles via bridge Pi + OpenCV

├── SESSION_RESUME.md              # 👈 CE FICHIER│   ├── README.md                  # Documentation pipeline

├── DEVELOPMENT_SUMMARY.md         # Résumé technique détaillé│   └── requirements.txt           # Dépendances PyTorch

├── INDEX.md                       # Index documentation│

├── README.md                      # README principal└── scripts/                       # Scripts shell utilitaires

│```

├── mycobot_gateway/               # 📦 Package ROS2 Principal

│   ├── mycobot_gateway/---

│   │   ├── bridge_tour.py         # Client TCP vers Pi

│   │   ├── simple_gui.py          # GUI Tkinter## 🚀 DÉMARRAGE RAPIDE

│   │   ├── slider_control.py      # Contrôle sliders temps réel

│   │   └── synthetic_data_collector_v2.py  # Collecte données Gazebo### ⚠️ PRÉREQUIS CRITIQUE - Éviter le conflit Conda

│   ├── scripts/

│   │   ├── bridge_pi_simple.py    # Script Pi (serveur TCP robot)```bash

│   │   └── pi_camera_server.py    # Script Pi (serveur TCP caméras)# TOUJOURS exécuter avant ROS2 (conflit Python 3.13 vs 3.12)

│   └── launch/                    # Fichiers launch ROS2conda deactivate

│

├── mycobot_description/           # 📦 URDF + Gazebo# OU utiliser la commande "propre" :

│   ├── urdf/320_pi/               # Modèle 3D (URDF + 4 caméras Gazebo)env -i HOME=$HOME PATH="/usr/bin:/bin:/opt/ros/jazzy/bin" DISPLAY=$DISPLAY bash

│   └── worlds/randomized.sdf      # Monde Gazebo avec domain randomization```

│

├── training/                      # 📦 Pipeline ML/IA### Étape 1 : Démarrer le Bridge sur le Pi

│   ├── dataset.py                 # Datasets: single/multi-view + PerImageNormalize

│   ├── model.py                   # PoseResNet + MultiViewPoseResNet```bash

│   ├── train.py                   # Training: multi-view, finetune, auto-views# SSH vers le Pi

│   ├── predict.py                 # Inférence sur image(s)ssh er@10.10.0.225

│   ├── capture_real.py            # Capture réelle (FK safety, bridge + camera)

│   └── checkpoints_*/             # Modèles entraînés# Lancer le bridge

│python3 bridge_pi_simple.py

├── scripts/                       # Scripts shell utilitaires```

└── docs/                          # Documentation détaillée

```### Étape 2 : Lancer le contrôle sur le PC Tour



---```bash

# Option A : Slider Control (RECOMMANDÉ - testé et validé)

## 🚀 COMMANDES UTILESenv -i HOME=$HOME PATH="/usr/bin:/bin:/opt/ros/jazzy/bin" DISPLAY=$DISPLAY bash -c '

source /opt/ros/jazzy/setup.bash && 

### Démarrage Bridge Pisource ~/ros_jazzy/src/mycobot_R6A/install/setup.bash && 

```bashros2 launch mycobot_gateway slider_control.launch.py'

ssh er@10.10.0.225

# Terminal 1 : robot# Option B : GUI Simple

python3 bridge_pi_simple.pyros2 launch mycobot_gateway simple_gui.launch.py

# Terminal 2 : caméras

python3 pi_camera_server.py --cameras 0 3 --names cam0 cam3# Option C : Contrôle clavier

```ros2 launch mycobot_gateway teleop_keyboard.launch.py

```

### Capture Données Réelles

```bash### 🆕 Données Synthétiques (Gazebo)

cd ~/ros_jazzy/src/mycobot_R6A

/home/genji/miniconda/bin/python3 training/capture_real.py \```bash

  --output /tmp/real_dataset \# === v1 (single camera, 1000 samples) ===

  --num-samples 2000 \# Branche feature/synthetic-data

  --pi-host 10.10.0.225 \env -i HOME=$HOME PATH="/usr/bin:/bin:/opt/ros/jazzy/bin" DISPLAY=$DISPLAY bash -c '

  --settle-time 3.0 --speed 25 --limit-fraction 0.5source /opt/ros/jazzy/setup.bash && 

```source ~/ros_jazzy/src/mycobot_R6A/install/setup.bash && 

ros2 launch mycobot_gateway synthetic_data.launch.py \

### Entraînement (Conda)  num_samples:=1000 \

```bash  output_dir:=/tmp/mycobot_synth_dataset \

cd ~/ros_jazzy/src/mycobot_R6A  settle_time:=2.0'



# Multi-view synthétique (MEILLEUR : 12.97° MAE)# === v2 (4 cameras, domain randomization, 5000 samples) ===

/home/genji/miniconda/bin/python3 training/train.py \# Branche feature/pose-training

  --dataset /tmp/mycobot_synth_v2 \env -i HOME=$HOME PATH="/usr/bin:/bin:/opt/ros/jazzy/bin" DISPLAY=$DISPLAY bash -c '

  --multi-view --backbone resnet50 \source /opt/ros/jazzy/setup.bash && 

  --epochs 150 --batch-size 16 --lr 1e-4source ~/ros_jazzy/install/setup.bash && 

ros2 launch mycobot_gateway synthetic_data_v2.launch.py \

# Multi-view réel (cam0 + cam3)  num_samples:=5000'

/home/genji/miniconda/bin/python3 training/train.py \

  --dataset /tmp/real_dataset \# Résultat v2 : /tmp/mycobot_synth_v2/

  --multi-view --views cam0 cam3 --backbone resnet50 \#   ├── images/front/000000.png ... 004999.png

  --lr 1e-3 --epochs 300 --batch-size 32 --freeze-epochs 3#   ├── images/right/000000.png ... 004999.png

```#   ├── images/left/000000.png ... 004999.png

#   ├── images/top/000000.png ... 004999.png  (4×5000 = 20,000 images)

---#   └── labels.csv  (index, j1-j6_rad, j1-j6_deg, camera, image_path)

```

## 📊 Comparaison des Modèles

### 🆕 Entraînement IA (PyTorch)

### Données Synthétiques (Gazebo) ✅

```bash

| Configuration | Données | MAE moyenne |# Branche feature/pose-training

|---|---|---|# ⚠️ Utiliser l'environnement Conda (Python 3.13 + PyTorch CUDA)

| ResNet18, single-view | 1000 (v1) | 22.6° |cd ~/ros_jazzy/src/mycobot_R6A

| ResNet50, single-view (front) | 5000 (v2) | 16.5° |

| **ResNet50, multi-view (4 cam)** | **5000 (v2)** | **12.97°** |# === Single-view ResNet50 (front only, 5000 samples) ===

python3 training/train.py \

### Données Réelles (Pi Arducam) ❌  --dataset /tmp/mycobot_synth_v2 \

  --camera-filter front \

| Configuration | Best MAE | vs Baseline | Problème |  --backbone resnet50 --epochs 150 --batch-size 32 --lr 1e-4

|---|---|---|---|

| Toutes approches testées | ~31.7° | -1° | Signal visuel insuffisant |# === Multi-view ResNet50 (4 cameras, 5000 samples) — MEILLEUR RÉSULTAT ===

| Mean predictor (baseline) | 32.76° | — | Robot trop petit dans images |python3 training/train.py \

  --dataset /tmp/mycobot_synth_v2 \

---  --multi-view --backbone resnet50 \

  --epochs 150 --batch-size 16 --lr 1e-4

## 🔑 Points Importants

# === Fine-tune sur données réelles ===

1. **Conda vs ROS2** : Toujours `conda deactivate` avant ROS2. Pour le training ML, utiliser `/home/genji/miniconda/bin/python3`.python3 training/train.py \

2. **Dataset réel** : `/tmp/real_dataset/` — 2000 poses × 2 caméras = 4000 images PNG 640×480  --dataset /tmp/real_dataset \

3. **Dataset synthétique** : `/tmp/mycobot_synth_v2/` — 5000 poses × 4 caméras = 20,000 images  --checkpoint training/checkpoints_mv_resnet50/best_model.pth \

4. **Le problème #1** : les caméras sont trop loin du robot, le robot est trop petit dans l'image  --finetune --lr 1e-5 --epochs 50

5. **Le modèle fonctionne** : overfit test sur 10 samples → 0.01° MAE (prouvé sur données réelles)

6. **Corrélation à vérifier** : avant de relancer un training, toujours vérifier que corr(pose_diff, pixel_diff) > 0.3# Inférence sur une image

python3 training/predict.py \

---  --image /tmp/mycobot_synth_v2/images/front/000042.png \

  --checkpoint training/checkpoints_mv_resnet50/best_model.pth

## 📚 Documentation```



| Fichier | Description |---

|---------|-------------|

| `SESSION_RESUME.md` | 👈 Ce fichier — point de départ |## � Protocole de Communication

| `DEVELOPMENT_SUMMARY.md` | Résumé technique complet |

| `docs/QUICKSTART.md` | Guide démarrage rapide |### Format JSON (Recommandé)

| `docs/SYNTHETIC_DATA.md` | Pipeline données synthétiques |

| `docs/ROBOT_QUICKSTART.md` | Procédure robot réel |```json

| `training/README.md` | Documentation pipeline ML |// Mouvements

{"action": "send_angles", "angles": [0, 0, 0, 0, 0, 0], "speed": 30}

---{"action": "send_coords", "coords": [200, 0, 200, 180, 0, 0], "speed": 40, "mode": 1}

{"action": "go_home"}

*Ce fichier est le point de départ pour les prochaines sessions de développement.*  {"action": "go_zero"}

*Dernière mise à jour : 2 avril 2026*

// Lecture état
{"action": "get_angles"}    // → "ANGLES: [0.43, 0.0, 0.0, 0.35, 0.26, 0.35]"
{"action": "get_coords"}    // → "COORDS: [...]"

// Gripper
{"action": "gripper_open"}
{"action": "gripper_close"}

// Contrôle moteurs
{"action": "power_on"}
{"action": "power_off"}
{"action": "emergency_stop"}
```

### Commandes Texte (Alternative)

| Commande | Description |
|----------|-------------|
| `ping` | Test connexion → `PONG` |
| `status` | État robot |
| `home` / `go_home` | Position home |
| `zero` / `go_zero` | Position zéro |
| `get_angles` / `angles` | Lire angles |
| `get_coords` / `coords` | Lire coordonnées |
| `power_on` | Allumer moteurs |
| `power_off` | Relâcher servos |
| `gripper_open` | Ouvrir pince |
| `gripper_close` | Fermer pince |
| `stop` | Arrêt d'urgence |

---

## 🔌 Configuration Réseau

| Machine | IP | Port | Rôle |
|---------|-----|------|------|
| PC Tour | 10.10.0.115 | - | Calcul, GUI, RViz |
| Raspberry Pi | 10.10.0.225 | 5005 | Bridge robot |

---

## ✅ Tests Validés (Session 26/03/2026)

| Test | Résultat | Notes |
|------|----------|-------|
| `ping` | ✅ PONG | Connexion TCP OK |
| `get_angles` | ✅ `[0.43, 0.0, 0.0, 0.35, 0.26, 0.35]` | Lecture réelle |
| `go_home` | ✅ Robot moved | Position home |
| JSON `send_angles` | ✅ Temps réel | Via slider_control |
| RViz visualisation | ✅ Robot visible | Fixed Frame = base |
| simple_gui | ✅ Fonctionnel | Interface Tkinter |
| slider_control | ✅ Validé complet | Robot suit sliders |

### Tests Validés (Session 31/03/2026)

| Test | Résultat | Notes |
|------|----------|-------|
| Gazebo Harmonic spawn | ✅ Robot visible | `ros_gz_sim` + URDF avec inertials |
| `robot_state_publisher` | ✅ Initialisé | Publie `/robot_description` |
| `ros_gz_bridge` | ✅ Actif | Bridge `/joint_states` Gz → ROS2 |
| Synthetic data pipeline | ✅ Fonctionnel | 5 samples test OK |
| Camera Gz → ROS2 image | ✅ 640x480 RGB PNG | Via `ros_gz_image image_bridge` |
| Joint cmd ROS2 → Gz | ✅ 6 axes bougent | Per-joint Float64 via `ros_gz_bridge` |
| CSV labels export | ✅ rad + deg | Angles réels depuis `/joint_states` |

### Tests Validés (Session 31/03/2026 — Pose Training)

| Test | Résultat | Notes |
|------|----------|-------|
| PyTorch + CUDA | ✅ torch 2.6.0+cu124 | RTX 4000 Ada 20GB VRAM |
| Training pipeline (smoke) | ✅ 2 epochs OK | Pas d'erreur, loss décroissante |
| Training complet 90 epochs | ✅ Early stop | ~3 min, best val loss 0.0348 |
| Mean val MAE | ✅ 22.6° | J1:10.8° J2:15.4° J3:18.3° J4:25.0° J5:33.0° J6:33.2° |
| Inférence `predict.py` | ✅ GPU | Prédictions cohérentes sur images test |
| Git push | ✅ `feature/pose-training` | Commit `bdd63e7` |

### Tests Validés (Session 31/03-01/04/2026 — Pipeline v2)

| Test | Résultat | Notes |
|------|----------|-------|
| URDF 4 caméras | ✅ front/right/left/top | Toutes publient en Gazebo |
| World randomized.sdf | ✅ Chargé | 3 lights, clutter, table |
| Domain randomization (gz service) | ✅ Active | Light direction/color randomisés |
| Collecte v2 5000 samples | ✅ 20,000 images | 4 vues × 5000 poses, 8.3 GB, ~2.5h |
| Gaussian pixel noise | ✅ σ aléatoire 0–5 | Appliqué à chaque image |
| Multi-view smoke test | ✅ 2 epochs OK | 4 vues fusionnées, loss décroissante |
| **Multi-view ResNet50 (150 ep)** | ✅ **12.97° MAE** | 139 epochs (early stop), 121 min |
| Single-view ResNet50 (5k data) | ✅ 16.49° MAE | 89 epochs, front camera only, 20 min |

### 📊 Comparaison des Modèles

| Configuration | Données | MAE moyenne | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|---|---|---|
| ResNet18, single-view | 1000 (v1) | 22.6° | 10.8° | 15.4° | 18.3° | 25.0° | 33.0° | 33.2° |
| ResNet50, single-view | 1000 (v1) | 20.9° | — | — | — | — | — | — |
| ResNet50, single-view (front) | 5000 (v2) | 16.5° | 7.2° | 9.6° | 11.3° | 16.0° | 28.4° | 26.4° |
| **ResNet50, multi-view (4 cam)** | **5000 (v2)** | **12.97°** | **6.4°** | **9.1°** | **8.5°** | **10.8°** | **17.1°** | **25.9°** |

---

## 🔑 Points Importants à Retenir

### 1. Conflit Python Conda
```bash
# TOUJOURS désactiver Conda avant ROS2
conda deactivate
# OU utiliser env -i pour environnement propre
```

### 2. Position HOME vs ZERO
- **HOME** : `[0, 8, -127, 40, 0, 0]` (position sécurisée)
- **ZERO** : `[0, 0, 0, 0, 0, 0]` (tous joints à zéro)

### 3. Ordre de démarrage
1. Pi : `python3 bridge_pi_simple.py`
2. Tour : `ros2 launch mycobot_gateway <launch_file>`

### 4. Compilation
```bash
cd ~/ros_jazzy/src/mycobot_R6A
colcon build --packages-select mycobot_gateway --symlink-install
source install/setup.bash
```

---

## 🚧 TODO - Prochaines Étapes

### Priorité Haute
- [x] Lancer une collecte complète (1000+ samples) de données synthétiques ✅ (31/03/2026)
- [x] Vérifier visuellement les images (le robot change bien de pose à chaque capture) ✅ (31/03/2026)
- [x] Entraîner un modèle de prédiction de pose (CNN/ResNet) ✅ (31/03/2026 — ResNet18, MAE 22.6°)
- [x] Améliorer la précision : plus de données, domain randomization, resnet50 ✅ (01/04/2026)
- [x] Domain randomization Gazebo (éclairage, textures, bruit caméra) ✅ (01/04/2026)
- [x] Augmenter nombre de vues (4 caméras multi-view) ✅ (01/04/2026 — MAE 12.97°)
- [ ] Streaming caméra Pi → Tour (pour inférence en temps réel)
- [ ] Capturer images réelles + fine-tune (nécessite Pi + caméra connectés)
- [ ] Tester `teleop_keyboard` (contrôle clavier)
- [ ] Tester `marker_follower` (suivi ArUco)

### Priorité Moyenne
- [ ] Nœud ROS2 d'inférence temps réel (camera → predict → joint angles)
- [ ] Interface web (option future)
- [ ] Enregistrement/rejeu trajectoires
- [ ] Path planning MoveIt2

### Priorité Basse
- [x] Intégration Gazebo simulation ✅ (31/03/2026 — branche `feature/gazebo`)
- [x] Pipeline données synthétiques ✅ (31/03/2026 — branche `feature/synthetic-data`)
- [x] Pipeline v2 multi-view + domain rand ✅ (01/04/2026 — branche `feature/pose-training`)
- [ ] Multi-robot coordination

---

## 📚 Documentation Complémentaire

| Fichier | Description |
|---------|-------------|
| `DEVELOPMENT_SUMMARY.md` | Résumé technique détaillé |
| `INDEX.md` | Index de toute la documentation |
| `docs/QUICKSTART.md` | Guide démarrage rapide |
| `docs/ARCHITECTURE.md` | Architecture système |
| `docs/DEPLOYMENT.md` | Guide de déploiement |
| `docs/SYNTHETIC_DATA.md` | 🆕 Pipeline données synthétiques |
| `mycobot_gateway/README.md` | Documentation du package |

---

## 🔗 Liens Utiles

- **GitHub** : https://github.com/ABMI-software/mycobot_320pi_R6A
- **ROS2 Jazzy** : https://docs.ros.org/en/jazzy/
- **pymycobot** : https://github.com/elephantrobotics/pymycobot

---

*Ce fichier est le point de départ pour les prochaines sessions de développement.*  
*Dernière mise à jour : 1 avril 2026*
