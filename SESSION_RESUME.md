# 📋 SESSION RESUME - MyCobot 320 Pi Gateway Bridge

> **Date de dernière mise à jour :** 1 avril 2026  
> **Version :** 1.4.0  
> **Repository GitHub :** https://github.com/ABMI-software/mycobot_320pi_R6A  
> **Branche :** `main` | `feature/gazebo` | `feature/synthetic-data` | `feature/pose-training`  
> **Dernier commit :** (feature/pose-training)

---

## 🎯 POINT DE DÉPART - Session Précédente

### ✅ Ce qui a été accompli

| Tâche | Statut | Détails |
|-------|--------|---------|
| Création repo GitHub | ✅ Complété | `ABMI-software/mycobot_320pi_R6A` |
| Architecture TCP Tour ↔ Pi | ✅ Validé | Connexion stable |
| bridge_tour (PC) | ✅ Fonctionnel | Client TCP ROS2 |
| bridge_pi_simple (Pi) | ✅ Fonctionnel | Serveur TCP + pymycobot |
| simple_gui | ✅ Testé | Interface Tkinter |
| slider_control | ✅ Testé | Robot suit les sliders en temps réel |
| RViz visualisation | ✅ Corrigé | Config avec Fixed Frame = base |
| Commandes JSON | ✅ Validé | send_angles, get_angles, go_home... |

### 🔧 Dernières modifications

1. **`slider_control.launch.py`** - Ajout du chemin config RViz
2. **`bridge_pi_simple.py`** - Ajout commandes texte (get_angles, power_on/off, gripper)

---

## �️ Architecture du Système

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              PC TOUR (10.10.0.115)                          │
│                         ROS2 Jazzy / Ubuntu 24.04 / Python 3.12             │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │ simple_gui  │  │slider_control│  │teleop_keyb. │  │marker_follow│        │
│  │  (Tkinter)  │  │(joint_states)│  │  (clavier)  │  │   (ArUco)   │        │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘        │
│         │                │                │                │               │
│         └────────────────┴────────────────┴────────────────┘               │
│                                   │                                         │
│                          /to_robot (JSON)                                   │
│                                   ▼                                         │
│                         ┌─────────────────┐                                │
│                         │   bridge_tour   │◄─── /from_robot                │
│                         │   (TCP Client)  │                                │
│                         └────────┬────────┘                                │
│                                  │ TCP:5005                                │
├──────────────────────────────────┼──────────────────────────────────────────┤
│                                  │                                          │
│                    RÉSEAU ETHERNET (10.10.0.x)                             │
│                                  │                                          │
├──────────────────────────────────┼──────────────────────────────────────────┤
│                                  ▼                                          │
│                         ┌─────────────────┐                                │
│                         │bridge_pi_simple │                                │
│                         │  (TCP Server)   │                                │
│                         └────────┬────────┘                                │
│                                  │                                          │
│                                  ▼                                          │
│                         ┌─────────────────┐                                │
│                         │    pymycobot    │                                │
│                         │  /dev/ttyAMA0   │                                │
│                         └────────┬────────┘                                │
│                                  │                                          │
│                         ┌────────▼────────┐                                │
│                         │  MyCobot 320 Pi │                                │
│                         │     (Robot)     │                                │
│                         └─────────────────┘                                │
│                                                                             │
│                     RASPBERRY PI (10.10.0.218)                             │
│                   ROS2 Galactic / Ubuntu 20.04 / Python 3.8                │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 📦 Structure du Projet

```
mycobot_R6A/
├── SESSION_RESUME.md              # 👈 CE FICHIER - Point de départ
├── DEVELOPMENT_SUMMARY.md         # Résumé technique détaillé
├── INDEX.md                       # Index de documentation
├── README.md                      # README principal
├── bridge_pi_debug.py             # Script debug pour Pi
│
├── mycobot_gateway/               # 📦 Package ROS2 Principal
│   ├── package.xml
│   ├── setup.py
│   │
│   ├── mycobot_gateway/           # Modules Python
│   │   ├── __init__.py
│   │   ├── bridge_tour.py         # ⭐ Client TCP vers Pi
│   │   ├── robot_commander.py     # Interface CLI
│   │   ├── joint_sync.py          # Sync angles → RViz
│   │   ├── simple_gui.py          # GUI Tkinter
│   │   ├── slider_control.py      # Contrôle sliders
│   │   ├── teleop_keyboard.py     # Contrôle clavier
│   │   ├── marker_follower.py     # Suivi ArUco
│   │   ├── synthetic_data_collector.py      # Collecte données Gazebo v1
│   │   └── synthetic_data_collector_v2.py   # 🆕 Collecte v2 (multi-cam + domain rand)
│   │
│   ├── scripts/
│   │   ├── bridge_pi_simple.py    # ⭐ Script Pi (serveur TCP)
│   │   ├── bridge_pi_standalone.py
│   │   ├── synthetic_data_collector  # Wrapper ros2 run v1
│   │   └── synthetic_data_collector_v2  # 🆕 Wrapper ros2 run v2
│   │
│   └── launch/
│       ├── simple_gui.launch.py
│       ├── slider_control.launch.py   # ⭐ Contrôle temps réel validé
│       ├── teleop_keyboard.launch.py
│       ├── rviz_sync.launch.py
│       ├── marker_follow_full.launch.py
│       ├── synthetic_data.launch.py       # Pipeline synthétique v1
│       └── synthetic_data_v2.launch.py    # 🆕 Pipeline v2 (4 caméras + randomized.sdf)
│
├── mycobot_description/           # 📦 Package URDF
│
├── mycobot_description/           # 📦 Package URDF
│   ├── urdf/320_pi/               # Modèle 3D robot
│   │   └── mycobot_pro_320_pi_gazebo.urdf  # URDF + 4 caméras + contrôleurs
│   ├── worlds/
│   │   └── randomized.sdf         # 🆕 Monde Gazebo avec domain randomization
│   ├── config/mycobot_320_pi.rviz # Config RViz
│   └── launch/
│       ├── display.launch.py
│       └── gazebo_sim.launch.py   # Lancement Gazebo + bridges
│
├── docs/                          # Documentation détaillée
│   ├── SYNTHETIC_DATA.md          # 🆕 Guide pipeline données synthétiques
│   └── ...
│
├── training/                      # 🆕 Pipeline entraînement IA (feature/pose-training)
│   ├── __init__.py
│   ├── dataset.py                 # MyCobotPoseDataset, MultiView, Merged, normalisation
│   ├── model.py                   # PoseResNet + MultiViewPoseResNet (fusion 4 vues)
│   ├── train.py                   # v2: multi-view, domain rand, finetune, camera filter
│   ├── predict.py                 # Inférence sur image(s)
│   ├── capture_real.py            # 🆕 Capture images réelles via bridge Pi + OpenCV
│   ├── README.md                  # Documentation pipeline
│   └── requirements.txt           # Dépendances PyTorch
│
└── scripts/                       # Scripts shell utilitaires
```

---

## 🚀 DÉMARRAGE RAPIDE

### ⚠️ PRÉREQUIS CRITIQUE - Éviter le conflit Conda

```bash
# TOUJOURS exécuter avant ROS2 (conflit Python 3.13 vs 3.12)
conda deactivate

# OU utiliser la commande "propre" :
env -i HOME=$HOME PATH="/usr/bin:/bin:/opt/ros/jazzy/bin" DISPLAY=$DISPLAY bash
```

### Étape 1 : Démarrer le Bridge sur le Pi

```bash
# SSH vers le Pi
ssh er@10.10.0.225

# Lancer le bridge
python3 bridge_pi_simple.py
```

### Étape 2 : Lancer le contrôle sur le PC Tour

```bash
# Option A : Slider Control (RECOMMANDÉ - testé et validé)
env -i HOME=$HOME PATH="/usr/bin:/bin:/opt/ros/jazzy/bin" DISPLAY=$DISPLAY bash -c '
source /opt/ros/jazzy/setup.bash && 
source ~/ros_jazzy/src/mycobot_R6A/install/setup.bash && 
ros2 launch mycobot_gateway slider_control.launch.py'

# Option B : GUI Simple
ros2 launch mycobot_gateway simple_gui.launch.py

# Option C : Contrôle clavier
ros2 launch mycobot_gateway teleop_keyboard.launch.py
```

### 🆕 Données Synthétiques (Gazebo)

```bash
# === v1 (single camera, 1000 samples) ===
# Branche feature/synthetic-data
env -i HOME=$HOME PATH="/usr/bin:/bin:/opt/ros/jazzy/bin" DISPLAY=$DISPLAY bash -c '
source /opt/ros/jazzy/setup.bash && 
source ~/ros_jazzy/src/mycobot_R6A/install/setup.bash && 
ros2 launch mycobot_gateway synthetic_data.launch.py \
  num_samples:=1000 \
  output_dir:=/tmp/mycobot_synth_dataset \
  settle_time:=2.0'

# === v2 (4 cameras, domain randomization, 5000 samples) ===
# Branche feature/pose-training
env -i HOME=$HOME PATH="/usr/bin:/bin:/opt/ros/jazzy/bin" DISPLAY=$DISPLAY bash -c '
source /opt/ros/jazzy/setup.bash && 
source ~/ros_jazzy/install/setup.bash && 
ros2 launch mycobot_gateway synthetic_data_v2.launch.py \
  num_samples:=5000'

# Résultat v2 : /tmp/mycobot_synth_v2/
#   ├── images/front/000000.png ... 004999.png
#   ├── images/right/000000.png ... 004999.png
#   ├── images/left/000000.png ... 004999.png
#   ├── images/top/000000.png ... 004999.png  (4×5000 = 20,000 images)
#   └── labels.csv  (index, j1-j6_rad, j1-j6_deg, camera, image_path)
```

### 🆕 Entraînement IA (PyTorch)

```bash
# Branche feature/pose-training
# ⚠️ Utiliser l'environnement Conda (Python 3.13 + PyTorch CUDA)
cd ~/ros_jazzy/src/mycobot_R6A

# === Single-view ResNet50 (front only, 5000 samples) ===
python3 training/train.py \
  --dataset /tmp/mycobot_synth_v2 \
  --camera-filter front \
  --backbone resnet50 --epochs 150 --batch-size 32 --lr 1e-4

# === Multi-view ResNet50 (4 cameras, 5000 samples) — MEILLEUR RÉSULTAT ===
python3 training/train.py \
  --dataset /tmp/mycobot_synth_v2 \
  --multi-view --backbone resnet50 \
  --epochs 150 --batch-size 16 --lr 1e-4

# === Fine-tune sur données réelles ===
python3 training/train.py \
  --dataset /tmp/real_dataset \
  --checkpoint training/checkpoints_mv_resnet50/best_model.pth \
  --finetune --lr 1e-5 --epochs 50

# Inférence sur une image
python3 training/predict.py \
  --image /tmp/mycobot_synth_v2/images/front/000042.png \
  --checkpoint training/checkpoints_mv_resnet50/best_model.pth
```

---

## � Protocole de Communication

### Format JSON (Recommandé)

```json
// Mouvements
{"action": "send_angles", "angles": [0, 0, 0, 0, 0, 0], "speed": 30}
{"action": "send_coords", "coords": [200, 0, 200, 180, 0, 0], "speed": 40, "mode": 1}
{"action": "go_home"}
{"action": "go_zero"}

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
