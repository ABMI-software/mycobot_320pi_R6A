# 🤖 MyCobot 320 Pi - Résumé de Développement

> **Date de dernière mise à jour:** 21 avril 2026
> **Version:** 2.1.0
> **Repository GitHub:** https://github.com/ABMI-software/mycobot_320pi_R6A
> **Branche:** `feature/pose-training`

---

## 📌 Point de Départ Rapide

👉 **Pour démarrer une nouvelle session, consultez [`SESSION_RESUME.md`](SESSION_RESUME.md)**

---

## 📋 Vue d'ensemble du projet

### Objectif
Contrôler un robot **MyCobot 320 Pi** depuis un PC distant (**Tour**) via ROS2 et une connexion TCP, avec un **pipeline vision-IA complet** : simulation Gazebo → données synthétiques → entraînement CNN (pose estimation) → capture et entraînement sur données réelles.

### Architecture distribuée Tour ↔ Raspberry Pi

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              PC TOUR (10.10.0.115)                          │
│                         ROS2 Jazzy / Ubuntu 24.04 / Python 3.12             │
│                         Conda: Python 3.13 / PyTorch 2.6 + CUDA 12.4       │
│                         GPU: NVIDIA RTX 4000 Ada (20 GB VRAM)               │
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
│                    │                         │                             │
│                    ▼                         ▼                             │
│           ┌─────────────────┐       ┌─────────────────┐                    │
│           │    pymycobot    │       │ Arducam USB ×2  │                    │
│           │  /dev/ttyAMA0   │       │  cam0 + cam3    │                    │
│           └────────┬────────┘       └─────────────────┘                    │
│                    ▼                                                       │
│           ┌─────────────────┐                                              │
│           │  MyCobot 320 Pi │                                              │
│           └─────────────────┘                                              │
│                                                                            │
│                     RASPBERRY PI (10.10.0.223)                             │
└────────────────────────────────────────────────────────────────────────────┘
```

### Principe
- **Tour (PC):** Calculs complexes (vision, IA, path planning, visualisation RViz/Gazebo)
- **Raspberry Pi:** Exécution simple des commandes robot (contrôle moteurs via pymycobot)

---

## 📁 Structure du Workspace Tour

```
~/ros_jazzy/src/mycobot_R6A/
├── SESSION_RESUME.md               # Point de départ sessions dev
├── DEVELOPMENT_SUMMARY.md          # Ce fichier
├── CHANGELOG.md                    # Historique des versions
├── README.md                       # README principal
│
├── mycobot_description/            # Package URDF et visualisation
│   ├── urdf/
│   │   ├── 320_pi/
│   │   │   ├── mycobot_pro_320_pi.urdf             # URDF RViz
│   │   │   ├── mycobot_pro_320_pi_gazebo.urdf      # URDF Gazebo (inertials + plugins)
│   │   │   ├── link6_2022.dae                      # Mesh link6 compat. gripper
│   │   │   └── base.dae, link1.dae … link6.dae     # Meshes 3D
│   │   └── pro_adaptive_gripper/                   # Gripper adaptatif (7 meshes DAE)
│   ├── worlds/
│   │   ├── randomized.sdf                          # Monde de base
│   │   ├── randomized_v2.sdf                       # 6 lights + 12 clutter objects (synth v2)
│   │   ├── pick_and_place.sdf                      # Cube rouge + zone verte (mono-objet)
│   │   └── pick_and_place_sorting.sdf              # 4 objets colorés + 4 bacs colorés à parois
│   ├── config/mycobot_320_pi.rviz
│   └── launch/
│       ├── display.launch.py
│       └── gazebo_sim.launch.py
│
├── mycobot_gateway/                # Package communication, contrôle et vision
│   ├── mycobot_gateway/
│   │   ├── bridge_tour.py                      # Client TCP vers la Pi
│   │   ├── simple_gui.py                       # GUI Tkinter
│   │   ├── slider_control.py                   # Sliders + Joint State Publisher
│   │   ├── teleop_keyboard.py                  # Contrôle clavier
│   │   ├── robot_commander.py                  # CLI interactif
│   │   ├── joint_sync.py                       # Synchro robot → RViz
│   │   ├── dream_inference_node.py             # Inférence DREAM + PnP ROS2
│   │   ├── pick_and_place_node.py              # State machine pick & place mono-objet
│   │   ├── color_object_detector.py            # HSV + back-projection (top camera) — sorting
│   │   ├── sorting_orchestrator.py             # Pick-and-place multi-couleur (boucle 4 objets)
│   │   ├── trajectory_to_robot_bridge.py       # JointTrajectory rad → JSON deg (téléop réel)
│   │   ├── gripper_to_robot_bridge.py          # Bridge gripper (no-op tant que pas de pince)
│   │   └── synthetic_data_collector_v2.py      # Collecte Gazebo + anti-collision FK
│   ├── scripts/
│   │   ├── bridge_pi_simple.py     # Script Pi (serveur TCP robot)
│   │   └── pi_camera_server.py     # Script Pi (serveur TCP caméras)
│   └── launch/
│       ├── bridge_only.launch.py
│       ├── simple_gui.launch.py
│       ├── slider_control.launch.py
│       ├── teleop_keyboard.launch.py
│       ├── commander.launch.py
│       ├── rviz_sync.launch.py
│       ├── pick_and_place.launch.py            # Mono-objet (cube rouge → zone verte)
│       ├── pick_and_place_sorting.launch.py    # Sorting 4 couleurs → 4 bacs
│       ├── mycobot_teleop.launch.py            # Téléop main (target=sim/real/both)
│       ├── synthetic_data.launch.py
│       ├── synthetic_data_v2.launch.py
│       └── synthetic_data_v3.launch.py
│
├── training/                       # Pipeline ML/IA
│   ├── model.py                    # Legacy: PoseResNet / MultiViewPoseResNet
│   ├── dataset.py                  # Legacy: datasets single/multi-view
│   ├── train.py                    # Legacy: régression directe (abandonné)
│   ├── predict.py                  # Legacy: inférence régression
│   ├── capture_real.py             # Capture données réelles (FK safety)
│   └── dream/                      # DREAM pipeline (actif)
│       ├── mycobot_fk.py           # Forward Kinematics DH + projection
│       ├── mycobot_ik.py           # Inverse Kinematics Jacobien
│       ├── convert_to_ndds.py      # Conversion → NDDS
│       ├── merge_and_convert.py    # Fusion réel+synth → NDDS
│       ├── train_dream.py          # Wrapper entraînement
│       ├── train_dream_augmented.py# Entraînement + augmentation agressive
│       ├── train_dream_weighted.py # Entraînement pondéré par keypoint
│       ├── evaluate_dream.py       # Évaluation + filtre sentinel -999.99
│       ├── infer_dream.py          # Inférence single-image + PnP
│       ├── visualize_ndds.py       # Vérification visuelle annotations
│       ├── finetune_real.py        # Fine-tuning expérimental (⚠️ ne fonctionne pas)
│       └── manip_configs/mycobot320.yaml
│
├── datasets/                       # Données (Git LFS)
│   ├── synthetic_dataset/          # 5000 poses × 4 vues = 20K images
│   └── real_dataset/               # 2000 poses × 2 caméras = 4000 images
│
├── scripts/
│   ├── train_pipeline.sh           # Pipeline merge → NDDS → training
│   └── monitor_collection.sh       # Monitoring collecte en temps réel
│
└── docs/                           # Documentation
```

---

## 🔧 Packages développés

### 1. `mycobot_description`
**Type:** ament_cmake
**But:** Modèle 3D du robot (URDF, meshes, Gazebo simulation)

| Composant | Description |
|-----------|-------------|
| `urdf/320_pi/*.urdf` | URDF RViz (sans inertials) |
| `urdf/320_pi/*_gazebo.urdf` | URDF Gazebo (inertials, plugins, 4 caméras) |
| `urdf/pro_adaptive_gripper/` | Gripper adaptatif (7 meshes DAE) |
| `worlds/randomized.sdf` | Monde Gazebo de base (synthetic data v1) |
| `worlds/randomized_v2.sdf` | Monde v2 (6 lights, 12 clutter objects, 3 murs) — synthetic data v2 |
| `worlds/pick_and_place.sdf` | Cube cible rouge + zone de dépose verte (pick-and-place mono-objet) |
| `worlds/pick_and_place_sorting.sdf` | 4 objets colorés (cube R, cube B, cylindre G, boîte Y) + 4 bacs assortis (sorting) |
| `config/mycobot_320_pi.rviz` | Config RViz prête à l'emploi |

### 2. `mycobot_gateway`
**Type:** ament_python
**But:** Bridge réseau, contrôle robot, vision et collecte de données

| Nœud | Description |
|------|-------------|
| `bridge_tour` | Client TCP vers Pi (TCP:5005) |
| `simple_gui` | GUI Tkinter (angles, coords, gripper, LED) |
| `slider_control` | Sliders + Joint State Publisher + RViz |
| `teleop_keyboard` | Contrôle clavier WASD+ZX |
| `robot_commander` | CLI interactif |
| `joint_sync` | Synchronisation robot réel → RViz |
| `dream_inference` | Inférence DREAM + PnP en temps réel |
| `pick_and_place` | State machine pick & place mono-objet (Gazebo) |
| `color_object_detector` | Segmentation HSV + back-projection (top camera) — sorting |
| `sorting_orchestrator` | Pick-and-place multi-couleur (4 objets → 4 bacs assortis) |
| `trajectory_to_robot_bridge` | JointTrajectory rad → JSON deg (téléop main → robot réel) |
| `gripper_to_robot_bridge` | Bridge gripper (no-op tant que le robot physique n'a pas de pince) |
| `synth_data_collector` | Collecte données synthétiques (anti-collision FK) |

---

## 🌐 Communication TCP - Protocole

### Configuration réseau
- **IP Raspberry Pi:** `10.10.0.223`
- **Port TCP robot:** `5005`
- **Port TCP caméras:** `5006`
- **Format commandes:** JSON avec terminaison `\n`

### Commandes supportées (Pi → Robot)

| Commande | Description | Exemple |
|----------|-------------|---------|
| `ping` | Test de connexion | `ping` → `pong` |
| `status` | État du robot | `status` → `status:ok` |
| `get_angles` | Lire angles actuels | `get_angles` → `angles:[0,0,0,0,0,0]` |
| `go_home:SPEED` | Position home | `go_home:20` → `home_ok:speed=20` |
| `set_led:R,G,B` | Changer LED | `set_led:255,0,0` → `led_ok:r=255,g=0,b=0` |
| `set_angle:J,A,S` | Un seul joint | `set_angle:1,45,20` → `angle_ok:j=1,a=45.0,s=20` |
| `set_angles:A1,A2,A3,A4,A5,A6:S` | Tous les joints | `set_angles:0,0,0,0,0,90:20` |

---

## 🚀 Guide d'utilisation

### Prérequis
```bash
# IMPORTANT: Désactiver Conda avant ROS2 (Python 3.13 vs 3.12)
conda deactivate
source /opt/ros/jazzy/setup.bash
source ~/ros_jazzy/src/mycobot_R6A/install/setup.bash
```

### 1. Visualisation standalone (sans robot)
```bash
ros2 launch mycobot_description display.launch.py
```

### 2. Communication avec le robot réel

**Sur la Raspberry Pi (10.10.0.223) :**
```bash
ssh er@10.10.0.223
# Terminal 1 : bridge robot
python3 bridge_pi_simple.py
# Terminal 2 : serveur caméras
python3 pi_camera_server.py --cameras 0 3 --names cam0 cam3
```

**Sur la Tour (en parallèle) :**
```bash
conda deactivate
source /opt/ros/jazzy/setup.bash
source ~/ros_jazzy/src/mycobot_R6A/install/setup.bash

# Modes de contrôle disponibles :
ros2 launch mycobot_gateway simple_gui.launch.py        # GUI graphique
ros2 launch mycobot_gateway slider_control.launch.py    # Sliders + RViz
ros2 launch mycobot_gateway teleop_keyboard.launch.py   # Clavier WASD
ros2 launch mycobot_gateway rviz_sync.launch.py         # Sync robot → RViz
```

### 3. Pick-and-place en simulation Gazebo

**Mono-objet** (cube rouge → zone verte) :
```bash
ros2 launch mycobot_gateway pick_and_place.launch.py
ros2 launch mycobot_gateway pick_and_place.launch.py use_vision:=false   # open-loop IK
```

**Multi-couleur sorting** (4 objets → 4 bacs assortis) — validé end-to-end le 23/04/2026 :
```bash
ros2 launch mycobot_gateway pick_and_place_sorting.launch.py
ros2 launch mycobot_gateway pick_and_place_sorting.launch.py use_detector:=false
ros2 launch mycobot_gateway pick_and_place_sorting.launch.py process_order:=blue,green
```

Stack : Gazebo + spawn robot + 4 caméras + `color_object_detector` (T+6 s, HSV) + `sorting_orchestrator` (T+10 s, IK + état machine). Émulation du grasp via `gz service /world/<world>/set_pose` (téléport du modèle sur l'EE pendant le portage).

Voir [`docs/TELEOP_SIM_TESTING.md`](docs/TELEOP_SIM_TESTING.md) pour les critères d'acceptation et les use cases sim-only.

### 4. Téléopération par la main (Wilor + Astra)

```bash
# 4 terminaux : rosbridge + Gazebo + teleop conda + dashboard
# Voir docs/TELEOPERATION.md pour le workflow complet
ros2 launch mycobot_gateway mycobot_teleop.launch.py target:=sim    # sim seul
ros2 launch mycobot_gateway mycobot_teleop.launch.py target:=both   # sim + bras réel
```

Pipeline validé sur le robot physique le 22/04/2026 — voir [`docs/REAL_ROBOT_TEST_PROCEDURE.md`](docs/REAL_ROBOT_TEST_PROCEDURE.md).

---

## ⚠️ Problèmes connus et solutions

### 1. `ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'`
**Cause:** Conda activé (Python 3.13) incompatible avec ROS2 Jazzy (Python 3.12)
**Solution:** `conda deactivate` avant toute commande ROS2

### 2. Connexion TCP instable
**Cause:** Plusieurs instances de `bridge_tour` en parallèle
**Solution:** `pkill -f bridge_tour` puis relancer une seule instance

### 3. Meshes Gazebo non trouvés
**Cause:** `GZ_SIM_RESOURCE_PATH` non défini
**Solution:** Ajouter dans le launch file ou `export GZ_SIM_RESOURCE_PATH=~/ros_jazzy/install/mycobot_description/share`

### 4. DREAM — Belief maps effondrées (all-zeros)
**Cause:** Fine-tuning manuel avec MSE sur grille quasi-vide
**Solution:** Utiliser uniquement `train_network.py` natif de DREAM (pas de fine-tuning custom)

### 5. DREAM — Détection 0% après training
**Cause:** `sigma=4` dans les belief maps au lieu de `sigma=2`
**Solution:** Vérifier la config YAML DREAM — `sigma: 2`

---

## 📝 Fichiers sur la Raspberry Pi (10.10.0.223)

Les scripts standalone à copier sur la Pi :

| Fichier (dans `mycobot_gateway/scripts/`) | Rôle |
|------------------------------------------|------|
| `bridge_pi_simple.py` | Serveur TCP:5005 → contrôle robot via pymycobot |
| `pi_camera_server.py` | Serveur TCP:5006 → streaming JPEG Arducam USB |

Démarrage :
```bash
# Robot
python3 bridge_pi_simple.py

# Caméras (cam0 et cam3)
python3 pi_camera_server.py --cameras 0 3 --names cam0 cam3
```

---

## 🔮 Développements futurs

### Prioritaire (domain gap sim-to-real)
- [x] Domain Randomization v2 (6 lights, 12 clutter, 3 murs)
- [x] Fine-tuning custom — tentatives v1/v2 échouées (belief map collapse, sigma mismatch)
- [x] Entraînement mixte réel+synth via DREAM natif (18K frames, 50 epochs, terminé 16/04/2026)
- [x] **Pick-and-place mono-objet** end-to-end (cube rouge → zone verte) — `pick_and_place.launch.py`
- [x] **Pick-and-place sorting 4 couleurs** end-to-end (HSV + IK + bins) — `pick_and_place_sorting.launch.py`, validé 23/04/2026
- [x] **Téléopération main → bras réel** validée sur le MyCobot 320 Pi physique (22/04/2026, gains 1.2/1.2/1.6/0.25, latence ~150–250 ms)
- [x] Procédure de validation **sim-only** documentée — [`docs/TELEOP_SIM_TESTING.md`](docs/TELEOP_SIM_TESTING.md)
- [x] **Évaluation finale du modèle mixte** sur réel + synth + relaxed (28/04/2026) — voir [`CHANGELOG.md` § 1.12.0](CHANGELOG.md). Verdict : 47.3 % réel / 91.9 % synth, distal keypoints (link4-6) = bottleneck restant.
- [ ] **Capturer 5–10 K poses réelles bras-étendu** + retrain mixte v2 (cible : ≥ 70 % détection sur réel) — étape suivante
- [ ] **Self-supervised labeling** : FK + caméra calibrée → annotations GT automatiques sur réel
- [ ] Fine-tune sur données réelles auto-annotées

### Moyen terme
- [x] Nœud ROS2 d'inférence DREAM (`dream_inference_node.py`)
- [x] Pipeline pick-and-place simulation (`pick_and_place_node.py`)
- [ ] **Bench test robot réel** une fois detection > 50%
- [ ] Intégration MoveIt2 pour planification de trajectoire

### Long terme
- [ ] Interface GUI avec overlay keypoints en temps réel
- [ ] Multi-robot coordination

---

## 🧠 Pipeline Vision / IA - Pose Estimation

### Architecture du pipeline

```
═══════════════════════════════════════════════════════════════════════
  Phase 1 : Régression directe (image → angles)    [ABANDONNÉ]
═══════════════════════════════════════════════════════════════════════

Simulation Gazebo (4 caméras) → Données Synthétiques (5000 poses × 4 vues)
                                        ↓
                               train.py (Multi-view ResNet50)
                                        ↓
                              Modèle synthétique : 12.97° MAE ✅
                                        ↓
                              Fine-tune sur données réelles
                                        ↓
Capture réelle (2 caméras Pi) → Données Réelles (2000 poses × 2 vues)
                                        ↓
                               train.py (transfer learning)
                                        ↓
                              ❌ BLOQUÉ : stagne à 32.76° (baseline)
                              Cause : robot trop petit (15% pixels)

═══════════════════════════════════════════════════════════════════════
  Phase 2 : DREAM Keypoint Detection (image → keypoints → PnP)  [ACTIF]
═══════════════════════════════════════════════════════════════════════

Données Synthétiques → convert_to_ndds.py → Format NDDS (20K frames)
                                                    ↓
                                         train_dream.py (VGG-19)
                                                    ↓
                                         Modèle VGG-aug : 97% détection,
                                         3.1px médiane sur synthétique ✅
                                                    ↓
                                         Transfert sim-to-real : ~26% détection
                                         ❌ Domain gap à réduire
```

### Résultats d'entraînement

#### Données synthétiques (Gazebo) ✅
| Configuration | Dataset | MAE |
|---|---|---|
| ResNet18 single-view | 1000 (v1) | 22.6° |
| ResNet50 single-view (front) | 5000 (v2) | 16.5° |
| **ResNet50 multi-view (4 cam)** | **5000 (v2)** | **12.97°** |

#### Données réelles (Pi Arducam) ❌
| Approche | Best MAE | vs Baseline (32.76°) |
|---|---|---|
| Multi-view ResNet50, lr=1e-3 | 32.7° | -0.06° |
| Single-view ResNet18, lr=3e-3 | 32.8° | +0.04° |
| SmoothL1 + OneCycleLR | 31.75° | -1.01° |
| PerImageNormalize + SmoothL1 | 31.69° | -1.07° |
| Overfit 10 samples | **0.01°** | — (prouve que le modèle fonctionne) |

### Diagnostic du problème données réelles

**Cause racine identifiée** : le signal visuel est insuffisant dans les images.

- **Corrélation pose ↔ pixel = 0.004** (quasi-nulle), même entre paires consécutives
- Robot = 15.4% (cam0) / 5.7% (cam3) des pixels — le reste est du fond statique
- Dérive d'éclairage massive pendant les 2h de capture (luminosité 140 → 82 → 143)
- Même entre poses extrêmes (J1 : ±83°), seulement 9.1% des pixels changent

**Solution recommandée** : recapturer avec exposition caméra fixe, caméras plus proches (robot > 50% du frame), éclairage artificiel constant.

### Fichiers du pipeline training/

| Fichier | Rôle |
|---|---|
| `training/model.py` | PoseResNet (single) + MultiViewPoseResNet (multi) |
| `training/dataset.py` | MyCobotDataset + MyCobotMultiViewDataset + PerImageNormalize |
| `training/train.py` | Script complet : fine-tune, multi-view, --views, auto num_views |
| `training/capture_real.py` | Capture réelle avec FK safety (dimensions URDF) |
| `training/predict.py` | Inférence sur image(s) |
| `training/preview_cameras.py` | Prévisualisation caméras Pi |

---

## 🎯 DREAM — Keypoint-Based Pose Estimation (Phase 2)

### Motivation du changement d'approche

L'approche directe (image → angles par régression CNN) a **échoué sur données réelles** car le robot est trop petit dans l'image (15% des pixels). L'approche DREAM résout ce problème en détectant les **positions 2D des articulations** (keypoints) via des belief maps, puis en résolvant la pose caméra par **PnP** (Perspective-n-Point).

**Avantage clé :** DREAM est agnostique à la taille du robot dans l'image — il suffit que les keypoints soient visibles.

### Pipeline DREAM

```
Image 640×480 → Resize 400×400 → VGG-19 backbone → 6 stages cascadés → 7 belief maps (100×100)
                                                                              ↓
                                                                      Peak Detection (argmax)
                                                                              ↓
                                                                      7 keypoints 2D (pixels)
                                                                              ↓
                        3D keypoints (FK URDF) → PnP (Levenberg-Marquardt) → Pose caméra [R|t]
```

### Architecture testées

#### VGG-Q (✅ Recommandé)
- **Backbone :** VGG-19 (ImageNet, **sans BatchNorm**)
- **Head :** 6 stages de raffinement cascadés (style DOPE)
- **Entrée :** 400×400 RGB → **Sortie :** 7 belief maps 100×100
- **Paramètres :** 28.2M
- **Entraînement :** Stable, convergence régulière

#### ResNet-H (❌ Instable)
- **Backbone :** ResNet-101 (ImageNet, **avec BatchNorm**)
- **Head :** Hourglass decoder avec deconvolutions
- **Entrée :** 400×400 RGB → **Sortie :** 7 belief maps 208×208
- **Paramètres :** 54.0M
- **Problème :** BatchNorm avec `batch_size < 64` → statistiques running corrompues → val loss oscillante (0.0003 → 96.0)

### 7 Keypoints (correspondance URDF)

| Keypoint | Frame URDF | Description |
|----------|-----------|-------------|
| `mycobot320_base` | `base` | Base du robot (fixe) |
| `mycobot320_link1` | `link1` | Après joint 1 (yaw) |
| `mycobot320_link2` | `link2` | Après joint 2 |
| `mycobot320_link3` | `link3` | Après joint 3 |
| `mycobot320_link4` | `link4` | Après joint 4 |
| `mycobot320_link5` | `link5` | Après joint 5 |
| `mycobot320_link6` | `link6` | Effecteur (end-effector) |

### Résultats d'entraînement

#### Comparaison des 3 modèles (données synthétiques, 20K frames)

| Modèle | Époques | Meilleur Val Loss | Stabilité | Détection | Erreur moyenne | Erreur médiane | <10px |
|--------|---------|-------------------|-----------|-----------|----------------|----------------|-------|
| ResNet-H | 10/25 (tué) | 0.000305 (E1 seul) | ❌ Oscillation sauvage | N/A | N/A | N/A | N/A |
| VGG-base | 25 ✅ | 0.000438 (E8) | ✅ Stable | 96.1% | 14.1px | 3.1px | 75% |
| **VGG-aug** | **25** ✅ | **0.000667** (E22) | ✅ **Stable** | **96.6%** | **13.1px** | **3.1px** | **78%** |

**Hyperparamètres :** batch_size=32, lr=0.0001, Adam, 25 époques, MSE loss sur belief maps.

#### Précision par keypoint (VGG-aug, meilleur modèle, keypoints détectés uniquement)

| Keypoint | Détection | Moyenne | Médiane | P90 | <5px | <10px | <20px |
|----------|-----------|---------|---------|-----|------|-------|-------|
| base | 100% | 2.9px | 2.8px | 3.1px | 100% | 100% | 100% |
| link1 | 100% | 2.7px | 2.6px | 3.0px | 100% | 100% | 100% |
| link2 | 100% | 2.7px | 2.6px | 3.0px | 100% | 100% | 100% |
| link3 | 99% | 11.0px | 5.6px | 23.9px | 45% | 74% | 88% |
| link4 | 96% | 19.2px | 6.4px | 50.5px | 39% | 65% | 80% |
| link5 | 95% | 27.4px | 8.8px | 77.2px | 26% | 53% | 70% |
| link6 | 86% | 28.9px | 10.1px | 77.4px | 19% | 50% | 69% |
| **TOTAL** | **97%** | **13.1px** | **3.1px** | **26.9px** | **63%** | **78%** | **87%** |

**Observation :** La précision se dégrade de la base vers l'effecteur (gradient base→link6). Les joints proximaux (base, link1, link2) sont quasi-parfaits (~2.7px), tandis que les joints distaux (link5, link6) sont plus difficiles (~10px médiane). C'est attendu : les joints distaux ont plus de variation positionnelle.

#### Interprétation physique : conversion px → mm → degrés

Les erreurs en **pixels** mesurent la distance euclidienne entre le keypoint prédit et sa vraie position dans l'image 640×480. Pour interpréter ces valeurs en termes physiques :

**Formule de conversion :** `1 px = distance_caméra / focal_length × 1000 mm`

Avec nos intrinsèques Gazebo (fx = fy = 554.38 px) :

| Caméra | Distance | 1 px ≈ |
|--------|----------|--------|
| front / right / left | 0.8 m | **1.44 mm** |
| top | 1.2 m | 2.16 mm |

**Erreur physique par keypoint (caméras latérales à 0.8m, 1px = 1.44mm) :**

| Keypoint | Erreur pixel | Erreur physique | Erreur angulaire estimée |
|----------|-------------|-----------------|--------------------------|
| base / link1 / link2 | 2.7px médiane | **3.9 mm** | ~0.7° |
| link3 | 5.6px médiane | 8.1 mm | ~2.1° |
| link4 | 6.4px médiane | 9.2 mm | ~3.4° |
| link5 | 8.8px médiane | 12.7 mm | ~8.7° |
| link6 (end-effector) | 10.1px médiane | **14.6 mm** | ~18.3° |
| **TOTAL médiane** | **3.1px** | **4.5 mm** | **~0.8°** |
| **TOTAL moyenne** | **13.1px** | **18.9 mm** | **~3.4°** |

> **Note :** L'erreur angulaire est estimée par `θ = arctan(erreur_mm / longueur_bras_levier)`. Pour les joints distaux (link5, link6), le bras de levier est court (57–70mm), donc une petite erreur en mm produit une grande erreur angulaire. Pour le bras complet (320mm), l'erreur est beaucoup plus faible.

**Comparaison Phase 1 (régression directe) vs Phase 2 (DREAM) :**

| Approche | Erreur angulaire (synthétique) |
|----------|-------------------------------|
| Phase 1 — ResNet50 multi-view | 12.97° MAE |
| Phase 2 — DREAM VGG-aug | **~0.8° médiane / ~3.4° moyenne** |

→ **DREAM est ~4× meilleur en moyenne et ~16× en médiane** par rapport à la régression directe.

**Adéquation pour pick-and-place :**

| Exigence | Valeur | Statut |
|----------|--------|--------|
| Précision requise (objets moyens) | ±5 mm | — |
| Joints proximaux (base→link2) | 3.9 mm | ✅ OK |
| Joints intermédiaires (link3–link4) | 8–9 mm | ⚠️ Limite |
| End-effector (link6) | 14.6 mm | ❌ Insuffisant |

→ Les joints proximaux sont dans la zone de précision pick-and-place. L'end-effector nécessite une amélioration (~3× pour atteindre ±5mm).

#### Instabilité ResNet-H — Détail

```
Epoch 1 : val_loss = 0.000305 ← seul bon résultat
Epoch 2 : val_loss = 0.152    ← BN stats corrompues
Epoch 3 : val_loss = 12.19    ← explosion
Epoch 5 : val_loss = 0.0017   ← récupération temporaire
Epoch 7 : val_loss = 95.97    ← explosion massive
→ Tué à epoch 10 (inutilisable)
```

**Cause racine :** `BatchNorm2d(momentum=0.1)` avec batch_size=16 produit des running mean/var non-représentatives. L'analyse per-batch montre que les 250 batchs de validation sont affectés dans les mauvaises époques (loss minimale par batch = 0.53 à epoch 7).

### Transfert Sim-to-Real

| Métrique | Synthétique | Réel |
|----------|-------------|------|
| Taux de détection | 97% | ~26% |
| Pics belief maps | 0.5–1.0 | 0.02–0.25 |
| Amélioration avec augmentation | — | 22.9% → 25.7% (marginal) |

**Le domain gap reste le problème principal.** Les belief maps ont des pics 10× plus faibles sur images réelles, ce qui empêche la détection fiable des keypoints. L'augmentation agressive (HueSaturation, GaussianBlur, MotionBlur, CLAHE, CoarseDropout, ImageCompression) n'apporte qu'une amélioration marginale.

**Pistes pour réduire le domain gap :**
1. **Domain Randomization avancée** dans Gazebo (textures aléatoires, éclairage variable, backgrounds photo-réalistes)
2. **Fine-tuning sur données réelles annotées** (auto-labeling via FK + caméra calibrée)
3. **Self-supervised labeling** : utiliser les angles lus du robot + FK + intrinsèques caméra pour générer les keypoints GT sur images réelles
4. **Style transfer** (CycleGAN) entre images Gazebo et réelles

### Fichiers du module DREAM

| Fichier | Rôle |
|---|---|
| `training/dream/__init__.py` | Init module |
| `training/dream/mycobot_fk.py` | Forward kinematics + projection caméra (7 keypoints) |
| `training/dream/convert_to_ndds.py` | Conversion dataset → format NDDS (DREAM) |
| `training/dream/train_dream.py` | Wrapper d'entraînement DREAM |
| `training/dream/train_dream_augmented.py` | Entraînement avec augmentation agressive |
| `training/dream/evaluate_dream.py` | Évaluation complète avec métriques par keypoint |
| `training/dream/infer_dream.py` | Inférence : détection keypoints + résolution PnP |
| `training/dream/visualize_ndds.py` | Visualisation des annotations keypoints sur images |
| `training/dream/manip_configs/mycobot320.yaml` | Configuration des 7 keypoints (noms, frames URDF) |

### Checkpoints DREAM (dans .gitignore)

| Répertoire | Contenu |
|------------|---------|
| `checkpoints_dream/resnet_synthetic_e25/` | ResNet-H (tué à epoch 10, inutilisable) |
| `checkpoints_dream/vgg_synthetic_e25/` | VGG base (25 époques, meilleur E8, val=0.000438) |
| `checkpoints_dream/vgg_augmented_e25/` | VGG augmenté (25 époques, meilleur E22, val=0.000667) |
| `checkpoints_dream/vgg_weighted_50k_e50/` | VGG 50K synth (50 époques, 98.3% det synth, 13.2% réel) |
| `checkpoints_dream/vgg_mixed_real_synth/` | VGG mixte 18K (10K réel×5 + 8K synth, terminé — à évaluer) |

### Tentatives de fine-tuning (❌ ÉCHOUÉES)

Deux tentatives de fine-tuning manuel avec `finetune_real.py` ont **totalement échoué** :

| Tentative | Val Loss | Détection | Cause |
|-----------|----------|-----------|-------|
| v1 (σ=4, single-stage) | 0.000651 | 0% | σ=4 au lieu de 2, pics belief maps écrasés |
| v2 (σ=2, MSE) | 0.000077 | 0% | Belief maps effondrées (max 0.000000) — MSE sur grille quasi-vide → all-zeros |

**Leçon :** Ne pas remplacer le pipeline natif DREAM. `train_network.py` gère correctement la génération des belief maps en interne.

### Dataset mixte réel+synthétique

Pour améliorer le transfert sim-to-real, un dataset mixte a été créé :

```
/tmp/dream_data/mixed_real_synth/   (18K frames, symlinks)
├── 2K images réelles × 5 copies (oversampling) = 10K
├── 8K images synthétiques (sous-ensemble aléatoire de 50K)
├── _camera_settings.json (fx=610, depuis real_cam0)
```

Entraînement natif DREAM sur ce dataset :
- Epoch 1 : train=0.000608, val=0.000474 (prometteur)

### Données NDDS converties

| Répertoire | Contenu |
|------------|---------|
| `/tmp/dream_data/synthetic/` | 20K frames (5000 poses × 4 caméras Gazebo) |
| `/tmp/dream_data/real_cam0/` | 2000 frames (cam0 → front mapping) |

### Dépendances DREAM

```bash
# Installation DREAM (NVlabs)
git clone https://github.com/NVlabs/DREAM.git /tmp/DREAM
cd /tmp/DREAM && pip install -e . -r requirements.txt

# Environnement Conda
# Python 3.13.5, PyTorch 2.6.0+cu124, NVIDIA RTX 4000 Ada 20GB
```

---

## 🧪 Tests validés

| Test | Date | Statut |
|------|------|--------|
| Connexion TCP Tour → Pi | 26/03/2026 | ✅ OK |
| Commandes robot (ping, status, angles, etc.) | 26/03/2026 | ✅ OK |
| RViz visualisation standalone | 26/03/2026 | ✅ OK |
| RViz synchronisé avec robot | 26/03/2026 | ⚠️ Instable (freq trop haute) |
| Simulation Gazebo 4 caméras | 31/03/2026 | ✅ OK |
| Collecte données synthétiques (5000 poses) | 31/03/2026 | ✅ OK |
| Training multi-view synthétique → 12.97° | 31/03/2026 | ✅ OK |
| Camera server Pi (cam0 + cam3 TCP:5006) | 01/04/2026 | ✅ OK |
| Capture réelle 2000 poses (0 collisions) | 02/04/2026 | ✅ OK |
| FK safety capture (table + câbles) | 02/04/2026 | ✅ OK |
| Training données réelles (régression directe) | 02/04/2026 | ❌ Bloqué à baseline |
| Diagnostic corrélation pose/pixel | 02/04/2026 | ✅ Cause identifiée |
| DREAM — Conversion NDDS (20K frames, 0 skip) | 03/04/2026 | ✅ OK |
| DREAM — FK + projection caméra (4 caméras) | 03/04/2026 | ✅ OK |
| DREAM — Training ResNet-H (25 époques) | 03/04/2026 | ❌ BN instable, tué epoch 10 |
| DREAM — Training VGG-base (25 époques) | 03/04/2026 | ✅ Stable, val=0.000438 |
| DREAM — Training VGG-aug (25 époques) | 03/04/2026 | ✅ Stable, val=0.000667 |
| DREAM — Évaluation synthétique (97% détection) | 03/04/2026 | ✅ OK |
| DREAM — Test sim-to-real (~26% détection) | 03/04/2026 | ⚠️ Domain gap trop large |
| Git commit DREAM (`0e452ece`) | 03/04/2026 | ✅ OK |
| DREAM — VGG 50K synth (98.3% det synth) | 15/04/2026 | ✅ OK |
| DREAM — Eval 50K sur réel (13.2% det) | 15/04/2026 | ⚠️ Domain gap |
| DREAM — Fine-tune v1 (σ=4, 0% det) | 15/04/2026 | ❌ Modèle mort |
| DREAM — Fine-tune v2 (σ=2, 0% det) | 16/04/2026 | ❌ Belief maps effondrées |
| DREAM — Dataset mixte 18K créé | 16/04/2026 | ✅ OK |
| DREAM — Training mixte natif (50 époques) | 16/04/2026 | ✅ Terminé (à évaluer sur réel) |
| Téléopération main → bras réel (premier test physique) | 22/04/2026 | ✅ Validé (gains 1.2/1.2/1.6/0.25) |
| Pick-and-place mono-objet end-to-end (Gazebo) | 22/04/2026 | ✅ Cycle complet ~33 s |
| Pick-and-place sorting 4 couleurs (HSV + IK) | 23/04/2026 | ✅ 4/4 couleurs sortées ~95 s |
| URDF caméras reshapées (corps + objectif + LED) | 23/04/2026 | ✅ Plus de confusion HSV |
| Doc validation sim-only `TELEOP_SIM_TESTING.md` | 23/04/2026 | ✅ Crée + référencée |
| Install `pandas` dans `venv_dream` | 28/04/2026 | ✅ |
| DREAM eval (a) strict réel — 47.3 % det | 28/04/2026 | ✅ Baseline reproduit |
| DREAM eval (b) strict synth val — 91.9 % det | 28/04/2026 | ✅ Régression -6.4 pts contrôlée |
| DREAM eval (c) relaxed réel — 48.0 % det | 28/04/2026 | ❌ Médianes explosées (peaks low-conf = bruit) |
| Diagnostic distal keypoints (link4-6) bottleneck | 28/04/2026 | ✅ Cf. CHANGELOG 1.12.0 + SESSION_RESUME |
| Domain randomization v2/v3 (worlds) | 15/04/2026 | ✅ OK |
| Documentation ARCHITECTURE.md rewrite | 16/04/2026 | ✅ OK |
| Gripper adaptatif intégré (pro_adaptive_gripper) | 15/04/2026 | ✅ OK |
| Anti-collision FK collecteur synthétique | 15/04/2026 | ✅ OK |
| Scripts train_pipeline.sh + monitor_collection.sh | 16/04/2026 | ✅ OK |
| merge_and_convert.py | 16/04/2026 | ✅ OK |

---

## 📚 Commandes utiles

```bash
# Compiler les packages
cd ~/ros_jazzy/src/mycobot_R6A
colcon build --symlink-install

# Compiler un seul package
colcon build --packages-select mycobot_gateway --symlink-install

# Nettoyer et recompiler
rm -rf build/mycobot_gateway install/mycobot_gateway
colcon build --packages-select mycobot_gateway --symlink-install

# Lister les exécutables d'un package
ros2 pkg executables mycobot_gateway

# Voir les topics actifs
ros2 topic list

# Écouter un topic
ros2 topic echo /from_robot

# Tuer tous les bridges
pkill -f bridge_tour
pkill -f bridge_pi
```

---

## 👥 Contributeurs

- **Développement initial (bridge ROS2):** 26 mars 2026
- **Simulation Gazebo + données synthétiques:** 31 mars 2026
- **Pipeline IA + capture réelle + diagnostic:** 1-2 avril 2026
- **DREAM keypoint pose estimation:** 3 avril 2026

---

*Ce document sert de point de départ pour les prochaines sessions de développement.*
