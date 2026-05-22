# 🏗️ Architecture du Projet MyCobot 320 Pi — R6A

> Dernière mise à jour : 21 avril 2026

## Vue d'ensemble

Le projet **MyCobot 320 Pi R6A** est une plateforme complète de robotique
intelligente combinant :

1. **Contrôle distribué** PC ↔ Raspberry Pi via TCP/ROS2
2. **Simulation Gazebo** avec domain randomization
3. **Estimation de pose par vision** (DREAM keypoint + PnP)
4. **Pipeline Sim-to-Real** : données synthétiques → fine-tuning réel

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                          PC TOUR  (10.10.0.115)                             │
│              Ubuntu 24.04 / ROS2 Jazzy / Python 3.12                        │
│              GPU: NVIDIA RTX 4000 Ada 20 GB / CUDA 12.4                     │
│                                                                             │
│   ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────┐ ┌──────────────────┐  │
│   │SimpleGUI │ │ Slider   │ │ Teleop   │ │Commander │ │  Gazebo Harmonic │  │
│   │ Tkinter  │ │ Control  │ │ Keyboard │ │  CLI     │ │  4 caméras +     │  │
│   └────┬─────┘ └────┬─────┘ └────┬─────┘ └────┬─────┘ │  randomization   │  │
│        └─────────────┴────────────┴─────────────┘      └────────┬─────────┘  │
│                      │                                          │           │
│               /to_robot (JSON)                          /synth_camera/*     │
│                      ▼                                          ▼           │
│             ┌──────────────┐                       ┌────────────────────┐   │
│             │ bridge_tour  │                       │ dream_inference    │   │
│             │  TCP:5005    │                       │ (DREAM VGG + PnP)  │   │
│             └──────┬───────┘                       └────────┬───────────┘   │
│                    │                                        ▼              │
│                    │                               ┌────────────────────┐   │
│                    │                               │  pick_and_place    │   │
│                    │                               │  (state machine)   │   │
│                    │                               └────────────────────┘   │
├────────────────────┼───────────────────────────────────────────────────────┤
│              RÉSEAU ETHERNET 10.10.0.x                                     │
├────────────────────┼───────────────────────────────────────────────────────┤
│                    ▼                                                       │
│    ┌──────────────────┐  ┌──────────────────┐                              │
│    │ bridge_pi_simple │  │ pi_camera_server │                              │
│    │   TCP:5005       │  │   TCP:5006       │                              │
│    └────────┬─────────┘  └────────┬─────────┘                              │
│             ▼                     ▼                                        │
│    ┌──────────────────┐  ┌──────────────────┐                              │
│    │   pymycobot      │  │ Arducam USB ×2   │                              │
│    │  /dev/ttyAMA0    │  │  cam0 + cam3     │                              │
│    └────────┬─────────┘  └──────────────────┘                              │
│             ▼                                                              │
│    ┌──────────────────┐          RASPBERRY PI  (10.10.0.225)               │
│    │  MyCobot 320 Pi  │          Ubuntu 20.04 / ROS2 Galactic              │
│    └──────────────────┘                                                    │
└────────────────────────────────────────────────────────────────────────────┘
```

---

## Packages ROS2

### `mycobot_gateway/` — Contrôle & Intelligence

| Nœud | Fichier | Rôle |
|------|---------|------|
| `bridge_tour` | `bridge_tour.py` | Client TCP, pont ROS2 ↔ Pi |
| `simple_gui` | `simple_gui.py` | GUI Tkinter (angles, coords, gripper, LED) |
| `slider_control` | `slider_control.py` | Joint State Publisher + RViz |
| `teleop_keyboard` | `teleop_keyboard.py` | Contrôle clavier WASD+ZX |
| `robot_commander` | `robot_commander.py` | CLI interactif |
| `joint_sync` | `joint_sync.py` | Synchro états articulaires |
| `marker_follower` | `marker_follower.py` | Suivi ArUco (legacy) |
| `dream_inference` | `dream_inference_node.py` | Inférence DREAM + PnP pose |
| `pick_and_place` | `pick_and_place_node.py` | Orchestre pick-and-place mono-objet via IK |
| `color_object_detector` | `color_object_detector.py` | Segmentation HSV + back-projection (top camera) |
| `sorting_orchestrator` | `sorting_orchestrator.py` | Pick-and-place multi-objets par couleur (boucle sur détections) |
| `synth_data_collector` | `synthetic_data_collector_v2.py` | Collecte Gazebo + randomization |

**Launch files :**

| Launch | Description |
|--------|-------------|
| `bridge_only.launch.py` | Bridge TCP seul |
| `simple_gui.launch.py` | GUI + bridge |
| `slider_control.launch.py` | Sliders + RViz + bridge |
| `teleop_keyboard.launch.py` | Clavier + bridge |
| `commander.launch.py` | CLI + bridge |
| `rviz_sync.launch.py` | Sync robot réel → RViz |
| `marker_follow.launch.py` | Suivi marqueur ArUco |
| `pick_and_place.launch.py` | Cycle pick & place mono-objet (cube rouge → zone verte) |
| `pick_and_place_sorting.launch.py` | Pick & place multi-objets par couleur (4 objets → 4 bacs) |
| `synthetic_data.launch.py` | Collecte données sim (monde de base) |
| `synthetic_data_v2.launch.py` | Collecte v2 (monde randomisé) |
| `synthetic_data_v3.launch.py` | Collecte v3 (monde randomized_v2 — 6 lights, 12 objets) |

### `mycobot_description/` — URDF & Simulation

| Composant | Chemin | Rôle |
|-----------|--------|------|
| URDF | `urdf/320_pi/` | Modèle 3D du MyCobot 320 Pi |
| Gripper | `urdf/pro_adaptive_gripper/` | Gripper adaptatif (meshes STL) |
| Monde de base | `worlds/randomized.sdf` | Table + fond simple |
| Monde v2 | `worlds/randomized_v2.sdf` | 6 lumières, 12 objets clutter, 3 murs |
| Monde pick-and-place | `worlds/pick_and_place.sdf` | Table + cube cible rouge + zone verte |
| Monde sorting | `worlds/pick_and_place_sorting.sdf` | Table 1.0×0.6 m + 4 objets colorés (cube R/B, cylindre G, boîte Y) + 4 bacs colorés |
| Config RViz | `config/mycobot_320_pi.rviz` | Préréglage visualisation |
| Visuels caméra | `urdf/320_pi/mycobot_pro_320_pi_gazebo.urdf` | 4 caméras stylisées (corps + objectif + LED) — visuellement distinctes des objets à trier |

---

## Pipeline Vision — DREAM Keypoint Pose Estimation

### Architecture du réseau

```
Image RGB (640×480)
       │
       ▼
  Shrink-and-crop → (400×400)
       │
       ▼
  ┌─────────────────────────────────────────┐
  │          VGG-19 Backbone                │
  │   (ImageNet pretrained, frozen/slow LR) │
  └────────────────┬────────────────────────┘
                   │
                   ▼
  ┌─────────────────────────────────────────┐
  │     DOPE-style Decoder Head             │
  │   Conv layers → 7 belief maps (100×100) │
  └────────────────┬────────────────────────┘
                   │
                   ▼
  peaks_from_belief_maps()  →  7 keypoints (u,v)
                   │
                   ▼
            solvePnP()  →  Pose 6-DoF (R,t)
```

**7 keypoints détectés :**

| Index | Nom DREAM | Nom robot | Description |
|-------|-----------|-----------|-------------|
| 0 | `mycobot320_base` | Base | Base fixe |
| 1 | `mycobot320_link1` | Joint1 | Rotation base |
| 2 | `mycobot320_link2` | Joint2 | Épaule |
| 3 | `mycobot320_link3` | Joint3 | Coude |
| 4 | `mycobot320_link4` | Joint4 | Poignet 1 |
| 5 | `mycobot320_link5` | Joint5 | Poignet 2 |
| 6 | `mycobot320_link6` | EndEffector | Effecteur final |

### Paramètres réseau

| Paramètre | Valeur |
|-----------|--------|
| Architecture | VGG-19 (DOPE style) |
| Entrée | 400×400 RGB |
| Sortie | 7 × 100×100 belief maps |
| Normalisation | mean=[0.5, 0.5, 0.5], std=[0.5, 0.5, 0.5] |
| Loss | MSE sur belief maps (sigma=2) |
| Preprocessing | shrink-and-crop |
| Seuil détection | peak > 0.01 (après Gaussian filter σ=3) |

---

## Pipeline de données

### Données synthétiques (Gazebo)

```
Gazebo Harmonic 8.9.0
    │
    ├─ synthetic_data_collector_v2.py
    │   ├─ Pose aléatoire (FK → joint limits)
    │   ├─ Domain randomization :
    │   │   ├─ 6 lumières (direction, couleur, position)
    │   │   ├─ 12 objets clutter (couleur matériau)
    │   │   ├─ 5 surfaces (table, murs, sol)
    │   │   ├─ Color temperature shift (30%)
    │   │   └─ Vignetting effect (20%)
    │   └─ Sauvegarde format NDDS (JSON + PNG)
    │
    ▼
/tmp/dream_data/synthetic_50k/
    ├─ 000000.json          # GT keypoints (projected_location)
    ├─ 000000.rgb.png       # Image 640×480
    ├─ ...  (50 000 frames)
    └─ _camera_settings.json  # fx=554.4, 640×480
```

### Données réelles (Pi + Caméras USB)

```
capture_real.py
    │
    ├─ Connexion TCP → bridge_pi (5005) + pi_camera_server (5006)
    ├─ Envoi pose aléatoire → lecture angles → FK → keypoints 3D
    ├─ Projection 3D→2D via matrice caméra calibrée
    └─ Sauvegarde format NDDS identique
    
    ▼
/tmp/dream_data/real_cam0/
    ├─ 000000.json          # GT par FK + calibration extrinsèque
    ├─ 000000.rgb.png       # Image réelle 640×480
    ├─ ...  (2 000 frames)
    └─ _camera_settings.json  # fx=610.0, 640×480
```

### Format NDDS (par frame)

```json
{
  "camera_data": {
    "location_worldframe": [0, 0, 0]
  },
  "objects": [{
    "class": "mycobot320",
    "keypoints": [
      {
        "name": "mycobot320_base",
        "projected_location": [320.5, 412.3],
        "location": [0.0, 0.0, 0.0]
      },
      ...
    ]
  }]
}
```

### Outils de données

| Script | Rôle |
|--------|------|
| `training/dream/convert_to_ndds.py` | Convertit le format custom → NDDS |
| `training/dream/merge_and_convert.py` | Fusionne réel+synth avec oversampling → NDDS |
| `training/dream/visualize_ndds.py` | Vérifie visuellement les keypoints GT |
| `training/dream/mycobot_fk.py` | Forward Kinematics (DH parameters) |
| `training/dream/mycobot_ik.py` | Inverse Kinematics numérique (Jacobien) |
| `training/dream/evaluate_dream.py` | Évaluation modèle (+ filtre sentinel -999.99) |
| `scripts/train_pipeline.sh` | Pipeline automatisé : merge → NDDS → training |
| `scripts/monitor_collection.sh` | Suivi en temps réel de la collecte Gazebo |
| `training/dream/evaluate_grid.py` | Évaluation automatique sur tous les checkpoints `vgg_grid_*`, résultats sauvegardés dans `grid_search_results.txt` |
| `training/dream/train_dream_ultimate.py` | Training VGG weighted loss + cosine LR, weights link5=3.0 link6=5.0 (v1) |
| `training/dream/train_dream_ultimate_v2.py` | Training VGG weighted loss + cosine LR + strong augmentation sim-to-real, weights link5=1.5 link6=6.0 (v2) |
---

## Modèles entraînés — Historique

### Pipeline DREAM (actif)

| Modèle | Dataset | Epochs | Synth val | Real val | Notes |
|--------|---------|--------|-----------|----------|-------|
| `vgg_weighted_e50` | 20K synth | 50 | 97% det, 3.2px med | 10.9% det, 173px | Baseline |
| `vgg_weighted_50k_e50` | 50K synth | 50 | 98.3% det, 3.15px med | 13.2% det, 172px | +1M frames |
| `vgg_finetuned_real_e30` | Mixed (custom v1) | 30 | ❌ 0% | ❌ 0% | Bug: sigma=4, single-stage |
| `vgg_finetuned_real_v2` | Mixed (custom v2) | 30 | ❌ 0% | ❌ 0% | Bug: belief map collapse |
| **`vgg_mixed_real_synth`** | **18K mixed native** | **25** | **À évaluer** | **À évaluer** | DREAM natif, terminé |

### Pipeline legacy (ResNet)

| Modèle | Dataset | MAE joints |
|--------|---------|-----------|
| ResNet18 single-view | 1K synth | 22.6° |
| ResNet50 single-view | 5K synth | 16.5° |
| ResNet50 multi-view (4 cam) | 5K synth | **12.97°** |
| Toutes approches | 2K réel | ~31.7° |

### Leçons apprises — Fine-tuning DREAM

| Problème | Cause | Solution |
|----------|-------|----------|
| **sigma mismatch** | Script custom σ=4, DREAM natif σ=2 | Utiliser σ=2 ou pipeline natif |
| **Loss mono-stage** | Entraînement sur output[-1] uniquement | Loss multi-stage sur tous les stages |
| **Belief map collapse** | MSE sur grille 99.9% zéros → optimum = tout zéro | Pipeline natif DREAM gère correctement |
| **Sentinel -999.99** | DREAM renvoie -999.99 quand peak < seuil | Filtrer coords < -900 dans l'évaluation |

---

## Protocole de communication TCP

### Tour → Pi (`/to_robot`, JSON)

```json
{"action": "send_coords", "coords": [200, 0, 250, 180, 0, 0], "speed": 40, "mode": 1}
{"action": "send_angles", "angles": [0, 8, -127, 40, 0, 0], "speed": 40}
{"action": "gripper_open"}
{"action": "go_home"}
{"action": "emergency_stop"}
```

### Pi → Tour (`/from_robot`, texte)

```
OK: send_coords [200.0, 0.0, 250.0]
ANGLES: [0.0, 8.0, -127.0, 40.0, 0.0, 0.0]
🚨 EMERGENCY STOP EXECUTED
```

---

## Sécurité

| Mécanisme | Détail |
|-----------|--------|
| Limites cartésiennes | X: 130–350, Y: ±200, Z: 100–400 mm |
| Limites articulaires | Respecte `joint_limits` URDF |
| Cooldown commandes | 200 ms min entre envois |
| Emergency stop | Relâche tous les servos |
| Auto-reconnexion | Bridge TCP reconnecte si perte |
| Collision avoidance | Vérification FK avant envoi |

---

## Environnements Python

| Environnement | Usage | Python | Activation |
|---------------|-------|--------|------------|
| Système ROS2 | `colcon build`, launch | 3.12 | `source /opt/ros/jazzy/setup.bash` |
| `venv_dream` | DREAM training/inférence | 3.12 | `source ~/ros_jazzy/venv_dream/bin/activate` |
| Conda (base) | Legacy ML (`train.py`) | 3.13 | `conda activate base` |

> ⚠️ Toujours `conda deactivate` avant toute opération ROS2.

---

## Arborescence clé

```
mycobot_R6A/
├── mycobot_gateway/                 # 📦 ROS2 : contrôle + vision
│   ├── mycobot_gateway/
│   │   ├── bridge_tour.py           # TCP client → Pi
│   │   ├── dream_inference_node.py  # Inférence DREAM temps réel
│   │   ├── pick_and_place_node.py   # State machine pick & place
│   │   ├── synthetic_data_collector_v2.py  # Collecte + randomization
│   │   └── vision/                  # Modules ArUco, etc.
│   ├── launch/                      # 12 fichiers launch
│   └── scripts/                     # Scripts Pi (bridge, caméra)
│
├── mycobot_description/             # 📦 ROS2 : modèle 3D
│   ├── urdf/320_pi/                 # URDF + meshes STL
│   └── worlds/                      # Mondes Gazebo SDF
│       ├── randomized.sdf           # Monde simple
│       └── randomized_v2.sdf        # 6 lights + 12 clutter
│
├── training/                        # 🧠 Pipeline ML
│   ├── train.py                     # Legacy ResNet multi-view
│   ├── model.py                     # PoseResNet / MultiViewPoseResNet
│   ├── dataset.py                   # Datasets single/multi-view
│   ├── capture_real.py              # Capture données réelles
│   └── dream/                       # 🧠 DREAM pipeline (actif)
│       ├── train_dream_weighted.py  # Training pondéré par keypoint
│       ├── evaluate_dream.py        # Évaluation + filtre sentinel
│       ├── finetune_real.py         # Fine-tuning custom (expérimental)
│       ├── infer_dream.py           # Inférence single-image
│       ├── mycobot_fk.py            # Forward Kinematics DH
│       ├── mycobot_ik.py            # Inverse Kinematics Jacobien
│       ├── convert_to_ndds.py       # Conversion → NDDS
│       └── checkpoints_dream/       # Modèles entraînés
│
├── datasets/                        # 📊 Données (Git LFS)
│   ├── synthetic_dataset/           # 20K images Gazebo
│   └── real_dataset/                # 4K images Pi
│
├── docs/                            # 📖 Documentation
└── scripts/                         # 🔧 Utilitaires bash
```
