# 🤖 MyCobot 320 Pi - Résumé de Développement

> **Date de dernière mise à jour:** 3 avril 2026  
> **Version:** 1.6.0  
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
│                     RASPBERRY PI (10.10.0.225)                             │
└────────────────────────────────────────────────────────────────────────────┘
```

### Principe
- **Tour (PC):** Calculs complexes (vision, IA, path planning, visualisation RViz/Gazebo)
- **Raspberry Pi:** Exécution simple des commandes robot (contrôle moteurs via pymycobot)

---

## 📁 Structure du Workspace Tour

```
~/ros_jazzy/src/mycobot_R6A/
├── DEVELOPMENT_SUMMARY.md          # Ce fichier
├── INDEX.md                        # Index de documentation
│
├── mycobot_description/            # Package URDF et visualisation
│   ├── package.xml
│   ├── CMakeLists.txt
│   ├── urdf/
│   │   └── 320_pi/
│   │       ├── mycobot_pro_320_pi.urdf
│   │       ├── base.dae
│   │       ├── link1.dae ... link6.dae
│   │       └── new_mycobot_pro_320_pi_moveit.urdf
│   ├── config/
│   │   └── mycobot_320_pi.rviz
│   └── launch/
│       └── display.launch.py
│
├── mycobot_gateway/                # Package communication et contrôle
│   ├── package.xml
│   ├── setup.py
│   ├── setup.cfg
│   ├── mycobot_gateway/
│   │   ├── __init__.py
│   │   ├── bridge_tour.py          # Client TCP vers la Pi
│   │   ├── robot_commander.py      # Interface commande interactive
│   │   ├── joint_sync.py           # Synchronisation RViz ↔ Robot
│   │   └── vision/
│   │       ├── __init__.py
│   │       ├── marker_detector.py  # Détection ArUco (future)
│   │       └── camera_publisher.py # Publication caméra (future)
│   ├── scripts/
│   │   ├── bridge_tour
│   │   ├── robot_commander
│   │   ├── joint_sync
│   │   ├── marker_detector
│   │   ├── camera_publisher
│   │   ├── bridge_pi_simple.py     # Script pour la Pi (standalone)
│   │   ├── bridge_pi_standalone.py
│   │   └── bridge_pi_vision.py
│   ├── launch/
│   │   ├── rviz_sync.launch.py     # Launch complet avec sync
│   │   ├── bridge_only.launch.py
│   │   ├── marker_follow.launch.py
│   │   └── commander.launch.py
│   └── resource/
│       └── mycobot_gateway
│
├── build/                          # Dossier de build (généré)
├── install/                        # Dossier d'installation (généré)
└── log/                            # Logs de build (généré)
```

---

## 🔧 Packages développés

### 1. `mycobot_description`
**Type:** ament_cmake  
**But:** Fournir le modèle URDF du robot pour la visualisation

| Fichier | Description |
|---------|-------------|
| `urdf/320_pi/*.urdf` | Modèle URDF du MyCobot 320 Pi |
| `urdf/320_pi/*.dae` | Meshes 3D (Collada) du robot |
| `config/mycobot_320_pi.rviz` | Configuration RViz prête à l'emploi |
| `launch/display.launch.py` | Lance RViz avec le modèle robot |

### 2. `mycobot_gateway`
**Type:** ament_python  
**But:** Bridge réseau et outils de contrôle

| Exécutable | Description |
|------------|-------------|
| `bridge_tour` | Client TCP qui se connecte à la Pi |
| `joint_sync` | Synchronise les joints RViz avec le robot réel |
| `robot_commander` | Interface interactive pour envoyer des commandes |
| `marker_detector` | Détection de marqueurs ArUco (future) |
| `camera_publisher` | Publication d'images caméra (future) |

---

## 🌐 Communication TCP - Protocole

### Configuration réseau
- **IP Raspberry Pi:** `10.10.0.218`
- **Port TCP:** `5005`
- **Format:** Texte avec terminaison `\n`

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
# Sur la Tour - IMPORTANT: Désactiver Conda avant ROS2
conda deactivate
source /opt/ros/jazzy/setup.bash
source ~/ros_jazzy/src/mycobot_R6A/install/setup.bash
```

### 1. Visualisation standalone (sans robot)
```bash
ros2 launch mycobot_description display.launch.py
```
→ Ouvre RViz avec le modèle robot et une GUI pour bouger les joints manuellement.

### 2. Communication avec le robot réel

**Terminal 1 - Sur la Raspberry Pi:**
```bash
source /opt/ros/galactic/setup.bash
cd ~/colcon_ws/src/mycobot_ros2/mycobot_320/mycobot_320pi/mycobot_gateway
python3 bridge_pi_debug.py
```

**Terminal 2 - Sur la Tour:**
```bash
conda deactivate
source /opt/ros/jazzy/setup.bash
source ~/ros_jazzy/src/mycobot_R6A/install/setup.bash
ros2 run mycobot_gateway bridge_tour
```

**Terminal 3 - Envoyer des commandes:**
```bash
# Ping
ros2 topic pub --once /to_robot std_msgs/msg/String "data: 'ping'"

# Status
ros2 topic pub --once /to_robot std_msgs/msg/String "data: 'status'"

# Lire angles
ros2 topic pub --once /to_robot std_msgs/msg/String "data: 'get_angles'"

# Aller à home
ros2 topic pub --once /to_robot std_msgs/msg/String "data: 'go_home:20'"

# Changer LED
ros2 topic pub --once /to_robot std_msgs/msg/String "data: 'set_led:0,255,0'"

# Bouger tous les joints
ros2 topic pub --once /to_robot std_msgs/msg/String "data: 'set_angles:0,0,0,0,0,90:20'"
```

### 3. Visualisation synchronisée avec robot réel
```bash
# Sur la Pi: lancer bridge_pi_debug.py (comme ci-dessus)

# Sur la Tour:
ros2 launch mycobot_gateway rviz_sync.launch.py sync_rate:=1.0
```
→ RViz affiche la position réelle du robot, mise à jour automatiquement.

---

## ⚠️ Problèmes connus et solutions

### 1. Erreur `ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'`
**Cause:** Conda utilise Python 3.13, mais ROS2 Jazzy nécessite Python 3.12  
**Solution:** Toujours exécuter `conda deactivate` avant les commandes ROS2

### 2. Connexion TCP instable (déconnexions fréquentes)
**Cause:** Plusieurs instances de `bridge_tour` qui tournent en parallèle  
**Solution:** 
```bash
pkill -f bridge_tour
# Puis relancer UNE SEULE instance
```

### 3. Fréquence de sync trop élevée
**Cause:** `joint_sync` interroge le robot trop souvent (5 Hz par défaut)  
**Solution:** Réduire la fréquence
```bash
ros2 launch mycobot_gateway rviz_sync.launch.py sync_rate:=1.0
```

---

## 📝 Fichiers sur la Raspberry Pi

Le bridge côté Pi se trouve à:
```
~/colcon_ws/src/mycobot_ros2/mycobot_320/mycobot_320pi/mycobot_gateway/bridge_pi_debug.py
```

Ce fichier:
- Crée un serveur TCP sur le port 5005
- Se connecte au robot via `/dev/ttyAMA0` (pymycobot)
- Exécute les commandes reçues et renvoie les réponses

---

## 🔮 Développements futurs prévus

### Court terme (PRIORITAIRE)
- [ ] **Réduire le domain gap sim-to-real** : domain randomization avancée ou self-supervised labeling sur images réelles
- [ ] **Pick-and-place en simulation** : intégrer DREAM inference dans un nœud ROS2 + MoveIt2 dans Gazebo
- [ ] Fine-tuning VGG-aug sur données réelles annotées automatiquement (FK + caméra calibrée)

### Moyen terme
- [ ] **Bench test robot réel** : confronter les résultats simulation vs robot réel pour pick-and-place
- [ ] Nœud ROS2 d'inférence DREAM temps réel (caméra → keypoints → PnP → pose)
- [ ] Intégration de la pose estimée dans la boucle de contrôle ROS2
- [ ] Path planning avec MoveIt2

### Long terme
- [ ] Multi-robot coordination
- [ ] Interface GUI améliorée (prédiction keypoints en temps réel)

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
