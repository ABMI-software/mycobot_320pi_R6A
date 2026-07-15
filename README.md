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
┌──────────────────────────────────────────────────────────────────────────────────────┐
│                              PC TOUR (10.10.0.115)                                   │
│           ROS2 Jazzy / Ubuntu 24.04 / Python 3.12 (system)                           │
│           Conda env hand-teleop : Python 3.10 (Wilor + Astra)                        │
│           Conda env venv_dream  : Python 3.12 / PyTorch 2.6 + CUDA 12.4              │
│           GPU : NVIDIA RTX 4000 Ada (20 GB VRAM)                                     │
├──────────────────────────────────────────────────────────────────────────────────────┤
│ ┌─────────────────── CONTRÔLES INTERACTIFS ────────────────────┐                     │
│ │ simple_gui · slider_control · teleop_keyboard · commander    │                     │
│ │ (Tkinter / RViz / clavier / CLI)                             │                     │
│ └──────────────────────────────┬───────────────────────────────┘                     │
│                                │                                                     │
│ ┌──────────── TÉLÉOPÉRATION PAR LA MAIN (conda hand-teleop) ───────────────┐        │
│ │  Astra S RGB ──► Wilor (hand 6-DoF) ──► mapping rel + filtres R5A        │        │
│ │  (oni_grabber, shm)    (PyTorch)        (Kalman + EMA + slew 1°/frame)   │        │
│ │                                  │                                       │        │
│ │                                  ▼  rosbridge :9090                      │        │
│ │  /mycobot_controller/joint_trajectory  +  /teleop/* (gains, recal, KPI)  │        │
│ └─────────────────┬────────────────────────────────────────┬───────────────┘        │
│                   │                                        │                         │
│           target=sim │                              target=real │                    │
│                   ▼                                        ▼                         │
│        ┌──────────────────┐                   ┌──────────────────────────┐          │
│        │ Gazebo Harmonic  │                   │ trajectory_to_robot_     │          │
│        │ + JTC + 4 caméras│                   │  bridge (rad → deg JSON, │          │
│        │ + gripper 4 DOF  │                   │  15 Hz, deadband 1°)     │          │
│        │ /joint_states    │                   └──────────────┬───────────┘          │
│        └──────────────────┘                                  │                       │
│                                                              ▼                       │
│ ┌──────── PIPELINE VISION DREAM ────────┐         ┌──────────────────┐              │
│ │ training/dream/  ·  venv_dream         │         │   bridge_tour    │              │
│ │ VGG-19 → belief maps → 7 keypoints     │         │  (JSON /to_robot)│              │
│ │ → PnP → pose 6-DoF                     │         └────────┬─────────┘              │
│ │ checkpoints_dream/vgg_*                │                  │                        │
│ └────────────────────────────────────────┘                  │                        │
│                                                             │                        │
│ ┌─────── PICK-AND-PLACE GAZEBO ─────────┐                   │                        │
│ │ pick_and_place_node  (mono)            │                  │                        │
│ │ sorting_orchestrator (4 couleurs)      │                  │                        │
│ │   ←  color_object_detector (HSV)       │                  │                        │
│ └────────────────────────────────────────┘                  │                        │
├─────────────────────────────────────────────────────────────┼────────────────────────┤
│                          RÉSEAU ETHERNET (10.10.0.x)        │                        │
├─────────────────────────────────────────────────────────────┼────────────────────────┤
│                                                             ▼                        │
│              ┌──────────────────┐              ┌─────────────────────┐              │
│              │ pi_camera_server │              │  bridge_pi_simple   │              │
│              │   TCP:5006       │              │   TCP:5005          │              │
│              └────────┬─────────┘              └──────────┬──────────┘              │
│                       │                                   ▼                          │
│              ┌────────▼────────┐                ┌─────────────────┐                  │
│              │ Arducam USB ×2  │                │    pymycobot    │                  │
│              │  cam0 + cam3    │                │  /dev/ttyAMA0   │                  │
│              └─────────────────┘                └────────┬────────┘                  │
│                                                          ▼                           │
│                                                ┌─────────────────┐                   │
│                                                │  MyCobot 320 Pi │                   │
│                                                └─────────────────┘                   │
│                          RASPBERRY PI (10.10.0.221)                                  │
└──────────────────────────────────────────────────────────────────────────────────────┘
```

> **Trois chemins de commande convergent vers le robot** :
> (1) GUI/CLI/clavier classique → `bridge_tour` → Pi ;
> (2) téléop main (Astra→Wilor) → rosbridge → JTC (sim) **ou** trajectory_to_robot_bridge → bridge_tour → Pi (réel) ;
> (3) pipeline vision DREAM (synthétique + mixte) pour l'estimation de pose qui alimentera le futur asservissement par caméra.

---

## 📦 Packages & Composants

| Composant | Description |
|-----------|-------------|
| `mycobot_gateway/` | Bridge TCP, GUI, contrôles, vision DREAM, pick-and-place mono + sorting (ROS2 package) |
| `mycobot_description/` | URDF avec gripper adaptatif et 4 caméras stylisées + worlds Gazebo (randomized, pick-and-place mono, pick-and-place sorting) |
| `training/` | Pipeline ML : DREAM keypoint detection (VGG-19, mixed real+synth), legacy ResNet regression |
| `teleop/` | Téléopération par la main : Wilor + Astra + dashboard ABMI + performance analyzer (env conda `hand-teleop`) |
| `datasets/` | Données synthétiques (Gazebo, 50K) et réelles (Pi, 4K) — via **Git LFS** |
| `scripts/` | Scripts utilitaires : preflight robot réel, train pipeline, monitoring, diagnostics |
| `docs/` | Documentation technique complète (architecture, téléop, real-robot, dashboard, tuning, sim testing) |

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
ssh er@10.10.0.221
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

# Pick-and-place en simulation (Gazebo)
ros2 launch mycobot_gateway pick_and_place.launch.py             # mono-objet (cube rouge → zone verte)
ros2 launch mycobot_gateway pick_and_place_sorting.launch.py     # multi-objet par couleur (4 objets → 4 bacs)
```

### Multi-object color sorting (`feature/pick-and-place-sorting`)

Branche dédiée au tri par couleur en Gazebo Harmonic. Le monde
[`mycobot_description/worlds/pick_and_place_sorting.sdf`](mycobot_description/worlds/pick_and_place_sorting.sdf)
contient 4 objets de couleurs et formes différentes (cube rouge, cube bleu,
cylindre vert, boîte jaune) côté +X, et 4 bacs colorés à parois côté −X.

| Composant | Rôle |
|-----------|------|
| `color_object_detector` | Segmentation HSV sur la caméra top-down + rétro-projection vers le repère robot (`/sorting/detections`) |
| `sorting_orchestrator` | Boucle sur les couleurs détectées, plan IK par objet, dépose dans le bac correspondant |
| `gz service set_pose` | Émulation du grasp : téléport du modèle sur l'EE pendant le portage |

```bash
# Lancement complet (détecteur HSV actif)
ros2 launch mycobot_gateway pick_and_place_sorting.launch.py

# Smoke-test sans perception (positions SDF connues)
ros2 launch mycobot_gateway pick_and_place_sorting.launch.py use_detector:=false

# Trier seulement un sous-ensemble
ros2 launch mycobot_gateway pick_and_place_sorting.launch.py process_order:=blue,green
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
  Phase 2 : DREAM Keypoint Detection  [ACTIF — écart sim-to-real comblé]
═══════════════════════════════════════════════════════════════
  Image → VGG-19 → 7 belief maps → keypoints 2D → PnP → pose

  VGG synth-only 20K : 97% det synth, 3.1px médiane ✅
  VGG synth-only 50K (v4, 2026-07-06) : 99.4% det synth, 2.61px ✅
  Fine-tune custom (σ=4 / σ=2)  : ❌ deux échecs documentés (abandonné, voir plus bas)
  VGG mix fine-tune (v4_mix_ft_e30, 2026-07-08) :
        50K synth + 6K réel ×5 oversampling → ~80K frames
        99.4% det synth (pas de régression) / 91.6% det réel ✅
        → écart sim-to-real fermé (27% → 91.6%)
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

| Modèle | Dataset entraînement | Eval synth | Eval réel | Notes |
|--------|----------------------|------------|-----------|-------|
| VGG base (synth-only) | 20K synth (5K poses × 4 vues) | 97% det · 3.1 px | ~26% det | val=0.000438, baseline DREAM |
| VGG augmenté (synth-only) | 20K synth + augmentation aggressive | 97% det · 3.1 px | 22.9 → 25.7% det | val=0.000667, gain marginal |
| VGG weighted (50K synth) | 50K synth + loss pondérée par keypoint | 98.3% det · 3.15 px | 13.2% det · 172 px | meilleure perf synth (ancien), gap sim-to-real majeur |
| VGG fine-tune v1 (σ=4) | 2K réel, single-stage | — | 0% det | ❌ pics belief écrasés, modèle mort |
| VGG fine-tune v2 (σ=2) | 2K réel, MSE direct | — | 0% det | ❌ belief maps effondrées (max ≈ 0) |
| **vgg_ultimate_v4_e50** (2026-07-06) | 50K synth v3 (intrinsèques corrigées, filtre capsule) | **99.4% det · 2.61 px** (13920/14000) | ≈27% det | Record synthétique — voir [`training/dream/VGG_ULTIMATE_V4_50K.md`](training/dream/VGG_ULTIMATE_V4_50K.md) |
| **vgg_ultimate_v4_mix_ft_e30** (2026-07-08) | 50K synth + 6K réel (real_3cam) ×5 oversampling → ~80K | 99.4% det (pas de régression) | **91.6% det · 2.91 px médiane** (9618/10500, 1500 frames jamais vues, 3 caméras) | **Écart sim-to-real fermé** — voir [`training/dream/README.md`](training/dream/README.md#fine-tune-mixte-réel-real_3cam-×5-oversampling--2026-07-03--2026-07-08) |

**Résultat final — validation synthétique 50k** (`vgg_ultimate_v4_e50`, split val 40000–50000, 2000 frames, 2026-07-06 · meilleure époque **49/50**) :

| Keypoint | Mean (px) | Median (px) | Std (px) | Max (px) | Det % |
|----------|-----------|-------------|----------|----------|-------|
| base | 3.47 | 3.38 | 0.17 | 3.89 | 100.0% |
| link1 | 3.20 | 3.17 | 0.21 | 3.64 | 100.0% |
| link2 | 3.20 | 3.18 | 0.21 | 3.65 | 100.0% |
| link3 | 1.88 | 1.61 | 2.40 | 51.88 | 99.8% |
| link4 | 2.11 | 1.69 | 4.55 | 97.69 | 100.0% |
| link5 | 2.11 | 1.59 | 5.15 | 127.65 | 99.2% |
| link6 | 2.30 | 1.77 | 5.35 | 112.54 | 97.0% |
| **OVERALL** | **2.61** | **2.78** | **3.46** | 127.65 | **99.4%** |

Précision par seuil : 37.1% <2px · 98.8% <5px · 99.5% <10px · 99.7% <20px · 99.9% <50px. Erreur moyenne par frame : 2.62 ± 2.33 px.

**Résultats finaux — évaluation réelle complète** (`vgg_ultimate_v4_mix_ft_e30`, 1500 frames, 3 caméras, 500 poses jamais vues, 2026-07-08) :

| Keypoint | Mean (px) | Median (px) | Std | Max | Det% |
|----------|-----------|-------------|-----|-----|------|
| base | 1.59 | 1.59 | 0.58 | 10.93 | 100.0% |
| link1 | 1.41 | 1.56 | 1.01 | 17.74 | 100.0% |
| link2 | 1.41 | 1.56 | 1.01 | 17.74 | 100.0% |
| link3 | 10.00 | 7.22 | 9.53 | 87.05 | 97.3% |
| link4 | 21.14 | 15.90 | 19.07 | 161.09 | 89.8% |
| link5 | 29.20 | 21.85 | 26.40 | 234.99 | 75.7% |
| link6 | 34.44 | 27.44 | 27.24 | 231.21 | 78.4% |
| **OVERALL** | **12.82** | **2.91** | **19.95** | 234.99 | **91.6%** (9618/10500) |

Précision par seuil : 35.1% <2px · 54.4% <5px · 64.8% <10px · 78.6% <20px · 94.4% <50px. Erreur moyenne par frame : 12.67 ± 9.28 px (meilleure frame 0.83 px, pire frame 100.25 px).

> Adéquation pick-and-place (cible ±5 mm) : les keypoints proximaux sont largement à niveau ; les distaux (link4/5/6) restent le point faible relatif mais ont le plus progressé pendant le fine-tune (+27–33% de MSE). Prochaine direction : pose estimation eye-to-hand + courbe d'écart par joint (angles DREAM vs encodeurs), voir `CHANGELOG.md` [1.13.0].

### Tests réalisés (DREAM)

| Test | Date | Résultat |
|------|------|----------|
| Conversion 20K frames → NDDS (0 skip) | 03/04/2026 | ✅ |
| FK + projection caméra (4 vues) | 03/04/2026 | ✅ |
| Training ResNet-H (25 epochs) | 03/04/2026 | ❌ BN instable, tué E10 |
| Training VGG-base (25 epochs) | 03/04/2026 | ✅ val=0.000438 |
| Training VGG-aug (25 epochs) | 03/04/2026 | ✅ val=0.000667 |
| Eval synthétique (20K) | 03/04/2026 | ✅ 97% det, 3.1 px |
| Eval sim-to-real (20K) | 03/04/2026 | ⚠️ ~26% det |
| Training VGG weighted 50K (50 epochs) | 15/04/2026 | ✅ 98.3% det synth |
| Eval VGG 50K sur réel | 15/04/2026 | ⚠️ 13.2% det, 172 px |
| Fine-tune custom v1 (σ=4) | 15/04/2026 | ❌ 0% det |
| Fine-tune custom v2 (σ=2) | 16/04/2026 | ❌ belief effondrées |
| Génération dataset synthétique 50k v3 (12.5K poses × 4 caméras, filtre capsule) | 02/07/2026 | ✅ couverture 100% du réel |
| Training `vgg_ultimate_v4_e50` (50 epochs, from scratch) | 02–06/07/2026 | ✅ **99.4% det synth**, 2.61px — record |
| Recalage extrinsèques caméras réelles (arducam/svpro/astra) | 03/07/2026 | ✅ débloque le fine-tune mixte |
| Fine-tune mixte `vgg_ultimate_v4_mix_ft_e30` (50K synth + 6K réel ×5, 13.8h) | 03–08/07/2026 | ✅ **91.6% det réel** (1500 frames jamais vues) — écart sim-to-real fermé |

### Pistes pour la suite

L'écart sim-to-real est fermé (27% → 91.6%). Direction actuelle (voir `CHANGELOG.md` [1.13.0]) :

1. **🔴 Pose estimation eye-to-hand + courbe d'écart par joint** — caméra fixe devant le bras, DREAM → angles articulaires (reprojection-min sur `mycobot_fk.py`/`mycobot_ik.py`) → comparaison angles estimés vs encodeurs réels. Outillage en place (`training/dream/estimate_angles_from_keypoints.py`, `plot_angle_error_curve.py`), calibration extrinsèque `T_base_camera` de la caméra fixe (astra) en cours.
2. **🟡 Fermer l'écart angulaire J1-J6** — le detection gap est fermé mais l'angle gap ne l'est pas (cible José : 0.5-0.9°, mesuré 10-20× ça). J6 structurellement non-observable (aucun keypoint ne dépend de sa rotation), J5 faiblement observable — nécessite une 2e caméra ou un keypoint supplémentaire en aval de J6. Voir `CLAUDE.md` § DREAM pose-estimation — validation status.
3. **🟢 Visual servoing** — une fois la courbe d'écart par joint validée, boucler la pose DREAM dans le contrôle pour le pick-and-place.
4. **🟡 Re-training Isaac Sim** (cf. [`POC direction`](CLAUDE.md) §1) — substitution de Gazebo par Isaac Sim + Isaac Lab pour rendu photoréaliste, piste de fond pour la suite du POC.

### Entraînement DREAM (recette actuelle — v4 + fine-tune mixte)

```bash
conda deactivate
source ~/ros_jazzy/venv_dream/bin/activate
cd training/dream

# From scratch sur le 50k synthétique (record 99.4%)
python train_dream_ultimate_v4.py \
  --data dream_data/synthetic_50k_ndds \
  --output output/checkpoints_dream/vgg_ultimate_v4_e50 \
  --epochs 50 --batch-size 8 --workers 8 --patience 5

# Fine-tune mixte depuis ce checkpoint (écart sim-to-real fermé à 91.6%)
python train_dream_ultimate_v4_mix.py \
  --data dream_data/<fusion_50k_synth_plus_real3cam_x5> \
  --pretrained output/checkpoints_dream/vgg_ultimate_v4_e50/best_network.pth \
  --epochs 30
```

Détails et méthodologie complète : [`training/dream/VGG_ULTIMATE_V4_50K.md`](training/dream/VGG_ULTIMATE_V4_50K.md), [`training/dream/FINETUNE_MIX_REAL3CAM_PLAN.md`](training/dream/FINETUNE_MIX_REAL3CAM_PLAN.md).

### Capture de données réelles

```bash
/home/genji/miniconda/bin/python3 training/capture_real.py \
  --output datasets/real_dataset \
  --num-samples 2000 \
  --pi-host 10.10.0.221 \
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
| **Hand Teleop** | `mycobot_teleop.launch.py` | **Téléop par caméra/main** (Wilor + Astra), `target={sim,real,both}` |
| **Pick-and-place mono** | `pick_and_place.launch.py` | Cube rouge → zone verte (Gazebo, vision DREAM optionnelle) |
| **Pick-and-place sorting** | `pick_and_place_sorting.launch.py` | 4 objets colorés → 4 bacs assortis (Gazebo, HSV + IK) |

---

## 🖐️ Téléopération par la main

Pipeline complet de pilotage du robot par la main de l'opérateur, adapté du R5A / LeRobot. **Orbbec Astra S** (RGB via OpenNI2 shared-memory) → **Wilor** (hand pose 6-DoF) → mapping relatif → filtres R5A → **rosbridge** → JTC Gazebo + `bridge_tour` vers le Pi réel.

**Outils livrés** ([teleop/](teleop/)) :

| Outil | Rôle |
|-------|------|
| `mycobot_teleop.py` | Script principal — caméra → joints |
| `teleop_dashboard.py` | GUI ABMI navy+pink, 3 onglets (🏠 Home · 📊 Analytics · 🎛️ Tuning) · KPI cards SIM/REAL · caméra opérateur intégrée · ActionButton dynamiques (tooltip + feedback + toast) · presets de gains (🐢 Safe / ⚙️ Nominal / ⚡ Reactive) · badge de mode auto |
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

# T4 — dashboard ABMI (Home / Analytics / Tuning)
python3 teleop_dashboard.py

# T5 — rapport de performance avant robot réel
python3 performance_analyzer.py --guided
```

**Documentation détaillée** :
- [`docs/TELEOPERATION.md`](docs/TELEOPERATION.md) — pipeline complet, filtres, historique
- [`docs/TELEOP_ARCHITECTURE_VIZ.md`](docs/TELEOP_ARCHITECTURE_VIZ.md) — **visuel détaillé** : de la détection main au mouvement du bras (types, unités, latences)
- [`docs/TELEOP_DASHBOARD.md`](docs/TELEOP_DASHBOARD.md) — manuel utilisateur du dashboard
- [`docs/TELEOP_TUNING.md`](docs/TELEOP_TUNING.md) — référence des paramètres + dépannage
- [`docs/TELEOP_SIM_TESTING.md`](docs/TELEOP_SIM_TESTING.md) — **valider la téléop en simulation seule** avant le robot réel : KPIs, scénarios guidés, seuils, use cases sim-only
- [`docs/REAL_ROBOT_TEST_PROCEDURE.md`](docs/REAL_ROBOT_TEST_PROCEDURE.md) — procédure de test sur robot physique

---

## 🎯 Pick-and-place (Gazebo)

Deux pipelines complets de pick-and-place en simulation, utilisés pour démontrer la chaîne perception → IK → contrôle moteur :

| Pipeline | Monde | Objets | Perception | Doc |
|----------|-------|--------|------------|-----|
| **Mono-objet** | `worlds/pick_and_place.sdf` | 1 cube rouge → zone verte | DREAM keypoints + PnP (optionnelle, fallback open-loop IK) | [pick_and_place_node.py](mycobot_gateway/mycobot_gateway/pick_and_place_node.py) |
| **Multi-couleur sorting** | `worlds/pick_and_place_sorting.sdf` | 4 objets dynamiques (cube R, cube B, cylindre G, boîte Y) → 4 bacs colorés à parois | HSV top-down + back-projection pinhole + IK numérique | [sorting_orchestrator.py](mycobot_gateway/mycobot_gateway/sorting_orchestrator.py) |

**Composants partagés** :
- IK numérique : `training/dream/mycobot_ik.py` (scipy L-BFGS-B + FK chain, multi-restart, warm-start, < 0.01 mm précision)
- Émulation grasp : appel au service Gazebo `/world/<world>/set_pose` pour téléporter l'objet sur l'EE pendant le portage et le déposer dans le bac à la couleur correspondante (le bras MyCobot 320 Pi physique n'a pas de gripper actuellement)

**Pipeline sorting (testé end-to-end le 23/04/2026)** :
```
   Top camera (1.2 m)              ┌──────────────────────────┐
        │                          │   sorting_orchestrator   │
        ▼                          │   ────────────────────   │
 ┌──────────────┐  /sorting/      │  for color in detections │
 │  HSV detector │ detections ──▶ │    1. plan IK            │
 │  + back-proj  │                 │    2. approach + descend │
 └──────────────┘                  │    3. GRASP (gz set_pose)│
                                    │    4. lift + carry       │
                                    │    5. place in bin       │
                                    │    6. RELEASE + retreat  │
                                    └──────────┬───────────────┘
                                               │
                                               ▼  /model/.../cmd_pos
                                       Gazebo joints (DART)
```

**Lancement** :
```bash
ros2 launch mycobot_gateway pick_and_place.launch.py             # mono-objet
ros2 launch mycobot_gateway pick_and_place_sorting.launch.py     # 4 couleurs
ros2 launch mycobot_gateway pick_and_place_sorting.launch.py use_detector:=false   # smoke-test
ros2 launch mycobot_gateway pick_and_place_sorting.launch.py process_order:=blue,green
```

**Résultats validation 23/04/2026** :
- ✅ 4/4 couleurs détectées par HSV (positions à ~1 mm près des positions SDF)
- ✅ IK résolue pour tous les waypoints (erreur < 0.01 mm sur l'EE)
- ✅ Cycle complet 4 objets en ~95 s (red → blue → green → yellow → home)
- ✅ Aucune chute, aucun objet manqué (téléport gz fiable)

---

## 📁 Structure du Projet

```
mycobot_R6A/
├── README.md                       # 👈 Ce fichier
├── SESSION_RESUME.md               # Point de départ sessions dev
├── DEVELOPMENT_SUMMARY.md          # Résumé technique complet
│
├── mycobot_gateway/                # 📦 Package ROS2 — contrôle + vision + sorting
│   ├── mycobot_gateway/
│   │   ├── bridge_tour.py                    # Client TCP vers Pi
│   │   ├── trajectory_to_robot_bridge.py     # JointTrajectory rad → JSON deg (téléop réel)
│   │   ├── gripper_to_robot_bridge.py        # Gripper bridge (no-op tant que pas de pince)
│   │   ├── simple_gui.py                     # GUI Tkinter
│   │   ├── slider_control.py                 # Contrôle sliders
│   │   ├── dream_inference_node.py           # Inférence DREAM + PnP pose
│   │   ├── pick_and_place_node.py            # State machine pick & place mono
│   │   ├── color_object_detector.py          # HSV + back-projection (top camera)
│   │   ├── sorting_orchestrator.py           # Pick & place multi-objets par couleur
│   │   └── synthetic_data_collector_v2.py    # Collecte Gazebo + anti-collision FK
│   ├── scripts/
│   │   ├── bridge_pi_simple.py     # Script Pi (serveur robot)
│   │   └── pi_camera_server.py     # Script Pi (serveur caméras)
│   └── launch/                     # Fichiers launch ROS2
│
├── mycobot_description/            # 📦 Package ROS2 — URDF/Gazebo
│   ├── urdf/320_pi/                # Modèle 3D + 4 caméras stylisées (corps + objectif + LED)
│   ├── urdf/pro_adaptive_gripper/  # Gripper adaptatif (meshes)
│   ├── config/controller.yaml      # JTC + gripper_position_controller (gz_ros2_control)
│   └── worlds/
│       ├── randomized.sdf                # Monde de base (synthetic data v1)
│       ├── randomized_v2.sdf             # 6 lights + 12 clutter objects (v2)
│       ├── pick_and_place.sdf            # Cube rouge + zone verte (mono-objet)
│       └── pick_and_place_sorting.sdf    # 4 objets colorés + 4 bacs colorés
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
| Raspberry Pi | 10.10.0.221 | 5005 (robot) + 5006 (caméras) |

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
ping 10.10.0.221
nc -zv 10.10.0.221 5005   # robot bridge
nc -zv 10.10.0.221 5006   # camera server
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
| [`SESSION_RESUME.md`](SESSION_RESUME.md) | Point de départ pour le développement (état courant) |
| [`DEVELOPMENT_SUMMARY.md`](DEVELOPMENT_SUMMARY.md) | Résumé technique complet |
| [`INDEX.md`](INDEX.md) | Index général de la documentation |
| [`CHANGELOG.md`](CHANGELOG.md) | Historique versionné (Keep a Changelog) |
| [`CLAUDE.md`](CLAUDE.md) | Onboarding + POC direction (Isaac Sim, VLA, etc.) |
| [`docs/ARCHITECTURE.md`](docs/ARCHITECTURE.md) | Architecture détaillée du système |
| [`docs/QUICKSTART.md`](docs/QUICKSTART.md) | Guide démarrage rapide |
| [`docs/SYNTHETIC_DATA.md`](docs/SYNTHETIC_DATA.md) | Pipeline données synthétiques |
| [`docs/ROBOT_QUICKSTART.md`](docs/ROBOT_QUICKSTART.md) | Procédure robot réel |
| **Téléopération** | |
| [`docs/TELEOPERATION.md`](docs/TELEOPERATION.md) | Pipeline téléop main (filtres, mapping, historique) |
| [`docs/TELEOP_ARCHITECTURE_VIZ.md`](docs/TELEOP_ARCHITECTURE_VIZ.md) | Visuel détaillé — détection main → mouvement bras |
| [`docs/TELEOP_DASHBOARD.md`](docs/TELEOP_DASHBOARD.md) | Manuel utilisateur du dashboard ABMI 3-onglets |
| [`docs/TELEOP_TUNING.md`](docs/TELEOP_TUNING.md) | Référence paramètres + dépannage téléop |
| [`docs/TELEOP_SIM_TESTING.md`](docs/TELEOP_SIM_TESTING.md) | **Procédure de validation en simulation seule** (avant le robot réel) |
| [`docs/REAL_ROBOT_TEST_PROCEDURE.md`](docs/REAL_ROBOT_TEST_PROCEDURE.md) | Protocole de calibration sécurisé sur robot physique |
| **Pick-and-place / sorting** | |
| [`mycobot_description/README_GAZEBO.md`](mycobot_description/README_GAZEBO.md) | Worlds Gazebo (mono, sorting) + visuels caméra |
| [`mycobot_gateway/README.md`](mycobot_gateway/README.md) | Nœuds, launches, topics — incluant `color_object_detector` et `sorting_orchestrator` |
| **Données / ML** | |
| [`datasets/README.md`](datasets/README.md) | Documentation des datasets |
| [`training/README.md`](training/README.md) | Documentation pipeline ML |
| [`training/dream/README.md`](training/dream/README.md) | Module DREAM (keypoints + PnP, training mixte) |

---

## 📄 License

Apache License 2.0

## 👥 Contributeurs

- ABMI Software Team
