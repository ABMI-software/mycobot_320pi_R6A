# SESSION RESUME — MyCobot 320 Pi R6A

> **Date de dernière mise à jour :** 23 avril 2026
> **Version :** 1.10.0
> **Branche active :** `feature/pick-and-place-sorting` (off `main`)
> **Repository :** https://github.com/ABMI-software/mycobot_320pi_R6A

---

## Point de départ rapide

```bash
# TOUJOURS exécuter avant ROS2
conda deactivate

source /opt/ros/jazzy/setup.bash
source ~/ros_jazzy/src/mycobot_R6A/install/setup.bash
```

---

## État actuel (21 avril 2026)

### Problème principal : Domain gap sim-to-real

Le modèle DREAM VGG atteint **97% de détection à 3.1px médiane** sur les données synthétiques, mais seulement **~26% de détection sur les images réelles**. Les pics des belief maps sont 10× plus faibles sur les images réelles que sur les synthétiques.

### Ce qui est fait

| Session | Tâche | Statut |
|---------|-------|--------|
| 26/03/2026 | Bridge TCP Tour ↔ Pi, GUI, RViz | ✅ |
| 31/03/2026 | Simulation Gazebo Harmonic 4 caméras | ✅ |
| 31/03/2026 | Collecte 5000 poses synthétiques × 4 vues (20K images) | ✅ |
| 31/03/2026 | Domain randomization (éclairage, matériaux) | ✅ |
| 31/03/2026 | Training multi-view ResNet50 → 12.97° MAE | ✅ |
| 01/04/2026 | Camera server Pi (cam0+cam3, TCP:5006) | ✅ |
| 02/04/2026 | Capture 2000 poses réelles (0 collisions) | ✅ |
| 02/04/2026 | FK safety capture (protection table+câbles) | ✅ |
| 02/04/2026 | Training régression directe sur données réelles | ❌ Bloqué à 32.76° baseline |
| 02/04/2026 | Diagnostic : corrélation pose/pixel = 0.004 | ✅ Cause identifiée |
| 03/04/2026 | Intégration DREAM (NVlabs) + FK 7 keypoints | ✅ |
| 03/04/2026 | Conversion 20K frames NDDS | ✅ |
| 03/04/2026 | Training VGG-base (25 époques) | ✅ val=0.000438 |
| 03/04/2026 | Training VGG-aug (25 époques) | ✅ val=0.000667 |
| 03/04/2026 | Évaluation synthétique : 97% détection, 3.1px | ✅ |
| 03/04/2026 | Test sim-to-real : ~26% détection | ⚠️ Domain gap |
| 15/04/2026 | Intégration gripper adaptatif (pro_adaptive_gripper) | ✅ |
| 15/04/2026 | Correction mesh link6 → link6_2022.dae | ✅ |
| 15/04/2026 | Limites articulaires corrigées (URDF officiel) | ✅ |
| 15/04/2026 | Anti-collision FK dans collecteur (rejet ~35% poses) | ✅ |
| 15/04/2026 | Training VGG 50K synth (98.3% det synth, 13.2% réel) | ✅ |
| 15/04/2026 | Fine-tune custom v1 (σ=4, 0% det) | ❌ Bug sigma |
| 16/04/2026 | Fine-tune custom v2 (σ=2, 0% det) | ❌ Belief maps effondrées |
| 16/04/2026 | Script merge_and_convert.py | ✅ |
| 16/04/2026 | Script train_pipeline.sh + monitor_collection.sh | ✅ |
| 16/04/2026 | Monde Gazebo v2 (randomized_v2.sdf — 6 lights, 12 objets) | ✅ |
| 16/04/2026 | Collecte 7500 poses × 4 vues (30K images) synth v2 | 🔄 À vérifier |
| 16/04/2026 | Training mixte natif (18K frames) — epoch 1: val=0.000474 | ✅ Terminé |
| 23/04/2026 | **Pick-and-place multi-objets par couleur** (4 objets → 4 bacs) | ✅ End-to-end vérifié |
| 23/04/2026 | `color_object_detector` (HSV + back-projection top camera) | ✅ 4/4 couleurs détectées |
| 23/04/2026 | `sorting_orchestrator` (boucle sur détections, gz `set_pose` carry) | ✅ Cycle complet ~95 s |
| 23/04/2026 | URDF caméras reshapées (corps + objectif + LED, plus de cubes 3 cm colorés) | ✅ |

### Ce qui reste à faire

1. **[ROUGE] Évaluer le modèle mixte** sur données réelles :
   ```bash
   source ~/ros_jazzy/venv_dream/bin/activate
   python training/dream/evaluate_dream.py \
     --weights training/checkpoints_dream/vgg_mixed_real_synth/best_network.pth \
     --data /tmp/dream_data/real_cam0 --split all
   ```

2. **[ROUGE] Si detection < 50%** sur réel → implémentation self-supervised labeling :
   - FK + angles joints lus → keypoints 3D → projection 2D → annotations GT automatiques
   - Fine-tune sur ces données réelles auto-annotées

3. **[JAUNE] Vérifier collecte 30K synth v2** dans `/tmp/dream_data/synthetic_50k_v2/`

4. **[VERT] Pick-and-place Gazebo (mono-objet)** : pipeline `pick_and_place.launch.py` validé
   et étendu en multi-objet par couleur (`pick_and_place_sorting.launch.py`)

5. **[VERT] Bench test robot réel** une fois detection > 50%

---

## Résultats DREAM par keypoint (VGG-aug, synthétique)

| Keypoint | Détection | Médiane px | Médiane mm | Erreur angulaire |
|----------|-----------|------------|------------|------------------|
| base | 100% | 2.8 px | 4.0 mm | ~0.7° |
| link1 | 100% | 2.6 px | 3.7 mm | ~0.7° |
| link2 | 100% | 2.6 px | 3.7 mm | ~0.7° |
| link3 | 99% | 5.6 px | 8.1 mm | ~2.1° |
| link4 | 96% | 6.4 px | 9.2 mm | ~3.4° |
| link5 | 95% | 8.8 px | 12.7 mm | ~8.7° |
| link6 | 86% | 10.1 px | 14.6 mm | ~18.3° |
| **TOTAL** | **97%** | **3.1 px** | **4.5 mm** | **~0.8°** |

> 1 px = 1.44 mm (caméra à 0.8m, fx=554.38)

### Comparaison approches

| Approche | Erreur angulaire (synthétique) |
|----------|-------------------------------|
| Phase 1 — ResNet50 multi-view | 12.97° MAE |
| **Phase 2 — DREAM VGG-aug** | **~0.8° médiane / ~3.4° moyenne** |

### Adéquation pick-and-place (exigence ±5mm)

| Zone | Erreur | Statut |
|------|--------|--------|
| Joints proximaux (base→link2) | 3.9 mm | ✅ OK |
| Joints intermédiaires (link3–link4) | 8–9 mm | ⚠️ Limite |
| End-effector (link6) | 14.6 mm | ❌ Insuffisant (besoin ~3×) |

---

## Chemins importants

| Ressource | Chemin |
|-----------|--------|
| Projet | `/home/genji/ros_jazzy/src/mycobot_R6A/` |
| Venv DREAM | `/home/genji/ros_jazzy/venv_dream/` |
| DREAM lib | `/tmp/DREAM/` |
| Data synth 50K | `/tmp/dream_data/synthetic_50k/` |
| Data réel | `/tmp/dream_data/real_cam0/` |
| Data mixte | `/tmp/dream_data/mixed_real_synth/` |
| Meilleur modèle (synth) | `training/checkpoints_dream/vgg_weighted_50k_e50/best_network.pth` |
| Modèle mixte | `training/checkpoints_dream/vgg_mixed_real_synth/best_network.pth` |

---

## Commandes utiles

### Prérequis

```bash
# Éviter conflit Conda ↔ ROS2 (Python 3.13 vs 3.12)
conda deactivate
```

### Démarrage bridge Pi

```bash
ssh er@10.10.0.225
# Terminal 1 : robot
python3 bridge_pi_simple.py
# Terminal 2 : caméras
python3 pi_camera_server.py --cameras 0 3 --names cam0 cam3
```

### Évaluation DREAM

```bash
source ~/ros_jazzy/venv_dream/bin/activate

# Sur données réelles
python training/dream/evaluate_dream.py \
  --weights training/checkpoints_dream/vgg_mixed_real_synth/best_network.pth \
  --data /tmp/dream_data/real_cam0 --split all

# Sur données synthétiques (vérification de régression)
python training/dream/evaluate_dream.py \
  --weights training/checkpoints_dream/vgg_mixed_real_synth/best_network.pth \
  --data /tmp/dream_data/synthetic_50k --split val
```

### Collecte données synthétiques v3

```bash
conda deactivate
source /opt/ros/jazzy/setup.bash
source ~/ros_jazzy/src/mycobot_R6A/install/setup.bash

# Monde randomized_v2 (6 lights, 12 objets clutter)
ros2 launch mycobot_gateway synthetic_data_v3.launch.py num_samples:=7500
```

### Merge + conversion NDDS + pipeline d'entraînement

```bash
# Script automatisé complet
bash scripts/train_pipeline.sh

# Ou manuellement :
source ~/ros_jazzy/venv_dream/bin/activate
python training/dream/merge_and_convert.py \
  --real /tmp/dream_data/real_cam0 \
  --synth /tmp/dream_data/synthetic_50k \
  --output /tmp/dream_data/mixed_v2 \
  --real-oversample 5

python /tmp/DREAM/scripts/train_network.py \
  -i /tmp/dream_data/mixed_v2 \
  -m /tmp/DREAM/manip_configs/mycobot320.yaml \
  -ar /tmp/DREAM/arch_configs/dream_vgg_q.yaml \
  -e 25 -b 32 -lr 0.0001 \
  -o training/checkpoints_dream/vgg_mixed_v2 -f
```

### Pick-and-place simulation

```bash
conda deactivate
source /opt/ros/jazzy/setup.bash
source ~/ros_jazzy/src/mycobot_R6A/install/setup.bash

# Mono-objet (cube rouge → bac vert)
ros2 launch mycobot_gateway pick_and_place.launch.py

# Multi-objet par couleur (4 objets → 4 bacs)
ros2 launch mycobot_gateway pick_and_place_sorting.launch.py
# Variantes :
ros2 launch mycobot_gateway pick_and_place_sorting.launch.py use_detector:=false
ros2 launch mycobot_gateway pick_and_place_sorting.launch.py process_order:=blue,green
```

---

## Architecture du système

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              PC TOUR (10.10.0.115)                          │
│                         ROS2 Jazzy / Ubuntu 24.04 / Python 3.12             │
│                         Conda: Python 3.13 / PyTorch 2.6 + CUDA 12.4       │
│                         GPU: NVIDIA RTX 4000 Ada (20 GB VRAM)               │
├─────────────────────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌──────────────┐       │
│  │ simple_gui  │  │slider_control│  │teleop_keyb. │  │ training/    │       │
│  │  (Tkinter)  │  │(joint_states)│  │  (clavier)  │  │ dream/       │       │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  └──────┬───────┘       │
│         └────────────────┴────────────────┘                │              │
│                          │                          TCP:5005 + 5006        │
│                   /to_robot (JSON)                         │              │
│                          ▼                                 ▼              │
│                ┌─────────────────┐                                         │
│                │   bridge_tour   │                                         │
│                └────────┬────────┘                                         │
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

## Structure du projet

```
mycobot_R6A/
├── SESSION_RESUME.md              # Ce fichier
├── DEVELOPMENT_SUMMARY.md         # Résumé technique détaillé
├── CHANGELOG.md                   # Historique des versions
├── README.md                      # README principal
│
├── mycobot_gateway/               # Package ROS2 — contrôle + vision
│   ├── mycobot_gateway/
│   │   ├── bridge_tour.py         # Client TCP vers Pi
│   │   ├── simple_gui.py          # GUI Tkinter
│   │   ├── slider_control.py      # Contrôle sliders temps réel
│   │   ├── dream_inference_node.py# Inférence DREAM + PnP ROS2
│   │   ├── pick_and_place_node.py # State machine pick & place
│   │   └── synthetic_data_collector_v2.py
│   ├── scripts/
│   │   ├── bridge_pi_simple.py    # Script Pi (serveur TCP robot)
│   │   └── pi_camera_server.py    # Script Pi (serveur TCP caméras)
│   └── launch/
│       ├── pick_and_place.launch.py
│       ├── synthetic_data_v2.launch.py
│       └── synthetic_data_v3.launch.py
│
├── mycobot_description/           # Package ROS2 — URDF/Gazebo
│   ├── urdf/320_pi/               # Modèle 3D + 4 caméras Gazebo
│   │   └── link6_2022.dae         # Mesh link6 pour compat. gripper
│   ├── urdf/pro_adaptive_gripper/ # Gripper adaptatif (meshes STL)
│   └── worlds/
│       ├── randomized.sdf         # Monde de base
│       └── randomized_v2.sdf      # 6 lights + 12 clutter objects
│
├── training/                      # Pipeline ML/IA
│   ├── train.py / model.py        # Legacy: régression directe (abandonné)
│   ├── capture_real.py            # Capture réelle avec FK safety
│   └── dream/                     # DREAM keypoint detection (actif)
│       ├── mycobot_fk.py          # FK + projection (7 keypoints)
│       ├── convert_to_ndds.py     # Conversion → format NDDS
│       ├── merge_and_convert.py   # Fusion datasets + conversion
│       ├── evaluate_dream.py      # Évaluation (filtre sentinel -999.99)
│       ├── infer_dream.py         # Inférence + PnP solving
│       ├── finetune_real.py       # Fine-tuning expérimental (⚠️ ne fonctionne pas)
│       └── manip_configs/mycobot320.yaml
│
├── scripts/
│   ├── train_pipeline.sh          # Pipeline merge→NDDS→training automatisé
│   └── monitor_collection.sh      # Suivi collecte en temps réel
│
└── docs/                          # Documentation détaillée
```

---

## Points importants

1. **Conda vs ROS2** : Toujours `conda deactivate` avant ROS2. Training ML → `/home/genji/miniconda/bin/python3` ou venv_dream.
2. **Fine-tuning DREAM custom** : Deux tentatives ont échoué (σ mismatch + belief map collapse). Utiliser uniquement `train_network.py` natif.
3. **Sentinel DREAM** : DREAM renvoie -999.99 quand peak < seuil — filtrer dans l'évaluation.
4. **VGG sans BatchNorm** est stable. ResNet+BN explose avec batch_size < 64.
5. **1 px = 1.44 mm** (caméras latérales à 0.8m, fx=554.38).
6. **Gripper** : intégré dans la simulation mais joints fixés (pas de support `mimic` dans Gazebo Harmonic).

---

## Documentation

| Fichier | Description |
|---------|-------------|
| [SESSION_RESUME.md](SESSION_RESUME.md) | Ce fichier — point de départ |
| [DEVELOPMENT_SUMMARY.md](DEVELOPMENT_SUMMARY.md) | Résumé technique complet |
| [CHANGELOG.md](CHANGELOG.md) | Historique des versions |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | Architecture détaillée |
| [docs/SYNTHETIC_DATA.md](docs/SYNTHETIC_DATA.md) | Pipeline données synthétiques |
| [training/README.md](training/README.md) | Pipeline ML / DREAM |
| [training/dream/README.md](training/dream/README.md) | Module DREAM |

---

*Dernière mise à jour : 21 avril 2026*
