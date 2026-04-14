# 📋 SESSION RESUME - MyCobot 320 Pi Gateway Bridge

> **Date de dernière mise à jour :** 14 avril 2026  
> **Version :** 1.7.0  
> **Repository GitHub :** https://github.com/ABMI-software/mycobot_320pi_R6A  
> **Branche :** `feature/pose-training`  
> **Dernier commit :** `0e452ece` (feat(dream): DREAM keypoint-based pose estimation module)

---

## 🎯 POINT DE DÉPART - Session Suivante

### Contexte Résumé

Le projet possède un pipeline complet : **Gazebo simulation → données synthétiques → entraînement CNN → capture réelle**. L'approche directe (régression image → angles) a échoué sur données réelles (signal visuel insuffisant). Le projet a pivoté vers **DREAM** (keypoint detection + PnP), qui atteint **97% de détection et 3.1px (4.5mm) médiane sur données synthétiques** avec VGG-Q. Cependant, le **transfert sim-to-real reste faible (~26% détection)**.

### �� Problème Principal à Résoudre

**Le domain gap Gazebo → caméra réelle est trop large pour DREAM.** Les belief maps ont des pics 10x plus faibles sur images réelles (0.02-0.25) vs synthétiques (0.5-1.0). L'augmentation agressive n'a apporté qu'une amélioration marginale (22.9% → 25.7%).

**Pistes prioritaires :**
1. **Self-supervised labeling** : FK + caméra calibrée → keypoints GT automatiques sur images réelles
2. **Domain Randomization avancée** : textures/backgrounds photo-réalistes dans Gazebo
3. **Fine-tuning sur données réelles** avec labels auto-générés
4. **Style transfer** (CycleGAN) entre Gazebo et réel

### ✅ Ce qui a été accompli

#### Session 26/03/2026 — Bridge ROS2
| Tâche | Statut |
|-------|--------|
| Création repo GitHub | ✅ |
| Architecture TCP Tour ↔ Pi | ✅ |
| bridge_tour (PC) + bridge_pi_simple (Pi) | ✅ |
| RViz visualisation + slider_control | ✅ |

#### Session 31/03/2026 — Gazebo + Synthétique
| Tâche | Statut |
|-------|--------|
| Simulation Gazebo Harmonic + 4 caméras | ✅ |
| Collecte 5000 poses × 4 vues = 20K images | ✅ |
| Domain randomization (éclairage, clutter) | ✅ |
| Multi-view ResNet50 : **12.97° MAE** | ✅ |

#### Sessions 01-02/04/2026 — Données Réelles
| Tâche | Statut |
|-------|--------|
| Camera server Pi (cam0+cam3, TCP:5006) | ✅ |
| Capture 2000 poses réelles (0 collisions) | ✅ |
| FK safety capture (protection table+câbles) | ✅ |
| Training données réelles (régression directe) | ❌ Bloqué à 32.76° baseline |
| Diagnostic : corrélation pose/pixel = 0.004 | ✅ Cause identifiée |

#### Session 03/04/2026 — DREAM Keypoint Pose Estimation
| Tâche | Statut |
|-------|--------|
| Clone DREAM (NVlabs) + installation | ✅ |
| Module FK MyCobot (7 keypoints, 4 caméras) | ✅ |
| Conversion NDDS (20K synth + 2K réel) | ✅ |
| Training ResNet-H (25 époques) | ❌ BN instable, tué epoch 10 |
| Training VGG-base (25 époques) | ✅ Stable, val=0.000438 |
| Training VGG-aug (25 époques) | ✅ Stable, val=0.000667 |
| Évaluation synthétique : 97% détection, 3.1px médiane | ✅ |
| Test sim-to-real : ~26% détection | ⚠️ Domain gap |
| Git commit `0e452ece` | ✅ |

---

## ❗ PROCHAINES ÉTAPES (Priorité)

### 1. 🔴 Réduire le domain gap sim-to-real (CRITIQUE)

Le modèle VGG-aug fonctionne bien sur synthétique mais échoue sur réel. Options :

#### Option A : Self-supervised labeling sur images réelles (RECOMMANDÉ)
- Utiliser les angles joints lus du robot + FK → positions 3D keypoints
- Projeter avec les intrinsèques caméra calibrée → labels 2D automatiques
- Fine-tuner VGG-aug sur ces données réelles auto-annotées

#### Option B : Domain randomization avancée dans Gazebo
- Textures photo-réalistes aléatoires (murs, table, sol)
- Backgrounds variés (images réelles du labo en arrière-plan)
- Plus de variation d'éclairage (HDR, ombres portées)

#### Option C : Style transfer (CycleGAN)
- Entraîner un réseau pour transformer images Gazebo → style réel
- Augmenter le dataset synthétique avec des images stylisées

### 2. 🟡 Pick-and-place en simulation Gazebo
- Créer un noeud ROS2 qui utilise DREAM inference sur le flux caméra Gazebo
- Intégrer avec MoveIt2 pour planification de trajectoire
- Valider la boucle complète : détection → pose → planification → exécution

### 3. 🟢 Bench test robot réel
- Confronter les résultats simulation vs robot réel
- Mesurer la précision de positionnement end-to-end

---

## 🏗️ Architecture du Système

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
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  │ dream/       │       │
│         │                │                │          │ predict.py   │       │
│         └────────────────┴────────────────┘          └──────┬───────┘       │
│                                   │                         │              │
│                          /to_robot (JSON)            TCP:5005 + 5006       │
│                                   ▼                         ▼              │
│                         ┌─────────────────┐                                │
│                         │   bridge_tour   │                                │
│                         └────────┬────────┘                                │
├──────────────────────────────────┼──────────────────────────────────────────┤
│                    RÉSEAU ETHERNET (10.10.0.x)                             │
├──────────────────────────────────┼──────────────────────────────────────────┤
│                                  ▼                                          │
│            ┌─────────────────┐       ┌─────────────────┐                   │
│            │bridge_pi_simple │       │pi_camera_server │                   │
│            │  TCP:5005       │       │  TCP:5006       │                   │
│            └────────┬────────┘       └────────┬────────┘                   │
│                     │                         │                            │
│                     ▼                         ▼                            │
│            ┌─────────────────┐       ┌─────────────────┐                   │
│            │    pymycobot    │       │ Arducam USB ×2  │                   │
│            │  /dev/ttyAMA0   │       │  cam0 + cam3    │                   │
│            └────────┬────────┘       └─────────────────┘                   │
│                     ▼                                                      │
│            ┌─────────────────┐                                             │
│            │  MyCobot 320 Pi │                                             │
│            └─────────────────┘                                             │
│                                                                             │
│                     RASPBERRY PI (10.10.0.225)                             │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 📦 Structure du Projet

```
mycobot_R6A/
├── SESSION_RESUME.md              # 👈 CE FICHIER
├── DEVELOPMENT_SUMMARY.md         # Résumé technique détaillé
├── INDEX.md                       # Index documentation
├── README.md                      # README principal
│
├── mycobot_gateway/               # 📦 Package ROS2 Principal
│   ├── mycobot_gateway/
│   │   ├── bridge_tour.py         # Client TCP vers Pi
│   │   ├── simple_gui.py          # GUI Tkinter
│   │   ├── slider_control.py      # Contrôle sliders temps réel
│   │   └── synthetic_data_collector_v2.py  # Collecte données Gazebo
│   ├── scripts/
│   │   ├── bridge_pi_simple.py    # Script Pi (serveur TCP robot)
│   │   └── pi_camera_server.py    # Script Pi (serveur TCP caméras)
│   └── launch/                    # Fichiers launch ROS2
│
├── mycobot_description/           # 📦 URDF + Gazebo
│   ├── urdf/320_pi/               # Modèle 3D (URDF + 4 caméras Gazebo)
│   └── worlds/randomized.sdf      # Monde Gazebo avec domain randomization
│
├── training/                      # 📦 Pipeline ML/IA
│   ├── dataset.py                 # Datasets: single/multi-view + PerImageNormalize
│   ├── model.py                   # PoseResNet + MultiViewPoseResNet
│   ├── train.py                   # Training: multi-view, finetune, auto-views
│   ├── predict.py                 # Inférence sur image(s)
│   ├── capture_real.py            # Capture réelle (FK safety, bridge + camera)
│   ├── dream/                     # 🆕 Module DREAM keypoint detection
│   │   ├── mycobot_fk.py          # FK + projection caméra (7 keypoints)
│   │   ├── convert_to_ndds.py     # Conversion → format NDDS
│   │   ├── train_dream.py         # Wrapper entraînement DREAM
│   │   ├── train_dream_augmented.py # Entraînement + augmentation agressive
│   │   ├── evaluate_dream.py      # Évaluation par keypoint
│   │   ├── infer_dream.py         # Inférence + PnP solving
│   │   ├── visualize_ndds.py      # Visualisation annotations
│   │   └── manip_configs/         # Config keypoints YAML
│   └── checkpoints_*/             # Modèles entraînés
│
├── scripts/                       # Scripts shell utilitaires
└── docs/                          # Documentation détaillée
```

---

## 🚀 COMMANDES UTILES

### Prérequis — Éviter le conflit Conda

```bash
# TOUJOURS exécuter avant ROS2 (conflit Python 3.13 vs 3.12)
conda deactivate
```

### Démarrage Bridge Pi
```bash
ssh er@10.10.0.225
# Terminal 1 : robot
python3 bridge_pi_simple.py
# Terminal 2 : caméras
python3 pi_camera_server.py --cameras 0 3 --names cam0 cam3
```

### Entraînement DREAM (Conda)
```bash
cd ~/ros_jazzy/src/mycobot_R6A

# 1. Conversion NDDS
/home/genji/miniconda/bin/python3 training/dream/convert_to_ndds.py \
  --input /tmp/mycobot_synth_v2 \
  --output /tmp/dream_data/synthetic \
  --source synth --cameras front right left top

# 2. Training VGG avec augmentation (RECOMMANDÉ)
/home/genji/miniconda/bin/python3 training/dream/train_dream_augmented.py \
  --data /tmp/dream_data/synthetic \
  --arch vgg --epochs 25 --batch-size 32 --lr 0.0001

# 3. Évaluation
/home/genji/miniconda/bin/python3 training/dream/evaluate_dream.py \
  --weights training/checkpoints_dream/vgg_augmented_e25/best_network.pth \
  --data /tmp/dream_data/synthetic \
  --split val --max-samples 500 --visualize
```

---

## 📊 Résultats — DREAM Keypoint Detection (VGG-aug)

### Précision par keypoint (synthétique, caméra à 0.8m, 1px = 1.44mm)

| Keypoint | Détection | Médiane px | Médiane mm | Erreur angulaire | <10px |
|----------|-----------|------------|------------|------------------|-------|
| base | 100% | 2.8px | 4.0mm | ~0.7° | 100% |
| link1 | 100% | 2.6px | 3.7mm | ~0.7° | 100% |
| link2 | 100% | 2.6px | 3.7mm | ~0.7° | 100% |
| link3 | 99% | 5.6px | 8.1mm | ~2.1° | 74% |
| link4 | 96% | 6.4px | 9.2mm | ~3.4° | 65% |
| link5 | 95% | 8.8px | 12.7mm | ~8.7° | 53% |
| link6 | 86% | 10.1px | 14.6mm | ~18.3° | 50% |
| **TOTAL** | **97%** | **3.1px** | **4.5mm** | **~0.8°** | **78%** |

### Comparaison Phase 1 vs Phase 2

| Approche | Erreur angulaire (synthétique) |
|----------|-------------------------------|
| Phase 1 — ResNet50 multi-view | 12.97° MAE |
| **Phase 2 — DREAM VGG-aug** | **~0.8° médiane / ~3.4° moyenne** |

→ DREAM est ~4x meilleur en moyenne, ~16x en médiane.

### Adéquation pick-and-place (exigence ±5mm)

| Zone | Erreur | Statut |
|------|--------|--------|
| Joints proximaux (base→link2) | 3.9 mm | ✅ OK |
| Joints intermédiaires (link3–link4) | 8–9 mm | ⚠️ Limite |
| End-effector (link6) | 14.6 mm | ❌ Insuffisant (besoin ~3x mieux) |

---

## 🔑 Points Importants

1. **Conda vs ROS2** : Toujours `conda deactivate` avant ROS2. Pour le training ML, utiliser `/home/genji/miniconda/bin/python3`.
2. **Dataset synthétique** : `/tmp/mycobot_synth_v2/` — 5000 poses × 4 caméras = 20K images
3. **Dataset réel** : `/tmp/real_dataset/` — 2000 poses × 2 caméras = 4000 images
4. **Données NDDS** : `/tmp/dream_data/synthetic/` — 20K frames DREAM format
5. **DREAM installé** : `/tmp/DREAM/` (pip install -e .)
6. **Meilleur modèle** : `checkpoints_dream/vgg_augmented_e25/best_network.pth`
7. **1px = 1.44mm** (caméra à 0.8m, fx=554.38)
8. **Problème #1** : domain gap sim-to-real (~26% détection vs 97%)

---

## 📚 Documentation

| Fichier | Description |
|---------|-------------|
| `SESSION_RESUME.md` | 👈 Ce fichier — point de départ |
| `DEVELOPMENT_SUMMARY.md` | Résumé technique complet (inclut DREAM + conversions px/mm/°) |
| `training/dream/README.md` | Documentation module DREAM |
| `docs/QUICKSTART.md` | Guide démarrage rapide |
| `docs/SYNTHETIC_DATA.md` | Pipeline données synthétiques |

---

*Ce fichier est le point de départ pour les prochaines sessions de développement.*  
*Dernière mise à jour : 14 avril 2026*
