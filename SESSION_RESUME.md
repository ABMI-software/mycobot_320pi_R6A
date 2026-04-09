# 📋 SESSION RESUME - MyCobot 320 Pi Gateway Bridge# 📋 SESSION RESUME - MyCobot 320 Pi Gateway Bridge# 📋 SESSION RESUME - MyCobot 320 Pi Gateway Bridge



> **Date de dernière mise à jour :** 3 avril 2026  

> **Version :** 1.6.0  

> **Repository GitHub :** https://github.com/ABMI-software/mycobot_320pi_R6A  > **Date de dernière mise à jour :** 2 avril 2026  > **Date de dernière mise à jour :** 1 avril 2026  

> **Branche :** `feature/pose-training`  

> **Dernier commit :** `0e452ece` (feat(dream): DREAM keypoint-based pose estimation module)> **Version :** 1.5.0  > **Version :** 1.4.0  



---> **Repository GitHub :** https://github.com/ABMI-software/mycobot_320pi_R6A  > **Repository GitHub :** https://github.com/ABMI-software/mycobot_320pi_R6A  



## 🎯 POINT DE DÉPART - Session Suivante (4 avril 2026)> **Branche :** `main`> **Branche :** `main` | `feature/gazebo` | `feature/synthetic-data` | `feature/pose-training`  



### Contexte Résumé> **Dernier commit :** (feature/pose-training)



Le projet possède un pipeline complet : **Gazebo simulation → données synthétiques → entraînement CNN → capture réelle**. L'approche directe (régression image → angles) a échoué sur données réelles (signal visuel insuffisant). Le projet a pivoté vers **DREAM** (keypoint detection + PnP), qui atteint **97% de détection et 3.1px médiane sur données synthétiques** avec VGG-Q. Cependant, le **transfert sim-to-real reste faible (~26% détection)**.---



### 🔴 Problème Principal à Résoudre---



**Le domain gap Gazebo → caméra réelle est trop large pour DREAM.** Les belief maps ont des pics 10× plus faibles sur images réelles (0.02-0.25) vs synthétiques (0.5-1.0). L'augmentation agressive n'a apporté qu'une amélioration marginale (22.9% → 25.7%).## 🎯 POINT DE DÉPART - Session Suivante (3 avril 2026)



**Pistes prioritaires :**## 🎯 POINT DE DÉPART - Session Précédente

1. **Self-supervised labeling** : FK + caméra calibrée → keypoints GT automatiques sur images réelles

2. **Domain Randomization avancée** : textures/backgrounds photo-réalistes dans Gazebo### Contexte Résumé

3. **Fine-tuning sur données réelles** avec labels auto-générés

4. **Style transfer** (CycleGAN) entre Gazebo et réel### ✅ Ce qui a été accompli



### ✅ Ce qui a été accompliLe projet a un pipeline complet : **Gazebo simulation → données synthétiques → entraînement CNN → capture réelle**. Le modèle multi-view ResNet50 atteint **12.97° MAE sur données synthétiques** (4 caméras, 5000 poses). Cependant, **l'entraînement sur les données réelles (2000 poses, 2 caméras) est bloqué** au niveau du mean-predictor baseline (~32.76°).



#### Session 26/03/2026 — Bridge ROS2| Tâche | Statut | Détails |

| Tâche | Statut |

|-------|--------|### 🔴 Problème Principal à Résoudre|-------|--------|---------|

| Création repo GitHub | ✅ |

| Architecture TCP Tour ↔ Pi | ✅ || Création repo GitHub | ✅ Complété | `ABMI-software/mycobot_320pi_R6A` |

| bridge_tour (PC) + bridge_pi_simple (Pi) | ✅ |

| RViz visualisation + slider_control | ✅ |**Les images réelles ne contiennent pas assez de signal visuel exploitable** pour que le CNN apprenne la relation image → pose. Diagnostic complet :| Architecture TCP Tour ↔ Pi | ✅ Validé | Connexion stable |



#### Session 31/03/2026 — Gazebo + Synthétique| bridge_tour (PC) | ✅ Fonctionnel | Client TCP ROS2 |

| Tâche | Statut |

|-------|--------|1. **Corrélation pose ↔ pixel quasi-nulle** (r = 0.004, même entre paires consécutives sans dérive d'éclairage)| bridge_pi_simple (Pi) | ✅ Fonctionnel | Serveur TCP + pymycobot |

| Simulation Gazebo Harmonic + 4 caméras | ✅ |

| Collecte 5000 poses × 4 vues = 20K images | ✅ |2. **Le robot est trop petit dans l'image** : cam0 = 15.4% des pixels varient, cam3 = 5.7% seulement| simple_gui | ✅ Testé | Interface Tkinter |

| Domain randomization (éclairage, clutter) | ✅ |

| Multi-view ResNet50 : **12.97° MAE** | ✅ |3. **56% de l'image est du fond statique** qui ne change jamais| slider_control | ✅ Testé | Robot suit les sliders en temps réel |



#### Sessions 01-02/04/2026 — Données Réelles4. **Dérive d'éclairage massive** pendant les 2h de capture (luminosité moyenne : 140 → 82 → 143)| RViz visualisation | ✅ Corrigé | Config avec Fixed Frame = base |

| Tâche | Statut |

|-------|--------|5. Même entre poses extrêmes (J1 : -83° vs +83°), seulement 9.1% des pixels changent significativement| Commandes JSON | ✅ Validé | send_angles, get_angles, go_home... |

| Camera server Pi (cam0+cam3, TCP:5006) | ✅ |

| Capture 2000 poses réelles (0 collisions) | ✅ |

| FK safety capture (protection table+câbles) | ✅ |

| Training données réelles (régression directe) | ❌ Bloqué à 32.76° baseline |### ✅ Ce qui a été accompli (sessions 1-2 avril 2026)### 🔧 Dernières modifications

| Diagnostic : corrélation pose/pixel = 0.004 | ✅ Cause identifiée |



#### Session 03/04/2026 — DREAM Keypoint Pose Estimation

| Tâche | Statut || Tâche | Statut | Détails |1. **`slider_control.launch.py`** - Ajout du chemin config RViz

|-------|--------|

| Clone DREAM (NVlabs) + installation | ✅ ||-------|--------|---------|2. **`bridge_pi_simple.py`** - Ajout commandes texte (get_angles, power_on/off, gripper)

| Module FK MyCobot (7 keypoints, 4 caméras) | ✅ |

| Conversion NDDS (20K synth + 2K réel) | ✅ || Capture 2000 échantillons réels | ✅ | 2 caméras (cam0, cam3), 4000 images, 1.2 GB, 0 collisions |

| Training ResNet-H (25 époques) | ❌ BN instable, tué epoch 10 |

| Training VGG-base (25 époques) | ✅ Stable, val=0.000438 || FK safety dans capture_real.py | ✅ | Calcul FK depuis dimensions URDF réelles, protection table + câbles |---

| Training VGG-aug (25 époques) | ✅ Stable, val=0.000667 |

| Évaluation synthétique : 97% détection, 3.1px médiane | ✅ || Auto-détection vues CSV | ✅ | dataset.py + train.py gèrent `--views cam0 cam3` |

| Test sim-to-real : ~26% détection | ⚠️ Domain gap |

| Git commit `0e452ece` | ✅ || Chargement checkpoint compatible | ✅ | train.py filtre les couches incompatibles (4→2 vues) |## �️ Architecture du Système



---| PerImageNormalize | ✅ testé | Élimine la dérive d'éclairage, mais insuffisant seul |



## ❗ PROCHAINES ÉTAPES (Priorité)| Diagnostic dataset complet | ✅ | Corrélation, variance map, uniqueness, off-by-one, SSIM |```



### 1. 🔴 Réduire le domain gap sim-to-real (CRITIQUE)| Test overfit 10 samples | ✅ 0.01° | Prouve que le modèle et le pipeline fonctionnent correctement |┌─────────────────────────────────────────────────────────────────────────────┐



Le modèle VGG-aug fonctionne bien sur synthétique mais échoue sur réel. Options :| Ablation preprocessing | 🔄 Partiel | Exp A (Standard) : 31.81°. Exp B-E interrompus |│                              PC TOUR (10.10.0.115)                          │



#### Option A : Self-supervised labeling sur images réelles (RECOMMANDÉ)│                         ROS2 Jazzy / Ubuntu 24.04 / Python 3.12             │

- Utiliser les angles joints lus du robot + FK → positions 3D keypoints

- Projeter avec les intrinsèques caméra calibrée → labels 2D automatiques### 📊 Résultats des Tentatives d'Entraînement sur Données Réelles├─────────────────────────────────────────────────────────────────────────────┤

- Fine-tuner VGG-aug sur ces données réelles auto-annotées

│                                                                             │

#### Option B : Domain randomization avancée dans Gazebo

- Textures photo-réalistes aléatoires (murs, table, sol)| Approche | Best MAE | vs Baseline (32.76°) | Note |│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │

- Backgrounds variés (images réelles du labo en arrière-plan)

- Plus de variation d'éclairage (HDR, ombres portées)|----------|----------|----------------------|------|│  │ simple_gui  │  │slider_control│  │teleop_keyb. │  │marker_follow│        │



#### Option C : Style transfer (CycleGAN)| Multi-view ResNet50, lr=1e-3 | 32.7° | -0.06° | Stagne dès epoch 1 |│  │  (Tkinter)  │  │(joint_states)│  │  (clavier)  │  │   (ArUco)   │        │

- Entraîner un réseau pour transformer images Gazebo → style réel

- Augmenter le dataset synthétique avec des images stylisées| Single-view ResNet18, lr=3e-3 | 32.8° | +0.04° | Aucun progrès |│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘        │



### 2. 🟡 Pick-and-place en simulation Gazebo| SmoothL1 + OneCycleLR (minimal) | 31.75° | -1.01° | Overfit epoch 20+ |│         │                │                │                │               │

- Créer un nœud ROS2 qui utilise DREAM inference sur le flux caméra Gazebo

- Intégrer avec MoveIt2 pour planification de trajectoire| PerImageNormalize + SmoothL1 | 31.69° | -1.07° | Overfit epoch 6 |│         └────────────────┴────────────────┴────────────────┘               │

- Valider la boucle complète : détection → pose → planification → exécution

| Standard ImageNet (ablation A) | 31.81° | -0.95° | Early stop epoch 32 |│                                   │                                         │

### 3. 🟢 Bench test robot réel

- Confronter les résultats simulation vs robot réel| **Mean predictor (baseline)** | **32.76°** | **0.00°** | Prédire toujours la moyenne |│                          /to_robot (JSON)                                   │

- Mesurer la précision de positionnement end-to-end

│                                   ▼                                         │

---

**→ Toutes les approches ne dépassent la baseline que d'~1° avant d'overfitter.**│                         ┌─────────────────┐                                │

## 🏗️ Architecture du Système

│                         │   bridge_tour   │◄─── /from_robot                │

```

┌─────────────────────────────────────────────────────────────────────────────┐---│                         │   (TCP Client)  │                                │

│                              PC TOUR (10.10.0.115)                          │

│                         ROS2 Jazzy / Ubuntu 24.04 / Python 3.12             ││                         └────────┬────────┘                                │

│                         Conda: Python 3.13 / PyTorch 2.6 + CUDA 12.4       │

│                         GPU: NVIDIA RTX 4000 Ada (20 GB VRAM)               │## ❗ PROCHAINES ÉTAPES (Priorité)│                                  │ TCP:5005                                │

├─────────────────────────────────────────────────────────────────────────────┤

│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌──────────────┐       │├──────────────────────────────────┼──────────────────────────────────────────┤

│  │ simple_gui  │  │slider_control│  │teleop_keyb. │  │ training/    │       │

│  │  (Tkinter)  │  │(joint_states)│  │  (clavier)  │  │ train.py     │       │### 1. 🔴 Résoudre le problème de signal visuel (CRITIQUE)│                                  │                                          │

│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  │ dream/       │       │

│         │                │                │          │ predict.py   │       ││                    RÉSEAU ETHERNET (10.10.0.x)                             │

│         └────────────────┴────────────────┘          └──────┬───────┘       │

│                                   │                         │              │Le problème fondamental : **le robot est trop petit dans les images et les changements de pose sont quasi-invisibles au niveau pixel**. Options :│                                  │                                          │

│                          /to_robot (JSON)            TCP:5005 + 5006       │

│                                   ▼                         ▼              │├──────────────────────────────────┼──────────────────────────────────────────┤

│                         ┌─────────────────┐                                │

│                         │   bridge_tour   │                                │#### Option A : Recapturer avec de meilleures conditions (RECOMMANDÉ)│                                  ▼                                          │

│                         └────────┬────────┘                                │

├──────────────────────────────────┼──────────────────────────────────────────┤- **Fixer l'exposition caméra** (évite la dérive d'éclairage naturelle)│                         ┌─────────────────┐                                │

│                    RÉSEAU ETHERNET (10.10.0.x)                             │

├──────────────────────────────────┼──────────────────────────────────────────┤- **Rapprocher encore les caméras** (robot doit remplir > 50% du frame)│                         │bridge_pi_simple │                                │

│                                  ▼                                          │

│            ┌─────────────────┐       ┌─────────────────┐                   │- **Éclairage artificiel constant** (pas de lumière naturelle variable)│                         │  (TCP Server)   │                                │

│            │bridge_pi_simple │       │pi_camera_server │                   │

│            │  TCP:5005       │       │  TCP:5006       │                   │- Modifier `pi_camera_server.py` pour fixer exposure + white-balance :│                         └────────┬────────┘                                │

│            └────────┬────────┘       └────────┬────────┘                   │

│                     │                         │                            │```python│                                  │                                          │

│                     ▼                         ▼                            │

│            ┌─────────────────┐       ┌─────────────────┐                   │cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 1)   # manual mode│                                  ▼                                          │

│            │    pymycobot    │       │ Arducam USB ×2  │                   │

│            │  /dev/ttyAMA0   │       │  cam0 + cam3    │                   │cap.set(cv2.CAP_PROP_EXPOSURE, -6)        # fixed exposure│                         ┌─────────────────┐                                │

│            └────────┬────────┘       └─────────────────┘                   │

│                     ▼                                                      │cap.set(cv2.CAP_PROP_AUTO_WB, 0)          # fixed white balance│                         │    pymycobot    │                                │

│            ┌─────────────────┐                                             │

│            │  MyCobot 320 Pi │                                             │```│                         │  /dev/ttyAMA0   │                                │

│            └─────────────────┘                                             │

│                                                                             ││                         └────────┬────────┘                                │

│                     RASPBERRY PI (10.10.0.225)                             │

└─────────────────────────────────────────────────────────────────────────────┘#### Option B : Finir l'ablation CLAHE + Crop + Grayscale│                                  │                                          │

```

```bash│                         ┌────────▼────────┐                                │

---

# Le script est prêt dans /tmp/experiment_ablation.py│                         │  MyCobot 320 Pi │                                │

## 📦 Structure du Projet

# Relancer avec stdbuf pour éviter le problème de buffering :│                         │     (Robot)     │                                │

```

mycobot_R6A/cd ~/ros_jazzy/src/mycobot_R6A│                         └─────────────────┘                                │

├── SESSION_RESUME.md              # 👈 CE FICHIER

├── DEVELOPMENT_SUMMARY.md         # Résumé technique détailléstdbuf -oL /home/genji/miniconda/bin/python3 /tmp/experiment_ablation.py 2>&1 | tee /tmp/ablation_log.txt│                                                                             │

├── INDEX.md                       # Index documentation

├── README.md                      # README principal```│                     RASPBERRY PI (10.10.0.218)                             │

│

├── mycobot_gateway/               # 📦 Package ROS2 PrincipalExperiments à tester : CLAHE, Crop robot region, CLAHE+Crop, Grayscale+Crop.│                   ROS2 Galactic / Ubuntu 20.04 / Python 3.8                │

│   ├── mycobot_gateway/

│   │   ├── bridge_tour.py         # Client TCP vers Pi└─────────────────────────────────────────────────────────────────────────────┘

│   │   ├── simple_gui.py          # GUI Tkinter

│   │   ├── slider_control.py      # Contrôle sliders temps réel#### Option C : Approches alternatives```

│   │   └── synthetic_data_collector_v2.py  # Collecte données Gazebo

│   ├── scripts/- Pré-traiter avec edge detection (Canny/Sobel) pour extraire les contours du robot

│   │   ├── bridge_pi_simple.py    # Script Pi (serveur TCP robot)

│   │   └── pi_camera_server.py    # Script Pi (serveur TCP caméras)- Segmentation (SAM) pour isoler le robot avant régression---

│   └── launch/                    # Fichiers launch ROS2

│- Prédire uniquement J1-J3 (les joints les plus visibles) au lieu des 6

├── mycobot_description/           # 📦 URDF + Gazebo

│   ├── urdf/320_pi/               # Modèle 3D (URDF + 4 caméras Gazebo)## 📦 Structure du Projet

│   └── worlds/randomized.sdf      # Monde Gazebo avec domain randomization

│### 2. 🟡 Si la recapture est choisie — checklist

├── training/                      # 📦 Pipeline ML/IA

│   ├── dataset.py                 # Datasets: single/multi-view + PerImageNormalize- [ ] Modifier `pi_camera_server.py` pour fixer exposition et white-balance```

│   ├── model.py                   # PoseResNet + MultiViewPoseResNet

│   ├── train.py                   # Training: multi-view, finetune, auto-views- [ ] Rapprocher les caméras (robot > 50% du frame)mycobot_R6A/

│   ├── predict.py                 # Inférence sur image(s)

│   ├── capture_real.py            # Capture réelle (FK safety, bridge + camera)- [ ] Recapturer 2000+ poses avec éclairage constant├── SESSION_RESUME.md              # 👈 CE FICHIER - Point de départ

│   ├── dream/                     # 🆕 Module DREAM keypoint detection

│   │   ├── mycobot_fk.py          # FK + projection caméra (7 keypoints)- [ ] Vérifier corrélation pose ↔ pixel > 0.3 avant de lancer le training├── DEVELOPMENT_SUMMARY.md         # Résumé technique détaillé

│   │   ├── convert_to_ndds.py     # Conversion → format NDDS

│   │   ├── train_dream.py         # Wrapper entraînement DREAM├── INDEX.md                       # Index de documentation

│   │   ├── train_dream_augmented.py # Entraînement + augmentation agressive

│   │   ├── evaluate_dream.py      # Évaluation par keypoint### 3. 🟢 Si le training fonctionne├── README.md                      # README principal

│   │   ├── infer_dream.py         # Inférence + PnP solving

│   │   ├── visualize_ndds.py      # Visualisation annotations- [ ] Lancer `train.py` complet avec meilleures transforms├── bridge_pi_debug.py             # Script debug pour Pi

│   │   └── manip_configs/         # Config keypoints YAML

│   └── checkpoints_*/             # Modèles entraînés- [ ] Fine-tune du modèle synthétique (12.97°) sur données réelles│

│

├── scripts/                       # Scripts shell utilitaires- [ ] Commit final et deploy├── mycobot_gateway/               # 📦 Package ROS2 Principal

└── docs/                          # Documentation détaillée

```│   ├── package.xml



------│   ├── setup.py



## 🚀 COMMANDES UTILES│   │



### ⚠️ PRÉREQUIS CRITIQUE — Éviter le conflit Conda## 🏗️ Architecture du Système│   ├── mycobot_gateway/           # Modules Python



```bash│   │   ├── __init__.py

# TOUJOURS exécuter avant ROS2 (conflit Python 3.13 vs 3.12)

conda deactivate```│   │   ├── bridge_tour.py         # ⭐ Client TCP vers Pi



# OU utiliser la commande "propre" :┌─────────────────────────────────────────────────────────────────────────────┐│   │   ├── robot_commander.py     # Interface CLI

env -i HOME=$HOME PATH="/usr/bin:/bin:/opt/ros/jazzy/bin" DISPLAY=$DISPLAY bash

```│                              PC TOUR (10.10.0.115)                          ││   │   ├── joint_sync.py          # Sync angles → RViz



### Démarrage Bridge Pi│                         ROS2 Jazzy / Ubuntu 24.04 / Python 3.12             ││   │   ├── simple_gui.py          # GUI Tkinter

```bash

ssh er@10.10.0.225│                         Conda: Python 3.13 / PyTorch 2.6 + CUDA 12.4       ││   │   ├── slider_control.py      # Contrôle sliders

# Terminal 1 : robot

python3 bridge_pi_simple.py│                         GPU: NVIDIA RTX 4000 Ada (20 GB VRAM)               ││   │   ├── teleop_keyboard.py     # Contrôle clavier

# Terminal 2 : caméras

python3 pi_camera_server.py --cameras 0 3 --names cam0 cam3├─────────────────────────────────────────────────────────────────────────────┤│   │   ├── marker_follower.py     # Suivi ArUco

```

│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌──────────────┐       ││   │   ├── synthetic_data_collector.py      # Collecte données Gazebo v1

### Capture Données Réelles

```bash│  │ simple_gui  │  │slider_control│  │teleop_keyb. │  │ training/    │       ││   │   └── synthetic_data_collector_v2.py   # 🆕 Collecte v2 (multi-cam + domain rand)

cd ~/ros_jazzy/src/mycobot_R6A

/home/genji/miniconda/bin/python3 training/capture_real.py \│  │  (Tkinter)  │  │(joint_states)│  │  (clavier)  │  │ train.py     │       ││   │

  --output /tmp/real_dataset \

  --num-samples 2000 \│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  │ predict.py   │       ││   ├── scripts/

  --pi-host 10.10.0.225 \

  --settle-time 3.0 --speed 25 --limit-fraction 0.5│         │                │                │          │ capture_real  │       ││   │   ├── bridge_pi_simple.py    # ⭐ Script Pi (serveur TCP)

```

│         └────────────────┴────────────────┘          └──────┬───────┘       ││   │   ├── bridge_pi_standalone.py

### Entraînement Régression Directe (Conda)

```bash│                                   │                         │              ││   │   ├── synthetic_data_collector  # Wrapper ros2 run v1

cd ~/ros_jazzy/src/mycobot_R6A

│                          /to_robot (JSON)            TCP:5005 + 5006       ││   │   └── synthetic_data_collector_v2  # 🆕 Wrapper ros2 run v2

# Multi-view synthétique (MEILLEUR Phase 1 : 12.97° MAE)

/home/genji/miniconda/bin/python3 training/train.py \│                                   ▼                         ▼              ││   │

  --dataset /tmp/mycobot_synth_v2 \

  --multi-view --backbone resnet50 \│                         ┌─────────────────┐                                ││   └── launch/

  --epochs 150 --batch-size 16 --lr 1e-4

```│                         │   bridge_tour   │                                ││       ├── simple_gui.launch.py



### 🆕 Entraînement DREAM (Conda)│                         └────────┬────────┘                                ││       ├── slider_control.launch.py   # ⭐ Contrôle temps réel validé

```bash

cd ~/ros_jazzy/src/mycobot_R6A├──────────────────────────────────┼──────────────────────────────────────────┤│       ├── teleop_keyboard.launch.py



# 1. Conversion des données au format NDDS│                    RÉSEAU ETHERNET (10.10.0.x)                             ││       ├── rviz_sync.launch.py

/home/genji/miniconda/bin/python3 training/dream/convert_to_ndds.py \

  --input /tmp/mycobot_synth_v2 \├──────────────────────────────────┼──────────────────────────────────────────┤│       ├── marker_follow_full.launch.py

  --output /tmp/dream_data/synthetic \

  --source synth --cameras front right left top│                                  ▼                                          ││       ├── synthetic_data.launch.py       # Pipeline synthétique v1



# 2. Training VGG avec augmentation (RECOMMANDÉ)│            ┌─────────────────┐       ┌─────────────────┐                   ││       └── synthetic_data_v2.launch.py    # 🆕 Pipeline v2 (4 caméras + randomized.sdf)

/home/genji/miniconda/bin/python3 training/dream/train_dream_augmented.py \

  --data /tmp/dream_data/synthetic \│            │bridge_pi_simple │       │pi_camera_server │                   ││

  --arch vgg --epochs 25 --batch-size 32 --lr 0.0001

│            │  TCP:5005       │       │  TCP:5006       │                   │├── mycobot_description/           # 📦 Package URDF

# 3. Évaluation

/home/genji/miniconda/bin/python3 training/dream/evaluate_dream.py \│            └────────┬────────┘       └────────┬────────┘                   ││

  --weights training/checkpoints_dream/vgg_augmented_e25/best_network.pth \

  --data /tmp/dream_data/synthetic \│                     │                         │                            │├── mycobot_description/           # 📦 Package URDF

  --split val --max-samples 500 --visualize

│                     ▼                         ▼                            ││   ├── urdf/320_pi/               # Modèle 3D robot

# 4. Inférence sur une image

/home/genji/miniconda/bin/python3 training/dream/infer_dream.py \│            ┌─────────────────┐       ┌─────────────────┐                   ││   │   └── mycobot_pro_320_pi_gazebo.urdf  # URDF + 4 caméras + contrôleurs

  --model training/checkpoints_dream/vgg_augmented_e25/best_network.pth \

  --image /path/to/image.png│            │    pymycobot    │       │ Arducam USB ×2  │                   ││   ├── worlds/

```

│            │  /dev/ttyAMA0   │       │  cam0 + cam3    │                   ││   │   └── randomized.sdf         # 🆕 Monde Gazebo avec domain randomization

### Données Synthétiques (Gazebo)

```bash│            └────────┬────────┘       └─────────────────┘                   ││   ├── config/mycobot_320_pi.rviz # Config RViz

# v2 (4 cameras, domain randomization, 5000 samples)

env -i HOME=$HOME PATH="/usr/bin:/bin:/opt/ros/jazzy/bin" DISPLAY=$DISPLAY bash -c '│                     ▼                                                      ││   └── launch/

source /opt/ros/jazzy/setup.bash && 

source ~/ros_jazzy/install/setup.bash && │            ┌─────────────────┐                                             ││       ├── display.launch.py

ros2 launch mycobot_gateway synthetic_data_v2.launch.py \

  num_samples:=5000'│            │  MyCobot 320 Pi │                                             ││       └── gazebo_sim.launch.py   # Lancement Gazebo + bridges

```

│            └─────────────────┘                                             ││

---

│                                                                             │├── docs/                          # Documentation détaillée

## 📊 Comparaison des Modèles

│                     RASPBERRY PI (10.10.0.225)                             ││   ├── SYNTHETIC_DATA.md          # 🆕 Guide pipeline données synthétiques

### Phase 1 : Régression Directe (Données Synthétiques) ✅

└─────────────────────────────────────────────────────────────────────────────┘│   └── ...

| Configuration | Données | MAE moyenne |

|---|---|---|```│

| ResNet18, single-view | 1000 (v1) | 22.6° |

| ResNet50, single-view (front) | 5000 (v2) | 16.5° |├── training/                      # 🆕 Pipeline entraînement IA (feature/pose-training)

| **ResNet50, multi-view (4 cam)** | **5000 (v2)** | **12.97°** |

---│   ├── __init__.py

### Phase 1 : Régression Directe (Données Réelles) ❌

│   ├── dataset.py                 # MyCobotPoseDataset, MultiView, Merged, normalisation

| Configuration | Best MAE | vs Baseline | Problème |

|---|---|---|---|## 📦 Structure du Projet│   ├── model.py                   # PoseResNet + MultiViewPoseResNet (fusion 4 vues)

| Toutes approches testées | ~31.7° | -1° | Signal visuel insuffisant |

| Mean predictor (baseline) | 32.76° | — | Robot trop petit dans images |│   ├── train.py                   # v2: multi-view, domain rand, finetune, camera filter



### Phase 2 : DREAM Keypoint Detection (Synthétique) ✅```│   ├── predict.py                 # Inférence sur image(s)



| Modèle | Époques | Val Loss | Détection | Erreur médiane | <10px |mycobot_R6A/│   ├── capture_real.py            # 🆕 Capture images réelles via bridge Pi + OpenCV

|--------|---------|----------|-----------|----------------|-------|

| ResNet-H | 10/25 (tué) | 0.000305* | N/A | N/A | N/A |├── SESSION_RESUME.md              # 👈 CE FICHIER│   ├── README.md                  # Documentation pipeline

| VGG-base | 25 | 0.000438 | 96.1% | 3.1px | 75% |

| **VGG-aug** | **25** | **0.000667** | **96.6%** | **3.1px** | **78%** |├── DEVELOPMENT_SUMMARY.md         # Résumé technique détaillé│   └── requirements.txt           # Dépendances PyTorch



*ResNet-H instable (BN issue), seul epoch 1 valide.├── INDEX.md                       # Index documentation│



### Phase 2 : DREAM Per-Keypoint (VGG-aug) ✅├── README.md                      # README principal└── scripts/                       # Scripts shell utilitaires



| Keypoint | Détection | Moyenne | Médiane | <5px | <10px | <20px |│```

|----------|-----------|---------|---------|------|-------|-------|

| base | 100% | 2.9px | 2.8px | 100% | 100% | 100% |├── mycobot_gateway/               # 📦 Package ROS2 Principal

| link1 | 100% | 2.7px | 2.6px | 100% | 100% | 100% |

| link2 | 100% | 2.7px | 2.6px | 100% | 100% | 100% |│   ├── mycobot_gateway/---

| link3 | 99% | 11.0px | 5.6px | 45% | 74% | 88% |

| link4 | 96% | 19.2px | 6.4px | 39% | 65% | 80% |│   │   ├── bridge_tour.py         # Client TCP vers Pi

| link5 | 95% | 27.4px | 8.8px | 26% | 53% | 70% |

| link6 | 86% | 28.9px | 10.1px | 19% | 50% | 69% |│   │   ├── simple_gui.py          # GUI Tkinter## 🚀 DÉMARRAGE RAPIDE

| **TOTAL** | **97%** | **13.1px** | **3.1px** | **63%** | **78%** | **87%** |

│   │   ├── slider_control.py      # Contrôle sliders temps réel

### Phase 2 : DREAM Sim-to-Real ⚠️

│   │   └── synthetic_data_collector_v2.py  # Collecte données Gazebo### ⚠️ PRÉREQUIS CRITIQUE - Éviter le conflit Conda

| Métrique | Synthétique | Réel |

|----------|-------------|------|│   ├── scripts/

| Détection | 97% | ~26% |

| Belief map peaks | 0.5–1.0 | 0.02–0.25 |│   │   ├── bridge_pi_simple.py    # Script Pi (serveur TCP robot)```bash



---│   │   └── pi_camera_server.py    # Script Pi (serveur TCP caméras)# TOUJOURS exécuter avant ROS2 (conflit Python 3.13 vs 3.12)



## 🔑 Points Importants│   └── launch/                    # Fichiers launch ROS2conda deactivate



1. **Conda vs ROS2** : Toujours `conda deactivate` avant ROS2. Pour le training ML, utiliser `/home/genji/miniconda/bin/python3`.│

2. **Dataset synthétique** : `/tmp/mycobot_synth_v2/` — 5000 poses × 4 caméras = 20K images

3. **Dataset réel** : `/tmp/real_dataset/` — 2000 poses × 2 caméras = 4000 images├── mycobot_description/           # 📦 URDF + Gazebo# OU utiliser la commande "propre" :

4. **Données NDDS** : `/tmp/dream_data/synthetic/` — 20K frames DREAM format

5. **DREAM installé** : `/tmp/DREAM/` (pip install -e .)│   ├── urdf/320_pi/               # Modèle 3D (URDF + 4 caméras Gazebo)env -i HOME=$HOME PATH="/usr/bin:/bin:/opt/ros/jazzy/bin" DISPLAY=$DISPLAY bash

6. **Meilleur modèle DREAM** : `checkpoints_dream/vgg_augmented_e25/best_network.pth`

7. **Problème #1** : domain gap sim-to-real pour les keypoints│   └── worlds/randomized.sdf      # Monde Gazebo avec domain randomization```

8. **Le modèle DREAM fonctionne** : 97% détection, 3.1px médiane sur synthétique

│

---

├── training/                      # 📦 Pipeline ML/IA### Étape 1 : Démarrer le Bridge sur le Pi

## 🔌 Configuration Réseau

│   ├── dataset.py                 # Datasets: single/multi-view + PerImageNormalize

| Machine | IP | Port | Rôle |

|---------|-----|------|------|│   ├── model.py                   # PoseResNet + MultiViewPoseResNet```bash

| PC Tour | 10.10.0.115 | — | Calcul, GUI, RViz, Training |

| Raspberry Pi | 10.10.0.225 | 5005 (robot), 5006 (caméras) | Bridge robot + caméras |│   ├── train.py                   # Training: multi-view, finetune, auto-views# SSH vers le Pi



---│   ├── predict.py                 # Inférence sur image(s)ssh er@10.10.0.225



## 🧪 Tests Validés│   ├── capture_real.py            # Capture réelle (FK safety, bridge + camera)



### Session 26/03/2026 — Bridge ROS2│   └── checkpoints_*/             # Modèles entraînés# Lancer le bridge

| Test | Résultat |

|------|----------|│python3 bridge_pi_simple.py

| Connexion TCP Tour ↔ Pi | ✅ |

| Commandes JSON (send_angles, get_angles, go_home) | ✅ |├── scripts/                       # Scripts shell utilitaires```

| RViz visualisation + slider_control | ✅ |

└── docs/                          # Documentation détaillée

### Session 31/03/2026 — Gazebo + Synthétique

| Test | Résultat |```### Étape 2 : Lancer le contrôle sur le PC Tour

|------|----------|

| Gazebo Harmonic 4 caméras | ✅ |

| Collecte 5000 × 4 = 20K images | ✅ |

| ResNet50 multi-view : 12.97° MAE | ✅ |---```bash



### Sessions 01-02/04/2026 — Données Réelles# Option A : Slider Control (RECOMMANDÉ - testé et validé)

| Test | Résultat |

|------|----------|## 🚀 COMMANDES UTILESenv -i HOME=$HOME PATH="/usr/bin:/bin:/opt/ros/jazzy/bin" DISPLAY=$DISPLAY bash -c '

| Camera server Pi TCP:5006 | ✅ |

| Capture 2000 poses (0 collisions) | ✅ |source /opt/ros/jazzy/setup.bash && 

| Training réel (régression directe) | ❌ Baseline |

| Diagnostic corrélation | ✅ (r=0.004) |### Démarrage Bridge Pisource ~/ros_jazzy/src/mycobot_R6A/install/setup.bash && 



### Session 03/04/2026 — DREAM```bashros2 launch mycobot_gateway slider_control.launch.py'

| Test | Résultat |

|------|----------|ssh er@10.10.0.225

| Conversion NDDS (20K frames, 0 skip) | ✅ |

| FK + projection caméra vérifiés | ✅ |# Terminal 1 : robot# Option B : GUI Simple

| VGG-aug : 97% détection, 3.1px médiane | ✅ |

| ResNet-H : BN instable | ❌ |python3 bridge_pi_simple.pyros2 launch mycobot_gateway simple_gui.launch.py

| Sim-to-real : ~26% détection | ⚠️ |

| Git commit `0e452ece` | ✅ |# Terminal 2 : caméras



---python3 pi_camera_server.py --cameras 0 3 --names cam0 cam3# Option C : Contrôle clavier



## 📊 Comparaison Phase 1 vs Phase 2 — Détaillée```ros2 launch mycobot_gateway teleop_keyboard.launch.py



| Configuration | Données | MAE / Métrique | J1 | J2 | J3 | J4 | J5 | J6 |```

|---|---|---|---|---|---|---|---|---|

| ResNet18 single-view | 1000 (v1) | 22.6° | 10.8° | 15.4° | 18.3° | 25.0° | 33.0° | 33.2° |### Capture Données Réelles

| ResNet50 single-view | 5000 (v2) | 16.5° | 7.2° | 9.6° | 11.3° | 16.0° | 28.4° | 26.4° |

| **ResNet50 multi-view** | **5000 (v2)** | **12.97°** | **6.4°** | **9.1°** | **8.5°** | **10.8°** | **17.1°** | **25.9°** |```bash### 🆕 Données Synthétiques (Gazebo)



---cd ~/ros_jazzy/src/mycobot_R6A



## 🚧 TODO - Prochaines Étapes/home/genji/miniconda/bin/python3 training/capture_real.py \```bash



### Priorité Haute  --output /tmp/real_dataset \# === v1 (single camera, 1000 samples) ===

- [x] Bridge ROS2 Tour ↔ Pi ✅ (26/03)

- [x] Simulation Gazebo 4 caméras ✅ (31/03)  --num-samples 2000 \# Branche feature/synthetic-data

- [x] Domain randomization + collecte 20K images ✅ (31/03)

- [x] Multi-view ResNet50 → 12.97° MAE ✅ (31/03)  --pi-host 10.10.0.225 \env -i HOME=$HOME PATH="/usr/bin:/bin:/opt/ros/jazzy/bin" DISPLAY=$DISPLAY bash -c '

- [x] Capture 2000 poses réelles ✅ (02/04)

- [x] DREAM keypoint detection → 97% détection ✅ (03/04)  --settle-time 3.0 --speed 25 --limit-fraction 0.5source /opt/ros/jazzy/setup.bash && 

- [ ] **Réduire le domain gap sim-to-real** (self-supervised / domain rand avancée)

- [ ] **Pick-and-place simulation** (DREAM + MoveIt2 + Gazebo)```source ~/ros_jazzy/src/mycobot_R6A/install/setup.bash && 

- [ ] **Bench test robot réel** (confronter sim vs réel)

ros2 launch mycobot_gateway synthetic_data.launch.py \

### Priorité Moyenne

- [ ] Nœud ROS2 d'inférence DREAM temps réel### Entraînement (Conda)  num_samples:=1000 \

- [ ] Calibration caméra réelle (intrinsèques + extrinsèques)

- [ ] Fine-tuning VGG-aug sur données réelles auto-annotées```bash  output_dir:=/tmp/mycobot_synth_dataset \



### Priorité Bassecd ~/ros_jazzy/src/mycobot_R6A  settle_time:=2.0'

- [ ] Interface web (option future)

- [ ] Multi-robot coordination



---# Multi-view synthétique (MEILLEUR : 12.97° MAE)# === v2 (4 cameras, domain randomization, 5000 samples) ===



## 📚 Documentation Complémentaire/home/genji/miniconda/bin/python3 training/train.py \# Branche feature/pose-training



| Fichier | Description |  --dataset /tmp/mycobot_synth_v2 \env -i HOME=$HOME PATH="/usr/bin:/bin:/opt/ros/jazzy/bin" DISPLAY=$DISPLAY bash -c '

|---------|-------------|

| `DEVELOPMENT_SUMMARY.md` | Résumé technique détaillé (inclut DREAM) |  --multi-view --backbone resnet50 \source /opt/ros/jazzy/setup.bash && 

| `INDEX.md` | Index de toute la documentation |

| `docs/QUICKSTART.md` | Guide démarrage rapide |  --epochs 150 --batch-size 16 --lr 1e-4source ~/ros_jazzy/install/setup.bash && 

| `docs/ARCHITECTURE.md` | Architecture système |

| `docs/DEPLOYMENT.md` | Guide de déploiement |ros2 launch mycobot_gateway synthetic_data_v2.launch.py \

| `docs/SYNTHETIC_DATA.md` | Pipeline données synthétiques |

| `training/README.md` | Documentation pipeline ML |# Multi-view réel (cam0 + cam3)  num_samples:=5000'

| `training/dream/README.md` | 🆕 Documentation module DREAM |

/home/genji/miniconda/bin/python3 training/train.py \

---

  --dataset /tmp/real_dataset \# Résultat v2 : /tmp/mycobot_synth_v2/

## 🔗 Liens Utiles

  --multi-view --views cam0 cam3 --backbone resnet50 \#   ├── images/front/000000.png ... 004999.png

- **GitHub** : https://github.com/ABMI-software/mycobot_320pi_R6A

- **ROS2 Jazzy** : https://docs.ros.org/en/jazzy/  --lr 1e-3 --epochs 300 --batch-size 32 --freeze-epochs 3#   ├── images/right/000000.png ... 004999.png

- **pymycobot** : https://github.com/elephantrobotics/pymycobot

- **DREAM (NVlabs)** : https://github.com/NVlabs/DREAM```#   ├── images/left/000000.png ... 004999.png



---#   ├── images/top/000000.png ... 004999.png  (4×5000 = 20,000 images)



*Ce fichier est le point de départ pour les prochaines sessions de développement.*  ---#   └── labels.csv  (index, j1-j6_rad, j1-j6_deg, camera, image_path)

*Dernière mise à jour : 3 avril 2026*

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
