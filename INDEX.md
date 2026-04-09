# 📚 Index de Documentation - MyCobot Gateway Bridge# 📚 Index de Documentation - MyCobot Gateway Bridge# 📚 Index de Documentation - MyCobot Gateway Bridge



Bienvenue dans la documentation du bridge ROS2 pour MyCobot !



## 🎯 Par où commencer ?Bienvenue dans la documentation du bridge ROS2 pour MyCobot !Bienvenue dans la documentation du bridge ROS2 pour MyCobot !



### Première utilisation

👉 **[docs/QUICKSTART.md](docs/QUICKSTART.md)** — Démarrage en 3 étapes

## 🎯 Par où commencer ?## 🎯 Par où commencer ?

### Système distribué (Vision + Robot)

👉 **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)** — Architecture Tour/Pi  

👉 **[docs/DEPLOYMENT.md](docs/DEPLOYMENT.md)** — Guide de déploiement complet

### Première utilisation### Première utilisation

### Pipeline IA / Pose Estimation

👉 **[training/README.md](training/README.md)** — Pipeline ML (régression directe)  👉 **[docs/QUICKSTART.md](docs/QUICKSTART.md)** — Démarrage en 3 étapes👉 **[QUICKSTART.md](QUICKSTART.md)** — Démarrage en 3 étapes

👉 **[training/dream/README.md](training/dream/README.md)** — 🆕 Module DREAM (keypoint detection + PnP)



### Problème à résoudre ?

👉 **[scripts/diagnose.sh](scripts/diagnose.sh)** — Script de diagnostic automatique### 🆕 Système distribué (Vision + Robot)### 🆕 Système distribué (Vision + Robot)



### Besoin d'aide ?👉 **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)** — Architecture Tour/Pi👉 **[ARCHITECTURE.md](ARCHITECTURE.md)** — Architecture Tour/Pi

👉 **[mycobot_gateway/README.md](mycobot_gateway/README.md)** — Documentation complète

👉 **[docs/DEPLOYMENT.md](docs/DEPLOYMENT.md)** — Guide de déploiement complet👉 **[DEPLOYMENT.md](DEPLOYMENT.md)** — Guide de déploiement complet

---



## 📁 Structure du Projet

### Problème à résoudre ?### Problème à résoudre ?

```

mycobot_R6A/👉 **[scripts/diagnose.sh](scripts/diagnose.sh)** — Script de diagnostic automatique👉 **[diagnose.sh](diagnose.sh)** — Script de diagnostic automatique

├── INDEX.md                      # 📖 Ce fichier

├── SESSION_RESUME.md             # ⭐ Point de départ sessions (état actuel)

├── DEVELOPMENT_SUMMARY.md        # Résumé technique détaillé

├── bridge_pi_debug.py            # ⭐ Serveur TCP (à copier sur Pi)### Besoin d'aide ?### Besoin d'aide ?

│

├── mycobot_gateway/              # 📦 Package ROS2👉 **[mycobot_gateway/README.md](mycobot_gateway/README.md)** — Documentation complète👉 **[mycobot_gateway/README.md](mycobot_gateway/README.md)** — Documentation complète

│   ├── mycobot_gateway/

│   │   ├── __init__.py

│   │   ├── bridge_tour.py        # ⭐ Client TCP (Tour)

│   │   ├── robot_commander.py------

│   │   └── ...

│   ├── scripts/

│   │   ├── bridge_pi_simple.py

│   │   ├── bridge_pi_standalone.py## 📁 Structure du Projet## 🆕 Système Distribué (v0.1.0)

│   │   └── pi_camera_server.py

│   ├── launch/

│   ├── setup.py

│   ├── package.xml```### Architecture

│   └── README.md

│mycobot_R6A/```

├── mycobot_description/          # 📦 URDF + Gazebo

│   ├── urdf/320_pi/              # Modèles 3D (URDF + 4 caméras Gazebo)├── INDEX.md                      # 📖 Ce fichierTour (PC)                          Raspberry Pi

│   ├── worlds/randomized.sdf     # Monde Gazebo avec domain randomization

│   ├── config/├── bridge_pi_debug.py            # ⭐ Serveur TCP (à copier sur Pi)┌─────────────────────┐            ┌─────────────────────┐

│   └── launch/

│││ • camera_publisher  │            │                     │

├── training/                     # 📦 Pipeline ML/IA

│   ├── model.py                  # PoseResNet + MultiViewPoseResNet├── mycobot_gateway/              # 📦 Package ROS2│ • marker_detector   │──TCP/IP──▶│ • bridge_pi         │

│   ├── dataset.py                # Datasets single/multi-view

│   ├── train.py                  # Training régression directe│   ├── mycobot_gateway/│ • robot_commander   │            │ • MyCobot control   │

│   ├── predict.py                # Inférence

│   ├── capture_real.py           # Capture images réelles│   │   ├── __init__.py│ • bridge_tour       │◀──────────│                     │

│   ├── dream/                    # 🆕 Module DREAM keypoint detection

│   │   ├── mycobot_fk.py         # Forward kinematics + projection│   │   ├── bridge_tour.py        # ⭐ Client TCP (Tour)└─────────────────────┘            └─────────────────────┘

│   │   ├── convert_to_ndds.py    # Conversion format NDDS

│   │   ├── train_dream.py        # Training DREAM│   │   ├── robot_commander.py```

│   │   ├── train_dream_augmented.py  # Training avec augmentation

│   │   ├── evaluate_dream.py     # Évaluation par keypoint│   │   └── command_executor_pi.py

│   │   ├── infer_dream.py        # Inférence + PnP

│   │   ├── visualize_ndds.py     # Visualisation│   ├── scripts/### Nouveaux Nodes (Tour)

│   │   └── manip_configs/        # Config YAML keypoints

│   └── checkpoints_*/            # Modèles entraînés│   │   ├── bridge_tour           # Wrapper exécutable| Node | Description |

│

├── docs/                         # 📚 Documentation│   │   ├── bridge_pi_standalone.py|------|-------------|

│   ├── QUICKSTART.md

│   ├── ROBOT_QUICKSTART.md│   │   └── ...| `camera_publisher` | Capture caméra USB |

│   ├── ARCHITECTURE.md

│   ├── DEPLOYMENT.md│   ├── launch/| `marker_detector` | Détection ArUco + transformation |

│   ├── SYNTHETIC_DATA.md

│   ├── TEST_COMPLET.md│   ├── setup.py| `robot_commander` | Interface commandes interactive |

│   ├── TEST_ROBOT_PROCEDURE.md

│   ├── SESSION_TEST.md│   ├── package.xml

│   ├── SUMMARY.md

│   ├── DEBUG_CONNECTION_GUIDE.md│   └── README.md### Launch Files

│   ├── BRIDGE_PI_UPGRADE_GUIDE.md

│   ├── DIAGNOSTIC_ROBOT.txt│```bash

│   └── ROBOT_TESTS_GUIDE.txt

│├── docs/                         # 📚 Documentation# Suivi de marqueur complet

├── datasets/                     # Datasets (Git LFS)

│   ├── synthetic_dataset/│   ├── QUICKSTART.md             # Démarrage rapideros2 launch mycobot_gateway marker_follow.launch.py

│   └── real_dataset/

││   ├── ROBOT_QUICKSTART.md       # Guide rapide robot

├── scripts/                      # 🔧 Scripts utilitaires

│   ├── quick_commands.sh│   ├── ARCHITECTURE.md           # Architecture système# Bridge seul

│   ├── diagnose.sh

│   ├── diagnostic_full.sh│   ├── DEPLOYMENT.md             # Guide de déploiementros2 launch mycobot_gateway bridge_only.launch.py

│   ├── check_pi_bridge.sh

│   ├── test_bridge.sh│   ├── TEST_COMPLET.md           # Procédure de test complète

│   └── robot_test_interactive.sh

││   ├── TEST_ROBOT_PROCEDURE.md   # Procédure détaillée# Commander interactif

├── build/                        # 🔨 Fichiers de build (généré)

├── install/                      # 📦 Package installé (généré)│   ├── SESSION_TEST.md           # Log de session de testros2 launch mycobot_gateway commander.launch.py

└── log/                          # 📝 Logs colcon (généré)

```│   ├── SUMMARY.md                # Résumé du projet```



---│   ├── DEBUG_CONNECTION_GUIDE.md # Guide de débogage



## ⚙️ Configuration│   ├── BRIDGE_PI_UPGRADE_GUIDE.md---



| Paramètre | Valeur |│   ├── DIAGNOSTIC_ROBOT.txt

|-----------|--------|

| IP Raspberry Pi | `10.10.0.225` |│   └── ROBOT_TESTS_GUIDE.txt## 📖 Documentation disponible

| Port TCP (robot) | `5005` |

| Port TCP (caméras) | `5006` |│

| Port série robot | `/dev/ttyAMA0` |

| GPU Tour | NVIDIA RTX 4000 Ada (20 GB) |├── scripts/                      # 🔧 Scripts utilitaires### 🚀 Guides utilisateur

| Conda Python | 3.13.5 |

| ROS2 Python | 3.12 |│   ├── quick_commands.sh         # Commandes rapides (source)



---│   ├── diagnose.sh               # Diagnostic complet| Fichier | Description |



## 📋 Documentation par catégorie│   ├── diagnostic_full.sh        # Diagnostic étendu|---------|-------------|



### 🚀 Démarrage│   ├── check_pi_bridge.sh        # Vérification Pi| **[QUICKSTART.md](QUICKSTART.md)** | Démarrage rapide |

| Document | Description |

|----------|-------------|│   ├── test_bridge.sh            # Test du bridge| **[ARCHITECTURE.md](ARCHITECTURE.md)** | 🆕 Architecture distribuée |

| [docs/QUICKSTART.md](docs/QUICKSTART.md) | Guide de démarrage rapide |

| [docs/ROBOT_QUICKSTART.md](docs/ROBOT_QUICKSTART.md) | Démarrage pour le robot |│   └── robot_test_interactive.sh # Test interactif| **[DEPLOYMENT.md](DEPLOYMENT.md)** | 🆕 Guide déploiement |

| [mycobot_gateway/README.md](mycobot_gateway/README.md) | README du package |

│| **[ROBOT_QUICKSTART.md](ROBOT_QUICKSTART.md)** | Tests robot réel |

### 🏗️ Architecture

| Document | Description |├── build/                        # 🔨 Fichiers de build (généré)| **[SUMMARY.md](SUMMARY.md)** | Résumé complet |

|----------|-------------|

| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | Architecture du système |├── install/                      # 📦 Package installé (généré)

| [docs/DEPLOYMENT.md](docs/DEPLOYMENT.md) | Guide de déploiement |

| [docs/SUMMARY.md](docs/SUMMARY.md) | Résumé du projet |└── log/                          # 📝 Logs colcon (généré)### 🔧 Scripts utiles



### 🧠 Intelligence Artificielle```

| Document | Description |

|----------|-------------|| Script | Commande |

| [training/README.md](training/README.md) | Pipeline ML (régression directe) |

| [training/dream/README.md](training/dream/README.md) | 🆕 Module DREAM (keypoint detection) |---|--------|----------|

| [docs/SYNTHETIC_DATA.md](docs/SYNTHETIC_DATA.md) | Pipeline données synthétiques |

| **diagnose.sh** | `./diagnose.sh` |

### 🧪 Tests

| Document | Description |## 🆕 Système Distribué (v0.1.0)| **test_bridge.sh** | `./test_bridge.sh` |

|----------|-------------|

| [docs/TEST_COMPLET.md](docs/TEST_COMPLET.md) | Procédure de test complète || **quick_commands.sh** | `source quick_commands.sh` |

| [docs/TEST_ROBOT_PROCEDURE.md](docs/TEST_ROBOT_PROCEDURE.md) | Procédure détaillée |

| [docs/SESSION_TEST.md](docs/SESSION_TEST.md) | Log de session de test |### Architecture

| [docs/ROBOT_TESTS_GUIDE.txt](docs/ROBOT_TESTS_GUIDE.txt) | Guide de tests robot |

```---

### 🐛 Débogage

| Document | Description |Tour (PC)                          Raspberry Pi

|----------|-------------|

| [docs/DEBUG_CONNECTION_GUIDE.md](docs/DEBUG_CONNECTION_GUIDE.md) | Guide de débogage connexion |┌─────────────────────┐            ┌─────────────────────┐## 📂 Structure

| [docs/DIAGNOSTIC_ROBOT.txt](docs/DIAGNOSTIC_ROBOT.txt) | Diagnostic robot |

| [docs/BRIDGE_PI_UPGRADE_GUIDE.md](docs/BRIDGE_PI_UPGRADE_GUIDE.md) | Mise à jour bridge Pi |│ • camera_publisher  │            │                     │



### 📊 Suivi Projet│ • marker_detector   │──TCP/IP──▶│ • bridge_pi         │```

| Document | Description |

|----------|-------------|│ • robot_commander   │            │ • MyCobot control   │mycobot_gateway/

| [SESSION_RESUME.md](SESSION_RESUME.md) | ⭐ Point de départ — état actuel + prochaines étapes |

| [DEVELOPMENT_SUMMARY.md](DEVELOPMENT_SUMMARY.md) | Résumé technique complet (toutes phases) |│ • bridge_tour       │◀──────────│                     │├── mycobot_gateway/

| [CHANGELOG.md](CHANGELOG.md) | Historique des changements |

└─────────────────────┘            └─────────────────────┘│   ├── bridge_tour.py

---

```│   ├── robot_commander.py      # 🆕

## 🔧 Scripts utilitaires

│   └── vision/                 # 🆕

| Script | Usage |

|--------|-------|### Nouveaux Nodes (Tour)│       ├── camera_publisher.py

| `source scripts/quick_commands.sh` | Charge les commandes rapides |

| `./scripts/diagnose.sh` | Diagnostic complet || Node | Description |│       └── marker_detector.py

| `./scripts/diagnostic_full.sh` | Diagnostic étendu |

| `./scripts/check_pi_bridge.sh` | Vérifie le bridge Pi ||------|-------------|├── scripts/

| `./scripts/test_bridge.sh` | Test du bridge |

| `./scripts/robot_test_interactive.sh` | Test interactif || `camera_publisher` | Capture caméra USB |│   └── bridge_pi_standalone.py # 🆕 Pour Pi



---| `marker_detector` | Détection ArUco + transformation |├── launch/                     # 🆕



## ❓ FAQ| `robot_commander` | Interface commandes interactive |│   ├── marker_follow.launch.py



**Q: Comment lancer le bridge ?**│   ├── bridge_only.launch.py

```bash

# Sur Pi### Launch Files│   └── commander.launch.py

ssh er@10.10.0.225

python3 bridge_pi_simple.py```bash└── setup.py



# Sur Tour# Suivi de marqueur complet```

conda deactivate

source /opt/ros/jazzy/setup.bashros2 launch mycobot_gateway marker_follow.launch.py

source ~/ros_jazzy/src/mycobot_R6A/install/setup.bash

ros2 run mycobot_gateway bridge_tour---

```

# Bridge seul

**Q: Puis-je utiliser ROS2 avec conda activé ?**  

**R:** Non ! Toujours faire `conda deactivate` avant ROS2.ros2 launch mycobot_gateway bridge_only.launch.py**Version:** 0.1.0 | **Mise à jour:** 26 mars 2026 | **ROS2:** Jazzy



**Q: Comment entraîner le modèle DREAM ?**  

**R:** Voir [training/dream/README.md](training/dream/README.md) pour le guide complet.# Commander interactif

ros2 launch mycobot_gateway commander.launch.py

**Q: Quel est le meilleur modèle actuel ?**  ```

**R:** VGG-aug DREAM — 97% détection, 3.1px médiane sur synthétique. Voir `checkpoints_dream/vgg_augmented_e25/`.

---

---

## ⚙️ Configuration

**Version :** 1.6.0  

**Dernière mise à jour :** 3 avril 2026  | Paramètre | Valeur |

**Auteur :** José BERNARDO|-----------|--------|

| IP Raspberry Pi | `10.10.0.218` |
| Port TCP | `5005` |
| ROS_DOMAIN_ID | `10` |
| Port série robot | `/dev/ttyAMA0` |

---

## 📋 Documentation par catégorie

### 🚀 Démarrage
| Document | Description |
|----------|-------------|
| [docs/QUICKSTART.md](docs/QUICKSTART.md) | Guide de démarrage rapide |
| [docs/ROBOT_QUICKSTART.md](docs/ROBOT_QUICKSTART.md) | Démarrage pour le robot |
| [mycobot_gateway/README.md](mycobot_gateway/README.md) | README du package |

### 🏗️ Architecture
| Document | Description |
|----------|-------------|
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | Architecture du système |
| [docs/DEPLOYMENT.md](docs/DEPLOYMENT.md) | Guide de déploiement |
| [docs/SUMMARY.md](docs/SUMMARY.md) | Résumé du projet |

### 🧪 Tests
| Document | Description |
|----------|-------------|
| [docs/TEST_COMPLET.md](docs/TEST_COMPLET.md) | Procédure de test complète |
| [docs/TEST_ROBOT_PROCEDURE.md](docs/TEST_ROBOT_PROCEDURE.md) | Procédure détaillée |
| [docs/SESSION_TEST.md](docs/SESSION_TEST.md) | Log de session de test |
| [docs/ROBOT_TESTS_GUIDE.txt](docs/ROBOT_TESTS_GUIDE.txt) | Guide de tests robot |

### 🐛 Débogage
| Document | Description |
|----------|-------------|
| [docs/DEBUG_CONNECTION_GUIDE.md](docs/DEBUG_CONNECTION_GUIDE.md) | Guide de débogage connexion |
| [docs/DIAGNOSTIC_ROBOT.txt](docs/DIAGNOSTIC_ROBOT.txt) | Diagnostic robot |
| [docs/BRIDGE_PI_UPGRADE_GUIDE.md](docs/BRIDGE_PI_UPGRADE_GUIDE.md) | Mise à jour bridge Pi |

---

## 🔧 Scripts utilitaires

| Script | Usage |
|--------|-------|
| `source scripts/quick_commands.sh` | Charge les commandes rapides |
| `./scripts/diagnose.sh` | Diagnostic complet |
| `./scripts/diagnostic_full.sh` | Diagnostic étendu |
| `./scripts/check_pi_bridge.sh` | Vérifie le bridge Pi |
| `./scripts/test_bridge.sh` | Test du bridge |
| `./scripts/robot_test_interactive.sh` | Test interactif |

---

## ❓ FAQ

**Q: Comment lancer le bridge ?**
```bash
# Sur Pi
source /opt/ros/galactic/setup.bash
python3 bridge_pi_debug.py

# Sur Tour
conda deactivate
source /opt/ros/jazzy/setup.bash
cd /home/genji/ros_jazzy/src/mycobot_R6A
source install/setup.bash
export ROS_DOMAIN_ID=10
ros2 run mycobot_gateway bridge_tour
```

**Q: Puis-je utiliser ROS2 avec conda activé ?**
**R:** Non ! Toujours faire `conda deactivate` avant ROS2

**Q: Comment envoyer une commande au robot ?**
```bash
ros2 topic pub --once /to_robot std_msgs/msg/String "{data: 'ping'}"
```

---

## 📊 Commandes disponibles

| Commande | Description | Exemple |
|----------|-------------|---------|
| `ping` | Test connexion | `ping` → `pong` |
| `status` | État du robot | `status` → `status:ok` |
| `get_angles` | Lire les angles | `get_angles` → `angles:[0,0,0,0,0,0]` |
| `set_led:R,G,B` | Changer la LED | `set_led:255,0,0` (rouge) |
| `go_home:SPEED` | Position zéro | `go_home:20` |
| `set_angle:J,A,S` | Bouger un joint | `set_angle:1,30,20` |
| `set_angles:A1,A2,A3,A4,A5,A6:S` | Tous les joints | `set_angles:0,0,0,0,0,0:20` |

---

**Version** : 0.0.1  
**Dernière mise à jour** : 26 mars 2026  
**Auteur** : José BERNARDO
