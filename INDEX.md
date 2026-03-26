# 📚 Index de Documentation - MyCobot Gateway Bridge# 📚 Index de Documentation - MyCobot Gateway Bridge



Bienvenue dans la documentation du bridge ROS2 pour MyCobot !Bienvenue dans la documentation du bridge ROS2 pour MyCobot !



## 🎯 Par où commencer ?## 🎯 Par où commencer ?



### Première utilisation### Première utilisation

👉 **[docs/QUICKSTART.md](docs/QUICKSTART.md)** — Démarrage en 3 étapes👉 **[QUICKSTART.md](QUICKSTART.md)** — Démarrage en 3 étapes



### 🆕 Système distribué (Vision + Robot)### 🆕 Système distribué (Vision + Robot)

👉 **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)** — Architecture Tour/Pi👉 **[ARCHITECTURE.md](ARCHITECTURE.md)** — Architecture Tour/Pi

👉 **[docs/DEPLOYMENT.md](docs/DEPLOYMENT.md)** — Guide de déploiement complet👉 **[DEPLOYMENT.md](DEPLOYMENT.md)** — Guide de déploiement complet



### Problème à résoudre ?### Problème à résoudre ?

👉 **[scripts/diagnose.sh](scripts/diagnose.sh)** — Script de diagnostic automatique👉 **[diagnose.sh](diagnose.sh)** — Script de diagnostic automatique



### Besoin d'aide ?### Besoin d'aide ?

👉 **[mycobot_gateway/README.md](mycobot_gateway/README.md)** — Documentation complète👉 **[mycobot_gateway/README.md](mycobot_gateway/README.md)** — Documentation complète



------



## 📁 Structure du Projet## 🆕 Système Distribué (v0.1.0)



```### Architecture

mycobot_R6A/```

├── INDEX.md                      # 📖 Ce fichierTour (PC)                          Raspberry Pi

├── bridge_pi_debug.py            # ⭐ Serveur TCP (à copier sur Pi)┌─────────────────────┐            ┌─────────────────────┐

││ • camera_publisher  │            │                     │

├── mycobot_gateway/              # 📦 Package ROS2│ • marker_detector   │──TCP/IP──▶│ • bridge_pi         │

│   ├── mycobot_gateway/│ • robot_commander   │            │ • MyCobot control   │

│   │   ├── __init__.py│ • bridge_tour       │◀──────────│                     │

│   │   ├── bridge_tour.py        # ⭐ Client TCP (Tour)└─────────────────────┘            └─────────────────────┘

│   │   ├── robot_commander.py```

│   │   └── command_executor_pi.py

│   ├── scripts/### Nouveaux Nodes (Tour)

│   │   ├── bridge_tour           # Wrapper exécutable| Node | Description |

│   │   ├── bridge_pi_standalone.py|------|-------------|

│   │   └── ...| `camera_publisher` | Capture caméra USB |

│   ├── launch/| `marker_detector` | Détection ArUco + transformation |

│   ├── setup.py| `robot_commander` | Interface commandes interactive |

│   ├── package.xml

│   └── README.md### Launch Files

│```bash

├── docs/                         # 📚 Documentation# Suivi de marqueur complet

│   ├── QUICKSTART.md             # Démarrage rapideros2 launch mycobot_gateway marker_follow.launch.py

│   ├── ROBOT_QUICKSTART.md       # Guide rapide robot

│   ├── ARCHITECTURE.md           # Architecture système# Bridge seul

│   ├── DEPLOYMENT.md             # Guide de déploiementros2 launch mycobot_gateway bridge_only.launch.py

│   ├── TEST_COMPLET.md           # Procédure de test complète

│   ├── TEST_ROBOT_PROCEDURE.md   # Procédure détaillée# Commander interactif

│   ├── SESSION_TEST.md           # Log de session de testros2 launch mycobot_gateway commander.launch.py

│   ├── SUMMARY.md                # Résumé du projet```

│   ├── DEBUG_CONNECTION_GUIDE.md # Guide de débogage

│   ├── BRIDGE_PI_UPGRADE_GUIDE.md---

│   ├── DIAGNOSTIC_ROBOT.txt

│   └── ROBOT_TESTS_GUIDE.txt## 📖 Documentation disponible

│

├── scripts/                      # 🔧 Scripts utilitaires### 🚀 Guides utilisateur

│   ├── quick_commands.sh         # Commandes rapides (source)

│   ├── diagnose.sh               # Diagnostic complet| Fichier | Description |

│   ├── diagnostic_full.sh        # Diagnostic étendu|---------|-------------|

│   ├── check_pi_bridge.sh        # Vérification Pi| **[QUICKSTART.md](QUICKSTART.md)** | Démarrage rapide |

│   ├── test_bridge.sh            # Test du bridge| **[ARCHITECTURE.md](ARCHITECTURE.md)** | 🆕 Architecture distribuée |

│   └── robot_test_interactive.sh # Test interactif| **[DEPLOYMENT.md](DEPLOYMENT.md)** | 🆕 Guide déploiement |

│| **[ROBOT_QUICKSTART.md](ROBOT_QUICKSTART.md)** | Tests robot réel |

├── build/                        # 🔨 Fichiers de build (généré)| **[SUMMARY.md](SUMMARY.md)** | Résumé complet |

├── install/                      # 📦 Package installé (généré)

└── log/                          # 📝 Logs colcon (généré)### 🔧 Scripts utiles

```

| Script | Commande |

---|--------|----------|

| **diagnose.sh** | `./diagnose.sh` |

## 🆕 Système Distribué (v0.1.0)| **test_bridge.sh** | `./test_bridge.sh` |

| **quick_commands.sh** | `source quick_commands.sh` |

### Architecture

```---

Tour (PC)                          Raspberry Pi

┌─────────────────────┐            ┌─────────────────────┐## 📂 Structure

│ • camera_publisher  │            │                     │

│ • marker_detector   │──TCP/IP──▶│ • bridge_pi         │```

│ • robot_commander   │            │ • MyCobot control   │mycobot_gateway/

│ • bridge_tour       │◀──────────│                     │├── mycobot_gateway/

└─────────────────────┘            └─────────────────────┘│   ├── bridge_tour.py

```│   ├── robot_commander.py      # 🆕

│   └── vision/                 # 🆕

### Nouveaux Nodes (Tour)│       ├── camera_publisher.py

| Node | Description |│       └── marker_detector.py

|------|-------------|├── scripts/

| `camera_publisher` | Capture caméra USB |│   └── bridge_pi_standalone.py # 🆕 Pour Pi

| `marker_detector` | Détection ArUco + transformation |├── launch/                     # 🆕

| `robot_commander` | Interface commandes interactive |│   ├── marker_follow.launch.py

│   ├── bridge_only.launch.py

### Launch Files│   └── commander.launch.py

```bash└── setup.py

# Suivi de marqueur complet```

ros2 launch mycobot_gateway marker_follow.launch.py

---

# Bridge seul

ros2 launch mycobot_gateway bridge_only.launch.py**Version:** 0.1.0 | **Mise à jour:** 26 mars 2026 | **ROS2:** Jazzy


# Commander interactif
ros2 launch mycobot_gateway commander.launch.py
```

---

## ⚙️ Configuration

| Paramètre | Valeur |
|-----------|--------|
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
