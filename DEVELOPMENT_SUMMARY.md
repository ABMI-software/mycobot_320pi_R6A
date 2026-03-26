# 🤖 MyCobot 320 Pi - Résumé de Développement

> **Date de dernière mise à jour:** 26 mars 2026  
> **Version:** 1.0.0  
> **Repository GitHub:** https://github.com/ABMI-software/mycobot_320pi_R6A  
> **Branche:** `main`  
> **Dernier commit:** `3e2333c`

---

## 📌 Point de Départ Rapide

👉 **Pour démarrer une nouvelle session, consultez [`SESSION_RESUME.md`](SESSION_RESUME.md)**

---

## 📋 Vue d'ensemble du projet

### Objectif
Contrôler un robot **MyCobot 320 Pi** depuis un PC distant (**Tour**) via ROS2 et une connexion TCP.

### Architecture distribuée Tour ↔ Raspberry Pi

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              TOUR (PC)                                       │
│                         Ubuntu 24.04 / ROS2 Jazzy                           │
│                              Python 3.12                                     │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐              │
│  │  bridge_tour    │  │   joint_sync    │  │     RViz2       │              │
│  │  (TCP Client)   │  │ (Synchronisation│  │ (Visualisation) │              │
│  └────────┬────────┘  └────────┬────────┘  └────────┬────────┘              │
│           │                    │                    │                        │
│           │         ROS2 Topics: /to_robot, /from_robot, /joint_states      │
│           │                    │                    │                        │
└───────────┼────────────────────┼────────────────────┼────────────────────────┘
            │                    │                    │
            │              TCP Port 5005              │
            │                    │                    │
┌───────────┼────────────────────┼────────────────────┼────────────────────────┐
│           │                    │                    │                        │
│           ▼                    │                    │                        │
│  ┌─────────────────┐          │                    │                        │
│  │  bridge_pi      │          │                    │                        │
│  │  (TCP Server)   │──────────┘                    │                        │
│  └────────┬────────┘                               │                        │
│           │                                        │                        │
│           ▼                                        │                        │
│  ┌─────────────────┐                               │                        │
│  │   MyCobot 320   │                               │                        │
│  │  /dev/ttyAMA0   │                               │                        │
│  └─────────────────┘                               │                        │
│                                                                              │
│                         RASPBERRY PI                                         │
│                     Ubuntu 20.04 / ROS2 Galactic                            │
│                           Python 3.8                                         │
└──────────────────────────────────────────────────────────────────────────────┘
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

### Court terme
- [ ] Stabiliser la connexion TCP (timeout, retry, heartbeat)
- [ ] Réduire la fréquence de synchronisation par défaut
- [ ] Ajouter un mode "debug" avec logs détaillés

### Moyen terme
- [ ] **Détection ArUco** sur la Pi avec caméra USB
- [ ] **Mode Follow** - robot suit le marqueur automatiquement
- [ ] **Interface GUI** sur la Tour (rqt ou PyQt)
- [ ] **Simulation Gazebo** du robot

### Long terme
- [ ] Path planning avec MoveIt2
- [ ] Intégration IA pour tâches complexes
- [ ] Multi-robot coordination

---

## 🧪 Tests validés

| Test | Date | Statut |
|------|------|--------|
| Connexion TCP Tour → Pi | 26/03/2026 | ✅ OK |
| Commande `ping` | 26/03/2026 | ✅ OK |
| Commande `status` | 26/03/2026 | ✅ OK |
| Commande `get_angles` | 26/03/2026 | ✅ OK |
| Commande `go_home` | 26/03/2026 | ✅ OK |
| Commande `set_led` | 26/03/2026 | ✅ OK |
| Commande `set_angle` | 26/03/2026 | ✅ OK |
| Commande `set_angles` | 26/03/2026 | ✅ OK |
| RViz visualisation standalone | 26/03/2026 | ✅ OK |
| RViz synchronisé avec robot | 26/03/2026 | ⚠️ Instable (freq trop haute) |

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

- **Développement initial:** Session du 26 mars 2026

---

*Ce document sert de point de départ pour les prochaines sessions de développement.*
