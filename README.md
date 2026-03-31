# MyCobot Gateway Bridge

🤖 **Bridge ROS2 pour contrôler un MyCobot 320 Pi depuis un PC distant**

Ce package permet de contrôler un robot MyCobot 320 Pi depuis un PC "Tour" via une communication TCP/ROS2, en séparant le calcul lourd (GUI, RViz, vision) du contrôle robot.

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              PC TOUR                                        │
│                    ROS2 Jazzy / Ubuntu 24.04 / Python 3.12                  │
├─────────────────────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │ simple_gui  │  │slider_control│  │teleop_keyb. │  │marker_follow│        │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘        │
│         └────────────────┴────────────────┴────────────────┘               │
│                                   │ /to_robot (JSON)                       │
│                         ┌─────────▼─────────┐                              │
│                         │   bridge_tour     │◄─── /from_robot              │
│                         └─────────┬─────────┘                              │
│                                   │ TCP:5005                               │
├───────────────────────────────────┼─────────────────────────────────────────┤
│                                   │ RÉSEAU                                 │
├───────────────────────────────────┼─────────────────────────────────────────┤
│                         ┌─────────▼─────────┐                              │
│                         │  bridge_pi_debug  │                              │
│                         └─────────┬─────────┘                              │
│                                   │ /dev/ttyAMA0                           │
│                         ┌─────────▼─────────┐                              │
│                         │  MyCobot 320 Pi   │                              │
│                         └───────────────────┘                              │
│                     RASPBERRY PI (ROS2 Galactic)                           │
└─────────────────────────────────────────────────────────────────────────────┘
```

## 📦 Packages

| Package | Description |
|---------|-------------|
| `mycobot_gateway` | Bridge TCP, GUI, contrôles (Tour/PC) |
| `mycobot_description` | URDF, meshes, configs RViz |

## 🚀 Quick Start

### Prérequis

**Sur le PC Tour :**
- Ubuntu 24.04
- ROS2 Jazzy
- Python 3.12

**Sur le Raspberry Pi :**
- Ubuntu 20.04
- ROS2 Galactic
- pymycobot (`pip3 install pymycobot`)

### Installation

```bash
# Cloner le repo
cd ~/ros2_ws/src
git clone https://github.com/ABMI-software/mycobot-gateway.git

# Compiler
cd ~/ros2_ws
colcon build --packages-select mycobot_gateway mycobot_description --symlink-install
source install/setup.bash
```

### Déploiement sur le Pi

```bash
# Copier le script bridge sur le Pi
scp bridge_pi_debug.py user@<PI_IP>:~/

# Sur le Pi, lancer le bridge
source /opt/ros/galactic/setup.bash
python3 ~/bridge_pi_debug.py
```

### Utilisation

```bash
# ⚠️ Important : désactiver conda avant ROS2
conda deactivate

# Sourcer ROS2
source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

# Lancer un des modes de contrôle
ros2 launch mycobot_gateway simple_gui.launch.py        # GUI graphique
ros2 launch mycobot_gateway slider_control.launch.py    # Sliders RViz
ros2 launch mycobot_gateway teleop_keyboard.launch.py   # Clavier
ros2 launch mycobot_gateway commander.launch.py         # CLI interactif
ros2 launch mycobot_gateway rviz_sync.launch.py         # Sync robot→RViz
```

## 🎮 Modes de Contrôle

### 1. Simple GUI (`simple_gui.launch.py`)
Interface Tkinter avec :
- Champs pour angles joints et coordonnées cartésiennes
- Boutons Home, Zero, Gripper Open/Close
- Affichage temps réel

### 2. Slider Control (`slider_control.launch.py`)
- Joint State Publisher GUI avec sliders
- Le robot réel suit les mouvements en temps réel
- Visualisation dans RViz

### 3. Teleop Keyboard (`teleop_keyboard.launch.py`)
Contrôle clavier :
- `W/S` - X+/X-
- `A/D` - Y-/Y+
- `Z/X` - Z-/Z+
- `G/H` - Gripper open/close
- `1/2` - Positions Init/Home

### 4. Commander CLI (`commander.launch.py`)
Commandes textuelles : `home`, `zero`, `angles`, `open`, `close`, `stop`

## 📡 Configuration Réseau

| Machine | IP (exemple) | Port |
|---------|--------------|------|
| PC Tour | 10.10.0.115 | - |
| Raspberry Pi | 10.10.0.225 | 5005 |

Modifier l'IP dans les launch files :
```bash
ros2 launch mycobot_gateway simple_gui.launch.py pi_ip:=<VOTRE_IP_PI>
```

## 📨 Format des Commandes

Le bridge communique en JSON :

```json
{"action": "go_home"}
{"action": "send_angles", "angles": [0,0,0,0,0,0], "speed": 30}
{"action": "send_coords", "coords": [200,0,200,180,0,0], "speed": 40}
{"action": "gripper_open"}
{"action": "get_angles"}
```

## 📁 Structure

```
mycobot_gateway/
├── mycobot_gateway/
│   ├── bridge_tour.py      # Client TCP (Tour)
│   ├── simple_gui.py       # Interface graphique
│   ├── slider_control.py   # Contrôle par sliders
│   ├── teleop_keyboard.py  # Contrôle clavier
│   ├── robot_commander.py  # Interface CLI
│   ├── joint_sync.py       # Sync robot→RViz
│   └── marker_follower.py  # Suivi ArUco
├── launch/
│   ├── simple_gui.launch.py
│   ├── slider_control.launch.py
│   ├── teleop_keyboard.launch.py
│   ├── commander.launch.py
│   └── rviz_sync.launch.py
└── scripts/
    └── bridge_pi_debug.py  # À déployer sur Pi
```

## ⚠️ Troubleshooting

### Erreur Python conda/ROS2
```bash
# Toujours désactiver conda avant ROS2
conda deactivate
# Ou utiliser :
export PATH="/usr/bin:$PATH"
unset PYTHONPATH
```

### Connexion TCP échoue
```bash
# Vérifier la connectivité
ping <PI_IP>
nc -zv <PI_IP> 5005
```

## 📄 License

Apache License 2.0

## 👥 Contributeurs

- ABMI Software Team
