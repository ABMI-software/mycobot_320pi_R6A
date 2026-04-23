# mycobot_gateway

Package ROS2 central du projet MyCobot 320 Pi R6A. Fournit le bridge TCP vers la Raspberry Pi, les modes de contrôle interactifs, le pipeline de vision DREAM et la collecte de données synthétiques Gazebo.

**Version :** 1.9.0
**ROS2 Distro :** Jazzy (Python 3.12)

---

## Architecture réseau

```
┌──────────────────────────────┐    TCP:5005    ┌────────────────────────────┐
│      PC Tour (10.10.0.115)   │◄──────────────►│  Raspberry Pi (10.10.0.225)│
│                              │                │                            │
│  bridge_tour (ROS2 node)     │                │  bridge_pi_simple.py       │
│  Sub: /to_robot (JSON)       │                │  → pymycobot /dev/ttyAMA0  │
│  Pub: /from_robot            │                │                            │
│                              │    TCP:5006    │  pi_camera_server.py       │
│  [caméra client]             │◄──────────────►│  → 2× Arducam USB          │
└──────────────────────────────┘                └────────────────────────────┘
                                                           │
                                                           ▼
                                                  ┌──────────────┐
                                                  │ MyCobot 320  │
                                                  │ /dev/ttyAMA0 │
                                                  └──────────────┘
```

---

## Noeuds ROS2

| Nœud | Fichier | Description |
|------|---------|-------------|
| `bridge_tour` | `bridge_tour.py` | Client TCP → Pi (TCP:5005), pub/sub JSON |
| `simple_gui` | `simple_gui.py` | GUI Tkinter (angles, coords, gripper, LED) |
| `slider_control` | `slider_control.py` | Joint State Publisher + RViz temps réel |
| `teleop_keyboard` | `teleop_keyboard.py` | Contrôle clavier WASD+ZX |
| `robot_commander` | `robot_commander.py` | CLI interactif |
| `joint_sync` | `joint_sync.py` | Sync état robot réel → RViz |
| `dream_inference` | `dream_inference_node.py` | Inférence DREAM + PnP pose estimation |
| `pick_and_place` | `pick_and_place_node.py` | State machine pick & place mono-objet |
| `color_object_detector` | `color_object_detector.py` | Segmentation HSV (top camera) + back-projection vers le repère robot |
| `sorting_orchestrator` | `sorting_orchestrator.py` | Pick-and-place multi-objets par couleur (boucle sur `/sorting/detections`) |
| `synth_data_collector` | `synthetic_data_collector_v2.py` | Collecte Gazebo + anti-collision FK |

## Launch files

| Launch | Description |
|--------|-------------|
| `bridge_only.launch.py` | Bridge TCP seul |
| `simple_gui.launch.py` | GUI + bridge |
| `slider_control.launch.py` | Sliders + RViz + bridge |
| `teleop_keyboard.launch.py` | Clavier + bridge |
| `commander.launch.py` | CLI + bridge |
| `rviz_sync.launch.py` | Sync robot réel → RViz |
| `pick_and_place.launch.py` | Cycle pick & place mono-objet (cube rouge → zone verte) |
| `pick_and_place_sorting.launch.py` | Pick & place multi-objets par couleur (4 objets → 4 bacs) |
| `synthetic_data.launch.py` | Collecte données sim (monde de base) |
| `synthetic_data_v2.launch.py` | Collecte v2 (domain randomization) |
| `synthetic_data_v3.launch.py` | Collecte v3 (monde randomized_v2 — 6 lights, 12 objets) |

---

## Installation

```bash
conda deactivate   # IMPORTANT : éviter Python 3.13

cd ~/ros_jazzy/src/mycobot_R6A
colcon build --packages-select mycobot_gateway --symlink-install
source install/setup.bash
```

---

## Utilisation

### Démarrer la Raspberry Pi

```bash
ssh er@10.10.0.225

# Terminal 1 : bridge robot
python3 bridge_pi_simple.py

# Terminal 2 : serveur caméras
python3 pi_camera_server.py --cameras 0 3 --names cam0 cam3
```

### Contrôler le robot (PC Tour)

```bash
conda deactivate
source /opt/ros/jazzy/setup.bash
source ~/ros_jazzy/src/mycobot_R6A/install/setup.bash

ros2 launch mycobot_gateway simple_gui.launch.py
# ou
ros2 launch mycobot_gateway slider_control.launch.py
# ou
ros2 launch mycobot_gateway teleop_keyboard.launch.py
```

### Collecter des données synthétiques

```bash
# Monde v2 (recommandé — domain randomization avancée)
ros2 launch mycobot_gateway synthetic_data_v3.launch.py num_samples:=7500

# Paramètres disponibles :
# num_samples:=7500     (nombre de poses)
# settle_time:=1.2      (secondes d'attente avant capture)
# output_dir:=/tmp/...  (répertoire de sortie)
```

### Pick-and-place en simulation

```bash
# Mono-objet (cube rouge → zone verte)
ros2 launch mycobot_gateway pick_and_place.launch.py

# Multi-objet par couleur (4 objets → 4 bacs)
ros2 launch mycobot_gateway pick_and_place_sorting.launch.py

# Variantes :
ros2 launch mycobot_gateway pick_and_place_sorting.launch.py use_detector:=false
ros2 launch mycobot_gateway pick_and_place_sorting.launch.py process_order:=blue,green
```

---

## Topics ROS2

| Topic | Type | Direction | Description |
|-------|------|-----------|-------------|
| `/to_robot` | `std_msgs/String` | Tour → Pi | Commandes JSON vers le robot |
| `/from_robot` | `std_msgs/String` | Pi → Tour | Réponses du robot |
| `/joint_states` | `sensor_msgs/JointState` | Gz → ROS2 | États articulaires Gazebo |
| `/synth_camera/image` | `sensor_msgs/Image` | Gz → ROS2 | Image caméra Gazebo (front) |
| `/synth_camera_top/image` | `sensor_msgs/Image` | Gz → ROS2 | Image caméra top-down |
| `/sorting/detections` | `std_msgs/String` | detector → orchestrator | `color,x,y;…` (positions m, repère robot) |
| `/sorting/detector_status` | `std_msgs/String` | detector → * | `WAITING_IMAGE` / `OK\|n=N` / `NO_DETECTIONS` |
| `/sorting/debug_image` | `sensor_msgs/Image` | detector → * | Overlay des centroïdes détectés |
| `/pickplace/status` | `std_msgs/String` | orchestrator → * | `STATE\|detail` (état machine) |

## Protocole JSON (tour → Pi)

```json
{"action": "send_angles", "angles": [0, 8, -127, 40, 0, 0], "speed": 40}
{"action": "send_coords", "coords": [200, 0, 250, 180, 0, 0], "speed": 40, "mode": 1}
{"action": "gripper_open"}
{"action": "gripper_close"}
{"action": "go_home"}
{"action": "get_angles"}
{"action": "emergency_stop"}
```

---

## Dépannage

### `ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'`
Conda est actif — ROS2 Jazzy nécessite Python 3.12, Conda utilise Python 3.13.
```bash
conda deactivate
```

### "Impossible de se connecter à la Pi"
```bash
ping 10.10.0.225
nc -zv 10.10.0.225 5005   # bridge robot
nc -zv 10.10.0.225 5006   # camera server
```

### "No executable found"
```bash
source ~/ros_jazzy/src/mycobot_R6A/install/setup.bash
ros2 pkg executables mycobot_gateway
```

### Plusieurs instances bridge_tour
```bash
pkill -f bridge_tour
# Puis relancer une seule instance
```

### Meshes Gazebo non trouvés
Le launch file doit définir `GZ_SIM_RESOURCE_PATH` :
```bash
export GZ_SIM_RESOURCE_PATH=~/ros_jazzy/install/mycobot_description/share:$GZ_SIM_RESOURCE_PATH
```

---

## Scripts Pi (`scripts/`)

| Fichier | Description |
|---------|-------------|
| `bridge_pi_simple.py` | Serveur TCP:5005 — reçoit commandes JSON → pymycobot |
| `pi_camera_server.py` | Serveur TCP:5006 — streaming JPEG depuis Arducam USB |

---

## Licence

Apache License 2.0
