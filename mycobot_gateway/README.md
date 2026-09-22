# mycobot_gateway

Package ROS2 central du projet MyCobot 320 Pi R6A. Fournit le bridge TCP vers la Raspberry Pi, les modes de contrôle interactifs, le pipeline de vision DREAM, le pick-and-place (ArUco + sorting couleur), l'asservissement visuel en boucle fermée, la calibration caméra (extrinsèque + hand-eye), le benchmark de précision et la collecte de données synthétiques Gazebo.

**Versions :** voir [`../CHANGELOG.md`](../CHANGELOG.md) — le package couvre plusieurs tracks versionnés indépendamment (téléop 2.x, sorting/pick-and-place 1.x, DREAM/calibration 1.x)
**ROS2 Distro :** Jazzy (Python 3.12)

---

## Architecture réseau

```
┌──────────────────────────────┐    TCP:5005    ┌────────────────────────────┐
│      PC Tour (10.10.0.115)   │◄──────────────►│  Raspberry Pi (10.10.0.221)│
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

### Contrôle de base

| Nœud | Fichier | Description |
|------|---------|-------------|
| `bridge_tour` | `bridge_tour.py` | Client TCP → Pi (TCP:5005), pub/sub JSON |
| `simple_gui` | `simple_gui.py` | GUI Tkinter (angles, coords, gripper, LED) |
| `slider_control` | `slider_control.py` | Joint State Publisher + RViz temps réel |
| `teleop_keyboard` | `teleop_keyboard.py` | Contrôle clavier WASD+ZX |
| `robot_commander` | `robot_commander.py` | CLI interactif |
| `joint_sync` | `joint_sync.py` | Sync état robot réel → RViz (accepte `ANGLES:`/`angles:`/`angles_ok:`, ignore les réponses d'erreur `-1`) |
| `marker_follower` | `marker_follower.py` | Suit un marqueur ArUco (legacy — écoute les TF, envoie les commandes via `/to_robot`) |
| `trajectory_to_robot_bridge` | `trajectory_to_robot_bridge.py` | Pont `JointTrajectory` (rad) → JSON `send_angles` (deg) pour `bridge_tour` — cœur de la téléop côté robot réel |
| `gripper_to_robot_bridge` | `gripper_to_robot_bridge.py` | Convertit la commande continue sim (`Float64MultiArray` rad) en `gripper_open`/`gripper_close` JSON via une machine à états Schmitt-trigger + debounce |

### Caméras

| Nœud | Fichier | Description |
|------|---------|-------------|
| `orbbec_camera_publisher` | `orbbec_camera_publisher.py` | Publie le flux RGB Orbbec Astra (via `teleop.orbbec_capture`, shared-memory) sur `/camera/color/image_raw` |
| `camera_live_view` | `camera_live_view.py` | Visionneuse OpenCV locale (`CAM_LIVE_LAYOUT=single\|split`) — diagnostic terrain |
| `camera_web_view` | `camera_web_view.py` | Visionneuse MJPEG sur `http://127.0.0.1:8090` (`CAM_WEB_LAYOUT=single\|split\|dual`) — utile sans `DISPLAY` |

### Calibration (§3.2 extrinsèque, §3.3 hand-eye)

| Nœud | Fichier | Description |
|------|---------|-------------|
| `calibrate_extrinsic` | `calibrate_extrinsic_node.py` | T_cam_world par solvePnP multi-marqueurs sol, validation reprojection + redondance, sauvegarde `camera_extrinsic.yaml`, publication TF optionnelle |
| `calibrate_hand_eye` | `calibrate_hand_eye_node.py` | Calibration hand-eye Tsai-Lenz (AX=XB) — marqueur solidaire de l'effecteur, capture interactive ou balayage auto, résout `T_base_camera` |

### Benchmark de précision

| Nœud | Fichier | Description |
|------|---------|-------------|
| `aruco_localizer` | `aruco_localizer_node.py` | Localizer robot réel : détecte 4 marqueurs workspace + objet (ID 10) via `cam_0.npz`, publie `/aruco/object_pose` + `/aruco/workspace_valid` |
| `gz_sim_localizer` | `gz_sim_localizer_node.py` | Équivalent simulation — republie la pose vérité-terrain Gazebo du cube cible, interface identique à `aruco_localizer` |
| `fk_ee_pose` | `fk_ee_pose_node.py` | `/joint_states` → cinématique directe URDF → `/fk/ee_pose` (identique sim/réel) |
| `precision_benchmark` | `precision_benchmark_node.py` | Grille de 9 cibles, IK → commande → attente stabilisation → erreur `‖cible − EE_réel‖` → rapport CSV |
| `reach_target_aruco` | `reach_target_aruco_node.py` | Test one-shot : atteint la pose ArUco détectée (sans pince), une seule `JointTrajectory` |

### Pick-and-place ArUco (sim + robot réel)

| Nœud | Fichier | Description |
|------|---------|-------------|
| `pick_and_place_aruco` | `pick_and_place_aruco_node.py` | Orchestrateur cycle complet — `mode=sim` (JTC + grasp émulé `gz set_pose`) ou `mode=real` (`trajectory_to_robot_bridge` + gripper JSON) |

### Pick-and-place mono + sorting couleur (Gazebo)

| Nœud | Fichier | Description |
|------|---------|-------------|
| `pick_and_place` | `pick_and_place_node.py` | State machine pick & place mono-objet |
| `color_object_detector` | `color_object_detector.py` | Segmentation HSV (top camera) + back-projection vers le repère robot |
| `sorting_orchestrator` | `sorting_orchestrator.py` | Pick-and-place multi-objets par couleur, grasp émulé par téléport (boucle sur `/sorting/detections`) |
| `sim_sorting_grasp` | `sim_sorting_grasp.py` | Variante `sorting_orchestrator` avec **saisie physique** (JTC + `gripper_position_controller`, prise vérifiée sur la pose Gazebo de l'objet — plus de téléportation) |

### Asservissement visuel en boucle fermée (§11)

| Nœud | Fichier | Description |
|------|---------|-------------|
| `object_pose_node` | `visual_servo/object_pose_node.py` | Détection objet par caméra → `/visual_servo/<cam>/detection`, une instance par caméra branchée |
| `visual_servo_controller` | `visual_servo/visual_servo_node.py` | Contrôleur unique — démarre **désarmé** (`dry_run:=true`), attend un `start` explicite sur `/visual_servo/command` |

### Vision DREAM

| Nœud | Fichier | Description |
|------|---------|-------------|
| `dream_inference` | `dream_inference_node.py` | Inférence DREAM + PnP pose estimation |
| `dream_validation_dashboard` | `dream_validation_dashboard.py` | Dashboard PyQt5 temps réel — overlay pose DREAM vs pose encodeurs, multi-caméras (auto-détection, fusion *solve-then-fuse*), 3 filtres temporels, acquisition CSV. Voir [`../docs/DREAM_VALIDATION_DASHBOARD.md`](../docs/DREAM_VALIDATION_DASHBOARD.md) |

### Données synthétiques (Gazebo)

| Nœud | Fichier | Description |
|------|---------|-------------|
| `synth_data_collector` | `synthetic_data_collector_v2.py` | Collecte Gazebo + anti-collision FK |
| — *(non enregistré comme entry point)* | `synthetic_data_collector.py` | Version legacy 4-caméras, intrinsèques Arducam calibrées (superseded par v2/v3) |
| — *(non enregistré comme entry point)* | `synthetic_data_collector_v3_garde_basse.py` | Variante v3 avec garde au sol abaissée (`TABLE_CLEARANCE`) pour couvrir la zone de pick — expérimental, à vérifier avant toute collecte longue |

### Non exécutable via `ros2 run`

| Fichier | Rôle |
|---------|------|
| `command_executor_pi.py` | Modèle d'exécuteur de commandes **à copier sur le Pi** (package `mycobot_320pi`) — n'est pas un nœud de ce package, vit ici comme référence |

## Launch files

| Launch | Description |
|--------|-------------|
| `bridge_only.launch.py` | Bridge TCP seul |
| `simple_gui.launch.py` | GUI + bridge |
| `slider_control.launch.py` | Sliders + RViz + bridge |
| `teleop_keyboard.launch.py` | Clavier + bridge |
| `commander.launch.py` | CLI + bridge |
| `rviz_sync.launch.py` | Sync robot réel → RViz |
| `marker_follow.launch.py` | `bridge_tour` + `camera_publisher` + `marker_detector` |
| `marker_follow_full.launch.py` | Stack complète suivi ArUco robot réel (caméra Pi → `marker_follower` → `bridge_tour`) |
| `mycobot_teleop.launch.py` | **Téléop par la main** — orchestre Gazebo + controllers + rosbridge + bridge_tour + trajectory_to_robot_bridge (target sim/real/both) |
| `pick_and_place.launch.py` | Cycle pick & place mono-objet (cube rouge → zone verte) |
| `pick_and_place_sorting.launch.py` | Pick & place multi-objets par couleur (4 objets → 4 bacs) |
| `pick_and_place_aruco.launch.py` | Pick-and-place ArUco en simulation (`precision_benchmark.sdf` + `gz_sim_localizer` + `fk_ee_pose` + `pick_and_place_aruco`) |
| `pick_and_place_aruco_real.launch.py` | Pick-and-place ArUco sur robot réel (`bridge_tour` + `aruco_localizer` + `fk_ee_pose` + `pick_and_place_aruco` mode=real) |
| `sim_grasp.launch.py` | Banc de saisie **physique** Gazebo (JTC + `gripper_position_controller`, pas de téléportation) — `headless:=true` disponible |
| `precision_benchmark.launch.py` | Benchmark de précision en simulation — grille 9 cibles + rapport CSV |
| `precision_benchmark_real.launch.py` | Benchmark de précision sur robot réel (4 marqueurs workspace + `aruco_localizer`) |
| `visual_servo.launch.py` | Asservissement visuel en boucle fermée — auto-détection caméras, démarre **désarmé** (`dry_run:=true`) |
| `dream_multicam.launch.py` | Validation DREAM multi-caméras — auto-détecte 1 ou 2 caméras (arducam/SVPRO), spawn dynamique + fusion dashboard |
| `synthetic_data.launch.py` | Collecte données sim (monde de base) |
| `synthetic_data_v2.launch.py` | Collecte v2 (domain randomization) |
| `synthetic_data_v3.launch.py` | Collecte v3 (monde randomized_v2 — 6 lights, 12 objets) |
| `synthetic_data_preview.launch.py` | Prévisualisation 5 poses × 4 caméras (contact sheet PNG) avant de lancer une collecte complète |

> Pour la téléopération complète, voir [`../docs/TELEOPERATION.md`](../docs/TELEOPERATION.md) et [`../docs/TELEOP_DASHBOARD.md`](../docs/TELEOP_DASHBOARD.md).

---

## Installation

```bash
conda deactivate   # IMPORTANT : éviter Python 3.13

cd ~/mycobot_320pi_R6A
colcon build --packages-select mycobot_gateway --symlink-install
source install/setup.bash
```

---

## Utilisation

### Démarrer la Raspberry Pi

```bash
ssh er@10.10.0.221

# Terminal 1 : bridge robot
python3 bridge_pi_simple.py

# Terminal 2 : serveur caméras
python3 pi_camera_server.py --cameras 0 3 --names cam0 cam3
```

### Contrôler le robot (PC Tour)

```bash
conda deactivate
source /opt/ros/jazzy/setup.bash
source ~/mycobot_320pi_R6A/install/setup.bash

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

# Pick-and-place ArUco (sim ou robot réel)
ros2 launch mycobot_gateway pick_and_place_aruco.launch.py
ros2 launch mycobot_gateway pick_and_place_aruco_real.launch.py

# Saisie physique (pas de téléportation)
ros2 launch mycobot_gateway sim_grasp.launch.py
```

### Calibration caméra

```bash
# Extrinsèque (marqueurs sol → repère monde)
ros2 run mycobot_gateway calibrate_extrinsic --ros-args \
    -p markers_yaml:=training/calibration/workspace_markers.yaml \
    -p camera_topic:=/camera/image_raw -p num_frames:=30 -p publish_tf:=true

# Hand-eye (Tsai-Lenz) — capture interactive c/u/l/s/a/x/q, ou balayage auto
ros2 run mycobot_gateway calibrate_hand_eye
```

### Benchmark de précision

```bash
ros2 launch mycobot_gateway precision_benchmark.launch.py          # simulation, grille 9 cibles
ros2 launch mycobot_gateway precision_benchmark_real.launch.py     # robot réel
```

### Asservissement visuel en boucle fermée

```bash
# Démarre désarmé (observe, ne commande rien)
ros2 launch mycobot_gateway visual_servo.launch.py
# Armer :
ros2 launch mycobot_gateway visual_servo.launch.py dry_run:=false
ros2 topic pub --once /visual_servo/command std_msgs/msg/String 'data: start'
# Suivre en direct :
ros2 topic echo /visual_servo/status
```

### Validation DREAM multi-caméras

```bash
ros2 launch mycobot_gateway dream_multicam.launch.py                                   # auto-détecte 1 ou 2 caméras
ros2 launch mycobot_gateway dream_multicam.launch.py model_name:=vgg_ultimate_v4_mix_ft_e30
ros2 launch mycobot_gateway dream_multicam.launch.py cameras:=arducam                  # forcer mono
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
| `/aruco/object_pose` | `geometry_msgs/PoseStamped` | localizer → * | Pose objet en repère base (source : `aruco_localizer` réel ou `gz_sim_localizer` sim) |
| `/aruco/workspace_valid` | `std_msgs/Bool` | localizer → * | `true` si ≥ 2 marqueurs workspace vus |
| `/aruco/debug_image` | `sensor_msgs/Image` | localizer → * | Frame annotée (marqueurs + pose objet) |
| `/fk/ee_pose` | `geometry_msgs/PoseStamped` | `fk_ee_pose` → * | Pose de l'effecteur terminal depuis `/joint_states` (identique sim/réel) |
| `/dream/keypoints` | — | `dream_inference` → dashboard | Keypoints 2D détectés (préfixe par caméra en mode multi-cam, ex. `/dream_svpro/keypoints`) |
| `/visual_servo/<cam>/detection` | — | `object_pose_node` → controller | Détection objet par caméra, une instance par caméra branchée |
| `/visual_servo/command` | `std_msgs/String` | opérateur → controller | `start` pour désarmer le contrôleur (démarre `dry_run:=true` par défaut) |
| `/visual_servo/status` | `std_msgs/String` | controller → * | État courant de la boucle d'asservissement |

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
ping 10.10.0.221
nc -zv 10.10.0.221 5005   # bridge robot
nc -zv 10.10.0.221 5006   # camera server
```

### "No executable found"
```bash
source ~/mycobot_320pi_R6A/install/setup.bash
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
