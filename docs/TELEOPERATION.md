# Hand Teleoperation — MyCobot 320 Pi

Pilotage du MyCobot 320 Pi par la main de l'opérateur, en simulation Gazebo et sur le robot réel. Adaptation du pipeline développé par l'équipe pour les robots ArmR5A et LeRobot (voir [ABMI-software/hand_controlLerobot](https://github.com/ABMI-software/hand_controlLerobot)).

---

## Principe

Une caméra filme la main de l'opérateur → [Wilor](https://github.com/warmshao/WiLor-mini-pipeline) estime la pose de la main → un mapping linéaire XYZ → 6 angles articulaires est publié sur le topic ROS2 `/mycobot_controller/joint_trajectory`. De là, deux consommateurs en parallèle :

- **Simulation** : `joint_trajectory_controller` (ros2_control) → `gz_ros2_control` → Gazebo
- **Robot réel** : `trajectory_to_robot_bridge` → JSON `send_angles` → `bridge_tour` → TCP:5005 → `bridge_pi_simple.py` → pymycobot → servos

```
┌───────────────────────────┐
│  mycobot_teleop.py        │  (conda env: hand-teleop)
│  Wilor hand pose → XYZ    │
│  xyz_to_joints_deg() → 6 angles deg
│  publish via rosbridge    │
└──────────────┬────────────┘
               ▼
     rosbridge_server (ws://localhost:9090)
               ▼
  /mycobot_controller/joint_trajectory   (trajectory_msgs/JointTrajectory)
          │                      │
          │ (sim)                │ (real)
          ▼                      ▼
  joint_trajectory_controller    trajectory_to_robot_bridge
  (ros2_control)                 (rclpy node, rad→deg, rate-limit, deadband)
          │                      │
          ▼                      ▼ /to_robot (JSON)
  gz_ros2_control                bridge_tour (TCP client)
          │                      │ TCP:5005
          ▼                      ▼
   Gazebo Harmonic        Raspberry Pi (10.10.0.225)
                          bridge_pi_simple.py → pymycobot
                                 │
                                 ▼
                          MyCobot 320 Pi (/dev/ttyAMA0)
```

---

## Prérequis

### Sur le PC Tour

```bash
# ROS2 stack (déjà installé sur le poste)
sudo apt install \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-joint-trajectory-controller \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros-gz-sim \
  ros-jazzy-ros-gz-bridge \
  ros-jazzy-rosbridge-server
```

### Environnement `hand-teleop` (conda)

Déjà présent sur ce poste à `/home/genji/miniconda/envs/hand-teleop`. Dépendances :
- `roslibpy` (pour le mode `--use-rosbridge`)
- `wilor`, `mediapipe`, `opencv-python`, `scipy`
- modules du repo `hand-teleop` (`hand_teleop.gripper_pose`, `hand_teleop.hand_pose`, `hand_teleop.tracking`)

### Sur la Raspberry Pi

Le même `bridge_pi_simple.py` que pour le contrôle GUI / sliders.

```bash
ssh er@10.10.0.225
python3 bridge_pi_simple.py
```

---

## Lancement (4 terminaux, pattern identique au R5A)

### Terminal 1 — rosbridge_server

```bash
conda deactivate
source /opt/ros/jazzy/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
# → Web socket ws://localhost:9090
```

### Terminal 2 — Gazebo + controllers + bridge_tour

```bash
conda deactivate
cd ~/ros_jazzy
colcon build --packages-select mycobot_description mycobot_gateway --symlink-install
source install/setup.bash

# Mode par défaut : simulation + robot réel en parallèle
ros2 launch mycobot_gateway mycobot_teleop.launch.py

# Simulation uniquement (utile pour tester sans le Pi)
ros2 launch mycobot_gateway mycobot_teleop.launch.py target:=sim

# Robot réel uniquement (sans Gazebo)
ros2 launch mycobot_gateway mycobot_teleop.launch.py target:=real

# IP Pi alternative
ros2 launch mycobot_gateway mycobot_teleop.launch.py pi_ip:=192.168.1.50
```

### Terminal 3 — script de téléopération

```bash
conda deactivate
conda activate hand-teleop
cd ~/ros_jazzy/src/mycobot_R6A/teleop

python3 mycobot_teleop.py \
  --ros --use-rosbridge \
  --ros-topic /mycobot_controller/joint_trajectory \
  --time-from-start 0.8 \
  --x-gain 1.2 --y-gain 1.2 --z-gain 1.6
```

### Terminal 4 — (optionnel) forwarder MQTT

Non utilisé pour le MyCobot : la liaison avec le robot réel passe par `bridge_tour` (TCP/JSON), pas par MQTT. Le script `mqtt_joint_forwarder.py` du projet R5A reste disponible si un jour on branche un broker MQTT, mais n'est pas nécessaire ici.

---

## Paramètres clés

### Gains et inversions (`mycobot_teleop.py`)

| Flag | Défaut | Rôle |
|------|--------|------|
| `--x-gain` | 1.2 | Gain X (main avant/arrière → épaule J2) |
| `--y-gain` | 1.2 | Gain Y (main gauche/droite → base J1) |
| `--z-gain` | 1.6 | Gain Z (main haute/basse → coude J3 + poignet J5) |
| `--invert-z` | True | Main haute ⇒ bras haut |
| `--time-from-start` | 0.8 | Durée de la trajectoire (s) — plus faible = plus réactif mais moins lissé |
| `--fps` | 60 | Fréquence de lecture de la main |

### Limites articulaires (URDF + `JOINT_LIMITS_DEG`)

| Joint | URDF | Limite (deg) |
|-------|------|--------------|
| `joint2_to_joint1` (J1) | ±2.93 rad | ±168° |
| `joint3_to_joint2` (J2) | ±2.35 rad | ±134.6° |
| `joint4_to_joint3` (J3) | ±2.53 rad | ±145° |
| `joint5_to_joint4` (J4) | ±2.53 rad | ±145° |
| `joint6_to_joint5` (J5) | ±2.93 rad | ±168° |
| `joint6output_to_joint6` (J6) | ±3.14 rad | ±180° |

### Envelope XYZ (SAFE_RANGE)

Tightened par rapport au R5A pour respecter la portée ~280 mm du MyCobot :
- X : 0.10 → 0.28 m
- Y : -0.20 → +0.20 m
- Z : 0.01 → 0.26 m

### Rate limiting vers le robot réel (`trajectory_to_robot_bridge`)

| Paramètre | Défaut | Rôle |
|-----------|--------|------|
| `rate_hz` | 15.0 | Fréquence max d'envoi JSON vers `/to_robot` |
| `deadband_deg` | 1.0 | Ignore les commandes qui changent de < 1° |
| `speed` | 40 | Vitesse pymycobot (0–100) |
| `enable` | true | Toggle pour couper le réel sans tuer la sim |

> Le rate-limit à 15 Hz est volontairement conservateur : `pymycobot` sature vers 20–25 Hz sur liaison série 115200 baud. En cas de latence excessive, baisser `rate_hz` à 10.

---

## Dépannage

### Le robot ne bouge pas en Gazebo

1. Vérifier que le `mycobot_controller` est bien actif :
   ```bash
   ros2 control list_controllers
   # → mycobot_controller [joint_trajectory_controller/JointTrajectoryController] active
   ```

2. Vérifier que `/mycobot_controller/joint_trajectory` reçoit bien des messages :
   ```bash
   ros2 topic hz /mycobot_controller/joint_trajectory
   ```

3. Si le spawner échoue avec *"Could not contact controller manager"*, augmenter le `period` dans le `TimerAction` du launch file (Gazebo met parfois >3 s à spawner le robot).

### Le robot réel ne bouge pas

1. Vérifier que `bridge_tour` est connecté :
   ```bash
   ros2 topic echo /from_robot
   # Devrait afficher les réponses du Pi à chaque JSON envoyé
   ```

2. Tester manuellement :
   ```bash
   ros2 topic pub --once /to_robot std_msgs/msg/String \
     '{data: "{\"action\":\"send_angles\",\"angles\":[0,0,0,0,0,0],\"speed\":40}"}'
   ```

3. Vérifier les limites — le Pi rejette les commandes hors-limite silencieusement.

### rosbridge refuse la connexion

```bash
# Vérifier que le port 9090 est libre
nc -zv localhost 9090
# Puis relancer le Terminal 1
```

### Main non détectée

Le modèle Wilor est sensible au cadrage et à l'éclairage. Tester avec la main bien visible, paume face caméra, fond uni. `--model mediapipe` est une alternative plus rapide mais moins précise.

---

## Limitations connues

1. **Pas d'IK réelle** — le mapping XYZ → angles est linéaire, pas géométrique. L'effecteur ne suit pas la main au mm près ; c'est un contrôle "gestuel" pas "précis". Pour du pick-and-place précis, utiliser `pick_and_place.launch.py` avec la détection DREAM.

2. **J4 et J6 sont fixes à 0°** — le mapping actuel n'utilise pas la rotation de la main pour piloter les poignets. Une extension future pourrait lire les angles d'Euler de la main (`rot` de `GripperPose`) et les mapper sur J4/J6.

3. **Pas de contrôle gripper** — le `pro_adaptive_gripper` est fixé en Gazebo (joints `fixed`). Pour le robot réel, ajouter une détection de pinch dans le script et publier un `{"action": "gripper_open"}` / `{"action": "gripper_close"}`.

4. **Dépendance sim-réel synchrone** — quand `target:=both`, la sim et le réel reçoivent les mêmes commandes. Si le réel est plus lent que la sim (limite servo), une désynchronisation visible apparaîtra. C'est accepté pour la démo ; pour un travail sérieux, utiliser `target:=real` ou `target:=sim`.

---

## Architecture des fichiers

```
mycobot_R6A/
├── teleop/
│   └── mycobot_teleop.py                # Script Wilor → JointTrajectory (conda env)
├── mycobot_description/
│   ├── config/controller.yaml           # Config JointTrajectoryController
│   └── urdf/320_pi/mycobot_pro_320_pi_gazebo.urdf  # URDF + ros2_control block
├── mycobot_gateway/
│   ├── mycobot_gateway/
│   │   └── trajectory_to_robot_bridge.py  # ROS2 node: trajectoire → JSON Pi
│   └── launch/mycobot_teleop.launch.py    # Orchestration complète
└── docs/TELEOPERATION.md                # Ce fichier
```
