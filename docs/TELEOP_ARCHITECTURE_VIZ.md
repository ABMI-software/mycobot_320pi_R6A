# Visuel détaillé — de la détection main au mouvement du bras

Ce document décortique **chaque étape** du pipeline de téléopération, avec :
- Les types de données à chaque noeud
- Les conversions d'unités (radians ↔ degrés, mètres ↔ ...)
- Les filtres appliqués
- Les topics ROS2 utilisés
- Les chemins différents pour Gazebo (sim) et MyCobot physique (réel)

> Pour une vue haute-niveau + workflow opérateur, voir [TELEOPERATION.md](TELEOPERATION.md). Pour le détail d'un paramètre précis, voir [TELEOP_TUNING.md](TELEOP_TUNING.md).

---

## Vue d'ensemble 1 coup d'œil

```
    👋 MA MAIN                             🦾 MYCOBOT 320 Pi
       │                                       ▲
       │    caméra →  Wilor  →  mapping        │  JSON TCP  →  pymycobot
       │              │          │              │  ↑            ↑
       ▼              ▼          ▼              │  │            │
    [frame RGB] [XYZ + Euler] [joints deg]  [TCP] [bridge_tour] [servos]
                                   │          │
                                   ▼          │
                                [filtres]     │
                                   │          │
                                   ▼          │
                               [/mycobot_controller/joint_trajectory] → [JTC Gazebo] │
                                   │                                                  │
                                   └─── trajectory_to_robot_bridge ────────────────────┘
```

---

## Diagramme détaillé (9 étapes, types explicites)

```
┌─────────────────────────────────────────────────────────────────────────┐
│ ÉTAPE 1 — ACQUISITION CAMÉRA                                            │
│                                                                          │
│  Orbbec Astra S                                                          │
│    │                                                                     │
│    │ USB + OpenNI2                                                       │
│    ▼                                                                     │
│  oni_grabber (binaire C++)                                               │
│    │                                                                     │
│    │ écrit les frames dans un buffer partagé                             │
│    ▼                                                                     │
│  /dev/shm/oni_color.rgb  (RGB 640 × 480 × 3, 8 bits, 30 Hz)              │
│  /dev/shm/oni_tick.txt   (timestamp — watchdog détecte si figé)         │
│                                                                          │
│  Type de donnée à ce stade : tableau uint8 (image brute)                 │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│ ÉTAPE 2 — ACCÈS PYTHON (OrbbecCapture wrapper)                          │
│                                                                          │
│  teleop/orbbec_capture.py                                                │
│    │                                                                     │
│    │ cap.read() lit le fichier dans /dev/shm                             │
│    ▼                                                                     │
│  BGR frame : np.ndarray shape (480, 640, 3), dtype=uint8                 │
│                                                                          │
│  Robustesse :                                                            │
│    - Watchdog auto-respawn oni_grabber si tick bloqué > 2 s              │
│    - start_new_session=True : oni_grabber survit au Ctrl+C du teleop     │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│ ÉTAPE 3 — WILOR HAND POSE ESTIMATION                                    │
│                                                                          │
│  hand_teleop.HandTracker (daemon thread)                                 │
│    │                                                                     │
│    │ cap.read() → frame cv2.flip → Wilor inference                       │
│    ▼                                                                     │
│  GripperPose (struct) :                                                  │
│    .pos          : np.ndarray shape (3,)  en mètres — ABSOLUE caméra    │
│    .rot          : np.ndarray shape (3,3) matrice de rotation           │
│    .open_degree  : float 2–90° (angle pouce-index)                      │
│    .keypoints    : list[ndarray] des 21 keypoints 3D                    │
│                                                                          │
│  Filtres internes :                                                      │
│    - Jump clamp à 2 m/s (rejette les sauts Wilor invalides)              │
│    - Kalman XYZ (dt=1/30, q=r=5e-3) lisse les positions                  │
│    - Exception wrapper (survit aux crashs Wilor)                         │
│                                                                          │
│  Sur la première détection après Recalibrate :                           │
│    pose_computer.initial_pose = GripperPose actuelle                     │
│  Ensuite :                                                               │
│    rel_pose = abs_pose − initial_pose  (pose RELATIVE)                   │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│ ÉTAPE 4 — MAPPING XYZ + RPY → 6 JOINTS (mycobot_teleop.py)              │
│                                                                          │
│  Entrée :                                                                │
│    rel_pos = (dx, dy, dz)  en mètres                                    │
│    rel_rot → Euler ZYX intrinsèque → (roll, pitch, yaw) en degrés       │
│                                                                          │
│  xyz_to_joints_deg() applique des gains + signes :                       │
│                                                                          │
│     scale = BASE_SCALE_DEG_PER_M = 150 °/m                              │
│                                                                          │
│     j1 =  dy  * scale * y_gain(1.2)      →  J1 base yaw                 │
│     j2 =  dz  * scale * z_gain(1.6)      →  J2 shoulder pitch            │
│     j3 =  dx  * scale * x_gain(1.2)      →  J3 elbow                    │
│     j4 =  pitch * pitch_gain(0.4) * 0.5  →  J4 wrist1 pitch              │
│     j5 =  pitch * pitch_gain(0.4) * 0.5  →  J5 wrist2 pitch              │
│     j6 =  yaw   * roll_gain(0.4)         →  J6 EE roll (doorknob)        │
│                                                                          │
│  Exemple concret :                                                       │
│    main qui s'élève de 15 cm (dz = +0.15 m)                              │
│      → j2 = 0.15 × 150 × 1.6 = 36° sur l'épaule                         │
│                                                                          │
│  Clampage aux limites URDF (±134° J2, ±145° J3, etc.)                    │
│                                                                          │
│  Sortie : np.ndarray shape (6,) en degrés                                │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│ ÉTAPE 5 — FILTRES STABILITÉ (portés de R5A / LeRobot)                   │
│                                                                          │
│  Filtre EMA (Exponential Moving Average) :                               │
│                                                                          │
│     q_smoothed = (1 − α) × q_smoothed_prev + α × q_raw                   │
│                                                                          │
│     α = DEFAULT_COMMAND_EMA_ALPHA = 0.20                                 │
│     → 80 % de l'ancienne valeur + 20 % de la nouvelle                    │
│     → dampens les petits jitters Wilor                                   │
│                                                                          │
│  Filtre slew rate (limiteur de pente) :                                  │
│                                                                          │
│     Δq = q_smoothed − q_published_prev                                   │
│     Δq = clip(Δq, −MAX_DELTA, +MAX_DELTA)                                │
│                                                                          │
│     MAX_DELTA = 1 °/frame à 30 Hz = 30 °/s max                           │
│     → évite les commandes qui saturent physiquement le JTC               │
│                                                                          │
│  Entrée : q_raw (6 valeurs deg)                                          │
│  Sortie : q_published (6 valeurs deg, lissé et borné)                    │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                                    ▼
┌─────────────────────────────────────────────────────────────────────────┐
│ ÉTAPE 6 — PUBLICATION SUR ROSBRIDGE (WebSocket, 30 Hz)                  │
│                                                                          │
│  mycobot_teleop.py (dans env conda) passe par rosbridge :                │
│    │                                                                     │
│    │ roslibpy.Topic.publish(...)                                         │
│    │ sur ws://localhost:9090                                             │
│    ▼                                                                     │
│  rosbridge_server.py                                                     │
│    │                                                                     │
│    │ relaie le message sur le bus DDS de ROS2                            │
│    ▼                                                                     │
│  /mycobot_controller/joint_trajectory                                    │
│     type: trajectory_msgs/JointTrajectory                                │
│     {                                                                    │
│       joint_names: [joint2_to_joint1, ..., joint6output_to_joint6],      │
│       points: [{                                                         │
│         positions: [q1, q2, q3, q4, q5, q6]  en RADIANS (!)              │
│         time_from_start: {sec: 0, nanosec: 250_000_000}                  │
│       }]                                                                 │
│     }                                                                    │
│                                                                          │
│  ⚠ Conversion deg → rad faite dans RosBridgeArmPublisher.send_deg()      │
│     positions = np.radians(joint_deg).tolist()                           │
│                                                                          │
│  Fréquence : 30 Hz (fps=30 par défaut)                                   │
└─────────────────────────────────────────────────────────────────────────┘
                                    │
                   ┌────────────────┴────────────────┐
                   │                                  │
                   ▼                                  ▼
┌────────────────────────────────┐   ┌───────────────────────────────────┐
│ CHEMIN SIM GAZEBO              │   │ CHEMIN ROBOT RÉEL                 │
│                                │   │                                   │
│ joint_trajectory_controller    │   │ trajectory_to_robot_bridge        │
│   (JTC, ros2_control)          │   │   (ROS2 node Python)              │
│                                │   │                                   │
│ PID par joint (p=100, i=0.01,  │   │ Reçoit JointTrajectory (rad)      │
│ d=1.0), interpolation splines  │   │   │                               │
│   │                            │   │   │ rate_limit 15 Hz (baud série) │
│   ▼                            │   │   │ deadband 1° (skip micro-Δ)    │
│ gz_ros2_control plugin         │   │   │ reorder per URDF joint names  │
│   │                            │   │   │ conversion RAD → DEG          │
│   │ écrit les commandes aux    │   │   ▼                               │
│   │ joints de la simulation    │   │ payload JSON :                    │
│   ▼                            │   │ {                                 │
│ DART physics engine            │   │   "action": "send_angles",        │
│   │                            │   │   "angles": [deg1, ..., deg6],    │
│   ▼                            │   │   "speed": 25   (real_speed)      │
│ 🤖 Bras Gazebo bouge           │   │ }                                 │
│                                │   │   │                               │
│ joint_state_broadcaster        │   │   │ publié sur /to_robot          │
│   │                            │   │   │ (std_msgs/String)             │
│   ▼                            │   │   ▼                               │
│ /joint_states                  │   │ bridge_tour (ROS2 node)           │
│   → dashboard actual           │   │   │                               │
│                                │   │   │ TCP socket vers Pi 10.10.0.223│
│                                │   │   │   msg.data + '\n' encodé UTF-8│
│                                │   │   ▼                               │
│                                │   │ Raspberry Pi : bridge_pi_simple.py│
│                                │   │   │                               │
│                                │   │   │ parse JSON, action='send_angles'│
│                                │   │   │ mc.send_angles(angles, speed) │
│                                │   │   ▼                               │
│                                │   │ pymycobot (lib constructeur)      │
│                                │   │   │                               │
│                                │   │   │ série /dev/ttyAMA0 @ 115200   │
│                                │   │   │ protocole bytes propriétaire  │
│                                │   │   ▼                               │
│                                │   │ 🦾 Servos MyCobot bougent         │
└────────────────────────────────┘   └───────────────────────────────────┘
```

---

## Exemple chiffré de bout en bout

**Scénario** : l'opérateur a calibré sa main à `initial_pose = (0.10, 0.00, 0.20)`, puis déplace sa main de 10 cm vers la gauche, 5 cm vers le haut, 0 cm en profondeur.

| Étape | Donnée | Valeur |
|-------|--------|--------|
| 1. Astra capture | frame RGB | 640 × 480 × 3 uint8 |
| 2. Wilor inference | abs_pose.pos | (0.10, -0.10, 0.25) m |
| 3. pose_computer | rel_pose.pos | (0.00, -0.10, +0.05) m |
| 4. xyz_to_joints_deg | q_raw | J1 = -0.10 × 150 × 1.2 = **-18°** J2 = 0.05 × 150 × 1.6 = **12°** J3 = 0, J4/J5/J6 = 0 |
| 5. EMA + slew (après 10 frames à 30 Hz) | q_smoothed | J1 converge vers -18°, J2 vers 12° en ~150 ms |
| 6. Publish JointTrajectory | positions (rad) | [-0.314, 0.209, 0, 0, 0, 0] |
| 7-sim. JTC | joints Gazebo | physique → bras pivote 18° à gauche, épaule +12° |
| 7-real. trajectory_to_robot_bridge | JSON /to_robot | `{"action":"send_angles","angles":[-18,12,0,0,0,0],"speed":25}` |
| 8. bridge_tour | TCP | envoie bytes sur socket vers Pi |
| 9. Pi pymycobot | servos | J1 = -18°, J2 = +12° en ~1 s |

**Visuel de l'opérateur** : le bras physique pivote vers la gauche et relève l'épaule, en miroir de la main.

---

## Flux de contrôle en sens inverse (feedback)

```
┌────────────┐     publish     ┌─────────────────┐    subscribe     ┌────────────┐
│ Dashboard  │◄────────────────│ rosbridge_server│◄─────────────────│  teleop_   │
│  sliders   │  /teleop/gains  │                 │  /teleop/gains   │ dashboard  │
│            │  /teleop/recal. │                 │                   │            │
└────────────┘                 └─────────────────┘                   └────────────┘
     ▲                                 ▲
     │ /mycobot_controller/            │
     │   joint_trajectory              │
     │ /joint_states                   │
     │ /teleop/hand_xyz                │
     │                                 │
     │                                 │
┌────┴────┐                        ┌───┴──────────────────────┐
│ Gazebo  │   /joint_states        │  mycobot_teleop.py       │
│ (JTC +  │──────────────────────► │    receives gains,       │
│  phys.) │                        │    updates runtime        │
└─────────┘                        │    re-publishes cmds     │
                                   └──────────────────────────┘
```

- Le **dashboard lit** `/mycobot_controller/joint_trajectory` (les commandes en cours), `/joint_states` (les positions Gazebo réelles), et `/teleop/hand_xyz` (les XYZ Wilor) pour ses 3 plots et le tableau stats.
- Le **dashboard écrit** sur `/teleop/gains` (Float64MultiArray [gx, gy, gz, tfs]) quand l'opérateur bouge un slider → `mycobot_teleop.py` met à jour les gains en runtime sans redémarrage.
- Le **dashboard écrit** sur `/teleop/recalibrate` (Empty) quand l'opérateur clique le bouton → reset de `initial_pose` dans Wilor.

---

## Latences typiques

Mesurées sur le test 22/04/2026 :

| Étage | Latence approximative |
|-------|----------------------|
| Astra capture → oni_grabber → /dev/shm | ~33 ms (30 Hz nominal) |
| OrbbecCapture read | < 1 ms |
| Wilor inference (CPU) | 30–50 ms |
| mapping + filtres | < 1 ms |
| rosbridge WebSocket | 3–10 ms |
| JTC + gz_ros2_control (sim) | 3–5 ms |
| trajectory_to_robot_bridge | < 1 ms (rate-limité 15 Hz) |
| TCP Tour → Pi | 5–20 ms |
| pymycobot → servo série | 10–30 ms par commande |
| servo → mouvement physique | variable selon speed (20–100 ms à speed 25) |
| **Total main → bras physique** | **~150–250 ms** |

Ressenti opérateur : **pas de latence perceptible** si les gestes sont fluides.

---

## Unités et conventions

| Quantité | Unité | Où |
|----------|-------|-----|
| Hand XYZ (Wilor) | **mètres** | Tout le pipeline en amont du mapping |
| Rotations hand (Euler) | **degrés** | xyz_to_joints_deg input |
| Joint angles (internes mycobot_teleop) | **degrés** | xyz_to_joints_deg output + EMA/slew |
| Joint angles (JointTrajectory ROS) | **RADIANS** | `/mycobot_controller/joint_trajectory` |
| Joint angles (JSON /to_robot) | **DEGRÉS** | trajectory_to_robot_bridge reconvertit |
| Joint angles (pymycobot) | **degrés** | côté Pi, attendu par `mc.send_angles()` |
| Joint angles (URDF limits) | **radians** | Dans les `<limit lower="-2.93" upper="2.93">` de l'URDF |

**Attention pièges** :
1. `ros2 topic pub --once /mycobot_controller/joint_trajectory ... positions: [45, ...]` envoie **45 radians** (qui sera clampé aux limites URDF ≈ ±3 rad = ±170°). Pour envoyer 45°, utiliser `positions: [0.785, ...]` (π/4).
2. Le robot physique répond via `bridge_pi_simple.py` → `ANGLES: [-0.35, ...]` — ces valeurs SEMBLENT être des radians mais sont en fait des **degrés** (pymycobot convention). Un angle `-0.35` = -0.35° = quasi-zéro.

---

## Résumé des topics et types

| Topic | Type | Dir | Rôle |
|-------|------|-----|------|
| `/teleop/hand_xyz` | `geometry_msgs/Vector3Stamped` | teleop → dashboard | Monitoring XYZ main |
| `/teleop/gains` | `std_msgs/Float64MultiArray` | dashboard → teleop | Live tuning gains |
| `/teleop/recalibrate` | `std_msgs/Empty` | dashboard → teleop | Reset initial_pose |
| `/mycobot_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | teleop → JTC + trajectory_to_robot_bridge | **Commande principale** (rad) |
| `/joint_states` | `sensor_msgs/JointState` | JTC → dashboard | Actual positions (sim seulement) |
| `/gripper_position_controller/commands` | `std_msgs/Float64MultiArray` | teleop → JTC gripper | Pince sim (pas utilisé sur réel actuel) |
| `/to_robot` | `std_msgs/String` | trajectory_to_robot_bridge → bridge_tour | JSON commande robot |
| `/from_robot` | `std_msgs/String` | bridge_tour → (listeners) | Réponses Pi (logs, status) |

---

*Dernière mise à jour : 22 avril 2026 — après premier test sur robot physique.*
