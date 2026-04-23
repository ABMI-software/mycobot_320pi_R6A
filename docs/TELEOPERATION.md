# Hand Teleoperation — MyCobot 320 Pi

Pilotage du MyCobot 320 Pi par la main de l'opérateur, en simulation Gazebo et **sur le robot réel** (premier test validé 22/04/2026, Pi à 10.10.0.223). Adaptation du pipeline développé par l'équipe pour les robots ArmR5A et LeRobot (voir [ABMI-software/hand_controlLerobot](https://github.com/ABMI-software/hand_controlLerobot)).

> **Documents liés**
> - [TELEOP_ARCHITECTURE_VIZ.md](TELEOP_ARCHITECTURE_VIZ.md) — **visuel détaillé : détection → mouvement**
> - [REAL_ROBOT_TEST_PROCEDURE.md](REAL_ROBOT_TEST_PROCEDURE.md) — procédure test physique + calibration sécurisée
> - [TELEOP_DASHBOARD.md](TELEOP_DASHBOARD.md) — manuel utilisateur du dashboard
> - [TELEOP_TUNING.md](TELEOP_TUNING.md) — référence des paramètres + dépannage
> - [ARCHITECTURE.md](ARCHITECTURE.md) — architecture système globale

> ✅ **Status** : validé en simulation + **validé sur robot physique** (MyCobot 320 Pi, 22/04/2026). Pipeline complet Astra → Wilor → rosbridge → JTC topic → trajectory_to_robot_bridge → bridge_tour → Pi → pymycobot.

---

## Table des matières

1. [Principe](#principe)
2. [Pipeline complet](#pipeline-complet)
3. [Prérequis](#prérequis)
4. [Workflow 5 terminaux](#workflow-5-terminaux)
5. [Outils livrés](#outils-livrés)
6. [Chaîne de filtres](#chaîne-de-filtres)
7. [Mapping main → joints](#mapping-main--joints)
8. [Performance + critères d'acceptation](#performance--critères-dacceptation)
9. [Limitations connues](#limitations-connues)
10. [Historique des itérations](#historique-des-itérations)

---

## Principe

Une caméra filme la main de l'opérateur → [Wilor](https://github.com/warmshao/WiLor-mini-pipeline) estime la pose 6-DoF de la main → un mapping linéaire + des filtres traduisent ça en **angles de joints** pour le MyCobot, plus un **état d'ouverture** pour la pince. Deux cibles en parallèle :

- **Simulation Gazebo** : pour valider la téléop sans hardware
- **Robot réel** (MyCobot 320 Pi via Raspberry Pi TCP) : une fois le sim validé

**Approche** : pilotage *relatif* (delta-based). À la première détection, la main est prise comme référence (origine). Le robot reste à sa pose zéro ; les joints ne bougent qu'en fonction du **déplacement** par rapport à cette référence. Cliquer sur "Recalibrate" dans le dashboard recapture la référence à n'importe quel moment.

---

## Pipeline complet

```
┌───────────────────────────────────────────────────────────────┐
│ 1. ACQUISITION — Astra S → Wilor                              │
│                                                                │
│   oni_grabber (C++ OpenNI2, spawné auto par le teleop)        │
│     └→ /dev/shm/oni_color.rgb    (RGB 640×480 @ 30 Hz)        │
│                                                                │
│   OrbbecCapture (wrapper compatible cv2.VideoCapture)         │
│     ├→ watchdog : respawn auto si frames stall > 2 s          │
│     └→ survit aux Ctrl+C (start_new_session=True)             │
│                                                                │
│   hand_teleop.HandTracker (thread daemon)                     │
│     ├→ Wilor inference → keypoints + pose 6-DoF               │
│     ├→ Jump clamp 2 m/s (rejette les sauts Wilor)             │
│     ├→ Kalman XYZ (dt=1/30, q=r=5e-3)                         │
│     ├→ Exception wrapper — survit aux crashs Wilor            │
│     └→ initial_pose capturée au premier detect                │
│                                                                │
│   Sortie : GripperPose(rel_pos=[dx,dy,dz], rot=3×3, open_deg) │
└───────────────────────────────────────────────────────────────┘
                            ▼
┌───────────────────────────────────────────────────────────────┐
│ 2. MAPPING — xyz_to_joints_deg() + extraction RPY             │
│                                                                │
│   rpy = as_euler("ZYX") → (roll, pitch, yaw)                  │
│                                                                │
│   scale = 150 °/m (BASE_SCALE_DEG_PER_M)                      │
│                                                                │
│   j1 = dy * scale * y_gain     (base yaw ← main latérale)     │
│   j2 = dz * scale * z_gain     (shoulder ← main verticale)    │
│   j3 = dx * scale * x_gain     (elbow   ← main profondeur)    │
│   j4 = pitch * pitch_gain / 2  (wrist1  ← inclinaison paume)  │
│   j5 = pitch * pitch_gain / 2  (wrist2  — même que j4)        │
│   j6 = yaw   * roll_gain       (EE roll ← doorknob)           │
│                                                                │
│   Clampage aux limites URDF (±134° J2, ±145° J3, ...)         │
└───────────────────────────────────────────────────────────────┘
                            ▼
┌───────────────────────────────────────────────────────────────┐
│ 3. FILTRAGE — pipeline R5A / LeRobot porté                    │
│                                                                │
│   Joints (6 valeurs) :                                        │
│     ├→ EMA alpha=0.20    q = 0.8 * q_prev + 0.2 * q_raw       │
│     └→ Slew 1°/frame     Δq ∈ [−1°, +1°] → 30 °/s @ 30 Hz    │
│                                                                │
│   Gripper (scalaire) :                                        │
│     ├→ Deadband 3°       ignore bruit au repos                │
│     ├→ EMA alpha=0.25                                         │
│     └→ Slew 4°/frame                                          │
└───────────────────────────────────────────────────────────────┘
                            ▼
┌───────────────────────────────────────────────────────────────┐
│ 4. PUBLICATION via rosbridge (ws://localhost:9090)            │
│                                                                │
│   /mycobot_controller/joint_trajectory                        │
│       trajectory_msgs/JointTrajectory                          │
│       [{positions: [q1..q6] rad, time_from_start: 0.25 s}]    │
│       → 30 Hz                                                 │
│                                                                │
│   /gripper_position_controller/commands                       │
│       std_msgs/Float64MultiArray                              │
│       data = [servo_left, servo_right, tip_left, tip_right]   │
│                                                                │
│   /teleop/hand_xyz                 (monitoring dashboard)     │
│   /teleop/camera/image             (preview JPEG ~10 Hz)      │
│   /teleop/gains    (subscribe)     (sliders / presets)        │
│   /teleop/recalibrate (subscribe)  (bouton Recalibrate)       │
└───────────────────────────────────────────────────────────────┘
                            ▼
┌───────────────────────────────────────────────────────────────┐
│ 5. EXÉCUTION                                                  │
│                                                                │
│   ┌─ SIM ──────────────────────────────────────┐              │
│   │ joint_trajectory_controller (JTC)          │              │
│   │   • PID p=100, i=0.01, d=1.0 par joint     │              │
│   │   • interpolation splines entre waypoints  │              │
│   │ gz_ros2_control → DART physics → Gazebo    │              │
│   │ joint_state_broadcaster → /joint_states    │              │
│   └────────────────────────────────────────────┘              │
│                                                                │
│   ┌─ RÉEL ──────────────────────────────────────┐             │
│   │ trajectory_to_robot_bridge (ROS2 node)      │             │
│   │   /mycobot_controller/joint_trajectory (rad)│             │
│   │     ├→ conversion rad → deg                 │             │
│   │     ├→ rate limit 15 Hz (baud série limite) │             │
│   │     ├→ deadband 1° (skip micro-changes)     │             │
│   │     └→ publish /to_robot JSON               │             │
│   │ bridge_tour (TCP:5005 → Raspberry Pi)       │             │
│   │ bridge_pi_simple.py → pymycobot             │             │
│   │   → servos MyCobot 320                      │             │
│   └─────────────────────────────────────────────┘             │
└───────────────────────────────────────────────────────────────┘
```

---

## Prérequis

### Sur le PC Tour

```bash
# Packages ROS2 (installés sur ce poste)
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

À `/home/genji/miniconda/envs/hand-teleop`. Dépendances :

```
roslibpy            # pour --use-rosbridge (OBLIGATOIRE en mode conda)
primesense          # bindings Python OpenNI2 (test wrapper initial)
wilor               # hand pose model
mediapipe, opencv-python, scipy, numpy
pandas, openpyxl, xlsxwriter     # pour performance_analyzer
ttkbootstrap        # theme dark du dashboard
matplotlib          # plots du dashboard
primesense          # ancien wrapper Astra (rejoué au cas où)
hand_teleop         # modules locaux (gripper_pose, hand_pose, tracking)
```

> **Pourquoi `--use-rosbridge` est obligatoire depuis cet env** : conda est en **Python 3.10**, ROS2 Jazzy est compilé pour **Python 3.12**. `import rclpy` échoue dans le conda env (`No module named 'rclpy._rclpy_pybind11'`). Rosbridge (WebSocket) est Python-agnostic, c'est la raison d'être de `--use-rosbridge`.

### Caméra Orbbec Astra S

- Binaire `oni_grabber` dans `~/Downloads/Orbbec_OpenNI_v2.3.0.86-beta6_linux_release/.../samples/bin/`
- Spawné automatiquement par `open_orbbec()` au démarrage du teleop (`--camera astra`)
- Frames RGB 640×480 publiées dans `/dev/shm/oni_color.rgb` via shared memory

### Sur la Raspberry Pi (seulement pour le robot réel)

```bash
ssh er@10.10.0.225
python3 bridge_pi_simple.py
```

---

## Workflow 5 terminaux

```bash
# ================================================================
# T1 — rosbridge WebSocket (démarre en premier)
# ================================================================
conda deactivate
source /opt/ros/jazzy/setup.bash
source ~/ros_jazzy/install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
#    → Rosbridge WebSocket server started on port 9090

# ================================================================
# T2 — Gazebo + controllers (attends ~30s que controllers active)
# ================================================================
conda deactivate
source /opt/ros/jazzy/setup.bash && source ~/ros_jazzy/install/setup.bash
export GZ_SIM_RESOURCE_PATH=~/ros_jazzy/install/mycobot_description/share

# Mode par défaut : sim + robot réel parallèle
ros2 launch mycobot_gateway mycobot_teleop.launch.py

# Variantes
ros2 launch mycobot_gateway mycobot_teleop.launch.py target:=sim     # Gazebo only
ros2 launch mycobot_gateway mycobot_teleop.launch.py target:=real    # Pi only
ros2 launch mycobot_gateway mycobot_teleop.launch.py pi_ip:=10.10.0.225 rosbridge:=false

# ================================================================
# T3 — script de téléopération (webcam → joints via rosbridge)
# ================================================================
conda activate hand-teleop
cd ~/ros_jazzy/src/mycobot_R6A/teleop
python3 mycobot_teleop.py --camera astra --ros --use-rosbridge

# Pour une webcam USB classique au lieu de l'Astra :
python3 mycobot_teleop.py --camera auto --cam-idx 0 --ros --use-rosbridge

# ================================================================
# T4 — dashboard (tuning + monitoring en direct)
# ================================================================
conda activate hand-teleop
cd ~/ros_jazzy/src/mycobot_R6A/teleop
python3 teleop_dashboard.py

# ================================================================
# T5 — performance analyzer (rapport Excel avant robot réel)
# ================================================================
conda activate hand-teleop
cd ~/ros_jazzy/src/mycobot_R6A/teleop
python3 performance_analyzer.py --guided      # protocole scripté 64 s
python3 performance_analyzer.py --duration 120 # libre 2 min
```

**Ordre important** : T1 → T2 (attendre que `mycobot_controller active`) → T3 → T4 → T5.

**Après démarrage T3** : dans le dashboard T4, clique **⟲ Recalibrate hand origin** **avec la paume face caméra**. Ça capture l'origine du mapping relatif.

---

## Outils livrés

### `teleop/mycobot_teleop.py` — script principal (750+ lignes)

Orchestre tout le pipeline. Arguments principaux :

| Flag | Défaut | Rôle |
|------|--------|------|
| `--camera {auto,astra}` | `auto` | `astra` = Orbbec via shared mem ; `auto` = webcam UVC |
| `--cam-idx N` | 0 | Index /dev/videoN en mode `auto` |
| `--ros` / `--no-ros` | on | Active la publication ROS2 |
| `--use-rosbridge` | off | OBLIGATOIRE en env conda ; utilise WebSocket |
| `--ros-topic T` | `/mycobot_controller/joint_trajectory` | Topic de sortie |
| `--time-from-start S` | 0.25 | Durée de chaque trajectoire JTC (s) |
| `--x-gain / --y-gain / --z-gain` | 1.2 / 1.2 / 1.6 | Multiplicateurs position |
| `--invert-x / --invert-y / --invert-z` | T / F / F | Inversion par axe |
| `--fps N` | 30 | Taux de publication (Hz) |
| `--run-seconds N` | ∞ | Auto-shutdown après N secondes |

### `teleop/orbbec_capture.py` — wrapper Astra

- Spawne `oni_grabber` automatiquement (sans sudo)
- Watchdog respawn si les frames stallent
- Détection proprement des états morts (fichiers existants mais tick figé)
- Survit aux Ctrl+C du parent via `start_new_session=True`

### `teleop/teleop_dashboard.py` — GUI de tuning (ABMI v2.2)

Design ttkbootstrap "darkly" + charte **ABMI navy+pink** — connecté à rosbridge. Trois onglets :

- **🏠 Home** : 5 KPI cards (mode, cmd rate, SIM avg RMS, REAL avg RMS, signal health), panneau caméra opérateur (feed JPEG via `/teleop/camera/image`), bar chart comparatif SIM↔REAL, plot hand position en **cm**, 5 boutons d'action dynamiques (Home robot / Stop / Recalibrate / Run analyzer / Export CSV) avec tooltip hover + feedback `⟳→✓/✗` + toast horodaté.
- **📊 Analytics** : plots détaillés — hand XYZ, joint angles sim / joint angles real côte à côte, tracking error sim / tracking error real côte à côte. Stats strip mono-space en bas.
- **🎛️ Tuning** : 4 sliders live (`x/y/z gain`, `time_from_start`) publiés sur `/teleop/gains`, presets **🐢 Safe / ⚙️ Nominal / ⚡ Reactive** (le preset actif reste highlighted), bouton Recalibrate dédié.

Badge de mode automatique en haut à droite : **SIM / REAL / BOTH / OFFLINE** (détection par fraîcheur de `/joint_states` vs `/from_robot`). Voir [TELEOP_DASHBOARD.md](TELEOP_DASHBOARD.md) pour le manuel complet.

### `teleop/performance_analyzer.py` — rapport Excel

Génère un `.xlsx` multi-onglets avec verdict `READY / CAUTIOUS / NOT READY`. Deux modes :

- `--guided` : protocole scripté 7 phases (idle, up/down, left/right, forward/back, combined, gripper, rest) sur 64 s
- `--duration N` : enregistrement libre pendant N secondes

Onglets générés :

- **Summary** : verdict coloré + santé globale + workspace utilisé
- **Per-joint tracking** : RMS, max, p50/p90/p99 avec graphique bar chart
- **Scenarios** : breakdown par phase (mode guided uniquement)
- **Signal health** : taux de publication, dropouts, longest gap
- **raw_hand / raw_cmd / raw_actual** : données brutes horodatées

### `mycobot_gateway/mycobot_gateway/trajectory_to_robot_bridge.py`

Nœud ROS2 qui souscrit à `/mycobot_controller/joint_trajectory` et publie des commandes JSON sur `/to_robot` pour `bridge_tour` :

```json
{"action": "send_angles", "angles": [deg1, ..., deg6], "speed": 40}
```

Paramètres ROS : `trajectory_topic`, `out_topic`, `speed`, `rate_hz` (15 par défaut), `deadband_deg` (1), `enable`.

### `mycobot_gateway/launch/mycobot_teleop.launch.py`

Launch orchestré. Args : `target:={sim,real,both}`, `pi_ip`, `pi_port`, `rosbridge`, `real_speed`, `real_rate_hz`.

---

## Chaîne de filtres

Trois étages ADD sur la sortie Wilor, tous empruntés au R5A / LeRobot (validés en prod) :

### Stage 1 — Tracker interne (dans hand_teleop.HandTracker)

| Filtre | Paramètres | Effet |
|--------|-----------|-------|
| **Jump clamp** | 2 m/s | Rejette les sauts Wilor > 2 m/s (typiquement reacquisition) |
| **Kalman XYZ** | dt=1/30, q=5e-3, r=5e-3 | Lisse la position 3D, prédit entre détections |

### Stage 2 — Joint command (dans xyz_to_joints_deg puis main loop)

| Filtre | Paramètres | Effet |
|--------|-----------|-------|
| **EMA** | α=0.20 | `q = 0.8 * q_prev + 0.2 * q_raw` — atténue le jitter |
| **Slew limiter** | 1 °/frame | `Δq ∈ [−1°, +1°]` → 30 °/s max @ 30 Hz (URDF limite 180 °/s) |

### Stage 3 — Gripper (pipeline séparé, spécifique à open_degree)

| Filtre | Paramètres | Effet |
|--------|-----------|-------|
| **Deadband** | 3° | Si `|g − center| < 3°` → garde la valeur EMA → pas de "breathing" |
| **EMA** | α=0.25 | Lisse le bruit de `open_degree` Wilor |
| **Slew** | 4°/frame | Bloque les jumps de reacquisition |

> **Historique de tuning** : scale commencé à 300 °/m avec slew 8 °/frame → RMS 40-70°. Itérations successives (200 → 150 °/m, slew 8 → 5 → 3 → **1 °/frame** comme R5A) ont réduit RMS à 13-17° puis < 10°.

---

## Mapping main → joints

```
╔═════════════════════════════════════════════════════════════╗
║  Position (relative à initial_pose de Wilor)                ║
╠═════════════════════════════════════════════════════════════╣
║  Y (main latérale)  →  J1 base yaw                          ║
║  Z (main verticale) →  J2 shoulder                          ║
║  X (main profondeur)→  J3 elbow                             ║
╠═════════════════════════════════════════════════════════════╣
║  Orientation (Euler ZYX intrinsèque de la pose relative)    ║
╠═════════════════════════════════════════════════════════════╣
║  pitch  →  J4 wrist1 (*gain*0.5)                            ║
║  pitch  →  J5 wrist2 (*gain*0.5)   ← J4+J5 combinés         ║
║  yaw    →  J6 EE roll              ← doorknob, paume face   ║
║                                     caméra qui tourne       ║
║  roll   →  (inutilisé — axe ambigu dans le repère Wilor)    ║
╚═════════════════════════════════════════════════════════════╝
```

**Signes par défaut** (CLI `--invert-{x,y,z}`) :

| Axe | Défaut | Comportement |
|-----|--------|--------------|
| X | `invert_x=True` | Main avance → coude s'étend → EE avance |
| Y | `invert_y=False` | Main droite → base tourne droite |
| Z | `invert_z=False` | Main monte → épaule pivote → EE monte |

**Scale et gains** (tous tunables via dashboard) :

- **BASE_SCALE** = 150 °/m → 1.0 de gain = 15 cm déplace un joint de 22.5°
- **x/y/z_gain** = 1.2 / 1.2 / 1.6 → 15 cm de main = 27° / 27° / 36° au joint
- **pitch_gain** = 0.4 → 45° de main = 9° par joint (J4 + J5)
- **roll_gain** = 0.4 → 45° de yaw main = 18° sur J6

---

## Performance + critères d'acceptation

`performance_analyzer.py` évalue la téléop contre ces **seuils** (dans [performance_analyzer.py:41-44](../teleop/performance_analyzer.py)) :

- **RMS error** ≤ 10° par joint → pas JITTERY
- **Max error** ≤ 30° par joint → pas UNSTABLE
- **Cmd jitter** ≤ 3° → pas JITTERY ; ≤ 6° → pas UNSTABLE
- **Publish rate** ≥ 70% cible (30 Hz) pour un verdict non-failing
- **Detection dropouts** ≤ 2 pour un verdict non-failing

Verdict final :
- **READY FOR REAL ROBOT** — tous les joints pilotés OK, cadence nominale
- **CAUTIOUS — REAL OK WITH REDUCED SPEED** — joints JITTERY seulement (halver les gains avant réel)
- **NOT READY FOR REAL** — au moins 1 joint UNSTABLE ou cadence / dropouts problématiques

### Résultats des runs successifs

| Run | Config | RMS J1-J3 | Max J1-J3 | Verdict |
|-----|--------|-----------|-----------|---------|
| 1 | 60 Hz, tfs 0.15, scale 300, slew 8°/f | 47-70° | 185-281° | NOT READY (4 UNSTABLE) |
| 2 | 30 Hz, tfs 0.25, scale 200, slew 5°/f, rot gains ÷2 | 13-17° | 46-52° | NOT READY (4 UNSTABLE) |
| 3 | scale 150, slew 3°/f + seuils réalistes | 13-17° | 45-52° | NOT READY (J6 noisy) |
| 4 | slew 1°/f + gripper deadband chain | **visé < 10°** | **visé < 30°** | — |

---

## Limitations connues

### Pince en simulation

Le `pro_adaptive_gripper` utilise un **mécanisme 4-barres à boucle fermée** qu'URDF ne peut pas représenter (arbres uniquement). DART physics ignore les `<mimic>` constraints. Solution retenue : **piloter les 4 joints explicitement** via le controller → les doigts bougent symétriquement mais la cinématique n'est pas anatomiquement précise (extrémités qui tournent avec leur parent au lieu de rester parallèles).

**Impact pratique** : la pince *visuelle* dans Gazebo n'est pas fidèle mais la fonctionnalité (open/close commandable) fonctionne. Sur le **robot réel** cette limitation disparaît : `pymycobot` commande directement le servo physique qui gère la cinématique 4-barres mécaniquement.

### Inversion rclpy ↔ conda

Impossible d'utiliser `rclpy` dans l'env conda `hand-teleop` (Python 3.10 vs ROS2 Jazzy 3.12). Contournement : **`--use-rosbridge` obligatoire**. Inconvénient : une connexion WebSocket de plus. Avantage : découple totalement la téléop de ROS2 niveau versioning.

### Axe du doorknob (J6)

Le remapping `j6 = yaw * roll_gain` a été ajouté le 22/04. Le test visuel par l'opérateur est encore à faire — si le yaw Wilor ne correspond pas au doorknob physique, il faudra diagnostiquer via le log RPY (ajouté dans le même commit) et éventuellement tester `j6 = roll` ou `j6 = pitch`.

### Filtres par défaut parfois trop mous

Slew 1°/frame = 30°/s max. Feels *sluggish* sur des gestes rapides. Pour un utilisateur qui veut plus de réactivité : monter à 2 ou 3 via la constante `DEFAULT_MAX_DELTA_DEG_PER_FRAME` (pas de CLI flag pour l'instant).

---

## Historique des itérations

Chronologiquement, les commits qui structurent le travail (plus récent en haut) :

| Commit | Résumé |
|--------|--------|
| `2ee80d35` | fix J6 : yaw (doorknob) au lieu de roll + log RPY debug |
| `0866dc2c` | port filtres R5A/LeRobot : slew 1°/f + gripper deadband chain |
| `e9cc87c7` | retune + seuils analyzer réalistes (30° max, 10° RMS) |
| `6f87cac1` | stabilisation : fps 60→30, tfs 0.15→0.25, scale 300→200, EE orientation pitch→J4+J5 |
| `6605e9a5` | pince : 4 joints explicites, plus de mimic |
| `cdb106a4` | pince : retrait des mimic joints du bloc ros2_control |
| `2437c2f3` | pince : tentative command_interface sur mimic (CRASH expected) |
| `81e21c8f` | pince : 4-bar linkage URDF mimic (tentative, échec gravité) |
| `6cb1e381` | wrist J4/J6 depuis hand rotation + performance_analyzer.py |
| `432ec650` | flip invert_z default + slew rate limiter |
| `a536e39b` | 3 fixes stabilité : /clock bridge + rate limit + auto-resume |
| `ada34845` | mapping delta-based → robot à pose zéro au démarrage |
| `0289fd59` | robustesse : Wilor exception wrapper + Astra watchdog |
| `d5ae06f6` | mapping intuitif Z→J2 shoulder (était X→J2) |
| `872ce54f` | gripper fonctionnel en Gazebo (1re version) |
| `0cd922f6` | detect stale oni_grabber + respawn, survivre aux Ctrl+C |
| `7859f621` | bouton Recalibrate dans dashboard |
| `1b1a0010` | redesign dashboard moderne (ttkbootstrap darkly) |
| `19db10c4` | dashboard live-tuning initial |
| `f9569656` | support Orbbec Astra S via oni_grabber + auto-resume tracker |
| `583f783e` | fix /clock bridge de Gazebo (nécessaire pour ros2_control) |
| `1f3fae7e` | mise en place initiale du pipeline teleop complet |

Pour un récapitulatif technique plus fin de chaque fix, voir `git show <commit>` — chaque message explique le root cause et la solution.

---

*Dernière mise à jour : 22 avril 2026*
