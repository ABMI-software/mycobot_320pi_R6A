# Teleop — Référence des paramètres + dépannage

Référence exhaustive de toutes les constantes, flags et paramètres qui pilotent la téléopération. Utile quand tu veux modifier le comportement par défaut ou diagnostiquer un souci spécifique.

> Documents liés : [TELEOPERATION.md](TELEOPERATION.md) — pipeline, [TELEOP_DASHBOARD.md](TELEOP_DASHBOARD.md) — GUI

---

## 1. Constantes de module (`mycobot_teleop.py`)

En haut du fichier, tu trouves ces constantes qui définissent le comportement par défaut. Change-les si tu veux une base différente pour tout le monde.

### Mapping position

```python
BASE_SCALE_DEG_PER_M = 150.0
```
Combien de degrés de joint par mètre de déplacement de main, au gain 1.0.
- **Baisser** (100) = besoin de plus de gestuelle pour le même angle
- **Monter** (300) = risque de saturation immédiate aux limites

### Filtres sur commande joints

```python
DEFAULT_COMMAND_EMA_ALPHA = 0.20          # 0.2 → dampen
DEFAULT_MAX_DELTA_DEG_PER_FRAME = 1.0     # → 30°/s @ 30 Hz
```
- `EMA alpha = 0.20` → chaque frame, la commande publie 20% de la nouvelle cible + 80% de l'ancienne
- `MAX_DELTA = 1°/frame` → même si l'EMA veut faire un saut plus grand, on clip

**Comment tuner** :
| Symptôme | Action |
|----------|--------|
| Robot traîne derrière la main | Monter EMA alpha (0.20 → 0.30) + slew (1 → 2) |
| Robot oscille ou instable | Baisser EMA alpha (0.15) + slew (0.5) |

### Filtres sur le gripper

```python
GRIPPER_DEADBAND_DEG = 3.0     # seuil d'activation depuis le centre
GRIPPER_EMA_ALPHA = 0.25
GRIPPER_MAX_STEP_DEG = 4.0     # slew maximum par frame
```

### Enveloppe hand pose (SAFE_RANGE)

```python
SAFE_RANGE = {
    "x": (0.13, 0.36),   # profondeur — distance main à caméra
    "y": (-0.23, 0.23),  # latéral — gauche/droite
    "z": (0.008, 0.25),  # vertical — bas/haut
    "g": (2, 90),        # grip open_degree — 2° fermé, 90° ouvert
}
```
Définit l'envelope Wilor qu'on considère "normale". Utilisé uniquement par le gripper deadband (center = 46°).

### Limites articulaires (JOINT_LIMITS_DEG)

```python
JOINT_LIMITS_DEG = {
    "joint2_to_joint1":      (-168.0, 168.0),  # J1 base yaw
    "joint3_to_joint2":      (-134.6, 134.6),  # J2 shoulder
    "joint4_to_joint3":      (-145.0, 145.0),  # J3 elbow
    "joint5_to_joint4":      (-145.0, 145.0),  # J4 wrist1
    "joint6_to_joint5":      (-168.0, 168.0),  # J5 yaw
    "joint6output_to_joint6": (-180.0, 180.0), # J6 EE roll
}
```
Tout mapping dépassant ces bornes est **clampé** dans `xyz_to_joints_deg()`. Si tu veux limiter les amplitudes commandées pour rester plus doux, resserre ces bornes.

---

## 2. Arguments de ligne de commande `mycobot_teleop.py`

### Flags principaux

| Flag | Défaut | Description |
|------|--------|-------------|
| `--camera {auto,astra}` | `auto` | `auto` = UVC via cv2 ; `astra` = Orbbec via shared memory |
| `--cam-idx N` | 0 | Index /dev/videoN en mode `auto` |
| `--hand {left,right}` | `right` | Main à tracker |
| `--model NAME` | `wilor` | Backend Wilor (pas d'alternative testée) |
| `--fps N` | 30 | Taux de publication ROS |
| `--run-seconds N` | ∞ | Auto-shutdown après N secondes |
| `--video PATH` | — | Lire depuis un fichier vidéo au lieu de la caméra |

### Flags ROS

| Flag | Défaut | Description |
|------|--------|-------------|
| `--ros` / `--no-ros` | on | Active la publication vers ROS2 |
| `--use-rosbridge` | off | **OBLIGATOIRE** en env conda ; sinon essaie rclpy direct (qui ne marche pas en 3.10) |
| `--rosbridge-host HOST` | localhost | Serveur rosbridge |
| `--rosbridge-port N` | 9090 | Port WebSocket |
| `--ros-topic T` | `/mycobot_controller/joint_trajectory` | Topic de sortie |
| `--joints NAME1,NAME2,...` | 6 joints URDF | Ordre des joints dans le message |
| `--time-from-start S` | 0.25 | Durée de chaque trajectoire (s) |

### Flags mapping / gains

| Flag | Défaut | Description |
|------|--------|-------------|
| `--x-gain F` | 1.2 | Multiplicateur scale sur axe X (profondeur → J3) |
| `--y-gain F` | 1.2 | Multiplicateur sur axe Y (latéral → J1) |
| `--z-gain F` | 1.6 | Multiplicateur sur axe Z (vertical → J2 + J5) |
| `--invert-x` / `--no-invert-x` | True | Inverser le signe X |
| `--invert-y` / `--no-invert-y` | False | Inverser le signe Y |
| `--invert-z` / `--no-invert-z` | False | Inverser le signe Z |

### Flags rotation (wrist)

Pour modifier les gains rotation (roll → J6, pitch → J4+J5), il faut éditer directement les défauts dans la signature de `xyz_to_joints_deg()` — pas de CLI flag pour ces paramètres :
```python
roll_gain: float = 0.4,     # hand yaw (doorknob) → J6
pitch_gain: float = 0.4,    # hand pitch → J4 + J5
invert_roll: bool = False,
invert_pitch: bool = False,
```

---

## 3. Arguments dashboard (`teleop_dashboard.py`)

Le dashboard n'a **aucun argument CLI**. Tout se configure via les sliders. Éditable dans `teleop/teleop_dashboard.py` si tu veux changer les limites min/max des sliders :

```python
# Dans _build_gains_panel :
self._make_slider(frame, 0, "X gain — ...", "x", 0.3, 3.0, INFO)
#                                               │    │
#                                               lo   hi
```

---

## 4. Arguments `performance_analyzer.py`

| Flag | Défaut | Description |
|------|--------|-------------|
| `--guided` | — | Protocole scripté 7 phases (64 s total) |
| `--duration N` | 60 | Enregistrement passif de N secondes (ignored si `--guided`) |
| `--host H` | localhost | Rosbridge host |
| `--port N` | 9090 | Rosbridge port |
| `--out PATH` | `teleop_report_<timestamp>.xlsx` | Fichier de sortie |

### Seuils d'acceptation (constantes du module)

```python
ERR_MAX_OK = 30.0       # max tracking error acceptable (°)
ERR_RMS_OK = 10.0       # RMS error acceptable (°)
JITTER_OK = 3.0         # cmd jitter acceptable (°)
PUBLISH_RATE_TARGET_HZ = 30.0
```

Si tu veux un verdict plus strict / laxiste, édite ces 4 constantes dans `performance_analyzer.py`.

### Phases du protocole guidé

Définies dans `GUIDED_PROTOCOL` en haut du fichier. Chaque phase est un `Scenario(name, instruction, duration_s)`. Tu peux ajouter, retirer ou changer les durées.

---

## 5. Arguments launch `mycobot_teleop.launch.py`

```bash
ros2 launch mycobot_gateway mycobot_teleop.launch.py \
    target:=sim          \   # {sim, real, both} — défaut both
    pi_ip:=10.10.0.225   \   # IP du Raspberry Pi
    pi_port:=5005        \   # Port TCP bridge_tour
    rosbridge:=true      \   # Démarrer rosbridge_server automatiquement
    real_speed:=40       \   # Vitesse pymycobot (0-100)
    real_rate_hz:=15.0       # Rate limit TCP vers le Pi (15 Hz max)
```

---

## 6. Paramètres du `trajectory_to_robot_bridge` (real robot path)

Définis dans le node Python et exposés via ROS params. Modifiable via ligne de commande ou le launch file :

```python
self.declare_parameter("trajectory_topic", "/mycobot_controller/joint_trajectory")
self.declare_parameter("out_topic", "/to_robot")
self.declare_parameter("speed", 40)              # vitesse servo pymycobot
self.declare_parameter("rate_hz", 15.0)          # limite publication (baud série)
self.declare_parameter("deadband_deg", 1.0)      # skip si |Δ| < 1°
self.declare_parameter("enable", True)           # toggle on/off
```

---

## 7. Configuration controller.yaml

Dans `mycobot_description/config/controller.yaml` — gains PID du JTC (Gazebo seulement) :

```yaml
mycobot_controller:
  ros__parameters:
    gains:
      joint2_to_joint1:       {p: 100.0, i: 0.01, d: 1.0}
      joint3_to_joint2:       {p: 100.0, i: 0.01, d: 1.0}
      ...
```

Effet :
- **Augmenter p** → tracking plus serré, risque oscillation
- **Augmenter d** → amortir les oscillations
- **Augmenter i** → réduire l'erreur statique (bouger plus vite vers la cible)

Gripper — 4 joints avec PID softer (p=60 servos, p=30 tips) :
```yaml
gripper_position_controller:
  gains:
    gripper_controller:                 {p: 60.0, ...}
    gripper_base_to_gripper_right3:     {p: 60.0, ...}
    gripper_left3_to_gripper_left1:     {p: 30.0, ...}
    gripper_right3_to_gripper_right1:   {p: 30.0, ...}
```

---

## 8. Dépannage

### "Wilor ne détecte pas ma main"

Symptômes : `hand_xyz: 0` dans le dashboard, XYZ figé dans la console T3.

- **Éclairage** : Wilor marche mal en contre-jour ou dans le noir. Éclaire ta main directement.
- **Cadrage** : paume grande ouverte, doigts écartés, ~50 cm de la caméra.
- **Couleur** : un arrière-plan contrasté par rapport à la peau aide.
- **Si première fois** : attends 30 s que le modèle se charge.

### "Le robot saute brutalement au démarrage"

Probablement `initial_pose` capturée avant que ta main soit bien dans le cadre. Clique **⟲ Recalibrate** avec ta main bien visible.

### "Une direction est inversée"

Flip le flag correspondant :
```bash
python3 mycobot_teleop.py --no-invert-x    # si avance → arrière
python3 mycobot_teleop.py --invert-y       # si gauche/droite inversé
python3 mycobot_teleop.py --no-invert-z    # si haut/bas inversé
```

### "J6 ne bouge pas quand je tourne la main"

Le mapping actuel est `j6 = yaw * roll_gain`. Si physiquement tu ne vois pas J6 tourner :
1. Regarde la console T3 — chaque ligne affiche `RPY yaw,pitch,roll`. Bouge la main doorknob-style et vois quel champ change.
2. Si c'est `pitch` qui change, édite `xyz_to_joints_deg()` et remplace `j6 = yaw * roll_gain` par `j6 = pitch * roll_gain` (ou `roll * roll_gain`).

### "`cannot reach rosbridge`" au démarrage du dashboard ou analyzer

T1 n'est pas lancé. Vérifie :
```bash
nc -zv localhost 9090
```
Si KO, relance le terminal 1 :
```bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

### "`No module named 'rclpy._rclpy_pybind11'`"

Conda env activé en même temps que ROS2 sourcé. Deux règles :
- **Teleop** (conda env) : **utilise `--use-rosbridge`**, jamais rclpy direct
- **Launch files ROS2** (Gazebo, controllers, rosbridge) : `conda deactivate` avant

### "Gazebo ne charge pas l'URDF"

```bash
export GZ_SIM_RESOURCE_PATH=~/ros_jazzy/install/mycobot_description/share
```
Ajoute cette ligne avant le `ros2 launch` de T2.

### "`oni_grabber: stale tick`"

Le wrapper Astra détecte automatiquement un `oni_grabber` mort et le respawn. Si ça boucle :
```bash
pkill -9 oni_grabber
rm -f /dev/shm/oni_*
```
Puis relance T3 — le wrapper va re-spawner proprement.

### "Les joints ne sont pas commandés en Gazebo"

Vérifie les controllers actifs :
```bash
ros2 control list_controllers
```
Tu dois voir `mycobot_controller` et `gripper_position_controller` tous deux en `active`. Sinon, regarde les logs T2 — probablement `/clock` non bridgé ou URDF incomplet.

### "Le verdict reste NOT READY même avec des gains minimalistes"

Diagnostics :
1. Ouvre le `.xlsx` généré, onglet **Scenarios** — identifie quelle(s) phase(s) causent les UNSTABLE
2. Onglet **raw_cmd + raw_actual** — trace manuellement pour voir où l'erreur pique
3. Si c'est sur `idle` (main immobile) → problème de bruit Wilor / dérive initial_pose
4. Si c'est sur `combined` → problème de couplage inter-joints → baisser tous les gains de 50%

---

## 9. Environnements d'exécution

| Script | Python | Env | Notes |
|--------|--------|-----|-------|
| `ros2 launch ...` (T1, T2) | 3.12 | ROS2 Jazzy système | `conda deactivate` avant |
| `mycobot_teleop.py` (T3) | 3.10 | conda `hand-teleop` | `--use-rosbridge` obligatoire |
| `teleop_dashboard.py` (T4) | 3.10 | conda `hand-teleop` | Tkinter + ttkbootstrap |
| `performance_analyzer.py` (T5) | 3.10 | conda `hand-teleop` | pandas + xlsxwriter |
| `bridge_pi_simple.py` | 3.x | sur la Pi uniquement | pymycobot |

---

*Dernière mise à jour : 22 avril 2026*
