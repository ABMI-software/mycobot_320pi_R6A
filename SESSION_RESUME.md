# 📋 Résumé de Session - MyCobot Gateway Bridge

> **Date de dernière mise à jour :** 26 mars 2026  
> **Point de départ pour les prochaines sessions de développement**

---

## 🎯 Architecture du Projet

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              PC TOUR (10.10.0.115)                          │
│                         ROS2 Jazzy / Ubuntu 24.04 / Python 3.12             │
├─────────────────────────────────────────────────────────────────────────────┤
│                                                                             │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │ simple_gui  │  │slider_control│  │teleop_keyb. │  │marker_follow│        │
│  │  (Tkinter)  │  │(joint_states)│  │  (clavier)  │  │   (ArUco)   │        │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘        │
│         │                │                │                │               │
│         └────────────────┴────────────────┴────────────────┘               │
│                                   │                                         │
│                          /to_robot (JSON)                                   │
│                                   ▼                                         │
│                         ┌─────────────────┐                                │
│                         │   bridge_tour   │◄─── /from_robot                │
│                         │   (TCP Client)  │                                │
│                         └────────┬────────┘                                │
│                                  │ TCP:5005                                │
├──────────────────────────────────┼──────────────────────────────────────────┤
│                                  │                                          │
│                    RÉSEAU ETHERNET (10.10.0.x)                             │
│                                  │                                          │
├──────────────────────────────────┼──────────────────────────────────────────┤
│                                  ▼                                          │
│                         ┌─────────────────┐                                │
│                         │ bridge_pi_debug │                                │
│                         │  (TCP Server)   │                                │
│                         └────────┬────────┘                                │
│                                  │                                          │
│                                  ▼                                          │
│                         ┌─────────────────┐                                │
│                         │    pymycobot    │                                │
│                         │  /dev/ttyAMA0   │                                │
│                         └────────┬────────┘                                │
│                                  │                                          │
│                         ┌────────▼────────┐                                │
│                         │  MyCobot 320 Pi │                                │
│                         │     (Robot)     │                                │
│                         └─────────────────┘                                │
│                                                                             │
│                     RASPBERRY PI (10.10.0.218)                             │
│                   ROS2 Galactic / Ubuntu 20.04 / Python 3.8                │
└─────────────────────────────────────────────────────────────────────────────┘
```

---

## 📦 Structure du Package

```
mycobot_R6A/
├── SESSION_RESUME.md              # CE FICHIER
├── bridge_pi_debug.py             # Script pour le Pi (à déployer)
│
├── mycobot_gateway/               # Package ROS2 (Tour/PC)
│   ├── package.xml
│   ├── setup.py
│   │
│   ├── mycobot_gateway/           # Modules Python
│   │   ├── bridge_tour.py         # Client TCP vers Pi
│   │   ├── robot_commander.py     # Interface CLI interactive
│   │   ├── joint_sync.py          # Sync angles → RViz
│   │   ├── simple_gui.py          # 🆕 GUI Tkinter
│   │   ├── slider_control.py      # 🆕 Contrôle par sliders
│   │   ├── teleop_keyboard.py     # 🆕 Contrôle clavier
│   │   ├── marker_follower.py     # 🆕 Suivi ArUco
│   │   └── vision/
│   │       ├── marker_detector.py
│   │       └── camera_publisher.py
│   │
│   └── launch/
│       ├── bridge_only.launch.py
│       ├── commander.launch.py
│       ├── rviz_sync.launch.py
│       ├── simple_gui.launch.py        # 🆕
│       ├── slider_control.launch.py    # 🆕
│       ├── teleop_keyboard.launch.py   # 🆕
│       └── marker_follow_full.launch.py # 🆕
│
├── mycobot_description/           # URDF, meshes, config
├── docs/                          # Documentation
└── scripts/                       # Scripts shell utilitaires
```

---

## 🚀 Modes de Contrôle Disponibles

### 1. **Simple GUI** - Interface graphique
```bash
ros2 launch mycobot_gateway simple_gui.launch.py
```
- Interface Tkinter avec champs pour joints et coords
- Boutons Home, Zero, Gripper, Stop
- Affichage temps réel des valeurs

### 2. **Slider Control** - Contrôle par sliders RViz
```bash
ros2 launch mycobot_gateway slider_control.launch.py
```
- Joint State Publisher GUI avec sliders
- Le robot réel suit les sliders en temps réel
- Visualisation simultanée dans RViz

### 3. **Teleop Keyboard** - Contrôle clavier
```bash
ros2 launch mycobot_gateway teleop_keyboard.launch.py
```
- Contrôle WASD + IJKL pour déplacements
- Touches G/H pour gripper
- Positions prédéfinies (1=Init, 2=Home)

### 4. **Commander CLI** - Interface ligne de commande
```bash
ros2 launch mycobot_gateway commander.launch.py
```
- Commandes textuelles : `home`, `angles`, `open`, `close`
- Mode interactif simple

### 5. **RViz Sync** - Synchronisation robot → RViz
```bash
ros2 launch mycobot_gateway rviz_sync.launch.py
```
- RViz affiche la position réelle du robot
- Mise à jour toutes les 200ms

### 6. **Marker Follow** - Suivi de marqueur ArUco
```bash
ros2 launch mycobot_gateway marker_follow_full.launch.py
```
- Détection ArUco sur Tour
- Robot suit le marqueur automatiquement

---

## 📝 Commandes de Référence

### Prérequis (TOUJOURS faire avant ROS2)
```bash
# Désactiver conda (conflit Python 3.13 vs 3.12)
conda deactivate

# OU utiliser cette commande complète :
export PATH="/usr/bin:$PATH"
unset PYTHONPATH
source /opt/ros/jazzy/setup.bash
source ~/ros_jazzy/src/mycobot_R6A/install/setup.bash
```

### Compilation
```bash
cd ~/ros_jazzy/src/mycobot_R6A
colcon build --packages-select mycobot_gateway --symlink-install
source install/setup.bash
```

### Déploiement sur Pi
```bash
# Copier le bridge sur le Pi
scp bridge_pi_debug.py er@10.10.0.218:~/colcon_ws/src/mycobot_R6A/

# Sur le Pi : lancer le bridge
source /opt/ros/galactic/setup.bash
python3 ~/colcon_ws/src/mycobot_R6A/bridge_pi_debug.py
```

---

## 🔌 Configuration Réseau

| Machine | IP | Port | Rôle |
|---------|-----|------|------|
| PC Tour | 10.10.0.115 | - | Calcul, GUI, RViz |
| Raspberry Pi | 10.10.0.218 | 5005 | Bridge robot |

**ROS_DOMAIN_ID = 10** (si besoin d'isolation)

---

## 📨 Format des Commandes JSON

Le bridge supporte ces actions JSON :

```json
// Mouvements
{"action": "go_home"}
{"action": "go_zero"}
{"action": "send_angles", "angles": [0,0,0,0,0,0], "speed": 30}
{"action": "send_coords", "coords": [200,0,200,180,0,0], "speed": 40, "mode": 1}

// Lecture état
{"action": "get_angles"}
{"action": "get_coords"}

// Gripper
{"action": "gripper_open"}
{"action": "gripper_close"}

// Contrôle
{"action": "emergency_stop"}
{"action": "power_on"}
{"action": "power_off"}

// Vision (pour marker_follower)
{"action": "follow_on"}
{"action": "follow_off"}
```

---

## ✅ Tests Validés

| Test | Résultat |
|------|----------|
| Ping Tour → Pi | ✅ OK |
| TCP 5005 connecté | ✅ OK |
| Commande `ping` → `pong` | ✅ OK |
| LED (rouge, vert, bleu) | ✅ OK |
| `get_angles` | ✅ OK |
| `go_home` / `go_zero` | ✅ OK |
| `send_angles` | ✅ OK |
| RViz sync | ✅ OK |

---

## 🔑 Points Importants

1. **Toujours `conda deactivate`** avant ROS2 (Python 3.13 ≠ 3.12)

2. **Position HOME** : `[0, 8, -127, 40, 0, 0]` (pas zéro!)

3. **Topics ROS2** :
   - `/to_robot` : Commandes Tour → Pi (JSON)
   - `/from_robot` : Réponses Pi → Tour

4. **Ordre de démarrage** :
   1. Pi : `python3 bridge_pi_debug.py`
   2. Tour : `ros2 launch mycobot_gateway <launch_file>`

---

## 🚧 TODO / Prochaines Étapes

- [ ] Streaming caméra Pi → Tour
- [ ] Améliorer marker_follower avec paramètres ajustables
- [ ] Interface web (option future)
- [ ] Intégration Gazebo simulation + robot réel simultané
- [ ] Enregistrement/rejeu de trajectoires

---

*Fichier mis à jour le 26 mars 2026*
