# 📋 SESSION RESUME - MyCobot 320 Pi Gateway Bridge

> **Date de dernière mise à jour :** 26 mars 2026  
> **Version :** 1.0.0  
> **Repository GitHub :** https://github.com/ABMI-software/mycobot_320pi_R6A  
> **Branche :** `main`  
> **Dernier commit :** `3e2333c`

---

## 🎯 POINT DE DÉPART - Session Précédente

### ✅ Ce qui a été accompli

| Tâche | Statut | Détails |
|-------|--------|---------|
| Création repo GitHub | ✅ Complété | `ABMI-software/mycobot_320pi_R6A` |
| Architecture TCP Tour ↔ Pi | ✅ Validé | Connexion stable |
| bridge_tour (PC) | ✅ Fonctionnel | Client TCP ROS2 |
| bridge_pi_simple (Pi) | ✅ Fonctionnel | Serveur TCP + pymycobot |
| simple_gui | ✅ Testé | Interface Tkinter |
| slider_control | ✅ Testé | Robot suit les sliders en temps réel |
| RViz visualisation | ✅ Corrigé | Config avec Fixed Frame = base |
| Commandes JSON | ✅ Validé | send_angles, get_angles, go_home... |

### 🔧 Dernières modifications

1. **`slider_control.launch.py`** - Ajout du chemin config RViz
2. **`bridge_pi_simple.py`** - Ajout commandes texte (get_angles, power_on/off, gripper)

---

## �️ Architecture du Système

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
│                         │bridge_pi_simple │                                │
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

## 📦 Structure du Projet

```
mycobot_R6A/
├── SESSION_RESUME.md              # 👈 CE FICHIER - Point de départ
├── DEVELOPMENT_SUMMARY.md         # Résumé technique détaillé
├── INDEX.md                       # Index de documentation
├── README.md                      # README principal
├── bridge_pi_debug.py             # Script debug pour Pi
│
├── mycobot_gateway/               # 📦 Package ROS2 Principal
│   ├── package.xml
│   ├── setup.py
│   │
│   ├── mycobot_gateway/           # Modules Python
│   │   ├── __init__.py
│   │   ├── bridge_tour.py         # ⭐ Client TCP vers Pi
│   │   ├── robot_commander.py     # Interface CLI
│   │   ├── joint_sync.py          # Sync angles → RViz
│   │   ├── simple_gui.py          # GUI Tkinter
│   │   ├── slider_control.py      # Contrôle sliders
│   │   ├── teleop_keyboard.py     # Contrôle clavier
│   │   └── marker_follower.py     # Suivi ArUco
│   │
│   ├── scripts/
│   │   ├── bridge_pi_simple.py    # ⭐ Script Pi (serveur TCP)
│   │   └── bridge_pi_standalone.py
│   │
│   └── launch/
│       ├── simple_gui.launch.py
│       ├── slider_control.launch.py   # ⭐ Contrôle temps réel validé
│       ├── teleop_keyboard.launch.py
│       ├── rviz_sync.launch.py
│       └── marker_follow_full.launch.py
│
├── mycobot_description/           # 📦 Package URDF
│   ├── urdf/320_pi/               # Modèle 3D robot
│   ├── config/mycobot_320_pi.rviz # Config RViz
│   └── launch/display.launch.py
│
├── docs/                          # Documentation détaillée
└── scripts/                       # Scripts shell utilitaires
```

---

## 🚀 DÉMARRAGE RAPIDE

### ⚠️ PRÉREQUIS CRITIQUE - Éviter le conflit Conda

```bash
# TOUJOURS exécuter avant ROS2 (conflit Python 3.13 vs 3.12)
conda deactivate

# OU utiliser la commande "propre" :
env -i HOME=$HOME PATH="/usr/bin:/bin:/opt/ros/jazzy/bin" DISPLAY=$DISPLAY bash
```

### Étape 1 : Démarrer le Bridge sur le Pi

```bash
# SSH vers le Pi
ssh er@10.10.0.218

# Lancer le bridge
python3 bridge_pi_simple.py
```

### Étape 2 : Lancer le contrôle sur le PC Tour

```bash
# Option A : Slider Control (RECOMMANDÉ - testé et validé)
env -i HOME=$HOME PATH="/usr/bin:/bin:/opt/ros/jazzy/bin" DISPLAY=$DISPLAY bash -c '
source /opt/ros/jazzy/setup.bash && 
source ~/ros_jazzy/src/mycobot_R6A/install/setup.bash && 
ros2 launch mycobot_gateway slider_control.launch.py'

# Option B : GUI Simple
ros2 launch mycobot_gateway simple_gui.launch.py

# Option C : Contrôle clavier
ros2 launch mycobot_gateway teleop_keyboard.launch.py
```

---

## � Protocole de Communication

### Format JSON (Recommandé)

```json
// Mouvements
{"action": "send_angles", "angles": [0, 0, 0, 0, 0, 0], "speed": 30}
{"action": "send_coords", "coords": [200, 0, 200, 180, 0, 0], "speed": 40, "mode": 1}
{"action": "go_home"}
{"action": "go_zero"}

// Lecture état
{"action": "get_angles"}    // → "ANGLES: [0.43, 0.0, 0.0, 0.35, 0.26, 0.35]"
{"action": "get_coords"}    // → "COORDS: [...]"

// Gripper
{"action": "gripper_open"}
{"action": "gripper_close"}

// Contrôle moteurs
{"action": "power_on"}
{"action": "power_off"}
{"action": "emergency_stop"}
```

### Commandes Texte (Alternative)

| Commande | Description |
|----------|-------------|
| `ping` | Test connexion → `PONG` |
| `status` | État robot |
| `home` / `go_home` | Position home |
| `zero` / `go_zero` | Position zéro |
| `get_angles` / `angles` | Lire angles |
| `get_coords` / `coords` | Lire coordonnées |
| `power_on` | Allumer moteurs |
| `power_off` | Relâcher servos |
| `gripper_open` | Ouvrir pince |
| `gripper_close` | Fermer pince |
| `stop` | Arrêt d'urgence |

---

## 🔌 Configuration Réseau

| Machine | IP | Port | Rôle |
|---------|-----|------|------|
| PC Tour | 10.10.0.115 | - | Calcul, GUI, RViz |
| Raspberry Pi | 10.10.0.218 | 5005 | Bridge robot |

---

## ✅ Tests Validés (Session 26/03/2026)

| Test | Résultat | Notes |
|------|----------|-------|
| `ping` | ✅ PONG | Connexion TCP OK |
| `get_angles` | ✅ `[0.43, 0.0, 0.0, 0.35, 0.26, 0.35]` | Lecture réelle |
| `go_home` | ✅ Robot moved | Position home |
| JSON `send_angles` | ✅ Temps réel | Via slider_control |
| RViz visualisation | ✅ Robot visible | Fixed Frame = base |
| simple_gui | ✅ Fonctionnel | Interface Tkinter |
| slider_control | ✅ Validé complet | Robot suit sliders |

---

## 🔑 Points Importants à Retenir

### 1. Conflit Python Conda
```bash
# TOUJOURS désactiver Conda avant ROS2
conda deactivate
# OU utiliser env -i pour environnement propre
```

### 2. Position HOME vs ZERO
- **HOME** : `[0, 8, -127, 40, 0, 0]` (position sécurisée)
- **ZERO** : `[0, 0, 0, 0, 0, 0]` (tous joints à zéro)

### 3. Ordre de démarrage
1. Pi : `python3 bridge_pi_simple.py`
2. Tour : `ros2 launch mycobot_gateway <launch_file>`

### 4. Compilation
```bash
cd ~/ros_jazzy/src/mycobot_R6A
colcon build --packages-select mycobot_gateway --symlink-install
source install/setup.bash
```

---

## 🚧 TODO - Prochaines Étapes

### Priorité Haute
- [ ] Tester `teleop_keyboard` (contrôle clavier)
- [ ] Tester `marker_follower` (suivi ArUco)
- [ ] Améliorer gestion erreurs bridge

### Priorité Moyenne
- [ ] Streaming caméra Pi → Tour
- [ ] Interface web (option future)
- [ ] Enregistrement/rejeu trajectoires

### Priorité Basse
- [ ] Intégration Gazebo simulation
- [ ] Path planning MoveIt2
- [ ] Multi-robot coordination

---

## 📚 Documentation Complémentaire

| Fichier | Description |
|---------|-------------|
| `DEVELOPMENT_SUMMARY.md` | Résumé technique détaillé |
| `INDEX.md` | Index de toute la documentation |
| `docs/QUICKSTART.md` | Guide démarrage rapide |
| `docs/ARCHITECTURE.md` | Architecture système |
| `docs/DEPLOYMENT.md` | Guide de déploiement |
| `mycobot_gateway/README.md` | Documentation du package |

---

## 🔗 Liens Utiles

- **GitHub** : https://github.com/ABMI-software/mycobot_320pi_R6A
- **ROS2 Jazzy** : https://docs.ros.org/en/jazzy/
- **pymycobot** : https://github.com/elephantrobotics/pymycobot

---

*Ce fichier est le point de départ pour les prochaines sessions de développement.*  
*Dernière mise à jour : 26 mars 2026*
