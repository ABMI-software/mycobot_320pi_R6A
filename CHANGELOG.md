# Changelog

Toutes les modifications notables de ce projet seront documentées dans ce fichier.

Le format est basé sur [Keep a Changelog](https://keepachangelog.com/fr/1.0.0/),
et ce projet adhère au [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.4.0] - 2026-06-11

### Ajouté
- **Synthetic data pipeline 50k** : 4 caméras en anneau 90°, 12 500 poses × 4 = 50 000 images
- **`lab_room.sdf`** : monde Gazebo avec éclairage naturel 3 sources (soleil chaud + fill froid + back fill)
- **`synthetic_data_preview.launch.py`** : preview 5 poses + contact sheet PNG automatique
- **Headless rendering** : flags `-s --headless-rendering` pour acquisition sans X11/GPU

### Modifié
- `synthetic_data.launch.py` : migration vers `lab_room.sdf` + mode headless
- `mycobot_pro_320_pi_gazebo.urdf` : 4 caméras ring avec intrinsèques Arducam calées
- `setup.py` : installation automatique `worlds/*.sdf`

### Corrigé
- Commentaires XML URDF : `--` interdit dans `<!-- -->` (parse error xacro)
- SDF couleurs > 1.0 invalides (SDFormat Error Code 34)
- Double-chargement plugin Sensors avec `--headless-rendering` → crash OGRE2 `ItemIdentityException`
- Nom du monde SDF forcé à `"empty"` pour compatibilité bridge ROS topic

---

## [0.3.0] - 2026-06-04

### Ajouté
- **Pipeline pick-and-place ArUco** (simulation + réel) : FSM complète, trajectoires, grasp/release
- **`aruco_localizer_node`** : auto-mapping workspace IDs, fallback objet au centre, robustesse OpenCV
- **`pick_and_place_aruco_node`** : orchestrateur sim + réel, calcul pick_z par géométrie objet
- **`reach_target_aruco_node`** : test intermédiaire reach-only sans pince
- **`camera_live_view.py` / `camera_web_view.py`** : visualisation live ArUco avec overlay IDs
- **`orbbec_camera_publisher.py`** : publisher RGB Orbbec → topic ROS image
- **`pick_and_place_aruco_real.launch.py`** : launch réel (cam_0 Arducam, Pi sur 10.10.0.221)

---

## [0.2.0] - 2026-04-01

### Ajouté
- **Calibration caméras Arducam** : ChArUco board, 4 caméras calibrées (fx cam_0=525.671, fx cam_3=496.308)
- **Intégration Gazebo Harmonic** : spawn URDF MyCobot 320 Pi avec `ros_gz_sim`
- **`synthetic_data_collector`** : collecteur initial pose aléatoire + capture image + labels.csv
- **`synthetic_data.launch.py`** : pipeline collecte 1000 échantillons

---

## [0.1.0] - 2026-03-26

### Ajouté
- **Architecture Tour/Pi** : Séparation calcul (PC) et contrôle robot (Pi)
- **Bridge TCP** : Communication bidirectionnelle Tour ↔ Pi
- **Modes de contrôle** :
  - `simple_gui` : Interface graphique Tkinter
  - `slider_control` : Contrôle via sliders RViz
  - `teleop_keyboard` : Contrôle clavier WASD
  - `commander` : Interface CLI interactive
  - `rviz_sync` : Synchronisation robot réel → RViz
- **Format JSON** : Protocol de commande standardisé
- **Documentation** : README, guides de déploiement, SESSION_RESUME

### Configuration
- PC Tour : ROS2 Jazzy, Ubuntu 24.04, Python 3.12
- Raspberry Pi : ROS2 Galactic, Ubuntu 20.04, Python 3.8
- Robot : MyCobot 320 Pi, port `/dev/ttyAMA0`, baudrate 115200

### Tests validés
- Communication TCP ping/pong
- Contrôle LED (RGB)
- Lecture/écriture angles joints
- Mouvements go_home, go_zero
- Gripper open/close
- Synchronisation RViz temps réel
