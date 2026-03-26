# Changelog

Toutes les modifications notables de ce projet seront documentées dans ce fichier.

Le format est basé sur [Keep a Changelog](https://keepachangelog.com/fr/1.0.0/),
et ce projet adhère au [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

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
