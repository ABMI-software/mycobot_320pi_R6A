# Handoff Pick-and-Place (Gazebo + Reel) — 3 juin 2026

## Objectif
Document de reprise pour la prochaine session Claude: etat exact des implementations pick-and-place en simulation et en environnement reel, et procedure de demarrage des tests.

## Resume executif
- Gazebo: cycle pick-and-place ArUco valide de bout en bout (FSM complete, trajectoires, grasp/release simules).
- Reel: pipeline lance, bridge robot OK, camera depth sur la tour integree, mais publication objet bloquee tant que les IDs ArUco ne matchent pas le schema initial.
- Correctif majeur du jour: localizer ArUco rendu compatible IDs non standards (auto-mapping workspace + fallback objet au centre).
- Contexte infra reel: camera fixe a 57 cm, objet ~6 cm, topic image /camera/color/image_raw, Pi bridge sur 10.10.0.221:5005.

## Ce qui a ete implemente aujourd'hui

### 1) Orchestrateur pick-and-place (sim + reel)
Fichier: mycobot_gateway/mycobot_gateway/pick_and_place_aruco_node.py
- Ajout d'une logique de hauteur de prise basee sur la geometrie objet.
- Parametres ajoutes/utilises:
  - object_diameter (reel: 0.06)
  - grasp_z_offset
  - min_pick_z
- Calcul pick_z: top marker objet -> correction par diametre -> clamp min_pick_z.

### 2) Localizer ArUco reel
Fichier: mycobot_gateway/mycobot_gateway/aruco_localizer_node.py
- Robustesse calibration:
  - fallback si calib_file vide
  - resolution plus robuste du chemin de calibration
- Compatibilite OpenCV:
  - support ArucoDetector (nouveau)
  - fallback detectMarkers legacy
- Observabilite:
  - topic debug IDs detectes: /aruco/detected_ids (Int32MultiArray)
  - warning de coherence de hauteur camera (camera_height_m)
- Adaptation IDs non standards (nouveau patch du jour):
  - params:
    - ws_marker_ids (mapping explicite des 4 IDs workspace)
    - auto_workspace_ids (auto mapping des 4 marqueurs workspace)
    - auto_object_from_center (si ID objet absent, prendre le marqueur le plus central)
  - en mode auto, test des combinaisons/permutations de 4 marqueurs et choix du meilleur score:
    - erreur de reprojection
    - penalite de coherence avec camera_height_m

### 3) Launch reel
Fichier: mycobot_gateway/launch/pick_and_place_aruco_real.launch.py
- Parametres alignes setup reel:
  - camera_topic: /camera/color/image_raw
  - camera_height_m: 0.57
  - object_diameter: 0.06
- IP Pi par defaut mise a jour: 10.10.0.221

### 4) Camera Orbbec vers ROS
Fichiers:
- mycobot_gateway/mycobot_gateway/orbbec_camera_publisher.py (nouveau)
- mycobot_gateway/setup.py
- mycobot_gateway/setup.cfg (nouveau)
- Publisher RGB Orbbec -> topic ROS image, integre au package.
- setup.cfg ajoute pour corriger l'installation des executables dans libexec ROS (fix "executable not found").

### 5) Bridge reseau Tour -> Pi
Fichier: mycobot_gateway/mycobot_gateway/bridge_tour.py
- IP par defaut synchronisee avec l'infra actuelle: 10.10.0.221.

## Etat de validation en fin de session
- Build package cible OK:
  - colcon build --paths mycobot_gateway --symlink-install
- Flux global reel:
  - bridge TCP vers Pi OK
  - localizer demarre OK
  - FSM passe INIT -> WAIT_POSE
- Blocage principal observe avant patch auto-IDs:
  - workspace_valid=false et /aruco/object_pose absent
  - IDs reels detectes differents des IDs historiques (0,1,2,3,10)

## Procedure de reprise (prochaine session Claude)

### A. Pre-requis environnement (important)
Toujours utiliser un shell ROS propre (evite pollution conda):

```bash
env -i HOME="$HOME" USER="$USER" SHELL=/bin/bash TERM="$TERM" PATH=/usr/bin:/bin:/usr/sbin:/sbin bash -lc '
  cd /home/genji/ros_jazzy/src/mycobot_R6A &&
  source /opt/ros/jazzy/setup.bash &&
  source install/setup.bash &&
  echo "ROS shell ready"'
```

### B. Test simulation Gazebo (reference)

```bash
cd /home/genji/ros_jazzy/src/mycobot_R6A
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ros2 launch mycobot_gateway pick_and_place_aruco.launch.py
```

Attendu:
- Cycle complet home -> pick -> place -> home_end -> DONE.

### C. Test reel (camera depth fixe + robot)

```bash
env -i HOME="$HOME" USER="$USER" SHELL=/bin/bash TERM="$TERM" PATH=/usr/bin:/bin:/usr/sbin:/sbin bash -lc '
  cd /home/genji/ros_jazzy/src/mycobot_R6A &&
  source /opt/ros/jazzy/setup.bash &&
  source install/setup.bash &&
  ros2 launch mycobot_gateway pick_and_place_aruco_real.launch.py'
```

Attendu:
- /aruco/detected_ids publie des IDs en continu
- /aruco/workspace_valid passe a true
- /aruco/object_pose est publie
- FSM sort de WAIT_POSE puis execute pick-and-place

### D. Checks rapides de diagnostic

```bash
# IDs vus par la camera
ros2 topic echo /aruco/detected_ids

# Validite workspace
ros2 topic echo /aruco/workspace_valid

# Pose objet
ros2 topic echo /aruco/object_pose
```

## Notes operatoires (important pour la prochaine session)
- Nouvelle contrainte utilisateur: session sur site physique (pas en SSH), donc acces interfaces graphiques disponible.
- Priorite reprise: valider visuellement dans RViz/Gazebo et camera debug que les 5 marqueurs sont bien detectes dans le meme frame.
- Si les 5 IDs exacts sont connus, possible de desactiver l'auto et fixer explicitement ws_marker_ids + obj_marker_id pour stabiliser.

## Plan de test recommande en mode onsite (GUI)
1. Lancer la stack reel.
2. Ouvrir RViz et afficher:
   - /aruco/object_pose
   - image debug ArUco
3. Verifier un cycle unique de pick-and-place.
4. Repeter 5 cycles et noter:
   - succes prise
   - succes depose
   - timeout WAIT_POSE
   - derive de pose observee

## Fichiers modifies aujourd'hui (zone pick-and-place)
- mycobot_gateway/mycobot_gateway/pick_and_place_aruco_node.py
- mycobot_gateway/mycobot_gateway/aruco_localizer_node.py
- mycobot_gateway/launch/pick_and_place_aruco_real.launch.py
- mycobot_gateway/mycobot_gateway/orbbec_camera_publisher.py
- mycobot_gateway/setup.py
- mycobot_gateway/setup.cfg
- mycobot_gateway/mycobot_gateway/bridge_tour.py
