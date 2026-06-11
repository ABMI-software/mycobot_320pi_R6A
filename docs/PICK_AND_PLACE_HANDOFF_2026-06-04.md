# Handoff Pick-and-Place (Reel, cam_0) - 4 juin 2026

## Objectif de cette session
- Stabiliser la vision ArUco en reel.
- Basculer de la depth camera vers Arducam cam_0.
- Obtenir un live utilisable avec IDs visibles.
- Preparer un test intermediaire reach-only (sans pince) avant cycle pick-and-place complet.

## Resume executif
- cam_0 detecte correctement plusieurs marqueurs (5+ selon scenes), contrairement a la depth camera.
- Le localizer publie des IDs stables (ex: 25, 26, 20, 19, 23).
- Un warning de hauteur persiste si `camera_height_m` ne correspond pas au montage reel.
- Avec certains mappings workspace forces, solvePnP peut tomber sur une solution miroir (hauteur estimee negative).
- Un nœud de test one-shot reach-only a ete ajoute pour valider vision -> IK -> commande robot, sans action de pince.

## Modifications code implementees aujourd'hui

### 1) Visualisation live
Fichiers:
- `mycobot_gateway/mycobot_gateway/camera_live_view.py`
- `mycobot_gateway/mycobot_gateway/camera_web_view.py`

Principales evolutions:
- Mode single/split configure par variable d'environnement.
- Overlay ArUco + IDs dans le live.
- Mode web dual/split pour debug terrain.
- Correctif de publication MJPEG (variable globale non mise a jour).
- Durcissement anti-segfault sur chemin de rendu dual (usage debug image privilegie).

### 2) Localizer ArUco
Fichier:
- `mycobot_gateway/mycobot_gateway/aruco_localizer_node.py`

Principales evolutions:
- Parametrage dictionnaire ArUco et tuning detecteur.
- `auto_workspace_ids` / `ws_marker_ids` pour mapping workspace.
- `auto_object_from_center` pour fallback objet.
- Penalisation forte des solutions non physiques (camera_height <= 0) pour eviter les poses miroir.

### 3) Test intermediaire reach-only (sans pince)
Fichiers:
- `mycobot_gateway/mycobot_gateway/reach_target_aruco_node.py` (nouveau)
- `mycobot_gateway/setup.py` (entrypoint ajoute)

But:
- Recevoir `/aruco/object_pose`.
- Calculer IK.
- Envoyer une sequence simple `approach -> target`.
- Terminer en DONE sans prise/depôt.

## Etat valide en fin de session
- Detection IDs: OK sur cam_0 (IDs multiples observes en continu).
- Live web cote-a-cote: operationnel en mode stable.
- Pipeline de test reach-only: code ajoute, mais execution terrain incomplete selon annulations/interruptions terminal.

## Point critique pour la prochaine session
- La camera est inclinee (environ 46 deg) par rapport au plan des marqueurs: c'est acceptable pour ArUco/PnP.
- Le parametre qui doit etre exact est `camera_height_m` (hauteur verticale base->camera), pas la distance selon l'axe optique.
- Ne pas forcer un mapping workspace incorrect, sinon solution miroir possible.

## Commandes de reprise recommandees (ordre)

### A) Build
```bash
cd /home/genji/ros_jazzy/src/mycobot_R6A
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mycobot_gateway --symlink-install
source install/setup.bash
```

### B) Camera cam_0 vers ROS (si necessaire)
```bash
ros2 run image_tools cam2image --ros-args -p device_id:=1 -p frequency:=20.0 -p width:=640 -p height:=480 -r image:=/camera/image_raw
```

### C) Localizer en mode robuste
```bash
ros2 run mycobot_gateway aruco_localizer --ros-args \
  -p camera_topic:=/camera/image_raw \
  -p calib_file:=/home/genji/ros_jazzy/src/mycobot_R6A/training/calibration/cam_0.npz \
  -p camera_height_m:=0.35 \
  -p obj_marker_id:=20 \
  -p aruco_dict_name:=DICT_4X4_1000 \
  -p aruco_try_multiple_dicts:=false \
  -p aruco_detect_scale:=1.8 \
  -p auto_workspace_ids:=true \
  -p auto_object_from_center:=false
```

### D) Reach-only sans pince (validation intermediaire)
```bash
ros2 run mycobot_gateway reach_target_aruco --ros-args \
  -p approach_height:=0.10 \
  -p target_z_offset:=-0.03 \
  -p move_time:=3.0 \
  -p require_workspace_valid:=true
```

### E) Diagnostics rapides
```bash
ros2 topic echo /aruco/detected_ids
ros2 topic echo /aruco/workspace_valid
ros2 topic echo /aruco/object_pose
ros2 topic echo /reach_target/status
```

## Probleme recurrent connu
- L'Arducam peut changer de device (`/dev/video0` -> `/dev/video1`) apres mouvement/deconnexion USB.
- Symptomes: flux fige, `Device '/dev/videoX' is busy` ou `Could not open video stream`.
- Action: verifier le bon device et relancer `cam2image` avec le bon `device_id`.
