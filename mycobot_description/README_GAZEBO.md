# 🏗️ Gazebo Simulation — MyCobot 320 Pi

## Overview

This launches the MyCobot 320 Pi inside **Gazebo Harmonic** (the simulator
shipped with ROS2 Jazzy) using the `ros_gz_sim` bridge stack.

### What it does

| Component | Role |
|-----------|------|
| `gz sim` (Gazebo Harmonic) | Physics / 3D visualisation |
| `robot_state_publisher` | Publishes `/robot_description` and TF tree |
| `ros_gz_sim create` | Spawns the URDF into the running Gazebo world |
| `ros_gz_bridge` | Forwards `/joint_states` from Gazebo → ROS2 |
| `rviz2` *(optional)* | RViz alongside Gazebo |

### Files

| File | Description |
|------|-------------|
| `launch/gazebo_sim.launch.py` | Main launch file |
| `urdf/320_pi/mycobot_pro_320_pi_gazebo.urdf` | Gazebo-compatible URDF (with inertials, dynamics, world link, Gz plugins) |
| `urdf/320_pi/mycobot_pro_320_pi.urdf` | Original URDF (RViz-only, no inertials) |

---

## Prerequisites

```bash
# These packages must be installed (should already be present on Jazzy)
sudo apt install ros-jazzy-ros-gz-sim ros-jazzy-ros-gz-bridge
```

## Worlds disponibles

| Fichier SDF | Description |
|-------------|-------------|
| `worlds/randomized.sdf` | Table + fond simple (monde de base) |
| `worlds/randomized_v2.sdf` | 6 lumières, 12 objets clutter (cubes/cylindres/sphères), 3 murs — domain randomization avancée |
| `worlds/pick_and_place.sdf` | Table 0.8×0.8 m + cube cible rouge + zone de dépose verte (mono-objet) |
| `worlds/pick_and_place_sorting.sdf` | Table 1.0×0.6 m + 4 objets dynamiques (cube R, cube B, cylindre G, boîte Y) côté +X + 4 bacs colorés à parois côté −X (multi-objet par couleur) |
| `worlds/real_table.sdf` | Plateau mesuré 622×449×8,5 mm + ArUco 19/23/25/26 de 50 mm + caméra de dessus + cube et bac |
| `worlds/banc_realiste_yolo26.sdf` | **Banc réaliste** — même plateau, marqueurs aux positions relevées AU ROBOT, les 4 pièces peintes et leurs 4 bacs, et les caméras **arducam et SVPRO à leur pose extrinsèque calibrée** (généré par `scripts/generer_banc_realiste.py`) |

Pour le plateau réel : `ros2 launch mycobot_gateway real_table.launch.py`
(cycle physique guidé par les quatre caméras, cube et bac aléatoires ; boutons
« Randomiser cube + bac » et « Lancer la prise » ; `demo:=false`
pour la scène seule, `seed:=7` pour reproduire une position). **La séquence complète
commence par `conda deactivate` puis un `colcon build`** — sans le build,
`models/` n'est pas installé et la scène se lance sans bois ni marqueurs. Voir
[le guide du plateau réel](../docs/GAZEBO_REAL_TABLE.md) pour la construction,
les coordonnées et les hypothèses de placement.

## Banc réaliste — les 4 pièces vues par yolo26

```bash
ros2 launch mycobot_gateway banc_realiste.launch.py   # terminal 1
/usr/bin/python3 scripts/yolo26_gazebo.py             # terminal 2
```

Les deux caméras sont posées à **leur pose extrinsèque mesurée**, avec leurs
intrinsèques : le même fichier de calibration vaut en simulation et au banc, donc
`Vision.vers_base` rend des millimètres robot sans qu'une ligne de la chaîne de
tri ne change. Mesuré, objets posés à des millimètres connus : **arducam 3,5 mm**
d'écart médian, SVPRO 9,0 mm — la hiérarchie du banc, sans réglage pour
l'obtenir.

Masses de la fiche 320 Pi 2022 (bras 3 kg, pince 0,340 kg) et butées mesurées
(J1 168, J2 135, J3 150, J4 145, J5 165, J6 180°).

Trois pièges, tous silencieux :

- **`real_table.sdf` n'a pas de système de capteurs** — sa `table_camera` déclare
  un sujet qui ne publiera jamais. Le monde réaliste l'ajoute.
- **Le rendu des capteurs segfaute sur NVIDIA** si libEGL choisit Mesa. Le
  lancement pose `__EGL_VENDOR_LIBRARY_FILENAMES`.
- **Le nom du `<world>` doit valoir celui du fichier**, sinon `ros_gz_sim create`
  attend un service qui n'existe pas et le robot n'apparaît jamais.

Après toute recalibration d'une caméra réelle, régénérer, sinon le jumeau reste
faux sans le signaler :

```bash
/usr/bin/python3 scripts/generer_banc_realiste.py
colcon build --packages-select mycobot_description --symlink-install
```

### Comment le banc a été rendu réaliste — les étapes, dans l'ordre

Chacune a été **mesurée**, pas estimée. Elles sont données dans l'ordre parce que
chacune dépend de la précédente.

**1. Les objets, générés depuis le dossier de fabrication.**
`scripts/generer_pieces_gazebo.py` lit `tri_couleur.py` : cotes
(`OBJET_PAR_COULEUR`, `BAC`) et teintes. Rien n'est recopié — une pièce
redimensionnée là-bas se propage par régénération. Origine au **centre de la
base** : une pose à Z = 0 repose sur la planche, sans calcul de demi-hauteur.

**2. Les marqueurs remis à la référence mesurée AU ROBOT.**
Le monde portait encore la forme au ruban, fausse de 10 à 16 mm.
`planche_actuelle.yaml` est la seule référence qui ne passe pas par une caméra.

**3. Les caméras à leur pose extrinsèque calibrée.**
C'est l'étape qui fait tout le reste. Gazebo oriente sa caméra +X vers l'avant,
OpenCV +Z : la conversion passe par `[z_cv, -x_cv, -y_cv]`. Les intrinsèques vont
dans `<lens><intrinsics>`, et le `horizontal_fov` en découle
(`2·atan(largeur / 2·fx)`). Vérification : les positions obtenues — arducam
(7 ; −128 ; 1070) mm, SVPRO (371 ; 458 ; 781) mm — retombent sur celles
enregistrées indépendamment dans les fichiers de calibration.

**4. Des extrinsèques simulées à distorsion NULLE.**
Gazebo rend une projection pinhole pure ; son `<distortion>` est un plumb-bob à
cinq paramètres, incapable du modèle rationnel du vrai objectif (k1 = 5,4,
k3 = −48,2). D'où `arducam_extrinsic_sim.yaml` et `svpro_extrinsic_sim.yaml` :
même `T_cam_world`, mêmes fx/fy/cx/cy, coefficients nuls. **Mélanger image
simulée et coefficients réels décale chaque point de plusieurs millimètres sans
rien signaler.**

**5. Les couleurs prises sur la peinture, pas sur le tracé.**
`tri_couleur.BGR` n'est qu'une couleur d'aperçu ; les teintes **mesurées** sont
`TEINTES` (rouge 0, jaune 21, vert 39, bleu 98). Le premier jet utilisait le
tracé — le vert était à H = 60 au lieu de 39.

**6. La valeur du matériau baissée pour ne pas écrêter.**
À V = 235, le rendu saturait et les teintes **s'effondraient l'une sur l'autre** :
jaune et vert sortaient tous deux à H = 30 au lieu de 21 et 39. À V = 165 les
quatre teintes rendues valent exactement les mesurées.

**7. L'éclairage.** Laissé **normal**, identique à `real_table.sdf`. Le baisser
d'un tiers (`FACTEUR_LUMIERE` dans `generer_banc_realiste.py`) ramène l'image
caméra à 76,6 de luminance — celle de l'arducam réelle à l'exposition 75 — mais
assombrit la scène qu'on regarde. À plein éclairage, 14 % des pixels saturent et
la détection tient quand même.

### Puis yolo26 par-dessus

Aucune adaptation du modèle : `scripts/yolo26_gazebo.py` lit les deux sujets
image, soumet les trames au **même** `ServiceYOLO26` que le banc réel, et projette
les boîtes avec `Vision.vers_base` sur les extrinsèques simulées. Le résultat est
en millimètres robot, directement comparable à la vérité du monde.

Contrôle en une commande, avec les deux vues annotées :

```bash
/usr/bin/python3 scripts/yolo26_gazebo.py            # --sans-fenetre, --une-passe
```

**Reste ouvert :** `bac_jaune` n'est détecté par aucune des deux vues, alors qu'il
l'est en réel. Ni la teinte (écart 0 après l'étape 6) ni l'exposition (75,5 contre
76,6) ne l'expliquent. Le jaune partage la teinte du bois — 19 contre 21, dans la
bande de ±8 — et le bac simulé, mat et à parois de 2 mm, n'a pas le reflet de
rebord du vrai.

## Tri des 4 objets par saisie physique

```bash
# Terminal 1 — le banc
ros2 launch mycobot_gateway sim_grasp.launch.py

# Terminal 2 — le tri, pince réelle
ros2 run mycobot_gateway sim_sorting_grasp --ros-args -p use_sim_time:=true
```

C'est **la** simulation de tri de référence : la pince se ferme, le contact est
simulé par `gz_ros2_control`, et chaque prise est vérifiée sur la pose Gazebo de
l'objet. `sorting_orchestrator` fait le même parcours mais **téléporte** l'objet
via `set_pose` — d'où l'objet qui saute. Il est antérieur à la pince modélisée
et conservé pour sa partie perception.

## Visuels caméra (URDF Gazebo)

Les 4 caméras embarquées dans le URDF (`mycobot_pro_320_pi_gazebo.urdf` —
`camera_link`, `camera_link_right`, `camera_link_left`, `camera_link_top`)
sont représentées par un corps gris foncé (boîte 0.06×0.04×0.04 m) +
un objectif noir cylindrique aligné sur l'axe optique (+X) + une LED rouge.
Cette forme « caméra de surveillance » les rend visuellement distinctes des
objets colorés à trier — important pour le pipeline `color_object_detector`
qui segmente la scène par couleur.

## Gripper adaptatif

Le gripper `pro_adaptive_gripper` d'Elephant Robotics est intégré au URDF Gazebo.
Ses joints sont **fixés** (pas de support `mimic` dans Gazebo Harmonic). Le mesh
`link6_2022.dae` est utilisé pour la compatibilité avec les maillages du gripper.

```
urdf/pro_adaptive_gripper/
├── gripper_base.dae
├── left_1.dae, left_2.dae, left_3.dae
└── right_1.dae, right_2.dae, right_3.dae
```

## Quick Start

```bash
# 1. Clean environment (avoid Conda conflicts)
env -i HOME=$HOME PATH="/usr/bin:/bin:/opt/ros/jazzy/bin" DISPLAY=$DISPLAY bash

# 2. Source ROS2 + workspace
source /opt/ros/jazzy/setup.bash
source install/setup.bash

# 3. Launch Gazebo
ros2 launch mycobot_description gazebo_sim.launch.py

# 3b. Launch Gazebo + RViz side-by-side
ros2 launch mycobot_description gazebo_sim.launch.py rviz:=true
```

## Troubleshooting

| Symptom | Fix |
|---------|-----|
| `[gz] [Err] Unable to find file …` | Rebuild: `colcon build --packages-select mycobot_description --symlink-install` |
| Gazebo opens but robot is invisible | Check `GZ_SIM_RESOURCE_PATH` includes the install share path |
| Robot falls through the ground | Make sure `mycobot_pro_320_pi_gazebo.urdf` is used (has `world` fixed joint) |
| Meshes are white / untextured | `.dae` files may not include materials — cosmetic only |
