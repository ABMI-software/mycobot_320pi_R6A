# mycobot_description

Le modèle du robot et de sa cellule : URDF, meshes, mondes Gazebo et
configuration des contrôleurs. Paquet `ament_cmake`, sans nœud — il ne fournit
que des données, installées dans `share/`.

## Contenu

| Chemin | Rôle |
|---|---|
| `urdf/320_pi/` | URDF du MyCobot 320 Pi, pince Pro adaptative comprise, avec les blocs `gz_ros2_control` |
| `meshes/` | Meshes DAE du bras et de la pince |
| `worlds/` | Scènes Gazebo (voir ci-dessous) |
| `models/` | Modèles SDF autonomes — plateau bois, marqueurs ArUco |
| `config/controller.yaml` | `mycobot_controller` (JTC, 6 axes) + `gripper_position_controller` (6 joints) |
| `launch/`, `rviz/` | Visualisation |

## Les mondes

| Monde | Usage |
|---|---|
| `real_table.sdf` | Réplique du banc physique : plateau **622 × 449 × 8,5 mm** mesuré, texture bois, quatre ArUco de 50 mm aux positions relevées |
| `pick_and_place_sorting.sdf` | Banc de tri, quatre objets et quatre bacs |
| `pick_and_place.sdf` | Pick-and-place mono-objet |
| `precision_benchmark.sdf` | Grille de cibles du benchmark de précision |
| `randomized.sdf`, `randomized_v2.sdf` | Génération de données annotées, scène randomisée |

## Construire et lancer

```bash
conda deactivate
cd <your_ws>
colcon build --packages-select mycobot_description --symlink-install
source install/setup.bash
ros2 launch mycobot_gateway real_table.launch.py
```

⚠ **Le `colcon build` n'est pas optionnel.** Sans lui, `models/` n'est pas
installé dans `share/`, les `package://mycobot_description/models/...` ne se
résolvent pas, et **la scène se lance sans bois ni marqueurs — silencieusement,
sans le moindre message d'erreur.** Si le plateau apparaît gris et nu, c'est
ça.

## Pour aller plus loin

- [`README_GAZEBO.md`](README_GAZEBO.md) — caméras du URDF, conventions de
  nommage droite/gauche, apparence réaliste, banc de tri
- [`../docs/GAZEBO_REAL_TABLE.md`](../docs/GAZEBO_REAL_TABLE.md) — construction
  du plateau, coordonnées, hypothèses de placement
- [`models/wood_table/README.md`](models/wood_table/README.md) — provenance de
  la texture

Toute modification d'un URDF, d'un monde ou d'un mesh se répercute dans
[`README_GAZEBO.md`](README_GAZEBO.md) **dans le même commit**.
