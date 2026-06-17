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
