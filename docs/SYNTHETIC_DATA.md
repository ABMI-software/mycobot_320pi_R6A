# Synthetic Data Pipeline — MyCobot 320 Pi (50k images)

## Objectif

Générer 50 000 images labellisées **(image, angles_joints)** dans Gazebo pour
entraîner un modèle de pose estimation (DREAM ou ResNet-based) sur le MyCobot 320 Pi.

**Setup :** 4 caméras en anneau à 90°, 12 500 poses × 4 = 50 000 images.

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                      GAZEBO HARMONIC (headless)                 │
│                         lab_room.sdf                            │
│                                                                 │
│  ┌────────────┐    ┌──────────┐  ┌──────────┐                  │
│  │ MyCobot 320│    │ cam_0    │  │ cam_1    │  cam_2  cam_3    │
│  │  (URDF)    │───▶│ front    │  │ left     │  back   right   │
│  └─────┬──────┘    └────┬─────┘  └────┬─────┘                  │
│        │ /joint_states  │ images       │                        │
└────────┼────────────────┼─────────────┼────────────────────────┘
         │  ros_gz_bridge │ ros_gz_image │
         ▼                ▼             ▼
┌─────────────────────────────────────────────────────────────────┐
│                         ROS2 JAZZY                              │
│   synthetic_data_collector                                      │
│     1. Pose aléatoire → commande joints                         │
│     2. Attendre settle_time (1 s)                               │
│     3. Capturer 4 images + angles                               │
│     4. Sauvegarder sur disque                                   │
└──────────────────┬──────────────────────────────────────────────┘
                   ▼
         /tmp/synth_50k/
         ├── images/
         │   ├── cam_0/  000000.png … 012499.png
         │   ├── cam_1/
         │   ├── cam_2/
         │   └── cam_3/
         └── labels.csv
```

---

## Caméras

4 caméras en anneau symétrique à z = 0.4 m, intrinsèques calées sur les Arducam réelles (calibration ChArUco).

| Cam | Position (x, y, z) | Intrinsics source | fx |
|-----|--------------------|-------------------|----|
| cam_0 | (0.8, 0.0, 0.4) front | cam_0 Arducam | 525.671 |
| cam_1 | (0.0, 0.8, 0.4) left | cam_3 Arducam | 496.308 |
| cam_2 | (-0.7, 0.0, 0.4) back | cam_0 Arducam | 525.671 |
| cam_3 | (0.0, -0.8, 0.4) right | cam_3 Arducam | 496.308 |

Résolution : 640 × 480 @ 10 Hz.

> **Note :** Le bloc `<intrinsics>` SDF n'est pas appliqué par Gazebo Harmonic —
> seul `<horizontal_fov>` est utilisé pour le rendu. Les intrinsèques calibrées
> sont correctes dans les fichiers `*_camera_settings.json`.

---

## Monde Gazebo : `lab_room.sdf`

Fichier : `mycobot_gateway/worlds/lab_room.sdf`

Éclairage naturel 3 sources (style intérieur de labo) :

| Lumière | Type | Rôle |
|---------|------|------|
| `sun` | directionnelle | lumière chaude fenêtre/soleil (0.95 0.88 0.75) |
| `fill` | directionnelle | lumière froide plafond/sky (0.38 0.42 0.55) |
| `back_fill` | directionnelle | débouche les ombres (0.20 0.20 0.22) |

Ambient : `0.55 0.52 0.48` (chaud), background : `0.80 0.82 0.86` (gris-bleu).
Sol : béton clair chaud.

> **Contraintes SDF** : version 1.6, world name `"empty"` (requis pour le bridge
> ROS `/world/empty/model/mycobot_320/joint_state`), **pas** de plugin `<Sensors>`
> (chargé automatiquement par `--headless-rendering`, double-chargement → crash OGRE2).

---

## Lancer le pipeline

### Prérequis

```bash
conda deactivate
source /opt/ros/jazzy/setup.bash
source ~/ros_jazzy/install/setup.bash
```

### Preview (vérification visuelle — 5 poses)

```bash
rm -rf /tmp/synth_preview
ros2 launch mycobot_gateway synthetic_data_preview.launch.py
# Résultat : /tmp/synth_preview/preview_grid.png (grille 4 cams × 5 poses)
```

### Acquisition complète (50 000 images)

```bash
ros2 launch mycobot_gateway synthetic_data.launch.py \
    num_samples:=12500 output_dir:=/tmp/synth_50k
```

Durée estimée : ~4-5 h (software rendering Mesa/llvmpipe, sans GPU).

### Paramètres

| Paramètre | Défaut | Description |
|-----------|--------|-------------|
| `num_samples` | 12500 | Nombre de poses (images = poses × 4) |
| `output_dir` | `/tmp/mycobot_synth_dataset` | Dossier de sortie |
| `settle_time` | 1.0 | Secondes d'attente après commande joint |

---

## Format de sortie

### labels.csv

| Colonne | Description |
|---------|-------------|
| `index` | Numéro d'échantillon (0-based) |
| `j1_rad` … `j6_rad` | Angles joints en radians |
| `j1_deg` … `j6_deg` | Angles joints en degrés |
| `cam` | Identifiant caméra (cam_0 … cam_3) |
| `image_path` | Chemin relatif vers le PNG |

### Ordre des joints

| Index | Joint URDF | Description |
|-------|-----------|-------------|
| j1 | `joint2_to_joint1` | Rotation base |
| j2 | `joint3_to_joint2` | Épaule |
| j3 | `joint4_to_joint3` | Coude |
| j4 | `joint5_to_joint4` | Poignet pitch |
| j5 | `joint6_to_joint5` | Poignet roll |
| j6 | `joint6output_to_joint6` | Effecteur |

---

## Build

```bash
cd ~/ros_jazzy/src/mycobot_R6A
conda deactivate
source /opt/ros/jazzy/setup.bash
colcon build --packages-select mycobot_description mycobot_gateway --symlink-install
source install/setup.bash
```

---

## Notes techniques

- **Headless Gazebo** : flags `-s --headless-rendering` → EGL off-screen via Mesa/llvmpipe.
  Pas d'affichage X11 requis. Rendu plus lent qu'avec GPU (~4-5 h vs ~3.5 h).
- **Intrinsics URDF** : block `<intrinsics>` ignoré par SDFormat, seul `hfov` est utilisé.
  Décalage point principal : 2.3 px (cam_0), 6.6 px (cam_3). Acceptable pour DREAM.
- **Commentaires XML URDF** : `--` interdit dans `<!-- -->` en XML strict.
  Les 4 commentaires de caméra ont été corrigés pour respecter cette règle.
