# Validation en simulation — téléopération + use cases (sans le bras réel)

Ce document décrit comment **valider la téléopération et les pipelines applicatifs (pick-and-place, sorting) en Gazebo seul**, avant tout passage sur le MyCobot 320 Pi physique. Il est complémentaire à [`REAL_ROBOT_TEST_PROCEDURE.md`](REAL_ROBOT_TEST_PROCEDURE.md), qui couvre la session physique.

> **Pourquoi ce document existe** : la procédure historique allume Gazebo + le bridge TCP vers le Pi en parallèle (`target:=both`). Elle ne couvre pas le cas « *je veux d'abord prouver que ma config tient en simulation, sans risquer le bras* ». Ce fichier comble ce trou.

---

## Quand utiliser ce protocole

- ✅ Avant **toute première session sur robot physique** (calibration des gains, validation visuelle des trajectoires, repérage des saturations).
- ✅ Après **tout changement** de gains, mapping, filtres, URDF ou plan de pick-and-place.
- ✅ Pour **démontrer un use case** (sorting 4 couleurs, pick mono-objet, range of motion) sans dépendance matérielle.
- ❌ **Ne remplace pas** la session physique. Les frottements, les jeux mécaniques et la latence pymycobot ne sont pas modélisés en Gazebo Harmonic / DART.

---

## Pré-requis

| Item | Vérification |
|------|--------------|
| Workspace ROS2 buildé | `cd ~/ros_jazzy && colcon build --packages-select mycobot_gateway mycobot_description --symlink-install` |
| Env conda `hand-teleop` (téléop main uniquement) | `conda activate hand-teleop && python -c "import roslibpy"` |
| Caméra Orbbec Astra S branchée (téléop main uniquement) | `lsusb \| grep -i orbbec` |
| Port 9090 libre (rosbridge) | `ss -tln \| grep 9090` doit être vide |
| **Robot physique éteint ou déconnecté** | optionnel, mais recommandé pour éviter les confusions |

> 💡 **Astuce isolation** : si une session réelle tourne déjà ailleurs sur le réseau, lancer la sim avec `ROS_DOMAIN_ID=42` pour ne pas voir / commander le vrai robot par accident.

---

## Use case A — Téléopération main → bras simulé

### Stack à lancer (3 terminaux)

```bash
# T1 — rosbridge (un seul, partagé entre toutes les sessions)
conda deactivate
source /opt/ros/jazzy/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml

# T2 — Gazebo + JTC (target=sim, AUCUNE connexion à la Pi)
conda deactivate
source /opt/ros/jazzy/setup.bash && source ~/ros_jazzy/install/setup.bash
ros2 launch mycobot_gateway mycobot_teleop.launch.py target:=sim

# T3 — Téléop main (Wilor + Astra) en env conda
conda activate hand-teleop && cd teleop
python3 mycobot_teleop.py --camera astra --ros --use-rosbridge --no-gripper
```

`target:=sim` empêche le launch de spawner `trajectory_to_robot_bridge` ou `bridge_tour`. Aucune commande ne part vers la Pi.

### Dashboard (T4 optionnel)

```bash
conda activate hand-teleop && cd teleop
python3 teleop_dashboard.py
```

Le badge en haut à droite doit afficher **🟦 SIM** (pas REAL ni BOTH).

### Critères d'acceptation avant de passer au robot réel

| KPI | Cible | Où le voir |
|-----|-------|------------|
| `Command rate` (/teleop topic) | 25–35 Hz stable | KPI card Home |
| `Signal health` | ≥ 90 % | KPI card Home |
| `SIM avg RMS` | ≤ 2.0 ° (sur 60 s d'usage normal) | KPI card Home |
| `Tracking error` par joint (J1–J6) | ≤ 5 ° médian | onglet 📊 Analytics |
| **Aucune saturation** sur J3 (elbow) | pas de plateau au max ou min des limites | plot Analytics |
| **Aucune oscillation** > 1° à main immobile | dérive < 0.3 ° / s sur 10 s | plot tracking error |
| Réactivité visuelle | délai main → EE < 300 ms perçu | observation Gazebo |

Si **un seul** de ces critères tombe → ne pas passer au réel. Re-tuner via les sliders du dashboard (presets : 🐢 Safe → ⚙️ Nominal → ⚡ Reactive).

### Protocole guidé (recommandé, ~2 min)

```bash
# T5 — performance_analyzer en mode guidé
conda activate hand-teleop && cd teleop
python3 performance_analyzer.py --guided
```

Le script affiche 7 phases (idle → up/down → left/right → forward/back → combined → gripper → rest), 8–10 s chacune. À la fin : un rapport Excel `teleop_report_<TIMESTAMP>.xlsx` avec un verdict global (READY / CAUTIOUS / NOT READY) et un onglet par scénario. **Si verdict ≠ READY, ne pas passer au réel.**

Les rapports validés s'accumulent dans `teleop/teleop_report_*.xlsx` (dans `.gitignore`).

---

## Use case B — Pick-and-place mono-objet (cube rouge → zone verte)

### Lancement

```bash
conda deactivate
source /opt/ros/jazzy/setup.bash && source ~/ros_jazzy/install/setup.bash
ros2 launch mycobot_gateway pick_and_place.launch.py
```

Tout est lancé : Gazebo + spawn robot + 4 caméras + DREAM inference (T+8 s) + state machine (T+12 s). Cycle complet : `Home → Vision check → Approach → Grasp → Lift → Place → Release → Retreat → Home → Done`.

### Variantes utiles en sim

| Cas | Commande |
|-----|----------|
| Sans vision (open-loop IK pur) | `… use_vision:=false` |
| Cube ailleurs sur la table | `… target_x:=0.20 target_y:=0.0 target_z:=0.04` |
| Zone de dépose ailleurs | `… place_x:=-0.18 place_y:=0.18` |

### Critères d'acceptation

| Vérif | Attendu |
|-------|---------|
| `IK failed` dans les logs | aucun |
| Status final dans `/pickplace/status` | `DONE\|COMPLETE` |
| Cycle total | ~30–35 s |
| Cube visuellement dans la zone verte à la fin | oui |

---

## Use case C — Sorting 4 couleurs

### Lancement

```bash
ros2 launch mycobot_gateway pick_and_place_sorting.launch.py
```

Stack : Gazebo + spawn + bridges + 4 caméras + `color_object_detector` (T+6 s) + `sorting_orchestrator` (T+10 s). 4 cycles complets : red → blue → green → yellow.

### Variantes utiles

| Cas | Commande |
|-----|----------|
| Smoke-test sans perception (positions SDF connues) | `… use_detector:=false` |
| Trier seulement un sous-ensemble | `… process_order:=blue,green` |
| Inverser l'ordre | `… process_order:=yellow,green,blue,red` |

### Critères d'acceptation

| Vérif | Attendu | Comment |
|-------|---------|---------|
| Détecteur trouve les 4 couleurs | `data: OK\|n=4` sur `/sorting/detector_status` | `ros2 topic echo --once /sorting/detector_status` |
| Positions détectées proches du SDF (~1 mm) | red ≈ (0.22, −0.12), blue ≈ (0.22, +0.12), etc. | `ros2 topic echo --once /sorting/detections` |
| Status final | `Sorting complete: ['red', 'blue', 'green', 'yellow']` | logs `sorting_orchestrator` |
| Cycle total | ~90–100 s pour 4 objets | chrono manuel |
| Aucun `set_pose failed` | logs `sorting_orchestrator` propres | `grep set_pose /tmp/log` |
| Aucun `IK failed` | logs propres | idem |
| Visuellement : chaque objet finit dans le bac de sa couleur | oui | inspection Gazebo |

> **Si Y est inversé** (rouge dans le bac bleu, etc.) : flipper `flip_u` dans [`mycobot_gateway/launch/pick_and_place_sorting.launch.py`](../mycobot_gateway/launch/pick_and_place_sorting.launch.py). Voir aussi le commentaire « Two things you'll likely have to tune on first run » dans le commit `bff55d0c`.

### Pipeline visuel attendu

```
T+0  s  : Gazebo s'ouvre, robot en pose home, 4 objets visibles
T+6  s  : color_object_detector annonce — `Color detector — topic /synth_camera_top/image, 640x480, …`
T+10 s  : sorting_orchestrator annonce — `Sorting orchestrator — world=…, order=[red,blue,green,yellow]`
T+15 s  : `Detections stable: ['blue', 'green', 'red', 'yellow']`
T+15 s  : `▶ Sorting red: pick=(…) → bin=(-0.22, -0.18)` puis cycle 25 s
T+40 s  : `▶ Sorting blue: …`
T+65 s  : `▶ Sorting green: …`
T+90 s  : `▶ Sorting yellow: …`
T+115 s : `Sorting complete: [red, blue, green, yellow]`
```

---

## Use case D — Range of motion (sans téléop)

Pour valider que les limites articulaires de l'URDF Gazebo correspondent à celles du robot physique (J2: ±134.6°, J3/J4: ±145°), avant de risquer une saturation sur le réel.

```bash
ros2 launch mycobot_gateway slider_control.launch.py target:=sim
```

Faire glisser chaque slider à son maximum / minimum dans la GUI Tkinter et vérifier visuellement que le bras :
- Atteint la position sans collision avec lui-même
- Ne traverse pas la table ni la base
- Revient à zéro sans hystérésis

---

## Use case E — Synthetic data smoke-test

Pour vérifier que les 4 caméras Gazebo publient bien (avant de lancer une collecte de 7500 poses qui peut prendre des heures).

```bash
ros2 launch mycobot_gateway synthetic_data_v3.launch.py num_samples:=10
```

Ouvrir RViz ou `rqt_image_view` et vérifier que `/synth_camera/image`, `/synth_camera_left/image`, `/synth_camera_right/image`, `/synth_camera_top/image` ont toutes une image vivante (10 Hz).

---

## Matrice « sim seul » vs « sim + réel » vs « réel seul »

| Use case | Sim seul (ce doc) | Sim + Réel (`target:=both`) | Réel seul (`target:=real`) |
|----------|-------------------|------------------------------|------------------------------|
| Tuning des gains téléop | ✅ obligatoire d'abord | ⚠️ après validation sim | ❌ jamais en premier |
| Validation pick-and-place | ✅ Use case B et C | n/a (bras n'a pas de gripper) | n/a |
| Range of motion | ✅ Use case D | ✅ pour cross-check | ⚠️ après use case D OK |
| Bench performance Excel | ✅ Use case A | ⚠️ `--duration N` libre | ⚠️ uniquement après READY en sim |
| Démonstration POC | ✅ idéal (reproductible, sans matériel) | ⚠️ besoin d'opérateur + Pi | ❌ trop fragile pour démo live |

---

## Troubleshooting sim-only

| Symptôme | Cause probable | Fix |
|----------|----------------|-----|
| Gazebo se lance mais le bras est invisible | `GZ_SIM_RESOURCE_PATH` mal défini | Vérifier que le launch contient `SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', …)` |
| `gz_ros_control` crash : *no ros2_control tag* | `controller.yaml` manquant ou URDF stale | Voir CHANGELOG 1.10.0 + restaurer `mycobot_description/config/controller.yaml` depuis `git show 6605e9a5` |
| `color_object_detector` died : *InvalidParameterTypeException* | Vieux launch avec `image_u_to_world_axis: 'y'` (YAML 1.1 → bool) | Mettre à jour `image_u_to_world_axis_name: 'world_y'` |
| Détections HSV au mauvais endroit (rouge à +0.12 au lieu de −0.12) | Y axis flip | `flip_u: True` dans le launch |
| `process has died` sur `image_bridge` | Camera topic typo | Vérifier que `synth_camera_top/image` apparaît dans `ros2 topic list` |
| Le robot bouge en sim mais aussi en réel ! | `target:=both` au lieu de `sim`, ou stale `bridge_tour` | `pkill -f bridge_tour && pkill -f mycobot_teleop` puis relancer avec `target:=sim` |
| Performance analyzer : verdict NOT READY répété | Gains trop hauts ou Wilor décroche | Revenir au preset 🐢 Safe (0.6/0.6/0.6/0.30), refaire le `--guided` |

---

## Workflow standard avant une session physique

1. **Sim seul (use cases A + D)** — 5 min — valider gains et limites
2. **Performance analyzer `--guided`** — 2 min — viser READY
3. **Pick-and-place mono (use case B)** — 1 min — sanity check IK
4. **Sorting (use case C)** — 2 min — sanity check perception + IK chained
5. **Si tous green** → lancer la session physique en suivant [`REAL_ROBOT_TEST_PROCEDURE.md`](REAL_ROBOT_TEST_PROCEDURE.md)

> **Règle d'or** : aucun nouveau gain, mapping ou plan de trajectoire ne doit jamais être essayé directement sur le robot physique. La sim est gratuite, le bras ne l'est pas.
