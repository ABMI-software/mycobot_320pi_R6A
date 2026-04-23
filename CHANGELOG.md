# Changelog

Toutes les modifications notables de ce projet sont documentées dans ce fichier.

Le format est basé sur [Keep a Changelog](https://keepachangelog.com/fr/1.0.0/),
et ce projet adhère au [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

---

## [2.2.0] - 2026-04-23

### 🎨 Dashboard ABMI + boutons dynamiques

Refonte complète de la GUI [`teleop/teleop_dashboard.py`](../teleop/teleop_dashboard.py) sur la charte **ABMI** (navy `#1B1A3E` + pink `#E6417A`) avec logo intégré. Trois onglets, KPI cards, caméra opérateur inline, comparaison sim ↔ réel côte à côte et ActionButton dynamiques avec feedback visuel.

### Ajouté

- **3-tab UI** (ttkbootstrap Notebook) — 🏠 Home · 📊 Analytics · 🎛️ Tuning
- **5 KPI cards** (`KpiCard` widget) sur Home : Execution mode, Command rate, SIM avg RMS, REAL avg RMS, Signal health — chacune avec accent coloré contextuel (vert / jaune / rose selon les seuils)
- **Badge de mode auto** en haut à droite : SIM / REAL / BOTH / OFFLINE, détecté par fraîcheur des topics `/joint_states` et `/from_robot` (fenêtre 2 s)
- **Caméra opérateur intégrée** — `mycobot_teleop.py` advertize `/teleop/camera/image` (JPEG compressée à 320 px de large, q=60, ~10 Hz via monkey-patch de `cap.read`) · le dashboard l'affiche dans le panneau gauche de Home. Plus besoin de fenêtre OpenCV séparée.
- **Comparaison SIM ↔ REAL** : bar chart RMS par joint (bleu/rose) sur Home · paires miroir de plots joint angles & tracking error sur Analytics
- **Hand position en cm** (au lieu de m) pour lecture directe
- **ActionButton dynamiques** (5 sur Home, 1 sur Tuning, 3 presets) :
  - Tooltip au survol (Toplevel navy) décrivant l'action + ses effets
  - Feedback visuel : label swap `⟳ …` → `✓` / `✗` pendant ~1,4 s
  - Désactivation in-flight pour absorber les double clicks
  - Toast horodaté dans la status bar Home (`✓ Send robot home · 14:23:05`)
- **Presets de gains** — 🐢 Safe start (0.6/0.6/0.6/0.3) · ⚙️ Nominal (1.2/1.2/1.6/0.25) · ⚡ Reactive (1.6/1.6/2.0/0.15). Le preset actif reste highlighted en SUCCESS solide.
- **Polling passif** de `get_angles` (0.3 s) pour remonter les angles réels quand `/joint_states` n'est pas là
- [`teleop/assets/abmi_logo.png`](../teleop/assets/) — logo chargé automatiquement

### Modifié

- [`docs/TELEOP_DASHBOARD.md`](TELEOP_DASHBOARD.md) — **réécrit** pour l'UI 3-tabs, sections par onglet, tableau topics in/out, troubleshooting mis à jour
- [`docs/TELEOPERATION.md`](TELEOPERATION.md) — section dashboard regénérée + topic `/teleop/camera/image` ajouté dans le listing des publications de Stage 4
- [`README.md`](../README.md) — ligne dashboard dans le tableau outils, commentaire T4 rafraîchi

### Non changé

- Protocole de téléopération · filtres (Kalman + EMA + slew 1°/frame) · mapping main → joints · bridge_tour / `trajectory_to_robot_bridge`
- Gains validés le 22/04/2026 sur robot physique : 1.2 / 1.2 / 1.6 / 0.25 (le preset ⚙️ Nominal)

---

## [2.1.0] - 2026-04-22 (soir)

### ✅ Premier test sur robot physique validé

Session de validation end-to-end sur le **MyCobot 320 Pi physique** (IP `10.10.0.223`). Le pipeline complet Astra → Wilor → rosbridge → JTC topic → trajectory_to_robot_bridge → bridge_tour → Pi → pymycobot est fonctionnel avec une latence main→bras de ~150–250 ms, imperceptible visuellement. Mouvements coordonnés, pas d'oscillation ni saturation sur les gains initiaux 0.6/0.6/0.6.

### Ajouté — Documentation

- `docs/TELEOP_ARCHITECTURE_VIZ.md` — visuel détaillé du pipeline complet (9 étapes, types, conversions d'unités, latences mesurées, exemples chiffrés). Répond au besoin de comprendre exactement comment la détection se traduit en mouvement.
- `docs/REAL_ROBOT_TEST_PROCEDURE.md` étendu :
  - Protocole de calibration sécurisé validé (gains 0.6/0.6/0.6, tfs 0.3, speed 25, montée progressive)
  - Conditions de Ctrl+C immédiat
  - Tableau de résultats du premier test
  - Points à creuser pour les prochaines sessions

### Ajouté — Infrastructure test réel

- `scripts/real_robot_preflight.sh` — check pré-vol 5 étapes avec exit codes distincts.
- `mycobot_gateway/gripper_to_robot_bridge.py` — bridge prêt pour gripper physique (non câblé : robot actuel sans pince).
- Flag `--no-gripper` dans `mycobot_teleop.py`.

### Corrigé

- IP par défaut Pi dans les docs : `10.10.0.225` → `10.10.0.223`.
- `bridge_tour` accepte `pi_ip` et `pi_port` comme paramètres ROS2.

### Validé — Session 22/04/2026 (soir)

| Check | Résultat |
|-------|----------|
| ping Pi + TCP 5005 | ✅ |
| bridge_tour TCP connect + ping/pong | ✅ |
| `get_angles`, `home`, `send_angles [45,0,…]` | ✅ |
| Téléop main complète (Wilor → bras physique) | ✅ |

### Points non bloquants

- `bridge_tour` receive_loop : ne log pas `📥 Reçu de Pi` (non bloquant pour téléop)
- Axe J6 (doorknob) : mapping `j6 = yaw * roll_gain` à valider visuellement sur réel

---

## [2.0.0] - 2026-04-22

Version majeure : **téléopération par la main** opérationnelle en simulation Gazebo, avec dashboard live de tuning + rapport de performance Excel.

### Ajouté — Téléopération

**Pipeline de téléopération** (hand-tracking → MyCobot) :
- `teleop/mycobot_teleop.py` — script principal Wilor + Orbbec Astra → joints via rosbridge. Arguments CLI exhaustifs (gains par axe, inversion, fps, time_from_start, camera backend).
- `teleop/orbbec_capture.py` — wrapper shared-memory pour Astra S via `oni_grabber` binaire OpenNI2. Auto-spawn, watchdog, survie aux Ctrl+C (start_new_session).
- `mycobot_gateway/trajectory_to_robot_bridge.py` — nœud ROS2 qui convertit `JointTrajectory` (rad) → JSON `send_angles` (deg) pour le `bridge_tour` / robot réel, avec rate-limit 15 Hz et deadband 1°.
- `mycobot_gateway/launch/mycobot_teleop.launch.py` — orchestration complète (Gazebo + controllers + rosbridge + bridge_tour + trajectory bridge) avec target `sim`/`real`/`both`.

**Dashboard de tuning** (`teleop/teleop_dashboard.py`) :
- GUI ttkbootstrap theme "darkly" connectée à rosbridge.
- 4 sliders live : x/y/z gain + `time_from_start`, appliqués via `/teleop/gains`.
- Bouton **⟲ Recalibrate hand origin** qui reset l'initial_pose de Wilor via `/teleop/recalibrate`.
- 3 plots matplotlib temps-réel (fenêtre 10 s) : Wilor XYZ / commandé vs actual / tracking error par joint avec ligne 5° cible.
- Tableau stats par joint : RMS, max, jitter avec flags colorés `✓ OK / △ JITTERY / ⚠ UNSTABLE`.
- Indicateur connexion rosbridge + compteurs de messages par topic.

**Performance analyzer** (`teleop/performance_analyzer.py`) :
- Générateur de rapport Excel multi-onglets avec verdict `READY / CAUTIOUS / NOT READY`.
- Mode `--guided` : protocole scripté 7 phases (idle, up/down, left/right, forward/back, combined, gripper, rest) sur 64 s.
- Mode `--duration N` : enregistrement passif libre.
- Onglets : Summary (verdict coloré), Per-joint tracking (+ bar chart), Scenarios, Signal health, raw_hand/raw_cmd/raw_actual.

**Pipeline de filtres porté du R5A / LeRobot** :
- Stage 1 (dans `HandTracker`) : jump clamp 2 m/s + Kalman XYZ (dt=1/30, q=r=5e-3).
- Stage 2 (sur les joints) : EMA α=0.20 + slew rate limiter 1 °/frame (= 30 °/s @ 30 Hz).
- Stage 3 (sur le gripper) : deadband 3° + EMA α=0.25 + slew 4 °/frame.

**Mapping main → joints** :
- Position delta-based (rel=(0,0,0) → tous joints 0°) :
  - Y → J1 base yaw, Z → J2 shoulder, X → J3 elbow
- Orientation (Euler ZYX) :
  - pitch → J4 + J5 (split 50/50, EE pointe comme la paume)
  - yaw → J6 (doorknob twist autour axe optique)
- `BASE_SCALE_DEG_PER_M` = 150 (tuné itérativement depuis 600 → 300 → 200 → 150).

**Gripper en Gazebo** :
- 4 joints revolute commandés explicitement (plus de `<mimic>`) — contournement du manque de support DART pour les contraintes mimic + limitations URDF pour les 4-bar linkages.
- `gripper_position_controller` (`JointGroupPositionController`) pilote les 4 joints simultanément via un `Float64MultiArray` avec signes mirrors `[servo_left, servo_right, tip_left, tip_right] = [-0.7, +0.7, +0.7, -0.7]` fermé.

**Robustesse** :
- Wrapper `try/except` sur `pose_computer.compute_relative_pose` pour éviter que le capture thread daemon meure silencieusement sur une exception Wilor.
- Watchdog `oni_grabber` qui respawn automatiquement si les frames s'arrêtent > 2 s.
- Bridge `/clock` Gazebo → ROS2 ajouté au launch (sans lui, le JTC dérive avec des deltas temporels erronés).
- Auto-resume du tracker au démarrage (sinon `tracking_paused=True` par défaut).

### Ajouté — Documentation téléopération

- `docs/TELEOPERATION.md` — pipeline complet, workflow 5 terminaux, filtres, mapping, limitations, historique des 25 commits.
- `docs/TELEOP_DASHBOARD.md` — manuel utilisateur du dashboard avec ascii art de l'UI, guide de tuning step-by-step, troubleshooting.
- `docs/TELEOP_TUNING.md` — référence exhaustive des paramètres (constantes module, flags CLI, controller.yaml) + troubleshooting complet.

### Ajouté — URDF / simulation

- Bloc `<ros2_control>` dans `mycobot_pro_320_pi_gazebo.urdf` avec 10 joints commandés (6 arm + 4 gripper).
- Plugin `gz_ros2_control/GazeboSimROS2ControlPlugin` lié au `controller.yaml`.
- `mycobot_description/config/controller.yaml` — 3 controllers : `joint_state_broadcaster`, `mycobot_controller` (JTC), `gripper_position_controller` avec PID par joint.

### Corrigé

- Bug `rclpy` dans env conda (Python 3.10 vs ROS2 Jazzy 3.12) → `--use-rosbridge` obligatoire documenté.
- IP Pi exposée en paramètre ROS de `bridge_tour` (`pi_ip`, `pi_port`) au lieu de hardcoded 10.10.0.218.

### Limitations connues

- Cinématique 4-barres du `pro_adaptive_gripper` non reproduisible en Gazebo (DART ne supporte pas les contraintes mimic, URDF pas de closed-loops). Contournement : commande explicite des 4 joints. Impact cosmétique uniquement — sur le robot réel, pymycobot gère le mécanisme mécaniquement.
- Axe J6 (doorknob) : mapping actuel `j6 = yaw`, à valider visuellement via le log RPY ajouté dans T3.

---

## [1.10.0] - 2026-04-23

### Ajouté
- **Pick-and-place multi-objets par couleur** (branche `feature/pick-and-place-sorting`)
  - Monde Gazebo `worlds/pick_and_place_sorting.sdf` :
    table 1.0×0.6 m, 4 objets à trier (cube rouge, cube bleu, cylindre vert, boîte
    jaune) côté +X, 4 bacs de réception colorés à parois côté −X, tous dans
    l'enveloppe d'atteinte ~0.32 m du MyCobot 320.
  - `color_object_detector` : segmentation HSV sur la caméra top-down, rétro-projection
    pinhole vers le repère robot, publie `/sorting/detections` (`color,x,y;…`),
    `/sorting/detector_status`, `/sorting/debug_image`.
  - `sorting_orchestrator` : machine à états qui boucle sur les détections,
    plan IK par objet, séquence approche/grasp/lift/place/retreat. Le « grasp »
    est émulé via le service Gazebo `/world/<world>/set_pose` (téléport du modèle
    sur l'EE pendant le portage, dépose dans le bac à la couleur correspondante).
  - Launch `pick_and_place_sorting.launch.py` : Gazebo + spawn + bridges (joints
    + 4 caméras) + détecteur (T+6 s) + orchestrateur (T+10 s).
- **Visuels caméra réalistes** dans le URDF Gazebo : les 4 caméras embarquent
  désormais un corps gris foncé + objectif noir cylindrique + LED rouge,
  au lieu de cubes 3 cm colorés qui ressemblaient aux objets à trier.

### Corrigé
- **`color_object_detector` : paramètre boolisé par YAML 1.1** —
  les valeurs `'y'` / `'x'` étaient coercées en `True` par rclpy (YAML 1.1
  truthy). Renommé en `image_u_to_world_axis_name` / `image_v_to_world_axis_name`
  avec valeurs `'world_y'` / `'world_x'`.
- **Axe Y de la caméra top-down inversé** : ajout de `flip_u: True` dans le
  launch ; les positions détectées correspondent maintenant aux positions SDF
  (vérifié rouge à −0.12, bleu à +0.12).

---

## [1.9.0] - 2026-04-21

### Documentation
- Réécriture complète de `SESSION_RESUME.md` (nettoyage du contenu fusionné corrompu)
- Mise à jour de tous les fichiers README et docs pour refléter l'état actuel

---

## [1.8.0] - 2026-04-15

### Ajouté
- **Gripper adaptatif** : Intégration du `pro_adaptive_gripper` d'Elephant Robotics dans le URDF Gazebo
  - 7 maillages DAE (gripper_base, left1/2/3, right1/2/3)
  - Joints fixés (pas de support `mimic` dans Gazebo Harmonic)
  - Mesh `link6_2022.dae` pour compatibilité avec les maillages du gripper
- **Vérification physique** : Limites articulaires corrigées selon l'URDF officiel elephantrobotics
  - J2 : ±159.9° → ±134.6°
  - J3, J4 : ±159.9° → ±145.0°
- **Anti-collision** : Rejet par cinématique directe dans le collecteur synthétique
  - Table clearance (z < 2cm)
  - Base column proximity check
  - Elbow height validation
  - Extreme fold-back rejection (|j2+j3| > 3.8 rad)
- **Pipeline d'automatisation** : `scripts/train_pipeline.sh` (merge → NDDS → training)
- **Script de merge** : `training/dream/merge_and_convert.py` pour combiner datasets réels+synthétiques
- **Monitoring** : `scripts/monitor_collection.sh` pour suivre la collecte en temps réel
- **Monde Gazebo v2** : `worlds/randomized_v2.sdf` — 6 lumières, 12 objets clutter, 3 murs
- **Collecteur v2** : `synthetic_data_collector_v2.py` — anti-collision FK, domain randomization avancée
- **Launch v3** : `synthetic_data_v3.launch.py` — collecte avec monde randomized_v2

### Corrigé
- **Stale install** : Suppression du répertoire `install/` orphelin dans `src/mycobot_R6A/`
- **Shebang Python** : `#!/usr/bin/python3` pour éviter conda Python 3.13
- **GZ_SIM_RESOURCE_PATH** : Ajout dans les launch files pour résoudre les meshes

---

## [1.7.0] - 2026-04-16

### Ajouté
- **DREAM fine-tuning expérimental** : `training/dream/finetune_real.py`
  - v1 (σ=4) : 0% détection — bug sigma mismatch avec DREAM natif
  - v2 (σ=2) : 0% détection — belief maps effondrées (MSE sur grille quasi-vide)
- **Dataset mixte** : `/tmp/dream_data/mixed_real_synth/` (18K frames — 10K réel ×5 + 8K synth)
- **Training mixte natif** : DREAM `train_network.py` sur dataset 18K, epoch 1 val=0.000474
- **Documentation ARCHITECTURE.md** : réécriture complète

---

## [1.6.0] - 2026-04-15

### Ajouté
- **DREAM VGG 50K** : training sur 50K frames synthétiques
  - Synthétique : 98.3% détection, 3.15px médiane
  - Réel (sim-to-real) : 13.2% détection, 172px médiane
- **Pick-and-place Gazebo** : `pick_and_place_node.py` + `pick_and_place.launch.py` (Step C)
- **DREAM inference node** : `dream_inference_node.py` — nœud ROS2 temps réel (YAML, venv, API)
- **Analyse adéquation** : conversion px→mm→degrés documentée

---

## [1.5.0] - 2026-04-03

### Ajouté
- **Module DREAM** : `training/dream/` — keypoint-based pose estimation (NVlabs DREAM 1.3.0)
  - `mycobot_fk.py` — Forward Kinematics + projection caméra (7 keypoints, paramètres DH)
  - `convert_to_ndds.py` — conversion datasets → format NDDS
  - `train_dream.py` / `train_dream_augmented.py` — wrappers d'entraînement
  - `evaluate_dream.py` — évaluation par keypoint (filtre sentinel -999.99)
  - `infer_dream.py` — inférence single-image + PnP
  - `visualize_ndds.py` — vérification visuelle des annotations
  - `manip_configs/mycobot320.yaml` — configuration 7 keypoints
- **Résultats training DREAM** :
  - ResNet-H : tué à epoch 10 (BatchNorm instable)
  - VGG-base : 25 époques, val=0.000438, 96.1% détection synth, 3.1px médiane
  - VGG-aug : 25 époques, val=0.000667, 96.6% détection synth, 3.1px médiane
- **Sim-to-real baseline** : ~26% détection réel (vs 97% synth) — domain gap identifié

### Corrigé
- Fix `evaluate_dream.py` : filtre coords < -900 (sentinel DREAM -999.99)

---

## [1.4.0] - 2026-04-02

### Ajouté
- **Capture réelle** : `training/capture_real.py`
  - 2000 poses × 2 caméras Pi = 4000 images réelles
  - FK safety : protection table, câbles, limites articulaires
  - 0 collisions sur 2000 poses
- **Diagnostic signal visuel** :
  - Corrélation pose↔pixel = 0.004 (quasi-nulle)
  - Robot = 15.4% (cam0) / 5.7% (cam3) des pixels
  - Dérive d'éclairage pendant capture (luminosité 140→82→143)
- **Dataset réel** : `datasets/real_dataset/` (4000 images, via Git LFS)

### Corrigé
- Gestion robuste des erreurs lors de la sauvegarde d'images

---

## [1.3.0] - 2026-04-01

### Ajouté
- **Camera server Pi** : `scripts/pi_camera_server.py` — serveur TCP pour 2 caméras Arducam USB
  - TCP:5006, streaming JPEG, nommage cam0/cam3
- **Prévisualisation caméras** : `training/preview_cameras.py`

---

## [1.2.0] - 2026-03-31

### Ajouté
- **Pipeline training v2** : `training/train.py` — multi-view ResNet50
  - Résultat synthétique : **12.97° MAE** (4 caméras)
  - ResNet18 single-view : 22.6° MAE
  - ResNet50 single-view : 16.5° MAE
- **Dataset synthétique** : `datasets/synthetic_dataset/` (5000 poses × 4 vues = 20K images, Git LFS)
- **Vérification dataset** : script montage + histogrammes + stats
- **Domain Randomization** : éclairage variable, materials aléatoires
- **Monde Gazebo v1** : `worlds/randomized.sdf` avec table et fond simple
- **PerImageNormalize** : normalisation par image pour les données réelles
- `training/dataset.py` — MyCobotDataset, MyCobotMultiViewDataset

---

## [1.1.0] - 2026-03-31

### Ajouté
- **Simulation Gazebo Harmonic** : intégration `ros_gz_sim` + bridge
  - URDF Gazebo compatible (`mycobot_pro_320_pi_gazebo.urdf`) avec inertials et plugins
  - 4 caméras simulées (front, right, left, top) à 640×480
  - Joint controllers Gazebo (`gz-sim-joint-position-controller`)
- **Collecteur données synthétiques v1** : `synthetic_data_collector.py`
  - Poses aléatoires → capture image + angles → format labels.csv
  - Collecte 5000 poses (20K images total, 4 caméras)
- **Launch files Gazebo** : `gazebo_sim.launch.py`, `synthetic_data.launch.py`

---

## [1.0.0] - 2026-03-26

### Ajouté
- **Architecture distribuée** Tour ↔ Raspberry Pi via TCP/IP
- **Bridge TCP** : `bridge_tour.py` (ROS2) ↔ `bridge_pi_simple.py` (Pi standalone)
  - Communication JSON bidirectionnelle sur TCP:5005
  - Auto-reconnexion Pi
- **Modes de contrôle** :
  - `simple_gui.py` : Interface graphique Tkinter (angles, coords, gripper, LED)
  - `slider_control.py` : Sliders RViz + Joint State Publisher
  - `teleop_keyboard.py` : Contrôle clavier WASD+ZX
  - `robot_commander.py` : CLI interactif
  - `joint_sync.py` : Synchronisation robot réel → RViz
- **URDF MyCobot 320 Pi** : modèle RViz + config RViz prête à l'emploi
- **Réseau** : PC Tour (10.10.0.115) ↔ Raspberry Pi (10.10.0.225)
- **Documentation** : README, SESSION_RESUME, guides de déploiement

### Validé
- Communication TCP ping/pong
- Contrôle LED (RGB)
- Lecture/écriture angles joints
- Mouvements go_home, go_zero
- Gripper open/close
- Synchronisation RViz temps réel

---

## [0.1.0] - 2026-03-26

### Initial
- Initialisation du dépôt
- Structure packages ROS2 (`mycobot_gateway`, `mycobot_description`)
- Configuration réseau Tour (10.10.0.115) / Pi (10.10.0.225)
