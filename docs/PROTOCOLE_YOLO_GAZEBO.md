# Protocole — YOLO dans Gazebo, validé contre la vérité terrain

*Protocole de travail de Claude pour l'intégration YOLO → localisation 3D →
tri dans Gazebo Harmonic, avec DREAM en parallèle plus tard. On avance **une
étape à la fois**. Chaque étape a un critère de sortie, et on ne passe à la
suivante qu'une fois ce critère atteint et **validé par Osama**. L'état de chaque étape
est tenu dans le [journal](#journal) en fin de fichier.*

Créé le 29/09/2026 · branche `feature/pick-and-place-osama`

---

## 0. Invariants — à vérifier avant chaque étape

| # | Invariant | Valeur exacte | Comment le vérifier |
|---|-----------|---------------|---------------------|
| I1 | **Caméras = celles du dataset DREAM 50K**, sans exception | voir tableau § 1.3 | test `tests/test_cameras_dream50k.py` (étape 1) ; `ros2 topic echo --once /synth_camera_top/camera_info` → `k[0] = 493.7925` |
| I2 | **Masses du robot inchangées** : bras 1,58 kg (répartition d'origine), pince 0,22 kg, total 1,840 kg | défauts xacro `arm_mass_kg=1.58`, `gripper_mass_kg=0.22` | xacro de l'URDF comparé à HEAD : 18 liens, 0 écart (vérifié le 29/09) |
| I3 | **Base du robot à l'origine du monde**, spawn `-z 0.0`, dessus de table à z = 0 | `world` = `base` | `ros2 run tf2_ros tf2_echo world base` → identité |
| I4 | **La vérité terrain Gazebo n'entre jamais dans la perception ni dans la planification** | topics GT sous `/validation/gt/*` uniquement | seul le comparateur s'y abonne ; `grep -rn "pose/info\|dynamic_pose\|/validation/gt" ` sur les nœuds de perception et de planification → vide |
| I5 | Horodatage de l'image conservé de bout en bout, `use_sim_time: true` | `header.stamp` de l'image recopié dans chaque détection | le comparateur apparie sur ce stamp |
| I6 | Environnements Python : les nœuds ROS tournent sous le Python système, YOLO sous `.venv` **par un tube** (`scripts/yolo26_service.py`) | jamais `import rclpy` dans `.venv`, jamais `ultralytics` dans le Python système | `conda deactivate`, `deactivate` avant `ros2` |
| I7 | Fichiers non modifiables | `mycobot_pro_320_pi_gazebo_nogripper.urdf` (référence DREAM), `training/dream/mycobot_fk.py` (étiquettes DREAM), `scripts/pick_dashboard.py`, tout dossier de checkpoints | `git diff` sur ces fichiers → vide |
| I8 | Apparence `robot_appearance:=original` pour tout ce qui touche DREAM | c'est le rendu de l'entraînement | argument de lancement |

**Masses (I2), état au 29/09.** Le passage bras 3 kg / pince 0,340 kg a été
**annulé** : les valeurs par défaut sont revenues à 1,58 / 0,22 dans l'URDF,
`sim_grasp.launch.py`, `banc_realiste.launch.py` et `trajectory_preview.launch.py`.
Les arguments `arm_mass_kg` et `gripper_mass_kg` restent disponibles en option
explicite, sans quoi la campagne de précision, qui les passe, casserait. **Ne
jamais remettre 3.0 / 0.34 par défaut.**

---

## 1. État des lieux (inspection du 29/09/2026)

### 1.1 Composants

| Composant | Existe ? | Fichier | Topic / interface | Réutilisable ? |
|---|---|---|---|---|
| Monde DREAM (génération 50K) | oui | `worlds/randomized.sdf` (table 0,9 × 0,9 m à z = 0, murs, 3 objets parasites) | `/world/randomized/...` | Référence seulement : on n'y ajoute pas d'objets |
| Monde « banc réel » | oui | `worlds/real_table.sdf` (plateau 622 × 449 mm, ArUco 19/23/25/26, cube et bac rouges, `table_camera`) | `/world/real_table/...` | **Oui, comme base de la scène de tri** (plateau et ArUco) |
| Monde de tri, ancien | oui | `worlds/pick_and_place_sorting.sdf` (4 primitives et 4 bacs plats) | — | Non : objets que yolo26 ne connaît pas |
| Monde « banc réaliste » | oui | `worlds/banc_realiste_yolo26.sdf` (8 pièces peintes, caméras arducam et SVPRO) | `/arducam/image_raw`, `/svpro/image_raw` | **Pièces : oui. Caméras : non** (ce ne sont pas celles de DREAM) |
| Robot avec pince | oui | `urdf/320_pi/mycobot_pro_320_pi_gazebo.urdf` (xacro) | `robot_description` | Oui, pour le pick-and-place |
| Robot sans pince (rendu DREAM) | oui, non suivi par git | `urdf/320_pi/mycobot_pro_320_pi_gazebo_nogripper.urdf` | — | Oui, comme **référence DREAM** (I7) |
| Pince | oui | liens `gripper_*` + `gripper_position_controller` (gz_ros2_control) | `/gripper_position_controller/...` | Oui : pince physique, prise par contact, **sans attache** |
| 4 caméras RGB | oui, dans les deux URDF, **à des poses différentes** | voir § 1.3 | `/synth_camera{,_right,_left,_top}/image` + `/camera_info` | Oui, avec les **poses du 50K** (étape 1) |
| Objets (4) | oui | `models/cube_rouge`, `cube_bleu`, `cylindre_vert`, `pave_jaune` | — | Oui |
| Bacs (4) | oui | `models/bac_rouge`, `bac_bleu`, `bac_vert`, `bac_jaune` (105 × 105 × 30 mm, statiques) | — | Oui |
| Générateur de modèles | oui | `scripts/generer_pieces_gazebo.py` (cotes lues dans `tri_couleur.py`) | — | Oui, ne pas retaper les cotes |
| Randomisation | partielle | `vision/sim_multicam_geometry.sample_scene_xy()` (1 cube + 1 bac), `real_table.launch.py randomize:=true seed:=`, `sim_scene_control` `/real_table/randomize` | service `Trigger` | À **étendre à 4 + 4** |
| YOLO | oui | `scripts/yolo26_service.py` (tube, `.venv`), poids `runs/detect/training/yolo/runs/pieces_v5_yolo26s/weights/best.pt`, 8 classes figées | JSON sur stdin/stdout | **Oui, tel quel** |
| YOLO sur Gazebo | oui, pour arducam et SVPRO | `scripts/yolo26_gazebo.py` (rclpy + service) | lit `/arducam/image_raw`, `/svpro/image_raw` | Modèle à suivre ; à transformer en nœud (étape 4) |
| Localisation multicaméra simulée | oui | `sim_multicam_detector.py` + `vision/sim_multicam_geometry.py` (silhouette HSV, cube de 40 mm) | — | Le modèle caméra (`Camera`, `load_cameras`) est à réutiliser. **Actuellement cassé**, voir § 1.4 |
| Vérité terrain | oui | `sim_sorting_grasp.object_poses()` (`gz topic -e /world/<w>/dynamic_pose/info`) ; bridge `/gz/dynamic_poses` (PoseArray **sans noms**) | — | Le principe : oui. À exposer proprement (étape 3) |
| Pick-and-place + FSM | oui | `sim_sorting_grasp.py` (IK `_solve_tip`, `sort_one`, prise vérifiée), `sim_scene_control.py` (`pose_source:=vision`, **bridé à `red_cube`**) | JTC `mycobot_controller` | **Oui, à réutiliser** (étape 9) |
| `/joint_states` | oui | `joint_state_broadcaster` (lancé par `sim_grasp.launch.py`) | `/joint_states` | Oui |
| DREAM | oui | `dream_inference_node.py` (`camera_topic`, défaut `/synth_camera/image` ; `output_prefix`) | `/dream/*` | Oui, sans modification |
| Dashboard | oui (réel) | `dream_validation_dashboard.py`, `scripts/yolo26_dashboard.py` | — | Plus tard ; le CSV de l'étape 10 le préparera |

### 1.2 Architecture actuelle (lancement `real_table.launch.py`)

```
 gz sim (real_table.sdf) ───────────────────────────────────────────────┐
   │ robot: mycobot_pro_320_pi_gazebo.urdf (pince, caméras à 1,1 m / 1,0 m)
   ├─ /clock ───────────────► parameter_bridge ─► /clock
   ├─ dynamic_pose/info ────► parameter_bridge ─► /gz/dynamic_poses (PoseArray, sans noms)
   ├─ synth_camera*/image ──► parameter_bridge ─► /synth_camera*/image + /camera_info  (bridge_multicam)
   ├─ /camera/image_raw ────► parameter_bridge ─► /camera/image_raw  (table_camera, 0,90 m)
   └─ gz_ros2_control ◄──── controller_manager
        ├─ joint_state_broadcaster ─► /joint_states ─► robot_state_publisher ─► /tf
        ├─ mycobot_controller (JTC) ◄─ /mycobot_controller/joint_trajectory
        └─ gripper_position_controller

 sim_multicam_detector  (/synth_camera*/image → silhouette HSV → cube XY)
 sim_scene_control      (FSM de sim_sorting_grasp, pose_source=vision, only=red_cube)
        services /real_table/randomize, /real_table/pick ; /real_table/bin_position
 sim_scene_panel        (boutons Randomiser / Lancer)
```

### 1.3 Les quatre caméras — **paramètres exacts du 50K DREAM**

Source vérifiée : `dream_data/synthetic_50k_ndds/_camera_settings.json`
(fx = 493,7925) et `mycobot_fk.GAZEBO_CAMERAS`. Les keypoints 3D des images
NDDS 0 à 3 (avant, droite, gauche, top) sont reproduits **à 0,000 mm** avec ces
poses. Documentation du run : `training/dream/VGG_ULTIMATE_V4_50K.md` (robot
**sans pince**).

| Caméra | Topic image | camera_info | Pose dans `world` = `base` : xyz (m), rpy (rad) |
|---|---|---|---|
| `synth_camera` (avant) | `/synth_camera/image` | `/synth_camera/camera_info` | `0.8 0.0 0.4`, `0 0.3 3.1416` |
| `synth_camera_right` | `/synth_camera_right/image` | `/synth_camera_right/camera_info` | `0.0 0.8 0.4`, `0 0.3 -1.5708` |
| `synth_camera_left` | `/synth_camera_left/image` | `/synth_camera_left/camera_info` | `0.0 -0.8 0.4`, `0 0.3 1.5708` |
| `synth_camera_top` | `/synth_camera_top/image` | `/synth_camera_top/camera_info` | `0.0 0.0 0.95`, `0 1.5708 0` |

Toutes les caméras : `horizontal_fov = 1.15`, 640 × 480, R8G8B8, 10 Hz, clip
0,05 à 10 m, sans bruit ni distorsion → **fx = fy = 493,7925, cx = 320, cy = 240**.

⚠ **Trois dispositions différentes coexistent**. Une seule est la bonne :

| Fichier | avant/droite/gauche | top | fov | Utilisé par |
|---|---|---|---|---|
| `…_nogripper.urdf` | 0,8 m | 0,95 m | 1,15 | **50K DREAM** ✅ |
| `…_gazebo.urdf` (HEAD et arbre de travail) | 1,1 m | 1,0 m | 1,15 | `real_table`, `sim_grasp` ❌ pas DREAM |
| `…_gazebo.urdf` au commit `bff55d0` (23/04) | 0,8 m | 1,2 m | 1,047 | anciens datasets v2 ❌ |

**Champ utile sur le plateau** (calcul du 29/09 avec les poses du 50K, points à
z = 20 mm) :

| Caméra | plateau visible | x max vu | cube de 40 mm vers (0,25 ; 0) |
|---|---|---|---|
| avant | 71 % | 0,39 m | ~30 px |
| droite | 93 % | 0,57 m | ~22 px |
| gauche | 95 % | 0,57 m | ~22 px |
| top | 81 % | 0,45 m | ~21 px |

→ Pour être vus par les quatre caméras, objets et bacs doivent avoir **x ≤ 0,37 m**
(avec marge). Les bacs de `banc_realiste_yolo26.sdf`, à x = 0,395 m, ne remplissent pas cette condition.
→ Les objets font environ 20 px dans la top : c'est le régime « petits objets » pour
YOLO, à mesurer, pas à supposer.

**Mesuré à l'exécution le 29/09** (`sim_grasp.launch.py world_name:=real_table
camera_layout:=dream50k bridge_multicam:=true headless:=true`) :

| Caméra | `frame_id` publié | TF `base → camera_link*` | K publié |
|---|---|---|---|
| avant | `mycobot_320/camera_link/synth_camera` | (0,800 ; 0 ; 0,400) | fx = fy = 493,7925, cx = 320, cy = 240 |
| droite | `mycobot_320/camera_link_right/synth_camera_right` | (0 ; 0,800 ; 0,400) | idem |
| gauche | `mycobot_320/camera_link_left/synth_camera_left` | (0 ; −0,800 ; 0,400) | idem |
| top | `mycobot_320/camera_link_top/synth_camera_top` | (0 ; 0 ; 0,950) | idem |

- Le `frame_id` est un nom Gazebo (`modèle/lien/capteur`), **pas une frame TF**.
  Pour la TF, on utilise le lien `camera_link*` + la rotation optique
  (`sensor_from_optical` de `sim_multicam_geometry`), jamais le `frame_id` tel quel.
- Fréquence mesurée en temps réel : **1,4 à 2 Hz** par caméra en headless
  (`update_rate` = 10 Hz en temps simulé ; Gazebo tourne plus lentement que le
  temps réel avec 4 caméras). Les appariements se font donc sur le stamp simulé,
  jamais sur l'horloge murale.
- Vue des 4 caméras : [`protocole_yolo_gazebo/etape1_cameras_dream50k.png`](protocole_yolo_gazebo/etape1_cameras_dream50k.png).

### 1.4 Défauts trouvés dans l'arbre de travail (non commité)

1. ✅ *corrigé le 29/09 (étape 0)* — **`load_cameras()` ne trouvait plus aucune caméra.** L'URDF enveloppe désormais
   les caméras dans `<xacro:if value="$(arg enable_cameras)">`, et
   `sim_multicam_geometry.load_cameras()` lit le fichier **brut** (`root.findall('gazebo')`).
   Résultat : `ValueError: four fixed cameras required in the URDF`, donc
   `sim_multicam_detector` et la démo `real_table` plantent au démarrage.
   Reproduit le 29/09.
2. **Butées articulaires modifiées** dans l'URDF avec pince (J3 ±2,53 → ±2,618,
   etc., les « butées mesurées » du banc réaliste). Elles sont sans effet sur DREAM,
   mais elles font partie du même chantier « réaliste ». **Décision D3.**

### 1.5 Pince

1. Le modèle actuel avec pince **a une pince** : 7 liens `gripper_*` et 4
   articulations pilotées par `gripper_position_controller`.
2. Un modèle **sans pince** existe : `…_nogripper.urdf`, c'est celui du 50K.
3. Le contrôleur l'ouvre et la ferme : `sim_sorting_grasp` s'en sert, avec `_SPAN_TABLE`.
4. `sim_sorting_grasp` **saisit physiquement** (contact, prise vérifiée sur la pose
   Gazebo de l'objet), sans attache. En revanche, `sorting_orchestrator` et
   `pick_and_place_node` **téléportent** l'objet (`set_pose`) : on ne les utilise pas.

---

## 2. Décisions

### 2.1 Pince → deux configurations, aucune modification des modèles existants

| Configuration | Fichier | Rôle |
|---|---|---|
| `robot_dream_baseline` | `…_nogripper.urdf` **tel quel** | Rendu identique au 50K : test de non-régression DREAM |
| `robot_yolo_pickplace` | `…_gazebo.urdf` (pince) **+ caméras du 50K** | Scène de tri, YOLO, pick-and-place, DREAM en parallèle |

Pourquoi : le keypoint distal `link6` est **sur la bride**, là où la pince est
montée. La pince le masque ou le rend méconnaissable depuis la plupart des vues, et
elle change la silhouette de `link5`. Le modèle de base a été entraîné sans pince.
Les liens et articulations `link1` à `link6` sont **identiques** entre les deux
fichiers. La pince n'ajoute que des liens enfants de `link6` : les repères des
keypoints ne bougent pas.

**On ne suppose pas l'effet sur DREAM, on le mesure** (étape 10) : même q, même
caméra, les deux configurations. On compare la détection par keypoint et l'erreur
en pixels par rapport à la projection FK.

### 2.2 Position du robot → on ne la bouge pas

Aujourd'hui, `world` = `base` : le robot est à l'origine, le dessus de table à
z = 0, dans le monde DREAM comme dans `real_table`. DREAM ne dépend que de
**T_cam←base**. Déplacer la base sans déplacer les caméras changerait :

- **T_cam←base** des 4 caméras, donc des vues que le réseau n'a jamais vues. Le
  point de vue du 50K serait perdu ;
- les **extrinsèques** : les poses caméra sont écrites dans le repère `world`,
  dans l'URDF et dans `mycobot_fk.GAZEBO_CAMERAS`. Il faudrait les réécrire en
  deux endroits, avec un risque de divergence silencieuse ;
- le **workspace** : `sample_scene_xy` et `_solve_tip` travaillent en
  coordonnées base ; les bornes seraient à recalculer ;
- la **génération DREAM** : un nouveau 50K ne serait plus comparable à l'ancien ;
- la **comparaison GT/DREAM** : il faudrait une transformation world→base de plus
  dans chaque comparaison, alors qu'elle est aujourd'hui l'identité.

Si un jour on veut le robot au bord de la table, on déplace **la table et les
objets**, pas le robot, et T_cam←base reste intact.

### 2.3 Décisions ouvertes (à trancher par Osama avant l'étape indiquée)

| # | Question | Recommandation | Avant l'étape |
|---|---|---|---|
| D1 | Quelle table pour la scène de tri ? | ✅ **retenu 29/09** : plateau et ArUco de `real_table.sdf` | 2 |
| D2 | Comment appliquer les caméras du 50K à l'URDF avec pince ? | ✅ **retenu 29/09** : argument xacro `camera_layout:={legacy,dream50k}`, défaut `legacy` | 1 |
| D2b | Habillage des boîtiers caméra | ✅ **retenu 29/09** : on garde les boîtiers **sombres** de l'URDF avec pince. Ceux du 50K sont des boîtes bleue, verte, rouge et jaune, qui donneraient à YOLO de faux cubes de couleur. Ce ne sont ni des paramètres de caméra ni des pièces du robot | 1 |
| D3 | Butées « mesurées » de l'arbre de travail : les garder ? | les garder (sans effet sur DREAM, plus sûres) ; à confirmer | 9 |
| D4 | Jeu d'objets | les 8 pièces peintes (yolo26 connaît leurs 8 classes) | 2 |
| D5 | Type de message des détections | `vision_msgs/Detection2DArray` (installé) + CSV complet | 4 |

---

## 3. Architecture visée

### 3.1 V1 : une caméra, YOLO seul

```
 gz sim  (tri_yolo.sdf : plateau réel + 4 pièces + 4 bacs ; robot avec pince ; caméras du 50K)
   │
   ├─ /synth_camera_top/image ─────────┬──────────────────────────► (plus tard) dream_inference
   │  /synth_camera_top/camera_info    │
   │                                   ▼
   │                       yolo_gazebo_node  ◄─tube─►  .venv : yolo26_service.py
   │                         ├─► /yolo/synth_camera_top/image_annotated   (visualisation)
   │                         └─► /yolo/synth_camera_top/detections        (Detection2DArray, stamp de l'image)
   │                                   │
   │                                   ▼
   │                       yolo_localizer (étape 6) ─► /yolo/objects_3d   (P_YOLO, repère base)
   │                                   │
   │                                   ▼
   │                          yolo_gt_comparator ─► /validation/report + CSV
   │                                   ▲
   └─ /world/<w>/pose/info ─► bridge ─► gazebo_ground_truth ─► /validation/gt/objects   (GT UNIQUEMENT)
```

### 3.2 Final

```
 4 caméras ─┬─► yolo_gazebo_node ×1 (4 abonnements) ─► detections/cam ─► yolo_localizer ─► /yolo/objects_3d
            │                                                                  │
            └─► dream_inference ×4 ─► /dream_<cam>/*                            ▼
                                                        sim_sorting_grasp (pose_source=perception, FSM existante)
                                                                               │
 /joint_states ─► FK ─► T_GT(t) ──┐                                            ▼
 gazebo_ground_truth ─────────────┴──► comparateurs (YOLO/GT, DREAM/GT) ─► CSV ─► dashboard
```

Nouveaux espaces de noms : `/yolo/*` et `/validation/*`. Ils sont à ajouter dans
`docs/ARCHITECTURE.md` (règle `ros2-conventions`).

---

## 4. Les étapes

Pour chaque étape : **objectif · fichiers · actions · critère de sortie · ce qu'on ne
touche pas**. Une étape ne commence qu'après validation de la précédente.

### Étape 0 — Remise d'aplomb *(avant tout code YOLO)*

- **Objectif** : une base qui démarre, des masses d'origine.
- **Fait le 29/09** : masses revenues à 1,58 / 0,22 (I2 vérifié).
- **À faire** : corriger `load_cameras()` pour qu'elle lise l'URDF **après
  xacro**. Soit elle prend le XML déjà développé, par exemple le paramètre
  `robot_description`, soit elle appelle `xacro` sur le fichier avec les mêmes
  arguments que le lancement. Ajouter un test qui charge les 4 caméras depuis l'URDF.
- **Critère de sortie** : `ros2 launch mycobot_gateway real_table.launch.py`
  redémarre, `sim_multicam_detector` trouve ses 4 caméras, le test passe.
- **Ne pas toucher** : aux poses caméra (c'est l'étape 1).

### Étape 1 — Caméras du 50K dans la scène avec pince, et contrôle des flux

- **Objectif** : l'URDF avec pince publie **exactement** les caméras du 50K.
- **Fichiers** (réalisé) : `…_gazebo.urdf` : argument `camera_layout` et deux
  propriétés (`camera_side_distance` 1,1 ou 0,8 ; `camera_top_height` 1,0 ou 0,95). Les
  poses sont les **seules** différences entre les deux dispositions, donc on ne duplique pas
  le bloc. Une valeur inconnue fait échouer xacro. `sim_grasp.launch.py` : argument
  `camera_layout`. `sim_multicam_detector` : paramètre `camera_layout`.
  `tests/test_cameras_dream50k.py` : 7 tests.
- **Le test doit échouer** si la sortie xacro avec `camera_layout:=dream50k` diffère de
  `mycobot_fk.GAZEBO_CAMERAS` (xyz, rpy à 1e-9), si fov ≠ 1,15, si la résolution
  ≠ 640 × 480, si fx calculé ≠ `_camera_settings.json` du 50K à 1e-6 près, ou
  si la fréquence ≠ 10. Il s'agit d'une garde qui bloque, pas d'un message affiché.
- **Contrôle à l'exécution**, noté dans le journal pour chacune des 4 caméras : `frame_id`,
  K publié, `ros2 topic hz` (on attend ~10 Hz, mais le facteur temps réel le
  réduit), TF `world → <lien caméra>`.
- **Critère de sortie** : test vert, tableau § 1.3 complété avec les valeurs
  mesurées, une image par caméra enregistrée dans le journal.
- **Ne pas toucher** : aux intrinsèques ni aux poses (I1), au fichier `_nogripper` (I7).

### Étape 2 — Scène de tri 4 objets + 4 bacs, randomisation contrôlée

- **Fichiers** : créer `worlds/tri_yolo.sdf`, **généré** à partir de `real_table.sdf`
  (plateau + ArUco, sans `red_cube`, `red_bin` ni `table_camera`), en y incluant les
  8 modèles `models/*` existants. Créer `launch/tri_yolo.launch.py`, qui inclut
  `sim_grasp.launch.py` avec `world_file`, `camera_layout:=dream50k`,
  `bridge_multicam:=true`, et les masses par défaut. Ajouter
  `sample_tri_scene(rng)` à côté de `sample_scene_xy` dans
  `vision/sim_multicam_geometry.py`.
- **Contraintes de la randomisation**, chacune vérifiée par un test sur 1 000 tirages :
  - chaque empreinte (bac 105 × 105 mm, objet selon ses cotes + marge) est entièrement sur le plateau ;
  - aucune empreinte ne chevauche une autre ; distance minimale objet-objet, objet-bac et bac-bac réglable ;
  - toutes les pièces sont hors du socle du robot (rayon à mesurer sur le mesh `base`) ;
  - **x ≤ 0,37 m**, et le centre projeté de chaque pièce est dans l'image de `synth_camera_top`
    (et on consigne dans combien des 3 autres caméras il se trouve) ;
  - chaque objet et chaque bac est atteignable : `_solve_tip` de `sim_sorting_grasp`
    trouve une solution pour l'approche et pour le largage ;
  - la graine est journalisée ; `seed:=N` reproduit la scène.
- **Pose du bras pendant la perception** : une « pose d'observation » qui libère la
  vue top du plateau (à définir et à vérifier sur l'image).
- **Critère de sortie** : 10 graines lancées, 8/8 pièces visibles à l'œil dans la
  top pour chacune, aucune collision initiale (les pièces ne bougent pas pendant
  les 2 premières secondes, GT à l'appui).

### Étape 3 — Vérité terrain Gazebo (validation uniquement)

- **Fichiers** : créer le nœud `gazebo_ground_truth.py` (+ entrée dans `setup.py`),
  et ajouter à `tri_yolo.launch.py` le bridge
  `/world/<w>/pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V`. Ce bridge couvre les
  modèles statiques, donc les bacs, et **conserve les noms**, contrairement à
  `/gz/dynamic_poses`.
- **Sortie** : `/validation/gt/objects`, un enregistrement par pièce : `object_id`
  (nom du modèle), `class` (nom = classe yolo26), position et orientation en
  `world`, puis en `base` via TF (l'identité ici, mais calculée et non supposée),
  `stamp` en temps simulé.
- **Critère de sortie** : pour une scène à graine fixe, la GT publiée est égale à
  la pose écrite dans le SDF généré, à moins de 1 mm près.
- **Garde I4** : aucun nœud de perception ou de planification ne référence `/validation/gt`
  ni `pose/info`. Un test fait ce `grep` et échoue sinon.

### Étape 4 — Nœud YOLO sur `synth_camera_top`

- **Fichiers** : créer `mycobot_gateway/yolo_gazebo_node.py`, qui reprend la
  mécanique de `scripts/yolo26_gazebo.py` : service `yolo26_service.py` lancé
  sous `.venv/bin/python`, dialogue par tube, **aucune modification du service**.
- **Paramètres** : `cameras` (V1 : `[synth_camera_top]`), `weights` (défaut
  `pieces_v5_yolo26s`), `conf_threshold` (défaut, celui du service, 0,10).
- **Sorties** : `/yolo/<cam>/detections` en `Detection2DArray` (D5). Le
  `header` est celui **de l'image** (stamp + `frame_id`). Pour chaque détection :
  `class_id` (nom), score, bbox. `/yolo/<cam>/image_annotated` affiche les boîtes,
  la classe et la confiance. Le CSV du nœud contient `camera_id, stamp, class_id
  (indice), class_name, confidence, x_min, y_min, x_max, y_max, center_u, center_v`.
- **Critère de sortie** : les boîtes s'affichent dans `rqt_image_view` ; sur 10
  scènes on consigne combien de pièces sont détectées et avec quelle confiance,
  **sans conclure** avant l'étape 5.
- **Ne pas faire** : connecter YOLO à la planification.

### Étape 5 — Comparaison YOLO 2D / GT

- **Fichiers** : créer `yolo_gt_comparator.py`, et ajouter à
  `sim_multicam_geometry` une projection de la boîte 3D d'un modèle.
- **Référence 2D** : Gazebo ne publie **pas** de boîte 2D. On n'en invente pas.
  La méthode : sommets de la boîte 3D du modèle (cotes lues dans `models/*/model.sdf` ;
  pour le cylindre, 2 cercles de 32 points), transformés par la pose GT, puis
  projetés avec K et T_cam←world de l'étape 1, et encadrés par min/max. C'est une
  boîte **amodale** : elle ignore les occultations. Les pièces masquées par le bras sont
  exclues grâce à la pose d'observation. Si c'est insuffisant, on passera au capteur
  Gazebo `boundingbox_camera` (+ système `gz-sim-label-system`), qui donne des boîtes
  visibles, à condition que sa pose et ses intrinsèques soient **identiques** à la caméra RGB.
- **Appariement** : par caméra et par image, algorithme hongrois sur le coût
  `1 − IoU` avec un seuil d'IoU de 0,5. Une détection non appariée est un faux
  positif ; une pièce GT visible non appariée est un faux négatif.
- **Métriques** : classe prédite contre classe réelle (matrice de confusion 8 × 8),
  `du, dv, e_pixel` (centre de la boîte YOLO contre centre de la boîte GT **et** contre le
  centre 3D projeté, les deux sont consignés), IoU, taux de détection, faux positifs et
  faux négatifs, confiance.
- **Critère de sortie** : rapport sur 50 scènes randomisées ; blocs de résultats
  **bruts** d'abord, analyse ensuite.

### Étape 6 — Localisation 3D par intersection rayon-plan

- **Fichiers** : créer `vision/ray_plane.py` (fonctions pures + test) et le nœud
  `yolo_localizer.py`.
- **Méthode** : (u, v) → rayon optique K⁻¹[u, v, 1] → repère `world` avec la TF
  de l'étape 1 → intersection avec le plan z = h. On calcule et on consigne
  **deux variantes** : h = 0 (plan de table) et h = hauteur utile de la classe
  (sommet de la pièce pour la top, mi-hauteur pour les vues obliques ; `y.HAUTEUR_BAC`
  et `pd.HAUTEUR_OBJET` existent déjà). On garde le meilleur des deux au vu des
  mesures, pas par principe.
- **Aucune calibration** : K et les poses sont celles de l'étape 1.
- **Métriques** : `dx, dy, dz, erreur_3d` entre P_YOLO et P_GT (centre de la pièce).
- **Critère de sortie** : distribution de `erreur_3d` par classe sur 50 scènes.

### Étape 7 — Les quatre caméras, sans fusion

- `cameras := [les 4]` dans `yolo_gazebo_node`. Même comparateur, **par caméra**.
- **Livrable** : tableau caméra × classe → taux de détection, confiance moyenne,
  `e_pixel` médian, `erreur_3d` médiane.
- La stratégie de sélection ou de fusion sera décidée **après** lecture de ce tableau.

### Étape 8 — Scénario 4 objets → 4 bacs, perception seule

- Association `cube_rouge→bac_rouge`, `cube_bleu→bac_bleu`,
  `cylindre_vert→bac_vert`, `pave_jaune→bac_jaune`, obtenue par la **classe**
  YOLO, jamais par un nom de modèle Gazebo.
- `/yolo/objects_3d` publie objets **et** bacs. Le système de tri ne reçoit rien d'autre.

### Étape 9 — Motion planning sur la perception

- **Réutiliser** la FSM et l'IK de `sim_sorting_grasp.py` (`sort_one`,
  `_solve_tip`, `_seed_over_shoulder`, prise vérifiée). **Pas de nouvelle FSM.**
- **Modification minimale** : une source de cibles `pose_source:=perception` qui lit
  `/yolo/objects_3d`, et la levée de la bride `only == ['red_cube']`.
  La vérification de prise sur la pose Gazebo reste possible, **mais comme contrôle
  a posteriori** journalisé, jamais comme cible.
- **Critère de sortie** : taux de tri réussi sur N scènes randomisées, avec la cause de
  chaque échec (détection, localisation, IK, prise, largage).

### Étape 10 — DREAM en parallèle (sans modifier DREAM)

- `dream_inference` × caméra, `camera_topic:=/synth_camera_top/image`,
  `output_prefix:=/dream_top`. Les mêmes images vont à YOLO et à DREAM.
- **Non-régression pince** : pour une série de q, rendu `robot_dream_baseline` et
  rendu `robot_yolo_pickplace`, mêmes caméras. On compare la détection par keypoint
  et l'erreur en pixels par rapport à la projection FK (`mycobot_fk`).
- Référence dynamique : `/joint_states` → FK → T_GT(t), comparée à T_DREAM(t), en appariant sur le stamp.

### Étape 11 — Journalisation CSV

Un fichier par campagne, `results/yolo_gazebo/<date>_<seed>/yolo_vs_gt.csv`
(`results/` n'est pas suivi par git). Colonnes :

```
stamp_sim, seed, camera_id, frame_id, object_id, gt_class, yolo_class_id, yolo_class,
confidence, x_min, y_min, x_max, y_max, u_yolo, v_yolo, u_gt, v_gt, du, dv, pixel_error,
iou, match (TP/FP/FN), plane_h, x_yolo, y_yolo, z_yolo, x_gt, y_gt, z_gt, dx, dy, dz, error_3d
```

Un en-tête `.yaml` à côté du CSV consigne la graine, les poids YOLO, le seuil, la
disposition des caméras (vérifiée par le test de l'étape 1), les masses et le commit.

---

## 5. Fichiers : récapitulatif

| Action | Fichier | Étape |
|---|---|---|
| modifié ✅ | `urdf/320_pi/mycobot_pro_320_pi_gazebo.urdf` (masses par défaut) | 0 |
| modifié ✅ | `launch/sim_grasp.launch.py`, `banc_realiste.launch.py`, `trajectory_preview.launch.py` (masses par défaut) | 0 |
| à modifier | `vision/sim_multicam_geometry.py` (`sample_tri_scene` ; projection de boîte 3D) | 2, 5 |
| modifié ✅ | `…_gazebo.urdf` (argument `camera_layout`), `sim_grasp.launch.py`, `sim_multicam_detector.py` | 1 |
| modifié ✅ | `vision/sim_multicam_geometry.py` (`load_cameras` passe par xacro) | 0 |
| créé ✅ | `tests/test_cameras_dream50k.py` | 1 |
| à créer | `worlds/tri_yolo.sdf` (généré), `launch/tri_yolo.launch.py` | 2 |
| à créer | `gazebo_ground_truth.py` | 3 |
| à créer | `yolo_gazebo_node.py` | 4 |
| à créer | `yolo_gt_comparator.py` | 5 |
| à créer | `vision/ray_plane.py`, `yolo_localizer.py` | 6 |
| à modifier | `sim_sorting_grasp.py` (`pose_source:=perception`) | 9 |
| à modifier | `setup.py` (entrées), `docs/ARCHITECTURE.md` (`/yolo`, `/validation`) | au fil de l'eau |
| **intouchables** | `…_nogripper.urdf`, `training/dream/mycobot_fk.py`, `scripts/yolo26_service.py`, `scripts/pick_dashboard.py`, checkpoints | — |

---

## Résultats yolo26 — campagne du 29/09 (10 graines, pose d'observation)

Pièces ni cachées ni tronquées ; bonne = bonne classe et IoU ≥ 0,5.

| Caméra | attendues | bonnes | mauvaise classe | manquées | fausses | e_pixel médian (p95) | IoU médian |
|---|---|---|---|---|---|---|---|
| **top** | 80 | **71 (89 %)** | 8 | 1 | 0 | 1,8 (2,5) px | 0,88 |
| avant | 47 | 31 (66 %) | 0 | 16 | 10 | 1,9 (13,0) px | 0,88 |
| gauche | 78 | 41 (53 %) | 5 | 32 | 10 | 2,1 (5,8) px | 0,85 |
| droite | 78 | 40 (51 %) | 5 | 33 | 5 | 1,8 (4,8) px | 0,88 |

Vue top, par classe, sur 10 scènes : cube_rouge, pave_jaune, cylindre_vert, cube_bleu,
bac_vert et bac_bleu 10/10 ; bac_rouge 9/10 ; **bac_jaune 2/10**, lu `bac_vert` 8 fois.

Lecture :
1. **La caméra top suffit pour la V1** : 7 classes sur 8 fiables, centre à ~2 px.
2. **bac_jaune → bac_vert** : c'est un écart de rendu. Dans Gazebo, le bac jaune sort
   moutarde ou olive (intérieur ombré), loin du jaune POSCA réel. Exemple :
   `protocole_yolo_gazebo/yolo26_top_seed4.png`. C'est le matériau du modèle qu'il faudrait
   corriger (`generer_pieces_gazebo.py`), pas yolo26. **Rien n'a été modifié.**
3. **Vues latérales** (tangage 17°) : les objets bas, derrière des bacs de 30 mm, sont
   masqués sans que la boîte amodale le sache. Les « manquées » y sont en partie des pièces invisibles.
4. **Fausses détections** : le socle gris clair du robot est lu `bac_bleu` à 0,10–0,13 dans la vue gauche,
   à chaque graine, juste au-dessus du seuil de 0,10.

## Correction des modèles Gazebo (29/09, validée par Osama)

Faite dans le **générateur** `scripts/generer_pieces_gazebo.py`, puis régénération des 8 modèles.
Les `model.sdf` ne sont jamais édités à la main.

### Bacs aux cotes du dossier de fabrication (`plans_cotes.pdf`, feuilles 6-7)

| | avant | maintenant (= plan) |
|---|---|---|
| extérieur | 105 × 105 × 30 mm | 105 × 105 × 30 mm |
| parois | 2 mm | **5 mm** |
| fond | 2 mm | 2 mm |
| ouverture | 101 × 101 mm | **95 × 95 mm** |
| coins | pleins | **retrait extérieur 2,5 × 2,5 mm** sur toute la hauteur |

`BIN_INNER_HALF_MM = 47,5` dans `sim_sorting_grasp` supposait déjà l'ouverture de 95 mm.
Le modèle et le code de dépose sont maintenant cohérents.

### Couleurs : la teinte RENDUE est calée sur la teinte des vraies images

Mesure : teinte médiane (H, échelle OpenCV 0-180) des pixels saturés (S ≥ 120) dans chaque
boîte. Images réelles : jeu `training/yolo/pieces_v5`, 74 images. Gazebo : caméra top dream50k,
graine 1.

| Bac | réel | Gazebo avant | matériau avant → après |
|---|---|---|---|
| jaune | 25 | 24 | 21 → **22** |
| vert | 44 | **36** | 39 → **47** |
| rouge | 173 | 0 | inchangé |
| bleu | 100 | 95 | inchangé |

- **Cause de la confusion** : ce n'était pas le jaune (24 contre 25 en réel), c'était le **vert**,
  rendu trop jaune. À 36, il tombait à mi-chemin du jaune réel (25) et du vert réel (44).
- **Origine du décalage** : l'éclairage de la scène décale la teinte rendue (vert 39 → 36, jaune 21 → 24).
  On corrige donc la teinte du matériau pour que le rendu tombe sur la teinte réelle.
- **Pièces non corrigées** : rouge et bleu, reconnus 10 fois sur 10, gardent la teinte de `tri_couleur`.
- **Objets de la même couleur** : le cylindre vert et le pavé jaune changent avec leur bac (même peinture).
- **Écarts non traités** : la saturation (185 en Gazebo contre 226-249 en réel) et la luminosité
  (212 contre 107-132) restent différentes. Elles dépendent de l'éclairage global, pas du modèle.

### Premier effet mesuré (graine 1, avant → après)

| Caméra | bonnes avant | bonnes après |
|---|---|---|
| top | 7/8 (bac_jaune lu `bac_vert`, 0,32) | **8/8** (bac_jaune 0,56 ; bac_vert 0,87 → 0,96) |
| droite | 5 | 7 |
| gauche | 5 (+ 1 mauvaise classe) | 5 (+ 1 mauvaise classe) |
| avant | 5 | 5 |

Image : `protocole_yolo_gazebo/avant_apres_seed1_top.png`.
### Campagne complète, mêmes 10 graines, avant → après

CSV : `protocole_yolo_gazebo/campagne_10_graines/` (avant), `campagne_10_graines_v2/` (après).

| Caméra | bonnes avant | bonnes après | mauvaise classe | manquées | fausses | e_pixel médian |
|---|---|---|---|---|---|---|
| **top** | 71/80 (89 %) | **73/80 (91 %)** | 8 → 6 | 1 → 1 | 0 → 0 | 1,8 px |
| avant | 31/47 (66 %) | 32/47 (68 %) | 0 → 0 | 16 → 15 | 10 → 10 | 1,9 px |
| gauche | 41/78 (53 %) | **48/78 (62 %)** | 5 → 6 | 32 → 24 | 10 → 9 | 2,2 px |
| droite | 40/78 (51 %) | 42/78 (54 %) | 5 → 5 | 33 → 31 | 5 → 4 | 1,8 px |

Caméra top, par paire objet / bac (10 scènes, confiance moyenne) :

| Couleur | objet avant → après | bac avant → après |
|---|---|---|
| rouge | 10/10 → 10/10 (0,91) | 9/10 → 9/10 (0,66 → 0,59) |
| bleu | 10/10 → 10/10 (0,95) | 10/10 → 10/10 (0,70 → 0,71) |
| vert | 10/10 → 10/10 (0,92 → 0,93) | 10/10 → 10/10 (**0,89 → 0,97**) |
| jaune | 10/10 → 10/10 (0,61 → 0,62) | **2/10 → 4/10**, lu `bac_vert` 8 → 6 fois |

Toutes caméras, pièces ni cachées ni tronquées : bac_vert 31/34 → 34/34,
cylindre_vert 16 → 19/35, pave_jaune 18 → 20/34, bac_jaune 3 → 6/36, le reste inchangé.

**Lecture :**
1. **La teinte est maintenant juste** : rendu bac jaune H = 25 (réel 25), bac vert H = 42
   (réel 44), identiques d'une scène à l'autre. Le bac vert est mieux reconnu (confiance
   0,97, min 0,92) et plus jamais confondu.
2. **Le bac jaune reste lu `bac_vert` 6 fois sur 10 malgré une teinte juste** : la teinte
   n'est donc plus la cause. Il reste la **saturation et la luminosité** : jaune Gazebo pâle et clair
   (S 184, V 212) contre jaune réel très saturé et plus sombre (S 249, V 132, caméra à
   l'exposition 75). Ni la couleur rendue (identique partout) ni la position n'expliquent seules
   les réussites et les échecs.
3. **Prochain levier possible, non appliqué** : caler aussi S et V rendus sur les images réelles.
   Pour les 8 pièces, ce serait un matériau plus sombre et plus saturé. Pour tout le banc, un éclairage
   ou une exposition de scène plus proche de l'arducam à 75. Cela touche aussi rouge et bleu,
   qui marchent : **à décider par Osama.**

### Campagne v3 : vert assombri (d'après les photos d'Osama, 29/09)

Les photos des vraies pièces et le jeu arducam s'accordent sur un point : **le vert est nettement plus
sombre que le jaune** (V ~105 contre 160-220 sur photo, 107 contre 132 sur arducam). Dans Gazebo, les deux
sortaient à V 212. Matériau vert : V 165 → 130 (`VALEUR_MATERIAU`). Rendu mesuré : vert V 191
(visé ~166 : l'éclairage de Gazebo n'est pas linéaire), jaune V 212.

**Résultat : aucun changement.** Mêmes chiffres qu'en v2 sur les 10 graines (top 73/80, bac_jaune 4/10).
Correction conservée, car elle rapproche le rapport jaune/vert de la réalité et ne dégrade rien.
**Arrêt des retouches de couleur** : ce n'est plus le levier. La confusion restante bac_jaune → bac_vert
tient au modèle (appris sur du jaune arducam très saturé et sombre) face à un rendu Gazebo pâle et clair.
Pour la supprimer, il faudrait soit ajouter des images Gazebo au jeu yolo26, soit caler l'éclairage
de la scène sur l'exposition 75. **Décision réservée à Osama.**

## Étapes 3, 4 et 6 en ROS (29/09)

Lancement unique : `ros2 launch mycobot_gateway tri_yolo.launch.py seed:=N` (options `yolo:=false`,
`observe:=false`, `headless:=true`).

```
gz sim ─ /synth_camera_top/image ─► yolo_gazebo_node ─► /yolo/synth_camera_top/detections   (Detection2DArray)
                                          │               /yolo/synth_camera_top/image_annotated
                                          ▼
                                    yolo_localizer ───► /yolo/objects_3d                   (Detection3DArray, base)

gz topic /world/tri_yolo/pose/info ─► gazebo_ground_truth ─► /validation/gt/objects       (Detection3DArray, base)
                                                             VALIDATION UNIQUEMENT
```

| Nœud | Rôle | Mesuré le 29/09 |
|---|---|---|
| `gazebo_ground_truth` | pose Gazebo des 8 pièces → repère `base` (via TF), stamp simulé, taille de la boîte | 8/8 à ≤ 0,5 mm du tirage ; **0,64 Hz** seulement |
| `yolo_gazebo_node` | yolo26 (service `.venv` inchangé) ; en-tête de l'image recopié ; `id` = indice de classe (ordre de `pieces_v5`), `class_id` = nom ; CSV optionnel (`csv_path`) | graine 1 : 8/8 ; ~1,8 Hz (= débit réel de la caméra) |
| `yolo_localizer` | centre de la boîte → rayon → plan z = h/2 (h de la classe, lu dans `model.sdf`) | graine 2 : 8/8, **erreur 3D 3,5 à 4,7 mm** |

Points techniques mesurés, à ne pas refaire :
- **Le bridge ROS perd les noms** : `ros_gz_bridge` Pose_V → `tf2_msgs/TFMessage` donne `child_frame_id` vide
  pour les 288 poses. Les bindings Python gz ne sont pas installés. D'où la lecture par
  `gz topic -e` avec un parseur qui tolère les champs nuls omis (`parse_pose_info`, testé sur un
  vrai message : `tests/data/pose_info_tri_yolo_seed1.txt`). Partagé avec `scripts/yolo26_tri_eval.py`.
- **0,64 Hz suffit pour des pièces immobiles**, pas pour suivre le bras à l'étape 10. Il faudra alors le
  système `PosePublisher` de Gazebo sur les modèles concernés.
- **`.venv` en tête du PATH** du shell : `python3` = `.venv`. Toujours retirer `.venv` du PATH avant `ros2`.

### Étape 6 : quelle hauteur de plan ? (campagne v3, détections justes, 10 graines)

| Caméra | z = 0 (table) | **z = h/2** | z = h (dessus) |
|---|---|---|---|
| **top** | 10,0 / 17,0 mm | **4,2 / 6,3 mm** | 2,7 / 8,2 mm |
| avant | 38,0 / 59,8 | 10,6 / 31,2 | 23,1 / 40,1 |
| gauche | 46,7 / 98,0 | 5,9 / 46,2 | 34,8 / 76,2 |
| droite | 46,1 / 77,8 | 5,9 / 25,7 | 31,9 / 57,6 |

(médiane / max de l'erreur XY). Choix V1 : **caméra top, plan z = h/2**, soit un pire cas de 6,3 mm sur les 73 détections.

**Biais systématique, diagnostiqué** (détail et méthode : rapport docx local, jamais poussé) :
toutes les erreurs ont **dx ≈ +3 à +4,5 mm**, quelle que soit la position. Décomposition sur les
73 détections justes de la caméra top :

| Source | dx moyen | dy moyen |
|---|---|---|
| A. géométrie seule : centre de la boîte **vraie** projetée → rayon → plan h/2 | +0,8 mm (radial) | 0,0 |
| B. boîte **YOLO** → rayon → plan h/2 | +4,0 mm | +0,7 |
| **C = B − A : part propre à YOLO** | **+3,2 mm (sd 0,7)** | +0,7 (sd 1,1) |

Autrement dit, les boîtes YOLO sont décalées de **−1,7 px en v**, vers le haut de l'image top, ce qui correspond à +x dans le monde.
- **Hypothèse de l'ombre réfutée** : le soleil (direction −0,5 ; 0,3 ; −0,9) projette les ombres vers **−x**.
  Une boîte qui les engloberait donnerait un biais dx **négatif** d'environ −6 mm (ombres de ~26 mm),
  alors qu'on mesure +3,2 mm.
- **Test du retournement** : on retourne l'image verticalement, on la passe à YOLO, puis on remet les boîtes à l'endroit.
  On obtient dv ≈ +0,2 px au lieu de −1,7 px. Un biais purement dû au modèle donnerait +1,7 px, un biais purement dû
  à la scène −1,7 px. **Le biais est donc moitié-moitié** :
  ~−0,8 px viennent de la scène (la face tournée vers la caméra, côté −x, est dans l'ombre car le soleil
  vient de +x, et YOLO la coupe) et ~−0,9 px du modèle (convention d'annotation des images réelles).
- **Solutions, par ordre de préférence** : (1) fine-tuner yolo26 avec des images Gazebo étiquetées
  automatiquement à partir de la GT (voir « Entraîner yolo26 » plus bas) ; (2) ajuster la boîte 3D connue de la
  classe sur la boîte YOLO, plutôt qu'un seul rayon par le centre, ce qui supprime la part A ; (3) un décalage
  constant (+3,2 mm) serait juste **en simulation seulement** : il n'est pas transférable au réel, donc **non appliqué**.

## Pourquoi yolo26 ne reconnaît pas le bac jaune dans Gazebo

1. **Ce n'est pas la teinte** : après correction, la teinte rendue est juste (H 25 contre 25 en réel ;
   bac vert 42 contre 44).
2. **C'est l'apparence globale** : le modèle a appris sur **74 images réelles** (44 arducam, 30 SVPRO,
   `training/yolo/pieces_v5`). Le jaune réel y est **très saturé et plutôt sombre**
   (S 249, V 132 à l'exposition 75). Dans Gazebo, le jaune rendu est **pâle et clair** (S 184, V 212) :
   la saturation plafonne sous l'éclairage de la scène. Le modèle n'a jamais vu un jaune aussi pâle et
   le range dans la classe la plus proche qu'il connaisse, `bac_vert` (le vert réel est plus clair et moins saturé).
3. **Le modèle est peu contraint** : 74 images, avec train = val (mAP 0,995 sans valeur de
   généralisation). La moindre différence de rendu fait basculer une classe.
4. **Ce qui ne marche pas** : retoucher les couleurs à la main (trois campagnes : v1 → v3, le bac jaune passe de 2/10
   à 4/10, puis plus rien ne bouge).
5. **Ce qui marchera** : montrer à yolo26 des images Gazebo (section suivante), ou rendre la scène
   aussi sombre et saturée que l'arducam réelle (éclairage de scène). La première solution est préférable : elle corrige
   aussi le biais de −1,7 px et ne touche pas au réel.

## Entraîner yolo26 avec des images Gazebo (procédure, non encore lancée)

Recette du modèle actuel (`runs/detect/training/yolo/runs/pieces_v5_yolo26s/args.yaml`,
`tri_couleur.ENTRAINEMENT`) : `yolo26s.pt`, 100 epochs, imgsz 640, batch 16, patience 30,
seed 1509, fliplr 0,5, mosaic 1, hsv_h/s/v 0,015/0,7/0,4.

1. **Générer les images Gazebo étiquetées automatiquement** : pour N graines, lancer `tri_yolo.launch.py`,
   enregistrer l'image top **brute**, puis écrire les étiquettes YOLO (`classe cx cy w h`, normalisées) à partir
   des boîtes GT projetées (même calcul que `yolo26_tri_eval.py`). Pièces cachées : à exclure. Ordre
   des classes : **celui de `pieces_v5/data.yaml`**, qu'il ne faut jamais changer.
2. **Nouveau jeu, dans un nouveau dossier** (ne jamais écraser `pieces_v5`) :
   `training/yolo/pieces_v6_gazebo/` = images réelles v5 + images Gazebo, avec un **val séparé**
   (images réelles tenues à part, pour mesurer que le réel ne se dégrade pas).
3. **Entraîner** (Python `.venv`, jamais ROS) :
   ```bash
   deactivate 2>/dev/null; cd ~/Osama_ws/src/mycobot_R6A
   .venv/bin/yolo detect train model=runs/detect/training/yolo/runs/pieces_v5_yolo26s/weights/best.pt \
       data=training/yolo/pieces_v6_gazebo/data.yaml epochs=100 imgsz=640 batch=16 \
       patience=30 seed=1509 project=training/yolo/runs name=pieces_v6_gazebo_yolo26s
   ```
   (on part des poids v5 : un fine-tune garde l'acquis réel ; les sorties vont sous
   `runs/detect/training/yolo/runs/`, car ultralytics préfixe le chemin)
4. **Valider sur les deux mondes** : `scripts/yolo26_tri_eval.py` sur les mêmes 10 graines (Gazebo) **et**
   les images réelles tenues à part. Critère d'adoption : bac_jaune ≥ 9/10 en top et aucune classe réelle
   qui recule.

## Journal

| Date | Étape | État | Preuve / mesure |
|---|---|---|---|
| 29/09/2026 | Inspection | fait | § 1 ; caméras du 50K identifiées et vérifiées à 0,000 mm sur NDDS 0 à 3 |
| 29/09/2026 | 0 — masses | fait | défauts 1,58 / 0,22 ; xacro contre HEAD : 18 liens, 0 écart, 1,840 kg |
| 29/09/2026 | 0 — `load_cameras` | fait | lit l'URDF après xacro ; `tests/test_sim_multicam.py` 7/7 (0/7 avant) ; `real_table` headless graine 1 : cube vu par les 4 caméras, localisé à **2,2 mm** de la position tirée |
| 29/09/2026 | 1 — caméras du 50K | fait | `camera_layout:=dream50k` ; `legacy` produit un URDF identique à l'ancien ; `tests/test_cameras_dream50k.py` 7/7, et il échoue bien si la top est décalée de 1 cm ; Gazebo : TF 0,800/0,950 m, K = 493,7925 ; images : `protocole_yolo_gazebo/etape1_cameras_dream50k.png` |
| 29/09/2026 | 2 — scène (en cours) | premier essai OK, **en attente de l'observation d'Osama** | `tri_yolo.launch.py` + `vision/tri_scene.py`. Le monde est reconstruit à chaque lancement depuis `real_table.sdf` (pas de `tri_yolo.sdf` figé qui pourrait diverger). Graine 1 : les 8 pièces sont en place à ≤ 0,5 mm du tirage, z = 0, rien n'a bougé ; images : `protocole_yolo_gazebo/etape2_seed1.png` |
| 29/09/2026 | 2 — mesures | fait | **portée** : colonne de prise (110 mm → prise → levée) et colonne de dépôt atteignables jusqu'à r = 0,28 m, pas à 0,30 m, quel que soit l'azimut (0, ±50, ±100°) → `REACH_MAX = 0,28`. **Place disponible** : avec 30 mm entre bacs, seuls 3 bacs tiennent dans la bande atteignable ; d'où 10 mm entre bacs et 30 mm autour des objets. Tirage : ~1 s médian, 5 s au pire. **Pose d'observation** q = (0, 60, −70, 0, 0, 0)° : 4,3 % du plateau masqué dans la vue top (13,4 % à q = 0), ombre du bras à x ≈ −0,15 m, hors plateau |
| 29/09/2026 | 2 — zone de travail | fait (consigne d'Osama) | objets et bacs **entre les marqueurs, devant 19/23**, comme sur le banc réel : bande x ∈ [0,146 ; 0,494] m, calculée depuis les poses des marqueurs dans le monde. La portée 0,28 m n'est **plus imposée** ; elle est **signalée** au lancement (en moyenne 4,9 pièces sur 8 au-delà) → l'IK outil vertical devra être étendue à l'étape 9, puisque le vrai bras atteint ~0,39 m avec l'outil incliné. Tirage : 134 ms médian. `tests/test_tri_scene.py` : 6 tests, dont 300 tirages vérifiés contrainte par contrainte |
| — | 2 — plus tard | demandé par Osama | bouton **« Randomiser »** qui redéplace objets et bacs sans relancer Gazebo (sur le modèle de `sim_scene_panel` et du service `/real_table/randomize`) |
| 29/09/2026 | 4-5 (version rapide) — yolo26 contre la GT | fait | `scripts/yolo26_tri_eval.py` (outil de VALIDATION : il lit la GT). yolo26 `pieces_v5_yolo26s`, seuil 0,10, service inchangé. Boîtes GT = boîte 3D du `model.sdf` projetée (amodale, rognée au bord de l'image ; « cachée » si > 50 % couverte par une pièce plus proche). 10 graines, CSV : `protocole_yolo_gazebo/campagne_10_graines/` |
| 29/09/2026 | modèles Gazebo | fait | bacs aux cotes du plan (parois 5 mm, ouverture 95, retraits d'angle) ; teinte du matériau vert 39 → 47, jaune 21 → 22, pour que le rendu tombe sur les teintes réelles mesurées ; graine 1 caméra top : 7/8 → 8/8. Détail : § « Correction des modèles Gazebo » |
| 29/09/2026 | campagne v3 (vert assombri) | fait | aucun changement (top 73/80, bac_jaune 4/10) ; arrêt des retouches de couleur |
| 29/09/2026 | 3 — vérité terrain | fait | `gazebo_ground_truth` → `/validation/gt/objects` ; `tests/test_gazebo_ground_truth.py` (5), `tests/test_gt_isolation.py` (garde I4) |
| 29/09/2026 | 4 — nœud YOLO | fait | `yolo_gazebo_node` → `/yolo/<cam>/detections` + image annotée ; 8/8 en direct |
| 29/09/2026 | 6 — localisation 3D | fait (V1 top) | `yolo_localizer` → `/yolo/objects_3d` ; 3,5-4,7 mm en direct ; biais +x à expliquer |
| — | 5 — comparateur en nœud | à faire | aujourd'hui `scripts/yolo26_tri_eval.py`, hors ligne (2D) ; à étendre au 3D et à passer en nœud |
| 29/09/2026 | campagne yolo26 après correction | fait | top 71 → 73/80 ; bac_vert 0,89 → 0,97 ; bac_jaune 2 → 4/10 (teinte juste, reste S/V) ; gauche 53 → 62 % |
| — | 2 — pour l'étape 9 | à retenir | `sort_one` ouvre la pince en grand (±81 mm de demi-encombrement) avant de descendre à 18 mm, sous le rebord des bacs (30 mm). Avec 30 mm d'écart autour des objets, les doigts peuvent toucher un bac voisin : il faudra une ouverture juste suffisante, ou un écart plus grand, à mesurer |
