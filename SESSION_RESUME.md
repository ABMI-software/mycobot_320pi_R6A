# Reprise — pick adaptatif LIVE par démonstration

## État actuel (10 septembre 2026 — après-midi, Gazebo réaliste et protocole d'essais)

### Ce qui a été accompli aujourd'hui

**La simulation reproduit enfin le banc réel.** `worlds/real_table.sdf` ne pose
plus une table générique mais le plateau mesuré — 622 × 449 × 8,5 mm — avec sa
texture bois reconstruite depuis les photos, et les quatre ArUco de 50 mm aux
positions relevées. Une apparence réaliste du robot est disponible en option
(`robot_appearance:=realistic`), strictement visuelle.

**Le protocole d'essais de précision est indexé.**
`training/calibration/PROTOCOLE_ESSAIS_PRECISION.md` relie chacun des treize
essais à sa norme, son mode opératoire, son résultat et ses quatre supports
(les deux MD, l'onglet du classeur, la section du rapport).

**Une conclusion fausse a été retirée.** Les marqueurs ArUco font bien 50 mm.
Le −2,6 % mesuré sur leurs côtés est un biais de détection lié à l'obliquité
(r = −0,920), pas une erreur d'impression : les distances entre centres, elles,
sont justes à −0,044 %. Une vraie erreur d'échelle frapperait les deux à
l'identique.

### Décisions prises

- **Ne pas recalibrer l'extrinsèque arducam.** La planche a bougé le 10/09
  (rotation −1,750°, translation 18,8 / −6,5 mm, résidu 0,39 mm) mais **pas la
  caméra** — le trépied du fond n'a bougé que de 0,2 px. Le lien caméra ↔ base
  robot est donc intact ; refaire l'ajustement contre des positions nominales
  périmées y injecterait les 19 mm.
- **`workspace_markers.yaml` est périmé** et doit être considéré comme tel par
  toute calibration future.
- **Le protocole du stylo est abandonné.** Faire tracer un point au robot pour
  mesurer la justesse contre une croix demandait un appui dont l'amplitude
  n'était pas maîtrisable : le stylo a ripé et est sorti de la pince.

### Prochaines actions

1. **[ROUGE]** Mesurer un marqueur **au pied à coulisse**. C'est l'artefact
   étalonné qu'exige VDI/VDE 2634-1, et son absence est exactement ce qui a
   laissé vivre deux jours la conclusion fausse sur la taille des tags.
2. **[ROUGE]** Le bridge de la Pi est tombé **deux fois** en une heure pendant
   des séquences de mouvement (`Connection refused` ensuite). Regarder ce qu'il
   affiche sur la Pi au moment où il meurt.
3. **[JAUNE]** Recalibrer avec des marqueurs **à plusieurs hauteurs** : les
   extrinsèques ne valent aujourd'hui que dans le plan Z = 0.
4. **[JAUNE]** Remonter dans le dépôt les scripts d'essai restés dans le
   répertoire de session — **onze essais sur treize ne sont pas rejouables**.
5. **[VERT]** ISO 9283 § 7.3, exactitude et répétabilité de **distance** : la
   seule caractéristique normalisée qu'une mesure aux codeurs puisse rapporter
   en valeur vraie, un décalage constant s'annulant dans la différence.

### Commande rapide de reprise

```bash
conda deactivate
cd ~/ros_jazzy && colcon build --packages-select mycobot_description mycobot_gateway --symlink-install
source install/setup.bash
ros2 launch mycobot_gateway real_table.launch.py robot_appearance:=realistic
```

---

## État actuel (9 septembre 2026 — soir, méthodologie et extrinsèque)
> **Date de dernière mise à jour :** 9 juin 2026 (calibration main-œil — nœud hand-eye en cours de validation)
> **Version :** 2.2.0 (téléop) · 1.10.0 (sorting) · 1.14.0-pre (calibration) · 1.15.2-pre (pick-and-place ArUco)
> **Branche active :** `feature/pick-and-place`
> **Repository :** https://github.com/ABMI-software/mycobot_320pi_R6A
> **Pi réelle :** `10.10.0.221` (pas `.223`/`.225` comme certains anciens docs)

---

## État actuel (9 juin 2026 — soir — calibration main-œil sur robot réel)

### Ce qui a été accompli aujourd'hui

- **Nœud `calibrate_hand_eye_node`** : implémenté et lancé sur robot réel.
  - Souscrit à `/camera/image_raw` (Orbbec, ~5 Hz) + `/joint_states`.
  - Détecte le marqueur ID 20 (3 cm, DICT_4X4_1000) via ArUco + solvePnP.
  - Balayage automatique (`a`) : génère 30 poses en perturbant j4/j5/j6 autour
    de la base, attend 3 s de stabilisation, capture si marqueur visible.
  - Solve Tsai (OpenCV hand-eye) + sauvegarde `hand_eye_calibration.yaml`.
- **Validation robot réel** : connexion confirmée à `10.10.0.221:5005`.
  - Angles lus : `[14.58, -136.05, 20.83, 32.43, -89.64, 0.26]°`.
  - Marqueur ID 20 détecté à `[0.001, -0.038, 0.510]` m (position stable).
  - Commande servo release opérationnelle : `ros2 topic pub --once /to_robot std_msgs/msg/String 'data: "stop"'`.
- **`aruco_localizer_node`** : mis à jour avec les vrais IDs et tailles mesurées
  (IDs 19/25/23/26, 25 mm) et chargement positions depuis `workspace_markers.yaml`.
- **`joint_sync.py`** : parsing d'angles refactorisé — accepte `ANGLES:`, `angles:`,
  `angles_ok:` ; ignore les réponses d'erreur `-1`.
- **Nodes caméra** : `orbbec_camera_publisher`, `camera_live_view`, `camera_web_view`
  ajoutés + enregistrés dans `setup.py`.
- **`calibrate_extrinsic_node`** : nœud d'étalonnage extrinsèque caméra (PnP 4 marqueurs sol).
- **`reach_target_aruco_node`** : nœud de déplacement vers cible ArUco.
- **`bridge_tour.py`** : IP par défaut `.225` → `.221` ; logs send/recv passés en `debug`.
- **Calibration sauvegardée** : `training/calibration/camera_extrinsic.yaml` + `workspace_markers.yaml`.

### Décisions prises

- Pi réelle confirmée à `10.10.0.221` (mettre à jour CLAUDE.md séparément).
- `aruco_detect_scale = 1.6` pas de callback live → redémarrer le nœud pour changer.
- Marqueur ID 20 (3 cm) à 51 cm détectable mais instable (~50 % des frames) :
  cause probable = éclairage rasant ou légère inclinaison. Pas bloquant pour le balayage.

### Prochaines actions

1. [ROUGE] Lancer le balayage auto (`a`) avec le marqueur stable face caméra — collecter ≥ 20 échantillons.
2. [ROUGE] Lancer `s` pour résoudre et sauvegarder `hand_eye_calibration.yaml`.
3. [JAUNE] Valider la calibration : envoyer une pose connue, comparer position prédite vs réelle.
4. [VERT] Commiter `scripts/real_robot_preflight.sh` (IP `.221`) sur `feature/teleoperation`.
5. [VERT] Mettre à jour `CLAUDE.md` : Pi IP `10.10.0.223` → `10.10.0.221`.

### Commande rapide de reprise

```bash
conda deactivate && source /opt/ros/jazzy/setup.bash && source ~/ros_jazzy/install/setup.bash
# Terminal 1 — bridge
ros2 launch mycobot_gateway pick_and_place_aruco_real.launch.py
# Terminal 2 — nœud hand-eye (log vers fichier pour surveillance)
ros2 run mycobot_gateway calibrate_hand_eye_node \
  --ros-args -p aruco_detect_scale:=2.5 -p marker_size:=0.03 -p marker_id:=20 \
  2>&1 | tee /tmp/handeye.log
# Surveiller
tail -f /tmp/handeye.log | grep -E "VISIBLE|hors champ|Capture|Balayage"
```

---

## Handoff pick-and-place (a lire en premier pour la prochaine session)

- Handoff detaille du 4 juin 2026 : [`docs/PICK_AND_PLACE_HANDOFF_2026-06-04.md`](docs/PICK_AND_PLACE_HANDOFF_2026-06-04.md)
- Handoff detaille du 3 juin 2026 : [`docs/PICK_AND_PLACE_HANDOFF_2026-06-03.md`](docs/PICK_AND_PLACE_HANDOFF_2026-06-03.md)
- Contient :
  - ce qui a ete implemente aujourd'hui (sim + reel),
  - l'etat exact de validation,
  - les commandes de reprise,
  - le plan de test onsite avec interfaces graphiques.

### Ce qui a été accompli

**Le maillon faible de la chaîne est identifié, et ce n'est pas le bras.**

Validation de l'extrinsèque par leave-one-out, sans recalibrer : on ajuste la
pose caméra sur une partie des marqueurs, on prédit un marqueur jamais vu.

| ajustement | redondance | erreur au point non vu |
|---|---|---|
| 4 centres | 2 ddl | inexploitable (le système s'effondre) |
| 16 coins | 26 ddl | **5,46 mm** en moyenne, 12,25 mm au pire |

Le fichier de calibration annonce `erreur_sol_rms_mm: 0.594` — c'est un
**résidu d'ajustement**, pas une justesse. La vraie erreur est **9× plus
grande**, et du même ordre que le biais de sens d'approche (5,9 mm).

Cause probable : les positions des marqueurs viennent d'un relevé au mètre
ruban dont `workspace_markers.yaml` borne l'erreur à ±5 mm. On ne peut pas être
plus juste que sa référence.

**Document de méthodologie** :
[`METHODOLOGIE_PRECISION.md`](training/calibration/METHODOLOGIE_PRECISION.md) —
ce que `FK(q_lu)` mesure, ISO 9283, les 7 types d'essai, quand une référence
externe est nécessaire.

### Décisions prises

- **Jamais d'ArUco sur la pince** : le montage n'est pas stable, l'hypothèse de
  transformation rigide constante tombe. Le hand-eye du 10/07 est abandonné
  (résidu 24,4 mm ; 20 poses sur 22 sous le seuil de 30 px du code).
- La répétabilité est donnée en **RP ISO 9283**, jamais en écart max.
- On ne recalibre pas : la validation évalue, elle n'écrase rien.

### Prochaines actions

1. [ROUGE] Re-relever les positions des 4 marqueurs de planche au pied à
   coulisse ou par ajustement conjoint des deux caméras (±0,3 mm comme déjà
   fait pour le 19). C'est ce qui plafonne toute la chaîne à ~5 mm.
2. [ROUGE] Passer l'extrinsèque de production aux **16 coins** au lieu des
   4 centres — 26 degrés de liberté de redondance au lieu de 2.
3. [JAUNE] Chiffrer la **précision globale en boucle fermée** (essai 7) :
   `P_vision`, `P_atteint`, correction, itérations, statut pince sur ~30
   tentatives. Aucune métrologie externe requise.
4. [JAUNE] Comparateur numérique (~60 €) pour transformer les bornes
   inférieures de répétabilité en vraies valeurs.
5. [VERT] Vérifier le tag de 100 mm au pied à coulisse.

### Commande rapide de reprise

```bash
cd /home/genji/Osama_ws/src/mycobot_R6A
python3 -c "import socket; s=socket.create_connection(('10.10.0.219',5005),timeout=8); \
s.sendall(b'{\"action\": \"get_angles\"}\n'); print(s.makefile().readline())"
# TOUJOURS exécuter avant ROS2
conda deactivate

source /opt/ros/jazzy/setup.bash
source ~/ros_jazzy/install/setup.bash
```

---

## État actuel (9 septembre 2026 — soir, test des 4 directions)
## État actuel (3 juin 2026 — nuit — pick-and-place Gazebo visual debug)

### Ce qui a été accompli (session de débogage visuel Gazebo)

- **Ouverture GUI Gazebo** : modifié `pick_and_place_aruco.launch.py` pour retirer le flag `-s` (server-only)
  et activer l'affichage graphique via `DISPLAY=:1`.
- **Marqueurs ArUco texturés** :
  - Généré 5 PNG ArUco DICT_4X4_1000 (IDs 0,1,2,3,10) via OpenCV, stockés dans
    `mycobot_description/worlds/textures/` et `materials/textures/`.
  - Remplacé les cubes colorés génériques dans `precision_benchmark.sdf` par des
    dalles 10×10 cm avec texture PBR (`albedo_map`) portant les vrais patterns ArUco.
  - Cube cible rouge conservé + face supérieure avec texture ArUco ID 10.
- **Correction tremblement HOME** :
  - `HOME_ANGLES` dans `pick_and_place_aruco_node.py` : `[0,0,0,0,0,0]` → `[0,-0.8,1.4,-0.8,0,0]` rad
    (position stable au-dessus du workspace, évite l'instabilité gravitationnelle).
  - URDF `mycobot_pro_320_pi_benchmark.urdf` : `initial_value` des joints 2/3/4 mis
    à jour pour correspondre, évitant le tremblement avant la première commande.
- **Correction suivi cube pendant transport** :
  - Ajout de `_gripper_base_world()` : calcule la position de `gripper_base` en frame
    monde via la chaîne FK complète + transform fixe `joint6output_to_gripper_base`.
  - `_do_grasp()` et SETTLING-carrying utilisent maintenant `gripper_base_world` au lieu
    de `ee_pos + 0.02` (le cube ne flotte plus au-dessus du bras).
  - **Limite connue** : l'IK ne contraint pas l'orientation ; la pince pointe latéralement
    (~5 cm en Y) à la position de saisie. Fix complet = IK avec contrainte d'orientation.
- CMakeLists.txt : `materials/` ajouté à l'install list.

### Décisions prises

- Textures ArUco stockées dans `worlds/textures/` (chemin relatif direct, sans `..`) :
  Gazebo Harmonic ne résout pas `../` dans les `albedo_map` PBR.
- `HOME_ANGLES` aligné avec `initial_value` URDF pour une initialisation stable.

### Prochaines actions

1. [ROUGE] IK avec contrainte d'orientation → pince pointe vers le bas au pick/place
2. [JAUNE] Imprimer marqueurs ArUco réels + test sur banc avec caméra
3. [VERT] `ros2 launch mycobot_gateway pick_and_place_aruco_real.launch.py`

### Commande rapide de reprise (sim avec GUI)

```bash
conda deactivate
source /opt/ros/jazzy/setup.bash && source ~/ros_jazzy/install/setup.bash
export DISPLAY=:1
ros2 launch mycobot_gateway pick_and_place_aruco.launch.py
```

---

## État actuel (3 juin 2026 — soir — pick-and-place Gazebo validé)

### Ce qui a été accompli aujourd'hui

- Scaffoldé l'orchestrateur FSM `pick_and_place_aruco_node.py` (10 segments,
  modes `sim` / `real`, IK numérique, interface `/aruco/object_pose` unifiée)
- Créé les deux launch files (`pick_and_place_aruco.launch.py` et `_real.launch.py`)
- Corrigé `KeyError: 'angles'` (FSM lisait le mauvais segment via index déjà incrémenté)
- **Cycle pick-and-place Gazebo complet validé** : home → approach_pick → grasp_pos
  → GRASP (gz set_pose) → lift → approach_place → place_pos → RELEASE → retreat
  → home_end → DONE, IK 0.0 mm d'erreur sur chaque waypoint

### Décisions prises

- Interface commune `gz_sim_localizer` (sim) / `aruco_localizer` (réel) → nœud
  orchestrateur identique en sim et sur robot
- `self._current_seg` stocké avant transition MOVING pour éviter off-by-one sur l'index

### Prochaines actions

1. [ROUGE] Imprimer marqueurs ArUco (4 workspace IDs 0-3, 50 mm + 1 objet ID 10, 40 mm)
2. [JAUNE] `bash scripts/real_robot_preflight.sh` + bridge Pi actif sur 10.10.0.223
3. [VERT] `ros2 launch mycobot_gateway pick_and_place_aruco_real.launch.py`

### Commande rapide de reprise

```bash
conda deactivate
source /opt/ros/jazzy/setup.bash && source ~/ros_jazzy/install/setup.bash

# Simulation (validation déjà passée)
ros2 launch mycobot_gateway pick_and_place_aruco.launch.py

# Robot réel (prérequis ci-dessus)
ros2 launch mycobot_gateway pick_and_place_aruco_real.launch.py
```

---

## État actuel (3 juin 2026 — pick-and-place ArUco scaffoldé)

### 🧭 Prochaine action prioritaire

**Valider le pick-and-place en Gazebo :** ✅ validé (voir entrée du soir)

**Ensuite — robot réel (prérequis) :**
1. Imprimer les 5 marqueurs ArUco (4 workspace IDs 0-3 + 1 objet ID 10)
2. `bash scripts/real_robot_preflight.sh`
3. `ssh er@10.10.0.223 'python3 bridge_pi_simple.py'`
4. `ros2 launch mycobot_gateway pick_and_place_aruco_real.launch.py`

---

## État actuel (28 avril 2026 — soir)

### Ce qui a été accompli

**Le test interrompu a été mené à son terme** : 4 directions × 10 retours,
40 approches, aucun échec, bras reparqué en pose d'observation.

| direction | RP ISO 9283 | états distincts | Z du barycentre |
|---|---|---|---|
| arrière | 0,000 mm | 1 | 42,655 |
| droite | 0,186 mm | 2 | 46,619 |
| avant | 0,794 mm | 4 | 44,400 |
| gauche | 0,838 mm | 3 | 42,992 |

**Trois conclusions du rapport ont dû être corrigées** :

1. « Le robot tient sa spécification » → **faux**. Trois séries sur six
   dépassent les ±0,5 mm une fois jugées en RP ISO 9283 et non en écart max.
2. « On mesure la répétabilité du relevé codeur, quantifié » → **faux**. Le
   plancher vaut 0,099 mm ; les dispersions sont 2 à 8× au-dessus.
3. « Dégradation presque entièrement verticale » → **surestimé**. L'étalement
   se répartit X 3,06 · Y 3,15 · Z 3,96 mm.

**Le résultat le plus solide de la campagne** reste le biais de sens
d'approche : 5,918 mm ici, 5,847 mm ce matin, 5,88 mm le 20/08 — trois
protocoles indépendants à 0,07 mm près.

### Décisions prises

- Retrait du test fixé à **40 mm** et non 50 : à 50 mm le départ « avant »
  plaçait le bras à J3 = −0,58°, résidu IK 0,611 mm. On aurait mesuré la
  singularité, pas le sens d'approche.
- La répétabilité est désormais toujours donnée en **RP ISO 9283**.
- Les séries *sous* la spec ne sont plus présentées comme une validation : la
  mesure étant une borne inférieure, seules les séries *au-dessus* informent.

### Prochaines actions

1. [ROUGE] Rejouer la prise du scotch outil incliné avec `ctx.classe_objet`
   correctement armé, pour voir si `Z_PRISE_PAR_CLASSE['scotch'][1] = 35,9`
   suffit ou doit descendre vers les ~8 mm qui saisissent réellement.
2. [JAUNE] Mesurer l'affaissement à **trois allonges** pour trancher si
   `d = L × θ` est prédictif (14,8 mm mesurés à 332 mm contre 13 mm modélisés
   à 390 mm).
3. [VERT] Reprendre la répétabilité avec un moyen de mesure **externe** si l'on
   veut réellement statuer sur les ±0,5 mm.

### Commande rapide de reprise

```bash
cd /home/genji/Osama_ws/src/mycobot_R6A
python3 -c "import json,socket; s=socket.create_connection(('10.10.0.219',5005),timeout=8); \
s.sendall(b'{\"action\": \"get_angles\"}\n'); print(s.makefile().readline())"
```

---

## État actuel (9 septembre 2026 — après-midi, campagne de précision)

### Ce qui a été accompli

**Campagne de précision menée et documentée** →
[`training/calibration/PRECISION_MYCOBOT_320PI.md`](training/calibration/PRECISION_MYCOBOT_320PI.md).
Elle corrige une erreur de catégorie qui circulait dans les supports : les
**±0,5 mm d'Elephant Robotics sont une *repeated positioning precision*** — une
répétabilité — et non une précision absolue. Comparer notre erreur 3D à ce
chiffre et conclure « hors spec ×2,18 » n'avait pas de sens.

**Le robot tient sa spécification** : répétabilité **0,306 mm** sur 10 retours
et **0,424 mm** sur 6, en approche unidirectionnelle par le haut. Sous les
±0,5 mm. Réserve à toujours citer : la position vient de `FK(angles relus)`,
donc **borne inférieure** — aveugle au jeu et à la souplesse en aval des
codeurs (indice : 10 essais ne donnent que 2 valeurs distinctes).

**Le sens d'approche est le vrai sujet** : 0,424 mm en unidirectionnel contre
**5,847 mm** en mélangeant 4 directions, soit **×21,4**, et presque tout sur Z
(σZ = 3,90 mm ; retours par le haut à ~48,5 mm, latéraux à ~41 mm pour la même
consigne). Reconfirme les 5,88 mm du 20/08, indépendamment.

**La vision n'a AUCUNE erreur d'échelle.** Le −3,25 % lu sur un ArUco de 50 mm
était un artefact : un tag de **100 mm** donne **+0,005 %**, et les trois
marqueurs de planche de 50 mm donnent −1,27 / −2,81 / −5,23 % **selon leur
obliquité**. Une vraie erreur d'échelle serait identique à toutes les tailles.
Le bloc « −2,6 % → 7,4 mm » doit être retiré des supports. Deux hypothèses ont
été essayées et écartées par la mesure : échelle de chaîne, et biais constant
de localisation des coins (il prédisait −1,55 % sur le 100 mm).

**Corrigé dans `pick_fsm.py`** : un commentaire comparait un résidu de
convergence aux ±0,5 mm constructeur. Remplacé par l'explication de la
distinction.

### Décisions prises

- **Ne pas citer la répétabilité sans sa réserve** (relevé aux codeurs).
- **Retirer le bloc échelle** des diapos ; le remplacer par la mesure au tag de
  100 mm si un chiffre est nécessaire.
- **Ne pas commiter les `.xlsx`** : aucun n'est suivi dans `training/calibration/`.

### Prochaines actions

1. [ROUGE] **Terminer le test 4 directions** (10 retours depuis devant,
   derrière, gauche, droite ; retrait uniforme 50 mm). Interrompu à 12 essais
   sur 40 — compter ~30 min, la stabilisation à chaque pose est lente.
   Script : `scratchpad/test_4directions.py`.
2. [JAUNE] **Mesurer l'affaissement à 3 portées.** Le modèle `d = L × θ` des
   diapos prédit un affaissement croissant avec l'allonge, or on mesure 14,8 mm
   à 332 mm quand le modèle donne 13 mm à 390 mm. Le modèle est indicatif, pas
   prédictif.
3. [JAUNE] Rejouer le cas *outil couché* de la prise du scotch avec la classe
   correctement armée (voir entrée précédente).
4. [VERT] Remonter l'éclairage vers 86 de luminance avant toute calibration
   visant le standard de 0,12 mm.

### Commande rapide de reprise

```bash
cd ~/Osama_ws/src/mycobot_R6A
python3 /tmp/.../scratchpad/test_4directions.py   # ~30 min, 40 approches
```

---

## État précédent (9 septembre 2026 — matin, tri complet)

### Ce qui a été accompli aujourd'hui

**Tri complet réussi sur le robot réel : 3 objets, 3 destinations.** Balle jaune
au centre du grand carton (largage à 2 mm du milieu), scotch bleu et scotch
blanc dans le petit carton. Cycles pilotés par `pick_fsm` en tête-à-tête avec le
robot, sans le tableau de bord. Vérifié à la SVPRO, planche vide en fin de
séance.

**La caméra avait bougé, et l'extrinsèque du 08/09 était périmée.** Sur les
pixels du jour elle reprojetait à **6,54 px** et laissait **14,1 mm** d'erreur au
sol (contre 0,022 px / 0,119 mm la veille) ; la position caméra avait bougé de
**9,9 mm et 1,63°**. Recalibration au protocole du 08/09 (60 trames, 4 centres
ArUco, SQPNP) : **14,220 → 0,594 mm**. Sans elle la pince visait 14,9 mm à côté
du centre de la balle, soit 45 % de son rayon. Ancienne version conservée en
`arducam_extrinsic_pick.avant_0909.yaml`. **Non commitée**, comme le reste des
extrinsèques.

**Le montage caméra ne bouge pas** — vérifié, contrairement à ma première
conclusion. Trois tests concordants : propagation Monte-Carlo du bruit pixel
mesuré (jitter prédit 1,57/2,29/0,83 mm contre 1,72/2,19/0,74 observé, rapport
≈ 1), moyennes par blocs sans excès, autocorrélation lag-1 ≈ 0. Le tremblement
de 2 à 3 mm est **entièrement** du bruit de détection, amplifié par un éclairage
faible (luminance 55 contre 86 de référence). L'exposition arducam est restée à
75, le réglage du registre.

**Shepard / IDW sur les 4 marqueurs : rien à corriger.** Les résidus aux coins
ne sont pas cohérents entre eux (+0,54 / −0,59 / −0,14 / +0,18 en X), donc
l'interpolation ne rend que **0,12 à 0,18 mm** — sous la dispersion de la
détection (σ 0,5 mm). L'erreur de 14 mm était un décalage **global**, pas une
déformation locale de la zone : un recalibrage global suffit.

**Le tag ArUco du carton fonctionne** (id 10 = grand, id 11 = petit, 30/30
trames, `DICT_4X4_50`, sans prétraitement). Il n'apparaissait pas sur les
premiers scans parce que le bras stationnait au-dessus du carton. Sa hauteur de
rebord reste non mesurable (12,8 px de côté contre 30 px requis — un tag de
30 mm à 1,16 m ne peut structurellement pas y arriver, il en faudrait ~70), mais
**l'identification, elle, est exacte** — et c'est tout ce qu'on lui demande.

**Un bug de fond trouvé dans `pick_fsm`** : le repli sur les hauteurs de prise
génériques était silencieux. Voir CHANGELOG. Corrigé par une trace ; les
constantes mesurées n'ont pas été touchées.

### Décisions prises

- **Ne pas modifier `Z_PRISE_PAR_CLASSE`.** La mesure ne montre pas que ces
  valeurs sont fausses, elle montre qu'elles n'ont jamais été utilisées. Les
  écraser sur une séance serait une régression probable contre des balayages
  documentés du 26/08.
- **Ne pas agrandir le tag du carton** : l'identification seule suffit à l'usage
  visé, la hauteur de rebord n'est pas nécessaire.
- **Ne pas rapprocher le petit carton** malgré 426 mm au point de largage :
  décision explicite de tenter tel quel. Les deux largages sont passés.

### Prochaines actions

1. **[ROUGE]** Rejouer le cas *outil couché* avec la classe correctement armée.
   Avec `scotch: (18.9, 35.9)` la cible devient 25,9 mm, alors que ce qui a
   réellement saisi est 8,3 mm. Si 25,9 ferme encore à vide, **là** on aura la
   mesure qui justifie de descendre 35,9.
2. **[ROUGE]** Ne plus laisser la FSM enchaîner ses trois essais internes de
   saisie sans surveillance : les 4 fermetures à vide du 09/09 ont déplacé le
   rouleau de 19 mm.
3. **[JAUNE]** Un client TCP résiduel (`/tmp/mycobot_centroid_grasp/run_staged.py`,
   lancé depuis 18 h) tient une connexion en `CLOSE-WAIT` sur le pont. Inoffensif
   tant que le pont répond, mais premier suspect s'il se fige.
4. **[JAUNE]** Remonter l'éclairage vers une luminance de 86 avant toute
   calibration visant le standard de 0,12 mm. À 55, on plafonne à 0,59 mm — assez
   pour saisir, pas pour la référence.
5. **[VERT]** Arbre de travail très chargé : 62 fichiers modifiés, 321 non
   suivis, dont ~90 Mo de poids YOLO et de dossiers `training/`. À trier.

### Commande rapide de reprise

```bash
# Le tableau de bord tourne en Python SYSTÈME — le .venv casse Qt en xcb
cd ~/Osama_ws/src/mycobot_R6A
env -u VIRTUAL_ENV MYCOBOT_PI=10.10.0.219 /usr/bin/python3 scripts/pick_dashboard.py
```

## État actuel (2 septembre 2026 — soir, la démo markerless est invalidée)

### Ce qui a été accompli

**Le résultat markerless de l'après-midi (27,9 mm / 1,62°) est faux, et on sait
pourquoi.** Le keypoint `base` de `vgg_montage0901_ft_e30` est une **constante
par montage** : (211,7 · 342,9) sur l'arducam les deux jours, (384,5 · 333,3)
sur la svpro les deux jours, (250,0 · 256,4) sur real_3cam — écarts-types 0,00
à 0,21. Décaler l'image de 30 px déplace `base`, `link1` et `link2` de **0 %**.
La svpro, réellement déplacée de ~30 px, est vue déplacée de **0,08 px**.

Ces trois keypoints pèsent 75 des 174 correspondances de l'ajustement arducam :
la pose retrouvée **restitue celle des données d'affinage**, elle ne la mesure
pas. Circulaire, et d'autant plus flatteuse (0,45 px de résidu) que c'est
circulaire.

### Décisions prises

- **Ne pas présenter la démo markerless comme validée.** Avec ce checkpoint,
  DREAM ne s'auto-calibre pas sur une caméra déplacée ; il n'a l'air de marcher
  que là où la caméra n'a pas bougé.
- Aucun autre checkpoint ne fait mieux (`vgg_synthetic_e25` : socle à 200 px du
  vrai ; `vgg_ultimate_v4_mix_ft_e30` : biais connu, 3/10 détections svpro).
- **La cause est le manque de diversité de points de vue à l'affinage**, pas
  l'architecture ni le nombre d'epochs.
- **La validation à 800 trames ne pouvait pas le détecter** : séparation en
  espace articulaire, point de vue unique.

### Prochaines actions

1. [ROUGE] Régénérer du synthétique à **poses de caméra randomisées** avec
   `TABLE_CLEARANCE = 0,05`, refaire le mix avec une part synthétique plus grosse.
2. [ROUGE] Capturer le montage réel depuis **4-5 positions de caméra**.
3. [JAUNE] Critère d'acceptation : tenir à l'écart un **point de vue**. Test
   unitaire : décaler l'image de N px, la détection doit suivre de N px.

### Commande rapide de reprise

```bash
source ~/ros_jazzy/venv_dream/bin/activate
python scripts/dream_extrinseque_markerless.py \
    --capture training/dream/captures/markerless_0902 --cameras arducam,svpro
```

## État actuel (2 septembre 2026 — après-midi, DREAM markerless démontré)
Branche active : **`feature/calibration-cam`**. Deux Arducams calibrées (intrinsèques mesurés). Plan validé :

- **Point 2 — Régénérer les GT** du dataset `/tmp/dream_data/real_cam0/` avec les K mesurés au lieu des `fx=fy=610` codés en dur. Les fichiers JSON NDDS contiennent les `projected_location` calculées avec la mauvaise matrice. À recalculer avec FK + nouveaux `K`. Voir [`training/dream/convert_to_ndds.py`](training/dream/convert_to_ndds.py).
- **Point 3 — Ré-évaluer DREAM** sur le dataset GT-corrigé. Si la détection link4-6 monte significativement, l'écart `fx=610` était la cause majeure. Sinon → collecte v2.
- **Différé** — Calibration Astra. Logistique trop fragile aujourd'hui. À refaire en session dédiée avec board fixé au mur + Astra sur trépied.

### Ce qui a été accompli aujourd'hui (28/04 soir)

#### 1. Tooling de calibration intrinsèque

Branche `feature/calibration-cam` créée. Sous [`training/calibration/`](training/calibration/) :

- `calibrate_camera.py` — calibrateur ChArUco (UVC ou OpenNI) avec gating qualité (markers + sharpness + coverage grid + diversité temporelle), rejet d'outliers per-view-error, **auto-save** quand target atteint, **save-on-quit** fallback ≥ 12 vues, support `--source v4l2` (Arducam) et `--source astra` (via wrapper OpenNI). CLAHE optionnel (`--clahe`).
- `generate_board.py` — génère un PNG ChArUco à imprimer aux dimensions exactes (DPI configurable).
- `probe_charuco.py` — probe diagnostique single-frame (a servi à débusquer 2 régressions cv2 4.6 : `DetectorParameters()` et `CharucoBoard((sx,sy),...)` segfault — fix par fallback legacy `_create()`).
- `probe_astra.py` — probe spécifique Astra (4 modes : raw, CLAHE, swap-RB, swap-RB+CLAHE) sur 15 s.

#### 2. Calibrations mesurées

| Caméra | Vues | RMS px | fx | fy | cx | cy | Note |
|--------|------|--------|----|----|----|----|------|
| **cam_0** | 18 | **0.67** | 525.67 | 529.70 | 317.73 | 226.00 | — |
| **cam_3** | 21 | **0.68** | 496.31 | 494.14 | 313.37 | 248.01 | premier essai cy=42 archivé en `cam_3.bad.*` |

Outputs : `training/calibration/cam_{0,3}.{npz,meta.json,snapshot.png}`.

#### 3. Comparaison avec le dataset DREAM — **finding majeur**

| Param | Dataset existant | cam_0 mesuré | cam_3 mesuré | Écart |
|-------|------------------|--------------|---------------|-------|
| fx | 610 | 525.67 | 496.31 | **−13.8 % (cam_0)** |
| fy | 610 | 529.70 | 494.14 | **−13.2 %** |
| cx | 320 | 317.73 | 313.37 | −0.7 % |
| cy | 240 | 226.00 | 248.01 | **−5.8 %** |

**Implication** : les `projected_location` GT du dataset `real_cam0` ont été calculées avec un `fx=610` qui ne correspond à AUCUNE caméra physique. Pour un point 3D à distance D, le pixel projeté est **faux d'un facteur ~14 %**. Cette erreur croît avec la distance au centre image → cohérent avec les link4-6 (loin du centre quand le bras est étendu) à 3-36 % de détection en 1.12.0. Le réseau a entraîné sur des **GT erronés** sur les distal — il ne peut pas converger sur les bonnes positions.

**Probablement la cause majeure** du gap distal. Si la régénération GT débloque ça, pas besoin de capturer un nouveau dataset.

#### 4. Calibration Astra — différée

Tentatives multiples sans succès :
- 640×480 RGB888 standard : 5-6 markers détectés sur 27 → trop peu pour `interpolateCornersCharuco`
- 1280×720 RGB888 @30 fps : USB 2.0 saturé (663 Mbps > 480 disponibles) → image corrompue (rayures multicolores)
- 1280×720 RGB888 @15 fps : refusé par le firmware Astra (seulement @30 fps listé pour cette résolution)
- 1280×720 GRAY8 @30 fps : démarre, mais grabber freeze sans frames après quelques secondes
- `findChessboardCorners` + `findChessboardCornersSB` : 0 corners (capteur Astra trop "soft" pour le damier ou board hors champ pendant que l'utilisateur tape au clavier)

Custom HD grabber compilé dans `/tmp/oni_grabber_hd.cpp` (non committed). À reprendre avec setup physique stable (board mural fixe, Astra sur trépied).

### Prochaines actions

1. **🔴 Régénérer les GT** du dataset `real_cam0` avec K cam_0 mesurés. Recalculer `projected_location` via FK + `cv2.projectPoints(..., K, dist)`.
2. **🔴 Ré-évaluer DREAM** sur GT-corrigé avec checkpoint e50. Comparer link4-6 detection.
3. **🟡 Si gap subsiste** : retrain mixte v2 sur dataset corrigé (×5 oversample + 8K synth).
4. **🟢 Plus tard** : calibration Astra avec setup stable.

---

## État précédent (28 avril 2026 — après-midi)

### 🧭 Reprise pour la prochaine session — lire en premier

Test cheap d'ajout de cam3 dans le mix terminé. Le retrain v2 (`vgg_mixed_v2_cam03`, 25 epochs sur 18K = 6K cam0 ×3 + 6K cam3 ×3 + 6K synth) a délivré son signal :

- ✅ **cam3 a appris** (link1/2 passent de 1.6 % à 100 % détection avec 2.75 px médiane)
- ⚠️ **cam0 a régressé** (47.3 % → 40.2 %, distal effondrés)
- 🟰 **Bilan net** : on échange perf cam0 contre perf cam3 sans gain global

**Conclusion** : les images cam3 contiennent l'info utile, mais les **extrinsèques approximatives** (`xyz=(0, 0.5, 0.3)`, `rpy=(0, 0.2, -π/2)`) sont effectivement load-bearing — elles introduisent du bruit GT qui dégrade les distal partout.

**Décision actée** : passer en chemin (A) propre = **calibrer cam3** (chessboard OpenCV pour intrinsèques + extrinsèques mesurées physiquement ou par PnP sur le checkpoint v1) **avant** le retrain v3.

En parallèle, l'option 2 d'origine (collecte de poses bras étendu sur cam0) reste valide mais devient secondaire — la valeur marginale de plus de cam0 est moindre qu'une 2ᵉ caméra exploitable.

Avant de calibrer, plan v3 détaillé dans CHANGELOG 1.13.0 § "Décision pour la prochaine session".

Commande rapide pour reproduire l'éval (résultats attendus dans le tableau ci-dessous) :
```bash
source ~/ros_jazzy/venv_dream/bin/activate
# (a) strict réel
python training/dream/evaluate_dream.py \
  --weights training/checkpoints_dream/vgg_mixed_real_synth/best_network.pth \
  --data /tmp/dream_data/real_cam0 --split all
# (b) strict synth val
python training/dream/evaluate_dream.py \
  --weights training/checkpoints_dream/vgg_mixed_real_synth/best_network.pth \
  --data /tmp/dream_data/synthetic --split val --max-samples 1000
# (c) relaxed réel
python training/dream/evaluate_dream_relaxed.py \
  --weights training/checkpoints_dream/vgg_mixed_real_synth/best_network.pth \
  --data /tmp/dream_data/real_cam0 --split all \
  --peak-thresh 0.001 --next-best-score 0.05
```

### Ce qui a été accompli aujourd'hui (28/04/2026 — après-midi)

#### 1. Test cheap : ajout cam3 dans le mix sans calibration

- Génération `/tmp/dream_data/real_cam3` (2000 frames NDDS, extrinsèques approximatives existantes).
- **Eval croisée préalable** du checkpoint v1 sur cam3 : 25.1 % détection, OVERALL **237 px** d'erreur — confirme que le modèle n'a aucune cross-view generalization.
- Build `mixed_v2_cam03` (18K = 6K cam0 ×3 + 6K cam3 ×3 + 6K synth, symlinks).
- Retrain DREAM natif 25 epochs (2h35 sur RTX 4000 Ada). Val loss 0.000356 (vs v1 e25 0.000334 — légèrement plus haute, cohérent avec annotations cam3 bruitées).
- 3 évals finales :

| Eval | v1 e50 | **v2 e25** | Δ |
|------|--------|------------|---|
| cam0 strict | 47.3 % / 2.78 px | **40.2 %** / 2.77 px | -7.1 pts ⚠️ |
| cam3 strict | 25.1 % / 237 px | **35.1 %** / 2.75 px (proximaux) | +10 pts, erreur ÷22 ✅ |
| synth val | 91.9 % / 2.72 px | **93.1 %** / 2.93 px | +1.2 pts ✅ |

#### 2. Verdict du test cheap

- ✅ Le modèle apprend cam3 (link1/2 à 100 % @ 2.75 px) — **les images cam3 contiennent l'info utile**.
- ⚠️ Mais cam0 régresse de 7 pts sur les distal (link3 -16.8, link4 -32.6, link5/link6 effondrés). Les extrinsèques cam3 approximatives propagent du bruit qui dégrade les distal partout.
- 🟰 **Pas de gain net** : trade-off perf cam0 ↔ perf cam3. La calibration propre devient nécessaire avant le retrain v3.

### Ce qui a été accompli ce matin (28/04/2026 — matin)

#### 1. Dépendances `venv_dream` complétées

- `pandas 3.0.2` ajouté (manquant pour `evaluate_dream.py`). Le reste (cv2, ruamel.yaml, tqdm, albumentations, torch+cu124, PyYAML, typeguard, PIL) déjà en place — vérifié au démarrage.

#### 2. DREAM — évaluation finale (3 passes) du checkpoint mixte e50

| Eval | Dataset | Split | Frames | Det rate | OVERALL méd. | base / link6 det% | link6 médiane |
|------|---------|-------|--------|----------|--------------|--------------------|----------------|
| **(a) strict** | `real_cam0` | all | 500/2000 | **47.3 %** | 2.78 px | 0 % / 3.0 % | 61.6 px |
| **(b) strict** | `synthetic` | val | 1000/4000 | **91.9 %** | 2.72 px | 99.9 % / 73.2 % | 18.59 px |
| **(c) relaxed** (peak=0.001) | `real_cam0` | all | 500/2000 | **48.0 %** | 2.78 px | 28 % (mais 328 px !) / 8.6 % | 170.7 px |

Logs : `/tmp/eval_a_real_strict.log`, `/tmp/eval_b_synth_val.log`, `/tmp/eval_c_real_relaxed.log`.

#### 3. Comparaison synth-only ↔ mixte

| Métrique | `vgg_weighted_50k_e50` (synth-only) | **`vgg_mixed_real_synth_e50`** | Δ |
|----------|--------------------------------------|--------------------------------|---|
| Détection synth val | 98.3 % | 91.9 % | -6.4 pts (régression contrôlée) |
| Détection réel all | 26.0 % | **47.3 %** | **+21.3 pts** |
| link6 sur réel (det / méd.) | 5.6 % @ 395 px | 3.0 % @ 61.6 px | détection ≈ identique, **erreur ÷6** |

#### 4. Verdict

- **Le mix-training a fonctionné** : +21 pts sur réel sans détruire le synth. Le modèle a appris des features réelles, pas écrasé celles de la simulation.
- **Le relaxed thresholding ne débloque rien** : +0.7 pt mais médianes catastrophiques (base 328 px, link6 170 px). Confirmation définitive — les peaks low-conf sont du bruit, pas des bonnes prédictions cachées par un filtre trop strict.
- **Bottleneck identifié** : *distal keypoints (link4–link6)*. Sur synth val déjà link6 n'est qu'à 73.2 %. Sur réel ça s'effondre à 3 %. Le modèle peine partout sur les distal mais c'est dramatique sur réel.
- Conclusion = ce qui était prescrit en 1.11.0 : enrichir le réel avec poses bras étendu.

### Ce qui a été accompli avant (récap)

#### 23/04/2026 — soir

> ⚠️ **Section historique conservée pour traçabilité.** Le diagnostic DREAM décrit ci-dessous est complété par les 3 évals du 28/04 (au-dessus).

### Ce qui a été accompli aujourd'hui (23/04/2026)

### Ce qui a été accompli

**La démonstration markerless est faite, sur l'arducam : 27,9 mm et 1,62°.**
Le robot sert de mire (FK des encodeurs en 3D, DREAM en 2D, PnP empilé sur
25 poses), aucun ArUco n'entre dans le calcul — ils ne servent que de juge.
Résidu 0,95 px, détection 7,0/7 de moyenne, 24 poses sur 25 à 7/7. Le résultat
ne dépend pas du choix des keypoints : en n'ajustant que sur link3→link6 on
retombe à 25,4 mm / 1,21°. Écrit dans
`training/calibration/arducam_extrinsic_markerless.yaml`.

**Pourquoi plusieurs poses.** Sur une seule configuration du bras les 7
keypoints sont quasi coplanaires et la rotation admet plusieurs branches qui
reprojettent aussi bien — 27 à 30° d'erreur mesurés en juillet. Le volume
balayé par une vingtaine de configurations lève l'ambiguïté.

**La SVPRO échoue (54,9 mm / 7,16°), et la cause est mesurée : elle a bougé
d'environ 30 px et 2,9° depuis l'affinage du 01/09.** Trois mesures
indépendantes du réseau, avec l'arducam en témoin :

| Mesure | SVPRO | Arducam (témoin) |
|---|---|---|
| Coins ArUco 01/09 → 02/09 | 24 à 37 px | 0,4 à 1,2 px |
| 467 appariements ORB, toutes couronnes | 30 à 43 px, centre optique compris | 1,0 à 1,2 px |
| Similitude ajustée | échelle 0,98 · rotation +2,90° | 1,0007 · −0,16° |
| Corrélation du patch socle | (+30, +9) px, corrélation 0,94 | — |

Un déplacement uniforme à **tous** les rayons, centre optique inclus, signe un
mouvement rigide de la caméra — une respiration de mise au point serait nulle
au centre. Ceci **valide** l'extrinsèque à 4 marqueurs du 02/09 : c'est elle
qui décrit la caméra d'aujourd'hui.

**Ce que le réseau fait alors.** DREAM place le socle SVPRO à (384,5 · 333,3),
le **même pixel** les deux jours à 0,01 px près, alors que le socle a réellement
bougé de (+30, +9) px. Il reproduit le cadrage sur lequel il a été affiné au
lieu de suivre l'image. L'écart DREAM ↔ marqueurs d'aujourd'hui, (+28,1 · +6,9)
px, vaut exactement ce déplacement.

### Décisions prises

- **Réponse à « DREAM calibre tout seul même si je déplace la caméra » : oui
  pour la pose, non hors domaine d'entraînement.** Il retrouve la caméra sans
  marqueur à 28 mm près, mais seulement pour un point de vue déjà vu. C'est le
  même mode d'échec hors-domaine que le biais de 52 px de l'arducam, et le même
  levier : réaffiner en incluant le nouveau cadrage.
- **La branche SVPRO du dashboard multicam n'est pas fiable** avec
  `vgg_montage0901_ft_e30` tant que la caméra reste où elle est. L'arducam l'est.
- `scripts/capture_markerless.py` n'introduit **aucune** nouvelle règle de
  sécurité : il importe telles quelles celles de `capture_trajectoires.py`, qui
  n'est pas modifié. Il ajoute en revanche un contrôle de **trajet** — `pose_sure`
  protège une pose immobile, pas le chemin qui y mène ; 3 segments sur 24
  passaient sous la garde avant ce contrôle.
- Visibilité exigée sur **les deux** caméras, pas seulement l'arducam.

### Prochaines actions

1. [ROUGE] Décider du sort de la SVPRO : la refixer et refaire l'extrinsèque à
   chaque déplacement, ou la réintégrer dans un prochain fine-tune pour que la
   markerless y marche aussi.
2. [JAUNE] Rejouer le dashboard multicam avec `model_name:=vgg_montage0901_ft_e30`
   et constater la dégradation SVPRO annoncée.
3. [VERT] Régénérer `docs/METHODOLOGIE_CAPTURE_FINETUNE.docx` (local, jamais commité).

### Commande rapide de reprise

```bash
source ~/ros_jazzy/venv_dream/bin/activate
python scripts/capture_markerless.py --simuler          # plan, sans bouger le bras
python scripts/dream_extrinseque_markerless.py \
    --capture training/dream/captures/markerless_0902 --cameras arducam
```

## État actuel (2 septembre 2026 — matin, le biais DREAM est corrigé)

### Ce qui a été accompli

**Le biais DREAM de 52 px est résolu.** Fine-tune `vgg_montage0901_ft_e30`
(30 epochs, 6 h 43, meilleure epoch 29), évalué sur 800 trames tenues à l'écart :
**53,82 px → 1,81 px** de médiane, détection **54,8 % → 100 %** sur les 7
keypoints, 0,3 % → 99,9 % sous 10 px. **Sans régression** : `real_3cam` reste à
2,32 px de médiane, au centième près.

**La cause est établie et écarte l'autre hypothèse.** Ce n'était pas un montage
de caméra différent : c'était de l'**extrapolation hors domaine**.
`synthetic_data_collector_v3.py:505` impose `TABLE_CLEARANCE = 0.13` m, le pick
travaille entre 72,5 et 114,5 mm — 100 % des poses réelles étaient rejetées à la
génération. D'où le mur sur J2 à −103,5° dans `synthetic_50k`, que le filtre
rejoué hors ligne reproduit exactement (±103,6°).

**La SVPRO est réparée sur deux plans.** Focus : rien dans le dépôt ne le fixait
(`pick_dashboard.py` ne mentionne jamais le mot, `camera_registry.py` déclare
`manual_focus=-1` sans que personne ne le lise), et le défaut du capteur est
l'autofocus continu — d'où `svpro_verrou_focus.sh`, à rejouer après chaque
rebranchement, **et après** l'ouverture du flux. Extrinsèque : refaite sur les
16 coins des 4 marqueurs (le 25 était devenu décodable), validation en laissant
un marqueur dehors 2,1-3,3 px contre 3,3-11,4 px à 3 marqueurs.

### Décisions prises

- **DREAM sert au markerless**, les 4 ArUco ne sont que le juge indépendant.
  La hauteur du bras n'est donc pas une exigence en soi — ce qui compte est que
  DREAM détecte bien. Le fine-tune l'assure désormais dans la zone du pick.
- **La régénération synthétique à garde basse n'est plus nécessaire.** Le
  collecteur (`synthetic_data_collector_v3_garde_basse.py`) est écrit et
  compilé, gardé au cas où.
- Le test tenu à l'écart se découpe en **bloc contigu de 400 poses**, pas 150 :
  la trajectoire repasse sur ses pas et à 150 un cinquième du test avait sa
  jumelle dans le train.

### Prochaines actions

1. **[ROUGE] J5 et J6 ne répondent plus** aux commandes, `power_on` compris
   (J6 : 0,0° de déplacement pour +20° commandés). J5 est à 91° et J6 à −170°,
   très hors de leur plage de travail (J5 ∈ [−41, +23], J6 ∈ [−10, +92]), donc
   la pince pointe à l'horizontale au lieu de 7-12° de la verticale. Le pont
   n'expose que `power_on` / `power_off` — reprise servo par servo impossible.
   Vérification physique puis cycle d'alimentation.
2. **[JAUNE] Démo markerless** : `capture_poses_hautes.py` est prêt (130 poses
   autour de 3 poses validées, pince vers le bas, bras 143-162 mm), bloqué sur
   le poignet. Puis `self_calibrate_arducam.py --compare-extrinsic` contre
   `svpro_extrinsic_4marqueurs.yaml`, référence du jour, même session.
3. **[VERT] Rejouer `fk_vs_dream_series.py --balayage`** avec le nouveau
   checkpoint pour confirmer en direct, sur le robot, les 1,81 px hors ligne.

### Commande rapide de reprise

```bash
bash scripts/svpro_verrou_focus.sh          # apres tout rebranchement
source ~/ros_jazzy/venv_dream/bin/activate
cd training/dream && python3 evaluate_dream.py \
  -w checkpoints_dream/vgg_montage0901_ft_e30/best_network.pth \
  -d dream_data/mix_montage0901_test --split all -n 800
```

---

## État actuel (1er septembre 2026 — soir, verdict sur DREAM au pick)

### Le résultat

**Le biais DREAM n'est corrigeable par aucune transformation 2D globale.**
75 correspondances / 13 poses : translation 52 mm, similitude 42 mm, affine
complète **36 mm**. Le décalage n'est pas constant (`dx` suit J1 : +8 px à
J1=0° → +53 px à J1=60°). Dispersion par pose 20-24 px ⇒ même systématique
entièrement retiré, le bruit propre de DREAM vaut **~40 mm**.

Échelle ajustée **0,845** : DREAM voit le robot 15 % plus petit. Cohérent avec
un affinage sur un autre montage (`real_3cam`).

**La fusion 2 caméras ne sauve pas** : SVPRO 3/7 détectés, 117-207 px d'erreur
contre 52-98 px pour l'arducam. Le seuil de fusion étant à 15 px, elle recevrait
un poids nul et la fusion retomberait sur « MONO via arducam ». C'est de la
robustesse, pas de la correction de biais.

### Décision à prendre

**En l'état DREAM n'est pas utilisable pour le pick sur ce banc** (~40 mm).
Deux voies :
1. Réaffiner le réseau sur CE montage d'arducam (la seule qui rende le
   markerless viable).
2. Acter que l'extrinsèque marqueurs reste la référence du pick, et cantonner
   DREAM à la validation d'angles du dashboard.

### Prochaines actions

1. [ROUGE] Trancher entre 1 et 2 ci-dessus. Si 1 : capture d'un jeu réel sur le
   montage actuel, puis mix-fine-tune comme `vgg_ultimate_v4_mix_ft_e30`.
2. [JAUNE] Poses de travail : garder le bras dans x ∈ [80, 560]. J1 ≈ 42° donne
   7/7, J1 ≈ 88° donne 1/7. `--balayage` écarte les poses aveugles avant de
   bouger.
3. [VERT] Corriger `convert_to_ndds.py:102` (`arducam → cam_0`, or c'est `cam_3`).

### Commande rapide de reprise

```bash
source ~/ros_jazzy/venv_dream/bin/activate
ln -sfn /home/genji/DREAM /tmp/DREAM
python3 scripts/fk_vs_dream_diagnostic.py       # pose courante, bras immobile
python3 scripts/fk_vs_dream_series.py --balayage # LE BRAS BOUGE
```

---

## État actuel (1er septembre 2026 — le biais DREAM est chiffré)

### Ce qui a été accompli

Le bras a été bougé sur les poses de travail, FK et extrinsèques déjà validées
la veille. **Le biais DREAM est mesuré : décalage systématique de ~52 px**,
soit ~11 cm sur la planche. Une translation `(+31,6, −41,2)` fait tomber le RMS
de 56,5 à 22,0 px ; échelle et rotation n'apportent que 4,7 px de plus.

**Cause la plus probable** : affinage sur `real_3cam`, où l'arducam était sur un
autre montage (`convert_to_ndds.py:87-92`) → a priori de point de vue appris.
Explique aussi la caméra placée ~1 m à côté par la self-cal : le PnP absorbe le
décalage uniforme en une translation.

**Fenêtre réseau** : `shrink-and-crop` 640×480 → 400×400 ne garde que
**x ∈ [80, 560]**. Bras à gauche (J1 ≈ 88°) → bride hors champ → 1/7.

### Hypothèses testées et ÉCARTÉES

| hypothèse | mesure | verdict |
|---|---|---|
| erreur de FK | vert collé au bras, 2 caméras, 2 poses | écartée (31/08) |
| exposition arducam | balayage 20→300 : 0 à 4/7, aucun optimum | écartée |
| recadrage réseau | 7/7 keypoints dans la fenêtre, décalage 51,2 px quand même | écartée |
| décalage en repère lien | l'écart ne tourne pas avec J1 (12°→58°) | écartée |

Alternative **non exclue** : décalage constant en XY monde — indiscernable d'un
décalage image sous caméra zénithale. La SVPRO trancherait mais ne détecte que
0-3/7.

### Prochaines actions

1. [ROUGE] Décider : réentraîner/affiner sur le montage actuel, ou acter que
   l'extrinsèque marqueurs reste la référence pour le pick. **En l'état, la
   self-calibration markerless n'est pas utilisable en production** — 11 cm.
2. [JAUNE] Poses de travail : garder le bras dans x ∈ [80, 560]. J1 ≈ 42° donne
   7/7, J1 ≈ 88° donne 1/7.
3. [VERT] Corriger `convert_to_ndds.py:102` (`arducam → cam_0`, or c'est `cam_3`).

### Commande rapide de reprise
| Session | Tâche | Statut |
|---------|-------|--------|
| 26/03/2026 | Bridge TCP Tour ↔ Pi, GUI, RViz | ✅ |
| 31/03/2026 | Simulation Gazebo Harmonic 4 caméras | ✅ |
| 31/03/2026 | Collecte 5000 poses synthétiques × 4 vues (20K images) | ✅ |
| 31/03/2026 | Domain randomization (éclairage, matériaux) | ✅ |
| 31/03/2026 | Training multi-view ResNet50 → 12.97° MAE | ✅ |
| 01/04/2026 | Camera server Pi (cam0+cam3, TCP:5006) | ✅ |
| 02/04/2026 | Capture 2000 poses réelles (0 collisions) | ✅ |
| 02/04/2026 | FK safety capture (protection table+câbles) | ✅ |
| 02/04/2026 | Training régression directe sur données réelles | ❌ Bloqué à 32.76° baseline |
| 02/04/2026 | Diagnostic : corrélation pose/pixel = 0.004 | ✅ Cause identifiée |
| 03/04/2026 | Intégration DREAM (NVlabs) + FK 7 keypoints | ✅ |
| 03/04/2026 | Conversion 20K frames NDDS | ✅ |
| 03/04/2026 | Training VGG-base (25 époques) | ✅ val=0.000438 |
| 03/04/2026 | Training VGG-aug (25 époques) | ✅ val=0.000667 |
| 03/04/2026 | Évaluation synthétique : 97% détection, 3.1px | ✅ |
| 03/04/2026 | Test sim-to-real : ~26% détection | ⚠️ Domain gap |
| 15/04/2026 | Intégration gripper adaptatif (pro_adaptive_gripper) | ✅ |
| 15/04/2026 | Correction mesh link6 → link6_2022.dae | ✅ |
| 15/04/2026 | Limites articulaires corrigées (URDF officiel) | ✅ |
| 15/04/2026 | Anti-collision FK dans collecteur (rejet ~35% poses) | ✅ |
| 15/04/2026 | Training VGG 50K synth (98.3% det synth, 13.2% réel) | ✅ |
| 15/04/2026 | Fine-tune custom v1 (σ=4, 0% det) | ❌ Bug sigma |
| 16/04/2026 | Fine-tune custom v2 (σ=2, 0% det) | ❌ Belief maps effondrées |
| 16/04/2026 | Script merge_and_convert.py | ✅ |
| 16/04/2026 | Script train_pipeline.sh + monitor_collection.sh | ✅ |
| 16/04/2026 | Monde Gazebo v2 (randomized_v2.sdf — 6 lights, 12 objets) | ✅ |
| 16/04/2026 | Collecte 7500 poses × 4 vues (30K images) synth v2 | 🔄 À vérifier |
| 16/04/2026 | Training mixte natif (18K frames) — epoch 1: val=0.000474 | ✅ Terminé |
| 23/04/2026 | **Pick-and-place multi-objets par couleur** (4 objets → 4 bacs) | ✅ End-to-end vérifié |
| 23/04/2026 | `color_object_detector` (HSV + back-projection top camera) | ✅ 4/4 couleurs détectées |
| 23/04/2026 | `sorting_orchestrator` (boucle sur détections, gz `set_pose` carry) | ✅ Cycle complet ~95 s |
| 23/04/2026 | URDF caméras reshapées (corps + objectif + LED, plus de cubes 3 cm colorés) | ✅ |
| 28/04/2026 | Install `pandas` dans `venv_dream` | ✅ |
| 28/04/2026 | DREAM eval (a) strict réel — 47.3 % det confirmé | ✅ Baseline 1.11.0 reproduit |
| 28/04/2026 | DREAM eval (b) strict synth val — 91.9 % det | ✅ Régression contrôlée vs synth-only |
| 28/04/2026 | DREAM eval (c) relaxed réel (peak=0.001) — 48.0 % | ❌ Médianes explosées, hypothèse réfutée |
| 28/04/2026 | Verdict diagnostic complet : distal keypoints = bottleneck | ✅ Cf. CHANGELOG 1.12.0 |
| 28/04/2026 (PM) | Convert cam3 → NDDS (extrinsèques approximatives) | ✅ 2000 frames |
| 28/04/2026 (PM) | Eval croisée v1 sur cam3 : 25.1 % / 237 px d'erreur | ✅ Confirme zéro cross-view generalization |
| 28/04/2026 (PM) | Build `mixed_v2_cam03` (18K) + retrain 25 epochs | ✅ 2h35, val=0.000356 |
| 28/04/2026 (PM) | Eval v2 : cam0 -7.1 pts, cam3 +10 pts, synth +1.2 pts | 🟰 Trade-off, calibration cam3 nécessaire |

```bash
source ~/ros_jazzy/venv_dream/bin/activate
ln -sfn /home/genji/DREAM /tmp/DREAM
python3 scripts/fk_vs_dream_diagnostic.py     # pose courante, bras immobile
python3 scripts/fk_vs_dream_series.py --n 4   # LE BRAS BOUGE
```

---

## État actuel (31 août 2026 — nuit, FK validée contre DREAM)

### Ce qui a été accompli

**La question « FK fausse ou DREAM biaisé ? » est tranchée : c'est DREAM.**
`scripts/fk_vs_dream_diagnostic.py` projette le squelette FK à travers les
extrinsèques **marqueurs** (indépendantes de DREAM) sur les deux caméras et le
compare aux détections. Le vert épouse le bras dans les deux vues → FK et
extrinsèques validées. Figure : `docs/fk_vs_dream.png`.

| caméra | extrinsèque | détectés | écart médian FK↔DREAM | `base` (point fixe) |
|---|---|---|---|---|
| arducam | `arducam_extrinsic_pick` (20/08) | 3/7 | 73 px | 57 px |
| svpro | `svpro_extrinsic_servo` (24/08) | 2/7 | 132 px | 62 px |

Le `base` est le juge de paix : sa projection ne dépend d'aucun angle.

### Décisions prises

- **Aucun marqueur ajouté sur le robot.** Un tag sur la bride aurait départagé
  FK et DREAM, mais contredisait l'objectif markerless — et s'est avéré inutile :
  les extrinsèques marqueurs déjà calculées suffisent comme référence extérieure.
- **`arducam_extrinsic_dream_v4.yaml` est écarté** (mauvais K `cam_0`, montage
  différent). Son désaccord de 1 m n'est plus une question ouverte.
- Le diagnostic ne bouge **jamais** le bras.

### Prochaines actions

1. [ROUGE] Refaire le tableau sur des **poses favorables** (bras au-dessus de la
   planche, outil incliné vers le bas comme aux points de travail). **Le bras
   bougera** : dégager la zone, `bash scripts/real_robot_preflight.sh` d'abord.
   C'est là que le vrai biais DREAM devient mesurable.
2. [JAUNE] Corriger `convert_to_ndds.py:102` (`arducam → cam_0`) ou documenter
   qu'il vise l'ancien montage, pour que les scripts d'entraînement ne
   retombent pas dans le piège.
3. [VERT] Décider quoi faire de `link1`/`link2` confondus dans le schéma à 7
   keypoints.

### Commande rapide de reprise

```bash
source ~/ros_jazzy/venv_dream/bin/activate
ln -sfn /home/genji/DREAM /tmp/DREAM          # /tmp est vidé au redémarrage
python3 scripts/fk_vs_dream_diagnostic.py     # bras immobile, capture live
# rejouer hors ligne, robot éteint :
python3 scripts/fk_vs_dream_diagnostic.py --brut \
    --angles 82.7,-122.6,88.76,-61.08,10.89,6.5
```

---

## État actuel (31 août 2026 — soir, self-calibration markerless)

### Ce qui a été accompli

**`scripts/pick_and_place_live_dashboard.py`** — le tableau de bord de pick avec
une extrinsèque **refaite au lancement, sans marqueur** (robot comme mire :
FK des encodeurs ↔ keypoints DREAM). Il importe `pick_dashboard` et remplace
`Vision` à l'exécution ; **`pick_dashboard.py` n'est jamais modifié**.

**Bibliothèque DREAM restaurée** dans `/home/genji/DREAM` avec `/tmp/DREAM` en
lien symbolique. Elle avait disparu — `/tmp` est vidé au redémarrage, et 10
scripts codent ce chemin en dur. Après un reboot : `ln -sfn /home/genji/DREAM /tmp/DREAM`.

### Trois causes de mauvaise détection, isolées et mesurées

| cause | effet | correction |
|---|---|---|
| tampon V4L2 non vidé | 7/7 → 2/7 (trame floue prise pendant le mouvement) | jeter 6 trames |
| bras hors de la planche | 7/7 → 0-1/7 (fond tapis + trépied) | poses au-dessus du bois |
| pince à l'horizontale (`J5=90`) | détection erratique | poses dérivées du travail, pince à 7-12° de la verticale |

Après correction : **4/4 keypoints sur 6 poses/8, 29 correspondances** (contre 10).

L'orientation d'outil se mesure sur **−X de la bride**, pas +Z : `pick` est à
7,2° de la verticale, `pick_approach` à 11,7°, `place` à 15,2°. Mes poses à
`J5=90` étaient à 90° — l'outil couché, orientation jamais prise en travail.

### Ce qui bloque

Résidu 10,97 px, **`link3` à 23,6 px sur 6 points**. Avec de bonnes détections
partout, aucune position de caméra n'explique les quatre keypoints : désaccord
**systématique**, donc problème de MODÈLE et non de capture. Trois pistes à
départager par la mesure : FK qui ne colle pas au robot réel, biais DREAM propre
à `link3`, intrinsèque `cam_3`.

### Décisions et faits durs

- **`link1` et `link2` sont le même point 3D** — 400 poses, écart max 0,000 mm.
  Les utiliser tous les deux compte deux fois la même mesure.
- **Une self-cal markerless ne peut pas se valider elle-même.** `dream_v4` place
  la caméra à 1,572 m ; markers et hand-eye disent 1,095 et 1,118 m et
  s'accordent entre eux à 2,3 cm. Non tranché.
- La SVPRO a été **écartée** : bonne géométrie de côté, mais DREAM n'y détecte
  que 0-2/7 — vue hors distribution d'entraînement.

### Prochaines actions

1. [ROUGE] Départager l'origine des 23,6 px de `link3` : reprojection FK contre
   détection, sur quelques poses.
2. [JAUNE] Une vérification ArUco ponctuelle, comme référence indépendante — le
   markerless ne peut pas dire qui a raison entre `dream_v4` et les marqueurs.
3. [VERT] Plus de poses (15-20) une fois la cause du résidu comprise.

### Commande rapide de reprise

```bash
ln -sfn /home/genji/DREAM /tmp/DREAM          # apres un reboot
source ~/ros_jazzy/venv_dream/bin/activate
cd ~/Osama_ws/src/mycobot_R6A
python scripts/pick_and_place_live_dashboard.py --calib-only --move   # LE BRAS BOUGE
```

---

## État actuel (31 août 2026 — simulation, saisie physique)

### Ce qui a été accompli aujourd'hui

**Les quatre objets triés par saisie physique dans Gazebo, 4/4.** Plus aucune
téléportation : le bras passe par le JTC `mycobot_controller`, la pince par
`gripper_position_controller`, et chaque prise est vérifiée sur la pose Gazebo
de l'objet (il monte avec les doigts). Nouveau nœud
`mycobot_gateway/mycobot_gateway/sim_sorting_grasp.py`.

| objet | serrage | bac | écart au centre |
|---|---|---|---|
| red_cube (40 mm) | 36 mm / 0,823 rad | red_bin | −1 / +0 mm |
| blue_cube (50 mm) | 46 mm / 0,740 rad | blue_bin | −11 / +10 mm |
| green_cylinder (⌀44) | 38 mm / 0,807 rad | green_bin | +1 / +2 mm |
| yellow_box (30 mm pincés) | 26 mm / 0,905 rad | yellow_bin | −13 / +4 mm |

Ouverture utile d'un bac : 95 mm. Les quatre objets finissent **à plat au fond**
(0,0° d'inclinaison, z au millimètre du fond). Cycle complet en **115 s**.

### Trois défauts corrigés dans la foulée

**Les barres extérieures de la pince étaient soudées à la bride.**
`gripper_left2` / `gripper_right2` en `fixed` : immobiles pendant que le doigt
tournait, elles partaient à l'horizontale — la pince paraissait cassée sur les
côtés. Ce sont les barres d'un quadrilatère articulé, parallèles à **0,0° près**
à la bielle motrice : parallélogramme, donc même angle que le servo. Passées en
`revolute` et pilotées, le bout reste à 1,0–1,3 mm du doigt sur toute la course.
Le contrôleur attend maintenant **6** valeurs (téléop mis à jour).

**L'objet était lâché au-dessus du bac.** Il tombait de 15 à 25 mm, rebondissait
sur la paroi et restait couché sur le rebord (cube bleu à 44,6°). Or la collision
doigts/bac demande **deux** conditions à la fois — sous le rebord (30 mm) ET plus
écarté que la paroi (±47,5 mm) — et refermés les doigts ne font que ±34 à ±44 mm.
Ils peuvent donc descendre au fond. Lâcher en deux temps : largeur exacte de
l'objet (force nulle, il repose déjà), remontée, puis ouverture complète. Marge
la plus faible : 1,5 mm sur le cube bleu.

**Le cycle attendait une durée fixe après chaque mouvement** (4 s + 2,5 s × 8
mouvements par objet). `move_to` dimensionne la durée sur le trajet et rend la
main dès que l'écart passe sous 0,35° ; l'IK est passée de 150 à 60 itérations
et s'arrête au premier résultat franc.

### Décisions prises

**Le point outil est le centre des PATINS, pas le bout des doigts.** Viser
l'extrémité décale la consigne de 15 mm sur l'axe Z de la pince, la largeur
d'un patin exactement. Le cube de 40 mm rattrapait l'erreur par sa largeur, le
cylindre de 44 mm non — les quatre joints atteignaient alors la consigne au
millième, preuve qu'ils ne touchaient rien.
`TOOL_OFFSET = (-0.001, +0.0078, 0.166)` m dans link6.

**φ figé sur toute une colonne de poses.** Résoudre l'IK indépendamment à
chaque hauteur laissait le poignet tourner entre la saisie et la levée : les
objets se dévissaient des doigts. `solve_column` impose une seule orientation à
survol + saisie + levée. C'est ce qui a fait passer le cube rouge de raté à
saisi, sans rien changer d'autre.

**Le bac vert n'est atteignable que par-dessus l'épaule** (J1 ≈ −35°, J3 > 0,
J5 < 0). L'outil sort à ~22° d'azimut de J1, donc l'azimut 164,7° du bac vert
demanderait J1 ≈ 187°, au-delà de la butée. Vaut aussi pour le vrai bras.

**Plafond de hauteur : la pointe ne monte pas au-dessus de ~0,14 m** outil à la
verticale, et pas au-dessus de 0,110 m à r = 0,28. Les 166 mm d'outil mangent
la course. `TRANSIT_Z = APPROACH_Z = 0.110`.

**Lâcher à 42 mm.** L'encombrement extérieur des doigts refermés (78–93 mm)
dépasse l'ouverture utile du bac (95 mm) : ils ne peuvent pas y descendre.
Mais l'objet pend sous les patins, donc à 42 mm il est déjà sous le rebord de
30 mm pendant que les doigts restent au-dessus.

**`setup.cfg` manquait au paquet** — sans lui les points d'entrée s'installent
dans `bin/` et `ros2 run` ne les trouve pas. Tout nœud ajouté depuis la
migration vers Osama_ws était introuvable malgré une compilation réussie.

### Prochaines actions

1. [JAUNE] Rejouer le cycle plusieurs fois de suite pour mesurer la
   répétabilité (un seul passage 4/4 à ce jour).
2. [JAUNE] Reporter `TOOL_OFFSET` centre-des-patins sur le vrai bras — la même
   erreur de 15 mm y est probablement présente.
3. [VERT] Brancher `color_object_detector` en entrée à la place des poses
   Gazebo, pour que le tri soit guidé par la vision.

### Commande rapide de reprise

```bash
conda deactivate
source /opt/ros/jazzy/setup.bash && source ~/Osama_ws/install/setup.bash
ros2 launch mycobot_gateway sim_grasp.launch.py
ros2 run mycobot_gateway sim_sorting_grasp          # -p only:="red_cube"
```

---

## État actuel (28 août 2026 — après-midi)

### Ce qui a été accompli aujourd'hui

**Cycle de tri complet validé bout à bout, deux fois.** Rouleau → petit carton
en 56 s, sans un seul échec ; figurine → grand carton en 73 s, avec un
rattrapage automatique en cours de route. Les trois correctifs du matin ont
tenu en conditions réelles.

**La figurine ne se prend qu'à certains roulis.** Balayage réel, couple par
défaut (jamais 250 sur une pièce imprimée) :

| roulis | angle de calage | issue |
|---|---|---|
| 0 | 22 (pince vide = 20) | rien saisi, figurine poussée de 18,3 mm |
| **+30** | **46-47** | **tenue** |

Le même +30 avait déjà été le seul à saisir le rouleau bleu à 389 mm. Deux
objets de formes très différentes, même réponse : le roulis 0 est mauvais, et
la géométrie seule ne le dit pas.

**Le cycle s'est rattrapé seul** : première saisie perdue à la remontée, pince
rouverte, dégagement, nouvelle position mesurée, seconde saisie à l'angle 47,
dépôt à 6,6 mm du point visé dans le grand carton — lequel, invisible depuis
l'observation, a été retrouvé objet en main (`vu depuis J1 = -28 deg`).

**Défaut trouvé et corrigé** : la pince rend *statut 2* — « objet saisi » — sur
une prise vide. Voir CHANGELOG.

**Mesures d'image sur la figurine** (aucune n'a débouché sur un correctif, mais
elles cadrent le problème) : silhouette 135 × 91 mm membres écartés ; largeur
locale minimale au point de prise **61,3 mm**, dans la direction 60° ;
sensibilité à `HAUTEUR_OBJET` de 0,36 mm par mm d'erreur de hauteur ; et surtout
**~2 mm par pixel** à 260 mm de portée — la figurine ne fait que 68 × 52 px,
l'érosion ne sépare pas ses jambes du torse à cette échelle.

### Décisions prises

- Ne rien coller sur la figurine (ni cible imprimée ni ArUco) tant que le
  balayage de roulis suffit à la saisir.
- Le couple de la pince reste au **défaut** pour la figurine : c'est une pièce
  imprimée à membres fins.

### Prochaines actions

1. [ROUGE] **Plafonner |J5|.** L'opérateur a signalé un risque de contact entre
   J4 et la pince. Au point de la figurine, roulis +90 demande **J5 = −121°** et
   +120 **−160°** ; les capsules annoncent pourtant 26-30 mm de marge, la paire
   `avant_bras/pince` étant désactivée. Mesuré : un plafond **|J5| ≤ 90°** ne
   refuse **aucune** des 120 poses de travail (max observé 83°). Le garde-fou
   n'est **pas** implémenté — le balayage incluant les inclinaisons a été
   interrompu, il faut le finir avant de figer le seuil.
2. [ROUGE] Cycle complet à **4 objets et 2 cartons** — les deux cycles validés
   n'avaient qu'un objet chacun.
3. [JAUNE] Mesurer la pince Pro ouverte (largeur et longueur) pour activer
   `avant_bras/pince` et retirer `PORTEE_MIN`.
4. [VERT] Recoller les marqueurs 10 et 11 : le grand carton reste invisible
   depuis l'observation miroir.

### Commande rapide de reprise

```bash
conda deactivate
cd ~/Osama_ws/src/mycobot_R6A
/usr/bin/python3 scripts/pick_dashboard.py
```

## État actuel (28 août 2026 — soir, simulation)

### Ce qui a été accompli

**La simulation de tri ne saisissait rien — et ne bougeait même pas.** Vérifié :
`No controllers are currently loaded!`, `/joint_states` à zéro, aucun abonné sur
les topics `cmd_pos`. La machine à états allait quand même jusqu'à « Sorting
complete » : elle est en boucle ouverte et **téléportait** les cubes via
`gz set_pose`. Le commit d'origine du 23/04 le disait déjà (*« emulates
grasp/release via the set_pose service »*) et l'historique git confirme qu'il
n'a **jamais** existé de version qui saisissait physiquement.

**Cause trouvée** : le plugin `gz-sim-joint-position-controller-system` a
disparu de l'URDF de cette copie du dépôt. Il est présent dans `~/ros_jazzy`,
d'où le souvenir d'une démo qui marchait — le bras y bougeait vraiment, et les
objets téléportés en même temps rendaient l'illusion complète. **Plugin
restauré.**

**Mesures faites sur la pince simulée** (elles resserviront) :

| | Écartement des doigts |
|---|---:|
| ouverte `[0,0,0,0]` | 136,0 mm |
| `[±0.7]` (ancienne butée) | 66,9 mm — trop large pour un cube de 40 mm |
| `[±1.10]` (nouvelle) | 17,9 mm |

Déport bride → milieu des doigts : **105,5 mm**, relevé sur `/world/.../pose/info`.
`diff_ik.fk_pose` et Gazebo donnent la **même bride au mm près** — l'IK du vrai
bras est donc directement réutilisable en simulation.

### Décisions prises

- La démo de tri reste **une démo de planification**, téléportation assumée et
  annoncée. Le travail vers une saisie physique est mis de côté dans le
  scratchpad, pas committé.
- Les gains du `mycobot_controller` restent à 100 : les monter empire le suivi.

### Prochaines actions

1. [ROUGE] **`gripper_controller` ne bouge jamais** en simulation, quelle que
   soit la consigne, alors que ses trois voisines suivent. La pince ne se
   referme que d'un côté. C'est le seul verrou avant une vraie préhension.
2. [JAUNE] Reprendre le travail sauvegardé (`ros2_control` + `diff_ik` +
   suppression de la téléportation) une fois ce joint réparé.
3. [VERT] Vérifier que la démo restaurée tourne de bout en bout, bras en
   mouvement — relancée mais non observée jusqu'au bout.

### Commande rapide de reprise

```bash
conda deactivate
source /opt/ros/jazzy/setup.bash && source ~/Osama_ws/install/setup.bash
ros2 launch mycobot_gateway pick_and_place_sorting.launch.py
```

## État actuel (28 août 2026 — matin)

### Ce qui a été accompli aujourd'hui

Séance courte, entièrement **mesurée en TCP sur le bras**.

**Le rouleau blanc n'a pas été perdu : il a été jeté.** Il était bel et bien
saisi (statut 2, angle 26) ; c'est `porte_objet` qui a répondu « lâché » sur une
lecture parasite, arrêté la remontée, puis ouvert la pince **en l'air**. Douze
lectures tracées pendant une remontée en trois paliers montrent d'où ça vient :
deux statuts « 6 » — la pince ne rend que 0 à 3 — et un angle « 65535 », le −1
du registre lu en 16 bits non signés. Une seule de ces réponses suffisait.
`porte_objet` relit maintenant jusqu'à trois fois, ignore un témoin illisible,
et conclut « tenu » quand les deux le restent. Le garde-fou a d'ailleurs joué
en direct pendant le dépôt du jour : « pince illisible trois fois — on considère
l'objet TENU », et l'objet était tenu.

**Le rouleau bleu à 389 mm se prend, mais pas avec n'importe quel roulis.** À
cette allonge l'outil vertical et l'inclinaison −15 **ne résolvent pas du
tout** ; seules −30 et −45 passent. À −30 les roulis 0, 30, 60 et 90 sont tous
géométriquement valides — et le roulis 0 referme la pince à vide en **poussant**
le rouleau de 6,4 mm, tandis que le roulis **+30** le saisit (statut 2, angle 26,
écart XY 0,59 mm) et le tient jusqu'à Z=170. La géométrie ne sépare pas les deux.
Le couple qui referme à vide est désormais mémorisé (`ctx.prises_ratees`) et
passe en dernier au prochain essai — écarté, jamais supprimé.

**Cycle complet rejoué en TCP** : rouleau repris à 283 mm (outil vertical,
écart XY 2,01 mm, statut 2, angle 25) puis déposé dans le **petit carton**, bord
proche à 382 mm, outil couché −30. Vérifié caméra : plus aucun rouleau sur la
planche.

**Seuil vertical/couché corrigé** : `INCLINAISONS_PAR_PORTEE` passe de 325 à
**320 mm**, mesuré au balayage de `colonne_continue` (le vertical descend
jusqu'à 320, refuse dès 324).

### Décisions prises

- Entre les deux erreurs possibles de `porte_objet`, on choisit **« tenu »** :
  croire tenir ce qu'on ne tient pas coûte un cycle, croire avoir lâché ce qu'on
  tient jette l'objet hors de la planche.
- Un couple (inclinaison, roulis) raté est **écarté, pas supprimé** — sinon un
  objet dont tous les couples ont échoué une fois devient insaisissable.

### Prochaines actions

1. [ROUGE] Rejouer un **cycle de tri complet** (4 objets, 2 cartons) avec les
   trois correctifs du jour — aucun n'a encore été vu bout à bout par la FSM.
2. [JAUNE] Mesurer la largeur/longueur réelles de la pince Pro ouverte, pour
   activer la paire de capsules `avant_bras/pince` et retirer `PORTEE_MIN`.
3. [VERT] Recoller les marqueurs 10 et 11 pour qu'ils restent lisibles des deux
   côtés de la planche.

### Commande rapide de reprise

```bash
conda deactivate
cd ~/Osama_ws/src/mycobot_R6A
/usr/bin/python3 scripts/pick_dashboard.py
```

## État actuel (27 août 2026 — soir)

### Ce qui a été accompli aujourd'hui

Séance entièrement **mesurée sur le bras et les caméras**, pas raisonnée à vide.

**La descente ne fait plus de va-et-vient.** Elle descendait, mesurait en bas,
remontait à 110 mm et recommençait — jusqu'à trois fois — et le biais repartait
de zéro à chaque objet, donc les quatre objets payaient chacun la passe perdue.
Désormais : **une seule descente**, recalée en vol. La descente étant verticale,
l'écart XY lu en route EST la dérive latérale, sans retard (le retard ne porte
que sur Z). Puis on saisit : c'est la pince qui tranche, plus un seuil.

**Le largage compense l'affaissement.** Mesuré : la pointe arrive 11,9 mm sous
la consigne à 340 mm de portée et 25,3 mm à 470 — la garde de 25 mm au-dessus du
rebord était mangée dès 410 mm et **nulle à 470**. Vérifié à un azimut autre que
celui de la régression : 108,0 / 107,1 / 109,0 mm atteints pour 108 voulus.

**L'anticollision par capsules est branchée**, rayons tirés des maillages du 320
(colonne 58, bras 48, avant-bras 43, poignet 44, bride 29 mm). 120 poses de
travail balayées : **0 refusée**, marge minimale +2,2 mm ; bras replié à 130 mm :
−5,7 mm, refusé. Coût 0,22 ms, 17 points par trajet.

**Trois défauts trouvés par le journal, pas par déduction :**

1. *La balle cochée sans avoir été soulevée.* Deux lignes consécutives : « objet
   lâché pendant la remontée » puis « la pince tient l'objet, on va le déposer ».
   Le bras est parti larguer du vide. Cause physique : l'objet qui glisse laisse
   les doigts à l'angle où ils s'étaient fermés et le statut bat. Corrigé en
   **rouvrant la pince** à la perte, et en ne cochant que ce que `ctx.en_main`
   confirme avoir été porté.
2. *Une tache de 543 cm² nommée « petit carton » sur la balle.* Bornes d'aire
   ajoutées (0,4 à 2,0 fois l'attendu), testées **après** le recentrage.
3. *L'étiquette « petit » posée sur le bras à 210 mm.* La reconstruction par
   marqueur ne peut plus placer une boîte loin de là où on la suivait.

**Le rouleau : diagnostic complet.** La prise réussit (statut 2, angle 24) puis
l'objet **glisse à la remontée**, et chaque essai raté le pousse — 65 mm de
dérive en sept tentatives. Six décalages latéraux (16 et 22 mm, quatre
directions) échouent ; le couple monté à 250 aussi. Ce n'est donc ni la visée ni
la force : les doigts ne prenaient qu'un quart de rouleau, parce que le scotch
visait le **centroïde de son anneau, c'est-à-dire le trou**. Il se prend
désormais par le **milieu de sa bande** (`PRISE_PAR_EPAISSEUR`).

**La reconnaissance des objets est vérifiée**, 40 images consécutives : rouleau
bleu 40/40, rouleau blanc 40/40, balle 40/40, petit robot 39/40 — zéro
hésitation. Correctement rejetés : le câble noir (44 × 101 mm, « pas sombre,
trop élancé »), les cartons, et quatre taches sous le seuil d'aire.

### Décisions prises

- `PORTEE_MIN` **reste à 170 mm**. Les capsules croisent zéro à ~143 mm, ce qui
  invitait à l'abaisser — mais l'incident du 26/08 s'est produit à **151 mm**, où
  le modèle donne une marge POSITIVE de +4 mm. La paire qui l'aurait vu venir
  (`avant_bras/pince`) est justement celle qu'il faut exclure faute de connaître
  le volume réel de la pince. Le raisonnement est figé dans un test.
- `Z_PRISE_MIN` passe de −18 à **−8**, la consigne nominale : les reprises ne
  creusent plus, la prise nominale ne bouge pas.
- Le couple monte pour le **rouleau seul** (250). Le petit robot reste au défaut.

### Prochaines actions

1. [ROUGE] Tri complet des 4 objets avec la prise par la bande du rouleau —
   c'est le seul changement non encore validé sur un cycle entier.
2. [JAUNE] Mesurer la largeur et la longueur réelles de la pince Pro, doigts
   ouverts : c'est ce qui débloquerait la paire `avant_bras/pince` et permettrait
   de supprimer `PORTEE_MIN` au profit du vrai modèle.
3. [JAUNE] Recoller les marqueurs 10 et 11 : leur lisibilité dépend du côté de la
   planche (mesuré : 40/40 puis 4/40 après échange des boîtes).
4. [VERT] Le rejet du câble noir tient à l'élancement — un câble mieux éclairé
   passerait pour un robot.

### Commande rapide de reprise

```bash
conda deactivate
cd ~/Osama_ws/src/mycobot_R6A
/usr/bin/python3 -m unittest discover -s tests -p 'test_*.py'
setsid nohup /usr/bin/python3 scripts/pick_dashboard.py > /tmp/dash.log 2>&1 &
tail -f /tmp/dash.log        # le journal sort maintenant du tableau de bord
```

## État actuel (27 août 2026 — après-midi)

### Ce qui a été accompli aujourd'hui

Séance de **mesure sur le vrai bras et les vraies caméras**, pas de conjecture.

- **La panne du 26/08 reproduite puis éteinte.** Le bras placé au-dessus du
  grand carton pour déposer le masque totalement : boîte vue **0 fois sur 40**
  pendant deux fenêtres de ~4 s, bien au-delà de `PEREMPTION_CARTON`. C'est là
  que le suivi la ré-adoptait ailleurs. Après correction : **0 déplacement**, le
  centre tenu à (388,4 ; 156,7) et retrouvé à 2,5 mm une fois le bras dégagé.
- **L'allonge n'était PAS la cause du largage raté.** Mesure d'arrivée à
  l'azimut +27 : écart XY de 2,7 à 7,4 mm à **toutes** les portées de 340 à
  470 mm. Le garde-fou d'arrivée à 45 mm ne se déclenchera donc pas à tort.
- **Ce qui manquait, c'est la hauteur.** La pointe arrive 11,9 mm sous la
  consigne à 340 mm et 25,3 mm à 470 : la garde de 25 mm au-dessus du rebord
  était mangée dès 410 mm et **nulle à 470** (82,7 mm atteints pour un rebord à
  82,9). Compensation linéaire ajoutée, vérifiée à un **autre azimut** que celui
  de la régression : 108,0 / 107,1 / 109,0 mm pour 108 voulus.
- **La vraie cause de « le petit se trompe ».** Son ouverture sombre a la
  signature d'un rouleau : trouvée 38 fois sur 40, **volée par le filtre des
  objets 33 fois**, nommée 5 fois sur 40. Une ouverture là où une boîte est déjà
  suivie n'est plus jetée → **5/40 puis 75/80**, et 80/80 marqueur revenu, avec
  0 déplacement fantôme.
- **Une boîte pleine se place par son marqueur.** Écart marqueur → ouverture
  appris boîte vide, exprimé dans le repère du marqueur donc valable même boîte
  tournée. Gelé 5 s, il reconstruit à **6,5 mm médian (13 max)** pour le petit,
  **10,3 (16,1)** pour le grand — contre 28 à 40 mm de marge intérieure au point
  de largage.
- **Suivi pendant que tu déplaces les boîtes** : re-ciblage mesuré en **0,1 à
  0,9 s**, jamais d'inversion entre les deux, y compris quand tu les as
  interverties côté pour côté.

### Décisions prises

- `PORTEE_CARTON_MAX` reste à 460 mm : la mesure d'arrivée montre que le bras y
  va vraiment. C'est la HAUTEUR qui manquait, pas la portée.
- Aucune position de carton n'est écrite sur disque, y compris la forme apprise
  (`Vision._forme_carton` vit en mémoire vive).

### Ce qui reste physique, pas logiciel

- **Les marqueurs 10 et 11 se voient mal selon le côté.** Mesuré : id 10 à
  **40/40** puis **4/40** après que les boîtes ont changé de côté, id 11 à
  **0/40** puis **27/40**. Celui qui part du côté défavorable devient illisible.
- Le contour du petit carton est un **C**, pas un rectangle — vu de biais au
  bord de la planche, son aire se mesure 90 à 117 cm² au lieu de 80. Le point de
  largage reste bon (28 mm de marge intérieure, croix vérifiée dans l'ouverture
  en image), mais le critère de TAILLE ne peut plus nommer cette boîte : seuls
  le marqueur et la continuité le peuvent.

### Prochaines actions

1. [ROUGE] Tri complet des 4 objets avec les cartons à leur place actuelle
   (grand à 414 mm, petit à 436 mm), puis en les déplaçant entre deux cycles.
2. [JAUNE] Recoller/agrandir les marqueurs 10 et 11 pour qu'ils restent lisibles
   des deux côtés de la planche.
3. [VERT] Rouleau à ~312-320 mm : reprises occasionnelles, non diagnostiquées.

### Commande rapide de reprise

```bash
conda deactivate
cd ~/Osama_ws/src/mycobot_R6A
/usr/bin/python3 -m unittest discover -s tests -p 'test_*.py'
setsid nohup /usr/bin/python3 scripts/pick_dashboard.py >/dev/null 2>&1 &
```

## État actuel (26 août 2026 — soir)

### Ce qui a été accompli aujourd'hui

- **La pince ne s'ouvre plus si le bras n'est pas arrivé.** C'est le défaut qui
  a mis le petit robot à côté du petit carton : la machine a commandé le largage
  en (401, 204), à 450 mm, le bras s'est immobilisé en (373, −52) — **256 mm
  avant** — et la pince s'est ouverte quand même. Rien ne vérifiait l'arrivée,
  seulement que l'ordre avait été accepté : `va_vers` rend la main quand le bras
  ne bouge **plus**, ce qui n'est pas la même chose qu'être à la cible (un ordre
  borné par les butées immobilise le bras en chemin). Écart toléré
  `ECART_LARGAGE_MAX = 45 mm` — au-delà du biais de modèle (18 mm mesuré à
  370 mm de portée), très en deçà d'une ouverture de carton (88 à 112 mm de
  côté). Au-delà, l'objet **reste en main** et le point de largage jamais atteint
  est mémorisé pour ne plus être proposé.
- **Un carton loin se vise par son bord proche, plus par son milieu.** Tous les
  candidats gardent le même recul des parois, donc le bord proche dépose dedans
  lui aussi. Au-delà de `PORTEE_LARGAGE_CONFORT = 400 mm`, les candidats sont
  ordonnés du plus proche du robot au plus lointain : sur le carton du 26/08 le
  premier point passe de 450 à 431 mm.
- **Une boîte marquée ne se déplace plus sur une occlusion.** Le journal
  annonçait « carton grand déplacé — point de largage à recalculer » quatre fois
  d'affilée. Cause : le bras se place **au-dessus** du carton pour déposer, la
  boîte sort de vue plus que `PEREMPTION_CARTON`, et la détection revient
  remplie de l'ombre et du bras, décalée de 50 à 170 mm. Cette position
  d'occlusion était ré-adoptée sans discuter — elle comptait un déplacement,
  périmait le point de largage **en plein transfert**, et la machine repartait
  chercher le carton. Désormais : la péremption ne relocalise plus une boîte
  identifiée par son marqueur, et un saut marqué demande **deux images
  concordantes** (~0,2 s à 10 im/s, immédiat à l'œil) au lieu d'une seule.
- **« Marqué » veut dire *cette position vient du marqueur*,** et non « le
  marqueur est visible quelque part ». Une position relayée par la SVPRO
  (corrigée d'un décalage appris de 14 à 60 mm) n'a plus cette autorité.

### Décisions prises

- **L'arrivée se mesure, elle ne se suppose pas.** Un objet gardé en main se
  redépose ; un objet lâché à côté se ramasse à la main.
- `PORTEE_CARTON_MAX` reste à 460 mm : l'IK y résout réellement (table du
  24/08), c'est l'**arrivée** qui ne suivait pas. Le contrôle d'arrivée découvre
  la vraie limite point par point plutôt que de la deviner.
- La position des cartons n'est toujours écrite nulle part (seul le roulis
  appris l'est) — voir l'entrée du 26/08 matin.

### Prochaines actions

1. [ROUGE] Séance robot : déplacer les deux cartons **et leurs marqueurs**,
   enchaîner plusieurs tris des 4 objets, vérifier qu'aucun « carton X déplacé »
   fantôme n'apparaît et qu'aucun objet ne tombe hors d'une boîte.
2. [JAUNE] Si un largage est annulé, relever la portée du carton : c'est la
   mesure qui dira où s'arrête vraiment l'allonge en dépose (460 mm est une
   limite d'IK, pas une limite mesurée).
3. [VERT] Rouleau à ~312-320 mm : reprises occasionnelles, non diagnostiquées
   (centrage XY ou biais de modèle croissant avec la portée).

### Commande rapide de reprise

```bash
conda deactivate
cd ~/Osama_ws/src/mycobot_R6A
/usr/bin/python3 -m unittest discover -s tests -p 'test_*.py'
setsid nohup /usr/bin/python3 scripts/pick_dashboard.py >/dev/null 2>&1 &
```

## État actuel (25 août 2026 — matin)

### Ce qui a été accompli aujourd'hui

- **Les deux cartons sont détectés 25/25, avec une étiquette stable.** Avant :
  2/20 pour l'un, 14/20 pour l'autre, et le nom `grand`/`petit` basculait d'une
  image à l'autre. Tremblement du centre ramené à 0,5 et 1,1 mm.
- **La cause était un seuil de 10 mm.** Le carton de gauche — 4400 px, anneau
  brun 0,79, contraste 40 — était jeté parce que son grand côté mesurait 220 mm
  contre 210 autorisés. Or ces 220 mm sont mesurés au plan **supposé** du
  rebord ; au vrai rebord il n'en fait que 214. Gabarit porté à 260 mm.
- **Le rebord des cartons est à 82,9 mm, pas 60** (triangulation des deux vues,
  écart des rayons 11,8 mm). C'était le plan sur lequel toute la géométrie des
  cartons se projetait : 50 mm d'écart sur le centre entre Z=0 et Z=100.
- **Le vrai coupable : le contour avalait l'ombre de la paroi extérieure.** Le
  petit carton, **115 × 70 mm au mètre ruban**, était mesuré 105 × 203 mm. Un
  seuil d'Otsu à l'intérieur du seul creux le ramène à **67 × 115,5 mm** — la
  mesure réelle. Du coup les deux cartons passent de 16 % d'écart (pour 18 % de
  bruit) à **115 ± 7 cm² contre 74 ± 1 cm²**, soit six fois le bruit : le
  gabarit les sépare tout seul. La piste « dimensions » était bonne, c'est la
  mesure qui était fausse.
- **Le carton fantôme au milieu de la table, c'était le bras.** Détecté comme un
  creux de 70 × 164 mm à 57 mm de la base, il prenait le nom de « petit
  carton » et ne bougeait pas quand on déplaçait le vrai. `RAYON_BASE_MIN =
  200 mm` : rien d'aussi près de la base n'est un carton. Le masque cinématique
  ne suffisait pas — il exige les angles, donc le pont vers la Pi.
- **Piste fermée : la hauteur par les deux caméras.** La SVPRO voit le carton
  lointain par la tranche, triangulation à **Z = −27 mm**, sous la table.
- **Trois filets de sécurité restent en place** pour le nom, si un jour deux
  cartons se ressemblent vraiment. D'abord `scripts/aruco_service.py`, détection ArUco
  déportée dans le venv (`cv2.aruco` fait segfaulter l'OpenCV du système),
  **4,5 ms par image**. `id 10` = grand, `id 11` = petit. Le marqueur donne le
  nom *et* la hauteur du rebord.
- Ensuite le **clic** : cliquer un carton dans le flux caméra le déclare GRAND. La désignation
  est gardée sur disque et suit les cartons qui bougent. Le détecteur ne
  l'écrit jamais seul — laissé libre il y a inscrit l'ombre du bras comme
  « petit carton » à (54, −18), au pied du robot.
- **Identité par continuité** (150 mm) : un carton déjà nommé garde son nom
  quand on le déplace à la main. C'est la réponse à « peu importe je bouge le
  carton ».
- **Un carton posé n'importe où reste atteignable.** Balayage IK de tout le
  plateau (X 200→480, Y −240→+240, pas de 40 mm) : aucun trou en deçà de
  460 mm de portée.
- 87 tests (9 neufs), seul l'échec IPPE pré-existant subsiste.

- **Les trois classes triées sur le robot réel, l'après-midi** : scotch blanc
  et scotch bleu dans le **petit** carton, petit robot dans le **grand**,
  chacun avec `objet saisi` confirmé par le statut pince. Cycles 64, 96 et 68 s.
- **Trois plafonds de gabarit trop serrés bloquaient la détection.** Les objets
  étaient mesurés au plan du rebord des cartons (83 mm) au lieu du plan où ils
  reposent (12 mm), soit 7 % trop gros ; le scotch blanc (72,8 mm) passait
  par-dessus le plafond de 70 mm ; le petit robot pattes étalées (79×146 mm)
  par-dessus celui de 140. Portés à 90 et 200 mm.
- **Le petit robot se saisit par son point le plus épais**, pas par le
  centroïde de son enveloppe — mesuré **23,5 mm hors du ventre**, sur un ventre
  de 41 mm. C'est ce qui faisait refermer la pince sur un maillon. Après
  correction : `objet saisi (angle 39)`, tenu jusqu'au largage.
- **Le couple de la pince ne doit PAS être monté** pour ce robot : pièce
  imprimée à maillons fins. Essayé à 250, annulé ; la raison est dans le code.
- **Un objet déjà dans un carton n'est plus une cible** — la balle déposée était
  redétectée 35,4 mm à l'intérieur de l'ouverture et le cycle repartait la
  chercher.

### Décisions prises

1. **Un gabarit ne doit pas être plus serré que l'incertitude sur le plan où on
   le mesure.** C'est ce qui a coûté un carton entier pour 10 mm.
2. **Le nom d'un carton se décide dans cet ordre** : marqueur, puis continuité,
   puis robe et gabarit. Les deux derniers sont réduits au rôle d'amorce —
   mesuré, ils ne tranchent pas entre deux cartons de même ouverture.
3. **La fusion des deux caméras sur les cartons est abandonnée.** La SVPRO reste
   utile sur les objets — elle s'accorde à 18 mm de l'arducam sur les deux
   scotchs — mais sur les cartons elle voit des parois, pas des ouvertures : 14
   mm d'accord sur le grand, 60 mm sur le petit. Son décalage appris est
   désormais **par carton**, un seul pour les deux étant faux pour les deux.
4. **Quand la pince lâche, chercher la visée avant la force.** Le réflexe de
   monter le couple a été essayé et annulé : la pièce est fragile, et le vrai
   défaut était de viser 23,5 mm à côté du ventre.

### Prochaines actions

1. [ROUGE] **Un objet déposé déforme le creux de son carton** — le scotch blanc
   a fait passer le petit carton de 74×127 à 83×172 mm, et l'aire ne sépare
   plus les deux boîtes. Contourné par la désignation gardée sur disque, mais
   la mesure reste fausse dès qu'il y a quelque chose dans la boîte.
2. [ROUGE] **Le suivi du petit carton a sauté à (500 · −27)**, hors planche,
   pendant le cycle du robot. Sans conséquence ce jour-là, mais c'est une
   fausse détection à traiter.
3. [JAUNE] Temps de cycle sous 60 s (DESCENTE 25 s, DETECTION 14 s,
   DEGAGEMENT jusqu'à 20 s).
4. [VERT] Saisie d'un scotch en régime incliné (> 355 mm) — jamais réussie.

### Commande rapide de reprise

```bash
conda deactivate
cd ~/Osama_ws/src/mycobot_R6A
/usr/bin/python3 scripts/pick_dashboard.py
```

---

## État actuel (24 août 2026 — soir)

### Ce qui a été accompli aujourd'hui

- **Le CYCLE de tri est validé sur le robot réel** — trois classes prises,
  transportées et larguées dans un carton, saisie confirmée par le statut
  pince. Cycles mesurés : 58 s (robot), 61 s (balle), 99 s (scotch).
  ⚠ **La DESTINATION ne l'est pas** : le scotch bleu, dirigé vers `petit`, a
  fini dans le grand carton. Voir « Le point qui bloque » ci-dessous.
- **Reconnaissance sans dépendance à l'éclairage.** Le scotch est un **anneau**
  (son bleu se lit H15 S90 V48 à l'exposition 75, indistinguable du bois) ; le
  robot est **noir désaturé** (S=44 contre 170 pour le bois même à l'ombre) et
  long d'au moins 60 mm.
- **Le carton fantôme est mort.** Trois largages au milieu de la table venaient
  de l'ombre du bras — elle le suit image après image, donc elle se confirme
  aussi bien qu'un vrai déplacement. Filtre à 200 mm du bras **entier**, et la
  mémoire sur disque, qui contenait le fantôme, est repartie propre et scindée
  par carton.
- **Hauteur de prise par catégorie et par régime.** Couché, l'outil visait Z=25,
  la mi-hauteur de la *balle* — sur un rouleau de 22 mm la pince se refermait
  au-dessus de lui. C'est ce qui faisait échouer toutes les saisies de scotch
  malgré 0,5 mm de précision latérale.
- **Enveloppe de dépose 360 → 460 mm**, l'outil se couche aussi pour larguer.
- 18 tests neufs (78 au total, seul l'échec IPPE pré-existant subsiste).

### Décisions prises

1. **L'arducam nomme, la SVPRO positionne.** Mesuré : depuis sa vue oblique la
   SVPRO prend une paroi de carton pour le robot et ne voit aucun anneau. Elle
   ne classe donc plus rien ; ses taches héritent du nom que l'arducam a donné
   au même endroit. Les deux vues affichent enfin la même chose.
2. **Grand carton = brun, petit = noir.** Le sens a été inversé une fois puis
   remis d'aplomb par trois mesures concordantes (consigne d'origine, ouvertures
   de 138×202 contre 62×113 mm, et l'essai réel). Verrouillé par deux tests.
3. **Aucune fermeture morphologique sur la détection d'objets** — elle bouchait
   le trou du rouleau, qui est toute sa signature.

### Le point qui bloque

**Distinguer le grand du petit carton n'est pas résolu.** Deux critères ont été
essayés et ont échoué :

* **l'aire de l'ouverture** — 10 915 contre 9 981 mm² à une position, 138×202
  contre 62×113 mm à une autre. Elle dépend trop de l'angle de vue et du
  débordement hors du plateau ; le classement bascule d'une image à l'autre ;
* **la robe** — brun (`S174 V87`, 2 % de pixels sous V=60) contre noir
  (`S148 V52`, 59 %). Le sens a dû être inversé deux fois, et le 24/08 au soir
  le scotch bleu dirigé vers `petit` a atterri dans le grand carton.

Le transport et le largage sont justes ; c'est **l'identité de la boîte** qui ne
l'est pas.

### Prochaines actions

1. [ROUGE] **Identifier les deux cartons de façon fiable, planche vide.** Les
   déplacer au hasard tous les deux, sans aucun objet autour, et trouver le
   critère qui tient : dimensions extérieures plutôt que l'ouverture, hauteur
   des parois par les deux caméras, ou un marqueur ArUco collé sur chacun — ce
   dernier étant mesuré faisable à 3,8 ms dans le Python du tableau de bord.
2. [ROUGE] **Le bras se positionne au-dessus du carton désigné** et le confirme
   visuellement, avant de rebrancher le tri complet.
3. [JAUNE] Ramener le temps de cycle sous 60 s : `DESCENTE` coûte 25 s en deux
   passes, `DETECTION` 14 s, `DEGAGEMENT` jusqu'à 20 s.
4. [VERT] Valider la saisie d'un scotch en régime **couché** (au-delà de
   355 mm) — jamais réussie ; à 331 mm en vertical elle passe du premier coup.

### Commande rapide de reprise

```bash
conda deactivate
cd ~/Osama_ws/src/mycobot_R6A
/usr/bin/python3 scripts/pick_dashboard.py
```

## État actuel (24 août 2026 — journée)

### Ce qui a été accompli aujourd'hui

- **Le cycle ne repart plus au ramassage avec la balle en main.** Garde unique
  dans `MachineEtats.pas()`, deux états neufs (`RECHERCHE_CARTON`,
  `ECHEC_PORTANT`). Seule une perte de prise relance la saisie.
- **Détection du carton refaite** : la couleur ne le sépare pas de la planche
  (mesuré carton H14 S171 V60, planche H15 S187 V84 — l'ancien seuil prenait la
  planche entière). Remplacée par la recherche d'un creux sombre entouré de
  brun, plus un suivi avec hystérésis qui stabilise le rectangle.
- **Temps de cycle : le vrai coupable trouvé.** Chaque mouvement attendait
  22,7 s parce que l'arrivée était jugée sur l'atteinte de la consigne, jamais
  satisfaite à cause de l'affaissement. Détection à l'immobilité :
  **22,7 s → 1,35 s par mouvement, à vitesse inchangée**.
- **SVPRO recalibrée et branchée en appui** de l'arducam (hauteur de la balle
  par triangulation, relais quand le bras masque la vue de dessus).
- **Portée corrigée** : la limite mesurée est 350 mm, pas 335 — des balles
  atteignables étaient refusées.
- 25 tests neufs (`tests/test_pick_fsm_depose.py`, `tests/test_suivi_carton.py`).

### Décisions prises

1. **L'arducam reste la source du X/Y** ; la SVPRO ne fournit que la hauteur et
   le relais en cas d'occultation. Refus si les rayons s'écartent de plus de 25 mm.
2. **Pas de veto de dernière seconde avant le largage** — à cet instant le bras
   masque le carton, la détection n'y est pas fiable. Le carton déplacé se
   rattrape à la recherche, bras dégagé.
3. **Viser le milieu du carton, et à défaut le point de l'ouverture le plus
   proche du milieu** que le bras atteint.
4. **La vitesse du bras reste à 25** : la mesure prouve que le temps ne venait
   pas de là (22,67 s par mouvement à vitesse 25 comme à vitesse 50).

### Prochaines actions

1. [ROUGE] Mesurer un cycle complet réel avec le chronomètre en place et
   attaquer les trois étapes les plus coûteuses qu'il désignera.
2. [JAUNE] Descendre sous ~9 mouvements par cycle : fusionner approche et
   recalage, supprimer la remontée par paliers quand le chemin direct est validé.
3. [JAUNE] Confirmer les dimensions réelles de l'ouverture du carton pour en
   faire un filtre dur (mesuré 101 × 135 mm, à recouper).
4. [VERT] Utiliser la SVPRO pour vérifier que la balle tombe bien dans le carton.

### Commande rapide de reprise

```bash
conda deactivate
/usr/bin/python3 scripts/pick_dashboard.py     # aucun autre client TCP sur la Pi
```

---

## État actuel (20 août 2026 — après-midi)

**Cycle pick-and-place complet réussi sur le robot réel**, de la localisation par
vision au dépôt en bac vérifié par image.

### Ce qui a été accompli

- Boucle d'asservissement visuel livrée (`mycobot_gateway/visual_servo/`, 47 tests).
- Calibration extrinsèque caméra→base sur 16 coins avec validation leave-one-out ;
  arducam RMS 1,01 px, stable à 1,7 mm près après deux jours.
- Cycle complet exécuté : balle localisée (base 360,6 / 23,1), approche, descente
  par paliers, saisie confirmée (statut 2), transport, dépôt en bac.
- Précision de placement finale : **3,9 / 0,6 / 0,1 mm** en X/Y/Z.
- Répétabilité mesurée sur 6 aller-retours : **0,67 mm** en approche unidirectionnelle
  par le haut (spec constructeur : 1 mm, donc tenue), mais **5,88 mm de biais
  directionnel** entre approche par le haut et par le côté.

### Décisions prises

1. **`send_coords` abandonné.** A/B chiffré : 247,8 mm d'erreur contre 18,2 mm
   pour `send_angles` + IK, sur cible identique. Échoue en silence (le bridge
   répond `OK`). Cause : blocage de cardan à RY ≈ −80°.
2. **Travailler sur la branche IK coude haut** (J3 < 0), pas celle de l'historique.
3. **Tourner l'orientation cible selon l'azimut** (`Rz(Δazimut) @ R_réf`) —
   facteur 100 sur le résidu IK.
4. **Compenser l'affaissement gravitaire** (~13 mm à vide, ~15 mm chargé).
5. Bridge Pi : **`gripper_bridge.py` obligatoire**, pas `bridge_pi_simple.py`.

### Prochaines actions

1. [ROUGE] Recalibrer la SVPRO — son extrinsèque est fausse de 32 à 149 mm
   (caméra déplacée). Elle ne voit que 2 des 4 marqueurs : la réorienter d'abord.
2. [ROUGE] `scripts/tool_offset.json` est **faux** (~90° d'erreur de direction) et
   le nœud d'asservissement le charge. Le refaire ou le neutraliser.
3. [JAUNE] Porter dans le nœud la rotation d'orientation selon l'azimut et la
   compensation d'affaissement — aujourd'hui appliquées dans les scripts d'essai.
4. [FAIT] Transfert haut validé — 150 mm au lieu de 120, garde au sol pendant la
   translation portée de ~120 à **185 mm**, statut pince vérifié à deux points du
   transit. Contrainte : le plafond au-dessus d'une cible à 368 mm de portée est
   de **160 mm** (à 200 mm le résidu IK monte à 12,8 mm) — choisir la hauteur de
   transfert comme la plus haute atteignable **aux deux extrémités**.
5. [VERT] Passer le serrage par `set_pro_gripper_torque` plutôt que par l'angle.

### Commande rapide de reprise

```bash
conda deactivate && source /opt/ros/jazzy/setup.bash
source ~/Osama_ws/install/setup.bash
ping -c1 10.10.0.221            # bridge = gripper_bridge.py sur la Pi
ros2 launch mycobot_gateway visual_servo.launch.py     # démarre DÉSARMÉ
```

⚠ Vérifier `ps aux | grep bridge_tour` avant : le pont de la Pi est mono-client
et bloquant, un `bridge_tour` résiduel (que le preflight laisse tourner) le fige.

---

**Date d'arrêt :** 31 juillet 2026

**Workspace :** `/home/genji/Osama_ws/src/mycobot_R6A`

**État robot connu :** pose de remise, pince ouverte. Ne pas supposer que cette
pose est encore actuelle à la prochaine session : relire les angles avant tout
mouvement.

## Objectif confirmé

L'objet peut être placé n'importe où dans la zone délimitée par les quatre
ArUco, sous réserve de la portée du bras et de la zone réellement enseignée.
Le système doit redétecter l'objet, recalculer sa position dans `base_link`,
adapter les angles articulaires appris, saisir, puis effectuer la remise.

## Architecture correcte

```text
Arducam -> centre objet (HSV aujourd'hui, YOLO custom possible)
         -> intrinsèque cam_0
         -> extrinsèque eye-to-hand caméra->base FIXE
         -> XYZ objet dans base_link

4 ArUco -> frontière géométrique de sécurité LIVE
         -> ils ne détectent pas l'objet
         -> ils ne remplacent pas l'extrinsèque

XYZ base -> interpolation de démonstrations -> send_angles uniquement
```

La planche ArUco peut glisser sur le même plan horizontal. La caméra doit rester
rigide par rapport à la base robot. Un changement de hauteur/inclinaison de la
planche ou un déplacement de la caméra exige une nouvelle calibration adaptée.

## Travail terminé

- `scripts/adaptive_pick_by_demo.py` créé et corrigé :
  - extrinsèque par défaut :
    `training/calibration/arducam_extrinsic_handeye.yaml` ;
  - détection balle multi-images sur `/camera/image_raw` ;
  - quatre IDs ArUco 19/23/25/26 obligatoires ;
  - frontière ArUco reconstruite en direct dans `base_link` ;
  - stabilité des tags exigée, dispersion maximale 3 px ;
  - cible refusée hors frontière ou trop près du bord ;
  - apprentissage transactionnel approche/pick ;
  - interpolation IDW en angles, sans `send_coords` ;
  - aucune extrapolation hors enveloppe des démonstrations ;
  - dry-run par défaut ; confirmation explicite pour le réel ;
  - re-détection avant descente et retrait de secours en cas d'échec.
- `scripts/live_aruco_geometry.py` et ses tests sont présents.
- Guide : `docs/ADAPTIVE_PICK_BY_DEMO.md`.
- Tests validés :
  - `19/19` dans `tests/test_adaptive_pick_by_demo.py` ;
  - `8/8` dans `tests/test_live_aruco_geometry.py` ;
  - compilation Python OK.

## Dernière validation LIVE, sans mouvement

La capture ROS/Arducam en lecture seule a réussi :

- balle stable : pixel `(162.6, 155.6)` ;
- position calculée : `(X,Y,Z) = (0.3633, 0.2131, 0.0335) m` ;
- ArUco vus : `[19, 23, 25, 26]` ;
- dispersion maximale : `0.58 px` ;
- frontière calculée dans la base :
  - `(0.0906, 0.2858)` ;
  - `(0.1031, -0.0674)` ;
  - `(0.4885, -0.0826)` ;
  - `(0.5206, 0.2748)`.

La transaction produite était seulement une sonde temporaire :
`/tmp/adaptive_pick_live_probe_v2.json.pending`.

## Ce qui n'est pas terminé

- Aucun dataset réel `scripts/adaptive_pick_demos.json` n'a encore été validé.
- Aucune commande de mouvement robot n'a été envoyée pendant cette correction.
- Il faut au minimum trois démonstrations synchronisées et non collinéaires ;
  cinq à neuf points donnent une couverture plus sûre de la table.
- Les anciennes poses de `scripts/pick_place_positions.json` ne doivent pas être
  importées silencieusement sans associer chaque pose à l'observation objet de
  la même démonstration.
- La remise automatique après le pick adaptatif n'est pas encore intégrée dans
  `adaptive_pick_by_demo.py`. Les poses `handover_approach` et `handover` sont
  déjà enregistrées.
- L'ouverture sur détection de main doit utiliser un vrai modèle YOLO possédant
  une classe `hand`, une ROI fixe sous la pince et plusieurs frames stables.
  `yolov8n.pt` COCO n'a pas de classe main : ne pas ouvrir sur la classe
  `person` par défaut.

## Reprise recommandée

1. Vérifier visuellement la scène, l'arrêt d'urgence et la pose réelle du bras.
2. Démarrer ou vérifier `dream_multicam.launch.py` et `bridge_tour`.
3. Ouvrir la première démonstration, sans mouvement automatique :

   ```bash
   cd /home/genji/Osama_ws/src/mycobot_R6A
   .venv/bin/python scripts/adaptive_pick_by_demo.py --begin-live --name point-1
   ```

4. Montrer manuellement l'approche puis la prise et capturer :

   ```bash
   .venv/bin/python scripts/adaptive_pick_by_demo.py \
     --capture-approach --read-current-angles
   .venv/bin/python scripts/adaptive_pick_by_demo.py \
     --capture-pick --read-current-angles
   ```

5. Répéter à au moins deux autres positions non collinéaires.
6. Vérifier le dataset puis faire uniquement un plan sec :

   ```bash
   .venv/bin/python scripts/adaptive_pick_by_demo.py --show
   .venv/bin/python scripts/adaptive_pick_by_demo.py --plan-live
   ```
> Mise à jour 28/04 (PM) : test cheap cam0+cam3 fait. Signal clair : **cam3 utile mais extrinsèques approximatives load-bearing**. Plan v3 = **calibrer cam3 avant retrain**. Les résultats détaillés sont dans CHANGELOG 1.13.0.

1. **[ROUGE] Calibrer cam3** :
   - Intrinsèques : chessboard OpenCV (~5 min, donne fx, fy, cx, cy spécifiques à cam3)
   - Extrinsèques : soit mesure physique au mètre + équerre, soit PnP sur 1 image de chessboard placée sur la base du robot, soit PnP sur les détections proximales du checkpoint v1 (link1/link2 à 100 % détection sur cam0 → applicable à cam3)
2. **[ROUGE] Calibrer cam0** par la même occasion (vérification de fx=610) — 5 min de plus.
3. **[ROUGE] Refactor `training/dream/convert_to_ndds.py`** :
   - `REAL_CAMERA_INTRINSICS` devient un dict `{cam0: K0, cam3: K3}`
   - Update `REAL_CAMERA_TRANSFORMS["cam3"]` avec les valeurs calibrées
   - Utiliser le bon K par cam dans `convert_real()`
4. **[ROUGE] Régénérer** `real_cam0_v3` + `real_cam3_v3` avec les bonnes annotations.
5. **[ROUGE] Build `mixed_v3`** : même structure (2K cam0 ×3 + 2K cam3 ×3 + 6K synth = 18K).
6. **[ROUGE] Retrain 50 epochs** (au lieu de 25 — la val loss n'avait pas plateauté à e25 sur v2). Output : `training/checkpoints_dream/vgg_mixed_v3/`.
7. **[ROUGE] Cible** : ≥ 50 % cam0 + ≥ 50 % cam3 simultanément, sans le trade-off observé en v2.
8. **[JAUNE] Si v3 dépasse 50 %** → augmenter dataset (capture poses bras-étendu sur les 2 caméras) puis retrain v4 → cible 70 %.
9. **[JAUNE] Vérifier collecte 30K synth v2** dans `/tmp/dream_data/synthetic_50k_v2/` — utiliser le worlds `randomized_v2.sdf`.
10. **[VERT] Tester l'inférence DREAM en sim Gazebo** (`pick_and_place.launch.py`) avec le checkpoint v1 actuel (toujours le meilleur sur cam0).
11. **[VERT] Bench test robot réel** une fois détection ≥ 70 %.

7. Ne lancer `--execute` qu'après inspection du plan et d'abord près d'une
   démonstration connue, à vitesse lente.

## Détails à ne pas oublier

- Pi/bridge observé : `192.168.223.59:5005`.
- Avec le dashboard actif, utiliser ROS `/to_robot` et `/from_robot`, pas une
  seconde connexion TCP directe.
- Utiliser `.venv/bin/python` pour le live : OpenCV système 4.6 a provoqué un
  crash ArUco ; le venv utilise OpenCV 5.
- `workspace_markers.yaml` contient encore une incohérence documentaire
  `marker_size_m: 0.080` contre commentaire `50 mm`. La frontière actuelle
  utilise les centres et n'utilise pas cette taille, mais elle devra être
  mesurée/corrigée avant un futur PnP par coins.
