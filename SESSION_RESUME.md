# SESSION RESUME — MyCobot 320 Pi R6A

> **Date de dernière mise à jour :** 24 juillet 2026 (soir — fusion multi-caméras solve-then-fuse + filtres temporels + anti-clignotement)
> **Version :** 2.2.0 (téléop) · 1.10.0 (sorting) · 1.11.0 (pose-est diagnostic + tooling)
> **Branche active :** `main`
> **Repository :** https://github.com/ABMI-software/mycobot_320pi_R6A
> **Pi réelle :** `10.10.0.221` (pas `.223` ni `.225` comme certains anciens docs)

---

## Point de départ rapide

```bash
# TOUJOURS exécuter avant ROS2
conda deactivate

source /opt/ros/jazzy/setup.bash
source ~/Osama_ws/install/setup.bash
```

---

## État actuel (24 juillet 2026 — soir)

### Ce qui a été accompli

**Fusion multi-caméras robuste + stabilité des courbes/affichage.** Suite directe
du multi-caméras du 23/07, sur retours en session live avec l'arducam + SVPRO.

- **Fusion *solve-then-fuse*** — abandon du bundle partagé (`solve_joint_angles_multiview`,
  qui basculait de branche : J1 −43°). Désormais chaque caméra résout son `q`
  séparément (mode cohérence par vue), puis fusion **par joint** pondérée par
  l'observabilité (keypoint observant détecté + reproj ≤ 15 px). MAE fusion
  ~1.1-1.9° ; l'occlusion d'une vue est reprise par l'autre. Repli **MONO via {caméra}**
  si la primaire (arducam) devient aveugle.
- **Affichage fusion** — vues empilées **verticalement**, HUD identique sur chaque
  vue secondaire (Caméra FPS / DREAM Hz / pastille pose). **Tableau keypoint =
  fusion** (erreur moyenne des caméras détectant le point) + **Détection globale
  (fusion) : N/7 kp** (union arducam + SVPRO).
- **3 filtres temporels au choix** — boutons radio `aucun`(défaut) / `kalman` /
  `passe_bas` (EMA) / `moyenne` (glissante). Kalman **plus** activé d'office (retour
  utilisateur). Sous-dossier CSV = nom du filtre.
- **Anti-clignotement** — keypoints secondaires tenus 0.8 s après détection
  (affichage seul, solveur intact) ; pastille pose verte tant qu'une vue a détecté
  ≥4 kp dans la dernière seconde (fin des faux jaunes « sans avoir bougé »).
- **Exposition arducam** — balayage 60/70/75/90 mesuré (`scratchpad/exposure_test.py`) :
  la détection DREAM reste ~4.8-5.0/7 quelle que soit l'expo → **garder 75** (ton
  d'entraînement session4).

### Décisions prises

- **Pas de commit** (demandé) — docs mises à jour uniquement.
- **SVPRO « propre »** : aucun réglage d'exposition/luminosité/focus (contrairement
  à l'arducam=75). Cause de la SVPRO sombre = backend GStreamer + une expo=75
  manuelle coincée dans le device par un conflit d'index → corrigé (backend V4L2 +
  `set_auto_exposure()` forcé au démarrage).
- **Astra toujours exclue** (pas de V4L2/intrinsèque PnP). J6 structurellement
  inobservable (pas de gripper). Le vrai levier J3-J6 reste le **placement caméra**
  ou une détection distale, pas les poids solveur.

### Prochaines actions

1. [ROUGE] Valider la fusion 2-cam **sur matériel réel** dans plusieurs poses (bras
   à plat où l'arducam décroche → la SVPRO doit reprendre).
2. [JAUNE] Réfléchir au **placement physique** des 2 caméras (recouvrement de FOV)
   pour lever l'ambiguïté de branche J1/J2 dans les poses dures.
3. [VERT] Commiter quand l'utilisateur le demande (branche `feature/pose-training`).

### Commande rapide de reprise

```bash
conda deactivate
export PATH="$(echo "$PATH" | tr ':' '\n' | grep -v '\.venv' | paste -sd:)"; unset VIRTUAL_ENV
source /opt/ros/jazzy/setup.bash && source ~/Osama_ws/install/setup.bash
pkill -f dream_validation_dashboard; sleep 1
ros2 launch mycobot_gateway dream_multicam.launch.py   # auto-détecte 1 ou 2 caméras
```

**Ce que ça affiche** : 2 vues empilées (arducam + svpro) avec HUD FPS/DREAM Hz,
6 courbes encodeur vs DREAM, tableau keypoint **fusionné** + « Détection globale
(fusion) N/7 kp », badge « 🔗 FUSION N vues » (ou « MONO via {caméra} »). Graphe
ROS2 : [`training/dream/rqt_dream_multicam.png`](training/dream/rqt_dream_multicam.png).
**Diagnostic** (symptôme → nœud manquant, piège `.venv`) :
[`docs/DREAM_VALIDATION_LAUNCH.md`](docs/DREAM_VALIDATION_LAUNCH.md).

---

## État actuel (23 juillet 2026 — soir)

### Ce qui a été accompli

**Dashboard DREAM rendu multi-caméras (flexible 1 ou 2 vues, auto-détection).**
Objectif : améliorer la détection/observabilité en fusionnant plusieurs caméras,
sans rien casser du mono.

- **`vision/camera_registry.py`** (nouveau) — sonde `v4l2-ctl`, reconnaît
  arducam/SVPRO, charge et **rescale** leur intrinsèque déjà calibrée
  (arducam=`cam_3`, SVPRO=`cam_2` 800×600→640×480), exposition par caméra
  (arducam 75, SVPRO normale). Testé : détecte l'arducam (fx=496).
- **`dream_angle_solver.solve_joint_angles_multiview`** (nouveau) — `q` partagé +
  une pose caméra par vue, résidu combiné. Testé synthétique : fusion 2 vues
  MAE 2.30° vs mono 2.58°.
- **`launch/dream_multicam.launch.py`** (nouveau) — un seul launch qui
  auto-détecte et spawne N branches parallèles + nœuds partagés + dashboard.
  `ros2 launch ... --show-args` OK.
- **Dashboard** — param `cameras`, branche fusion dans `estimate_dream_angles`
  (`_fuse_multiview`), vignettes par caméra secondaire, badge « 🔗 FUSION N vues ».
  Mono par défaut = comportement historique inchangé.
- **`camera_publisher`/`dream_inference`** paramétrés par caméra (`output_topic`
  / `output_prefix`). Build colcon OK dans `~/Osama_ws`.

### Décisions prises

- **Astra exclue** de la fusion : pas de nœud V4L2 ni d'intrinsèque PnP (cf.
  CLAUDE.md 2026-07-13). Fusion = arducam + SVPRO, les deux calibrées.
- **Pas de recalibration** : les intrinsèques `cam_2`/`cam_3` existent déjà ; le
  registry les charge/rescale automatiquement.
- **Rétrocompatibilité stricte** : topics arducam legacy conservés
  (`/camera/image_raw`, `/dream/keypoints`), SVPRO sur topics namespacés.

### Prochaines actions

1. [ROUGE] **Valider la fusion 2-cam sur matériel** : brancher la SVPRO, relancer
   `dream_multicam.launch.py`, vérifier mode FUSION + gain MAE réel. Non testé.
2. [JAUNE] Retirer le `.venv` du PATH avant lancement (`deactivate` inopérant ici).
3. [VERT] Après validation → commit (docs déjà à jour) sur `feature/pick-and-place-osama`.

### Commande rapide de reprise

```bash
# PATH sans .venv, puis :
source /opt/ros/jazzy/setup.bash && source ~/Osama_ws/install/setup.bash
ros2 launch mycobot_gateway dream_multicam.launch.py
```

---

## État actuel (23 juillet 2026 — après-midi)

### Ce qui a été accompli aujourd'hui

Session de **réglage et fiabilisation du dashboard de validation DREAM** (réel,
`10.10.0.221`), après remise en route de la caméra Arducam.

- **`reset_kalman()`** ajouté : les filtres sont réinitialisés sur `SET Angles`,
  `SET Coords` et `Pose automatique`. Règle le blocage où la courbe filtrée
  restait coincée sur l'ancien angle pendant un vrai mouvement commandé (le
  portail anti-aberration le prenait pour une excursion). Testé OK — J1 0→100°
  suivi correctement (erreur 0.04°).
- **CSV filtrés séparés** : sous-dossier `…/kalman/` créé quand le filtre est
  actif (colonne `dream` = filtrée), brut dans le dossier parent sinon. Vérifié
  sur `acquisitions/auto/kalman/…csv` — 19 colonnes correctes.
- **Réglages Kalman/solveur** : `q_pos` 3.0→0.5 ; `_CONSISTENCY_REG_VEC` J1→10,
  J2→40 (corrige la bascule de branche de J2, ~45°→~2-3°). Labels MAE/RMSE
  annotés `(J1–J6)`.
- **Caméra** : décrochage USB (`unable to enumerate`) résolu par changement de
  port. Backend GStreamer ignore toujours `CAP_PROP_FPS` (cause du FPS bas).

### Décisions prises

- **Le mode libre (`use_encoder_seed=False`) est inutilisable** en monoculaire
  (branches folles, MAE ~65°) → on garde le mode cohérence (seed encodeur).
- **Baisser les poids n'aide pas J3-J6** : leurs keypoints distaux ne sont
  souvent **pas détectés** → rien à raffiner. Le vrai levier est la **détection
  distale (modèle)** ou une **2ᵉ caméra**, pas le réglage solveur.
- **Restylage PyQt6/thème sombre : annulé.** Maquette produite puis abandonnée à
  la demande ; le dashboard reste en PyQt5, look d'origine.
- **Anti-saut pour pick-and-place** : à mettre dans la **couche trajectoire**
  (limite de vitesse), pas dans le lissage de perception (qui cacherait l'erreur).

### Prochaines actions

1. [ROUGE] Améliorer la **détection distale** (link4/5/6) — c'est le vrai
   plafond de J3-J6, pas le solveur.
2. [JAUNE] Optionnel : bouton **« Copier position actuelle »** dans SET Coords
   (évite les cibles IK inatteignables → LED bleu).
3. [VERT] Optionnel : acquisition auto qui **attend la stabilisation** avant
   d'enregistrer (mesure DREAM à l'arrêt, sans transitoire).

### Commande rapide de reprise

```bash
deactivate
source /opt/ros/jazzy/setup.bash
source ~/Osama_ws/install/setup.bash
# 5 nœuds — voir docs/DREAM_VALIDATION_LAUNCH.md
ros2 run mycobot_gateway dream_validation_dashboard
```

---

## État actuel (17 juillet 2026 — après-midi)

### Ce qui a été accompli aujourd'hui

Session de **diagnostic** (aucun code applicatif modifié ce jour — lecture,
mesures, interprétation des topics) sur la chaîne de validation DREAM live.

- **Chaîne de lancement du dashboard remise en route** : au démarrage seuls
  `camera_publisher` et `dream_inference` tournaient ; `joint_sync` et
  `bridge_tour` manquaient (d'où encodeurs à 0 et `SET Angles` sans effet).
  Relancés → `/joint_states` et `/from_robot` de nouveau vivants. Procédure de
  référence : [`docs/DREAM_VALIDATION_LAUNCH.md`](docs/DREAM_VALIDATION_LAUNCH.md).
- **Lecture des topics** confirmée conforme au code : `/dream/keypoints` =
  7×`[u,v,valid]` + `sec,nanosec,inference_ms` ; le réseau DREAM ne prédit **que
  des keypoints 2D**, les angles J1-J6 sont reconstruits *après coup* par le
  solveur `least_squares` (`dream_angle_solver.py`, reprojection FK vs détections).
- **Cause du bas débit vision isolée (matériel confirmé)** : la caméra tourne en
  **YUYV plafonné à 10 fps** (le `set(FOURCC, MJPG)` est ignoré par le backend
  GStreamer), le timer tire à 30 Hz → warnings `⚠️ Dropped frame`. S'ajoutent
  l'exposition manuelle=75 devenue trop sombre (image quasi noire, pixels 11-49)
  et `net.core.rmem_max`=208 Ko sous-dimensionné (gels DDS de ~2.8 s). Résultat
  live : 2-4 keypoints/7 au lieu de 91.6 %, débit DREAM ~1.8 Hz.
- **Gripper « force effect » branché** : support Pi présent
  (`bridge_pi_simple.py` : `gripper_open`/`gripper_close` via `set_gripper_state`),
  mais **`bridge_tour` et le dashboard ne connaissent pas le gripper** (zéro
  occurrence). API `set_gripper_state` générique tout-ou-rien — pas sûre pour un
  gripper à contrôle de force. Non testé (bridge Pi tombé en fin de session).

### Décisions prises

- Tester le gripper **isolément** (bras à l'arrêt, commande TCP unique) avant tout
  test combiné vision+gripper — ne pas mêler deux inconnues.
- Correctifs caméra (MJPG/V4L2, exposition, `rmem_max`) connus et déjà mesurés
  gagnants dans une session passée puis revertés — à ré-appliquer sur décision.

### Prochaines actions

1. [ROUGE] Relancer `bridge_pi_simple.py` sur le Pi (port 5005 fermé) et repasser
   `scripts/real_robot_preflight.sh` en 5/5.
2. [ROUGE] Confirmer modèle exact du gripper + s'il est monté sur la bride J6
   (visible arducam → rouvre la piste « keypoint aval J6 » pour l'observabilité).
3. [JAUNE] Remettre la chaîne caméra en état nominal (MJPG/V4L2 + exposition +
   `rmem_max`) puis vérifier que le 91.6 % détection se retrouve en direct.
4. [VERT] Mettre à jour les 6 docs affirmant « robot sans gripper / `--no-gripper`
   obligatoire » — **après** validation physique du gripper, pas avant.

### Commande rapide de reprise

```bash
ssh er@10.10.0.221            # puis: python3 bridge_pi_simple.py
bash scripts/real_robot_preflight.sh
```

---

## État actuel (13 juillet 2026 — après-midi)

### Ce qui a été accompli aujourd'hui

- **Dashboard unique PyQt** (les 3 dashboards demandés en réunion "Validation
  Modèle IA — Cercles EndEffector" fusionnés dans une seule fenêtre, pas
  d'onglets) : `mycobot_gateway/mycobot_gateway/dream_validation_dashboard.py`.
  - Vue caméra live : squelette encodeur (vert, gros cercles + liaisons) vs
    squelette DREAM brut (magenta, petits cercles), erreur par keypoint en px.
  - Panneau pilotage manuel (angles FK + coordonnées IK, boutons SET, verrouillage
    /libération servos `🔒 Fixer` / `🔓 Relâcher`).
  - Panneau KPI : précision (RMS reprojection px), répétabilité (jitter encodeur),
    temps de réponse commande→confirmation, tableau angulaire Encodeur/DREAM/Erreur
    par joint + RMS globale (J1-J6) et RMS sur joints observables (J1-J5).
  - 6 courbes temps réel (une par joint) : encodeur (plein) vs DREAM (pointillé).
  - **Toujours pas d'extrinsèque caméra pré-calibrée** — pose caméra résolue par
    PnP à partir de l'intrinsèque Arducam + encodeurs + détections DREAM,
    jamais depuis un fichier figé.
- **Ancre de pose de session robustifiée** : au lieu de figer la pose caméra sur
  le premier frame (risque : une détection bruitée fixe une mauvaise pose pour
  toute la session), accumulation de 30 solves PnP (encodeur connu) au démarrage
  et ancrage sur la **médiane** rvec/tvec.
- **Estimation d'angles DREAM indépendante** (`training/dream/dream_angle_solver.py`,
  `solve_joint_angles_fixed_pose`) : ancre de pose figée + régularisation légère
  (poids 1.5 px/rad) vers l'estimation DREAM de la frame précédente (pas
  l'encodeur), pour une courbe DREAM temporellement lissée et réellement
  indépendante plutôt que recollée à l'encodeur chaque frame.
- **J6 structurellement non observable, confirmé mathématiquement** : dans
  `mycobot_fk.forward_kinematics`, la position d'un keypoint ne dépend jamais
  de la rotation de son propre joint — donc aucun des 7 keypoints DREAM ne porte
  d'information sur J6. Badge `⚠` permanent sur J6 dans le tableau KPI.
  J4/J5 (peu de keypoints porteurs : 2 et 1 respectivement) reçoivent un badge
  **dynamique**, basé sur l'erreur de reprojection live des keypoints dont ils
  dépendent (seuil 15px) — affiché seulement quand réellement dégradé sur le
  frame courant, pas en permanence.
- **Validation offline chiffrée sur données réelles** (nouveau script
  `training/dream/validate_angle_solver_real.py`) : rejoue le pipeline complet
  (inférence DREAM + ancre multi-frame + solveur) sur les 2500 poses réelles de
  `training/dream_data/real_3cam` (5 sessions, caméra arducam), avec vérité
  terrain encodeur. Résultats sur 2343 poses tenues à l'écart :
  - Détections DREAM brutes (px, vs vérité reprojetée) : base/link1/link2 ~2.5px
    (bon), link3 ~11px, **link4/5/6 22-34px RMS, jusqu'à 250px sur les pires
    frames** — confirme le "distal keypoint problem" déjà documenté, maintenant
    quantifié sur données réelles.
  - Angles, warm-start encodeur (meilleur cas) : RMS J1=15.8° J2=13.7° J3=28.4°
    J4=34.3° J5=33.8° J6=0.0° (figé par construction) — RMS globale J1-J5 26.7°.
  - Warm-start à froid (q_init=0, pire cas) : RMS globale 40.2°.
  - Conclusion : la sensibilité aux distales explique directement la hiérarchie
    d'erreur par joint (J4/J5 vus par 1-2 keypoints seulement, sans redondance
    pour moyenner le bruit). Ce n'est pas un bug de solveur — augmenter le poids
    de régularisation masquerait le problème (collerait DREAM à l'encodeur) sans
    le résoudre.
- **Test live sur robot réel** (arducam + encodeurs, checkpoint
  `vgg_ultimate_v4_mix_ft_e30`) : pipeline temps réel opérationnelle de bout en
  bout, mais reconstruction articulaire **pas encore validée**. Malgré une
  erreur de reprojection visuelle modérée, RMS angulaire mesurée à **17,93° sur
  les joints observables** (J1-J5) — cohérent avec le test offline mais encore
  trop élevé pour être exploitable.
- Corrections de bugs découverts en testant sur robot réel : parsing des
  réponses bridge dans `joint_sync.py` (`/joint_states` restait à zéro — format
  `ANGLES:` en majuscules non reconnu) et `dream_validation_dashboard.py`
  (même bug + coalescence multi-lignes TCP) ; résolution de checkpoint et de
  symlink dans `dream_inference_node.py` ; polling périodique manquant pour
  `get_coords` (lecture position figée) ; validation bornes image pour les
  keypoints DREAM (sentinelle "non détecté" lue comme détection confiante hors
  cadre).

### Prochaines actions

1. [ROUGE] Investiguer la stabilité du solveur d'angles (pourquoi 17,93° en
   live alors que l'ancre + régularisation sont en place), la fréquence
   d'inférence DREAM, et les ambiguïtés liées à la vue monoculaire quasi
   zénithale (J1 confondu avec un biais de lacet caméra).
2. [JAUNE] Évaluer si une 2e caméra (svpro/astra) réduit l'ambiguïté de vue
   unique — question posée en session, pas encore tranchée.
3. [VERT] Une fois la reconstruction articulaire validée : mise à jour
   documentaire IP `10.10.0.223` → `10.10.0.221` (identifiée, pas encore faite),
   puis commit/push final.

### Commande rapide de reprise
```bash
conda deactivate
source /opt/ros/jazzy/setup.bash
cd ~/Osama_ws && colcon build --packages-select mycobot_gateway --symlink-install
source install/setup.bash
ros2 run mycobot_gateway dream_validation_dashboard
```

---

## État actuel (8 juillet 2026 — soir)

### Ce qui a été accompli aujourd'hui (soir)

- **Cartographie eye-to-hand sur les 3 caméras.** Constat clé : **détection ≠
  récupération d'angles** (deux problèmes séparés).
  - Détection real_3cam (`keypoint_accuracy_curve.py`, par caméra) : svpro 98% ·
    astra 95% · arducam 89%. → la **caméra astra est bonne** ; l'astra fraîche à
    48% = **placement** hors-domaine, pas la caméra.
  - Angles : caméra **mono** (arducam/svpro, sans depth) mal conditionnée (7–24°
    même amorcée) ; **depth (astra) indispensable** pour j1–j4 <2°.
- **Plateforme visual-servoing** livrée : `visual_servoing_platform.py` (6 fenêtres
  joint réel vs estimé + écart) et `visual_servoing_dashboard.py` (avancé : vue
  caméra avec squelette FK-vérité + keypoints IA superposés + 6 courbes + barre
  d'état). Modes **live** (robot+caméra) et **rejeu** (dataset).
- **Courbe 1 — précision keypoints, SANS calibration** (`keypoint_accuracy_curve.py`) :
  synth ~2,9px/100% ; real_3cam proximaux 1,6px, distaux jusqu'à 10,5px. C'est « la
  courbe avant de calibrer » (évalue le modèle seul).
- **Self-calibration marker-free** codée (`self_calibrate_arducam.py`, RANSAC) mais
  **prouvée circulaire** : elle absorbe le biais DREAM → le squelette vérité suit
  les détections décalées, pas le vrai bras. → extrinsèque **indépendant (ArUco)
  nécessaire** pour un dashboard/courbe honnêtes.
- **`ik_reach_point.py`** validé sur robot réel : point → IK → angles (résidu IK
  0 mm), écart mécanique **1°** (le plancher physique).

### Décisions prises (soir)

- **Arducam mono écartée** pour la courbe en degrés (pas de depth + distaux faibles, 89%).
- **Astra = meilleur choix caméra unique** (95% in-domain + depth). Reste à régler son placement.
- Courbe en degrés + dashboard honnête **bloqués** tant qu'il n'y a pas d'extrinsèque
  indépendant (self-cal circulaire insuffisant).

### Prochaines actions (soir)

1. [ROUGE] **4 ArUco → extrinsèque indépendant** (IDs 19/23/25/26, une photo) —
   débloque dashboard honnête **et** courbe degrés, sur n'importe quelle caméra.
2. [ROUGE] Sinon **astra + fine-tune** sur le nouveau placement (seul chemin sans marqueurs).
3. [VERT] Puis `plot_angle_error_curve.py` (mode 3D astra / 2D arducam) + dashboard live.

### Commande rapide de reprise (soir)

```bash
source ~/ros_jazzy/venv_dream/bin/activate
cd ~/Osama_ws/src/mycobot_R6A/training/dream
# démo dashboard (rejeu — extrinsèque self-cal encore approximatif)
python3 visual_servoing_dashboard.py --mode replay --dataset dream_data/real_arducam \
  --weights checkpoints_dream/vgg_ultimate_v4_mix_ft_e30/best_network.pth \
  --extrinsic ../calibration/arducam_extrinsic_selfcal.yaml --intrinsics-npz ../calibration/cam_0.npz
```

---

## État actuel (8 juillet 2026 — après-midi)

### Ce qui a été accompli aujourd'hui

- **Sim-to-real DREAM comblé.** Le fine-tune mixte `vgg_ultimate_v4_mix_ft_e30`
  (`checkpoints_dream/vgg_ultimate_v4_mix_ft_e30/best_network.pth`) atteint
  **91,6% de détection réelle** (3 caméras, 1500 frames jamais vues), contre
  ≈27% pour le v4 synth-only et 99,4% en synthétique. Détail par keypoint :
  base/link1/link2 100%, link3 97,3%, link4 89,8%, link5 75,7%, link6 78,4% ;
  erreur médiane overall 2,91px. Les distaux (link5/6) restent le point faible
  relatif. Plan : `training/dream/FINETUNE_MIX_REAL3CAM_PLAN.md`.
- **Pipeline eye-to-hand construit et validé en sim** : `estimate_angles_from_keypoints.py`
  (modes 2D + 3D depth ; 3D récupère j1–j4 <2° sans amorçage, j6 non observable),
  grabber RGB-D `oni_grabber_rgbd.cpp` (depth aligné couleur via `/dev/shm`),
  calibration extrinsèque 3D `calibrate_astra_extrinsic_shm.py`, capture
  `capture_astra_rgbd.py`, courbe `plot_angle_error_curve.py`.
- **Dataset astra RGB-D capturé** : 90 poses (`dream_data/real_astra_rgbd`,
  color+depth+encodeurs), mouvement calqué sur capture_real_3cam.
- Docs à jour : `docs/ARCHITECTURE.md`, `CHANGELOG.md` (1.13.0), `CALIBRATION_ASTRA_EXTRINSIC.md`.

### Décisions prises

- **Direction pose estimation : eye-to-hand.** Une caméra **fixe devant le bras**
  observe tout le bras et estime sa pose (paradigme DREAM), pas eye-in-hand.
- **Livrable demandé par l'encadrant : courbe d'écart par joint** — comparer les
  angles estimés par la caméra (DREAM → keypoints → angles) aux encodeurs réels,
  joint par joint (j1…j6), en degrés.
- **Ordre de construction validé** : (1) calibration extrinsèque `T_base_camera`,
  (2) brique glue keypoints → angles (reprojection-min sur la FK existante),
  (3) courbe d'écart, (4) visual servoing pick-and-place — dans cet ordre, le
  servoing seulement une fois la courbe jugée exploitable.
- La cinématique nécessaire existe déjà : `training/dream/mycobot_fk.py` (FK +
  projection keypoints) et `training/dream/mycobot_ik.py` (IK position/pose).

### Prochaines actions — pour tracer la courbe d'écart sur le réel

1. **[ROUGE]** Remettre la lib DREAM `/tmp/DREAM` (effacée au reboot, aucune
   inférence possible sans elle) — re-cloner NVlabs DREAM.
2. **[ROUGE]** Calibrer l'extrinsèque astra : poser les 4 marqueurs sol, lancer
   `oni_grabber_rgbd` + `calibrate_astra_extrinsic_shm.py` → `astra_extrinsic.yaml`
   + `cam_astra.npz`. Viser < 5 mm de résidu (voir `CALIBRATION_ASTRA_EXTRINSIC.md`).
3. **[VERT]** Tracer : `plot_angle_error_curve.py` sur `dream_data/real_astra_rgbd`
   (90 poses déjà capturées) avec les poids `vgg_ultimate_v4_mix_ft_e30`.

### Commande rapide de reprise

```bash
# 1) grabber (terminal A)
cd ~/Osama_ws/src/mycobot_R6A/training/calibration && ./oni_grabber_rgbd
# 2) courbe (une fois /tmp/DREAM remis + calibration faite)
source ~/ros_jazzy/venv_dream/bin/activate
cd ~/Osama_ws/src/mycobot_R6A/training
python3 dream/plot_angle_error_curve.py \
  --dataset dream/dream_data/real_astra_rgbd \
  --weights checkpoints_dream/vgg_ultimate_v4_mix_ft_e30/best_network.pth \
  --extrinsic ../calibration/astra_extrinsic.yaml \
  --intrinsics-npz ../calibration/cam_astra.npz \
  --out dream/dream_data/real_astra_rgbd/angle_error_curve.png
```

---

## État actuel (23 avril 2026 — soir)

### 🧭 Reprise pour demain — lire en premier

La pose estimation (DREAM) est **l'urgence actuelle**. Les tests sur la tour et la simulation téléop + sorting sont validés.

**Question en attente pour demain** : on passe à l'option 2 (collecte de plus de données réelles, biaisée vers des poses bras étendu). Deux chemins possibles :

- **(a) recommandé** : garder `/tmp/dream_data/real_cam0/` intact (baseline mesurable) et capturer un nouveau dataset 5-10 K poses sous `real_cam0_v2` → merger avec le synthétique pour un `mixed_v2` → retrain.
- **(b)** : étendre `real_cam0` in-place.

Avant de capturer, ouvrir `training/capture_real.py` et identifier le sampler de poses — il faut biaiser vers des poses où J3/J4/J5 sont loin du repos pour que les distal keypoints couvrent plus de la grille de pixels.

Commande rapide pour confirmer l'état DREAM (doit reporter 47.3% de détection globale, link6 à 3.0% @ 62px) :
```bash
source ~/ros_jazzy/venv_dream/bin/activate
python training/dream/evaluate_dream.py \
  --weights training/checkpoints_dream/vgg_mixed_real_synth/best_network.pth \
  --data /tmp/dream_data/real_cam0 --split all
```

### Ce qui a été accompli aujourd'hui (23/04/2026)

#### 1. Diagnostic complet de la pose estimation DREAM

3 évaluations + visualisation sur 500 frames réelles issues de `/tmp/dream_data/real_cam0/` :

| Checkpoint / config | Détection globale | Médiane px | Diagnostic |
|---------------------|-------------------|------------|------------|
| `vgg_weighted_50k_e50` (synth-only) | 12.8 % | 128 | Mauvais — confirme que la data réelle est *load-bearing* |
| `vgg_mixed_real_synth` epoch 25 (baseline) | **47.3 %** | 2.95 | État d'avant-session |
| `vgg_mixed_real_synth` epoch 25 + threshold relaxé (0.001 au lieu de 0.01) | 49.8 % | 2.95 | Débloque link5 en détection (48%) mais la précision s'effondre (176px) — hypothèse "filtre de confiance trop strict" réfutée |
| `vgg_mixed_real_synth` epoch 50 (après option 1) | **47.3 %** | 2.78 | Détection inchangée, raffinement proximal seulement |

**Analyse par keypoint (mixed e50)** :

| Keypoint | Détection | Médiane px | Verdict |
|----------|-----------|------------|---------|
| base | 0 % | — | À investiguer (FK ? toujours occlu par le mount ?) |
| link1 | 100 % | 2.78 | ✅ Résolu |
| link2 | 100 % | 2.78 | ✅ Résolu |
| link3 | 89 % | 2.20 | 🟡 OK, quelques outliers |
| link4 | 36 % | 81 | ❌ Réseau perd la localisation |
| link5 | 3.8 % | 7.3 | ❌ Presque jamais détecté |
| link6 | 3.0 % | 62 | ❌ End-effector inutilisable pour pick-and-place |

**Visualisations sauvegardées** : `/tmp/dream_eval_viz_mixed/` (30 frames + montage).

**Hypothèse C confirmée** (réseau n'a pas appris les distal keypoints sur les images réelles) :
- Le bras est bien visible dans les frames échantillonnées (pas d'occlusion systématique)
- Les GT hollow circles sont à la bonne place sur le bras (FK / calibration OK)
- Les prédictions distal sont placées à des positions aléatoires hors bras, ou absentes

**Raison racine** : 2000 poses réelles uniques × 5 oversample, c'est suffisant pour les keypoints proximaux (stables dans l'image) mais pas pour link4/5/6 qui doivent être localisés sur toute la grille de pixels 640×480.

#### 2. Option 1 (entraînement prolongé sur la même data) — épuisée

Resume training `e25 → e50` sur les 18K mixed frames existantes (2.6 h, 25 époques × ~6 min) :

| Métrique | e25 | e50 | Delta |
|----------|-----|-----|-------|
| Détection globale | 47.3 % | 47.3 % | 0 |
| Val loss | 0.000334 | 0.000322 | −3.6 % (plateau après ~5 époques) |
| Train loss | 0.000225 | 0.000166 | −26 % (overfitting qui s'installe) |
| Per-frame mean error | 21.2 px | 14.2 px | ✓ |
| link6 | 1.2 % @ 263 px | 3.0 % @ 62 px | amélioration marginale |

La val loss a plafonné dès les premières époques du resume → l'information pour apprendre les distal keypoints n'est pas dans le dataset actuel. **Plus d'époques ne résout rien, seul plus de diversité le pourra.**

Backup du meilleur checkpoint d'avant-resume : `training/checkpoints_dream/vgg_mixed_real_synth/best_network.e25.pth`. Fichier de remplacement (post-resume) : `best_network.pth` pointe maintenant sur e50.

#### 3. Scaffold Claude Code (CLAUDE.md + `.claude/`)

Structure complète pour que les futures sessions Claude aient le contexte du projet dès le démarrage (pas de re-découverte à chaque session) :

- `CLAUDE.md` à la racine — project overview, 3 env Python, branch map, POC scope (digital twin · AI physics · VLA · pose estimation)
- `.claude/settings.json` — permissions partagées (per-user resté dans `settings.local.json`)
- `.claude/rules/` (5) — python-environments · ros2-conventions · real-robot-safety · git-branching · documentation
- `.claude/commands/` (5) — launch-sim · launch-teleop · real-robot-preflight · train-dream · collect-synthetic
- `.claude/skills/` (6) — teleop-troubleshoot · dream-workflow · gazebo-setup · real-robot-session · isaac-sim-integration · lerobot-dataset
- `.claude/agents/` (6) — ros2-debugger · dream-trainer · teleop-tuner · urdf-surgeon · digital-twin-engineer · vla-integrator
- `.claude/hooks/validate-ros2-build.sh` — inactif par défaut (à câbler via settings si désiré)

Le fichier `isaac-sim-integration/SKILL.md` contient la roadmap 5-phases pour Isaac Sim (USD conversion → ROS2 bridge → synthetic data for DREAM → parallel envs for VLA → real-robot validation). **Aucune migration réelle démarrée** — uniquement la planification. Gazebo reste sur `main`.

#### 4. Artefacts code nouveaux

- `training/dream/evaluate_dream_relaxed.py` — wrapper de `evaluate_dream.py` qui monkey-patch les seuils de peak detection (sans toucher `/tmp/DREAM/`). CLI : `--peak-thresh 0.001 --next-best-score 0.05`.

### Prochaines actions (par ordre)

1. **[ROUGE] Choisir (a) ou (b)** pour le dataset v2 (voir section "Reprise pour demain" ci-dessus)
2. **[ROUGE] Lire et adapter `training/capture_real.py`** pour biaiser le sampler vers les poses bras-étendu (J3 > 45°, J4 > 30°, J5 ≠ 0)
3. **[ROUGE] Capturer le nouveau dataset** (5-10K poses sur cam0 + cam3 avec FK safety obligatoire) — compter ~2-3 h caméra + preflight
4. **[JAUNE] Training mixte v2** — merger le nouveau `real_cam0_v2` avec le `synthetic_50k_v2` (world randomized_v2) → viser 30-40K mixed, 50 époques
5. **[JAUNE] Eval v2** — cible minimale pour débloquer pick-and-place : détection ≥ 70 % sur tous les keypoints, link6 médiane ≤ 10 px
6. **[VERT] Ensuite seulement** : envisager Isaac Sim ou self-supervised labeling selon le résultat

### État par checkpoint DREAM

| Checkpoint | Best val | Real det (overall) | link6 (det / med px) |
|------------|----------|---------------------|-----------------------|
| `vgg_weighted_50k_e50` | ~0.00019 | 12.8 % | 5.6 % / 395 |
| `vgg_mixed_real_synth` e25 (backup) | 0.000334 | 47.3 % | 1.2 % / 263 |
| **`vgg_mixed_real_synth` e50 (best)** | **0.000322** | **47.3 %** | **3.0 % / 62** |

---

## État précédent (22 avril 2026 — soir)

### ✅ MILESTONE : premier test physique réussi

Le pipeline complet de téléopération main a été **validé sur le MyCobot 320 Pi physique** (IP 10.10.0.221) dans la session du 22/04/2026 soir. Chaîne testée :

```
👋 Main opérateur → Astra S → Wilor → mapping → filtres
    → rosbridge → /mycobot_controller/joint_trajectory
    → trajectory_to_robot_bridge → JSON /to_robot
    → bridge_tour (TCP) → Pi 10.10.0.221:5005
    → bridge_pi_simple.py → pymycobot → servos → 🦾
```

**Résultats mesurés** :
- Latence main→bras ~150–250 ms, imperceptible visuellement
- Mouvements coordonnés, pas d'oscillation, pas de saturation
- Gains initiaux 0.6/0.6/0.6 + tfs 0.3 + speed 25 (voir [`docs/REAL_ROBOT_TEST_PROCEDURE.md`](docs/REAL_ROBOT_TEST_PROCEDURE.md) § protocole sécurisé)

### Outils livrés pour la téléop

Dans [`teleop/`](teleop/) (env conda `hand-teleop`, Python 3.10) :
- [`mycobot_teleop.py`](teleop/mycobot_teleop.py) — script principal caméra → joints via rosbridge · publie aussi `/teleop/camera/image` pour le dashboard
- [`teleop_dashboard.py`](teleop/teleop_dashboard.py) — **GUI ABMI v2.2** (navy+pink) · 3 onglets (🏠 Home · 📊 Analytics · 🎛️ Tuning) · KPI cards · caméra intégrée · ActionButton dynamiques · presets de gains
- [`performance_analyzer.py`](teleop/performance_analyzer.py) — rapport Excel avec verdict READY/CAUTIOUS/NOT READY
- [`orbbec_capture.py`](teleop/orbbec_capture.py) — wrapper Astra shared-memory via `oni_grabber`
- [`assets/abmi_logo.png`](teleop/assets/) — logo chargé par le dashboard

Dans [`mycobot_gateway/`](mycobot_gateway/) (env ROS2 Jazzy, Python 3.12) :
- `trajectory_to_robot_bridge` — JointTrajectory (rad) → JSON send_angles (deg) pour le Pi
- `gripper_to_robot_bridge` — *(prêt, non câblé : robot actuel sans gripper physique)*
- `bridge_tour` — TCP client vers Pi (IP paramétrable)
- `mycobot_teleop.launch.py` — launch `target:={sim,real,both}`

Dans [`scripts/`](scripts/) :
- `real_robot_preflight.sh` — check pré-vol 5 étapes (ping, TCP, ROS2, bridge_tour round-trip, get_angles)

### Documentation

- [`docs/TELEOPERATION.md`](docs/TELEOPERATION.md) — pipeline complet, filtres, mapping, historique
- [`docs/TELEOP_ARCHITECTURE_VIZ.md`](docs/TELEOP_ARCHITECTURE_VIZ.md) — **visuel détaillé** : détection → mouvement (types, unités, latences)
- [`docs/TELEOP_DASHBOARD.md`](docs/TELEOP_DASHBOARD.md) — manuel utilisateur du dashboard
- [`docs/TELEOP_TUNING.md`](docs/TELEOP_TUNING.md) — référence paramètres + dépannage
- [`docs/REAL_ROBOT_TEST_PROCEDURE.md`](docs/REAL_ROBOT_TEST_PROCEDURE.md) — procédure + protocole calibration sécurisé

**Status checklist** :
- ✅ Pipeline complet en simulation (Astra → Wilor → filtres → Gazebo JTC)
- ✅ Dashboard ABMI v2.2 (3 onglets · KPI cards · caméra intégrée · ActionButton dynamiques) + rapport Excel
- ✅ Filtres R5A/LeRobot portés (EMA + slew 1°/f + gripper deadband chain)
- ✅ **Pipeline réel validé** (22/04/2026, IP 10.10.0.221)
- ✅ Preflight + procédure documentés
- ⚠️ Axe J6 (doorknob) : mapping `yaw` implémenté, validation visuelle toujours à faire
- ⚠️ bridge_tour receive_loop : n'affiche pas les `📥 Reçu de Pi` (Pi envoie bien mais logging Tower absent) — non bloquant
- ⚠️ Pince Gazebo : limitation cosmétique (4-bar linkage pas reproductible sous DART)
- ℹ️ Robot physique actuel **sans gripper** — flag `--no-gripper` obligatoire
- 🔜 **Prochain** : tuning fin des gains sur robot réel, test performance_analyzer en conditions réelles

### Problème DREAM toujours ouvert : Domain gap sim-to-real

Le modèle DREAM VGG atteint **97% de détection à 3.1px médiane** sur les données synthétiques, mais seulement **~26% de détection sur les images réelles**. Les pics des belief maps sont 10× plus faibles sur les images réelles que sur les synthétiques.

### Ce qui est fait

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

### Ce qui reste à faire

1. **[ROUGE] Évaluer le modèle mixte** sur données réelles :
   ```bash
   source ~/ros_jazzy/venv_dream/bin/activate
   python training/dream/evaluate_dream.py \
     --weights training/checkpoints_dream/vgg_mixed_real_synth/best_network.pth \
     --data /tmp/dream_data/real_cam0 --split all
   ```

2. **[ROUGE] Si detection < 50%** sur réel → implémentation self-supervised labeling :
   - FK + angles joints lus → keypoints 3D → projection 2D → annotations GT automatiques
   - Fine-tune sur ces données réelles auto-annotées

3. **[JAUNE] Vérifier collecte 30K synth v2** dans `/tmp/dream_data/synthetic_50k_v2/`

4. **[VERT] Pick-and-place Gazebo (mono-objet)** : pipeline `pick_and_place.launch.py` validé
   et étendu en multi-objet par couleur (`pick_and_place_sorting.launch.py`)

5. **[VERT] Bench test robot réel** une fois detection > 50%

---

## Résultats DREAM par keypoint (VGG-aug, synthétique)

| Keypoint | Détection | Médiane px | Médiane mm | Erreur angulaire |
|----------|-----------|------------|------------|------------------|
| base | 100% | 2.8 px | 4.0 mm | ~0.7° |
| link1 | 100% | 2.6 px | 3.7 mm | ~0.7° |
| link2 | 100% | 2.6 px | 3.7 mm | ~0.7° |
| link3 | 99% | 5.6 px | 8.1 mm | ~2.1° |
| link4 | 96% | 6.4 px | 9.2 mm | ~3.4° |
| link5 | 95% | 8.8 px | 12.7 mm | ~8.7° |
| link6 | 86% | 10.1 px | 14.6 mm | ~18.3° |
| **TOTAL** | **97%** | **3.1 px** | **4.5 mm** | **~0.8°** |

> 1 px = 1.44 mm (caméra à 0.8m, fx=554.38)

### Comparaison approches

| Approche | Erreur angulaire (synthétique) |
|----------|-------------------------------|
| Phase 1 — ResNet50 multi-view | 12.97° MAE |
| **Phase 2 — DREAM VGG-aug** | **~0.8° médiane / ~3.4° moyenne** |

### Adéquation pick-and-place (exigence ±5mm)

| Zone | Erreur | Statut |
|------|--------|--------|
| Joints proximaux (base→link2) | 3.9 mm | ✅ OK |
| Joints intermédiaires (link3–link4) | 8–9 mm | ⚠️ Limite |
| End-effector (link6) | 14.6 mm | ❌ Insuffisant (besoin ~3×) |

---

## Chemins importants

| Ressource | Chemin |
|-----------|--------|
| Projet | `/home/genji/Osama_ws/src/mycobot_R6A/` |
| Venv DREAM | `/home/genji/ros_jazzy/venv_dream/` |
| DREAM lib | `/tmp/DREAM/` |
| Data synth 50K | `/tmp/dream_data/synthetic_50k/` |
| Data réel | `/tmp/dream_data/real_cam0/` |
| Data mixte | `/tmp/dream_data/mixed_real_synth/` |
| Meilleur modèle (synth) | `training/checkpoints_dream/vgg_weighted_50k_e50/best_network.pth` |
| Modèle mixte | `training/checkpoints_dream/vgg_mixed_real_synth/best_network.pth` |

---

## Commandes utiles

### Prérequis

```bash
# Éviter conflit Conda ↔ ROS2 (Python 3.13 vs 3.12)
conda deactivate
```

### Démarrage bridge Pi

```bash
ssh er@10.10.0.225
# Terminal 1 : robot
python3 bridge_pi_simple.py
# Terminal 2 : caméras
python3 pi_camera_server.py --cameras 0 3 --names cam0 cam3
```

### Évaluation DREAM

```bash
source ~/ros_jazzy/venv_dream/bin/activate

# Sur données réelles
python training/dream/evaluate_dream.py \
  --weights training/checkpoints_dream/vgg_mixed_real_synth/best_network.pth \
  --data /tmp/dream_data/real_cam0 --split all

# Sur données synthétiques (vérification de régression)
python training/dream/evaluate_dream.py \
  --weights training/checkpoints_dream/vgg_mixed_real_synth/best_network.pth \
  --data /tmp/dream_data/synthetic_50k --split val
```

### Collecte données synthétiques v3

```bash
conda deactivate
source /opt/ros/jazzy/setup.bash
source ~/Osama_ws/install/setup.bash

# Monde randomized_v2 (6 lights, 12 objets clutter)
ros2 launch mycobot_gateway synthetic_data_v3.launch.py num_samples:=7500
```

### Merge + conversion NDDS + pipeline d'entraînement

```bash
# Script automatisé complet
bash scripts/train_pipeline.sh

# Ou manuellement :
source ~/ros_jazzy/venv_dream/bin/activate
python training/dream/merge_and_convert.py \
  --real /tmp/dream_data/real_cam0 \
  --synth /tmp/dream_data/synthetic_50k \
  --output /tmp/dream_data/mixed_v2 \
  --real-oversample 5

python /tmp/DREAM/scripts/train_network.py \
  -i /tmp/dream_data/mixed_v2 \
  -m /tmp/DREAM/manip_configs/mycobot320.yaml \
  -ar /tmp/DREAM/arch_configs/dream_vgg_q.yaml \
  -e 25 -b 32 -lr 0.0001 \
  -o training/checkpoints_dream/vgg_mixed_v2 -f
```

### Pick-and-place simulation

```bash
conda deactivate
source /opt/ros/jazzy/setup.bash
source ~/Osama_ws/install/setup.bash

# Mono-objet (cube rouge → bac vert)
ros2 launch mycobot_gateway pick_and_place.launch.py

# Multi-objet par couleur (4 objets → 4 bacs)
ros2 launch mycobot_gateway pick_and_place_sorting.launch.py
# Variantes :
ros2 launch mycobot_gateway pick_and_place_sorting.launch.py use_detector:=false
ros2 launch mycobot_gateway pick_and_place_sorting.launch.py process_order:=blue,green
```

---

## Architecture du système

```
┌─────────────────────────────────────────────────────────────────────────────┐
│                              PC TOUR (10.10.0.115)                          │
│                         ROS2 Jazzy / Ubuntu 24.04 / Python 3.12             │
│                         Conda: Python 3.13 / PyTorch 2.6 + CUDA 12.4       │
│                         GPU: NVIDIA RTX 4000 Ada (20 GB VRAM)               │
├─────────────────────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐  ┌──────────────┐       │
│  │ simple_gui  │  │slider_control│  │teleop_keyb. │  │ training/    │       │
│  │  (Tkinter)  │  │(joint_states)│  │  (clavier)  │  │ dream/       │       │
│  └──────┬──────┘  └──────┬──────┘  └──────┬──────┘  └──────┬───────┘       │
│         └────────────────┴────────────────┘                │              │
│                          │                          TCP:5005 + 5006        │
│                   /to_robot (JSON)                         │              │
│                          ▼                                 ▼              │
│                ┌─────────────────┐                                         │
│                │   bridge_tour   │                                         │
│                └────────┬────────┘                                         │
├─────────────────────────┼───────────────────────────────────────────────────┤
│                   RÉSEAU ETHERNET (10.10.0.x)                              │
├─────────────────────────┼───────────────────────────────────────────────────┤
│                         ▼                                                   │
│           ┌─────────────────┐       ┌─────────────────┐                    │
│           │bridge_pi_simple │       │pi_camera_server │                    │
│           │  TCP:5005       │       │  TCP:5006       │                    │
│           └────────┬────────┘       └────────┬────────┘                    │
│                    ▼                         ▼                             │
│           ┌─────────────────┐       ┌─────────────────┐                    │
│           │    pymycobot    │       │ Arducam USB ×2  │                    │
│           │  /dev/ttyAMA0   │       │  cam0 + cam3    │                    │
│           └────────┬────────┘       └─────────────────┘                    │
│                    ▼                                                       │
│           ┌─────────────────┐                                              │
│           │  MyCobot 320 Pi │                                              │
│           └─────────────────┘                                              │
│                     RASPBERRY PI (10.10.0.225)                             │
└────────────────────────────────────────────────────────────────────────────┘
```

---

## Structure du projet

```
mycobot_R6A/
├── SESSION_RESUME.md              # Ce fichier
├── DEVELOPMENT_SUMMARY.md         # Résumé technique détaillé
├── CHANGELOG.md                   # Historique des versions
├── README.md                      # README principal
│
├── mycobot_gateway/               # Package ROS2 — contrôle + vision
│   ├── mycobot_gateway/
│   │   ├── bridge_tour.py         # Client TCP vers Pi
│   │   ├── simple_gui.py          # GUI Tkinter
│   │   ├── slider_control.py      # Contrôle sliders temps réel
│   │   ├── dream_inference_node.py# Inférence DREAM + PnP ROS2
│   │   ├── pick_and_place_node.py # State machine pick & place
│   │   └── synthetic_data_collector_v2.py
│   ├── scripts/
│   │   ├── bridge_pi_simple.py    # Script Pi (serveur TCP robot)
│   │   └── pi_camera_server.py    # Script Pi (serveur TCP caméras)
│   └── launch/
│       ├── pick_and_place.launch.py
│       ├── synthetic_data_v2.launch.py
│       └── synthetic_data_v3.launch.py
│
├── mycobot_description/           # Package ROS2 — URDF/Gazebo
│   ├── urdf/320_pi/               # Modèle 3D + 4 caméras Gazebo
│   │   └── link6_2022.dae         # Mesh link6 pour compat. gripper
│   ├── urdf/pro_adaptive_gripper/ # Gripper adaptatif (meshes STL)
│   └── worlds/
│       ├── randomized.sdf         # Monde de base
│       └── randomized_v2.sdf      # 6 lights + 12 clutter objects
│
├── training/                      # Pipeline ML/IA
│   ├── train.py / model.py        # Legacy: régression directe (abandonné)
│   ├── capture_real.py            # Capture réelle avec FK safety
│   └── dream/                     # DREAM keypoint detection (actif)
│       ├── mycobot_fk.py          # FK + projection (7 keypoints)
│       ├── convert_to_ndds.py     # Conversion → format NDDS
│       ├── merge_and_convert.py   # Fusion datasets + conversion
│       ├── evaluate_dream.py      # Évaluation (filtre sentinel -999.99)
│       ├── infer_dream.py         # Inférence + PnP solving
│       ├── finetune_real.py       # Fine-tuning expérimental (⚠️ ne fonctionne pas)
│       └── manip_configs/mycobot320.yaml
│
├── scripts/
│   ├── train_pipeline.sh          # Pipeline merge→NDDS→training automatisé
│   └── monitor_collection.sh      # Suivi collecte en temps réel
│
└── docs/                          # Documentation détaillée
```

---

## Points importants

1. **Conda vs ROS2** : Toujours `conda deactivate` avant ROS2. Training ML → `/home/genji/miniconda/bin/python3` ou venv_dream.
2. **Fine-tuning DREAM custom** : Deux tentatives ont échoué (σ mismatch + belief map collapse). Utiliser uniquement `train_network.py` natif.
3. **Sentinel DREAM** : DREAM renvoie -999.99 quand peak < seuil — filtrer dans l'évaluation.
4. **VGG sans BatchNorm** est stable. ResNet+BN explose avec batch_size < 64.
5. **1 px = 1.44 mm** (caméras latérales à 0.8m, fx=554.38).
6. **Gripper** : intégré dans la simulation mais joints fixés (pas de support `mimic` dans Gazebo Harmonic).

---

## Documentation

| Fichier | Description |
|---------|-------------|
| [SESSION_RESUME.md](SESSION_RESUME.md) | Ce fichier — point de départ |
| [DEVELOPMENT_SUMMARY.md](DEVELOPMENT_SUMMARY.md) | Résumé technique complet |
| [CHANGELOG.md](CHANGELOG.md) | Historique des versions |
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | Architecture détaillée |
| [docs/SYNTHETIC_DATA.md](docs/SYNTHETIC_DATA.md) | Pipeline données synthétiques |
| [training/README.md](training/README.md) | Pipeline ML / DREAM |
| [training/dream/README.md](training/dream/README.md) | Module DREAM |

---

*Dernière mise à jour : 21 avril 2026*
