# SESSION RESUME — MyCobot 320 Pi R6A

> **Date de dernière mise à jour :** 28 avril 2026 (après-midi — test cheap cam3 dans le mix, signal clair mais calibration manquante)
> **Version :** 2.2.0 (téléop) · 1.10.0 (sorting) · 1.13.0 (test mixte cam0+cam3)
> **Branche active :** `main`
> **Repository :** https://github.com/ABMI-software/mycobot_320pi_R6A
> **Pi réelle :** `10.10.0.223` (pas `.225` comme certains anciens docs)

---

## Point de départ rapide

```bash
# TOUJOURS exécuter avant ROS2
conda deactivate

source /opt/ros/jazzy/setup.bash
source ~/ros_jazzy/src/mycobot_R6A/install/setup.bash
```

---

## État actuel (28 avril 2026 — après-midi)

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

Le pipeline complet de téléopération main a été **validé sur le MyCobot 320 Pi physique** (IP 10.10.0.223) dans la session du 22/04/2026 soir. Chaîne testée :

```
👋 Main opérateur → Astra S → Wilor → mapping → filtres
    → rosbridge → /mycobot_controller/joint_trajectory
    → trajectory_to_robot_bridge → JSON /to_robot
    → bridge_tour (TCP) → Pi 10.10.0.223:5005
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
- ✅ **Pipeline réel validé** (22/04/2026, IP 10.10.0.223)
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
| 28/04/2026 | Install `pandas` dans `venv_dream` | ✅ |
| 28/04/2026 | DREAM eval (a) strict réel — 47.3 % det confirmé | ✅ Baseline 1.11.0 reproduit |
| 28/04/2026 | DREAM eval (b) strict synth val — 91.9 % det | ✅ Régression contrôlée vs synth-only |
| 28/04/2026 | DREAM eval (c) relaxed réel (peak=0.001) — 48.0 % | ❌ Médianes explosées, hypothèse réfutée |
| 28/04/2026 | Verdict diagnostic complet : distal keypoints = bottleneck | ✅ Cf. CHANGELOG 1.12.0 |
| 28/04/2026 (PM) | Convert cam3 → NDDS (extrinsèques approximatives) | ✅ 2000 frames |
| 28/04/2026 (PM) | Eval croisée v1 sur cam3 : 25.1 % / 237 px d'erreur | ✅ Confirme zéro cross-view generalization |
| 28/04/2026 (PM) | Build `mixed_v2_cam03` (18K) + retrain 25 epochs | ✅ 2h35, val=0.000356 |
| 28/04/2026 (PM) | Eval v2 : cam0 -7.1 pts, cam3 +10 pts, synth +1.2 pts | 🟰 Trade-off, calibration cam3 nécessaire |

### Ce qui reste à faire

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
| Projet | `/home/genji/ros_jazzy/src/mycobot_R6A/` |
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
source ~/ros_jazzy/src/mycobot_R6A/install/setup.bash

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
source ~/ros_jazzy/src/mycobot_R6A/install/setup.bash

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
