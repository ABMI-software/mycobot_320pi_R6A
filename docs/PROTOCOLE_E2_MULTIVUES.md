# Protocole E2 — jeu réel multi-vues pour l'article DREAM

Expérience E2 de [`ARTICLE_DREAM.md`](ARTICLE_DREAM.md). Méthode :
`.claude/skills/campagne-mesure`. Rédigé le 10/11/2026, **avant toute mesure** :
les critères ci-dessous ne se recalent pas sur le résultat. Amendé le 19/11,
toujours avant mesure : intrinsèques refaites **dans la session** (étape 0b), après
avoir constaté que celles du jour ne décrivaient plus les caméras du 13/10.

## Ce que le 19/11 a appris, et qui fixe la méthode

- **Une intrinsèque ne survit pas aux semaines.** Le 19/11, la SVPRO mesure
  fx = 394 en 640 × 480 ; sur les images du 13/10, les marqueurs de la planche
  s'ajustent au mieux vers fx ≈ 430, et mieux avec l'ancienne `cam_2` qu'avec la
  neuve. Son objectif est à focale variable : une bague touchée, et l'étiquette
  est fausse sans que rien ne le signale. D'où : calibrer **le jour même**,
  **bloquer** mise au point et bague à l'adhésif, **revérifier en fin de session**.
- **La mise au point seule pèse peu** : focus 40 → fx 394,2, focus 90 → 397,2
  (0,8 %). On la fixe quand même, pour la netteté et la traçabilité.
- **La planche ChArUco du banc** : 11 × 8, carrés 30 mm, marqueurs 22 mm,
  identifiants 8 à 51, ancienne disposition — d'où `--first-marker-id 8
  --legacy-pattern`, sous `.venv/bin/python`.
- **Un ajustement ne prouve rien sur sa propre focale en vue zénithale** : sur
  l'arducam, le profil RMS(fx) des 4 marqueurs est plat. Le contrôle de focale
  par les marqueurs (étape 2) ne vaut que pour les vues obliques.

## Pourquoi ce jeu

E3 affirme qu'un DREAM entraîné sur le seul synthétique se transfère au banc
**depuis des points de vue jamais vus**. Les jeux réels actuels ne peuvent pas
le montrer : `real_tri_test` vient des deux montages fixes, ceux que
`vgg_tri_reel_ft_e10` a mémorisés (le `base` constant par montage du 02/09). Il
faut un jeu où la caméra est à des places nouvelles, étiqueté sans DREAM.

E2 produit **un jeu figé**, pas un chiffre DREAM. Les modèles ne s'évaluent
dessus qu'en E3.10, avec la revue du metrologue.

## Référence et ce qu'elle ne voit pas

- **Label 2D** = FK des **angles lus** (codeurs, bras arrêté), projetée par
  l'extrinsèque ArUco de la position, avec la distorsion
  (`cv2.projectPoints`). Aucune sortie DREAM n'entre dans le label.
- **Extrinsèque** : 4 ArUco de la planche, positions de
  `training/calibration/planche_actuelle.yaml` (19 et 23 mesurés **au
  robot**, 25 et 26 déduits). Jamais « Relever la planche » (`--reference`),
  qui referme le cercle.
- **Ce que la référence ne voit pas** :
  - l'extrinsèque ne vaut que dans le plan Z = 0, or les keypoints sont en
    hauteur (§ « Limites de la référence » de
    [`PROTOCOLE_REEL.md`](PROTOCOLE_REEL.md)) → contrôle en hauteur à l'étape 2 ;
  - la flexion en aval des codeurs (affaissement de 13-15 mm à la pointe) ;
  - le placement des 7 keypoints dans les maillages (même modèle FK qu'en
    synthétique : erreur commune, invisible ici).
- **Budget d'étiquette** : 1 mm d'erreur 3D vaut ~0,69 px à 0,75 m pour
  l'arducam (f = 518 px, 19/11), ~0,53 px pour la SVPRO (f = 394 px, 19/11). Un PCK@2,5
  n'a de sens que si l'incertitude d'étiquette reste nettement sous 2,5 px :
  c'est elle qui fixe le seuil de l'étape 2.

## Plan des positions

**6 à 10 positions de caméra**, chacune fixée sur trépied ou bras articulé,
jamais sur les supports des deux montages actuels. Couvrir :

| Contrainte | Pourquoi |
|---|---|
| ≥ 2 positions dans le secteur d'azimut **60-100°**, tenu à l'écart de l'entraînement synthétique (E3.7) | Le point de vue qui n'est vu ni en réel, ni en synthétique |
| ≥ 3 positions hors de ce secteur, réparties sur l'azimut atteignable | Généralisation hors secteur |
| Distances de 0,7 à 1,45 m, au moins une sous 0,9 m et une au-delà de 1,2 m | La plage de la caméra libre synthétique |
| Au moins une vue rasante (élévation < 25°) et une plongeante (> 55°) | Les deux régimes de PnP |
| Optionnel, **P0** = montage arducam actuel | Témoin, relie E2 à `real_tri_test` ; **ne compte pas** parmi les vues jamais vues |

Azimut, distance et élévation se calculent depuis l'extrinsèque par
`training/dream/geometrie_camera.py`, dans la convention du collecteur
synthétique (azimut autour du point visé, par la fonction même qui découpe le
secteur tenu) : une position réelle « dans le secteur » l'est au même sens
qu'une trame synthétique.

Les deux montages actuels, mesurés le 10/11 :

| Montage | Azimut | Distance | Élévation | Secteur tenu |
|---|---|---|---|---|
| arducam (`arducam_extrinsic_pick.yaml`) | −109° | 0,88 m | 74° | non |
| SVPRO (`svpro_extrinsic_servo.yaml`, `…_labels.yaml`) | 70° | 0,98 m | 25° | **oui** |

La SVPRO fixe est donc déjà dans le secteur 60-100°. Les positions E2 de ce
secteur doivent s'en écarter nettement (≥ 15° d'azimut ou ≥ 20° d'élévation),
sinon elles ne sont pas « jamais vues » pour un modèle affiné sur
`real_tri/train`.

Les deux caméras (arducam, SVPRO) peuvent servir, une position chacune à la
fois, avec les intrinsèques **de la session** (étape 0b), jamais `cam_2` /
`cam_3` du registre. **Mise au point et bague de focale bloquées** (aucun
autofocus) : sans quoi les intrinsèques ne valent plus.

## Poses du bras

- **Les mêmes poses à toutes les positions** : même générateur que
  `real_tri/test` (`training/capture_real_tri.py`, pointe au-dessus de la
  planche, gardes du FSM), une seule graine **2001**, jamais utilisée en
  entraînement (train 1-3, test 1002). L'écart entre positions mesure alors le
  point de vue seul, à poses appariées.
- **50 poses par position** ; une pose où le bras n'est pas immobile ou dont un
  keypoint sort de l'image n'est pas remplacée, elle est consignée.
- J5 relevé à chaque pose : les résultats se donneront aussi dans l'enveloppe
  de J5 où le réel est fiable.

## Étape 0 — Préparation du banc

- **Objectif** : un banc dans l'état où la référence est valide.
- **Actions** : Pi identifiée par un aller-retour TCP sur 5005,
  `gripper_bridge.py` sur la Pi — **aucun script d'essai** de `~/Test-JB/Essaies`
  ne tourne en même temps (ils ouvrent `/dev/ttyAMA0` en direct) —,
  `scripts/real_robot_preflight.sh $MYCOBOT_PI`, puis arrêt du `bridge_tour`
  qu'il laisse. **Pince montée** (pas le stylo des essais E1 : `tool_offset.json`
  décrit la pince). Planche dégagée (aucune feuille sur les marqueurs).
  Contrôle de la planche : pince sur 19 et 23, lecture des codeurs sans ordre
  moteur ; écart ≤ 2 mm à `planche_actuelle.yaml`, sinon remesurer au robot et
  réécrire la référence (le 19/11, l'arducam voyait les marqueurs à 250 mm de
  la référence : banc réarrangé pour les essais au stylo).
- **Consigner** : date, opérateur, IP de la Pi, commit, écart 19/23, éclairage
  (lampes, fenêtres, heure), température.
- **Critère de sortie** : preflight OK, planche à ≤ 2 mm.
- **Ne pas faire** : relever la planche à travers une caméra.

## Étape 0b — Intrinsèques du jour (par caméra, une fois par session)

- **Objectif** : des intrinsèques qui décrivent la caméra telle qu'elle capture.
- **Actions** : mise au point manuelle réglée sur le bras (SVPRO : 40 ou 90,
  noter la valeur), puis bague et molette **bloquées à l'adhésif**, photo. Mire
  ChArUco du banc, 640 × 480, 35 vues :
  ```bash
  env -u GTK_PATH -u GTK_EXE_PREFIX -u LOCPATH -u GIO_MODULE_DIR \
      -u GSETTINGS_SCHEMA_DIR -u GTK_IM_MODULE_FILE \
    .venv/bin/python training/calibration/calibrate_camera.py \
      --camera <index> --name e2_<AAAAMMJJ>_<camera> --preset none \
      --width 640 --height 480 --fourcc MJPG --autofocus off --focus <valeur> \
      --squares-x 11 --squares-y 8 --square-length-m 0.03 --marker-length-m 0.022 \
      --first-marker-id 8 --legacy-pattern --max-per-cell 4 --target-samples 35
  ```
  (les variables GTK du snap retirées, sans quoi la fenêtre ne s'ouvre pas).
  Réajuster au modèle standard (k1 k2 p1 p2 k3) sur les coins gardés dans le
  `.npz`, comme le 19/11, et l'écrire en `training/calibration/e2_<date>_<camera>.meta.json`.
- **Consigner** : valeur de focus, photo de l'adhésif, RMS, fx ± écart-type,
  cx, cy, vues par case de la grille 4 × 3.
- **Critère de sortie** : RMS ≤ 0,5 px ; **≥ 2 vues dans chacune des 12 cases**
  (le 19/11, le haut de l'image de l'arducam en manquait) ; écart-type de fx
  ≤ 0,5 %. Sinon recapturer.
- **Ne pas faire** : reprendre une intrinsèque d'une autre session, même
  « récente » ; toucher la bague après la mire.

## Étape 1 — Poser la caméra (par position)

- **Objectif** : une vue nouvelle, avec les 4 marqueurs exploitables.
- **Actions** : fixer la caméra, serrer, vérifier dans l'aperçu que les 4
  ArUco sont décodés et à ≥ 10 px du bord, et que le bras reste dans l'image
  sur les poses extrêmes (`capture_real_tri.py --plan` donne les pointes).
- **Consigner** : identifiant `Pxx`, caméra, support, photo du montage.
- **Critère de sortie** : 4 marqueurs décodés sur ≥ 90 % de 60 images.
- **Ne pas faire** : toucher la caméra une fois l'étape 2 commencée.

## Étape 2 — Extrinsèque et contrôle d'étiquette (par position)

- **Objectif** : une extrinsèque dont l'erreur, au sol et en hauteur, tient
  dans le budget d'étiquette.
- **Actions** :
  1. 60 images, `scripts/calibrer_extrinseque_4aruco.py --intrinsics
     training/calibration/e2_<date>_<camera>.meta.json --frames 60` (16 coins,
     taille mesurée, leave-one-marker-out), positions de
     `planche_actuelle.yaml` ; sortie
     `training/calibration/e2_Pxx_<camera>_extrinsic.yaml` (non commité), qui
     porte `intrinsics_stem` : capture, conversion et `eval_papier.py` le
     relisent.
  1b. **Contrôle de focale par les marqueurs** (vues obliques, élévation
     < 60°) : profil du RMS d'ajustement en fonction de fx (± 10 %), méthode
     du 19/11 (`~/dream_article/E3/E3_9c/`). Le minimum doit tomber à ± 3 % du
     fx de l'étape 0b ; sinon la caméra a changé depuis la mire.
  2. **Contrôle en hauteur** : un ArUco de 50 mm sur un bloc rigide, posé à
     ~100 mm, ~200 mm et ~280 mm de haut (hauteur de `link3`-`link5`), sa
     position relevée **au robot** (pince sur son centre, codeurs lus) ; on
     mesure l'écart entre sa projection et sa détection. Ajouté le 18/11 :
     sur `real_tri`, l'erreur d'étiquette des points en hauteur n'a pas pu
     être bornée (jusqu'à ~10 px possibles, `docs/ARTICLE_DREAM.md`).
  3. Squelette FK projeté sur 3 poses (dont la plus haute) : contrôle à l'œil.
- **Consigner** : RMS, leave-one-out par marqueur (mm), écart du marqueur
  haut (px et mm), azimut / distance / élévation calculés.
- **Critère de sortie** : leave-one-out ≤ 3 mm sur chaque marqueur, écart
  du marqueur haut ≤ 2 px, squelette sur le bras aux 3 poses. Sinon : repositionner
  et refaire l'étape 1 ; une position qui échoue deux fois est abandonnée et
  consignée comme telle.
- **Ne pas faire** : publier le RMS comme une précision (c'est un résidu) ;
  passer à la capture sur un leave-one-out hors seuil « parce que le squelette
  a l'air bon ».

## Étape 3 — Capture (par position)

- **Objectif** : 50 poses appariées, images et codeurs.
- **Actions** : bras arrêté à chaque prise, angles mesurés et non commandés :
  ```bash
  /usr/bin/python3 training/capture_real_tri.py --jeu e2 --poses 50 --graine 2001 \
      --extrinseque arducam=training/calibration/e2_Pxx_arducam_extrinsic.yaml \
      --sortie training/dream/dream_data/real_e2/Pxx
  ```
  Seule la caméra nommée capture ; l'extrinsèque utilisée est écrite dans
  `sessions.json`. `--plan` tire et résout les poses sans bouger.
- **Consigner** : `labels.csv`, `keypoints_2d.csv`, images brutes, poses non
  atteintes et leur raison.
- **Critère de sortie** : ≥ 45 poses valides sur 50.
- **Ne pas faire** : relancer une seule pose ratée dans une autre session ; la
  série d'une position se fait d'un trait.

## Étape 4 — La caméra a-t-elle bougé ? (par position)

- **Objectif** : savoir si l'extrinsèque vaut encore à la fin de la capture.
- **Actions** : bras en pose de dégagement, 60 nouvelles images, nouvelle
  extrinsèque, écart à celle de l'étape 2.
- **Critère de sortie** : ≤ 1 mm et ≤ 0,1° ; sinon la position est
  **invalide et se rejoue en entier** (étapes 1 à 4), elle ne se complète pas.
- **En fin de session**, par caméra : 15 vues de mire (même commande que 0b,
  `--target-samples 15`, autre `--name`), adhésif intact. fx à ± 1 % de
  l'étape 0b, sinon **toutes les positions de cette caméra depuis la dernière
  vérification sont invalides**.

## Étape 5 — Figer le jeu

- **Actions** : `training/dream/convert_real_tri_ndds.py <Pxx> <sortie>
  --cameras <camera> --extrinseque <camera>=training/calibration/e2_Pxx_<camera>_extrinsic.yaml`
  par position (l'intrinsèque suit par `intrinsics_stem`, écrite dans chaque
  label) ; md5 de chaque image ; contrôle
  qu'aucune image n'est commune avec `real_tri_train_ndds` ni
  `real_tri_test_ndds` ; 10 images par position tirées au hasard, squelette
  projeté, revues à l'œil.
- **Consigner** : `run.yaml` de E2 (commit, `git status`, extrinsèques,
  intrinsèques, graine, positions et leur géométrie, md5), archivé avec les
  données brutes sous `~/dream_article/E2/`, hors du dépôt.
- **Critère de sortie de E2** :
  - ≥ 6 positions valides, dont ≥ 2 dans le secteur 60-100° ;
  - ≥ 45 poses valides par position ;
  - leave-one-out et contrôle en hauteur publiés par position ;
  - md5 consignés, aucun recouvrement avec les jeux d'entraînement.
- **Ne pas faire** : évaluer un modèle sur le jeu avant qu'il soit figé ; ajouter
  ou retirer une position après avoir vu un résultat DREAM.

## Ce qui invalide un essai

Une caméra touchée ou déplacée (étape 4), une planche déplacée, un changement
de mise au point ou de bague de focale (adhésif décollé, fx de fin de session
hors ± 1 %), un bridge tombé pendant une série. L'essai se rejoue en entier.

## Ce qui manque avant la session

1. **Fait le 10/11** : `capture_real_tri.py --jeu e2 --extrinseque CAMERA=YAML
   --sortie DOSSIER`.
2. **Fait le 10/11** : `training/dream/geometrie_camera.py`.
3. **Fait le 19/11** : `calibrate_camera.py` lit la planche du banc ;
   `calibrer_extrinseque_4aruco.py` accepte un `.meta.json` et écrit
   `intrinsics_stem` ; `convert_real_tri_ndds.py --extrinseque` ;
   `eval_papier.py` prend l'intrinsèque écrite dans chaque label.
4. Un script du profil RMS(fx) de l'étape 1b (aujourd'hui en ligne de
   commande dans `~/dream_article/E3/E3_9c/`).
5. Rouleau d'adhésif, et la feuille de session (`run.yaml`) avec focus et
   photo par caméra.
6. Le bloc du contrôle en hauteur (ArUco de 50 mm sur un support rigide de
   ~100 et ~200 mm) et une procédure de relevé au robot.
7. Les supports de caméra pour 6 à 10 positions (trépieds, bras), et les
   câbles USB sur des ports arrière sans hub (erreurs `-71` sinon).

## Journal

| Date | Position | Caméra | Leave-one-out max (mm) | Marqueur haut (px) | Poses valides | État |
|---|---|---|---|---|---|---|
