# Protocole E2 — jeu réel multi-vues pour l'article DREAM

Expérience E2 de [`ARTICLE_DREAM.md`](ARTICLE_DREAM.md). Méthode :
`.claude/skills/campagne-mesure`. Rédigé le 10/11/2026, **avant toute mesure** :
les critères ci-dessous ne se recalent pas sur le résultat.

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
- **Budget d'étiquette** : 1 mm d'erreur 3D vaut ~0,66 px à 0,75 m pour
  l'arducam (f = 496 px), ~0,75 px pour la SVPRO (f = 560 px). Un PCK@2,5
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

Azimut, distance et élévation se calculent depuis l'extrinsèque, dans le repère
base et avec la convention de `convert_to_ndds_camera_libre.azimut_deg`, pour
que le secteur tenu à l'écart soit le même en réel et en synthétique.

Les deux caméras (arducam `cam_3`, SVPRO `cam_2`) peuvent servir, une position
chacune à la fois : leurs intrinsèques sont celles du registre
(`camera_registry.load_intrinsics`). **Mise au point et zoom figés** (aucun
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
  `gripper_bridge.py` sur la Pi, `scripts/real_robot_preflight.sh $MYCOBOT_PI`,
  puis arrêt du `bridge_tour` qu'il laisse. Contrôle de la planche : pince sur
  19 et 23, lecture des codeurs sans ordre moteur ; écart ≤ 2 mm à
  `planche_actuelle.yaml`, sinon remesurer.
- **Consigner** : date, opérateur, IP de la Pi, commit, écart 19/23, éclairage
  (lampes, fenêtres, heure), température.
- **Critère de sortie** : preflight OK, planche à ≤ 2 mm.
- **Ne pas faire** : relever la planche à travers une caméra.

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
  1. 60 images, `scripts/calibrer_extrinseque_4aruco.py` (16 coins, taille
     mesurée, leave-one-marker-out), positions de `planche_actuelle.yaml` ;
     sortie `training/calibration/e2_Pxx_<camera>_extrinsic.yaml` (non commité).
  2. **Contrôle en hauteur** : un ArUco de 50 mm sur un bloc rigide, posé à
     ~100 mm et ~200 mm de haut, sa position relevée **au robot** (pince sur
     son centre, codeurs lus) ; on mesure l'écart entre sa projection et sa
     détection.
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
- **Actions** : `training/capture_real_tri.py` avec la graine 2001 et
  l'extrinsèque de la position (option à ajouter, voir « Ce qui manque »), bras
  arrêté à chaque prise, angles mesurés et non commandés.
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

## Étape 5 — Figer le jeu

- **Actions** : `training/dream/convert_real_tri_ndds.py` par position, avec la
  distorsion et l'extrinsèque de la position ; md5 de chaque image ; contrôle
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
de mise au point, un bridge tombé pendant une série. L'essai se rejoue en
entier.

## Ce qui manque avant la session

1. `capture_real_tri.py` lit des extrinsèques **en dur**
   (`EXTRINSEQUES`, `arducam_extrinsic_pick.yaml` et `svpro_extrinsic_labels.yaml`) :
   ajouter une option pour l'extrinsèque de la position, une caméra à la fois,
   et un dossier de sortie par position.
2. Un petit script qui donne azimut, distance et élévation depuis une
   extrinsèque, avec la convention du synthétique.
3. Le bloc du contrôle en hauteur (ArUco de 50 mm sur un support rigide de
   ~100 et ~200 mm) et une procédure de relevé au robot.
4. Les supports de caméra pour 6 à 10 positions (trépieds, bras), et les
   câbles USB sur des ports arrière sans hub (erreurs `-71` sinon).

## Journal

| Date | Position | Caméra | Leave-one-out max (mm) | Marqueur haut (px) | Poses valides | État |
|---|---|---|---|---|---|---|
