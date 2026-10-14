# Protocole réel — YOLO et DREAM sur le banc, deux caméras

*Créé le 13/10/2026. Suite de [PROTOCOLE_YOLO_GAZEBO.md](PROTOCOLE_YOLO_GAZEBO.md), validé en
simulation le 07/10. Critères : [protocole_yolo_gazebo/CRITERES.md](protocole_yolo_gazebo/CRITERES.md).*

On refait sur le robot réel ce qui a été fait dans Gazebo : le tri par YOLO, puis DREAM en
parallèle, mesuré en statique et en dynamique. Une étape à la fois : chaque étape a un critère de
sortie, et on ne passe à la suivante qu'une fois ce critère atteint et **validé**.

## Ce qui change entre la simulation et le réel

| | simulation (07/10) | réel (ce protocole) |
|---|---|---|
| caméras | 4 (front, left, right, top), K exactes | 2 : **arducam** (`cam_3`, exposition 75) et **SVPRO** (`cam_2`, 800×600 → 640×480) |
| pose des caméras | exacte (Gazebo) | **extrinsèque ArUco**, 4 marqueurs 19 / 23 / 25 / 26, recalibrée à chaque session |
| référence du bras | vérité Gazebo | **codeurs** : FK(angles lus) |
| référence des objets | vérité Gazebo | **aucune** à ce jour (voir « Ce qui manque ») |
| tri | `sim_sorting_grasp` + `tri_dream_dashboard` | `lancer_pick_dashboard_final.py` (yolo26, 4 pièces peintes, 4 bacs) |
| modèle DREAM | `vgg_tri_mix_ft_e10` | le même : `training/dream/checkpoints_dream/vgg_tri_mix_ft_e10/best_network.pth` |
| modèle YOLO | `pieces_v6c_gazebo_yolo26s` | celui du lanceur (`--modele`, seuil 0,10) |

**Ce qu'on mesure.** Comme en simulation, DREAM donne la **pose caméra ↔ robot** (T_DREAM), pas
les angles : PnP entre ses 7 keypoints 2D et les mêmes points calculés par FK aux angles des
codeurs. On la compare à la pose donnée par les marqueurs (T_ArUco). La pointe de la pince, connue
par les codeurs, est placée à travers les deux poses :

```
pointe DREAM = inv(T_ArUco) · T_DREAM · pointe_codeurs
erreur       = pointe DREAM − pointe_codeurs
```

C'est la formule de `dream_point` dans `tri_dream_dashboard.py`, avec T_ArUco à la place de la
vérité Gazebo. **Ni T_ArUco ni les codeurs ne sont une vérité exacte** (voir § Limites de la
référence) : l'erreur mesurée est un écart entre deux estimations, pas une erreur absolue de DREAM.

## Rôle des caméras (décidé le 13/10)

- **arducam** : extrinsèque ArUco (les 4 marqueurs, vus nettement) **et** YOLO. Elle seule donne
  le XY des objets.
- **SVPRO** : **DREAM seulement**. Ni calibrée, ni utilisée par YOLO, même en appui :
  `lancer_pick_dashboard_final.py --svpro-dream`. Vue de côté, les marqueurs 23 et 26 n'y font
  que ~25 × 11 px et sa détection les perd une image sur deux.

**Conséquence pour DREAM sur la SVPRO.** Sans extrinsèque, il n'y a pas de référence extérieure
pour cette caméra : placer la pointe par `inv(T_DREAM) · T_DREAM · p` redonne `p`, les codeurs,
par construction. Ce qu'on peut mesurer sans extrinsèque :

- le **taux de détection** par keypoint (aucune référence nécessaire) ;
- la **stabilité de T_DREAM** : la caméra est fixe, donc T_DREAM(t) devrait être constante. Sa
  dispersion en statique puis en dynamique, et celle de la pointe placée par
  `inv(médiane T_DREAM) · T_DREAM(t) · p`, mesurent le bruit de DREAM ;
- le **résidu de reprojection** des keypoints à travers T_DREAM.

Un **biais** constant de DREAM n'est pas mesurable ainsi : il faut une référence extérieure. Pour
le chiffrer, il faudrait soit une extrinsèque SVPRO, soit la même mesure sur l'arducam, qui en a
une.

**Mise à jour 13/10** : une extrinsèque SVPRO existe désormais, `svpro_extrinsic_labels.yaml`,
**uniquement pour étiqueter les images d'entraînement** (étape 8). DREAM en marche et YOLO ne la
lisent pas ; la SVPRO reste hors de la chaîne YOLO.

## Règles du banc (rappel, toutes mesurées)

- `conda deactivate`, puis le bridge **`scripts/gripper_bridge.py`** sur la Pi. Identifier la Pi
  par un **aller-retour TCP sur le port 5005**, jamais par `ping` (l'adresse change).
  `bash scripts/real_robot_preflight.sh` avant tout mouvement.
- Le bridge est **mono-client et bloquant** : un `bridge_tour` resté ouvert le fige.
- Pince **uniquement sur ordre** ; moteurs **jamais relâchés** ; annoncer avant toute descente ;
  au premier échec de saisie, sonder sans bouger, **pas d'essais en rafale**.
- Toujours relire `get_angles` après `send_angles` : hors limite, le bridge répond OK et le bras
  ne bouge pas.
- Exposition arducam **75**, posée avant d'ouvrir. Caméras ouvertes en `CAP_V4L2`, identifiées par
  le nom de carte (l'index bouge).
- **Marqueurs de la planche jamais déplacés** : si un marqueur manque, régler la caméra.
- `pick_dashboard.py` ne se modifie pas : tout ajout passe par un module branché, comme
  `yolo26_dashboard.py`.

## Étape 0 — Préparation et extrinsèques

- **Actions** : preflight ; `lancer_pick_dashboard_final.py` calibre les deux caméras d'un seul
  « oui » contre `training/calibration/planche_actuelle.yaml` (référence mesurée au robot le 15/09).
  Ne jamais utiliser « Relever la planche » (`--reference`).
- **Consigner** : leave-one-out des deux caméras, date et heure des fichiers
  `arducam_extrinsic_pick.yaml` et `svpro_extrinsic_servo.yaml`.
- **Critère de sortie** : calibration arducam acceptée sans `--force` (la SVPRO n'est pas
  calibrée, voir § Rôle des caméras).

## Étape 1 — Le tri YOLO fonctionne

```bash
conda deactivate
/usr/bin/python3 scripts/lancer_pick_dashboard_final.py --svpro-dream
```

- **Objectif** : vérifier que la chaîne yolo26 → pixel → extrinsèque → mm robot → IK trie les
  4 pièces peintes dans leurs 4 bacs, **avant** d'y ajouter DREAM.
- **Consigner** par pièce : classe, confiance, caméra(s), tri réussi Oui / Non, cause d'un échec.
- **Critère de sortie** : 4/4 sur une scène, cause écrite pour chaque échec.
- **Ne pas faire** : ajouter DREAM au même moment.

**Résultat (13/10) — atteint.** `scripts/tri_fluide.py --toutes` : 4/4, chaque pièce posée au
fond de son bac, 1 à 2 mm du centre (cylindre vert, pavé jaune, cube bleu, cube rouge). Le
dashboard final est rebranché sur la même boucle (`branche_tri_fluide`), pas encore testé sur le
robot.

## Étape 2 — Ce que voient les deux caméras (occlusion)

DREAM est sensible à l'occlusion. Avant de mesurer, on vérifie que le bras est visible.

- **Outil** : `scripts/fk_vs_dream_diagnostic.py` projette le squelette FK (codeurs) à travers les
  extrinsèques ArUco sur les **deux** vues. Le bras ne bouge pas.
- **Actions** : pour chaque pose prévue aux étapes 4 à 6 (pose d'observation, approche, saisie,
  dépôt dans chaque bac), noter, par caméra, les keypoints dans l'image et ceux cachés par le bras,
  la pince ou une pièce.
- **Enveloppe** : J5 dans [0 ; −60°], sinon la détection chute ; J6 n'est pas observable.
- **Critère de sortie** : un tableau pose × caméra × keypoint (visible / caché / hors image), et
  chaque keypoint visible sur au moins une caméra pour chaque pose retenue.

## Étape 3 — Test de décalage (avant tout chiffre DREAM)

**Pourquoi** : le 02/09, le keypoint `base` d'un modèle affiné sur deux caméras fixes s'est révélé
être une **constante mémorisée par montage** : décalée de N px, l'image donnait une détection
décalée de 0 px. Une pose ajustée dessus restitue le cadrage des données d'affinage, et la
comparaison aux marqueurs devient circulaire. `vgg_tri_mix_ft_e10` a été affiné sur `real_3cam`
(arducam, SVPRO, Astra, juin 2026), **avant que les caméras soient déplacées** : le risque est le
même.

- **Actions** : sur des images fixes de chaque caméra, décaler l'image de ±10, ±20, ±40 px en u et
  en v ; mesurer de combien chaque keypoint suit.
- **Critère de sortie** : chaque keypoint suit le décalage (seuil **à trancher**, par exemple
  ≥ 90 %). Sinon, les étapes 4 à 6 ne sont pas interprétables : passer à l'étape 8.

## Étape 4 — DREAM, keypoints, bras immobile (statique)

- **Référence** : FK(codeurs) projetée à travers l'extrinsèque ArUco de chaque caméra.
- **Actions** : N poses (à fixer, ≥ 20), bras arrêté, une série d'images par pose et par caméra.
- **Livrable** : un tableau **par caméra** (arducam, SVPRO), au format de l'évaluation simulée :

| keypoint | moyenne (px) | médiane (px) | écart-type (px) | max (px) | détection |
|---|---|---|---|---|---|
| base … link6 | | | | | |
| **global** | | | | | |

  puis la précision par seuil (< 2, < 5, < 10, < 20, < 50 px) et l'erreur moyenne par image.
- **Critère de sortie** : tableaux remplis, blocs de résultats **bruts** d'abord, analyse ensuite.

## Étape 5 — DREAM, keypoints, bras en mouvement (dynamique)

- Même tableau, pendant le tri ou une trajectoire rejouée.
- **Synchronisation** : chaque image doit être appariée aux angles lus **au même instant**.
  Consigner l'horodatage de l'image et celui de `get_angles` ; l'écart entre les deux est une
  source d'erreur propre au dynamique, à mesurer avant de conclure.
- Statique = bras en pose d'observation à 2° près, comme en simulation (`tri_bilan.ArmTimeline`) ;
  dynamique = le reste.
- **Critère de sortie** : tableau statique et tableau dynamique côte à côte, par caméra.

### Résultat des étapes 4 et 5 (13/10) — DREAM ne transfère pas au banc

**Mesure.** Un tri complet 4/4 (`tri_fluide`, inchangé) enregistré par
`scripts/enregistre_tri_dream.py` : images arducam et SVPRO à ~5 Hz, codeurs à ~8 Hz sur la
**même** connexion au bridge que le tri (verrou), l'arducam rendue pendant chaque localisation
YOLO. DREAM (`vgg_tri_mix_ft_e10`) passé hors ligne par `scripts/analyse_tri_dream.py`.
Statique = codeurs immobiles (< 0,3° sur chaque joint) dans ± 0,4 s autour de l'image ;
dynamique = le reste, si les codeurs encadrent l'image à 0,3 s près. Cette définition remplace
« pose d'observation à 2° près » : elle compte aussi les arrêts du tri (saisie, dépôt).
Session : `results/dream_reel/2026-10-13_16-26-05/` (`rapport_dream.md`, `keypoints_arducam.csv`).

**Arducam** — référence : FK(codeurs) projetée par `arducam_extrinsic_pick.yaml`, extérieure à
DREAM.

Statique — 133 images

| Keypoint | Moyenne (px) | Médiane (px) | Écart-type (px) | Max (px) | Détection |
|---|---|---|---|---|---|
| base | 14,76 | 14,21 | 1,84 | 19,53 | 100,0 % |
| link1 | 49,01 | 49,60 | 1,74 | 52,02 | 100,0 % |
| link2 | 49,01 | 49,60 | 1,74 | 52,01 | 100,0 % |
| link3 | 45,79 | 47,00 | 17,68 | 67,44 | 86,5 % |
| link4 | 31,65 | 29,67 | 17,08 | 72,46 | 85,7 % |
| link5 | 76,28 | 44,18 | 91,91 | 319,90 | 66,9 % |
| link6 | 93,74 | 41,38 | 115,79 | 328,16 | 69,9 % |
| **GLOBAL** | 48,62 | 46,80 | 55,70 | 328,16 | 87,0 % (810/931) |

| < 2 px | < 5 px | < 10 px | < 20 px | < 50 px |
|---|---|---|---|---|
| 0,4 % | 0,9 % | 3,2 % | 25,4 % | 68,8 % |

Erreur moyenne par image : 48,67 ± 27,71 px (meilleure 20,40, pire 136,18 px).

Dynamique — 294 images

| Keypoint | Moyenne (px) | Médiane (px) | Écart-type (px) | Max (px) | Détection |
|---|---|---|---|---|---|
| base | 14,87 | 13,64 | 3,03 | 25,84 | 99,7 % |
| link1 | 49,49 | 49,54 | 2,94 | 60,42 | 99,0 % |
| link2 | 49,49 | 49,54 | 2,94 | 60,42 | 99,0 % |
| link3 | 57,50 | 58,44 | 20,04 | 114,39 | 68,4 % |
| link4 | 35,50 | 33,83 | 20,48 | 128,98 | 78,9 % |
| link5 | 94,89 | 54,39 | 111,06 | 358,32 | 70,7 % |
| link6 | 117,87 | 53,65 | 127,22 | 356,04 | 68,0 % |
| **GLOBAL** | 56,10 | 47,37 | 66,93 | 358,32 | 83,4 % (1716/2058) |

| < 2 px | < 5 px | < 10 px | < 20 px | < 50 px |
|---|---|---|---|---|
| 0,5 % | 1,0 % | 5,1 % | 25,5 % | 61,8 % |

Erreur moyenne par image : 56,27 ± 36,26 px (meilleure 19,65, pire 167,08 px).

**SVPRO** — détection seule (pas d'extrinsèque au moment de la mesure).

| Keypoint | Statique (275 images) | Dynamique (292 images) |
|---|---|---|
| base | 78,5 % | 81,8 % |
| link1 / link2 | 98,5 % | 89,0 % |
| link3 | 35,6 % | 63,0 % |
| link4 | 69,8 % | 83,6 % |
| link5 | 36,7 % | 76,0 % |
| link6 | 99,3 % | 95,2 % |
| **GLOBAL** | 73,9 % (1422/1925) | 82,5 % (1687/2044) |

7/7 sur 28,0 % des images en statique, 40,1 % en dynamique.

**Analyse.**
- Dans Gazebo, le même modèle donnait 3,0 px de médiane et 98,6 % sous 10 px. Sur le banc :
  ~47 px de médiane, 3-5 % sous 10 px. Statique et dynamique sont du même ordre : le mouvement
  n'est pas la cause.
- La référence n'est pas en cause : le squelette des codeurs projeté épouse le bras dans l'image,
  et `mycobot_fk` et les étiquettes d'entraînement ont les mêmes longueurs entre keypoints
  (0,162 / 0,1364 / 0,1458 / 0,084 / 0,0664 m).
- link1 est décalé de 49 px avec 1,7 px d'écart-type quelle que soit la pose : DREAM le place au
  même endroit sans suivre le bras, le symptôme du keypoint `base` mémorisé le 02/09.
- « Détecté » veut dire un pic dans l'image, pas un pic au bon endroit : sur l'arducam, des
  keypoints détectés à 100 % sont à ~50 px du bras.
- Essai rejeté : ajuster une pose SVPRO sur les détections elles-mêmes. Selon le point de départ,
  link6 sortait à 3 px ou à 208 px, pour une caméra déplacée de 145 mm. Ce chiffre ne mesure rien.

**Décision** : passer à l'étape 8.

## Étape 6 — Pose DREAM contre marqueurs, et pointe de la pince

- **Mesures** par caméra, puis fusionnées (médiane par axe des pointes placées, comme en
  simulation, vues à ≥ 5 keypoints) :
  - T_DREAM ↔ T_ArUco : écart de translation (mm) et de rotation (°) ;
  - pointe DREAM ↔ pointe codeurs : x, y, z et norme 3D (mm), en continu et à chaque saisie.
- **Statique et dynamique**, comme aux étapes 4 et 5.
- **Critère de sortie** : tableau min / médiane / max sur toutes les saisies, avec le détail par
  saisie (format du tableau 27 de la simulation).

### Réalisé (14/10) — tri de 16:17, `vgg_tri_reel_ft_e10`

Trois cas, définis une fois pour toutes :

- **test sur images** : `real_tri_test` (13/10, jamais vu à l'entraînement), sans dashboard ;
- **bras immobile pendant le tri** : arrêts au serrage et au fond du bac ;
- **bras en mouvement pendant le tri** : du départ vers la pièce à la pose dans le bac (retour exclu).

**Keypoints contre la FK des codeurs projetée (px, médiane / détection)** :

| | arducam | SVPRO |
|---|---|---|
| test sur images (100 + 100) | 2,00 px / 92,0 % | 1,38 px / 100 % |
| bras immobile pendant le tri (86 + 88) | 1,87 px / 92,6 % | 5,18 px / 96,4 % |
| bras en mouvement pendant le tri (245 + 244) | 1,97 px / 81,8 % | 5,30 px / 95,0 % |
| bras au repos, pose d'observation, hors tri | 4,75 px / 58,1 % | 9,42 px / 58,0 % |

La SVPRO est à ~5 px en direct contre 1,4 px sur le test : `base`, immobile, y est décalé de
4,2 px avec un écart-type de 0,08 px. La caméra a bougé depuis la capture du 13/10. La pose
d'observation (J5 ≈ 2°) est hors du domaine d'entraînement (J5 46-92°) : elle n'intervient ni dans
la saisie ni dans la pose au bac.

**T_DREAM contre T_ArUco**, calculé comme `dream_fk_compare.pose_error` (translation dans le repère
caméra, dz = profondeur), médiane (p90), vues ≥ 5 keypoints :

| | caméra | vues | translation | \|dz\| | \|dx\| / \|dy\| | rotation |
|---|---|---|---|---|---|---|
| Gazebo, 10 graines | front | 859 | 8,3 (18,2) mm | — | — | 1,8 (4,4)° |
| | right | 853 | 12,9 (21,1) mm | — | — | 2,6 (5,3)° |
| test sur images | arducam | 93 / 100 | 5,4 (15,3) mm | 4,5 mm | 1,4 / 1,6 mm | 0,9 (1,8)° |
| | SVPRO | 100 / 100 | 10,2 (20,7) mm | 10,1 mm | 1,6 / 1,2 mm | 1,8 (3,6)° |
| bras immobile, tri | arducam | 75 / 86 | 7,8 (16,6) mm | 4,6 mm | 1,6 / 4,3 mm | 1,4 (2,4)° |
| | SVPRO | 86 / 88 | 19,6 (81,9) mm | 16,3 mm | 7,8 / 3,0 mm | 3,5 (15,5)° |
| bras en mouvement, tri | arducam | 177 / 245 | 9,0 (30,7) mm | 6,9 mm | 2,4 / 4,4 mm | 1,8 (4,8)° |
| | SVPRO | 230 / 244 | 14,1 (62,4) mm | 10,2 mm | 8,7 / 2,4 mm | 2,4 (19,6)° |

Critère de rotation < 5° : tenu par l'arducam en médiane et au p90 dans les trois cas ; la SVPRO le
tient sur le test, pas en direct (caméra déplacée).

**Pointe DREAM − pointe codeurs**, le long de la trajectoire (départ → pose au bac, J5 dans le
domaine), médiane par axe, mm :

| objet | 2 caméras (médiane 1 s) x / y / z | 3D | arducam seule 3D | SVPRO seule 3D |
|---|---|---|---|---|
| pave_jaune | −1,6 / +1,1 / +13,0 | 13,1 | 13,8 | 15,5 |
| cylindre_vert | −4,2 / +3,5 / +15,6 | 16,5 | 14,9 | 20,9 |
| cube_bleu | +0,7 / +8,3 / +10,9 | 13,7 | 13,8 | 20,1 |
| cube_rouge | −5,9 / +1,9 / +15,5 | 16,7 | 15,3 | 18,5 |

L'erreur 3D est **surtout en Z** (DREAM place la pince 11 à 16 mm trop haut). Deux causes, toutes
deux structurelles : (1) un PnP sur une image tire la profondeur de la taille apparente du bras,
~150 px pour ~0,3 m à ~1 m, soit ~7 mm de profondeur par pixel d'erreur — et l'arducam, presque
zénithale (~85°), met cette profondeur sur Z ; (2) la FK des codeurs ne voit pas l'affaissement
(13-15 mm) : l'écart en Z croît avec l'allonge (+2 à +6 mm sous 320 mm, +9 à +12 mm au-delà).

**Fusion des deux vues par triangulation** (`analyse_fusion_dream.py`, à travers les extrinsèques
ArUco ; un keypoint vu d'une seule caméra compte en détection seulement) : erreur 3D médiane
**4,5 mm** sur le test, 7,0 mm (bras immobile) et 7,6 mm (en mouvement) pendant le tri ; détection
100 % / 97,8 % / 95,3 %. La vue de côté corrige la profondeur que l'arducam voit mal.

Fichiers : `results/dream_reel/rapport_pose_test.md`, `rapport_fusion_test.md`, et dans
`results/dream_reel/2026-10-14_16-17-42_dashboard/` : `rapport_dream.md`,
`rapport_dream_par_fenetre.md`, `rapport_pose.md`, `rapport_fusion.md`, `keypoints_*.csv`.
Scripts : `analyse_tri_dream.py`, `analyse_pose_dream.py`, `analyse_fusion_dream.py`.

## Étape 7 — YOLO, statique et dynamique

- **Livrable** : par caméra et par classe (4 pièces, 4 bacs), en statique et en dynamique :
  détection %, confiance, erreur px, erreur XY mm.
- **Référence à construire** (rien n'existe aujourd'hui) :
  - détection : nombre de pièces et de bacs réellement posés, compté à la main ;
  - position : centre de chaque pièce **mesuré au robot** (pince posée dessus, codeurs lus, sans
    ordre moteur), comme pour les marqueurs 19 et 23 le 15/09 ;
  - erreur px : boîtes annotées à la main sur un sous-échantillon d'images.
- **Critère de sortie** : tableau statique / dynamique par caméra et par classe.

## Étape 8 — Si DREAM ne tient pas : nouvelles images des deux caméras

- **Capture** : `scripts/capture_markerless.py` (poses diversifiées, visibilité exigée sur les deux
  caméras). Ajouter de la **diversité de point de vue** (plusieurs positions de caméra), sinon le
  socle redevient mémorisé.
- **Étiquettes** : FK(codeurs) projetée par l'extrinsèque ArUco de la session.
- **Entraînement** : `train_dream_ultimate_v5_geo.py`, départ `vgg_tri_mix_ft_e10`, **dossier de
  sortie neuf** (jamais écrire dans un dossier de checkpoints existant).
- **Test** : un jeu réel **hors échantillon**, vérifié par **hachage md5**, pas par nom.
- Reprendre à l'étape 3.

### Réalisé (13/10)

**Labels SVPRO.** DREAM n'a pas besoin d'extrinsèque **en marche** : il trouve lui-même la pose
caméra ↔ robot. Mais pour l'entraîner, chaque image doit porter la position des 7 keypoints en
pixels, et on ne la calcule qu'en connaissant la pose de la caméra. Dans Gazebo, la caméra placée
dans la simulation la donnait ; sur le banc, ce sont les 4 ArUco, utilisés **une seule fois, hors
ligne**, pour étiqueter.
- `scripts/calibre_svpro_labels.py` (sans mouvement) → `training/calibration/svpro_extrinsic_labels.yaml`.
  Ne remplace ni `svpro_extrinsic_servo.yaml`, ni aucun réglage YOLO.
- À 640×480, le marqueur 26 n'est jamais décodé (0/40 images) : 23 et 26 font ~25 × 11 px. Image
  agrandie ×3 pour le décodage : 26 vu sur 78 % des images, les autres sur 100 %. Chaque marqueur
  est moyenné sur les images où il est vu.
- RMS 0,79 px ; leave-one-out 4,7 / 7,0 / 7,7 / 9,4 mm (23 / 26 / 19 / 25), au-dessus de l'arducam
  (vue rasante). Contrôle décisif : le squelette des codeurs projeté épouse le bras sur toutes
  les images vérifiées.

**Capture.** `training/capture_real_tri.py`, à la place de `capture_real_3cam.py` (même format :
`labels.csv`, `images/<camera>/NNNNNN.png`) et de `capture_markerless.py` :
- arducam + SVPRO ; labels 2D des deux caméras dans `keypoints_2d.csv` ;
- poses **pas** tirées sur 50 % des débattements (pince montée : bacs, écran) : pointe tirée
  au-dessus de la planche (X 150-400, Y −200 à 250, Z 90-200 mm), outil orienté comme dans le tri
  (inclinaison minimale exigée par l'allonge, roulis ± 90°), IK du FSM, coude jamais à plus de
  40 mm derrière la base, marge d'auto-collision ≥ 10 mm ;
- mouvements par `pick_fsm.va_vers_par_etapes` ; un trajet qui passerait sous 70 mm remonte d'abord
  en pose d'observation ; la pince n'est jamais commandée.

| Jeu | Graines | Poses | Images |
|---|---|---|---|
| `real_tri/train` | 1, 2, 3 | 600 | 1 200 |
| `real_tri/test` | 1002 | 100 | 200 |

md5 : 0 image commune entre `real_tri_train_ndds` et `real_tri_test_ndds`.

Incident : sans la marge d'auto-collision, une pose tirée à 0,5 mm de marge a été atteinte à
−0,03 mm (pose 538, J3 −139,6°, affaissement). La garde du FSM teste aussi le départ : plus aucun
mouvement accepté, retour en observation compris. Dégagé par un seul ordre, J3 seul vers −120°
(marge 0 → 17 mm, pointe 77 → 158 mm), validé avant envoi. 62 poses perdues en session 2.

**Conversion** : `training/dream/convert_real_tri_ndds.py` → `real_tri_train_ndds` (1 200) et
`real_tri_test_ndds` (200).

**Avant fine-tuning** — `evaluate_dream.py`, `vgg_tri_mix_ft_e10`, `real_tri_test_ndds`,
`--split all -n 200` :

```
Detection rate: 1187/1400 (84.8%)
  Keypoint                 Mean   Median      Std      Max   Det%
  base                    11.54    11.76     8.04    66.38  90.5%
  link1                   31.55    30.11    19.49    97.36  95.0%
  link2                   31.55    30.11    19.49    97.35  95.0%
  link3                   40.07    19.96    33.85   167.37  70.0%
  link4                   23.21    16.59    16.46    87.97  81.0%
  link5                   29.90    21.42    21.55   103.75  86.5%
  link6                   30.30    24.69    29.03   224.15  75.5%
  OVERALL                 27.96    18.73    23.35   224.15
  <2 px 0.3%   <5 px 4.6%   <10 px 13.2%   <20 px 52.7%   <50 px 82.1%
  Per-frame mean error: 28.80 ± 15.38 px
```

**Fine-tuning, lancé le 13/10 à 18:10.**
- Script `train_dream_ultimate_v5_geo.py`, départ `vgg_tri_mix_ft_e10`, 10 époques (~41 min),
  `--workers 6`, `taskset -c 16-27`, pause automatique à 90 °C.
- Données `mix_tri_reel_synth50k_real3camx5_ndds`, **128 160 images** : 50 000 synthétique 50K
  + 30 000 réel ancien (6 000 × 5) + 24 160 scène de tri Gazebo (12 080 × 2) + **24 000 réel tri
  (1 200 × 20)**, soit 69 280 images uniques. Découpage 80 / 10 / 10 : 102 528 images
  d'entraînement par époque. Le réel tri pèse 19 %. Validation optimiste (copies des deux côtés).
- Sortie : `checkpoints_dream/vgg_tri_reel_ft_e10`, dossier neuf.
- Terminé le 14/10 à 01:36 : 445,3 min, meilleure époque 10 (val 0,000238). Deux pauses
  thermiques de 10 s (90 et 93 °C).

### Après fine-tuning (14/10) — test réel hors échantillon

`evaluate_dream.py --split all -n 200`, `vgg_tri_reel_ft_e10`, sur `real_tri_test_ndds`
(100 poses × 2 caméras, md5 : 0 commune avec l'entraînement). Blocs bruts :

```
=== vgg_tri_reel_ft_e10 | real_tri_test_ndds (arducam + SVPRO, 200 images)
Detection rate: 1344/1400 (96.0%)
  Keypoint                 Mean   Median      Std      Max   Det%
  base                     1.27     1.15     0.74    10.72 100.0%
  link1                    1.71     2.18     0.76     5.01 100.0%
  link2                    1.71     2.18     0.76     5.01 100.0%
  link3                    2.10     1.86     1.18     8.50 100.0%
  link4                    2.29     1.94     1.49    11.13  96.5%
  link5                    2.30     2.16     1.22     7.16  87.0%
  link6                    2.38     2.24     1.24     6.26  88.5%
  OVERALL                  1.95     1.69     1.15    11.13
  <2 px 56.0%   <5 px 98.1%   <10 px 99.9%   <20 px 100.0%   <50 px 100.0%
  Per-frame mean error: 1.95 ± 0.49 px   Best 1.06 px   Worst 5.16 px

=== vgg_tri_reel_ft_e10 | arducam seule (100 images)
Detection rate: 644/700 (92.0%)
  base                     1.30     1.26     0.36     3.87 100.0%
  link1                    2.37     2.35     0.27     5.01 100.0%
  link2                    2.37     2.35     0.27     5.01 100.0%
  link3                    1.74     1.58     1.08     8.50 100.0%
  link4                    1.77     1.60     0.98     6.82  93.0%
  link5                    1.79     1.68     0.90     4.87  74.0%
  link6                    1.89     1.64     1.05     4.92  77.0%
  OVERALL                  1.90     2.00     0.85     8.50
  <2 px 50.0%   <5 px 99.4%   <10 px 100.0%   <20 px 100.0%   <50 px 100.0%
  Per-frame mean error: 1.90 ± 0.43 px   Best 1.21 px   Worst 5.16 px

=== vgg_tri_reel_ft_e10 | SVPRO seule (100 images)
Detection rate: 700/700 (100.0%)
  base                     1.23     1.13     0.98    10.72 100.0%
  link1                    1.05     0.98     0.44     4.79 100.0%
  link2                    1.05     0.98     0.44     4.79 100.0%
  link3                    2.45     2.21     1.16     6.07 100.0%
  link4                    2.77     2.51     1.71    11.13 100.0%
  link5                    2.67     2.60     1.30     7.16 100.0%
  link6                    2.77     2.58     1.24     6.26 100.0%
  OVERALL                  2.00     1.38     1.37    11.13
  <2 px 61.6%   <5 px 96.9%   <10 px 99.7%   <20 px 100.0%   <50 px 100.0%
  Per-frame mean error: 2.00 ± 0.53 px   Best 1.06 px   Worst 4.56 px
```

Avant, sur les mêmes images (`vgg_tri_mix_ft_e10`) : arducam 77,7 % / médiane 45,26 px ;
SVPRO 91,9 % / 14,60 px ; les deux 84,8 % / 18,73 px (blocs complets dans
`results/dream_reel/eval_avant_apres_1410.txt`).

| Test réel (200 images) | Détection | Médiane | < 5 px | < 10 px | Erreur par image |
|---|---|---|---|---|---|
| avant (`vgg_tri_mix_ft_e10`) | 84,8 % | 18,73 px | 4,6 % | 13,2 % | 28,80 px |
| **après (`vgg_tri_reel_ft_e10`)** | **96,0 %** | **1,69 px** | **98,1 %** | **99,9 %** | **1,95 px** |

Non-régression synthétique (mêmes sous-échantillons que le 06/10) :

| Jeu | Détection avant → après | Médiane avant → après |
|---|---|---|
| scène de tri Gazebo, test 2001-2003 (500 images) | 99,7 % → 99,7 % | 3,00 → 2,99 px |
| 50K synthétique (2 000 images) | 99,4 % → 99,4 % | 2,91 → 2,91 px |

Contrôle visuel : les keypoints prédits tombent dans les cercles des labels sur les deux
caméras ; link1 n'est plus figé.

**Analyse.**
- Seuil de la simulation (médiane < 5 px, détection > 90 %) **atteint** sur le test réel, pour
  chaque caméra.
- Ce qui reste faible : la **détection** de link5 et link6 sur l'arducam (74 % et 77 %) ; quand ils
  sont détectés, ils sont justes (< 2 px de médiane).
- **Limite du test** : hors échantillon en poses (graine 1002, md5 vérifié), mais pris le même
  jour, mêmes caméras au même endroit, même éclairage, même planche. Un déplacement de caméra ou
  un autre éclairage n'est pas couvert.
- Les labels dépendent des extrinsèques (arducam LOO ≤ 3,6 mm, SVPRO 4,7-9,4 mm) et de la FK des
  codeurs (affaissement ~13-15 mm à la pointe). 1,7 px mesure l'accord avec ces labels, pas une
  erreur absolue.

### Étapes 4 et 5 avec le nouveau modèle (14/10) — tri du 13/10 rejoué

Le tri enregistré le 13/10 à 16:26 n'a pas servi à l'entraînement (celui-ci ne contient que
`real_tri/train`) : ses images sont hors échantillon. `analyse_tri_dream.py` les repasse avec
`vgg_tri_reel_ft_e10` ; la SVPRO est désormais mesurée en pixels, à travers
`svpro_extrinsic_labels.yaml`. Rapport : `results/dream_reel/2026-10-13_16-26-05/rapport_dream.md`
(celui du 13/10 conservé en `rapport_dream_vgg_tri_mix_ft_e10.md`).

**Arducam** — statique, 133 images

| Keypoint | Moyenne (px) | Médiane (px) | Écart-type (px) | Max (px) | Détection |
|---|---|---|---|---|---|
| base | 1,73 | 1,27 | 1,04 | 4,01 | 100,0 % |
| link1 | 2,20 | 2,31 | 0,23 | 2,40 | 100,0 % |
| link2 | 2,20 | 2,31 | 0,23 | 2,40 | 100,0 % |
| link3 | 9,78 | 2,08 | 17,35 | 50,86 | 97,0 % |
| link4 | 4,25 | 2,59 | 9,01 | 51,34 | 83,5 % |
| link5 | 4,26 | 2,75 | 10,90 | 101,82 | 61,7 % |
| link6 | 4,79 | 2,21 | 18,47 | 174,21 | 64,7 % |
| **GLOBAL** | 4,10 | 2,31 | 10,73 | 174,21 | 86,7 % (807/931) |

| < 2 px | < 5 px | < 10 px | < 20 px | < 50 px |
|---|---|---|---|---|
| 36,8 % | 93,8 % | 96,5 % | 96,5 % | 98,5 % |

Erreur moyenne par image : 4,78 ± 6,47 px (meilleure 1,53, pire 55,51 px).

**Arducam** — dynamique, 294 images

| Keypoint | Moyenne (px) | Médiane (px) | Écart-type (px) | Max (px) | Détection |
|---|---|---|---|---|---|
| base | 2,12 | 1,26 | 2,55 | 16,88 | 100,0 % |
| link1 | 2,94 | 2,33 | 4,59 | 57,77 | 97,3 % |
| link2 | 2,94 | 2,33 | 4,59 | 57,77 | 97,3 % |
| link3 | 5,95 | 1,76 | 12,84 | 59,15 | 95,2 % |
| link4 | 7,54 | 2,24 | 19,32 | 102,46 | 76,2 % |
| link5 | 5,05 | 2,46 | 14,88 | 102,74 | 50,3 % |
| link6 | 4,37 | 2,91 | 13,46 | 174,03 | 55,8 % |
| **GLOBAL** | 4,23 | 2,30 | 11,22 | 174,03 | 81,7 % (1682/2058) |

| < 2 px | < 5 px | < 10 px | < 20 px | < 50 px |
|---|---|---|---|---|
| 39,2 % | 90,9 % | 95,5 % | 96,9 % | 98,6 % |

Erreur moyenne par image : 4,95 ± 7,23 px (meilleure 1,26, pire 47,04 px).

**SVPRO** — statique, 275 images

| Keypoint | Moyenne (px) | Médiane (px) | Écart-type (px) | Max (px) | Détection |
|---|---|---|---|---|---|
| base | 1,11 | 0,98 | 0,68 | 5,03 | 100,0 % |
| link1 | 1,06 | 1,08 | 0,05 | 1,17 | 100,0 % |
| link2 | 1,06 | 1,08 | 0,05 | 1,17 | 100,0 % |
| link3 | 5,06 | 4,37 | 3,08 | 19,10 | 100,0 % |
| link4 | 2,45 | 2,43 | 1,46 | 5,65 | 38,2 % |
| link5 | 5,01 | 2,42 | 8,62 | 41,79 | 45,8 % |
| link6 | 29,08 | 44,62 | 21,40 | 50,40 | 99,6 % |
| **GLOBAL** | 6,94 | 1,12 | 13,77 | 50,40 | 83,4 % (1605/1925) |

| < 2 px | < 5 px | < 10 px | < 20 px | < 50 px |
|---|---|---|---|---|
| 61,2 % | 82,2 % | 87,9 % | 89,1 % | 99,3 % |

Erreur moyenne par image : 7,58 ± 4,37 px (meilleure 1,20, pire 16,18 px).

**SVPRO** — dynamique, 292 images

| Keypoint | Moyenne (px) | Médiane (px) | Écart-type (px) | Max (px) | Détection |
|---|---|---|---|---|---|
| base | 1,07 | 1,09 | 0,07 | 1,43 | 100,0 % |
| link1 | 1,04 | 0,99 | 0,43 | 6,69 | 100,0 % |
| link2 | 1,04 | 0,99 | 0,43 | 6,69 | 100,0 % |
| link3 | 4,98 | 3,49 | 4,92 | 40,16 | 99,0 % |
| link4 | 5,36 | 2,28 | 11,08 | 97,39 | 82,2 % |
| link5 | 6,97 | 2,13 | 12,95 | 118,21 | 89,0 % |
| link6 | 8,92 | 2,45 | 14,42 | 59,93 | 95,2 % |
| **GLOBAL** | 4,09 | 1,12 | 8,94 | 118,21 | 95,1 % (1943/2044) |

| < 2 px | < 5 px | < 10 px | < 20 px | < 50 px |
|---|---|---|---|---|
| 66,1 % | 85,6 % | 92,3 % | 94,8 % | 99,5 % |

Erreur moyenne par image : 4,30 ± 4,81 px (meilleure 0,84, pire 28,48 px).

**La pose d'observation est hors distribution.** Médiane de link6 et médiane globale, selon
que le bras est en pose d'observation (à 3° près) ou non :

| Caméra | Régime | Pose | Images | link6 médiane | Globale médiane |
|---|---|---|---|---|---|
| arducam | statique | observation | 26 | 174,2 px | 3,9 px |
| arducam | statique | autres | 107 | 2,1 px | 2,3 px |
| arducam | dynamique | observation | 15 | 174,0 px | 3,9 px |
| arducam | dynamique | autres | 279 | 2,9 px | 2,2 px |
| SVPRO | statique | observation | 170 | 44,9 px | 4,0 px |
| SVPRO | statique | autres | 105 | 1,7 px | 1,3 px |
| SVPRO | dynamique | observation | 14 | 44,8 px | 4,0 px |
| SVPRO | dynamique | autres | 278 | 2,3 px | 1,4 px |

La pose d'observation a J5 = 2,4° (outil à l'horizontale) ; toutes les poses de `real_tri/train`
ont J5 entre 65 et 90° (outil vers la table). DREAM n'a jamais vu le poignet ainsi sur le banc.
Hors de cette pose, statique comme dynamique sont sous 3 px de médiane sur les deux caméras.

**Correctif proposé** : une session de capture autour de la pose d'observation et des
transitions vers elle (J5 de 0 à 90°), ajoutée à `real_tri/train`, puis un fine-tuning court
depuis `vgg_tri_reel_ft_e10`. Le jeu de test devra lui aussi contenir ces poses.

**Suite** : un nouveau tri enregistré (robot rallumé, caméras rebranchées) pour confirmer, puis
étape 6.

## Étape 9 — Tri avec DREAM affiché

- **Objectif** : le même dashboard qu'en simulation, sur le banc : état du tri, trajectoires,
  écart XYZ (mm), latence, YOLO + DREAM.
- **Fichiers** : `scripts/dream_tri_panneau.py` (branché par `lancer_pick_dashboard_final.py
  --dream`) et `scripts/dream_service.py` (DREAM sous `venv_dream`, par tube). `pick_dashboard` et
  `pick_fsm` ne sont pas modifiés.
- **Critère de sortie** : un tri 4/4 avec, par saisie, pointe DREAM ↔ pointe codeurs dans un CSV
  par essai.

### Réalisé (14/10)

```bash
MYCOBOT_PI=10.10.0.223 /usr/bin/python3 scripts/lancer_pick_dashboard_final.py --svpro-dream --dream
```

- **À gauche** la fenêtre du tri, inchangée (dans une zone qui défile : ses libellés longs
  élargissaient la fenêtre au-delà de l'écran). **À droite** :
  - tableau par saisie : objet YOLO (x, y), pince codeurs et pince DREAM (x, y, z), **DREAM ↔
    codeurs en 3D** √(Δx²+Δy²+Δz²), YOLO ↔ codeurs en XY (YOLO ne donne pas de hauteur), vues DREAM ;
  - onglet « distance à la cible » : codeurs (vert), YOLO (jaune), DREAM (magenta), par objet ou
    tous ; un essai avorté (détection fantôme) ne masque plus la vraie saisie ;
  - onglets « trajectoire (arducam) » et « trajectoire (svpro) » : la pointe des codeurs et la
    pointe DREAM dessinées en direct sur l'image, cibles YOLO en croix.
- **Pointe DREAM**, comme `dream_point` de la simulation : par image, PnP keypoints DREAM ↔ FK des
  codeurs → T_DREAM(t) ; pointe = inv(T_réf) · T_DREAM(t) · pointe(q(t)), T_réf remplaçant T_GT.
  Au serrage : médiane des vues des deux caméras sur 2 s.
- **Domaine J5** : les vues avec J5 hors de [45°, 93°] (couverture de `real_tri/train`) sont
  écartées des courbes, du tracé et de la saisie, mais enregistrées (`dans_domaine_j5`). Sur le tri
  de 16:17, erreur de pointe médiane 13-19 mm dedans contre 240-394 mm dehors.
- **Détections hors portée ignorées** (170-430 mm de la base) : une poubelle verte au sol, vue
  « cylindre_vert » à (10 ; −845) mm, lançait un essai sans pose atteignable à chaque tri.
- **Fichiers par essai** (`results/dream_reel/<date>_dashboard/`) : `tri_dream.csv` (par saisie,
  dont Δx, Δy, Δz), `trajectoires.csv` (points datés, codeurs, DREAM fusionné et chaque vue),
  `session.json` (codeurs, images, cycles, vues DREAM) — rejouable par `analyse_tri_dream.py`.
- **Résultat** : tri 4/4 avec DREAM affiché (16:17 et 18:02) ; DREAM ↔ codeurs en 3D au serrage
  8,4 à 14,0 mm (18:02).

## Extrinsèque DREAM à la place des 4 marqueurs (14/10)

**Principe** (DREAM, Lee et al. 2020) : le robot est sa propre mire. Le réseau donne les 7
keypoints 2D, la FK des codeurs les mêmes points en 3D, le PnP la pose de la caméra. Une image ne
suffit pas (7 points presque coplanaires, ~8 mm / 1,4° de dispersion d'une image à l'autre) : on
empile les poses d'un tri dans **un seul PnP**.

`scripts/extrinseque_dream_session.py <session> --ecrire` :

1. images prises pendant le tri, angles interpolés à ±0,3 s, J5 dans le domaine, ≥ 5 keypoints
   (link2 retiré : même point 3D que link1) ;
2. PnP RANSAC (8 px) sur toutes les correspondances, puis affinage robuste (Huber 3 px) ;
3. écrit `training/calibration/<caméra>_extrinsic_dream_tri.yaml` (jamais par-dessus un fichier).

**Aucun marqueur dans le calcul.** L'ArUco ne sert qu'à juger, après coup. Tri de 18:02 :

| | arducam | SVPRO |
|---|---|---|
| images / correspondances gardées | 229 / 1 336 | 289 / 1 623 |
| résidu médian | 1,7 px | 1,7 px |
| pose : DREAM contre ArUco | 4,7 mm, 1,14° | 20,7 mm, 0,56° |
| stabilité (1re moitié du tri contre 2de) | 5,1 mm, 0,53° | 2,2 mm, 0,84° |
| un point de la table ramené par DREAM au lieu de l'ArUco | 7,4 mm médian, 10,6 mm max | 19,7 mm médian |

Sur la table, l'écart arducam est un décalage presque constant d'environ (−5 ; −5) mm. Pour la
SVPRO (+19 mm en X), on ne sait pas lequel des deux a raison : son ArUco est le moins précis
(leave-one-out 4,7-9,4 mm).

**Test de décalage** (`vgg_tri_reel_ft_e10`, image décalée de 20 px) : les keypoints suivent à
96-98 %, `base` compris. Le modèle regarde l'image ; ce n'était pas le cas de `vgg_montage0901`
(0 %, socle mémorisé), dont l'extrinsèque « sans marqueur » du 02/09 était circulaire.

**Tri avec l'extrinsèque DREAM** — `scripts/lancer_tri_extrinseque_dream.py` : le même dashboard,
aucun fichier modifié, `Vision` lit les extrinsèques DREAM des deux caméras et le panneau DREAM
compare à ces mêmes extrinsèques (aucun marqueur nulle part). Sans `--svpro-dream`, YOLO tourne
sur les deux caméras (arducam pour le XY, SVPRO en appui). Les messages de la boucle vont au
journal du tri, et une erreur pendant une saisie arrête la boucle au lieu de la bloquer.

| essai | réglage | résultat |
|---|---|---|
| 18:16 | arducam DREAM, YOLO arducam seule | 4/4, posées à ~1 mm du centre ; YOLO ↔ codeurs 8,8-9,8 mm |
| 18:26 | arducam + SVPRO DREAM, panneau sans marqueur | 3 pièces au centre ; cube_bleu largué à 111 mm du centre de son bac (bac non vu, dernière position reprise) — à vérifier |
| 18:35 | bacs déplacés, rouge et vert à 445 et 477 mm | **hors portée** : largage à 150-184 mm des bacs, cube_bleu gardé en main — disposition, pas DREAM |
| 18:48 | arducam + SVPRO DREAM, YOLO deux caméras, bacs à portée | **4/4**, posées à 1-3 mm du centre ; YOLO ↔ codeurs 7,5-11,6 mm |

Avec l'ArUco, YOLO ↔ codeurs était de 8,2 à 12,6 mm : remplacer les marqueurs par DREAM n'a pas
dégradé la saisie sur ces essais.

**Limites** :

- les labels du fine-tuning réel ont été calculés par l'ArUco (une fois, à l'entraînement) : une
  partie de l'accord à ~5 mm peut en venir ;
- l'extrinsèque DREAM est un **fichier figé**, valable pour la position de caméra du calcul. Si
  une caméra bouge, il faut la réestimer — sans marqueur, en faisant bouger le bras devant elle ;
  aujourd'hui à la main (un tri puis `--ecrire`). Prochaine étape : réestimation continue pendant
  le tri, puis un test avec une caméra légèrement déplacée ;
- le modèle n'a vu que deux positions de caméra : un grand déplacement (autre angle, autre
  distance) sort de son domaine.

## Limites de la référence

- **Extrinsèque valable dans le plan de la table seulement** : tous les marqueurs sont à Z = 0.
  1,92 mm sur la table, mais l'erreur croît en hauteur (> 150 mm mesurés à 17 cm). Les keypoints du
  bras sont en hauteur : leur projection de référence est d'autant moins sûre. Pour lever cette
  limite, recalibrer avec des marqueurs à 2-3 hauteurs.
- **Codeurs ≠ position réelle du bras** : J2 s'affaisse d'environ 1,9° sous la consigne ;
  affaissement gravitaire de ~13 mm à vide, ~15 mm chargé à la pointe. La FK des codeurs porte ce
  biais.
- **Pas de vérité objet** sur le banc : seulement la mesure au robot.
- **Dynamique** : la lecture des codeurs passe par le bridge TCP ; sa latence n'est pas mesurée.

## Ce qui manque avant de commencer

1. **Fait le 14/10** — le module dashboard DREAM réel (étape 9) et l'enregistrement CSV
   (`dream_tri_panneau.py`, `tri_dream.csv`, `trajectoires.csv`, `session.json`).
2. Le **partage des codeurs** : le bridge est mono-client. Le dashboard et DREAM (`joint_sync`,
   `bridge_tour`) ne peuvent pas ouvrir chacun leur connexion ; il faut une seule source d'angles.
   Résolu hors dashboard le 13/10 : `enregistre_tri_dream.py` lit les codeurs sur la connexion du
   tri, sous verrou. Deux `bridge_tour` lancés en double ont fait tomber `gripper_bridge.py` ce
   jour-là (port 5005 fermé, relancé à la main).
3. Le **test de décalage** (étape 3) : script à écrire.
4. La **référence YOLO** (étape 7) : positions des pièces mesurées au robot, boîtes annotées.
5. Les **seuils** : ceux de la simulation (détection > 90 %, médiane < 5 px, rotation < 5°) sont
   proposés, pas validés pour le réel ; aucun seuil en mm pour YOLO ni pour la pointe DREAM.
6. La **charge du PC** : 2 × DREAM + 2 × YOLO + dashboard ; arrêter vers 90 °C.

## Journal

| Date | Étape | État | Preuve / mesure |
|---|---|---|---|
| 13/10/2026 | — | protocole écrit | — |
| 13/10/2026 | 3 — test de décalage | script écrit, non concluant | `scripts/dream_test_decalage.py` ; validé sur image simulée (suivi médian 0,96-0,98) ; sur le banc 2/7 (arducam) et 1/7 (SVPRO) détectés : bras hors champ, SVPRO tournée vers le mur |
| 13/10/2026 | 0 — caméras | décidé | arducam = extrinsèque + YOLO ; SVPRO = DREAM seul, `--svpro-dream` (23 et 26 intermittents sur la SVPRO) |
| 13/10/2026 | 1 — tri YOLO | **atteint** | `tri_fluide.py --toutes` : 4/4, posées au fond, 1-2 mm du centre |
| 13/10/2026 | 4-5 — DREAM statique / dynamique | **mesuré, non tenu** | `results/dream_reel/2026-10-13_16-26-05/` ; arducam médiane 46,8 px (statique) / 47,4 px (dynamique), 3,2 % / 5,1 % < 10 px ; SVPRO détection 73,9 % / 82,5 % |
| 13/10/2026 | 8 — labels SVPRO | fait | `svpro_extrinsic_labels.yaml`, RMS 0,79 px, LOO 4,7-9,4 mm, squelette vérifié |
| 13/10/2026 | 8 — capture | fait | `real_tri/train` 600 poses / `real_tri/test` 100 poses, md5 0 commune ; avant : 84,8 %, médiane 18,7 px |
| 13/10/2026 | 8 — fine-tuning | fait | `vgg_tri_reel_ft_e10`, 128 160 images, 10 époques, 445 min |
| 14/10/2026 | 8 — test réel | **atteint** | `real_tri_test_ndds` 200 images : 84,8 % / 18,73 px → **96,0 % / 1,69 px**, 98,1 % < 5 px ; synthétique inchangé (tri 2,99 px, 50K 2,91 px) |
| 14/10/2026 | 4-5 — tri du 13/10 rejoué, nouveau modèle | mesuré | arducam 2,31 / 2,30 px, SVPRO 1,12 / 1,12 px (statique / dynamique) ; pose d'observation hors distribution : link6 174 px (arducam), 45 px (SVPRO) |
| 14/10/2026 | 9 — dashboard DREAM réel | **atteint** | `dream_tri_panneau.py` ; tri 4/4 (16:17, 18:02) ; DREAM ↔ codeurs 3D au serrage 8,4-14,0 mm ; vues hors domaine J5 écartées |
| 14/10/2026 | 4-5-6 — tri de 16:17 | mesuré | keypoints arducam 1,87 / 1,97 px (immobile / mouvement), SVPRO 5,18 / 5,30 px (caméra déplacée) ; T_DREAM arducam 7,8 / 9,0 mm, 1,4 / 1,8° ; pointe 3D 13-17 mm, surtout Z ; triangulation 2 vues 4,5 mm (test) |
| 14/10/2026 | — extrinsèque DREAM | fait | `extrinseque_dream_session.py` : arducam 4,7 mm / 1,14° de l'ArUco, 7,4 mm sur la table ; décalage d'image suivi à 96-98 % |
| 14/10/2026 | — tri sans marqueur | **4/4** (18:16 et 18:48) | `lancer_tri_extrinseque_dream.py` : extrinsèques DREAM sur les deux caméras, YOLO ↔ codeurs 7,5-11,6 mm ; caméras non déplacées depuis le calcul |
