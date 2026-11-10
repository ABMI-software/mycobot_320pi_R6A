# Article DREAM — plan d'expériences et règles de comparabilité

*Ouvert le 29/10/2026. Document de travail partagé (analyse, état de l'art,
plan d'article) : <https://claude.ai/artifact/BRwGPWB832zmRZYrpSWqmo>. Ce
fichier-ci est la référence **dans le dépôt** : ce qui est mesuré, comment, et
où en est chaque expérience.*

## Thèse visée

Un DREAM entraîné **uniquement sur du synthétique Gazebo à caméras multiples**
atteint sur le banc réel une précision qui permet le pick-and-place, et se
compare chiffre contre chiffre à Lee et al. 2020 (arXiv:1911.09231) : PCK, ADD,
erreur atteinte.

Le fine-tuning réel actuel (`vgg_tri_reel_ft_e10`) reste la **borne haute** :
il dit ce qu'on perd en se passant du réel. Il n'est pas la thèse.

## Ce qu'on ne revendique pas

DREAM **n'a jamais fermé la boucle de commande** d'un tri. En Gazebo, le tri
suit la vérité terrain ou YOLO, et DREAM tourne en parallèle. Sur le banc, le
bras est commandé depuis les codeurs. Dans le « tri sans marqueur » des 14 et
15/10, l'extrinsèque DREAM remplace seulement l'ArUco pour convertir les
détections YOLO en coordonnées robot. La boucle fermée est l'expérience E7.

## Règles de comparabilité (s'appliquent à tout chiffre de l'article)

1. **PCK au sens du papier** : sur tous les keypoints dont la projection de
   référence est dans l'image, **détectés ou non**. Un keypoint non détecté
   compte faux. Seuils 2,5 / 5 / 10 px, plus l'AUC de 0 à 20 px. Outil :
   [`training/dream/eval_papier.py`](../training/dream/eval_papier.py).
   `evaluate_dream.py` rapporte des pourcentages **sur les détectés** : ce
   n'est pas une PCK (98,1 % contre 0,941 à 5 px sur real_tri_test).
2. **ADD** : distance moyenne entre les keypoints 3D transformés par la pose
   estimée et par la pose de référence, en mm. Courbe en fonction du seuil,
   AUC de 0 à 100 mm.
3. **Seuil de pic déclaré** : le code officiel NVlabs utilise 0,01 après
   lissage σ = 3 (`~/DREAM/dream/image_proc.py:925`), alors que le texte du
   papier annonce 0,03. Rapporter 0,01 en chiffre principal et 0,03 en
   sensibilité ; `eval_papier.py` rend les deux (0,03 appliqué après coup sur
   la carte lissée).
4. **Résolution déclarée** : variante Q, H ou F, et taille d'entrée.
5. **Ce qui est tenu à l'écart** : poses, jour, scène, **point de vue**. Un
   test qui ne retient pas un point de vue ne mesure pas la généralisation
   aux vues.
6. **Test de décalage** passé avant tout chiffre réel
   (`scripts/dream_test_decalage.py`, ≥ 95 % sur chaque keypoint).
7. **Référence indépendante** : si la référence a servi aux labels, le dire
   et borner la conclusion.
8. **Effectif et dispersion** : n, médiane, p90 ; pour un entraînement,
   au moins 3 graines dans les ablations.
9. **Aucun chiffre dans l'article sans entrée au registre**
   ([`training/dream/article/registre_resultats.yaml`](../training/dream/article/registre_resultats.yaml))
   portant le verdict `tenu` de l'agent `metrologue`.

## Expériences

Chaque expérience suit `.claude/skills/campagne-mesure` (protocole écrit avant,
`run.yaml`, CSV bruts archivés hors du dépôt). L'état est tenu à jour ici.

| Id | Expérience | Dépend de | Critère de sortie | Priorité | État |
|---|---|---|---|---|---|
| E1 | Métriques du papier (PCK + AUC, ADD, seuils 0,01 et 0,03) recalculées pour les modèles existants | — | Tableau au format du tableau I du papier pour les jeux A, E, E', G du registre | rouge | fait le 29/10 : tableau pour A, E, E', G avec PCK 7 et 6 points, ADD, deux seuils, IC bootstrap par blocs, md5 train/test (registre, `~/dream_article/E1/`). Tout reste `a_preciser` : référence ArUco et point de vue unique, que seul E2 lève |
| E2 | Jeu réel multi-vues : 6 à 10 positions de caméra jamais vues, référence indépendante de DREAM | — | Jeu figé, md5, positions relevées, leave-one-out de l'extrinsèque publié | rouge | protocole rédigé le 10/11 ([`PROTOCOLE_E2_MULTIVUES.md`](PROTOCOLE_E2_MULTIVUES.md)) ; session au banc à faire ; préalables 1-2 (capture, géométrie) faits le 10/11, 3-4 (bloc en hauteur, supports) au banc |
| E3 | Synthétique seul multi-vues : caméra libre + randomisation du papier + augmentation v5, aucune image réelle | E1 | PCK@5 ≥ 0,83 et PCK@10 ≥ 0,90 sur E2 ; test de décalage ≥ 95 % ; ADD ≥ 0,80 à 20 mm | rouge | en cours : feuille de route E3.1-E3.10 ci-dessous |
| E4 | Ablations : résolution Q/H/F, encodage des cartes (entier contre sous-pixel), volume 25k/50k/100k, perte L2 contre pondérée, recette du papier, randomisation par bloc | E3 | Un tableau par facteur, 3 graines minimum | jaune | à faire |
| E5 | Plusieurs images contre hand-eye (marqueur sur la bride) : erreur en fonction du nombre d'images | E2 | Courbe équivalente à la Fig. 6 du papier | jaune | à faire |
| E6 | Erreur atteinte : 10 cibles (5 marqueurs × 2 hauteurs), une série par extrinsèque (ArUco, hand-eye, DREAM), ISO 9283 | E5 | Tableau équivalent au tableau II du papier | jaune | à faire |
| E7 | Boucle fermée réelle : pick-and-place avec l'extrinsèque DREAM synthétique seul, puis pointe de pince estimée par DREAM | E3, E6 | ≥ 20 essais, taux de réussite et erreur de saisie | jaune | à faire |
| E8 | Schéma de keypoints étendu : un point en aval de J6 (bride ou doigts) | E3 | J6 observable à 2 vues, erreur médiane < 5° | vert | à faire |

E1 et E2 se font en parallèle. E4 réutilise le générateur de E3. E8 change le
format des labels : à lancer une fois E3 stabilisé.

### Quantification des cartes de croyance (constaté le 29/10)

DREAM apprend chaque keypoint sur une gaussienne centrée sur `int(x)` en
sortie réseau (`~/DREAM/dream/image_proc.py:866`) et décode le pic à
+0,4395. L'erreur, uniforme sur [−0,56 ; +0,44] px réseau, vaut ×4,8 en pixels
image en Q, de moyenne −0,29 px (c'est le −0,3 px mesuré sur link3-6). Seule,
avec une phase uniforme, elle donne un PCK@2,5 de **0,79** (erreur médiane
1,92 px, max 3,80 px) ; c'est une référence et non un plafond strict : nos
keypoints mobiles font un peu mieux (0,81-0,84, médiane 1,6-1,7 px). H
(200×200, même décalage) borne l'erreur à 1,90 px ; F (400×400) décode sans
décalage (`network.py:535-538`), erreur dans ]−1 ; 0] px réseau, max 1,70 px.
Les points fixes dans l'image (base, link1, link2) restent sur une seule phase :
(+2,1 ; −2,6) px pour base, (+2,1 ; −2,1) px pour link1-2 sur la caméra du
dessus, constants sur nos modèles synthétiques. Conséquences :

- Le PCK@2,5 en Q mesure surtout cette quantification. Ne le comparer qu'à
  une variante de même résolution de sortie ; PCK@5 et PCK@10 restent sous
  le maximum de 3,8 px de la quantification.
- Elle n'explique pas le PCK@2,5 réel du papier (0,24, loin de 0,79) : là,
  l'écart vient du transfert sim-réel.
- Pour E3 : `train_dream_ultimate_v5_geo.py --encodage sous-pixel` dessine la
  gaussienne à la position exacte moins le décalage du décodeur, qui reste
  celui de DREAM (rien ne change à l'inférence ni dans `~/DREAM`). Cibles
  décodées contre labels, sur `real_tri_test` : entier 2,10 px médiane /
  3,63 max ; sous-pixel σ 2 : 1,23 / 2,02 (la fenêtre 5×5 du décodeur tronque
  la gaussienne) ; sous-pixel σ 1 (`--sigma-carte 1`) : 0,18 / 0,28. Un σ plus
  étroit est plus dur à apprendre : encodage et σ sont des facteurs de E4.

### Feuille de route de E3 (tenue à jour à chaque session)

`scripts/etat_session.sh` affiche au démarrage la première étape qui n'est pas
« fait ». Une étape ne passe à « fait » qu'avec son critère atteint ; ce qui a
été appris en route va dans « Ce qu'on a appris », plus bas.

| Étape | Contenu | Critère de sortie | État |
|---|---|---|---|
| E3.0 | Référence : synthétique seul sur le jeu caméra libre de septembre (17 702 images, pince montée, sans randomisation d'apparence), encodage DREAM | Checkpoint `vgg_e3_0_camlibre_synth_e50`, évalué sur la vue tenue et `real_tri_test` | fait le 03/11 (registre E3.0, `a_preciser`) : azimut tenu à l'écart PCK@5 0,932 ; réel `real_tri_test` PCK@5 0,014, détection 0,534 — ne transfère pas sans randomisation d'apparence |
| E3.1 | Collecteur caméra libre : contrôle visuel des labels reprojetés | 24 images, keypoints superposés au robot à l'œil sur toutes | fait le 29/10 : jeu de septembre (23 272 images collectées, 17 702 + 2 145 vue tenue en NDDS), labels sur le robot partout, vue tenue comprise |
| E3.2 | Masque du robot : caméra de segmentation au même endroit que la caméra libre | Masque par image, aligné au pixel sur le rendu | fait le 29/10 : capteur `cam_libre_masque` (label 1 via `mycobot_pro_320_pi_gazebo_masque.urdf.xacro`), `images/masque/` + `masques.csv` ; capture resynchronisée sur l'horodatage : 31/40 masques du même rendu que l'image, tous alignés au pixel (planche de 24), les 9 autres marqués `frais = 0` |
| E3.3 | Fonds aléatoires : composition hors ligne derrière le masque, sur une part des images | Source d'images choisie (décision utilisateur), part fixée, 50 images contrôlées | fait le 03/11 : COCO val2017 (5 000 images, comme le papier, `~/dream_article/fonds/coco/val2017`) ; `training/dream/composer_fonds.py --part 0.5` par défaut (la part est un facteur de E4) ; 31 trames composées sur 40, keypoints en place sur la planche de 24 |
| E3.4 | Distracteurs : primitives aux couleurs tirées, apparues et retirées à chaque lot, occultations comprises | Labels inchangés, collecte stable sur 1 000 images | fait sur 80 images le 03/11 : 2-6 formes visuelles seules par lot de 40 trames, via le pont ROS (`create`/`remove`), présentes dans les 80 images, consignées dans `distracteurs.csv` et les annotations ; stabilité sur 1 000 images à vérifier au pilote de E3.7 |
| E3.5 | Recoloration du robot et lumières libres (couleur, nombre) | Diversité visible sur une planche de 50 images | fait le 03/11 : 0-3 lumières ajoutées par lot de 40 trames (point ou spot, 40 % teintées, `lumieres.csv`), robot teinté hors ligne par le masque (`composer_fonds.py --part-teinte 0.3`, ombrage conservé) ; planche de 24 sur 45 trames |
| E3.6 | Pince montée | Pince visible, keypoints identiques au schéma à 7 points | fait : le launch caméra libre monte déjà la pince Pro (`mycobot_pro_320_pi_gazebo.urdf`), visible sur la planche de E3.1 |
| E3.6b | Annotations complètes pour d'autres méthodes (RoboPEPP, HPE…) : pose caméra-robot, 12 articulations, 18 liens en 3D et 2D, doigts, masque, boîtes | JSON validés sur une collecte (écart FK ≤ 0,1 mm, points superposés à l'œil), lus par DREAM | fait le 03/11 : format dans `docs/SYNTHETIC_DATA.md`, 20 trames, écart 0,02 mm, `eval_papier.py` les lit |
| E3.7 | Collecte : volume visé ~100k (papier), un secteur d'azimut tenu à l'écart, `run.yaml`, md5 | Jeu NDDS figé, md5, secteur tenu à l'écart documenté | en cours : en cours : pilote de 1 000 images fait le 03/11 (`~/dream_article/E3/E3_7_pilote/`) — complet, 0 gel, 97,5 % de masques frais, mais 9,1 s/image en moyenne (7,9 → 10,1 s, en dérive) et articulations lues jusqu'à 0,73 s de l'image ; articulations interpolées au temps de l'image, 4,1 s/image ; **50 000 images** décidées le 04/11 (10 tranches de 5 000, `scripts/collecte_tranches.sh`, `~/dream_article/E3/E3_7/journal.txt`), 100k à décider après E3.9 ; au 10/11, 8 tranches closes (39 740 images), la 9e en cours, fin prévue le 11/11 |
| E3.8 | Pilote 10k : encodage entier σ 2 contre sous-pixel σ 1 | Choix de l'encodage pour E3.9, sur PCK synthétique tenu à l'écart | à faire |
| E3.9 | Entraînement synthétique seul (VGG ImageNet, sans `--pretrained` réel), v5_geo | Checkpoint + `run.yaml` | à faire |
| E3.10 | Évaluation : E2, `real_tri_test`, test de décalage, revue du metrologue | Critère de E3 (tableau des expériences) | à faire |

E3.10 dépend de E2 (jeu réel multi-vues), qui demande une session au banc : la
préparer pendant que la collecte et l'entraînement tournent.

### Protocole de E3.8 (rédigé le 10/11, avant mesure)

- **Objectif** : choisir l'encodage des cartes de croyance de E3.9 sur du
  synthétique seul, sans regarder une image réelle.
- **Données** : les tranches 01 et 02 de E3.7, composées (~9 850 images),
  converties avec `--azimut-tenu 60,100`. Entraînement sur le reste, **test
  sur le secteur tenu à l'écart** de ces deux tranches (~1 100 images).
- **Trois bras**, identiques à tout le reste près (VGG-19 ImageNet, sans
  `--pretrained`, `train_dream_ultimate_v5_geo.py`, 50 époques, patience 8,
  graine 42, `taskset -c 16-27`), ~3 h chacun :
  `--encodage entier` (σ 2, DREAM) ; `--encodage sous-pixel --sigma-carte 2` ;
  `--encodage sous-pixel --sigma-carte 1`. Le bras du milieu sépare l'effet de
  l'encodage de celui de σ.
- **Mesure** : `eval_papier.py` sur le secteur tenu, PCK@2,5/5/10 (non détecté
  = faux), AUC 0-20 px, détection ; IC bootstrap par pose (3 vues d'une pose
  dans le même bloc), et IC de la **différence** à l'encodage entier.
- **Critère de sortie** : on retient le sous-pixel σ 1 si son PCK@2,5 dépasse
  l'entier d'au moins **0,05** avec un IC de la différence qui exclut 0, sans
  perdre plus de 0,01 en PCK@5, PCK@10 et détection ; sinon le sous-pixel σ 2
  à la même règle ; sinon l'entier. Choix **provisoire** (une graine) : E4
  le rejoue sur 3 graines.
- **Ne pas faire** : départager sur `real_tri_test` ou sur E2 (ce serait choisir
  sur le test) ; prolonger un bras qui perd « pour lui laisser sa chance ».

### Point de départ de E3

- Générateur : `mycobot_gateway/synthetic_data_collector_v3_camera_libre.py`
  (pose de caméra tirée à chaque image, garde au sol 50 mm). À ajouter :
  fonds d'images, distracteurs, recoloration du robot, trois lumières libres
  (§ II.C du papier), pince montée.
- Entraînement : `training/dream/train_dream_ultimate_v5_geo.py` (géométrie
  appliquée avec les labels). **Sans** `--pretrained` depuis un modèle affiné
  sur du réel, qui porterait le montage mémorisé.
- Garde thermique : cœurs E (`taskset -c 16-27`), voir
  [`DONNEES_ET_CONSOMMATION.md`](DONNEES_ET_CONSOMMATION.md).

## Ce qu'on a appris (tenu à jour)

Chaque constat porte sa date. Quand un constat change, on le **corrige ici
même** (« corrigé le JJ/MM : … ») plutôt que d'en empiler un contradictoire.

- **10/11 — La SVPRO fixe est dans le secteur tenu à l'écart** : azimut 70°,
  0,98 m, élévation 25° (`geometrie_camera.py`, convention du synthétique) ;
  l'arducam est à −109°, 0,88 m, 74°. La moitié SVPRO de `real_tri_test` est
  donc un point de vue que le synthétique de E3 n'a pas vu : à séparer par
  caméra en E3.10. Mais elle est vue par `vgg_tri_reel_ft_e10` (borne haute),
  d'où l'écart imposé aux positions E2 de ce secteur.
- **10/11 — Positionnement (bibliographie)** : « synthétique seul vers réel »
  est déjà publié sur Panda (DREAM, RoboPose, SGTAPose) et ne peut pas être
  revendiqué. Aucun papier ouvert n'évalue sur un **point de vue réel tenu à
  l'écart** : c'est l'apport possible, avec un 6 axes bas coût mesuré au
  format PCK/ADD de DREAM (CRAVES traite déjà un bras bas coût, autrement).
  Les seules références comparables en synthétique seul sont DREAM, RoboPose et
  SGTAPose ; les autres (CtRNet, HPE, RoboPEPP…) s'adaptent sur du réel non
  étiqueté. Détail et lignes encore à vérifier : `training/dream/article/biblio.md`.
- **29/10 — PCK** : `evaluate_dream.py` ne compte que les détectés ; au sens du
  papier, 98,1 % « < 5 px » devient 0,941 sur `real_tri_test`.
- **29/10 — Seuil de pic** : 0,01 (code) ou 0,03 (texte) ne change rien au-delà
  de 0,001 sur nos modèles, sauf hors domaine. Un pic net vaut ~0,30 après
  lissage.
- **29/10 — Intrinsèques** : `real_tri_test` mélange arducam et SVPRO, son
  `_camera_settings.json` ne décrit que l'arducam, et les labels incluent la
  distorsion. Tout PnP sur un jeu réel prend K et distorsion par caméra
  (`source.camera`).
- **29/10 — Quantification des cartes** : voir la section dédiée. Le biais de
  (+2,1 ; −2,6) px sur base, link1, link2 n'est pas un biais d'étiquette
  (hypothèse du metrologue, réfutée le jour même) mais l'arrondi `int(x)` de
  DREAM sur des points fixes dans l'image.
- **29/10 — Encodage sous-pixel** : compenser seulement l'arrondi ne suffit
  pas ; à σ 2, la fenêtre 5×5 du décodeur garde 1,1-1,2 px d'erreur. Le gain
  vient de σ 1 (0,18 px médiane sur les cibles). Reste à savoir si le réseau
  l'apprend (E3.8).
- **29/10 — E' et pose d'observation** : sur un tri réel, la pose
  d'observation (J5 ≈ 2,6°) coûte ~12 points de PCK@5 ; le régime
  « statique » d'une session de tri est surtout cette pose.
- **29/10 — Effectif** : les images d'une session à ~0,2 s d'intervalle sont
  corrélées ; un IC se calcule par blocs (2 s ou par pose), jamais sur n images.
- **03/11 — E3.0, diversité de caméra sans apparence** : synthétique seul sur
  17 702 images caméra libre, le modèle est bon dans son domaine (secteur
  d'azimut tenu à l'écart : PCK@5 0,932, médiane 1,85 px) et nul sur le banc
  réel (PCK@5 0,014, détection 0,534 ; papier en synthétique seul : 0,83).
  Formulation tenue par le metrologue : « sans randomisation d'apparence, un
  modèle synthétique seul à caméra libre ne transfère pas à nos deux caméras
  réelles ». On ne peut pas en conclure que la diversité de caméra n'aide pas :
  il faudrait un témoin à caméra fixe et 3 graines. Le « secteur tenu à
  l'écart » est une interpolation d'azimut (élévation, distance, décor
  partagés), pas un point de vue absolument nouveau.
- **03/11 — Composition des fonds (E3.3)** : elle se fait hors ligne sur la
  collecte brute, avant la conversion NDDS, dans un dossier jumeau ; labels et
  poses de caméra sont copiés tels quels. Le robot flotte sans table ni ombre,
  comme dans le papier. Les 4 caméras fixes de l'URDF sont des liens du modèle
  robot : elles portent son label et restent dans le masque, petites boîtes
  sombres sur certains fonds (déjà présentes dans les rendus bruts).
- **03/11 — Garde au sol de la pince fausse depuis septembre** : le
  collecteur caméra libre surveillait le « bout de la pince » en (0, −0,107,
  0,056) dans le repère de link6, repris du collecteur `v4_gripper`. Or la
  pince s'étend selon +z de link6 (doigts à z ≈ 0,136 m) : le point était
  10 cm à côté. Sur les 23 270 images de septembre (jeu de E3.0), **1,0 % ont
  les doigts dans la table**, jusqu'à −47 mm, la garde croyant tenir 50 mm.
  Corrigé dans le collecteur (bouts des deux doigts et leur milieu, tirés des
  maillages). Les collectes suivantes en sont exemptes ; E3.0 reste tel quel,
  ce défaut est à citer avec lui.
- **03/11 — Masque et caméras fixes** : le label de segmentation couvre tout le
  modèle, y compris les 4 caméras de l'URDF ; la boîte du masque en devenait
  fausse ([0, 13, 460, 382] pour un bras de 200 px). On ne garde que les
  régions touchées par un keypoint (`masque_du_robot`), dans les annotations
  comme dans la composition des fonds.
- **03/11 — Distracteurs (E3.4)** : visuels seuls (ni collision ni
  physique), ils ne bloquent pas le bras mais peuvent le traverser quand il
  bouge pendant un lot ; le robot réel n'aura jamais d'objet en travers. Le
  rayon minimal autour du socle compte désormais la taille de l'objet. Les
  occultations restent rares (4 keypoints sur 413) : objets petits et
  dispersés. Écart au papier : formes simples au lieu des objets YCB.
- **03/11 — Pilote de 1 000 images (E3.7)** : collecte complète sans gel ni
  plantage, 975 masques frais sur 1 000 (97,5 %), 83 lots de distracteurs,
  35 de lumières. Trois constats :
  - **Le bras bouge encore pendant les vues** : les 3 vues d'une « même » pose
    diffèrent de 13° (médiane) ; il converge lentement vers sa cible (J6 à
    ~7°/s), bien après `move_time` + `settle_time`. Cause probable : le gain
    proportionnel par défaut de l'interface `position` de `gz_ros2_control`,
    non réglé dans l'URDF (partagé avec le tri, non modifié). Les étiquettes
    restent celles des angles mesurés.
  - **Articulations et image pas toujours synchrones** : écart médian 20 ms,
    p95 0,12 s, max 0,73 s, soit jusqu'à ~7° d'erreur d'étiquette sur un bras
    en mouvement. Corrigé le 03/11 : attendre ne suffit pas (l'image arrive avec
    jusqu'à ~1 s de retard sur `/joint_states`, par la latence du pont image),
    donc le collecteur **interpole les articulations au temps simulé de
    l'image** dans un historique de 3 s (mesures à 4 ms d'écart) : 100/100
    trames interpolées, liens superposés au robot même quand il a bougé de
    20-29° entre deux vues. Le pilote, collecté avant, garde ce défaut.
  - **Cadence** : 9,1 s/image, dont l'essentiel en relances de stabilisation
    (≈ 4 par image, le bras n'atteignant presque jamais sa cible à 2°), et une
    dérive de 7,9 à 10,1 s sur 1 000 images. À ce rythme, 100 000 images
    prennent ~11 jours. Avec `max_settle_skips:=1` (les étiquettes étant
    interpolées, viser la cible au degré près n'apporte rien) : **4,1 s/image**,
    ~5 jours pour 100 000. La dérive impose de collecter par tranches relancées.
    Corrigé le 10/11 : sur les tranches de 5 000 de E3.7, la dérive est bien
    plus forte qu'au pilote : **3,7 s/image au départ, 11,5 au milieu, 23 à la
    fin** (tranches 03, 08, 09, reproductible), soit ~16,5 h par tranche et
    ~12 s/image en moyenne au lieu de 4,1. Quatre tranches sur huit ont touché
    le délai de `collecte_tranches.sh` (12 s/image) à 4 866-4 988 images :
    données valides, marquées `fait` à la main pour qu'une relance ne les
    écrase pas. Cause non établie (piste : entités `create`/`remove` qui
    s'accumulent côté rendu). Des tranches de ~1 000 images tiendraient
    ~5 s/image en moyenne.
- **03/11 — Lumières ajoutées et caméra de segmentation** : une lumière posée
  par `create` porte par défaut un repère visuel sans label, et la caméra de
  segmentation plante dessus dès la première image (`Ogre::
  ItemIdentityException` dans `Ogre2SegmentationCamera::Render`, Gazebo
  s'arrête, 3 images sur 80). `<visualize>false</visualize>` dans le SDF de
  la lumière règle le problème (45 images sans plantage). Toute entité ajoutée
  à la scène doit être testée avec le capteur de masque actif.
- **03/11 — Recoloration du robot** : sans liaisons Python de gz-transport,
  `visual_config` n'est joignable que par la ligne de commande `gz`, celle
  qui avait empoisonné la découverte en septembre. Le robot est donc teinté
  hors ligne, par le masque (multiplication des pixels, ombrage conservé), au
  même titre que la composition des fonds.
- **03/11 — Rendu logiciel** : les notes de juin (collecteur v3) disent que
  Gazebo peut tomber en rendu logiciel sur cette machine hybride (Intel +
  NVIDIA ; `libEGL: failed to create dri2 screen`) et figer des images. Le
  message apparaît encore, alors que `nvidia-smi` montre le serveur sur le
  GPU. Pour la grande collecte, forcer le GPU :
  `__EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
  __GLX_VENDOR_LIBRARY_NAME=nvidia __NV_PRIME_RENDER_OFFLOAD=1`.
- **03/11 — ADD et PCK** : à PCK@5 0,93, l'ADD médiane vaut 17,7 mm mais le
  p90 65 mm : 6 points quasi alignés conditionnent mal le PnP en profondeur.
  Toujours donner la part sous 20 mm et l'AUC avec échecs comptés.
- **29/10 — Jeu caméra libre existant** : 17 702 images NDDS + 2 145 d'un
  secteur d'azimut tenu à l'écart (60-100°, `convert_to_ndds_camera_libre.py
  --azimut-tenu`), pince montée, collectées en septembre. Aucun modèle
  synthétique seul n'y avait été entraîné : `vgg_v5_geo_ft_e30` les mélange à
  du réel. D'où E3.0.
- **29/10 — Apparence** : la planche de E3.1 montre la même table, le même sol
  gris et les trois mêmes objets sur toutes les images. Seuls la caméra, la
  pose et l'éclairage varient : c'est l'écart principal avec le papier.
- **29/10 — Gazebo lancé depuis Claude Code (VSCode en snap)** : l'interface
  plante au démarrage (`symbol lookup error … /snap/core20 … libpthread`) et,
  le processus Gazebo étant `required`, le launch s'arrête sans rien
  collecter. Lancer avec `env -u GTK_PATH -u GTK_EXE_PREFIX -u LOCPATH -u
  GIO_MODULE_DIR -u GSETTINGS_SCHEMA_DIR -u GTK_IM_MODULE_FILE`. Un terminal
  normal n'est pas touché.
- **29/10 — Collecte et entraînement ne cohabitent pas sur le GPU** : pendant
  l'entraînement E3.0, Gazebo tournait à 0,08× le temps réel (2,4 s simulées
  en 31 s) ; il attend chaque rendu de capteur, et les contrôleurs ratent leur
  activation (« Switch controller timed out after 5 seconds »), donc le bras
  ne bouge pas et rien n'est collecté. Pour un essai court, mettre
  l'entraînement en pause (`kill -STOP -<pgid>`, puis `-CONT`) ; pour une
  collecte longue, ne pas les lancer ensemble. Gazebo sur les cœurs 16-19,
  l'entraînement sur 20-27.
- **29/10 — Masque du robot (E3.2)** : la caméra de segmentation de Gazebo
  Harmonic (`type="segmentation"`, `semantic`) et le système `Label` sur le
  modèle robot donnent un masque aligné quand il vient du même rendu que
  l'image. Mais le flux du masque traîne de 0,1 à 0,7 s derrière l'image : 9
  paires synchrones sur 40 en prenant le dernier message reçu, et ces masques
  en retard suivent une autre pose du bras. Le CRC ne les voit pas, seul
  l'horodatage les distingue. Le collecteur réessaie donc la capture toutes
  les 0,1 s jusqu'à 1,5 s, bras immobile : 31 paires synchrones sur 40, les
  autres marquées périmées.
- **29/10 — Contrôleurs au démarrage** : avec le second capteur, le monde
  charge plus longtemps et `joint_state_broadcaster` rate l'activation à 5 s.
  Les spawners du launch caméra libre ont `--switch-timeout 60
  --service-call-timeout 60`, et le collecteur attend `/joint_states` et un
  masque avant la première pose.
- **29/10 — Arrêter une collecte** : le launch ne s'arrête pas quand le
  collecteur a fini, et un `ros2 launch` tué laisse Gazebo orphelin. Lancer
  sous `setsid`, attendre « Collection complete » dans le journal, puis
  `kill -INT -<pgid>`. Jamais `pgrep -f "gz sim"` dans une boucle d'attente :
  il se trouve lui-même.
- **29/10 — Mettre un entraînement en pause sans le tuer** : un entraînement
  lancé en tâche de fond par Claude Code meurt si on le suspend (`kill -STOP`)
  : le shell qui l'a lancé prend le processus suspendu pour terminé (`code
  147`), sort, et l'outil nettoie le groupe de processus. E3.0 a perdu ses 6
  premières époques ainsi, et la garde thermique (pause à 90 °C) aurait eu le
  même effet. Lancer tout entraînement long détaché : `setsid nohup … >
  journal 2>&1 < /dev/null &`, puis suivre le journal ; la pause et la garde
  thermique (`scripts/garde_thermique.sh <pid> <journal>`) deviennent sûres.
- **29/10 — Données** : jeux, checkpoints et sessions réelles sont dans
  `~/Osama_ws` (216 Go pour `dream_data`), lus sur place ; les sorties de
  mesure vont dans `~/dream_article/<expérience>/`, hors du dépôt.

## Rôles (outillage `.claude/`)

| Phase | Qui | Sortie |
|---|---|---|
| Orchestration, choix de la prochaine expérience | skill `article-dream`, commande `/article` | Plan de la session |
| État de l'art, chiffres publiés des concurrents | agent `veilleur-biblio` | `training/dream/article/biblio.md` |
| Génération synthétique, monde Gazebo | agents `digital-twin-engineer`, `urdf-surgeon` | Jeu NDDS + `run.yaml` |
| Entraînement, évaluation | agent `dream-trainer` | Checkpoint + sortie d'évaluation brute |
| Banc réel (E2, E5-E7) | skills `real-robot-session`, `precision-iso9283` | Dossier d'essai |
| Contrôle de chaque chiffre | agent `metrologue` | Verdict dans le registre |
| Rédaction | agent `redacteur-article` | `training/dream/article/manuscrit/` |
| Cohérence de la doc en fin de jalon | agent `gardien-doc` | Liste de corrections |
