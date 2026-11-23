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
| E3.7 | Collecte : volume visé ~100k (papier), un secteur d'azimut tenu à l'écart, `run.yaml`, md5 | Jeu NDDS figé, md5, secteur tenu à l'écart documenté | en cours : en cours : pilote de 1 000 images fait le 03/11 (`~/dream_article/E3/E3_7_pilote/`) — complet, 0 gel, 97,5 % de masques frais, mais 9,1 s/image en moyenne (7,9 → 10,1 s, en dérive) et articulations lues jusqu'à 0,73 s de l'image ; articulations interpolées au temps de l'image, 4,1 s/image ; **50 000 images** décidées le 04/11 (10 tranches de 5 000, `scripts/collecte_tranches.sh`, `~/dream_article/E3/E3_7/journal.txt`), 100k à décider après E3.9 ; au 10/11, 8 tranches closes (39 740 images), la 9e en cours ; la 10e en 5 sous-tranches de 1 000 (masques et cadence dérivent dans une tranche de 5 000) ; tranches 01-08 recomposées à `--part 0.85 --part-teinte 0.5` (~59 % de masques à jour, d'où ~50 % de fonds et ~30 % de teintes sur l'ensemble) |
| E3.7b | Complément de E3.7 avec les poses du pick (40 % du tirage), 30 × 1 000 (porté de 10 à 30 le 12/11, voir « Chaîne du 12/11 ») | Pointe de pince à 72-115 mm sur une part nettement supérieure aux 1,7 % de E3.7 ; même format, md5 | fait le 14/11 : 30 tranches collectées puis composées (masques à jour 0,95, ~53 % de fonds, ~32 % de teintes) ; jeu E3.9 = 49 595 images d'entraînement + 6 125 du secteur tenu, 24 014 images à angles non interpolés écartées (`~/dream_article/E3/E3_9/run.yaml`, md5 des labels) |
| E3.8 | Pilote 10k : encodage entier σ 2 contre sous-pixel σ 1 | Choix de l'encodage pour E3.9, sur PCK synthétique tenu à l'écart | fait le 13/11 : **sous-pixel σ 1 retenu** par la règle écrite d'avance (484 images à angles interpolés : Δ PCK@2,5 +0,149 [+0,132 ; +0,167], Δ PCK@5 +0,042 [+0,031 ; +0,052], pas de perte à 10 px ni en détection) ; sous-pixel σ 2 : +0,037 seulement ; 1 graine par bras (`~/dream_article/E3/E3_8/choix.txt`) ; verdict du metrologue à inscrire |
| E3.9 | Entraînement synthétique seul (VGG ImageNet, sans `--pretrained` réel), v5_geo | Checkpoint + `run.yaml` | fait le 15/11 (`vgg_e3_9_synth_seul`) : secteur 60-100° tenu, 6 125 images, PCK 0,910/0,968/0,984, détection 0,994 ; **réel (`real_tri_test`, 200 images) : PCK 0,004/0,068/0,169, détection 0,70, ADD médiane 83 mm** (E3.0 : 0,014 à 5 px, 173 mm) — le transfert au réel reste un échec ; biais vertical constant de la base (−4 à −6 px, MAD 0,3 px, deux caméras) à expliquer avant E3.10 (`~/dream_article/E3/E3_9/`) |
| E3.9b | Diagnostic : les 100 poses de `real_tri_test` rejouées en Gazebo aux caméras d'étiquetage — monde d'entraînement (C), réplique du banc (A), et rendu `realistic` (B) ; étiquettes exactes | Part de l'échec réel due au cadrage, à la scène, à l'apparence du robot, au reste (protocole ci-dessous) | protocole écrit le 18/11, amendé le 19/11 (metrologue : condition C, équivalence, cadrage arducam hors domaine) ; outil `scripts/rejouer_poses_banc.py` écrit, mondes générés (`~/dream_article/E3/E3_9b/`) ; chaîne lancée le 19/11, arrêtée le 22/11 à l'essai (monde C sans système `Sensors`, corrigé le 23/11, voir « Ce qu'on a appris ») ; relancée le 23/11 avec le contrôle C_ogre (`--pas 3`, `analyse_moteur.py`), en attente de la carte (RoboPEPP run 4) |
| E3.10 | Évaluation : E2, `real_tri_test`, test de décalage, revue du metrologue | Critère de E3 (tableau des expériences) | à faire |
| E3.11 | Modèle de déploiement : même recette que E3.9, tous azimuts (secteur 60-100° compris), hors article | Checkpoint pour le banc, si E3.10 atteint son critère | à faire (décision du 12/11) |
| E3.12 | Campagne sim-to-réel ciblée, décidée le 19/11 : A diagnostic sans collecte (E3.9b, distorsion, cadrage, échelle sur les images réelles), A5 réentraînement sur les 49 595 images avec randomisation du modèle de caméra, B collecte de 100k avec les leviers retenus, C 50k contre 100k, D évaluation | Le levier de l'écart sim-réel, puis le meilleur modèle synthétique seul (protocole ci-dessous) | protocole écrit le 19/11, amendé après relecture du metrologue (bootstrap par bloc, deux graines, contrôle d'identité, sous-ensemble fixe, décision de B par l'utilisateur), avant mesure ; à lancer quand RoboPEPP libère la carte graphique |

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

### Protocole de E3.9b (rédigé le 18/11, amendé le 19/11 après relecture du metrologue, puis le 23/11 sur le moteur de rendu, avant mesure)

- **Objectif** : dire quelle part de l'échec réel d'E3.9 (PCK@5 0,068 sur
  `real_tri_test`) tient à l'apparence du robot, à la scène du banc, au
  cadrage des caméras réelles, ou à ce que la simulation ne modélise pas. La
  simulation donne des étiquettes **exactes** : c'est ce qui sépare ces
  facteurs d'une erreur d'étiquette.
- **Point de vue, établi avant mesure** (19/11, extrinsèques d'étiquetage,
  autour du point visé du collecteur ; relu par le metrologue) :

  | | azimut | élévation | distance | roulis | visée hors axe |
  |---|---|---|---|---|---|
  | arducam (`avant_1410_1454`) | −102,7° | 75,3° | 0,878 m | 2,2° | 15,8° |
  | SVPRO (`labels.avant_1410`) | 70,2° (secteur tenu) | 24,9° | 0,976 m | 3,6° | 8,0° |

  E3.9 sur le secteur synthétique tenu, à ce point de vue de la SVPRO
  (élévation 20-30°, distance 0,90-1,05 m) : PCK@5 **0,976** [0,963 ; 0,987],
  n = 198. **Le point de vue de la SVPRO n'explique pas son échec.** L'arducam,
  elle, est **hors domaine par le cadrage** : à l'entraînement, la visée n'est
  jamais à plus de 9,0° de l'axe optique (critère `_robot_dans_le_champ` du
  collecteur, qui rejetterait cette pose), contre 15,8° ; le PCK@5 tenu baisse
  déjà de 0,971 (0-5°) à 0,953 (5-10°). Son échec (détection 0,454) peut en
  partie venir de là (`~/dream_article/E3/E3_9b/tranche.py`).
- **Référence réelle, par caméra** (E3.9 sur `real_tri_test`) : arducam
  détection 0,454, PCK@5 0,021 ; SVPRO détection 0,946, PCK@5 0,114.
- **Trois conditions**, mêmes 100 poses, mêmes caméras (poses des
  extrinsèques **qui ont étiqueté** `real_tri_test`, intrinsèques `cam_3` et
  `cam_2` rééchelonnée en 640 × 480, **sans distorsion**) :
  - **C** = monde d'entraînement (`randomized`, celui de la collecte), sans
    fonds COCO, teinte ni distracteurs, rendu **ogre2** comme E3.7 ;
  - **A** = réplique du banc (`banc_realiste`), `robot_appearance:=original`,
    rendu **ogre** ;
  - **B** = réplique du banc, `robot_appearance:=realistic`, rendu **ogre**.

  **Amendement du 23/11, avant mesure** : ogre2 plante sur les matériaux
  des ArUco de la réplique, A et B sont donc en ogre. Décision de
  l'utilisateur : garder C en ogre2, au plus près du rendu d'entraînement,
  plutôt que d'aligner les trois moteurs. Conséquence : **C − A mesure la
  scène et le moteur de rendu ensemble**, sans pouvoir les séparer. A − B
  reste propre (même scène, même moteur).

  **Contrôle C_ogre, fixé avant mesure** (proposé par le metrologue le 23/11) :
  le monde C est aussi rendu en ogre, sur les poses d'indice multiple de 3
  (34 poses, deux caméras, ~30 min de Gazebo). Cela donne C − C_ogre (le
  moteur seul) et C_ogre − A (la scène seule), sur ce même sous-ensemble.
  Bootstrap apparié par pose, graine 0 :
  - IC à 90 % de C − C_ogre dans [−0,05 ; +0,05] → moteur écarté, C − A se
    lit comme la scène (« non démontré » si l'écart-type par pose dépasse
    0,26) ;
  - borne basse de l'IC à 95 % de C − C_ogre > 0,10 → le moteur est un facteur
    majeur ; la scène ne s'écrit que par C_ogre − A (n = 34, avec son IC) ;
  - sinon : « moteur non borné », et on n'écrit pas « la scène ».

  C, A et B rendent un **sous-ensemble** du domaine d'entraînement (aucune
  randomisation d'apparence), pas « le rendu d'entraînement » tout court.
- **Différences connues avec les images réelles**, consignées et non
  corrigées : sur le banc réel, bacs en colonne avec pièces, câbles,
  plante, écran ; la pince porte un cube à ArUco ; l'ouverture de la pince
  n'est pas consignée dans `real_tri_test` (pince ouverte en simulation) ;
  éclairage de labo contre lumières de la réplique ; l'affaissement du bras
  (13-15 mm) n'est pas simulé, l'étiquette réelle est la FK des codeurs.
- **Actions** : `scripts/rejouer_poses_banc.py` (docstring) : `monde` place
  les deux caméras aux extrinsèques d'étiquetage dans le monde choisi ;
  `rejouer` passe par le bras droit entre deux poses (la physique ne déplace
  pas la scène différemment d'une passe à l'autre), attend le repos, prend
  une image par caméra, étiquette par la FK des angles lus. Gazebo sur
  `taskset -c 16-19`, **jamais pendant un entraînement**. Puis
  `eval_papier.py` avec `vgg_e3_9_synth_seul` sur C, A, B et le réel.
- **Contrôles avant d'évaluer** : écart entre la pose Gazebo des liens
  (`gz model -m mycobot_320 -l <lien>`) et la FK de `mycobot_fk.py` ≤ 1 mm
  sur 10 poses par monde ; planche de 24 poses, étiquettes superposées ;
  planche C / A / B / réel côte à côte, et luminance médiane du robot
  masqué dans chacune (un chiffre, pas seulement un coup d'œil).
- **Invalidation** : une pose dont un angle lu s'écarte de plus de 0,5° de la
  consigne est écartée **des trois conditions** (le seuil protège
  l'appariement : l'étiquette, elle, suit l'angle lu). Les poses écartées
  sont listées ; au-delà de 5, l'analyse est refaite aussi sur les 100.
  Contrôle FK > 1 mm → corriger et rejouer la condition en entier.
- **Consigner** : `run.yaml` (commit, modèle, extrinsèques et md5,
  intrinsèques, monde, apparence) ; `rejeu.csv` (angles consigne et lus) ;
  NDDS à la numérotation du jeu réel ; dans `~/dream_article/E3/E3_9b/`.
- **Mesure** : PCK@2,5/5/10 (non détecté = faux), détection, par caméra et
  ensemble. Bootstrap par pose (100 blocs, les deux vues d'une pose dans le
  même bloc), **2 000 rééchantillonnages, graine 0** ; différences appariées
  C − A, A − B, B − réel.
- **Critère de sortie** (PCK@5, par caméra puis ensemble ; une seule graine
  d'entraînement : tout écart est conditionnel à ce modèle, et 0,83 n'a pas
  d'intervalle de graine) :
  1. **Cadrage et poses** : si PCK@5(C) < 0,83 (seuil de E3), le point de vue
     ou la répartition des poses suffit à faire échouer E3.9 sans la scène ni
     l'apparence (attendu pour l'arducam, au vu du cadrage).
  2. **Scène (et moteur de rendu)** : C − A ≥ 0,20 avec une borne basse
     d'IC à 95 % > 0,10 → la scène du banc **ou** le passage d'ogre2 à ogre
     est un facteur majeur ; on n'écrit « la scène » qu'après C_ogre.
     « Écarté » (règle 4) exige en plus |C − C_ogre| dans ±0,05, pour
     exclure que la scène et le moteur se compensent.
  3. **Apparence du robot** : A − B, même règle. Ce seuil commande une
     décision : un facteur majeur fait passer la randomisation d'apparence
     du robot (textures, coques blanches) avant la session au banc.
  4. **« Écarté »** seulement par équivalence : IC à **90 %** (TOST) de la
     différence dans [−0,05 ; +0,05]. Si l'écart-type par pose de la
     différence dépasse 0,26, la règle est hors de portée à n = 100 : on écrit
     « non démontré », jamais « écarté ».
  5. **Reste** : si PCK@5(B) ≥ 0,83, l'écart B − réel revient à ce que la
     simulation ne modélise pas (distorsion, bruit, flou, exposition,
     éclairage, scène réelle encombrée, cube sur la pince), à l'écart de
     moteur ogre/ogre2 avec le rendu d'entraînement **et** aux étiquettes
     réelles, sans pouvoir les séparer. A − B, mesuré en ogre, n'est pas
     établi pour ogre2.
- **Ce que E3.9b ne peut pas voir** : une erreur d'extrinsèque ou
  d'intrinsèque (la simulation place la caméra à l'extrinsèque d'étiquetage
  et rend avec le même K : une erreur commune s'y annule). Le défaut
  possible des intrinsèques SVPRO, ou l'erreur d'étiquette en hauteur,
  tombent dans le « reste » : la session au banc (marqueur en hauteur, mire)
  les tranche.
- **Ne pas faire** : retoucher le rendu `realistic` après avoir vu les
  chiffres et le réévaluer sous le même nom (ce serait E3.9c, avec son propre
  protocole) ; entraîner sur ces images ; comparer C, A ou B au secteur
  synthétique tenu comme à une même distribution ; tirer de E3.9b une
  conclusion sur la justesse des étiquettes réelles.

### Protocole de E3.12 — campagne sim-to-réel ciblée (rédigé le 19/11, amendé le même jour après relecture du metrologue, avant mesure)

Décision de l'utilisateur du 19/11 : viser le meilleur résultat synthétique seul
possible, avec un plan et un protocole, plutôt que des corrections au coup par
coup ; s'il faut du temps (collecte de 100k), on le prend. Démarre quand
l'entraînement RoboPEPP libère la carte graphique (règle : jamais deux tâches
GPU à la fois, collecte comprise).

**Pourquoi pas « 100k de plus de la même chose ».** E3.9 a appris son domaine :
0,968 de PCK@5 sur le secteur d'azimut tenu à l'écart, avec 49 595 images,
fonds COCO (~53 %), distracteurs, lumières, teintes (~32 %) et l'augmentation
v5 (luminosité, teinte, gamma, flou, bruit, JPEG, masquage ; géométrie DREAM :
rotation 15°, translation 6,25 %, échelle ± 10 %). Il échoue sur tous les jeux
réels : PCK@5 0,068 sur `real_tri_test`, 0,16-0,19 sur `real_montage_0901`,
≈ 0 sur les jeux de juin-juillet (étiquettes suspectes). Doubler la même
distribution ajoute ce qu'il sait déjà. Les écarts identifiés :

| Écart | Synthétique (E3.7/E3.7b) | Réel | Source |
|---|---|---|---|
| Focale | fixe, hfov 1,15 rad → fx ≈ 491 px | arducam 496-518, SVPRO 394-448 | SDF ; mires du 17/06 et du 19/11 |
| Distorsion | aucune | arducam forte, SVPRO modérée | mires |
| Cadrage hors axe | ≤ 9° (sphère de 0,34 m entière dans le champ) | arducam 15,8° | 18-19/11, `_robot_dans_le_champ` |
| Apparence du robot | rendu d'entraînement (teintes randomisées) | coques blanches, base grise | E3.9b le mesure |
| Scène | fonds COCO, distracteurs | planche en bois, marqueurs, cube à ArUco sur la pince | photos du banc |

**Ce que la campagne peut et ne peut pas établir** (metrologue, 19/11). À
n = 100 poses et peu de graines, elle peut distinguer un levier **fort**
(ΔPCK@10 ≥ 0,15-0,20). Elle ne peut pas établir un « sans effet » ni un « 100k
non limitant » : les verdicts probables seront « à trancher », et les règles
ci-dessous le prévoient au lieu de laisser une chaîne choisir.

#### Jeux, étiquettes, mesure — communs à toutes les phases

- **Jeux de développement** : ils servent aux **décisions**, jamais au chiffre
  publié.
  - **Décisif** : `real_tri_test`, **par caméra** (SVPRO et arducam
    séparées, 100 poses chacune), plus l'ensemble.
  - **Indicatif** : `real_montage_0901`, **toutes ses images** (1103 par
    caméra), bootstrap **par blocs de 20 images consécutives** (une séquence
    filmée est corrélée), n effectif annoncé.
- **Étiquettes figées** : la version qui a servi à E3.9 (`real_tri_test_ndds`
  du 13/10, les `_ndds` de `montage_0901` tels quels), md5 dans `run.yaml`.
  Elles ne sont **pas** neutres pour la comparaison : déplacer les étiquettes
  SVPRO de 4-6 px (extrinsèque du 15/10) fait passer le PCK@10 SVPRO de E3.9 de
  0,24 à 0,45-0,59 (diagnostic du 18/11, `~/dream_article/E3/E3_9/diag_base/`).
  D'où : pour A5 et C, le Δ est **aussi** calculé sous la seconde version
  SVPRO, et on ne décide que si le sens et le seuil tiennent sous les deux.
- **Mesure** : `training/dream/eval_papier.py`, PCK au sens du papier (non
  détecté = faux). Bootstrap apparié, 2000 tirages, graine 0, par pose
  (`real_tri_test`) ou par bloc (`montage_0901`).
- **On décide sur PCK@10, la cible de l'article reste PCK@5** : un levier à
  PCK@10 ne garantit pas un gain à PCK@5. Chaque verdict donne aussi l'erreur
  médiane et le PCK@5, à titre indicatif.
- **Pour chaque verdict, annoncer** : n, Δ, IC95, IC90, écart-type par pose du
  Δ, nombre de poses exclues.
- **Verdicts** :
  - **levier** : Δ ≥ seuil de la phase, borne basse de l'IC95 > 0,05 ;
  - **sans effet** : IC90 dans ± 0,05 — rare à ce n ;
  - **non démontré** : écart-type par pose du Δ > 0,30 ;
  - sinon **à trancher**.
- **Plafond de référence** : `vgg_tri_reel_ft_e10` évalué sur les mêmes jeux,
  transformés ou non, après contrôle md5 qu'aucune image de `real_tri_test` n'est
  dans son entraînement. Ce n'est pas un concurrent, c'est un repère.
- **Ce que E2 mesurera ensuite** : la généralisation à de nouvelles poses et
  positions **avec ces mêmes caméras**, dont A5 copie les paramètres. Ce n'est
  pas une généralisation à de nouvelles caméras. Et E2 partage la méthode
  d'étiquetage ArUco, valable à Z = 0.
- **Traçabilité**, `~/dream_article/E3/E3_12/<phase>/run.yaml` :
  - commit ;
  - md5 des poids, des jeux et des étiquettes, avec la version d'extrinsèque ;
  - intrinsèques exactes (`cam_3`, `cam_2`, `_camera_settings.json` de
    `montage_0901`) ;
  - graines ;
  - poses exclues ;
  - taux de refus de A5.

#### Phase A — sensibilité du modèle à la géométrie, sans collecte (1 à 2 jours)

E3.9 tel quel, évalué sur des images réelles transformées, avec leurs étiquettes
transformées **de la même façon**. Ces essais mesurent la **sensibilité** du
modèle à une géométrie corrigée, **pas la cause** de l'échec. Ils sont
aveugles à une erreur des intrinsèques : les mêmes servent à transformer et à
étiqueter.

- **A1 — E3.9b** : son protocole, ci-dessus.
- **Contrôles préalables, bloquants** :
  - **identité** : la chaîne de transformation avec K inchangé, R = I et
    distorsion nulle redonne **exactement** les chiffres de E3.9. Sinon le
    ré-échantillonnage est un facteur, et on s'arrête ;
  - **aller-retour synthétique** : une image Gazebo distordue avec une
    distorsion connue puis redressée ramène ses étiquettes à ≤ 0,5 px ;
  - **test de décalage** (`scripts/dream_test_decalage.py`, ± 10/20/40 px) sur
    les sorties de A3 : la détection doit suivre l'image.
- **Sous-ensemble fixe** : seules les poses dont les 7 keypoints restent à plus
  de 4 px du bord **sous toutes les transformations** sont comptées. La liste est
  fixée avant la première évaluation, et les exclusions sont données par caméra.
- **A2 — distorsion** : redressement avec les intrinsèques qui ont produit les
  étiquettes, sans changer K (alpha = 0), zones vides laissées noires et
  hors du sous-ensemble. `cam_3` est en modèle rationnel extrême (k1 5,4,
  k3 −48) : redressement numériquement fragile près des bords, d'où
  l'aller-retour ci-dessus.
- **A3 — cadrage** : rotation virtuelle `K R K⁻¹` (exacte pour une rotation pure,
  si l'extrinsèque est juste) qui amène l'axe optique sur la projection de VISEE
  (0,10 ; 0 ; 0,20), hors axe ramené sous 9°.
- **A4 — échelle** : zoom autour du point principal à fx = 491 px. Lisible pour la
  **SVPRO seulement** (facteur 1,10-1,25) ; pour l'arducam (0,95-1,0) c'est
  quasi l'identité, donc « sans effet » garanti et sans valeur.
- **Lecture** : A2, A3 et A4 chacun **séparément contre l'image brute**, puis
  leur cumul. Seuil de la phase : ΔPCK@10 ≥ 0,15.

#### Phase A5 — randomiser le modèle de caméra à l'entraînement (≈ 3 jours)

Même recette que E3.9 (VGG ImageNet, v5_geo, encodage retenu en E3.8, mêmes
49 595 images, même secteur tenu à l'écart), **plus** une augmentation
géométrique appliquée à l'image **et** aux keypoints.

**Ce qui est nouveau par rapport à v5_geo**, qui couvre déjà la translation
(6,25 %, soit ≈ 40 px, ce qui couvre le point principal) et l'échelle 0,9-1,1 :

| Paramètre | Tirage | Raison |
|---|---|---|
| Focale courte | facteur 0,78-0,90 sur fx = 491 (≈ 383-442 px) | SVPRO 394-448, hors de l'échelle ± 10 % |
| Hors axe | rotation virtuelle `K R K⁻¹`, angle 0-25°, direction uniforme | arducam à 15,8° |
| Distorsion radiale | k1 tiré pour un déplacement au coin de l'image de 0 à ± 25 px, k2 = 0 ; carte refusée si non monotone | le plus fort effet mesuré sur les étiquettes `cam_3` vaut 15,8 px (4,4 px médian) |
| Probabilité | 0,5 par paramètre, indépendants | limiter les cumuls de déformations fortes |

- **Bords** : un zoom arrière et une rotation créent des zones sans image. On les
  remplit avec un **fond COCO** tiré au hasard, comme pour les fonds du jeu.
  Jamais en noir : le réseau apprendrait le cadre noir, que le réel n'a pas.
- **Modèle de distorsion** : la distorsion est appliquée à l'image par carte
  inverse (`cv2.remap`). Les keypoints sont transformés par le modèle direct.
  Aller-retour contrôlé ≤ 0,5 px sur 100 images avant l'entraînement.
- **Refus** : un tirage qui fait sortir plus d'un keypoint à moins de 4 px du
  bord est retiré. Si plus de 40 % des tirages sont refusés, c'est une
  **alarme** et l'entraînement s'arrête.
- **Graines** : **deux pour A5 et deux pour E3.9** (E3.9 réentraîné une fois avec
  une seconde graine). L'écart d'une graine à l'autre sur un jeu réel hors
  domaine est probablement de 0,03 à 0,08, soit la taille des effets testés.
- **Critère « retenue »**, sur `real_tri_test` :
  - moyenne des deux graines A5 moins moyenne des deux graines E3.9 :
    ΔPCK@10 ≥ 0,10, caméras réunies ;
  - pour **chaque** paire de graines, borne basse de l'IC95 > 0 ;
  - Δ > 0 sur chaque caméra ;
  - le sens et le seuil tiennent sous les deux versions d'étiquettes SVPRO ;
  - secteur tenu à l'écart non inférieur : borne basse de l'IC95 du ΔPCK@5
    ≥ −0,04.
- **A5 est un paquet** : si elle est retenue, B reprend le paquet entier.
  L'ablation de chacun de ses paramètres va dans E4.
- **Ne pas faire** : régler les amplitudes sur un résultat réel. Elles sont
  fixées ici, une fois.

#### Décision avant la collecte (fixée avant mesure, prise par l'utilisateur)

| Résultat | Suite |
|---|---|
| A5 retenue | B avec le paquet A5 |
| A1 désigne l'apparence ou la scène (« facteur majeur » au sens de E3.9b) | B avec ce levier, que l'augmentation ne couvre pas |
| Un levier A2-A4 sur `real_tri_test`, mais A5 non retenue | l'augmentation ne suffit pas : B avec le **rendu** des caméras (plusieurs hfov, hors axe au collecteur) |
| Tout « à trancher » ou « non démontré » | **pas de B automatique** : chiffres présentés à l'utilisateur, qui décide ; E2 reste le test final, jamais un jeu de décision |

La chaîne sans surveillance s'arrête à cette décision. Elle ne lance jamais B
d'elle-même.

#### Phase B — collecte de 100k (≈ 5 jours, carte graphique réservée)

`synthetic_data_collector_v3_camera_libre.py`, mêmes règles que E3.7/E3.7b :
- tranches de 1 000 relancées ;
- seuls les angles interpolés au temps de l'image sont gardés ;
- masques à jour, md5, `run.yaml` ;
- **même secteur d'azimut 60-100° tenu à l'écart**.

À 4,1 s par image, 100 000 images ≈ 114 h. Modifications, chacune selon la
décision ci-dessus :

- **Hors axe** : `_robot_dans_le_champ` accepte un axe optique jusqu'à 25° de
  la visée, la sphère de travail pouvant sortir en partie ; `VISEE_JEU` passe
  de 0,10 à 0,20 m.
- **Focale** : deux à trois caméras SDF de hfov différents (≈ 1,0 ; 1,15 ;
  1,35 rad), une par tranche.
- **Apparence** : `robot_appearance` tiré par tranche (`original`,
  `realistic`, teintes randomisées).
- **Scène du banc** : ~20 % des tranches dans `real_table.sdf` (planche,
  marqueurs 19/23/25/26) avec le cube à ArUco sur la pince.
- **Critère de sortie** :
  - ≥ 95 000 images valides ;
  - secteur tenu à l'écart vérifié par `geometrie_camera.py` ;
  - md5 sans recouvrement avec E3.7/E3.7b.

#### Phase C — entraînements (≈ 2 jours)

Recette retenue (A5 ou E3.9) : **C50** sur 50 000 images tirées au hasard dans B,
**C100** sur les 100 000, même graine, même secteur tenu à l'écart.

- **« 100k utile »** : ΔPCK@10 (C100 − C50) ≥ 0,10 sur `real_tri_test`, borne
  basse de l'IC95 > 0, sous les deux versions d'étiquettes SVPRO. Sinon
  **non démontré**.
  - « Non limitant » n'est pas atteignable : il faudrait ≈ 450 poses
    indépendantes.
  - Le verdict porte la mention « conditionnel à une graine ».
- E4 reprendra le volume sur trois graines.

#### Phase D — évaluation

Le meilleur modèle passe, dans l'ordre :

1. le secteur tenu à l'écart (synthétique) ;
2. les jeux de développement, par caméra ;
3. le test de décalage ;
4. **E2** (E3.10), une seule fois : c'est le chiffre de l'article ;
5. le tableau au format du tableau I de Lee et al. 2020 (PCK@2,5/5/10, AUC 0-20 px,
   ADD et son AUC 0-100 mm), avec l'outil de E1.

La comparaison à DREAM porte sur le **format** et la méthode, pas sur des
scores : le robot, les caméras et les jeux diffèrent. Des chiffres publiés de
DREAM, RoboPose et SGTAPose, on ne prend que ceux vérifiés dans
`training/dream/article/biblio.md` (agent `veilleur-biblio`). Chaque chiffre passe
au metrologue, puis au registre.

#### Ce qu'il faut écrire avant de lancer

| Élément | Où | Phase |
|---|---|---|
| Transformations A2-A4 (images + étiquettes, NDDS en sortie), sous-ensemble fixe, contrôle d'identité, aller-retour | `training/dream/transformer_reel.py` | A |
| Analyse appariée : bootstrap par pose ou par bloc, par caméra, deux versions d'étiquettes, deux graines | reprendre `~/dream_article/E3/E3_9b/analyse.py` | A, A5, C |
| Augmentation « modèle de caméra » (focale courte, rotation virtuelle, distorsion bornée, fond COCO aux bords, refus et alarme) | option de `train_dream_ultimate_v5_geo.py` | A5 |
| Collecteur : hors axe 25°, `VISEE_JEU`, caméra, apparence et monde par tranche | `synthetic_data_collector_v3_camera_libre.py`, `scripts/collecte_tranches.sh` | B |
| Chaîne sans surveillance A → A5, **arrêt** sur : échec d'un contrôle préalable, alarme de refus, évaluation manquante ; arrêt final à la décision | `~/dream_article/E3/E3_12/` | A, A5 |

#### Durées et ordre

| Étape | Durée | Carte graphique |
|---|---|---|
| A1 (E3.9b) | ~1 h 30 | exclusive (Gazebo) |
| Contrôles + A2-A4 | < 2 h | évaluations |
| E3.9, 2e graine | ~1 jour | exclusive |
| A5, 2 graines | ~2 jours | exclusive |
| Décision | utilisateur | — |
| B | ~5 jours | exclusive |
| C50 + C100 | ~2 jours | exclusive |
| D hors E2 | < 1 h | évaluations |

Indépendant de la carte : la session E2 au banc (mardi 24/11), qui fournit le
jeu de test final.

### Chaîne du 12/11 (absence de l'utilisateur ≥ 4 jours) — règles écrites avant les résultats

Décision utilisateur du 12/11 : **écarter de l'entraînement les images dont les
angles ne sont pas interpolés au temps de l'image**, compléter le jeu si
nécessaire, et juger soi-même d'une nouvelle collecte. Jugement porté le 12/11,
avant tout résultat de E3.8 :

- **Volume** : filtré, E3.7 ne garde que ~23 000 images d'entraînement sur
  44 262 (52 % interpolées), contre les 50 000 décidés le 04/11. **E3.7b passe
  de 10 à 30 tranches de 1 000** (~33 h) : ~26 000 images propres de plus, soit
  ~49 000 au total, dont ~40 % de poses du pick sur la partie E3.7b.
- **Collecteur** : historique des angles porté de 3 à 10 s
  (`HISTORIQUE_JOINTS_S`), pour que le retard de l'image ne le dépasse plus.
- **Choix de l'encodage (E3.8)** : la règle de E3.8 s'applique aux **484 images
  de test à angles interpolés** (`tenu_interpolees.npy`), le test complet étant
  donné à titre de contrôle. Si le choix échoue (évaluation manquante),
  encodage entier de DREAM.
- **E3.9** : conversion de E3.7 + E3.7b avec `--angles-interpoles`, secteur
  60-100° à part, puis entraînement synthétique seul (VGG-19 ImageNet, 50
  époques, patience 8) avec l'encodage retenu ; évaluation sur le secteur tenu
  et sur `real_tri_test`, références (Osama `vgg_tri_reel_ft_e10`, E3.0) sur
  le même secteur tenu.
- **Arrêts** : moins de 24 tranches E3.7b finies, conversion ou entraînement
  en échec → la chaîne s'arrête et l'écrit dans son journal. Aucun chiffre
  n'entre au registre sans la revue du metrologue.
- Script : `~/dream_article/E3/E3_9/chaine.sh` (reprend où il s'est arrêté),
  journal : `~/dream_article/E3/E3_9/journal.txt`.

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

- **23/11 — E3.9b arrêté à l'essai : le monde d'entraînement n'avait pas de
  caméras actives** : la chaîne est partie le 22/11 à 15:58 et s'est arrêtée
  sur « ni /joint_states ni les deux caméras ». `/joint_states` était bien
  publié (~50 Hz). Les caméras ne rendaient rien, même côté Gazebo : la
  configuration par défaut de Gazebo (`server.config`) ne charge **pas** le
  système `Sensors`, et `banc_realiste.launch.py` lance l'URDF avec
  `enable_cameras:=false`. `randomized.sdf` n'a aucun `<plugin>` : il
  comptait sur celui de l'URDF. `rejouer_poses_banc.py monde` ajoute
  désormais `Sensors`, avec les trois systèmes par défaut si le monde n'en
  déclare aucun. Le moteur se choisit par `--moteur-rendu`. Le monde C a été
  régénéré en **ogre2**, comme l'URDF qui a rendu E3.7. Ses caméras sont
  identiques à celles d'avant. Le monde du banc (A, B) avait déjà ses
  capteurs, en **ogre** (ogre2 y plante sur les matériaux des ArUco). ⚠ C
  contre A mêle donc la scène et le moteur de rendu : à signaler au
  metrologue avant de lire l'écart. L'échec du spawner
  `joint_state_broadcaster` au démarrage est inoffensif (il réessaie un
  contrôleur déjà actif). *Décidé le 23/11 : C reste en ogre2 ; le
  protocole est amendé en conséquence (C − A = scène et moteur réunis).*
- **18/11 — E3.9 : très bon en simulation, échec sur le réel** : le modèle
  synthétique seul (49 595 images, sous-pixel σ 1) atteint PCK@5 0,968 sur
  les 6 125 images du secteur 60-100° tenu à l'écart, mais 0,068 sur les 200
  images réelles de `real_tri_test` (détection 0,70 ; E3.0 : 0,014). La
  randomisation d'apparence aide, sans suffire. Piste à fermer d'abord : la
  base, point fixe, est prédite **4 à 6 px trop haut sur les deux caméras,
  avec une dispersion de 0,3 px** — un biais constant, pas du bruit. Sim et
  réel prennent leurs keypoints dans le même `mycobot_fk.py` ; le metrologue
  note que le rapport arducam/SVPRO ne colle pas à un décalage en Z et
  suggère une convention d'image (rééchelonnage 800×600 → 640×480 de la
  SVPRO, origine du pixel) ou du point « base ». Hypothèse, pas une cause
  (registre E3.9, `a_preciser`).
  **Diagnostic du 18/11** (`~/dream_article/E3/E3_9/diag_base/` et
  `diag_arducam/`), corrigé le même jour après relecture du metrologue :
  - **écarté** : le décodage du réseau (la base n'a aucun biais en
    simulation : 0,26 ; 0,01 px) et un décalage 3D commun de la base (aucun
    n'explique les deux caméras, résidu jusqu'à 2,9 px) ;
  - **étiquettes SVPRO** : celles de `real_tri` (train et test, 13/10)
    sortent de `svpro_extrinsic_labels.avant_1410.yaml`. Reprojetées avec
    l'extrinsèque du 15/10, elles bougent de 4-6 px et le PCK@10 SVPRO
    d'E3.9 passe de 0,24 à 0,45-0,59 ; **mais les marqueurs visibles dans
    ces images ne discriminent pas ces versions (1,8-3,6 px pour celle du
    13/10, 5,1-8,0 px pour `labels.yaml`) : la plage mesure la sensibilité
    aux étiquettes, pas une étiquette corrigée**. Juger une extrinsèque par
    l'accord avec le modèle évalué serait circulaire ;
  - **arducam, `base` et `link1`** : erreurs de 20 et 40 px même après
    ajustement de pose. La base est prédite avec 0,3 px de dispersion, une
    sortie quasi constante ; l'occultation sous l'épaule est plausible mais
    non mesurée (pas de visibilité par keypoint) ;
  - **arducam, `link3`-`link6`** : cohérents avec la géométrie du robot
    (une pose fixe : résidu 1,4-5,7 px). Une pose à 65 mm de l'ArUco les
    explique à 3,7 px et les marqueurs à 0,7-3,1 px, contre 0,3-0,7 px pour
    la pose ArUco ; avec 4 centres coplanaires (2 degrés de liberté), ce
    n'est pas une preuve. Cet ajustement absorbe aussi tout biais lisse du
    réseau (erreur radiale −11 à −16 px, sans lien avec la hauteur, r = −0,18) ;
  - **SVPRO** (tenu) : une pose fixe explique 92 % des prédictions à 2,2 px
    mais place les marqueurs à 7-11 px ; une similitude (×1,053, décalage
    −7,5 ; −11,6 px) ramène l'erreur de 13,7 à 4,5 px. Plutôt la géométrie
    d'image (intrinsèques rééchelonnées 800×600 → 640×480) qu'un biais
    d'échelle du réseau, à vérifier sur mire ;
  - **distorsion arducam** (`cam_3`) : coefficients extrêmes (k3 = −48),
    effet 4,4 px médian, 15,8 px max ; sans elle le PCK@10 passe de 0,10 à
    0,20, mais sans réajuster l'extrinsèque : indicatif seulement.
  Conséquences : l'échec réel d'E3.9 **demeure** ; l'hypothèse d'une erreur
  d'étiquette pour les points en hauteur est compatible, pas démontrée.
  `real_tri_test` ne départage pas les deux pour ces points, à un niveau
  pouvant atteindre ~10 px ; un modèle entraîné sur ces étiquettes
  (`vgg_tri_reel_ft_e10`) en apprend le biais, son PCK@5 0,94 mesure
  l'accord avec elles. Contrôles au banc pour trancher : (1) un marqueur de
  50 mm posé à 250-300 mm de haut, vu des deux caméras (erreur d'étiquette
  en hauteur lue sans modèle) ; (2) marqueurs revus le jour même, extrinsèque
  réajustée, mire pour les intrinsèques SVPRO et la distorsion arducam.
- **13/11 — E3.8 : sous-pixel σ 1 retenu** par la règle écrite d'avance
  (Δ PCK@2,5 +0,149 [+0,132 ; +0,167] sur 484 images). La perte de détection
  n'est pas exclue par l'IC ([−0,017 ; +0,000], tolérance 0,01) : le critère
  passe sur l'estimation ponctuelle. Une graine par bras (registre E3.8).

- **12/11 — E3.7 : la moitié des angles ne sont pas interpolés** : sur les
  tranches 01-09 (5 000 images), ~53 % des images ont les angles de la
  dernière mesure reçue et non ceux de l'instant de l'image (écart médian
  0,12 s, max 1,4 s) ; tranche 10 (5 × 1 000) : 99 % interpolés. Même cause
  que la dérive des masques : le retard de l'image dépasse l'historique de 3 s.
  Erreur estimée sur ces images (vitesse × écart, reprojetée, pire keypoint) :
  médiane 0,05 px, **7-8 % au-delà de 2,5 px**, 1 % au-delà de ~17 px, soit
  ~4 % du jeu. Le drapeau `source.joints_interpoles` permet de filtrer.
  Conséquence pour E3.8 : 484 des 1 063 images de test sont interpolées ;
  comparer aussi sur ce sous-ensemble (`comparer_pck.py --garder
  ~/dream_article/E3/E3_8/tenu_interpolees.npy`). Tranché le 12/11 (utilisateur) : on écarte, voir « Chaîne du 12/11 ». E3.7b (tranches de 1 000) n'en souffre pas.
- **12/11 — Le meilleur modèle d'Osama sur le secteur synthétique tenu** :
  `vgg_tri_reel_ft_e10` (affiné sur le réel, 19/10) évalué sur le test de E3.8
  (1 063 images, azimut 60-100°, randomisation E3.7) : détection 0,457,
  PCK@2,5/5/10 0,044 / 0,111 / 0,192. E3.0 (synthétique seul, sans
  randomisation d'apparence) : 0,773 et 0,392 / 0,579 / 0,666. Hors du
  domaine du premier (caméras fixes, apparence du banc) : ce n'est pas une
  comparaison sur sa tâche. La comparaison qui compte pour lui est le réel
  (`real_tri_test`, E1 : PCK@5 0,941), en E3.10. `a_preciser`.
- **12/11 — Secteur 60-100° tenu à l'écart : pourquoi, et pour quel modèle** :
  un secteur d'azimut entier est retiré de l'entraînement parce qu'une
  validation tirée au hasard partage les points de vue de l'entraînement et ne
  voit pas un réseau qui récite le cadrage (leçon de septembre ;
  `convert_to_ndds_camera_libre.py`, 19/10). Les bornes 60-100° ne sont
  justifiées nulle part ; elles se trouvent contenir la SVPRO fixe (70°).
  **Décision utilisateur du 12/11** : E3.9 (modèle de l'article) garde le
  secteur à l'écart ; si E3.10 est bon, un modèle de déploiement est entraîné
  en plus sur tous les azimuts, hors article.
- **12/11 — Jeu E3.7 figé** : 44 262 images d'entraînement, 5 473 dans le
  secteur tenu, 5 écartées hors champ ; md5 des labels 2456b9cf… et fe3e84c0…
  (`~/dream_article/E3/E3_7/run.yaml`), copie NDDS dans
  `~/Desktop/E3_7_synthetique_50k/` (15 Go).
- **12/11 — E3.7 tiré sans les poses du pick** : le collecteur prévoit 40 %
  de poses tirées autour des configurations réelles du pick
  (`training/dream/captures/real_montage_0901/labels.csv`), mais ce fichier
  n'existe que dans le clone `~/Osama_ws` ; les 14 `launch.log` de E3.7 disent
  « aucune pose du pick, tirage uniforme seul ». E3.7 est donc en tirage
  uniforme dans les butées, garde au sol 50 mm. Le manuscrit le décrit tel
  quel. Tranché le 12/11 : complément E3.7b avant E3.9. Est-ce gênant pour le tri réel (pointe à
  72-114 mm, la zone dont l'absence avait causé le biais de 52 px en
  septembre) ? Mesuré le 12/11 sur 2 054 annotations (1 sur 5) : pointe de
  pince médiane à 385 mm, p5 à 160 mm ; **1,7 % des images seulement ont la
  pointe entre 72 et 115 mm**. La zone du tri est présente mais rare.
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
    ~5 s/image en moyenne. **Les masques dérivent avec** : 95 % de masques à
    jour en début de tranche, 50-55 % au milieu, 22-27 % à la fin (tranches
    01, 03, 08), 59 % sur une tranche entière contre 97,5 % au pilote de
    1 000. Le rendu ralentit, le masque arrive au-delà des 15 relances. Les
    étiquettes, elles, restent justes en fin de tranche (8 images de la fin
    de la tranche 08 contrôlées à l'œil). Conséquence : à `--part 0.5`, ~29 %
    des images d'une tranche reçoivent un fond COCO au lieu de ~50 %.
    Confirmé le 11/11 par la tranche 10, collectée en 5 × 1 000 : **3,85 s/image**
    (3,8 au début, 4,1 à la fin de chaque sous-tranche) et **95-97 % de masques
    à jour**, contre 12 s et 59 % en tranches de 5 000. La dérive tient à la
    durée de vie de Gazebo : collecter par tranches de 1 000. Tranches 01-08
    recomposées à `--part 0.85 --part-teinte 0.5` : 49,3 % de fonds et 29,6 %
    de teintes sur 39 740 images.
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
