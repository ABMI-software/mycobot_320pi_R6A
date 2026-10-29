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
   papier annonce 0,03. Rapporter le seuil utilisé ; tester 0,03 demande une
   copie modifiée de `peaks_from_belief_maps`, la valeur y étant en dur.
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
| E2 | Jeu réel multi-vues : 6 à 10 positions de caméra jamais vues, référence indépendante de DREAM | — | Jeu figé, md5, positions relevées, leave-one-out de l'extrinsèque publié | rouge | à faire |
| E3 | Synthétique seul multi-vues : caméra libre + randomisation du papier + augmentation v5, aucune image réelle | E1 | PCK@5 ≥ 0,83 et PCK@10 ≥ 0,90 sur E2 ; test de décalage ≥ 95 % ; ADD ≥ 0,80 à 20 mm | rouge | à faire |
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
