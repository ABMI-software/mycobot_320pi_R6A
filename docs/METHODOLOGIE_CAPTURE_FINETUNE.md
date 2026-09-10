# Méthodologie — de la caméra au réseau affiné

*Protocole complet, tel qu'exécuté du 31 août au 2 septembre 2026 sur la branche
`feature/pick-and-place-osama`. Résultat : DREAM passe de **53,8 px à 1,81 px**
de médiane sur le montage du pick.*

Ce document est le **mode d'emploi**. Il dit quoi faire, dans quel ordre, avec
quelles commandes et quels chiffres attendre. Le *raisonnement* — pourquoi cette
méthode et pas une autre, quelles hypothèses ont été tuées en route — est dans
[`DREAM_DIAGNOSTIC_BIAIS.md`](DREAM_DIAGNOSTIC_BIAIS.md).

---

## Vue d'ensemble

```
0. environnement          conda deactivate / venv_dream
1. cameras                focus SVPRO verrouille, exposition arducam 75
2. extrinseques           4 marqueurs ArUco -> pose des cameras       (la reference)
3. plan de capture        72 trajectoires, 3 filtres -> 1102 poses    (deterministe)
4. capture                bras arrete a chaque prise, angles MESURES  (~48 min)
5. conversion NDDS        FK + extrinseques + distorsion              (2,7-3,8 px)
6. melange + test         symlinks, bloc contigu de 400 poses         (39 040 trames)
7. fine-tune              depuis les poids existants, 30 epochs       (6 h 43)
8. evaluation             tenu a l ecart + non-regression             (1,81 / 2,32 px)
```

Chaque étape produit un chiffre qui valide la précédente. Si un chiffre sort de
sa fourchette, **on ne passe pas à la suite** — c'est ce qui a permis de trouver
que la SVPRO était à 21,5 px avant qu'elle ne contamine 1102 étiquettes.

---

## 0. L'environnement

Trois interpréteurs coexistent et **ne se mélangent pas**.

```bash
conda deactivate                                  # TOUJOURS en premier
source /opt/ros/jazzy/setup.bash                  # ROS2, colcon, rclpy
source ~/Osama_ws/install/setup.bash

source ~/ros_jazzy/venv_dream/bin/activate        # DREAM : capture, conversion, entrainement
```

Le workspace actif est **`~/Osama_ws`**, pas `~/ros_jazzy` — ce dernier est une
copie figée. Vérifier : `which python3`.

⚠ L'OpenCV du `venv_dream` est **headless**. Aucune fenêtre ne s'y ouvre ; pour
afficher, passer par le python système (`/usr/bin/python3`), dont l'API aruco est
l'ancienne.

---

## 1. Préparer les caméras

**La SVPRO perd sa mise au point à chaque ouverture de flux.** Rien dans le dépôt
ne la fixe, et le défaut du capteur est l'autofocus continu.

```bash
bash scripts/svpro_verrou_focus.sh          # focus 40 par defaut
```

Trois règles, toutes mesurées :

- **Après** le lancement de ce qui ouvre le flux. Un contrôle posé avant le
  démarrage est réinitialisé par le pilote.
- Après **chaque** rebranchement ou redémarrage.
- La caméra se trouve par son **nom V4L2**, jamais par son index — celui-ci
  change avec le port USB (`-4 → video0`, `-5 → video3`).

Balayage de référence sur le montage actuel :

| focus | netteté (Laplacien) | marqueurs | détections |
|---|---|---|---|
| **40** | **418** | **4/4** | **48/48** |
| 90 | 211 | 2/4 | 14 |
| 140 | 51 | 0/4 | — |

**L'arducam n'a rien à régler** : son autofocus est coupé d'usine
(`focus_automatic_continuous` défaut 0) et elle revient toujours au même état.
Seule son **exposition** dérive, et le dashboard la repose toutes les 4 s.

---

## 2. Les extrinsèques marqueurs — la référence

C'est ce qui fabrique la vérité terrain. Tout le reste en dépend.

```bash
python3 scripts/svpro_extrinsic_4_marqueurs.py            # mesure seule
python3 scripts/svpro_extrinsic_4_marqueurs.py --ecrire   # si les chiffres tiennent
```

**Les 4 ArUco de la planche ne bougent JAMAIS.** Quand un marqueur manque, le
levier est la **caméra** — orientation, cadrage, mise au point. Jamais la
planche : c'est la seule référence stable du banc, mesurée à ±5 mm et recoupée
par deux jeux de mesures indépendants.

**Les coins 3D sont rétroprojetés depuis l'arducam** sur le plan Z=0, pas
mesurés au ruban : 16 correspondances au lieu de 4, et une précision meilleure
que les ±5 mm du ruban.

**Le juge n'est pas le RMS d'ajustement** — il est mesuré sur les points qui ont
servi à ajuster. C'est la validation en **laissant un marqueur dehors** :

| marqueur écarté | 3 marqueurs | 4 marqueurs |
|---|---|---|
| 19 | 7,3 px | **2,45 px** |
| 23 | 3,3 px | **3,32 px** |
| 25 | *non vu* | **2,43 px** |
| 26 | 11,4 px | **2,09 px** |

Les 11,4 px sur le 26 disaient que l'ajustement à 3 marqueurs extrapolait mal
vers le fond de la planche, faute d'y avoir un point.

**Ne jamais écraser une extrinsèque déjà utilisée** pour convertir un jeu : ce
jeu doit garder celle avec laquelle il a été fait. Toujours un nom neuf.

---

## 3. Le plan de capture

**Déterministe. Aucun tirage aléatoire, aucune pose inventée.**

```bash
python3 scripts/capture_trajectoires.py --simuler    # compte sans bouger le bras
```

### Les quatre bases

Des poses **réellement jouées** par l'opérateur, reprises telles quelles :

```python
BASES = [
    [30, -118.7, 82.8, -102.6, -17.8, 49.8],
    [30, -128.7, 76.5,  -53.4,  -3.7, 21.8],
    [30, -120.0, 90.0,  -60.5,  10.3,  6.2],
    [30, -110.0, 70.0,  -80.0, -10.0, 30.0],
]
```

### Les six balayages

Une seule articulation varie par trajectoire, sur sa plage de travail :

```python
BALAYAGES = [
    (0,    8,  58),   # J1 : azimut
    (1, -134, -108),  # J2 : hauteur
    (2,   68,  96),   # J3 : allonge
    (3, -104, -50),   # J4 : poignet
    (4,  -42,  22),   # J5 : inclinaison outil
    (5,  -10,  92),   # J6 : rotation outil
]
```

**4 bases × 3 azimuts × 6 balayages = 72 trajectoires**, échantillonnées à 2,5°.

J6 est balayé bien qu'aucun keypoint ne dépende de sa rotation : il ne déplace
rien dans la FK mais il **change l'image**, et le réseau doit apprendre que
cette variation n'est pas un signal.

### Les trois filtres

| filtre | rôle | écartées |
|---|---|---|
| `pose_sure` | pointe ≥ 40 mm, liens mobiles ≥ 30 mm, hors volume de base | **525** |
| `visible` | les 7 keypoints dans la fenêtre réseau 400×400, marge 30 px | **29** |
| — | **retenues** | **1102** |

La sécurité taille dans **la moitié** du plan brut. Ce n'est pas un garde-fou de
principe.

### Pourquoi 2,5°

| | NVlabs Panda | `real_3cam` | **ici** |
|---|---|---|---|
| écart image à image | 0,23° | 94,3° | **2,46°** |
| robot pendant la prise | en mouvement | à l'arrêt | à l'arrêt |

NVlabs **filme** des trajectoires lentes et enregistre l'état articulaire
synchronisé à la trame. Ici les angles arrivent par requête-réponse TCP : à
10 °/s, une latence de 100 ms fait **1° d'erreur d'étiquette**. On reprend donc
leur **densité**, pas leur mouvement — arrêt à chaque prise.

---

## 4. La capture

```bash
python3 scripts/capture_trajectoires.py --nom real_montage_0901
```

**Trois règles de fabrication**, chacune apprise à ses dépens :

1. **L'étiquette est l'angle MESURÉ, bras arrêté** — jamais la consigne.
   `immobile()` attend deux lectures consécutives à moins de 0,35° l'une de
   l'autre. Une pose que le bras n'atteint pas à 6° près est **ignorée**, pas
   enregistrée avec sa consigne.
2. **Un départ de trajectoire se rejoint par paliers de 30°.** Un ordre unique
   depuis loin fait partir toutes les articulations à fond en même temps.
3. **Écrire dans `training/dream/captures/`, jamais dans `dream_data/`.** Le
   script refuse toute autre racine. `labels.csv` s'ouvre en `append` : une
   capture interrompue reprend où elle s'est arrêtée.

Vider le tampon V4L2 (6 trames) avant chaque prise, sinon on enregistre une
image **antérieure**, prise pendant le déplacement, donc floue — la détection
tombe de 7/7 à 2/7.

**Deux corrections venues du réel, pas du calcul :**

- **La pince a touché la table.** Le filtre plan repris de
  `capture_real_3cam.py` ignore J5, qui incline l'outil. Corrigé sur la FK
  complète, avec l'axe de pince **mesuré** (−X de la bride) et non supposé.
- **Le bras s'affaisse de 7 mm** sous la gravité (J2 tombe ~1,9° sous la
  consigne). Une simulation cinématique ne peut pas le voir. Garde portée de 25
  à 40 mm.

Résultat : `1102 poses × 2 caméras = 2204 images`, ~48 min, zéro incident.

---

## 5. La conversion NDDS

```bash
python3 scripts/convert_capture_ndds.py --capture real_montage_0901 --verifier 20
```

Trois exigences, chacune correspondant à une erreur observée :

- **Les intrinsèques viennent du `_camera_settings.json` écrit PAR la capture**,
  donc de la caméra qui a réellement pris les images. `convert_to_ndds.py` mappe
  `arducam → cam_0` alors que l'arducam du banc est `cam_3` : 6,4 % d'écart de
  focale. C'est ce qui a rendu `arducam_extrinsic_dream_v4.yaml` incomparable à
  tout.
- **Les extrinsèques sont celles des marqueurs, validées**, pas les poses codées
  en dur de `_real_camera_transform`.
- **La projection applique la DISTORSION** (`cv2.projectPoints`). Sans elle les
  `projected_location` ne tombent pas où le keypoint apparaît, et le réseau
  apprend des cartes de croyance décalées.

**Chiffre attendu : 2 à 4 px.** Obtenu : 3,8 px (arducam), 2,7 px (svpro). Si ce
résidu dépasse ~5 px, l'extrinsèque est fausse — s'arrêter là.

---

## 6. Le mélange et le test tenu à l'écart

```bash
python3 scripts/build_mix_ndds.py \
  --reel-neuf  training/dream/captures/real_montage_0901_{arducam,svpro}_ndds \
  --test-poses 400 --oversample 10 \
  --reel-ancien training/dream/dream_data/real_3cam_{arducam,svpro}_ndds \
  --synth training/dream/dream_data/synthetic_50k_ndds --n-synth 20000 \
  --train-out training/dream/dream_data/mix_montage0901_train \
  --test-out  training/dream/dream_data/mix_montage0901_test
```

### La composition, et pourquoi

```
synthetique                20 000   51,2 %    le socle general
montage_0901 (x10)         14 040   36,0 %    la region ou le reseau est aveugle
real_3cam                   5 000   12,8 %    l assurance contre l oubli
                           39 040
```

Les 12,8 % de `real_3cam` ne sont pas décoratifs : **sans eux le réseau optimise
la nouvelle région en abandonnant l'ancienne**, et on déplace le problème au lieu
de le résoudre. C'est ce que vérifie l'évaluation de non-régression, étape 8.

Tout est en **liens symboliques** : 351 Mo réels, pas une image copiée.

### Le découpage du test — trois pièges

**Le tirage aléatoire est exclu d'emblée** : deux images consécutives sont à
2,46° l'une de l'autre, quasi jumelles.

**Le bloc contigu ne suffit pas non plus.** La trajectoire repasse sur ses pas —
121 poses sur 1042 reviennent à moins de 2,5° d'une pose vue plus de 50 trames
plus tôt :

| test | médiane à la plus proche du train | jumelles < 2,5° |
|---|---|---|
| 150 dernières | 14,2° | **30 / 150** |
| 250 dernières | 12,6° | 53 / 250 |
| **400 dernières** | **26,0°** | **1 / 400** |

**Le troisième piège est de fabrication.** Reconstruire à 400 par-dessus un
tirage à 150 laisse la queue du premier : 2 500 liens périmés, dont 129 pointant
sur des trames passées du train au test — une fuite créée par la reconstruction,
invisible dans le compte affiché. D'où `purge()`, qui refuse tout ce qui n'est
pas un lien symbolique.

**Vérification obligatoire avant d'entraîner** : 0 fichier commun train/test,
0 lien cassé, séparation médiane ≥ 20° en espace articulaire.

---

## 7. Le fine-tune

```bash
cd training/dream
python3 train_dream_ultimate_v4.py \
  --data dream_data/mix_montage0901_train \
  --pretrained checkpoints_dream/vgg_ultimate_v4_mix_ft_e30/best_network.pth \
  --output checkpoints_dream/vgg_montage0901_ft_e30 \
  --epochs 30 --batch-size 8 --lr 0.0001 --workers 8
```

**Fine-tune** = repartir des poids existants au lieu de zéro. Le réseau garde ce
qu'il sait et l'ajuste ; 30 passes suffisent là où un entraînement complet en
demande 50 et perdrait l'acquis synthétique.

**Aucun fichier existant n'est modifié.** `train_dream_ultimate_v4.py` accepte
déjà `--data`, `--pretrained` et `--output` : le checkpoint de départ est
seulement *lu*, la sortie va dans un dossier neuf.

```
13,6 min par epoch   ->   6 h 43 pour 30 epochs
arret anticipe : patience de 5 epochs
best_network.pth reecrit a chaque amelioration
```

Suivi sans se noyer dans la barre tqdm : `bash scripts/suivre_finetune.sh`

---

## 8. L'évaluation — deux mesures, pas une

```bash
# 1. le jeu tenu a l ecart : est-ce que le probleme est resolu ?
python3 evaluate_dream.py -w checkpoints_dream/vgg_montage0901_ft_e30/best_network.pth \
                          -d dream_data/mix_montage0901_test --split all -n 800

# 2. l ancien jeu : est-ce qu on a casse autre chose ?
python3 evaluate_dream.py -w checkpoints_dream/vgg_montage0901_ft_e30/best_network.pth \
                          -d dream_data/real_3cam_arducam_ndds --split all -n 400
```

**Toujours mesurer la ligne de base AVANT**, avec l'ancien checkpoint. Sans elle
le chiffre d'après ne veut rien dire.

| `mix_montage0901_test` | avant | après |
|---|---|---|
| médiane | 53,82 px | **1,81 px** |
| détection | 54,8 % | **100 %** (7/7) |
| sous 10 px | 0,3 % | **99,9 %** |
| pire trame | 274,79 px | 5,79 px |

| `real_3cam` (non-régression) | avant | après |
|---|---|---|
| médiane | 2,32 px | **2,32 px** |
| sous 10 px | 79,2 % | 78,9 % |

**La seconde mesure est aussi importante que la première.** Si `real_3cam` se
dégrade, le fine-tune a échangé un problème contre un autre : remonter la part
synthétique ou baisser l'oversampling.

---

### Pourquoi 1,81 px est plus bas que les 2,32 px de `real_3cam`

Le chiffre du montage est meilleur que celui de l'ancien jeu, et ce n'est pas
parce que le réseau y serait devenu plus précis. La différence est entièrement
concentrée sur les **keypoints distaux** :

| keypoint | `mix_montage0901_test` | `real_3cam` |
|---|---|---|
| `base` | 1,18 px | 2,10 px |
| `link1` | 1,20 px | 2,25 px |
| `link3` | 2,78 px | 7,80 px |
| `link4` | 2,39 px | **14,68 px** |
| `link5` | 2,47 px | **23,29 px** |
| `link6` | 3,22 px | **19,40 px** |
| médiane | 1,81 px | 2,32 px |
| moyenne | 2,06 px | 9,17 px |

Sur les points proches de la base, les deux jeux sont comparables. C'est à partir
de `link4` que ça diverge — six à dix fois pire sur `real_3cam`.

**La cause est la diversité de poses.** `real_3cam` balaie ±80° sur les six axes
contre `[41 12 28 53 63 102]°` pour le bloc de test du montage. Quand le bras se
retourne, l'avant-bras et la bride passent **derrière** lui, ou pointent vers la
caméra et se raccourcissent jusqu'à quelques pixels. Un keypoint occulté ou
écrasé n'a pas de position bien définie dans l'image.

La géométrie de pose le mesure directement. Longueur **apparente** du poignet
dans l'image — le segment `link4`→`link6` — et distance du keypoint distal au
reste du bras, sur 120 trames de chaque jeu :

| | `real_3cam` | `montage_0901` |
|---|---|---|
| poignet apparent, minimum | **4,1 px** | 41,3 px |
| poignet apparent, médiane | 45,8 px | 52,3 px |
| poignet écrasé sous 20 px | **12 / 120** | 0 / 120 |
| distal à moins de 30 px du bras | **12 / 120** | 0 / 120 |

Quand le bras pointe vers la caméra, tout le poignet se réduit à **quatre
pixels** : les trois keypoints distaux se superposent en une grappe, et il
n'existe plus de position distincte à trouver. Ce n'est pas une faiblesse du
réseau, c'est une information absente de l'image.

Le taux de détection le confirme : sur `real_3cam`, `link5` n'est trouvé que
**70,5 %** du temps et `link6` **73,2 %**. Sur le montage, 100 % partout.

**Et il y a un effet pervers là-dedans** : l'erreur n'est calculée que sur les
keypoints *détectés*. Les cas les plus difficiles de `real_3cam` — ceux où
`link5` disparaît — sont donc **exclus du calcul**. Le 23,29 px est déjà la
version indulgente.

Conclusion : le 1,81 px dit *dans la configuration où le pick travaille, la
détection est excellente*. Il ne dit pas que le réseau ferait 1,81 px sur des
poses aussi variées que `real_3cam` — et la ligne `link5 : 23,29 px` prouve le
contraire.

---

## Ce qui n'est PAS validé

**[`scripts/capture_poses_hautes.py`](../scripts/capture_poses_hautes.py) n'a
jamais tourné.** Il vise le domaine d'entraînement (bras ≥ 140 mm) pour la démo
markerless, et il est bloqué : J5 et J6 ne répondent plus aux commandes,
`power_on` compris. Son approche est donc **non éprouvée**.

Il apporte cependant deux contrôles que `capture_trajectoires.py` n'a pas, et
qui valent d'être repris :

- **auto-collision par capsules**, repris du collecteur synthétique.
  `pose_sure` ne vérifie que le sol et le volume de la base — rien n'y empêchait
  un lien de rentrer dans un autre.
- **direction de la pince** comme critère de rejet, pas comme conséquence.
  Relever J2 sans compenser J4 fait basculer l'outil : mesuré, la pince passait
  de 12° à 27–38° de la verticale.

**Le collecteur synthétique à garde basse**
(`synthetic_data_collector_v3_garde_basse.py`, `table_clearance` en paramètre
ROS) est écrit et compilé mais **n'a pas servi** : 1404 poses réelles ont suffi.
Avant toute collecte longue avec lui, vérifier sur les premières images que le
filtre à 130 mm était bien arbitraire et ne masquait pas un artefact de
simulation.

---

## Les erreurs à ne pas refaire

| erreur | ce qu'elle a coûté |
|---|---|
| Tirer des poses au **hasard** sur toute la plage articulaire | 166° entre deux prises, le bras n'a pas suivi, le pont est tombé |
| Croire que les gardes protègent un **trajet** | Elles ne valident qu'une pose immobile ; 5 paliers intermédiaires descendaient à 125 mm |
| Relever J2 sans compenser J4 | La pince est passée à 91° de la verticale, à l'horizontale |
| Exiger la visibilité sur **les deux** caméras | Écarte des poses de travail valides ; `capture_trajectoires.py` juge sur la seule arducam |
| Accuser le plan quand « le bras n'a pas suivi » | La cause était J5/J6 relâchés. Lire les angles **mesurés** et comparer joint par joint |
| Insister après un échec | Trois échecs d'affilée ne sont jamais du bruit : c'est le bras ou le pont |
