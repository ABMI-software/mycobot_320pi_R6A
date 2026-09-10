# Pourquoi DREAM se trompait de 11 cm — diagnostic et méthode

*31 août – 2 septembre 2026. Branche `feature/pick-and-place-osama`.*
*Résolu : 53,8 px → 1,81 px. Le chemin complet est ci-dessous.*

Ce document garde le **raisonnement**, pas seulement les résultats. Les
conclusions sont dans le [`CHANGELOG`](../CHANGELOG.md) ; ce qui suit explique
comment on y est arrivé, et surtout pourquoi on ne pouvait pas y arriver plus tôt.

Pour **refaire** la manipulation plutôt que la comprendre, aller directement au
protocole : [`METHODOLOGIE_CAPTURE_FINETUNE.md`](METHODOLOGIE_CAPTURE_FINETUNE.md).

---

## Le point de départ

La self-calibration markerless laissait un résidu systématique — `link3` à
23,6 px — qu'aucune pose de caméra unique n'expliquait. Deux lectures
possibles : **la FK est fausse**, ou **DREAM détecte mal**. Les deux prédisent
exactement le même désaccord.

## Le piège : DREAM ne peut pas s'auto-vérifier

C'est le cœur de l'affaire, et c'est structurel.

Dans le schéma de NVlabs, la **FK est une entrée du PnP, pas une sortie** :

```
image → réseau → keypoints 2D ─┐
                                ├→ PnP → T_caméra→robot
FK(encodeurs) → keypoints 3D ──┘
```

La self-calibration ajuste donc la pose caméra **sur les détections de DREAM
elles-mêmes**. Si DREAM est biaisé, l'extrinsèque absorbe le biais et le résidu
de reprojection reste petit. En juillet il valait 3,3 px : **ça ressemblait à une
réussite.**

La mémoire du projet, au 10/07/2026, le disait déjà — *« reproj basse TROMPEUSE,
DREAM se cale sur ses propres détections biaisées »*. Le constat était posé, il
n'avait jamais été chiffré.

**Conséquence de méthode : il faut une référence extérieure à DREAM.**

## La méthode

Trois principes, qui ont tous servi plusieurs fois.

**1. Une référence indépendante.** Les extrinsèques **marqueurs**
(`arducam_extrinsic_pick` du 20/08, erreur moyenne 0,33 mm sur les marqueurs) ne
doivent rien à DREAM. On projette le squelette FK à travers elles et on le
compare aux détections.

**2. Deux témoins qui ne peuvent pas se concerter.** Arducam au zénith, SVPRO de
côté, extrinsèques calculées séparément. Deux géométries opposées ne se trompent
pas de la même manière par hasard.

**3. Un juge de paix.** Le keypoint `base` est **fixe** : sa projection ne dépend
d'aucun angle articulaire. S'il tombe juste, la chaîne est saine.

Verdict : le squelette vert épouse le bras dans les deux vues ; DREAM rate le
`base` de 57 px (arducam) et 62 px (svpro). **La FK et les extrinsèques sont
justes, l'écart est du côté de DREAM.** Sans coller un seul marqueur sur le robot.

> Un tag sur la bride aurait tranché aussi, mais il contredisait l'objectif
> markerless — et s'est avéré inutile : les extrinsèques marqueurs déjà
> calculées suffisaient.

## Trois erreurs empilées, prises pour une seule

### a. DREAM : ~52 px, soit 11 cm sur la planche

75 correspondances, 13 poses, FK validée :

| modèle ajusté | RMS | sur la planche |
|---|---|---|
| aucun (DREAM = FK) | 56,9 px | 122 mm |
| translation | 24,3 px | 52 mm |
| similitude (**échelle 0,845**, rot +8,1°) | 19,8 px | 42 mm |
| affine complète (6 param.) | 17,0 px | **36 mm** |

**Aucune transformation 2D globale ne sauve la mise.** La dispersion par pose
vaut déjà 20-24 px : tout le systématique retiré, il reste le bruit propre de
DREAM, ~40 mm.

Cause la plus probable — *probable, pas prouvée* : le réseau a été affiné sur
`real_3cam`, capturé avec l'arducam sur un **autre montage**
(`convert_to_ndds.py:87-92`, écrit depuis toujours, jamais relié au problème).
L'échelle 0,845 dit qu'il voit le robot 15 % plus petit, ce qu'on attend d'un
entraînement caméra plus loin. La seule preuve serait un réaffinage.

### b. La self-cal plaçait la caméra 1 m à côté — deux fautes composées

`self_calibrate_arducam.py:66-67` charge les intrinsèques **`cam_0`**
(focale ≈ 527) alors que cette caméra est **`cam_3`** (≈ 495) : 6,4 % d'écart et
les mauvais coefficients de distorsion. Par-dessus, le PnP était nourri de
keypoints décalés de 52 px — un décalage uniforme en image se traduit par une
grande translation de caméra. Les deux tiraient dans le même sens.

`arducam_extrinsic_dream_v4.yaml` n'était donc **comparable à rien**, et son
désaccord de 1 m sort des questions ouvertes.

### c. La SVPRO avait bougé de 64 mm

Pas un bug : une extrinsèque périmée que personne ne rejouait. 21,5 px d'erreur
d'étiquette.

## Cinq hypothèses tuées par la mesure

Deux étaient les miennes. C'est en les écartant une par une qu'il n'est resté que
le montage d'entraînement.

| hypothèse | mesure | verdict |
|---|---|---|
| la FK est fausse | vert collé au bras, 2 caméras, 2 poses | écartée |
| l'exposition arducam | balayage 20→300 : 0 à 4/7, aucun optimum | écartée |
| le recadrage réseau (`shrink-and-crop`) | 7/7 keypoints dans la fenêtre, 51 px quand même | écartée |
| un décalage constant | `dx` suit J1 : +8 px à 0°, +53 px à 60° | écartée |
| **agrandir le robot dans le cadre** | erreur **doublée**, 61 → 143 px | écartée |

La dernière mérite un mot, car elle est contre-intuitive et c'est elle qui donne
la clé. Le Panda de NVlabs occupe 53 % de la largeur de l'image, notre bras 29 %.
L'arducam sort du 1600×1200 : capturer en grand, recadrer autour du robot et
redescendre en 640×480 amène **exactement** à 53 %. L'image est visiblement plus
nette — et l'erreur double. La transformation d'intrinsèque a été vérifiée
correcte (le squelette FK suit le bras au pixel dans l'image recadrée), donc le
résultat est réel.

> **Le réseau n'est pas limité par la taille du robot : il est verrouillé sur le
> cadrage de son affinage.** Une image que l'œil trouve meilleure lui est
> étrangère. D'où la règle qui dicte toute capture future : *le cadrage
> d'entraînement doit être celui de l'inférence.*

Le recadrage explique en revanche autre chose : `shrink-and-crop` 640×480 →
400×400 ne garde que **x ∈ [80, 560]** et jette 25 % de l'image. Bras à gauche
(J1 ≈ 88°), la bride tombe à x ≈ 60, hors champ réseau, détection 1/7. Ce n'était
pas « le bras a quitté la planche ».

## Ce que fait NVlabs, mesuré sur `panda-3cam_azure`

| | NVlabs Panda | `real_3cam` |
|---|---|---|
| images (1 caméra) | 6394 | 2500 |
| écart image à image (médiane) | **0,23°** | **94,3°** |
| structure | 10 trajectoires continues | 2500 poses indépendantes |
| robot pendant la capture | en mouvement, ~10°/s | à l'arrêt |
| robot dans le cadre | 53 % de la largeur | 29 % |

Ils **filment** des trajectoires lentes. Le champ `velocity` non nul de chaque
json le prouve : sur 170 images consécutives, une seule a le robot immobile.

**On reprend leur densité, pas leur mouvement.** NVlabs enregistre l'état
articulaire synchronisé à la trame ; ici les angles arrivent par requête-réponse
TCP, et à 10 °/s une latence de 100 ms fait **1° d'erreur d'étiquette**. D'où un
arrêt à chaque prise — angles **mesurés**, jamais la consigne — avec des pas de
2,5°.

## Le plan de capture, en détail

Le plan est **déterministe** : aucun tirage aléatoire, aucune pose inventée. Il
se déduit de quatre poses de travail et de six balayages.

**Les quatre bases** sont des poses réellement jouées par l'opérateur, reprises
telles quelles :

```python
BASES = [
    [30, -118.7, 82.8, -102.6, -17.8, 49.8],
    [30, -128.7, 76.5,  -53.4,  -3.7, 21.8],
    [30, -120.0, 90.0,  -60.5,  10.3,  6.2],
    [30, -110.0, 70.0,  -80.0, -10.0, 30.0],
]
```

**Les six balayages** font varier une seule articulation à la fois, sur sa plage
de travail :

```python
BALAYAGES = [
    (0,    8,  58),   # J1 : azimut, conserve l'inclinaison de l'outil
    (1, -134, -108),  # J2 : hauteur
    (2,   68,  96),   # J3 : allonge
    (3, -104, -50),   # J4 : poignet
    (4,  -42,  22),   # J5 : inclinaison outil
    (5,  -10,  92),   # J6 : rotation outil, invisible en FK mais vue en image
]
```

**4 bases × 3 azimuts × 6 balayages = 72 trajectoires**, échantillonnées à 2,5°.
Varier un seul axe par trajectoire donne des images voisines qui ne diffèrent
que par un mouvement, ce qui est exactement ce que fait NVlabs — sans le
mouvement continu, impossible à étiqueter ici.

J6 est balayé bien qu'aucun keypoint ne dépende de sa rotation : il ne déplace
rien dans la FK, mais il **change l'image** (la pince tourne), et le réseau doit
apprendre que cette variation n'est pas un signal.

**Trois filtres, dans cet ordre**, appliqués avant que le bras ne bouge :

| filtre | rôle | écartées |
|---|---|---|
| `pose_sure` | pointe ≥ 40 mm, liens mobiles ≥ 30 mm, hors volume de base | **525** |
| `visible` | les 7 keypoints dans la fenêtre réseau 400×400, marge 30 px | **29** |
| — | retenues | **1102** |

Les 525 écartées comme dangereuses sont la moitié du plan brut : la sécurité
n'est pas un garde-fou de principe ici, elle taille vraiment dans le plan.

```
1102 poses × 2 cameras = 2204 images
ecart median entre images consecutives : 2,46°   (NVlabs 0,23 ; real_3cam 94,3)
duree : ~48 min
```

**Trois règles de fabrication**, chacune apprise à ses dépens :

1. **L'étiquette est l'angle MESURÉ, bras arrêté** — jamais la consigne.
   `immobile()` attend deux lectures consécutives à moins de 0,35° l'une de
   l'autre. Une pose que le bras n'atteint pas à 6° près est **ignorée**, pas
   enregistrée avec sa consigne.
2. **On rejoint un départ de trajectoire par paliers de 30°.** Un ordre unique
   depuis loin fait partir toutes les articulations à fond en même temps.
3. **La capture écrit dans `training/dream/captures/`, jamais dans
   `dream_data/`**, et le script refuse toute racine hors de là. `labels.csv`
   s'ouvre en `append` : une capture interrompue reprend où elle s'est arrêtée.

## Deux corrections venues du réel, pas du calcul

**La pince a touché la table.** Le filtre de sécurité repris de
`capture_real_3cam.py` est un modèle **plan** sur J2, J3, J4 : il **ignore J5**,
qui incline l'outil. Nos balayages font varier J5 de −42° à +22°. Neuf poses
mettaient la pointe sous la table. Corrigé sur la FK complète, avec l'axe de la
pince **mesuré** et non supposé : −X de la bride est le seul qui place la pointe
à 23 mm au point `pick` et 5 mm au `handover` ; tous les autres la mettraient 120
à 310 mm en l'air, impossible pour une saisie.

**Le bras s'affaisse.** Garde calculée à 25 mm, pointe réelle mesurée à 18 mm :
7 mm d'affaissement gravitaire (J2 tombe ~1,9° sous la consigne). **Une
simulation cinématique ne peut pas le voir** — il faut mesurer les poses jouées.
Garde portée à 40 mm.

## Récupérer la SVPRO : un raisonnement à corriger

Premier verdict : « elle ne décode que 2 marqueurs sur 4, donc PnP impossible ».
**Faux.** Deux marqueurs ne donnent pas 2 points mais **8 coins**, et 8 points
coplanaires suffisent.

Les positions 3D de ces coins ne sont pas supposées : elles sont
**rétro-projetées depuis l'arducam** sur le plan Z=0, celle-ci étant validée.
Contrôle de cohérence — côtés reconstruits 50,7 / 49,2 / 50,7 / 49,4 mm pour un
marqueur de 50 mm. L'ordre canonique des coins ArUco règle la correspondance
entre vues exactement.

Validation **hors ajustement**, sur le marqueur 25 (jamais utilisé, à 40 cm des
deux autres, localisé par contours puisque le détecteur ne le trouve même pas) :
**4,5 px** contre 18,4 px pour l'ancienne extrinsèque.

> Le RMS de 0,66 px sur les points ajustés ne prouvait rien : 8 points pour 6
> degrés de liberté, c'est presque de l'interpolation. Le test « un marqueur de
> côté » donnait 11,8 et 14,5 px, mais il est **pessimiste** — ajuster six
> degrés de liberté sur un seul carré de 50 mm est mal conditionné. Le chiffre
> honnête est celui du marqueur tenu entièrement dehors.

## Outils laissés

| script | rôle |
|---|---|
| [`scripts/fk_vs_dream_diagnostic.py`](../scripts/fk_vs_dream_diagnostic.py) | FK ↔ DREAM sur les deux caméras, bras immobile. `--brut` rejoue hors ligne |
| [`scripts/fk_vs_dream_series.py`](../scripts/fk_vs_dream_series.py) | mesure du biais sur plusieurs poses ; `--balayage` teste sa stabilité |
| [`scripts/capture_trajectoires.py`](../scripts/capture_trajectoires.py) | capture par trajectoires à petits pas, garde au sol, `--simuler` |
| [`scripts/convert_capture_ndds.py`](../scripts/convert_capture_ndds.py) | conversion NDDS avec distorsion et extrinsèques marqueurs |

Tous **autonomes** : ni `pick_dashboard.py`, ni `capture_real_3cam.py`, ni
`convert_to_ndds.py` ne sont modifiés ou importés.

## Le jeu réel produit

**1102 poses × 2 caméras**, étiquettes validées à 3,8 px (arducam) et 2,7 px
(svpro), écart médian entre images consécutives 2,46°.

**Limite connue dès la capture** : le jeu est dense mais **étroit**. Amplitude
par joint `[42 21 28 54 64 101]°` contre `[169 161 161 159 169 174]°` pour
`real_3cam` — conséquence des trois filtres (planche, fenêtre réseau, garde au
sol). Cette étroitesse, qu'on prenait pour un défaut, s'est révélée être
l'indice principal.

---

# Deuxième partie — la cause, et sa correction

## L'indice qu'on regardait sans le voir

En comparant les distributions articulaires avant de lancer l'entraînement,
J2 est apparu **disjoint** :

```
J2 — histogramme, largeur 10 deg
synth      .+##################+.
ancien       .+##############+.
montage +##
        -140      -110      -80       -50       -20       10        40       70      100
```

Le synthétique s'arrête à **−103,5°**, le montage commence à **−110,4°**. Un
trou de 6,9°, et **0 pose sur 50 000** sous −104°. Sur les six axes
simultanément, la couverture du montage par le synthétique est de **0,0 %**.

Ce n'était pas une queue de distribution qui s'amenuise : 432 poses dans la
dernière tranche, puis zéro. Un **mur**. Or J1 atteint ±167,9° et J3 ±145°,
leurs limites pleines. J2 n'était donc pas bridé par l'échantillonneur.

## La cause : une constante dans le générateur

[`synthetic_data_collector_v3.py:505`](../mycobot_gateway/mycobot_gateway/synthetic_data_collector_v3.py#L505) :

```python
TABLE_CLEARANCE = 0.13     # toute pose descendant sous 130 mm est rejetee
```

Et la mesure sur les 1102 poses réelles, avec **la même définition** que le
filtre (échantillonnage le long des segments, colonne de base exclue) :

```
hauteur mini du bras   min 72,5 mm   mediane 91,2 mm   max 114,5 mm
rejetees par le filtre a 130 mm :  1102/1102   (100,0 %)
```

**Cent pour cent.** Le jeu synthétique ne pouvait structurellement pas contenir
la tâche. Et descendre bas oblige à fermer l'épaule : interdire le bas coupe J2.

**Vérification directe sur le générateur**, sans Gazebo — le filtre ne dépend que
de constantes de classe et de sa FK, donc il se rejoue hors ligne sur des poses
tirées uniformément :

| garde | acceptées | J2 min / max | dans [−131, −110] |
|---|---|---|---|
| **130 mm** | 48,4 % | **−103,6 / 103,6** | **0** |
| 80 mm | 58,6 % | −127,0 / 127,0 | 3,2 % |
| 50 mm | 62,9 % | −132,8 / 132,8 | 4,0 % |

À 130 mm, le générateur reproduit **exactement** le mur du jeu 50k : ±103,6°
calculé contre −103,5° mesuré. La cause est établie, pas déduite.

## Ce que ça écarte

L'hypothèse tenue jusque-là — un montage de caméra différent à l'entraînement —
devient inutile. Et deux autres explications tombent avec elle :

- **Ce n'est pas la calibration.** La vérité terrain reprojette à 2,7–3,8 px,
  vingt fois moins que l'erreur mesurée. Si la référence était fausse, l'écart
  serait de son ordre de grandeur.
- **Ce n'est pas la fusion multi-caméras.** Elle intervient *après* la
  détection : chaque caméra résout son `q`, puis on fusionne les solutions. Or
  le keypoint `base` est raté de **57 px sur l'arducam et 62 px sur la SVPRO** —
  deux montages, deux extrinsèques indépendantes, même erreur. Fusionner 57 et
  62 ne donne pas 3.

Le 1,1–1,9° du dashboard multicam ne contredit rien : ce solveur part des
**encodeurs** à chaque trame (`q_init = self.latest_joint_q`) et y est épinglé
par `_CONSISTENCY_REG_VEC`. Son propre commentaire le dit — *« a
consistency/refinement result, NOT an independent DREAM prediction »*.

## Construire un test qui ne mente pas

Deux pièges, tous deux mesurés avant d'entraîner.

**Le tirage aléatoire était exclu d'emblée** : deux images consécutives sont à
2,46° l'une de l'autre, quasi jumelles.

**Mais le bloc contigu ne suffit pas non plus.** La trajectoire repasse sur ses
pas — 121 poses sur 1042 reviennent à moins de 2,5° d'une pose vue plus de
50 trames plus tôt :

| test | médiane à la plus proche du train | jumelles < 2,5° |
|---|---|---|
| 150 dernières poses | 14,2° | **30 / 150** |
| 250 dernières poses | 12,6° | 53 / 250 |
| **400 dernières poses** | **26,0°** | **1 / 400** |

À 150, un cinquième du test avait son jumeau dans l'entraînement. La coupe à 400
passe avant l'excursion qui revisite la fin de trajectoire.

**Un troisième piège, celui-là de fabrication.** En reconstruisant à 400
par-dessus un tirage à 150, le dossier gardait la queue du premier : 2 500 liens
périmés, dont 129 pointant sur des trames passées du train au test — une fuite
créée par la reconstruction, invisible dans le compte affiché. D'où le `purge()`
de [`build_mix_ndds.py`](../scripts/build_mix_ndds.py), qui refuse tout ce qui
n'est pas un lien symbolique.

## Le mélange

```
synthetique                20 000   51,2 %    le socle general
montage_0901 (x10)         14 040   36,0 %    la region ou le reseau est aveugle
real_3cam                   5 000   12,8 %    l assurance contre l oubli
                           39 040
```

Les 12,8 % de `real_3cam` ne sont pas décoratifs : sans eux le réseau optimise la
nouvelle région en abandonnant l'ancienne, et on déplace le problème au lieu de
le résoudre. Les 36 % reprennent la proportion du mélange qui avait fait passer
le réel de 26 % à 91,6 % en juillet.

Tout est en liens symboliques : 351 Mo réels, pas une image copiée.

## Le fine-tune

Départ depuis `vgg_ultimate_v4_mix_ft_e30`, **jamais modifié** — seulement lu
comme poids initiaux. Sortie dans un dossier neuf. Aucun script d'entraînement
touché : `train_dream_ultimate_v4.py` accepte déjà `--data`, `--pretrained` et
`--output`.

```bash
python3 train_dream_ultimate_v4.py \
  --data dream_data/mix_montage0901_train \
  --pretrained checkpoints_dream/vgg_ultimate_v4_mix_ft_e30/best_network.pth \
  --output checkpoints_dream/vgg_montage0901_ft_e30 \
  --epochs 30 --batch-size 8 --lr 0.0001
```

30 epochs, 6 h 43, meilleure epoch 29.

## Le verdict

Sur les **800 trames tenues à l'écart** (400 poses × 2 caméras, index 702–1101,
jamais vues) :

| | avant | après |
|---|---|---|
| médiane | 53,82 px | **1,81 px** |
| détection | 54,8 % | **100 %** (7/7 keypoints) |
| sous 10 px | 0,3 % | **99,9 %** |
| pire trame | 274,79 px | 5,79 px |
| `base` | 66,96 px | 1,18 px |
| `link5` | 59,64 px | 1,94 px |

Et le contrôle de non-régression sur `real_3cam` : **2,32 px de médiane avant et
après**, au centième près ; sous 10 px 79,2 % → 78,9 %.

**1404 poses réelles ont suffi.** La régénération synthétique à garde basse,
préparée et compilée
([`synthetic_data_collector_v3_garde_basse.py`](../mycobot_gateway/mycobot_gateway/synthetic_data_collector_v3_garde_basse.py),
`table_clearance` en paramètre ROS), n'a pas eu à servir.

## Ce que ce chiffre ne dit pas

Le 100 % de détection mérite d'être lu correctement. Trois choses jouent :

1. Le jeu est **construit** pour n'avoir aucun keypoint hors champ —
   `capture_trajectoires.py` pré-filtre avec `visible()`. Mesuré : 100 % des
   keypoints dans la fenêtre réseau, contre 97,6 % pour `real_3cam`. Ça ne fait
   que 2,4 points.
2. La **plage articulaire du test est étroite** : `[41 12 28 53 63 102]°` contre
   `[168 161 161 159 169 174]°`. Sur `real_3cam` le bras se retourne, se
   raccourcit en perspective, et les liens distaux passent derrière lui — c'est
   ce qui fait tomber `link5` à 70,5 % là-bas.
3. C'est **la même scène** : même planche, même éclairage, mêmes caméras au même
   endroit.

Le résultat est donc honnête sur ce qu'il mesure — *sur ce banc, dans cette
configuration, le réseau trouve les 7 points à 1,81 px* — et c'est exactement ce
qu'il fallait pour le pick et pour la démo markerless. Il ne prouve pas que le
réseau généralise à une autre scène. Pour trancher ça : déplacer une caméra ou
changer l'éclairage, et réévaluer **sans** réentraîner.

## Outils laissés (deuxième partie)

| Fichier | Rôle |
|---|---|
| [`scripts/build_mix_ndds.py`](../scripts/build_mix_ndds.py) | mélange par symlinks, test tenu à l'écart, `purge()` |
| [`scripts/svpro_verrou_focus.sh`](../scripts/svpro_verrou_focus.sh) | verrou de mise au point, à rejouer après chaque rebranchement |
| [`scripts/svpro_extrinsic_4_marqueurs.py`](../scripts/svpro_extrinsic_4_marqueurs.py) | extrinsèque sur 16 coins, validation en laissant un marqueur dehors |
| [`scripts/capture_poses_hautes.py`](../scripts/capture_poses_hautes.py) | capture dans le domaine d'entraînement, auto-collision par capsules |
| `synthetic_data_collector_v3_garde_basse.py` | sous-classe, `table_clearance` en paramètre ROS |

## Reste ouvert

- **J5 et J6 ne répondent plus** aux commandes, `power_on` compris (J6 : 0,0° de
  déplacement pour +20° commandés). Le pont n'expose rien de plus fin.
- **La démo markerless** attend ce poignet. La condition qui manquait aux deux
  tentatives de juillet est enfin réunie : référence marqueurs et images de la
  **même session**, caméras non bougées entre les deux.
- **Confirmer en direct** avec `fk_vs_dream_series.py --balayage` et le nouveau
  checkpoint, sur le robot.
