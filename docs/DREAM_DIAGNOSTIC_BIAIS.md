# Pourquoi DREAM se trompait de 11 cm — diagnostic et méthode

*31 août – 1er septembre 2026. Branche `feature/pick-and-place-osama`.*

Ce document garde le **raisonnement**, pas seulement les résultats. Les
conclusions sont dans le [`CHANGELOG`](../CHANGELOG.md) ; ce qui suit explique
comment on y est arrivé, et surtout pourquoi on ne pouvait pas y arriver plus tôt.

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

## Où on en est

Jeu réel sur le montage actuel : **1102 poses × 2 caméras**, étiquettes validées
à 3,8 px (arducam) et 2,7 px (svpro), écart médian entre images consécutives
2,46°.

**Limite à connaître** : le jeu est dense mais **étroit**. Amplitude par joint
`[42 21 28 54 64 101]°` contre `[169 161 161 159 169 174]°` pour `real_3cam` —
conséquence des trois filtres (planche, fenêtre réseau, garde au sol). Le réseau
affiné là-dessus sera bon **près de ces poses**, pas ailleurs.

Reste à faire : le mix-fine-tune, puis rejouer
`fk_vs_dream_series.py --balayage` pour mesurer si les 52 px ont bougé. C'est ce
test, et lui seul, qui prouvera ou réfutera l'hypothèse du montage.
