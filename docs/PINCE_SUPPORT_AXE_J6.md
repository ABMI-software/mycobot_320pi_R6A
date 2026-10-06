# Pince montée sur support, dans l'axe de J6 — essais du 30/09/2026 et plan de reprise de la FSM

La pince Pro adaptative (`gripper_id=14`) est remontée **comme le prévoit le
constructeur** : fixée sur un support mécanique, dans le prolongement de l'axe
de J6. Avant, elle était fixée **sur le côté** de la bride, à 93 mm de l'axe de
J6. Toute la géométrie du pick en dépend.

Ce document rassemble ce qui a été mesuré et testé ce jour-là sur le banc
réel, puis ce qu'il faut changer dans `scripts/pick_fsm.py` pour que la machine
à états saisisse avec ce montage.

---

## 1. Pourquoi l'ancienne FSM ne marche plus

| | Ancien montage | Nouveau montage (support) |
|---|---|---|
| Position de la pince | sur le côté de la bride, −X bride, 93 mm | dans l'axe de J6 (Z bride) |
| `tool_offset.json` | `[-92.77, 1.07, 22.26]` | **périmé** |
| Pince vers le bas | poignet plié, orientation imposée par `Q_REFERENCE` | **J5 ≈ 90°**, axe de J6 vertical |
| Si on garde l'ancien code | — | l'orientation imposée tient la pince **à l'horizontale** |

La « pose imposée » de l'ancienne FSM, c'est `Q_REFERENCE` / `R_REFERENCE` et
`orientation()`. Toute cible reprenait l'orientation de la prise validée le
20/08, tournée selon l'azimut. Avec le support, la pince pointe
naturellement vers le bas dès que l'axe de J6 est vertical. Cette
orientation imposée doit donc disparaître.

---

## 2. Recalibration des caméras (même jour)

Les deux caméras avaient bougé.

**Arducam** — `calibration_extrinseque_auto.py`, contre `planche_actuelle.yaml`
(marqueurs mesurés au robot), sans `--force` :

| Mesure | Valeur |
|---|---|
| Déplacement détecté | 69,5 mm |
| Reprojection RMS | 0,301 px |
| Leave-one-out (19 / 23 / 25 / 26) | 0,29 / 1,65 / 2,62 / 1,49 mm |
| Contrôle après écriture | 0,12 mm moyen, 0,13 mm pire |

L'ancienne extrinsèque est sauvegardée dans
`training/calibration/arducam_extrinsic_pick.avant_3009_1533.yaml`.

**SVPRO** — elle ne voit plus que les marqueurs **19 et 25** (23 et 26
cachés ou hors champ). J'ai calculé une pose **provisoire** à partir des
8 coins de ces deux marqueurs, relevés à travers l'arducam (reprojection
0,2–1,9 px). Elle a servi à mesurer le décalage pince/cube, mais **n'est pas
écrite en production**. Pour une vraie recalibration, il faut réorienter la
caméra jusqu'à ce qu'elle voie les 4 marqueurs, sans toucher à la planche.

---

## 3. Géométrie mesurée de la pince sur support

Mesures faites sur la première saisie (cube rouge 40 mm), avec la SVPRO pour
le latéral et une descente par paliers pour la hauteur :

| Grandeur | Valeur | Méthode |
|---|---|---|
| Centre de prise, repère bride | **≈ [−24, 0, +146] mm** | décalage latéral lu à la SVPRO, hauteur au dernier palier qui a saisi |
| Bout des doigts **ouverts** sous la bride | ≈ 165 mm | SVPRO, doigt droit contre le cube |
| Orientation de prise | `R = [[1,0,0],[0,-1,0],[0,0,-1]]` (colonnes) : Z bride vers le bas | — |
| Affaissement de la bride | 8 à 11 mm sous la consigne en Z | codeurs relus + FK |
| Pose de prise du cube rouge | `[41.92, -21.79, -93.42, 22.5, 89.73, -48.16]` | codeurs |

⚠ **Ces valeurs ne sont pas encore une calibration d'outil.** Elles viennent
d'un seul essai. Le centre de prise n'est pas le point que la FSM appelle
« pointe » : ce point-là est le bout des doigts **fermés**. Il reste à le
mesurer au réglet, comme le 08/09 (voir § 6).

**Portée** — avec la pince verticale et le coude haut (J3 < 0), le bras bute
sur une singularité (J3 → 0) vers **280 mm** d'allonge au centre de prise.
Les pièces (197 à 271 mm) passent. Les bacs (383 à 433 mm) sont hors
d'atteinte pince verticale : on y lâche **pince inclinée de 15 à 30° vers
l'extérieur**.

---

## 4. Essais : les quatre pièces triées

Les cycles se font en boucle ouverte, sans la FSM, avec
`scripts/cycle_pince_axe_j6.py`. Les positions des pièces viennent de
l'arducam (yolo26 et rétroprojection à la hauteur connue de la pièce).

| Pièce | Position (mm) | Prise | Lâcher | Résultat |
|---|---|---|---|---|
| Cube rouge 40 | (211, 100) | statut 2, calée à 38 | statut 3 | dans le bac, **23 mm** du milieu |
| Pavé jaune | (227, 57) | statut 2 | statut 3 | dans le bac, presque centré |
| Cylindre vert | (192, −45) | statut 2 | statut 3 | dans le bac |
| Cube bleu 50 | (205, 140) puis (205, 177) | statut 2 au 2e essai | statut 3 | dans le bac |

**Échec du cube bleu, 1er essai.** La prise a répondu statut 2, mais le cube
était pincé par le bord : il a glissé à la remontée, déplacé de 37 mm. Le
statut est passé à 3 **avant** l'ouverture, pendant le transport. Après
relocalisation à l'arducam, le 2e essai a réussi. **À retenir :** un statut 2
à la fermeture ne garantit pas une prise tenue. Il faut relire le statut
après la remontée.

**Décalage au lâcher (cube rouge).** Il est de (−19, +13) mm par rapport au
milieu du bac rouge, à 434 mm d'allonge : J2 cède sous la consigne (2,5° d'écart
aux codeurs) et la pince lâche inclinée. Ce décalage est appliqué comme
correction fixe à tous les bacs (`CORRECTION_BAC`). C'est un réglage
empirique, pas encore un modèle.

---

## 5. Le cycle continu

Demande de l'utilisateur : descendre d'un seul mouvement, fermer, puis remonter
et aller au bac sans s'arrêter.

1. **Toutes les poses sont calculées avant le premier mouvement** : survol,
   descente, largage, point haut, dégagement. Aucun calcul d'IK pendant le
   mouvement.
2. **Descente en deux segments enchaînés.** D'un seul trait, l'interpolation
   articulaire dérive de 8,9 mm en XY. En deux segments, 3,2 mm.
3. **Fermeture dès l'arrivée**, statut relu toutes les 0,2 s.
4. **Mouvements enchaînés** : l'ordre suivant part dès que l'écart aux codeurs
   passe sous un seuil (remontée 10°, trajet vers le bac 6°, dégagement 8°).

**Gardes conservées :**

| Garde | Valeur | Effet |
|---|---|---|
| Hauteur minimale sur le trajet (pince et segments) | 80 mm en survol, 60 mm vers le bac | arrêt avant envoi |
| Dérive XY de la descente | ≤ 6 mm | arrêt avant envoi |
| Écart aux codeurs à l'arrivée | ≤ 2,8° | arrêt (contact ou refus) |
| Statut pince après fermeture | 2 obligatoire | arrêt, pince laissée fermée |
| Mode `--essai` | — | calcule et vérifie tout, n'envoie rien |

Commande :

```bash
cd ~/Osama_ws/src/mycobot_R6A
.venv/bin/python scripts/cycle_pince_axe_j6.py <x> <y> <hauteur> <bac_x> <bac_y> --essai
.venv/bin/python scripts/cycle_pince_axe_j6.py <x> <y> <hauteur> <bac_x> <bac_y>
```

L'adresse de la Pi est écrite en dur (`10.10.0.219`) et le bras doit partir de
la pose dégagée `[164.53, 19.24, -32.95, 10.63, 2.37, -38.93]`.

---

## 6. Plan de reprise de la FSM (`scripts/pick_fsm.py`)

Dans l'ordre. Chaque étape se valide sur le robot avant de passer à la
suivante.

1. **Mesurer le nouveau déport d'outil** : le bout des doigts **fermés**, au
   réglet, à plusieurs azimuts, comme le 08/09. Écrire le résultat dans
   `tool_offset.json` et archiver l'ancien vecteur dans `remplace_4`.
2. **Supprimer l'orientation imposée.** `Q_REFERENCE`, `AZIMUT_REFERENCE` et
   `R_REFERENCE` disparaissent. `orientation(p_xy, roulis)` devient « axe de J6
   vers le bas, tourné de `roulis` autour de la verticale ».
3. **Amorces d'IK** : remplacer `_AMORCES_MESUREES` par des poses à J5 ≈ 90°
   (dont la pose de prise du cube rouge).
4. **Portées et inclinaisons** : remesurer `PORTEE_VERTICALE_MAX` (≈ 280 mm au
   centre de prise, contre 355 avant) et `INCLINAISONS_PAR_PORTEE` avec la
   nouvelle géométrie.
5. **Hauteurs** : réétablir `Z_PRISE`, `Z_SURVOL`, `PLANCHER_POINTE` et les
   hauteurs par classe à partir du nouveau déport. Les douze constantes
   décalées le 08/09 allaient avec l'ancien déport.
6. **Largage** : pince inclinée par défaut au-dessus des bacs, et correction
   de l'affaissement de J2 à grande allonge (`affaissement.json`), au lieu de
   la correction fixe (+19, −13).
7. **Prise tenue** : relire le statut après la remontée, pas seulement à la
   fermeture.
8. **Enchaînement continu** (§ 5) dans les états de descente et de transport.

Hors FSM, à faire aussi : recalibrer la SVPRO (4 marqueurs), et aligner le
modèle Gazebo (URDF) sur le nouveau montage.
