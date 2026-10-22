---
name: precision-iso9283
description: Essais de performance de référence du bras MyCobot 320 Pi selon l'ISO 9283 (plan expérimental ABMI V1.0 du 01/10/2026) — essais E0 (encodeurs, sens d'approche), E1 (cible physique), E2 (vision ArUco), E1-bis ; formules AP, RP, vAP ; seuils d'acceptation ; pièges connus du banc. À invoquer pour préparer, exécuter, analyser ou relire une campagne de précision du bras, à Lyon ou à Nanterre.
---

# Précision du bras — ISO 9283

Plan de référence : *Plan expérimental — Performances de référence du bras
MyCobot 320 PI* (V1.0, 01/10/2026, pilote DRI J. Bernardo ; Lyon et Nanterre).
Méthode générale : skill `campagne-mesure`. Historique et pièges de la campagne
de septembre : `training/calibration/PROTOCOLE_ESSAIS_PRECISION.md` et
`METHODOLOGIE_PRECISION.md`.

## Les essais

| Essai | Points | Sens | Cycles | Mesure | Réponses |
|---|---|---|---|---|---|
| E0 | P1 | D0 (haut), D1 +X, D2 +Y, D3 −X, D4 −Y, séries séparées | 10 / sens | encodeurs → FK | RP interne, AP interne, vAP |
| E1 | P1–P5, plan de la table | D0 | 3 sessions × 10 | encodeurs + moyen externe | AP et RP réelles (XY) |
| E2 | 4 ArUco + 1 hors plan | — | 10 prises × 6 trames (médiane) | vision | bruit, erreur leave-one-out |
| E1-bis | P1 | D0 | 4 × 30 (2² vitesse × charge) | comme E1 | effet de H1 |

Chauffe avant toute série ; un essai invalide (chauffe absente, TCP non
conforme, correction en ligne, sens non respecté, défaut robot) se rejoue en
entier.

## Formules (ISO 9283 § 7.2)

```
AP  = √[(x̄−x_c)² + (ȳ−y_c)² + (z̄−z_c)²]
lᵢ  = √[(xᵢ−x̄)² + (yᵢ−ȳ)² + (zᵢ−z̄)²]     l̄ = (1/n)·Σlᵢ
S_l = √[Σ(lᵢ−l̄)² / (n−1)]                 RP = l̄ + 3·S_l
vAP = max ‖G_j − G_k‖  entre barycentres de sens d'approche différents
```

Valeur retenue pour le robot = maximum sur les points. Jamais l'écart maximal à
la place de RP. Intervalle de Student sur l̄ ; sur RP, bootstrap des cycles.
RP par session pour E1, dispersion entre sessions rapportée à part.

## Seuils (proposés, à valider par le pilote DRI)

| Grandeur | Conforme | Avec réserves | Non conforme |
|---|---|---|---|
| RP (E1) | ≤ 0,5 mm | 0,5–1 mm | > 1 mm |
| AP réelle XY (E1) | ≤ 3 mm | 3–5 mm | > 5 mm |
| Bruit vision (E2) | ≤ 1 mm | 1–2 mm | > 2 mm |
| vAP (E0) | informatif | 0,5–1 mm | > 1 mm ⇒ règle d'approche unique |

Spécification constructeur : ±0,5 mm (documentation technique) — la page
produit annonce 1 mm ; citer les deux.

## Pièges connus du banc (mesurés)

- **Consigne** : IK du dépôt puis `send_angles`. `send_coords` est écarté
  (247,8 mm d'erreur le 20/08). La bride du firmware et celle de la FK du dépôt
  diffèrent de ~11 mm : la « position commandée » est celle de la FK du dépôt.
- **Toujours relire `get_angles`** après `send_angles` : hors limite, le bridge
  répond OK et le bras ne bouge pas.
- **Sens d'approche** : biais de ~5,9 mm entre directions, confirmé trois fois.
- **Résolution des codeurs** : en septembre, 5 retours UNI sur 6 identiques au
  1/10 000 mm — la RP interne est une borne inférieure, dominée par la
  quantification.
- **Affaissement de J2** (~1,9° sous la consigne, ~13-15 mm à la pointe) : la FK
  des angles lus le contient, pas la flexion en aval des codeurs.
- **Incertitude de E1** : la règle du rapport 4 impose ≤ 0,125 mm pour une RP de
  0,5 mm ; un trait de marqueur fait 0,3-0,5 mm. Qualifier le marquage avant E1
  ou passer au comparateur.
- **Extrinsèque** valable à Z = 0 seulement tant que E2 n'a pas son marqueur
  hors plan. Marqueurs de la planche jamais déplacés ; exposition arducam 75.

## Scripts

Les scripts de septembre (`campagne_precision.py`, `test_directions.py`,
`bruit_vision.py`…) ont été perdus avec le répertoire de session. Seuls
`scripts/calibrer_extrinseque_4aruco.py` et `scripts/aruco_check.py` subsistent.
Tout nouveau script de cette campagne est versionné, testé (calculs vérifiés sur
`training/calibration/repetabilite_directions_2026-09-09.csv`), et écrit un
`run.yaml` par série.
