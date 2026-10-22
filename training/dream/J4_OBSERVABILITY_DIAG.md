# Diagnostic J3/J4 — observabilité mono-caméra (mode cohérence)

*2026-07-15. Contexte : mode cohérence encoder-seedé du `dream_validation_dashboard`
(prior distal `_CONSISTENCY_REG_VEC = [1.5,1.5,40,40,40,1.5]`). MAE session ~5.5°,
glissant ~3°. J1/J2 ~0°, mais J4 restait à 7-9° : biais corrigeable ou faible
observabilité ? Objectif : trancher, sans modifier solveur / régularisation / offsets.*

## 1. Cache (200 poses réelles, détections DREAM) — `distal_reg_sweep.py`, `j3j4_bias_diag.py`

Offset argmin-reprojection par joint (pose caméra fixée à l'oracle PnP, balayage
d'un joint seul, autres à l'encodeur) :

| Joint | offset moyen | σ | \|moy\|/σ | sensibilité |
|-------|-------------|-----|-----------|-------------|
| J1 | −0.23° | 5.86° | 0.04 | 0.331 px/° |
| J2 | +0.37° | 5.19° | 0.07 | 1.707 px/° |
| J3 | −0.01° | 8.39° | 0.00 | 1.376 px/° |
| J4 | −3.04° | 14.75° | 0.21 | **0.315 px/°** |

→ Pas d'offset systématique (J3 ~0, J4 −3° noyé dans σ 15°). J4 est **le joint le
moins observable** : sensibilité 0.315 px/°, ~5× sous J2, ~4× sous J3.

## 2. Live (5 poses, position caméra actuelle) — `j4_live_capture.py` + `j4_live_analyse.py`

**Protocole** : pose de base `[0,-25,-25,J4,40,0]`, J4 ∈ {−30,−15,0,+15,+30}°,
vitesse 25, encodeur = init de branche, J4 DREAM = solve *libre* (reg 1.5) pour
révéler ce que veulent les keypoints. J4 DREAM = `degrees(q[3])`.

| J4 cible | enc J4 | DREAM J4 | signée | abs | reproj | lk5 | lk6 |
|----------|--------|----------|--------|-----|--------|-----|-----|
| −30 | −30.3 | 26.3 | +56.6° | 56.6° | 2.09 px | Y | Y |
| −15 | −15.5 | −32.5 | −17.0° | 17.0° | 2.71 px | Y | Y |
|   0 |  −0.3 | −32.3 | −31.9° | 31.9° | 2.71 px | Y | Y |
| +15 |  14.6 |  37.2 | +22.6° | 22.6° | 1.39 px | Y | Y |
| +30 |  29.6 |  45.8 | +16.2° | 16.2° | 0.45 px | Y | Y |

**Erreur signée J4 (DREAM − enc) : moyenne +9.3° · σ 31.2° · |moy|/σ = 0.30.**

Observations :
- L'erreur signée **change de signe** (+57, −17, −32, +23, +16) → pas de décalage constant.
- **DREAM J4 ne corrèle pas avec l'encodeur** : il erre entre −32° et +46° pendant que
  l'encodeur balaie proprement −30→+30.
- **Reprojection basse partout (0.45–2.71 px)** malgré 17–57° d'erreur J4 → bouger J4 ne
  change quasi rien à l'image.
- **link5 ET link6 valides (Y) dans les 5 poses** → ce n'est pas une détection manquante ;
  les keypoints sont présents mais géométriquement peu sensibles à J4 (faible bras de levier).

## 3. Conclusion

Aucun biais systématique corrigeable n'est identifié sur J3 ni J4 (cache et live
concordent : |moy|/σ ≪ 1, signes alternés). **Aucune correction fixe (offset, poids,
géométrie) n'a été ajoutée** — un offset constant dégraderait autant de poses qu'il
en améliorerait.

Formulé prudemment : **dans la configuration monoculaire actuelle et avec ce schéma de
7 keypoints, J4 reste insuffisamment observable pour une estimation fiable sur toutes
les poses.** C'est une limite d'information (au même titre que J6), non un défaut de
calibration ou de solveur. Gagner ces derniers degrés exige de l'information géométrique
supplémentaire (2ᵉ vue ou keypoint mieux placé en aval), hors du périmètre actuel.

*Artefacts : `scratchpad/j4_live_raw.json`, `j4_live_analyse.py`, `j3j4_bias_diag.py`,
`distal_reg_sweep.py`. Solveur, régularisation et offsets inchangés.*
