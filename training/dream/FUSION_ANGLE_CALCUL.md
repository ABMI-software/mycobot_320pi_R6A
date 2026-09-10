# Calcul des angles estimés à partir de deux caméras (fusion DREAM)

Ce document explique **comment le dashboard de validation multi-caméras calcule
les 6 angles articulaires** à partir de deux caméras (Arducam + SVPRO), et
détaille les cas de fusion : moyenne pondérée, prise d'une seule caméra, ou
repli encodeur.

Code de référence :
- `mycobot_gateway/mycobot_gateway/dream_validation_dashboard.py` →
  `_fuse_multiview()` et `JOINT_OBSERVING_KP`
- `training/dream/dream_angle_solver.py` → `solve_joint_angles_and_pose`
- `training/dream/mycobot_fk.py` → schéma des 7 keypoints

---

## 1. Architecture : SOLVE-THEN-FUSE (résoudre puis fusionner)

Ce n'est **PAS** un bundle partagé (un seul `q` commun résolu contre les deux
vues). Ce bundle a été essayé puis abandonné : il faisait basculer les branches
monoculaires (J1 partait à −43°).

À la place, deux étapes :

```
Étape A — chaque caméra résout SES 6 angles séparément (mode cohérence)
Étape B — fusion JOINT PAR JOINT, pondérée par l'observabilité
```

```
cam Arducam ──► solve ──► q_arducam (6 angles) ┐
cam SVPRO   ──► solve ──► q_svpro   (6 angles) ┤ ├─► fusion joint/joint ─► q_fused
                                               ┘
```

**Propriété garantie** : la fusion n'est jamais pire que la meilleure caméra sur
un joint donné (une caméra qui n'observe pas un joint a poids 0 dessus).

---

## 2. Étape A — estimation mono (par caméra)

Chaque caméra appelle le solveur `least_squares` qui minimise, sur 12 inconnues
(6 angles `q` + pose caméra 6-DoF `rvec, tvec`, ré-estimée chaque frame car pas
d'extrinsèque persisté) :

```
résidu = [ projeter FK(q) via (rvec,tvec) − keypoints détectés   ]  ← reprojection
         [ reg_weight · (q − q_seed)                             ]  ← prior angles
         [ pose_reg_weight · (pose − pose_seed)                  ]  ← prior pose
```

En **mode cohérence** (`use_encoder_seed=True`, défaut), `q_seed` = angles
encodeur et `reg_weight = _CONSISTENCY_REG_VEC = [10, 40, 40, 40, 40, 1.5]`.
Ce prox fort épingle les joints distaux/mal-observés (J2–J5) sur la branche
encodeur que l'image monoculaire ne peut pas trancher, et laisse J6 (poids 1.5)
à l'encodeur puisqu'aucun keypoint ne l'observe.

Chaque caméra retourne : `q` (6 angles), `valid` (7 booléens : quels keypoints
détectés), `reproj` (7 erreurs de reprojection en px).

---

## 3. Le poids : quels keypoints observent quel joint

Un joint ne peut être estimé par un keypoint que si **tourner ce joint déplace
ce keypoint dans l'image**. Par la cinématique directe, le joint `m` (0-based,
= J{m+1}) n'est observé que par les keypoints **strictement en aval** :

```python
JOINT_OBSERVING_KP = {m: list(range(m + 2, 7)) for m in range(6)}
```

| Joint | keypoints observants | nb max |
|-------|---------------------|--------|
| J1 | kp2,3,4,5,6 *(voir note)* | 5 |
| J2 | kp3,4,5,6 | 4 |
| J3 | kp4,5,6 | 3 |
| J4 | kp5,6 | 2 |
| J5 | kp6 | 1 |
| J6 | — (aucun) | 0 |

> **Note vérifiée numériquement** (`prove_observability.py`, perturbation de
> chaque joint + mesure du déplacement FK, moyenné sur 200 poses) : pour **J1**,
> `kp2` ne bouge PAS en réalité — `link2` est coïncident avec `link1` sur l'axe
> vertical de J1 (`xyz=(0,0,0)` dans `mycobot_fk.py`), donc J1 est réellement
> observé par kp3,4,5,6 (4 observateurs, pas 5). La formule `kp[m+2:]` surévalue
> J1 d'un keypoint. Impact sur l'angle fusionné : **négligeable** (le +1 fantôme
> s'ajoute symétriquement aux deux caméras, ne biaise pas le résultat). J2–J6
> sont exacts. **J6 = ligne entièrement nulle → structurellement inobservable.**

**Poids d'une caméra sur le joint `m`** =

```
w = nombre de keypoints de JOINT_OBSERVING_KP[m] qui sont
    (détectés valides)  ET  (reproj ≤ JOINT_CONFIDENCE_PX_THRESHOLD = 15 px)
```

Un keypoint détecté mais mal reprojeté (> 15 px) compte pour 0 : présent mais
géométriquement incohérent.

---

## 4. Étape B — la formule de fusion

Pour chaque joint `m` (0…5), on parcourt les caméras et on calcule la
**moyenne pondérée** :

```
q_fused[m] = Σ (w_cam · q_cam[m])  /  Σ w_cam
```

Le dénominateur est **la somme des poids** (jamais un nombre fixe). Conséquence :
`q_fused[m]` retombe toujours ENTRE les estimations des caméras.

Si `Σ w_cam = 0` (aucune caméra n'observe le joint) → repli :
`q_fused[m] = q_cam0[m]` (la 1re caméra ; elles sont de toute façon toutes
épinglées à l'encodeur à ce niveau).

---

## 5. Les trois cas, avec exemples chiffrés

### Cas 1 — les deux caméras observent → MOYENNE PONDÉRÉE

Joint J3 (observé par kp4, kp5, kp6). Les deux caméras voient tout :

| | keypoints vus (≤15px) | poids | angle |
|---|---|---|---|
| Arducam | kp4,kp5,kp6 | 3 | 42.0° |
| SVPRO | kp4,kp5,kp6 | 3 | 38.0° |

```
q_fused[J3] = (3·42.0 + 3·38.0) / (3+3) = 40.0°
```

Poids inégaux (Arducam voit 3 kp, SVPRO seulement 2 car kp6 > 15px) :

```
q_fused[J3] = (3·42.0 + 2·38.0) / (3+2) = 202/5 = 40.4°
```
→ tiré vers la caméra qui voit le mieux. **⚠ dénominateur = 3+2 = 5, pas 4.**

### Cas 2 — une seule caméra observe → PRENDRE CETTE CAMÉRA

Joint J4 (observé par kp5, kp6). Le poignet est masqué dans la vue Arducam :

| | keypoints vus | poids | angle |
|---|---|---|---|
| Arducam | aucun (occlusion) | 0 | 15.3° *(seed aveugle)* |
| SVPRO | kp5,kp6 | 2 | 15.0° |

```
q_fused[J4] = (0·15.3 + 2·15.0) / (0+2) = 15.0°   ← 100% SVPRO
```
→ l'occlusion est comblée automatiquement. Une moyenne simple aurait mélangé le
15.3° aveugle d'Arducam ; le poids 0 l'écarte. **C'est la vraie raison de
pondérer.**

### Cas 3 — personne n'observe → REPLI ENCODEUR

Joint J6 (0 keypoint observant, structurellement inobservable) :

| | poids | angle |
|---|---|---|
| Arducam | 0 | = encodeur |
| SVPRO | 0 | = encodeur |

```
Σ w = 0  →  q_fused[J6] = q_cam0[J6]  (= valeur encodeur)
```
→ aucune fusion ne peut fixer J6. Il faudrait un keypoint EXCENTRÉ en aval de J6
(gripper/bride) ajouté au schéma DREAM. Ce cas arrive aussi sur J4/J5 quand les
distaux ne sont détectés par AUCUNE caméra.

---

## 6. Pourquoi pondérer et non moyenner simplement

Le poids = « combien de keypoints observant ce joint cette caméra voit bien » =
mesure directe de la fiabilité de la vue sur ce joint.

| Formule | J4 occlus (Arducam w=0) | Problème |
|---|---|---|
| Moyenne simple `(15.3+15.0)/2` | 15.15° | incorpore un 15.3° **aveugle** |
| Moyenne pondérée `(0·15.3+2·15.0)/2` | 15.0° | écarte la vue sans info |

**Le poids 0 est la fonctionnalité centrale** : il garantit qu'une caméra ne
peut jamais polluer un joint qu'elle n'observe pas. Le nombre de keypoints
observant un joint n'est pas arbitraire — c'est la structure triangulaire
`kp[m+2:]`, c.-à-d. les lignes non-nulles de la jacobienne géométrique du joint,
filtrées par « détecté + bien reprojeté sur la frame courante ».

---

## 7. Tableau récapitulatif d'une frame typique

| Joint | w Arducam | w SVPRO | Cas | Résultat |
|-------|-----------|---------|-----|----------|
| J1 | 4 | 3 | moyenne pondérée | proximaux, les 2 votent |
| J2 | 4 | 3 | moyenne pondérée | — |
| J3 | 3 | 3 | moyenne 50/50 | — |
| J4 | 0 | 2 | une seule | 100% SVPRO (occlusion comblée) |
| J5 | 1 | 0 | une seule | 100% Arducam |
| J6 | 0 | 0 | repli | valeur encodeur |

Le champ `fusion_src_per_joint` du dashboard trace la caméra dominante par
joint ; l'affichage bascule en « MONO via {cam} » quand une seule vue reste.

Après fusion, `q_fused` passe par le filtre temporel sélectionné (`kalman` /
`passe_bas` / `moyenne` / `aucun`) avant affichage — jamais l'encodeur.
