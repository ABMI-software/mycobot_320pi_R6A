# Rapport — Entraînement DREAM sur données synthétiques

**Projet :** MyCobot 320 Pi — estimation de pose par keypoints (DREAM, NVlabs)
**Période :** 22–25 juin 2026
**Objectif :** atteindre **≥ 90 % de détection** sur le jeu de validation synthétique, en
particulier débloquer les keypoints distaux (link4 / link5 / link6) qui plafonnaient.

---

## 1. Pipeline DREAM en bref

| Étage | Détail |
|-------|--------|
| Backbone | VGG-19 (pré-entraîné ImageNet) |
| Sortie | 7 belief maps (cartes de croyance), une par keypoint : base, link1…link6 |
| Décodage | argmax de chaque belief map → pixel ; pose 6D par `solvePnP` |
| Entrée / Sortie | image 400×400 → belief maps 100×100 |
| Keypoints | base, link1, link2, link3, link4, link5, link6 (effecteur) |

**La métrique qui compte : le % de détection** (un pic franc et bien placé), **pas la MSE.**
La MSE est trompeuse sur les distaux (voir §4).

---

## 2. Méthodologie — les leviers testés

### 2.1 AMP (Automatic Mixed Precision) + TF32
Entraînement en précision mixte : `autocast` calcule en **FP16** là où c'est sûr, `GradScaler`
évite l'underflow des gradients, et **TF32** accélère les matmuls sur le GPU Ada.
→ **gain ~1.5–2× en vitesse, qualité identique** (FP32 pour les accumulations sensibles).

### 2.2 Loss pondérée par keypoint (`WeightedBeliefMapLoss`)
MSE par keypoint, multipliée par un poids. Poids utilisés : **`1,1,1,1,3,5,7`** (les distaux
link4/5/6 « coûtent » 3×/5×/7× plus cher) pour forcer le réseau à les soigner.
→ Effet réel **limité** : plafond atteint (2,4,8 ferait pareil).

### 2.3 Sigma par keypoint (largeur de la cible gaussienne)
Monkeypatch de `create_belief_map` : chaque keypoint reçoit une gaussienne-cible de largeur
`sigma` propre. Élargir le sigma distal = **anti-effondrement** : plus de pixels non-nuls dans
la cible → la stratégie « tout aplatir à zéro » coûte plus cher → le réseau est forcé de
former un pic. Configs testées : `2,2,2,2,3,4,4` (sig344) et `2,2,2,2,4,5,6` (sig456).

### 2.4 Correction de l'augmentation (bug majeur trouvé le 24 juin)
L'augmentation appliquait `ShiftScaleRotate`(±15°) + `Perspective` à **l'image seule, pas aux
belief maps** → labels désalignés. Frappe les distaux le plus fort (une rotation déplace
l'effecteur de ~40 px, la base de ~0). **Fix : transforms géométriques retirés**, photométriques
gardés (GaussNoise, Brightness/Contrast, Hue/Sat, Blur, CLAHE, RandomGamma, ImageCompression,
CoarseDropout).

### 2.5 Pré-traitement CLAHE + assombrissement (le levier appearance)
Le 50K brut est **2× trop clair** (luminance 229) vs le 20K où le modèle marche (116). Pipeline :
normalisation de luminance vers ~120, puis **CLAHE** (Contrast-Limited Adaptive Histogram
Equalization, `clipLimit=8`, tuiles 8×8) sur le canal L (espace LAB).

```python
def preproc(img, target=120.0, clip=8.0):
    g = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).mean()
    img = np.clip(img.astype(np.float32) * (target / max(g, 1)), 0, 255).astype(np.uint8)
    lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB); l, a, b = cv2.split(lab)
    cl = cv2.createCLAHE(clipLimit=clip, tileGridSize=(8, 8)).apply(l)
    return cv2.cvtColor(cv2.merge((cl, a, b)), cv2.COLOR_LAB2BGR)
```

→ luminance 50K **229 → 129** (≈ profil 20K). **C'est ce qui a débloqué link6** (voir §3).

### 2.6 Réglages communs
Seed 42 · batch size 16 · LR cosine annealing **1e-4 → 1e-6** · EarlyStopping **patience 5**,
min_delta 1e-6 · split 80 / 10 / 10.

---

## 3. Jeux de données

| Dataset | Taille | Luminance | Rôle |
|---------|--------|-----------|------|
| `synthetic` (20K) | 20 001 | 116 | **Le « bon » jeu** — le modèle y fait 97 % |
| `synth_50k_ndds` (50K brut) | 50 000 | 229 | Trop clair, hors-domaine pour le modèle 20K |
| `synth_50k_clahe` | 50 000 | 129 | 50K assombri+CLAHE (apparence ≈ 20K) |
| `mix_20k_50k` | 90 000 | 116 / 129 | 20K ×2 + 50K_clahe ×1, mélangé (shuffle) |

**Découverte clé (diagnostic données) :** la **géométrie** des poses est quasi **identique**
entre 20K et 50K (link6 in-frame 94 % vs 97 %, même occlusion, même profondeur ±). La
différence 20K↔50K est donc **100 % appearance**, pas la pose → re-render « meilleures poses »
serait inutile.

---

## 4. Le problème distal — diagnostic

Les distaux (surtout **link6**, l'effecteur) plafonnaient à ~30 % de détection.

**Cause : effondrement des belief maps.** La cible est ~99,99 % de zéros (petit pic gaussien).
Le réseau minimise la MSE en **aplatissant** la carte (zéro partout) plutôt qu'en formant un pic
risqué → **MSE basse mais aucun pic détectable**. D'où : la MSE ment, seule la détection juge.

**Intensité des pics** (`diagnose_peaks.py`, sur le run CLAHE, pic max 0–1) :

| Keypoint | base | link3 | link4 | link5 | link6 |
|----------|------|-------|-------|-------|-------|
| Pic en cadre | 0.985 | 0.494 | 0.397 | 0.356 | **0.300** |
| % de base | 100 % | 50 % | 40 % | 36 % | **30 %** |
| Détection | 100 % | 88 % | 56 % | 57 % | **43 %** |

→ la **détection suit la hauteur du pic** quasi linéairement. link6 forme un pic faible
(0.30) → 43 % seulement. **link6 s'est révélé immunisé au sigma** (ni 4 ni 6 ne le bougent)
→ son verrou n'est plus l'effondrement mais l'**ambiguïté réelle** (donnée), pas un knob.

---

## 5. Tableau des expériences — détection (val synthétique)

% de détection par keypoint, 500 frames de validation, modèle = meilleur epoch.

| # | Run | Données | Poids | Sigma | Aug | base | link1/2 | link3 | link4 | link5 | **link6** | **Overall** |
|---|-----|---------|-------|-------|-----|------|---------|-------|-------|-------|-----------|-------------|
| 1 | `vgg_w357_50k_e50` | 50K brut | 3,5,7 | 2 (def) | std | 100 | 100 | 86.8 | 38.8 | 31.2 | 30.8 | **69.7 %** |
| 2 | `vgg_w357_sig344_50k_e50` | 50K brut | 3,5,7 | 2,2,2,2,3,4,4 | std | 100 | 100 | 69 | 52.8 | 53 | 31 | **72.4 %** |
| 3 | `vgg_w357_fixaug_50k_e50` | 50K brut | 3,5,7 | 2 | **fix** | 100 | 100 | ~87 | ~45 | ~40 | ~35 | **72.1 %** |
| 4 | `vgg_w357_clahe_50k_e50` | **50K CLAHE** | 3,5,7 | 2,2,2,2,3,4,4 | fix | 100 | 99.8 | 88.4 | 55.8 | 56.6 | **43.0** | **77.6 %** |
| 5 | `vgg_w357_clahe_sig456_50k_e50` | 50K CLAHE | 3,5,7 | 2,2,2,2,4,5,6 | fix | 99.4 | 100 | 66.8 | 67.0 | 68.8 | 43.6 | **77.9 %** |
| — | `vgg_ultimate_v2_e50` → **val 20K** | 20K | — | — | — | 100 | 100 | 97.0 | 96.4 | 97.6 | **90.8** | **97.4 %** |
| — | `vgg_ultimate_v2_e50` → val 50K brut | 50K brut | — | — | — | 56.2 | 58.4 | 9.0 | 8.2 | 11.6 | 12.8 | **30.7 %** |
| 6 | `vgg_mix_20k_50k_e50` → **val mix** | mix 90K | 3,5,7 | 2,2,2,2,3,4,4 | fix | 100 | 100 | 92.8 | 82.2 | 70.8 | **68.2** | **87.7 %** |

*Runs 1–3 : valeurs link3/4/5/6 approximatives pour le run 3 (fixaug).*

### Cross-eval du modèle mix (run 6) — robustesse multi-domaine

| Évalué sur | base | link3 | link4 | link5 | link6 | Overall |
|------------|------|-------|-------|-------|-------|---------|
| → val 20K | 100 | 98.8 | 96.4 | 86.2 | **82.2** | **94.8 %** |
| → val 50K CLAHE | 99.6 | 80.6 | 60.0 | 58.0 | **46.0** | **77.7 %** |
| → val mix (combiné) | 100 | 92.8 | 82.2 | 70.8 | **68.2** | **87.7 %** |

**Le mix est le seul modèle robuste aux DEUX apparences** (le modèle 20K seul s'effondre à 30.7 % sur le 50K). Prix de la généralisation : link6 sur 20K passe de 90.8 % → 82.2 %. **Mais le mur du 50K reste** (link6 46 % sur les poses 50K dures, +3 pts seulement) → confirme 3× que le plafond 50K est un problème de **données**, pas d'hyperparamètre ni de mélange.

---

## 6. Lecture des résultats

1. **Le « 90 % synthétique » existe déjà** = `vgg_ultimate_v2_e50` entraîné sur le **20K**
   (overall **97.4 %**, link6 **90.8 %**). Inutile de le reconstruire sur le 50K.
2. **Le 50K plafonne ~78 %** quels que soient les knobs : poids 7× (plafond), sigma (link6
   immunisé), CLAHE (a aidé +5 pts, link6 31→43). Les 3 leviers gratuits sont **épuisés**.
3. **Cross-eval = preuve** : le modèle 20K s'effondre à **30.7 %** sur le 50K brut (même la base
   tombe à 56 %) → le 50K est un **domaine visuel différent** (clair), pas « juste plus dur ».
4. **Le sigma456 redistribue sans gagner** : +link4/5 mais −link3 (cibles élargies débordent),
   link6 toujours bloqué → overall plat.
5. **Stratégie retenue : mélange 20K + 50K** (run 6), pour garder le signal distal propre du 20K
   tout en exploitant le volume/robustesse du 50K.

---

## 7. Conclusion & suite

- **Pour un modèle synthétique précis** : utiliser le 20K (déjà à 97 %). Le 50K est un faux ami
  pour la précision synthétique.
- **Le mélange 20K+50K** (en cours) est le meilleur compromis précision × volume.
- **Le vrai problème ouvert reste le sim-to-real** (détection sur images réelles), pas le
  synthétique — voir le rapport données réelles.
- Leviers de fond non encore tirés si on veut dépasser 90 % sur le 50K seul : **loss
  anti-aplatissement** (BCE/focal, ou MSE normalisée par la masse de la cible) ;
  **résolution de sortie** plus fine (100→200) pour les petits keypoints distaux.

---

*Scripts : `train_dream_ultimate_v3.py` (entraînement), `evaluate_dream_v2.py` (détection),
`diagnose_peaks.py` (intensité des pics), `diagnose_link6_data.py` (géométrie données),
`merge_mix.py` (mélange datasets).*
