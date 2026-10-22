# Plan — Fine-tune/mix réel (real_3cam) sur DREAM v4

Plan d'action préparé le 2026-07-03. Objectif : faire progresser la détection
réelle (actuellement ≈27 % sur `real_3cam_ndds`, stable depuis v2) sans perdre
l'acquis synthétique de v4 (99,4 % détection, val_loss 0.000750).

**Mise à jour 2026-07-03 (soir) — blocage extrinsèques levé, plan débloqué.**
Les extrinsèques caméra (`REAL_CAMERA_TRANSFORMS` dans `convert_to_ndds.py`)
étaient des placeholders devinés à la main, tellement faux que la vérité-terrain
2D tombait complètement à côté du bras sur l'image (vérifié visuellement avec
`visualize_ndds.py`). Recalage manuel rapide effectué : annotation visuelle de
2 keypoints (base + link6) sur 5 poses (`real_3cam` index 0/500/1000/1500/2000),
résolu par `cv2.solvePnP` (seedé depuis les anciens placeholders pour rester
dans la branche de solution physiquement plausible). Résultat :
- Overlay GT désormais visuellement correct : `base` tombe exactement sur le
  socle dans les 3 caméras ; link2/link3 proches des vrais coudes ; link4/5/6
  suivent la bonne direction générale mais dérivent encore un peu (seulement
  2 points ancrés par pose).
- Erreur pixel moyenne sur `real_3cam_ndds` (7500 images) : 103,1 px → **63,2 px**
  (−39 %). link6 : 349,2 px → **139,7 px** (−60 %).
- Taux de détection inchangé (27,0 %) — confirmé indépendant de la GT (voir
  code `evaluate_dream.py`), donc cette mesure était déjà fiable avant.
- **Reste approximatif** : 2 landmarks/pose, annotation à l'œil, pas de vraie
  calibration hand-eye (mire ChArUco sur la bride). Suffisant pour ne plus
  risquer de corrompre un fine-tune (§2.6-2.7 ci-dessous), mais une vraie
  calibration hand-eye améliorerait encore la précision si besoin.
- Valeurs sauvegardées dans `convert_to_ndds.py` (commentaire daté au-dessus
  de `REAL_CAMERA_TRANSFORMS`), datasets régénérés
  (`real_3cam_{arducam,svpro,astra}_ndds`, `real_3cam_ndds`).

first :
git clone https://github.com/NVlabs/DREAM.git /tmp/DREAM
--------------------------------------------------------
cd /tmp/DREAM && pip install -e . -r requirements.txt
---

## 0. Contexte — pourquoi ce plan

- **v4** (`vgg_ultimate_v4_e50`) a atteint 99,4 % de détection en synthétique
  (`synthetic_50k_ndds`, val split) — dépasse l'objectif 98 %+ et le score v2 (97,7 %).
- Mais évalué sur le réel, **v4 plafonne à ≈27 %** de détection
  (`real_3cam_ndds`, 7500 images, 3 caméras) — quasiment identique à v2 (~26 %).
  Le gain synthétique ne transfère pas.
- Diagnostic (voir conversation du 2026-07-02/03) :
  - Les 3 keypoints proches de la base (base/link1/link2) sont détectés ~50 % du
    temps ; l'effondrement commence à link3 (19,5 %) et devient sévère sur
    link4/5/6 (6,6 % / 1,8 % / 1,0 %).
  - L'augmentation d'échelle d'entraînement (`ShiftScaleRotate scale_limit=0.1`,
    soit ±10 %) est plus étroite que l'écart de focale mesuré entre la caméra
    synthétique (fx=493,8) et les caméras réelles calibrées :

    | Caméra | fx | Écart vs synthétique |
    |---|---|---|
    | Synthétique (entraînement) | 493.8 | référence |
    | arducam | 525.7 | +6,4 % |
    | svpro | 559.7 | +13,3 % |
    | astra (non calibrée) | 610.0 | +23,5 % |

  - `real_3cam` (raw) a été converti en NDDS, une fois par caméra
    (`real_3cam_arducam_ndds`, `real_3cam_svpro_ndds`, `real_3cam_astra_ndds`,
    2500 frames chacun) puis fusionné en un seul dossier multi-vue
    (`real_3cam_ndds`, 7500 frames, 0 skip) pour couvrir les occlusions comme
    le fait le synthétique avec ses 4 caméras.

---

## 1. Piège identifié — ne pas utiliser `merge_and_convert.py --ndds` sur du mixte

`merge_and_convert.py --ndds <path>` appelle en interne `convert_synthetic()`
(dans `convert_to_ndds.py`), qui applique les **intrinsèques Gazebo** (fx=493.8)
et les **transforms caméra synthétiques** (front/right/left/top) à *toutes* les
images qu'on lui donne. Si on lui passait des images arducam/svpro/astra, il
écraserait les vraies coordonnées 2D calibrées obtenues via `convert_real()`.

**Conclusion : `merge_and_convert.py` ne sert qu'à dupliquer/renuméroter du
brut homogène** (même jeu de caméras dans tous les datasets d'entrée). La
fusion réel + synthétique doit se faire **après** conversion NDDS de chaque
côté, pas avant.

---

## 2. Étapes prévues

### 2.1 Split train/val sur les POSES (pas les images) de `real_3cam`

Split par pose (2500 poses) pour qu'une même pose (3 vues caméra) ne se
retrouve jamais à cheval entre train et val — sinon la mesure de gain réel
serait faussée par une fuite (leakage). Exemple : 2000 poses train (6000
images) / 500 poses val (1500 images), val jamais oversamplé ni vu en
entraînement.

### 2.2 Oversampling ×5 du train réel

```bash
python merge_and_convert.py \
  --datasets dream_data/real_3cam_train dream_data/real_3cam_train \
             dream_data/real_3cam_train dream_data/real_3cam_train \
             dream_data/real_3cam_train \
  --merged dream_data/real_3cam_train_x5
```

(Pas de `--ndds` ici — voir §1.)

### 2.3 Conversion NDDS du réel oversamplé (chemin correct, déjà validé)

```bash
python convert_to_ndds.py \
  --input dream_data/real_3cam_train_x5 \
  --output dream_data/real_3cam_train_x5_ndds \
  --source real --cameras arducam svpro astra
```

### 2.4 Fusion au niveau NDDS (pas au niveau brut)

Concaténer `synthetic_50k_ndds` (50 000 frames, déjà correct) +
`real_3cam_train_x5_ndds` (30 000 frames) → dossier final ~80 000 frames,
renumérotés en continu. Chaque frame garde ses propres annotations 2D
correctes (déjà calculées avec les bonnes intrinsèques/extrinsèques lors de
sa conversion respective). Le `_camera_settings.json` final sera cosmétique
(vérifié : le prétraitement "shrink-and-crop" ne dépend que de la résolution
image, pas de fx) — sans impact sur l'entraînement des belief maps.

*(Script de fusion NDDS-à-NDDS à écrire — n'existe pas encore, contrairement
à la fusion brut-à-brut de `merge_and_convert.py`.)*

### 2.5 Élargir l'augmentation d'échelle

Dans `train_dream_ultimate_v4.py`, passer `ShiftScaleRotate(scale_limit=0.1, …)`
à environ `0.3` — couvre le pire écart de focale mesuré (astra +23,5 %).
Choix du script custom plutôt que `train_network.py` natif de `/tmp/DREAM` :
ce dernier n'expose pas ce paramètre en CLI.

### 2.6 Fine-tune depuis `best_network.pth`

```bash
python train_dream_ultimate_v4.py \
  --data dream_data/<fusion_finale> \
  --pretrained output/checkpoints_dream/vgg_ultimate_v4_e50/best_network.pth \
  --epochs 20-30
```

Fine-tune (pas from-scratch) pour ne pas perdre l'acquis synthétique (99,4 %).
20-30 epochs a priori suffisant pour un fine-tune vs. 50 pour un entraînement
complet.

### 2.7 Réévaluation sur le split val réel réservé

```bash
python evaluate_dream.py \
  --weights <nouveau best_network.pth> \
  --data dream_data/real_3cam_val_ndds --split all
```

Seule mesure honnête du gain réel : les 500 poses jamais vues, jamais
oversamplées.

---

## Checklist

- [ ] Split train/val par pose sur `real_3cam` (2000/500 poses)
- [ ] Oversampling ×5 du train réel (brut, `merge_and_convert.py`)
- [ ] Conversion NDDS du réel oversamplé (`convert_to_ndds.py --source real`)
- [ ] Script de fusion NDDS-à-NDDS (synthétique + réel ×5) — à écrire
- [ ] `scale_limit` 0.1 → 0.3 dans `train_dream_ultimate_v4.py`
- [ ] Fine-tune depuis `best_network.pth`, 20-30 epochs
- [ ] Réévaluation sur le split val réel réservé (jamais vu)

**Statut : plan validé dans la conversation, exécution en attente du feu vert
utilisateur (plusieurs heures de calcul GPU).**
