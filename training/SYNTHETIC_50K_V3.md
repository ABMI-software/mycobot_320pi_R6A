# Pipeline dataset synthétique 50k (v3) — MyCobot 320 / DREAM

Rapport des étapes réalisées pour générer, valider et convertir le nouveau
dataset synthétique **50 000 images** (pose des 6 joints, 4 caméras), avec filtre
d'auto-collision fiable, distribution d'angles vérifiée et couverture du réel.

Date : 2026-07-02 · Robot : MyCobot 320 Pi (variante sans grippeur)

---

## 0. Emplacements

| Élément | Chemin |
|---|---|
| Dataset brut (images + labels.csv) | `training/dream/dream_data/synthetic_50k/` |
| Dataset **NDDS** (prêt entraînement) | `training/dream/dream_data/synthetic_50k_ndds/` |
| Collecteur | `mycobot_gateway/mycobot_gateway/synthetic_data_collector_v3.py` (ros_jazzy ws) |
| URDF de rendu | `mycobot_description/urdf/320_pi/mycobot_pro_320_pi_gazebo_nogripper.urdf` |
| FK / intrinsèques conversion | `training/dream/mycobot_fk.py` |
| Script conversion NDDS | `training/dream/convert_to_ndds.py` |
| Script entraînement v4 | `training/dream/train_dream_ultimate_v4.py` |
| Plots (distribution + couverture) | dans `synthetic_50k/` (voir §3–4) |

---

## 1. Problème résolu — auto-collision du bras

Les anciennes poses laissaient passer des configurations **repliées sur elles-mêmes**
(bras "en boule", liens qui s'interpénètrent) → images inutilisables pour l'entraînement.

Deux approches testées :

1. **`self_collide` physique de Gazebo/DART** (collisions maillage↔maillage) →
   **NON FIABLE** : le bras atteignait quand même des poses interpénétrées
   (ex. j2=131° j3=140° j4=137° visiblement en boule mais non bloquée). **Abandonné.**
2. **Filtre géométrique capsule déterministe** (dans le collecteur) → **retenu**.

### Filtre capsule (R3 dans `_is_collision_free`)

Chaque lien = **segment (ligne centrale) + rayon**. Deux liens non-adjacents sont
en collision si la distance entre leurs segments < somme des rayons + marge.

- Squelette distinct : `[pts0, pts1, pts3, pts4, pts5, pts6]` (pts1==pts2 : épaule
  composée à translation nulle) → 5 liens **L0..L4** (colonne base, bras, avant-bras,
  poignet, flange).
- `LINK_RADII = [0.055, 0.045, 0.040, 0.036, 0.045]`, marge `0.005`.
- Paires testées `SELF_PAIRS = [(0,2),(0,3),(0,4),(1,3),(1,4)]`.
  La paire (L2 avant-bras, L4 flange) est **exclue** (séparée seulement par le
  poignet 0.084 m → naturellement proche bras tendu, sinon faux rejets).

### Validation hors Gazebo

| Vérif | Résultat |
|---|---|
| Poses normales (N1..N4) | **FREE** ✓ |
| Poses repliées (dont la capture "en boule") | **COLLIDE** (rejetées) ✓ |
| Taux de rejet sur 20 000 poses uniformes | **24.6 %** (vs 17.8 % ancien filtre) |

---

## 2. Génération du 50k

Lancement (GPU NVIDIA forcé + nettoyage DDS + garde anti-figeage/freshness) :

```bash
# nettoyage DDS
rm -f /dev/shm/fastrtps_*
# GPU (évite le rendu logiciel qui fige les caméras)
export __EGL_VENDOR_LIBRARY_FILENAMES=/usr/share/glvnd/egl_vendor.d/10_nvidia.json
export __GLX_VENDOR_LIBRARY_NAME=nvidia
export __NV_PRIME_RENDER_OFFLOAD=1

ros2 launch mycobot_gateway synthetic_data_v3.launch.py \
    num_samples:=12500 output_dir:=/tmp/synthetic_50k multi_view:=true
```

Résultat : **12 500 poses × 4 caméras (front/right/left/top) = 50 000 images**,
1367 poses non-stabilisées écartées, 0 figeage. Échantillonnage **uniforme**,
`grasp_fraction=0` (pas de pics), clearance table 13 cm, filtre capsule actif.

---

## 3. Distribution des angles par joint

Fichiers : `synthetic_50k/joint_distributions_50k.png`, `synthetic_50k/angle_dist.png`.

CoV = écart-type des hauteurs de barres / moyenne (0 = parfaitement plat/uniforme).

| Joint | Plage obtenue | Moyenne | Uniformité |
|---|---|---|---|
| j1 | ±168° | 0.5° | ✅ plat (CoV 0.04) |
| j2 | ±103° | −0.1° | cloche (CoV 0.26 sur plage réelle) |
| j3 | ±145° | 0.2° | ~plat, atténué aux bords (0.26) |
| j4 | ±145° | −0.6° | ✅ plat (0.14) |
| j5 | ±165° | −1.0° | ✅ plat (0.10) |
| j6 | ±180° | −0.0° | ✅ plat (0.04) |

**Lecture :**
- j1/j4/j5/j6 uniformes ; j2 en cloche bridée à ±103° car au-delà l'épaule
  bascule le bras **dans la table / auto-collision** → poses physiquement
  infaisables, rejetées (comportement correct, pas un biais).
- Toutes les moyennes ≈ 0 → cohérent avec le réel.
- **Plus aucun pic-artefact** (contrairement à l'ancien 50k qui avait des pics de
  clamp-limites / poses de saisie à j2=−135°, j5=−90°, etc.).

> Sur le CoV de j2 = 0.62 affiché dans `angle_dist.png` : gonflé par **4 barres
> vides** (plage tracée [−135,135] alors que les données vont à ±103.5). Sur la
> plage réelle [−104,104], CoV = **0.26** (cloche douce).

---

## 4. Couverture du réel (sim doit couvrir real)

Couverture = % des poses de référence dont l'angle tombe dans la plage du NEW-50k.
Fichiers : `synthetic_50k/coverage_REAL4k.png`, `coverage_REAL7500.png`,
`coverage_REAL7500_4k.png`, `coverage_OLD50k.png`.

| Comparaison | j1 | j2 | j3 | j4 | j5 | j6 | MIN |
|---|---|---|---|---|---|---|---|
| NEW-50k vs RÉEL-4k | 100 | 100 | 100 | 100 | 100 | 100 | **100** |
| NEW-50k vs RÉEL-7500 | 100 | 100 | 100 | 100 | 100 | 100 | **100** |
| NEW-50k vs RÉEL (7500+4000) | 100 | 100 | 100 | 100 | 100 | 100 | **100** |
| NEW-50k vs ANCIEN-50k | 100 | 86.5 | 100 | 100 | 100 | 100 | 86.5 |

- ✅ Le synthétique **couvre 100 % du réel** sur les 6 joints, avec marge.
- L'écart j2=86.5 % vs l'ancien 50k = les **pics-artefacts** de l'ancien
  (poses bras-dans-table) que le nouveau rejette → amélioration, pas une perte.

---

## 5. Correction des intrinsèques caméra (avant NDDS)

3 intrinsèques incohérentes trouvées. `mycobot_fk.py` (utilisé par la conversion)
ne correspondait pas au rendu URDF no-gripper. Corrections appliquées
(ros_jazzy + Osama_ws synchronisés) :

| Paramètre | Avant | Après (= rendu URDF) |
|---|---|---|
| `_HFOV` | 1.047 (fx≈554) | **1.15** (fx≈**494**) |
| caméra `top` z | 1.2 m | **0.95 m** |
| cx, cy | 320, 240 | inchangés (centrés) |

---

## 6. Conversion NDDS

```bash
python3 training/dream/convert_to_ndds.py \
    -i /tmp/synthetic_50k \
    -o training/dream/dream_data/synthetic_50k_ndds -s synth
```

Résultat : **50 000** `XXXXXX.rgb.png` + **50 000** `XXXXXX.json` (keypoints) +
`_camera_settings.json` (fx=fy=493.8, cx=320, cy=240). **0 frame skippé**.

**Vérification overlays** (keypoints projetés dessinés sur l'image) sur les
caméras front / top / right → keypoints **parfaitement alignés** sur les joints.
La correction top 1.2→0.95 est validée visuellement.

---

## 7. Entraînement (à lancer)

Script `training/dream/train_dream_ultimate_v4.py` — **même recette que v2
(run qui a atteint 97.7)** : perte pondérée par keypoint
`[1,1,1,1,1.5,1.5,6.0]`, augmentation forte sim-to-real, cosine annealing LR,
early-stopping. Dataset par défaut = `synthetic_50k_ndds`. Option `--pretrained`
pour partir des poids `vgg_ultimate_v2`.

```bash
# from scratch sur le nouveau 50k :
python train_dream_ultimate_v4.py --epochs 50 --batch-size 8

# finetune depuis vgg_ultimate_v2 :
python train_dream_ultimate_v4.py \
    --pretrained checkpoints_dream/vgg_real_ultimate_v2_e15/best_network.pth --epochs 30
```

⚠️ **Prérequis** : la lib DREAM (`/tmp/DREAM`) doit être remise (nettoyée au reboot) :
```bash
git clone https://github.com/NVlabs/DREAM.git /tmp/DREAM
cd /tmp/DREAM && pip install -e . -r requirements.txt
```

Pour le transfert sim→réel, prévoir un **finetune/mix sur le réel** après le
pré-entraînement synthétique (la couverture d'angles est à 100 %, le facteur
limitant restant est l'écart visuel fond/éclairage).

---

## Checklist

- [x] Filtre capsule auto-collision (fiable, déterministe)
- [x] 50 000 images générées (uniforme, sans repli, sans pic)
- [x] Distribution des angles vérifiée (plat où faisable, means ≈ 0)
- [x] Couverture 100 % du réel (4k / 7500 / combiné)
- [x] Intrinsèques corrigées (fov 1.15, top 0.95)
- [x] Conversion NDDS + overlays keypoints validés
- [x] Script d'entraînement v4 créé
- [ ] Remise de `/tmp/DREAM` + lancement entraînement
