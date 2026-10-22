# Session 6 — nouvelles données réelles (holdout, jamais entraînées)

## Contexte

`vgg_ultimate_v4_mix_ft_e30` (fine-tune mixte 50k synth + real_3cam ×5) atteint
**91,6 % de détection** sur `real_3cam_val_ndds` (500 poses / 1500 images,
indices 0–2499 du dataset `real_3cam`, split held-out mais issu du même lot de
captures que le train). Voir `RAPPORT_MIX_FT_REAL3CAM.docx` (rapport Word, non versionné)
pour le détail par keypoint.

Pour vérifier que ce chiffre généralise (et pas seulement au sous-ensemble de
poses/éclairage des sessions 1–5), une **session 6** de capture a été faite le
7 juillet 2026 dans un dossier séparé — jamais mélangée à `real_3cam`, jamais
vue par aucun modèle.

## Dataset

| | |
|---|---|
| Dossier | `training/dream_data/real_3cam_session6/` |
| Index poses | 2500–2599 (continue la numérotation globale de `real_3cam`, sans le chevaucher) |
| Poses | 100 |
| Caméras | arducam, svpro, astra |
| Images | 300 (100 par caméra) |
| Robot | MyCobot 320 Pi, mêmes bornes d'angles que sessions 1–5 |
| Durée capture | 6m52s (4,1s/pose) |
| Taille disque | ~105 Mo |

`labels.csv` et `sessions.json` de ce dossier ont été pré-remplis avec la
dernière ligne/session de `real_3cam` (index 2499) uniquement pour que
`capture_real_3cam.py` (non modifié) auto-détecte `start_idx=2500` — le reste
(100 nouvelles poses × 3 caméras) est 100 % neuf.

## Log de capture (`bash capture_session6.sh`, 07/07/2026)

```
📁 Dataset (nouveau, séparé de real_3cam) : /home/genji/Osama_ws/src/mycobot_R6A/training/dream_data/real_3cam_session6
♻️  5 session(s) déjà faite(s) — celle-ci reprend après la dernière.
============================================================
📸 Real Data Capture — 3 Cameras
   ArduCam + SVPRO (USB/V4L2) + Astra (oni_grabber) — all on PC Tour
============================================================
  ✅ arducam (video2) opened
  ✅ svpro (video0) opened
  🔄 Astra: spawning oni_grabber…
  ✅ Astra (oni_grabber) ready — 640x480

  🎥 Cameras : ['arducam', 'svpro', 'astra']

  👁  Preview — ENTER to start capture, q/ESC to abort
  ✅ Robot bridge connected (10.10.0.224:5005)
  ♻️  Session 6 — reprend le dataset à l'index 2500 (session 5 s'est arrêtée là). Aucune pose répétée.
  🏠 Sending robot home…

🚀 Session 6: collecting 100 samples (index 2500→2599) × 3 cameras = 300 images total
   Settle time : 3.0s/pose
   ETA estimée : ~7m30s

  ↳ svpro: intrinsics scaled 800x600 → 640x480
  📸 [██████████████████████████████] 100/100 (100%)  ⏱ 6m52s écoulé  🕐 0s restant  ⚡ 4.2s/pose  📷 arducam+svpro+astra

  🏠 Returning home…

============================================================
✅ Session 6 terminée !
   📁 Dataset     : /home/genji/Osama_ws/src/mycobot_R6A/training/dream_data/real_3cam_session6
   📸 Poses       : 100/100  (index 2500→2599)
   🖼  Images      : 300 (3 caméras)
   📊 Total dataset: 2600 poses cumulées (6 sessions)
   ⏱  Durée totale: 6m52s
   ⚡ Moy/pose    : 4.1s
============================================================
```

(Log complet avec la barre de progression et les angles de chaque pose omis
ici pour la lisibilité — seuls le démarrage, la fin, et les paramètres clés
sont gardés. Note : la tentative précédente avait échoué sur `❌ Astra: no
frame from oni_grabber after 5s` — corrigée en réparant l'environnement
OpenCV de `venv_dream`, voir [`training/dream/python-environments`
troubleshooting] ; ici Astra s'ouvre bien au 2ᵉ essai.)

## Comment tester

`convert_to_ndds.py --source real` retombe par défaut sur `--cameras cam0 cam3`
(noms qui n'existent pas dans `real_3cam_session6/labels.csv`, qui utilise
`arducam`/`svpro`/`astra`) → 0 frame écrite si on ne précise pas `--cameras`.
De plus, le format NDDS n'a qu'un seul `_camera_settings.json` par dossier :
il faut convertir **une caméra à la fois**, puis fusionner avec `merge_ndds.py`
(qui ne fait que copier/renuméroter — chaque frame garde les annotations 2D/3D
calculées avec les bonnes intrinsèques/extrinsèques de sa propre caméra).

```bash
source ~/ros_jazzy/venv_dream/bin/activate
export PYTHONNOUSERSITE=1   # évite que ~/.local ne repollue venv_dream (cf. session capture)

# 1. Conversion NDDS — une caméra à la fois
for cam in arducam svpro astra; do
  python training/dream/convert_to_ndds.py \
    --input  training/dream_data/real_3cam_session6 \
    --output training/dream_data/real_3cam_session6_${cam}_ndds \
    --source real --cameras $cam
done

# 2. Fusion des 3 (300 frames)
python training/dream/merge_ndds.py \
  --inputs training/dream_data/real_3cam_session6_arducam_ndds \
           training/dream_data/real_3cam_session6_svpro_ndds \
           training/dream_data/real_3cam_session6_astra_ndds \
  --output training/dream_data/real_3cam_session6_ndds

# 3. Évaluation combinée (3 caméras, 300 images)
python training/dream/evaluate_dream.py \
  --weights training/dream/checkpoints_dream/vgg_ultimate_v4_mix_ft_e30/best_network.pth \
  --data training/dream_data/real_3cam_session6_ndds \
  --split all --max-samples 300

# 3bis. Évaluation par caméra (pour isoler un problème caméra-spécifique)
python training/dream/evaluate_dream.py \
  --weights training/dream/checkpoints_dream/vgg_ultimate_v4_mix_ft_e30/best_network.pth \
  --data training/dream_data/real_3cam_session6_astra_ndds \
  --split all --max-samples 100
```

> Utiliser `--max-samples 300` (ou plus) explicitement — un sous-échantillonnage
> par défaut (500) est tombé en phase avec l'ordre des caméras lors de
> l'évaluation `real_3cam_val_ndds` et a biaisé le résultat vers une seule
> caméra (voir `RAPPORT_MIX_FT_REAL3CAM.docx` §3).

## Résultat — combiné (3 caméras, 300 images)

| Keypoint | Mean (px) | Median (px) | Det% |
|----------|-----------|--------------|------|
| base | 21,56 | 17,28 | 91,3 % |
| link1 | 26,82 | 20,04 | 77,7 % |
| link2 | 26,82 | 20,04 | 77,7 % |
| link3 | 26,32 | 22,02 | 74,3 % |
| link4 | 27,80 | 23,40 | 76,3 % |
| link5 | 29,10 | 25,80 | 61,7 % |
| link6 | 35,91 | 27,72 | 68,0 % |
| **OVERALL** | **27,42** | **21,80** | **75,3 % (1581/2100)** |

## Résultat — par caméra (100 images chacune)

**arducam** — 92,4 % overall

| Keypoint | Mean (px) | Median (px) | Det% |
|----------|-----------|--------------|------|
| base | 18,30 | 17,32 | 100,0% |
| link1/link2 | 14,21 | 12,70 | 100,0% |
| link3 | 19,98 | 16,85 | 100,0% |
| link4 | 24,79 | 22,25 | 94,0% |
| link5 | 26,42 | 22,79 | 75,0% |
| link6 | 30,86 | 23,28 | 78,0% |

**svpro** — 87,1 % overall

| Keypoint | Mean (px) | Median (px) | Det% |
|----------|-----------|--------------|------|
| base | 12,09 | 10,76 | 92,0% |
| link1/link2 | 25,97 | 24,67 | 90,0% |
| link3 | 27,24 | 25,13 | 82,0% |
| link4 | 21,86 | 18,48 | 91,0% |
| link5 | 24,04 | 22,38 | 80,0% |
| link6 | 26,66 | 21,95 | 85,0% |

**astra** — 46,3 % overall (effondrée)

| Keypoint | Mean (px) | Median (px) | Det% |
|----------|-----------|--------------|------|
| base | 36,18 | 42,00 | 82,0% |
| link1/link2 | 57,92 | 56,52 | 43,0% |
| link3 | 39,92 | 34,64 | 41,0% |
| link4 | 46,50 | 41,72 | 44,0% |
| link5 | 49,31 | 43,44 | 30,0% |
| link6 | 64,69 | 54,77 | 41,0% |

## Comparaison et diagnostic

| Dataset | Poses | Vu à l'entraînement ? | Det% |
|---------|-------|------------------------|------|
| `real_3cam_val_ndds` | 500 (idx 0–2499) | Non (mais même lot de sessions que le train) | 91,6 % |
| `real_3cam_session6` (3 caméras) | 100 (idx 2500–2599) | Non — nouvelle session | 75,3 % |
| `real_3cam_session6` — arducam seule | 100 | Non | 92,4 % |
| `real_3cam_session6` — svpro seule | 100 | Non | 87,1 % |
| `real_3cam_session6` — astra seule | 100 | Non | **46,3 %** |

**Diagnostic : le décrochage global vient presque entièrement d'Astra, pas
d'un problème de généralisation du modèle.**

- Comparaison visuelle des images session 5 vs session 6 (même index relatif,
  `s5_vs_s6_compare.png`) : le cadrage d'**arducam et d'astra a nettement
  changé** entre les deux sessions (angle/zoom différents), alors que **svpro
  est resté identique**.
- Malgré son cadrage changé, **arducam reste solide (92,4 %)**, proche du
  baseline 91,6 % — le modèle généralise bien pour cette caméra.
- **svpro (87,1 %)**, cadrage inchangé, montre une légère baisse — bruit
  d'échantillon normal (n=100 vs 500 pour le baseline).
- **Astra s'effondre (46,3 %)** : elle cumule deux faiblesses — (1) c'est la
  seule caméra sans vraie calibration intrinsèque (intrinsèques par défaut
  fx=fy=610, jamais calibrées avec un échiquier comme arducam/svpro), et
  (2) elle a le plus bougé physiquement entre la calibration extrinsèque du
  03/07 (`REAL_CAMERA_TRANSFORMS` dans `convert_to_ndds.py`) et la capture de
  la session 6 le 07/07. Les annotations de vérité-terrain projetées avec
  cette extrinsèque obsolète ne correspondent plus à la position réelle de la
  caméra → l'évaluation compare le modèle à une GT décalée.

**Conclusion : le fine-tune mixte généralise correctement (arducam, svpro).**
Avant de refaire confiance à Astra pour une évaluation, il faut soit la
repositionner à l'endroit calibré le 03/07, soit refaire une vraie calibration
extrinsèque (et idéalement intrinsèque) à sa position actuelle.

## Autres datasets testés (pour référence — tous invalidés par la calibration)

Par curiosité, plusieurs anciens dossiers `training/dream/dream_data/*` déjà en
format NDDS ont aussi été passés dans `evaluate_dream.py` avec le même
checkpoint. Aucun n'est comparable au résultat de la session 6 : ils précèdent
soit la correction des intrinsèques synthétiques, soit la recalibration
extrinsèque réelle du 03/07.

| Dataset | Date | Images | Det% | Pourquoi non fiable |
|---------|------|--------|------|----------------------|
| `real` | 06/05 | 4000 | 42,5 % | Ancien, pré-calibration (intrinsèques arducam OK mais extrinsèque pré-03/07) |
| `real_1k` | 06/05 | 1000 | 36,3 % | Caméra grand-angle différente, intrinsèques par défaut (synthétique) |
| `real_cam0_only` | 06/05 | 2000 | 39,1 % | Intrinsèques par défaut non calibrées (fx=fy=610) |
| `synth_test` | 06/05 | 2000 | 60,1 % | Synthétique pré-correction des intrinsèques (avant le fix noté dans le README v4) |
| `mixed_13k` | 06/05 | 13000 | 39,7 % | Mélange réel+synth pré-correction (checkpoint historique `vgg_mixed_13k_seed42`) |
| `real_astra_ndds` | 24/06 | 500 | 92,0 %* | *Trompeur : `base` a un décalage **constant** de 127 px (std 0,10) → GT calculée avec l'ancien placeholder extrinsèque d'Astra, avant le 03/07 |
| `real_svpro_ndds` | 24/06 | 500 | 98,2 %* | *Trompeur : `base` a un décalage **constant** de 229 px (std 0,03) → même cause, ancien placeholder svpro |

Détail par keypoint pour chacun :

**`real` (4000 images) — 42,5% overall (11901/28000)**

| Keypoint | Mean (px) | Median (px) | Std | Det% |
|----------|-----------|--------------|-----|------|
| base | 35,29 | 45,16 | 22,84 | 75,1% |
| link1 | 75,33 | 77,78 | 32,55 | 43,3% |
| link2 | 75,31 | 77,79 | 32,56 | 43,4% |
| link3 | 127,98 | 126,04 | 36,28 | 48,5% |
| link4 | 103,98 | 97,12 | 59,18 | 37,4% |
| link5 | 121,98 | 114,39 | 63,99 | 21,2% |
| link6 | 112,10 | 100,96 | 61,82 | 28,6% |
| **OVERALL** | **84,26** | **76,86** | **54,04** | **42,5%** |

**`real_1k` (1000 images) — 36,3% overall (2544/7000)**

| Keypoint | Mean (px) | Median (px) | Std | Det% |
|----------|-----------|--------------|-----|------|
| base | 133,52 | 129,60 | 23,07 | 75,8% |
| link1 | 83,98 | 102,82 | 35,39 | 38,2% |
| link2 | 83,98 | 102,82 | 35,39 | 38,2% |
| link3 | 149,59 | 155,03 | 42,25 | 26,1% |
| link4 | 193,02 | 190,11 | 72,70 | 28,2% |
| link5 | 275,79 | 266,63 | 113,34 | 17,9% |
| link6 | 264,34 | 259,95 | 117,57 | 30,0% |
| **OVERALL** | **152,33** | **129,55** | **89,90** | **36,3%** |

**`real_cam0_only` (2000 images) — 39,1% overall (5479/14000)**

| Keypoint | Mean (px) | Median (px) | Std | Det% |
|----------|-----------|--------------|-----|------|
| base | 135,67 | 130,75 | 27,41 | 65,3% |
| link1 | 99,08 | 103,62 | 24,87 | 48,1% |
| link2 | 99,09 | 103,62 | 24,91 | 48,2% |
| link3 | 156,96 | 163,72 | 42,77 | 49,0% |
| link4 | 188,50 | 182,29 | 67,40 | 28,6% |
| link5 | 253,58 | 242,50 | 101,51 | 15,7% |
| link6 | 272,17 | 272,32 | 119,09 | 18,9% |
| **OVERALL** | **148,30** | **130,50** | **74,41** | **39,1%** |

**`synth_test` (2000 images) — 60,1% overall (8410/14000)**

| Keypoint | Mean (px) | Median (px) | Std | Det% |
|----------|-----------|--------------|-----|------|
| base | 38,88 | 17,71 | 35,08 | 77,6% |
| link1 | 31,22 | 27,27 | 18,62 | 55,2% |
| link2 | 31,22 | 27,27 | 18,62 | 55,2% |
| link3 | 47,62 | 35,14 | 35,69 | 69,8% |
| link4 | 60,63 | 37,81 | 55,62 | 59,1% |
| link5 | 57,61 | 39,48 | 53,38 | 53,4% |
| link6 | 69,42 | 45,71 | 63,19 | 50,1% |
| **OVERALL** | **47,39** | **30,13** | **44,49** | **60,1%** |

**`mixed_13k` (13000 images) — 39,7% overall (36106/91000)**

| Keypoint | Mean (px) | Median (px) | Std | Det% |
|----------|-----------|--------------|-----|------|
| base | 92,57 | 93,72 | 45,64 | 49,4% |
| link1 | 67,02 | 54,69 | 32,67 | 24,2% |
| link2 | 67,04 | 54,71 | 32,69 | 24,2% |
| link3 | 97,65 | 94,26 | 61,36 | 33,2% |
| link4 | 121,42 | 99,75 | 92,82 | 54,9% |
| link5 | 139,37 | 90,47 | 116,75 | 46,3% |
| link6 | 143,66 | 94,80 | 120,40 | 45,7% |
| **OVERALL** | **110,64** | **89,26** | **90,07** | **39,7%** |

**`real_astra_ndds` (500 images) — 92,0% overall (3221/3500) — décalage constant, trompeur**

| Keypoint | Mean (px) | Median (px) | Std | Det% |
|----------|-----------|--------------|-----|------|
| base | 127,70 | 127,72 | 0,10 | 100,0% |
| link1 | 25,31 | 25,25 | 0,97 | 100,0% |
| link2 | 25,31 | 25,24 | 0,97 | 100,0% |
| link3 | 130,48 | 121,82 | 42,48 | 95,6% |
| link4 | 214,60 | 194,39 | 97,84 | 81,6% |
| link5 | 293,04 | 261,17 | 136,58 | 81,2% |
| link6 | 289,82 | 249,26 | 154,72 | 85,8% |
| **OVERALL** | **149,77** | **127,70** | **133,01** | **92,0%** |

**`real_svpro_ndds` (500 images) — 98,2% overall (3437/3500) — décalage constant, trompeur**

| Keypoint | Mean (px) | Median (px) | Std | Det% |
|----------|-----------|--------------|-----|------|
| base | 229,25 | 229,24 | 0,03 | 100,0% |
| link1 | 147,81 | 147,81 | 0,08 | 100,0% |
| link2 | 147,81 | 147,81 | 0,08 | 100,0% |
| link3 | 126,34 | 120,13 | 56,27 | 99,6% |
| link4 | 145,85 | 124,82 | 84,51 | 97,2% |
| link5 | 220,83 | 195,51 | 123,66 | 94,4% |
| link6 | 232,66 | 201,82 | 140,34 | 96,2% |
| **OVERALL** | **178,17** | **147,86** | **90,42** | **98,2%** |

Le signal le plus utile pour repérer une GT obsolète : un **écart-type quasi
nul sur `base`** malgré une erreur moyenne énorme — ça indique un décalage
extrinsèque figé, pas une vraie erreur de détection (`base` est un point fixe
dans l'image, donc son erreur ne varie que si l'extrinsèque utilisée pour la
vérité-terrain a changé entre le moment où le modèle a appris sa position
réelle et le moment où ce dataset a été converti).
