# Calibration intrinsèque des caméras

Manuel d'utilisation de [`training/calibration/calibrate_camera.py`](../training/calibration/calibrate_camera.py) et des outils associés. Objectif : mesurer la matrice `K = [fx, fy, cx, cy]` et les coefficients de distorsion de chaque caméra du projet (Arducam Pi cam_0 et cam_3, Orbbec Astra RGB), pour que le pipeline DREAM puisse projeter des points 3D vers les images réelles **sans biais systématique**.

> Branche : [`feature/calibration-cam`](https://github.com/ABMI-software/mycobot_320pi_R6A/tree/feature/calibration-cam)
> Validé 28/04/2026 sur cam_0 (RMS 0.67 px) et cam_3 (RMS 0.68 px). Astra différée.

---

## Pourquoi calibrer

Le dataset DREAM `real_cam0` contient des fichiers `*.json` NDDS avec des `projected_location` calculées via FK + matrice caméra. Si la matrice utilisée diverge de la vraie caméra physique, les GT pixels sont systématiquement faux — et le réseau apprend à tracker des positions qui ne correspondent pas aux pixels réels.

**Constaté le 28/04/2026** :

| Param | `_camera_settings.json` du dataset | cam_0 mesuré | Écart |
|-------|-------------------------------------|--------------|-------|
| fx | 610 | 525.67 | **−13.8 %** |
| fy | 610 | 529.70 | **−13.2 %** |
| cx | 320 | 317.73 | −0.7 % |
| cy | 240 | 226.00 | **−5.8 %** |

Sur un point 3D à distance D, le pixel projeté est faux d'un facteur ~14 %, et **l'erreur croît avec la distance au centre image**. Cohérent avec les link4-6 (loin du centre image quand le bras est étendu) à 3-36 % de détection sur réel.

---

## Prérequis

### 1. Imprimer un board ChArUco

Le projet utilise par défaut un board **6 × 9 cases** avec :
- Square = **30 mm**
- ArUco marker = **24 mm** (ratio 0.8 standard)
- Dictionnaire **DICT_4X4_1000** (les markers font 4×4 bits)

Pour générer le PNG à imprimer :

```bash
python3 training/calibration/generate_board.py \
    --output /tmp/charuco_board.png \
    --squares-x 6 --squares-y 9 \
    --square-length-m 0.030 --marker-length-m 0.024 \
    --dpi 300
```

À 300 DPI le PNG fait ~178 × 267 mm — rentre sur A4 avec marges. Imprime en **100 %** (pas de "fit to page") et **vérifie au pied à coulisse** qu'un square fait bien 30 mm. Colle sur **foam-board** ou carton rigide — un board sur papier libre ondule et **ruine la calibration** (cy diverge à 42 px au lieu de 240).

### 2. Trois Python environnements

La calibration tourne dans le **système ROS2 / Python 3.12** (pas conda) :

```bash
conda deactivate     # impératif
which python3        # /usr/bin/python3, version 3.12
```

Aucune dépendance supplémentaire à installer — `cv2.aruco` et `numpy` sont déjà présents.

### 3. Caméras branchées

Ne **pas** brancher les deux Arducams en même temps si elles partagent le même contrôleur USB 2.0 — la bande passante combinée provoque des déconnexions au-dessus de quelques secondes de capture. Calibre **une caméra à la fois**.

```bash
ls /dev/video*               # voir quelles caméras sont visibles
v4l2-ctl --list-devices      # mapping device -> nom + port USB
```

L'Astra n'expose pas `/dev/videoN` (interface OpenNI propriétaire) — voir la section dédiée plus bas.

---

## Calibrer une Arducam (cam_0 ou cam_3)

### Commande

```bash
python3 training/calibration/calibrate_camera.py \
    --camera 0 --name cam_0 \
    --squares-x 6 --squares-y 9 \
    --square-length-m 0.030 --marker-length-m 0.024 \
    --target-samples 25
```

Substitue `--camera 2 --name cam_3` pour la 2ᵉ caméra. Le **preset arducam** s'applique automatiquement quand `--name` commence par `cam_` : exposure 80, gain 0, focus 50, WB 4600 K, autofocus off.

### Workflow opérateur

1. Une fenêtre `Calibrate cam_0` s'ouvre.
2. Une grille de **4 × 3 cellules** divise l'image en zones de couverture.
3. **Bouge le board lentement** devant la caméra. À chaque frame :
   - Détecter ≥ 6 marqueurs ArUco
   - Interpoler ≥ 12 coins ChArUco
   - Sharpness Laplacian ≥ 80
   - Cellule différente des dernières captures (max 6 par cellule, **abaisse à 4 avec `--max-per-cell 4` pour forcer plus de diversité**)
4. Quand toutes les conditions passent : capture automatique. Le terminal affiche `[auto] view N/25 cell=(c,r) sharp=X`.
5. À 25 vues, le script déclenche `_solve_and_save()` automatiquement, rejette les outliers (per-view error > 1.2 px) et sauvegarde.
6. **Pas besoin d'appuyer C** — l'auto-save remplace ce flux. Si la fenêtre est fermée prématurément avec ≥ 12 vues, save-on-quit déclenche aussi.

### Couvrir la grille — important

Le solveur a besoin de vues **réparties sur tout le frame**, en particulier les coins et les bords. Une couverture concentrée au centre fait diverger le principal point (`cy` à 42 au lieu de 240, observé sur un premier essai cam_3). Vise au moins **2 vues par cellule extérieure** (haut, bas, gauche, droite). L'overlay affiche la cellule courante en bas du board.

### Outputs

```
training/calibration/cam_0.npz             # mtx, dist, rvecs, tvecs
training/calibration/cam_0.meta.json       # fx/fy/cx/cy + recipe + RMS
training/calibration/cam_0.snapshot.png    # dernière vue acceptée (référence visuelle)
```

Le `.meta.json` est le fichier à consulter — il contient les valeurs en clair (pas en binaire) :

```json
{
  "name": "cam_0",
  "results": {
    "rms_reprojection_error_px": 0.6738,
    "accepted_views": 18,
    "fx": 525.67, "fy": 529.70,
    "cx": 317.73, "cy": 226.00,
    "dist_coeffs": [33.6, 129.6, ..., -1937, ...]
  },
  "settings_applied": { "auto_exposure": "off", "exposure": 80.0, ... }
}
```

### Cibles RMS

- **< 1.0 px** → calibration excellente, prête pour PnP / DREAM
- **1.0 – 2.0 px** → acceptable mais regarde le `cy` : s'il est très loin du centre image / 2, recommencer avec une meilleure couverture
- **> 2.0 px** → recapture, le board ondule ou la couverture est mauvaise

### Si une calibration échoue

Le premier essai cam_3 a produit `cy = 42` (alors que l'image fait 480 px de haut, donc cy devrait être proche de 240). Cause = couverture insuffisante du haut de l'image. Le script archive ce premier essai en `cam_3.bad.npz` quand on le renomme manuellement avant la 2ᵉ tentative. Pratique : permet de comparer.

---

## Calibrer l'Astra RGB

⚠️ **À ce jour (28/04/2026), la calibration Astra est différée.** Voir [SESSION_RESUME](../SESSION_RESUME.md) pour les détails. Les sections ci-dessous documentent le chemin théorique pour la prochaine session.

### Commande

```bash
python3 training/calibration/calibrate_camera.py \
    --source astra --name astra_rgb \
    --squares-x 6 --squares-y 9 \
    --square-length-m 0.030 --marker-length-m 0.024 \
    --target-samples 30 --max-per-cell 4 \
    --min-markers 4 --min-charuco 6 --clahe
```

`--source astra` bascule de `cv2.VideoCapture(N, V4L2)` au wrapper `OrbbecCapture` du projet ([`teleop/orbbec_capture.py`](../teleop/orbbec_capture.py)) qui auto-spawn le binaire `oni_grabber` (OpenNI2 + shared-memory `/dev/shm/oni_color.rgb`).

### Particularités vs Arducam

- **Capteur RGB plus bruité** — le décodage des markers 4×4 (16 bits chacun) échoue souvent. Seuils `min_markers=4` / `min_charuco=6` (au lieu de 6/12) tolèrent les détections partielles.
- **`--clahe`** active une égalisation de contraste local (CLAHE 3.0, tile 8×8) avant détection. Aide les capteurs à faible SNR.
- **FOV beaucoup plus large** (~60° horizontal) → le board apparaît plus petit à distance équivalente. Il faut le **rapprocher fortement** (~25-40 cm de l'objectif, qu'il occupe ~50 % de la largeur de l'image).
- **USB 2.0 saturé à 1280×720@30 fps RGB888** (663 Mbps > 480 dispo). À 1280×720 il faut passer en GRAY8 (220 Mbps). Voir le custom HD grabber dans `/tmp/oni_grabber_hd.cpp` pour la prochaine session.

### Setup physique recommandé pour la prochaine tentative

L'expérience du 28/04 a montré que la session interactive (utilisateur qui revient au clavier régulièrement, board mobile en main) ne fonctionne pas pour l'Astra. Pour la prochaine fois :

1. **Board fixé au mur** ou sur un panneau vertical, bien éclairé, sans contre-jour
2. **Astra sur un trépied ou un support stable**, pointée vers le board
3. **Caméra qui bouge, board statique** (inverse du flux Arducam)
4. Distance Astra ↔ board : ~40-60 cm de manière à remplir 50 % de l'image
5. Capture en continu pendant 30-60 s en variant l'angle et la distance — laisser l'auto-capture faire son travail sans intervention au clavier

---

## Probes diagnostiques

Si la calibration semble bloquée (0 vues acceptées en >30 s), utilise les probes pour isoler le problème.

### Single-frame ChArUco

```bash
python3 training/calibration/probe_charuco.py --camera 0 \
    --squares-x 6 --squares-y 9 \
    --output /tmp/probe_cam_0.png
```

Capture **une frame** après 10 frames de warmup, tente la détection, sauvegarde une image annotée avec `markers=N charuco=M sharp=X`. Permet de voir d'un coup d'œil si :
- Le board est dans le champ
- Combien de markers sont décodés
- Si la GT (cercles vides) est correctement placée sur le board

### Astra 4-mode

```bash
python3 training/calibration/probe_astra.py
```

15 secondes de capture continue, garde la frame avec le **plus de markers détectés**, puis tente la détection en 4 modes :
1. `raw_bgr` — pas de prétraitement
2. `raw_bgr+clahe` — CLAHE avant détection
3. `swap_rb` — canaux R↔B inversés (au cas où oni_grabber renvoie RGB au lieu de BGR)
4. `swap_rb+clahe`

Sauvegarde 4 PNG annotées dans `/tmp/probe_astra_*.png`. Compare visuellement quel mode décode le plus.

### Régressions OpenCV 4.6

Le script `calibrate_camera.py` contourne deux régressions connues d'OpenCV 4.6 sur cette machine :

- `cv2.aruco.DetectorParameters()` retourne un objet dont l'access à `cornerRefinementMethod` provoque un **segfault**. Fix : utiliser la fabrique legacy `DetectorParameters_create()`.
- `cv2.aruco.CharucoBoard((sx, sy), …)` retourne un objet dont `interpolateCornersCharuco` provoque un **segfault**. Fix : utiliser la fabrique legacy `CharucoBoard_create(sx, sy, …)`.

Ces deux fixes sont déjà dans `_create_detector_parameters()` et `_build_charuco_board()`. Si tu vois un `Erreur de segmentation (core dumped)`, c'est probablement une régression similaire dans une autre API — utilise les probes pour isoler.

---

## Utiliser les K mesurés dans le pipeline DREAM

Une fois `cam_0.npz` et `cam_3.npz` produits, l'étape suivante est de **régénérer les fichiers GT du dataset** `/tmp/dream_data/real_cam0/` avec ces matrices au lieu du `fx=fy=610` codé en dur.

Pseudo-code (à intégrer dans une variante de [`training/dream/convert_to_ndds.py`](../training/dream/convert_to_ndds.py)) :

```python
import numpy as np

calib = np.load("training/calibration/cam_0.npz")
mtx = calib["mtx"]            # (3, 3)
dist = calib["dist"].ravel()  # (14,) rational model

# Pour chaque frame du dataset :
for json_path in real_dataset_paths:
    frame_data = json.load(open(json_path))
    angles = frame_data["angles"]                  # joints en deg
    keypoints_3d = mycobot_fk_keypoints(angles)    # (7, 3) en m
    keypoints_3d_cam = world_to_camera @ keypoints_3d
    pixels, _ = cv2.projectPoints(keypoints_3d_cam, np.zeros(3), np.zeros(3), mtx, dist)
    for i, kp in enumerate(frame_data["objects"][0]["keypoints"]):
        kp["projected_location"] = pixels[i].ravel().tolist()
    json.dump(frame_data, open(json_path, "w"))
```

Le `_camera_settings.json` doit aussi être mis à jour pour exposer les vrais `fx, fy, cx, cy` (DREAM relit ces valeurs au moment de l'eval).

> Voir le plan détaillé dans [`SESSION_RESUME.md`](../SESSION_RESUME.md) — points 2 et 3 pour la prochaine session.

---

## Référence rapide

| Action | Commande |
|--------|----------|
| Générer un board imprimable | `python3 training/calibration/generate_board.py --output /tmp/board.png` |
| Calibrer une Arducam | `python3 training/calibration/calibrate_camera.py --camera N --name cam_X` |
| Calibrer l'Astra RGB | `python3 training/calibration/calibrate_camera.py --source astra --name astra_rgb --clahe --min-markers 4 --min-charuco 6` |
| Probe single-frame | `python3 training/calibration/probe_charuco.py --camera N` |
| Probe Astra 15 s | `python3 training/calibration/probe_astra.py` |

| Fichier | Contenu |
|---------|---------|
| `<name>.npz` | `mtx`, `dist`, `rvecs`, `tvecs` (NumPy binary) |
| `<name>.meta.json` | `fx, fy, cx, cy`, `dist_coeffs`, RMS, recipe complète |
| `<name>.snapshot.png` | Dernière frame acceptée (référence visuelle) |

| Cible | Valeur |
|-------|--------|
| RMS reprojection error | < 1.0 px (excellent) |
| Vues finales (post-rejet) | ≥ 15 |
| `cy` final | proche de `image_height / 2` (240 pour 480p, 360 pour 720p) |
| `fx ≈ fy` | écart < 5 % entre les deux |

---

*Mise à jour : 28 avril 2026 (soir). Branche [`feature/calibration-cam`](https://github.com/ABMI-software/mycobot_320pi_R6A/tree/feature/calibration-cam).*
