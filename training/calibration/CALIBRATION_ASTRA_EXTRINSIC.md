# Calibration extrinsèque de l'astra (RGB-D) — procédure

> Objectif : obtenir `T_base_camera` de l'astra fixe (eye-to-hand) par recalage
> 3D, pour la courbe d'écart par joint. Sans ChArUco : intrinsèques d'usine
> (FOV) + profondeur alignée couleur.
>
> **Statut : documenté, pas encore exécuté.** L'extrinsèque n'est nécessaire
> qu'à l'étape « courbe » (`plot_angle_error_curve.py`). La **capture**
> (`capture_astra_rgbd.py`) n'en a **pas** besoin — on peut la faire avant.

---

## Vue d'ensemble

```
oni_grabber_rgbd (C, /dev/shm)  ─┐
                                 ├─▶  calibrate_astra_extrinsic_shm.py  ─▶  astra_extrinsic.yaml
workspace_markers.yaml (base)  ──┘                                          + cam_astra.npz
```

Le grabber écrit couleur + depth **aligné couleur** + le champ de vision (→
intrinsèques). La calibration détecte les 4 marqueurs sol, les place en 3D via
la profondeur, et les aligne (Kabsch) sur leurs positions connues en repère base.

---

## Prérequis

- Astra montée **fixe** à sa position eye-to-hand (ne plus la bouger après).
- Les 4 marqueurs ArUco `DICT_4X4_1000`, 25 mm, posés à plat aux positions
  mesurées (`workspace_markers.yaml`), **tous visibles** dans l'image astra.

### Disposition des marqueurs (repère base : X=avant, Y=gauche, Z=0=table)

```
                 Y (gauche)
                    ▲
    23 (0.065,+0.241)     26 (0.365,+0.241)
         •───────────────────•
         │                   │
   ──────┼──── robot ────────┼────►  X (avant)
         │                   │
         •───────────────────•
    19 (0.065,-0.241)     25 (0.365,-0.241)
```

---

## Étape A — compiler le grabber (une fois)

```bash
SDK=~/Downloads/Orbbec_OpenNI_v2.3.0.86-beta6_linux_release/OpenNI_2.3.0.86_202210111154_4c8f5aa4_beta6_linux_x64/OpenNI_2.3.0.86_202210111154_4c8f5aa4_beta6_linux/sdk
cd ~/Osama_ws/src/mycobot_R6A/training/calibration
g++ oni_grabber_rgbd.cpp -o oni_grabber_rgbd -I$SDK/Include -L$SDK/libs -lOpenNI2 -Wl,-rpath,$SDK/libs
./oni_grabber_rgbd            # laisser tourner (terminal A)
```

`Warning: USB events thread - failed to set priority` est **bénin**.

Vérifier le flux :
```bash
ls -la /dev/shm/oni_*        # oni_color.rgb (921600), oni_depth.raw (614400), oni_info.txt, oni_tick.txt
cat /dev/shm/oni_info.txt    # doit contenir CHFOV=.../CVFOV=...  (= nouveau grabber)
```
`oni_depth.raw` à la même résolution que la couleur (640×480) ⇒ registration D2C OK.

---

## Étape B — cadrer les 4 marqueurs

```bash
source ~/ros_jazzy/venv_dream/bin/activate
cd ~/Osama_ws/src/mycobot_R6A/training/calibration
python3 check_astra_markers.py       # boucle ; ouvre astra_markers_preview.png
```

Bouger l'astra / les marqueurs jusqu'à `✅ LES 4 VUS`, puis Ctrl+C. Les 4
doivent être à plat, non masqués (bras non interposé), assez gros (25 mm à
~70 cm passent bien).

---

## Étape C — lancer la calibration

```bash
python3 calibrate_astra_extrinsic_shm.py --num-frames 30
```

Sorties : `astra_extrinsic.yaml` (T_cam_world / T_world_cam) + `cam_astra.npz`
(intrinsèques d'usine).

### Interpréter le résidu moyen

| Résidu | Verdict |
|--------|---------|
| **< 5 mm** | bon — on peut tracer la courbe |
| 5–10 mm | acceptable ; **surélever un marqueur ~10 cm** (casse la coplanarité) et relancer |
| > 10 mm | debug : depth mal aligné, un marqueur mal vu, ou positions `workspace_markers.yaml` fausses |

**Caveat coplanarité** : les 4 marqueurs sont au sol (z=0) → fit un peu moins
contraint hors-plan (~6 mm avec bruit depth 2 mm). Surélever un marqueur resserre.

---

## Fichiers

| Fichier | Rôle |
|---------|------|
| `oni_grabber_rgbd.cpp` | grabber OpenNI couleur + depth aligné + FOV |
| `check_astra_markers.py` | aide au cadrage (compte les marqueurs vus) |
| `calibrate_astra_extrinsic_shm.py` | calibration 3D Kabsch → yaml + npz |
| `workspace_markers.yaml` | positions mesurées des 4 marqueurs (repère base) |
| `astra_extrinsic.yaml` | **sortie** : T_base_camera |
| `cam_astra.npz` | **sortie** : intrinsèques d'usine (mtx, dist) |

Consommé ensuite par `training/dream/plot_angle_error_curve.py` (la courbe).
