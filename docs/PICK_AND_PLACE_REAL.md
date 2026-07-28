# Pick-and-Place réel — MyCobot 320 Pi + gripper Pro

Pipeline pick-and-place **sur le vrai robot**, sans IA (positions apprises), avec
le gripper Pro adaptatif piloté en TCP, une interface graphique d'apprentissage/
exécution, et un outil d'enregistrement démo 3-vues.

Tout passe par le **bridge TCP** sur le Pi (`10.10.0.221:5005`) — aucun ROS requis
côté PC. Le firmware du robot fait l'IK (`send_coords`/`send_angles`), rien n'est
calculé côté PC.

---

## Architecture

```
PC Tour ──TCP:5005──▶ gripper_bridge.py (Pi) ──série /dev/ttyAMA0──▶ bras + gripper
   │
   ├─ pick_and_place_gui.py     (interface : apprendre + exécuter)
   ├─ pick_and_place_real.py    (moteur pick-and-place, aussi utilisable en CLI)
   └─ record_demo.sh            (démo mosaïque Arducam + SVPRO + écran)
```

---

## Le gripper Pro adaptatif

- Classe pymycobot : **`MyCobot320`** @ **1000000 baud** (PAS `MyCobot` @ 115200).
- API : `set_pro_gripper_*(gripper_id, value)`, **gripper_id = 14**.
- Angle 0–100 (0 = fermé, 100 = ouvert). Couple 100–300. Vitesse 1–100.
- Contrainte : **≥ 1.5 s entre deux commandes gripper** (respecté par un gap de 1.6 s).
- Le bras et le gripper **partagent le port série** : il faut laisser le robot
  s'immobiliser (~1.2 s) avant d'envoyer une commande gripper, sinon elle est perdue
  (bug observé en mode boucle continue, corrigé).

### Détection de préhension (statut) — pas de capteur de force

Le protocole `ProGripper` de pymycobot n'expose **aucun registre de courant/force**
(vérifié par scan des adresses 0–50 : aucune ne change entre pince ouverte et
serrage). Le LCD du gripper affiche `Current (mA)` mais le firmware le lit en interne
et ne le relaie pas sur le bus série.

**Seule information d'effort disponible = le statut** (`get_pro_gripper_status`) :

| statut | signification |
|--------|---------------|
| 0 | en mouvement |
| 1 | arrêté, rien saisi |
| **2** | **arrêté, objet saisi** ← indicateur de prise |
| 3 | objet tombé après saisie |

Le statut passe à **2** uniquement si la pince **bute sur l'objet avant d'atteindre
sa consigne** (fermeture plus poussée que ce que l'objet permet). Fermer à un angle
exact que la pince atteint → statut 1.

---

## Scripts

### `scripts/gripper_bridge.py` (sur le Pi)

Copie de `bridge_pi_simple.py` + actions gripper Pro. Actions JSON ajoutées :
`pro_gripper_open/close/angle`, `pro_gripper_calib`, `get_pro_gripper_angle/status/
torque/speed/position`, `set_pro_gripper_speed/torque`, `get_pro_gripper` (registre
générique). Lancer sur le Pi : `python3 ~/gripper_bridge.py`.

### `scripts/gripper_gui.py` (PC)

GUI PyQt5 de contrôle du gripper seul : consigne d'ouverture (boîte + OK),
ouvrir/fermer, réglage vitesse/couple, lecture live (position/statut/couple/vitesse),
sonde de registres. `python3 scripts/gripper_gui.py --host 10.10.0.221`.

### `scripts/pick_and_place_real.py` (PC) — moteur CLI

Apprentissage **en angles articulaires** (`get_angles`/`send_angles`) : reproduction
exacte, sans ambiguïté d'IK (les coords cartésiennes en bras-relâché font sauter les
angles d'Euler → non fiable). 4 poses : `pick_approach`, `pick`, `place_approach`,
`place`, sauvées dans `scripts/pick_place_positions.json`.

```bash
# apprentissage (bras déplacé à la main)
python3 scripts/pick_and_place_real.py --gripper-open
python3 scripts/pick_and_place_real.py --release        # servos mous — TIENS le bras
python3 scripts/pick_and_place_real.py --capture pick_approach
python3 scripts/pick_and_place_real.py --capture pick
python3 scripts/pick_and_place_real.py --capture place_approach
python3 scripts/pick_and_place_real.py --capture place
python3 scripts/pick_and_place_real.py --power-on

# trouver le bon serrage sur l'objet
python3 scripts/pick_and_place_real.py --goto pick        # va sur l'objet
python3 scripts/pick_and_place_real.py --grip 15          # ferme à 15, montre le statut
python3 scripts/pick_and_place_real.py --set-grasp 15     # enregistre l'angle

# rejouer
python3 scripts/pick_and_place_real.py --run --speed 20         # une fois
python3 scripts/pick_and_place_real.py --run --step --speed 15  # pas-à-pas
```

Séquence RUN : ouvre → `pick_approach` → `pick` → ferme (**vérifie statut 2**) →
soulève → `place_approach` → `place` → ouvre → retrait → home.

### `scripts/pick_and_place_gui.py` (PC) — interface

Même logique, en boutons (PyQt5). 3 sections :
1. **Apprentissage** : ouvrir pince · relâcher servos · capturer les 4 poses · re-tendre.
2. **Serrage** : aller pick_approach/pick · tester un angle (voit le statut) · enregistrer.
3. **Exécution** : vitesse · `pas-à-pas` · `boucle continue` · RUN · Continuer · Stop boucle.

Les poses et l'angle de serrage sont **rechargés automatiquement** au démarrage.
Le mode **boucle continue** enchaîne les cycles (⚠️ il faut remettre l'objet à la
position `pick` entre chaque cycle, ou déposer au même endroit).

```bash
python3 scripts/pick_and_place_gui.py --host 10.10.0.221
```

### `scripts/record_demo.sh` (PC) — démo vidéo 3 vues

Enregistre une **mosaïque** (Arducam + SVPRO en haut, écran/GUI en bas) dans un seul
fichier, avec une **fenêtre LIVE** (les 2 caméras, via ffplay). Fichier en **`.mkv`**
(reste lisible même si coupé brutalement).

```bash
bash scripts/record_demo.sh                       # arducam=/dev/video2 svpro=/dev/video0
bash scripts/record_demo.sh /dev/video2 /dev/video0
```
Stop : **Ctrl+C dans le terminal**. Sortie : `~/Videos/pick_place_mosaic_<date>.mkv`.
Si une caméra reste bloquée : `pkill -9 ffmpeg ffplay`.

---

## Vision YOLO (préparé, pas encore branché)

Pour remplacer plus tard la pose `pick` apprise par une position détectée :

- `scripts/yolo_object_detect.py` — YOLOv8s (COCO) sur l'Arducam → coordonnées base.
- `mycobot_gateway/vision/object_localizer.py` — déprojection pixel→base (intrinsèque
  `cam_3` + extrinsèque caméra→base + plan table).
- Env : le **`.venv` d'Osama_ws** (torch+CUDA+ultralytics, rclpy présent).

⚠️ **Bloquant avant utilisation** : l'extrinsèque `arducam_extrinsic_dream_v4.yaml`
est **périmé** (la caméra a bougé) — les coordonnées objet tombent hors de portée
(~880 mm au lieu de ≤ 350 mm), et **aucun `table_z` plausible ne corrige ça**. Il faut
**recalibrer l'extrinsèque caméra→base** (DREAM self-calibrate ou ArUco — un marqueur
est présent sur la table) avant de laisser le bras viser une cible détectée.

---

## Lancer la démo complète

Prérequis : bridge lancé sur le Pi (`python3 ~/gripper_bridge.py`).

```bash
# Terminal 1 — interface
python3 scripts/pick_and_place_gui.py --host 10.10.0.221
# Terminal 2 — enregistrement
bash scripts/record_demo.sh
```
Puis dans le GUI : RUN (ou boucle) → Ctrl+C dans le terminal 2 quand fini.

---

## Sécurité

- `--release` rend le bras **mou** : le tenir avant, `--power-on` pour le re-tendre.
- Premier RUN à **vitesse basse** (`--speed 15-20`), zone dégagée, main sur l'arrêt.
- Le RUN **vérifie le statut** après fermeture : si ≠ 2, il s'arrête et remonte à vide.
- Objet rond (balle) = glisse facilement ; préférer un objet à faces planes
  (rectangle) pour une prise fiable.
