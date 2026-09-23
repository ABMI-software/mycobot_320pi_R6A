# Pick-and-Place réel — MyCobot 320 Pi + gripper Pro

Pipeline pick-and-place **sur le vrai robot**, sans IA (positions apprises), avec
le gripper Pro adaptatif piloté en TCP, une interface graphique d'apprentissage/
exécution, et un outil d'enregistrement démo 3-vues.

Tout passe par le **bridge TCP** sur le Pi (`10.10.0.224:5005`) — aucun ROS requis
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
sonde de registres. `python3 scripts/gripper_gui.py --host 10.10.0.224`.

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
python3 scripts/pick_and_place_gui.py --host 10.10.0.224
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

## Saisie guidée par deux caméras — méthodologie du 14/09/2026

Validée sur le robot réel : **deux saisies de balle réussies** (statut 2, angle de
calage 51 et 53), la seconde soulevée et tenue. Menée hors dashboard, étape par
étape, en contrôlant les deux caméras avant chaque fermeture.

### Pourquoi le bras ratait après chaque calibration extrinsèque

Symptôme : juste après une calibration, la pince descend **5 à 9 cm à côté** de la
balle, toujours du même côté, alors que le modèle se dit à moins de 3 mm de la
cible. La correction Shepard/IDW ne pouvait pas rattraper ça : l'écart dépasse sa
borne de 25 mm, et ce n'est pas une déformation locale.

Cause : **`planche_actuelle.yaml` est décalé de −7,46° et (+64,0 ; +17,6) mm**
par rapport à `workspace_markers.yaml`. Il a été relevé le 11/09 *à travers*
l'extrinsèque de l'époque, alors que c'était l'ensemble **table + planche + robot**
qui avait bougé par rapport à la caméra. Dans le repère du robot, les marqueurs
sont toujours aux positions de `workspace_markers.yaml`. Chaque recalibration
contre `planche_actuelle.yaml` recopie donc l'ancien lien caméra ↔ robot.

Preuves : l'hypothèse explique les cinq observations indépendantes de la séance
(premier essai du dashboard, survol, marqueurs 19 et 23, prise placée à la main),
puis la balle a été saisie **du premier coup** en visant sa position convertie
dans le repère robot.

> **Règle** : ne jamais relever les positions des marqueurs à travers une
> extrinsèque. Les vérifier **depuis le robot** (pointe placée sur 19 et 23,
> angles relus) avant de s'en servir comme référence de calibration.

### Rôle de chaque caméra

| | Arducam (vue de dessus) | SVPRO (vue de côté) |
|---|---|---|
| Position | ~1,06 m au-dessus de la planche, presque verticale | sur le côté, vue plongeante |
| Ce qu'elle mesure bien | **XY de l'objet** sur la planche | **hauteur des doigts** par rapport à l'objet, **mors de part et d'autre** |
| Ce qu'elle mesure mal | la hauteur ; objet masqué par la pince en bas de descente | les coordonnées : extrinsèque du 24/08 périmée (~13 cm) |
| Utilisation | pixel → repère robot, via extrinsèque | **dans l'image seulement**, sans coordonnées |
| Extrinsèque | juste dans le plan Z = 0 uniquement | à refaire avant toute triangulation |

### Déroulé d'une saisie

1. **Dégager** le bras en pose d'observation (la vue de dessus doit voir l'objet).
2. **Localiser à l'arducam** : 8 vues, dispersion ≤ 3 mm, sinon on ne vise pas.
3. **Passer dans le repère robot** : `XY_robot = Rᵀ · (XY_vision − t)`, avec
   (R, t) la transformation rigide `workspace_markers` → `planche_actuelle`.
   Correctif **provisoire**, à supprimer une fois l'extrinsèque refaite contre
   les bonnes positions.
4. **Choisir le roulis** de la pince. Il doit être atteignable du survol jusqu'à
   22 mm, sur une descente verticale continue, avec **J4 ≤ 135°**, la vraie butée
   du robot (le modèle `diff_ik` croit pouvoir aller à 145°). De préférence, les
   mors s'ouvrent **en travers** de la ligne de visée SVPRO.
5. **Survoler** à 127 mm, puis **descendre verticalement** à la hauteur de prise.
6. **Contrôler avant de fermer** :
   - relire les angles : la hauteur atteinte est-elle la hauteur visée ? Le pont
     répond `OK` même quand une articulation est en butée ;
   - arducam : l'objet est-il entre les deux mors, vu de dessus ?
   - SVPRO : les doigts sont-ils à la hauteur de l'objet, un mors de chaque côté ?
7. **Fermer**, puis confirmer par **statut 2 ET angle de calage ≠ pince vide**
   (vide = 20 ; balle = 51–53).
8. **Soulever par paliers** en relisant le statut à chaque palier.

### Résultats de la séance

| Essai | Cible (repère robot) | Roulis | Résultat |
|---|---|---|---|
| Balle 1 | (178,2 ; 48,6), 185 mm | 0° | statut 2, angle 51 — tenue |
| Balle 2, 1er essai | (307,0 ; 71,1), 315 mm | +60° | **fermée à vide** : J4 en butée à 135°, le bras s'est arrêté à 112 mm |
| Balle 2, 2e essai | (307,0 ; 71,1), 315 mm | +30° | statut 2, angle 53 — **soulevée et tenue** |
| Balle posée à 378 mm | (361,8 ; 109,3) | — | non tentée : pince verticale impossible, seul l'outil couché à −30° atteint |

### Comment ça aide à détecter l'objet

- **L'arducam trouve l'objet, la SVPRO vérifie la prise.** Aucune des deux ne
  suffit seule : la vue de dessus ne voit pas la hauteur, et la vue de côté n'a
  pas de coordonnées fiables.
- **Contrôle « objet entre les mors » dans l'image SVPRO.** Les mors sont les
  deux plus grandes zones sombres autour de l'objet. Une ouverture morphologique
  7 × 7 supprime le câble noir (~3 px de large) et garde les mors (≥ 8 px). La
  position de l'objet le long du segment mors-mors vaut `t = 0,5` au milieu ;
  on déclare « entre » pour `0,2 ≤ t ≤ 0,8`. Mesuré : `t = 0,48` sur une balle
  correctement encadrée.
- **Limite du contrôle SVPRO** : fiable seulement quand les mors s'ouvrent **en
  travers** de la vue. Quand ils s'ouvrent dans l'axe de la caméra (roulis 0° ou
  +30° sur ce banc), un mors cache l'autre et le détecteur répond « pas entre » à
  tort. Il faut alors trancher avec la vue de dessus.
- **Il empêche la fermeture à vide.** Chaque fermeture à vide pousse l'objet :
  un essai raté fausse le suivant. D'où l'arrêt au premier échec, pour regarder,
  plutôt qu'une nouvelle tentative à l'aveugle.

⚠️ Les scripts de cette séance (`saisie_seule.py`, `saisie_roulis.py`) sont
restés dans le répertoire de session. **Ils sont à remonter dans `scripts/`**
pour que la méthode soit rejouable.

---

## Calibration contre le robot — 15/09/2026

**Résultat : balle saisie du premier coup avec l'arducam recalibrée**, sans aucun
correctif de repère. Balle vue en (279,8 ; 14,9) mm ; recalage 2,0 mm, descente
0,5 mm en XY, fermeture statut 2 et angle 53 (tenue). Un seul essai : il prouve
que la cause est trouvée, pas encore la répétabilité.

### Le problème : une calibration qui se vérifiait elle-même

La chaîne d'une saisie est `pixel de l'objet → extrinsèque → mm robot → IK`.
Le robot était déjà précis à quelques mm. L'étape fausse était l'extrinsèque :
elle était calibrée contre `planche_actuelle.yaml`, dont les positions avaient
été relevées **à travers une extrinsèque déjà fausse**. Le contrôle comparait
donc la caméra à elle-même et affichait 0,3 mm, alors que la pince tombait
5 à 10 cm à côté de la balle.

Mesuré le 15/09 : position estimée par chaque caméra contre position mesurée au
robot.

| Marqueur | Réel, mesuré au robot | Arducam avant | SVPRO |
|---|---|---|---|
| 19 | (87,1 ; 212,4) | 99,5 mm d'écart (+99,2 ; −6,4) | 62,4 mm (−61,8 ; +8,5) |
| 23 | (110,9 ; −163,0) | 40,2 mm (+38,4 ; −12,1) | 97,0 mm (−96,2 ; −13,1) |

Les deux caméras se trompaient **en sens opposés** : l'arducam trop loin du
robot, la SVPRO trop près. Le dashboard « apprend » l'écart entre les deux, ce
qui masquait le problème.

`workspace_markers.yaml` n'est pas juste non plus : au robot, le 19 et le 23 sont
à 13 et 16 mm de ses valeurs. La règle du 14/09 « vrais marqueurs =
`workspace_markers.yaml` » est remplacée par : **vrais marqueurs = mesurés au
robot**.

### La méthode : le robot sert de règle

1. **Mesurer au robot le 19 et le 23**, les seuls à portée (le bras atteint
   ~390 mm, le 25 et le 26 sont à ~580 mm). La pince est placée à la main, le
   marqueur au milieu des doigts, et on lit seulement les codeurs :
   `fsm.pointe(pont.angles())`, médiane de 5 lectures, sans ordre moteur.
   **Ne jamais relâcher les moteurs** pour cela.
2. **Déduire le 25 et le 26** : forme de la planche vue par l'arducam (distances
   entre centres justes à 0,04 %), posée rigidement sur les deux points du robot.
   Résidu 3,3 mm, écart de 19-23 entre robot et image : 376 contre 383 mm.
3. **Écrire cette référence** dans `training/calibration/planche_actuelle.yaml`.
4. **Recalibrer** : `.venv/bin/python scripts/calibration_extrinseque_auto.py`,
   bras en pose de dégagement, 4/4 marqueurs visibles. Pas de `--force` : la
   validation leave-one-out doit passer seule.
5. **Contrôler** en lecture seule avec `--controle`, puis **une saisie**, en
   s'arrêtant avant de fermer pour regarder.

Ce qui n'a pas marché, et pourquoi :

| Référence essayée | Pire leave-one-out arducam | Verdict |
|---|---|---|
| `workspace_markers.yaml` (ruban 18/08) | 10,9 mm sur le 25 | refusée |
| forme ruban posée sur 19/23 robot | 9,1 mm sur le 25 | refusée |
| 19/23 robot + distances au ruban de l'opérateur (±1 cm) | 18,6 mm | refusée |
| **forme arducam posée sur 19/23 robot** | 6,6 mm, puis **2,7 mm** à la passe suivante | **écrite sans forcer** |

Calibration retenue : leave-one-out 19 → 1,7 mm, 23 → 1,5, 25 → 2,7, 26 → 1,4 ;
RMS 0,42 px ; contrôle 0,22 mm. Ancienne extrinsèque :
`arducam_extrinsic_pick.avant_1509_1435.yaml`, locale et non versionnée.

### Pourquoi c'est robuste à chaque nouvelle calibration

- **Si la caméra bouge**, la recalibration automatique du dashboard la replace
  contre une référence qui vient du robot. Aucune erreur ne se recopie.
- **Si la planche bouge**, il faut remesurer le 19 et le 23 au robot, jamais
  par `--reference` à travers une extrinsèque.
- **Quel que soit le détecteur** (HSV, YOLOE ou autre), la calibration corrige la
  même étape « pixel → mm ». Chaque détecteur garde ses propres limites :
  - **précision du pixel** : ~2 mm/px sur l'arducam, prendre le centre du contour
    plutôt que celui de la boîte ;
  - **hauteur de projection** par classe : l'extrinsèque n'est juste qu'à Z = 0 ;
  - **roulis de la pince** tiré du contour, pour les objets non ronds.

### Limites connues

- **SVPRO recalibrée à 15:20, validée par la balle et non par le seul
  leave-one-out.**
  - **Cadrage :** caméra inclinée vers le bas, les marqueurs du bas passent de
    5-8 px à 54-56 px du bord.
  - **Commande :** `calibrate_camera_base_extrinsic.py --camera svpro --markers
    <référence>`.
  - **Leave-one-out :** 2,4 / 5,9 / 3,7 / 4,2 mm, au-dessus du seuil de 5 mm.
  - **Contrôle par la balle,** en (287 ; −137) : 3,3 mm de l'arducam, rayons
    écartés de 2,1 mm (fusion acceptée), centre triangulé à 29,8 mm pour 32
    attendus. L'ancienne donnait 38 mm d'écart et 168 mm de hauteur.
  - **Deuxième emplacement,** près du 19, en (176,6 ; 194,9) : 7,9 mm de
    l'arducam, rayons écartés de 6,4 mm (fusion acceptée), centre à 26,6 mm.
    Moins précis de ce côté, ce qui est cohérent avec le leave-one-out. L'XY
    reste donné par l'arducam, la SVPRO ne corrige que la hauteur.
- **Déplacement de caméra testé à 15:03.** Arducam bougée volontairement puis
  recalibrée : 10,2 mm détectés, leave-one-out ≤ 1,2 mm. Balle saisie du premier
  coup en (303,0 ; 46,5), roulis +30°, descente à 4,5 mm, statut 2 et angle 53.
- **Mesure à la main sensible à J6.** Le bout des doigts est à 22 mm de l'axe J6
  dans le modèle. Au 23, J6 était tourné de 92° par rapport à la pose commandée.
- Le script de lecture des codeurs est resté dans le répertoire de session, comme
  `saisie_seule.py`.

---

## Prochaine étape — détection YOLO

> **Fait le 18/09/2026**, mais pas comme prévu ici : ni COCO ni YOLOE au
> vocabulaire ouvert n'ont suffi, il a fallu **entraîner**. Voir
> « Détection yolo26 entraînée sur les pièces peintes » plus bas. La présente
> section est conservée pour le raisonnement qui y a mené.

### Pourquoi

- La détection actuelle de la balle est un **seuil de couleur HSV** : elle ne
  vaut que pour la balle jaune. Scotch et petit robot passent par
  `Vision.objets` (couleur et forme), moins robuste.
- **YOLO générique (COCO, 80 classes) ne suffit pas, même en YOLO26** (15/09,
  photo arducam + SVPRO) : balle vue `sports ball` 0,76 sur la SVPRO, mais
  **absente sur l'arducam** vue de dessus (`yolo26s/m`) ; `yolov8s` la prend
  pour une horloge, la pince pour un téléphone. Ce n'est pas la version, ce
  sont les classes COCO et la vue de dessus.
- **YOLOE-26 (vocabulaire ouvert, par le nom)** fait mieux sans aucun
  entraînement : `tennis ball` 0,45 et `robot arm` 0,78 sur l'arducam,
  `tennis ball` 0,19 sur la SVPRO. Il rate la pince (boîte sur la base, < 0,1).
  YOLO-World v2 et YOLOE sans consigne sont moins bons. Mesuré sur **une**
  photo seulement.
- Le contrôle SVPRO repose sur des **zones sombres** : il confond facilement un
  mors avec un câble ou avec la base du robot.

### Ce que YOLO apporterait

- **Une seule méthode pour tous les objets** : classes `balle`, `scotch`, `robot`.
- **Une classe `mors`** pour la pince : le contrôle « objet entre les mors »
  devient valable **quel que soit l'objet** et beaucoup moins sensible au câble.
- Des boîtes dans **les deux vues**, base de la triangulation arducam + SVPRO
  (`mycobot_gateway/vision/multiview_localizer.py`), qui donne aussi la hauteur
  de l'objet sans la supposer.

### Plan

1. **Constituer le jeu d'images** avec
   `.venv/bin/python scripts/yolo_capture.py capture` : fenêtre arducam + SVPRO
   en direct avec les détections YOLOE-26, Espace = prise brute, objets
   déplacés à la main, sans mouvement du robot. Les photos des saisies du 14/09
   ne servent pas côté arducam (squelette dessiné dessus). Viser 200 à 300
   images par classe, positions et éclairages variés.
   Classes `balle`, `cylindre`, `cube`, `mors`, `objet`. Trois couches :
   objets nommés (balle sur 26s à 0,10, cylindre sur 26l à 0,40), balle par
   la couleur quand elle est dans la pince, et **`objet` = tout ce qui est posé
   sur la planche** (YOLOE-26l sans consigne, 4 585 noms, filtré en mm par les
   marqueurs 19/23/25/26 : table, hors base, hors marqueurs, hors silhouette du
   bras, confiance ≥ 0,35). Sur 12 photos : balle 12/12, cylindre 5/5 arducam
   et 5/5 SVPRO (vu de côté par `objet`), aucun faux positif ; sur les 14 vues
   de saisie du 14/09, aucun faux objet. Un objet inconnu tenu dans la pince
   est écarté avec le bras.
   **Balle dans la pince : YOLOE lui donne 0 de confiance** ; le seuil jaune
   prend alors le relais (14/14 sur les vues des saisies du 14/09). `mors` vient
   des zones sombres, hors boîtes d'objets, deux zones exigées : trouvés sur les
   12 vues où la pince entoure la balle, sur aucune où elle est loin. À relire
   quand même avant l'entraînement.
2. **Partir de YOLOE-26** (objets désignés par leur nom, un objet nouveau sans
   réentraînement) et **l'affiner** sur nos images pour ce qu'il rate : les
   mors vus par la SVPRO. Dans le **`.venv` d'Osama_ws** (ultralytics
   8.4.108, torch avec CUDA déjà installés).
3. **Arducam — branché le 15/09**, sans modifier `pick_dashboard.py` :
   `/usr/bin/python3 scripts/lancer_pick_dashboard.py --yolo --inventaire robot=4 scotch=2`.
   `yolo_service.py` (`.venv`, GPU) détecte ; `yolo_dashboard.py` remplace
   `Vision.objets` de l'arducam et juge avec les fonctions du dashboard
   (planche, marqueurs détectés, silhouette du bras, jaune laissé à la balle,
   cartons, taille, point le plus épais, `vers_base`). Cylindre → `scotch`
   (petit carton), tout autre objet → `robot` (grand carton). Balle : détecteur jaune
   d'abord, YOLO quand il ne la voit pas (6/6 photos, 0,6-3,2 mm du jaune ;
   seul le jaune la voit dans la pince). YOLO tourne en
   tâche de fond (une image de retard) ; après un dégagement du bras, le fil du
   robot attend une image prise après son appel (~0,25 s) avant de conclure
   « rien vu », sans quoi un objet tout juste découvert serait oublié. Sur 6 photos : cylindre 5/5, aucun faux
   objet. **Pas encore essayé sur le robot**, et la position reste fausse de
   5 à 9 cm tant que l'extrinsèque n'est pas recalibrée (voir préalables).
4. **SVPRO** : boîte de l'objet + boîtes des deux mors → critère « entre les
   mors » dans l'image. Remplace le détecteur de zones sombres.
5. **Triangulation** arducam + SVPRO, une fois l'extrinsèque SVPRO refaite.

### Préalables, sans quoi YOLO visera à côté

- Recalibrer l'extrinsèque arducam **contre `workspace_markers.yaml` vérifié
  depuis le robot**, pour supprimer le correctif de repère provisoire.
- Limiter **J4 à 135°** dans le modèle, pour que la planification ne choisisse
  plus de posture que le robot refuse sans le dire.
- Refaire l'extrinsèque SVPRO si l'on veut trianguler.

Fichiers déjà en place, antérieurs à cette séance : `scripts/yolo_object_detect.py`
(YOLOv8s COCO sur l'arducam → repère base) et
`mycobot_gateway/vision/object_localizer.py` (pixel → base sur le plan de la table).

---

## Détection yolo26 entraînée sur les pièces peintes — 18/09/2026

Les quatre bacs et les quatre objets du dossier de fabrication
(`plans_cotes`, feuilles 2 à 5) ont été **peints**. La détection ne repose donc
plus sur la forme ni sur le noir, mais sur la couleur — et sur un yolo26
entraîné à nommer les huit pièces.

### Pourquoi entraîner, et non régler un détecteur générique

- **yolo26 COCO est aveugle à ces pièces.** Mesuré sur la scène réelle :
  **3 détections en tout** (clavier ×2, ciseaux), **zéro sur la planche**. Un
  cube de bois peint n'est pas une classe COCO.
- **YOLOE nomme approximativement** (4 585 noms) : il voit les taches mais ne
  sait pas dire laquelle est le bac jaune.

D'où **8 classes portant les noms du dossier** : `bac_rouge` `bac_jaune`
`bac_vert` `bac_bleu`, `cube_rouge` (40³), `pave_jaune` (50×30×40),
`cylindre_vert` (Ø44×50), `cube_bleu` (50³). **L'ordre des indices 0-7 est
figé** : les jeux étiquetés le portent, le changer les invalide en silence.

### La teinte nomme, la saturation ne tranche que le jaune

`scripts/tri_couleur.py` étiquette automatiquement, à partir de teintes
**mesurées sur la peinture** — et non tirées du SDF, qui donne bleu 116 et
vert 62 au lieu de 98 et 39, soit 17 et 22 d'écart :

| | rouge | jaune | vert | bleu | bois |
|---|---|---|---|---|---|
| H (OpenCV 0-179) | 0 / 179,5 | 21 | 39 | 98 | **16** |

Le bois tombe au plus près du jaune : c'est la seule couleur que sa teinte
n'écarte pas, et la saturation doit la trancher. Mesure du 18/09 sur les
**deux** caméras, 8 trames, 170 000 px de bois dans la bande jaune :

| | bois q99,9 | pavé jaune | bac jaune |
|---|---|---|---|
| arducam | **218** | 254-255 | 243-244 |
| SVPRO | 195 | **226-228** | 233-234 |

La vue rasante de la SVPRO délave le bois (médiane 146 contre 180) mais délave
le jaune avec. Posée à 230, la porte rejetait le pavé SVPRO **à trois unités
près** — cause du `pave_jaune` absent de toutes les trames SVPRO, et de son
contour rempli à 5 %. `SATURATION_JAUNE = 220` : six unités de marge sous le
pavé, deux au-dessus du bois arducam, remplissage à 53 %. Le bois qui fuit ne
forme aucun amas — l'ouverture 3×3 le balaie, jusqu'à 215.

### Le contrôle dimensionnel ne vaut pas en vue rasante

L'extrinsèque ne vaut que dans le plan Z = 0. Sur la SVPRO, le contour ramasse
les **flancs** des pièces, hautes de 30 à 50 mm :

```
bleu  cube_50    79,1 x 48,2 mm  (plan  50 x  50)   ecart 29,1
vert  cylindre   66,3 x 43,6 mm  (plan  44 x  44)   ecart 22,3
vert  bac       110,2 x 76,1 mm  (plan 105 x 105)   ecart 29,4
jaune bac        90,2 x 67,1 mm  (plan 105 x 105)   ecart 40,7
```

Un côté juste, l'autre faux de 30 à 60 % : ce n'est pas un biais qu'un débord
scalaire corrige. D'où `range_par_aire()` (option `--par-aire`) : **l'aire en
pixels** sépare le bac de son objet d'un facteur **2,2 à 4,9**, sans aucune
reconstruction métrique et sans une seule inversion sur les 4 trames. Une
couleur qui ne donne pas exactement deux taches sur la planche est abandonnée
entière — on ne devine pas. La taille en mm reste **affichée**, jamais un motif
de rejet.

### Quelle boîte étiqueter dépend de l'obliquité

| vue | boîte écrite | pourquoi |
|---|---|---|
| plongeante (arducam) | **contour** | la boîte YOLOE déborde de 10-20 mm ; le contour tient entre 85 et 130 % de son aire |
| rasante (SVPRO) | **détecteur** | le contour devient erratique : 60 % de l'aire sur le bac jaune, 151 % sur le cylindre vert |

**L'aperçu trace la boîte réellement écrite**, et non le contour : sans cela il
montrerait autre chose que l'étiquette, et la relecture — seule garantie de
cette chaîne — ne garantirait plus rien.

### Résultat

| | v2 (arducam seule) | v3 (+ 4 trames SVPRO) |
|---|---|---|
| arducam, 4 trames **hors échantillon** | 8/8 — conf. moy. 0,71 | 8/8 — **0,82**, minimum 0,44 |
| SVPRO, 4 trames | **6/8** (manquent `bac_bleu`, `cube_rouge`) | **8/8** — 0,77 |

⚠ Le 8/8 SVPRO est mesuré **sur les images d'entraînement** : il ne prouve
aucune généralisation. `cylindre_vert` y tient à **0,12 pour un seuil à 0,10**,
sur ses propres images d'apprentissage.

### Ce qui NE marche pas : régler les augmentations

Quatre recettes ont été comparées (saturation protégée, effacement coupé,
mosaïque atténuée, obliquité ajoutée). Puis **la même recette a été relancée
avec trois graines** :

```
                    1509       7      42    étendue
  moyenne          0.851   0.665   0.721     0.186
  minimum          0.670   0.107   0.559     0.563
  cylindre_vert    0.817   0.107   0.580     0.710
```

**Le bruit de tirage (0,186) est cinq fois l'effet attribué à la recette
(0,035).** Toutes les recettes sont indiscernables ; celle qui semblait gagner
avait eu de la chance avec sa graine. Avec 17 images d'une scène figée, le
modèle n'est pas déterminé par les données. **Aucun réglage n'est validable
dans ces conditions** — le seul levier est un jeu d'images aux dispositions
franchement variées, sur les deux caméras.

L'ajout d'obliquité (`degrees`, `shear`, `perspective`) est même nettement
nuisible ici (moyenne 0,559) : sur si peu d'images quasi identiques, elle
ajoute du bruit au lieu de remplacer de vraies vues.

### Quatre pièges, tous rencontrés

- **Les poids ne sont pas où `project=` les demande.** Ultralytics préfixe
  `runs/detect/` à un `project` relatif :
  `runs/detect/training/yolo/runs/<nom>/weights/best.pt`.
- **YOLO26 est sans NMS** : quasi-doublons à un pixel près, à dédoublonner par
  recouvrement (> 0,5).
- **La mAP du run ne veut rien dire** : `train` et `val` pointent sur le même
  dossier.
- **Son échelle de confiance n'est pas celle de YOLOE** : le seuil 0,20 de
  `tri_taille` faisait disparaître le cube rouge, vu à 0,13 sur 6 trames sur 6.
  D'où `SEUIL_MODELE_ENTRAINE = 0,10`.

### Commandes

```bash
# vue en direct des deux cameras, avec les 8 pieces (exposition arducam 75 surveillee)
.venv/bin/python scripts/yolo26_visualisation.py \
    --modele runs/detect/training/yolo/runs/pieces_v3_yolo26s/weights/best.pt

# etiqueter une trame SVPRO (vue rasante)
.venv/bin/python scripts/tri_couleur.py --image trame.png \
    --extrinseque svpro_extrinsic_4marqueurs --par-aire --etiquette training/yolo/pieces_v3

# la chaine complete sur l'arducam, avec le modele entraine
.venv/bin/python scripts/tri_couleur.py --modele runs/detect/.../best.pt
```

**Relire l'aperçu avant d'entraîner.** Une pré-annotation se relit ; celle des
`mors`, le 15/09, était fausse.

---

## Le tri des 4 pièces — commande de référence

C'est **la** commande du banc au quotidien :

```bash
/usr/bin/python3 scripts/lancer_pick_dashboard_final.py
```

Elle enchaîne, dans cet ordre : la question de calibration (un seul « oui »
calibre les **deux** caméras), le chargement de yolo26 sur les 8 classes, la
surveillance d'exposition de l'arducam, puis la fenêtre du tableau de bord.

Prérequis, tous vérifiables avant de lancer :

| | Comment le vérifier |
|---|---|
| Le bridge tourne sur le Pi | **Aller-retour TCP sur le port 5005**, jamais un `ping` — `.224` répond au ping sans servir le bridge |
| Aucune autre caméra ouverte | `fuser /dev/video*` : un second client remet l'exposition en auto |
| Aucun `bridge_tour` résiduel | `gripper_bridge.py` est **mono-client et bloquant** : la connexion est acceptée puis plus rien ne répond |

Sans le bridge, la fenêtre s'ouvre quand même — la vision fonctionne, la barre
d'état affiche « pont injoignable » et aucun mouvement n'est possible.

Options utiles : `--sans-question` (sauter la calibration), `--sans-yolo`,
`--inventaire cube_rouge=2` (nombre de pièces à trier par classe).

> Les correctifs validés vivent dans `scripts/yolo26_dashboard.py`, qui greffe
> yolo26 et ses gardes sur `pick_dashboard` **sans jamais l'éditer**. Attention :
> `fsm.ACTIONS` capture les fonctions **à l'import** — réaffecter `fsm.<fn>` seul
> est inerte, il faut aussi réécrire l'entrée du dictionnaire. Une garde posée
> sans cela reste inactive sans rien signaler.

Le même banc en simulation : `ros2 launch mycobot_gateway banc_realiste.launch.py`
puis `/usr/bin/python3 scripts/yolo26_gazebo.py` — voir
[`../mycobot_description/README_GAZEBO.md`](../mycobot_description/README_GAZEBO.md).

---

## Lancer la démo complète

Prérequis : bridge lancé sur le Pi (`python3 ~/gripper_bridge.py`).

```bash
# Terminal 1 — interface
python3 scripts/pick_and_place_gui.py --host 10.10.0.224
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
