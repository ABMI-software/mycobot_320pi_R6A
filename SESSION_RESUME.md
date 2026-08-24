# Reprise — pick adaptatif LIVE par démonstration

## État actuel (24 août 2026 — journée)

### Ce qui a été accompli aujourd'hui

- **Le cycle ne repart plus au ramassage avec la balle en main.** Garde unique
  dans `MachineEtats.pas()`, deux états neufs (`RECHERCHE_CARTON`,
  `ECHEC_PORTANT`). Seule une perte de prise relance la saisie.
- **Détection du carton refaite** : la couleur ne le sépare pas de la planche
  (mesuré carton H14 S171 V60, planche H15 S187 V84 — l'ancien seuil prenait la
  planche entière). Remplacée par la recherche d'un creux sombre entouré de
  brun, plus un suivi avec hystérésis qui stabilise le rectangle.
- **Temps de cycle : le vrai coupable trouvé.** Chaque mouvement attendait
  22,7 s parce que l'arrivée était jugée sur l'atteinte de la consigne, jamais
  satisfaite à cause de l'affaissement. Détection à l'immobilité :
  **22,7 s → 1,35 s par mouvement, à vitesse inchangée**.
- **SVPRO recalibrée et branchée en appui** de l'arducam (hauteur de la balle
  par triangulation, relais quand le bras masque la vue de dessus).
- **Portée corrigée** : la limite mesurée est 350 mm, pas 335 — des balles
  atteignables étaient refusées.
- 25 tests neufs (`tests/test_pick_fsm_depose.py`, `tests/test_suivi_carton.py`).

### Décisions prises

1. **L'arducam reste la source du X/Y** ; la SVPRO ne fournit que la hauteur et
   le relais en cas d'occultation. Refus si les rayons s'écartent de plus de 25 mm.
2. **Pas de veto de dernière seconde avant le largage** — à cet instant le bras
   masque le carton, la détection n'y est pas fiable. Le carton déplacé se
   rattrape à la recherche, bras dégagé.
3. **Viser le milieu du carton, et à défaut le point de l'ouverture le plus
   proche du milieu** que le bras atteint.
4. **La vitesse du bras reste à 25** : la mesure prouve que le temps ne venait
   pas de là (22,67 s par mouvement à vitesse 25 comme à vitesse 50).

### Prochaines actions

1. [ROUGE] Mesurer un cycle complet réel avec le chronomètre en place et
   attaquer les trois étapes les plus coûteuses qu'il désignera.
2. [JAUNE] Descendre sous ~9 mouvements par cycle : fusionner approche et
   recalage, supprimer la remontée par paliers quand le chemin direct est validé.
3. [JAUNE] Confirmer les dimensions réelles de l'ouverture du carton pour en
   faire un filtre dur (mesuré 101 × 135 mm, à recouper).
4. [VERT] Utiliser la SVPRO pour vérifier que la balle tombe bien dans le carton.

### Commande rapide de reprise

```bash
conda deactivate
/usr/bin/python3 scripts/pick_dashboard.py     # aucun autre client TCP sur la Pi
```

---

## État actuel (20 août 2026 — après-midi)

**Cycle pick-and-place complet réussi sur le robot réel**, de la localisation par
vision au dépôt en bac vérifié par image.

### Ce qui a été accompli

- Boucle d'asservissement visuel livrée (`mycobot_gateway/visual_servo/`, 47 tests).
- Calibration extrinsèque caméra→base sur 16 coins avec validation leave-one-out ;
  arducam RMS 1,01 px, stable à 1,7 mm près après deux jours.
- Cycle complet exécuté : balle localisée (base 360,6 / 23,1), approche, descente
  par paliers, saisie confirmée (statut 2), transport, dépôt en bac.
- Précision de placement finale : **3,9 / 0,6 / 0,1 mm** en X/Y/Z.
- Répétabilité mesurée sur 6 aller-retours : **0,67 mm** en approche unidirectionnelle
  par le haut (spec constructeur : 1 mm, donc tenue), mais **5,88 mm de biais
  directionnel** entre approche par le haut et par le côté.

### Décisions prises

1. **`send_coords` abandonné.** A/B chiffré : 247,8 mm d'erreur contre 18,2 mm
   pour `send_angles` + IK, sur cible identique. Échoue en silence (le bridge
   répond `OK`). Cause : blocage de cardan à RY ≈ −80°.
2. **Travailler sur la branche IK coude haut** (J3 < 0), pas celle de l'historique.
3. **Tourner l'orientation cible selon l'azimut** (`Rz(Δazimut) @ R_réf`) —
   facteur 100 sur le résidu IK.
4. **Compenser l'affaissement gravitaire** (~13 mm à vide, ~15 mm chargé).
5. Bridge Pi : **`gripper_bridge.py` obligatoire**, pas `bridge_pi_simple.py`.

### Prochaines actions

1. [ROUGE] Recalibrer la SVPRO — son extrinsèque est fausse de 32 à 149 mm
   (caméra déplacée). Elle ne voit que 2 des 4 marqueurs : la réorienter d'abord.
2. [ROUGE] `scripts/tool_offset.json` est **faux** (~90° d'erreur de direction) et
   le nœud d'asservissement le charge. Le refaire ou le neutraliser.
3. [JAUNE] Porter dans le nœud la rotation d'orientation selon l'azimut et la
   compensation d'affaissement — aujourd'hui appliquées dans les scripts d'essai.
4. [FAIT] Transfert haut validé — 150 mm au lieu de 120, garde au sol pendant la
   translation portée de ~120 à **185 mm**, statut pince vérifié à deux points du
   transit. Contrainte : le plafond au-dessus d'une cible à 368 mm de portée est
   de **160 mm** (à 200 mm le résidu IK monte à 12,8 mm) — choisir la hauteur de
   transfert comme la plus haute atteignable **aux deux extrémités**.
5. [VERT] Passer le serrage par `set_pro_gripper_torque` plutôt que par l'angle.

### Commande rapide de reprise

```bash
conda deactivate && source /opt/ros/jazzy/setup.bash
source ~/Osama_ws/install/setup.bash
ping -c1 10.10.0.221            # bridge = gripper_bridge.py sur la Pi
ros2 launch mycobot_gateway visual_servo.launch.py     # démarre DÉSARMÉ
```

⚠ Vérifier `ps aux | grep bridge_tour` avant : le pont de la Pi est mono-client
et bloquant, un `bridge_tour` résiduel (que le preflight laisse tourner) le fige.

---

**Date d'arrêt :** 31 juillet 2026

**Workspace :** `/home/genji/Osama_ws/src/mycobot_R6A`

**État robot connu :** pose de remise, pince ouverte. Ne pas supposer que cette
pose est encore actuelle à la prochaine session : relire les angles avant tout
mouvement.

## Objectif confirmé

L'objet peut être placé n'importe où dans la zone délimitée par les quatre
ArUco, sous réserve de la portée du bras et de la zone réellement enseignée.
Le système doit redétecter l'objet, recalculer sa position dans `base_link`,
adapter les angles articulaires appris, saisir, puis effectuer la remise.

## Architecture correcte

```text
Arducam -> centre objet (HSV aujourd'hui, YOLO custom possible)
         -> intrinsèque cam_0
         -> extrinsèque eye-to-hand caméra->base FIXE
         -> XYZ objet dans base_link

4 ArUco -> frontière géométrique de sécurité LIVE
         -> ils ne détectent pas l'objet
         -> ils ne remplacent pas l'extrinsèque

XYZ base -> interpolation de démonstrations -> send_angles uniquement
```

La planche ArUco peut glisser sur le même plan horizontal. La caméra doit rester
rigide par rapport à la base robot. Un changement de hauteur/inclinaison de la
planche ou un déplacement de la caméra exige une nouvelle calibration adaptée.

## Travail terminé

- `scripts/adaptive_pick_by_demo.py` créé et corrigé :
  - extrinsèque par défaut :
    `training/calibration/arducam_extrinsic_handeye.yaml` ;
  - détection balle multi-images sur `/camera/image_raw` ;
  - quatre IDs ArUco 19/23/25/26 obligatoires ;
  - frontière ArUco reconstruite en direct dans `base_link` ;
  - stabilité des tags exigée, dispersion maximale 3 px ;
  - cible refusée hors frontière ou trop près du bord ;
  - apprentissage transactionnel approche/pick ;
  - interpolation IDW en angles, sans `send_coords` ;
  - aucune extrapolation hors enveloppe des démonstrations ;
  - dry-run par défaut ; confirmation explicite pour le réel ;
  - re-détection avant descente et retrait de secours en cas d'échec.
- `scripts/live_aruco_geometry.py` et ses tests sont présents.
- Guide : `docs/ADAPTIVE_PICK_BY_DEMO.md`.
- Tests validés :
  - `19/19` dans `tests/test_adaptive_pick_by_demo.py` ;
  - `8/8` dans `tests/test_live_aruco_geometry.py` ;
  - compilation Python OK.

## Dernière validation LIVE, sans mouvement

La capture ROS/Arducam en lecture seule a réussi :

- balle stable : pixel `(162.6, 155.6)` ;
- position calculée : `(X,Y,Z) = (0.3633, 0.2131, 0.0335) m` ;
- ArUco vus : `[19, 23, 25, 26]` ;
- dispersion maximale : `0.58 px` ;
- frontière calculée dans la base :
  - `(0.0906, 0.2858)` ;
  - `(0.1031, -0.0674)` ;
  - `(0.4885, -0.0826)` ;
  - `(0.5206, 0.2748)`.

La transaction produite était seulement une sonde temporaire :
`/tmp/adaptive_pick_live_probe_v2.json.pending`.

## Ce qui n'est pas terminé

- Aucun dataset réel `scripts/adaptive_pick_demos.json` n'a encore été validé.
- Aucune commande de mouvement robot n'a été envoyée pendant cette correction.
- Il faut au minimum trois démonstrations synchronisées et non collinéaires ;
  cinq à neuf points donnent une couverture plus sûre de la table.
- Les anciennes poses de `scripts/pick_place_positions.json` ne doivent pas être
  importées silencieusement sans associer chaque pose à l'observation objet de
  la même démonstration.
- La remise automatique après le pick adaptatif n'est pas encore intégrée dans
  `adaptive_pick_by_demo.py`. Les poses `handover_approach` et `handover` sont
  déjà enregistrées.
- L'ouverture sur détection de main doit utiliser un vrai modèle YOLO possédant
  une classe `hand`, une ROI fixe sous la pince et plusieurs frames stables.
  `yolov8n.pt` COCO n'a pas de classe main : ne pas ouvrir sur la classe
  `person` par défaut.

## Reprise recommandée

1. Vérifier visuellement la scène, l'arrêt d'urgence et la pose réelle du bras.
2. Démarrer ou vérifier `dream_multicam.launch.py` et `bridge_tour`.
3. Ouvrir la première démonstration, sans mouvement automatique :

   ```bash
   cd /home/genji/Osama_ws/src/mycobot_R6A
   .venv/bin/python scripts/adaptive_pick_by_demo.py --begin-live --name point-1
   ```

4. Montrer manuellement l'approche puis la prise et capturer :

   ```bash
   .venv/bin/python scripts/adaptive_pick_by_demo.py \
     --capture-approach --read-current-angles
   .venv/bin/python scripts/adaptive_pick_by_demo.py \
     --capture-pick --read-current-angles
   ```

5. Répéter à au moins deux autres positions non collinéaires.
6. Vérifier le dataset puis faire uniquement un plan sec :

   ```bash
   .venv/bin/python scripts/adaptive_pick_by_demo.py --show
   .venv/bin/python scripts/adaptive_pick_by_demo.py --plan-live
   ```

7. Ne lancer `--execute` qu'après inspection du plan et d'abord près d'une
   démonstration connue, à vitesse lente.

## Détails à ne pas oublier

- Pi/bridge observé : `192.168.223.59:5005`.
- Avec le dashboard actif, utiliser ROS `/to_robot` et `/from_robot`, pas une
  seconde connexion TCP directe.
- Utiliser `.venv/bin/python` pour le live : OpenCV système 4.6 a provoqué un
  crash ArUco ; le venv utilise OpenCV 5.
- `workspace_markers.yaml` contient encore une incohérence documentaire
  `marker_size_m: 0.080` contre commentaire `50 mm`. La frontière actuelle
  utilise les centres et n'utilise pas cette taille, mais elle devra être
  mesurée/corrigée avant un futur PnP par coins.
