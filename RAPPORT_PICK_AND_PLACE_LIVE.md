# Rapport — Pick-and-place vision-guidé LIVE (balle de tennis)

**Date :** 2026-07-31
**Robot :** MyCobot 320 Pi (`10.10.0.221`) + Pro gripper (`gripper_bridge.py` sur le Pi)
**Caméra :** Arducam 8MP (`/dev/video2`), mono, vue oblique du dessus
**Résultat :** ✅ **Pick-and-place réel réussi** — balle détectée, saisie (statut pince 2 confirmé par soulevé + balle décollée de la table), transportée, déposée, retour home.

---

## 1. Objectif

Le robot doit être **auto-adaptatif** : la vision détecte la balle *où qu'elle soit* et le bras va la chercher — **pas** un simple rejeu de poses figées (teach-and-playback). Ce dernier ne servait que de référence pour comprendre une préhension sûre.

---

## 2. Ce qui marche

| Brique | État | Détail |
|--------|------|--------|
| Détection balle | ✅ | **Couleur HSV + saturation** (`--hsv-lo 20 170 70 --hsv-hi 35 255 255`) + **exclusion des bords** pour rejeter un objet jaune parasite collé au cadre. Voir §3.1. |
| Localisation | ✅ (à ~5 cm près) | Extrinsèque ArUco `arducam_extrinsic_markers.yaml` (4 markers, 0.71 px) → balle en base. **Décalage systématique ~5 cm**, voir §3.3. |
| Mouvement sûr | ✅ | Approche top-down **par paliers** + surtout **posture-poignet du démo** (J5≈−15), voir §3.2. |
| Saisie | ✅ | Pro gripper angle 20 → statut 2. Faux positifs possibles, voir §3.4. |
| Dépose | ✅ | Poses apprises (`place_approach`, `place`) en `send_angles`. |

---

## 3. Problèmes rencontrés + solutions (les vraies leçons)

### 3.1 YOLO générique ne détecte pas la balle
- `yolov8n/s` (COCO), image plein cadre : voit *sink, keyboard, mouse, tv* — **jamais** la balle. Sur un **crop** centré : `sports ball` à **0.11** conf seulement.
- Cause : balle ~35 px dans un cadre 640×480 sombre et oblique → trop petite pour COCO générique.
- **Solution retenue : détecteur couleur.** La table en bois (H≈18, S≈93) chevauche la teinte de la balle (H≈25) → discriminant = **saturation** (balle S≈230). Seuil S élevé + exclusion des blobs touchant le bord.
- *Pour YOLO plus tard : poids custom entraînés sur cette balle.*

### 3.2 ⚠️ Bug central — top-down cartésien [180,0,0] → mauvaise branche IK
- Commander `send_coords` avec orientation **top-down [180,0,0]** → l'IK de pymycobot choisit une branche avec **J5 ≈ +90°** (poignet plié à 90°). Conséquences :
  - **auto-collision** (la pince rentre dans le corps du bras lors des grandes transitions),
  - **mauvaise géométrie de préhension** (le gripper part de travers, même flange "top-down").
- **Le démo saisissait parfaitement** parce qu'il travaillait en **angles articulaires** avec un **poignet quasi droit (J5 ≈ −15°)**.
- **Solution :** reprendre l'**orientation de saisie apprise** (≈ `-11.7, -74.7, 141.8`, poignet façon démo) au lieu de [180,0,0]. En gardant cette orientation lors des `send_coords`, l'IK **reste sur la bonne branche** (J5 vérifié ≈ −15 à chaque pas). On adapte seulement la position (X, Y, Z) selon la vision.
- Portée réelle mesurée : **~440 mm** (et non ~350 mm comme supposé au départ — la balle à 327 mm n'était jamais hors d'atteinte ; c'était l'orientation le problème).

### 3.3 Extrinsèque mono-caméra : décalage systématique ~5 cm
- La saisie one-shot ratait de **~5 cm** : la vue oblique fait **se superposer par parallaxe** la pince (haute) et la balle (sur la table), et surtout le **repère "monde" des ArUco n'est pas parfaitement aligné au repère BASE du robot**. Le RMS 0.71 px ne valide que l'ajustement des marqueurs, **pas** l'alignement au robot.
- **Contournement de session :** alignement fin **humain-dans-la-boucle** (l'opérateur regarde de côté, dicte "droite / recule de N cm", nudges cartésiens en gardant l'orientation).
- **Vrai fix à faire :** **calibration hand-eye** (`mycobot_gateway/mycobot_gateway/calibrate_hand_eye_node.py`) → transfo caméra↔base précise, saisie one-shot fiable et **restée adaptative**.

### 3.4 Faux positif du statut pince + sous-dépassement Z
- `get_pro_gripper_status` a renvoyé **2 (objet saisi)** alors que les doigts étaient **au-dessus** de la balle (rien serré). → **Toujours vérifier par un soulevé test** (la balle doit décoller) et/ou la vue caméra.
- `send_coords` **sous-descend de ~11–15 mm** par rapport à la consigne Z. → commander Z en conséquence, **plancher de sécurité** (jamais sous la table).

---

## 4. Recette qui a fonctionné (reproductible)

1. **Détection** : couleur HSV+saturation, exclusion bords → pixel balle → base via extrinsèque markers.
2. **Poignet sûr** : aller à la pose apprise `pick_approach` (bon poignet), lire son orientation de saisie.
3. **Adaptation** : `send_coords` vers (X,Y balle, Z sûr, **orientation-démo**), vérifier `|J5| < 40` (bonne branche).
4. **Alignement fin** : nudges cartésiens ±cm guidés par vue latérale humaine (compense le décalage extrinsèque).
5. **Descente** : petits pas Z (~1–2 cm), vue arducam à chaque pas, plancher sûr.
6. **Saisie** : gripper angle 20 → **vérifier par soulevé test** (statut 2 seul insuffisant).
7. **Dépose** : poses apprises `place_approach` → `place` → ouverture → retrait → home.

---

## 5. Sécurité — incidents & garde-fous

- **Deux quasi-casses de la pince** :
  1. descente à hauteur de saisie **sans balle** → doigts au ras de la table ;
  2. transition vers top-down forcé → **pince dans le corps du bras** (branche IK J5≈+90).
- Garde-fous adoptés : bras **≥ 200 mm** sauf descente finale ; **jamais** de descente à vide ; vitesse **10–15** ; observation **arducam après chaque mouvement** ; `power_on` requis pour sortir la LED du rouge.

---

## 6. Prochaines actions

1. **[ROUGE] Calibration hand-eye** (`calibrate_hand_eye_node.py`) → supprimer le décalage ~5 cm → saisie one-shot autonome sans nudges humains.
2. **[JAUNE]** Générer l'**orientation de saisie adaptée à l'azimut** (au lieu de figer l'orientation-démo) pour couvrir toute la zone atteignable proprement.
3. **[JAUNE]** IK côté PC avec **seed sur la branche du démo** pour garantir `J5` bon sans dépendre de pymycobot.
4. **[VERT]** YOLO custom entraîné sur la balle (si on veut abandonner la couleur).
5. **[VERT]** 2ᵉ caméra / vue plus top-down pour réduire parallaxe et occlusion.

---

*Chaîne : `scripts/pick_and_place_vision_live.py` (perception) + `scripts/pick_and_place_real.py` (contrôle socket, poses apprises) + `gripper_bridge.py` (Pi).*
