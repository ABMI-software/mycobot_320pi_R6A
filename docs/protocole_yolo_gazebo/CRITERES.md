# Critères et plages de données attendues — protocole YOLO / Gazebo (état au 13/10)

Il n'existe pas encore de plage attendue colonne par colonne pour les CSV par essai. Les seules
bornes écrites sont les critères et les seuils ci-dessous, et ce sont des **critères proposés, pas
encore validés**.

## DREAM et banc

| donnée | plage ou critère | où c'est écrit |
|---|---|---|
| détection DREAM | > 90 % | [PROTOCOLE_YOLO_GAZEBO.md](../PROTOCOLE_YOLO_GAZEBO.md), critères proposés (étape 10) |
| erreur médiane DREAM | < 5 px | idem |
| rotation de T_DREAM | < 5° | idem |
| température PC | gel à 85 °C, reprise à 75 °C, arrêt au-delà de 95 °C | idem, § garde thermique (campagne du 07/10) |
| vues DREAM utilisables | ≥ 5 keypoints par caméra | `MIN_KP_FUSED` dans [`tri_dream_dashboard.py`](../../mycobot_gateway/mycobot_gateway/tri_dream_dashboard.py) |
| position des objets | bande x ∈ [146 ; 494] mm | étape 2, zone de travail |

## YOLO

Pour YOLO, le protocole ne fixe **aucun critère de performance chiffré** : ni taux de détection
minimal, ni erreur maximale en px ou en mm. Il n'y a que des seuils de fonctionnement et un critère
d'adoption de modèle :

| donnée | seuil | où c'est écrit |
|---|---|---|
| confiance YOLO | détection gardée si ≥ 0,10 (`conf_threshold`, celui du service) | [PROTOCOLE_YOLO_GAZEBO.md](../PROTOCOLE_YOLO_GAZEBO.md), étape 4 |
| appariement YOLO ↔ vérité | bonne détection = bonne classe et IoU ≥ 0,5 | idem, étape 5 |
| pièce « cachée » | exclue si > 50 % couverte par une pièce plus proche | idem, journal 29/09 (`yolo26_tri_eval.py`) |
| adoption d'un nouveau modèle | bac_jaune ≥ 9/10 en caméra top, et aucune classe réelle qui recule | idem, réentraînement v6 |
| erreur de localisation 3D | **pas de seuil** : le critère de sortie de l'étape 6 est seulement la distribution de `erreur_3d` par classe | idem, étape 6 |

Pour comparaison, les valeurs mesurées sur la campagne du 07/10 (16 saisies, 4 caméras) sont :
YOLO ↔ Gazebo en XY, **0,4 mm** de médiane (max 0,6). Ce sont des mesures, pas des plages
attendues.

**Sans plage** à ce jour : `erreur_dream_gazebo_x/y/z_mm`, `erreur_yolo_gazebo_*`,
`yolo_confiance`, `temps_du_tri_s`, `dream_points`, et les colonnes de `dream.csv` et `yolo.csv`.

Données concernées : les 5 CSV par essai de la campagne du 07/10, voir
[PROTOCOLE_YOLO_GAZEBO.md](../PROTOCOLE_YOLO_GAZEBO.md) § « CSV par essai ».

Pour les fixer sans les caler sur les résultats (ce qui serait circulaire), chaque colonne doit
recevoir deux bornes :

1. **plage valide** : ce qui est physiquement possible. Une valeur en dehors signale un bug, pas une
   mauvaise performance. Exemples : `yolo_confiance` ∈ [0 ; 1], détection ∈ [0 ; 100] %,
   `dream_points` ∈ [0 ; 7], positions dans la zone de travail ;
2. **plage attendue** : ce qu'on vise, tirée d'un critère existant ou d'une référence
   indépendante (codeurs, tolérance de la pince). Sans source, elle reste **à trancher**.
