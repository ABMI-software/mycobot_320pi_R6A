"""Asservissement visuel en boucle fermée — perception, suivi, commande, sûreté.

Les modules de ce paquet sont du Python pur (ni rclpy ni cv2 pour la logique de
commande) : ils sont testables hors ROS2, et les nœuds de `..visual_servo_node`
ne font que les câbler sur des topics. Découpage :

    object_tracker  §8.1-8.2  Kalman [X,Y,VX,VY] + prédiction de latence
    fusion          §7.2      sélection/pondération des mesures multi-caméras
    servo_law       §8.3      V_EE = V_O − K·e, saturé
    safety          §12       superviseur : fraîcheur, sauts, divergence, limites
    state_machine   §10       SEARCH → … → SAFE_STOP

Les numéros de section renvoient au dossier d'asservissement visuel du projet.
Repère de travail : `base_link`, mètres, secondes. Le pilote du robot attend des
millimètres — la conversion est faite au moment de commander, pas avant.
"""
