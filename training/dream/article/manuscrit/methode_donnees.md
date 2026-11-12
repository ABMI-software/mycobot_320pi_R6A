# III. Method — Synthetic Data Generation

<!-- Section du manuscrit. Langue : anglais. Les marqueurs \todo{À COMPLÉTER : …}
     attendent une entrée du registre ou de la biblio ; voir les notes de travail
     en fin de fichier. Aucun chiffre de performance dans cette section. -->

We train DREAM \cite{lee2020dream} on synthetic images only. Training from
synthetic data alone is not new: DREAM, RoboPose \cite{labbe2021robopose} and
SGTAPose \cite{tian2023sgtapose} all transfer to real images of the Franka
Panda. Our generator follows the domain-randomization recipe of DREAM
(§ II.C of \cite{lee2020dream}) and departs from it where our robot, our
simulator or our evaluation require it. Table I lists these departures. The
generator was frozen before the main collection; this section describes that
frozen version.

## A. Robot and keypoints

The robot is a MyCobot 320 Pi, a low-cost 6-axis arm driven by servos, fitted
with the adaptive gripper mounted on our physical robot. We model it from its
URDF, gripper included. Training on a bare flange would teach a configuration
that never occurs at test time, since the gripper always occludes the flange
on the real arm.

Following DREAM, we place one keypoint at the origin of each joint frame
given by the URDF: the base and the six link frames, seven keypoints in all.
On this robot the frames of link1 and link2 share the same origin (the
shoulder has two rotation axes and no translation between them). These two
keypoints are therefore one 3D point. We keep both to stay within the DREAM
output format, and we report metrics on both seven and six distinct points.

## B. Scene and rendering

We render with Gazebo Harmonic (Ogre2 renderer) instead of the UE4-based NDDS
tool used by DREAM. The robot stands in a fixed world with a floor, a back
wall, a side panel, a table and three static objects. The arm is not
teleported. A joint trajectory controller (`gz_ros2_control`) drives it to
each target, so the arm moves under simulated dynamics, gripper load
included.

The camera is a separate model that the generator moves before each frame.
Its intrinsics are fixed: 640 × 480 pixels, horizontal field of view 1.15 rad,
which matches the focal length of one of our real cameras. We do not randomize
intrinsics. This keeps any viewpoint effect separate from a focal-length
effect.

A second sensor on the same link renders a semantic segmentation of the robot
(arm and gripper) with the same pose and field of view. The mask therefore
aligns pixel for pixel with the colour image. The mask stream can lag the image
stream. We accept a mask only if its timestamp is within 0.05 s of the image
timestamp and its content has changed since the previous frame. Otherwise the
frame keeps its Gazebo background and is excluded from compositing (§ III-E).

## C. Camera sampling

DREAM places the camera on a truncated hemispherical shell with azimuth
from −135° to +135°, elevation from −10° to 75° and distance from 75 to
120 cm \cite{lee2020dream}. We draw a new camera pose for every frame on a
shell centred on the workspace point (0.10, 0, 0.20) m in the robot base
frame:

- azimuth uniform over the full circle;
- elevation uniform in [18°, 86°];
- distance uniform in [0.70, 1.45] m;
- the optical axis aims at the workspace point plus a uniform offset of up
  to ±0.10 m on each axis, so the robot is not always centred;
- roll uniform in [−9°, 9°].

We reject a draw if the camera falls below 0.25 m or if a sphere of radius
0.34 m around the workspace point does not fit entirely in the field of view.
The distance and elevation ranges enclose the poses of our two real cameras,
with a margin on both sides.

A new camera pose for every frame matters for our robot. With fixed training
viewpoints a network can learn the image position of the base as a constant
per viewpoint instead of detecting it. We observed this failure on an earlier
model fine-tuned on two fixed real cameras \todo{À COMPLÉTER : résultat
négatif « base mémorisée, 0 \% d'équivariance au décalage » — attend
l'entrée D du registre en verdict tenu ; sinon retirer la phrase}.

## D. Arm configurations and labels

Arm configurations are drawn uniformly within the joint limits of the robot
(J1 ±168°, J2 ±135°, J3 ±150°, J4 ±145°, J5 ±165°, J6 ±180°). We reject a
configuration if any link or either fingertip comes within 50 mm of the table
plane, if a distal link folds into the base column, if two non-adjacent links
or the gripper and a proximal link intersect (capsule model), or if the arm
meets a static object of the scene. Because the arm moves between targets,
we also check the straight joint-space path from the previous configuration.
The gripper is not actuated during collection \todo{À COMPLÉTER : état
d'ouverture de la pince pendant la collecte, à relever dans
`joints_complets.csv`}.

Each arm configuration is seen from three camera poses. The same arm pose then
appears at three places in the image, which further prevents the base from
being tied to a fixed image position.

Labels come from measured joint angles, not commanded ones: under gravity and
gripper load the two differ. The arm can still be moving when an image is
rendered, and the image reaches the generator with a delay. We therefore
interpolate the joint angles at the simulation timestamp of the image, between
two joint-state messages at most 0.05 s apart. We recheck the measured
configuration against the collision rules above and discard the frame if it
fails. The 2D labels are the projections of the seven keypoints through the
logged camera pose and the ideal pinhole intrinsics. We drop a frame if a
keypoint lies behind the camera or more than 50 pixels outside the image.

## E. Appearance randomization

We randomize appearance in two places: in the simulator during collection,
and offline on the rendered images.

*Distractors.* Every 40 frames we remove the previous set and spawn 2 to 6
primitives (box, cylinder or sphere) of random size and uniformly random
colour, 0.12 to 0.65 m from the base. Thirty percent float at 0.05 to 0.50 m
with random orientation; the others rest on the floor. They are visual only:
they occlude the arm but do not collide with it. DREAM uses YCB objects as
flying distractors \cite{lee2020dream}; we use primitives.

*Lights.* With each distractor set we add 0 to 3 point or spot lights at
random positions, with intensity uniform in [0.2, 1.0]. Forty percent are
tinted; the rest are near-white. They add to the neutral lights of the world,
whose direction and intensity are also redrawn every 40 frames. Because the
camera moves at every frame, the angle of incidence on the arm changes at
every frame even within a lighting set. DREAM uses three free lights with
random intensity and colour \cite{lee2020dream}.

*Backgrounds.* For frames with a valid mask, we replace everything outside the
robot by a random crop of a COCO val2017 image, as DREAM does
\cite{lee2020dream}. Each background is rescaled by a random factor in
[1, 1.6] above the size that covers the frame, cropped at a random position to
640 × 480, and mirrored with probability 0.5. A Gaussian blur of 0.8 pixel on
the mask edge removes the residual rim of the Gazebo scene. Composited frames
lose the table and the arm's shadow; the robot floats in front of the photo,
as in DREAM. The segmentation label also covers four fixed camera bodies
attached to the robot model. We remove them from the mask by keeping only the
mask regions that contain a projected keypoint.

*Robot colour.* DREAM colours the robot mesh at random. We tint the robot
offline instead: every robot pixel is multiplied by a per-channel factor drawn
in [0.25, 1]. Multiplication keeps the rendered shading. The lower bound keeps
the arm, which is white on the real robot, recognizable.

The share of frames with a valid mask varied between collection batches. We
therefore set the compositing probabilities per batch so that, over the whole
set, about half of the frames receive a COCO background and about 30 % a tint.
\todo{À COMPLÉTER : parts réalisées de fonds et de teintes sur le jeu final
(fichiers `composition.csv`) — attend une entrée E3.7 du registre}.

## F. Held-out viewpoint sector

A validation set drawn at random from the same distribution shares its
viewpoints with the training set, so it cannot detect a network that memorizes
the framing. We hold out every frame whose camera azimuth around the workspace
point lies in [60°, 100°]. No frame of this sector is used for training. This
sector is our synthetic test of viewpoint generalization.

The test has a limit. The held-out frames share the elevation and distance
ranges, the scene and the arm-pose distribution of the training set. They
measure interpolation across azimuth, not a new viewpoint in every respect.
The question that motivates this work is whether a synthetic-only model holds
on *real* viewpoints that training never saw. We address it with a real test
set taken from camera positions withheld from every training stage (§ IV)
\todo{À COMPLÉTER : renvoi à la section du jeu réel multi-vues E2}. To our
knowledge, no published camera-to-robot evaluation withholds a real viewpoint
explicitly; DREAM's Panda-Orb set does vary the camera pose
\cite{lee2020dream}.

## G. Dataset and annotations

We collected the data in ten independent batches with the simulator restarted
between batches \todo{À COMPLÉTER : nombre d'images collectées, puis
d'images d'entraînement et du secteur tenu après conversion, et md5 du jeu
figé — attend une entrée E3.7 du registre}. The data are written in the NDDS
format that DREAM reads. Each frame also carries annotations for other
camera-to-robot methods: the camera-to-base pose, all measured joint
positions (gripper included), the 3D pose and 2D projection of every URDF
link, both fingertips and their midpoint, the robot mask, bounding boxes, the
camera pose and azimuth, and the distractors and lights in place. These
fields are added next to those read by DREAM, which are left unchanged.
\todo{À COMPLÉTER : accord des liens annotés avec la cinématique des
keypoints — attend une entrée E3.6b du registre}.

---

**Table I. Synthetic data: DREAM \cite{lee2020dream} and this work.**

| | DREAM (Panda) | This work (MyCobot 320 Pi) |
|---|---|---|
| Robot | 7-axis Panda | 6-axis servo-driven arm, adaptive gripper mounted |
| Keypoints | joint frames from the URDF | 7 joint-frame origins; link1 and link2 coincide in 3D |
| Renderer | UE4 with NDDS | Gazebo Harmonic (Ogre2), arm moved by a position controller |
| Camera | per frame, azimuth −135°…135°, elevation −10°…75°, 0.75–1.20 m | per frame, azimuth full circle, elevation 18°…86°, 0.70–1.45 m, roll ≤ 9° |
| Views per arm pose | — | 3 |
| Joint angles | random within limits | uniform within limits, collision and 50 mm ground clearance rejection, measured and interpolated at image time |
| Distractors | YCB objects | 2–6 primitives per 40 frames, 30 % floating |
| Lights | 3 free lights, random intensity and colour | world lights plus 0–3 point/spot lights per 40 frames, 40 % tinted |
| Background | COCO | COCO val2017, composited offline behind the mask on about half of the frames |
| Robot colour | random mesh colour | multiplicative tint, offline, on about 30 % of the frames |
| Held-out viewpoints | — | azimuth sector 60°–100° |
| Volume | ~100k per robot | \todo{À COMPLÉTER : registre E3.7} |

---

## Notes de travail (hors manuscrit — à retirer avant soumission)

### Sources, par affirmation

- Thèse, positionnement, règles de comparabilité, feuille de route E3, constats
  datés : `docs/ARTICLE_DREAM.md` (« Thèse visée », « Règles de
  comparabilité », E3.0-E3.10, « Ce qu'on a appris » au 03/11 et au 10/11).
- Recette DREAM § II.C (coquille −135…135°, −10…75°, 75-120 cm, cône d'axe
  optique ; 3 lumières ; COCO ; YCB en distracteurs volants ; couleur du
  maillage ; angles dans les butées ; NDDS sur UE4 ; ~100k images par robot) :
  `papiers/DREAM-image came to estimate robot pose.pdf`, p. 2-3 (lu ce jour).
  Panda-Orb (27 positions, caméra en orbite) : même PDF, p. 3.
- Robot, URDF avec pince, pourquoi la pince :
  `mycobot_gateway/launch/synthetic_data_camera_libre.launch.py` (commentaire
  sur `urdf_path`) ; ARTICLE_DREAM E3.6.
- 7 keypoints = origines des repères, link1 = link2 en 3D :
  `training/dream/mycobot_fk.py` (`KEYPOINT_NAMES`, `forward_kinematics`) ;
  `mycobot_gateway/mycobot_gateway/synthetic_data_collector_v3.py`
  (commentaire « pts[1]==pts[2] (compound shoulder) ») ; CLAUDE.md (§ DREAM
  septembre). PCK 7 et 6 points : ARTICLE_DREAM E1 et registre (champ recalcul).
- Monde `randomized` (sol, mur, panneau, table, 3 objets statiques) :
  `synthetic_data_collector_v3_camera_libre.py` (`CLUTTER_MODELS`,
  `SURFACE_MODELS`, `OBSTACLES`) ; launch (`WORLD_NAME = 'randomized'`).
- Bras piloté par contrôleur, pas téléporté : launch (spawners
  `mycobot_controller`) ; v3 `_command_joints` / `move_time`.
- Caméra : `mycobot_description/worlds/camera_libre.sdf` (hfov 1,15 rad,
  640×480, capteur de segmentation sur le même lien, focale non randomisée et
  pourquoi) ; `training/dream/mycobot_fk.py` (`GAZEBO_INTRINSICS`, sans
  distorsion).
- Tirage caméra (azimut uniforme, élévation 18-86°, distance 0,70-1,45 m,
  visée (0,10 ; 0 ; 0,20) ± 0,10 m, roulis ≤ 9°, z ≥ 0,25 m, sphère de
  0,34 m dans le champ, 60 essais) : `synthetic_data_collector_v3_camera_libre.py`
  (`_tire_pose`, `_robot_dans_le_champ`, constantes de classe).
- Masque synchronisé (≤ 0,05 s, CRC, relances 0,1 s × 15, `frais`) : même
  fichier (`ECART_MASQUE_MAX_S`, `_enregistre_masque`) ; `docs/SYNTHETIC_DATA.md`
  (« Caméra libre avec masque »).
- Poses du bras (butées, garde 50 mm doigts compris, colonne, capsules,
  obstacles, trajet en 14 pas) : `synthetic_data_collector_v3.py`
  (`JOINT_LIMITS`, `_is_collision_free`) ;
  `synthetic_data_collector_v3_garde_basse.py` (50 mm) ;
  `synthetic_data_collector_v3_camera_libre.py` (`_BOUTS_DOIGTS`,
  `PAIRES_PINCE`, `OBSTACLES`, `_trajet_sur`, `_capture_and_save` qui revalide
  la pose mesurée).
- **Tirage uniforme seul dans E3.7** : le collecteur prévoit 40 % de poses
  réelles du pick bruitées (`FRACTION_PICK = 0.4`), mais le fichier
  `training/dream/captures/real_montage_0901/labels.csv` est absent de ce
  clone, et les 14 `launch.log` de `~/dream_article/E3/E3_7/` portent
  « absent — aucune pose du pick, tirage uniforme seul ». `grasp_fraction`
  vaut 0 dans le launch. Le texte dit donc « uniforme dans les butées ».
- 3 vues par pose, 0,7 s entre vues : `VUES_PAR_POSE`, `TEMPS_VUE`.
- Angles mesurés interpolés au temps de l'image (historique 3 s, pas ≤ 0,05 s) :
  `_joints_a_l_image`, `HISTORIQUE_JOINTS_S`, `PAS_INTERPOLATION_MAX_S` ;
  ARTICLE_DREAM 03/11 (pilote E3.7).
- Rejet à la conversion (> 50 px hors image, derrière la caméra, sans pose) :
  `training/dream/convert_to_ndds_camera_libre.py` (boucle principale).
- Distracteurs (2-6, lot de 40, formes et tailles, rayon 0,12 + demi-taille à
  0,65 m, 30 % flottants à 0,05-0,50 m, visuels seuls) :
  `_tire_distracteur`, `_renouvelle_distracteurs`, constantes ;
  ARTICLE_DREAM 03/11 (« Distracteurs (E3.4) », écart YCB).
- Lumières (0-3, point/spot, 40 % teintées, intensité 0,2-1,0, sans ombre,
  `visualize false`) : `_pose_lumieres` ; lumières du monde redessinées tous
  les 40 : `_randomize_scene` (`PERIODE_ECLAIRAGE`) et v3 `_randomize_lights`.
- Fonds (COCO val2017, échelle 1-1,6, recadrage, miroir 0,5, bord flouté
  0,8 px), teinte (0,25-1 multiplicative), trames à masque frais seulement :
  `training/dream/composer_fonds.py` ; ARTICLE_DREAM E3.3 (5 000 images
  COCO, « comme le papier ») et 03/11 (« Recoloration du robot »).
- Nettoyage du masque des 4 caméras fixes : `training/dream/annotations_completes.py`
  (`masque_du_robot`, rayon 6 px) ; ARTICLE_DREAM 03/11.
- Parts visées ≈ 50 % / ≈ 30 % et réglage par tranche : ARTICLE_DREAM E3.7 ;
  `~/dream_article/E3/E3_7/journal_compose.txt` et `journal_final.txt`.
- Secteur tenu 60-100°, azimut autour du point visé, limite « interpolation
  d'azimut » : `convert_to_ndds_camera_libre.py` (docstring, `azimut_deg`,
  `dans_secteur`) ; ARTICLE_DREAM 03/11 (E3.0, formulation du metrologue).
- Apport « point de vue réel tenu » : `training/dream/article/biblio.md`
  (« Recouvrement », menaces 1 et 2 : non trouvé, non exclu ; Panda-Orb).
- Annotations complètes : `docs/SYNTHETIC_DATA.md` (« Annotations NDDS
  complètes ») ; `annotations_completes.py` (`enrichir`).
- Dix tranches, relance du simulateur : `~/dream_article/E3/E3_7/run.yaml`,
  `journal.txt`.
- Références citées (toutes « vérifié : oui » dans `biblio.md`) : DREAM
  (Lee et al., ICRA 2020), RoboPose (Labbé et al., lieu à confirmer), SGTAPose
  (Tian et al., CVPR 2023). Clés BibTeX provisoires : `lee2020dream`,
  `labbe2021robopose`, `tian2023sgtapose`.

### Choix de rédaction

- Aucun chiffre de performance, aucun résultat de transfert : E3.8, E3.9, E2
  ne sont pas mesurés, et le registre ne contient aucune entrée `tenu`.
- Les valeurs de réglage (plages de tirage, probabilités, seuils) viennent du
  code et décrivent la méthode. Les **comptes mesurés sur le jeu** (images
  collectées et converties, effectifs du secteur tenu, parts réalisées de
  fonds et de teintes, taux de masques frais, écart FK des annotations) sont
  des mesures : ils restent en `\todo` jusqu'à une entrée E3.7 / E3.6b
  `tenu`. Valeurs provisoires relevées dans les journaux, pour le metrologue
  seulement : 49 740 images collectées (4 988 + 4 866 + 5 000 + 5 000 +
  4 960 + 5 000 + 4 926 + 5 000 + 5 000 + 5 000) ; fonds 24 597 (49,5 %),
  teintes 14 725 (29,6 %) sur 49 740 ; masques frais 31 012 (62,3 %) ;
  01-08 seuls : 49,3 % / 29,6 % sur 39 740 (identique à ARTICLE_DREAM).
- La phrase « about half / about 30 % » décrit la **cible** de conception,
  pas la part réalisée ; à remplacer par la valeur du registre si le
  metrologue préfère.
- Le résultat négatif E3.0 (pas de transfert sans randomisation d'apparence)
  motiverait bien § III-E, mais il est `a_preciser` : non cité. Le résultat
  négatif de la base mémorisée (registre D, `non_soutenu` pour le 1,81 px,
  « seule l'invalidation est publiable ») est appelé par un `\todo` ; à
  retirer si le metrologue ne le passe pas en `tenu` sous cette forme.
- Pas de « novel » : la nouveauté revendiquée se limite au protocole de vue
  réelle tenue à l'écart, annoncé comme motivation, sans résultat.
- Encodage des cartes (entier contre sous-pixel) et augmentation v5_geo : hors
  de cette section (E3.8 ouvert ; entraînement décrit ailleurs).
- Détails d'ingénierie omis : dérive de cadence, pont `create`/`remove`,
  plantage Ogre sur les lumières, GPU forcé. Une seule phrase sur les tranches
  relancées, nécessaire pour expliquer le réglage des parts par tranche.

### Points ouverts pour la relecture

1. **Paramètres du papier DREAM** (coquille, 3 lumières, YCB, UE4, ~100k) :
   relevés dans le PDF de `papiers/` p. 2-3, mais ils ne figurent pas dans la
   ligne DREAM de `biblio.md`. À y ajouter par `veilleur-biblio` avant
   soumission, ou à retirer de la Table I.
2. **Couleurs des surfaces du monde** : le v3 les randomise par `gz service
   visual_config` en tir-et-oublie ; ARTICLE_DREAM (29/10) dit que la table,
   le sol et les objets restent les mêmes sur toutes les images. Le texte ne
   revendique donc pas de randomisation des surfaces. À confirmer sur une
   planche E3.7.
3. **Pince** : jamais commandée pendant la collecte (aucun appel dans v3 ni
   camera_libre). Son ouverture n'est pas tirée ; valeur à relever dans
   `joints_complets.csv` (`\todo` dans § III-D).
4. **Bras en mouvement entre les vues** (pilote E3.7 : vues d'une « même »
   pose distantes de 13° en médiane) : le texte dit « the arm can still be
   moving » sans chiffre. Les 3 vues ne sont donc pas strictement la même
   pose ; la phrase « the same arm pose then appears at three places » est
   approximative. À nuancer si le relecteur le juge trompeur.
5. **Position des caméras réelles dans la coquille** : le commentaire du
   collecteur (arducam 1,077 m / 80°, SVPRO 0,98 m / 53°) contredit
   ARTICLE_DREAM du 10/11 (`geometrie_camera.py` : SVPRO 70° / 0,98 m / 25°,
   arducam −109° / 0,88 m / 74°). Les deux sources tombent dans les plages
   0,70-1,45 m et 18-86°, d'où la phrase « the distance and elevation ranges
   enclose the poses of our two real cameras » ; la marge basse en élévation
   n'est que de 7° pour la SVPRO selon ARTICLE_DREAM. Corriger le commentaire
   du collecteur. La SVPRO est dans le **secteur tenu** 60-100° (à dire en
   § IV, E3.10 le sépare par caméra).
9. **Origine de la recette caméra** : la phrase « fixed viewpoints … base as a
   constant » vient du docstring du collecteur (mesure du 02/09) et de
   CLAUDE.md ; elle s'appuie sur le résultat négatif D, d'où le `\todo`.
6. **Graines** : le tirage du collecteur n'est pas initialisé (pas de
   `random.seed`), la composition l'est : graine = numéro de tranche
   (01-09) et 4 derniers chiffres du chemin pour les sous-tranches de la 10
   (`~/dream_article/E3/E3_7/scripts/composer_tranches.sh`, `finaliser.sh`).
   La collecte n'étant pas régénérable à l'identique, la reproductibilité
   passe par le md5 du jeu figé (`run.yaml`).
7. **Distorsion** : les labels synthétiques sont projetés sans distorsion,
   les labels réels l'incluent (ARTICLE_DREAM 29/10, « Intrinsèques ») — à
   dire dans la section d'évaluation, pas ici.
8. **« No published evaluation withholds a real viewpoint »** : formulé « to
   our knowledge », conformément à `biblio.md` (« non trouvé, non exclu »).
