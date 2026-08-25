# Pick-and-place en boucle fermée — MyCobot 320 Pi

État au 20 août 2026 — **cycle complet validé en autonome** : balle détectée,
saisie et déposée en carton sans intervention. Document de reprise : ce qui a été
**mesuré** sur le robot réel, ce qui marche, ce qui reste ouvert, et les pièges
dans lesquels on est déjà tombé (deux fois pour certains).

---

## 1. Objectif

Saisir une balle de tennis posée n'importe où sur la planche instrumentée, la
transporter et la déposer dans un carton — sans intervention humaine, la position
de la balle étant fournie par une caméra fixe.

Chaîne : `arducam (vue de dessus) → détection HSV → extrinsèque caméra→base →
XYZ dans base_link → IK différentielle → send_angles`.

---

## 2. Matériel et prérequis

| Élément | Valeur |
|---|---|
| Pi | `10.10.0.224:5005` |
| Bridge sur la Pi | **`scripts/gripper_bridge.py`** — obligatoire, `bridge_pi_simple.py` n'implémente pas `get_pro_gripper_status` |
| Pince | Pro adaptative, `gripper_id=14`, ~1,6 s entre deux ordres |
| Caméra de travail | arducam, intrinsèque `training/calibration/cam_3.npz`, **calibrée en 640×480** — capturer dans ce mode |
| Extrinsèque | `training/calibration/arducam_extrinsic_servo.yaml` (16 coins, RANSAC+LM, LOO) |
| Python | `/usr/bin/python3` pour le robot ; `.venv/bin/python` pour ArUco (OpenCV système 4.6 fait planter `cv2.aruco`) |

⚠ Le bridge de la Pi est **mono-client et bloquant**. Un `bridge_tour` résiduel
(que `real_robot_preflight.sh` laisse tourner) le fige : la connexion TCP est
acceptée mais plus rien ne répond. Vérifier `ps aux | grep bridge_tour` avant de
conclure à une panne robot.

Le bridge répond en texte, pas en JSON : `get_angles` rend
`ANGLES: [42.36, -76.99, ...]`. Parser en conséquence.

---

## 3. Les règles non négociables, toutes mesurées

### 3.1 `send_coords` est écarté

A/B sur cible identique, 267 mm à parcourir : méthode constructeur **247,8 mm
d'erreur finale**, contre **18,2 mm** via `send_angles` + IK différentielle. Les
deux reçoivent `OK` du bridge — **la méthode constructeur échoue en silence**.
Cause : blocage de cardan, la tâche se déroulant entre RY = −78° et −83°, où RX
et RZ sont dégénérés.

Passer par [`scripts/diff_ik.py`](../scripts/diff_ik.py) : `fk_pose` rend une
**matrice de rotation**, `solve_pose` la tient, puis `send_angles`.

### 3.2 Branche IK coude haut (`J3 < 0`)

Toutes les poses historiques sont sur la branche coude bas, plaquée contre la
butée J2 — **marge 0°**, d'où des sauts de branche de 150° sur J4 en boucle
fermée. Filtrer les solutions IK sur `J3 < 0` (marge 23–72°).

### 3.3 Tourner l'orientation cible selon l'azimut

`R_cible = Rz(azimut_cible − azimut_référence) @ R_référence`. Sur 33° d'écart :
résidu IK **0,19 mm** au lieu de **20,0 mm** à orientation figée.

### 3.4 L'affaissement gravitaire tourne AUSSI l'outil

`send_angles` n'atteint pas la consigne : **+1,9° sur J2** de façon reproductible
(plus ~0,5° sur J3/J4). Ce n'est pas qu'une chute en Z — ça **fait aussi pivoter
la bride**. Une correction d'orientation envoyée en boucle ouverte est à moitié
annulée à l'arrivée (mesuré : 4,17° obtenus, repartis à 6,92° après un simple
repositionnement).

**Remède** : réinjecter l'écart articulaire mesuré,
`q_cmd += (q_voulu_IK − q_mesuré)`, et itérer. Convergence en **3 passes**,
0,57 mm de résidu.

Plancher observé : la convergence stagne vers **2,4 mm** sur des déplacements de
l'ordre du millimètre — jeu et résolution des servos. Un ordre de +1 mm a donné
**−1,47 mm**. Ne pas chercher mieux que ~2,5 mm en incrémental fin.

### 3.5 Exiger l'outil vertical coûte de l'allonge

L'allonge nominale est ≈ 390 mm, mais **avec l'outil tenu vertical**, à hauteur
de table, le bras plafonne à **≈ 305 mm**. Vertical veut dire bride à 110 mm pile
au-dessus de la pointe : le bras doit à la fois s'étendre et plonger.

| hauteur de pointe | portée max, outil vertical |
|---|---|
| Z = −5 mm (prise) | ~305 mm |
| Z = 120 mm (largage) | ≥ 320 mm |

Au-delà, le coude est tendu (J3 → 0) et le résidu croît régulièrement : 8–12 mm
à 320, 19–28 mm à 340, 54–63 mm à 381. **Aucune butée articulaire n'est en
cause** — c'est l'enveloppe géométrique. Ni la branche coude bas ni une autre
hauteur ne rattrapent le coup.

### 3.5 bis. Le roulis autour de la verticale est LIBRE — s'en servir

Imposer `R = Rz(azimut − azimut_réf) @ R_réf` fixe l'orientation des doigts dans
le plan horizontal. **Pour une sphère, cette rotation n'a aucune importance** :
c'est un degré de liberté entier, gratuit, qu'on jetait.

L'exploiter (balayer le roulis, garder le premier angle qui résout *toutes* les
hauteurs de l'étape) porte la portée utile à hauteur de table de **300 à 330 mm**,
soit 48 % de la zone ArUco au lieu de 40 %.

Gain constaté en séance : une balle à **330 mm** et le centre d'un carton à
**345 mm**, tous deux refusés à roulis imposé, sont devenus atteignables avec
**+30°** de roulis — cycle complet réussi, descente 1,78 mm en une passe.

À comparer avec l'autre levier envisagé, **incliner l'outil** : seulement 20 mm
de gain (300 → 320 mm), et **au-delà de 20° la portée rediminue**. Le roulis est
meilleur et sans risque, l'inclinaison ne vaut pas la peine.

⚠ Ne vaut que pour un objet à symétrie de révolution. Pour une pièce orientée,
le roulis redevient contraint et la portée retombe à 300 mm.

### 3.6 Ne jamais corriger latéralement doigts en bas

La boucle fermée « descendre puis corriger » fait arriver la première passe
~7 mm à côté en XY, doigts déjà au niveau de la balle : **elle la pousse**
(constaté, la balle a glissé de 33 mm).

**Ordre correct** :

1. converger en XY à **Z ≈ 110 mm**, au-dessus du sommet de la balle (~71 mm) ;
2. mémoriser la correction articulaire apprise ;
3. descendre **d'un seul mouvement** en la réappliquant ;
4. mesurer l'écart XY en bas ; s'il dépasse ~2,5 mm, **remonter à 110 mm**,
   décaler la *cible* de l'écart, redescendre. Jamais de translation latérale au
   ras de la balle.

Raison du point 4 : la correction d'affaissement apprise à 110 mm ne vaut plus en
bas. Mesuré : **0,8 mm d'écart à Z = 110, 8,2 mm à hauteur de prise** — la portée
tombe de 235,2 à 229,4 mm, le bras se rapproche de la base en plongeant.

Baisser la vitesse ne corrige pas ça : la balle est poussée moins fort, mais
poussée quand même.

---

## 4. Procédure de saisie qui fonctionne

```
détection balle (3 captures, dispersion < 3 mm, circularité > 0,7)
  → contrôle portée ≤ 300 mm, sinon refuser
  → approche      (x, y, Z=170)
  → convergence   (x, y, Z=110)      3 passes, garde la correction
  → descente      (x, y, Z=-5)       1 mouvement, itérée si écart XY > 2,5 mm
  → fermeture     pro_gripper_angle 20
  → CONTRÔLE      get_pro_gripper_status == 2, sinon arrêt
  → remontée      (x, y, Z=170)
  → transfert     (carton, Z=170)     contrôle du statut à chaque point
  → largage       (carton, Z=120)     pro_gripper_open
```

Vitesse `send_angles` : **25**. À 40 la balle est bousculée, à 60 c'est trop
brutal.

La pince **cale sur l'objet** à un angle qui n'est pas celui commandé (53–54
mesuré pour une consigne de 20). Viser plus bas ne serre pas davantage ; le seul
levier est `set_pro_gripper_torque`. **`get_pro_gripper_status` est la seule
confirmation de prise valable** — un statut *inconnu* n'est pas une vérification.

Codes de statut : `0` en mouvement · `1` rien saisi · `2` objet saisi · `3`
objet lâché.

---

## 5. Pièges de calcul — on y est tombé, deux fois pour l'IK

### 5.1 Le solveur IK doit passer un auto-test

`solve_pose` est un solveur local. Deux façons opposées de se tromper, toutes
deux rencontrées :

* **enchaîner les amorces** (chaque solution amorce la cible suivante) → cascade
  de faux « hors d'atteinte » dès qu'une solution dérape ;
* **éventail d'amorces purement synthétiques** → rate le bon bassin et déclare
  inatteignable un point que le robot **a physiquement atteint dix minutes plus
  tôt**.

Règle : éventail large **incluant les poses réellement mesurées sur le robot**,
`max_joint_step_deg=3.0` (8.0 oscille), et un **auto-test** qui vérifie que
chaque pose atteinte est retrouvée depuis sa propre pose cartésienne. Une falaise
nette dans un balayage de portée (300 mm passe à 0,00 mm, 310 échoue totalement)
est le signe d'un solveur cassé, pas d'une limite mécanique.

### 5.2 L'extrinsèque dérive — recalculer le PnP, ne pas rustiner

Constaté : **21,8 mm d'erreur moyenne** sur les quatre marqueurs. J'ai d'abord
corrigé par une **translation rigide** ajustée sur les marqueurs (résidu ramené à
3,9 mm) en me disant que l'origine — planche glissée ou caméra déplacée —
n'importait pas. **C'était faux, deux fois :**

* le PnP recalculé montre que la caméra a bougé de **13,7 mm ET tourné de 1,35°** ;
* **une rotation ne se corrige pas par une translation.** Le rattrapage rigide est
  juste *aux marqueurs* et faux *entre* eux — or l'objet est entre les marqueurs.

Recalculer l'extrinsèque par `solvePnP` sur les 4 centres de marqueurs ramène
l'erreur à **0,33 mm** (contre 21,8 brut, 3,9 rustiné). C'est deux lignes de
code ; il n'y a aucune raison de rustiner.

Garder un **contrôle de santé** à chaque détection : si l'écart aux marqueurs
remonte au-dessus de ~5 mm, la caméra a rebougé, recalculer avant de commander.

⚠ Le recalcul exige les **4** marqueurs. Le bras en masque facilement un (le 23
depuis la pose d'observation) : dégager avant de recalibrer.

### 5.4 Un client TCP qui lit la mauvaise réponse fait planter le bras

Le pont de la Pi répond en texte, une ligne par commande — sauf quand il en
répond deux. Un client qui lit « une ligne » sans vérifier qu'elle correspond à
la commande envoyée se **désynchronise** : la lecture suivante récupère la
réponse précédente. Pire, une boucle de réessai qui **renvoie la commande** à
chaque tentative aggrave le décalage au lieu de le résorber.

Conséquence réelle le 20/08 : lecture d'une pose périmée à **Z = 585 mm**, dont
le XY a été repris pour commander Z = −5, soit un plongeon de 590 mm. Le bras a
forcé contre la table, les servos ont tiré le courant, **la Pi a décroché et le
bras est tombé**.

Deux règles qui en découlent :

1. **Vider le tampon avant chaque envoi**, et refuser toute réponse dont
   l'étiquette ne correspond pas (`ANGLES`, ici), plus un contrôle de domaine
   (|angle| ≤ 200°).
2. **Aucune trajectoire n'est envoyée sans avoir été simulée**, garde au sol
   vérifiée sur tout le chemin. Ce contrôle a ensuite refusé deux dégagements qui
   faisaient plonger la pointe à −41,8 mm.

Piège du garde-fou lui-même : **le seuil ne peut pas être plus exigeant que
l'état de départ**, sinon il refuse aussi les *remontées* quand la pince est déjà
en position basse. Ce qu'il faut interdire, c'est de **descendre** en chemin :
`seuil = min(GARDE_MIN, garde(pose_courante) − 2 mm)`.

### 5.5 Une seule tâche doit lire la caméra

Le tableau de bord fait tourner les mouvements robot dans un fil séparé — ils
durent plusieurs secondes et figeraient l'interface. Première version : ce fil
appelait `cap.read()` pour rafraîchir sa propre image de détection, pendant que
le minuteur graphique lisait la **même** `VideoCapture` toutes les 60 ms.

Deux fils sur une capture GStreamer : elle se bloque sans jamais rendre la main.
Le symptôme trompe — l'interface reste **vivante** (les images défilent, la
position s'affiche en direct), seule l'étape ne se termine jamais et les boutons
restent grisés. On soupçonne le robot ; le pont répondait en 0,02 s.

Règle : **un seul fil touche `VideoCapture`**, le fil graphique. Le détecteur
consomme un tampon horodaté que ce fil alimente, et attend au plus 3 s d'y
trouver 3 images fraîches (< 1,2 s) et cohérentes (dispersion < 3 mm) — sinon il
refuse la cible. Toute étape est ainsi **bornée dans le temps** : elle peut
échouer, elle ne peut plus pendre.

### 5.3 Ne pas valider un déport d'outil sur le point qui l'a produit

`scripts/tool_offset.json` a été ajusté sur **un seul point enseigné à la main**.
Il le reproduit donc à 0,01 mm **par construction** : ce point n'arbitre rien.
Piège circulaire, on y est tombé.

Sa **longueur** est fiable (110,4 mm). Sa **direction** ne l'est pas : ses
composantes latérales (−8,9 et +11,9 mm) sont autant l'imprécision du geste que
la géométrie du montage. Deux références plausibles diffèrent de **7,73°** :

* le déport du fichier ;
* `-X` de la bride, l'axe mécanique nominal.

Arbitre géométrique : **seul `-X` bride à la verticale met les deux doigts de
niveau** (Y et Z de la bride deviennent exactement horizontaux). C'est donc lui
qu'il faut viser pour une descente verticale.

⚠ La verticale exacte n'est pas toujours atteignable : à certaines poses J4
sature à sa butée (145°) avant, et l'IK part alors en reconfiguration complète
(55° sur J3, 53° sur J6) en laissant 7,3° d'erreur. Résoudre par paliers et
s'arrêter à ~2° résiduels.

---

## 6. Le biais latéral pince ↔ modèle — RÉSOLU

**Mesuré à 17,5 mm, corrigé, cycle complet validé en autonome le 20/08.**

Symptôme : la descente amenait la pointe à 0,5 mm de la position détectée de la
balle *dans le modèle*, et la pince se refermait pourtant sur **un quart de balle
de côté**. Ni la descente ni la détection n'étaient en cause : le biais était
entre le repère du modèle et la réalité.

### Ce qui l'a mesuré : la balle TENUE, jamais lâchée

Deux méthodes ont été essayées, une seule marche.

**Par dépose — FAUSSE.** Le robot pose la balle à une position commandée, la
caméra dit où elle est tombée. Résultat : 25,5 mm d'écart, **de signe opposé** à
ce que l'opérateur observait. Cause : la balle **roule** après le lâcher (les
doigts de la pince Pro s'écartent en s'ouvrant et la chassent). On mesure le
roulement, pas le biais.

**Par balle tenue — JUSTE.** On ne lâche jamais la balle : sa position vue par la
caméra **est** le vrai milieu des doigts. La caméra n'étant pas à l'aplomb
(centre à Y = −182 mm), à ~150 mm de hauteur la balle serrée dépasse de l'ombre
du bras et reste détectable. Aucun jugement visuel, rien qui puisse rouler.

Mesures à deux azimuts (les deux autres poses : balle masquée par le bras) :

| azimut | écart monde | écart repère bride |
|---|---|---|
| +13° | (+4,26 · −16,93) | (−0,26 · +10,95 · +13,60) |
| −2° | (+2,02 · −17,71) | (−0,11 · +9,35 · +15,18) |

Norme **17,5 mm**, dont la composante **radiale ne vaut que 0,34 mm** : l'écart
est presque entièrement **tangentiel**. C'est un décalage *sur le côté*, ce qui
explique pourquoi une correction en avant/arrière ne le rattrapait jamais.

### Correction appliquée

```
deport avant   [-109.43,  -8.88,  11.91]   longueur 110.43 mm
deport corrige [-109.62,  +1.27,  26.30]   longueur 112.73 mm   axe tourne de 8.97 deg
```

La hauteur de prise validée reste inchangée (Z = −5,5 contre −5,0), cohérent avec
un biais purement latéral.

**Résultat immédiat, premier essai, sans intervention** : recalage XY 0,94 mm,
descente verticale **1,50 mm en une seule passe**, fermeture `statut 2`, transfert
sur 206° de balayage J1 avec prise tenue, dépôt en carton vérifié par image.

### Réserve

Les deux mesures valides ne sont séparées que de **15° d'azimut** — trop peu pour
prouver formellement que le biais est constant dans la bride plutôt que dans le
monde (dispersion 0,80 mm contre 1,12 mm, l'écart penche pour la bride sans le
démontrer). À revalider à des azimuts éloignés. La saisie réussie à +19°
d'azimut, alors que la mesure venait de +13° et −2°, est un premier signe
favorable mais pas une preuve.

---

## 6 bis. Ancien texte — pourquoi une première mesure avait échoué

**Conservé : c'est le piège qui a coûté le plus de temps.**

Symptôme : la descente amène la pointe à **0,5 mm** de la position détectée de la
balle *dans le modèle*, et pourtant la pince se referme physiquement sur **un
quart de balle de côté** (~18 mm). Le biais n'est donc ni dans la descente
(réglée), ni dans la détection (dispersion 0,15–0,95 mm sur 3 captures), mais
entre **le repère du modèle et la réalité** — soit la direction du déport d'outil
(§ 5.3), soit un biais résiduel de l'extrinsèque, soit les deux.

Une première tentative de mesure a été **invalidée** : elle supposait la balle
centrée entre les doigts sans confirmation de l'opérateur, et mélangeait le biais
recherché avec les 8 mm de dérive de descente non encore diagnostiqués. Elle
donnait un déport corrigé `[-109.00, -16.83, 9.25]` (axe tourné de 4,35°) —
**ne pas le réutiliser tel quel**, il a dégradé la visée.

Protocole propre, à faire maintenant que la descente est fiable :

1. amener la pince **ouverte** à hauteur de prise sur la balle ;
2. la déplacer par petits pas (`nudge.py`) jusqu'à ce que **l'opérateur
   confirme** que la balle est au milieu des deux doigts — sa confirmation
   explicite, pas une déduction sur photo ;
3. l'écart entre la pointe du modèle à cette pose et la position caméra de la
   balle **est** le biais cherché ;
4. le convertir dans le repère bride (`R.T @ erreur_monde`) pour qu'il vaille à
   toutes les orientations, et l'ajouter au déport ;
5. **revalider sur au moins trois positions de balle d'azimuts différents** avant
   d'écrire quoi que ce soit dans `scripts/tool_offset.json`.

Attention pendant les pas latéraux : le Z dérive tout seul (−4,4 → −8,1 → −11,4
sur trois pas de ~11 mm), l'affaissement augmentant avec le recul. Recaler la
hauteur en même temps.

---

## 6 ter. Couverture réelle : 40 % de la zone ArUco

Question naturelle une fois la saisie autonome acquise : « le bras rattrape-t-il
la balle où que je la pose entre les marqueurs ? » **Non — sur 40 % de la zone
seulement**, et c'est géométrique, pas réglable.

La zone délimitée par les quatre ArUco (X de 97 à 535 mm, Y de −179 à +215 mm)
couvre des portées de **105 à 570 mm**. Le bras, outil tenu vertical à hauteur de
table, plafonne à **305 mm** (§ 3.5). Donc : toute la moitié proche, rien de la
moitié lointaine.

| levier | portée | couverture |
|---|---|---|
| roulis imposé | 300 mm | 40 % |
| outil incliné 10–20° | 320 mm | 45 % |
| **roulis libre** (§ 3.5 bis) | **330 mm** | **48 %** |

**Rapprocher la planche ne sert à rien : le robot est boulonné dessus**, ils
bougent ensemble et la géométrie relative est inchangée. Les tags sont fixes eux
aussi. Il ne reste donc, pour couvrir davantage, que de surélever le robot ou le
monter sur un rail.

En attendant, la bonne pratique est de **refuser proprement les cibles trop
lointaines** — ce que fait déjà `detecte.py` — plutôt que de laisser croire à une
couverture totale.

---

## 6 quater. Ce qui a changé le 24 août 2026

### L'objet en main interdit tout retour au ramassage

Le cycle a tourné la balle en pince sans jamais déposer. La cause n'était pas la
vision : `TRANSFERT` échouait, renvoyait vers `ECHEC`, donc `ATTENTE`, donc
`DEGAGEMENT` — le bras repartait chercher une balle qu'il tenait déjà.

La garde est **unique et dans `MachineEtats.pas()`**, pas répartie dans les
états : toutes les sorties d'erreur passent par là, c'est le seul point où on
puisse l'arrêter. Tant que `get_pro_gripper_status == 2`, les états de ramassage
sont détournés vers `RECHERCHE_CARTON`.

| état neuf | rôle |
|---|---|
| `RECHERCHE_CARTON` | cherche le carton **en hauteur, sans lâcher** (balayage J1 aux poses d'observation, pointe à 172 mm) ; à défaut, dernière position connue persistée dans `scripts/carton_position.json` |
| `ECHEC_PORTANT` | carton introuvable ou hors d'atteinte : **garde l'objet** et s'arrête, même en automatique |

Seule une perte de prise — statut 1 ou 3 — relance la saisie. Et le carton est
jugé **avant** de saisir : `DETECTION` refuse de démarrer un cycle si le carton
n'est ni vu ni mémorisé à portée. C'est le seul moment où le bras est dégagé.

### Détecter le carton : la couleur ne suffit pas

| zone | H | S | V |
|---|---|---|---|
| intérieur du carton | 14 | 171 | 60 |
| planche en bois | 15 | 187 | 84 |

Même teinte, même saturation. L'ancien seuil `HSV_CARTON` prenait **la planche
entière** pour un carton — tache de 30 000 px², centroïde à 300 mm de la vraie
boîte. Ce qui distingue le carton, c'est que son **ouverture est un creux plus
sombre que le bois, entouré de brun sur tout son pourtour** :

1. masque du plateau projeté depuis les 4 ArUco, marqueurs découpés ;
2. `V` inférieur d'au moins 12 à la médiane locale (noyau 61 px) ;
3. côtés réels 55–330 mm, **anneau brun ≥ 60 %** — c'est lui qui élimine les
   marqueurs (bordure blanche) et l'ombre du bras (silhouette blanche) —
   contraste anneau/creux ≥ 8 ;
4. visée au centre de l'enveloppe convexe du creux ;
5. repli couleur, en refusant toute tache couvrant plus de 35 % de la planche.

`SuiviCarton` stabilise ensuite : lissage tant que la détection reste à moins de
60 mm, **6 images concordantes** avant d'admettre un déplacement, garde de
taille ±45 %, péremption 4 s. Le rectangle affiché est la position suivie.

⚠ **Pas de contrôle de dernière seconde avant le largage.** Il a été essayé et
retiré : à cet instant le bras est au-dessus du carton et le masque, la
détection saute de 55 à 171 mm, et le cycle repartait en boucle sans déposer.
Une détection de carton à moins de 120 mm sous la pointe est ignorée.

### Viser le milieu, sans renoncer au carton quand il est hors d'atteinte

Le milieu du carton mesuré à 363 mm est inatteignable **à toutes les hauteurs de
largage** — 100, 120, 140 et 160 mm, aucune ne passe : la limite est
horizontale, pas verticale. La même ouverture a son bord proche à 290 mm.

Les points de largage sont donc classés **par distance au milieu**, reculés des
parois d'autant que l'ouverture le permet (jamais plus des trois quarts du rayon
inscrit, sinon seul le centre géométrique survit — justement le point le plus
lointain). Mesure : largage à 2–22 mm du milieu, 31–43 mm de marge aux parois.

### La portée réelle est de 350 mm, pas 335

Roulis libre, outil vertical, sur les **trois** hauteurs du cycle (transfert
170, survol 110, prise −5) :

| portée | 335 | 344 | 350 | 355 | 360 |
|---|---|---|---|---|---|
| roulis qui résout | +30 | +30 | +60 | +60 | aucun |

(360 mesuré sur prise + survol seulement — il échoue déjà là.)

`PORTEE_MAX` valait 335 et refusait des balles atteignables — une à 343,6 mm.
Porté à **355**. Incliner l'outil n'ajoute rien ici : testé de +10 à +30°,
aucune solution (−10 et −20° passent, mais 0° passe déjà).

### La SVPRO assiste l'arducam

L'arducam reste la source du X/Y : elle regarde presque à la verticale. La SVPRO
ne fournit que **la hauteur**, par triangulation des deux rayons, en
remplacement de l'hypothèse « centre de balle à 35 mm » — hypothèse dont
l'erreur se transforme en erreur XY, d'autant plus grande que le rayon est
oblique, donc aux bords du plateau.

Mesure du 24/08 : écartement des deux rayons **2,3 mm** (validation croisée des
deux extrinsèques), hauteur réelle **29,9 mm**, correction XY **2,01 mm** à
324 mm de portée. Relais complet quand le bras masque la vue de dessus — le
journal montrait `balle vue sur 0 image(s)` à chaque recalage. Appui refusé si
les rayons s'écartent de plus de 25 mm.

⚠ La SVPRO est calibrée en **800×600** et lue en 640×480 : sa matrice
intrinsèque doit être rescalée (`camera_registry.load_intrinsics`). Avec la
matrice brute du `.npz`, les marqueurs se reprojettent à 200 mm de leur
position. Son extrinsèque, dérivée à 16,5 mm, a été recalibrée sur 16 coins :
**1,57 mm**, au niveau de l'arducam (1,24 mm).

### Temps de cycle : le coupable n'était pas le robot

**Chaque mouvement attendait 22,7 s.** L'arrivée était jugée sur l'atteinte de
la consigne à 1,2° près, or l'affaissement laisse un écart permanent d'environ
1,9° sur J2 : le test n'était jamais satisfait et l'attente allait au bout de sa
patience. Mesuré **identique à vitesse 25 et à vitesse 50**, ce qui prouve que
le temps ne venait pas du robot. L'arrivée se détecte désormais à
l'**immobilité** du bras : **22,7 s → 1,35 s par mouvement**, vitesse inchangée.

Trois autres postes, tous mesurés :

- **le roulis.** Un roulis qui échoue épuise les 22 amorces — 5 s ; celui qui
  marche répond en 40 ms. Le roulis retenu est réessayé en premier, rangé **par
  bande d'allonge de 25 mm** : celui appris à 257 mm échoue à 357 mm. La pose du
  carton est mise en cache tant qu'il n'a pas bougé de plus de 12 mm.
  **16,7 s → 0,04 s** au retour sur une position connue.
- **l'affaissement.** Il est reproductible : le redécouvrir coûtait deux passes
  de convergence, donc deux mouvements (passe 1 à 10,79 mm, passe 2 à 4,15,
  passe 3 à 0,38). La correction est retenue sur disque et réinjectée dès la
  première passe.
- **la seconde de stabilisation** après chaque mouvement ne sert qu'avant une
  *mesure* : supprimée sur les transits, gardée sur la convergence et la
  descente.

Un chronomètre est en place : durée par état dans le journal, et à chaque
retrait une ligne `CYCLE COMPLET : XX s — les trois étapes les plus coûteuses`.

### Descendre depuis le bras dressé

La pointe au repos est à 518 mm, la cible d'approche à 110 : le garde-fou
anti-plongeon (220 mm en un seul ordre, né du plongeon de 590 mm du 20/08)
refusait, et le cycle bouclait sur `APPROCHE REFUSE`. Le mouvement est découpé
en étapes interpolées dans l'espace **articulaire**. Deux mesures qui imposent
cette forme :

- descendre par paliers **au-dessus de la cible** ne marche pas : à Z = 340 mm,
  l'outil ne peut pas être tenu vertical, l'IK n'a aucune solution ;
- le nombre d'étapes ne se déduit pas de la chute totale : l'interpolation
  articulaire n'est pas monotone en hauteur, la pointe **monte d'abord à
  562 mm** avant de plonger. Il se règle sur le profil réel.

---

## 6 quinquies. Tri par catégorie — 24 août 2026 (soir)

La tâche est passée de « une balle dans un carton » à **trois classes d'objets
et deux cartons de destination** : `scotch` → petit carton, `balle` et `robot`
→ grand carton. Les trois ont été prises et déposées sur le robot réel.

### Reconnaître sans dépendre de l'éclairage

La couleur ne sépare pas ces objets. À l'exposition 75 de l'arducam, le rouleau
de scotch **bleu** se lit `H15 S90 V48` — indistinguable du bois sombre. Trois
signatures géométriques ou structurelles ont donc été retenues, chacune mesurée :

| Classe | Signature | Mesure du 24/08 |
|--------|-----------|-----------------|
| `scotch` | **anneau** : un trou dans le contour | trous de 28 et 65 px ; aucun autre objet de la planche n'en a |
| `robot` | **noir désaturé**, ≥ 60 mm de long | S=44 contre S=170 pour le bois même à l'ombre ; 82×134 mm contre 33×39 et 38×43 pour les rouleaux |
| `balle` | disque jaune plein | détecteur historique, inchangé |

Deux pièges, tous deux corrigés :

* **la fermeture morphologique bouchait le trou du rouleau.** Le trou de
  l'anneau bleu ne fait que 28 px et disparaît dès un `CLOSE 3×3`. Le rouleau
  tombait alors dans la catégorie `robot` et repartait vers le mauvais carton.
  Il n'y a plus aucune fermeture sur la détection d'objets ;
* **« sombre » ne suffit pas pour le robot.** Le veinage du bois descend sous
  V=60 et fournissait quatre faux positifs sur une planche vide. C'est la
  saturation qui tranche.

### Deux cartons, classés par leur robe

Les deux ouvertures ont des aires trop proches — et surtout dépendantes de
l'angle de vue — pour servir de critère : le classement basculait d'une image à
l'autre. C'est la **robe** qui les sépare, mesurée sur un anneau de 25 px autour
de l'ouverture : le **grand** est brun (`S174 V87`, 2 % de pixels sous V=60), le
**petit** est noir (`S148 V52`, 59 %).

Le sens de cette règle a été inversé une fois puis remis d'aplomb par trois
mesures concordantes : la consigne d'origine de l'opérateur, les ouvertures
(138×202 mm pour le brun contre 62×113 mm pour le noir) et l'essai réel — le
robot dirigé vers `grand` avait atterri dans le petit carton. Deux tests
verrouillent désormais le sens.

Chaque carton a son propre `SuiviCarton` : déplacer l'un est rattrapé en 0,12 s
sans que la cible de l'autre bouge.

### Le carton fantôme au milieu de la table

Trois fois de suite, la machine a visé une ouverture inexistante — (211, 1),
(205, −16), puis (212, 6) prise sur le fait à 141 mm du bras, 18 410 mm² — et y
a lâché l'objet, pendant que le suivi affichait le vrai carton à (364, 161).

C'était **l'ombre du bras**. Elle est sombre, rectangulaire, cerclée de bois
brun : elle passe tous les filtres de forme et de taille. Et surtout elle suit le
bras image après image, donc elle **se confirme aussi bien qu'un vrai
déplacement** — l'abaissement du seuil de confirmation à 2 images, qui rend le
suivi réactif, la rend aussi crédible.

Deux corrections, la seconde étant la vraie raison de la persistance du défaut :

1. **un carton ne peut plus être localisé à moins de 200 mm du bras**, distance
   mesurée au bras **entier** (`fsm.distance_au_bras`) et non à sa pointe : ce
   sont les segments qui portent l'ombre, bien plus loin que leur bout ;
2. **la mémoire sur disque contenait le fantôme.** `scripts/carton_position.json`
   gardait (207, 14) comme dernière position connue, et `RECHERCHE_CARTON` y
   retombait même sans détection live — ce qui survivait aux redémarrages. Le
   fichier est désormais **scindé par carton** et n'est écrit que sur une
   détection propre.

### Hauteur de prise : par catégorie ET par régime

La saisie du scotch échouait systématiquement alors que le recalage donnait
**0,50 mm** et la descente **1,30 mm**. La position était juste ; la hauteur ne
l'était pas, et pour deux raisons distinctes :

* **outil vertical**, viser −5 mm comme la balle fait taper les doigts sur la
  planche avant qu'ils se referment. La balle fait 66 mm et les tient écartés,
  un rouleau couché 22 mm ;
* **outil couché** (au-delà de 355 mm), la pointe visait `Z = 25` — la
  mi-hauteur de la **balle**. Sur un objet plat de 22 mm, la pince se refermait
  entièrement au-dessus de lui.

D'où `Z_PRISE_PAR_CLASSE`, qui donne les deux hauteurs de chaque classe :
balle (−5 / 25), scotch (2 / 11), robot (2 / 10). Et un essai raté descend
ensuite de 4 mm au lieu de refaire le même geste — trois tentatives identiques
donnaient trois échecs identiques.

### La SVPRO nomme plus rien

Laissée libre de classifier depuis sa vue oblique, elle contredisait l'arducam :
paroi de carton prise pour le robot, plateau étiqueté « petit carton », anneaux
jamais vus. Ce n'est pas un réglage — c'est la géométrie : projeter sur un seul
plan horizontal n'est juste que vue de dessus, et un anneau de 48 mm vu en
rasant perd son trou.

Elle rend donc des **taches sans nom**, et chacune hérite du nom que l'arducam a
donné au même endroit (appariement à 90 mm). Une tache sans équivalent côté
arducam n'est pas utilisée : la caméra d'appui ne peut plus inventer un objet ni
un carton. Les deux vues affichent enfin les mêmes étiquettes.

### Piste fermée : allonger la portée verticale

`PORTEE_VERTICALE_MAX = 355` avait été mesurée en exigeant aussi `Z_TRANSFERT`
au-dessus du point de prise, contrainte sans objet sur une pièce posée à plat.
En ne demandant que le survol et la prise, **la limite ne bouge pas d'un
millimètre** : 350 mm passe, 360 non, dans les deux cas. C'est mécanique. Ne pas
re-tenter.

---

## 6 sexies. Identifier les deux cartons — 25 août 2026

Le tri était juste sur tout sauf sur un point : **lequel des deux cartons est le
grand**. Le 24 au soir le scotch bleu, dirigé vers `petit`, a atterri dans le
grand. Trois pistes avaient été retenues — dimensions extérieures, hauteur des
parois par les deux caméras, marqueur ArUco. Voici ce que chacune a donné.

### Le rebord est à 83 mm, pas 60

Triangulation de l'ouverture du grand carton entre l'arducam et la SVPRO :
**Z = 82,9 mm**, écart entre les deux rayons **11,8 mm** — la mesure est bonne.
Or `HAUTEUR_CARTON = 60` était le plan sur lequel toute la géométrie des cartons
se projetait. L'erreur n'est pas anodine : entre Z = 0 et Z = 100, le centre
projeté d'un carton se déplace de **50 mm**, et son grand côté passe de 233 à
206 mm.

### Piste fermée : la hauteur par les deux caméras

Elle tient sur le carton proche et **pas** sur le lointain. La SVPRO, très
oblique, n'en voit pas l'ouverture mais la **paroi du fond par la tranche** : son
contour est une bande de 42 × 122 px, et son centroïde est à 40 mm de celui de
l'arducam. La triangulation rend alors **Z = −27 mm** — sous la table — avec
38,7 mm d'écart entre les rayons.

Deux rattrapages ont été essayés et écartés :

- **apparier par la distance entre rayons** plutôt que par les positions : sans
  gabarit métrique il n'y a plus rien pour écarter les mauvaises paires, et les
  centroïdes ne portent de toute façon pas sur le même point physique ;
- **maximiser le recouvrement** du contour arducam reprojeté dans la SVPRO, en
  balayant la hauteur de 0 à 140 mm : la courbe est plate sur le carton proche
  (0,24 à 0,30) et monotone décroissante sur le lointain — aucun maximum, donc
  aucune hauteur désignée.

### Piste fermée : les dimensions

Les deux cartons posés côte à côte mesurent **160 × 214 et 144 × 205 mm**, soit
5 % d'écart — sous le bruit de détection, qui fait varier l'aire relevée de
270 à 379 cm² d'une image à l'autre sur le même carton. Aucun gabarit ne les
sépare.

### Ce qui bloquait vraiment la détection

Avant même la question du nom, un carton sur deux n'était pas vu. Sur
20 images consécutives, bras dégagé : **2/20 pour l'un, 14/20 pour l'autre**, et
l'étiquette basculait d'une image à l'autre.

La cause est un seul seuil. Le carton de gauche — 4400 px, anneau brun à 0,79,
contraste 40, remplissage 0,74, un carton parfait — était rejeté pour **10 mm de
trop** : 220 mm contre `COTE_CARTON_MM[1] = 210`. Et ces 220 mm étaient mesurés
au plan supposé ; au vrai rebord il n'en fait que 214. **Un gabarit ne doit pas
être plus serré que l'incertitude sur le plan où on le mesure.** Porté à 260 mm,
sa vraie tâche restant d'écarter la planche entière (450 mm) et les petits
objets.

### Trois sources de nom, de la plus sûre à la moins sûre

`Vision.cartons` décide dans cet ordre :

1. **le marqueur ArUco** collé sur un rabat — `id 10` grand, `id 11` petit,
   45 mm de côté. Son plan est celui du rebord : il donne le nom *et* la
   hauteur. `~/marqueurs_cartons.png` est la feuille à imprimer à 100 %.
2. **la continuité** (`CONTINUITE_CARTON = 150 mm`) — un carton déjà nommé garde
   son nom tant qu'il reste près de là où on l'a vu. C'est ce qui permet de le
   déplacer à la main sans qu'il échange son nom avec l'autre.
3. **la robe puis le gabarit** — réduits au rôle d'amorce, puisqu'on vient de
   voir qu'ils ne tranchent pas.

`cv2.aruco` fait **segfaulter** l'OpenCV 4.6 du système, où tourne le tableau de
bord. La détection est donc déportée dans un processus du venv qui reste ouvert
et reçoit les images brutes par un tube ([`scripts/aruco_service.py`](../scripts/aruco_service.py)) :
**4,5 ms par image** aller-retour compris, contre ~1 s si on relançait un
interpréteur à chaque fois. Service absent, la géométrie reprend la main sans
bruit — c'est un supplément d'information, pas une dépendance.

Résultat sur 20 images consécutives : **20/20 et 20/20**, étiquette stable,
tremblement du centre 0,2 mm et 7,9 mm.

### Le lâcher se cale sur le rebord mesuré

`fsm.z_largage` rend `max(Z_LARGAGE, rebord + GARDE_LARGAGE)` au lieu de la
constante. Avec un rebord réel à 83 mm et `Z_LARGAGE = 100`, la garde n'était que
de 17 mm — et négative pour un carton plus haut.

### Un carton déplacé n'importe où reste atteignable

Balayage IK de tout le plateau, sans mouvement, X de 200 à 480 mm et Y de −240 à
+240 mm par pas de 40 mm : **aucun trou**. Toute position en deçà de
`PORTEE_CARTON_MAX = 460 mm` admet une solution de largage, l'inclinaison de
l'outil passant de 0° au centre à −60° dans les coins. La limite est la portée,
pas la géométrie.

## 7. Autres points ouverts

- **Extrinsèque SVPRO invalide** (32–149 mm de dérive, caméra déplacée). Elle ne
  voit que 2 des 4 marqueurs : la réorienter avant toute recalibration. La SVPRO
  reste utile en **vue de côté qualitative** — c'est elle qui a montré que la
  pince se fermait à côté de la balle, ce que la vue de dessus ne pouvait pas
  voir (l'avant-bras masque les doigts).
- **Serrage par couple** : remplacer l'angle par `set_pro_gripper_torque`.
- **Portage dans le nœud** : la rotation d'orientation selon l'azimut, la
  convergence en boucle fermée et la descente itérative vivent aujourd'hui dans
  des scripts d'essai, pas dans
  [`visual_servo_node.py`](../mycobot_gateway/mycobot_gateway/visual_servo/visual_servo_node.py).

---

## 8. Scripts d'essai

Ils vivent dans le scratchpad de session (`/tmp/...`, **effacé au redémarrage**).
À porter dans le dépôt s'ils doivent survivre.

| Script | Rôle |
|---|---|
| `mc.py` | client TCP du bridge (parse le format texte), FK, pointe, inclinaison |
| `ik_multi.py` | IK à amorces multiples + **auto-test** contre les poses réellement atteintes |
| `locate_box.py` | extrinsèque, projection pixel↔base, contrôle des 4 ArUco et correction rigide |
| `detecte.py` | détection balle 3 captures avec contrôles dispersion/circularité/portée |
| `cycle.py` | cycle complet, `--dry-run` pour valider le plan sans bouger |
| `nudge.py` | petit déplacement latéral à hauteur de prise, affaissement compensé |
| `converge.py` | convergence en boucle fermée sur une pose complète |

---

## 9. À lire aussi

- [`CLAUDE.md`](../CLAUDE.md) § « Commande cartésienne — `send_coords` est écarté »
- [`docs/DEPLOYMENT.md`](DEPLOYMENT.md) — IP, VNC, bridge
- [`scripts/diff_ik.py`](../scripts/diff_ik.py) — docstring sur le pourquoi de l'IK maison
- [`scripts/tool_offset.json`](../scripts/tool_offset.json) — déport actuel et son historique
