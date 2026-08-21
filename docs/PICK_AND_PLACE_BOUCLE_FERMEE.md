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
| Pi | `10.10.0.221:5005` |
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
