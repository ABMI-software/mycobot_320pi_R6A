# Méthodologie des essais de précision — myCobot 320 Pi

Ce document répond à une question précise : **la méthode employée pour mesurer
l'écart du bras est-elle valable, et pour quoi ?** Il sépare les sept types
d'essais qu'on peut mener, dit lesquels sont faits, lesquels ne le sont pas, et
lesquels exigent un instrument externe.

Données : [`precision_campagne_2026-09-09.xlsx`](precision_campagne_2026-09-09.xlsx)
· [`PRECISION_MYCOBOT_320PI.md`](PRECISION_MYCOBOT_320PI.md)
· [`repetabilite_4directions_2026-09-09.csv`](repetabilite_4directions_2026-09-09.csv)

---

## 1. Ce que mesure réellement `e = FK(q_lu) − P_cible`

Aucun instrument externe n'a été utilisé : ni règle, ni pied à coulisse. La
position vient de la cinématique directe appliquée aux angles **relus** :

```
P_mesurée = FK(q_lu)
e         = P_mesurée − P_cible
e_3D      = √(ex² + ey² + ez²)
```

### La décomposition

```
P_cible --IK--> q_cmd --servo--> q_phys --codeur--> q_lu --FK_nominale--> P_mesurée
```

d'où :

```
e = [FK(q_lu) − FK(q_cmd)]  +  [FK(q_cmd) − P_cible]
     erreur de suivi servo       résidu d'IK
```

Le résidu d'IK a été mesuré sur les cinq poses du test G : **0,004 à 0,016 mm**.
Négligeable. Il reste donc :

$$e \simeq J(q)\,(q_{lu} - q_{cmd})$$

> **`e` est l'erreur de suivi des servos, projetée en cartésien par la
> jacobienne. Ce n'est pas la position physique de la pointe.**

| contenu dans `e` | absent de `e` |
|---|---|
| erreur de suivi du servo | écart entre `q_lu` et l'angle réel de l'articulation |
| affaissement gravitaire (le servo se cale décalé sous charge) | erreurs du modèle : longueurs de segments, offsets, zéros |
| répétabilité du codeur | jeu de réducteur en aval du codeur |
| | flexion des bras et de l'outil sous charge |
| | erreur de définition du TCP |

### Pourquoi la méthode reste valable pour la répétabilité

Sur un **retour répété au même point**, tous les termes systématiques de la
colonne de droite s'annulent : ils sont identiques à chaque passage. Ne
subsistent que les termes **non répétables** — jeu et flexion.

D'où le statut exact de nos chiffres : **borne inférieure**. Ils peuvent
sous-estimer la dispersion réelle, jamais la surestimer.

Sur une **précision absolue**, rien ne s'annule : `e` ne veut alors plus rien
dire comme mesure physique.

### Le plancher instrumental

`get_angles` rend deux décimales. À la pose des essais, 1 LSB (0,01°) déplace la
pointe de 16 à 58 µm selon l'axe, soit un plancher quadratique de **0,099 mm**.
Nos dispersions non nulles valent 0,19 à 0,84 mm : **2 à 8× au-dessus**. Elles
ne sont donc pas un artefact de lecture.

---

## 2. La norme de référence : ISO 9283

*« Manipulating industrial robots — Performance criteria and related test
methods »*. C'est la méthodologie que citent les constructeurs.

Elle définit la répétabilité comme *« closeness of agreement between the
attained positions after n repeat visits to the same command pose **in the same
direction** »*.

> Le « **in the same direction** » est dans la définition elle-même. Notre biais
> de 5,9 mm selon le côté d'arrivée est donc **hors du champ de la
> spécification par construction** — ce n'est pas une interprétation.

La grandeur normalisée est :

$$RP = \bar{l} + 3\,S_l$$

où $\bar{l}$ est la moyenne des distances au barycentre et $S_l$ leur
écart-type. C'est celle employée dans toute notre campagne depuis le 09/09.
L'« écart max » utilisé auparavant **n'est pas** cette grandeur et la
sous-estime systématiquement.

### Notre protocole face à celui de la norme

| exigence ISO 9283 | notre campagne | conforme |
|---|---|---|
| cube ISO = plus grand cube inscrit dans l'espace de travail | un point unique | non |
| 5 points P1–P5 sur un plan incliné du cube | 1 point | non |
| 30 cycles | 10 retours | non |
| 100 % de la charge nominale | à vide | non |
| 100 % de la vitesse nominale | vitesse non contrôlée | non |
| sens d'arrivée constant | respecté | **oui** |
| formule RP = l̄ + 3S_l | appliquée | **oui** |
| mesure par moyen externe | FK sur les codeurs | non |

Notre essai est donc une **version réduite** : conforme sur la définition et la
formule, non conforme sur l'échantillonnage, la charge, la vitesse et
l'instrument.

⚠️ Réserve sur la comparaison elle-même : **rien n'indique qu'Elephant Robotics
ait suivi ISO 9283** pour annoncer ses ±0,5 mm. Comparer deux chiffres suppose
de connaître les deux protocoles ; on ne connaît que le nôtre.

---

## 3. Les sept types d'essai

| # | essai | référence externe | état | résultat |
|---|---|---|---|---|
| 1 | Répétabilité du bras | **nécessaire** pour statuer sur ±0,5 mm | fait, borne inférieure | 0,00 à 0,84 mm selon la direction |
| 2 | Erreur cartésienne reconstruite | non — se suffit | fait | 14,84 mm brut · ≈2 mm compensé |
| 3 | Précision physique absolue | **indispensable** | **non fait** | inaccessible par cette méthode |
| 4 | Répétabilité de la vision | non — auto-référencée | fait | sous la quantification du détecteur |
| 5 | Précision métrique de la vision | **oui** — une longueur connue | fait, référence non vérifiée | +0,005 % sur 100 mm · −0,29 % base longue |
| 6 | Calibration extrinsèque | **oui** — un point indépendant | **fait le 09/09 au soir** | **5,46 mm** en moyenne au point non vu |
| 7 | Précision globale vision + robot | non — l'objet est sa cible | **non fait** | à chiffrer |

### 1 — Répétabilité du bras

Six séries unidirectionnelles au même point (330,5 · 31,8 · 60) mm, portée
332 mm, outil incliné à −15°.

| série | n | RP ISO 9283 | états distincts | vs ±0,5 mm |
|---|---|---|---|---|
| A — par le haut | 10 | 0,403 mm | 2 | sous |
| F-UNI — par le haut | 6 | 0,557 mm | 2 | **au-dessus** |
| G — depuis l'arrière | 10 | 0,000 mm | 1 | sous |
| G — depuis la droite | 10 | 0,186 mm | 2 | sous |
| G — depuis l'avant | 10 | 0,794 mm | 4 | **au-dessus** |
| G — depuis la gauche | 10 | 0,838 mm | 3 | **au-dessus** |

**Trois séries sur six dépassent la spécification.** Les séries *sous* le seuil
ne prouvent rien — une borne inférieure ne peut qu'échouer à infirmer. Seuls
les **dépassements** informent : une borne inférieure déjà au-dessus du seuil ne
peut pas redescendre avec une meilleure instrumentation.

**Conclusion défendable : à cette pose, la répétabilité réelle est ≥ 0,84 mm
dans la pire direction.**

### 2 — Erreur cartésienne reconstruite

14,84 mm en boucle ouverte, dont ~13 mm d'affaissement gravitaire. Ramenée à
**≈2 mm** par la réinjection d'écart de `converge`. **Ne se compare à aucun
chiffre constructeur** — Elephant Robotics n'en publie pas.

### 3 — Précision physique absolue

**Non mesurée, et non mesurable par notre méthode.** Exige un moyen externe.

### 4 — Répétabilité de la vision

Tag et caméra immobiles, 60 trames : sortie du détecteur **identique** d'une
trame à l'autre (vérifié par md5 que les images étaient bien distinctes, 30/30).
Sur les marqueurs de planche, l'écart-type des centres vaut **0,008 à 0,166 px**
sur 20 trames. Ce n'est pas une précision, c'est une **absence de gigue**.

### 5 — Précision métrique de la vision

Seule vérité terrain disponible : la **taille imprimée**, la position du tag
ayant été posée à l'œil.

| objet | taille vraie | vue | px | écart |
|---|---|---|---|---|
| tag opérateur | 100 mm | 100,005 | 46,2 | **+0,005 %** |
| tag opérateur | 50 mm | 48,374 | — | −3,252 % |
| marqueur 19 | 50 mm | 49,368 | 24,5 | −1,265 % |
| marqueur 25 | 50 mm | 48,595 | 21,8 | −2,811 % |
| marqueur 26 | 50 mm | 47,385 | 20,7 | −5,230 % |
| distances 19–25 / 19–26 / 25–26 | 380–580 mm | — | — | **−0,29 % moyen** |

**Aucune erreur d'échelle dans la chaîne** : une vraie erreur d'échelle serait
identique à toutes les tailles. Les trois marqueurs de 50 mm, physiquement
identiques, donnent −1,3 à −5,2 % selon leur obliquité. Le −3,25 % est un
**artefact de mesure sur petit marqueur oblique**.

⚠️ Réserve : la taille imprimée **n'a pas été vérifiée au pied à coulisse**.
Tant que ce contrôle n'est pas fait, le +0,005 % reste une auto-cohérence.

### 6 — Calibration extrinsèque caméra ↔ robot

Voir la section 4 ci-dessous : c'est le résultat neuf de la soirée.

### 7 — Précision globale vision + robot

**Non chiffrée.** C'est pourtant le seul nombre qui décrit le système tel qu'il
fonctionne, et **il ne demande aucune métrologie externe** : l'objet est sa
propre cible.

---

## 4. Validation de l'extrinsèque par *leave-one-out* (09/09, soir)

Aucun instrument, aucune recalibration : on ajuste la pose caméra sur une partie
des marqueurs et on **prédit un marqueur que l'ajustement n'a jamais vu**.

Le bras a été écarté du champ (pose derrière la base, pointe à 408 mm) pour que
les **4 marqueurs** soient visibles — la pose d'observation en masquait un.

### Le problème du réglage à 4 centres

| | 4 centres | 16 coins |
|---|---|---|
| contraintes | 8 | 32 |
| inconnues | 6 | 6 |
| **redondance** | **2** | **26** |
| résidu d'ajustement | 0,30 px · 0,59 mm | 0,96 px · 2,01 mm |

Avec 4 centres, en retirer un laisse **exactement 6 contraintes pour
6 inconnues** : le système est tout juste déterminé et devient instable. Mesuré :

| marqueur exclu | erreur au sol | déplacement de la caméra estimée |
|---|---|---|
| 19 | 77,25 mm | 494 mm |
| 23 | 6,63 mm | 45 mm |
| 25 | 89,98 mm | 664 mm |
| 26 | 3,83 mm | 23 mm |

Ces chiffres ne mesurent pas la qualité de l'extrinsèque : ils mesurent
l'**absence de redondance**.

### Le vrai test, avec les 16 coins

| marqueur exclu | erreur image | erreur au sol | déplacement caméra |
|---|---|---|---|
| 19 | 1,82 px | 3,64 mm | 17,0 mm |
| 23 | 1,48 px | 3,12 mm | 31,2 mm |
| 25 | 5,75 px | **12,25 mm** | 78,6 mm |
| 26 | 1,22 px | 2,82 mm | 26,6 mm |
| **moyenne** | **2,57 px** | **5,46 mm** | 38,4 mm |

> **L'extrinsèque est juste à ≈ 5 mm en un point qu'elle n'a pas servi à
> ajuster, jusqu'à 12 mm au marqueur le plus lointain.**

C'est **neuf fois** le `erreur_sol_rms_mm: 0.594` annoncé dans le fichier de
calibration. Ce 0,594 est un **résidu d'ajustement** avec 2 degrés de liberté de
redondance : il était pratiquement garanti d'être petit et ne mesure pas la
justesse.

Signature révélatrice : l'ajustement à 16 coins a un résidu **plus grand**
(0,96 px contre 0,30) précisément parce qu'il a de la redondance et ne peut plus
absorber les erreurs.

### La cause probable

Les positions des marqueurs viennent d'un relevé **au mètre ruban** du
18/08/2026, dont l'en-tête de [`workspace_markers.yaml`](workspace_markers.yaml)
borne lui-même l'erreur à **±5 mm**. On ne peut pas être plus juste que sa
référence : les 5,46 mm mesurés sont exactement de cet ordre.

---

## 5. L'absence de référence externe : où c'est acceptable

Le critère : **une référence externe devient nécessaire dès que l'affirmation
porte sur le robot lui-même plutôt que sur la boucle de commande.**

| situation | référence externe |
|---|---|
| comparer à une spécification constructeur | **obligatoire** |
| annoncer une précision absolue | **obligatoire** |
| valider la transformation caméra ↔ robot | **obligatoire** (position des marqueurs) |
| caractériser la boucle d'asservissement | **inutile** |
| mesurer la répétabilité d'un capteur | **inutile** |

### Pourquoi jamais la règle

Règle de métrologie : l'instrument doit être **4 à 10× plus fin** que la
grandeur mesurée. Pour statuer sur 0,5 mm il faut lire à 0,05 mm. Une règle
graduée lit à ~0,5 mm — l'ordre de ce qu'on cherche. Elle ne peut rien dire.

| instrument | résolution | prix | ce qu'il donne |
|---|---|---|---|
| **comparateur à cadran** | 0,01 mm | **20–200 €** | répétabilité, 1 axe à la fois |
| interféromètre laser | 0,001 mm | ~50 k$ | 1 coordonnée, mise en œuvre lourde |
| ballbar | 0,1 µm | ~10 k€ | trajectoire circulaire, jeux d'axes |
| laser tracker | 10–20 µm | 80–150 k€ | pose complète, référence absolue |
| photogrammétrie | 0,1–1 mm | 20–100 k€ | pose complète, grand volume |

La littérature confirme le choix pragmatique : les comparateurs *« sont moins
chers, ont une précision optimale et permettent la mesure décrite par
ISO 9283 »*.

---

## 6. La méthode proposée, sans refaire l'existant

**Étape 1 — un comparateur numérique sur base magnétique (~60 €, 1 h).**
Palpeur contre une face plane de la pince, 10 retours depuis le même sens :
lecture directe au centième, sans modèle. Puis 10 retours depuis un autre sens
sans rien démonter → biais de direction **jeu compris**, ce que la FK ne verra
jamais. Cette seule manipulation transforme six bornes inférieures en valeurs.

**Étape 2 — re-relever les positions des marqueurs de planche.** C'est le
maillon faible démontré en section 4 : ±5 mm de ruban plafonnent toute la
chaîne à ~5 mm. Un relevé au pied à coulisse depuis un bord de référence, ou un
ajustement conjoint des deux caméras comme cela a déjà été fait pour le
marqueur 19 (±0,3 mm), ferait tomber l'erreur d'un ordre.

**Étape 3 — passer l'extrinsèque de production aux 16 coins.** Le fichier actuel
est ajusté sur 4 centres, sans redondance. Les 16 coins donnent 26 degrés de
liberté et un résidu qui *signifie* quelque chose.

**Étape 4 — vérifier le tag imprimé au pied à coulisse (5 min).** Referme la
seule réserve du test 5.

**Étape 5 — le chiffre du mémoire : la précision globale en boucle fermée.**
Journaliser à chaque tentative `P_vision`, `P_atteint`, la correction appliquée
par `converge`, le nombre d'itérations et le statut de la pince. Sur ~30
tentatives : résidu de convergence, vitesse de convergence, taux de réussite.
**Aucune métrologie externe requise.**

### Ce qu'il ne faut pas refaire

- ❌ **Hand-eye avec un marqueur sur la pince.** Le montage n'est pas stable :
  l'hypothèse de transformation rigide constante tombe. Le jeu de données du
  10/07 (`handeye_data.npz`, 22 poses) cumule ce défaut et un
  sous-dimensionnement du marqueur — 40 mm à 639–1169 mm, soit 17 à 31 px quand
  `COTE_MARQUEUR_PX_MIN` vaut 30, **20 poses sur 22 sous le seuil**. Résidu de
  fermeture 24,4 mm : inexploitable.
- ❌ **Refaire la répétabilité à la FK.** Elle est faite, elle est bornée, elle
  ne progressera qu'avec un instrument.
- ❌ **Chercher une précision absolue au laser tracker.** Hors budget, et
  **inutile** pour le sujet.

---

## 7. Conséquence pour l'asservissement visuel

En boucle fermée, **la précision absolue du bras n'a pas d'importance** : la
boucle annule toute erreur de modèle constante. Les 14,8 mm de boucle ouverte
sont exactement ce que la boucle est là pour absorber.

Ce qui limite réellement la boucle est ce qui n'est **pas** répétable :

| terme | valeur | remédiable par la boucle ? |
|---|---|---|
| bruit de la vision | sous la quantification | sans objet |
| échelle de la vision | +0,005 % sur 100 mm | sans objet |
| erreur de modèle du bras | ~15 mm | **oui**, absorbée |
| justesse de l'extrinsèque | **~5 mm** | **oui** si l'objet est revu à chaque itération |
| répétabilité du bras | ≥ 0,84 mm | non — plancher |
| **biais de sens d'approche** | **5,9 mm** | **non** si la direction change |

> **Contrainte de conception** : si le dernier segment d'approche change de
> direction d'une itération à l'autre, on injecte 5,9 mm de bruit dans une
> boucle qui cherche à converger au millimètre. Terminer toujours par le même
> vecteur d'approche.

---

## Sources

- [ISO 9283 Performance Testing — RoboDK](https://robodk.com/doc/en/Robot-Validation-ISO9283.html)
- [Measurement of industrial robot pose repeatability — MATEC Web of Conferences](https://www.matec-conferences.org/articles/matecconf/pdf/2018/103/matecconf_itep2018_01015.pdf)
- [Measuring position repeatability of industrial robots — Robohub](https://robohub.org/measuring-position-repeatability-of-industrial-robots/)
- [Repeatability and Accuracy of an Industrial Robot — TIIJ](https://tiij.org/issues/issues/spring2009/11_Sirinterlikci/Sirinterlikci-Robot%20Repeatability.pdf)
- [ISO 9283 — GlobalSpec](https://standards.globalspec.com/std/323561/ISO%209283)
