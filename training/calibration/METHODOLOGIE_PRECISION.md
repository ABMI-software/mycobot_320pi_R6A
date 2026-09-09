# Méthodologie des essais de précision — myCobot 320 Pi

> ## ⚠️ Deux corrections majeures du 09/09 au soir
>
> ## ⚠️ Le constructeur se contredit — deux chiffres officiels
>
> | source Elephant Robotics | répétabilité | rayon | poids |
> |---|---|---|---|
> | [page produit 320 Pi](https://www.elephantrobotics.com/en/mycobot-320-pi-en/) | **1 mm** | 350 mm | 850 g |
> | [GitBook, paramètres produit 320](https://docs.elephantrobotics.com/docs/gitbook-en/2-serialproduct/2.2-320/2.2.2.1%20Introduction%20of%20product%20parameters.html) | **±0,5 mm** | 350 mm | 3 kg |
>
> Le **±0,5 mm** est donc bien officiel : il vient du GitBook, la documentation
> technique. La page produit annonce 1 mm. Le poids départage sans trancher le
> reste : le 320 pèse ~3 kg, donc les 850 g de la page produit sont faux — ce
> qui affaiblit cette page, sans prouver que son 1 mm l'est aussi.
>
> **Verdict selon la source retenue :**
>
> | | ±0,5 mm (GitBook) | 1 mm (page produit) |
> |---|---|---|
> | séries conformes | **3 sur 6** | **6 sur 6** |
> | pire série (0,838 mm) | ×1,7 au-dessus | conforme, 84 % du budget |
>
> Les tableaux de ce document donnent les deux colonnes. **Tant que le
> constructeur n'est pas départagé, on ne peut pas conclure** — et c'est la
> réponse honnête à donner.
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
ait suivi ISO 9283** pour annoncer son 1 mm. Comparer deux chiffres suppose de
connaître les deux protocoles ; on ne connaît que le nôtre.

⚠️ Et d'où venait le ±0,5 mm ? Pas de la fiche du 320 Pi. Il faut retirer ce
chiffre de tous les supports et le remplacer par **1 mm**, en citant la fiche
officielle.

---

## 3. Les sept types d'essai

| # | essai | référence externe | état | résultat |
|---|---|---|---|---|
| 1 | Répétabilité du bras | utile, non bloquante | fait, borne inférieure | 0,00 à 0,84 mm — **toutes sous la spec de 1 mm** |
| 2 | Erreur cartésienne reconstruite | non — se suffit | fait | 14,84 mm brut · ≈2 mm compensé |
| 3 | Précision physique absolue | **indispensable** | **non fait** | inaccessible par cette méthode |
| 4 | Répétabilité de la vision | non — auto-référencée | fait | sous la quantification du détecteur |
| 5 | Précision métrique de la vision | **oui** — une longueur connue | fait, référence non vérifiée | +0,005 % sur 100 mm · −0,29 % base longue |
| 6 | Calibration extrinsèque | **oui** — un point indépendant | **fait le 09/09 au soir** | **1,86 mm** en interpolation |
| 7 | Précision globale vision + robot | non — l'objet est sa cible | **non fait** | à chiffrer |

### 1 — Répétabilité du bras

Six séries unidirectionnelles au même point (330,5 · 31,8 · 60) mm, portée
332 mm, outil incliné à −15°.

| série | n | RP ISO 9283 | états distincts | portée du départ | vs 1 mm |
|---|---|---|---|---|---|
| A — par le haut | 10 | 0,403 mm | 2 | — | sous |
| F-UNI — par le haut | 6 | 0,557 mm | 2 | — | sous |
| G — depuis l'arrière | 10 | 0,000 mm | 1 | 292 mm | sous |
| G — depuis la droite | 10 | 0,186 mm | 2 | 331 mm | sous |
| G — depuis l'avant | 10 | 0,794 mm | 4 | **372 mm** | sous |
| G — depuis la gauche | 10 | 0,838 mm | 3 | 338 mm | sous |

**Les six séries sont sous la spécification officielle de 1 mm.** Le robot tient
son chiffre constructeur, y compris dans sa pire direction.

Deux observations qui restent valables :

- la répétabilité **varie d'un facteur ≥ 4** selon le côté d'où l'on arrive
  (0,19 à 0,84 mm en écartant le 0,000 qui est sous le plancher de mesure) ;
- la pire série, « depuis l'avant », est aussi la seule dont le point de départ
  se situe à **372 mm, au-delà du rayon nominal de 350 mm**. Travailler hors
  enveloppe n'est couvert par aucune spécification.

Réserve inchangée : la mesure est une **borne inférieure**. Passer sous le seuil
ne prouve pas la conformité — cela échoue seulement à l'infirmer.

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

### Le côté réel des marqueurs : 48,49 mm, pas 50

Avant tout test, il faut corriger le modèle. En laissant le côté du marqueur
libre et en minimisant la reprojection sur les 16 coins :

| côté supposé | reprojection |
|---|---|
| 48,49 mm (**optimum ajusté**) | **0,8277 px** |
| 50,00 mm (valeur du YAML) | 0,9624 px |

**−3,02 %.** C'est exactement le −3,25 % mesuré l'après-midi sur le tag de
50 mm et classé alors « artefact ». Il y a bien une **erreur d'impression
réelle** d'environ −3 % sur la planche de 50 mm. Le tag de 100 mm, imprimé
séparément, est juste à +0,005 %.

### Ce que le fit dit vraiment, une fois le côté corrigé

| contrôle | résultat |
|---|---|
| écart centre vu ↔ position YAML, par marqueur | **0,56 à 0,71 mm** |
| forme de la constellation (distances entre marqueurs) | **0,57 mm moyen · 1,36 mm max** |
| **leave-one-CORNER-out** (1 coin sur 16, interpolation) | **1,86 mm moyen · 1,42 mm médian · 4,87 mm max** |
| leave-one-marker-out : 19, 23, 26 | 2,4 à 2,8 mm |
| leave-one-marker-out : 25 (extrapolation) | 12,6 mm |

> **L'extrinsèque est juste à ≈ 1,5–2 mm à l'intérieur de la constellation.**

La **forme** de la constellation est le contrôle décisif : un ajustement rigide
à 6 DoF peut absorber une translation ou une rotation globale du jeu de
marqueurs, mais **pas une déformation**. Les distances entre marqueurs collent
au plan à 0,57 mm en moyenne. Les positions du YAML sont donc **bien meilleures
que le ±5 mm annoncé** par son en-tête.

### Pourquoi le premier chiffre de 5,46 mm était faux

Retirer 1 marqueur sur 4, c'est retirer **un coin entier** de la constellation :
la prédiction cesse d'être une interpolation et devient une **extrapolation sur
~400 mm**. Une erreur angulaire minime de la pose ajustée s'y amplifie
linéairement. Le 5,46 mm mesurait la géométrie du test, pas la justesse de
l'extrinsèque.

Le protocole correct sur une constellation aussi pauvre est le
**leave-one-corner-out**, qui retire un point sur seize et reste entouré de
données.

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

**Étape 2 — corriger `marker_size_mm` : 48,5 et non 50.** C'est la seule erreur
franche trouvée dans les données. Elle déplace chaque coin de 0,75 mm et entre
directement dans l'extrinsèque à 16 coins. À confirmer au pied à coulisse avant
d'éditer le YAML.

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
