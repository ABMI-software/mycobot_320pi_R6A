# Méthodologie des essais de précision — myCobot 320 Pi

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

⚠️ Les deux chiffres constructeur sont officiels — voir l'en-tête. **1 mm**
vient de la page produit, **±0,5 mm** du GitBook. Les tableaux de ce document
donnent donc les deux colonnes plutôt que d'en choisir une.

---

## 3. Les sept types d'essai

| # | essai | référence externe | état | résultat |
|---|---|---|---|---|
| 1 | Répétabilité du bras | utile, non bloquante | fait, borne inférieure | 0,00 à 0,84 mm — **toutes sous la spec de 1 mm** |
| 2 | Erreur cartésienne reconstruite | non — se suffit | fait | 14,84 mm brut · ≈2 mm compensé |
| 3 | Précision physique absolue | **indispensable** | **non fait** | inaccessible par cette méthode |
| 4 | Répétabilité de la vision | non — auto-référencée | **refait le 10/09** | **0,725 mm** (6 trames) · **17,5 mm** (1 trame) |
| 5 | Précision métrique de la vision | **oui** — un artefact étalonné | repris le 10/09 | échelle longue **−0,044 %** · le tag fait bien **50 mm**, le biais est dans la détection |
| 6 | Calibration extrinsèque | **oui** — un point indépendant | **fait le 09/09 au soir** | **1,86 mm** en interpolation |
| 7 | Précision globale vision + robot | non — l'objet est sa cible | **FAIT le 10/09, 20 essais** | **10,79 mm** brut → **0,63 mm** après 2 corrections |

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

**Non mesurée.** C'est le seul des sept essais qui reste à faire, et le seul
qui exige un achat.

Notre méthode reconstruit la position par `FK(q_lu)` : elle ne peut voir ni
l'écart entre le codeur et l'articulation, ni les erreurs du modèle
cinématique, ni le jeu des réducteurs, ni la flexion. Aucun raffinement du
protocole ne changera cela — il faut un instrument qui touche le bras.

| ce qu'il faut | comparateur à cadran sur base magnétique |
|---|---|
| résolution | 0,01 mm, soit 40 à 80× plus fin que la grandeur cherchée |
| coût | 20 à 200 € |
| durée | environ 1 h |
| ce que ça change | les six répétabilités passent de **bornes inférieures** à des **valeurs** |

### 4 — Répétabilité de la vision

**Mesure refaite le 10/09, et la première conclusion était fausse.** Le premier
essai concluait à une « absence de gigue » : sortie du détecteur identique d'une
trame à l'autre. C'était un artefact du détecteur par défaut, dont les coins ne
sont pas raffinés au sous-pixel — sur une scène immobile il retombe sur les
mêmes pixels entiers.

Refait avec `CORNER_REFINE_SUBPIX`, 60 détections, bras écarté, scène immobile :

| méthode | n | l̄ | σ | RP ISO 9283 | max |
|---|---|---|---|---|---|
| détection brute, 1 trame | 60 | 0,610 mm | 2,220 | 7,270 mm | **17,46 mm** |
| sous-pixel, 1 trame | 60 | 0,632 mm | 2,251 | 7,384 mm | 17,52 mm |
| **sous-pixel + moyenne de 6 trames** | 55 | **0,141 mm** | 0,195 | **0,725 mm** | **0,80 mm** |

**Le raffinement sous-pixel n'était pas le problème** — il ne change presque
rien (0,610 → 0,632 mm). Le vrai coupable, ce sont les **trames corrompues** :
le flux MJPEG du pilote perd régulièrement des segments (messages
`Corrupt JPEG data`), l'image arrive tronquée et le marqueur est détecté de
travers, jusqu'à **17,5 mm**. Rare, mais énorme.

Moyenner six trames l'élimine : le maximum tombe de 17,5 mm à **0,80 mm**.

> **Règle opérationnelle.** Ne jamais commander le robot sur une seule trame.
> Une prise sur trame unique rate de temps en temps, de près de 2 cm, sans que
> rien ne le signale. Le test 7 était déjà dans ce régime — il moyenne six
> trames — donc ses chiffres tiennent.

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
identique à toutes les tailles. Le +0,005 % sur le tag de 100 mm le montre.

**Correction du 10/09 — et cette fois dans l'autre sens.** Ce document a
d'abord dit « artefact de mesure », puis « vraie erreur d'impression ». La
deuxième affirmation était fausse : **la première avait raison.**

Cinq marqueurs identiques, 12 acquisitions chacun, extrinsèque recalibrée :

| marqueur | côté mesuré | obliquité |
|---|---|---|
| 19 | **49,725 ± 0,019 mm** | 1,0096 |
| 25 | 49,154 ± 0,040 | 1,0197 |
| 23 | 48,472 ± 0,107 | 1,0181 |
| 1 | 48,306 ± 0,131 | 1,0246 |
| 26 | 47,749 ± 0,242 | 1,0287 |

Répétabilité sur un **même** marqueur : **± 0,108 mm**. Étalement entre
marqueurs **identiques** : **1,98 mm**, soit 18 fois plus. La mesure est précise
et fausse.

Corrélation du côté mesuré avec l'**obliquité : r = −0,920**. Ajustement
`côté = −97,96 × obliquité + 148,62`, résidus 0,27 mm RMS. Extrapolé à une vue
de face : **50,65 mm**.

Preuve indépendante — les **grandes distances** entre centres (383 à 580 mm)
sont justes à **−0,044 %**, contre **−2,6 %** sur les côtés de 50 mm. Une vraie
erreur d'échelle frapperait les deux à l'identique.

> **La calibration est juste ; c'est la détection des coins ArUco qui rétrécit
> les petits carrés vus de biais. Les tags font 50 mm.**

Le biais porte sur la **taille**, jamais sur la **position** : les centres
tombent à 0,11–0,42 mm de leurs valeurs connues, parce que les quatre coins sont
tirés vers l'intérieur de façon symétrique.

**Certitude : 50,7 ± 0,7 mm** (0,27 mm de résidu d'ajustement, ~0,6 mm de la
référence d'échelle au ruban). Cela encadre 50 et **exclut 48,3** à plus de
trois écarts. Un artefact étalonné resserrerait à 0,05 mm et satisferait
VDI/VDE 2634-1, mais **ne changerait aucune décision** : l'argument ne repose
pas sur la valeur absolue, mais sur le fait qu'une erreur d'échelle serait
identique à toutes les longueurs — or elle vaut −0,044 % sur 383–580 mm et
−2,6 % sur 50 mm.

### 6 — Calibration extrinsèque caméra ↔ robot

**1,86 mm** de justesse à l'intérieur de la constellation de marqueurs, mesurés
par *leave-one-corner-out* : on ajuste sur 15 coins et on prédit le 16ᵉ.

Deux contrôles complémentaires. La **forme** de la constellation est reproduite
à 0,57 mm en moyenne — un ajustement rigide peut absorber une translation ou une
rotation globale, jamais une déformation. Et une **validation croisée à deux
caméras**, sur un point que ni l'une ni l'autre n'a utilisé pour s'ajuster,
donne **1,92 mm**.

Protocole et chiffres détaillés au § 4.

### 7 — Précision globale vision + robot

**10,79 mm en boucle ouverte, 0,63 mm après deux corrections**, sur 20 essais.

C'est le seul chiffre qui décrive le système tel qu'il fonctionne, et il n'a
demandé aucune métrologie externe : l'objet est sa propre référence. L'erreur
est **à 100 % un biais** — il chute d'un facteur 20 pendant que la dispersion
reste plate, ce qui est exactement la condition pour qu'un asservissement
visuel la corrige.

Protocole et chiffres détaillés au § 8.

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

### Le côté ajusté à 48,49 mm — ce que ça mesurait vraiment

*Section écrite le 09/09, corrigée le 10/09. Conservée parce que le raisonnement
qu'elle contient est un piège instructif.*

En laissant le côté du marqueur libre et en minimisant la reprojection sur les
16 coins :

| côté supposé | reprojection |
|---|---|
| 48,49 mm (**optimum ajusté**) | **0,8277 px** |
| 50,00 mm (valeur du YAML) | 0,9624 px |

J'en avais conclu à une erreur d'impression de −3 %, en la croyant confirmée par
la mesure directe du côté. **Les deux étaient fausses, et pour la même raison.**

L'ajustement minimise l'écart aux **coins détectés**. Or la détection ArUco tire
les coins d'un petit carré **vers l'intérieur** (§ 3, essai 5 — corrélation
−0,920 avec l'obliquité). Un ajustement sur ces coins trouve donc
*nécessairement* un côté trop petit : il reproduit fidèlement le biais au lieu
de le révéler. Le gain de 0,96 à 0,83 px est réel, mais il mesure **l'ampleur du
biais de détection**, pas la taille du marqueur.

**C'était un piège circulaire** : je validais une mesure biaisée par un
ajustement sur les mêmes données biaisées, et je comptais l'accord des deux
comme une confirmation indépendante.

Ce qui l'a détecté, en fin de compte, c'est la seule chose que le biais ne peut
pas atteindre : les **grandes distances**, justes à −0,044 % quand les petits
côtés sont faux de −2,6 %.

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

**Étape 2 — ~~corriger `marker_size_mm` : 48,5 et non 50~~ — RETIRÉ le 10/09.**
Cette consigne était fausse, **ne pas l'appliquer** : les marqueurs font bien
50 mm et `marker_size_mm` doit rester à 50. Le −2,6 % mesuré sur leurs côtés est
un biais de détection lié à l'obliquité, pas une erreur d'impression (§ 3,
essai 5). Le texte est barré plutôt que supprimé, parce que d'autres documents
l'avaient recopié et doivent pouvoir retrouver ce qui a été retiré.

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
| justesse de l'extrinsèque | **1,86 mm** | **oui** si l'objet est revu à chaque itération |
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


---

## 8. Test 7 — précision globale vision + robot (10/09/2026)

### Ce qui est mesuré

L'écart entre la position que la **caméra** attribue à une cible et celle que
les **encodeurs** attribuent au bout des doigts après un déplacement commandé.
C'est la précision d'une **prise du premier coup**, puis après correction.

### Protocole — 20 essais, 3 mesures chacun

Cible : un tag ArUco 50 mm posé au **centre** du plan de travail, à
(260,49 · 21,66) mm, portée 261 mm. Caméra **arducam seule**.

À chaque essai :

1. le bras s'écarte à la pose d'observation ;
2. la caméra détecte le tag — **moyenne de 6 trames**, jamais une seule ;
3. le bras y va **en une seule fois, sans correction** → mesure (passe 0) ;
4. on réinjecte l'écart articulaire consigne − mesure → mesure (passe 1) ;
5. on réinjecte de nouveau → mesure (passe 2) ;
6. le bras revient, la caméra revoit le tag pour **prouver qu'il n'a pas bougé**.

Survol à **Z = 30 mm : aucun contact avec la table.** Pince fermée, le déport
d'outil désignant le bout des doigts fermés.

### Résultats

| passe | ce que c'est | erreur moy. | biais | dispersion RP ISO | erreur Z | retard | < 2 mm | < 1 mm |
|---|---|---|---|---|---|---|---|---|
| 0 | **boucle ouverte** | **10,79 mm** | 10,79 | 0,671 mm | −9,78 mm | 2,07° | 0/20 | 0/20 |
| 1 | après **une** réinjection | **1,29 mm** | 1,27 | 0,810 mm | −0,42 mm | 0,24° | 20/20 | 4/20 |
| 2 | après **deux** | **0,63 mm** | 0,54 | 0,956 mm | **+0,06 mm** | 0,17° | 20/20 | **18/20** |

### Le résultat, en une phrase

**Les 10,8 mm sont à 100 % un biais.** Le biais s'effondre de 10,79 à 0,54 mm —
vingt fois moins — pendant que la dispersion, elle, ne bouge pas : 0,230 puis
0,291 puis 0,337 mm.

C'est la signature d'une erreur **purement systématique** : l'affaissement
gravitaire, qui tire toujours dans le même sens (X −7,65 et Y −7,60 mm, jamais
l'inverse sur 20 essais). Un biais se rattrape par une correction ; une
dispersion, non.

> **C'est exactement la condition pour qu'un asservissement visuel fonctionne :
> la boucle annule toute erreur de modèle constante.**

En Z c'est plus net encore : **−9,78 mm** en boucle ouverte, **+0,06 mm** après
deux passes. L'affaissement est intégralement repris.

### Contrôles de validité

- **Le tag n'a pas bougé** : déplacement maximal 0,767 mm sur 20 vérifications.
- **L'IK n'est pas en cause** : résidu moyen 0,021 mm, maximum 0,024 mm.
- **La vision était dans le bon régime** : moyenne de 6 trames (cf. § 3, essai 4).

### Réserve

C'est une **borne inférieure**. La position atteinte est reconstruite par
`FK(q_lu)` : elle ignore l'écart encodeur↔articulation, les erreurs du modèle
cinématique, le jeu des réducteurs et la flexion. Le vrai chiffre physique est
plus grand. Le mesurer demande un comparateur à cadran (~60 €).

---

## 9. Les extrinsèques ne valent que dans le plan de la table (10/09/2026)

C'est la découverte la plus lourde de conséquence de la campagne, et elle n'a
été trouvée qu'en cherchant à mesurer autre chose.

### Comment elle est apparue

En essayant de trancher optiquement les **17,33 mm** de doute sur le déport
d'outil (`tool_offset.json`, champ `contradiction_non_resolue`), la SVPRO a
d'abord été recalibrée : elle avait dérivé de **21,3 mm** depuis le 24/08 et de
**6,6 mm** depuis le 02/09. Recalibrée sur la planche, elle retombe à
**0,38–0,72 mm**.

Son marqueur 25 reste indétectable. Il est **physiquement présent**, mais son
bord bas tombe **8,8 px sous le cadre** — ArUco exige les quatre coins. Vérifié
à toutes les résolutions du capteur (2592×1944 à 640×480) : le champ est
identique, ce n'est pas une question de définition. Le corriger demande de
bouger la caméra, jamais la planche.

### La validation croisée, sur un point neuf

Le tag posé au centre n'a servi à **aucun** des deux ajustements — l'arducam est
ajustée sur 19/23/25/26, la SVPRO sur 19/23/26. Chaque caméra le prédit
indépendamment.

| point | arducam (X,Y) | SVPRO (X,Y) | désaccord |
|---|---|---|---|
| **tag central — jamais utilisé** | 260,84 / 21,59 | 259,44 / 22,90 | **1,92 mm** |
| marqueur 19 | 96,66 / 202,66 | 96,78 / 203,96 | 1,30 mm |
| marqueur 26 | 529,43 / −175,68 | 529,93 / −175,61 | 0,50 mm |

Ces **1,92 mm** bornent l'erreur des **deux chaînes optiques réunies**, sur un
point neuf et central. C'est le premier contrôle vraiment indépendant de la
campagne : ce n'est pas un résidu qui se mesure lui-même, ce sont deux
intrinsèques, deux extrinsèques et deux angles de vue qui convergent.

### La découverte

Les quatre marqueurs de planche sont **tous à Z = 0**. Douze coins coplanaires
fixent très bien la pose **dans** le plan — d'où les 0,4 mm de résidu et les
1,92 mm de recoupement — mais **contraignent mal la rotation hors plan**. C'est
la dégénérescence classique de la cible plane, et **le résidu ne peut pas la
voir** : il est mesuré sur ce même plan.

Contrôle direct, pince à **Z = 172 mm**, patins verts vus par les deux caméras :

| patin | l'arducam dit | la SVPRO dit |
|---|---|---|
| arrière | (192 / +7) | (299 / +127) |
| avant | (194 / −43) | (243 / +153) |

Or **toute la pince tient entre Y = +92 mm** (le flasque) **et Y = −20 mm** (la
pointe). La SVPRO les place *derrière* le flasque, là où il n'y a rien. Et la
triangulation des deux rayons rend un point à **Z = −12 mm, sous la table** :
physiquement impossible.

### Conséquence

| | |
|---|---|
| **Reste valide** | viser un objet **posé sur la table** — le pick-and-place, validé à 1,92 mm, et le test 7 ci-dessus |
| **N'est pas valide** | toute mesure **en hauteur** : position de la pince en vol, hauteur d'un objet, obstacle |
| **Pour lever la limite** | refaire la calibration avec des marqueurs à **plusieurs hauteurs** — un tag collé sur une boîte de hauteur connue suffit |

Deux ou trois niveaux rendent la rotation hors plan observable, et débloquent du
même coup la mesure du déport d'outil, restée indécidable.

### Pourquoi le déport d'outil n'a pas pu être tranché

Les deux candidats sont distants de 17,33 mm, mais **16,9 mm de cet écart est
vertical** et seulement 3,9 mm latéral. Or les deux caméras regardent d'en haut :

| caméra | élévation au-dessus de l'horizontale | séparation des deux candidats |
|---|---|---|
| arducam | ~85° | < 2 px |
| SVPRO | 58° | **4,2 px** |

La mesure n'est pas dans l'image. Ce n'est ni un problème de calibration ni
d'algorithme.

**Ce que dit le faisceau d'indices**, à défaut de preuve : le déport actuel
repose sur **7 mesures au réglet, 3 azimuts couvrant 59°, écart-type 1,0 mm**.
Ce qui le contredit est **un seul** point enseigné du 18/08, à une pose qui
n'est même plus atteignable (J2 à −135,7°, au-delà de la butée −135°). Sept
mesures reproductibles contre une non reproductible.

### Une méthode écartée, et pourquoi

La première approche proposée était de **descendre jusqu'au contact avec la
table** et de lire la hauteur. Elle a été rejetée, à raison : le bras a
**1,0–2,1° de retard de suivi**, soit 6 à 10 mm en cartésien. Une détection de
contact par écart d'encodeurs ne se déclenche donc qu'après plusieurs
millimètres d'appui. **La résolution du protocole était pire que la grandeur
cherchée**, et il abîmait le matériel en prime.

> Règle générale qui en découle : avant de proposer un protocole, chiffrer sa
> résolution et la comparer à la grandeur cherchée.


---

## 10. Les normes : celles que j'ai suivies, celles que j'ai ignorées

Cette section répond à une question directe : *cette méthodologie s'appuie-t-elle
sur des normes publiées et nommées, ou est-elle inventée ?*

**Réponse honnête : une seule norme a été suivie, et seulement en partie. Toute
la moitié « vision » de la campagne repose sur des protocoles construits pour
l'occasion.**

### 10.1 Ce qui a été suivi

| norme | ce qui en a été pris | ce qui a été laissé |
|---|---|---|
| **ISO 9283:1998** — *Manipulating industrial robots — Performance criteria and related test methods* | la **définition** de la répétabilité de pose : `RP = l̄ + 3S_l` (distance moyenne au barycentre + 3 écarts-types) | **tout le protocole** : cube ISO, 5 poses P1–P5 sur plan incliné, 30 cycles, 100 % de la charge nominale, 100 % de la vitesse |
| **ISO 9787:1999** — *Coordinate systems and motion nomenclatures* | suivie **de fait** sans être citée : toutes les poses sont rapportées dans un repère de base unique (`base_link`), ce qu'ISO 9283 exige en s'y référant | rien |
| **ISO 8373** — *Vocabulary* | le vocabulaire employé | rien |

Nos essais sont donc menés dans des conditions **plus favorables** que la norme
(à vide, à vitesse réduite, sur une seule pose). Les chiffres sont optimistes,
et c'était déjà signalé au § 2.

### 10.2 La norme qui manquait : VDI/VDE 2634 Partie 1

**C'est la norme du test E, et je ne la connaissais pas en le concevant.**

VDI/VDE 2634 *Optical 3-D measuring systems* définit les méthodes de réception
et de re-vérification des systèmes de mesure optique 3D. La **partie 1** couvre
les systèmes à **palpage point par point** — exactement notre cas : on mesure la
position de points (centres et coins de marqueurs) dans un volume.

Elle définit **un seul paramètre de qualité**, l'**erreur de mesure de
longueur** :

```
Δl = l_m − l_k          l_m = longueur mesurée, l_k = longueur étalonnée
E  = A + K·L ≤ B        tolérance admissible, dépendante de la longueur
```

Et elle impose une manière de la mesurer :

| exigence VDI/VDE 2634-1 | notre campagne | conforme |
|---|---|---|
| artefact **étalonné**, dimensions et forme certifiées (marques circulaires sur cales étalon) | un tag ArUco **imprimé**, taille jamais vérifiée au pied à coulisse | **non** |
| **7 lignes de mesure** réparties dans le **volume** de mesure | **1 seule**, et entièrement dans le plan Z = 0 | **non** |
| **≥ 5 longueurs** de test par ligne | quelques distances inter-marqueurs | **non** |
| tolérance déclarée sous la forme `A + K·L` | aucune tolérance déclarée | **non** |

### 10.3 Ce que cette norme aurait évité

Les deux difficultés majeures de la campagne sont **exactement** ce que
VDI/VDE 2634-1 prévient.

**1. L'artefact non étalonné.** Le « −3,25 % » du tag de 50 mm a demandé six
mesures indépendantes et deux jours pour être tranché, parce que la caméra était
comparée à une référence elle-même fausse. La norme exige un artefact **calibré**
précisément pour rendre cette ambiguïté impossible. Le tag de 100 mm reste dans
ce cas aujourd'hui.

**2. Les 7 lignes dans le VOLUME, pas dans un plan.** La dégénérescence
découverte au § 9 — extrinsèques justes à 1,92 mm dans le plan de la table,
sans aucune validité 17 cm plus haut — est précisément ce qu'un plan
d'échantillonnage volumique interdit. **La norme l'avait anticipée ; je l'ai
redécouverte à mes dépens.**

Elle dit enfin une chose contre-intuitive et utile :

> *« Separate testing of the probing error is not required as this effect is
> considered in the determination of the length measurement error. »*

Autrement dit, notre **test C** (bruit de détection, § 3 essai 4) est, du point
de vue de la norme, **redondant** avec un test E correctement mené. Il garde sa
valeur de diagnostic — c'est lui qui a révélé les trames MJPEG corrompues — mais
il ne constitue pas un paramètre de qualité au sens VDI/VDE 2634.

### 10.4 Ce qui reste sans norme, assumé

| protocole | statut |
|---|---|
| validation *leave-one-out* de l'extrinsèque | **construit pour l'occasion** — pas de norme connue |
| validation croisée à deux caméras sur un point neuf | **construit pour l'occasion** |
| test 7, précision globale vision + robot en boucle fermée | **construit pour l'occasion** — ISO 9283 ne couvre pas les systèmes asservis par vision |
| réinjection de l'écart articulaire | pratique d'ingénierie, pas une norme |

Ces protocoles ne sont pas moins valides pour autant — ils sont simplement
**non normalisés**, et doivent donc être décrits intégralement pour être
reproductibles. C'est l'objet de ce document.

### 10.5 Ce qu'il faudrait faire pour être conforme

Par ordre de coût croissant :

1. **Faire étalonner l'artefact** — mesurer les tags au pied à coulisse.
   Resserre le test E de ±0,7 mm à ±0,05 mm. **Ne change aucune décision** : la
   conclusion « les tags font 50 mm » est déjà établie à plus de trois écarts.
   Utile pour une déclaration de conformité, pas pour le projet.
2. **Étendre le test E à plusieurs hauteurs** — des marqueurs sur des cales de
   hauteur connue, 3 niveaux minimum. Lève **du même coup** la dégénérescence du
   § 9 et la question du déport d'outil. C'est la seule action qui débloque
   trois problèmes à la fois.
3. **Déclarer une tolérance** sous la forme `E = A + K·L`, seule façon de dire
   « conforme » ou « non conforme » au lieu de donner un chiffre nu.
4. **Suivre le protocole ISO 9283 complet** pour la partie robot : cube ISO,
   5 poses, 30 cycles, charge et vitesse nominales. C'est le plus lourd, et le
   moins urgent — le robot tient déjà sa spec dans des conditions plus douces.

### 10.6 Traçabilité — quel essai, contre quelle norme, avec quel résultat

Le tableau ci-dessous relie **chaque essai réellement mené** à la norme dont il
relève, à ce qui a été fait, et à ce qui manque pour être conforme. C'est la
lecture à faire si l'on veut savoir *sur quoi repose* un chiffre donné.

| essai | norme applicable | ce que j'ai fait | résultat | conforme ? |
|---|---|---|---|---|
| **A** — répétabilité, 10 A/R verticaux | **ISO 9283:1998** § répétabilité de pose | 10 retours au même point, sens d'approche constant, `RP = l̄ + 3S_l` | **0,403 mm** | **définition oui**, protocole non (1 pose au lieu de 5, 10 cycles au lieu de 30, à vide, vitesse réduite) |
| **F-UNI** — contrôle, 6 A/R | ISO 9283:1998 | idem, série indépendante | **0,557 mm** | idem |
| **G** — 40 A/R, 10 par côté | ISO 9283:1998 | 4 séries unidirectionnelles, une par côté | **0,000 à 0,838 mm** | idem — et c'est la seule à explorer la dépendance au sens d'approche, que la norme n'impose pas |
| **F-MULTI / D** — sens mélangés | **hors norme par construction** | sens d'approche alternés | 5,847 / 8,091 mm | **non applicable** : ISO 9283 définit la répétabilité *« in the same direction »*. Une série multidirectionnelle est hors du champ de la norme |
| **B** — erreur de cible absolue | ISO 9283:1998 § **exactitude** de pose (AP) | écart consigne ↔ position atteinte, boucle ouverte | 14,84 mm → ≈2 mm compensé | **non** — AP exige une mesure externe, la nôtre est reconstruite aux codeurs |
| **C** — bruit de la vision | **VDI/VDE 2634-1** — mais la norme le déclare **redondant** | 60 trames, scène immobile, avec et sans sous-pixel | **0,725 mm** (6 trames) · 17,5 mm (1 trame) | **sans objet** : *« separate testing of the probing error is not required »*. Gardé comme **diagnostic** — c'est lui qui a trouvé les trames MJPEG corrompues |
| **E** — échelle de la vision | **VDI/VDE 2634-1** § erreur de mesure de longueur `Δl = l_m − l_k` | grandes distances (383–580 mm) contre relevé ; côtés de 50 mm contre l'obliquité | **−0,044 %** sur l'échelle longue · le tag fait **50 mm** | **non** — artefact **non étalonné** (c'est ce qui a produit l'erreur de deux jours), 1 plan au lieu du volume, aucune tolérance `A + K·L` |
| **H** — extrinsèque, leave-one-out | **aucune norme connue** | ajustement sur 3 marqueurs, prédiction du 4ᵉ | 1,86 mm en interpolation | **protocole construit pour l'occasion** |
| **K** — validation croisée 2 caméras | **aucune norme connue** ; l'esprit de VDI/VDE 2634-3 (vues multiples) | point neuf jamais utilisé par aucun des deux ajustements | **1,92 mm** | **protocole construit pour l'occasion**, mais c'est le contrôle le plus solide de la campagne |
| **7** — précision globale vision + robot | **aucune norme** — ISO 9283 ne couvre pas les systèmes asservis par vision | 20 essais, 3 passes, cible détectée puis atteinte | **10,79 mm** brut → **0,63 mm** | **protocole construit pour l'occasion** |
| tous | **ISO 9787:1999** — systèmes de coordonnées | toutes les poses rapportées dans `base_link`, repère de base unique | — | **oui**, de fait |
| tous | **ISO 8373** — vocabulaire | termes employés conformes | — | **oui** |

**Ce que ce tableau montre en une lecture :** sur onze lignes, **deux seulement
sont conformes** (ISO 9787 et ISO 8373, les plus faciles), **trois suivent une
définition normalisée sans son protocole** (A, F-UNI, G), et **cinq n'ont aucune
norme applicable** — dont les trois résultats les plus intéressants de la
campagne (H, K et le test 7).

Ce n'est pas un défaut à cacher : la précision d'un système **vision + robot en
boucle fermée** n'est normalisée nulle part à ma connaissance. ISO 9283 mesure
le bras seul, VDI/VDE 2634 mesure l'optique seule. Le chiffre qui compte pour un
asservissement visuel — *quand la caméra dit d'aller là, on arrive à combien ?* —
tombe entre les deux. D'où l'obligation de décrire le protocole intégralement,
ce que fait le § 8.

---

### Sources — et ce que j'ai testé contre chacune

Chaque source est suivie de **ce qu'elle contient** et de **quel essai s'y
rapporte**.

**[ISO 9283:1998 — *Manipulating industrial robots: performance criteria and related test methods*](https://standards.iteh.ai/catalog/standards/iso/b00c9665-53ff-4265-933d-77a544eb4e26/iso-9283-1998)**
· [version européenne EN ISO 9283:1998](https://standards.iteh.ai/catalog/standards/cen/bd6e0b51-df41-44c2-806f-fbd0f53f30de/en-iso-9283-1998)
Définit l'exactitude de pose (AP) et la **répétabilité de pose (RP)**, ainsi que
le protocole d'essai : cube ISO, 5 poses, 30 cycles, charge et vitesse
nominales. → **Essais A, F-UNI, G** en reprennent la formule `RP = l̄ + 3S_l`,
sans le protocole. **Essai B** relève de l'AP mais n'y satisfait pas, faute de
mesure externe. C'est aussi cette norme qui pose la répétabilité *« in the same
direction »*, ce qui met **F-MULTI et D hors de son champ**.

**[ISO 9787:1999 — *Coordinate systems and motion nomenclatures*](https://cdn.standards.iteh.ai/samples/26484/10ceacc990ec4f968ca74878008f0364/ISO-9787-1999.pdf)**
Définit les repères monde, base, interface mécanique et outil. ISO 9283 s'y
réfère explicitement : les poses mesurées doivent être rapportées dans un repère
de base commun. → **Tous les essais** : tout est exprimé dans `base_link`,
origine au centre de la base, Z = 0 au plan de travail. Conforme de fait.

**[ISO — portail Robotique](https://www.iso.org/sectors/engineering/robotics)** ·
**ISO 8373 — *Vocabulary***
Vocabulaire normalisé des robots. → **Tous les essais**, pour la terminologie.

**[VDI/VDE 2634 Blatt 1 — *Optical 3-D measuring systems, point-by-point probing*](https://store.accuristech.com/standards/vdi-vdi-vde-2634-blatt-1?product_id=2936117)**
**La norme du test E, et la plus importante de cette liste.** Définit l'erreur
de mesure de longueur `Δl = l_m − l_k`, la tolérance `E = A + K·L ≤ B`, et la
procédure : artefact **étalonné**, **7 lignes de mesure dans le volume**, **≥ 5
longueurs par ligne**. Précise que le test séparé de l'erreur de palpage n'est
pas requis. → **Essai E** en relève directement et **n'y satisfait sur aucun
point**. **Essai C** est déclaré redondant par cette norme. Ses 7 lignes *dans
le volume* sont exactement ce dont l'absence a produit la dégénérescence du § 9.

**[VDI/VDE 2634 Blatt 2 — *area scanning*](https://standards.globalspec.com/std/9914533/VDI/VDE%202634%20BLATT%202)** ·
**[Blatt 3 — *multiple view systems*](https://standards.globalspec.com/std/9914423/vdi-vde-2634-blatt-3)**
Parties 2 et 3, pour les systèmes à balayage surfacique et à vues multiples.
→ Ne s'appliquent pas directement (nous faisons du palpage point par point),
mais **la partie 3 est l'esprit de l'essai K** : évaluer un système à plusieurs
vues. À consulter si la fusion arducam + SVPRO devient un mode de mesure et non
un simple recoupement.

**[VDI/VDE 2634-1 performance evaluation tests and systematic errors in passive stereo vision systems — ScienceDirect](https://www.sciencedirect.com/science/article/abs/pii/S0141635922002318)** ·
[fiche ResearchGate](https://www.researchgate.net/publication/365369859_VDIVDE_2634-1_performance_evaluation_tests_and_systematic_errors_in_passive_stereo_vision_systems)
Article scientifique appliquant VDI/VDE 2634-1 à la stéréovision passive et
analysant ses **erreurs systématiques**. → C'est la source qui donne le contenu
concret de la partie 1 sans acheter la norme, et le cadre le plus proche de
notre configuration à deux caméras (**essai K**).

**[NIST — *Industrial Robotics Standards*, chapitre 27](https://tsapps.nist.gov/publication/get_pdf.cfm?pub_id=820605)**
Panorama des normes de robotique industrielle et de leurs organismes.
→ Sert à **vérifier qu'aucune norme n'a été oubliée** pour les essais H, K et 7.
C'est sur cette base que j'affirme qu'il n'en existe pas pour la précision d'un
système vision + robot en boucle fermée.

⚠️ **Réserve sur ces sources.** Les textes intégraux d'ISO 9283 et de
VDI/VDE 2634 sont **payants**. Tout ce qui est rapporté ici provient des résumés
publics des organismes de normalisation et de la littérature scientifique citée,
**jamais du texte normatif lui-même**. C'est suffisant pour orienter une
méthode et pour un rapport de stage ; **ce n'est pas suffisant pour écrire
« conforme à ISO 9283 » sur un document officiel.** Il faut alors se procurer
les normes.
