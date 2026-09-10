# Précision du MyCobot 320 Pi — campagne du 9 septembre 2026

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
Ce que le robot tient, ce qu'il ne tient pas, et **ce que la spécification
constructeur ne dit pas**.

Données brutes : [`precision_campagne_2026-09-09.csv`](precision_campagne_2026-09-09.csv)
· [`.xlsx`](precision_campagne_2026-09-09.xlsx) (10 feuilles)
· [`repetabilite_directions_2026-09-09.csv`](repetabilite_directions_2026-09-09.csv)
· [`repetabilite_4directions_2026-09-09.csv`](repetabilite_4directions_2026-09-09.csv)
  — 40 retours, 10 par côté

---

## 0. La confusion à ne plus faire

> **Le 1 mm d'Elephant Robotics est une *repeated positioning precision* —
> une RÉPÉTABILITÉ. Ce n'est pas une précision absolue de positionnement
> cartésien.**

| grandeur | définition | chiffre constructeur |
|---|---|---|
| **Répétabilité** | dispersion en revenant plusieurs fois au **même** point, dans les **mêmes** conditions | **1 mm** (fiche officielle 320 Pi) |
| **Erreur de cible absolue** | ‖FK(q_lu) − P_cible‖ | *aucun publié* |
| **Précision de la vision** | ce que la caméra dit d'un objet de taille connue | *sans objet* |

Comparer notre erreur absolue au 1 mm et conclure « hors spec » est une
**erreur de catégorie**. L'erreur absolue agrège la lecture articulaire, le
modèle FK, les offsets articulaires, la définition du TCP, le settling des
servos et les changements de repère. Aucune de ces contributions n'est couverte
par une spécification de répétabilité.

### Limite qui s'applique à TOUS nos chiffres de répétabilité

La position est reconstruite par **FK sur les angles RELUS**. Elle est donc
aveugle à tout ce qui se trouve **en aval des codeurs** : jeu de réducteur,
souplesse des bras, flexion de l'outil. Le constructeur, lui, mesure par un
moyen externe.

**Nos chiffres sont une borne inférieure** : ils peuvent sous-estimer la vraie
dispersion, jamais la surestimer.

### Le plancher de la méthode — et pourquoi il n'explique pas les chiffres

`get_angles` rend 2 décimales. À la pose du test, 1 LSB (0,01°) déplace la
pointe de 16 à 58 µm selon l'axe, soit un plancher quadratique de **0,099 mm**.

Nos dispersions non nulles valent 0,19 à 0,84 mm : **2 à 8× au-dessus de ce
plancher**. Elles ne sont donc *pas* un artefact de lecture — ce que la
première rédaction de ce document laissait croire.

Un cas mérite d'être isolé : depuis l'arrière, les 10 relevés sont **bit à bit
identiques**, RP = 0,000. Cela ne veut pas dire « répétabilité parfaite » mais
« dispersion sous le plancher de 0,099 mm » — la seule lecture honnête de ce
zéro.

Ce que les données montrent réellement, c'est que le bras se stabilise sur un
**petit nombre d'états discrets** : sur 10 retours, on relève 1 à 4 positions
distinctes seulement, séparées de 0,4 à 0,9 mm — soit plusieurs LSB sur
plusieurs axes à la fois. C'est une **bande morte d'asservissement**, pas du
bruit de mesure. Le servo s'arrête sur l'un ou l'autre cran, et l'écart entre
crans est du même ordre que la spécification elle-même.

### Définition employée

La répétabilité est donnée en **RP au sens ISO 9283** — moyenne des distances
au barycentre **+ 3σ**. C'est la définition qui sert à annoncer un « 1 mm ».
L'« écart max » cité dans la première version de ce document n'est pas cette
grandeur et **la sous-estime systématiquement** ; les deux sont donnés côte à
côte ci-dessous.

Toute citation de ces chiffres hors de ce document doit porter ces réserves.

---

## 1. Répétabilité — la spec est encadrée, pas tenue

Cible : tag ArUco à (330,5 · 31,8) mm, portée 332 mm, Z = 60 mm.
Outil incliné à **−15°** : à cette portée l'outil vertical **n'a aucune
solution IK** (vérifié sur 8 hauteurs de 40 à 180 mm).

Six séries unidirectionnelles ont été faites, chacune avec un sens d'approche
**constant** — la seule condition dans laquelle la spécification a un sens.

| série | n | RP ISO 9283 | écart max | états | portée du départ | vs ±0,5 (historique) | **vs 1 mm (officiel)** |
|---|---|---|---|---|---|---|---|
| A — par le haut | 10 | **0,403 mm** | 0,306 | 2 | — | sous | **sous** |
| F-UNI — par le haut | 6 | **0,557 mm** | 0,424 | 2 | — | au-dessus | **sous** |
| G — depuis l'arrière | 10 | **0,000 mm** | 0,000 | 1 | 292 mm | sous | **sous** |
| G — depuis la droite | 10 | **0,186 mm** | 0,138 | 2 | 331 mm | sous | **sous** |
| G — depuis l'avant | 10 | **0,794 mm** | 0,551 | 4 | **372 mm** | au-dessus | **sous** |
| G — depuis la gauche | 10 | **0,838 mm** | 0,783 | 3 | 338 mm | au-dessus | **sous** |

**Le robot tient sa spécification dans les six séries**, y compris la pire.

Ce qui reste vrai et intéressant :

- la répétabilité **varie d'un facteur ≥ 4** selon le côté d'arrivée (0,19 à
  0,84 mm, en écartant le 0,000 qui est sous le plancher de mesure) ;
- la pire série est la seule dont le point de départ, à **372 mm**, sort du
  **rayon nominal de 350 mm**. Hors enveloppe, aucune spécification ne
  s'applique ;
- le jugement doit se faire en **RP ISO 9283**, pas en écart max, qui
  sous-estime systématiquement.

Réserve inchangée : la mesure est une **borne inférieure** relevée aux codeurs.
Passer sous le seuil ne prouve pas la conformité — cela échoue seulement à
l'infirmer.

---

## 2. Le sens d'approche — c'est là que tout se joue

Même point d'arrivée, même consigne. Seule l'origine du dernier segment change.

Le test G le mesure proprement : **10 retours depuis chacun des 4 côtés**,
retrait identique de 40 mm, même point d'arrivée, même consigne.

| barycentre atteint | X | Y | Z |
|---|---|---|---|
| depuis l'avant | 322,95 | 24,89 | **44,400** |
| depuis l'arrière | 320,76 | 27,41 | **42,655** |
| depuis la gauche | 321,59 | 28,04 | **42,992** |
| depuis la droite | 323,82 | 25,50 | **46,619** |

**Étalement des quatre barycentres : 5,918 mm.** À comparer aux 0,00–0,84 mm
de dispersion *à l'intérieur* d'une direction : le sens d'approche pèse **7×
la pire des quatre répétabilités**, et davantage encore face aux trois autres.

L'étalement se répartit X 3,06 · Y 3,15 · **Z 3,96 mm**. Le vertical domine,
mais **pas au point que la première rédaction l'affirmait** (« presque
entièrement verticale ») : ce déséquilibre-là venait d'un échantillonnage
bancal — 6 essais répartis inégalement sur 4 côtés. Avec 10 par côté, les trois
axes sont du même ordre.

### Trois mesures indépendantes, trois fois le même chiffre

| date | protocole | biais inter-directions |
|---|---|---|
| 20/08/2026 | autre point, autre jour | 5,88 mm |
| 09/09/2026 — F-MULTI | 6 essais, 4 directions | 5,847 mm |
| 09/09/2026 — G | 40 essais, 10 par côté | **5,918 mm** |

Trois protocoles distincts s'accordent à **0,07 mm près**. C'est le résultat le
mieux établi de toute la campagne — bien mieux que n'importe quel chiffre de
répétabilité.

> **Règle opérationnelle** : ne jamais mélanger les sens d'approche entre
> l'apprentissage d'un point et sa reprise.

---

## 3. Erreur de cible absolue — à ne pas confondre avec un défaut

| grandeur | valeur |
|---|---|
| ‖FK(q_lu) − P_cible‖, **boucle ouverte** | **14,84 mm** (moyenne sur 10) |
| dont vertical | 11,5 mm |
| dont horizontal | 9,4 mm |
| une fois compensé par `converge` | **≈ 2 mm** |

Ces 14,8 mm contiennent l'affaissement documenté (~13 mm). **Ce n'est pas
l'erreur du système asservi.** Citer ce chiffre sans préciser « non compensé »
serait trompeur.

> ⚠ Le modèle `d = L × θ` de la présentation est **indicatif, pas prédictif** :
> il prédit un affaissement croissant avec l'allonge, or on mesure 14,8 mm à
> **332 mm** alors que le modèle donne 13 mm à **390 mm**. Il faudrait mesurer à
> trois portées pour trancher.

---

## 4. La vision n'a pas d'erreur d'échelle

C'est la correction la plus importante de la campagne. Un ArUco de **50 mm**
donnait −3,25 %, ce qui avait été lu comme une erreur d'échelle de 7,4 mm.
**C'est faux.**

| objet mesuré | taille imprimée | mesurée | px | erreur | base |
|---|---|---|---|---|---|
| **tag opérateur** | **50 mm** | 48,374 | — | **−3,252 %** | courte |
| **tag opérateur** | **100 mm** | 100,005 | 46,17 | **+0,005 %** | courte |
| marqueur planche 19 | 50 mm | 49,368 | 24,55 | −1,265 % | courte |
| marqueur planche 25 | 50 mm | 48,595 | 21,80 | −2,811 % | courte |
| marqueur planche 26 | 50 mm | 47,385 | 20,66 | −5,230 % | courte |
| distance 19–25 | 438,4 mm | 437,235 | — | −0,255 % | longue |
| distance 19–26 | 575,9 mm | 574,650 | — | −0,209 % | longue |
| distance 25–26 | 391,0 mm | 389,472 | — | −0,399 % | longue |

**Une vraie erreur d'échelle serait identique à toutes les tailles.** Or :

- le tag de **100 mm** donne **+0,005 %** — la chaîne est juste ;
- les trois marqueurs de **50 mm**, pourtant identiques, donnent −1,27 %,
  −2,81 % et −5,23 % **selon leur obliquité** (les ids 25 et 26 sont les
  lointains, à 558 et 576 mm, près du bord de l'image) ;
- les distances longues (380–580 mm) donnent −0,29 %.

**Conclusion : mesurer le côté d'un petit marqueur oblique n'est pas une mesure
d'échelle.** Le −3,25 % est un artefact de mesure, pas une propriété du système.
Le bloc « −2,6 % → 7,4 mm d'erreur d'échelle » doit être retiré des supports.

> Deux hypothèses ont été essayées et **écartées par la mesure** : une erreur
> d'échelle de la chaîne (le 100 mm l'infirme) et un biais constant de
> localisation des coins (il prédisait −1,55 % sur le 100 mm, on mesure
> +0,005 %).

### Bruit de la vision

Tag et caméra immobiles, 60 trames : la sortie du détecteur est **identique
d'une trame à l'autre**. Le bruit est sous la quantification du détecteur. Ce
n'est pas une précision, c'est une absence de gigue.

---

## 5. Domaine d'allonge

| régime | portée |
|---|---|
| outil vertical | jusqu'à ~320 mm |
| outil couché −15° à −45° | jusqu'à ~390 mm mesurés |
| au-delà | aucune solution IK |

Vérifié ce jour : à **332 mm**, l'outil vertical n'a **aucune** solution ; −15°,
−30° et −45° les ont toutes. L'inclinaison n'est donc pas un choix de confort
mais une **condition d'atteignabilité**, et c'est `INCLINAISONS_PAR_PORTEE` qui
l'impose.

---

## 6. Ce qu'il faut retenir

**La spécification n'est pas le sujet — le protocole l'est.**

Deux choses, dans cet ordre :

1. **Le robot tient son chiffre constructeur.** Les six séries sont sous le
   **1 mm** de la fiche officielle, la pire à 0,838 mm. Réserve : la mesure
   étant une borne inférieure relevée aux codeurs, elle échoue à infirmer la
   conformité, elle ne la démontre pas. Un comparateur à cadran suffirait à
   trancher.

2. **Et c'est secondaire**, parce que le sens d'approche coûte **5,92 mm**,
   c'est-à-dire 7 fois la pire répétabilité et six fois la spécification.
   Dans un tri réel, ce terme écrase tous les autres.

La précision utile ne vient donc ni du constructeur ni du solveur, mais du
**protocole** :

1. affaissement compensé par réinjection de l'écart (14,8 mm → ~2 mm) ;
2. approche toujours dans le même sens (5,92 mm de biais sinon) ;
3. cible refusée si hors domaine, jamais tentée de travers ;
4. la vision mesurée sur une **taille connue**, pas sur une position posée à
   l'œil.

---

## 7. Réserves de méthode

| réserve | portée |
|---|---|
| Répétabilité relevée aux **codeurs** | borne inférieure ; ne voit pas le jeu ni la souplesse en aval. Les séries *sous* la spec ne prouvent donc pas la conformité |
| Plancher instrumental **0,099 mm** | 1 LSB de 0,01° sur les 6 axes ; nos chiffres sont 3–8× au-dessus, donc non limités par la lecture |
| Répétabilité donnée en **RP ISO 9283** | moyenne des distances au barycentre + 3σ ; l'« écart max » la sous-estime |
| Une seule **pose** testée | 332 mm d'allonge, outil à −15° ; la répétabilité varie avec la configuration |
| Test B en **boucle ouverte** | contient l'affaissement ; ≈ 2 mm une fois compensé |
| **Position** du tag posée à l'œil | n'est pas une vérité terrain ; seule sa **taille** en est une |
| Éclairage à 55 de luminance (réf. 86) | l'extrinsèque du jour plafonne à 0,59 mm au lieu de 0,12 |
| Une seule portée testée (332 mm) | l'affaissement n'a pas été mesuré à plusieurs allonges |
| Retrait de **40 mm** au test G | à 50 mm, le départ « avant » tombe à J3 = −0,58° (bras tendu) et le résidu IK passe à 0,611 mm : on aurait mesuré une singularité, pas un sens d'approche |
| Extrinsèques ajustées **uniquement à Z = 0** | validées dans le plan de la table (1,92 mm), **sans aucune validité en hauteur** — voir § 11 |
| Détection ArUco sur **une seule trame** | jusqu'à 17,5 mm d'erreur par trame MJPEG corrompue — voir § 10 |
| **Taille** d'un marqueur mesurée à l'image | biaisée jusqu'à **4 %** par l'obliquité ; la **position**, elle, reste juste à 0,4 mm — voir § 10 |

---

## 8. Ce que mesure vraiment `FK(q_lu)` — et ce qu'il ne mesure pas

Toute la campagne repose sur une seule grandeur :

```
e = FK(q_lu) − P_cible
```

Il faut savoir ce qu'elle contient, sous peine de lui faire dire ce qu'elle ne
dit pas.

### La décomposition

```
P_cible --IK--> q_cmd --servo--> q_phys --codeur--> q_lu --FK_nominale--> P_mesurée
```

d'où :

```
e = [FK(q_lu) − FK(q_cmd)]  +  [FK(q_cmd) − P_cible]
     erreur de suivi servo      résidu d'IK
```

Le résidu d'IK a été mesuré sur les 5 poses du test G : **0,004 à 0,016 mm**.
Négligeable. Il reste donc `e ≃ J(q)·(q_lu − q_cmd)` :

> **`e` est l'erreur de suivi des servos projetée en cartésien par la
> jacobienne. Ce n'est pas la position physique de la pointe.**

| contenu dans `e` | absent de `e` |
|---|---|
| erreur de suivi du servo | écart entre `q_lu` et l'angle réel de l'articulation |
| affaissement gravitaire (le servo se cale décalé sous charge) | erreurs du modèle : longueurs, offsets, zéros |
| répétabilité du codeur | jeu de réducteur en aval du codeur |
| | flexion des bras et de l'outil |
| | erreur de définition du TCP |

### Pourquoi la répétabilité reste valable, mais pas la précision absolue

Sur un **retour répété**, tous les termes systématiques de la colonne de droite
(modèle, offsets, TCP) **s'annulent** : ils sont identiques à chaque passage.
Ne subsistent que les termes non répétables — jeu et flexion. D'où le statut de
**borne inférieure**, et non d'invalidité.

Sur une **précision absolue**, rien ne s'annule. `e` n'est alors pas une mesure
physique et ne doit jamais être présentée comme telle.

À noter : le biais de sens d'approche de 5,9 mm est **visible dans `q_lu`** —
les servos se calent réellement à des angles différents selon le côté d'arrivée.
Le jeu en aval s'y **ajoute** ; le vrai biais est donc ≥ 5,9 mm.

### Le jeu de données hand-eye de juillet est inexploitable

`handeye_data.npz` contient 22 poses avec un ArUco monté sur J6 —
`T_base_gripper` par FK, `T_cam_marker` par la caméra, 155 à 198° de
débattement sur chaque axe. C'est le dispositif qui donnerait la précision
absolue et la validation de l'extrinsèque. Résolution de `A·X = Y·B` :

| | |
|---|---|
| résidu moyen | **24,4 mm** |
| RMS | 26,9 mm |
| max | 55,1 mm |
| désaccord angulaire | 29° en moyenne, 123° au pire |

**La cause n'est pas la FK, c'est le dimensionnement du marqueur.** Il fait
40 mm et la caméra est à 639–1169 mm : son côté apparent vaut **17 à 31 px**,
alors que `COTE_MARQUEUR_PX_MIN = 30`. **20 poses sur 22 sont sous le seuil du
code lui-même.** La profondeur d'un carré de 20 px à 1 m est incertaine de
plusieurs centimètres et son orientation est ambiguë.

Refait avec le tag de 100 mm à 500–800 mm, le côté apparent passerait à ~71 px
et l'incertitude de profondeur à ~2 mm — assez pour détecter une erreur de
modèle ou d'extrinsèque, qui se comptent en millimètres.

### Ce qui exige une référence externe, et ce qui n'en exige pas

| test | référence externe | état |
|---|---|---|
| Répétabilité du bras | **oui**, pour se prononcer formellement sur le 1 mm | borne inférieure seulement |
| Erreur cartésienne reconstruite | non — elle se suffit | fait |
| Précision physique absolue | **indispensable** | impossible par cette méthode |
| Répétabilité de la vision | non — auto-référencée | fait |
| Précision métrique de la vision | **oui** — une longueur connue | fait, référence non vérifiée au pied à coulisse |
| Calibration extrinsèque | **oui** — un point indépendant | auto-cohérence seulement |
| Précision globale vision + robot | non — l'objet est sa propre cible | à chiffrer |

Le critère : **une référence externe devient nécessaire dès que l'affirmation
porte sur le robot lui-même plutôt que sur la boucle de commande.**

### Conséquence pour l'asservissement visuel

En boucle fermée, la précision absolue du bras **n'a pas d'importance** : la
boucle annule toute erreur de modèle constante. Les 14,8 mm de boucle ouverte
sont ce que la boucle est là pour absorber.

Ce qui limite réellement la boucle est ce qui n'est **pas** répétable : le bruit
de la vision, le jeu, et le biais de 5,9 mm selon le sens d'arrivée. D'où une
contrainte de conception : **si le dernier segment d'approche change de
direction d'une itération à l'autre, on injecte 5,9 mm de bruit dans une boucle
qui cherche à converger au millimètre.** Terminer toujours par le même vecteur
d'approche.


---

## 9. Test 7 — la précision du système tel qu'il fonctionne

**C'est le seul chiffre qui décrit la chaîne complète**, et il manquait. Les
tests A à H mesuraient des morceaux : la répétabilité du bras, le bruit de la
caméra, la justesse de l'extrinsèque. Aucun ne disait ce qui compte vraiment :
*quand la caméra dit au robot d'aller quelque part, il arrive à combien ?*

### Le protocole, en clair

**20 essais.** Cible : un tag ArUco 50 mm posé au **centre** du plan de travail,
à (260,49 · 21,66) mm, portée 261 mm. Caméra **arducam seule**.

Chaque essai enchaîne :

| étape | ce qui se passe |
|---|---|
| 1 | le bras s'écarte à la pose d'observation, il ne masque plus rien |
| 2 | la caméra détecte le tag — **moyenne de 6 trames**, jamais une seule |
| 3 | le bras y va **en une seule fois, sans correction** → **passe 0** |
| 4 | on réinjecte l'écart articulaire consigne − mesure → **passe 1** |
| 5 | on réinjecte de nouveau → **passe 2** |
| 6 | retour, et la caméra revoit le tag pour **prouver qu'il n'a pas bougé** |

Survol à **Z = 30 mm — aucun contact avec la table.** Pince fermée, parce que le
déport d'outil désigne le bout des doigts fermés.

### Les résultats

| passe | ce que c'est | erreur moy. | biais | dispersion (RP ISO) | erreur Z | retard | < 2 mm | < 1 mm |
|---|---|---|---|---|---|---|---|---|
| **0** | **boucle ouverte, aucune correction** | **10,79 mm** | 10,79 | 0,671 mm | −9,78 mm | 2,07° | **0/20** | 0/20 |
| **1** | après **une** réinjection | **1,29 mm** | 1,27 | 0,810 mm | −0,42 mm | 0,24° | **20/20** | 4/20 |
| **2** | après **deux** | **0,63 mm** | 0,54 | 0,956 mm | **+0,06 mm** | 0,17° | 20/20 | **18/20** |

### Ce qu'il faut lire dans ce tableau

Regarde les deux colonnes du milieu, pas la première.

**Le biais s'effondre de 10,79 à 0,54 mm — vingt fois moins. La dispersion, elle,
ne bouge pas : 0,230 puis 0,291 puis 0,337 mm.**

C'est la signature d'une erreur **purement systématique**. L'affaissement
gravitaire tire toujours dans le même sens — **X −7,65 et Y −7,60 mm, jamais
l'inverse sur 20 essais**. Ce n'est pas du hasard qu'on moyenne, c'est un décalage
qu'on annule.

> Un biais se rattrape par une correction. Une dispersion, non.
> **C'est exactement la condition pour qu'un asservissement visuel fonctionne :
> la boucle annule toute erreur de modèle constante.**

En Z c'est plus net encore : **−9,78 mm** en boucle ouverte, **+0,06 mm** après
deux passes. Les 10 mm d'affaissement sont intégralement repris.

### Les contrôles qui rendent le chiffre crédible

| contrôle | résultat | ce que ça écarte |
|---|---|---|
| le tag a-t-il bougé ? | **0,767 mm** max sur 20 vérifications | la cible n'a pas dérivé pendant la série |
| résidu de la cinématique inverse | **0,021 mm** moyen, 0,024 max | l'IK n'est pour rien dans l'erreur |
| régime de détection | moyenne de 6 trames | pas de trame corrompue (§ 10) |

### La réserve, en toutes lettres

**C'est une borne inférieure.** La position atteinte est reconstruite par
`FK(q_lu)` depuis les codeurs : elle ignore l'écart codeur↔articulation, les
erreurs du modèle cinématique, le jeu des réducteurs et la flexion. Le vrai
chiffre physique est **plus grand**. Le mesurer exige un comparateur à cadran.

---

## 10. Le bruit de la vision — et un piège qui coûtait 17 mm

Le test C concluait à une « absence de gigue » : la détection rendait exactement
la même valeur d'une trame à l'autre. **C'était un artefact.** Le détecteur ArUco
par défaut ne raffine pas ses coins au sous-pixel ; sur une scène immobile il
retombe sur les mêmes pixels entiers.

Refait avec `CORNER_REFINE_SUBPIX`, 60 détections, bras écarté, scène immobile :

| méthode | n | l̄ | σ | RP ISO 9283 | max |
|---|---|---|---|---|---|
| détection brute, 1 trame | 60 | 0,610 mm | 2,220 | 7,270 mm | **17,46 mm** |
| sous-pixel, 1 trame | 60 | 0,632 mm | 2,251 | 7,384 mm | 17,52 mm |
| **sous-pixel + moyenne de 6 trames** | 55 | **0,141 mm** | 0,195 | **0,725 mm** | **0,80 mm** |

**Le sous-pixel n'était pas le problème** — il ne change presque rien. Le vrai
coupable, ce sont les **trames corrompues** : le flux MJPEG du pilote perd
régulièrement des segments (`Corrupt JPEG data`), l'image arrive tronquée et le
marqueur est détecté de travers, **jusqu'à 17,5 mm**. Rare, mais énorme.

Moyenner six trames l'élimine : le maximum tombe de **17,5 mm à 0,80 mm**.

> **Règle opérationnelle.** Ne jamais commander le robot sur une seule trame.
> Une prise sur trame unique rate de temps en temps, de près de 2 cm, **sans que
> rien ne le signale**.

### Le tag de 50 mm fait bien 50 mm — correction du 10/09

**Ce rapport a affirmé le contraire, et c'était faux.** Il concluait à une
erreur d'impression de −3,38 % sur la foi de six mesures concordantes. Les six
étaient biaisées **dans le même sens par la même cause** : ce n'étaient pas six
confirmations indépendantes, c'était six fois la même erreur.

#### La mesure qui tranche

Cinq marqueurs, **12 acquisitions chacun**, extrinsèque fraîchement recalibrée
(RMS 0,482 px, positions justes à 0,11–0,42 mm) :

| marqueur | côté mesuré | obliquité | taille à l'image |
|---|---|---|---|
| 19 | **49,725 ± 0,019 mm** | 1,0096 | 24,9 px |
| 25 | 49,154 ± 0,040 | 1,0197 | 22,3 px |
| 23 | 48,472 ± 0,107 | 1,0181 | 22,9 px |
| 1 (tag central) | 48,306 ± 0,131 | 1,0246 | 22,6 px |
| 26 | 47,749 ± 0,242 | 1,0287 | 20,9 px |

Ces marqueurs sont **physiquement identiques**, imprimés sur la même feuille.

| | |
|---|---|
| répétabilité sur **un même** marqueur | **± 0,108 mm** |
| étalement entre **marqueurs identiques** | **1,98 mm — 4,0 %** |

**L'étalement est 18 fois plus grand que la répétabilité.** La mesure est donc
*précise* mais pas *juste* : elle répète très bien une valeur fausse.

#### D'où vient le biais

| le côté mesuré est corrélé à… | r |
|---|---|
| **l'obliquité** | **−0,920** |
| la taille à l'image | +0,844 |
| la distance au centre de l'image | +0,676 |

L'obliquité domine. Ajustement `côté = −97,96 × obliquité + 148,62`, résidus
**0,27 mm RMS**. Extrapolé à une vue **parfaitement de face** (obliquité =
1,000) :

> ### **50,65 mm**

#### La preuve indépendante : l'échelle longue est juste

Si la caméra avait une erreur d'échelle, elle se verrait **à toutes les
longueurs**. On compare donc les grandes distances entre centres de marqueurs
aux valeurs du relevé :

| paire | ruban | caméra | écart |
|---|---|---|---|
| 19–23 | 382,6 | 382,45 | −0,047 % |
| 19–25 | 438,4 | 438,33 | −0,005 % |
| 19–26 | 575,9 | 575,85 | −0,001 % |
| 23–25 | 579,5 | 579,13 | −0,071 % |
| 23–26 | 420,0 | 420,59 | +0,137 % |
| 25–26 | 391,0 | 389,95 | −0,278 % |

**Erreur d'échelle sur 383 à 580 mm : −0,044 %.** Contre **−2,6 %** sur les
côtés de 50 mm. Une erreur d'échelle réelle frapperait les deux à l'identique.
Elle ne frappe que les petites longueurs.

#### Conclusion

**La calibration est juste. C'est la détection des coins ArUco qui rétrécit les
petits carrés vus de biais.** Les tags font 50 mm.

#### Ce que ce biais affecte, et ce qu'il n'affecte pas

| grandeur | état |
|---|---|
| **position** d'un marqueur (son centre) | **juste** — 0,11 à 0,42 mm |
| distances entre marqueurs | **justes** — 0,044 % |
| **taille** d'un petit marqueur vu de biais | **fausse jusqu'à 4 %** |

C'est cohérent géométriquement : la détection tire les quatre coins vers
l'intérieur de façon à peu près symétrique. Le carré rétrécit, **son centre ne
bouge pas**. Une erreur de taille ne se propage donc **pas** en erreur de
position.

#### Le degré de certitude

**Conclusion établie, à 0,7 mm près.** 50,65 mm mesurés, dont 0,27 mm de résidu d'ajustement et ~0,6 mm venant de la référence d'échelle (relevé au ruban, ±5 mm sur 400). Soit **50,7 ± 0,7 mm** : cela encadre 50 et **exclut 48,3**, à plus de trois écarts. `marker_size_mm` reste donc à **50.0** — c'était déjà sa valeur, elle n'a jamais été modifiée.

Un artefact étalonné (pied à coulisse) resserrerait à 0,05 mm et satisferait VDI/VDE 2634-1, mais **ne changerait aucune décision** : l'argument ne repose pas sur la valeur absolue, il repose sur le fait qu'une erreur d'échelle serait identique à toutes les longueurs, or elle vaut −0,044 % sur 383–580 mm et −2,6 % sur 50 mm.

---

## 11. Les extrinsèques ne valent que dans le plan de la table

C'est la découverte la plus lourde de la campagne, et elle a été trouvée en
cherchant tout autre chose.

### Le point de départ

`tool_offset.json` porte une contradiction inscrite dans ses propres champs :
deux définitions du bout des doigts distantes de **17,33 mm**, dont l'une est
forcément fausse. Pour la trancher optiquement, la SVPRO a été recalibrée — elle
avait dérivé de **21,3 mm** depuis le 24/08 et **6,6 mm** depuis le 02/09.
Recalibrée sur la planche, elle retombe à **0,38–0,72 mm**.

Son marqueur 25 reste indétectable : il est **physiquement présent**, mais son
bord bas tombe **8,8 px sous le cadre**, et ArUco exige les quatre coins.
Vérifié à toutes les résolutions du capteur (2592×1944 jusqu'à 640×480) — le
champ est identique, ce n'est pas une question de définition. Le corriger
demande de bouger la **caméra**, jamais la planche.

### La validation croisée, sur un point que personne n'a utilisé

Le tag posé au centre n'a servi à **aucun** des deux ajustements : l'arducam est
ajustée sur 19/23/25/26, la SVPRO sur 19/23/26. Chacune le prédit indépendamment.

| point | arducam (X,Y) | SVPRO (X,Y) | désaccord |
|---|---|---|---|
| **tag central — jamais utilisé** | 260,84 / 21,59 | 259,44 / 22,90 | **1,92 mm** |
| marqueur 19 | 96,66 / 202,66 | 96,78 / 203,96 | 1,30 mm |
| marqueur 26 | 529,43 / −175,68 | 529,93 / −175,61 | 0,50 mm |

Ces **1,92 mm** bornent l'erreur des **deux chaînes réunies**. C'est le premier
contrôle vraiment indépendant de la campagne : pas un résidu qui se mesure
lui-même, mais deux intrinsèques, deux extrinsèques et deux angles de vue qui
convergent sur un point neuf.

### Et pourtant

Les quatre marqueurs de planche sont **tous à Z = 0**. Douze coins coplanaires
fixent très bien la pose **dans** le plan — d'où les 0,4 mm de résidu et les
1,92 mm de recoupement — mais **contraignent mal la rotation hors plan**. C'est
la dégénérescence classique de la cible plane, et **le résidu ne peut pas la
voir**, puisqu'il est mesuré sur ce même plan.

Contrôle direct, pince à **Z = 172 mm**, patins verts vus par les deux caméras :

| patin | l'arducam dit | la SVPRO dit |
|---|---|---|
| arrière | (192 / +7) | (299 / +127) |
| avant | (194 / −43) | (243 / +153) |

Or **toute la pince tient entre Y = +92 mm** (le flasque) **et Y = −20 mm** (la
pointe). La SVPRO les place *derrière* le flasque, là où il n'y a rien. La
triangulation des deux rayons rend un point à **Z = −12 mm, sous la table** :
physiquement impossible.

### Ce que ça change

| | |
|---|---|
| **Reste valide** | viser un objet **posé sur la table** — le pick-and-place, validé à 1,92 mm, et le test 7 du § 9 |
| **N'est pas valide** | toute mesure **en hauteur** : pince en vol, hauteur d'un objet, obstacle |
| **Pour lever la limite** | des marqueurs à **plusieurs hauteurs** — un tag collé sur une boîte de hauteur connue suffit |

Deux ou trois niveaux rendent la rotation hors plan observable, et débloquent du
même coup la mesure du déport d'outil.

### Pourquoi le déport n'a pas pu être tranché

Les deux candidats sont distants de 17,33 mm, mais **16,9 mm de cet écart est
vertical** et 3,9 mm seulement latéral. Les deux caméras regardent d'en haut :

| caméra | élévation | séparation des deux candidats dans l'image |
|---|---|---|
| arducam | ~85° | < 2 px |
| SVPRO | 58° | **4,2 px** |

**La mesure n'est pas dans l'image.** Ni la calibration ni l'algorithme n'y
peuvent rien.

À défaut de preuve, le faisceau : le déport actuel repose sur **7 mesures au
réglet, 3 azimuts couvrant 59°, écart-type 1,0 mm**. Ce qui le contredit est
**un seul** point enseigné du 18/08, à une pose qui n'est plus atteignable
(J2 à −135,7°, au-delà de la butée −135°).

### Une méthode écartée, et pourquoi elle l'a été

La première approche était de **descendre jusqu'au contact** et de lire la
hauteur. Elle a été rejetée, à raison : le bras a **1,0 à 2,1° de retard de
suivi**, soit 6 à 10 mm en cartésien. Une détection de contact par écart de
codeurs ne se déclenche donc qu'après plusieurs millimètres d'appui. **La
résolution du protocole était pire que la grandeur cherchée** — et il abîmait le
matériel.

> Règle générale : avant de proposer un protocole, chiffrer sa résolution et la
> comparer à la grandeur cherchée. Si elle n'est pas nettement meilleure, ne pas
> le proposer, même s'il est simple.

---

## 12. Les sept types d'essai — état au 10 septembre 2026

| # | essai | état | résultat |
|---|---|---|---|
| 1 | Répétabilité du bras | fait — borne inférieure | 0,00 à 0,84 mm, **6/6 sous la spec de 1 mm** |
| 2 | Erreur cartésienne reconstruite | fait | 14,84 mm brut · ≈2 mm compensé |
| 3 | **Précision physique absolue** | **non fait** | inaccessible sans comparateur (~60 €) |
| 4 | Répétabilité de la vision | **refait le 10/09** | **0,725 mm** (6 trames) · 17,5 mm (1 trame) |
| 5 | Précision métrique de la vision | **repris le 10/09** | échelle longue **−0,044 %** · le tag fait bien **50 mm** |
| 6 | Calibration extrinsèque | fait — **et sa limite trouvée** | 1,86 mm en interpolation · **nulle hors du plan** |
| 7 | **Précision globale vision + robot** | **fait le 10/09** | **10,79 mm** brut → **0,63 mm** après 2 corrections |

Il reste **un seul** essai non fait, et c'est le seul qui exige d'acheter
quelque chose.
