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
