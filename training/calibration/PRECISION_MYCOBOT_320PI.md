# Précision du MyCobot 320 Pi — campagne du 9 septembre 2026

Ce que le robot tient, ce qu'il ne tient pas, et **ce que la spécification
constructeur ne dit pas**.

Données brutes : [`precision_campagne_2026-09-09.csv`](precision_campagne_2026-09-09.csv)
· [`.xlsx`](precision_campagne_2026-09-09.xlsx) (9 feuilles)
· [`repetabilite_4directions_2026-09-09.csv`](repetabilite_4directions_2026-09-09.csv)

---

## 0. La confusion à ne plus faire

> **Les ±0,5 mm d'Elephant Robotics sont une *repeated positioning precision* —
> une RÉPÉTABILITÉ. Ce n'est pas une précision absolue de positionnement
> cartésien.**

| grandeur | définition | chiffre constructeur |
|---|---|---|
| **Répétabilité** | dispersion en revenant plusieurs fois au **même** point, dans les **mêmes** conditions | **±0,5 mm** |
| **Erreur de cible absolue** | ‖FK(q_lu) − P_cible‖ | *aucun publié* |
| **Précision de la vision** | ce que la caméra dit d'un objet de taille connue | *sans objet* |

Comparer notre erreur absolue aux ±0,5 mm et conclure « hors spec » est une
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
dispersion, jamais la surestimer. Indice visible dans les données — sur 10
essais, seules **2 valeurs distinctes** apparaissent : on mesure la
répétabilité du *relevé codeur*, quantifié, pas celle du bras.

Toute citation de ces chiffres hors de ce document doit porter cette réserve.

---

## 1. Répétabilité — la spec est tenue

Cible : tag ArUco à (330,5 · 31,8) mm, portée 332 mm, Z = 60 mm.
Outil incliné à **−15°** : à cette portée l'outil vertical **n'a aucune
solution IK** (vérifié sur 8 hauteurs de 40 à 180 mm).

| test | n | écart max | étendue 3D | verdict |
|---|---|---|---|---|
| A — retours par le haut | 10 | **0,306 mm** | 0,510 mm | **sous les ±0,5 mm** |
| F-UNI — retours par le haut | 6 | **0,424 mm** | 0,509 mm | **sous les ±0,5 mm** |

Le robot **tient sa spécification**. La valeur de 0,67 mm citée auparavant
était moins bonne que ce que la machine sait faire, et surtout elle était
présentée comme un dépassement, ce qu'elle n'est pas.

---

## 2. Le sens d'approche — c'est là que tout se joue

Même point d'arrivée, même consigne. Seule l'origine du dernier segment change.

| test | n | écart max | étendue 3D |
|---|---|---|---|
| F-UNI — toujours par le haut | 6 | 0,424 mm | 0,509 mm |
| F-MULTI — 4 directions alternées | 6 | **5,847 mm** | **10,900 mm** |

**Dégradation ×21,4**, et elle est **presque entièrement verticale**
(σZ = 3,90 mm contre σX = 1,48 et σY = 1,84) :

```
retours par le haut     -> Z = 48,480 · 48,600 mm
retours par l arriere   -> Z = 40,949 · 39,741 mm
retours par la gauche   -> Z = 42,144 mm
retours par la droite   -> Z = 41,811 mm
```

**~7,5 mm d'écart en Z pour une consigne identique.** L'explication est
physique : venir du haut, c'est descendre contre la gravité et s'arrêter avant ;
venir de côté, c'est arriver avec le bras déjà fléchi.

Ce résultat **reconfirme les 5,88 mm mesurés le 20/08** — 5,847 mm, autre jour,
autre point, autre protocole.

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

**La spécification constructeur est tenue — et elle ne suffit pas.**

Les ±0,5 mm décrivent un retour dans des conditions *identiques*. Dès qu'on
change de sens d'approche, ce qui est le cas normal d'un tri, la dispersion
passe à 5,85 mm **sans que le robot soit en défaut**.

La précision utile ne vient donc ni du constructeur ni du solveur, mais du
**protocole** :

1. affaissement compensé par réinjection de l'écart (14,8 mm → ~2 mm) ;
2. approche toujours dans le même sens (×21 sinon) ;
3. cible refusée si hors domaine, jamais tentée de travers ;
4. la vision mesurée sur une **taille connue**, pas sur une position posée à
   l'œil.

---

## 7. Réserves de méthode

| réserve | portée |
|---|---|
| Répétabilité relevée aux **codeurs** | borne inférieure ; ne voit pas le jeu ni la souplesse en aval |
| Test B en **boucle ouverte** | contient l'affaissement ; ≈ 2 mm une fois compensé |
| **Position** du tag posée à l'œil | n'est pas une vérité terrain ; seule sa **taille** en est une |
| Éclairage à 55 de luminance (réf. 86) | l'extrinsèque du jour plafonne à 0,59 mm au lieu de 0,12 |
| Une seule portée testée (332 mm) | l'affaissement n'a pas été mesuré à plusieurs allonges |
