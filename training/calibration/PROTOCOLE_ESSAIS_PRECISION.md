# Protocole des essais de précision — myCobot 320 Pi

*Campagne des 8, 9 et 10 septembre 2026. Ce document est la **table de
correspondance** de la campagne : pour chaque essai, ce qui a été fait, contre
quelle norme, ce que ça a donné, et **où le retrouver** dans les quatre
supports.*

Il ne remplace ni le rapport de résultats ni la note de méthodologie — il les
**indexe**. Quand un chiffre est cité ailleurs sans qu'on sache d'où il sort,
c'est ici qu'on le retrouve.

---

## 1. Les quatre supports, et ce que chacun porte

| support | fichier | rôle |
|---|---|---|
| **Résultats** | [`PRECISION_MYCOBOT_320PI.md`](PRECISION_MYCOBOT_320PI.md) | les chiffres, leur lecture, les réserves |
| **Méthodologie** | [`METHODOLOGIE_PRECISION.md`](METHODOLOGIE_PRECISION.md) | pourquoi ces essais, contre quelles normes, ce que la méthode ne peut pas mesurer |
| **Données** | `precision_campagne_2026-09-09.xlsx` | 17 feuilles — les relevés bruts, essai par essai |
| **Rapport** | `RAPPORT_COMPLET_PRECISION_320PI.docx` | 17 sections, 35 tableaux — version diffusable |

> Les `.xlsx` et `.docx` **ne sont pas versionnés** (règle du dépôt). Les deux
> `.md` le sont. En cas de divergence, **le `.md` fait foi**.

### Légende des liens

Chaque fiche d'essai se termine par un bloc **« Liens partagés »** à quatre
entrées, toujours dans le même ordre :

```
Résultats  → PRECISION_MYCOBOT_320PI.md § <n> « <titre> »
Méthode    → METHODOLOGIE_PRECISION.md § <n> « <titre> »
Données    → classeur, feuille « <nom exact de l'onglet> »
Rapport    → RAPPORT_COMPLET_PRECISION_320PI.docx § <n>
Script     → <chemin>
```

---

## 2. Tableau maître

| code | essai | norme | résultat | feuille Excel | § Word |
|---|---|---|---|---|---|
| **A** | Répétabilité, 10 A/R verticaux | ISO 9283:1998 | **0,403 mm** | `A repetabilite haut` | 3 |
| **F-UNI** | Répétabilité, 6 A/R (contrôle) | ISO 9283:1998 | **0,557 mm** | `F-UNI repetabilite haut` | 4 |
| **G** | 40 A/R, 10 par côté | ISO 9283:1998 | **0,000 à 0,838 mm** | `G 4 directions x10` | 5 |
| **F-MULTI** | Sens d'approche mélangés | *hors norme* | **5,847 mm** | `F-MULTI 4 directions` | 6 |
| **D** | Approche latérale | *hors norme* | **8,091 mm** | `D approche laterale` | 6 |
| **B** | Erreur de cible absolue, boucle ouverte | ISO 9283 § exactitude (AP) | **14,84 mm** → ≈2 mm compensé | `B erreur absolue` | 7 |
| **C** | Bruit de la vision, 60 trames | VDI/VDE 2634-1 (déclaré redondant) | **0,725 mm** / 17,5 mm | `C bruit vision` | 8 |
| **E** | Échelle de la vision, 8 objets | VDI/VDE 2634-1 § `Δl = l_m − l_k` | **−0,044 %** | `E echelle vision` | 9 |
| **H** | Extrinsèque, *leave-one-out* | *aucune norme connue* | **1,86 mm** | `H extrinseque leave-one-out` | 10 |
| **7 / I** | Précision globale vision + robot | *aucune norme* | **10,79 → 0,63 mm** | `I test 7 boucle fermee` | 13 |
| **J** | Bruit vision sous-pixel, trames corrompues | diagnostic | **17,5 mm** sur 1 trame | `J bruit vision sous-pixel` | 14 |
| **K** | Validation croisée 2 caméras | *aucune norme* | **1,92 mm** | `K svpro et validite du plan` | 15 |
| **L** | Taille réelle des marqueurs | VDI/VDE 2634-1 § artefact étalonné | **50 mm** (50,65 extrapolé) | `L taille des marqueurs` | 16 |

Feuilles de service du classeur : `Comment lire ce classeur`, `Synthese`,
`Methodologie ISO 9283`, `Les 7 essais`.

---

## 3. Fiches par essai

### A — Répétabilité de pose, approche unidirectionnelle

**Objet.** La dispersion du bras quand il revient au même point, toujours par le
même chemin.

**Norme.** ISO 9283:1998, définition de la **répétabilité de pose** :
`RP = l̄ + 3·S_l` — distance moyenne au barycentre plus trois écarts-types.
**Ne jamais utiliser l'écart maximum** : il sous-estime RP systématiquement.

**Mode opératoire.**
1. Une pose de travail unique, bras à vide.
2. 10 allers-retours : montée de 40 mm, redescente sur le même point.
3. **Sens d'approche constant, toujours par le haut.**
4. Lecture de `FK(q_lu)` après stabilisation, pas après accusé de commande.
5. Calcul du barycentre des 10 points, puis `l̄ + 3S_l`.

**Résultat. 0,403 mm.**

**Réserve.** C'est une **borne inférieure** : la mesure est reconstruite aux
codeurs, elle ne voit ni la flexion des liens ni le jeu des réducteurs.

**Écart au protocole normatif.** 1 pose au lieu de 5, 10 cycles au lieu de 30,
à vide au lieu de 100 % de charge nominale, vitesse réduite au lieu de 100 %.
La **définition** est respectée, pas le **protocole**.

> **Liens partagés**
> Résultats → `PRECISION_MYCOBOT_320PI.md` § 1 « Répétabilité — la spec est encadrée, pas tenue »
> Méthode → `METHODOLOGIE_PRECISION.md` § 3, essai 1 « Répétabilité du bras »
> Données → feuille `A repetabilite haut`
> Rapport → § 3
> Script → `campagne_precision.py` *(bac à sable)*

---

### F-UNI — Contrôle indépendant de A

**Objet.** Vérifier que le 0,403 mm de A n'est pas un coup de chance.

**Mode opératoire.** Identique à A, série indépendante, 6 allers-retours.

**Résultat. 0,557 mm.** Même ordre de grandeur, la conclusion tient.

> **Liens partagés**
> Résultats → § 1 · Méthode → § 3 essai 1 · Données → `F-UNI repetabilite haut` · Rapport → § 4
> Script → `test_4directions.py` *(bac à sable)*

---

### G — Répétabilité côté par côté, 40 allers-retours

**Objet.** La répétabilité dépend-elle de la direction d'approche ?

**Mode opératoire.** Quatre séries **unidirectionnelles** de 10 A/R, une par
côté (+X, −X, +Y, −Y). Chaque série est traitée séparément, jamais mélangée.

**Résultat. 0,000 à 0,838 mm** selon le côté — **6 séries sur 6 sous la spec de
1 mm**.

**Ce que ça apporte.** C'est le seul essai qui explore la dépendance au sens
d'approche, que la norme n'impose pas. Il établit que **chaque direction prise
isolément est bonne**.

> **Liens partagés**
> Résultats → § 2 « Le sens d'approche — c'est là que tout se joue »
> Méthode → § 3 essai 1 · Données → `G 4 directions x10` · Rapport → § 5
> Script → `test_directions.py` *(bac à sable)*

---

### F-MULTI et D — Sens d'approche mélangés

**Objet.** Ce qui se passe quand on **mélange** les directions.

**Statut normatif.** **Hors du champ d'ISO 9283 par construction.** La norme
définit la répétabilité *« in the same direction »*. Une série multidirectionnelle
ne mesure pas la répétabilité au sens normatif — elle mesure un **biais de jeu
mécanique**.

**Résultat. 5,847 mm** (F-MULTI) et **8,091 mm** (D) — soit **10 à 20 fois** les
séries unidirectionnelles.

**Conséquence opérationnelle, la plus importante de la campagne.**
**Ne jamais mélanger les sens d'approche entre l'apprentissage d'un point et sa
reprise.** Un point appris par le haut doit être repris par le haut.

> **Liens partagés**
> Résultats → § 2, sous-section « Trois mesures indépendantes, trois fois le même chiffre »
> Méthode → § 3 essai 1 · Données → `F-MULTI 4 directions`, `D approche laterale` · Rapport → § 6

---

### B — Erreur de cible absolue, boucle ouverte

**Objet.** L'écart entre la position commandée et la position atteinte, sans
aucune correction.

**Norme.** Relève de l'**exactitude de pose (AP)** d'ISO 9283 — mais **n'y
satisfait pas** : l'AP exige une mesure **externe**, la nôtre est reconstruite
aux codeurs.

**Résultat. 14,84 mm** brut, **≈2 mm** après compensation de l'affaissement
gravitaire.

**Réserve dirimante.** Ce chiffre ne peut pas être présenté comme une exactitude
au sens de la norme. Voir l'essai **3** au § 4, non fait.

> **Liens partagés**
> Résultats → § 3 « Erreur de cible absolue — à ne pas confondre avec un défaut »
> Méthode → § 3 essai 2 · Données → `B erreur absolue` · Rapport → § 7

---

### C — Bruit propre de la vision

**Objet.** De combien la vision bouge toute seule, scène immobile.

**Norme.** VDI/VDE 2634-1 traite l'erreur de palpage — et **déclare son essai
séparé non nécessaire** (*« separate testing of the probing error is not
required »*). Gardé ici comme **diagnostic**, pas comme essai normatif.

**Mode opératoire.** 60 trames consécutives, aucun mouvement, avec et sans
raffinement sous-pixel.

**Résultats.**
- raffinement sous-pixel : **sans effet utile** (0,610 → 0,632 mm) ;
- **une seule trame** peut être fausse de **17,5 mm** — trames MJPEG corrompues ;
- **moyenne sur 6 trames** : RP **0,725 mm**, maximum 0,80 mm.

**Règle opérationnelle qui en découle.** **Ne jamais commander le robot sur une
seule trame.** Six trames médianées, au minimum.

**Ce que ce diagnostic a rapporté.** C'est lui, et non un essai normatif, qui a
trouvé les trames corrompues.

> **Liens partagés**
> Résultats → § 10 « Le bruit de la vision — et un piège qui coûtait 17 mm »
> Méthode → § 3 essai 4 · Données → `C bruit vision`, `J bruit vision sous-pixel` · Rapport → § 8 et § 14
> Script → `bruit_vision.py` *(bac à sable)*

---

### E — Échelle métrique de la vision

**Objet.** La vision a-t-elle une erreur d'échelle ?

**Norme.** VDI/VDE 2634-1, **erreur de mesure de longueur** `Δl = l_m − l_k`,
avec tolérance `E = A + K·L ≤ B`.

**Mode opératoire.** Deux familles de longueurs, sur la même image :
- **longues** : distances entre centres de marqueurs, 383 à 580 mm ;
- **courtes** : côtés des marqueurs de 50 mm.

**Résultats. −0,044 %** sur les longues distances contre **−2,6 %** sur les
côtés de 50 mm.

**Lecture — et c'est le point le plus important du document.** Une vraie erreur
d'échelle frapperait **les deux à l'identique**. Elle ne le fait pas. Donc il
n'y a **pas** d'erreur d'échelle : le −2,6 % sur les petits côtés est un **biais
de détection**, traité à l'essai **L**.

**Non-conformité assumée.** Artefact **non étalonné**, un seul plan au lieu du
volume, aucune tolérance `A + K·L` déclarée. C'est précisément cette
non-conformité qui a laissé vivre deux jours une erreur d'interprétation.

> **Liens partagés**
> Résultats → § 4 « La vision n'a pas d'erreur d'échelle »
> Méthode → § 3 essai 5, **réécrit le 10/09** · Données → `E echelle vision` · Rapport → § 9
> Script → `test_echelle.py` *(bac à sable)*

---

### H — Justesse de l'extrinsèque par *leave-one-out*

**Objet.** L'extrinsèque prédit-elle un marqueur qu'elle n'a **pas** servi à
l'ajuster ?

**Norme.** **Aucune connue.** Protocole construit pour l'occasion.

**Mode opératoire.**
1. Ajuster la pose caméra sur **3** marqueurs seulement.
2. Prédire la position du **4ᵉ**.
3. Comparer à son relevé.
4. Recommencer en changeant le marqueur exclu.

**Pourquoi ce protocole.** Un ajustement sur 4 marqueurs qui reproduit ces 4
marqueurs ne prouve rien — **il est circulaire**. Seule la prédiction d'un point
exclu mesure quelque chose.

**Résultat. 1,86 mm** en interpolation.

> **Liens partagés**
> Résultats → § 8 · Méthode → § 4 « Validation de l'extrinsèque par *leave-one-out* »
> Données → `H extrinseque leave-one-out` · Rapport → § 10
> Script → [`scripts/calibrer_extrinseque_4aruco.py`](../../scripts/calibrer_extrinseque_4aruco.py) *(versionné)*

---

### 7 (I) — Précision globale vision + robot, en boucle fermée

**Objet.** **Le chiffre qui compte.** Quand la caméra dit « la cible est là », à
combien le robot y arrive ?

**Norme.** **Aucune.** ISO 9283 mesure le bras seul, VDI/VDE 2634 l'optique
seule ; un système asservi par vision tombe entre les deux.

**Mode opératoire.**
1. Détecter la cible à la caméra, 6 trames médianées.
2. Résoudre l'IK, envoyer, **attendre la stabilisation** — pas l'accusé de
   commande.
3. Mesurer l'écart, **réinjecter l'écart articulaire** : `correction += (q_cible − q_lu)`.
4. Répéter deux fois.
5. **20 essais**, trois mesures chacun (brut, 1 correction, 2 corrections).

**Résultats.**

| | écart moyen | dispersion |
|---|---|---|
| boucle ouverte | **10,79 mm** | 0,230 mm |
| après 1 correction | **1,29 mm** | 0,291 mm |
| après 2 corrections | **0,63 mm** | 0,337 mm |

En Z : **−9,78 mm → +0,06 mm**. **18 essais sur 20 sous le millimètre.**

**La lecture, en une phrase.** Le biais s'effondre d'un facteur 20 pendant que
**la dispersion reste plate** — donc l'erreur est **purement systématique**,
donc **corrigeable par une boucle visuelle**. C'est ce qui justifie
l'asservissement plutôt qu'un recalage statique.

**Contrôles de validité.** Le tag n'a pas bougé de plus de **0,767 mm** pendant
la série ; résidu IK **0,021 mm**.

> **Liens partagés**
> Résultats → § 9 « Test 7 — la précision du système tel qu'il fonctionne »
> Méthode → § 8 « Test 7 — précision globale vision + robot »
> Données → `I test 7 boucle fermee` · Rapport → § 13
> Scripts → `test7b.py`, `analyse7b.py` *(bac à sable)*

---

### K — Validation croisée deux caméras, et la limite du plan

**Objet.** Deux caméras indépendantes voient-elles le même point au même
endroit ?

**Norme.** **Aucune** ; l'esprit de VDI/VDE 2634-3 (vues multiples).

**Mode opératoire.** Un point **neuf**, jamais utilisé par aucun des deux
ajustements, mesuré par l'arducam et par la SVPRO.

**Résultat. 1,92 mm.** C'est **le contrôle le plus solide de la campagne** :
deux chaînes optiques séparées, aucune donnée partagée.

**Et la découverte qui va avec.** Trianguler les mors de la pince à 172 mm de
haut donnait **Z = −12 mm**, sous la table. Cause : **tous les marqueurs
d'étalonnage sont dans le plan Z = 0**. Des points coplanaires fixent la pose
*dans* le plan mais contraignent mal la rotation hors plan, et le résidu ne peut
pas le voir puisqu'il est mesuré **sur ce plan**.

**Conséquence à retenir.** **Les deux extrinsèques ne valent que dans le plan de
la table.** 1,92 mm au sol, sans valeur à 17 cm de haut. C'est aussi pourquoi
la question du déport d'outil n'a **pas** pu être tranchée optiquement.

> **Liens partagés**
> Résultats → § 11 « Les extrinsèques ne valent que dans le plan de la table »
> Méthode → § 9, même titre · Données → `K svpro et validite du plan` · Rapport → § 15
> Fichier produit → `svpro_extrinsic_0909.yaml` (champ `VALABLE_DANS_LE_PLAN_SEULEMENT`)
> Scripts → `triangule.py`, `tri2.py`, `svpro_4tags.py` *(bac à sable)*

---

### L — Taille réelle des marqueurs (correction du 10/09)

**Objet.** Les marqueurs font-ils vraiment 50 mm ?

**Pourquoi cet essai existe.** Parce qu'une conclusion antérieure — « les tags
sont imprimés à 48,3 mm, erreur de −3,38 %, confirmée six fois » — **était
fausse**. Les six mesures partageaient **la même cause d'erreur** ; l'une
d'elles ajustait le côté en minimisant la reprojection sur les coins **détectés**,
donc sur les données déjà biaisées : **elle était circulaire**.

**Mode opératoire.**
1. **5 marqueurs identiques**, **12 acquisitions** chacun.
2. Mesurer la répétabilité **sur un même** marqueur.
3. Mesurer l'étalement **entre** marqueurs identiques.
4. Corréler le côté mesuré avec l'**obliquité** de la vue.
5. Extrapoler à une vue de face.
6. Contrôler par une grandeur **insensible au biais** : les longues distances.

**Résultats.**

| grandeur | valeur |
|---|---|
| répétabilité sur un même marqueur | **± 0,108 mm** |
| étalement entre marqueurs identiques | **1,98 mm (4,0 %)** — 18 fois plus |
| corrélation côté ↔ obliquité | **r = −0,920** |
| extrapolation à une vue de face | **50,65 mm** |
| échelle longue (383–580 mm) | **−0,044 %** contre −2,6 % sur les côtés |

**Conclusion. Les marqueurs font 50 mm** (50,7 ± 0,7 mm). La détection ArUco
tire les coins d'un petit carré vers l'**intérieur**, d'autant plus qu'il est vu
de biais.

**Ce que ce biais affecte — et ce qu'il n'affecte pas.** Il porte sur la
**taille**, jamais sur la **position** : les quatre coins sont tirés
symétriquement, donc **le centre reste juste** (0,11 à 0,42 mm). Une erreur de
taille ne se propage **pas** en erreur de position.

**Consigne ferme.** **Ne jamais corriger `marker_size_mm` dans
`workspace_markers.yaml` sur la foi d'une mesure optique.** Le champ
`taille_marqueur_NE_PAS_CORRIGER` de `arducam_extrinsic_pick.yaml` le rappelle
sur le fichier lui-même.

**La leçon de méthode.** *Des mesures qui partagent une cause d'erreur ne se
confirment pas les unes les autres.* Le seul contrôle valable serait un
**artefact étalonné** — un pied à coulisse — ce qu'exige VDI/VDE 2634-1 et ce
qui manque toujours.

> **Liens partagés**
> Résultats → § 10, sous-section « Le tag de 50 mm fait bien 50 mm — correction du 10/09 »
> Méthode → § 3 essai 5 **et** § 4 « Le côté ajusté à 48,49 mm — ce que ça mesurait vraiment » *(les deux réécrits)*
> Données → `L taille des marqueurs` · Rapport → § 16
> Aide visuelle → `mesure_pied_a_coulisse.jpg`
> Script → `tailles.py` *(bac à sable)*

---

## 4. Les sept types d'essai — couverture

| # | type d'essai | état | résultat |
|---|---|---|---|
| 1 | Répétabilité du bras | fait — borne inférieure | 0,00 à 0,84 mm, **6/6 sous la spec** |
| 2 | Erreur cartésienne reconstruite | fait | 14,84 mm brut · ≈2 mm compensé |
| 3 | **Précision physique absolue** | **non fait** | exige un comparateur (~60 €) |
| 4 | Répétabilité de la vision | refait le 10/09 | **0,725 mm** (6 trames) |
| 5 | Précision métrique de la vision | repris le 10/09 | **−0,044 %** · tag = **50 mm** |
| 6 | Calibration extrinsèque | fait — **et sa limite trouvée** | 1,86 mm · **nulle hors du plan** |
| 7 | Précision globale vision + robot | fait le 10/09 | **10,79 → 0,63 mm** |

**Un seul essai reste non fait, et c'est le seul qui exige un achat.**

> Détail → `PRECISION_MYCOBOT_320PI.md` § 12 · feuille `Les 7 essais`

---

## 5. Conformité aux normes — la lecture honnête

Sur onze lignes de traçabilité :

- **deux sont conformes** — ISO 9787:1999 (repères, tout est en `base_link`) et
  ISO 8373 (vocabulaire), les deux plus faciles ;
- **trois suivent une définition normalisée sans son protocole** — A, F-UNI, G ;
- **cinq n'ont aucune norme applicable** — dont **les trois résultats les plus
  intéressants de la campagne** : H, K et le test 7.

**Ce n'est pas un défaut à cacher.** La précision d'un système **vision + robot
en boucle fermée** n'est normalisée nulle part à ma connaissance. ISO 9283
mesure le bras seul ; VDI/VDE 2634 mesure l'optique seule. Le chiffre qui
compte pour un asservissement visuel — *quand la caméra dit d'aller là, on
arrive à combien ?* — tombe entre les deux. D'où l'obligation de décrire le
protocole intégralement.

**Exigences normatives connues et non tenues**, à déclarer :

| exigence | source | notre situation |
|---|---|---|
| incertitude de mesure ≤ **25 %** de la grandeur mesurée | ISO 9283 § 6.5 | répétabilités 0,4–0,8 mm → budget 0,10–0,20 mm ; **plancher codeur 0,099 mm** — **limite** |
| **100 % de la charge nominale**, obligatoire | ISO 9283 Table 1 | essais **à vide** |
| **100 % de la vitesse nominale**, obligatoire | ISO 9283 Table 2 | essais **à vitesse réduite** |
| 20 °C ± 2, stabilisation « de préférence une nuit » | ISO 9283 § 6.3.2.2 | non contrôlé |
| **artefact étalonné** | VDI/VDE 2634-1 | **absent** — cause directe de l'erreur de deux jours |
| **7 lignes de mesure** dans le volume | VDI/VDE 2634-1 | **1 plan** |
| nombre de cycles normatif | ISO 9283 § 6.9 | **inconnu** — page 15, absente de l'extrait public |

**Ce que la norme autorise explicitement.** ISO 9283, introduction :
*« It is intended that the user of this International Standard selects which
performance characteristics are to be tested… The tests described in this
International Standard may be applied in whole or in part. »* Un protocole
partiel est légitime **à condition d'être déclaré**, ce que fait ce tableau.

> Détail complet → `METHODOLOGIE_PRECISION.md` § 10, et sa table § 10.6

---

## 6. Où sont les scripts

| script | essai | statut |
|---|---|---|
| [`scripts/calibrer_extrinseque_4aruco.py`](../../scripts/calibrer_extrinseque_4aruco.py) | H, extrinsèque | **versionné** |
| [`scripts/aruco_check.py`](../../scripts/aruco_check.py) | contrôle de détection | **versionné** |
| `campagne_precision.py`, `test_directions.py`, `test_echelle.py` | A, G, E | bac à sable |
| `bruit_vision.py` | C, J | bac à sable |
| `test7b.py`, `analyse7b.py` | test 7 | bac à sable |
| `triangule.py`, `tri2.py`, `svpro_4tags.py` | K | bac à sable |
| `tailles.py` | L | bac à sable |

> ⚠ **Le « bac à sable » est le répertoire de session et il est effacé.** Les
> scripts qui y sont ne survivront pas. Si un essai doit pouvoir être **rejoué**,
> son script doit être remonté dans `scripts/` ou `training/calibration/` et
> commité. À ce jour, **seuls H et le contrôle de détection le sont.**

---

## 7. Fichiers de calibration produits

| fichier | contenu | statut |
|---|---|---|
| `arducam_extrinsic_pick.yaml` | extrinsèque de production. Ajustement RMS **0,4823 px**, positions **0,11 à 0,42 mm**, caméra en (35,0 · −192,2 · 1058,5) mm. Porte `taille_marqueur_NE_PAS_CORRIGER`, `exposition`, `piege_detection`, `deplacement_constate` | non versionné |
| `arducam_extrinsic_pick.avant_1009.yaml` | sauvegarde de la version précédente (caméra déplacée de 17 mm) | non versionné |
| `svpro_extrinsic_0909.yaml` | SVPRO, 3 marqueurs, RMS **0,663 px**, champs `marqueur_25_absent`, `validation_croisee` (1,92 mm), `VALABLE_DANS_LE_PLAN_SEULEMENT` | non versionné |
| `workspace_markers.yaml` | `marker_size_mm: 50.0` — **inchangé, et doit le rester** | versionné |

**Deux pièges consignés dans les fichiers eux-mêmes**, parce qu'ils ont chacun
coûté une demi-journée :

1. **Exposition.** Après un rebranchement USB, la même valeur d'exposition ne
   rend plus la même luminance, et les anciens scripts remettent le **gain à 0**,
   ce qui noircit l'image. Régler exposition **et** gain, puis **vérifier la
   luminance**.
2. **Détection.** Un **câble noir touchant la bordure noire** d'un marqueur
   fusionne avec elle : ArUco ne trouve plus de quadrilatère fermé et le
   marqueur devient invisible — alors que contraste et netteté sont **meilleurs**
   que sur les marqueurs détectés. Écarter le câble a suffi pour repasser à 4/4.

---

## 8. Historique des corrections

| date | document | correction |
|---|---|---|
| 10/09 | `PRECISION_MYCOBOT_320PI.md` | § 10 réécrit : la mesure, le biais d'obliquité, la preuve par l'échelle longue. Ligne ajoutée aux réserves |
| 10/09 | `METHODOLOGIE_PRECISION.md` | § 3 essai 5 réécrit ; **et le § 4 aussi** — il affirmait 48,49 mm par une troisième voie, atteinte par le même biais |
| 10/09 | classeur | nouvelle feuille `L taille des marqueurs`, bloc faux purgé de la feuille `J`, 4 lignes ajoutées à `Synthese` |
| 10/09 | rapport Word | paragraphe faux remplacé sur place, section 16 ajoutée — 35 tableaux |
| 10/09 | `arducam_extrinsic_pick.yaml` | champ `intrinsics_stem: cam_3` **rétabli** — il avait été perdu la veille, et `scripts/pick_dashboard.py` le lit |

**Pourquoi le § 4 devait être corrigé lui aussi.** Il présentait le côté ajusté
à 48,49 mm comme une **troisième voie indépendante** confirmant l'erreur
d'impression. Elle ne l'était pas : elle minimisait la reprojection sur les
coins **détectés**, donc sur les données déjà biaisées. **Elle reproduisait le
biais au lieu de le révéler.** Laisser ce paragraphe aurait laissé une preuve
apparente derrière une conclusion retirée.

---

## 9. Ce qui reste à faire

| priorité | action | ce que ça débloque |
|---|---|---|
| **1** | **Mesurer un marqueur au pied à coulisse** | l'artefact étalonné qu'exige VDI/VDE 2634-1. Son absence est **exactement** ce qui a laissé vivre l'erreur deux jours |
| **2** | **Recalibrer avec des marqueurs à plusieurs hauteurs** (2–3 niveaux, cale de hauteur connue) | lève la dégénérescence du plan, rend les extrinsèques valables en volume, débloque la question du déport d'outil |
| **3** | Essai **3** — précision physique absolue | exige un comparateur (~60 €) |
| **4** | ISO 9283 § 7.3 — **exactitude et répétabilité de distance** | seule caractéristique normalisée qu'une mesure aux codeurs peut rapporter en **valeur vraie** : un décalage constant — affaissement, erreur de modèle, déport faux — **s'annule dans la différence** |
| **5** | Remonter les scripts du bac à sable dans le dépôt | sans quoi **aucun essai n'est rejouable** |

---

## 10. Avertissement — l'état du banc a changé le 10/09

**La planche a bougé.** Mesure sur les 4 marqueurs : **rotation −1,750°,
translation (18,8 · −6,5) mm**, résidu **0,39 mm**, distances entre centres
conservées à **0,14 %** — donc un mouvement **rigide**, pas une déformation.

**Ce n'est pas la caméra :** le trépied au sol, l'objet de fond le plus net, n'a
bougé que de **0,2 px** ; un déplacement caméra de 19 mm l'aurait décalé de
plusieurs dizaines de pixels, et un basculement aurait cassé le modèle rigide,
qui tient.

**Deux conséquences, opposées, à ne pas confondre :**

- **`arducam_extrinsic_pick.yaml` reste VALABLE.** Le lien caméra ↔ base robot
  est intact. **Ne pas la recalibrer** — la refaire contre des positions
  nominales périmées y injecterait les 19 mm.
- **`workspace_markers.yaml` est PÉRIMÉ.** Toute calibration future qui s'appuie
  dessus sera fausse de 12 à 28 mm selon le marqueur. Tout point enseigné en
  coordonnées planche **avant le 10/09** est décalé d'autant.

*Dernière mise à jour : 10 septembre 2026.*
