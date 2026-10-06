# Données d'entraînement DREAM et consommation du PC Tour

*État au 06/10/2026, pendant le fine-tuning `vgg_tri_mix_ft_e10`.*

Ce document répond à quatre questions :
- comment les données du fine-tuning sont réparties ;
- combien consomment Gazebo, YOLO et DREAM, seuls ou ensemble ;
- quelles sont les limites physiques du PC ;
- pourquoi le capteur de température monte à 90-100 °C.

Chaque chiffre vient d'une mesure dont la source est indiquée. Ce qui n'a pas été mesuré est marqué **non mesuré**, sans estimation à la place.

---

## 1. Le matériel

Mesuré le 06/10 avec `lscpu`, `/sys/devices/system/cpu`, `nvidia-smi` et `sensors`.

| Élément | Valeur |
|---|---|
| Processeur | Intel Core **i7-14700**, 1 socket |
| Cœurs | **20 cœurs physiques, 28 threads**, détaillés ci-dessous |
| Cœurs P (performance) | **8 cœurs**, 2 threads chacun, soit les CPU logiques 0-15. Turbo jusqu'à **5,3 GHz** |
| Cœurs E (efficacité) | **12 cœurs**, 1 thread chacun, soit les CPU logiques 16-27. Maximum **4,2 GHz** |
| Limites de puissance (RAPL) | **85 W** en continu, sur une fenêtre de 224 s ; **219 W** en pointe |
| Température critique | **100 °C** (`crit`). Le seuil `high` est à 80 °C |
| Carte graphique | NVIDIA **RTX 4000 Ada**, 20 Go de mémoire, limite de puissance **130 W** |
| Mémoire vive | 31 Go, plus 8 Go de swap, déjà entièrement occupé le 06/10 |

**Correspondance entre CPU logiques et capteurs.** Le capteur `Core N` de `sensors` ne porte pas le numéro du CPU logique :

| CPU logiques | Capteur `sensors` |
|---|---|
| 0-1 | Core 0 |
| 2-3 | Core 4 |
| … | … |
| 14-15 | **Core 28** |
| 16-27 (cœurs E) | Core 32 à Core 43 |

**Ce que mesure « Package id 0 ».** La ligne `Package id 0`, qui sert à toutes les gardes thermiques, affiche **la température du cœur le plus chaud**. Ce n'est pas une moyenne de la puce. Le 06/10 à 14:58, `Package id 0` et `Core 28` lisaient **85 °C** tous les deux, alors que `Core 0` lisait 64 °C.

---

## 2. Distribution des données du fine-tuning

### 2.1 Les sources

Comptage des fichiers `.json` NDDS, fait le 06/10.

| Jeu | Images | Origine | Rôle |
|---|---|---|---|
| `synthetic_50k_ndds` | 50 000 | Gazebo, monde du 50K. Robot sans pince, 4 caméras | Garder l'acquis synthétique |
| `real_3cam_train_x5_ndds` | 30 000 | 6 000 images réelles uniques **× 5** (sur-échantillonnées). 3 caméras | Garder l'acquis réel |
| Scène de tri, entraînement (graines 1001-1016) | 12 080 | Gazebo, scène de tri **avec pince**. 4 caméras. 1001 : 20 poses ; 1002-1016 : 200 poses chacune | Nouveau domaine |
| **Mélange** `mix_tri_synth50k_real3camx5_ndds` | **104 160** | 50 000 + 30 000 + **12 080 × 2** | Ce que lit l'entraînement |

**Proportions dans le mélange.** Le synthétique 50K pèse **48 %**, le réel ×5 **28,8 %** et la scène de tri ×2 **23,2 %**.

**Scène de tri : comment les poses ont été tirées.** Le générateur est `synthetic_data_collector_tri.py`.
- **60 %** des poses sont celles du trieur :
  - la pointe de la pince est placée au-dessus de la table, entre 10 et 160 mm de hauteur, à un azimut de ±70° et un rayon de 0,10 à 0,42 m ;
  - l'outil est vertical, ou incliné jusqu'à 45° ;
  - chaque articulation reçoit ensuite un bruit de ±3°.
- **40 %** des poses sont uniformes, avec une garde au sol de 50 mm.
- L'ouverture de la pince est tirée au hasard à chaque pose.
- La randomisation de domaine est coupée, parce que la scène de tri est elle-même le domaine visé.

### 2.2 Le découpage dans l'entraînement

`train_dream_ultimate_v5_geo.py` mélange les indices avec la graine 42, puis coupe en **80 / 10 / 10** :

| Partie | Images | Itérations par époque (lots de 8) |
|---|---|---|
| Entraînement | 83 328 | 10 416 |
| Validation | 10 416 | — |
| Test interne | 10 416 | — |

⚠ **Les pertes de validation et de test interne sont optimistes.** Le découpage se fait *après* les répétitions, et deux familles de doublons existent dans le mélange :
- chaque image réelle y figure 5 fois ;
- chaque image de la scène de tri y figure 2 fois.

Une même image peut donc tomber à la fois en entraînement et en validation. Ces pertes servent seulement à **choisir l'époque**. Elles ne mesurent pas la généralisation.

### 2.3 Les jeux d'évaluation, jamais vus en entraînement

| Jeu | Images | Ce qu'il mesure |
|---|---|---|
| `synth_tri_test_ndds` (graines **2001-2003**) | 1 200 | La scène de tri. Ces graines ne sont pas dans le mélange |
| `real_3cam_val_ndds` | 1 500 | La non-régression sur le réel |
| `synthetic_50k_ndds` (échantillon) | — | La non-régression sur le monde 50K. ⚠ Ces images *sont* dans le mélange : le résultat montre une absence d'oubli, pas une généralisation |

**Point de départ, mesuré le 05/10.** Sur `synth_tri_test_ndds`, le modèle de départ `vgg_ultimate_v4_mix_ft_e30` détecte **52,6 %** des keypoints. Son erreur médiane est de **16,7 px**, et **12,2 %** des keypoints sont à moins de 5 px.

---

## 3. Consommation mesurée

### 3.1 Par configuration

Ce tableau vient des journaux des runs : le scratchpad `etape10/*_resume.txt`, puis `dream_tri/resume.txt` et `entrainement_temp.txt`. « Tmax » est le maximum de `Package id 0` pendant le run.

| Date | Configuration | Cœurs autorisés | Tmax | Résultat |
|---|---|---|---|---|
| 29/09 | Gazebo sans écran + YOLO 4 caméras en boucle | non épinglé | **91 °C** dès la 3e graine | Arrêté. 53 °C 20 s après l'arrêt |
| 05/10 | Gazebo + 4 DREAM + YOLO + tri | **non épinglé** | **99 °C en moins d'une minute** | Arrêté |
| 05/10 | Gazebo sans écran + 4 DREAM (cadence par défaut) + YOLO + tri, graine 2 | 16-19 (4 cœurs E) | **95 °C** | 4/4 triés |
| 05/10 | Même chose avec `dream_rate:=1.0` + dashboard (démo graine 3) | 16-19 | **85 °C** | 4/4 triés |
| 05/10 | Gazebo sans écran + 4 DREAM `dream_rate:=1.0`, sans YOLO (balayages montage, monde 50K, ablations) | 16-19 | **75-84 °C** | Tous menés à terme |
| 05-06/10 | Campagne de rendu : Gazebo sans écran + collecteur, ni YOLO ni DREAM (19 graines) | 16-19 | **74-84 °C** | 13 280 images, aucun arrêt. ~9,3 s par pose, soit 4 images |
| 06/10 | Entraînement DREAM, `--workers 6`, sans Gazebo | 8-19, puis 14-19 | **90-100 °C** en pointes | 284 pauses thermiques à 14:57 (détail au § 4) |

Pour la dernière ligne : la carte graphique était à **99 %**, à **122 W** sur 130, à **84 °C**, avec 4,3 Go de mémoire. Le processus principal utilisait 109 % de CPU et les chargeurs de données jusqu'à 100 % chacun.

### 3.2 Par composant : **non mesuré**

Aucun run n'a lancé **Gazebo seul**, **YOLO seul** ou **DREAM seul** avec un relevé de leur charge CPU (`pidstat`) et GPU. Les chiffres ci-dessus portent sur des **combinaisons**. Les seules comparaisons qu'on peut en tirer sont des différences entre deux runs. Elles ne sont pas des mesures propres, parce qu'il y a une seule répétition, que la température de départ varie et que le scénario change :

- **YOLO pèse lourd** : 84 °C sans YOLO (balayages) contre 95 °C avec (graine 2), sur les mêmes 4 cœurs.
- **La cadence de DREAM pèse lourd** : 95 °C à la cadence par défaut contre 85 °C à `dream_rate:=1.0`, sur les mêmes cœurs.
- **L'épinglage pèse le plus** : 99 °C en moins d'une minute sans `taskset`, contre 81-85 °C avec `taskset -c 16-19`, pour la même combinaison.

**Protocole proposé pour mesurer chaque composant** (à faire hors entraînement) :
- lancer chaque composant seul, 5 minutes, épinglé sur 16-19 ;
- relever toutes les 2 s :
  - `pidstat -u -p <pids> 2`, pour le CPU par processus ;
  - `nvidia-smi --query-gpu=utilization.gpu,power.draw,temperature.gpu`, pour la carte graphique ;
  - `sensors` (Package et Core 28/32-35), pour la température ;
- dans cet ordre : Gazebo sans écran → + fenêtre Gazebo → + 1 DREAM → + 4 DREAM → + YOLO.

Chaque marche donne alors le coût de ce qu'on vient d'ajouter.

---

## 4. Pourquoi ça monte à 90-100 °C

### 4.1 Ce que montrent les mesures

Relevés de la garde de l'entraînement du 06/10, dans `entrainement_temp.txt` :

| Heure | Pauses | Temps en pause |
|---|---|---|
| 10 h | 50 | 257 s |
| 11 h | 70 | 351 s |
| 12 h | 45 | 227 s |
| 13 h | 60 | 300 s |
| 14 h (avec le `git add` de VS Code) | 57 | **1 238 s** |

Au total, **284 pauses**. La température lue au moment de la pause va de 90 à 100 °C, dont **35 fois 100 °C**, la température critique.

### 4.2 L'explication

1. **Ce qui chauffe, c'est un seul cœur, pas toute la puce.** À 14:58, Core 28 lisait 85 °C contre 64 °C pour Core 0 et 68-75 °C pour les cœurs E. Core 28 correspond aux CPU logiques 14-15, un **cœur P**. Or l'entraînement est épinglé sur 14-19, et le CPU 15 tournait à **5,3 GHz**, son turbo maximal. Le processus principal de l'entraînement et ses chargeurs concentrent donc la charge sur ce cœur P, et `Package id 0` recopie la température de ce cœur.

2. **La température monte et redescend en quelques secondes.** Plusieurs pauses montrent 90 → 68 °C ou 93 → 64 °C en **5 secondes**. Un boîtier saturé de chaleur ne refroidit pas aussi vite. C'est le comportement d'un **point chaud local** sur un cœur qui passe d'un coup au turbo. La puce est autorisée à tirer **219 W en pointe**, contre 85 W en continu, et tout cet écart de puissance part dans une petite surface de silicium.

3. **La carte graphique chauffe l'air du boîtier.** Elle tourne à **122 W** et 84 °C pendant tout l'entraînement. Le refroidisseur du processeur aspire donc un air déjà chaud, ce qui laisse moins de marge avant le seuil.

4. **Tout processus non épinglé passe sur les cœurs P au turbo.** Le `git add` lancé par VS Code le 06/10 l'a fait. Il tournait à 66-70 % d'un cœur, sans `taskset`, sur les cœurs P. Pendant l'heure où il a tourné, le temps en pause est passé de **4-6 min par heure à 20 min**.

5. **Pourquoi épingler sur les cœurs E fonctionne.** Les cœurs E sont plafonnés à **4,2 GHz** et consomment beaucoup moins à la tâche. Toutes les campagnes Gazebo épinglées sur 16-19, qui sont 4 cœurs E, sont restées sous **85 °C**. La même combinaison sans épinglage a atteint 99 °C en moins d'une minute.

### 4.3 Les limites physiques à retenir

- **Le PC tient en continu une combinaison Gazebo + DREAM + YOLO, à condition de l'épingler sur les cœurs E et de régler DREAM à `dream_rate:=1.0`.** Mesuré à 81-85 °C.
- **Sans épinglage, aucune combinaison lourde ne tient plus d'une minute** sous 90 °C.
- **L'entraînement DREAM est limité par la carte graphique** : 99 %, à 8 W de sa limite de puissance. Ajouter des cœurs CPU ne l'accélérerait pas. En revanche, le placer sur un cœur P crée le point chaud décrit plus haut.
- **Lancer Gazebo pendant un entraînement n'est pas mesuré.** Les deux se disputeraient la carte graphique et l'air du boîtier. À ne pas faire sans un relevé préalable.

---

## 5. Ce qu'on peut changer

Rien de ceci n'est appliqué au 06/10.

| Action | Effet attendu | Statut |
|---|---|---|
| Épingler les entraînements sur les cœurs E seulement (`taskset -c 16-27`), sans aucun cœur P | Supprimer le point chaud de Core 28 | **Vérifié le 06/10 à 15:15**, sur l'entraînement en cours déplacé à chaud (`taskset -a -c -p 16-27`). Package passe de 85-100 °C à **74 °C**, Core 28 à 61 °C, plus aucune pause. Vitesse **inchangée** : 5,23 itérations par seconde contre 5,1-5,3 avant |
| Ajouter `training/dream/dream_data/` au `.gitignore` | Plus aucun `git add` de VS Code ne lira les ~100 000 images | Proposé, pas fait |
| Ne pas utiliser « Stage All » dans VS Code pendant un calcul | Supprime la source de chaleur du 06/10 à 14 h | Consigne |
| Préfixer toute commande manuelle Gazebo/DREAM/YOLO par `taskset -c 16-19` | Déjà la règle depuis le 05/10 | En place |
| Mesurer chaque composant séparément (protocole du § 3.2) | Remplir la colonne « par composant », aujourd'hui vide | À faire |

---

## Sources

- Scratchpad de session `dream_tri/` : `resume.txt` (campagne de rendu), `entrainement_temp.txt` (pauses de l'entraînement), `entrainement.sh` (épinglage 8-19 et garde 90/80 °C).
- Scratchpad `etape10/` : `resume.txt`, `demo_resume.txt`, `montage_resume.txt`, `monde50k_resume.txt`, `ablation_resume.txt`, avec les options des scripts `.sh` correspondants.
- Mesures en direct du 06/10 à 14:58 : `sensors`, `nvidia-smi`, `top`, `taskset -p 502707` (masque `fc000` = CPU 14-19), `/sys/devices/system/cpu/cpu*/topology/core_id`, `/sys/class/powercap/intel-rapl:0`.
- Mémoire projet : relevés des 29/09 et 05/10, sans épinglage.
- Données : `training/dream/dream_data/`. Découpage : `train_dream_ultimate_v5_geo.py`, lignes 120-121 et 425-429.
