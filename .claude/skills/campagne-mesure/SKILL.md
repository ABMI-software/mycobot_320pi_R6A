---
name: campagne-mesure
description: Méthode du projet pour toute campagne de mesure ou d'essai (précision du bras, DREAM, YOLO, tri, calibration) — protocole avec critère de sortie, traçabilité run.yaml, données brutes archivées, statistique juste, contrôle d'anti-circularité. À invoquer avant de lancer une campagne, d'écrire un protocole, ou de publier un chiffre.
---

# Campagne de mesure — la méthode du dépôt

Le projet a déjà publié puis retiré des conclusions fausses : une répétabilité
jugée sur l'écart maximal, une extrinsèque « sans marqueur » qui récitait la
pose d'entraînement, un 4/4 de tri pris pour un état alors que c'était un
tirage. Cette méthode existe pour ne pas recommencer. Modèles à suivre :
`docs/PROTOCOLE_REEL.md`, `docs/PROTOCOLE_YOLO_GAZEBO.md`,
`training/calibration/PROTOCOLE_ESSAIS_PRECISION.md`.

## 1. Avant : le protocole

Chaque étape d'un protocole a, dans cet ordre :

- **Objectif** — une phrase.
- **Actions** — ce qu'on fait, outil et commande exacte.
- **Consigner** — les colonnes qu'on relève.
- **Critère de sortie** — chiffré, écrit **avant** la mesure, ou explicitement
  « à trancher » avec qui tranche. Jamais calé après coup sur le résultat.
- **Ne pas faire** — la tentation à éviter à cette étape.

Fixer aussi : la **référence** (vérité Gazebo, codeurs, moyen externe — et ce
qu'elle ne voit pas), l'**effectif** (§ 4), et ce qui invalide un essai (un essai
invalide se rejoue en entier, il ne se complète pas).

## 2. Pendant : la traçabilité

Chaque essai produit un dossier daté contenant au minimum :

- `run.yaml` : commit et branche, état `git status` (propre ou non), modèles
  (chemin + date), extrinsèques et déport d'outil utilisés (chemin + date),
  IP de la Pi, opérateur, conditions (chauffe, température, éclairage).
- les **données brutes** (CSV ligne par mesure), jamais seulement les agrégats ;
- le journal de température si le PC calcule (garde thermique : gel 85 °C,
  reprise 75 °C, arrêt au-delà de 95 °C).

`scripts/essai_reel_csv.py` fait déjà cela pour le tri réel ; s'en inspirer.
Les résultats ne vont **pas** dans Git (`results/` est interdit par le hook) :
ils s'archivent hors du dépôt, et la doc cite le dossier d'archive.

## 3. Après : publier un chiffre

- **Résultats bruts d'abord**, analyse ensuite, dans deux blocs distincts.
- Chaque chiffre est accompagné de : son protocole, sa référence, n, et sa
  dispersion (médiane et p90, ou intervalle de confiance).
- Une conclusion qui contredit une précédente **corrige** l'ancienne là où elle
  est écrite (CLAUDE.md, INDEX.md, README), et le dit dans le CHANGELOG.

## 4. Statistique — les confusions déjà payées

| Ne pas confondre | Parce que |
|---|---|
| Répétabilité (RP ISO 9283 = l̄ + 3·S_l) et écart maximal | l'écart max sous-estime RP |
| Répétabilité et exactitude | elles répondent à deux questions différentes |
| Résidu d'ajustement d'une extrinsèque et sa justesse | seul le leave-one-out sur un repère exclu est une justesse (5,46 mm contre 0,594 mm annoncés le 09/09) |
| Un tirage et un état | le tri Gazebo n'est pas déterministe : jamais de conclusion sur moins de plusieurs cycles |
| « 0 échec » et « fiable » | 0 échec sur n essais borne seulement le taux d'échec sous 3/n (règle de trois, 95 %) ; 59 succès sans échec pour affirmer ≥ 95 % |
| Détecté et bien placé | un keypoint « détecté à 100 % » peut être à 50 px |

## 5. Anti-circularité — le contrôle obligatoire

Avant de croire un bon chiffre, se demander si la mesure pouvait donner autre
chose. Tests minimaux :

- **Décalage** : décaler l'image de N px, la détection doit suivre de N px
  (`scripts/dream_test_decalage.py`). Échec = le réseau récite.
- **Hors échantillon** : contrôle md5 des images entre entraînement et test, et
  tenir à l'écart un **point de vue**, pas seulement des poses.
- **Référence indépendante** : si la référence a servi à produire la mesure
  (labels ArUco → DREAM jugé contre l'ArUco), le dire et borner ce qu'on peut
  conclure.

En cas de doute sur une affirmation chiffrée, déléguer la relecture à l'agent
`metrologue`.
