---
name: article-dream
description: Orchestration du projet d'article DREAM (comparaison à Lee et al. 2020, synthétique seul multi-vues, PCK/ADD au format du papier). À invoquer pour choisir la prochaine expérience E1-E8, lancer ou reprendre l'une d'elles, faire entrer un chiffre dans le registre, ou rédiger une partie du manuscrit.
---

# Article DREAM — orchestration

Référence : [`docs/ARTICLE_DREAM.md`](../../../docs/ARTICLE_DREAM.md) (thèse,
règles de comparabilité, expériences E1-E8 et leur état). Registre des
chiffres : [`training/dream/article/registre_resultats.yaml`](../../../training/dream/article/registre_resultats.yaml).
Si ce fichier et `docs/ARTICLE_DREAM.md` divergent, le second fait foi.

## Le cycle d'une expérience

1. **Choisir** : la première expérience « à faire » dont les dépendances sont
   tenues (tableau de `docs/ARTICLE_DREAM.md`). E1 et E2 avant tout le reste.
2. **Écrire le protocole avant de mesurer** (skill `campagne-mesure`) : objectif,
   actions, colonnes consignées, critère de sortie chiffré, ce qui est tenu à
   l'écart (poses, jour, scène, **point de vue**). Le critère vient du tableau
   des expériences ; on ne le recale jamais sur le résultat.
3. **Exécuter** avec le bon spécialiste (tableau ci-dessous), dans le bon
   environnement Python. Chaque essai : `run.yaml` + données brutes archivées
   hors du dépôt.
4. **Évaluer au format du papier** : `training/dream/eval_papier.py` pour la
   PCK (non détecté = faux) ; jamais les « % < N px » d'`evaluate_dream.py`
   dans l'article.
5. **Faire juger** chaque chiffre par l'agent `metrologue`, puis l'inscrire au
   registre avec son verdict. Seul `tenu` entre dans le manuscrit.
6. **Mettre à jour** la colonne « État » de `docs/ARTICLE_DREAM.md`, puis
   CHANGELOG et SESSION_RESUME (règle `auto-commit`).

## Qui fait quoi

| Besoin | Spécialiste |
|---|---|
| Générateur synthétique, monde Gazebo, caméra libre, randomisation | agent `digital-twin-engineer` ; `urdf-surgeon` si l'URDF bouge |
| Entraînement, ablation, évaluation | agent `dream-trainer` (tâches longues en arrière-plan) |
| Capture réelle, hand-eye, erreur atteinte | skills `real-robot-session`, `precision-iso9283` |
| Chiffre à juger | agent `metrologue` (lecture seule) |
| État de l'art, chiffres publiés des concurrents | agent `veilleur-biblio` |
| Manuscrit | agent `redacteur-article` |
| Cohérence de la doc en fin de jalon | agent `gardien-doc` |

Lancer en parallèle ce qui est indépendant (E1 et E2 ; la biblio et toute
expérience), jamais deux entraînements GPU à la fois.

## Garde-fous propres à l'article

- **Synthétique seul veut dire zéro image réelle**, ni en entraînement, ni en
  `--pretrained` depuis un modèle affiné sur du réel (il porterait le montage
  mémorisé). Le fine-tuning réel `vgg_tri_reel_ft_e10` est la borne haute,
  jamais la thèse.
- **Test de décalage** (`scripts/dream_test_decalage.py`, ±10/20/40 px en u et
  v, par keypoint, avec n) avant tout chiffre réel.
- **Un point de vue tenu à l'écart** dans tout test qui parle de
  généralisation.
- **Boucle fermée** : DREAM n'a jamais commandé le bras. Le tri sans marqueur
  valide une extrinsèque figée dérivée de DREAM, pas DREAM en direct. Ne pas
  écrire autre chose avant E7.
- **Labels réels issus de l'ArUco** : tout accord DREAM ↔ ArUco est
  partiellement circulaire ; le dire.
- **Données brutes** : celles des essais réels d'octobre sont dans le clone
  `~/Osama_ws` ; citer le clone dans `archive` tant qu'elles n'ont pas été
  centralisées.
- **Thermique** : entraînements et Gazebo sur les cœurs E
  (`taskset -c 16-27`), arrêt au-delà de 90 °C.

## Démarrer une session sur l'article

```bash
conda deactivate
sed -n '/^## Expériences/,/^### Point de départ/p' docs/ARTICLE_DREAM.md
python3 -c "import yaml,collections; r=yaml.safe_load(open('training/dream/article/registre_resultats.yaml'))['resultats']; print(collections.Counter(x['verdict'] for x in r))"
```

Ou la commande `/article`, qui fait la même lecture et propose la prochaine
action.
