# Règle — configuration : rien de propre à un poste dans le code

> **`CONTRIBUTING.md` (racine du dépôt) fait foi.** Ce fichier le reformule pour
> l'outillage. En cas de désaccord, corriger celui-ci.

Le dépôt tourne sur plusieurs postes (PC Tour de Lyon, poste de Nanterre, WSL2)
et dans deux copies sur le même PC (`~/ros_jazzy`, `~/Osama_ws`). Tout ce qui
dépend du poste vit hors du code.

## Interdit dans le code (bloqué par le hook pre-commit)

- **Une IP du banc** (`10.10.0.x`). L'adresse de la Pi change d'une semaine à
  l'autre. Lire `os.environ['MYCOBOT_PI']` ; en shell, `${MYCOBOT_PI:?}`.
- **Un chemin `/home/<utilisateur>/…`.** Chemin relatif à la racine du dépôt
  (`Path(__file__).resolve().parents[n]`) ou `get_package_share_directory()`
  pour un nœud ROS installé.

Le hook ne juge que les lignes **ajoutées** : la dette existante (51 fichiers
avec une IP, 20 avec un chemin au 22/10/2026) ne bloque personne, elle ne peut
plus grossir. Quand on touche un de ces fichiers, on corrige la ligne au
passage. Une valeur littérale réellement voulue porte `# verifier: ok` avec sa
raison.

## Fichiers locaux du banc

Valeurs mesurées sur un banc donné, qui ne se commitent pas telles quelles :
`scripts/tool_offset.json` (déport de la pince), extrinsèques de session
(`training/calibration/*_extrinsic_*.yaml` non suivis). Un script qui en a besoin
**refuse de bouger le bras** s'ils manquent, avec un message qui dit où les
prendre — comme `pick_fsm.py` le fait déjà. Ne jamais remplacer par une valeur
par défaut silencieuse.

`scripts/etat_session.sh` signale au démarrage de chaque session ceux qui
manquent.

## Vérifier

```bash
python3 scripts/verifier_depot.py --complet   # dette signalée, secrets et liens bloquants
```
