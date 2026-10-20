## Summary

<!-- Ce qui change et, surtout, pourquoi. Les chiffres mesurés vont ici. -->

## Test plan

<!-- Ce qui a été exécuté, pas ce qui devrait marcher. -->

- [ ] `colcon build --packages-select mycobot_gateway mycobot_description --symlink-install`
- [ ] `python3 -m pytest tests/ -q`

## Robot physique

<!-- Supprimer cette section si la PR ne touche ni le robot, ni un pont, ni la téléop. -->

- [ ] `bash scripts/real_robot_preflight.sh` passé
- [ ] Adresse de la Pi confirmée par **aller-retour TCP sur le port 5005** (un `ping` ne prouve rien)
- [ ] Passage d'essai physique décrit ci-dessous, avec son résultat

## Mesures

<!-- Si la PR avance un chiffre, donner le protocole avec. Rappels :
     une répétabilité n'est pas une justesse ; un résidu d'ajustement
     d'extrinsèque n'est pas une justesse ; un seul passage du banc de tri
     n'est pas un résultat. -->

## Documentation

- [ ] `CHANGELOG.md` mis à jour **dans ce commit**
- [ ] Document de domaine concerné mis à jour (`docs/`, README de paquet)
- [ ] Aucun chemin absolu vers un répertoire personnel
