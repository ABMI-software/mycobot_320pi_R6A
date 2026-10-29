# Bibliographie de l'article DREAM

Tenue par l'agent `veilleur-biblio`. Une ligne « vérifié : non » n'est pas
citable : la référence a été notée de mémoire le 29/10/2026 et doit être
ouverte, contrôlée et complétée (lieu, pages, chiffres publiés).

## Keypoints + PnP, caméra → robot

| Travail | Année | Hypothèses | Jeux / métriques | Chiffres publiés | Vérifié |
|---|---|---|---|---|---|
| Lee et al., « Camera-to-Robot Pose Estimation from a Single Image » (DREAM), ICRA, [arXiv:1911.09231](https://arxiv.org/abs/1911.09231) | 2020 | Angles connus, synthétique seul, 1 vue | Panda-3Cam, Panda-Orb ; PCK, ADD | Tableau I : RS 0,24 / 0,83 / 0,96 (PCK@2,5/5/10) ; Orb 0,28 / 0,67 / 0,83 ; tableau II : erreur atteinte 27,4 ± 4,7 mm | oui (`papiers/`) |
| Lambrecht & Kästner, ICAR | 2019 | Réel + synthétique | — | — | non |
| Lu et al., placement optimisé des keypoints, RA-L | 2022 | — | — | — | non |

## Sim-to-real

| Travail | Année | Hypothèses | Jeux / métriques | Chiffres publiés | Vérifié |
|---|---|---|---|---|---|
| Lu et al., CtRNet, ICCV | 2023 | Auto-supervisé par segmentation | — | — | non |

## Angles inconnus

| Travail | Année | Hypothèses | Jeux / métriques | Chiffres publiés | Vérifié |
|---|---|---|---|---|---|
| Zuo et al., CRAVES, CVPR | 2019 | Bras bas coût, adaptation de domaine | — | — | non |
| Labbé et al., RoboPose, CVPR | 2021 | Rendu-et-comparaison | — | — | non |
| Ban et al., estimation holistique, ECCV | 2024 | — | — | — | non |
| RoboPEPP | 2024-25 | — | — | — | non |

## Temporel et multi-vues

| Travail | Année | Hypothèses | Jeux / métriques | Chiffres publiés | Vérifié |
|---|---|---|---|---|---|
| Tian et al., SGTAPose, CVPR | 2023 | Séquences | — | — | non |
