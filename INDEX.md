# 📚 Index de Documentation — MyCobot 320 Pi R6A

Bienvenue dans la documentation du projet MyCobot ! Ce fichier sert de carte centrale vers tous les autres documents.

## 🎯 Par où commencer ?

### Première utilisation
- **[README.md](README.md)** — Vue d'ensemble, architecture, quick start, commandes principales
- **[docs/QUICKSTART.md](docs/QUICKSTART.md)** — Démarrage en 3 étapes
- **[CLAUDE.md](CLAUDE.md)** — Onboarding pour les sessions Claude Code + roadmap POC (Isaac Sim, VLA, etc.)

### Système distribué (PC Tour ↔ Pi)
- **[docs/ARCHITECTURE.md](docs/ARCHITECTURE.md)** — Architecture détaillée Tour/Pi + nœuds + topics
- **[docs/DEPLOYMENT.md](docs/DEPLOYMENT.md)** — Guide de déploiement complet

### Suivi du projet
- **[SESSION_RESUME.md](SESSION_RESUME.md)** — Point de départ sessions (état actuel)
- **[DEVELOPMENT_SUMMARY.md](DEVELOPMENT_SUMMARY.md)** — Résumé technique détaillé
- **[CHANGELOG.md](CHANGELOG.md)** — Historique versionné (Keep a Changelog, double track téléop 2.x + sorting 1.x)

---

## 📋 Documentation par catégorie

### 🚀 Démarrage
| Document | Description |
|----------|-------------|
| [README.md](README.md) | Vue d'ensemble + quick start + index principal |
| [docs/QUICKSTART.md](docs/QUICKSTART.md) | Guide de démarrage rapide |
| [docs/ROBOT_QUICKSTART.md](docs/ROBOT_QUICKSTART.md) | Démarrage côté robot physique |
| [mycobot_gateway/README.md](mycobot_gateway/README.md) | README du package gateway (nœuds, launches, topics) |
| [mycobot_description/README_GAZEBO.md](mycobot_description/README_GAZEBO.md) | README du package description (URDF, worlds Gazebo, caméras) |

### 🖐️ Téléopération par la main
| Document | Description |
|----------|-------------|
| [docs/TELEOPERATION.md](docs/TELEOPERATION.md) | Pipeline complet (Astra → Wilor → filtres → JTC), historique commits |
| [docs/TELEOP_ARCHITECTURE_VIZ.md](docs/TELEOP_ARCHITECTURE_VIZ.md) | Visuel détaillé : détection main → mouvement bras (types, unités, latences) |
| [docs/TELEOP_DASHBOARD.md](docs/TELEOP_DASHBOARD.md) | Manuel du dashboard ABMI 3-onglets (Home / Analytics / Tuning) |
| [docs/TELEOP_TUNING.md](docs/TELEOP_TUNING.md) | Référence des paramètres + dépannage téléop |
| [docs/TELEOP_SIM_TESTING.md](docs/TELEOP_SIM_TESTING.md) | **Validation en simulation seule** avant le bras réel : KPIs, scénarios guidés, use cases sim-only (téléop, pick mono, sorting, RoM) |
| [docs/REAL_ROBOT_TEST_PROCEDURE.md](docs/REAL_ROBOT_TEST_PROCEDURE.md) | Protocole de calibration sécurisé sur robot physique (validé 22/04/2026) |

### 🎯 Asservissement visuel en boucle fermée (robot réel)
| Document | Description |
|----------|-------------|
| [docs/PICK_AND_PLACE_BOUCLE_FERMEE.md](docs/PICK_AND_PLACE_BOUCLE_FERMEE.md) | **Document de reprise.** Ce qui a été mesuré sur le robot réel : règles non négociables (`send_coords` écarté, branche coude haut, orientation tournée selon l'azimut), l'affaissement qui fait aussi *pivoter* l'outil, le coût en allonge de l'outil vertical, l'ordre correct de la descente, les pièges de calcul (auto-test IK, extrinsèque qui dérive en bloc, déport validé sur son propre point) et le biais latéral encore ouvert |
| [mycobot_gateway/launch/visual_servo.launch.py](mycobot_gateway/launch/visual_servo.launch.py) | Lancement de la boucle — démarre **désarmé** (`dry_run:=true`), attend un `start` explicite. Prérequis et pièges dans le docstring |
| [mycobot_gateway/mycobot_gateway/visual_servo/state_machine.py](mycobot_gateway/mycobot_gateway/visual_servo/state_machine.py) | Machine à états SEARCH→TRACK→APPROACH→FINE_SERVO→DESCEND→GRASP→LIFT→PLACE, testable sans matériel |
| [mycobot_gateway/mycobot_gateway/visual_servo/safety.py](mycobot_gateway/mycobot_gateway/visual_servo/safety.py) | Superviseur : 9 conditions d'arrêt, dont l'incohérence commande/mouvement mesuré |
| [training/calibration/calibrate_camera_base_extrinsic.py](training/calibration/calibrate_camera_base_extrinsic.py) | Extrinsèque caméra→base : 16 coins, RANSAC+LM, **validation leave-one-out** |
| [scripts/diff_ik.py](scripts/diff_ik.py) | IK différentielle sur **matrice de rotation**. Son docstring explique pourquoi `send_coords` est écarté |

> ⚠ **`send_coords` est inutilisable sur cette unité.** Mesuré le 20/08 sur cible
> identique : 247,8 mm d'erreur contre 18,2 mm via `send_angles` + IK, les deux
> avec un `OK` du bridge — la méthode constructeur échoue en silence (blocage de
> cardan à RY ≈ −80°). Voir le CHANGELOG § Corrigé.

### 🎯 Pick-and-place / sorting (Gazebo)
| Document | Description |
|----------|-------------|
| [mycobot_description/README_GAZEBO.md](mycobot_description/README_GAZEBO.md) | Worlds disponibles : `pick_and_place.sdf` (mono) + `pick_and_place_sorting.sdf` (4 couleurs / 4 bacs) + visuels caméra |
| [mycobot_gateway/README.md](mycobot_gateway/README.md) | Nœuds `pick_and_place_node`, `color_object_detector`, `sorting_orchestrator` + launches associés |
| [README.md § Pick-and-place](README.md) | Section synthétique avec diagramme du pipeline sorting et résultats de validation 23/04/2026 |

### 🧠 Intelligence Artificielle
| Document | Description |
|----------|-------------|
| [training/README.md](training/README.md) | Pipeline ML (régression directe legacy + DREAM actif) |
| [training/dream/README.md](training/dream/README.md) | Module DREAM keypoint detection — VGG-19, checkpoint `vgg_ultimate_v4_mix_ft_e30` : 99.4% synthétique (50K), 91.6% réel (fine-tune mixte 50K synth + real_3cam ×5) |
| [docs/DREAM_VALIDATION_DASHBOARD.md](docs/DREAM_VALIDATION_DASHBOARD.md) | Dashboard PyQt de validation live **multi-caméras** (Arducam + SVPRO, fusion *solve-then-fuse* par joint) : ce qu'il affiche (vues empilées, courbes enc vs DREAM, tableau keypoint fusionné + détection globale), 3 filtres temporels au choix (aucun défaut), poids solveur, mode cohérence, acquisition CSV. Inclut le graphe ROS2 ![png](training/dream/rqt_dream_multicam.png) |
| [docs/DREAM_VALIDATION_LAUNCH.md](docs/DREAM_VALIDATION_LAUNCH.md) | Lancement : **launch unique `dream_multicam.launch.py`** (auto-détecte 1 ou 2 caméras) ou les 5 nœuds à la main, graphe nœuds/topics + rqt, **table de diagnostic** (quel symptôme → quel nœud manquant) + piège `.venv` |
| [docs/SYNTHETIC_DATA.md](docs/SYNTHETIC_DATA.md) | Pipeline données synthétiques Gazebo + domain randomization v2 |
| [datasets/README.md](datasets/README.md) | Documentation des datasets (synthétique 50K + réel 4K via Git LFS) |

### 🏗️ Architecture
| Document | Description |
|----------|-------------|
| [docs/ARCHITECTURE.md](docs/ARCHITECTURE.md) | Architecture du système (3 chemins de commande : GUI/CLI, téléop main, vision DREAM) |
| [docs/DEPLOYMENT.md](docs/DEPLOYMENT.md) | Guide de déploiement |
| [docs/SUMMARY.md](docs/SUMMARY.md) | Résumé court du projet |

### 🧪 Tests & Procédures
| Document | Description |
|----------|-------------|
| [docs/TELEOP_SIM_TESTING.md](docs/TELEOP_SIM_TESTING.md) | **Validation en simulation seule** (téléop, pick-and-place mono + sorting, RoM, synthetic data smoke test) |
| [docs/REAL_ROBOT_TEST_PROCEDURE.md](docs/REAL_ROBOT_TEST_PROCEDURE.md) | Protocole sur robot physique |
| [docs/TEST_COMPLET.md](docs/TEST_COMPLET.md) | Procédure de test complète (legacy) |
| [docs/TEST_ROBOT_PROCEDURE.md](docs/TEST_ROBOT_PROCEDURE.md) | Procédure détaillée robot (legacy) |
| [scripts/real_robot_preflight.sh](scripts/real_robot_preflight.sh) | Preflight 5 étapes avant toute session physique |

### 🐛 Débogage
| Document | Description |
|----------|-------------|
| [docs/DEBUG_CONNECTION_GUIDE.md](docs/DEBUG_CONNECTION_GUIDE.md) | Guide de débogage connexion |
| [docs/BRIDGE_PI_UPGRADE_GUIDE.md](docs/BRIDGE_PI_UPGRADE_GUIDE.md) | Mise à jour bridge Pi |

### 🔬 Roadmap POC (Isaac Sim, VLA, AI physics)
| Document | Description |
|----------|-------------|
| [CLAUDE.md § POC direction](CLAUDE.md) | Migration Isaac Sim, fine-tune VLA (OpenVLA / π0), benchmarks LeRobot |
| [.claude/skills/isaac-sim-integration/SKILL.md](.claude/skills/isaac-sim-integration/SKILL.md) | Plan de migration Gazebo → Isaac Sim |
| [.claude/skills/dream-workflow/SKILL.md](.claude/skills/dream-workflow/SKILL.md) | Workflow DREAM end-to-end |
| [.claude/skills/lerobot-dataset/SKILL.md](.claude/skills/lerobot-dataset/SKILL.md) | Format LeRobot pour datasets épisodiques VLA |

---

**Version :** 2.2.0 (téléop) · 1.10.0 (sorting) · 1.7.0 (legacy DREAM)
**Mise à jour :** 20 août 2026 — asservissement visuel en boucle fermée, cycle
pick-and-place complet validé sur robot réel (`feature/pick-and-place-osama`)
