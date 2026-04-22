# Procédure de test — Téléopération sur le MyCobot 320 Pi physique

Checklist à suivre avant et pendant le test de la téléopération sur le robot réel. **À lire intégralement la première fois** — les sections sont organisées par ordre chronologique de la session.

> ✅ **Premier test validé le 22/04/2026 sur le MyCobot 320 Pi (IP 10.10.0.223).**
> Le pipeline complet `Astra → Wilor → rosbridge → trajectory_to_robot_bridge → bridge_tour → bridge_pi_simple → pymycobot → servos` est fonctionnel. Gains initiaux 0.6/0.6/0.6, tfs 0.3 s, speed 25 — mouvements lents et précis du bras.

> ⚠️ **Le robot réel MyCobot 320 Pi actuel n'a pas de gripper physique.** Ce document assume donc un test **bras seul**. Le pipeline gripper est codé mais non câblé par défaut (voir [`TELEOPERATION.md § Limitations`](TELEOPERATION.md#limitations-connues)).

> 📝 **L'IP par défaut de la Pi est `10.10.0.223`** (pas `10.10.0.225` comme indiqué dans les anciens docs).

---

## Avant de commencer

### Matériel à préparer

- [ ] MyCobot 320 Pi **sous tension** (alim 24 V branchée)
- [ ] Raspberry Pi allumée, connectée au réseau filaire
- [ ] Port USB série TTL : câble OK, LED servo allumée
- [ ] PC Tour sur le même réseau que la Pi (ping possible)
- [ ] Caméra Orbbec Astra S branchée sur USB du PC Tour
- [ ] **Bouton d'arrêt d'urgence accessible** — soit main sur le clavier (Ctrl+C), soit la prise 24V débranchable rapidement

### Environnement physique

- [ ] **Zone de travail dégagée** autour du robot (rayon ~35 cm)
- [ ] Rien de fragile ni de mobile dans la zone de balayage
- [ ] L'opérateur est **debout ou assis loin du robot**, face caméra, ~50 cm
- [ ] Éclairage uniforme sur la main (pas de contre-jour fort)

### État logiciel

- [ ] Branch `feature/teleoperation` checked out sur le PC Tour
- [ ] Workspace `colcon build` à jour (`colcon build --packages-select mycobot_gateway`)
- [ ] Env conda `hand-teleop` opérationnel (vérifier avec `conda activate hand-teleop && python -c "import roslibpy"`)

---

## Étape 1 — Démarrage côté Pi

```bash
ssh er@10.10.0.223
```

```bash
# Sur la Pi : bridge robot
python3 bridge_pi_simple.py
```

**Attendu** :
```
Bridge listening on 0.0.0.0:5005
Waiting for Tower connection...
```

La Pi attend une connexion TCP sur 5005.

---

## Étape 2 — Preflight check (côté PC Tour)

**Dans un terminal ROS2** (pas conda) :

```bash
conda deactivate
source /opt/ros/jazzy/setup.bash && source ~/ros_jazzy/install/setup.bash
cd ~/ros_jazzy/src/mycobot_R6A
bash scripts/real_robot_preflight.sh
```

Le script exécute **5 vérifications** :

1. `ping` de l'IP Pi
2. TCP port 5005 ouvert
3. ROS2 sourcé
4. `bridge_tour` démarre + ping/pong round-trip sur `/to_robot` / `/from_robot`
5. `get_angles` pour vérifier que le bus servo répond

**Si tout passe** : `✓ Preflight PASSED — robot ready for teleop`.

**Si un échec** : le script dit précisément quoi faire (Pi pas allumée, bridge_pi pas lancé, ROS2 pas sourcé, etc.). **Ne pas avancer tant que le preflight ne passe pas.**

### Options du preflight

- Argument positionnel : IP custom (`bash real_robot_preflight.sh 192.168.1.50`)
- Variable d'env `GO_HOME=1` : envoie un `go_home` à la fin (demande confirmation interactive) — pratique pour s'assurer que le robot part d'une pose connue avant teleop

---

## Étape 3 — Lancer le stack ROS2 côté Tour

### 3a. Terminal 1 : rosbridge

```bash
conda deactivate
source /opt/ros/jazzy/setup.bash && source ~/ros_jazzy/install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

Attendu : `Rosbridge WebSocket server started on port 9090`.

### 3b. Terminal 2 : launch real robot

Trois choix selon ce que tu veux tester :

**A. Real + Sim côté (sécurité max, tu vois le bras Gazebo + le vrai bouger)** :
```bash
ros2 launch mycobot_gateway mycobot_teleop.launch.py target:=both rosbridge:=false
```
*Note* : rosbridge:=false parce qu'il est déjà lancé en T1. Démarrer les deux causerait un conflit de port.

**B. Real seul** (pas de Gazebo) :
```bash
ros2 launch mycobot_gateway mycobot_teleop.launch.py target:=real rosbridge:=false
```

**C. IP Pi custom** :
```bash
ros2 launch mycobot_gateway mycobot_teleop.launch.py \
    target:=real rosbridge:=false pi_ip:=10.10.0.223 real_speed:=20
```

> **⚠️ Paramètre `real_speed`** : c'est la vitesse pymycobot (0–100) pour chaque `send_angles`. **Pour le premier test commence bas** (20–30). Tu pourras monter ensuite si le comportement est sain.

**Attendu dans le log T2** :
```
[bridge_tour]: ✅ Connecté à la Pi (10.10.0.223:5005)
[trajectory_to_robot_bridge]: Bridging /mycobot_controller/joint_trajectory → /to_robot
```

### 3c. Test statique : un send_angles à zéro

Avant de lancer la téléop, envoie une trajectoire **à l'arrêt** pour valider la chaîne :

```bash
ros2 topic pub --once /mycobot_controller/joint_trajectory \
    trajectory_msgs/msg/JointTrajectory "{
joint_names: [joint2_to_joint1, joint3_to_joint2, joint4_to_joint3,
              joint5_to_joint4, joint6_to_joint5, joint6output_to_joint6],
points: [{positions: [0, 0, 0, 0, 0, 0], time_from_start: {sec: 3, nanosec: 0}}]
}"
```

Le `trajectory_to_robot_bridge` devrait loguer :
```
publishing {"action": "send_angles", "angles": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0], "speed": 20}
```

Et le robot physique doit aller à sa pose **zéro** en 3 s environ. **Si ça bouge correctement**, la chaîne ROS2→TCP→pymycobot marche.

**Si le robot ne bouge pas** :
- Vérifier que `bridge_tour` est bien connecté (log T2)
- Vérifier `ros2 topic echo /from_robot` — le Pi doit logger des `OK: send_angles ...`
- Si rien n'arrive, débranche et rebranche l'alim 24V du MyCobot (reset servo)

---

## Étape 4 — Lancer la téléop main

### 4a. Terminal 3 : script hand-teleop

```bash
conda activate hand-teleop
cd ~/ros_jazzy/src/mycobot_R6A/teleop
python3 mycobot_teleop.py --camera astra --ros --use-rosbridge --no-gripper
```

⚠ **Flag `--no-gripper`** obligatoire pour cette version sans pince physique — évite d'envoyer des commandes vers un topic gripper qui n'existe pas côté réel.

### 4b. Terminal 4 : dashboard (fortement recommandé)

```bash
conda activate hand-teleop
cd ~/ros_jazzy/src/mycobot_R6A/teleop
python3 teleop_dashboard.py
```

### 4c. Procédure de calibration main

1. **Main ouverte, paume face caméra, 50 cm, centre**
2. Clique **⟲ Recalibrate hand origin** dans le dashboard
3. Attend ~1 seconde que Wilor capture la référence
4. Tu peux commencer à bouger

---

## Étape 5 — Progression incrémentale du test

Dans cet ordre, valide chaque étape avant de passer à la suivante. **À chaque étape, Ctrl+C immédiatement si comportement anormal.**

### 5a. Mouvements isolés par axe — gains bas

Commence avec des **gains réduits** via le dashboard (faire glisser les sliders) :
- `X gain : 0.6`
- `Y gain : 0.6`
- `Z gain : 0.8`
- `time_from_start : 0.3 s`

- [ ] Bouge main **uniquement en lateral (gauche/droite)** — J1 doit tourner doucement
- [ ] Bouge main **uniquement vers l'avant/arrière** — J3 doit s'étendre/se replier
- [ ] Bouge main **uniquement haut/bas** — J2 doit monter/descendre

Si une direction saccade, est inversée, ou bloque sur une limite → Ctrl+C → retour au dashboard pour ajuster les gains / les inversions, puis relancer.

### 5b. Mouvements combinés — gains normaux

Monte les sliders graduellement vers les valeurs par défaut (1.2 / 1.2 / 1.6). À chaque palier, teste 10 s de mouvement combiné.

### 5c. Validation globale via l'analyzer

Dans un 5e terminal :

```bash
conda activate hand-teleop
cd ~/ros_jazzy/src/mycobot_R6A/teleop
python3 performance_analyzer.py --guided
```

Tu fais les 7 phases (64 s) pendant que le robot réel bouge. À la fin, regarde le verdict Excel.

> Les seuils acceptation de l'analyzer sont calibrés pour le **sim** (comparaison `commandé` vs `actual` où actual vient de `/joint_states` Gazebo). Sur le vrai robot, `/joint_states` n'est pas disponible automatiquement (Pi doit publier dessus), donc les métriques RMS/max seront faussées. **C'est normal en mode real-only** — dans ce cas juge seulement sur la fluidité visuelle du robot, pas sur le verdict Excel.

---

## Étape 6 — Arrêt propre

Ordre des Ctrl+C :

1. **T5** (analyzer) — libère la dernière socket rosbridge
2. **T4** (dashboard) — ferme la GUI
3. **T3** (teleop) — arrête les commandes → le robot reste à sa dernière pose
4. **Envoie `go_home`** avant T2 pour que le robot retourne à une pose connue :
   ```bash
   ros2 topic pub --once /to_robot std_msgs/msg/String "data: 'go_home'"
   ```
5. **T2** (ROS2 stack) — arrête bridge_tour
6. **T1** (rosbridge) — arrête le WebSocket
7. **Sur la Pi** : Ctrl+C sur `bridge_pi_simple.py`
8. **Couper l'alim 24V** du MyCobot

---

## Dépannage terrain

### Le robot bouge trop vite / violemment

- **Immédiat** : Ctrl+C sur T3 (teleop), puis débrancher alim 24V si nécessaire
- **Cause probable** : `real_speed` trop élevé dans le launch T2 (par ex. 60 au lieu de 20)
- **Fix** : relancer T2 avec `real_speed:=20`, et baisser les gains du dashboard

### Le robot "tremble" ou oscille

- **Cause** : baud série saturé, ou `rate_hz` du trajectory_bridge trop élevé
- **Fix** : ajouter `real_rate_hz:=10` au launch T2 (au lieu de 15)

### Latence énorme (robot suit avec 2 s de retard)

- **Cause** : buffer TCP qui s'accumule ou pymycobot qui traite en série une file
- **Fix** : baisse `real_rate_hz` à 8 et augmente `deadband_deg` à 2 dans `trajectory_to_robot_bridge.py` (ou via ROS params dans le launch)

### Un joint dépasse sa limite et se bloque

- **Cause** : mapping teleop qui commande au-delà du joint limit, pymycobot rejette silencieusement
- **Fix** : dans le dashboard, Recalibrate + baisser le gain de l'axe concerné. Le teleop clampe aux limites URDF (±134° J2, ±145° J3, etc.) mais la marge réelle du servo peut être plus serrée.

### Le Pi se déconnecte du Tour

- Le log T2 montre `❌ Broken pipe - Pi déconnectée`
- `bridge_tour` tente une reconnexion auto toutes les 2 s
- **Si la reconnexion échoue** : Pi surchargée ou process pymycobot crashé. Redémarrer `bridge_pi_simple.py` sur la Pi.

---

## Conservation des résultats du test

Pour chaque session de test réel, garde :

- [ ] Logs T2 et T3 (redirigés ou copiés en fin de session)
- [ ] Fichier Excel du `performance_analyzer.py` (même s'il est juste indicatif en real-only)
- [ ] Un journal court (fichier texte) avec : gains finaux utilisés, anomalies observées, décisions prises

Range-les dans `docs/real_robot_tests/YYYY-MM-DD/` pour traçabilité.

---

## Checklist de session (résumé 1 page)

```
□ Robot sous tension, zone dégagée
□ ssh Pi + python3 bridge_pi_simple.py
□ PC : conda deactivate, source ROS2, source workspace
□ bash scripts/real_robot_preflight.sh      ← doit passer tout vert
□ T1 : rosbridge_server
□ T2 : mycobot_teleop.launch.py target:=real rosbridge:=false real_speed:=20
□ Test statique : send_angles [0,0,0,0,0,0]   ← robot doit aller à zéro
□ T3 : mycobot_teleop.py --camera astra --ros --use-rosbridge --no-gripper
□ T4 : teleop_dashboard.py
□ Main face caméra → ⟲ Recalibrate
□ Gains bas (0.6 / 0.6 / 0.8) → test axe par axe
□ Montée graduelle vers gains nominaux
□ (Optionnel) T5 : performance_analyzer.py --guided
□ Arrêt propre : go_home → Ctrl+C dans l'ordre → coupure 24V
```

---

## Protocole de calibration sécurisé (validé 22/04/2026)

Séquence exacte utilisée lors du premier test réussi sur le MyCobot physique — à reproduire à l'identique pour les prochaines sessions.

### Avant de lancer la téléop

1. **Dans le dashboard, descends les sliders AVANT de bouger ta main** :
   - `X gain` : **0.6**
   - `Y gain` : **0.6**
   - `Z gain` : **0.6**
   - `time_from_start` : **0.3 s**

### Calibration Wilor

2. **Paume ouverte face à l'Astra, centrée, ~50 cm de distance, main immobile**
3. Clique **⟲ Recalibrate hand origin** dans le dashboard
4. **Attends 2 secondes** sans bouger la main — ça laisse Wilor stabiliser
5. Tu peux maintenant commencer à bouger

### Montée incrémentale

6. **Commence UNIQUEMENT avec des mouvements lents gauche/droite** (axe Y, J1 base). Le robot physique doit pivoter doucement à sa base.
7. Si ça va bien, **ajoute haut/bas** (axe Z, J2 shoulder). Puis avant/arrière (axe X, J3 elbow).
8. Une fois les 3 axes position validés sans saturation ni oscillation, **monte les gains vers 1.0 / 1.0 / 1.2** puis vers les nominaux `1.2 / 1.2 / 1.6` si le comportement reste sain.

### Conditions de Ctrl+C immédiat sur T3

Arrête la téléop (`Ctrl+C` sur T3) **immédiatement** si tu observes :

- Le robot **accélère** tout seul ou prend de la vitesse sans raison
- **Oscillations** visibles sur un ou plusieurs joints
- Robot qui part dans une **position suspecte** (près d'une limite ou vers un obstacle)
- **Perte de synchronisation** entre ta main et le robot (robot qui continue alors que ta main est immobile)

### Ce que l'opérateur surveille en parallèle

- **Dashboard → flags par joint** : passe à `✓ OK` en régime établi ; `△ JITTERY` ponctuel toléré, `⚠ UNSTABLE` = reprise
- **Dashboard → plot error** : doit rester sous la ligne 5° cible la plupart du temps
- **Terminal T2 (bridge_tour)** : JSON envoyés ~15 Hz max (rate_hz garde-fou)
- **Terminal Pi (bridge_pi_simple.py)** : logs `📥 Reçu` et `📤 Envoyé` alignés, pas de `ERROR` dans la stack pymycobot

---

## Résultats du premier test (22/04/2026)

| Métrique | Observation |
|----------|-------------|
| Pi IP | `10.10.0.223` |
| Vitesse pymycobot | 25 (real_speed=25 dans le launch) |
| Gains teleop | 0.6 / 0.6 / 0.6 puis montée progressive |
| Latence visuelle | imperceptible (<200 ms entre geste et mouvement robot) |
| Stabilité | **Pas d'oscillation, pas de saturation observée** |
| Pipeline validé | Astra → Wilor → rosbridge → JTC topic → trajectory_to_robot_bridge → bridge_tour → Pi → pymycobot → servos |
| Commande statique send_angles [45,0,0,0,0,0] | ✅ Base pivote 45° en ~3 s |
| Commande texte `home` | ✅ Robot rejoint `[0, 8, -127, 40, 0, 0]` |
| Téléop main complète | ✅ Mouvements coordonnés, pas d'accélération anormale |

### Points à creuser lors des prochaines sessions

1. **bridge_tour receive_loop** : les réponses de la Pi arrivent à la Tour mais ne sont pas logguées (observation terrain). La Pi log montre bien `📤 Envoyé` mais bridge_tour côté Tower n'affiche pas `📥 Reçu`. Non-bloquant (la téléop n'a pas besoin du retour) mais à debug pour le monitoring.

2. **Unités** : `bridge_pi_simple.py` appelle `mc.send_angles(angles_deg, speed)` — pymycobot attend des **degrés**. Notre `trajectory_to_robot_bridge` convertit bien rad → deg avant publication. ✓

3. **Défaut d'IP dans les docs anciens** : `10.10.0.225` → à remplacer par `10.10.0.223` partout.

4. **Priorité prochaine session** : test performance_analyzer sur le réel (attention : `/joint_states` Gazebo pas disponible côté réel, donc les RMS/max seront faussés — juger visuellement).

---

*Dernière mise à jour : 22 avril 2026 — après le premier test physique validé.*
*Voir aussi : [TELEOPERATION.md](TELEOPERATION.md), [TELEOP_DASHBOARD.md](TELEOP_DASHBOARD.md), [TELEOP_TUNING.md](TELEOP_TUNING.md).*
