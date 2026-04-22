# Manuel utilisateur — Dashboard de téléopération

Guide d'utilisation du `teleop_dashboard.py` pour tuner les gains en live et suivre les performances du système de téléopération en simulation.

> Prérequis : rosbridge en cours sur `ws://localhost:9090` (T1), Gazebo + controllers actifs (T2), teleop script en cours avec `--use-rosbridge` (T3). Voir [TELEOPERATION.md](TELEOPERATION.md) pour le workflow complet.

---

## Lancement

```bash
conda activate hand-teleop
cd ~/ros_jazzy/src/mycobot_R6A/teleop
python3 teleop_dashboard.py
```

Une fenêtre 1400 × 900 s'ouvre, thème sombre "darkly".

---

## Anatomie de l'interface

```
┌─────────────────────────────────────────────────────────────────────┐
│  🦾 MyCobot 320 Pi — Teleop Dashboard                               │
│     Live tuning · Hand → Joint tracking · Signal stability          │
├─────────────────────────────────────────────────────────────────────┤
│ ┌─ Live gain tuning ──────────────────────────────────────────────┐ │
│ │  X gain — hand forward → J2 shoulder      ━━━━━━━●━━━   1.20    │ │
│ │  Y gain — hand lateral → J1 yaw           ━━━━━━━●━━━   1.20    │ │
│ │  Z gain — hand vertical → J3 + J5         ━━━━━━━━━●━   1.60    │ │
│ │  time_from_start — trajectory duration(s) ━━━●━━━━━━━   0.25    │ │
│ │                                                                  │ │
│ │  Changes apply live via rosbridge. Higher gain = more joint ... │ │
│ │                               [⟲  Recalibrate hand origin]       │ │
│ └──────────────────────────────────────────────────────────────────┘ │
│                                                                       │
│  ● rosbridge connected    hand_xyz:  800   commanded:  800  joint_states:  800
│                                                                       │
│ ┌─ Tracking stability (last 10s) ─────────────────────────────────┐  │
│ │ Joint         RMS err    Max err   Cmd jitter σΔ    Signal      │  │
│ │ ─────────────────────────────────────────────────────────────── │  │
│ │ J1 yaw         2.30 °    8.40 °       0.45         ✓ OK         │  │
│ │ J2 shoulder    3.10 °   12.00 °       0.62         ✓ OK         │  │
│ │ J3 elbow       5.80 °   18.20 °       0.91         △ JITTERY    │  │
│ │ J4 wrist1      0.50 °    1.20 °       0.10         ✓ OK         │  │
│ │ J5 yaw         0.50 °    1.20 °       0.10         ✓ OK         │  │
│ │ J6 roll        1.20 °    3.50 °       0.28         ✓ OK         │  │
│ └──────────────────────────────────────────────────────────────────┘ │
│                                                                       │
│ ┌─ Live signals ───────────────────────────────────────────────────┐ │
│ │                                                                   │ │
│ │  [Plot 1] Wilor hand position (m)                                │ │
│ │   0.2 ▂▃▆▆▅▄▂▁▁▂▄▆▇▅    x (forward) — red                       │ │
│ │   0.0 ▁▁▂▅▇▆▄▂▁▂▅▇▆▃    y (lateral) — green                     │ │
│ │  -0.2                   z (vertical)— blue                       │ │
│ │       └─────── 10 s glissantes ───────┘                          │ │
│ │                                                                   │ │
│ │  [Plot 2] Joint angles — solid=commanded, dashed=actual          │ │
│ │   100° ╱╲╱╲      ╱╲╱╲        solid = ce qu'on envoie             │ │
│ │     0° ╱  ╲    ╱    ╲        dashed = ce que Gazebo rapporte     │ │
│ │  -100° ╲  ╱    ╲    ╱        6 couleurs pour 6 joints            │ │
│ │                                                                   │ │
│ │  [Plot 3] Tracking error per joint — |commanded − actual|        │ │
│ │    20° ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─ ─          │ │
│ │     5° ━━━━━━━━━━━━ 5° target (ligne pointillée) ━━━━━━━━━       │ │
│ │     0° ╱╲    ╱╲    ╱╲    ╱╲    ╱╲                               │ │
│ └───────────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────────────┘
```

---

## Bandeau du haut — Live gain tuning

### Les 4 sliders

Tous les changements sont **appliqués en direct via rosbridge** — pas besoin de redémarrer `mycobot_teleop.py`. Chaque glisse publie sur `/teleop/gains` (un `Float64MultiArray` `[x, y, z, tfs]`) et le script teleop applique les nouvelles valeurs à la prochaine frame (latence ~33 ms à 30 Hz).

| Slider | Plage | Défaut | Effet |
|--------|-------|--------|-------|
| **X gain — hand forward → J2 shoulder** | 0.3 → 3.0 | 1.20 | ⚠ Note : le label dit "J2 shoulder" pour compat avec R5A ; dans notre mapping actuel X → **J3 elbow**. Plus c'est haut, plus un petit mouvement avant/arrière de la main produit un grand mouvement de coude. |
| **Y gain — hand lateral → J1 yaw** | 0.3 → 3.0 | 1.20 | Main à gauche/droite → base qui pivote. Amplifier pour couvrir plus d'angle avec moins de mouvement de main. |
| **Z gain — hand vertical → J3 + J5** | 0.3 → 3.0 | 1.60 | ⚠ Note : le label dit "J3 + J5" mais dans le mapping actuel Z → **J2 shoulder**. Plus haut = main monte/descend produit plus d'élévation de l'EE. |
| **time_from_start — trajectory duration (s)** | 0.1 → 2.5 | 0.25 | Durée de chaque trajectoire envoyée au JTC. Plus bas = plus réactif mais risque de sursaturation ; plus haut = plus lissé mais lent à répondre. |

> Les labels des sliders X et Z mentionnent les anciens routages (R5A). La routage **actuel** est documenté dans [TELEOPERATION.md § Mapping](TELEOPERATION.md#mapping-main--joints).

### Le bouton ⟲ Recalibrate hand origin

**À utiliser en début de session** (et à chaque fois que tu te déplaces ou changes de position devant la caméra).

Cliquer publie un message vide sur `/teleop/recalibrate` ; le teleop reçoit, appelle `tracker._pause()` + `tracker._resume()` qui résèt l'`initial_pose` de Wilor. La **prochaine détection** avec ta main dans le cadre devient la nouvelle origine du mapping.

**Protocole recommandé** :
1. Place ta paume ouverte, bien centrée, à ~50 cm face à la caméra
2. Clique **Recalibrate**
3. Dans T3 tu verras `[RECAL] Tracker initial_pose cleared — keep palm in view`
4. Ne bouge pas la main pendant ~1 s le temps que Wilor stabilise sa nouvelle référence
5. Commence à bouger — le robot reste à la pose zéro tant que ta main reste à cette position de référence

**Quand re-recalibrer** :
- Le robot dérive alors que ta main est immobile → la référence a "vieilli"
- Tu as bougé de position (siège / chaise) devant la caméra
- Le robot est bloqué dans une position saturée (joints à ±168°) et tu veux repartir propre

---

## Ligne de statut (entre gains et stats)

```
● rosbridge connected    hand_xyz:  800   commanded:  800   joint_states:  800
```

- **● rosbridge connected** (vert) — le dashboard est bien abonné au WebSocket. Si ça devient rouge → relancer T1 (`rosbridge_websocket_launch.xml`).
- **hand_xyz: N** — nombre de messages de position de main reçus depuis le démarrage du dashboard. Monte à ~30/s quand le teleop tourne correctement.
- **commanded: N** — messages `/mycobot_controller/joint_trajectory` reçus. Devrait monter au même rythme que `hand_xyz`.
- **joint_states: N** — messages `/joint_states` de Gazebo reçus. Attendu à ~150 Hz depuis le joint_state_broadcaster.

**Diagnostic** :

| Symptôme | Cause probable |
|----------|----------------|
| hand_xyz: 0 (ne monte pas) | Teleop pas lancé, ou Wilor ne détecte pas la main → regarder la fenêtre `hand-teleop` |
| hand_xyz monte, commanded: 0 | Teleop sans `--use-rosbridge`, ou erreur d'advertize du topic |
| commanded monte, joint_states: 0 | Gazebo pas lancé ou controllers pas actifs → regarder les logs T2 |
| Les 3 montent, robot ne bouge pas | Conflit de contrôleurs ou gains PID à zéro |

---

## Panneau central — Tracking stability (last 10s)

Tableau de 6 lignes, une par joint. Les stats sont recalculées toutes les 250 ms sur la **fenêtre glissante des 10 dernières secondes**.

### Colonnes

- **RMS err** (°) : racine de la moyenne des carrés de l'erreur `|commandé − actual|` sur la fenêtre. Indicateur principal de qualité de tracking.
- **Max err** (°) : pic d'erreur sur la fenêtre. Révèle les transients ponctuels (reacquisition Wilor, saut brusque).
- **Cmd jitter σΔ** (°) : écart-type des différences entre deux commandes consécutives. Mesure la propreté du signal d'entrée indépendamment du tracking.
- **Signal** : drapeau synthétique basé sur les 3 précédents.

### Les flags

| Flag | Critère | Interprétation |
|------|---------|----------------|
| ✓ **OK** (vert) | max ≤ 15° AND jitter ≤ 3° AND RMS ≤ 5° | Le robot suit proprement. Tu peux y aller sur le réel si tous les joints pilotés sont OK. |
| △ **JITTERY** (orange) | max ≤ 15° AND jitter entre 3 et 6° OR RMS entre 5 et 8° | Signal un peu bruyant mais robot tracke. Halver les gains avant réel. |
| ⚠ **UNSTABLE** (rouge) | max > 15° OR jitter > 6° | Commandes trop amples ou trop rapides. Baisser gains + time_from_start. Ne pas passer au réel. |

> Les seuils du dashboard (15°/3°) sont **plus stricts** que ceux du `performance_analyzer.py` (30°/3°) — le dashboard est un outil de réglage fin, l'analyzer juge l'acceptation finale avant robot réel.

### Lecture rapide

- **J4 wrist1 et J5 yaw à 0.50° RMS identiques** → normal, le mapping split `pitch / 2` envoie la même valeur aux deux.
- **J6 roll qui grimpe brutalement** quand tu bouges → Wilor est sensible aux rotations de main, penser à réduire `roll_gain` dans le code si c'est systématique.
- **J3 elbow jittery** alors que J1/J2 OK → le x_gain est probablement trop haut pour ton amplitude de gestes.

---

## Plots — Live signals

Trois graphes empilés, fenêtre glissante de 10 s, rafraîchis toutes les 250 ms.

### Plot 1 — Wilor hand position

La position XYZ de la main en mètres, telle qu'estimée par Wilor après les filtres du tracker (Kalman + jump clamp) mais AVANT mapping vers les joints.

| Courbe | Axe main | Ordre de grandeur |
|--------|----------|-------------------|
| 🔴 rouge — x (forward) | Distance main → caméra | ±0.1 m au repos, ±0.25 m amplitude |
| 🟢 vert — y (lateral) | Main gauche/droite | ±0.15 m amplitude |
| 🔵 bleu — z (vertical) | Main haute/basse | ±0.15 m amplitude |

**Ce qu'il faut surveiller** :
- **Trois courbes constantes à 0** : main immobile, mapping relatif à l'origine (normal après Recalibrate)
- **Saut brutal simultané sur x+y+z** : reacquisition Wilor (main a quitté le cadre puis revenue)
- **Bruit haute fréquence ~0.01 m** : normal (bruit capteur)
- **Dérive lente** : la référence initial_pose a vieilli → Recalibrate

### Plot 2 — Joint angles

Les 6 joints en degrés, **commandé en solide** (ce qu'on envoie à Gazebo) vs **actual en pointillé** (ce que Gazebo rapporte via /joint_states).

| Couleur | Joint |
|---------|-------|
| 🔴 | J1 yaw |
| 🟠 | J2 shoulder |
| 🟢 | J3 elbow |
| 🔵 | J4 wrist1 |
| 🟣 | J5 yaw |
| 🟤 | J6 roll |

**Lecture** :
- **Solide et pointillée superposés** : tracking parfait (le robot suit la commande sans retard)
- **Pointillée en retard avec un décalage constant** : latence de tracking (normal, 50-100 ms)
- **Pointillée plate alors que solide bouge** : le robot ne reçoit pas les commandes (crash du JTC, sat. joint, ou contrôleur pas actif)
- **Solide qui sature à ±168°** : le mapping + gain commande au-delà des limites → baisser les gains

### Plot 3 — Tracking error per joint

Erreur absolue `|commandé − actual|` en degrés, **une courbe par joint** (6 couleurs).

- **Ligne pointillée horizontale à 5°** : cible de bonne téléopération. En dessous = excellent tracking.
- **Pics ponctuels < 30°** : acceptables (transients de Wilor, reacquisition)
- **Erreur qui reste > 30° en steady-state** : vrai problème — baisser les gains, augmenter `time_from_start`

---

## Workflow de tuning complet

### 1. Démarrage

```
T1 rosbridge    →  T2 Gazebo     →  T3 teleop     →  T4 dashboard
```

**Attendre** que T2 affiche `mycobot_controller active` + `gripper_position_controller Configured and activated`.

### 2. Calibration

Paume face caméra, 50 cm, centrée. **Clique Recalibrate**. Ne bouge pas pendant 1 s.

### 3. Vérification signal brut

Regarde le plot Wilor XYZ : tes courbes bougent-elles quand tu bouges la main ? Si `hand_xyz: 0` dans le bandeau, Wilor ne détecte rien — voir troubleshooting.

### 4. Vérification tracking

Bouge **lentement** ta main en Y (gauche/droite). Le plot joint angles doit montrer :
- Solide rouge (J1) qui suit ta main
- Pointillé rouge qui suit la solide (avec peut-être 100 ms de retard)

Si le pointillé reste à plat → le robot ne reçoit pas tes commandes.

### 5. Tuning amplitude

Si tu dois bouger ta main de 50 cm pour obtenir un mouvement significatif → **augmente les gains** (Y gain 1.2 → 2.0 par ex.).

Si ton robot oscille ou sature avec des mouvements de 5 cm → **baisse les gains** (1.2 → 0.8).

### 6. Tuning lissage

Si le tracking error dépasse la ligne 5° pendant des mouvements normaux → **augmente `time_from_start`** (0.25 → 0.4). Effet : commandes plus lissées mais plus lentes.

Si le robot "traîne" visiblement derrière ta main → **diminue `time_from_start`** (0.25 → 0.15). Attention, peut réintroduire de l'instabilité.

### 7. Validation finale

Tous les joints pilotés (J1, J2, J3, J4, J5, J6) affichent **✓ OK** en stats panel pendant au moins 30 s de mouvement continu → lance `performance_analyzer.py --guided` pour un rapport formel.

---

## Troubleshooting spécifique dashboard

| Problème | Vérifier | Solution |
|----------|---------|----------|
| Fenêtre ne s'ouvre pas | `echo $DISPLAY` non vide | Relancer avec DISPLAY défini : `DISPLAY=:0 python3 teleop_dashboard.py` |
| `cannot reach rosbridge` au démarrage | T1 lancé ? port 9090 libre ? | `nc -zv localhost 9090` ; si KO, relance T1 |
| Sliders ne bougent rien dans T3 | Teleop lancé sans `--use-rosbridge` ? | Ajouter le flag et relancer T3 |
| Compteurs hand_xyz / commanded / joint_states à 0 | Topics bien advertized ? | `ros2 topic list \| grep teleop` ; `ros2 topic hz /mycobot_controller/joint_trajectory` |
| Plot gelé à t+X secondes | Matplotlib backend ? | Le dashboard utilise TkAgg ; si Qt interfère, désactive `QT_QPA_PLATFORM` |
| Flag reste ⚠ UNSTABLE quoi qu'il arrive | Maximum error vient d'un transient unique | Relis l'onglet "scenarios" de `performance_analyzer.py` — le pic est souvent sur UN geste précis |
| Tous les joints à 0° commandé mais flag UNSTABLE | Max error mesuré sur les samples initiaux avant Recalibrate | Clique Recalibrate + attends 30 s que la fenêtre glissante se recompose |

---

## Utiliser le dashboard + performance_analyzer ensemble

Le dashboard te montre la **situation instantanée** (fenêtre 10 s glissante). L'analyzer te produit un **rapport formel** pour la prise de décision "on passe au réel ou pas".

Protocole recommandé avant robot réel :

```
1. Dashboard ouvert en permanence
2. Tu tunes les gains jusqu'à voir tous les joints ✓ OK pendant 30 s
3. Tu lances :
   python3 performance_analyzer.py --guided
4. Pendant les 64 s du protocole, tu suis les instructions affichées
   (idle → up/down → left/right → forward/back → combined → gripper → rest)
5. À la fin, l'analyzer produit un .xlsx + un verdict console
6. Si verdict = READY FOR REAL ROBOT → passer au robot physique
   Si verdict = CAUTIOUS → halver les gains (dashboard) et refaire
   Si verdict = NOT READY → diagnostiquer via les onglets "Scenarios" et
     "raw_*" de l'Excel, identifier quel joint / phase pose problème
```

Exemples de lectures utiles dans le `.xlsx` :

- **Onglet Scenarios** : si une phase spécifique (par ex. `forward_back`) génère tous les errors UNSTABLE, c'est que le x_gain est mal tuné, pas le système global.
- **Onglet Summary** → Workspace used : si `x_range_mm` est < 100 mm, tu n'as pas bougé assez pour vraiment tester.
- **Onglet raw_cmd + raw_actual** : tu peux charger dans Excel / un script Python pour tracer toi-même des graphs au-delà de la fenêtre 10 s du dashboard.

---

*Voir aussi : [TELEOPERATION.md](TELEOPERATION.md) pour le pipeline technique, [TELEOP_TUNING.md](TELEOP_TUNING.md) pour la référence des paramètres internes.*

*Dernière mise à jour : 22 avril 2026*
