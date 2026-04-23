# Manuel utilisateur — Dashboard de téléopération (ABMI v2.2)

Guide d'utilisation du [`teleop_dashboard.py`](../teleop/teleop_dashboard.py) : tuning en direct, suivi sim ↔ réel côte à côte, caméra opérateur intégrée et boutons d'action dynamiques.

> Prérequis : rosbridge sur `ws://localhost:9090` (T1), Gazebo + controllers actifs ou `bridge_tour` vers le Pi en cours (T2), teleop en cours avec `--use-rosbridge` (T3). Voir [TELEOPERATION.md](TELEOPERATION.md) pour le workflow complet.

---

## 1. Lancement

```bash
conda activate hand-teleop
cd ~/ros_jazzy/src/mycobot_R6A/teleop
python3 teleop_dashboard.py
```

La fenêtre s'ouvre en **1500 × 950** sur la charte **ABMI** (navy `#1B1A3E` + pink `#E6417A`, thème ttkbootstrap *darkly*). Le logo se charge automatiquement depuis [`teleop/assets/abmi_logo.png`](../teleop/assets/).

---

## 2. Bandeau du haut

```
┌─────────────────────────────────────────────────────────────────────┐
│  [ABMI logo]  MyCobot 320 Pi — Teleop Performance     🟦 SIM (Gazebo)│
│               Live tuning · Sim ↔ Real comparison                    │
└─────────────────────────────────────────────────────────────────────┘
```

### Badge de mode (en haut à droite)

Détecté automatiquement selon la fraîcheur des topics `/joint_states` (sim) et `/from_robot` (réel) — fenêtre de 2 s.

| État | Badge | Couleur | Signification |
|------|-------|---------|---------------|
| `SIM` | 🟦 SIM (Gazebo) | bleu | Seuls les `/joint_states` Gazebo arrivent |
| `REAL` | 🟥 REAL (MyCobot) | rose ABMI | Seuls les `ANGLES:` du Pi arrivent |
| `BOTH` | ⚡ SIM + REAL | jaune sur navy | Les deux streams sont live — mode comparaison |
| `OFFLINE` | ⚪ OFFLINE | gris | Rien reçu depuis > 2 s |

---

## 3. L'onglet 🏠 Home

Au-dessus de tout : **5 KPI cards** avec barre d'accent à gauche (couleur contextuelle).

| Carte | Mesure | Couleur adaptative |
|-------|--------|--------------------|
| **Execution mode** | Reflète le badge mode | Héritée du badge |
| **Command rate** | Hz mesuré sur `/mycobot_controller/joint_trajectory` | Vert ≥ 20 Hz · Jaune sinon · Gris si vide |
| **SIM tracking — avg RMS** | Moyenne des 6 joints sur la fenêtre 10 s (Gazebo) | Vert < 5° · Jaune < 15° · Rose au-delà |
| **REAL tracking — avg RMS** | Idem côté robot physique | Mêmes seuils |
| **Signal health** | Pire flag parmi tous les joints (OK / JITTERY / UNSTABLE) | Vert / Jaune / Rose |

### Caméra opérateur (panneau gauche)

Feed JPEG annoté (Wilor skeleton + pose) publié par `mycobot_teleop.py` sur `/teleop/camera/image` à ~10 Hz (downscale 320 px de large, qualité 60). La dernière frame s'affiche en inline avec un footer horodaté — plus besoin de la fenêtre OpenCV séparée.

**Si le panneau affiche "waiting for /teleop/camera/image…"** :
- Le teleop tourne-t-il bien en `--use-rosbridge` ?
- Pillow/ImageTk disponible ? (`pip install pillow` dans l'env `hand-teleop`)

### Comparatif SIM ↔ REAL (panneau droit)

Deux mini-plots empilés :

1. **Per-joint RMS tracking error** — bar chart 2 barres par joint (bleu = SIM, rose = REAL). Lignes pointillées à 5° (cible) et 15° (alerte). Regarde à vue d'œil si un joint tracke moins bien côté robot réel vs. sim.
2. **Hand position (cm)** — XYZ relatifs de la main après filtres Wilor. **L'échelle est en cm** (pas en m comme dans l'ancienne version) pour comparaison plus directe avec les mm du workspace.

### Quick actions (bas de l'onglet)

Les 5 boutons sont des **ActionButton** : tooltip au survol, feedback visuel au clic (label `⟳` → `✓`/`✗`), désactivation pendant l'action, et une **ligne de statut toast** en dessous qui horodate le résultat (vert = succès, rose = erreur, fondu vers gris après ~4,5 s).

| Bouton | Publie | Effet |
|--------|--------|-------|
| 🏠 **Send robot home** | `/to_robot` : `"home"` | bridge_tour → Pi → `send_angles([0, 8, -127, 40, 0, 0])` (pose home officielle) |
| ⊘ **Stop (release servos)** | `/to_robot` : `"stop"` | Relâche les servos. **Maintenir le bras avant de cliquer** — il va tomber ! |
| ⟲ **Recalibrate hand origin** | `/teleop/recalibrate` (Empty) | Reset de l'initial_pose Wilor → la prochaine frame devient l'origine du mapping |
| 📊 **Run performance analyzer** | Lance `performance_analyzer.py --guided` | Protocole scripté 64 s → rapport `.xlsx` à côté |
| 💾 **Export CSV snapshot** | Fichier local `snapshot_YYYYMMDD_HHMMSS.csv` | Dump de cmd / sim / real de la fenêtre courante |

**Exemple de toast** : `✓  🏠  Send robot home · 14:23:05`

---

## 4. L'onglet 📊 Analytics

Cinq plots matplotlib sur fond navy, fenêtre glissante 10 s, rafraîchis toutes les 250 ms :

```
┌─────────────────────────────────────────────────────────┐
│  [Wilor hand position (cm)]   — grand plot full-width    │
├──────────────────────────────┬──────────────────────────┤
│ [Joint angles — SIM]         │ [Joint angles — REAL]     │
│  solide = commandé           │  solide = commandé        │
│  pointillé = /joint_states   │  pointillé = /from_robot  │
├──────────────────────────────┼──────────────────────────┤
│ [Tracking error — SIM]       │ [Tracking error — REAL]   │
│  |cmd − sim| par joint       │  |cmd − real| par joint   │
│  ligne cible 5°              │  ligne cible 5°           │
└──────────────────────────────┴──────────────────────────┘
```

Les plots marqués *"not active in current mode"* sont en attente du stream correspondant (ex: REAL hors ligne → les deux colonnes droites affichent le placeholder).

En bas : ligne mono-space **stats strip** :

```
SIM avg RMS 2.14° · flags: O/O/J/O/O/O    REAL avg RMS 5.32° · flags: O/O/U/O/O/J
```

(`O` = OK, `J` = JITTERY, `U` = UNSTABLE)

---

## 5. L'onglet 🎛️ Tuning

### Les 4 sliders live

Tous les changements publient sur `/teleop/gains` (`Float64MultiArray [x, y, z, tfs]`) — le teleop applique à la prochaine frame (~33 ms à 30 Hz).

| Slider | Plage | Défaut | Hint affiché |
|--------|-------|--------|--------------|
| **X gain — hand forward → J3 elbow** | 0.3 → 3.0 | 1.20 | *Plus = petit mouvement avant produit un grand mouvement de coude* |
| **Y gain — hand lateral → J1 base yaw** | 0.3 → 3.0 | 1.20 | *Plus = petit mouvement latéral produit une grande rotation base* |
| **Z gain — hand vertical → J2 + J5** | 0.3 → 3.0 | 1.60 | *Plus = petit mouvement vertical produit un grand tilt épaule+poignet* |
| **time_from_start (s)** | 0.1 → 2.5 | 0.25 | *Plus bas = plus réactif (risqué) · plus haut = plus lissé (mais lent)* |

La valeur courante s'affiche en gros à droite de chaque slider (JetBrains Mono) et suit en direct le drag.

### Bouton Recalibrate (dans l'onglet Tuning aussi)

Même effet que celui du Home — `ActionButton` avec tooltip et feedback. Pratique quand tu es en train de tuner et que tu veux re-zéro sans changer d'onglet.

### Presets

Trois boutons sous les sliders — **le preset actif reste mis en évidence** (bootstyle solide) tant qu'aucun slider n'est modifié ni qu'un autre preset n'est cliqué.

| Preset | Valeurs (x, y, z, tfs) | Quand l'utiliser |
|--------|------------------------|------------------|
| 🐢 **Safe start** | 0.6 · 0.6 · 0.6 · 0.30 | Première session sur robot physique. Débit lent, amplitude réduite. Validé sur le MyCobot 320 Pi le 22/04/2026. |
| ⚙️ **Nominal** | 1.2 · 1.2 · 1.6 · 0.25 | Point de fonctionnement post-calibration, validé sur physique. **Default au démarrage.** |
| ⚡ **Reactive** | 1.6 · 1.6 · 2.0 · 0.15 | Débit rapide — uniquement quand l'opérateur est calé et l'espace dégagé. |

---

## 6. Workflow tuning complet

### 1. Démarrage

```
T1 rosbridge  →  T2 Gazebo (ou bridge_tour)  →  T3 teleop --use-rosbridge  →  T4 dashboard
```

### 2. Calibration

Dans l'onglet **Home** : paume ouverte à 50 cm face caméra, clique **⟲ Recalibrate hand origin**. Attends 1 s que le toast `✓ Recalibrate…` apparaisse.

### 3. Mode comparaison SIM ↔ REAL

Laisse tourner `target:=both` dans ton launch → badge **⚡ SIM + REAL**. Les deux colonnes des KPI cards et le bar chart Home te permettent de voir immédiatement où la sim diverge du réel (souvent en tracking error sur J3 elbow parce que le gear ratio du MyCobot n'est pas modélisé dans le DART).

### 4. Tuning

Onglet **Tuning** :
- Démarre avec le preset **🐢 Safe start**
- Ajuste X/Y/Z progressivement — regarde la KPI **REAL tracking** sur Home
- `time_from_start` : descends vers 0.15 seulement si tout reste vert
- En cas d'instabilité : preset **🐢 Safe start** pour tout remettre en place, puis recommence

### 5. Validation avant robot réel

Retour Home :
- **SIM tracking avg RMS** < 5° pendant 30 s continues
- **REAL tracking avg RMS** idem si `target:=both`
- **Signal health** au vert
- Clique **📊 Run performance analyzer** — le `.xlsx` te donnera un verdict formel
- Si `READY FOR REAL ROBOT` → go

---

## 7. Topics utilisés

### Souscriptions (lectures)

| Topic | Type | Usage |
|-------|------|-------|
| `/teleop/hand_xyz` | `geometry_msgs/Vector3Stamped` | Plot hand position + carte rate |
| `/teleop/camera/image` | `sensor_msgs/CompressedImage` | Panneau caméra Home |
| `/mycobot_controller/joint_trajectory` | `trajectory_msgs/JointTrajectory` | Courbe solide des plots joints · base pour toutes les erreurs |
| `/joint_states` | `sensor_msgs/JointState` | Pointillé SIM · KPI SIM tracking |
| `/from_robot` | `std_msgs/String` | Parse `ANGLES: [...]` → pointillé REAL · KPI REAL tracking |

### Publications (écritures)

| Topic | Type | Déclencheur |
|-------|------|-------------|
| `/teleop/gains` | `std_msgs/Float64MultiArray` | Sliders + presets tuning |
| `/teleop/recalibrate` | `std_msgs/Empty` | Bouton Recalibrate (Home et Tuning) |
| `/to_robot` | `std_msgs/String` | Boutons Home / Stop + polling passif `get_angles` (0.3 s) pour remonter les angles réels |

---

## 8. Troubleshooting

| Problème | Vérifier | Solution |
|----------|----------|----------|
| Fenêtre ne s'ouvre pas | `echo $DISPLAY` | `DISPLAY=:0 python3 teleop_dashboard.py` |
| `cannot reach rosbridge` | T1 lancé, port 9090 libre | `nc -zv localhost 9090` ; sinon relance T1 |
| Badge reste **⚪ OFFLINE** | Teleop lancé avec `--use-rosbridge` ? Topics advertized ? | `ros2 topic hz /mycobot_controller/joint_trajectory` |
| Caméra Home vide en permanence | PIL/Pillow absent · teleop sans `--use-rosbridge` | `pip install pillow` dans `hand-teleop` |
| KPI **Command rate** à 0 Hz | Teleop publie-t-il bien ? | Regarder T3 : voir si `tracking_paused` est True |
| **REAL tracking — avg RMS** reste "—" | bridge_tour / Pi OK ? | `ros2 topic echo /from_robot` — devrait afficher `ANGLES: [..]` |
| Toast bloqué sur `✗ … failed` | Action a levé une exception | Regarder la console du dashboard — le traceback imprime la raison |
| Preset ne reste pas highlighted | Tu as ensuite bougé un slider | Comportement normal : dès qu'un slider bouge, l'état preset est "cassé" |
| Signal health reste UNSTABLE | Window 10 s bloquée sur le pic | Clique Recalibrate + attends 30 s |

---

## 9. Changements depuis la v2.1 (22/04/2026)

- **Refonte 3 onglets** (Home / Analytics / Tuning) · charte **ABMI** navy+pink + logo
- **5 KPI cards** avec couleurs contextuelles (vert/jaune/rose selon les seuils)
- **Caméra opérateur intégrée** (topic `/teleop/camera/image`) — plus de fenêtre OpenCV
- **Badge de mode** automatique SIM / REAL / BOTH / OFFLINE
- **Comparaison SIM ↔ REAL** côte à côte (bar chart + plots miroir dans Analytics)
- **Hand position en cm** au lieu de m
- **ActionButton dynamiques** : tooltip au survol, feedback `⟳ → ✓/✗`, toast horodaté
- **Presets de gains** (Safe / Nominal / Reactive) avec highlight du preset actif
- Polling passif de `get_angles` pour pouvoir tracer les angles réels quand `/joint_states` n'est pas là

---

*Voir aussi : [TELEOPERATION.md](TELEOPERATION.md) pour le pipeline technique, [TELEOP_TUNING.md](TELEOP_TUNING.md) pour la référence des paramètres internes, [TELEOP_ARCHITECTURE_VIZ.md](TELEOP_ARCHITECTURE_VIZ.md) pour le visuel complet.*

*Dernière mise à jour : 23 avril 2026.*
