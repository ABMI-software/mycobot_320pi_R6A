# Lancer le dashboard de validation DREAM

*Procédure vérifiée le 2026-07-16 sur le robot réel (`10.10.0.221`).*

Le dashboard est un **pur consommateur** : il ne capture pas d'image, ne fait pas
d'inférence, ne parle pas au robot. Lancé seul, il affiche une fenêtre vide
(`Caméra : 0.0 FPS`, `DREAM : 0.0 Hz`, encodeurs à `0.0`). Il lui faut **quatre
autres nœuds**, chacun dans son propre terminal, tous laissés ouverts.

## Le piège numéro un — le `.venv`

**Toute commande ROS2 échoue si `(.venv)` apparaît dans le prompt.** Le venv à la
racine du repo met son `bin/` en tête du `PATH`, donc `python3` devient celui du
venv, avec son NumPy 2.4.4 et son PyQt5/cv2 :

| Symptôme | Cause |
|----------|-------|
| `Could not load the Qt platform plugin "xcb" in ".../.venv/.../cv2/qt/plugins"` | Le Qt de `cv2` (venv) écrase celui de PyQt5 |
| `KeyError: 16` dans `cv_bridge.cv2_to_imgmsg` | NumPy 2.4.4 (venv) vs `cv_bridge` compilé NumPy 1.x |

**Correctif :** `deactivate` avant de sourcer. C'est tout — inutile de changer de
répertoire (`ros2 run` résout via `install/`, pas via le CWD) et inutile de
réinstaller DREAM. Le réglage `"python.terminal.activateEnvironment": false` dans
`.vscode/settings.json` empêche VS Code de le réactiver à chaque nouveau terminal.

**Vérification en une ligne** — doit afficher `/usr/lib/python3/dist-packages/…`,
jamais `.venv` :

```bash
python3 -c "import numpy; print(numpy.__file__)"
```

## Préambule (dans chacun des 5 terminaux)

```bash
deactivate                              # si (.venv) est dans le prompt
source /opt/ros/jazzy/setup.bash
source ~/Osama_ws/install/setup.bash    # PAS ~/ros_jazzy/install — autre clone
```

## Les 5 nœuds

```bash
# 1 — caméra Arducam → /camera/image_raw
ros2 run mycobot_gateway camera_publisher

# 2 — inférence DREAM → /dream/keypoints
#     ⚠ défauts trompeurs : camera_topic=/synth_camera/image (Gazebo)
#       et model_name=vgg_weighted_e50 (ancien). À surcharger :
ros2 run mycobot_gateway dream_inference --ros-args \
  -p camera_topic:=/camera/image_raw \
  -p model_name:=vgg_ultimate_v4_mix_ft_e30

# 3 — encodeurs → /joint_states   (sans lui : statut NO_JOINTS)
ros2 run mycobot_gateway joint_sync

# 4 — pont ROS2 ↔ Pi (TCP 5005)   (sans lui : le bras ne bouge pas)
ros2 run mycobot_gateway bridge_tour

# 5 — le dashboard
ros2 run mycobot_gateway dream_validation_dashboard
```

Côté Pi, `bridge_pi_simple.py` doit tourner. Vérification :
`ping -c1 10.10.0.221` puis `bash scripts/real_robot_preflight.sh`.

## Raccourci — launch unique multi-caméras (2026-07-23/24)

Un seul launch remplace les 5 terminaux et **auto-détecte 1 ou 2 caméras** :

```bash
ros2 launch mycobot_gateway dream_multicam.launch.py
ros2 launch mycobot_gateway dream_multicam.launch.py cameras:=arducam   # forcer mono
```

Il sonde `v4l2-ctl` (`vision/camera_registry.py`), spawne une branche
`camera_publisher + dream_inference` par caméra reconnue, chacune avec son
intrinsèque (`cam_3`/`cam_2`) et son exposition (arducam 75, SVPRO normale), puis
les nœuds partagés `joint_sync`, `bridge_tour` et le dashboard (param `cameras`).

### Graphe des nœuds / topics (2 caméras)

```
camera_publisher_arducam ─ /camera/image_raw ─────→ dream_inference_arducam ─ /dream/keypoints ──────┐
camera_publisher_svpro   ─ /camera_svpro/image_raw → dream_inference_svpro   ─ /dream_svpro/keypoints ┤
joint_sync ─ /joint_states                                                                           ├→ dream_validation_dashboard
bridge_tour ↔ TCP 5005 ↔ Pi ─ /from_robot, /to_robot                                                 ┘
```

Chaque `camera_publisher` prend son topic via le param `output_topic` ; chaque
`dream_inference` publie sous le préfixe `output_prefix` (`/dream` vs
`/dream_svpro`). Inspecter en direct : `rqt_graph`, `ros2 node list`,
`ros2 topic list` (en fusion : `/dream/*` **et** `/dream_svpro/*` présents),
`ros2 topic hz /dream_svpro/keypoints`.

- **≥2 caméras calibrées** → le dashboard passe en **fusion *solve-then-fuse***
  (chaque caméra résout son propre `q`, puis fusion **par joint** pondérée par
  l'observabilité — **pas** un `q` partagé, qui basculait de branche). Badge
  « 🔗 FUSION N vues » ; repli « MONO via {caméra} » si la primaire aveugle.
- **1 caméra** → mode mono (comportement historique).

`deactivate` le `.venv` d'abord malgré tout. Détail :
[`DREAM_VALIDATION_DASHBOARD.md` § Multi-caméras](DREAM_VALIDATION_DASHBOARD.md).

## Diagnostic

| Ce que montre le dashboard | Nœud manquant / cause | Action |
|---------------------------|----------------------|--------|
| `Caméra : 0.0 FPS`, image noire | `camera_publisher` | Le relancer |
| `DREAM : 0.0 Hz`, colonne px vide | `dream_inference`, ou il écoute `/synth_camera/image` | Relancer avec `-p camera_topic:=/camera/image_raw` |
| `Statut : NO_JOINTS`, encodeurs `0.0` | `joint_sync` | Le relancer |
| `SET Angles` sans effet, `/from_robot` muet | `bridge_tour` mort | Le relancer ; sinon redémarrer le bridge du Pi |
| `État servos : relâché (power_off envoyé)` | servos coupés | **Tenir le bras**, puis cliquer 🔒 Fixer (power_on) |

Chaîne complète : `camera_publisher` → `/camera/image_raw` → `dream_inference` →
`/dream/keypoints` → dashboard ; en parallèle `joint_sync` → `/joint_states` et
`bridge_tour` ↔ TCP 5005 ↔ Pi pour `/from_robot` et `/to_robot`.

---

# Diagnostic latence — 2026-07-16

*Point de départ : `Latence totale image → angles : 1485 ms`, `DREAM : 1.7 Hz`,
`Caméra : 4.7 FPS` alors que le nœud est réglé à 30 fps. **Aucun code n'a été
gardé** — tout a été reverté à la demande. Ce qui suit est le diagnostic, pour
repartir de là sans refaire les mesures.*

## Cause racine : `net.core.rmem_max` (non corrigée)

```
net.core.rmem_max = 212992      →  208 Ko   (défaut Linux, jamais touché)
taille d'une image             →  921 Ko   (640×480×3, brute)
RMW                            →  rmw_fastrtps_cpp (Fast DDS, UDP)
```

**Le tampon de réception du noyau est 4.4× plus petit qu'une seule image.** Fast DDS
fragmente les 921 Ko en ~640 datagrammes ; le socket déborde, des fragments sont
perdus, et Fast DDS jette l'image entière. C'est le problème classique de ROS2 avec
`sensor_msgs/Image` brut.

Correctif à tester (nécessite sudo, non persistant, réversible) :
```bash
sudo sysctl -w net.core.rmem_max=8388608      # ~9 images ; doc Fast DDS
# puis redémarrer les nœuds (les sockets sont créés au démarrage)
# retour arrière : sudo sysctl -w net.core.rmem_max=212992
```

## Preuves qui isolent la cause

| Mesure | Résultat | Ce que ça élimine |
|--------|----------|-------------------|
| `dmesg` / `kern.log` | 0 erreur USB/UVC/xhci | matériel, driver |
| Banc hors ROS (`bench_capture.py`) | **0 gel / 800 captures**, 27.8 Hz | OpenCV, capture |
| `camera_publisher` instrumenté | **0 callback > 100 ms** | le nœud publie bien à 30 Hz |
| `ros2 topic hz` côté abonné | 5 Hz, trous jusqu'à **3.6 s** | → le transport DDS |
| Gels avec / sans `dream_inference` | 12 vs 10 par 30 s (**identique**) | le nombre d'abonnés |

Le publisher est sain, les abonnés ne reçoivent pas : la perte est **entre les deux**.

⚠️ Deux pièges de mesure rencontrés — le pourcentage de gels est trompeur (il varie
avec le nombre de messages reçus, pas avec le nombre de gels : toujours ~10 par 30 s),
et le **débit médian** est la bonne métrique (la moyenne est écrasée par les gels).

## Ce qui n'est PAS en cause

- `exposure_dynamic_framerate` : mis à 0, effet nul (1.91 → 2.16 Hz).
- Le dashboard : il sature un cœur (102% CPU) et double la cadence quand on le
  ferme, mais il ne cause **pas** les gels (identiques avec 0 abonné).

## Rappels

- **DREAM vit dans `/tmp/DREAM`** (editable dans `venv_dream`), et `/tmp` est vidé
  au reboot : il faut donc **le re-cloner / réinstaller à chaque redémarrage**.
  Vérifier avec `import dream` ; s'il manque, refaire le clone dans `/tmp/DREAM`.
- La case **⚠ Mode cohérence** est **OFF par défaut** et doit le rester pour une
  mesure DREAM indépendante. Voir `CLAUDE.md` § 2026-07-15.
- Le robot **n'a pas de gripper**.
- Sur la fiabilité des angles mesurés, voir
  [`training/dream/J4_OBSERVABILITY_DIAG.md`](../training/dream/J4_OBSERVABILITY_DIAG.md).
