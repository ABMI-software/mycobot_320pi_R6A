# Session de Test du Bridge MyCobot

## Configuration réseau actuelle

- **Raspberry Pi** : 10.10.0.218:5005 (Ubuntu 20.04, ROS2 Galactic)
- **PC Tour** : 10.10.0.115 (Ubuntu 24.04, ROS2 Jazzy)

## ✅ Test Réussi - Session du 6 mars 2026

### Terminal Pi (bridge_pi.py)
```bash
er@er:~/colcon_ws/src/mycobot_ros2/mycobot_320/mycobot_320pi/mycobot_gateway$ python3 bridge_pi.py
[INFO] [1772805422.178179358] [bridge_pi]: Bridge Bidirectionnel démarré. Port 5005.
[INFO] [1772805993.404149907] [bridge_pi]: Tour connectée : ('10.10.0.115', 41754)
[INFO] [1772806749.926792273] [bridge_pi]: Tour connectée : ('10.10.0.115', 44684)
[INFO] [1772806775.686673937] [bridge_pi]: Tour connectée : ('10.10.0.115', 58414)
[INFO] [1772806998.371606830] [bridge_pi]: Commande relayée : test_moteur_1
```

### Terminal 1 Tour (bridge_tour)
```bash
genji@genji-HP:~/ros_jazzy/src/mycobot_R6A$ conda deactivate
genji@genji-HP:~/ros_jazzy/src/mycobot_R6A$ source install/setup.bash
genji@genji-HP:~/ros_jazzy/src/mycobot_R6A$ ros2 run mycobot_gateway bridge_tour
[INFO] [1772806775.696938571] [bridge_tour]: ✅ Connecté à la Pi (10.10.0.218)
[INFO] [timestamp] [bridge_tour]: 📤 Envoyé vers Pi: test_moteur_1
```

### Terminal 2 Tour (vérification topics)
```bash
genji@genji-HP:~/ros_jazzy$ conda deactivate
genji@genji-HP:~/ros_jazzy$ source src/mycobot_R6A/install/setup.bash
genji@genji-HP:~/ros_jazzy$ ros2 topic list
/from_robot
/parameter_events
/rosout
/to_robot
```

### Terminal 3 Tour (envoi commande)
```bash
genji@genji-HP:~/ros_jazzy$ conda deactivate
genji@genji-HP:~/ros_jazzy$ source src/mycobot_R6A/install/setup.bash
genji@genji-HP:~/ros_jazzy$ ros2 topic pub /to_robot std_msgs/msg/String "{data: 'test_moteur_1'}" -1
publisher: beginning loop
publishing #1: std_msgs.msg.String(data='test_moteur_1')
```

## 📝 Points importants identifiés

### ✅ Ce qui fonctionne
1. ✅ La connexion TCP Tour → Pi est établie
2. ✅ Le bridge_tour se connecte correctement
3. ✅ Les topics ROS2 sont créés (`/to_robot`, `/from_robot`)
4. ✅ Le message est bien relayé par la Pi

### ⚠️ Problèmes résolus

#### Problème 1 : "No executable found"
**Cause** : Les console_scripts Python s'installent dans `bin/` mais ROS2 cherche dans `lib/<package_name>/`

**Solution appliquée** :
- Création d'un script wrapper dans `scripts/bridge_tour`
- Modification de `setup.py` pour installer le script dans `lib/mycobot_gateway/`

#### Problème 2 : "ModuleNotFoundError: rclpy._rclpy_pybind11"
**Cause** : Conda active Python 3.13, ROS2 Jazzy nécessite Python 3.12

**Solution** : `conda deactivate` dans tous les terminaux ROS2

#### Problème 3 : "Waiting for at least 1 matching subscription(s)..."
**Cause** : Environnements ROS2 différents entre terminaux

**Solution** : 
- Sourcer `install/setup.bash` dans **tous** les terminaux
- NE PAS utiliser `ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST` (ou le définir partout)

## 🎯 Workflow recommandé

### Pour chaque nouveau terminal :
```bash
cd /home/genji/ros_jazzy/src/mycobot_R6A
conda deactivate  # IMPORTANT !
source install/setup.bash
```

### Ou utiliser le script rapide :
```bash
source /home/genji/ros_jazzy/src/mycobot_R6A/quick_commands.sh
setup_bridge  # Configure l'environnement
```

## 🔄 Séquence de démarrage complète

### 1. Sur la Raspberry Pi
```bash
cd ~/colcon_ws/src/mycobot_ros2/mycobot_320/mycobot_320pi/mycobot_gateway
python3 bridge_pi.py
```

### 2. Sur le PC Tour - Terminal 1 (Bridge)
```bash
source quick_commands.sh
start_bridge
# Ou manuellement :
# conda deactivate
# source install/setup.bash
# ros2 run mycobot_gateway bridge_tour
```

### 3. Sur le PC Tour - Terminal 2 (Commandes)
```bash
source quick_commands.sh
send_cmd "test_moteur_1"
# Ou manuellement :
# conda deactivate
# source install/setup.bash
# ros2 topic pub /to_robot std_msgs/msg/String "{data: 'test_moteur_1'}" -1
```

### 4. Sur le PC Tour - Terminal 3 (Monitoring)
```bash
source quick_commands.sh
listen_robot
# Ou manuellement :
# conda deactivate
# source install/setup.bash
# ros2 topic echo /from_robot
```

## 📊 Résultat attendu

Quand tout fonctionne correctement :

1. **Bridge Pi** affiche : `[INFO] Tour connectée : ('10.10.0.115', xxxxx)`
2. **Bridge Tour** affiche : `[INFO] ✅ Connecté à la Pi (10.10.0.218)`
3. À l'envoi d'une commande :
   - **Terminal 3 (pub)** : `publishing #1: std_msgs.msg.String(data='test_moteur_1')`
   - **Terminal 1 (bridge)** : `[INFO] 📤 Envoyé vers Pi: test_moteur_1`
   - **Bridge Pi** : `[INFO] Commande relayée : test_moteur_1`

## 🐛 Debug

### Vérifier la connectivité réseau
```bash
ping 10.10.0.218
nc -zv 10.10.0.218 5005
```

### Vérifier les topics ROS2
```bash
conda deactivate
source install/setup.bash
ros2 topic list
ros2 topic info /to_robot -v
ros2 topic info /from_robot -v
```

### Vérifier l'environnement Python
```bash
which python3  # Doit pointer vers /usr/bin/python3, pas miniconda
python3 --version  # Doit être 3.12.x pour Jazzy
```

### Logs détaillés
```bash
ros2 run mycobot_gateway bridge_tour --ros-args --log-level debug
```

## 📈 Métriques de performance

- **Latence réseau Pi ↔ Tour** : ~10-50ms (dépend du réseau local)
- **Temps de connexion initial** : ~100-200ms
- **Taille max message** : 1024 bytes par recv()

## 🔮 Prochaines étapes suggérées

1. Ajouter un heartbeat pour détecter les déconnexions
2. Implémenter la reconnexion automatique
3. Créer un fichier launch ROS2
4. Ajouter des métriques de monitoring (latence, nb messages, etc.)
5. Supporter des types de messages structurés (JSON)
