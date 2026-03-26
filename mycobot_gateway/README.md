# MyCobot Gateway Bridge

Bridge réseau ROS2 pour communication entre MyCobot Pi (ROS2 Galactic) et PC Tour (ROS2 Jazzy).

## 📋 Description

Ce package permet de relayer les messages ROS2 entre deux machines via TCP/IP :
- **Raspberry Pi** (Ubuntu 20.04, ROS2 Galactic) — contrôle le MyCobot 320 Pi
- **PC Tour** (Ubuntu 24.04, ROS2 Jazzy) — envoie des commandes

### Architecture

```
┌─────────────────────┐         TCP         ┌──────────────────────┐
│   PC Tour (Jazzy)   │◄───────5005────────►│  Raspberry Pi        │
│                     │                      │  (Galactic)          │
│  bridge_tour        │                      │  bridge_pi           │
│  - Sub: /to_robot   │                      │  - Pub: /to_robot    │
│  - Pub: /from_robot │                      │  - Sub: /from_robot  │
└─────────────────────┘                      └──────────────────────┘
                                                       │
                                                       ▼
                                               ┌──────────────┐
                                               │ MyCobot 320  │
                                               └──────────────┘
```

## ⚙️ Configuration

### IP de la Raspberry Pi

Par défaut, le bridge se connecte à `10.10.0.218:5005`. Pour changer l'IP :

Éditer `mycobot_gateway/bridge_tour.py` :
```python
self.pi_ip = '10.10.0.218'  # Modifier cette ligne
self.port = 5005
```

Puis rebuild :
```bash
colcon build --packages-select mycobot_gateway --symlink-install
```

## 🚀 Installation

### Sur le PC Tour (Ubuntu 24.04, ROS2 Jazzy)

```bash
# Depuis le workspace
cd /home/genji/ros_jazzy/

# Build du package
colcon build --packages-select mycobot_gateway --symlink-install

# Sourcer l'environnement
source install/setup.bash
```

## 📦 Utilisation

### ⚠️ Important : Désactiver Conda

ROS2 Jazzy utilise Python 3.12, mais conda active Python 3.13 par défaut. Il **faut** désactiver conda avant de lancer ROS2 :

```bash
conda deactivate
```

### Lancer le bridge

**Terminal 1** — Lancer le bridge :
```bash
cd /home/genji/ros_jazzy/src/mycobot_R6A
conda deactivate
source install/setup.bash
ros2 run mycobot_gateway bridge_tour
```

Vous devriez voir :
```
[INFO] [timestamp] [bridge_tour]: ✅ Connecté à la Pi (10.10.0.218)
```

### Envoyer une commande au robot

**Terminal 2** — Publier un message :
```bash
cd /home/genji/ros_jazzy/src/mycobot_R6A
conda deactivate
source install/setup.bash
ros2 topic pub /to_robot std_msgs/msg/String "{data: 'test_moteur_1'}" -1
```

Le terminal 1 devrait afficher :
```
[INFO] [timestamp] [bridge_tour]: 📤 Envoyé vers Pi: test_moteur_1
```

### Écouter les réponses du robot

**Terminal 3** — Écouter les messages provenant du robot :
```bash
cd /home/genji/ros_jazzy/src/mycobot_R6A
conda deactivate
source install/setup.bash
ros2 topic echo /from_robot
```

## 🔧 Dépannage

### "No executable found"

**Cause** : Le package n'est pas dans le PATH ROS2.

**Solution** :
```bash
# Vérifier que vous avez sourcé le bon environnement
source install/setup.bash  # PAS seulement /opt/ros/jazzy/setup.bash

# Vérifier que l'exécutable existe
ros2 pkg executables mycobot_gateway
# Devrait afficher : mycobot_gateway bridge_tour
```

### "ModuleNotFoundError: No module named 'rclpy._rclpy_pybind11'"

**Cause** : Conda est activé avec Python 3.13, incompatible avec ROS2 Jazzy (Python 3.12).

**Solution** :
```bash
conda deactivate
```

### "Waiting for at least 1 matching subscription(s)..."

**Cause** : Environnements ROS2 différents entre terminaux, ou `ROS_AUTOMATIC_DISCOVERY_RANGE` mal configuré.

**Solution** :
```bash
# Dans TOUS les terminaux ROS2 :
cd /home/genji/ros_jazzy/src/mycobot_R6A
conda deactivate
source install/setup.bash

# Ne PAS définir ROS_AUTOMATIC_DISCOVERY_RANGE
# (ou le définir identiquement partout)
```

### "Impossible de se connecter à la Pi"

**Causes possibles** :
1. La Pi n'est pas allumée ou pas sur le réseau
2. L'IP est incorrecte (vérifier avec `ping 10.10.0.218`)
3. Le bridge_pi n'est pas lancé sur la Pi
4. Firewall bloque le port 5005

**Vérification** :
```bash
# Tester la connectivité
ping 10.10.0.218

# Tester si le port 5005 est ouvert
nc -zv 10.10.0.218 5005
```

## 📊 Topics ROS2

| Topic          | Type              | Direction        | Description                    |
|----------------|-------------------|------------------|--------------------------------|
| `/to_robot`    | `std_msgs/String` | Tour → Pi        | Commandes envoyées au robot    |
| `/from_robot`  | `std_msgs/String` | Pi → Tour        | Réponses du robot              |

## 🧪 Script de test

Un script de test est fourni pour vérifier la configuration :

```bash
./test_bridge.sh
```

## 📝 Notes

- Le bridge utilise une socket TCP persistante
- Les messages sont encodés en UTF-8 avec un `\n` de fin
- Si la connexion est perdue, le bridge se termine (pas de reconnexion automatique pour l'instant)
- Le thread de réception est en mode `daemon` et se termine avec le nœud principal

## 🔮 Améliorations futures

- [ ] Reconnexion automatique en cas de perte de connexion
- [ ] Configuration via paramètres ROS2 (IP, port)
- [ ] Fichier de lancement (launch file)
- [ ] Support de types de messages plus complexes (JSON, protobuf)
- [ ] Monitoring de la latence réseau
- [ ] Tests unitaires

## 📄 Licence

Apache License 2.0

## 👤 Auteur

José BERNARDO

---

**Version** : 0.0.1  
**ROS2 Distro** : Jazzy  
**Python** : 3.12
