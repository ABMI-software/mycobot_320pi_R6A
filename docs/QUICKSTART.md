# 🚀 Guide de Démarrage Rapide - MyCobot Gateway Bridge

## ⚡ Démarrage en 3 étapes

### 1️⃣ Sur la Raspberry Pi (Ubuntu 20.04 + ROS2 Galactic)

```bash
cd ~/colcon_ws/src/mycobot_ros2/mycobot_320/mycobot_320pi/mycobot_gateway
python3 bridge_pi.py
```

Attendez le message :
```
[INFO] [bridge_pi]: Bridge Bidirectionnel démarré. Port 5005.
```

### 2️⃣ Sur le PC Tour (Ubuntu 24.04 + ROS2 Jazzy)

```bash
cd /home/genji/ros_jazzy/src/mycobot_R6A
conda deactivate  # IMPORTANT !
source install/setup.bash
ros2 run mycobot_gateway bridge_tour
```

Attendez le message :
```
[INFO] [bridge_tour]: ✅ Connecté à la Pi (10.10.0.218)
```

### 3️⃣ Envoyer une commande au robot

Nouveau terminal sur le PC Tour :
```bash
cd /home/genji/ros_jazzy/src/mycobot_R6A
conda deactivate
source install/setup.bash
ros2 topic pub /to_robot std_msgs/msg/String "{data: 'test_moteur_1'}" -1
```

Vous devriez voir :
- **Terminal bridge_tour** : `[INFO] 📤 Envoyé vers Pi: test_moteur_1`
- **Terminal bridge_pi** : `[INFO] Commande relayée : test_moteur_1`

---

## 🛠️ Scripts utiles fournis

### Script de diagnostic
Vérifie que tout est correctement configuré :
```bash
cd /home/genji/ros_jazzy/src/mycobot_R6A
source install/setup.bash
./diagnose.sh
```

### Commandes rapides
```bash
source /home/genji/ros_jazzy/src/mycobot_R6A/quick_commands.sh
```

Puis utilisez les alias :
- `start_bridge` — Lance le bridge
- `test_send` — Envoie un test
- `listen_robot` — Écoute les réponses
- `send_cmd "ma_commande"` — Envoie une commande personnalisée

---

## ❗ Points importants

### ⚠️ TOUJOURS désactiver conda
```bash
conda deactivate
```
ROS2 Jazzy utilise Python 3.12, conda active Python 3.13 → incompatible !

### ⚠️ TOUJOURS sourcer le bon environnement
```bash
source install/setup.bash  # Dans /home/genji/ros_jazzy/src/mycobot_R6A
```
Ne PAS utiliser uniquement `/opt/ros/jazzy/setup.bash`

### ⚠️ Ne PAS utiliser ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
Ou alors le définir de manière identique dans **tous** les terminaux.

---

## 📋 Checklist avant de démarrer

- [ ] Raspberry Pi allumée et sur le réseau
- [ ] IP de la Pi correcte : `ping 10.10.0.218`
- [ ] bridge_pi.py lancé sur la Pi
- [ ] Conda désactivé : `conda deactivate`
- [ ] Environnement sourcé : `source install/setup.bash`
- [ ] Package builded : `ros2 pkg executables mycobot_gateway` → doit retourner `bridge_tour`

---

## 🔍 Dépannage rapide

### Problème : "No executable found"
```bash
cd /home/genji/ros_jazzy/src/mycobot_R6A
source install/setup.bash  # Important !
ros2 pkg executables mycobot_gateway  # Doit afficher bridge_tour
```

### Problème : "ModuleNotFoundError: rclpy"
```bash
conda deactivate  # Désactiver conda !
which python3     # Doit afficher /usr/bin/python3 (pas conda)
```

### Problème : "Impossible de se connecter à la Pi"
```bash
ping 10.10.0.218                    # Test connectivité
nc -zv 10.10.0.218 5005            # Test port
# Vérifier que bridge_pi.py tourne sur la Pi
```

### Problème : "Waiting for at least 1 matching subscription(s)"
```bash
# Dans TOUS les terminaux :
conda deactivate
source install/setup.bash
# Vérifier que les topics existent :
ros2 topic list  # Doit afficher /to_robot et /from_robot
```

---

## 📚 Documentation complète

- `README.md` — Documentation utilisateur détaillée
- `SESSION_TEST.md` — Logs de test et résultats
- `SUMMARY.md` — Résumé complet du projet
- `test_bridge.sh` — Script de test automatisé
- `diagnose.sh` — Script de diagnostic complet

---

## 🎯 Topics ROS2

| Topic | Type | Description |
|-------|------|-------------|
| `/to_robot` | `std_msgs/String` | Commandes Tour → Pi → Robot |
| `/from_robot` | `std_msgs/String` | Réponses Robot → Pi → Tour |

---

## 💡 Astuce

Ajoutez ceci à votre `~/.bashrc` :
```bash
alias mycobot_env='cd /home/genji/ros_jazzy/src/mycobot_R6A && conda deactivate && source install/setup.bash'
```

Ensuite, dans chaque nouveau terminal, tapez simplement :
```bash
mycobot_env
```

---

**Status** : ✅ Testé et fonctionnel  
**Date** : 6 mars 2026  
**Version** : 0.0.1
