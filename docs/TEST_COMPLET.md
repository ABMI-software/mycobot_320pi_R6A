# 🧪 TEST COMPLET DE LA COMMUNICATION

## Problème identifié
Le domaine ROS2 n'était pas exporté dans `send_cmd` !

## 🔧 Correction appliquée
Ajouté `export ROS_DOMAIN_ID=10` dans `quick_commands.sh`

## ✅ Procédure de test

### 1️⃣ Rechargez les commandes
```bash
cd /home/genji/ros_jazzy/src/mycobot_R6A
source quick_commands.sh
```

### 2️⃣ Vérifiez que bridge_tour tourne
Dans un terminal séparé :
```bash
setup_bridge
start_bridge
```

Vous devriez voir :
```
✅ Connecté à la Pi (10.10.0.218)
```

### 3️⃣ Testez la communication
```bash
send_cmd "ping"
```

**Résultat attendu :**
- **Tour (bridge_tour)** affiche : `📤 Envoyé vers Pi: ping`
- **Pi (bridge_pi_debug.py)** affiche :
  ```
  📦 Données reçues : 5 bytes
  📥 Commande : 'ping'
  🔧 Exécution : ping
  📤 Réponse envoyée : pong
  ```
- **Tour** affiche ensuite : `📥 Reçu de Pi: pong`

### 4️⃣ Autres commandes de test

```bash
# Obtenir les angles actuels
send_cmd "get_angles"

# Vérifier le statut
send_cmd "status"

# Allumer la LED en rouge
send_cmd "set_led:255,0,0"

# Aller à la position HOME
send_cmd "go_home:20"

# Bouger le joint 1 à 45° (vitesse 20)
send_cmd "set_angle:1,45,20"
```

### 5️⃣ Écouter les réponses
Dans un autre terminal :
```bash
listen_robot
```

Vous verrez les réponses du robot en temps réel.

## 🐛 Si ça ne fonctionne toujours pas

### Vérifier les logs sur le Pi
SSH vers le Pi et regardez les logs de `bridge_pi_debug.py` :
```bash
ssh er@10.10.0.218
# Terminal où bridge_pi_debug.py tourne
```

### Vérifier les topics
```bash
ros2 topic list
ros2 topic info /to_robot
ros2 topic info /from_robot
```

### Vérifier la découverte ROS2
```bash
export ROS_DOMAIN_ID=10
ros2 node list
# Devrait afficher : /bridge_tour
```

## 📊 Architecture du système

```
┌─────────────────────────────────────────────────────────────┐
│ PC Tour (10.10.0.115)                                       │
│                                                             │
│  Terminal 1:                     Terminal 2:                │
│  ┌─────────────────┐            ┌──────────────────┐       │
│  │ bridge_tour     │            │ send_cmd "ping"  │       │
│  │ (ROS2 Node)     │            │ (ros2 pub)       │       │
│  └────────┬────────┘            └─────────┬────────┘       │
│           │                               │                │
│           │    Subscribe                  │ Publish        │
│           │◄─────────── /to_robot ────────┤                │
│           │                                                 │
│           │ TCP Socket                                      │
│           │ 10.10.0.218:5005                               │
│           ▼                                                 │
└───────────┼─────────────────────────────────────────────────┘
            │
            │ Internet / LAN
            │
┌───────────▼─────────────────────────────────────────────────┐
│ Raspberry Pi (10.10.0.218)                                  │
│                                                             │
│  ┌──────────────────────────────────────────────┐          │
│  │ bridge_pi_debug.py                           │          │
│  │ - TCP Server (port 5005)                     │          │
│  │ - Receive commands via TCP                    │          │
│  │ - Execute on robot (pymycobot)                │          │
│  │ - Send responses back via TCP                 │          │
│  └──────────────────┬───────────────────────────┘          │
│                     │                                       │
│                     │ Serial (/dev/ttyAMA0)                 │
│                     ▼                                       │
│           ┌──────────────────┐                             │
│           │  MyCobot 320 Pi  │                             │
│           │  (6 servos)      │                             │
│           └──────────────────┘                             │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

## 🎯 Points clés
1. **ROS_DOMAIN_ID=10** doit être défini partout
2. **bridge_tour** et **bridge_pi_debug.py** doivent tourner en même temps
3. La communication se fait par **TCP** entre Tour et Pi, pas par ROS2
4. Les topics ROS2 (`/to_robot`, `/from_robot`) sont **locaux** à chaque machine
