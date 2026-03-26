# 🔧 Guide de Mise à Jour du bridge_pi.py

## 🎯 Problème identifié

**Symptôme** : Les commandes sont bien relayées par bridge_pi, mais le robot ne bouge pas.

**Cause** : Le bridge_pi actuel **reçoit** les commandes mais ne les **exécute pas** sur le robot.

**Solution** : Ajouter la logique d'exécution des commandes avec pymycobot.

---

## 📝 Ce qui manque dans votre bridge_pi.py actuel

1. ❌ Import de `pymycobot`
2. ❌ Initialisation de la connexion au robot
3. ❌ Fonction `execute_command()` pour interpréter et exécuter les commandes
4. ❌ Appels aux fonctions pymycobot (send_angles, set_color, etc.)

---

## 🛠️ Solution : Code à ajouter

### Fichier exemple complet

J'ai créé un fichier **`bridge_pi_example.py`** dans votre workspace qui contient :

- ✅ Connexion au robot MyCobot via pymycobot
- ✅ Fonction `execute_command()` qui interprète les commandes
- ✅ Support des commandes :
  - `ping` → Réponse "pong"
  - `get_angles` → Retourne les angles actuels
  - `status` → Retourne le statut du robot
  - `set_led:R,G,B` → Change la couleur de la LED
  - `go_home:speed` → Position HOME
  - `set_angle:joint,angle,speed` → Bouge un joint
  - `set_angles:j1,j2,j3,j4,j5,j6:speed` → Bouge tous les joints

### Localisation

```
/home/genji/ros_jazzy/src/mycobot_R6A/bridge_pi_example.py
```

---

## 🚀 Étapes pour intégrer dans votre bridge_pi.py

### Option 1 : Remplacer complètement (recommandé si vous débutez)

```bash
# Sur la Raspberry Pi
cd ~/colcon_ws/src/mycobot_ros2/mycobot_320/mycobot_320pi/mycobot_gateway

# Backup de l'ancien
cp bridge_pi.py bridge_pi_old.py

# Copier le nouveau (vous devrez transférer bridge_pi_example.py vers la Pi)
# Méthode 1 : scp depuis le PC Tour
scp genji@10.10.0.115:/home/genji/ros_jazzy/src/mycobot_R6A/bridge_pi_example.py bridge_pi.py

# OU Méthode 2 : copier le contenu manuellement
nano bridge_pi.py
# Puis coller le contenu de bridge_pi_example.py
```

### Option 2 : Ajouter seulement les parties manquantes

Si vous voulez garder votre code actuel et juste ajouter l'exécution :

#### 1. Ajouter l'import pymycobot

Au début du fichier :

```python
try:
    from pymycobot.mycobot import MyCobot
    PYMYCOBOT_AVAILABLE = True
except ImportError:
    print("⚠️  ATTENTION : pymycobot n'est pas installé !")
    PYMYCOBOT_AVAILABLE = False
```

#### 2. Dans `__init__()`, initialiser le robot

```python
# Configuration robot
self.robot_port = '/dev/ttyUSB0'  # OU /dev/ttyAMA0
self.robot_baudrate = 115200
self.robot = None

# Initialiser la connexion
self.init_robot()
```

#### 3. Ajouter la fonction `init_robot()`

```python
def init_robot(self):
    """Initialise la connexion avec le MyCobot"""
    if not PYMYCOBOT_AVAILABLE:
        self.get_logger().error("❌ pymycobot non disponible")
        return
        
    try:
        self.get_logger().info(f"🔌 Connexion au robot sur {self.robot_port}...")
        self.robot = MyCobot(self.robot_port, self.robot_baudrate)
        
        # Test
        angles = self.robot.get_angles()
        if angles:
            self.get_logger().info(f"✅ Robot connecté ! Angles: {angles}")
            
    except Exception as e:
        self.get_logger().error(f"❌ Erreur connexion robot : {e}")
        self.robot = None
```

#### 4. Dans la fonction qui reçoit les commandes, ajouter

```python
# Après avoir loggé "Commande relayée"
self.execute_command(command)
```

#### 5. Ajouter la fonction `execute_command()`

Copiez la fonction complète depuis `bridge_pi_example.py` (lignes ~120-240)

---

## 🔍 Vérifications avant de lancer

### Sur la Raspberry Pi

#### 1. Vérifier le port série

```bash
ls /dev/ttyUSB* /dev/ttyAMA*
```

**Résultat attendu** : `/dev/ttyUSB0` ou `/dev/ttyAMA0`

Si vide, vérifier que le robot est bien branché en USB.

#### 2. Vérifier les permissions

```bash
sudo usermod -a -G dialout $USER
# Puis se déconnecter/reconnecter ou :
newgrp dialout
```

#### 3. Tester pymycobot directement

```bash
python3 -c "from pymycobot.mycobot import MyCobot; mc = MyCobot('/dev/ttyUSB0', 115200); print('Angles:', mc.get_angles())"
```

**Résultat attendu** : Affichage des angles actuels

**Si erreur** :
- Vérifier l'installation : `pip3 list | grep pymycobot`
- Installer si nécessaire : `pip3 install pymycobot`

---

## 🧪 Test après modification

### 1. Lancer le nouveau bridge_pi

```bash
cd ~/colcon_ws/src/mycobot_ros2/mycobot_320/mycobot_320pi/mycobot_gateway
python3 bridge_pi.py
```

**Vous devriez voir** :
```
[INFO] 🔌 Connexion au robot sur /dev/ttyUSB0...
[INFO] ✅ Robot connecté ! Angles: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
[INFO] 🌐 Bridge Bidirectionnel démarré. Port 5005.
```

### 2. Depuis le PC Tour, relancer les tests

```bash
cd /home/genji/ros_jazzy/src/mycobot_R6A
./robot_test_interactive.sh
```

### 3. Tester dans l'ordre

1. **[1] ping** → Vous devriez recevoir "pong" sur `/from_robot`
2. **[2] get_angles** → Vous devriez recevoir les angles
3. **[4] LED** → La LED devrait changer de couleur ! 💡
4. **[5] HOME** → Le robot devrait bouger ! 🤖
5. **[6] Mouvement simple** → Le joint 1 devrait bouger !

---

## 📊 Résultats attendus

### Terminal bridge_pi (Pi)

```
[INFO] 📥 Commande reçue : ping
[INFO] 🔧 Exécution : ping
[INFO] 📤 Réponse envoyée : pong

[INFO] 📥 Commande reçue : set_led:255,0,0
[INFO] 🔧 Exécution : set_led:255,0,0
[INFO] 💡 LED : R=255, G=0, B=0
[INFO] 📤 Réponse envoyée : led_ok:r=255,g=0,b=0

[INFO] 📥 Commande reçue : go_home:20
[INFO] 🔧 Exécution : go_home:20
[INFO] 🏠 HOME (vitesse 20)
[INFO] 📤 Réponse envoyée : home_ok:speed=20
```

### Terminal bridge_tour (PC)

```
[INFO] 📤 Envoyé vers Pi: ping
[INFO] 📥 Reçu de Pi: pong

[INFO] 📤 Envoyé vers Pi: set_led:255,0,0
[INFO] 📥 Reçu de Pi: led_ok:r=255,g=0,b=0

[INFO] 📤 Envoyé vers Pi: go_home:20
[INFO] 📥 Reçu de Pi: home_ok:speed=20
```

### Sur le robot

- 💡 **LED change de couleur**
- 🤖 **Le robot bouge** vers HOME
- 🔧 **Les joints se déplacent** selon les commandes

---

## 🐛 Dépannage

### Problème : "pymycobot non disponible"

```bash
# Sur la Pi
pip3 install pymycobot
```

### Problème : "Permission denied /dev/ttyUSB0"

```bash
sudo usermod -a -G dialout $USER
sudo chmod 666 /dev/ttyUSB0  # Temporaire
# Puis se reconnecter
```

### Problème : Le robot ne bouge toujours pas

1. Vérifier que `self.robot` n'est pas `None` (logs au démarrage)
2. Vérifier que `execute_command()` est bien appelé
3. Tester pymycobot en direct (commande ci-dessus)
4. Vérifier que le robot est alimenté
5. Vérifier que le câble USB est bien connecté

### Problème : Le robot bouge de façon erratique

1. Réduire les vitesses (max 20-30 pour les tests)
2. Vérifier que les angles sont dans les limites (-170° à +170°)
3. Vérifier l'alimentation du robot (voltage suffisant)

---

## 📝 Commandes supportées (référence)

| Commande | Format | Exemple | Action |
|----------|--------|---------|--------|
| ping | `ping` | `ping` | Réponse "pong" |
| get_angles | `get_angles` | `get_angles` | Retourne les angles actuels |
| status | `status` | `status` | Retourne le statut |
| set_led | `set_led:R,G,B` | `set_led:255,0,0` | LED rouge |
| go_home | `go_home:speed` | `go_home:20` | Position HOME vitesse 20 |
| set_angle | `set_angle:joint,angle,speed` | `set_angle:1,45,20` | Joint 1 → 45° |
| set_angles | `set_angles:j1,j2,j3,j4,j5,j6:speed` | `set_angles:0,0,0,0,0,0:30` | Tous joints → HOME |

---

## 🎯 Prochaines étapes après succès

Une fois que le robot bouge :

1. ✅ Documenter les limites observées (vitesse max, angles max)
2. ✅ Ajouter des commandes plus complexes (gripper, coordonnées, etc.)
3. ✅ Implémenter des séquences pré-programmées
4. ✅ Ajouter la télémétrie (position en continu)
5. ✅ Créer une interface de contrôle avancée

---

**Fichiers de référence** :
- `bridge_pi_example.py` — Exemple complet fonctionnel
- Ce guide — Instructions d'intégration

Bonne chance ! 🚀
