# 🐛 Guide Debug - Connexion Tour se ferme immédiatement

## 🔍 Problème identifié

```
[INFO] ✅ Tour connectée : ('10.10.0.115', 49270)
[WARN] ⚠️  Tour déconnectée
```

La Tour se **connecte** puis se **déconnecte immédiatement**. Aucune commande n'est reçue.

---

## 🛠️ Solution : Utiliser la version DEBUG

J'ai créé `bridge_pi_debug.py` avec des logs détaillés pour voir exactement ce qui se passe.

### 📦 Étape 1 : Transférer le fichier vers la Pi

**Depuis le PC Tour** :

```bash
scp /home/genji/ros_jazzy/src/mycobot_R6A/bridge_pi_debug.py \
    er@10.10.0.218:~/colcon_ws/src/mycobot_ros2/mycobot_320/mycobot_320pi/mycobot_gateway/
```

### 🚀 Étape 2 : Lancer la version debug sur la Pi

**Sur la Raspberry Pi** :

```bash
cd ~/colcon_ws/src/mycobot_ros2/mycobot_320/mycobot_320pi/mycobot_gateway
python3 bridge_pi_debug.py
```

**Logs attendus au démarrage** :
```
[INFO] 🔌 Connexion au robot sur /dev/ttyAMA0...
[INFO] ✅ Robot connecté ! Angles: [...]
[INFO] 🌐 Bridge démarré. Port 5005.
[INFO] 🔄 En attente de connexion...
```

### 🧪 Étape 3 : Lancer un test depuis le PC Tour

```bash
cd /home/genji/ros_jazzy/src/mycobot_R6A
source quick_commands.sh
send_cmd "ping"
```

**Logs attendus sur la Pi** (avec bridge_pi_debug.py) :
```
[INFO] ✅ Tour connectée : ('10.10.0.115', xxxxx)
[INFO] 🎧 Thread de réception démarré
[INFO] 📡 En attente de données...
[INFO] 📦 Données reçues : XX bytes
[INFO] 📥 Commande : 'ping'
[INFO] 🔧 Exécution : ping
[INFO] 📤 Réponse envoyée : pong
```

---

## 🎯 Ce que le debug va révéler

### Scénario 1 : Vous voyez "📡 En attente de données..." mais jamais "📦 Données reçues"
**Cause** : Le bridge_tour n'envoie pas les données via TCP  
**Solution** : Problème côté bridge_tour

### Scénario 2 : Vous voyez "📦 Données reçues : 0 bytes"
**Cause** : La connexion TCP se ferme immédiatement  
**Solution** : Vérifier le bridge_tour, peut-être qu'il ferme la connexion

### Scénario 3 : Vous voyez "❌ Erreur recv : ..."
**Cause** : Problème de socket  
**Solution** : L'erreur exacte nous dira quoi

### Scénario 4 : Vous voyez "📥 Commande : '...'" mais pas "🔧 Exécution"
**Cause** : Le code d'exécution ne se lance pas  
**Solution** : Bug dans execute_command()

### Scénario 5 : Tout fonctionne dans les logs mais le robot ne bouge pas
**Cause** : Problème avec pymycobot ou le robot  
**Solution** : Tester pymycobot directement

---

## 🔧 Tests à faire après le debug

### Test 1 : Ping simple

```bash
# PC Tour
source quick_commands.sh
send_cmd "ping"
```

**Résultat attendu sur Pi** :
```
[INFO] 📥 Commande : 'ping'
[INFO] 🔧 Exécution : ping
[INFO] 📤 Réponse : pong
```

### Test 2 : Get angles

```bash
send_cmd "get_angles"
```

**Résultat attendu** :
```
[INFO] 📥 Commande : 'get_angles'
[INFO] 🔧 Exécution : get_angles
[INFO] 📤 Réponse : angles:[0.0, 0.0, ...]
```

### Test 3 : LED

```bash
send_cmd "set_led:255,0,0"
```

**Résultat attendu** :
```
[INFO] 📥 Commande : 'set_led:255,0,0'
[INFO] 🔧 Exécution : set_led:255,0,0
[INFO] 💡 LED : R=255, G=0, B=0
[INFO] 📤 Réponse : led_ok:r=255,g=0,b=0
```

**ET la LED du robot devrait devenir ROUGE** 💡

---

## 📊 Différences avec bridge_pi_mycobot.py

La version debug ajoute :

1. ✅ **Logs détaillés** à chaque étape
2. ✅ **Timeout sur recv()** pour éviter de bloquer
3. ✅ **Try/except avec traceback** pour voir les erreurs exactes
4. ✅ **Logs de démarrage du thread** de réception
5. ✅ **Logs de chaque byte reçu**

---

## 🐛 Si ça marche avec bridge_pi_debug.py

Si les commandes passent avec la version debug, alors le problème vient de `bridge_pi_mycobot.py`.

**Différences possibles** :
- Manque de timeout sur `recv()`
- Thread qui crash silencieusement
- Exception non catchée

**Solution** : Remplacer `bridge_pi_mycobot.py` par `bridge_pi_debug.py` :

```bash
cd ~/colcon_ws/src/mycobot_ros2/mycobot_320/mycobot_320pi/mycobot_gateway
cp bridge_pi_mycobot.py bridge_pi_mycobot_old.py
cp bridge_pi_debug.py bridge_pi_mycobot.py
```

---

## 📝 Informations à collecter

Quand vous testez avec `bridge_pi_debug.py`, notez :

1. **Logs de démarrage** (connexion robot, démarrage serveur)
2. **Logs de connexion Tour** (IP, port)
3. **Logs de réception** (thread démarré ? données reçues ?)
4. **Logs d'exécution** (commande exécutée ? réponse envoyée ?)
5. **Comportement du robot** (LED change ? robot bouge ?)

---

## 🎯 Hypothèse sur le problème actuel

Mon hypothèse : Le `recv()` dans `bridge_pi_mycobot.py` **bloque indéfiniment** ou **retourne immédiatement 0** sans timeout, ce qui fait que le thread se termine.

La version debug corrige ça avec :
```python
self.client_socket.settimeout(1.0)  # Timeout 1 seconde
```

Cela permet de checker `self.running` régulièrement et de ne pas bloquer éternellement.

---

**Prochaine étape** : Transférer et lancer `bridge_pi_debug.py` sur la Pi, puis copier les logs complets ici.
