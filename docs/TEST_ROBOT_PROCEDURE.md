# 🤖 Procédure de Test sur Robot Réel - MyCobot 320 Pi

## ⚠️ SÉCURITÉ AVANT TOUT

### Checklist de sécurité
- [ ] Zone de travail du robot dégagée (au moins 50cm autour)
- [ ] Aucun objet fragile à proximité
- [ ] Arrêt d'urgence accessible
- [ ] Câbles bien fixés (pas de risque d'arrachement)
- [ ] Robot sur une surface stable
- [ ] Vous êtes prêt à couper l'alimentation si nécessaire

### En cas de problème
- **CTRL+C** dans les terminaux pour arrêter les nœuds
- **Bouton d'arrêt d'urgence** du robot si mouvement anormal
- **Couper l'alimentation** en dernier recours

---

## 📋 Phase 1 : Vérification de l'environnement (5 min)

### 1.1 Sur la Raspberry Pi

#### Terminal Pi-1 : Vérifier la connexion au robot
```bash
# Se connecter en SSH à la Pi
ssh er@10.10.0.218

# Vérifier que le robot est connecté
ls /dev/ttyUSB* /dev/ttyAMA*
# Devrait afficher quelque chose comme /dev/ttyUSB0 ou /dev/ttyAMA0

# Tester la communication série (optionnel)
# Si vous avez pymycobot installé :
python3 -c "from pymycobot.mycobot import MyCobot; mc = MyCobot('/dev/ttyUSB0', 115200); print('Angles:', mc.get_angles())"
```

**Résultat attendu** :
- Affichage des angles actuels du robot
- Pas d'erreur de communication

#### Terminal Pi-2 : Lancer le bridge_pi
```bash
cd ~/colcon_ws/src/mycobot_ros2/mycobot_320/mycobot_320pi/mycobot_gateway
python3 bridge_pi.py
```

**Résultat attendu** :
```
[INFO] [bridge_pi]: Bridge Bidirectionnel démarré. Port 5005.
```

### 1.2 Sur le PC Tour

#### Terminal Tour-1 : Diagnostic
```bash
cd /home/genji/ros_jazzy/src/mycobot_R6A
source quick_commands.sh
setup_bridge
./diagnose.sh
```

**Vérifier que tout est ✅ sauf peut-être le port 5005 qui sera ouvert après le lancement du bridge_pi**

#### Terminal Tour-2 : Lancer le bridge_tour
```bash
cd /home/genji/ros_jazzy/src/mycobot_R6A
conda deactivate
source install/setup.bash
ros2 run mycobot_gateway bridge_tour
```

**Résultat attendu** :
```
[INFO] [bridge_tour]: ✅ Connecté à la Pi (10.10.0.218)
```

**Sur la Pi, vous devriez voir** :
```
[INFO] [bridge_pi]: Tour connectée : ('10.10.0.115', xxxxx)
```

---

## 📋 Phase 2 : Test de communication basique (5 min)

### 2.1 Test de ping (Tour → Pi → Tour)

#### Terminal Tour-3 : Monitorer les messages entrants
```bash
cd /home/genji/ros_jazzy/src/mycobot_R6A
conda deactivate
source install/setup.bash
ros2 topic echo /from_robot
```

#### Terminal Tour-4 : Envoyer un message de test
```bash
cd /home/genji/ros_jazzy/src/mycobot_R6A
source quick_commands.sh
send_cmd "ping"
```

**Résultat attendu** :
- **Terminal Tour-2 (bridge_tour)** : `[INFO] 📤 Envoyé vers Pi: ping`
- **Terminal Pi-2 (bridge_pi)** : `[INFO] Commande relayée : ping`
- Si le bridge_pi répond avec un "pong", **Terminal Tour-3** affichera le message

### 2.2 Test de status robot

#### Terminal Tour-4 : Demander les angles
```bash
send_cmd "get_angles"
```

**Résultat attendu** :
- Le bridge_pi doit recevoir la commande
- Si bridge_pi interroge le robot, vous devriez voir une réponse dans Terminal Tour-3

---

## 📋 Phase 3 : Test de mouvement simple (10 min)

### ⚠️ ATTENTION : Le robot va bouger !

### 3.1 Test de mouvement minimal (LED ou servo unique)

**Option A : Test LED (le plus sûr)**
```bash
send_cmd "set_led:255,0,0"  # LED rouge
```

**Option B : Test d'un seul servo (mouvement minimal)**
```bash
# Récupérer d'abord la position actuelle
send_cmd "get_angles"

# Bouger LÉGÈREMENT le servo 1 (joint 1) de quelques degrés
send_cmd "set_angle:1,0,20"  # Joint 1, angle 0°, vitesse 20
```

**Résultat attendu** :
- Mouvement doux et contrôlé
- Pas de mouvement brusque
- Le robot reste stable

### 3.2 Test de retour à la position de repos

```bash
# Position HOME (tous les servos à 0)
send_cmd "go_home:20"  # Vitesse 20 (lent)
```

**Résultat attendu** :
- Mouvement fluide vers la position de repos
- Tous les joints à 0°

### 3.3 Test de séquence simple

```bash
# Séquence : HOME → légère rotation → HOME
send_cmd "sequence_start"
# Attendre 2 secondes
send_cmd "set_angles:0,0,0,0,0,0:30"  # HOME
# Attendre que le mouvement se termine
send_cmd "set_angles:10,0,0,0,0,0:20"  # Légère rotation joint 1
# Attendre
send_cmd "set_angles:0,0,0,0,0,0:20"  # Retour HOME
send_cmd "sequence_end"
```

---

## 📋 Phase 4 : Validation complète (10 min)

### 4.1 Test de latence

Mesurer le temps de réponse du système :

```bash
# Dans un terminal avec watch
watch -n 1 'date +%s.%N && ros2 topic pub /to_robot std_msgs/msg/String "{data: \"status\"}" -1'
```

**Objectif** : Latence < 200ms

### 4.2 Test de charge (messages multiples)

```bash
# Envoyer 10 commandes rapidement
for i in {1..10}; do 
  send_cmd "test_$i"
  sleep 0.5
done
```

**Vérifier** :
- Tous les messages sont reçus
- Pas de perte de messages
- Ordre préservé

### 4.3 Test de stabilité (connexion longue durée)

Laisser tourner le bridge pendant 5 minutes et vérifier :
- Pas de déconnexion
- Pas de fuite mémoire
- Logs propres

---

## 📊 Grille de validation

### Communication
- [ ] Bridge_tour se connecte à bridge_pi
- [ ] Messages Tour → Pi fonctionnent
- [ ] Messages Pi → Tour fonctionnent (si implémenté)
- [ ] Pas de perte de messages
- [ ] Latence acceptable (< 200ms)

### Robot
- [ ] Le robot reçoit les commandes
- [ ] Les mouvements sont fluides
- [ ] Pas de mouvement brusque ou dangereux
- [ ] Position HOME fonctionne
- [ ] Get angles fonctionne (lecture état)

### Stabilité
- [ ] Pas de déconnexion TCP
- [ ] Pas d'erreur après 5 minutes
- [ ] Logs propres et lisibles

---

## 🐛 Dépannage

### Le robot ne bouge pas
1. Vérifier que le port série est correct (`/dev/ttyUSB0` ou `/dev/ttyAMA0`)
2. Vérifier les permissions : `sudo usermod -a -G dialout $USER`
3. Vérifier que pymycobot est installé : `pip3 list | grep pymycobot`
4. Tester en direct sur la Pi : `python3 -c "from pymycobot import MyCobot; mc = MyCobot('/dev/ttyUSB0'); mc.send_angles([0,0,0,0,0,0], 30)"`

### Le bridge_pi ne relaye pas les commandes
1. Vérifier le code de `bridge_pi.py` :
   - Est-ce qu'il parse les commandes ?
   - Est-ce qu'il appelle pymycobot ?
2. Ajouter des logs dans bridge_pi.py pour débugger

### Mouvements erratiques
1. **ARRÊT D'URGENCE**
2. Réduire les vitesses (< 20)
3. Vérifier que les angles sont dans les limites (-170° à +170° selon les joints)
4. Vérifier l'alimentation du robot (voltage suffisant)

### Latence élevée
1. Vérifier la qualité du réseau : `ping 10.10.0.218`
2. Réduire la charge réseau (fermer autres applications)
3. Vérifier qu'aucun processus n'utilise trop de CPU sur la Pi

---

## 📝 Log de test à remplir

### Informations système
- **Date/Heure** : ________________
- **Version bridge** : 0.0.1
- **ROS2 Tour** : Jazzy
- **ROS2 Pi** : Galactic
- **Modèle robot** : MyCobot 320 Pi

### Tests effectués

| Test | Statut | Notes |
|------|--------|-------|
| Connexion bridge | ⬜ ✅ ❌ | |
| Ping/Pong | ⬜ ✅ ❌ | |
| Get angles | ⬜ ✅ ❌ | |
| LED test | ⬜ ✅ ❌ | |
| Mouvement simple | ⬜ ✅ ❌ | |
| Position HOME | ⬜ ✅ ❌ | |
| Séquence | ⬜ ✅ ❌ | |
| Test latence | ⬜ ✅ ❌ | Latence: _____ ms |
| Test charge | ⬜ ✅ ❌ | |
| Test stabilité | ⬜ ✅ ❌ | Durée: _____ min |

### Problèmes rencontrés
```
(Décrire ici)




```

### Améliorations identifiées
```
(Décrire ici)




```

---

## 🎯 Prochaines étapes après validation

Si tous les tests passent :

1. **Documenter** les commandes qui fonctionnent
2. **Créer** un fichier de commandes prédéfinies
3. **Implémenter** des séquences de mouvement complexes
4. **Ajouter** la télémétrie (position, vitesse, etc.)
5. **Créer** une interface de contrôle (CLI ou GUI)

---

**Bonne chance ! 🚀**

*N'oubliez pas : sécurité d'abord. En cas de doute, arrêtez tout.*
