# 🚀 Démarrage Rapide - Tests Robot

## ⚡ En 4 étapes simples

### 1️⃣ Sur la Raspberry Pi

```bash
# SSH vers la Pi
ssh er@10.10.0.218

# Lancer le bridge
cd ~/colcon_ws/src/mycobot_ros2/mycobot_320/mycobot_320pi/mycobot_gateway
python3 bridge_pi.py
```

**Attendez** : `[INFO] Bridge Bidirectionnel démarré. Port 5005.`

---

### 2️⃣ Sur le PC Tour - Terminal 1 (Bridge)

```bash
cd /home/genji/ros_jazzy/src/mycobot_R6A
conda deactivate
source install/setup.bash
ros2 run mycobot_gateway bridge_tour
```

**Attendez** : `[INFO] ✅ Connecté à la Pi (10.10.0.218)`

---

### 3️⃣ Sur le PC Tour - Terminal 2 (Tests interactifs)

```bash
cd /home/genji/ros_jazzy/src/mycobot_R6A
./robot_test_interactive.sh
```

**Menu interactif** avec toutes les commandes de test ! 🎮

---

### 4️⃣ Tester !

Depuis le menu interactif :

1. **D'abord** : Test de communication (option 1)
2. **Ensuite** : Demander les angles (option 2)
3. **Si OK** : Test LED (option 4) — sûr, pas de mouvement
4. **Si OK** : Position HOME (option 5) — premier mouvement
5. **Puis** : Tests de mouvement (options 6-7)

---

## 📋 Checklist avant de commencer

- [ ] Robot alimenté et allumé
- [ ] Câble USB connecté (Pi ↔ Robot)
- [ ] Zone dégagée autour du robot (50cm)
- [ ] Arrêt d'urgence accessible
- [ ] bridge_pi.py lancé sur la Pi
- [ ] bridge_tour lancé sur le PC

---

## 🆘 Commandes rapides utiles

### Vérifier la connexion série sur la Pi
```bash
ls /dev/ttyUSB* /dev/ttyAMA*
```

### Tester pymycobot directement (sur la Pi)
```bash
python3 -c "from pymycobot.mycobot import MyCobot; mc = MyCobot('/dev/ttyUSB0', 115200); print(mc.get_angles())"
```

### Monitorer les messages du robot (PC Tour)
```bash
cd /home/genji/ros_jazzy/src/mycobot_R6A
conda deactivate
source install/setup.bash
ros2 topic echo /from_robot
```

### Envoyer une commande manuelle (PC Tour)
```bash
source quick_commands.sh
send_cmd "votre_commande"
```

---

## ⚠️ Sécurité

### En cas de mouvement anormal
1. **CTRL+C** dans les terminaux
2. **Bouton d'arrêt d'urgence** du robot
3. **Débrancher l'alimentation**

### Limites de sécurité recommandées
- Vitesse max : **30** (pour les tests)
- Angles : vérifier les limites de chaque joint
- Toujours commencer par position HOME

---

## 📊 Tests recommandés dans l'ordre

| # | Test | Risque | Description |
|---|------|--------|-------------|
| 1 | Communication | ⚪ Aucun | Ping/Pong |
| 2 | Get angles | ⚪ Aucun | Lecture position |
| 3 | Status | ⚪ Aucun | État robot |
| 4 | LED | 🟢 Faible | Test visuel sans mouvement |
| 5 | Home | 🟡 Moyen | Premier mouvement |
| 6 | Mouvement simple | 🟡 Moyen | Un seul joint |
| 7 | Séquence | 🟠 Élevé | Mouvements multiples |

---

## 📖 Documentation complète

Pour la procédure détaillée, voir : **[TEST_ROBOT_PROCEDURE.md](TEST_ROBOT_PROCEDURE.md)**

---

**Bonne chance ! 🤖✨**
