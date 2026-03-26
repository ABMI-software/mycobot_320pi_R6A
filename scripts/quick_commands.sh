#!/bin/bash
# Commandes rapides pour le bridge MyCobot
# Source ce fichier : source quick_commands.sh

# Alias pour sourcer l'environnement correctement
alias setup_bridge='conda deactivate 2>/dev/null; cd /home/genji/ros_jazzy/src/mycobot_R6A && source install/setup.bash'

# Alias pour lancer le bridge
alias start_bridge='setup_bridge && ros2 run mycobot_gateway bridge_tour'

# Alias pour publier un test
alias test_send='setup_bridge && ros2 topic pub /to_robot std_msgs/msg/String "{data: \"test_moteur_1\"}" -1'

# Alias pour écouter les réponses
alias listen_robot='setup_bridge && ros2 topic echo /from_robot'

# Alias pour voir les topics
alias list_topics='setup_bridge && ros2 topic list'

# Fonction pour rebuild
rebuild_bridge() {
    cd /home/genji/ros_jazzy/src/mycobot_R6A
    colcon build --packages-select mycobot_gateway --symlink-install
    source install/setup.bash
    echo "✅ Bridge rebuild et environnement sourcé"
}

# Fonction pour envoyer une commande personnalisée
send_cmd() {
    if [ -z "$1" ]; then
        echo "Usage: send_cmd 'votre_commande'"
        return 1
    fi
    setup_bridge
    export ROS_DOMAIN_ID=10
    ros2 topic pub /to_robot std_msgs/msg/String "{data: '$1'}" -1
}

echo "✅ Commandes rapides chargées :"
echo "  setup_bridge    - Configure l'environnement ROS2"
echo "  start_bridge    - Lance le bridge_tour"
echo "  test_send       - Envoie 'test_moteur_1'"
echo "  listen_robot    - Écoute les réponses du robot"
echo "  list_topics     - Liste les topics disponibles"
echo "  rebuild_bridge  - Rebuild le package"
echo "  send_cmd 'cmd'  - Envoie une commande personnalisée"
