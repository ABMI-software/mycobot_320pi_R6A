#!/bin/bash
# Script de test du bridge MyCobot
# Usage: ./test_bridge.sh

set -e

echo "🚀 Test du Bridge MyCobot Gateway"
echo "=================================="
echo ""

# Désactiver conda si actif
if [[ -n "$CONDA_DEFAULT_ENV" ]]; then
    echo "⚠️  Désactivation de conda ($CONDA_DEFAULT_ENV)..."
    conda deactivate 2>/dev/null || true
fi

# Sourcer l'environnement ROS2
echo "📦 Chargement de l'environnement ROS2..."
cd /home/genji/ros_jazzy/src/mycobot_R6A
source install/setup.bash

echo ""
echo "✅ Environnement prêt !"
echo "ROS_DISTRO: $ROS_DISTRO"
echo "ROS_VERSION: $ROS_VERSION"
echo ""

# Vérifier que le package est bien installé
echo "🔍 Vérification du package..."
if ros2 pkg prefix mycobot_gateway &>/dev/null; then
    echo "✅ Package mycobot_gateway trouvé"
else
    echo "❌ Package mycobot_gateway introuvable !"
    exit 1
fi

# Lister les exécutables
echo ""
echo "📋 Exécutables disponibles:"
ros2 pkg executables mycobot_gateway

echo ""
echo "=================================="
echo "Pour lancer le bridge, exécutez:"
echo "  ros2 run mycobot_gateway bridge_tour"
echo ""
echo "Pour publier un message de test:"
echo "  ros2 topic pub /to_robot std_msgs/msg/String \"{data: 'test_moteur_1'}\" -1"
echo ""
echo "Pour écouter les réponses:"
echo "  ros2 topic echo /from_robot"
echo "=================================="
