#!/bin/bash
# Script de diagnostic complet des tests robot
# Vérifie ce qui fonctionne et ce qui ne fonctionne pas

set -e

echo "═══════════════════════════════════════════════════════════"
echo "🔍 DIAGNOSTIC COMPLET - Tests Robot MyCobot"
echo "═══════════════════════════════════════════════════════════"
echo ""

cd /home/genji/ros_jazzy/src/mycobot_R6A
source install/setup.bash 2>/dev/null

echo "1️⃣  Vérification bridge_tour..."
if ps aux | grep -q "[b]ridge_tour"; then
    echo "   ✅ bridge_tour tourne"
else
    echo "   ❌ bridge_tour ne tourne PAS"
    echo "   → Lancez: ros2 run mycobot_gateway bridge_tour"
    exit 1
fi

echo ""
echo "2️⃣  Vérification topics ROS2..."
TOPICS=$(ros2 topic list 2>/dev/null)
if echo "$TOPICS" | grep -q "/to_robot"; then
    echo "   ✅ /to_robot existe"
else
    echo "   ❌ /to_robot n'existe pas"
fi

if echo "$TOPICS" | grep -q "/from_robot"; then
    echo "   ✅ /from_robot existe"
else
    echo "   ❌ /from_robot n'existe pas"
fi

echo ""
echo "3️⃣  Test de connectivité Pi..."
if ping -c 1 -W 2 10.10.0.218 &>/dev/null; then
    echo "   ✅ Pi accessible (10.10.0.218)"
else
    echo "   ❌ Pi non accessible"
    exit 1
fi

if nc -zv 10.10.0.218 5005 &>/dev/null 2>&1; then
    echo "   ✅ Port 5005 ouvert (bridge_pi tourne)"
else
    echo "   ⚠️  Port 5005 fermé (bridge_pi pas lancé ?)"
fi

echo ""
echo "4️⃣  Vérification subscribers sur /to_robot..."
SUBS=$(ros2 topic info /to_robot -v 2>/dev/null | grep -A 10 "Subscription count" || true)
echo "$SUBS" | head -15

echo ""
echo "5️⃣  Test d'envoi de message..."
echo "   Envoi d'un ping de test..."
timeout 3 ros2 topic pub /to_robot std_msgs/msg/String "{data: 'diagnostic_ping'}" -1 2>&1 | grep -E "(publishing|Waiting)" | head -5

echo ""
echo "═══════════════════════════════════════════════════════════"
echo "📊 RÉSUMÉ"
echo "═══════════════════════════════════════════════════════════"
echo ""
echo "Si vous voyez 'Waiting for at least 1 matching subscription' :"
echo "  → Le bridge_tour ne s'est PAS abonné au topic /to_robot"
echo "  → Problème possible : QoS mismatch ou environnement ROS2"
echo ""
echo "Vérifications à faire sur la Pi :"
echo "  1. Le bridge_pi affiche-t-il les commandes reçues ?"
echo "  2. Le bridge_pi affiche-t-il les exécutions ?"
echo "  3. Y a-t-il des erreurs dans les logs du bridge_pi ?"
echo ""
echo "Commandes utiles :"
echo "  Monitorer les messages : ros2 topic echo /from_robot"
echo "  Voir les infos détaillées : ros2 topic info /to_robot -v"
echo ""
