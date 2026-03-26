#!/bin/bash
# Script pour vérifier si le bridge_pi tourne sur le Raspberry Pi

PI_IP="10.10.0.218"
PI_USER="er"

echo "🔍 Vérification du bridge_pi sur le Raspberry Pi..."
echo ""

echo "1️⃣  Processus Python sur le Pi :"
ssh ${PI_USER}@${PI_IP} "ps aux | grep -E 'bridge_pi|python3.*mycobot' | grep -v grep"

echo ""
echo "2️⃣  Processus ROS2 sur le Pi :"
ssh ${PI_USER}@${PI_IP} "ps aux | grep ros2 | grep -v grep"

echo ""
echo "3️⃣  Port 5005 ouvert ?"
nc -zv ${PI_IP} 5005

echo ""
echo "4️⃣  Connexion au robot (/dev/ttyAMA0) :"
ssh ${PI_USER}@${PI_IP} "ls -l /dev/ttyAMA0 2>/dev/null || echo '❌ /dev/ttyAMA0 non trouvé'"

echo ""
echo "📝 Si aucun processus bridge_pi ne tourne, lancez-le sur le Pi :"
echo "   ssh ${PI_USER}@${PI_IP}"
echo "   cd ~/colcon_ws/src/mycobot_ros2/mycobot_320/mycobot_320pi/mycobot_gateway"
echo "   python3 bridge_pi_debug.py"
