#!/bin/bash
# Script de diagnostic du bridge MyCobot
# Usage: ./diagnose.sh

set +e  # Continue même en cas d'erreur

echo "╔════════════════════════════════════════════════════════════╗"
echo "║  🔍 Diagnostic du Bridge MyCobot Gateway                  ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo ""

# Couleurs
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

check_ok() {
    echo -e "${GREEN}✅ $1${NC}"
}

check_warn() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

check_fail() {
    echo -e "${RED}❌ $1${NC}"
}

# 1. Vérifier l'environnement Python
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "1️⃣  Environnement Python"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

PYTHON_PATH=$(which python3)
PYTHON_VERSION=$(python3 --version 2>&1 | cut -d' ' -f2)

echo "Python path: $PYTHON_PATH"
echo "Python version: $PYTHON_VERSION"

if [[ "$PYTHON_PATH" == *"conda"* ]] || [[ "$PYTHON_PATH" == *"miniconda"* ]]; then
    check_fail "Python pointe vers conda ! Exécutez 'conda deactivate'"
    CONDA_ISSUE=1
elif [[ "$PYTHON_VERSION" == 3.12.* ]]; then
    check_ok "Python 3.12 détecté (compatible ROS2 Jazzy)"
else
    check_warn "Python $PYTHON_VERSION (ROS2 Jazzy recommande 3.12.x)"
fi

if [[ -n "$CONDA_DEFAULT_ENV" ]]; then
    check_fail "Conda activé : $CONDA_DEFAULT_ENV"
    CONDA_ISSUE=1
else
    check_ok "Conda non activé"
fi

# 2. Vérifier ROS2
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "2️⃣  Environnement ROS2"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

if [[ -n "$ROS_DISTRO" ]]; then
    check_ok "ROS_DISTRO: $ROS_DISTRO"
else
    check_fail "ROS_DISTRO non défini. Exécutez 'source install/setup.bash'"
    exit 1
fi

if [[ -n "$ROS_VERSION" ]]; then
    check_ok "ROS_VERSION: $ROS_VERSION"
else
    check_warn "ROS_VERSION non défini"
fi

if [[ -n "$AMENT_PREFIX_PATH" ]]; then
    check_ok "AMENT_PREFIX_PATH défini"
else
    check_fail "AMENT_PREFIX_PATH non défini"
fi

if [[ -n "$ROS_AUTOMATIC_DISCOVERY_RANGE" ]]; then
    check_warn "ROS_AUTOMATIC_DISCOVERY_RANGE=$ROS_AUTOMATIC_DISCOVERY_RANGE (peut causer des problèmes)"
else
    check_ok "ROS_AUTOMATIC_DISCOVERY_RANGE non défini (recommandé)"
fi

# 3. Vérifier le package mycobot_gateway
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "3️⃣  Package mycobot_gateway"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

if ros2 pkg prefix mycobot_gateway &>/dev/null; then
    PKG_PREFIX=$(ros2 pkg prefix mycobot_gateway)
    check_ok "Package trouvé : $PKG_PREFIX"
else
    check_fail "Package mycobot_gateway introuvable"
    echo "   → Exécutez : colcon build --packages-select mycobot_gateway"
    exit 1
fi

EXECUTABLES=$(ros2 pkg executables mycobot_gateway 2>/dev/null)
if [[ "$EXECUTABLES" == *"bridge_tour"* ]]; then
    check_ok "Exécutable bridge_tour trouvé"
else
    check_fail "Exécutable bridge_tour introuvable"
    echo "   → Vérifiez que le build s'est bien passé"
    exit 1
fi

# Vérifier l'existence physique du script
BRIDGE_SCRIPT="$PKG_PREFIX/lib/mycobot_gateway/bridge_tour"
if [[ -f "$BRIDGE_SCRIPT" ]]; then
    check_ok "Script installé : $BRIDGE_SCRIPT"
    if [[ -x "$BRIDGE_SCRIPT" ]]; then
        check_ok "Script exécutable"
    else
        check_warn "Script non exécutable"
    fi
else
    check_fail "Script non trouvé : $BRIDGE_SCRIPT"
fi

# 4. Vérifier la connectivité réseau
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "4️⃣  Connectivité réseau (Pi)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

PI_IP="10.10.0.218"
PI_PORT="5005"

echo "Test de ping vers $PI_IP..."
if ping -c 1 -W 2 "$PI_IP" &>/dev/null; then
    check_ok "Pi accessible ($PI_IP)"
else
    check_fail "Pi inaccessible ($PI_IP)"
    echo "   → Vérifiez que la Pi est allumée et sur le réseau"
    NETWORK_ISSUE=1
fi

echo "Test du port TCP $PI_PORT..."
if command -v nc &>/dev/null; then
    if timeout 2 nc -zv "$PI_IP" "$PI_PORT" &>/dev/null; then
        check_ok "Port $PI_PORT ouvert sur la Pi"
    else
        check_fail "Port $PI_PORT fermé ou inaccessible"
        echo "   → Vérifiez que bridge_pi.py tourne sur la Pi"
        NETWORK_ISSUE=1
    fi
else
    check_warn "Commande 'nc' non disponible (impossible de tester le port)"
fi

# 5. Vérifier les dépendances Python
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "5️⃣  Dépendances Python ROS2"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

if [[ -z "$CONDA_ISSUE" ]]; then
    if python3 -c "import rclpy" 2>/dev/null; then
        check_ok "Module rclpy importable"
    else
        check_fail "Module rclpy non importable"
        echo "   → Vérifiez l'installation de ROS2"
    fi

    if python3 -c "from std_msgs.msg import String" 2>/dev/null; then
        check_ok "Module std_msgs importable"
    else
        check_fail "Module std_msgs non importable"
    fi

    if python3 -c "import socket" 2>/dev/null; then
        check_ok "Module socket disponible"
    else
        check_fail "Module socket non disponible"
    fi
else
    check_warn "Tests Python ignorés (conda activé)"
fi

# 6. Vérifier les topics ROS2 (si bridge actif)
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "6️⃣  Topics ROS2 (si bridge actif)"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

TOPICS=$(ros2 topic list 2>/dev/null)

if echo "$TOPICS" | grep -q "/to_robot"; then
    check_ok "Topic /to_robot trouvé"
else
    check_warn "Topic /to_robot absent (bridge non lancé ?)"
fi

if echo "$TOPICS" | grep -q "/from_robot"; then
    check_ok "Topic /from_robot trouvé"
else
    check_warn "Topic /from_robot absent (bridge non lancé ?)"
fi

# Résumé final
echo ""
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"
echo "📊 Résumé"
echo "━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━"

if [[ -n "$CONDA_ISSUE" ]]; then
    echo ""
    check_fail "CONDA ACTIVÉ — Désactivez conda avant d'utiliser ROS2"
    echo "   Exécutez : conda deactivate"
    exit 1
fi

if [[ -n "$NETWORK_ISSUE" ]]; then
    echo ""
    check_warn "PROBLÈMES RÉSEAU — Vérifiez la connectivité avec la Pi"
    echo "   - Pi allumée ?"
    echo "   - bridge_pi.py lancé sur la Pi ?"
    echo "   - IP correcte : $PI_IP ?"
    echo "   - Port ouvert : $PI_PORT ?"
fi

echo ""
check_ok "Diagnostic terminé"
echo ""
echo "Pour lancer le bridge :"
echo "  ros2 run mycobot_gateway bridge_tour"
echo ""
echo "Pour envoyer un test :"
echo "  ros2 topic pub /to_robot std_msgs/msg/String \"{data: 'test'}\" -1"
echo ""
