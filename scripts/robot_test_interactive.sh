#!/bin/bash
# Script de test interactif pour MyCobot
# Usage: ./robot_test_interactive.sh

set -e

# Couleurs
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Setup environnement
cd /home/genji/ros_jazzy/src/mycobot_R6A
conda deactivate 2>/dev/null || true
source install/setup.bash 2>/dev/null

echo -e "${CYAN}"
echo "╔════════════════════════════════════════════════════════════╗"
echo "║      🤖 Tests Interactifs MyCobot 320 Pi                  ║"
echo "╚════════════════════════════════════════════════════════════╝"
echo -e "${NC}"

# Fonction pour envoyer une commande
send_command() {
    local cmd="$1"
    echo -e "${BLUE}📤 Envoi : ${YELLOW}$cmd${NC}"
    ros2 topic pub /to_robot std_msgs/msg/String "{data: '$cmd'}" -1
    sleep 0.5
}

# Fonction pour demander confirmation
confirm() {
    local prompt="$1"
    echo -e "${YELLOW}$prompt (y/n) ${NC}"
    read -r response
    case "$response" in
        [yY][eE][sS]|[yY]) 
            return 0
            ;;
        *)
            return 1
            ;;
    esac
}

# Vérifier que le bridge tourne
echo -e "${CYAN}🔍 Vérification de l'environnement...${NC}"
if ! ros2 topic list 2>/dev/null | grep -q "/to_robot"; then
    echo -e "${RED}❌ Le bridge_tour ne semble pas lancé !${NC}"
    echo ""
    echo "Lancer d'abord dans un autre terminal :"
    echo "  ros2 run mycobot_gateway bridge_tour"
    echo ""
    exit 1
fi

echo -e "${GREEN}✅ Topics trouvés !${NC}"
echo ""

# Menu principal
while true; do
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo -e "${CYAN}Menu de test${NC}"
    echo -e "${CYAN}━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━${NC}"
    echo ""
    echo "  ${GREEN}[1]${NC} Test de communication (ping)"
    echo "  ${GREEN}[2]${NC} Demander les angles du robot (get_angles)"
    echo "  ${GREEN}[3]${NC} Demander le statut (status)"
    echo ""
    echo "  ${YELLOW}[4]${NC} Test LED (si disponible)"
    echo "  ${YELLOW}[5]${NC} Position HOME (⚠️  robot va bouger)"
    echo "  ${YELLOW}[6]${NC} Mouvement simple - Joint 1 (⚠️  robot va bouger)"
    echo "  ${YELLOW}[7]${NC} Test séquence complète (⚠️  robot va bouger)"
    echo ""
    echo "  ${BLUE}[8]${NC} Commande personnalisée"
    echo "  ${BLUE}[9]${NC} Monitorer les réponses du robot"
    echo ""
    echo "  ${RED}[0]${NC} Quitter"
    echo ""
    echo -e -n "${CYAN}Votre choix : ${NC}"
    read -r choice

    case $choice in
        1)
            echo ""
            echo -e "${CYAN}━━━ Test de communication ━━━${NC}"
            send_command "ping"
            echo -e "${GREEN}✅ Commande envoyée. Vérifiez les logs du bridge.${NC}"
            echo ""
            ;;
        
        2)
            echo ""
            echo -e "${CYAN}━━━ Demande des angles ━━━${NC}"
            send_command "get_angles"
            echo -e "${GREEN}✅ Commande envoyée. Vérifiez les logs ou le topic /from_robot${NC}"
            echo ""
            ;;
        
        3)
            echo ""
            echo -e "${CYAN}━━━ Demande de statut ━━━${NC}"
            send_command "status"
            echo -e "${GREEN}✅ Commande envoyée.${NC}"
            echo ""
            ;;
        
        4)
            echo ""
            echo -e "${CYAN}━━━ Test LED ━━━${NC}"
            echo "Couleurs disponibles :"
            echo "  1. Rouge"
            echo "  2. Vert"
            echo "  3. Bleu"
            echo "  4. Éteindre"
            echo -n "Choix : "
            read -r led_choice
            case $led_choice in
                1) send_command "set_led:255,0,0" ;;
                2) send_command "set_led:0,255,0" ;;
                3) send_command "set_led:0,0,255" ;;
                4) send_command "set_led:0,0,0" ;;
                *) echo -e "${RED}Choix invalide${NC}" ;;
            esac
            echo ""
            ;;
        
        5)
            echo ""
            echo -e "${YELLOW}━━━ Position HOME ━━━${NC}"
            echo -e "${RED}⚠️  ATTENTION : Le robot va bouger vers sa position de repos !${NC}"
            if confirm "Continuer ?"; then
                send_command "go_home:20"
                echo -e "${GREEN}✅ Commande envoyée. Le robot devrait aller en position HOME.${NC}"
            else
                echo -e "${YELLOW}Annulé.${NC}"
            fi
            echo ""
            ;;
        
        6)
            echo ""
            echo -e "${YELLOW}━━━ Mouvement simple - Joint 1 ━━━${NC}"
            echo -e "${RED}⚠️  ATTENTION : Le robot va bouger !${NC}"
            echo ""
            echo "Le joint 1 va faire un petit mouvement."
            if confirm "Continuer ?"; then
                echo "Mouvement à +10°..."
                send_command "set_angle:1,10,20"
                sleep 2
                echo "Retour à 0°..."
                send_command "set_angle:1,0,20"
                echo -e "${GREEN}✅ Séquence terminée.${NC}"
            else
                echo -e "${YELLOW}Annulé.${NC}"
            fi
            echo ""
            ;;
        
        7)
            echo ""
            echo -e "${YELLOW}━━━ Test séquence complète ━━━${NC}"
            echo -e "${RED}⚠️  ATTENTION : Le robot va exécuter une séquence de mouvements !${NC}"
            echo ""
            echo "Séquence : HOME → Rotation J1 → HOME"
            if confirm "Continuer ?"; then
                echo "1/3 - Position HOME..."
                send_command "set_angles:0,0,0,0,0,0:30"
                sleep 3
                
                echo "2/3 - Rotation joint 1..."
                send_command "set_angles:15,0,0,0,0,0:20"
                sleep 3
                
                echo "3/3 - Retour HOME..."
                send_command "set_angles:0,0,0,0,0,0:20"
                sleep 3
                
                echo -e "${GREEN}✅ Séquence terminée !${NC}"
            else
                echo -e "${YELLOW}Annulé.${NC}"
            fi
            echo ""
            ;;
        
        8)
            echo ""
            echo -e "${CYAN}━━━ Commande personnalisée ━━━${NC}"
            echo -n "Entrez votre commande : "
            read -r custom_cmd
            if [ -n "$custom_cmd" ]; then
                send_command "$custom_cmd"
                echo -e "${GREEN}✅ Commande envoyée.${NC}"
            else
                echo -e "${RED}Commande vide, annulé.${NC}"
            fi
            echo ""
            ;;
        
        9)
            echo ""
            echo -e "${CYAN}━━━ Monitoring des réponses ━━━${NC}"
            echo "Lancement de ros2 topic echo /from_robot..."
            echo "Appuyez sur CTRL+C pour arrêter."
            echo ""
            ros2 topic echo /from_robot
            echo ""
            ;;
        
        0)
            echo ""
            echo -e "${CYAN}👋 Au revoir !${NC}"
            exit 0
            ;;
        
        *)
            echo ""
            echo -e "${RED}❌ Choix invalide !${NC}"
            echo ""
            ;;
    esac
    
    # Petite pause avant de réafficher le menu
    sleep 1
done
