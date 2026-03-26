#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Bridge Pi - Serveur TCP Simple et Robuste
Version corrigée pour maintenir une connexion stable.

PROBLÈME RÉSOLU: Le serveur acceptait plusieurs connexions en parallèle,
ce qui causait des déconnexions rapides.

SOLUTION: Maintenir UNE SEULE connexion à la fois.
"""

import socket
import json
import time
import threading

# Essayer d'importer ROS2 (optionnel)
try:
    import rclpy
    from rclpy.node import Node
    HAS_ROS = True
except ImportError:
    HAS_ROS = False
    print("⚠️ ROS2 non disponible - Mode standalone")

# Essayer d'importer pymycobot
try:
    from pymycobot.mycobot import MyCobot
    HAS_ROBOT = True
except ImportError:
    HAS_ROBOT = False
    print("⚠️ pymycobot non disponible - Mode simulation")


class BridgePiSimple:
    """
    Serveur TCP simple qui maintient UNE SEULE connexion.
    """
    
    def __init__(self, host='0.0.0.0', port=5005, robot_port='/dev/ttyAMA0', baud=115200):
        self.host = host
        self.port = port
        self.running = False
        
        # Robot
        self.mc = None
        if HAS_ROBOT:
            print(f"🔌 Connexion au robot sur {robot_port}...")
            try:
                self.mc = MyCobot(robot_port, baud)
                time.sleep(1)
                angles = self.mc.get_angles()
                print(f"✅ Robot connecté ! Angles: {angles}")
            except Exception as e:
                print(f"❌ Erreur robot: {e}")
                self.mc = None
        
        # Positions prédéfinies
        self.home_angles = [0, 8, -127, 40, 0, 0]
        self.zero_angles = [0, 0, 0, 0, 0, 0]
        
        # Socket
        self.server_socket = None
        self.client_socket = None
        
    def log(self, msg):
        """Afficher un message avec timestamp"""
        timestamp = time.strftime("%H:%M:%S")
        print(f"[{timestamp}] {msg}")

    def start(self):
        """Démarrer le serveur"""
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((self.host, self.port))
        self.server_socket.listen(1)  # Une seule connexion en attente
        self.running = True
        
        self.log(f"🌐 Serveur démarré sur {self.host}:{self.port}")
        
        while self.running:
            self.log("⏳ En attente de connexion Tour...")
            
            try:
                # Accepter UNE connexion
                self.client_socket, addr = self.server_socket.accept()
                self.log(f"✅ Tour connectée: {addr}")
                
                # Gérer cette connexion jusqu'à déconnexion
                self.handle_connection()
                
            except Exception as e:
                if self.running:
                    self.log(f"❌ Erreur: {e}")
            
            # Fermer proprement le socket client
            if self.client_socket:
                try:
                    self.client_socket.close()
                except:
                    pass
                self.client_socket = None
            
            self.log("🔄 Connexion fermée, prêt pour nouvelle connexion")

    def handle_connection(self):
        """Gérer la connexion avec la Tour - BLOQUANT"""
        buffer = ""
        self.client_socket.settimeout(None)  # Pas de timeout
        
        while self.running:
            try:
                # Recevoir des données (bloquant)
                data = self.client_socket.recv(1024)
                
                if not data:
                    self.log("⚠️ Tour déconnectée (recv vide)")
                    break
                
                # Décoder et traiter
                buffer += data.decode('utf-8')
                
                # Traiter les messages complets (terminés par \n)
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    line = line.strip()
                    if line:
                        response = self.process_command(line)
                        self.send_response(response)
                        
            except socket.timeout:
                continue
            except ConnectionResetError:
                self.log("⚠️ Connexion réinitialisée par la Tour")
                break
            except Exception as e:
                self.log(f"❌ Erreur de réception: {e}")
                break

    def send_response(self, message):
        """Envoyer une réponse à la Tour"""
        if self.client_socket:
            try:
                self.client_socket.sendall((message + '\n').encode('utf-8'))
                self.log(f"📤 Envoyé: {message[:50]}...")
            except Exception as e:
                self.log(f"❌ Erreur d'envoi: {e}")

    def process_command(self, raw_cmd):
        """Traiter une commande reçue"""
        self.log(f"📥 Reçu: {raw_cmd}")
        
        # Essayer JSON d'abord
        try:
            cmd = json.loads(raw_cmd)
            return self.execute_json(cmd)
        except json.JSONDecodeError:
            return self.execute_raw(raw_cmd)

    def execute_json(self, cmd):
        """Exécuter une commande JSON"""
        action = cmd.get('action', '').lower()
        
        if not self.mc:
            return "ERROR: Robot non connecté"
        
        try:
            if action == 'send_angles':
                angles = cmd.get('angles', [])
                speed = cmd.get('speed', 30)
                self.mc.send_angles(angles, speed)
                return f"OK: angles envoyés {angles}"
                
            elif action == 'send_coords':
                coords = cmd.get('coords', [])
                speed = cmd.get('speed', 40)
                mode = cmd.get('mode', 1)
                self.mc.send_coords(coords, speed, mode)
                return f"OK: coords envoyées {coords[:3]}"
                
            elif action == 'go_home':
                self.mc.send_angles(self.home_angles, 30)
                return "OK: retour home"
                
            elif action == 'go_zero':
                self.mc.send_angles(self.zero_angles, 30)
                return "OK: retour zero"
                
            elif action == 'gripper_open':
                self.mc.set_gripper_state(0, 50)
                return "OK: pince ouverte"
                
            elif action == 'gripper_close':
                self.mc.set_gripper_state(1, 50)
                return "OK: pince fermée"
                
            elif action == 'power_on':
                self.mc.power_on()
                return "OK: moteurs allumés"
                
            elif action == 'power_off':
                self.mc.release_all_servos()
                return "OK: moteurs relâchés"
                
            elif action == 'get_angles':
                angles = self.mc.get_angles()
                return f"ANGLES: {angles}"
                
            elif action == 'get_coords':
                coords = self.mc.get_coords()
                return f"COORDS: {coords}"
                
            elif action == 'emergency_stop':
                self.mc.release_all_servos()
                return "🚨 ARRÊT D'URGENCE"
                
            else:
                return f"ERROR: Action inconnue '{action}'"
                
        except Exception as e:
            return f"ERROR: {str(e)}"

    def execute_raw(self, cmd):
        """Exécuter une commande texte simple"""
        cmd_lower = cmd.lower().strip()
        
        if cmd_lower == 'ping':
            return "PONG"
        
        if cmd_lower == 'status':
            if self.mc:
                try:
                    angles = self.mc.get_angles()
                    coords = self.mc.get_coords()
                    return f"STATUS: angles={angles}, coords={coords}"
                except:
                    return "STATUS: robot connecté mais erreur lecture"
            return "STATUS: robot non connecté"
        
        if not self.mc:
            return f"ECHO: {cmd} (robot non connecté)"
        
        try:
            if cmd_lower == 'home':
                self.mc.send_angles(self.home_angles, 30)
                return "OK: retour home"
                
            elif cmd_lower == 'zero':
                self.mc.send_angles(self.zero_angles, 30)
                return "OK: retour zero"
                
            elif cmd_lower == 'stop':
                self.mc.release_all_servos()
                return "OK: arrêt"
                
            else:
                return f"ECHO: {cmd}"
                
        except Exception as e:
            return f"ERROR: {str(e)}"

    def stop(self):
        """Arrêter le serveur"""
        self.running = False
        if self.client_socket:
            try:
                self.client_socket.close()
            except:
                pass
        if self.server_socket:
            try:
                self.server_socket.close()
            except:
                pass
        self.log("🛑 Serveur arrêté")


def main():
    import argparse
    
    parser = argparse.ArgumentParser(description='Bridge Pi Simple')
    parser.add_argument('--host', default='0.0.0.0', help='Adresse d\'écoute')
    parser.add_argument('--port', type=int, default=5005, help='Port')
    parser.add_argument('--robot-port', default='/dev/ttyAMA0', help='Port série robot')
    parser.add_argument('--baud', type=int, default=115200, help='Baud rate')
    
    args = parser.parse_args()
    
    print("="*50)
    print("🤖 Bridge Pi - Version Simple et Robuste")
    print("="*50)
    print(f"📡 Réseau: {args.host}:{args.port}")
    print(f"🔌 Robot: {args.robot_port}")
    print("="*50)
    print("\nCommandes disponibles:")
    print("  ping, status, home, zero, stop")
    print("  JSON: {\"action\": \"go_home\"}")
    print("="*50 + "\n")
    
    bridge = BridgePiSimple(
        host=args.host,
        port=args.port,
        robot_port=args.robot_port,
        baud=args.baud
    )
    
    try:
        bridge.start()
    except KeyboardInterrupt:
        print("\n⏹️ Interruption utilisateur")
    finally:
        bridge.stop()


if __name__ == '__main__':
    main()
