#!/usr/bin/env python3#!/usr/bin/env python3#!/usr/bin/env python3#!/usr/bin/env python3

"""

Bridge Pi - Version DEBUG avec support JSON et commandes simples"""

Pour contrôler le robot MyCobot depuis la Tour.

Bridge Pi - Version DEBUG avec support JSON et commandes simples""""""

DEPLOYMENT: Copier ce fichier sur le Raspberry Pi et le lancer avec :

    source /opt/ros/galactic/setup.bashPour contrôler le robot MyCobot depuis la Tour.

    python3 bridge_pi_debug.py

Bridge Pi - Version DEBUG avec logs détaillésBridge Pi - Version DEBUG avec logs détaillés

Supporte deux formats de commandes:

1. Simple: "ping", "go_home:20", "set_led:255,0,0"DEPLOYMENT: Copier ce fichier sur le Raspberry Pi et le lancer avec :

2. JSON: {"action": "go_home"}, {"action": "send_angles", "angles": [0,0,0,0,0,0], "speed": 20}

    source /opt/ros/galactic/setup.bashPour diagnostiquer et contrôler le robot MyCobot depuis la Tour.Pour diagnostiquer pourquoi la connexion se ferme immédiatement

IP Tour: 10.10.0.115

IP Pi: 10.10.0.218    python3 bridge_pi_debug.py

Port: 5005

""""""



import socketSupporte deux formats de commandes:

import rclpy

from rclpy.node import Node1. Simple: "ping", "go_home:20", "set_led:255,0,0"DEPLOYMENT: Copier ce fichier sur le Raspberry Pi et le lancer avec :

from std_msgs.msg import String

import threading2. JSON: {"action": "go_home"}, {"action": "send_angles", "angles": [0,0,0,0,0,0], "speed": 20}

import sys

import time"""    source /opt/ros/galactic/setup.bashimport socket

import json



# Import pymycobot

try:import socket    python3 bridge_pi_debug.pyimport rclpy

    from pymycobot.mycobot import MyCobot

    PYMYCOBOT_AVAILABLE = Trueimport rclpy

except ImportError:

    print("⚠️  ATTENTION : pymycobot n'est pas installé !")from rclpy.node import Nodefrom rclpy.node import Node

    print("   Installez-le avec : pip3 install pymycobot")

    PYMYCOBOT_AVAILABLE = Falsefrom std_msgs.msg import String



import threadingIP Tour: 10.10.0.115 (ou autre)from std_msgs.msg import String

class BridgePiDebug(Node):

    def __init__(self):import sys

        super().__init__('bridge_pi')

        import timeIP Pi: 10.10.0.218import threading

        # Publishers et Subscribers ROS2

        self.publisher_to_robot = self.create_publisher(String, '/to_robot', 10)import json

        self.subscription_from_robot = self.create_subscription(

            String, '/from_robot', self.from_robot_callback, 10Port: 5005import sys

        )

        # Import pymycobot

        # Configuration TCP

        self.port = 5005try:"""import time

        self.server_socket = None

        self.client_socket = None    from pymycobot.mycobot import MyCobot

        self.client_address = None

        self.running = True    PYMYCOBOT_AVAILABLE = True

        

        # Configuration robotexcept ImportError:

        self.robot_port = '/dev/ttyAMA0'

        self.robot_baudrate = 115200    print("⚠️  ATTENTION : pymycobot n'est pas installé !")import socket# Import pymycobot

        self.robot = None

            PYMYCOBOT_AVAILABLE = False

        # Positions prédéfinies

        self.home_angles = [0, 8, -127, 40, 0, 0]import rclpytry:

        self.zero_angles = [0, 0, 0, 0, 0, 0]

        

        # Initialiser la connexion au robot

        self.init_robot()class BridgePiDebug(Node):from rclpy.node import Node    from pymycobot.mycobot import MyCobot

        

        # Démarrer le serveur TCP    def __init__(self):

        self.start_tcp_server()

                super().__init__('bridge_pi')from std_msgs.msg import String    PYMYCOBOT_AVAILABLE = True

    def init_robot(self):

        """Initialise la connexion avec le MyCobot"""        

        if not PYMYCOBOT_AVAILABLE:

            self.get_logger().error("❌ pymycobot non disponible, mode simulation")        # Publishers et Subscribers ROS2import threadingexcept ImportError:

            return

                    self.publisher_to_robot = self.create_publisher(String, '/to_robot', 10)

        try:

            self.get_logger().info(f"🔌 Connexion au robot sur {self.robot_port}...")        self.subscription_from_robot = self.create_subscription(import sys    print("⚠️  ATTENTION : pymycobot n'est pas installé !")

            self.robot = MyCobot(self.robot_port, self.robot_baudrate)

            time.sleep(0.5)            String, '/from_robot', self.from_robot_callback, 10

            

            # Test de connexion        )import time    PYMYCOBOT_AVAILABLE = False

            angles = self.robot.get_angles()

            self.get_logger().info(f"✅ Robot connecté ! Angles: {angles}")        

                

        except Exception as e:        # Configuration TCP

            self.get_logger().error(f"❌ Erreur connexion robot : {e}")

            self.robot = None        self.port = 5005

    

    def start_tcp_server(self):        self.server_socket = None# Import pymycobotclass BridgePiDebug(Node):

        """Démarre le serveur TCP"""

        try:        self.client_socket = None

            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)        self.client_address = Nonetry:    def __init__(self):

            self.server_socket.bind(('0.0.0.0', self.port))

            self.server_socket.listen(1)        self.running = True

            self.get_logger().info(f"🌐 Bridge Pi démarré sur port {self.port}")

            self.get_logger().info(f"🔄 En attente de connexion de la Tour...")            from pymycobot.mycobot import MyCobot        super().__init__('bridge_pi')

            

            # Thread pour accepter les connexions        # Configuration robot

            accept_thread = threading.Thread(target=self.accept_connections, daemon=True)

            accept_thread.start()        self.robot_port = '/dev/ttyAMA0'    PYMYCOBOT_AVAILABLE = True        

            

        except Exception as e:        self.robot_baudrate = 115200

            self.get_logger().error(f"❌ Erreur serveur TCP : {e}")

            sys.exit(1)        self.robot = Noneexcept ImportError:        # Publishers et Subscribers ROS2

    

    def accept_connections(self):        

        """Accepte les connexions TCP entrantes"""

        while self.running:        # Positions prédéfinies    print("⚠️  ATTENTION : pymycobot n'est pas installé !")        self.publisher_to_robot = self.create_publisher(String, '/to_robot', 10)

            try:

                self.client_socket, self.client_address = self.server_socket.accept()        self.home_angles = [0, 8, -127, 40, 0, 0]

                self.get_logger().info(f"✅ Tour connectée : {self.client_address}")

                        self.zero_angles = [0, 0, 0, 0, 0, 0]    PYMYCOBOT_AVAILABLE = False        self.subscription_from_robot = self.create_subscription(

                # Thread pour recevoir les données

                recv_thread = threading.Thread(target=self.receive_from_tour, daemon=True)        

                recv_thread.start()

                        # Initialiser la connexion au robot            String, '/from_robot', self.from_robot_callback, 10

            except Exception as e:

                if self.running:        self.init_robot()

                    self.get_logger().error(f"❌ Erreur accept : {e}")

            class BridgePiDebug(Node):        )

    def receive_from_tour(self):

        """Reçoit les commandes depuis la Tour via TCP"""        # Démarrer le serveur TCP

        self.get_logger().info("🎧 Réception active depuis la Tour")

                self.start_tcp_server()    def __init__(self):        

        try:

            while self.running and self.client_socket:        

                self.client_socket.settimeout(1.0)

                    def init_robot(self):        super().__init__('bridge_pi')        # Configuration TCP

                try:

                    data = self.client_socket.recv(1024)        """Initialise la connexion avec le MyCobot"""

                    

                    if data:        if not PYMYCOBOT_AVAILABLE:                self.port = 5005

                        command = data.decode('utf-8').strip()

                                    self.get_logger().error("❌ pymycobot non disponible, mode simulation")

                        if command:

                            self.get_logger().info(f"📥 Commande reçue : '{command}'")            return        # Publishers et Subscribers ROS2        self.server_socket = None

                            

                            # Publier sur ROS2            

                            msg = String()

                            msg.data = command        try:        self.publisher_to_robot = self.create_publisher(String, '/to_robot', 10)        self.client_socket = None

                            self.publisher_to_robot.publish(msg)

                                        self.get_logger().info(f"🔌 Connexion au robot sur {self.robot_port}...")

                            # EXÉCUTER la commande

                            self.execute_command(command)            self.robot = MyCobot(self.robot_port, self.robot_baudrate)        self.subscription_from_robot = self.create_subscription(        self.client_address = None

                    else:

                        self.get_logger().warn("⚠️  Connexion fermée par la Tour")            time.sleep(0.5)

                        break

                                                String, '/from_robot', self.from_robot_callback, 10        self.running = True

                except socket.timeout:

                    continue            # Test de connexion

                except Exception as e:

                    self.get_logger().error(f"❌ Erreur recv : {e}")            angles = self.robot.get_angles()        )        

                    break

                                self.get_logger().info(f"✅ Robot connecté ! Angles: {angles}")

        except Exception as e:

            self.get_logger().error(f"❌ Erreur receive_from_tour : {e}")                                # Configuration robot

        finally:

            self.get_logger().warn("⚠️  Tour déconnectée")        except Exception as e:

            if self.client_socket:

                try:            self.get_logger().error(f"❌ Erreur connexion robot : {e}")        # Configuration TCP        self.robot_port = '/dev/ttyAMA0'  # Changez en /dev/ttyUSB0 si nécessaire

                    self.client_socket.close()

                except:            self.robot = None

                    pass

                self.client_socket = None            self.port = 5005        self.robot_baudrate = 115200

    

    def execute_command(self, command):    def start_tcp_server(self):

        """Exécute la commande - supporte JSON et format simple"""

        self.get_logger().info(f"🔧 Exécution : {command}")        """Démarre le serveur TCP"""        self.server_socket = None        self.robot = None

        

        try:        try:

            # Essayer de parser en JSON

            if command.startswith('{'):            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)        self.client_socket = None        

                try:

                    cmd_json = json.loads(command)            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

                    self.execute_json_command(cmd_json)

                    return            self.server_socket.bind(('0.0.0.0', self.port))        self.client_address = None        # Initialiser la connexion au robot

                except json.JSONDecodeError:

                    self.get_logger().warn("⚠️  JSON invalide, traitement comme commande simple")            self.server_socket.listen(1)

            

            # Commande simple (format texte)            self.get_logger().info(f"🌐 Bridge Bidirectionnel démarré. Port {self.port}.")        self.running = True        self.init_robot()

            self.execute_simple_command(command)

                        

        except Exception as e:

            self.get_logger().error(f"❌ Erreur exécution : {e}")            # Thread pour accepter les connexions                

            self.send_response(f"error:execution:{e}")

                accept_thread = threading.Thread(target=self.accept_connections, daemon=True)

    def execute_json_command(self, cmd):

        """Exécute une commande JSON"""            accept_thread.start()        # Configuration robot        # Démarrer le serveur TCP

        action = cmd.get('action', '')

        self.get_logger().info(f"🎯 Action JSON : {action}")            

        

        try:        except Exception as e:        self.robot_port = '/dev/ttyAMA0'  # Changez en /dev/ttyUSB0 si nécessaire        self.start_tcp_server()

            # ==================== MOUVEMENTS ====================

                        self.get_logger().error(f"❌ Erreur serveur TCP : {e}")

            if action == 'go_home':

                if self.robot:            sys.exit(1)        self.robot_baudrate = 115200        

                    speed = cmd.get('speed', 20)

                    self.robot.send_angles(self.home_angles, speed)    

                    self.get_logger().info(f"🏠 HOME → {self.home_angles} (vitesse {speed})")

                    self.send_response(f"home_ok:angles={self.home_angles}")    def accept_connections(self):        self.robot = None    def init_robot(self):

                else:

                    self.send_response("error:robot_not_connected")        """Accepte les connexions TCP entrantes"""

                return

                    while self.running:                """Initialise la connexion avec le MyCobot"""

            if action == 'go_zero':

                if self.robot:            try:

                    speed = cmd.get('speed', 20)

                    self.robot.send_angles(self.zero_angles, speed)                self.get_logger().info("🔄 En attente de connexion...")        # Initialiser la connexion au robot        if not PYMYCOBOT_AVAILABLE:

                    self.get_logger().info(f"0️⃣ ZERO → {self.zero_angles} (vitesse {speed})")

                    self.send_response(f"zero_ok:angles={self.zero_angles}")                self.client_socket, self.client_address = self.server_socket.accept()

                else:

                    self.send_response("error:robot_not_connected")                self.get_logger().info(f"✅ Tour connectée : {self.client_address}")        self.init_robot()            self.get_logger().error("❌ pymycobot non disponible, mode simulation")

                return

                            

            if action == 'send_angles':

                if self.robot:                # Thread pour recevoir les données                    return

                    angles = cmd.get('angles', self.zero_angles)

                    speed = cmd.get('speed', 20)                recv_thread = threading.Thread(target=self.receive_from_tour, daemon=True)

                    self.robot.send_angles(angles, speed)

                    self.get_logger().info(f"🔧 Angles → {angles} (vitesse {speed})")                recv_thread.start()        # Démarrer le serveur TCP            

                    self.send_response(f"angles_sent:{angles}")

                else:                

                    self.send_response("error:robot_not_connected")

                return            except Exception as e:        self.start_tcp_server()        try:

            

            if action == 'send_coords':                if self.running:

                if self.robot:

                    coords = cmd.get('coords', [0, 0, 0, 0, 0, 0])                    self.get_logger().error(f"❌ Erreur accept : {e}")                    self.get_logger().info(f"🔌 Connexion au robot sur {self.robot_port}...")

                    speed = cmd.get('speed', 40)

                    mode = cmd.get('mode', 1)    

                    self.robot.send_coords(coords, speed, mode)

                    self.get_logger().info(f"📍 Coords → {coords} (vitesse {speed}, mode {mode})")    def receive_from_tour(self):    def init_robot(self):            self.robot = MyCobot(self.robot_port, self.robot_baudrate)

                    self.send_response(f"coords_sent:{coords}")

                else:        """Reçoit les commandes depuis la Tour via TCP"""

                    self.send_response("error:robot_not_connected")

                return        self.get_logger().info("🎧 Thread de réception démarré")        """Initialise la connexion avec le MyCobot"""            

            

            if action == 'send_radians':        

                if self.robot:

                    radians = cmd.get('radians', [0, 0, 0, 0, 0, 0])        try:        if not PYMYCOBOT_AVAILABLE:            # Test de connexion

                    speed = cmd.get('speed', 20)

                    self.robot.send_radians(radians, speed)            while self.running and self.client_socket:

                    self.get_logger().info(f"📐 Radians → {radians} (vitesse {speed})")

                    self.send_response(f"radians_sent:{radians}")                self.client_socket.settimeout(1.0)            self.get_logger().error("❌ pymycobot non disponible, mode simulation")            angles = self.robot.get_angles()

                else:

                    self.send_response("error:robot_not_connected")                

                return

                            try:            return            self.get_logger().info(f"✅ Robot connecté ! Angles: {angles}")

            # ==================== LECTURE ÉTAT ====================

                                data = self.client_socket.recv(1024)

            if action == 'get_angles':

                if self.robot:                                                

                    angles = self.robot.get_angles()

                    self.get_logger().info(f"📐 Angles actuels : {angles}")                    if data:

                    self.send_response(f"angles:{angles}")

                else:                        self.get_logger().info(f"📦 Données reçues : {len(data)} bytes")        try:        except Exception as e:

                    self.send_response("error:robot_not_connected")

                return                        command = data.decode('utf-8').strip()

            

            if action == 'get_coords':                                    self.get_logger().info(f"🔌 Connexion au robot sur {self.robot_port}...")            self.get_logger().error(f"❌ Erreur connexion robot : {e}")

                if self.robot:

                    coords = self.robot.get_coords()                        if command:

                    self.get_logger().info(f"📍 Coords actuels : {coords}")

                    self.send_response(f"coords:{coords}")                            self.get_logger().info(f"📥 Commande : '{command}'")            self.robot = MyCobot(self.robot_port, self.robot_baudrate)            self.robot = None

                else:

                    self.send_response("error:robot_not_connected")                            

                return

                                        # Publier sur ROS2                

            # ==================== GRIPPER ====================

                                        msg = String()

            if action == 'gripper_open':

                if self.robot:                            msg.data = command            # Test de connexion    def start_tcp_server(self):

                    self.robot.set_gripper_state(0, 50)

                    self.get_logger().info("✋ Gripper OUVERT")                            self.publisher_to_robot.publish(msg)

                    self.send_response("gripper:open")

                else:                                        angles = self.robot.get_angles()        """Démarre le serveur TCP"""

                    self.send_response("error:robot_not_connected")

                return                            # EXÉCUTER la commande

            

            if action == 'gripper_close':                            self.execute_command(command)            self.get_logger().info(f"✅ Robot connecté ! Angles: {angles}")        try:

                if self.robot:

                    self.robot.set_gripper_state(1, 50)                        else:

                    self.get_logger().info("✊ Gripper FERMÉ")

                    self.send_response("gripper:closed")                            self.get_logger().warn("⚠️  Commande vide")                            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)

                else:

                    self.send_response("error:robot_not_connected")                    else:

                return

                                    self.get_logger().warn("⚠️  Données vides (connexion fermée par Tour)")        except Exception as e:            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)

            # ==================== CONTRÔLE ALIMENTATION ====================

                                    break

            if action == 'emergency_stop':

                if self.robot:                                    self.get_logger().error(f"❌ Erreur connexion robot : {e}")            self.server_socket.bind(('0.0.0.0', self.port))

                    self.robot.release_all_servos()

                    self.get_logger().warn("🚨 ARRÊT D'URGENCE !")                except socket.timeout:

                    self.send_response("emergency:stopped")

                else:                    continue            self.robot = None            self.server_socket.listen(1)

                    self.send_response("error:robot_not_connected")

                return                except Exception as e:

            

            if action == 'power_on':                    self.get_logger().error(f"❌ Erreur recv : {e}")                self.get_logger().info(f"🌐 Bridge Bidirectionnel démarré. Port {self.port}.")

                if self.robot:

                    self.robot.power_on()                    break

                    self.get_logger().info("⚡ Power ON")

                    self.send_response("power:on")                        def start_tcp_server(self):            

                else:

                    self.send_response("error:robot_not_connected")        except Exception as e:

                return

                        self.get_logger().error(f"❌ Erreur receive_from_tour : {e}")        """Démarre le serveur TCP"""            # Thread pour accepter les connexions

            if action == 'power_off':

                if self.robot:            import traceback

                    self.robot.power_off()

                    self.get_logger().info("🔌 Power OFF")            self.get_logger().error(traceback.format_exc())        try:            accept_thread = threading.Thread(target=self.accept_connections, daemon=True)

                    self.send_response("power:off")

                else:        finally:

                    self.send_response("error:robot_not_connected")

                return            self.get_logger().warn("⚠️  Tour déconnectée")            self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)            accept_thread.start()

            

            # ==================== VISION (future) ====================            if self.client_socket:

            

            if action in ['follow_on', 'follow_off', 'get_marker']:                try:            self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)            

                self.get_logger().warn(f"⚠️  Action vision non implémentée : {action}")

                self.send_response(f"not_implemented:{action}")                    self.client_socket.close()

                return

                            except:            self.server_socket.bind(('0.0.0.0', self.port))        except Exception as e:

            # Action inconnue

            self.get_logger().warn(f"⚠️  Action JSON inconnue : {action}")                    pass

            self.send_response(f"error:unknown_action:{action}")

                            self.client_socket = None            self.server_socket.listen(1)            self.get_logger().error(f"❌ Erreur serveur TCP : {e}")

        except Exception as e:

            self.get_logger().error(f"❌ Erreur action JSON : {e}")    

            self.send_response(f"error:json:{e}")

        def execute_command(self, command):            self.get_logger().info(f"🌐 Bridge Bidirectionnel démarré. Port {self.port}.")            sys.exit(1)

    def execute_simple_command(self, command):

        """Exécute une commande au format simple (texte)"""        """Exécute la commande - supporte JSON et format simple"""

        try:

            # Ping/Pong        self.get_logger().info(f"🔧 Exécution : {command}")                

            if command == "ping":

                self.send_response("pong")        

                return

                    try:            # Thread pour accepter les connexions    def accept_connections(self):

            # Status

            if command == "status":            # Essayer de parser en JSON

                if self.robot:

                    angles = self.robot.get_angles()            if command.startswith('{'):            accept_thread = threading.Thread(target=self.accept_connections, daemon=True)        """Accepte les connexions TCP entrantes"""

                    self.send_response(f"status:ok,angles:{angles}")

                else:                try:

                    self.send_response("status:robot_not_connected")

                return                    cmd_json = json.loads(command)            accept_thread.start()        while self.running:

            

            # Get angles                    self.execute_json_command(cmd_json)

            if command == "get_angles":

                if self.robot:                    return                        try:

                    angles = self.robot.get_angles()

                    self.send_response(f"angles:{angles}")                except json.JSONDecodeError:

                else:

                    self.send_response("error:robot_not_connected")                    self.get_logger().warn("⚠️  JSON invalide, traitement comme commande simple")        except Exception as e:                self.get_logger().info("🔄 En attente de connexion...")

                return

                        

            # Set LED : set_led:R,G,B

            if command.startswith("set_led:"):            # Commande simple (format texte)            self.get_logger().error(f"❌ Erreur serveur TCP : {e}")                self.client_socket, self.client_address = self.server_socket.accept()

                if self.robot:

                    try:            self.execute_simple_command(command)

                        rgb = command.split(":")[1].split(",")

                        r, g, b = int(rgb[0]), int(rgb[1]), int(rgb[2])                        sys.exit(1)                self.get_logger().info(f"✅ Tour connectée : {self.client_address}")

                        self.robot.set_color(r, g, b)

                        self.get_logger().info(f"💡 LED : R={r}, G={g}, B={b}")        except Exception as e:

                        self.send_response(f"led_ok:{r},{g},{b}")

                    except Exception as e:            self.get_logger().error(f"❌ Erreur exécution : {e}")                    

                        self.send_response(f"error:led:{e}")

                else:            import traceback

                    self.send_response("error:robot_not_connected")

                return            self.get_logger().error(traceback.format_exc())    def accept_connections(self):                # Thread pour recevoir les données

            

            # Go HOME : go_home ou go_home:speed            self.send_response(f"error:execution:{e}")

            if command.startswith("go_home"):

                if self.robot:            """Accepte les connexions TCP entrantes"""                recv_thread = threading.Thread(target=self.receive_from_tour, daemon=True)

                    try:

                        parts = command.split(":")    def execute_json_command(self, cmd):

                        speed = int(parts[1]) if len(parts) > 1 else 20

                        self.robot.send_angles(self.home_angles, speed)        """Exécute une commande JSON"""        while self.running:                recv_thread.start()

                        self.get_logger().info(f"🏠 HOME (vitesse {speed})")

                        self.send_response(f"home_ok:{speed}")        action = cmd.get('action', '')

                    except Exception as e:

                        self.send_response(f"error:home:{e}")        self.get_logger().info(f"🔧 Action JSON : {action}")            try:                

                else:

                    self.send_response("error:robot_not_connected")        

                return

                    try:                self.get_logger().info("🔄 En attente de connexion...")            except Exception as e:

            # Go ZERO : go_zero ou go_zero:speed

            if command.startswith("go_zero"):            # Go Home

                if self.robot:

                    try:            if action == 'go_home':                self.client_socket, self.client_address = self.server_socket.accept()                if self.running:

                        parts = command.split(":")

                        speed = int(parts[1]) if len(parts) > 1 else 20                if self.robot:

                        self.robot.send_angles(self.zero_angles, speed)

                        self.get_logger().info(f"0️⃣ ZERO (vitesse {speed})")                    speed = cmd.get('speed', 20)                self.get_logger().info(f"✅ Tour connectée : {self.client_address}")                    self.get_logger().error(f"❌ Erreur accept : {e}")

                        self.send_response(f"zero_ok:{speed}")

                    except Exception as e:                    self.robot.send_angles(self.home_angles, speed)

                        self.send_response(f"error:zero:{e}")

                else:                    self.get_logger().info(f"🏠 HOME (vitesse {speed})")                    

                    self.send_response("error:robot_not_connected")

                return                    self.send_response(f"home_ok:speed={speed}")

            

            # Set angle (un seul joint) : set_angle:joint,angle,speed                else:                # Thread pour recevoir les données    def receive_from_tour(self):

            if command.startswith("set_angle:"):

                if self.robot:                    self.send_response("error:robot_not_connected")

                    try:

                        params = command.split(":")[1].split(",")                return                recv_thread = threading.Thread(target=self.receive_from_tour, daemon=True)        """Reçoit les commandes depuis la Tour via TCP"""

                        joint = int(params[0])

                        angle = float(params[1])            

                        speed = int(params[2]) if len(params) > 2 else 20

                                    # Go Zero                recv_thread.start()        self.get_logger().info("🎧 Thread de réception démarré")

                        self.robot.send_angle(joint, angle, speed)

                        self.get_logger().info(f"🔧 Joint {joint} → {angle}° (vitesse {speed})")            if action == 'go_zero':

                        self.send_response(f"angle_ok:{joint},{angle},{speed}")

                    except Exception as e:                if self.robot:                        

                        self.send_response(f"error:set_angle:{e}")

                else:                    speed = cmd.get('speed', 20)

                    self.send_response("error:robot_not_connected")

                return                    self.robot.send_angles(self.zero_angles, speed)            except Exception as e:        try:

            

            # Set angles (tous les joints) : set_angles:a1,a2,a3,a4,a5,a6:speed                    self.get_logger().info(f"0️⃣ ZERO (vitesse {speed})")

            if command.startswith("set_angles:"):

                if self.robot:                    self.send_response(f"zero_ok:speed={speed}")                if self.running:            while self.running and self.client_socket:

                    try:

                        parts = command.split(":")                else:

                        angles = [float(a) for a in parts[1].split(",")]

                        speed = int(parts[2]) if len(parts) > 2 else 20                    self.send_response("error:robot_not_connected")                    self.get_logger().error(f"❌ Erreur accept : {e}")                self.get_logger().debug("📡 En attente de données...")

                        

                        self.robot.send_angles(angles, speed)                return

                        self.get_logger().info(f"🔧 Angles → {angles} (vitesse {speed})")

                        self.send_response(f"angles_ok:{angles}")                                

                    except Exception as e:

                        self.send_response(f"error:set_angles:{e}")            # Send Angles

                else:

                    self.send_response("error:robot_not_connected")            if action == 'send_angles':    def receive_from_tour(self):                # Recv avec timeout pour pouvoir check self.running

                return

                            if self.robot:

            # Commande inconnue

            self.get_logger().warn(f"⚠️  Commande simple inconnue : {command}")                    angles = cmd.get('angles', self.zero_angles)        """Reçoit les commandes depuis la Tour via TCP"""                self.client_socket.settimeout(1.0)

            self.send_response(f"error:unknown:{command}")

                                speed = cmd.get('speed', 20)

        except Exception as e:

            self.get_logger().error(f"❌ Erreur commande simple : {e}")                    self.robot.send_angles(angles, speed)        self.get_logger().info("🎧 Thread de réception démarré")                

            self.send_response(f"error:simple:{e}")

                        self.get_logger().info(f"🔧 Angles → {angles} (vitesse {speed})")

    def send_response(self, response):

        """Envoie une réponse à la Tour via TCP"""                    self.send_response(f"angles_ok:{angles}")                        try:

        try:

            if self.client_socket:                else:

                self.client_socket.sendall((response + '\n').encode('utf-8'))

                self.get_logger().info(f"📤 Réponse : {response}")                    self.send_response("error:robot_not_connected")        try:                    data = self.client_socket.recv(1024)

            else:

                self.get_logger().warn("⚠️  Pas de client connecté pour envoyer la réponse")                return

        except Exception as e:

            self.get_logger().error(f"❌ Erreur envoi réponse : {e}")                        while self.running and self.client_socket:                    

    

    def from_robot_callback(self, msg):            # Send Coords

        """Callback pour les messages ROS2 /from_robot"""

        if self.client_socket:            if action == 'send_coords':                self.get_logger().debug("📡 En attente de données...")                    if data:

            try:

                self.client_socket.sendall((msg.data + '\n').encode('utf-8'))                if self.robot:

                self.get_logger().info(f"📤 ROS→TCP : {msg.data}")

            except Exception as e:                    coords = cmd.get('coords', [0, 0, 0, 0, 0, 0])                                        self.get_logger().info(f"📦 Données reçues : {len(data)} bytes")

                self.get_logger().error(f"❌ Erreur envoi ROS→TCP : {e}")

                        speed = cmd.get('speed', 40)

    def shutdown(self):

        """Arrêt propre"""                    mode = cmd.get('mode', 1)                # Recv avec timeout pour pouvoir check self.running                        command = data.decode('utf-8').strip()

        self.get_logger().info("🛑 Arrêt du bridge Pi...")

        self.running = False                    self.robot.send_coords(coords, speed, mode)

        if self.client_socket:

            try:                    self.get_logger().info(f"📍 Coords → {coords} (vitesse {speed})")                self.client_socket.settimeout(1.0)                        

                self.client_socket.close()

            except:                    self.send_response(f"coords_ok:{coords}")

                pass

        if self.server_socket:                else:                                        if command:

            try:

                self.server_socket.close()                    self.send_response("error:robot_not_connected")

            except:

                pass                return                try:                            self.get_logger().info(f"📥 Commande : '{command}'")



            

def main(args=None):

    print("="*60)            # Get Angles                    data = self.client_socket.recv(1024)                            

    print("🤖 MyCobot Bridge Pi - Version DEBUG")

    print("="*60)            if action == 'get_angles':

    print(f"Port TCP    : 5005")

    print(f"Port robot  : /dev/ttyAMA0")                if self.robot:                                                # Publier sur ROS2

    print(f"Baudrate    : 115200")

    print("="*60)                    angles = self.robot.get_angles()

    

    rclpy.init(args=args)                    self.get_logger().info(f"📐 Angles : {angles}")                    if data:                            msg = String()

    node = BridgePiDebug()

                        self.send_response(f"angles:{angles}")

    try:

        rclpy.spin(node)                else:                        self.get_logger().info(f"📦 Données reçues : {len(data)} bytes")                            msg.data = command

    except KeyboardInterrupt:

        print("\n⚠️  Interruption clavier")                    self.send_response("error:robot_not_connected")

    finally:

        node.shutdown()                return                        command = data.decode('utf-8').strip()                            self.publisher_to_robot.publish(msg)

        node.destroy_node()

        rclpy.shutdown()            

        print("👋 Bridge Pi arrêté")

            # Get Coords                                                    



if __name__ == '__main__':            if action == 'get_coords':

    main()

                if self.robot:                        if command:                            # EXÉCUTER la commande sur le robot

                    coords = self.robot.get_coords()

                    self.get_logger().info(f"📍 Coords : {coords}")                            self.get_logger().info(f"📥 Commande : '{command}'")                            self.execute_command(command)

                    self.send_response(f"coords:{coords}")

                else:                                                    else:

                    self.send_response("error:robot_not_connected")

                return                            # Publier sur ROS2                            self.get_logger().warn("⚠️  Commande vide")

            

            # Gripper Open                            msg = String()                    else:

            if action == 'gripper_open':

                if self.robot:                            msg.data = command                        self.get_logger().warn("⚠️  Données vides (connexion fermée par Tour)")

                    self.robot.set_gripper_state(0, 50)

                    self.get_logger().info("✋ Gripper OUVERT")                            self.publisher_to_robot.publish(msg)                        break

                    self.send_response("gripper:open")

                else:                                                    

                    self.send_response("error:robot_not_connected")

                return                            # EXÉCUTER la commande sur le robot                except socket.timeout:

            

            # Gripper Close                            self.execute_command(command)                    # Timeout normal, on continue

            if action == 'gripper_close':

                if self.robot:                        else:                    continue

                    self.robot.set_gripper_state(1, 50)

                    self.get_logger().info("✊ Gripper FERMÉ")                            self.get_logger().warn("⚠️  Commande vide")                except Exception as e:

                    self.send_response("gripper:closed")

                else:                    else:                    self.get_logger().error(f"❌ Erreur recv : {e}")

                    self.send_response("error:robot_not_connected")

                return                        self.get_logger().warn("⚠️  Données vides (connexion fermée par Tour)")                    break

            

            # Emergency Stop                        break                    

            if action == 'emergency_stop':

                if self.robot:                                except Exception as e:

                    self.robot.release_all_servos()

                    self.get_logger().warn("🚨 ARRÊT D'URGENCE !")                except socket.timeout:            self.get_logger().error(f"❌ Erreur receive_from_tour : {e}")

                    self.send_response("emergency:stopped")

                else:                    # Timeout normal, on continue            import traceback

                    self.send_response("error:robot_not_connected")

                return                    continue            self.get_logger().error(traceback.format_exc())

            

            # Power On                except Exception as e:        finally:

            if action == 'power_on':

                if self.robot:                    self.get_logger().error(f"❌ Erreur recv : {e}")            self.get_logger().warn("⚠️  Tour déconnectée")

                    self.robot.power_on()

                    self.get_logger().info("⚡ Power ON")                    break            if self.client_socket:

                    self.send_response("power:on")

                else:                                    try:

                    self.send_response("error:robot_not_connected")

                return        except Exception as e:                    self.client_socket.close()

            

            # Power Off            self.get_logger().error(f"❌ Erreur receive_from_tour : {e}")                except:

            if action == 'power_off':

                if self.robot:            import traceback                    pass

                    self.robot.power_off()

                    self.get_logger().info("🔌 Power OFF")            self.get_logger().error(traceback.format_exc())                self.client_socket = None

                    self.send_response("power:off")

                else:        finally:    

                    self.send_response("error:robot_not_connected")

                return            self.get_logger().warn("⚠️  Tour déconnectée")    def execute_command(self, command):

            

            # Action inconnue            if self.client_socket:        """Exécute la commande sur le robot MyCobot"""

            self.get_logger().warn(f"⚠️  Action JSON inconnue : {action}")

            self.send_response(f"error:unknown_action:{action}")                try:        self.get_logger().info(f"🔧 Exécution : {command}")

            

        except Exception as e:                    self.client_socket.close()        

            self.get_logger().error(f"❌ Erreur action JSON : {e}")

            self.send_response(f"error:json:{e}")                except:        try:

    

    def execute_simple_command(self, command):                    pass            # Ping/Pong

        """Exécute une commande au format simple (texte)"""

        try:                self.client_socket = None            if command == "ping":

            # Ping/Pong

            if command == "ping":                    self.send_response("pong")

                self.send_response("pong")

                return    def execute_command(self, command):                return

            

            # Get angles        """Exécute la commande sur le robot MyCobot"""            

            if command == "get_angles":

                if self.robot:        self.get_logger().info(f"🔧 Exécution : {command}")            # Get angles

                    angles = self.robot.get_angles()

                    self.send_response(f"angles:{angles}")                    if command == "get_angles":

                else:

                    self.send_response("error:robot_not_connected")        try:                if self.robot:

                return

                        # Ping/Pong                    angles = self.robot.get_angles()

            # Status

            if command == "status":            if command == "ping":                    self.send_response(f"angles:{angles}")

                if self.robot:

                    angles = self.robot.get_angles()                self.send_response("pong")                else:

                    self.send_response(f"status:ok,angles:{angles}")

                else:                return                    self.send_response("error:robot_not_connected")

                    self.send_response("status:robot_not_connected")

                return                            return

            

            # Set LED            # Get angles            

            if command.startswith("set_led:"):

                if self.robot:            if command == "get_angles":            # Status

                    try:

                        rgb = command.split(":")[1].split(",")                if self.robot:            if command == "status":

                        r, g, b = int(rgb[0]), int(rgb[1]), int(rgb[2])

                        self.robot.set_color(r, g, b)                    angles = self.robot.get_angles()                if self.robot:

                        self.get_logger().info(f"💡 LED : R={r}, G={g}, B={b}")

                        self.send_response(f"led_ok:r={r},g={g},b={b}")                    self.send_response(f"angles:{angles}")                    self.send_response("status:ok")

                    except Exception as e:

                        self.get_logger().error(f"❌ Erreur LED : {e}")                else:                else:

                        self.send_response(f"error:led:{e}")

                else:                    self.send_response("error:robot_not_connected")                    self.send_response("status:robot_not_connected")

                    self.send_response("error:robot_not_connected")

                return                return                return

            

            # Go HOME                        

            if command.startswith("go_home"):

                if self.robot:            # Status            # Set LED

                    try:

                        parts = command.split(":")            if command == "status":            if command.startswith("set_led:"):

                        speed = int(parts[1]) if len(parts) > 1 else 20

                        self.robot.send_angles(self.zero_angles, speed)                if self.robot:                if self.robot:

                        self.get_logger().info(f"🏠 HOME (vitesse {speed})")

                        self.send_response(f"home_ok:speed={speed}")                    self.send_response("status:ok")                    try:

                    except Exception as e:

                        self.get_logger().error(f"❌ Erreur HOME : {e}")                else:                        rgb = command.split(":")[1].split(",")

                        self.send_response(f"error:home:{e}")

                else:                    self.send_response("status:robot_not_connected")                        r, g, b = int(rgb[0]), int(rgb[1]), int(rgb[2])

                    self.send_response("error:robot_not_connected")

                return                return                        self.robot.set_color(r, g, b)

            

            # Set angle (un seul joint)                                    self.get_logger().info(f"💡 LED : R={r}, G={g}, B={b}")

            if command.startswith("set_angle:"):

                if self.robot:            # Set LED                        self.send_response(f"led_ok:r={r},g={g},b={b}")

                    try:

                        params = command.split(":")[1].split(",")            if command.startswith("set_led:"):                    except Exception as e:

                        joint = int(params[0])

                        angle = float(params[1])                if self.robot:                        self.get_logger().error(f"❌ Erreur LED : {e}")

                        speed = int(params[2]) if len(params) > 2 else 20

                                            try:                        self.send_response(f"error:led:{e}")

                        self.robot.send_angle(joint, angle, speed)

                        self.get_logger().info(f"🔧 Joint {joint} → {angle}° (vitesse {speed})")                        rgb = command.split(":")[1].split(",")                else:

                        self.send_response(f"angle_ok:j={joint},a={angle},s={speed}")

                    except Exception as e:                        r, g, b = int(rgb[0]), int(rgb[1]), int(rgb[2])                    self.send_response("error:robot_not_connected")

                        self.get_logger().error(f"❌ Erreur set_angle : {e}")

                        self.send_response(f"error:set_angle:{e}")                        self.robot.set_color(r, g, b)                return

                else:

                    self.send_response("error:robot_not_connected")                        self.get_logger().info(f"💡 LED : R={r}, G={g}, B={b}")            

                return

                                    self.send_response(f"led_ok:r={r},g={g},b={b}")            # Go HOME

            # Set angles (tous les joints)

            if command.startswith("set_angles:"):                    except Exception as e:            if command.startswith("go_home"):

                if self.robot:

                    try:                        self.get_logger().error(f"❌ Erreur LED : {e}")                if self.robot:

                        parts = command.split(":")[1:]

                        angles = [float(a) for a in parts[0].split(",")]                        self.send_response(f"error:led:{e}")                    try:

                        speed = int(parts[1]) if len(parts) > 1 else 20

                                        else:                        parts = command.split(":")

                        self.robot.send_angles(angles, speed)

                        self.get_logger().info(f"🔧 Angles → {angles} (vitesse {speed})")                    self.send_response("error:robot_not_connected")                        speed = int(parts[1]) if len(parts) > 1 else 20

                        self.send_response(f"angles_ok:{angles},s={speed}")

                    except Exception as e:                return                        self.robot.send_angles([0, 0, 0, 0, 0, 0], speed)

                        self.get_logger().error(f"❌ Erreur set_angles : {e}")

                        self.send_response(f"error:set_angles:{e}")                                    self.get_logger().info(f"🏠 HOME (vitesse {speed})")

                else:

                    self.send_response("error:robot_not_connected")            # Go HOME                        self.send_response(f"home_ok:speed={speed}")

                return

                        if command.startswith("go_home"):                    except Exception as e:

            # Commande inconnue

            self.get_logger().warn(f"⚠️  Commande inconnue : {command}")                if self.robot:                        self.get_logger().error(f"❌ Erreur HOME : {e}")

            self.send_response(f"error:unknown_command:{command}")

                                try:                        self.send_response(f"error:home:{e}")

        except Exception as e:

            self.get_logger().error(f"❌ Erreur commande simple : {e}")                        parts = command.split(":")                else:

            self.send_response(f"error:simple:{e}")

                            speed = int(parts[1]) if len(parts) > 1 else 20                    self.send_response("error:robot_not_connected")

    def send_response(self, response):

        """Envoie une réponse à la Tour via TCP"""                        self.robot.send_angles([0, 0, 0, 0, 0, 0], speed)                return

        try:

            if self.client_socket:                        self.get_logger().info(f"🏠 HOME (vitesse {speed})")            

                self.client_socket.sendall((response + '\n').encode('utf-8'))

                self.get_logger().info(f"📤 Réponse envoyée : {response}")                        self.send_response(f"home_ok:speed={speed}")            # Set angle (un seul joint)

        except Exception as e:

            self.get_logger().error(f"❌ Erreur envoi réponse : {e}")                    except Exception as e:            if command.startswith("set_angle:"):

    

    def from_robot_callback(self, msg):                        self.get_logger().error(f"❌ Erreur HOME : {e}")                if self.robot:

        """Callback pour les messages provenant du robot (si besoin)"""

        if self.client_socket:                        self.send_response(f"error:home:{e}")                    try:

            try:

                self.client_socket.sendall((msg.data + '\n').encode('utf-8'))                else:                        params = command.split(":")[1].split(",")

                self.get_logger().info(f"📤 Message robot→Tour : {msg.data}")

            except Exception as e:                    self.send_response("error:robot_not_connected")                        joint = int(params[0])

                self.get_logger().error(f"❌ Erreur envoi : {e}")

                    return                        angle = float(params[1])

    def shutdown(self):

        """Arrêt propre"""                                    speed = int(params[2]) if len(params) > 2 else 20

        self.get_logger().info("🛑 Arrêt du bridge...")

        self.running = False            # Set angle (un seul joint)                        

        if self.client_socket:

            self.client_socket.close()            if command.startswith("set_angle:"):                        self.robot.send_angle(joint, angle, speed)

        if self.server_socket:

            self.server_socket.close()                if self.robot:                        self.get_logger().info(f"🔧 Joint {joint} → {angle}° (vitesse {speed})")



                    try:                        self.send_response(f"angle_ok:j={joint},a={angle},s={speed}")

def main(args=None):

    rclpy.init(args=args)                        params = command.split(":")[1].split(",")                    except Exception as e:

    node = BridgePiDebug()

                            joint = int(params[0])                        self.get_logger().error(f"❌ Erreur set_angle : {e}")

    try:

        rclpy.spin(node)                        angle = float(params[1])                        self.send_response(f"error:set_angle:{e}")

    except KeyboardInterrupt:

        pass                        speed = int(params[2]) if len(params) > 2 else 20                else:

    finally:

        node.shutdown()                                            self.send_response("error:robot_not_connected")

        node.destroy_node()

        rclpy.shutdown()                        self.robot.send_angle(joint, angle, speed)                return



                        self.get_logger().info(f"🔧 Joint {joint} → {angle}° (vitesse {speed})")            

if __name__ == '__main__':

    main()                        self.send_response(f"angle_ok:j={joint},a={angle},s={speed}")            # Set angles (tous les joints)


                    except Exception as e:            if command.startswith("set_angles:"):

                        self.get_logger().error(f"❌ Erreur set_angle : {e}")                if self.robot:

                        self.send_response(f"error:set_angle:{e}")                    try:

                else:                        parts = command.split(":")[1:]

                    self.send_response("error:robot_not_connected")                        angles = [float(a) for a in parts[0].split(",")]

                return                        speed = int(parts[1]) if len(parts) > 1 else 20

                                    

            # Set angles (tous les joints)                        self.robot.send_angles(angles, speed)

            if command.startswith("set_angles:"):                        self.get_logger().info(f"🔧 Angles → {angles} (vitesse {speed})")

                if self.robot:                        self.send_response(f"angles_ok:{angles},s={speed}")

                    try:                    except Exception as e:

                        parts = command.split(":")[1:]                        self.get_logger().error(f"❌ Erreur set_angles : {e}")

                        angles = [float(a) for a in parts[0].split(",")]                        self.send_response(f"error:set_angles:{e}")

                        speed = int(parts[1]) if len(parts) > 1 else 20                else:

                                            self.send_response("error:robot_not_connected")

                        self.robot.send_angles(angles, speed)                return

                        self.get_logger().info(f"🔧 Angles → {angles} (vitesse {speed})")            

                        self.send_response(f"angles_ok:{angles},s={speed}")            # Commande inconnue

                    except Exception as e:            self.get_logger().warn(f"⚠️  Commande inconnue : {command}")

                        self.get_logger().error(f"❌ Erreur set_angles : {e}")            self.send_response(f"error:unknown_command:{command}")

                        self.send_response(f"error:set_angles:{e}")            

                else:        except Exception as e:

                    self.send_response("error:robot_not_connected")            self.get_logger().error(f"❌ Erreur exécution : {e}")

                return            import traceback

                        self.get_logger().error(traceback.format_exc())

            # Commande inconnue            self.send_response(f"error:execution:{e}")

            self.get_logger().warn(f"⚠️  Commande inconnue : {command}")    

            self.send_response(f"error:unknown_command:{command}")    def send_response(self, response):

                    """Envoie une réponse à la Tour via TCP"""

        except Exception as e:        try:

            self.get_logger().error(f"❌ Erreur exécution : {e}")            if self.client_socket:

            import traceback                self.client_socket.sendall((response + '\n').encode('utf-8'))

            self.get_logger().error(traceback.format_exc())                self.get_logger().info(f"📤 Réponse envoyée : {response}")

            self.send_response(f"error:execution:{e}")        except Exception as e:

                self.get_logger().error(f"❌ Erreur envoi réponse : {e}")

    def send_response(self, response):    

        """Envoie une réponse à la Tour via TCP"""    def from_robot_callback(self, msg):

        try:        """Callback pour les messages provenant du robot (si besoin)"""

            if self.client_socket:        if self.client_socket:

                self.client_socket.sendall((response + '\n').encode('utf-8'))            try:

                self.get_logger().info(f"📤 Réponse envoyée : {response}")                self.client_socket.sendall((msg.data + '\n').encode('utf-8'))

        except Exception as e:                self.get_logger().info(f"📤 Message robot→Tour : {msg.data}")

            self.get_logger().error(f"❌ Erreur envoi réponse : {e}")            except Exception as e:

                    self.get_logger().error(f"❌ Erreur envoi : {e}")

    def from_robot_callback(self, msg):    

        """Callback pour les messages provenant du robot (si besoin)"""    def shutdown(self):

        if self.client_socket:        """Arrêt propre"""

            try:        self.get_logger().info("🛑 Arrêt du bridge...")

                self.client_socket.sendall((msg.data + '\n').encode('utf-8'))        self.running = False

                self.get_logger().info(f"📤 Message robot→Tour : {msg.data}")        if self.client_socket:

            except Exception as e:            self.client_socket.close()

                self.get_logger().error(f"❌ Erreur envoi : {e}")        if self.server_socket:

                self.server_socket.close()

    def shutdown(self):

        """Arrêt propre"""def main(args=None):

        self.get_logger().info("🛑 Arrêt du bridge...")    rclpy.init(args=args)

        self.running = False    node = BridgePiDebug()

        if self.client_socket:    

            self.client_socket.close()    try:

        if self.server_socket:        rclpy.spin(node)

            self.server_socket.close()    except KeyboardInterrupt:

        pass

def main(args=None):    finally:

    rclpy.init(args=args)        node.shutdown()

    node = BridgePiDebug()        node.destroy_node()

            rclpy.shutdown()

    try:

        rclpy.spin(node)if __name__ == '__main__':

    except KeyboardInterrupt:    main()

        pass
    finally:
        node.shutdown()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
