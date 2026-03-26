import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
import sys
import time

class BridgeTour(Node):
    def __init__(self):
        super().__init__('bridge_tour')
        self.publisher_ = self.create_publisher(String, '/from_robot', 10)
        self.subscription = self.create_subscription(String, '/to_robot', self.send_callback, 10)
        
        # --- VERIFIEZ BIEN CETTE IP ---
        self.pi_ip = '10.10.0.218' 
        self.port = 5005
        self.socket = None
        self.connected = False
        self.lock = threading.Lock()
        
        # Connexion initiale
        self.connect()

    def connect(self):
        """Se connecte ou se reconnecte au Pi"""
        with self.lock:
            # Fermer l'ancien socket si présent
            if self.socket:
                try:
                    self.socket.close()
                except:
                    pass
                self.socket = None
            
            try:
                self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.socket.settimeout(5.0) 
                self.socket.connect((self.pi_ip, self.port))
                self.socket.settimeout(None)
                self.connected = True
                self.get_logger().info(f"✅ Connecté à la Pi ({self.pi_ip}:{self.port})")
                return True
            except Exception as e:
                self.connected = False
                self.get_logger().error(f"❌ Connexion impossible : {e}")
                return False

    def reconnect(self):
        """Tente de se reconnecter avec délai"""
        self.connected = False
        self.get_logger().warn("🔄 Tentative de reconnexion dans 2s...")
        time.sleep(2)
        return self.connect()

    def send_callback(self, msg):
        if not self.connected:
            self.get_logger().warn(f"⚠️  Non connecté, tentative de reconnexion...")
            if not self.reconnect():
                return
        
        try:
            with self.lock:
                if self.socket:
                    self.socket.sendall((msg.data + '\n').encode('utf-8'))
                    self.get_logger().info(f"📤 Envoyé vers Pi: {msg.data}")
        except BrokenPipeError:
            self.get_logger().error(f"❌ Broken pipe - Pi déconnectée")
            self.connected = False
        except Exception as e:
            self.get_logger().error(f"❌ Erreur d'envoi : {e}")
            self.connected = False

    def receive_loop(self):
        while rclpy.ok():
            if not self.connected:
                time.sleep(1)
                continue
                
            try:
                with self.lock:
                    sock = self.socket
                
                if not sock:
                    time.sleep(1)
                    continue
                    
                sock.settimeout(1.0)
                try:
                    data = sock.recv(1024)
                except socket.timeout:
                    continue
                    
                if data:
                    msg = String()
                    msg.data = data.decode('utf-8').strip()
                    if msg.data:
                        self.publisher_.publish(msg)
                        self.get_logger().info(f"📥 Reçu de Pi: {msg.data}")
                else:
                    self.get_logger().warn("⚠️  Pi a fermé la connexion.")
                    self.connected = False
                    self.reconnect()
            except Exception as e:
                self.get_logger().error(f"❌ Erreur de réception : {e}")
                self.connected = False

def main(args=None):
    rclpy.init(args=args)
    node = BridgeTour()
    
    # On utilise un thread pour la réception TCP
    thread = threading.Thread(target=node.receive_loop, daemon=True)
    thread.start()
    
    try:
        # Spin est crucial : c'est lui qui déclare les topics au reste du système
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()        

if __name__ == '__main__':
    main()