import rclpy
from rclpy.node import Node
from tf2_ros.transform_listener import TransformListener
from tf2_ros.buffer import Buffer
from pymycobot.mycobot import MyCobot
import time

class MarkerFollower(Node):
    def __init__(self):
        super().__init__("following_marker")

        # 1. Connexion Hardware (Priorité absolue)
        try:
            # On initialise avec un timeout pour ne pas bloquer
            self.mc = MyCobot('/dev/ttyAMA0', 115200)
            self.get_logger().info("Connexion série établie avec le MyCobot 320.")
            # On s'assure que le robot est en mode commande
            self.mc.power_on()
            time.sleep(1)
        except Exception as e:
            self.get_logger().error(f"Erreur connexion hardware : {e}")

        # 2. Setup TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 3. Boucle de contrôle (réduite à 5Hz pour laisser le temps au robot de bouger)
        self.timer = self.create_timer(0.2, self.timer_callback)

    def timer_callback(self):
        try:
            # On cherche la position de la cible (Aruco) par rapport à la base du robot
            now = rclpy.time.Time()
            # Remplacez 'base' par le nom exact de votre lien de base dans l'URDF (souvent 'base' ou 'link1')
            trans = self.tf_buffer.lookup_transform("base", "basic_shapes", now)
            
            # Conversion mètres -> millimètres
            x = trans.transform.translation.x * 1000
            y = trans.transform.translation.y * 1000
            z = trans.transform.translation.z * 1000

            # --- LOGIQUE DE SÉCURITÉ ---
            # Le MyCobot 320 a une portée d'environ 320mm. 
            # Si le marqueur est trop loin, on ne commande pas le mouvement.
            distance = (x**2 + y**2 + z**2)**0.5
            if distance > 350:
                self.get_logger().warn(f"Cible trop éloignée : {distance:.2f}mm")
                return

            # Coordonnées [x, y, z, rx, ry, rz]
            # rx=180 : pince vers le bas. Ajustez selon votre besoin.
            coords = [x, y, z, 180.0, 0.0, 0.0]

            self.get_logger().info(f"Envoi commande : {coords}")
            
            # send_coords(coords, speed, mode)
            # mode 1 = linéaire (mouvement plus fluide pour du tracking)
            self.mc.send_coords(coords, 50, 1)

        except Exception as e:
            # Souvent : transform non trouvée car le marqueur n'est pas dans le champ
            self.get_logger().debug("En attente du marqueur ArUco...")