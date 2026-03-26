import rclpy
from rclpy.node import Node
from tf2_ros import TransformListener, Buffer
from pymycobot.mycobot import MyCobot
import math

class MarkerFollower(Node):
    def __init__(self):
        super().__init__("following_marker")
        try:
            self.mc = MyCobot('/dev/ttyAMA0', 115200)
            self.mc.power_on()
            self.get_logger().info("Connecté au MyCobot 320.")
        except Exception as e:
            self.get_logger().error(f"Erreur série : {e}")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.timer = self.create_timer(0.2, self.timer_callback)

    def timer_callback(self):
        try:
            now = rclpy.time.Time()
            trans = self.tf_buffer.lookup_transform("base", "basic_shapes", now)

            # Coordonnées récupérées
            x = trans.transform.translation.x * 1000
            y = trans.transform.translation.y * 1000
            z = trans.transform.translation.z * 1000

            self.get_logger().info(f"COORD REÇUES -> X:{int(x)} Y:{int(y)} Z:{int(z)}")

            # ON ACCEPTE LES X NÉGATIFS (car la caméra est "devant" mais orientée vers l'arrière dans ROS)
            # On vérifie si l'objet est à une distance raisonnable (entre 10cm et 80cm du robot)
            if (-800 < x < -100) and (-500 < y < 500):
                
                # TRANSFORMATION : On inverse X pour envoyer une commande positive au robot
                # Si X_ros = -400, alors target_x devient +400
                target_x = abs(x) 
                target_y = -y # Inversion Y pour corriger l'effet miroir si nécessaire
                target_z = z 

                # Limites physiques de sécurité pour le MyCobot 320
                target_x = max(130, min(target_x, 350))
                target_y = max(-200, min(target_y, 200))
                target_z = max(100, min(target_z, 400))

                coords = [float(target_x), float(target_y), float(target_z), 180.0, 0.0, 0.0]
                self.get_logger().info(f"MOUVEMENT envoyé : {coords}")
                self.mc.send_coords(coords, 40, 1)
            else:
                self.get_logger().warn("Cible hors zone de capture (Vérifiez X entre -100 et -800)")

        except Exception as e:
            # self.get_logger().error(f"Erreur TF: {e}")
            pass

def main(args=None):
    rclpy.init(args=args)
    node = MarkerFollower()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()