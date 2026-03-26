import rclpy
from rclpy.node import Node
import cv2
import cv2.aruco as aruco
import numpy as np
from pymycobot.mycobot320 import MyCobot320

class RelativeFollower(Node):
    def __init__(self):
        super().__init__('relative_follower')
        
        # 1. Connexion au robot (Port adapté pour MyCobot Pi)
        try:
            self.mc = MyCobot320('/dev/ttyAMA0', 115200)
            self.get_logger().info("Connexion MyCobot réussie.")
        except Exception as e:
            self.get_logger().error(f"Erreur connexion robot: {e}")

        # 2. Paramètres Vision
        # Si la fenêtre reste noire, change l'index (0, 1, ou 2)
        self.cap = cv2.VideoCapture(0)
        
        # Dictionnaire ArUco : Essaye DICT_4X4_50 si tes marqueurs ne sont pas détectés
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters_create()
        
        # IDs des marqueurs
        self.id_robot = 0   # Sur le bras
        self.id_target = 1  # Dans la main
        
        # Ratio de conversion Pixel -> mm (à ajuster selon la distance caméra-robot)
        self.pixel_to_mm = 0.6 

        # Boucle de contrôle à 10Hz (0.1s)
        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Nœud de suivi relatif (Vision Directe) démarré...")

    def control_loop(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn("Impossible de lire le flux caméra")
            return

        # Conversion en gris pour améliorer la détection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, rejected = aruco.detectMarkers(gray, self.aruco_dict, parameters=self.parameters)

        # Dessiner les détections pour le debug visuel
        if ids is not None:
            aruco.drawDetectedMarkers(frame, corners, ids)
            
            pos_robot = None
            pos_target = None

            # Extraction des centres des marqueurs
            for i in range(len(ids)):
                c = corners[i][0]
                center = (int(c[:, 0].mean()), int(c[:, 1].mean()))
                
                if ids[i][0] == self.id_robot:
                    pos_robot = center
                elif ids[i][0] == self.id_target:
                    pos_target = center

            if pos_robot and pos_target:
                # Calcul de l'écart relatif en pixels (Vue Caméra)
                dx_px = pos_target[0] - pos_robot[0]
                dy_px = pos_target[1] - pos_robot[1]

                # MAPPING DES AXES (Adapté selon ta vue caméra face au robot) :
                # Caméra X (horizontal) -> Robot Y
                # Caméra Y (vertical)   -> Robot Z
                delta_y = -(dx_px * self.pixel_to_mm)
                delta_z = -(dy_px * self.pixel_to_mm)

                self.get_logger().info(f"Deltas calculés -> dY:{int(delta_y)} dZ:{int(delta_z)}")

                # Commande Robot
                curr = self.mc.get_coords()
                if curr and len(curr) > 0:
                    new_y = curr[1] + delta_y
                    new_z = curr[2] + delta_z

                    # Limites de sécurité
                    new_y = max(-180.0, min(new_y, 180.0))
                    new_z = max(150.0, min(new_z, 320.0))

                    # Seuil de mouvement pour éviter les tremblements (5mm)
                    if abs(delta_y) > 5 or abs(delta_z) > 5:
                        self.mc.send_coords([curr[0], new_y, new_z, 180, 0, 0], 40, 1)
            else:
                self.get_logger().warn("Un des marqueurs (0 ou 1) manque à l'appel", once=True)
        
        # Affichage de la fenêtre de debug
        cv2.imshow("MyCobot Vision Directe", frame)
        cv2.waitKey(1)

    def __del__(self):
        if self.cap.is_opened():
            self.cap.release()
        cv2.destroyAllWindows()

def main():
    rclpy.init()
    node = RelativeFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Arrêt du nœud...")
    finally:
        node.destroy_node()
        rclpy.shutdown()