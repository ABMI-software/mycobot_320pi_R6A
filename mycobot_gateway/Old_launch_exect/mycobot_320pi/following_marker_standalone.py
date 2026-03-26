import rclpy
from rclpy.node import Node
import cv2
import cv2.aruco as aruco
import numpy as np
from pymycobot.mycobot320 import MyCobot320

class MyCobotRelativeFollower(Node):
    def __init__(self):
        super().__init__('relative_follower')
        
        # 1. Configuration du Robot
        self.mc = MyCobot320('/dev/ttyAMA0', 115200)
        self.id_robot = 0   # ID ArUco sur le bras
        self.id_target = 1  # ID ArUco dans ta main
        
        # 2. Configuration Vision
        self.cap = cv2.VideoCapture(0) # 0 pour la caméra par défaut
        self.aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
        self.parameters = aruco.DetectorParameters_create()
        
        # Timer pour la boucle de contrôle (10Hz)
        self.create_timer(0.1, self.control_loop)
        self.get_logger().info("Démarrage du suivi relatif (Méthode Directe)...")

    def control_loop(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        # Détection des marqueurs
        corners, ids, rejected = aruco.detectMarkers(frame, self.aruco_dict, parameters=self.parameters)
        
        if ids is not None:
            # On cherche les positions (centres) des deux marqueurs
            pos_robot = None
            pos_target = None
            
            for i in range(len(ids)):
                c = corners[i][0]
                center = (int(c[:, 0].mean()), int(c[:, 1].mean()))
                
                if ids[i][0] == self.id_robot:
                    pos_robot = center
                elif ids[i][0] == self.id_target:
                    pos_target = center

            # Si on voit les deux marqueurs
            if pos_robot and pos_target:
                # Calcul de l'écart en pixels (Vecteur Delta)
                dx_pixel = pos_target[0] - pos_robot[0]
                dy_pixel = pos_target[1] - pos_robot[1]

                # Conversion approximative Pixel -> mm (à ajuster selon ta distance)
                # Ratio : env. 1px = 0.5mm à 60cm de distance
                ratio = 0.6 
                move_y = -(dx_pixel * ratio) # Horizontal caméra -> Axe Y Robot
                move_z = -(dy_pixel * ratio) # Vertical caméra -> Axe Z Robot

                self.get_logger().info(f"Action -> dY:{move_y:.1f}mm, dZ:{move_z:.1f}mm")

                # Récupération et mise à jour
                curr = self.mc.get_coords()
                if curr and len(curr) > 0:
                    new_coords = [
                        curr[0],           # On ne touche pas au X (profondeur) pour ce test
                        curr[1] + move_y,  # Gauche/Droite
                        curr[2] + move_z,  # Haut/Bas
                        180, 0, 0          # Orientation fixe
                    ]
                    
                    # Sécurité simple pour ne pas cogner
                    if 150 < new_coords[2] < 350: 
                        self.mc.send_coords(new_coords, 40, 1)

        # Affichage pour debug
        cv2.imshow("MyCobot Vision", frame)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = MyCobotRelativeFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
