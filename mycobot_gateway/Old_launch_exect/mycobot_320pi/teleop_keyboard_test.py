import sys
import termios
import tty
import time
import rclpy
from pymycobot.mycobot import MyCobot

# On garde ton message d'aide
msg = """ ... (ton message) ... """

def teleop_keyboard():
    # Connexion explicite
    print("Connexion au MyCobot 320 Pi...")
    mc = MyCobot("/dev/ttyAMA0", 115200)
    time.sleep(0.5)
    
    speed = 40
    model = 0 # 0 = move joint, 1 = move linear
    change_len = 10.0 # 10mm par appui
    change_angle = 5.0 # 5 degrés par appui

    # On vérifie qu'on arrive à lire le robot
    res = mc.get_coords()
    while not res:
        print("Attente des coordonnées initiales...")
        res = mc.get_coords()
        time.sleep(0.5)
    
    print(f"Position actuelle détectée : {res}")
    record_coords = [res, speed, model]

    def reset_to_current():
        """Récupère la position réelle pour éviter les décalages"""
        curr = mc.get_coords()
        if curr:
            record_coords[0] = curr

    try:
        while True:
            # Lecture d'une touche sans appuyer sur Entrée
            fd = sys.stdin.fileno()
            old_settings = termios.tcgetattr(fd)
            try:
                tty.setraw(sys.stdin.fileno())
                key = sys.stdin.read(1)
            finally:
                termios.tcsetattr(fd, termios.TCSANOW, old_settings)

            if key == "q":
                break
            
            # Logique de mouvement (X, Y, Z)
            if key == "w": record_coords[0][0] += change_len
            elif key == "s": record_coords[0][0] -= change_len
            elif key == "a": record_coords[0][1] -= change_len
            elif key == "d": record_coords[0][1] += change_len
            elif key == "z": record_coords[0][2] -= change_len
            elif key == "x": record_coords[0][2] += change_len
            
            # Envoi de la commande
            print(f"\rEnvoi Coordonnées : {record_coords[0]}", end="")
            mc.send_coords(record_coords[0], speed, model)
            
            # Optionnel : petit sleep pour ne pas spammer le port série
            time.sleep(0.1)

    except Exception as e:
        print(f"\nErreur : {e}")

def main():
    teleop_keyboard()

if __name__ == "__main__":
    main()