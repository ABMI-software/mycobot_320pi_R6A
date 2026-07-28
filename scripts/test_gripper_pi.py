#!/usr/bin/env python3
"""Test AUTONOME du gripper Pro adaptatif — À LANCER SUR LE PI (10.10.0.221).

Pilote UNIQUEMENT le gripper (pas de mouvement du bras), via pymycobot en direct.
API : docs Elephant myCobot320 (jiazhua_pi). gripper_id par défaut = 14.

⚠ Contraintes matérielles :
  - Le port série /dev/ttyAMA0 est mono-accès : ARRÊTER bridge_pi_simple.py avant
    (sinon 'port busy' / réponses corrompues).
  - Elephant impose ≥1.5 s entre deux commandes gripper -> on met 2.0 s.
  - Lancer une calibration zéro au premier montage.

Usage (sur le Pi) :
    python3 test_gripper_pi.py            # cycle open/close x3
    python3 test_gripper_pi.py --calib    # calibration zéro d'abord
    python3 test_gripper_pi.py --angle 40 # va à un angle précis (0-100)
"""
import argparse
import time

from pymycobot import MyCobot320

PORT = "/dev/ttyAMA0"
BAUD = 1000000
GID = 14           # gripper_id par défaut
GAP = 2.0          # s entre commandes (> 1.5 s imposé)

STATUS = {0: "en mouvement", 1: "arrêté (rien saisi)",
          2: "arrêté (objet saisi)", 3: "objet tombé"}


def pause():
    time.sleep(GAP)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--calib", action="store_true", help="calibration zéro d'abord")
    ap.add_argument("--angle", type=int, default=None, help="aller à un angle 0-100")
    ap.add_argument("--cycles", type=int, default=3, help="nb de cycles open/close")
    args = ap.parse_args()

    print(f"Connexion {PORT} @ {BAUD} ...")
    mc = MyCobot320(PORT, BAUD)
    pause()

    if args.calib:
        print("Calibration zéro du gripper ...")
        mc.set_pro_gripper_calibration(GID)
        pause()

    # vitesse et couple modérés pour un premier test
    mc.set_pro_gripper_speed(GID, 40)
    pause()
    mc.set_pro_gripper_torque(GID, 150)   # plage réelle 100-300 (pas 1-100)
    pause()

    if args.angle is not None:
        a = max(0, min(100, args.angle))
        print(f"-> angle {a}")
        mc.set_pro_gripper_angle(GID, a)
        pause()
        print(f"   angle lu : {mc.get_pro_gripper_angle(GID)}  "
              f"état : {STATUS.get(mc.get_pro_gripper_status(GID), '?')}")
        return

    for i in range(args.cycles):
        print(f"[cycle {i+1}/{args.cycles}] OUVRE")
        mc.set_pro_gripper_open(GID)
        pause()
        print(f"   angle : {mc.get_pro_gripper_angle(GID)}")
        print(f"[cycle {i+1}/{args.cycles}] FERME")
        mc.set_pro_gripper_close(GID)
        pause()
        print(f"   angle : {mc.get_pro_gripper_angle(GID)}  "
              f"état : {STATUS.get(mc.get_pro_gripper_status(GID), '?')}")

    print("Test terminé. On laisse le gripper ouvert.")
    mc.set_pro_gripper_open(GID)


if __name__ == "__main__":
    main()
