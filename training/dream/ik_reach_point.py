#!/usr/bin/env python3
"""Estime les angles articulaires pour atteindre un point (cinématique inverse).

Donne un point cible (x, y, z) en mètres dans le repère BASE du robot -> l'IK
(mycobot_ik.py) rend les 6 angles j1..j6 nécessaires pour y placer l'effecteur.
Optionnellement, commande le robot vers ces angles (bridge TCP).

Repère base : X = avant, Y = gauche, Z = haut ; origine = centre de la base,
Z = 0 = table. Unités : mètres.

Usage :
    # estimer seulement
    python ik_reach_point.py 0.25 0.10 0.06
    # estimer PUIS commander le robot
    python ik_reach_point.py 0.25 0.10 0.06 --send --pi-host 10.10.0.224
"""
import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from mycobot_ik import inverse_kinematics_position, fk_end_effector  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("point", type=float, nargs=3, metavar=("X", "Y", "Z"),
                    help="point cible en mètres, repère base (X avant, Y gauche, Z haut)")
    ap.add_argument("--send", action="store_true", help="commander le robot vers ces angles")
    ap.add_argument("--pi-host", default="10.10.0.224")
    ap.add_argument("--speed", type=int, default=25)
    args = ap.parse_args()

    target = np.array(args.point, dtype=float)
    ok, q_rad, res = inverse_kinematics_position(target)   # cinématique inverse
    q_deg = np.degrees(q_rad)
    ee = fk_end_effector(q_rad)                            # vérification par FK

    print(f"Cible          : {target} m")
    print(f"Atteignable    : {'OUI' if ok else 'NON'}  (résidu {res * 1000:.1f} mm, seuil 5 mm)")
    print(f"Angles j1..j6  : {np.round(q_deg, 1)} deg")
    print(f"Vérif FK       : effecteur à {np.round(ee, 3)} m (doit ~= la cible)")

    if not ok:
        print("⚠️  Point non atteignable proprement — je n'envoie PAS au robot.")
        return

    if args.send:
        import time
        sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
        from capture_real_3cam import RobotBridge
        b = RobotBridge(args.pi_host)
        b.connect()
        print("🏠 home d'abord…")
        b.go_home()
        time.sleep(3)
        b.send_angles([round(float(a), 2) for a in q_deg], args.speed)
        print("✅ commande envoyée. Lis les encodeurs pour confirmer la pose atteinte.")


if __name__ == "__main__":
    main()
