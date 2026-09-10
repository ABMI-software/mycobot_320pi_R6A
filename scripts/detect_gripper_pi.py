#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Détecteur de gripper — À LANCER SUR LE PI.

Essaie plusieurs API pymycobot et plusieurs gripper_id pour trouver
CE QUI RÉPOND réellement. Ne fait bouger rien de dangereux (lectures +
petites commandes gripper). Aide à identifier le modèle et l'ID.
"""
import time
from pymycobot import MyCobot320

PORT = "/dev/ttyAMA0"
BAUD = 1000000

mc = MyCobot320(PORT, BAUD)
time.sleep(1)
print("Robot angles:", mc.get_angles())   # confirme la liaison série
time.sleep(0.5)

print("\n=== API GRIPPER BASIQUE (adaptive gripper) ===")
for name, fn in [
    ("get_gripper_value()", lambda: mc.get_gripper_value()),
    ("is_gripper_moving()", lambda: mc.is_gripper_moving()),
]:
    try:
        print(f"  {name} -> {fn()}")
    except Exception as e:
        print(f"  {name} -> ERREUR {e}")
    time.sleep(0.4)

print("\n=== API GRIPPER PRO : balayage des gripper_id ===")
print("  (angle valide = 0..100 ; -1 ou erreur = pas de gripper à cet id)")
found = []
for gid in list(range(1, 21)) + [254]:
    try:
        a = mc.get_pro_gripper_angle(gid)
        flag = "  <-- RÉPOND !" if (a is not None and a != -1) else ""
        if flag:
            found.append(gid)
        print(f"  id={gid:>3} : angle={a}{flag}")
    except Exception as e:
        print(f"  id={gid:>3} : ERREUR {str(e)[:40]}")
    time.sleep(0.3)

print("\n=== RÉSUMÉ ===")
if found:
    print(f"gripper Pro trouvé aux id : {found}")
else:
    print("Aucun gripper Pro ne répond (angle -1 partout).")
    print("-> soit c'est un gripper ADAPTIVE (API basique ci-dessus, set_gripper_value),")
    print("-> soit problème matériel : alimentation gripper / câble data / connecteur.")
