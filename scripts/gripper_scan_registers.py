#!/usr/bin/env python3
"""Scan des registres du gripper Pro via TCP — À LANCER SUR LE PC TOUR.

But : trouver l'adresse du COURANT (mA) — proxy de la force de serrage — que le
LCD "miniRobot Lite" affiche mais que pymycobot n'expose par aucun getter dédié.

Méthode : on lit toutes les adresses 0..50 via get_pro_gripper(gripper_id, addr).
Les adresses connues du protocole ProGripper :
   12 = position(%)   14 = statut   28 = couple   33 = vitesse defaut
Tout ce qui bouge quand tu SERRES un objet et pas à vide = candidat courant/force.

Usage :
   1) doigts OUVERTS, rien saisi :   python3 gripper_scan_registers.py --tag ouvert
   2) en train de SERRER un objet :  python3 gripper_scan_registers.py --tag serre
   -> compare les deux colonnes : l'adresse qui change = le courant/effort.
"""
import argparse
import json
import socket
import time

KNOWN = {12: "position(%)", 14: "statut", 28: "couple", 33: "vitesse"}


def send(sock, buf, obj):
    sock.sendall((json.dumps(obj) + "\n").encode())
    while "\n" not in buf[0]:
        chunk = sock.recv(1024).decode()
        if not chunk:
            break
        buf[0] += chunk
    line, _, buf[0] = buf[0].partition("\n")
    return line.strip()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="10.10.0.221")
    ap.add_argument("--port", type=int, default=5005)
    ap.add_argument("--id", type=int, default=14)
    ap.add_argument("--lo", type=int, default=0)
    ap.add_argument("--hi", type=int, default=50)
    ap.add_argument("--tag", default="")
    args = ap.parse_args()

    sock = socket.create_connection((args.host, args.port), timeout=8)
    sock.settimeout(8)
    buf = [""]
    print(f"# scan {args.lo}..{args.hi}  tag={args.tag or '(aucun)'}  id={args.id}")
    print(f"{'addr':>4} | {'valeur':>28} | connu")
    print("-" * 55)
    for a in range(args.lo, args.hi + 1):
        try:
            r = send(sock, buf, {"action": "get_pro_gripper", "address": a, "gripper_id": args.id})
        except Exception as e:
            r = f"ERR {e}"
        print(f"{a:>4} | {r:>28} | {KNOWN.get(a, '')}")
        time.sleep(0.15)   # lecture seule : pas besoin du gap 1.5 s des mouvements
    sock.close()


if __name__ == "__main__":
    main()
