#!/usr/bin/env python3
"""Contrôle du gripper Pro via TCP depuis le PC Tour — passe par bridge_pi_simple.py.

Aucune dépendance pymycobot côté PC : on envoie du JSON au bridge (Pi:5005), le
bridge appelle set_pro_gripper_* sur le robot. Le port série reste géré par le Pi.

⚠ Elephant impose >= 1.5 s entre deux commandes gripper -> ce client espace de 2 s.

Usage (sur le PC Tour) :
    python3 gripper_tcp_client.py --calib          # calibration zéro
    python3 gripper_tcp_client.py --open
    python3 gripper_tcp_client.py --close
    python3 gripper_tcp_client.py --angle 40
    python3 gripper_tcp_client.py --cycles 3       # ouvre/ferme x3
    python3 gripper_tcp_client.py --get            # lit l'angle courant
"""
import argparse
import json
import socket
import time

PI_HOST = "10.10.0.221"
PI_PORT = 5005
GAP = 2.0   # s entre commandes gripper (> 1.5 s imposé par Elephant)


class GripperClient:
    def __init__(self, host=PI_HOST, port=PI_PORT):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.sock.settimeout(5.0)
        self.sock.connect((host, port))
        self._buf = ""

    def send(self, cmd: dict) -> str:
        self.sock.sendall((json.dumps(cmd) + "\n").encode())
        # lire une ligne de réponse
        while "\n" not in self._buf:
            chunk = self.sock.recv(1024).decode()
            if not chunk:
                break
            self._buf += chunk
        line, _, self._buf = self._buf.partition("\n")
        return line.strip()

    def close(self):
        self.sock.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--calib", action="store_true")
    ap.add_argument("--open", action="store_true", dest="do_open")
    ap.add_argument("--close", action="store_true", dest="do_close")
    ap.add_argument("--angle", type=int, default=None)
    ap.add_argument("--cycles", type=int, default=None)
    ap.add_argument("--get", action="store_true")
    ap.add_argument("--id", type=int, default=14, help="gripper_id (défaut 14)")
    args = ap.parse_args()

    g = GripperClient()
    try:
        if args.calib:
            print(g.send({"action": "pro_gripper_calib", "gripper_id": args.id})); time.sleep(GAP)
        if args.get:
            print(g.send({"action": "get_pro_gripper_angle", "gripper_id": args.id})); return
        if args.do_open:
            print(g.send({"action": "pro_gripper_open", "gripper_id": args.id})); time.sleep(GAP)
        if args.do_close:
            print(g.send({"action": "pro_gripper_close", "gripper_id": args.id})); time.sleep(GAP)
        if args.angle is not None:
            print(g.send({"action": "pro_gripper_angle", "angle": args.angle, "gripper_id": args.id})); time.sleep(GAP)
        if args.cycles:
            for i in range(args.cycles):
                print(f"[cycle {i+1}] {g.send({'action':'pro_gripper_open','gripper_id':args.id})}"); time.sleep(GAP)
                print(f"[cycle {i+1}] {g.send({'action':'pro_gripper_close','gripper_id':args.id})}"); time.sleep(GAP)
    finally:
        g.close()


if __name__ == "__main__":
    main()
