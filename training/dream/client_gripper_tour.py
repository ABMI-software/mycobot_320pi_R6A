#!/usr/bin/env python3
"""Client de test gripper — À LANCER SUR LE PC TOUR (pas sur le Pi).

Se connecte au bridge (gripper_bridge.py, lancé sur le Pi) et envoie des
commandes JSON pour piloter le gripper Pro adaptatif.

Usage :
    python3 client_gripper_tour.py --host 192.168.223.59
    python3 client_gripper_tour.py --host 192.168.223.59 --angle 40
    python3 client_gripper_tour.py --host 192.168.223.59 --status
"""
import argparse
import json
import socket


def send(sock, obj_or_str):
    msg = json.dumps(obj_or_str) if isinstance(obj_or_str, dict) else obj_or_str
    sock.sendall((msg + "\n").encode())
    data = sock.recv(4096).decode().strip()
    print(f">>> {msg}\n<<< {data}\n")
    return data


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", required=True, help="IP du Pi (ex: 192.168.223.59)")
    ap.add_argument("--port", type=int, default=5005)
    ap.add_argument("--angle", type=int, default=None, help="aller à un angle 0-100")
    ap.add_argument("--status", action="store_true", help="lire seulement le statut")
    args = ap.parse_args()

    print(f"Connexion à {args.host}:{args.port} ...")
    s = socket.create_connection((args.host, args.port), timeout=10)
    s.settimeout(10)

    send(s, "ping")

    if args.status:
        send(s, {"action": "get_pro_gripper_status"})
        send(s, {"action": "get_pro_gripper_angle"})
        s.close()
        return

    if args.angle is not None:
        send(s, {"action": "pro_gripper_angle", "angle": args.angle})
        send(s, {"action": "get_pro_gripper_status"})
        s.close()
        return

    # Sans option : un cycle ouvre / ferme
    send(s, {"action": "pro_gripper_open"})
    send(s, {"action": "get_pro_gripper_status"})
    send(s, {"action": "pro_gripper_close"})
    send(s, {"action": "get_pro_gripper_status"})
    s.close()


if __name__ == "__main__":
    main()
