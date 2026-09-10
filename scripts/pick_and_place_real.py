#!/usr/bin/env python3
"""Pick-and-place RÉEL (sans vision) — À LANCER SUR LE PC TOUR.

Pipeline TCP pur : PC → bridge (gripper_bridge.py sur le Pi :5005) → robot + gripper.

Apprentissage en ANGLES ARTICULAIRES (get_angles/send_angles) : reproduction
EXACTE de la config apprise, sans IK ni ambiguïté d'orientation cartésienne
(get_coords en bras-relâché fait sauter les angles d'Euler -> non fiable).

On apprend 4 poses :
    pick_approach  : au-dessus de l'objet, pince ouverte
    pick           : sur l'objet, prêt à fermer
    place_approach : au-dessus du point de dépose
    place          : au point de dépose

Workflow :
  1) APPRENDRE (bras déplacé à la main) :
       python3 pick_and_place_real.py --gripper-open
       python3 pick_and_place_real.py --release            # relâche les servos, TIENS le bras
       # -> place le bras, puis pour chaque pose :
       python3 pick_and_place_real.py --capture pick_approach
       python3 pick_and_place_real.py --capture pick
       python3 pick_and_place_real.py --capture place_approach
       python3 pick_and_place_real.py --capture place
       python3 pick_and_place_real.py --power-on           # re-tend les servos

  2) REJOUER :
       python3 pick_and_place_real.py --run
"""
import argparse
import json
import re
import socket
import time
from pathlib import Path

POS_FILE = Path(__file__).resolve().parent / 'pick_place_positions.json'
GRIPPER_ID = 14
GRIP_GAP = 1.6
WAYPOINTS = ['pick_approach', 'pick', 'place_approach', 'place']
STATUS_TXT = {0: "en mouvement", 1: "rien saisi", 2: "objet saisi", 3: "objet tombé"}


class Bridge:
    def __init__(self, host, port=5005):
        self.sock = socket.create_connection((host, port), timeout=8)
        self.sock.settimeout(8)
        self._buf = ""
        self._last_grip = 0.0

    def send(self, obj) -> str:
        self.sock.sendall((json.dumps(obj) + "\n").encode())
        while "\n" not in self._buf:
            chunk = self.sock.recv(4096).decode()
            if not chunk:
                break
            self._buf += chunk
        line, _, self._buf = self._buf.partition("\n")
        return line.strip()

    def grip(self, obj) -> str:
        dt = time.time() - self._last_grip
        if dt < GRIP_GAP:
            time.sleep(GRIP_GAP - dt)
        r = self.send(obj)
        self._last_grip = time.time()
        return r

    def get_angles(self):
        r = self.send({'action': 'get_angles'})
        if 'ANGLES:' in r:
            return json.loads(r.split('ANGLES:', 1)[1].strip())
        raise RuntimeError(f'get_angles a répondu : {r}')

    def gripper_status(self):
        r = self.grip({'action': 'get_pro_gripper_status', 'gripper_id': GRIPPER_ID})
        m = re.search(r'-?\d+', r.split('PRO_GRIPPER_STATUS:', 1)[-1])
        return int(m.group()) if m else None


def load_positions() -> dict:
    return json.loads(POS_FILE.read_text()) if POS_FILE.is_file() else {}


def save_positions(pos: dict):
    POS_FILE.write_text(json.dumps(pos, indent=2))


def confirm(msg: str) -> bool:
    return input(f'{msg} [o/N] ').strip().lower() in ('o', 'oui', 'y', 'yes')


def move_angles(b: Bridge, angles, speed, settle=3.0):
    print(f'  → send_angles {[round(a, 1) for a in angles]} v={speed}')
    print('   ', b.send({'action': 'send_angles', 'angles': angles, 'speed': speed}))
    time.sleep(settle)


def run_sequence(b: Bridge, pos: dict, speed: int, grasp_angle: int, open_angle: int,
                 step: bool = False):
    missing = [w for w in WAYPOINTS if w not in pos]
    if missing:
        raise SystemExit(f'poses manquantes : {missing} — fais --capture d\'abord')
    grasp_angle = int(pos.get('grasp_angle', grasp_angle))  # angle appris > défaut

    def gate(msg):
        """En mode --step : attend le 'o' avant chaque action. Sinon passe."""
        if step and not confirm(f'    ⏸  {msg} — continuer ?'):
            raise SystemExit('interrompu par l\'utilisateur.')

    print('\n=== SÉQUENCE PICK-AND-PLACE (angles articulaires) ===')
    for w in WAYPOINTS:
        print(f'  {w:15s}: {[round(a,1) for a in pos[w]]}')
    print(f'  vitesse {speed} | pince fermée={grasp_angle} ouverte={open_angle}'
          f'{"  | MODE PAS-À-PAS" if step else ""}')
    if not confirm('\n⚠️  Le bras VA BOUGER. Zone dégagée, main sur l\'arrêt ? Lancer ?'):
        print('annulé.'); return

    print('\n[1] pince ouverte'); print('   ', b.grip(
        {'action': 'pro_gripper_angle', 'angle': open_angle, 'gripper_id': GRIPPER_ID}))
    gate('approcher au-dessus du pick'); print('[2] approche pick')
    move_angles(b, pos['pick_approach'], speed)
    gate('DESCENDRE sur l\'objet');      print('[3] descente pick')
    move_angles(b, pos['pick'], max(20, speed // 2))
    gate('FERMER la pince sur l\'objet'); print('[4] fermeture pince')
    print('   ', b.grip({'action': 'pro_gripper_angle', 'angle': grasp_angle, 'gripper_id': GRIPPER_ID}))
    st = b.gripper_status()
    print(f'    statut gripper : {st} ({STATUS_TXT.get(st, "?")})')
    if st != 2 and not confirm('    ⚠️  objet NON détecté (statut≠2). Continuer ?'):
        print('    interrompu — remonte à vide.'); move_angles(b, pos['pick_approach'], speed); return
    gate('SOULEVER');        print('[5] soulève');        move_angles(b, pos['pick_approach'], speed)
    gate('aller au place');  print('[6] approche place'); move_angles(b, pos['place_approach'], speed)
    gate('DESCENDRE place'); print('[7] descente place'); move_angles(b, pos['place'], max(20, speed // 2))
    gate('OUVRIR (déposer)'); print('[8] ouvre pince'); print('   ', b.grip(
        {'action': 'pro_gripper_angle', 'angle': open_angle, 'gripper_id': GRIPPER_ID}))
    print('[9] retrait');        move_angles(b, pos['place_approach'], speed)
    print('[10] home');          print('   ', b.send({'action': 'go_home'}))
    print('\n✅ terminé.')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', default='10.10.0.221')
    ap.add_argument('--speed', type=int, default=30, help='vitesse robot 1-100 (défaut 30)')
    ap.add_argument('--grasp-angle', type=int, default=20, help='angle pince fermée sur objet')
    ap.add_argument('--open-angle', type=int, default=100, help='angle pince ouverte')
    ap.add_argument('--release', action='store_true', help='relâche les servos (déplacer à la main)')
    ap.add_argument('--power-on', action='store_true', help='re-tend les servos')
    ap.add_argument('--gripper-open', action='store_true', help='ouvre la pince')
    ap.add_argument('--gripper-close', action='store_true', help='ferme la pince')
    ap.add_argument('--capture', choices=WAYPOINTS, help='capture la pose ARTICULAIRE courante')
    ap.add_argument('--grip', type=int, metavar='ANGLE',
                    help='ferme la pince à ANGLE et affiche le statut (pour trouver le bon serrage)')
    ap.add_argument('--set-grasp', type=int, metavar='ANGLE',
                    help='enregistre l\'angle de fermeture à utiliser au --run')
    ap.add_argument('--goto', choices=WAYPOINTS, help='amène le bras à une pose apprise')
    ap.add_argument('--show', action='store_true', help='affiche les poses apprises')
    ap.add_argument('--run', action='store_true', help='exécute la séquence pick-and-place')
    ap.add_argument('--step', action='store_true', help='--run pas-à-pas (confirme chaque mouvement)')
    args = ap.parse_args()

    b = Bridge(args.host)

    if args.release:
        if confirm('⚠️  Relâcher les servos ? TIENS le bras (il devient mou).'):
            print(b.send({'action': 'power_off'}))
        return
    if args.power_on:
        print(b.send({'action': 'power_on'})); return
    if args.gripper_open:
        print(b.grip({'action': 'pro_gripper_angle', 'angle': args.open_angle,
                      'gripper_id': GRIPPER_ID})); return
    if args.gripper_close:
        print(b.grip({'action': 'pro_gripper_angle', 'angle': args.grasp_angle,
                      'gripper_id': GRIPPER_ID})); return
    if args.capture:
        angles = b.get_angles()
        pos = load_positions(); pos[args.capture] = angles; save_positions(pos)
        print(f'{args.capture} capturé : {[round(a, 1) for a in angles]}')
        return
    if args.grip is not None:
        print('   ', b.grip({'action': 'pro_gripper_angle', 'angle': args.grip,
                             'gripper_id': GRIPPER_ID}))
        st = b.gripper_status()
        print(f'angle {args.grip} → statut {st} ({STATUS_TXT.get(st, "?")})'
              + ('  ✅ OBJET SAISI' if st == 2 else '  (pas de prise)'))
        return
    if args.set_grasp is not None:
        pos = load_positions(); pos['grasp_angle'] = args.set_grasp; save_positions(pos)
        print(f'angle de fermeture enregistré : {args.set_grasp} (utilisé au --run)')
        return
    if args.goto:
        pos = load_positions()
        if args.goto not in pos:
            raise SystemExit(f'{args.goto} non capturé')
        if confirm(f'⚠️  amener le bras à "{args.goto}" ? Zone dégagée ?'):
            move_angles(b, pos[args.goto], args.speed)
        return
    if args.show:
        pos = load_positions()
        for w in WAYPOINTS:
            print(f'{w:15s}: {pos.get(w, "— non capturé")}')
        return
    if args.run:
        run_sequence(b, load_positions(), args.speed, args.grasp_angle, args.open_angle,
                     step=args.step)
        return

    ap.print_help()


if __name__ == '__main__':
    main()
