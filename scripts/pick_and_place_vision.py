#!/usr/bin/env python3
"""Pick-and-place vision-guidé (look-then-move) — moitié CONTRÔLE.

Prend un objet localisé (x, y, z) dans le repère BASE (mètres) et l'exécute en
CARTÉSIEN sur le vrai robot via le bridge TCP : approche top-down -> descente ->
serrage pince -> vérif statut -> soulève -> dépose -> ouvre -> retrait.

La moitié PERCEPTION (YOLO + extrinsèques DREAM live + triangulation 2 caméras,
cf. vision/multiview_localizer.py) fournira ce (x, y, z). En attendant, on le
passe en CLI (`--object x y z`) pour valider la séquence en `--dry-run` sans
robot, puis pour de vrai une fois le bridge relancé.

À lancer (pas de torch ici -> n'importe quel python avec socket) :
    python3 scripts/pick_and_place_vision.py --pi-host 10.10.0.221 \
        --object 0.15 -0.08 0.02 --place 0.20 0.15 0.05 --dry-run
"""
import argparse
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from pick_and_place_real import Bridge, GRIPPER_ID, STATUS_TXT  # noqa: E402

# --- Géométrie de saisie (mm / deg, repère base) ---
APPROACH_H   = 80.0    # hauteur d'approche AU-DESSUS du point de saisie (mm)
FLANGE_ABOVE = 110.0   # la bride (send_coords) est ~110mm au-dessus du bout de
                       # doigt -> on vise l'objet + cette longueur de gripper.
# Orientation top-down de la bride (pince vers le bas). À AFFINER sur le vrai
# robot — dépend du montage du gripper. rx,ry,rz en degrés.
TOP_DOWN_ORI = [180.0, 0.0, 0.0]

OPEN_ANGLE   = 100     # pince ouverte (pro gripper 0-100)
GRASP_ANGLE  = 20      # pince serrée
SETTLE       = 2.5     # s d'attente après un send_coords


def plan(xyz_m, ori):
    """Objet (x,y,z) base en mètres -> (approche, saisie) en coords bride mm+deg."""
    x, y, z = (c * 1000.0 for c in xyz_m)      # m -> mm
    pick_z = z + FLANGE_ABOVE
    pick     = [x, y, pick_z] + list(ori)
    approach = [x, y, pick_z + APPROACH_H] + list(ori)
    return approach, pick


def read_coords(b):
    """Lit la pose cartésienne actuelle [x,y,z,rx,ry,rz] ou None."""
    import json
    r = b.send({'action': 'get_coords'})
    if 'COORDS:' in r:
        try:
            return json.loads(r.split('COORDS:', 1)[1].strip())
        except Exception:
            return None
    return None


def _round(c):
    return [round(v, 1) for v in c]


def move(b: Bridge, coords, speed, mode, dry, settle=SETTLE):
    print(f'  → send_coords {_round(coords)} v={speed} mode={mode}')
    if dry:
        return
    print('   ', b.send({'action': 'send_coords', 'coords': coords,
                         'speed': speed, 'mode': mode}))
    time.sleep(settle)


def grip(b: Bridge, angle, dry):
    print(f'  → pince angle={angle}')
    if dry:
        return
    print('   ', b.grip({'action': 'pro_gripper_angle', 'angle': angle,
                        'gripper_id': GRIPPER_ID}))


def gate(auto, msg):
    if auto:
        print(f'[auto] {msg}')
        return True
    return input(f'{msg} — ENTER pour continuer, q pour annuler : ').strip().lower() != 'q'


def run(b, obj_xyz, place_xyz, speed, mode, dry, auto, keep_ori=False, ori=None):
    ori = list(TOP_DOWN_ORI) if ori is None else list(ori)
    if not dry and b is not None:
        cur = read_coords(b)
        if cur:
            print(f'  position actuelle : {[round(c, 1) for c in cur]}')
            if keep_ori and len(cur) >= 6:
                ori = cur[3:6]
                print(f'  → garde l\'orientation atteignable actuelle {[round(o, 1) for o in ori]}')

    appr_pick, pick = plan(obj_xyz, ori)
    appr_place, place = plan(place_xyz, ori)

    print('\n=== PLAN ===')
    print(f'  objet  base(m) : {obj_xyz}   -> saisie   {_round(pick)}')
    print(f'  dépose base(m) : {place_xyz} -> dépose   {_round(place)}')
    print('============\n')

    if not gate(auto, '[1] ouvrir la pince + approcher l\'objet'):
        return
    grip(b, OPEN_ANGLE, dry)
    move(b, appr_pick, speed, mode, dry)

    if not gate(auto, '[2] DESCENDRE sur l\'objet'):
        return
    move(b, pick, speed, mode, dry)

    if not gate(auto, '[3] SERRER'):
        return
    grip(b, GRASP_ANGLE, dry)
    if not dry:
        st = b.gripper_status()
        print(f'    statut pince : {st} ({STATUS_TXT.get(st, "?")})')
        if st != 2:
            print('    ⚠️ rien saisi — on remonte à vide.')
            move(b, appr_pick, speed, mode, dry)
            return

    if not gate(auto, '[4] SOULEVER + aller à la dépose'):
        return
    move(b, appr_pick, speed, mode, dry)
    move(b, appr_place, speed, mode, dry)
    move(b, place, speed, mode, dry)

    if not gate(auto, '[5] OUVRIR (déposer) + retrait'):
        return
    grip(b, OPEN_ANGLE, dry)
    move(b, appr_place, speed, mode, dry)
    print('\n✅ pick-and-place terminé.')


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--pi-host', default='10.10.0.221')
    ap.add_argument('--object', nargs=3, type=float, required=True,
                    metavar=('X', 'Y', 'Z'), help='objet en mètres, repère base')
    ap.add_argument('--place', nargs=3, type=float, default=[0.20, 0.15, 0.05],
                    metavar=('X', 'Y', 'Z'), help='point de dépose (m, base)')
    ap.add_argument('--speed', type=int, default=40)
    ap.add_argument('--mode', type=int, default=1, choices=[0, 1],
                    help='0=angulaire, 1=linéaire (descente droite)')
    ap.add_argument('--auto', action='store_true', help='enchaîne sans confirmer chaque étape')
    ap.add_argument('--dry-run', action='store_true', help='affiche le plan sans commander le robot')
    args = ap.parse_args()

    b = None
    if not args.dry_run:
        b = Bridge(args.pi_host)
        print(f'  ✅ bridge {args.pi_host}:5005')

    run(b, args.object, args.place, args.speed, args.mode, args.dry_run, args.auto)


if __name__ == '__main__':
    main()
