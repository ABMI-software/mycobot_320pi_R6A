#!/usr/bin/env python3
"""Pick-and-place vision-guidé (look-then-move) — moitié CONTRÔLE.

Prend un objet localisé (x, y, z) dans le repère BASE (mètres) et l'exécute par
ANGLES calculés sur PC via le bridge TCP : approche top-down -> descente ->
serrage pince -> vérif statut -> soulève -> dépose -> ouvre -> retrait.

La moitié PERCEPTION (YOLO + extrinsèques DREAM live + triangulation 2 caméras,
cf. vision/multiview_localizer.py) fournira ce (x, y, z). En attendant, on le
passe en CLI (`--object x y z`) pour valider la séquence en `--dry-run` sans
robot, puis pour de vrai une fois le bridge relancé.

À lancer (pas de torch ici -> n'importe quel python avec socket) :
    python3 scripts/pick_and_place_vision.py --pi-host 10.10.0.224 \
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
FLANGE_ABOVE = 110.0   # la bride est ~110mm au-dessus du bout de
                       # doigt -> on vise l'objet + cette longueur de gripper.
# Orientation top-down de la bride (pince vers le bas). À AFFINER sur le vrai
# robot — dépend du montage du gripper. rx,ry,rz en degrés.
TOP_DOWN_ORI = [180.0, 0.0, 0.0]

OPEN_ANGLE   = 100     # pince ouverte (pro gripper 0-100)
GRASP_ANGLE  = 20      # pince serrée
SETTLE       = 2.5     # s d'attente après une commande de mouvement


def plan(xyz_m, ori, flange_above=FLANGE_ABOVE, approach_h=APPROACH_H):
    """Objet (x,y,z) base en mètres -> (approche, saisie) en coords bride mm+deg."""
    x, y, z = (c * 1000.0 for c in xyz_m)      # m -> mm
    pick_z = z + float(flange_above)
    pick     = [x, y, pick_z] + list(ori)
    approach = [x, y, pick_z + float(approach_h)] + list(ori)
    return approach, pick


def read_coords(b):
    """Lit la pose cartésienne actuelle [x,y,z,rx,ry,rz] ou None."""
    import json
    r = b.send({'action': 'get_coords'})
    if 'COORDS:' in r:
        try:
            coords = json.loads(r.split('COORDS:', 1)[1].strip())
            return coords if isinstance(coords, list) and len(coords) >= 6 else None
        except Exception:
            return None
    return None


def _round(c):
    return [round(v, 1) for v in c]


def _verifie(reponse, quoi):
    """Un refus du bridge arrete la sequence au lieu de la laisser continuer.

    La reponse etait imprimee puis ignoree : les etapes suivantes partaient
    sur une pose que le robot n'a jamais atteinte, et le cycle se deroulait
    a vide. C'est d'autant plus couteux ici que `send_coords` echoue
    normalement EN SILENCE en rendant `OK` (247,8 mm d'erreur mesures le
    20/08) : un `ERROR:` explicite est la seule fois ou il dit la verite, et
    c'est precisement ce qu'on jetait.
    """
    texte = '' if reponse is None else str(reponse).strip()
    if texte.upper().startswith('ERROR'):
        raise RuntimeError(f'commande {quoi} refusée par le bridge : {texte}')
    return reponse


def move(b: Bridge, coords, speed, mode, dry, settle=SETTLE):
    print(f'  → send_coords {_round(coords)} v={speed} mode={mode}')
    if dry:
        return
    response = b.send({'action': 'send_coords', 'coords': coords,
                       'speed': speed, 'mode': mode})
    print('   ', response)
    if not response.startswith('OK:'):
        raise RuntimeError(f'commande cartésienne refusée : {response}')
    time.sleep(settle)


def move_angles(b: Bridge, angles, speed, dry, settle=SETTLE):
    print(f'  → send_angles {_round(angles)} v={speed}')
    if dry:
        return
    response = b.send({'action': 'send_angles', 'angles': list(angles), 'speed': speed})
    print('   ', response)
    if not response.startswith('OK:'):
        raise RuntimeError(f'commande articulaire refusée : {response}')
    time.sleep(settle)


def grip(b: Bridge, angle, dry):
    print(f'  → pince angle={angle}')
    if dry:
        return
    response = b.grip({'action': 'pro_gripper_angle', 'angle': angle,
                       'gripper_id': GRIPPER_ID})
    print('   ', response)
    if not response.startswith('OK:'):
        raise RuntimeError(f'commande pince refusée : {response}')


def gate(auto, msg):
    if auto:
        print(f'[auto] {msg}')
        return True
    return input(f'{msg} — ENTER pour continuer, q pour annuler : ').strip().lower() != 'q'


def run(b, obj_xyz, place_xyz, speed, mode, dry, auto, keep_ori=False, ori=None,
        relocalize=None, adaptive_tolerance_m=0.015, max_replans=4,
        handover_angles=None, release_ready=None, grasp_angle=GRASP_ANGLE,
        open_angle=OPEN_ANGLE, flange_above=FLANGE_ABOVE,
        approach_h=APPROACH_H, joint_controller=None):
    """Exécute un pick-and-place, avec correction visuelle optionnelle.

    ``relocalize`` doit retourner une position objet ``[x, y, z]`` en mètres.
    Elle est rappelée après chaque approche. Tant que l'objet s'est déplacé de
    plus de ``adaptive_tolerance_m``, l'approche est recalculée. La descente ne
    commence donc qu'après une mesure récente et stable.
    """
    ori = list(TOP_DOWN_ORI) if ori is None else list(ori)
    if not dry and b is not None:
        cur = read_coords(b)
        if cur:
            print(f'  position actuelle : {[round(c, 1) for c in cur]}')
            if keep_ori and len(cur) >= 6:
                ori = cur[3:6]
                print(f'  → garde l\'orientation atteignable actuelle {[round(o, 1) for o in ori]}')

    obj_xyz = [float(v) for v in obj_xyz]
    appr_pick, pick = plan(obj_xyz, ori, flange_above, approach_h)
    appr_place, place = plan(place_xyz, ori, flange_above, approach_h)

    print('\n=== PLAN ===')
    print(f'  objet  base(m) : {obj_xyz}   -> saisie   {_round(pick)}')
    print(f'  dépose base(m) : {place_xyz} -> dépose   {_round(place)}')
    print('============\n')

    def controlled_move(coords, context, descend=False):
        """Route tout mouvement cartésien par l'IK PC lorsqu'elle est fournie."""
        if joint_controller is None:
            return move(b, coords, speed, mode, dry)
        if descend:
            return joint_controller.descend_vertical(coords[:3])
        return joint_controller.move_pose(coords[:3], context=context)

    if not gate(auto, '[1] ouvrir la pince + approcher l\'objet'):
        return False
    grip(b, open_angle, dry)
    controlled_move(appr_pick, 'approche pick')

    if relocalize is not None:
        for attempt in range(max_replans + 1):
            fresh = relocalize()
            if fresh is None:
                raise RuntimeError(
                    'cible invisible après approche — descente interdite (aucun mouvement bas)')
            fresh = [float(v) for v in fresh]
            shift = ((fresh[0] - obj_xyz[0]) ** 2 +
                     (fresh[1] - obj_xyz[1]) ** 2) ** 0.5
            print(f'  👁️  contrôle cible frais : déplacement XY={shift*1000:.1f} mm')
            obj_xyz = fresh
            appr_pick, pick = plan(obj_xyz, ori, flange_above, approach_h)
            if shift <= adaptive_tolerance_m:
                print('  ✅ cible stable — descente autorisée')
                break
            if attempt >= max_replans:
                raise RuntimeError(
                    f'cible encore mobile après {max_replans} corrections — descente annulée')
            print(f'  🔄 cible déplacée — nouvelle approche {_round(appr_pick)}')
            controlled_move(appr_pick, 'correction approche pick')

    if not gate(auto, '[2] DESCENDRE sur l\'objet'):
        return False
    controlled_move(pick, 'descente pick', descend=True)

    if not gate(auto, '[3] SERRER'):
        return False
    grip(b, grasp_angle, dry)
    if not dry:
        st = b.gripper_status()
        print(f'    statut pince : {st} ({STATUS_TXT.get(st, "?")})')
        if st != 2:
            print('    ⚠️ rien saisi — on remonte à vide.')
            controlled_move(appr_pick, 'retrait après prise vide')
            return False

    destination = 'la remise en main' if handover_angles else 'la dépose'
    if not gate(auto, f'[4] SOULEVER + aller à {destination}'):
        return False
    controlled_move(appr_pick, 'levée pick')

    if handover_angles:
        approach_angles, present_angles = handover_angles
        print('  🤝 approche de la pose de remise apprise')
        move_angles(b, approach_angles, speed, dry)
        move_angles(b, present_angles, max(10, speed // 2), dry)
        if not dry:
            if release_ready is not None:
                print('  👁️  attente de la main dans la zone de remise…')
                if not release_ready():
                    print('  🛑 main non validée — objet conservé, retour à l\'approche')
                    move_angles(b, approach_angles, speed, dry)
                    return False
            elif not gate(False, '[5] Main prête sous l\'objet ?'):
                move_angles(b, approach_angles, speed, dry)
                return False
        print('[5] main validée — ouverture de la pince')
        grip(b, open_angle, dry)
        move_angles(b, approach_angles, speed, dry)
        print('\n✅ objet remis en main.')
        return True

    controlled_move(appr_place, 'approche dépose')
    controlled_move(place, 'descente dépose', descend=True)

    if not gate(auto, '[5] OUVRIR (déposer) + retrait'):
        return False
    grip(b, open_angle, dry)
    controlled_move(appr_place, 'retrait dépose')
    print('\n✅ pick-and-place terminé.')
    return True


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--pi-host', default='10.10.0.224')
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
