"""Un cycle saisie -> bac pour la pince sur support (axe J6), boucle ouverte.

    .venv/bin/python scripts/cycle_pince_axe_j6.py <piece_x> <piece_y> <hauteur_piece> <bac_x> <bac_y>

Geometrie mesuree le 30/09 sur le cube rouge : centre de prise = bride +
R @ [-24, 0, 146], R = axe J6 vers le bas ; affaissement ~10 mm en Z.
S'arrete au premier ecart (codeurs, statut pince, IK).
"""
import itertools
import json
import socket
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import diff_ik                                                       # noqa: E402
import pick_fsm as f                                                 # noqa: E402

PI = ('10.10.0.219', 5005)
T_OUTIL = np.array([-24.0, 0.0, 146.0])
R0 = np.column_stack([[1, 0, 0], [0, -1, 0], [0, 0, -1]]).astype(float)
AFFAISSEMENT = 10.0
Z_SURVOL_BRIDE = 260.0
CORRECTION_BAC = np.array([19.0, -13.0])     # ecart mesure au depot du cube rouge
POSE_DEGAGEE = [164.53, 19.24, -32.95, 10.63, 2.37, -38.93]
ECART_CODEURS_MAX = 2.8

sock = socket.create_connection(PI, timeout=5)
sock.settimeout(5)


def cmd(d):
    sock.sendall((json.dumps(d) + '\n').encode())
    return sock.recv(4096).decode().strip()


def angles():
    return json.loads(cmd({'action': 'get_angles'}).split(':', 1)[1])


ESSAI = '--essai' in sys.argv
_courant = [None]


def hauteur_min(a, b):
    m = 1e9
    for s in np.linspace(0, 1, 41):
        qq = np.asarray(a) + s * (np.asarray(b) - np.asarray(a))
        p, R = diff_ik.fk_pose(qq)
        pos, _ = diff_ik.forward_kinematics(np.radians(qq))
        pointes = [p + R @ np.array(v) for v in ([-24, 0, 170], [-24, 80, 160], [-24, -80, 160])]
        m = min(m, min(t[2] for t in pointes),
                min(np.asarray(v)[2] * 1000 for k, v in pos.items()
                    if 'link' in k and k != 'mycobot320_link1'))
    return m


def va(q, vitesse, nom, plancher=None, seuil=None):
    q = [round(float(x), 2) for x in q]
    depart = _courant[0] if _courant[0] is not None else angles()
    hm = hauteur_min(depart, q)
    if plancher is not None and hm < plancher:
        raise SystemExit(f'{nom} : trajet descend a {hm:.0f} mm < {plancher:.0f} — ARRET')
    _courant[0] = np.array(q)
    if ESSAI:
        print(f'  [essai] {nom:22s} {q}  hauteur min trajet {hm:.0f}', flush=True)
        return np.array(q)
    if not np.all((np.array(q) >= f.LIMITES[:, 0]) & (np.array(q) <= f.LIMITES[:, 1])):
        raise SystemExit(f'{nom} : hors limites {q}')
    print(f'  {nom:22s} hauteur min trajet {hm:.0f}', flush=True)
    cmd({'action': 'send_angles', 'angles': q, 'speed': vitesse})
    t0, prec = time.time(), None
    while time.time() - t0 < 30:
        time.sleep(0.15 if seuil else 0.5)
        a = angles()
        if seuil and max(abs(x - y) for x, y in zip(a, q)) < seuil:
            print(f'  {nom:22s} enchaine (ecart < {seuil})', flush=True)
            return np.array(q)
        if prec and max(abs(x - y) for x, y in zip(a, prec)) < 0.3:
            break
        prec = a
    ecart = max(abs(x - y) for x, y in zip(a, q))
    print(f'  {nom:22s} lu {a}  ecart {ecart:.2f}', flush=True)
    if ecart > ECART_CODEURS_MAX:
        raise SystemExit(f'{nom} : ecart codeurs {ecart:.2f} deg — contact ou refus, ARRET')
    return np.array(q)


def bride_pour(centre, R):
    return np.asarray(centre, float) - R @ T_OUTIL


def suit(q, cible_bride, R, pas=5):
    """Colonne continue : resolution par petits pas depuis q."""
    p0, _ = diff_ik.fk_pose(q)
    for s in np.linspace(0, 1, pas + 1)[1:]:
        q = diff_ik.solve_pose(q.copy(), p0 + (cible_bride - p0) * s, R, rot_weight=400.0,
                               max_joint_step_deg=3.0, iterations=300)
    p, _ = diff_ik.fk_pose(q)
    if np.linalg.norm(p - cible_bride) > 1.0 or diff_ik.orientation_error_deg(q, R) > 1.0:
        raise SystemExit(f'IK : {cible_bride.round(1)} non atteint ({p.round(1)})')
    return q


def ik_global(cible_bride, R, az):
    for j2, j3, j4 in itertools.product((-60, -20, 20), (-90, -40, 20, 60), (-40, 0, 40)):
        q = diff_ik.solve_pose(np.array([az, j2, j3, j4, 90.0, -45.0]), cible_bride, R,
                               rot_weight=400.0, max_joint_step_deg=3.0, iterations=300)
        p, _ = diff_ik.fk_pose(q)
        if (np.linalg.norm(p - cible_bride) < 1 and diff_ik.orientation_error_deg(q, R) < 1
                and q[2] < 0 and np.all((q >= f.LIMITES[:, 0]) & (q <= f.LIMITES[:, 1]))):
            return q
    raise SystemExit(f'IK globale : aucune solution coude haut pour {cible_bride.round(1)}')


def rz(a):
    a = np.radians(a)
    return np.array([[np.cos(a), -np.sin(a), 0], [np.sin(a), np.cos(a), 0], [0, 0, 1]])


def pose_largage(bac_xy, z_centre):
    az = np.degrees(np.arctan2(bac_xy[1], bac_xy[0]))
    tang = np.array([-np.sin(np.radians(az)), np.cos(np.radians(az)), 0.0])
    for theta in (0, 15, 30, 45):
        for roulis in (-120, -90, -150, -60, 180, 60, 90, 120, 150, 0, 30, -30):
            M, _ = cv2.Rodrigues(tang * np.radians(-theta))
            R = M @ R0 @ rz(roulis)
            fl = bride_pour([bac_xy[0], bac_xy[1], z_centre], R)
            for j2, j3 in itertools.product((-60, -20, 20), (-40, -80, 20, 60)):
                q = diff_ik.solve_pose(np.array([az - 5, j2, j3, 0, 60, 0.0]), fl, R,
                                       rot_weight=400.0, max_joint_step_deg=3.0, iterations=300)
                p, _ = diff_ik.fk_pose(q)
                if (np.linalg.norm(p - fl) < 1 and diff_ik.orientation_error_deg(q, R) < 1
                        and q[2] < 0
                        and np.all((q >= f.LIMITES[:, 0]) & (q <= f.LIMITES[:, 1]))):
                    return q, R, theta
    raise SystemExit('aucune pose de largage coude haut')


def main():
    px, py, h, bx, by = map(float, sys.argv[1:6])
    az = float(np.degrees(np.arctan2(py, px)))
    zc = h / 2.0
    q = np.array(angles())

    print(f'--- piece ({px:.0f}, {py:.0f}) h {h:.0f}, bac ({bx:.0f}, {by:.0f})', flush=True)
    survol = bride_pour([px, py, 0], R0)
    survol[2] = Z_SURVOL_BRIDE
    q_survol = ik_global(survol, R0, az)
    q = va(q_survol, 20, 'survol', plancher=80)

    z_prise_bride = zc + T_OUTIL[2] + AFFAISSEMENT
    cible = bride_pour([px, py, 0], R0)
    cible[2] = z_prise_bride
    milieu = cible.copy()
    milieu[2] = (Z_SURVOL_BRIDE + z_prise_bride) / 2.0
    q_mi = suit(q_survol, milieu, R0, pas=6)
    q_prise = suit(q_mi, cible, R0, pas=6)
    derive = max(np.hypot(*(diff_ik.fk_pose(a_ + s_ * (b_ - a_))[0][:2] - cible[:2]))
                 for a_, b_ in ((q_survol, q_mi), (q_mi, q_prise)) for s_ in np.linspace(0, 1, 21))
    print(f'  descente d un seul trait : derive XY max {derive:.1f} mm', flush=True)
    if derive > 6.0:
        raise SystemExit('descente articulaire trop courbe — ARRET')
    visee = np.array([bx, by]) + CORRECTION_BAC
    q_bac, R_bac, theta = pose_largage(visee, 75.0)
    p_bac, _ = diff_ik.fk_pose(q_bac)
    q_haut = diff_ik.solve_pose(q_bac.copy(), p_bac + [0, 0, 60], R_bac, rot_weight=400.0,
                                max_joint_step_deg=3.0, iterations=400)
    releve = q_haut.copy()
    releve[1], releve[2] = -10.0, -40.0
    print(f'  largage incline {theta} deg — tout est calcule, on enchaine', flush=True)

    va(q_mi, 12, 'descente', seuil=4.0)
    q = va(q_prise, 12, 'descente fin')
    if ESSAI:
        statut = 'STATUS: 2 (essai)'
    else:
        print('  fermeture :', cmd({'action': 'pro_gripper_close'}), flush=True)
        statut = ''
        for _ in range(15):
            time.sleep(0.2)
            statut = cmd({'action': 'get_pro_gripper_status'})
            if 'STATUS: 2' in statut or 'STATUS: 1' in statut:
                break
    print('  ', statut, flush=True)
    if 'STATUS: 2' not in statut:
        raise SystemExit('prise NON confirmee — pince laissee fermee, bras a la prise, ARRET')

    va(q_survol, 25, 'remontee', seuil=10.0)
    va(q_haut, 25, 'vers le bac', plancher=60, seuil=6.0)
    va(q_bac, 15, 'largage')
    statut = 'STATUS: 2' if ESSAI else cmd({'action': 'get_pro_gripper_status'})
    if 'STATUS: 2' not in statut:
        print('  ATTENTION objet perdu en route :', statut, flush=True)
    if not ESSAI:
        print('  ouverture :', cmd({'action': 'pro_gripper_open'}), flush=True)
        time.sleep(0.8)
        print('  ', cmd({'action': 'get_pro_gripper_status'}), flush=True)
    va(q_haut, 25, 'degagement', seuil=8.0)
    va(releve, 25, 'releve', seuil=10.0)
    va(POSE_DEGAGEE, 25, 'pose degagee')
    print('--- cycle termine', flush=True)


try:
    main()
finally:
    sock.close()
