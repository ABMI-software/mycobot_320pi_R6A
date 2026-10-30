#!/usr/bin/env python3
"""Tri des pieces peintes en un geste par objet, sans dashboard.

    conda deactivate
    /usr/bin/python3 scripts/tri_fluide.py            # une piece, puis s'arrete
    /usr/bin/python3 scripts/tri_fluide.py --toutes   # jusqu'a planche vide
    /usr/bin/python3 scripts/tri_fluide.py --plan     # mesure et IK, AUCUN mouvement

Le dashboard recale l'objet pendant l'approche, avec la camera de dessus que le
bras masque justement a ce moment-la : il hesite, recale, ferme a vide, change
de cible. Ici tout est mesure UNE fois, bras degage en pose d'observation :

    yolo26 (2D) -> rayon de l'arducam -> plan de mi-hauteur de la piece (3D)

puis un seul geste : survol, descente, prise, remontee, survol du bac de SA
couleur, largage, retour. Les briques sont celles du FSM (`pick_fsm`) : choix de
la pose de prise, IK, pince fermee sur statut, largage verifie a l'arrivee.

Une prise a vide arrete le script : pas d'essais en rafale, chaque fermeture a
vide deplace la piece.
"""
import argparse
import json
import subprocess
import sys
import time
from pathlib import Path

import numpy as np

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE / 'scripts'))
import pick_fsm as fsm  # noqa: E402
import tri_couleur as tc  # noqa: E402
import yolo26_dashboard as y26  # noqa: E402

VENV = RACINE / '.venv' / 'bin' / 'python'
LOCALISE = RACINE / 'scripts' / 'yolo26_localise.py'
PIECES = {tc.NOM_PIECE[c]: f'bac_{c}' for c in tc.TEINTES}
# Le FSM lache a ~113 mm (rebord + 42 mm de garde) : la piece tombait de 8 cm
# dans le bac (13/10). On descend verticalement jusqu'a ce que le DESSOUS de la
# piece soit a GARDE_REBORD_MM au-dessus du rebord : les doigts, qui tiennent la
# piece a mi-hauteur, restent au-dessus du rebord, et la chute tombe a ~3 cm.
GARDE_REBORD_MM = 5.0
VITESSE_DEPOT = fsm.VITESSE_DESCENTE
VITESSE_POSE = 8
# Bac : ouverture 95 mm, fond 2 mm (dossier de fabrication, PROTOCOLE_YOLO_GAZEBO).
OUVERTURE_BAC_MM = 95.0
FOND_BAC_MM = 2.0
GARDE_FOND_MM = 6.0
# Estimations, NON mesurees : epaisseur d'un doigt, jeu d'ouverture au lacher,
# erreur XY residuelle apres correction.
DOIGT_MM = 10.0
JEU_OUVERTURE_MM = 6.0
MARGE_XY_MM = 4.0
JEU_OUVERTURE_DEG = 15
VITESSE_PINCE = 20
VITESSE_PINCE_NORMALE = 50
# Avant que l'arc ne tourne, la piece est levee VERTICALEMENT jusqu'a ce que son
# dessous passe la plus haute piece du banc (50 mm) de cette garde : une rotation
# commencee bas balaierait une piece voisine.
GARDE_PIECES_MM = 10.0
HAUTEUR_MAX_PIECE_MM = 50.0
VITESSE_ARC_LENTE = fsm.VITESSE_DESCENTE
VITESSE_ARC = 35
# Variantes du transfert en arc. `bosse` : surelevation au milieu de l'arc (mm) ;
# `ventre` : ecart radial au milieu (mm, + vers l'exterieur) ; `rotation` :
# exposant de l'avance en azimut (< 1 tourne tot, > 1 tourne tard) ; `points` :
# nombre de points de passage. 'actuelle' = remontee par paliers puis transfert
# direct du FSM, la trajectoire validee le 13/10.
VARIANTES = {
    'actuelle': None,
    'arc': dict(bosse=20.0, ventre=0.0, rotation=1.0, points=6, j6=90.0),
    # Mesure hors ligne du 13/10 sur les 4 pieces : bosse > 30 mm et tout ventre
    # vers l'exterieur sortent de l'IK (bacs a 315-350 mm, deja pres de la portee).
    # L'arc large se creuse donc vers le socle.
    'arc_haut': dict(bosse=30.0, ventre=0.0, rotation=1.0, points=6, j6=90.0),
    'arc_large': dict(bosse=20.0, ventre=-40.0, rotation=1.0, points=6, j6=90.0),
    'rotation_tot': dict(bosse=20.0, ventre=0.0, rotation=0.6, points=6, j6=90.0),
    'rotation_tard': dict(bosse=20.0, ventre=0.0, rotation=1.6, points=6, j6=90.0),
    'plus_de_points': dict(bosse=20.0, ventre=0.0, rotation=1.0, points=10, j6=90.0),
}


BACS_CONNUS = RACINE / 'results' / 'tri_fluide' / 'bacs_connus.json'
# Un bac vu SUR (>= 0,5) est memorise. Sous ce seuil, une detection n'est retenue
# que si elle tombe pres de cette memoire : le 13/10 le bac jaune, vide et bien
# visible, sortait a 0,13, a 1 mm de sa position sure, tandis qu'un bac_bleu
# fantome sortait a 0,15 a 690 mm.
ECART_BAC_FAIBLE_MM = 20.0


def localise():
    subprocess.run([str(VENV), str(LOCALISE)], cwd=str(RACINE),
                   capture_output=True, text=True, check=True)
    vue = json.loads((RACINE / 'results' / 'tri_fluide' / 'derniere_vue.json').read_text())
    vus = {nom: np.array(v['xy'], float) for nom, v in vue['surs'].items()}
    connus = json.loads(BACS_CONNUS.read_text()) if BACS_CONNUS.exists() else {}
    connus.update({n: v['xy'] for n, v in vue['surs'].items() if n.startswith('bac_')})
    BACS_CONNUS.write_text(json.dumps(connus))
    for nom, candidats in vue['faibles'].items():
        if nom in vus or nom not in connus:
            continue
        memoire = np.array(connus[nom], float)
        proche = min(candidats, key=lambda v: np.linalg.norm(np.array(v['xy']) - memoire))
        if np.linalg.norm(np.array(proche['xy']) - memoire) <= ECART_BAC_FAIBLE_MM:
            print(f'{nom} vu a {proche["conf"]:.2f} seulement, retenu : a moins de '
                  f'{ECART_BAC_FAIBLE_MM:.0f} mm de sa position sure')
            vus[nom] = np.array(proche['xy'], float)
    return vus


def contexte(pont):
    ctx = fsm.Contexte()
    ctx.z_rebord = y26.HAUTEUR_BAC
    for couleur, (_, poses) in tc.OBJET_PAR_COULEUR.items():
        mi = poses[0][2] / 2.0 + y26.MARGE_HAUTEUR_PRISE_MM
        fsm.Z_PRISE_PAR_CLASSE[tc.NOM_PIECE[couleur]] = (mi, mi)
    if pont:
        ctx.pont = fsm.Pont()
    return ctx


def journal(ctx):
    for ligne in ctx.journal:
        print('   ', ligne, flush=True)
    ctx.journal.clear()


def plan(ctx, piece, xy, bac_xy):
    ctx.classe_objet = piece
    prise = fsm.choisit_pose_prise(ctx, xy)
    depot = fsm.carton_atteignable(ctx, bac_xy)
    return prise, depot


def largeur_mm(piece):
    couleur = next(c for c, n in tc.NOM_PIECE.items() if n == piece)
    return max(tc.OBJET_PAR_COULEUR[couleur][1][0][:2])


def pose_au_fond(piece):
    """La piece peut-elle etre POSEE au fond, doigts dans le bac ?

    Simulation du 02/10 (PROTOCOLE_YOLO_GAZEBO) : doigts dans le bac, la paroi
    empechait la pince de s'ouvrir plus que le cube de 50 mm et l'objet repartait
    avec elle. On ne descend donc les doigts dans le bac que si la piece, les deux
    doigts, le jeu d'ouverture et l'erreur XY y tiennent.
    """
    return (largeur_mm(piece) + 2 * (DOIGT_MM + JEU_OUVERTURE_MM + MARGE_XY_MM)
            <= OUVERTURE_BAC_MM)


def va_lent(ctx, xyz, R, vitesse, nom):
    q = fsm.resout_ik(np.asarray(xyz, float), R)
    if q is None:
        print(f'   {nom} : pas d IK en ({xyz[0]:.0f} ; {xyz[1]:.0f} ; {xyz[2]:.0f})')
        return None
    return fsm.va_vers(ctx, q[0], vitesse=vitesse, nom=nom)


def depose_bas(ctx, piece, z_prise, R):
    """Poser la piece, pas la lacher : descente lente, correction, ouverture douce.

    1. descente verticale jusqu'a doigts juste au-dessus du rebord ;
    2. la pose LUE aux codeurs donne l'ecart reel (l'affaissement du bras laisse
       la pointe ~9 mm cote robot) : il est reporte sur la consigne finale ;
    3. descente finale tres lente, au fond si `pose_au_fond`, sinon doigts
       au-dessus du rebord ;
    4. ouverture PARTIELLE et lente (doigts dans le bac : une ouverture complete
       toucherait les parois et pousserait le bac), remontee, ouverture complete.
    """
    xy = np.asarray(ctx.carton_xy, float)
    au_fond = pose_au_fond(piece)
    z_mi = y26.HAUTEUR_BAC + GARDE_REBORD_MM + z_prise + 15.0
    z_fin = (FOND_BAC_MM + GARDE_FOND_MM + z_prise if au_fond
             else y26.HAUTEUR_BAC + GARDE_REBORD_MM + z_prise)
    sag = fsm.affaissement_largage(np.hypot(*xy))
    print(f'{piece} : descente lente dans le bac', flush=True)
    if va_lent(ctx, [xy[0], xy[1], z_mi + sag], R, VITESSE_DEPOT, 'descente dans le bac') is None:
        return False
    tp = fsm.pointe(ctx.pont.angles())
    ecart_xy, ecart_z = xy - tp[:2], z_mi - tp[2]
    if np.linalg.norm(ecart_xy) > fsm.ECART_LARGAGE_MAX:
        print(f'{piece} : pointe a {np.linalg.norm(ecart_xy):.0f} mm du bac — on ne pose pas')
        return False
    vise = xy + ecart_xy
    if va_lent(ctx, [vise[0], vise[1], z_fin + sag + ecart_z], R, VITESSE_POSE, 'pose') is None:
        return False
    tp = fsm.pointe(ctx.pont.angles())
    reste = float(np.linalg.norm(tp[:2] - xy))
    ctx.pont.envoie('set_pro_gripper_speed', speed=VITESSE_PINCE)
    if au_fond:
        ctx.pont.envoie('pro_gripper_angle', angle=min(100, ctx.pont.angle_pince() + JEU_OUVERTURE_DEG))
        time.sleep(1.5)
    else:
        fsm.ouvre_pince(ctx, force=True)
    ctx.en_main = ''
    print(f'{piece} : pose en ({tp[0]:.0f} ; {tp[1]:.0f}) a Z={tp[2]:.0f} mm, '
          f'{reste:.0f} mm du centre, {"au fond" if au_fond else "doigts au-dessus du rebord"}',
          flush=True)
    va_lent(ctx, [xy[0], xy[1], z_mi + sag], R, VITESSE_DEPOT, 'remontee lente')
    fsm.ouvre_pince(ctx, force=True)
    ctx.pont.envoie('set_pro_gripper_speed', speed=VITESSE_PINCE_NORMALE)
    va_lent(ctx, [xy[0], xy[1], fsm.Z_TRANSFERT], R, VITESSE_DEPOT, 'remontee du bac')
    return True


def _lisse(t):
    return t * t * (3.0 - 2.0 * t)


def theta_bac(ctx, cible):
    return ctx.prise_apprise.get(fsm.cle_roulis('carton', cible), (0.0, 0.0))[0]


def tour_j6(ctx, piece, cible_bac, roulis_bac, z_prise, souhait):
    """(roulis de depot, R de depot) avec le quart de tour de J6 demande.

    Dans le GIF de reference (13/10) la pince tourne d'environ un quart de tour
    pendant le transfert. Quand prise et depot ont le meme roulis et presque le
    meme azimut (pave a 0 deg, bac jaune a -5 deg), rien ne tourne : on ajoute
    donc ce tour, reparti sur l'arc. Pieces et bacs carres ou ronds : la piece
    tient dans son bac apres un quart de tour. Le sens est celui que l'IK accepte
    aux hauteurs du depot ; a defaut, moins de tour, puis aucun.
    """
    az = float(np.degrees(np.arctan2(cible_bac[1], cible_bac[0])))
    th = theta_bac(ctx, cible_bac)
    sag = fsm.affaissement_largage(np.hypot(*cible_bac))
    z_fin = (FOND_BAC_MM + GARDE_FOND_MM + z_prise if pose_au_fond(piece)
             else y26.HAUTEUR_BAC + GARDE_REBORD_MM + z_prise)
    hauteurs = [fsm.Z_TRANSFERT, y26.HAUTEUR_BAC + GARDE_REBORD_MM + z_prise + 15.0 + sag, z_fin + sag]
    for d in (souhait, -souhait, souhait * 2 / 3, -souhait * 2 / 3, 0.0):
        R = fsm.incline(fsm.orientation(cible_bac, roulis_bac + d), az, th)
        if all(fsm.resout_ik(np.array([cible_bac[0], cible_bac[1], z]), R) is not None
               for z in hauteurs):
            return roulis_bac + d, R
    return None, None


def arc(ctx, depart, prise, cible_bac, roulis_bac, z_prise, v):
    """Points de passage (xyz, R) de l'arc, du point de degagement au-dessus du bac.

    Coordonnees cylindriques autour du socle : l'azimut porte la rotation du
    bras (J1), le rayon passe de celui de la piece a celui du bac, la hauteur
    monte sans jamais redescendre sous son depart. L'outil reste vertical (ou a
    l'inclinaison de prise puis de largage, interpolee) et son roulis suit
    l'azimut : la piece reste droite.
    """
    roulis, theta, _, _ = prise
    a0, a1 = (float(np.degrees(np.arctan2(p[1], p[0]))) for p in (depart, cible_bac))
    r0, r1 = float(np.hypot(*depart[:2])), float(np.hypot(*cible_bac))
    z0, z1 = float(depart[2]), fsm.Z_TRANSFERT
    t1 = theta_bac(ctx, cible_bac)
    points = []
    for k in range(1, v['points'] + 1):
        s = k / v['points']
        lisse = _lisse(s)
        az = a0 + (a1 - a0) * _lisse(s ** v['rotation'])
        r = r0 + (r1 - r0) * lisse + v['ventre'] * np.sin(np.pi * s)
        z = max(z0, z0 + (z1 - z0) * lisse + v['bosse'] * np.sin(np.pi * s))
        xy = r * np.array([np.cos(np.radians(az)), np.sin(np.radians(az))])
        rl = roulis + (roulis_bac - roulis) * lisse
        th = theta + (t1 - theta) * lisse
        points.append((np.array([xy[0], xy[1], z]), fsm.incline(fsm.orientation(xy, rl), az, th)))
    return points


def degagement(xy, z_prise):
    return np.array([xy[0], xy[1], HAUTEUR_MAX_PIECE_MM + GARDE_PIECES_MM + z_prise])


def resout_arc(points):
    """Angles de chaque point, ou None au premier point sans IK."""
    qs = []
    for p, R in points:
        q = fsm.resout_ik(p, R)
        if q is None:
            return None
        qs.append(q[0])
    return qs


def suit_arc(ctx, qs):
    """Enchaine les points sans s'arreter ; lent aux deux bouts, plus vite au milieu."""
    for k, q in enumerate(qs):
        dernier = k == len(qs) - 1
        lent = k == 0 or k >= len(qs) - 2
        if fsm.va_vers(ctx, q, vitesse=VITESSE_ARC_LENTE if lent else VITESSE_ARC,
                       nom=f'arc {k + 1}/{len(qs)}', stabilise=dernier,
                       enchaine=not dernier) is None:
            return False
    return True


def approche_continue(ctx, q_survol, q_bas):
    """Observation -> survol -> prise en UN geste : aucun arret avant la prise.

    Meme decoupage que `fsm.va_vers_par_etapes` (chaque morceau sous la chute
    maximale), mais le survol est un point de PASSAGE, enchaine comme les autres :
    le bras ne s'immobilise qu'en bas, ou la pince doit fermer a l'arret.
    """
    q0 = ctx.pont.angles()
    etapes = 1
    while etapes < fsm.ETAPES_MAX:
        z = [float(fsm.pointe(q0 + (q_survol - q0) * (i / etapes))[2]) for i in range(etapes + 1)]
        if max(a - b for a, b in zip(z, z[1:])) <= fsm.CHUTE_MAX - 20.0:
            break
        etapes += 1
    for i in range(1, etapes + 1):
        if fsm.va_vers(ctx, q0 + (q_survol - q0) * (i / etapes), nom=f'approche {i}/{etapes}',
                       stabilise=False, enchaine=True) is None:
            return False
    return fsm.va_vers(ctx, q_bas, vitesse=fsm.VITESSE_DESCENTE, nom='descente',
                       stabilise=True) is not None


def trie(ctx, piece, xy, bac_xy, variante='actuelle'):
    """True si la piece est deposee dans son bac."""
    prise, (cible_bac, roulis_bac, R_bac) = plan(ctx, piece, xy, bac_xy)
    if prise is None or R_bac is None:
        print(f'{piece} : pas de pose atteignable (prise {prise is not None}, '
              f'bac {R_bac is not None})')
        return False
    roulis, theta, R, z_prise = prise
    ctx.balle_xy, ctx.R_balle = xy, R
    ctx.roulis_balle, ctx.inclinaison_balle, ctx.z_prise = roulis, theta, z_prise

    fsm.ouvre_pince(ctx, force=True)
    survol = fsm.resout_ik(np.array([xy[0], xy[1], fsm.Z_SURVOL]), R)
    bas = fsm.resout_ik(np.array([xy[0], xy[1], max(fsm.Z_PRISE_MIN, z_prise + fsm.BIAIS_Z_PRISE)]), R)
    if survol is None or bas is None:
        print(f'{piece} : IK refusee au survol ou a la prise')
        return False
    print(f'{piece} : approche et descente sans arret vers ({xy[0]:.0f} ; {xy[1]:.0f}), '
          f'Z={z_prise + fsm.BIAIS_Z_PRISE:.0f} mm', flush=True)
    if not approche_continue(ctx, survol[0], bas[0]):
        journal(ctx)
        print(f'{piece} : descente refusee')
        return False
    if fsm._saisie(ctx) != 'REMONTEE':
        journal(ctx)
        ctx.pont.envoie('pro_gripper_open')
        fsm.va_vers(ctx, survol[0], nom='remontee a vide', stabilise=False)
        print(f'{piece} : PRISE A VIDE — arret, aucune nouvelle tentative')
        return False
    print(f'{piece} : saisi ({ctx.resultats.get("pince")})', flush=True)
    ctx.carton_xy, ctx.roulis_carton, ctx.R_carton = cible_bac, roulis_bac, R_bac
    v = VARIANTES[variante]
    if v is None:
        if fsm._remontee(ctx) != 'RECHERCHE_CARTON':
            journal(ctx)
            print(f'{piece} : perdu a la remontee')
            return False
        print(f'{piece} : vers {PIECES[piece]} ({cible_bac[0]:.0f} ; {cible_bac[1]:.0f})', flush=True)
        arrive = fsm._transfert(ctx) == 'LARGAGE'
    else:
        depart = degagement(xy, z_prise)
        leve = fsm.resout_ik(depart, R)
        roulis_depot, R_depot = tour_j6(ctx, piece, cible_bac, roulis_bac, z_prise, v['j6'])
        if R_depot is None:
            print(f'{piece} : aucun roulis de depot accepte par l IK — objet garde en main')
            return False
        R_bac = R_depot
        print(f'{piece} : J6 tourne de {roulis_depot - roulis_bac:+.0f} deg pendant l arc', flush=True)
        qs = resout_arc(arc(ctx, depart, prise, cible_bac, roulis_depot, z_prise, v))
        if leve is None or qs is None:
            print(f'{piece} : arc {variante} sans IK — objet garde en main')
            return False
        print(f'{piece} : leve a Z={depart[2]:.0f} mm puis arc {variante} '
              f'vers {PIECES[piece]} ({cible_bac[0]:.0f} ; {cible_bac[1]:.0f})', flush=True)
        if fsm.va_vers(ctx, leve[0], vitesse=VITESSE_ARC_LENTE, nom='levee verticale',
                       stabilise=False, enchaine=True) is None or not fsm.porte_objet(ctx):
            journal(ctx)
            print(f'{piece} : perdu a la levee')
            return False
        arrive = suit_arc(ctx, qs)
    if not arrive or not depose_bas(ctx, piece, z_prise, R_bac):
        journal(ctx)
        print(f'{piece} : largage non fait — objet garde en main')
        return False
    fsm._retrait(ctx)
    journal(ctx)
    return True


def main():
    a = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument('--toutes', action='store_true', help='enchainer jusqu a planche vide')
    a.add_argument('--plan', action='store_true', help='mesurer et resoudre, sans bouger')
    a.add_argument('--piece', choices=sorted(PIECES), help='ne trier que cette piece')
    a.add_argument('--variante', choices=sorted(VARIANTES), default='actuelle',
                   help='trajectoire du transfert (defaut : actuelle, validee le 13/10)')
    args = a.parse_args()

    ctx = contexte(pont=not args.plan)
    if not args.plan:
        if fsm.porte_objet(ctx):
            raise SystemExit('la pince tient deja un objet — le deposer d abord')
        fsm.va_vers_par_etapes(ctx, fsm.POSE_OBSERVATION, nom='observation', stabilise=False)

    while True:
        vus = localise()
        restantes = sorted(((float(np.hypot(*xy)), p) for p, xy in vus.items()
                            if p in PIECES and PIECES[p] in vus
                            and (args.piece is None or p == args.piece)))
        manque_bac = [p for p in vus if p in PIECES and PIECES[p] not in vus]
        print('vu :', ', '.join(f'{n} ({v[0]:.0f} ; {v[1]:.0f})' for n, v in vus.items()))
        if manque_bac:
            print('bac non vu pour :', ', '.join(manque_bac))
        if not restantes:
            print('plus rien a trier')
            return
        if args.plan:
            for _, p in restantes:
                prise, (cible_bac, roulis_bac, R_bac) = plan(ctx, p, vus[p], vus[PIECES[p]])
                if prise is not None and R_bac is not None:
                    for nom, v in VARIANTES.items():
                        if v is None:
                            continue
                        rd, _ = tour_j6(ctx, p, cible_bac, roulis_bac, prise[3], v['j6'])
                        pts = arc(ctx, degagement(vus[p], prise[3]), prise, cible_bac,
                                  roulis_bac if rd is None else rd, prise[3], v)
                        ok = resout_arc(pts) is not None
                        zs = [q[2] for q, _ in pts]
                        rs = [float(np.hypot(*q[:2])) for q, _ in pts]
                        print(f'    {p:14s} {nom:15s} J6 {0 if rd is None else rd - roulis_bac:+4.0f}  IK {"OK" if ok and rd is not None else "IMPOSSIBLE"}  '
                              f'Z {min(zs):.0f}-{max(zs):.0f} mm  rayon max {max(rs):.0f} mm')
                print(f'  {p:14s} prise {"OK" if prise else "IMPOSSIBLE"}'
                      + (f' (incl {prise[1]:+.0f}, roulis {prise[0]:+.0f}, Z {prise[3]:.0f})' if prise else '')
                      + f'   depot {"OK" if R_bac is not None else "IMPOSSIBLE"}')
            journal(ctx)
            return
        _, piece = restantes[0]
        if not trie(ctx, piece, vus[piece], vus[PIECES[piece]], args.variante) or not args.toutes:
            return


if __name__ == '__main__':
    main()
