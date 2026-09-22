#!/usr/bin/env python3
"""Saisir une piece peinte et la lacher AU MILIEU de son bac. Une piece par appel.

    cd ~/Osama_ws/src/mycobot_R6A
    MYCOBOT_PI=10.10.0.219 .venv/bin/python -u scripts/trier_pieces.py cube_rouge
    MYCOBOT_PI=10.10.0.219 .venv/bin/python -u scripts/trier_pieces.py pave_jaune --sans-saisie

Generalise `saisir_cube_bleu.py` aux quatre pieces et en reprend toute la
mecanique mesuree : descente asservie aux codeurs, verdict de prise par statut
ET angle, arret au premier echec. Seul le largage change.

**Le milieu du bac se prend a l'ARDUCAM, pas a la SVPRO.** Les deux caméras ne
placent pas le centre d'un meme bac au meme endroit — 8,7 a 28,4 mm d'ecart
mesures le 22/09. Ce n'est pas du bruit, c'est de la parallaxe : un bac a
30 mm de haut, et projeter son bord sur le plan de la table decale le XY de
30/tan(elevation). L'arducam est a ~85 deg d'elevation, soit 2,6 mm ; la SVPRO
a ~58 deg, soit 18,7 mm. Le largage du cube bleu du 22/09 avait pris le bac vu
par la SVPRO, faute de mieux — d'ou un depot excentre. Ici la SVPRO ne sert
qu'en repli, et le repli est annonce.

Le bac reste relu APRES la saisie, bras remonte : on ne peut pas le localiser
pendant qu'on le survole.
"""
from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE / 'scripts'))

import pick_dashboard as pd                                            # noqa: E402
import pick_fsm as fsm                                                 # noqa: E402
import saisir_cube_bleu as sc                                          # noqa: E402
import yolo26_dashboard as y                                           # noqa: E402

DEMI_COTE_BAC_MM = 52.5        # bac de 105 mm : au-dela, on n'est plus dedans


_ORIGINE_Z_LARGAGE = fsm.z_largage


def _hauteur_voulue(ctx):
    """Hauteur de lacher voulue : au-dessus du rebord, sans le plancher carton.

    `fsm.z_largage` rend `max(Z_LARGAGE, rebord + GARDE_LARGAGE)`. Le plancher
    de 116,9 mm a ete dimensionne quand la cible etait un carton a rebord
    83 mm : 83 + 41,9 = 124,9, il ne mordait jamais. Un bac a 30 mm de rebord,
    donc 30 + 41,9 = 71,9, et le plancher ajoute 45 mm — la piece tombe de
    87 mm au lieu de 42. Le cylindre vert, seule piece qui roule, est ressorti
    du bac vert par ce mecanisme le 22/09.

    Quand le rebord est inconnu, le plancher garde sa raison d'etre.
    """
    rebord = getattr(ctx, 'z_rebord', None)
    if rebord is None:
        return _ORIGINE_Z_LARGAGE(ctx)
    return rebord + fsm.GARDE_LARGAGE


def pose_de_prise(ctx, xy, z_cible):
    """(roulis, inclinaison, R) pour saisir en (xy, z_cible), ou (None, None, None).

    Meme methode que `fsm.choisit_pose_prise`, a une difference pres qui compte
    ici : cette derniere tire sa hauteur de `Z_PRISE_PAR_CLASSE`, qui ne
    connait pas les pieces peintes et retomberait sur des valeurs generiques.
    Nous, on TRIANGULE la hauteur — donc on valide la colonne a la hauteur
    reellement commandee, pas a une autre.

    L'outil vertical est essaye d'abord quand l'allonge le permet. Mesure du
    26/08 : a partir de 335 mm la colonne verticale est trouee pour TOUS les
    roulis, le bras ne peut plus rejoindre la prise qu'en changeant de branche.
    D'ou le depart a l'inclinaison MINIMALE que l'allonge exige.
    """
    xy = np.asarray(xy, float)
    portee = float(np.hypot(*xy))
    azimut = float(np.degrees(np.arctan2(xy[1], xy[0])))
    depart = next(t for limite, t in fsm.INCLINAISONS_PAR_PORTEE if portee <= limite)
    for theta in [t for t in fsm.INCLINAISONS if t <= depart] or [fsm.INCLINAISONS[-1]]:
        hauteurs = ([fsm.Z_TRANSFERT, fsm.Z_SURVOL, z_cible] if theta == 0.0
                    else [fsm.Z_SURVOL, z_cible])
        for roulis in fsm.ROULIS:
            R = fsm.incline(fsm.orientation(xy, roulis), azimut, theta)
            if (all(fsm.resout_ik(np.array([xy[0], xy[1], z]), R) is not None
                    for z in hauteurs)
                    and fsm.colonne_continue(xy, R, z_cible)):
                return roulis, theta, R
    return None, None, None


def descend_asservi(ctx, xy, z_vise, R):
    """Descend jusqu'a ce que la POINTE REELLE soit a `z_vise`. Rend le Z atteint.

    Copie de `saisir_cube_bleu.descend_asservi`, orientation en plus : le pave
    jaune est a 331 mm, ou l'outil ne peut pas etre tenu vertical.
    """
    consigne = z_vise
    p = sc.va(ctx, xy, consigne, 'descente', vitesse=fsm.VITESSE_DESCENTE, R=R)
    for passe in range(sc.PASSES_ASSERVIES):
        ecart = z_vise - p[2]
        if abs(ecart) <= sc.TOLERANCE_Z_MM:
            break
        consigne += ecart
        print(f'  reinjection {passe + 1} : ecart {ecart:+.1f} mm -> consigne {consigne:.1f}',
              flush=True)
        p = sc.va(ctx, xy, consigne, f'descente {passe + 2}',
                  vitesse=fsm.VITESSE_DESCENTE, R=R)
    return p[2]


def rejoint_transfert(ctx, xy, R=None):
    """Amener la pointe a Z_TRANSFERT au-dessus de `xy`, depuis n'importe ou.

    On part souvent de la pose de degagement, pointe a plus de 550 mm : c'est
    une descente de pres de 400 mm, que `va_vers` refuse d'un seul ordre
    (CHUTE_MAX, garde nee du plongeon du 20/08). Le decoupage ne peut pas se
    faire en paliers de hauteur au-dessus de la cible — a Z=382 et 288 mm
    d'allonge l'outil vertical n'a aucune solution IK. `va_vers_par_etapes`
    decoupe dans l'espace ARTICULAIRE, ou les etapes sont atteignables par
    construction.
    """
    if R is None:
        sol = sc.pose_verticale(xy, fsm.Z_TRANSFERT)
    else:
        s = fsm.resout_ik(np.array([xy[0], xy[1], fsm.Z_TRANSFERT]), R)
        sol = None if s is None else (s[0], s[1])
    if sol is None:
        raise SystemExit(f'transfert : aucune solution IK a Z={fsm.Z_TRANSFERT:.1f} '
                         '— RIEN ENVOYE')
    if fsm.va_vers_par_etapes(ctx, sol[0], nom='transfert') is None:
        raise SystemExit('transfert : REFUSE par va_vers_par_etapes — on s arrete')
    time.sleep(0.6)
    p = fsm.pointe(ctx.pont.angles())
    print(f'  {"transfert":22s} consigne Z {fsm.Z_TRANSFERT:6.1f} -> reel Z {p[2]:6.1f} '
          f'({p[2] - fsm.Z_TRANSFERT:+5.1f})  X {p[0]:6.1f} Y {p[1]:6.1f}', flush=True)
    return p


def centre_du_bac(service, visions, bac, accepte_svpro):
    """(xy, camera, conf) du MILIEU du bac. L'arducam fait foi."""
    vu = sc.detecte(service, visions, {bac})
    if bac not in vu:
        raise SystemExit(f'{bac} non vu — la piece reste EN MAIN, bras en hauteur')
    camera, boite, conf, z = vu[bac]
    if camera != 'arducam':
        if not accepte_svpro:
            raise SystemExit(
                f'{bac} vu seulement par la {camera}, dont la parallaxe vaut ~19 mm '
                f'sur un bord a 30 mm. La piece reste EN MAIN. Relancer avec '
                f'--accepte-svpro pour larguer quand meme, en connaissance de cause.')
        print(f'  /!\\ repli {camera} : centre du bac connu a ~19 mm pres, pas 3')
    # Projeter a la hauteur du BORD, pas au plan de la table : c'est le bord que
    # la camera voit, et c'est sur lui que le decalage de parallaxe se calcule.
    xy = y._centre_base(visions[camera], boite, y.HAUTEUR_BAC if z is None else z)
    print(f'{bac} : {camera} conf {conf:.2f}, milieu X {xy[0]:.1f} Y {xy[1]:.1f}')
    return xy, camera, conf


def depose_au_milieu(ctx, service, visions, objet, bac, accepte_svpro):
    """La piece est en main : la porter au milieu de son bac et l'y lacher."""
    xy, _, _ = centre_du_bac(service, visions, bac, accepte_svpro)
    service.ferme()
    # Le rebord du bac AVANT de choisir la pose : `choisit_pose_largage` valide
    # l'IK a `z_largage_commande`, et commander ensuite une autre hauteur
    # invalide sa reponse. Meme regle que le tableau de bord — le plancher de
    # 116,9 mm etait taille pour un carton a rebord 83 mm et lache la piece
    # 87 mm au-dessus d'un bac de 30 au lieu de 42.
    ctx.z_rebord = y.HAUTEUR_BAC
    fsm.z_largage = _hauteur_voulue
    portee = float(np.hypot(*xy))
    if portee > fsm.PORTEE_CARTON_MAX:
        raise SystemExit(f'{bac} a {portee:.0f} mm, hors d atteinte — piece EN MAIN')

    # L'inclinaison de l'outil est une CONDITION D'ATTEIGNABILITE : au-dela de
    # ~320 mm d'allonge l'outil vertical n'a aucune solution IK.
    roulis, theta, R_bac = fsm.choisit_pose_largage(ctx, xy)
    if R_bac is None:
        raise SystemExit(f'{bac} a {portee:.0f} mm : aucune pose de largage resolue '
                         '— la piece reste EN MAIN, bras en hauteur')
    print(f'largage : inclinaison {theta:+.0f} deg, roulis {roulis:+.0f} deg, '
          f'portee {portee:.0f} mm')

    z_lache = fsm.z_largage_commande(ctx, xy)
    print(f'lacher : pointe voulue a {fsm.z_largage(ctx):.1f} mm '
          f'({fsm.GARDE_LARGAGE:.0f} mm au-dessus du rebord de {y.HAUTEUR_BAC:.0f} mm, '
          f'plancher historique {fsm.Z_LARGAGE:.1f}), consigne {z_lache:.1f} mm')
    sc.va(ctx, xy, fsm.Z_TRANSFERT, 'au-dessus du bac', R=R_bac)
    p = sc.va(ctx, xy, z_lache, 'largage', vitesse=fsm.VITESSE_DESCENTE, R=R_bac)
    ecart = float(np.linalg.norm(np.array([p[0], p[1]]) - xy))
    print(f'  pointe a {ecart:.1f} mm du milieu vise '
          + ('(DANS le bac)' if ecart <= DEMI_COTE_BAC_MM else '/!\\ HORS du bac'))
    # `force` : la pince Pro cale a un angle qui n'est pas celui commande (52 pour
    # une consigne de 20). Se fier a l'angle reviendrait a ne jamais relacher.
    fsm.ouvre_pince(ctx, force=True)
    time.sleep(0.5)
    print(f'largue dans {bac}')
    sc.va(ctx, xy, fsm.Z_TRANSFERT, 'retrait', R=R_bac)
    print(f'\n=== termine — {objet} est dans {bac} ===')


def main():
    a = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument('piece', choices=sorted(y.OBJETS))
    a.add_argument('--sans-saisie', action='store_true',
                   help='descendre et mesurer, sans fermer la pince')
    a.add_argument('--depose-seulement', action='store_true',
                   help='la piece est DEJA en main : reprendre au largage')
    a.add_argument('--accepte-svpro', action='store_true',
                   help='larguer meme si seule la SVPRO voit le bac (~19 mm)')
    args = a.parse_args()

    objet = args.piece
    bac = y.DESTINATION[objet]
    print(f'=== {objet} -> {bac} ===')

    y._renomme_le_banc(pd)
    service = y.ServiceYOLO26()
    visions = {'arducam': pd.Vision(), 'svpro': pd.Vision('svpro_extrinsic_servo')}

    if args.depose_seulement:
        ctx = fsm.Contexte()
        ctx.pont = fsm.Pont()
        ctx.en_main = objet
        angle = ctx.pont.angle_pince()
        if angle is None or angle < fsm.ANGLE_PINCE_FERMEE + fsm.MARGE_ANGLE_TENUE:
            service.ferme()
            raise SystemExit(f'la pince est vide (angle {angle}) — rien a deposer')
        print(f'reprise : {objet} en main (angle pince {angle})')
        depose_au_milieu(ctx, service, visions, objet, bac, args.accepte_svpro)
        return

    vu = sc.detecte(service, visions, {objet})
    if objet not in vu:
        service.ferme()
        raise SystemExit(f'{objet} non vu par aucune camera — RIEN ENVOYE')
    camera, boite, conf, z = vu[objet]
    if z is None:
        service.ferme()
        raise SystemExit('pas de hauteur triangulee (une seule vue) — RIEN ENVOYE')
    xy = y._centre_base(visions[camera], boite, z)
    portee = float(np.hypot(*xy))
    print(f'{objet} : {camera} conf {conf:.2f}, milieu X {xy[0]:.1f} Y {xy[1]:.1f} '
          f'Z {z:.1f} mm, portee {portee:.0f} mm')
    if not fsm.PORTEE_MIN <= portee <= fsm.PORTEE_MAX:
        service.ferme()
        raise SystemExit(f'hors enveloppe {fsm.PORTEE_MIN}-{fsm.PORTEE_MAX} mm — RIEN ENVOYE')

    ctx = fsm.Contexte()
    ctx.pont = fsm.Pont()
    depart = fsm.pointe(ctx.pont.angles())
    print(f'depart : X {depart[0]:.1f} Y {depart[1]:.1f} Z {depart[2]:.1f} mm')
    if depart[2] < sc.Z_DEPART_MIN:
        service.ferme()
        raise SystemExit(f'bras sous {sc.Z_DEPART_MIN:.0f} mm — le degager a la main d abord')

    roulis, theta, R = pose_de_prise(ctx, xy, z)
    if R is None:
        service.ferme()
        raise SystemExit(f'{objet} a {portee:.0f} mm : aucune orientation de saisie ne '
                         'descend droit jusqu a la piece — RIEN ENVOYE')
    print(f'saisie : inclinaison {theta:+.0f} deg, roulis {roulis:+.0f} deg')

    print('\n--- approche ---')
    rejoint_transfert(ctx, xy, R if theta != 0.0 else None)
    sc.va(ctx, xy, fsm.Z_SURVOL, 'survol', R=R)
    z_reel = descend_asservi(ctx, xy, z, R)
    print(f'\npointe a {z_reel:.1f} mm, milieu de la piece vise a {z:.1f} mm')

    if args.sans_saisie:
        service.ferme()
        print('PINCE NON TOUCHEE (--sans-saisie).')
        return

    print('\n--- saisie ---')
    tenu, statut, angle = sc.saisit(ctx)
    print(f'statut {statut} ({fsm.ETAT_PINCE.get(statut, "?")}), angle {angle} — '
          + ('TENU' if tenu else 'RIEN TENU'))
    if not tenu:
        service.ferme()
        raise SystemExit(
            'prise non confirmee. ON S ARRETE : chaque fermeture a vide deplace '
            'l objet. Regarder la scene avant de rejouer.')

    print('\n--- transfert ---')
    sc.va(ctx, xy, fsm.Z_TRANSFERT, 'remontee', R=R)
    depose_au_milieu(ctx, service, visions, objet, bac, args.accepte_svpro)


if __name__ == '__main__':
    main()
