#!/usr/bin/env python3
"""Saisir le cube bleu et le deposer dans son bac. UNE fois, sans rafale.

    cd ~/Osama_ws/src/mycobot_R6A
    MYCOBOT_PI=10.10.0.219 .venv/bin/python -u scripts/saisir_cube_bleu.py
    MYCOBOT_PI=10.10.0.219 .venv/bin/python -u scripts/saisir_cube_bleu.py --sans-saisie

Le bras doit etre AU-DESSUS de 150 mm au depart : le script refuse de partir
autrement, il ne sait pas d'ou il vient.

`--sans-saisie` s'arrete juste avant de fermer la pince, apres avoir annonce la
hauteur reellement atteinte. C'est la mesure, sans l'engagement.

Ce que ce script fait, et qui n'est pas dans la machine a etats :

* **la hauteur de fermeture n'est pas tabulee, elle est ASSERVIE.**
  `Z_PRISE_PAR_CLASSE` ne connait pas ces pieces, et leurs valeurs ne se
  calculent pas : le modele du bras place la pointe plus bas que la consigne,
  d'un biais qui croit avec l'allonge (5,9 mm a 206 mm, 17,9 a 370 — mesure le
  22/09 a 228 mm : -6,5 mm). Plutot que d'inventer un nombre, on descend, on
  relit les codeurs, on reinjecte l'ecart. C'est la methode qui a fait passer
  la precision de 10,79 mm a 0,63 mm sur le test 7.
* **le milieu de l'objet vient des DEUX cameras.** yolo26 donne la boite dans
  chaque vue, la triangulation donne la hauteur, et l'arducam donne le XY a
  cette hauteur — pas sur un plan fixe. L'arducam est quasi zenithale, la SVPRO
  rasante : le XY triangule ferait entrer le biais hors-plan de la seconde.
* **le verdict de prise exige le statut ET l'angle.** Le 28/08, statut 2
  (« objet saisi ») avec un angle de 22 — deux degres au-dessus de la pince vide
  — et la figurine simplement poussee de 18,3 mm. Rien n'etait tenu.
* **au premier echec, on s'arrete.** Chaque fermeture a vide deplace l'objet :
  reessayer en aveugle, c'est le pousser hors d'atteinte.
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
import tri_couleur as tc                                               # noqa: E402
import yolo26_dashboard as y                                           # noqa: E402
from yolo_capture import ouvre_camera                                  # noqa: E402

OBJET = 'cube_bleu'
BAC = 'bac_bleu'
Z_DEPART_MIN = 150.0        # le bras doit etre degage avant qu'on parle de trajectoire
PASSES_ASSERVIES = 2        # au-dela, on n'apprend plus rien : le biais est systematique
TOLERANCE_Z_MM = 1.5


def detecte(modele_service, visions, classes):
    """{classe: (camera, boite, conf, z triangule ou None)} sur une trame fraiche."""
    specs = {s.name: s for s in pd.registre.detect_cameras(probe_capture=False)}
    for camera in ('svpro', 'arducam'):
        cap = ouvre_camera(specs[camera].v4l2_index)
        if cap is None:
            continue
        if camera == 'arducam':
            pd.regle_exposition(specs[camera].v4l2_index, specs[camera].manual_exposure)
        for _ in range(8):
            ok, image = cap.read()
        cap.release()
        modele_service.soumet(camera, image)
        vu = None
        for _ in range(80):
            vu = modele_service.dernier(camera)
            if vu is not None:
                break
            time.sleep(0.1)
        if vu is not None:
            y._VISIONS[camera], y._DERNIER[camera] = visions[camera], (vu[0], vu[1], time.time())
    # Les deux vues sont prises en serie ici, pas en parallele comme dans le
    # dashboard : la garde de fraicheur d'une seconde les rejetterait.
    y.FRAICHEUR_AUTRE_VUE_S = 60.0
    rendu = {}
    for camera in ('arducam', 'svpro'):
        vu = y._DERNIER.get(camera)
        if vu is None:
            continue
        for d in vu[1]:
            if d['classe'] not in classes or d['classe'] in rendu:
                continue                       # l'arducam d'abord : elle est la reference XY
            rendu[d['classe']] = (camera, d['boite'], d['conf'],
                                  y.hauteur_triangulee(d['classe'], camera, d['boite']))
    return rendu


def pose_verticale(xy, z):
    """(angles, residu) pour poser la pointe en (xy, z), outil vertical, ou None."""
    azimut = float(np.degrees(np.arctan2(xy[1], xy[0])))
    R = fsm.incline(fsm.orientation(xy, 0.0), azimut, 0.0)
    sol = fsm.resout_ik(np.array([xy[0], xy[1], z]), R)
    return None if sol is None else (sol[0], sol[1])


def va(ctx, xy, z, nom, vitesse=fsm.VITESSE, R=None):
    """Un palier, relu aux codeurs. Leve si l'IK manque ou si `va_vers` refuse.

    `R` impose une orientation : le largage se fait OUTIL COUCHE des que le bac
    sort du domaine vertical (~320 mm), et c'est `fsm.choisit_pose_largage` qui
    dit de combien. Le premier essai du 22/09 a echoue ici meme — bac bleu a
    412 mm, outil force a la verticale, aucune solution IK, cube reste en main.
    """
    if R is None:
        sol = pose_verticale(xy, z)
    else:
        s = fsm.resout_ik(np.array([xy[0], xy[1], z]), R)
        sol = None if s is None else (s[0], s[1])
    if sol is None:
        raise SystemExit(f'{nom} : aucune solution IK a Z={z:.1f} — RIEN ENVOYE')
    if fsm.va_vers(ctx, sol[0], vitesse=vitesse, nom=nom) is None:
        raise SystemExit(f'{nom} : REFUSE par va_vers — on s arrete')
    time.sleep(0.6)
    # Le pont repond OK meme quand une limite articulaire est refusee en silence :
    # la seule preuve que le bras y est alle, c'est get_angles.
    p = fsm.pointe(ctx.pont.angles())
    print(f'  {nom:22s} consigne Z {z:6.1f} -> reel Z {p[2]:6.1f} '
          f'({p[2] - z:+5.1f})  X {p[0]:6.1f} Y {p[1]:6.1f}', flush=True)
    return p


def descend_asservi(ctx, xy, z_vise):
    """Descend jusqu'a ce que la POINTE REELLE soit a `z_vise`. Rend le Z atteint.

    Le biais du modele n'est pas connu d'avance a cette allonge : on le mesure
    en descendant, et on le reinjecte. Deux passes suffisent — il est
    systematique, pas aleatoire.
    """
    consigne = z_vise
    p = va(ctx, xy, consigne, 'descente', vitesse=fsm.VITESSE_DESCENTE)
    for passe in range(PASSES_ASSERVIES):
        ecart = z_vise - p[2]
        if abs(ecart) <= TOLERANCE_Z_MM:
            break
        consigne += ecart
        print(f'  reinjection {passe + 1} : ecart {ecart:+.1f} mm -> consigne {consigne:.1f}',
              flush=True)
        p = va(ctx, xy, consigne, f'descente {passe + 2}', vitesse=fsm.VITESSE_DESCENTE)
    return p[2]


def saisit(ctx):
    """Ferme la pince et rend (tenu, statut, angle). Ne reessaie JAMAIS."""
    ctx.pont.envoie('set_pro_gripper_torque', torque=fsm.COUPLE_PINCE_DEFAUT)
    time.sleep(0.3)
    ctx.pont.envoie('pro_gripper_angle', angle=fsm.ANGLE_PINCE_FERMEE)
    statut, limite = 0, time.time() + fsm.ATTENTE_PINCE_MAX
    while time.time() < limite:
        time.sleep(0.12)
        statut = ctx.pont.statut_pince()
        if statut in (1, 2, 3):
            break
    angle = ctx.pont.angle_pince()
    vide = angle is None or angle < fsm.ANGLE_PINCE_FERMEE + fsm.MARGE_ANGLE_TENUE
    return (statut == 2 and not vide), statut, angle


def main():
    a = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument('--sans-saisie', action='store_true',
                   help='descendre et mesurer, sans fermer la pince')
    a.add_argument('--depose-seulement', action='store_true',
                   help='le cube est DEJA en main : reprendre au largage')
    args = a.parse_args()

    y._renomme_le_banc(pd)
    service = y.ServiceYOLO26()
    visions = {'arducam': pd.Vision(), 'svpro': pd.Vision('svpro_extrinsic_servo')}

    if args.depose_seulement:
        ctx = fsm.Contexte()
        ctx.pont = fsm.Pont()
        ctx.en_main = OBJET
        angle = ctx.pont.angle_pince()
        if angle is None or angle < fsm.ANGLE_PINCE_FERMEE + fsm.MARGE_ANGLE_TENUE:
            service.ferme()
            raise SystemExit(f'la pince est vide (angle {angle}) — rien a deposer')
        print(f'reprise : cube en main (angle pince {angle})')
        depose(ctx, service, visions)
        return

    vu = detecte(service, visions, {OBJET})
    if OBJET not in vu:
        service.ferme()
        raise SystemExit(f'{OBJET} non vu par aucune camera — RIEN ENVOYE')
    camera, boite, conf, z = vu[OBJET]
    if z is None:
        service.ferme()
        raise SystemExit('pas de hauteur triangulee (une seule vue) — RIEN ENVOYE')
    xy = y._centre_base(visions['arducam' if camera == 'arducam' else camera], boite, z)
    portee = float(np.hypot(*xy))
    print(f'{OBJET} : {camera} conf {conf:.2f}, milieu X {xy[0]:.1f} Y {xy[1]:.1f} '
          f'Z {z:.1f} mm, portee {portee:.0f} mm')
    if not fsm.PORTEE_MIN <= portee <= fsm.PORTEE_MAX:
        service.ferme()
        raise SystemExit(f'hors enveloppe {fsm.PORTEE_MIN}-{fsm.PORTEE_MAX} mm — RIEN ENVOYE')

    ctx = fsm.Contexte()
    ctx.pont = fsm.Pont()
    depart = fsm.pointe(ctx.pont.angles())
    print(f'depart : X {depart[0]:.1f} Y {depart[1]:.1f} Z {depart[2]:.1f} mm')
    if depart[2] < Z_DEPART_MIN:
        service.ferme()
        raise SystemExit(f'bras sous {Z_DEPART_MIN:.0f} mm — le degager a la main d abord')

    print('\n--- approche ---')
    va(ctx, xy, fsm.Z_TRANSFERT, 'transfert')
    va(ctx, xy, fsm.Z_SURVOL, 'survol')
    z_reel = descend_asservi(ctx, xy, z)
    print(f'\npointe a {z_reel:.1f} mm, milieu du cube vise a {z:.1f} mm')

    if args.sans_saisie:
        service.ferme()
        print('PINCE NON TOUCHEE (--sans-saisie). Regarde ou les doigts encadrent le cube.')
        return

    print('\n--- saisie ---')
    tenu, statut, angle = saisit(ctx)
    print(f'statut {statut} ({fsm.ETAT_PINCE.get(statut, "?")}), angle {angle} — '
          + ('TENU' if tenu else 'RIEN TENU'))
    if not tenu:
        service.ferme()
        raise SystemExit(
            'prise non confirmee. ON S ARRETE : chaque fermeture a vide deplace '
            'l objet. Regarder la scene, puis rejouer avec --sans-saisie pour '
            'relire la hauteur.')

    print('\n--- transfert ---')
    va(ctx, xy, fsm.Z_TRANSFERT, 'remontee')
    depose(ctx, service, visions)


def depose(ctx, service, visions):
    """Le cube est en main : le porter dans son bac et l'y lacher."""
    # Le bac ne peut pas etre localise pendant que le bras le survole : on le
    # relit une fois le cube en main et le bras remonte.
    vu = detecte(service, visions, {BAC})
    service.ferme()
    if BAC not in vu:
        raise SystemExit(f'{BAC} non vu — le cube reste EN MAIN, bras en hauteur')
    camera_bac, boite_bac, conf_bac, z_bac = vu[BAC]
    xy_bac = y._centre_base(visions[camera_bac], boite_bac,
                            y.HAUTEUR_BAC if z_bac is None else z_bac)
    print(f'{BAC} : {camera_bac} conf {conf_bac:.2f}, X {xy_bac[0]:.1f} Y {xy_bac[1]:.1f}')
    if float(np.hypot(*xy_bac)) > fsm.PORTEE_CARTON_MAX:
        raise SystemExit(f'{BAC} hors d atteinte — le cube reste EN MAIN')

    # L'inclinaison de l'outil est une CONDITION D'ATTEIGNABILITE, pas un
    # confort : au-dela de ~320 mm l'outil vertical n'a aucune solution.
    roulis, theta, R_bac = fsm.choisit_pose_largage(ctx, xy_bac)
    if R_bac is None:
        raise SystemExit(f'{BAC} a {float(np.hypot(*xy_bac)):.0f} mm : aucune pose de '
                         'largage resolue — le cube reste EN MAIN, bras en hauteur')
    print(f'largage : inclinaison {theta:+.0f} deg, roulis {roulis:+.0f} deg')
    va(ctx, xy_bac, fsm.Z_TRANSFERT, 'au-dessus du bac', R=R_bac)
    va(ctx, xy_bac, fsm.Z_LARGAGE, 'largage', vitesse=fsm.VITESSE_DESCENTE, R=R_bac)
    # `force` : la pince Pro cale a un angle qui n'est pas celui commande (52 pour
    # une consigne de 20). Se fier a l'angle reviendrait a ne jamais relacher.
    fsm.ouvre_pince(ctx, force=True)
    time.sleep(0.5)
    print(f'largue dans {BAC}')
    va(ctx, xy_bac, fsm.Z_TRANSFERT, 'retrait', R=R_bac)
    print('\n=== termine — le cube bleu est dans son bac ===')


if __name__ == '__main__':
    main()
