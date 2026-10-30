#!/usr/bin/env python3
"""Lanceur du tri des PIECES PEINTES : yolo26 partout, les deux extrinseques.

    conda deactivate
    /usr/bin/python3 scripts/lancer_pick_dashboard_final.py

Ce que ce lanceur fait, et que `lancer_pick_dashboard.py` ne fait pas :

1. **Il calibre les DEUX cameras d'un seul « oui »**, l'une apres l'autre, avant
   d'ouvrir le dashboard : l'arducam par `calibration_extrinseque_auto.py` (sauvegarde,
   validation leave-one-out, refus sans `--force`), la SVPRO par
   `calibrate_camera_base_extrinsic.py` (16 coins, RANSAC + LM, meme validation).
   Les deux sont ajustees contre `planche_actuelle.yaml`, la reference MESUREE
   AU ROBOT le 15/09 — jamais contre `workspace_markers.yaml`, ni par
   `--reference`, qui releve les marqueurs A TRAVERS une extrinseque et referme
   le cercle.
2. **Il detecte tout par yolo26 entraine**, sur les deux vues : plus de seuil
   d'Otsu sur le coeur sombre des cartons, plus de tri par la robe et les cotes
   en mm, plus de seuil HSV jaune pour la balle. Voir `yolo26_dashboard.py`.
3. **Il trie les quatre pieces du dossier dans leurs quatre bacs**, chacune dans
   le bac de SA couleur. Il n'y a plus de balle, plus de scotch, plus de petit
   robot imprime.

**Pas de carte de correction IDW** (methode de Shepard), et donc pas de test de
saisie au demarrage. La chaine est courte et se lit d'un bout a l'autre :

    yolo26 -> pixel -> extrinseque -> mm robot -> IK

La carte corrigeait, point par point, ce que l'extrinseque rendait faux. Les
deux extrinseques etant recalibrees ici contre la reference mesuree au robot,
c'est l'extrinseque qui porte la justesse — pas un rattrapage pose par-dessus.
`lancer_pick_dashboard.py` garde la carte pour l'ancien banc.

Options :

    --sans-question      ouvrir sans proposer aucune calibration
    --sans-svpro         calibrer l'arducam SEULE (la SVPRO garde son extrinseque)
    --svpro-dream        SVPRO reservee a DREAM : ni calibree, ni utilisee par YOLO
    --automatique        demarre seul la boucle automatique, 8 s apres l'ouverture
    --dream              DREAM a cote du tri (protocole reel, etape 9) : la fenetre du
                         tri a gauche, DREAM <-> YOLO <-> codeurs a droite, comme le
                         dashboard Gazebo. Voir `dream_tri_panneau.py`.

« demarrer la boucle » enchaine les pieces comme `tri_fluide.py` : un geste par
piece, pose au fond du bac de sa couleur. « etape suivante » reste le pas a pas du FSM.
    --sans-yolo          revenir aux detecteurs couleur/forme (diagnostic)
    --modele <best.pt>   autres poids que le run `pieces_v3` (les deux cameras)
    --seuil 0.10         confiance minimale du modele entraine
    --inventaire cube_rouge=2 cube_bleu=1

Sans `--inventaire`, une piece de chaque : c'est ce que le banc porte. Le compte
sert a savoir quand une classe est FINIE — sans lui, une classe l'est des le
premier depot.
"""
from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

import yaml

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE / 'scripts'))

from PyQt5.QtWidgets import QApplication, QMessageBox                   # noqa: E402
from PyQt5.QtCore import QTimer                                         # noqa: E402

import calibration_dialogue                                            # noqa: E402
import yolo26_dashboard                                                # noqa: E402

CALIB = RACINE / 'training' / 'calibration'
VENV = RACINE / '.venv' / 'bin' / 'python'
CALIBRE_SVPRO = CALIB / 'calibrate_camera_base_extrinsic.py'
REFERENCE = CALIB / 'planche_actuelle.yaml'
SVPRO = CALIB / 'svpro_extrinsic_servo.yaml'
# La SVPRO voit la planche de biais : ses marqueurs du bas sont pres du bord et
# son leave-one-out est monte a 5,9 mm le 15/09, au-dessus du seuil habituel de
# 5 mm. Elle avait alors ete validee PAR LA BALLE, pas par le residu. On ne
# refuse donc pas a ce seuil — on l'affiche et on le dit.
LOO_SVPRO_ATTENDU_MM = 5.0


PERIODE_EXPOSITION_S = 1.0


def tient_l_exposition(module):
    """Repose l'exposition manuelle de l'arducam chaque seconde, sans condition.

    Mesure du 22/09 a 17:51:54 : le pilote UVC relache le reglage tout seul et
    repart sur ses defauts — `auto_exposure = 3` et `exposure_time_absolute =
    157` au lieu de 75. L'image se lave sans que rien ne le dise.

    Deux pieges, tous deux payes ce jour-la :

    * la relecture rend la valeur STOCKEE, pas celle que le capteur applique.
      Les deux se separent des qu'un reglage a ete pose dans le mauvais ordre
      ou qu'un second processus a ouvert le peripherique : on lit 75 sur une
      image lavee. Une surveillance qui n'ecrit QUE si la relecture a derive
      est donc aveugle a la panne qu'elle surveille. On repose sans condition.
    * `regle_exposition` doit passer en manuel AVANT de poser le temps, en deux
      appels — sa propre docstring le dit. Tout ecrire d'un coup laisse le
      pilote stocker la valeur sans l'appliquer.

    Ecrire un controle V4L2 ne demande pas l'acces exclusif : ce fil ne prend
    jamais la camera, donc il ne provoque pas lui-meme la bascule qu'il corrige.
    """
    specs = [s for s in module.registre.detect_cameras(probe_capture=False)
             if s.manual_exposure >= 0]
    if not specs:
        return
    def boucle():
        while True:
            for spec in specs:
                module.regle_exposition(spec.v4l2_index, spec.manual_exposure)
            time.sleep(PERIODE_EXPOSITION_S)
    threading.Thread(target=boucle, daemon=True).start()
    print('exposition : ' + ', '.join(f'{s.name} tenue a {s.manual_exposure}'
                                      for s in specs)
          + f' (repose toutes les {PERIODE_EXPOSITION_S:.0f} s)', flush=True)


def calibre_svpro():
    """Recalibre la SVPRO contre la reference robot. Rend le leave-one-out, ou None.

    Appelee SANS poser de question : le « oui » de la fenetre de calibration vaut
    pour les deux cameras. Les faire separement n'avait pas de sens — elles
    regardent la meme planche, servent le meme cycle, et une seule des deux
    remise a jour laisse le couple incoherent.

    La camera doit etre LIBRE : l'apercu de `calibration_dialogue` la tient, d'ou
    l'appel APRES `demande()`, qui la referme.
    """
    if not CALIBRE_SVPRO.exists():
        print(f'calibration SVPRO sautee — {CALIBRE_SVPRO.name} introuvable', flush=True)
        return None
    print('SVPRO : calibration contre planche_actuelle.yaml…', flush=True)
    if SVPRO.exists():
        secours = SVPRO.with_suffix(f'.avant_{datetime.now():%d%m_%H%M}.yaml')
        shutil.copy2(SVPRO, secours)
        print(f'SVPRO : ancienne extrinseque sauvegardee -> {secours.name}', flush=True)
    sortie = subprocess.run(
        [str(VENV), str(CALIBRE_SVPRO), '--camera', 'svpro',
         '--markers', str(REFERENCE), '--out', str(SVPRO)],
        cwd=str(RACINE), capture_output=True, text=True)
    print(sortie.stdout, end='', flush=True)
    if sortie.returncode != 0:
        print(sortie.stderr, end='', flush=True)
        QMessageBox.warning(None, 'Extrinsèque SVPRO',
                            'La calibration SVPRO a échoué — l’ancienne extrinsèque '
                            'reste en service. Détail dans le terminal.')
        return None
    # Le leave-one-out est LU DANS LE YAML ecrit, pas devine dans le texte du
    # script : c'est le champ qui fait foi, et il survit a un changement de
    # formatage de la sortie.
    ecrit = yaml.safe_load(SVPRO.read_text())
    pire = max((float(v['plane_error_mm'])
                for v in ecrit.get('validation_leave_one_out') or []), default=None)
    if pire is None:
        print('SVPRO : aucun leave-one-out ecrit — moins de 4 marqueurs vus', flush=True)
    elif pire > LOO_SVPRO_ATTENDU_MM:
        print(f'SVPRO : leave-one-out {pire:.1f} mm > {LOO_SVPRO_ATTENDU_MM:.0f} mm — '
              'la valider par une SAISIE, pas par le residu (15/09)', flush=True)
    else:
        print(f'SVPRO : leave-one-out {pire:.1f} mm', flush=True)
    return pire


def sans_test_de_saisie():
    """Le test de saisie appartient a la carte de correction, qui n'est plus la.

    `calibration_dialogue.demande` le propose toujours, en nommant « balle,
    scotch, robot » — des objets qui ne sont plus sur le banc. Sans carte pour
    l'exploiter, la question n'a plus d'objet du tout.
    """
    calibration_dialogue.propose_test = lambda recalibree: False


def branche_tri_fluide(fenetre):
    """Bouton « tri fluide » : le geste de `tri_fluide.py`, avec les detections du dashboard.

    Positions mesurees bras degage (pose d'observation), puis un geste par piece :
    approche sans arret, prise, remontee, transfert direct, descente lente, pose au
    fond du bac de SA couleur. Pas de recalage pendant l'approche, ou la camera de
    dessus est masquee par le bras. STOP arrete la boucle entre deux pieces ; une
    prise a vide l'arrete aussi, sans nouvel essai.
    """
    import json
    import threading
    import time

    import numpy as np
    from PyQt5.QtWidgets import QPushButton

    import pick_fsm as fsm
    import tri_fluide

    arret = threading.Event()
    en_cours = threading.Lock()

    def pieces_a_trier():
        with fenetre._verrou:
            objets = list(fenetre._objets_vus)
        bacs = {}
        connus = (json.loads(tri_fluide.BACS_CONNUS.read_text())
                  if tri_fluide.BACS_CONNUS.exists() else {})
        for bac in yolo26_dashboard.BACS:
            vu = fenetre.suivi_cartons[bac].position()
            if vu is not None:
                bacs[bac] = np.asarray(vu[0], float)
                connus[bac] = [round(float(v), 1) for v in bacs[bac]]
            elif bac in connus:
                # Un bac ne bouge pas : yolo26 sort le bac jaune vide sous 0,5 une image
                # sur deux. Sa derniere position sure tient lieu de detection.
                bacs[bac] = np.asarray(connus[bac], float)
                print(f'tri fluide : {bac} non vu, derniere position sure {connus[bac]}', flush=True)
        tri_fluide.BACS_CONNUS.parent.mkdir(parents=True, exist_ok=True)
        tri_fluide.BACS_CONNUS.write_text(json.dumps(connus))
        # Hors portee = fausse detection : la poubelle verte au sol, vue « cylindre_vert »
        # a (10 ; -845) mm le 14/10, lancait un essai sans pose atteignable a chaque tri.
        # Borne HAUTE seulement : avec PORTEE_MIN aussi, le vrai cylindre a 164 mm de la
        # base n'etait plus tente (15/10), alors que tri_fluide le prenait avant ce filtre.
        hors_portee = [(p, xy) for p, xy in objets if p in tri_fluide.PIECES
                       and float(np.hypot(*xy)) > fsm.PORTEE_MAX]
        for p, xy in hors_portee:
            print(f'tri fluide : {p} ignore, hors portee ({xy[0]:.0f} ; {xy[1]:.0f}) mm', flush=True)
        # Une piece deja posee reste visible AU FOND de son bac : elle n'est plus a trier.
        return sorted((float(np.hypot(*xy)), piece, np.asarray(xy, float), bacs[tri_fluide.PIECES[piece]])
                      for piece, xy in objets
                      if piece in tri_fluide.PIECES and tri_fluide.PIECES[piece] in bacs
                      and float(np.hypot(*xy)) <= fsm.PORTEE_MAX
                      and all(np.linalg.norm(np.asarray(xy, float) - b) > 50.0 for b in bacs.values()))

    def boucle():
        ctx = fenetre.ctx
        ctx.z_rebord = yolo26_dashboard.HAUTEUR_BAC
        if fsm.porte_objet(ctx):
            print('tri fluide : la pince tient deja un objet — le deposer d abord', flush=True)
            en_cours.release()
            return
        while not arret.is_set():
            fsm.va_vers_par_etapes(ctx, fsm.POSE_OBSERVATION, nom='observation', stabilise=False)
            time.sleep(2.0)
            restantes = pieces_a_trier()
            if not restantes:
                print('tri fluide : plus rien a trier', flush=True)
                break
            _, piece, xy, bac = restantes[0]
            if not tri_fluide.trie(ctx, piece, xy, bac, 'actuelle'):
                print('tri fluide : arret', flush=True)
                break
        en_cours.release()

    def lance():
        if fenetre.ouvrier is not None and fenetre.ouvrier.isRunning():
            print('tri fluide refuse : la boucle du dashboard tourne — STOP d abord', flush=True)
            return
        if not en_cours.acquire(blocking=False):
            return
        fenetre._assure_pont()
        arret.clear()
        threading.Thread(target=boucle, daemon=True).start()

    # « demarrer la boucle » lance le TRI FLUIDE, plus la boucle du FSM : c'est le
    # geste qui a trie les 4 pieces du premier coup le 13/10, alors que la boucle
    # du FSM s'est bloquee en SAISIE sur le meme banc. Un seul chemin, pas deux.
    fenetre.bouton_auto.clicked.disconnect()
    fenetre.bouton_auto.clicked.connect(lance)
    fenetre.bouton_auto.setToolTip('tri fluide : un geste par piece, pose au fond de son bac')
    fenetre.mode.setCurrentText('automatique')
    for b in fenetre.findChildren(QPushButton):
        if 'STOP' in b.text():
            b.clicked.connect(arret.set)
    return lance


def main():
    a = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument('--sans-question', action='store_true',
                   help='ouvrir le dashboard sans proposer de calibration')
    a.add_argument('--sans-svpro', action='store_true',
                   help='ne pas proposer la calibration de la SVPRO')
    a.add_argument('--svpro-dream', action='store_true',
                   help='SVPRO reservee a DREAM : ni calibree, ni utilisee par YOLO')
    a.add_argument('--dream', action='store_true',
                   help='DREAM (vgg_tri_reel_ft_e10) a cote du tri : pince DREAM, YOLO, codeurs')
    a.add_argument('--automatique', action='store_true',
                   help='demarrer seul la boucle automatique (le bras bouge sans clic)')
    a.add_argument('--sans-yolo', action='store_true',
                   help='detecteurs couleur/forme d origine (diagnostic)')
    a.add_argument('--modele', type=Path,
                   help='poids yolo26 (defaut : le run pieces_v3, 13 arducam + 4 SVPRO)')
    a.add_argument('--seuil', type=float,
                   help='confiance minimale (defaut : 0,10, mesure sur modele entraine)')
    a.add_argument('--inventaire', nargs='+', default=[], metavar='CLASSE=N',
                   help='nombre de pieces a trier par classe, ex. cube_rouge=2')
    args = a.parse_args()

    connues = set(yolo26_dashboard.OBJETS)
    inventaire = {}
    for terme in args.inventaire:
        classe, _, nombre = terme.partition('=')
        if classe not in connues or not nombre.isdigit():
            a.error(f'--inventaire : « {terme} » — attendu {"|".join(sorted(connues))}=N')
        inventaire[classe] = int(nombre)

    application = QApplication(sys.argv)
    sans_test_de_saisie()
    # UN seul « oui » calibre les DEUX cameras. `demande` rend le resultat de
    # l'arducam, ou None si l'etape a ete sautee ou refusee : la SVPRO suit le
    # meme sort, sans seconde question.
    resultat = calibration_dialogue.demande(sauter=args.sans_question)
    if resultat is not None and not (args.sans_svpro or args.svpro_dream):
        calibre_svpro()

    import pick_dashboard

    if not args.sans_yolo:
        print('yolo26 : chargement du modele (quelques secondes)…', flush=True)
        service = yolo26_dashboard.branche(pick_dashboard, inventaire=inventaire,
                                           poids=args.modele, seuil=args.seuil,
                                           arducam_seule=args.svpro_dream)
        cameras = 'arducam SEULE (SVPRO reservee a DREAM)' if args.svpro_dream else 'arducam ET SVPRO'
        print(f'yolo26 : {service.poids}, seuil {service.seuil} — {cameras}', flush=True)
        print('tri : chaque piece dans le bac de sa couleur — '
              + ', '.join(f'{p} -> {b}' for p, b in sorted(
                  yolo26_dashboard.DESTINATION.items())), flush=True)
    else:
        pick_dashboard.INVENTAIRE.update(inventaire)

    tient_l_exposition(pick_dashboard)

    panneau = None
    if args.dream:
        import dream_tri_panneau
        print('DREAM : chargement du modele (quelques secondes)…', flush=True)
        panneau = dream_tri_panneau.installe(pick_dashboard)

    fenetre = pick_dashboard.Fenetre()
    lance_tri = branche_tri_fluide(fenetre)
    if panneau is not None:
        vue = dream_tri_panneau.FenetreTriDream(panneau, fenetre)
        vue.showMaximized()
    else:
        fenetre.show()
    if args.automatique:
        # Le delai laisse aux cameras et a yolo26 le temps de rendre leurs
        # premieres detections : la boucle commence par DETECTION.
        def demarre():
            print('tri fluide demarre (--automatique)', flush=True)
            lance_tri()
        QTimer.singleShot(8000, demarre)
    sys.exit(application.exec_())


if __name__ == '__main__':
    main()
