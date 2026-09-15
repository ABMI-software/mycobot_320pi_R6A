#!/usr/bin/env python3
"""Lance le dashboard de pick-and-place en proposant d abord de recalibrer.

    conda deactivate
    /usr/bin/python3 scripts/lancer_pick_dashboard.py

Enchainement, identique a `pick_dashboard.py` : calibration extrinseque (oui /
non) -> test de saisie (oui / non) -> dashboard. Le test de saisie vit dans
`calibration_dialogue.py` et `correction_vision.py`.

Ce lanceur ajoute la TOURNEE DES 4 COINS : quatre cercles dans la vue arducam,
balle posee sur un cercle, un cycle, et l'ecart reel - vision de ce coin entre
dans la carte. Elle couvre toute la zone de prise la ou le test n'apprend qu'un
point.

    /usr/bin/python3 scripts/lancer_pick_dashboard.py --sans-question --tournee

`--yolo` remplace la detection des objets de l'arducam par YOLOE-26
(`yolo_dashboard.py`) : cylindre -> petit carton, tout autre objet -> grand
carton. `--inventaire robot=4 scotch=2` dit combien il y en a a trier, sans quoi
une categorie est « finie » des le premier depot.

    /usr/bin/python3 scripts/lancer_pick_dashboard.py --yolo --inventaire robot=4 scotch=2
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE / 'scripts'))

from PyQt5.QtWidgets import QApplication                                # noqa: E402

import calibration_dialogue                                            # noqa: E402
import correction_vision                                               # noqa: E402


def dessine_coins(module_dashboard, carte):
    """Les coins sur la vue arducam, sans toucher a `pick_dashboard.py`."""
    origine = module_dashboard.Fenetre._affiche

    def _affiche(self, nom, image):
        if nom == 'arducam':
            correction_vision.dessine_tournee(image, self.vision, carte.tournee, carte.z_mm)
        origine(self, nom, image)

    module_dashboard.Fenetre._affiche = _affiche


def main():
    a = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument('--sans-question', action='store_true',
                   help='ouvrir le dashboard sans proposer calibration ni test')
    a.add_argument('--sans-correction', action='store_true',
                   help='ne pas appliquer la carte de correction IDW')
    a.add_argument('--tournee', action='store_true',
                   help='armer la tournee des 4 coins (ouvre la pince sur chaque coin)')
    a.add_argument('--yolo', action='store_true',
                   help='objets de l arducam detectes par YOLOE-26 (weights/yoloe/, .venv)')
    a.add_argument('--inventaire', nargs='+', default=[], metavar='CLASSE=N',
                   help='nombre d objets a trier par classe, ex. robot=4 scotch=2')
    args = a.parse_args()
    if args.tournee and args.sans_correction:
        a.error('--tournee exige la carte : retirer --sans-correction')
    inventaire = {}
    for terme in args.inventaire:
        classe, _, nombre = terme.partition('=')
        if classe not in ('balle', 'scotch', 'robot') or not nombre.isdigit():
            a.error(f'--inventaire : « {terme} » — attendu balle|scotch|robot=N')
        inventaire[classe] = int(nombre)

    application = QApplication(sys.argv)
    calibration_dialogue.demande(sauter=args.sans_question)

    import pick_dashboard

    if args.yolo:
        import yolo_dashboard
        print('YOLOE-26 : chargement des modeles (quelques secondes)…', flush=True)
        yolo_dashboard.branche(pick_dashboard)
        print('YOLOE-26 : objets de l arducam detectes par yolo_service.py', flush=True)
    pick_dashboard.INVENTAIRE.update(inventaire)

    if not args.sans_correction:
        carte = correction_vision.branche(
            pick_dashboard.Vision,
            coins=correction_vision.COINS_TOURNEE if args.tournee else None)
        print(f'carte de correction : {len(carte)} echantillon(s)'
              + (' — tournee des 4 coins armee' if args.tournee else '')
              + (' — test de saisie au demarrage' if calibration_dialogue.TEST_SAISIE
                 and not args.tournee else ''), flush=True)
        if args.tournee:
            dessine_coins(pick_dashboard, carte)

    fenetre = pick_dashboard.Fenetre()
    fenetre.show()
    sys.exit(application.exec_())


if __name__ == '__main__':
    main()
