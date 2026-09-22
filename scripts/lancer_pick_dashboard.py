#!/usr/bin/env python3
"""Lance le dashboard de pick-and-place en proposant d abord de recalibrer.

    conda deactivate
    /usr/bin/python3 scripts/lancer_pick_dashboard.py

Depuis que `pick_dashboard.main()` pose lui-meme la question, ce lanceur ne
sert plus qu a deux choses : garder la commande historique valable, et offrir
les deux drapeaux qui sautent une etape.

La fenetre elle-meme vit dans `calibration_dialogue.py`, la carte de
correction dans `correction_vision.py`.
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


def main():
    a = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument('--sans-question', action='store_true',
                   help='ouvrir le dashboard sans proposer la calibration')
    a.add_argument('--sans-correction', action='store_true',
                   help='ne pas appliquer la carte de correction IDW')
    args = a.parse_args()

    application = QApplication(sys.argv)
    calibration_dialogue.demande(sauter=args.sans_question)

    import pick_dashboard

    if not args.sans_correction:
        correction_vision.branche(pick_dashboard.Vision)

    fenetre = pick_dashboard.Fenetre()
    fenetre.show()
    sys.exit(application.exec_())


if __name__ == '__main__':
    main()
