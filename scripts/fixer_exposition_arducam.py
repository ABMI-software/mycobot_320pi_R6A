#!/usr/bin/env python3
"""Maintient l'arducam a l'exposition 75, en manuel, quoi qu'il arrive.

    /usr/bin/python3 scripts/fixer_exposition_arducam.py            # en continu
    /usr/bin/python3 scripts/fixer_exposition_arducam.py --une-fois # pose et sort

Toutes les mesures du banc sont prises a l'exposition 75 : extrinseques,
detections yolo26, campagnes de precision. Une image prise a une autre
exposition n'est pas comparable a celles-la.

Ce que ce programme corrige, et que poser la valeur une fois ne corrige pas :
le pilote UVC RELACHE le reglage manuel tout seul. Ses defauts sont
`auto_exposure = 3` (automatique) et `exposure_time_absolute = 157`, et c'est
exactement l'image lavee qu'on voit quand il repart dessus. Le declencheur
connu est l'ouverture du peripherique par un SECOND processus.

La camera est identifiee par le NOM DE SA CARTE, jamais par son index :
/dev/video2 devient /dev/video3 au rebranchement.

Ecrire un controle V4L2 ne demande pas l'acces exclusif : ce programme tourne
en meme temps que le tableau de bord, sans lui prendre la camera — il ne
l'ouvre jamais, donc il ne provoque pas lui-meme la bascule qu'il corrige.

**La pose passe par `pick_dashboard.regle_exposition`, pas par un v4l2-ctl
ecrit ici.** Sa docstring porte le piege : il faut passer en manuel AVANT de
poser le temps d'exposition, en DEUX appels. Tout ecrire d'un coup laisse le
pilote stocker la valeur — une relecture repond bien 75 — tout en continuant
a exposer en automatique. On lit alors 75 sur une image lavee.
"""
import argparse
import subprocess
import sys
import time
from pathlib import Path

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE / 'scripts'))

import pick_dashboard as pd                                            # noqa: E402

CARTE = 'Arducam_8mp'
VOULU = {'auto_exposure': 1, 'exposure_time_absolute': 75}
PERIODE_S = 1.0


def peripherique():
    """/dev/videoN dont la carte est l'arducam, ou None."""
    for chemin in sorted(Path('/dev').glob('video*')):
        info = subprocess.run(['v4l2-ctl', '-d', str(chemin), '--info'],
                              capture_output=True, text=True)
        if CARTE not in info.stdout:
            continue
        # Un peripherique UVC expose plusieurs noeuds ; seul celui qui porte les
        # controles d'exposition est le bon.
        lu = subprocess.run(['v4l2-ctl', '-d', str(chemin), '-C', 'exposure_time_absolute'],
                            capture_output=True, text=True)
        if 'exposure_time_absolute' in lu.stdout:
            return chemin
    return None


def lit(dev):
    sortie = subprocess.run(['v4l2-ctl', '-d', str(dev), '-C', ','.join(VOULU)],
                            capture_output=True, text=True).stdout
    etat = {}
    for ligne in sortie.splitlines():
        if ':' in ligne:
            nom, valeur = ligne.split(':', 1)
            etat[nom.strip()] = int(valeur.strip().split()[0])
    return etat


def pose(dev):
    """La recette du projet, en deux appels — voir l'en-tete."""
    pd.regle_exposition(int(str(dev).removeprefix('/dev/video')),
                        VOULU['exposure_time_absolute'])


def main():
    a = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument('--une-fois', action='store_true', help='poser la valeur et sortir')
    args = a.parse_args()

    dev = peripherique()
    if dev is None:
        raise SystemExit(f'aucune camera « {CARTE} » trouvee')
    pose(dev)
    etat = lit(dev)
    print(f'{dev} ({CARTE}) — ' + '  '.join(f'{n}={etat.get(n)}' for n in VOULU), flush=True)
    if args.une_fois:
        return

    # REPOSE INCONDITIONNELLE, et non « seulement si la relecture a derive ».
    # La relecture rend la valeur STOCKEE, pas celle que le capteur applique :
    # les deux se separent des qu'un reglage a ete pose dans le mauvais ordre,
    # ou qu'un second processus a ouvert le peripherique. On lit alors 75 sur
    # une image lavee, la garde ne voit rien a corriger, et n'ecrit jamais.
    # Une garde aveugle a la panne qu'elle surveille ne sert a rien ; reposer
    # coute deux appels v4l2-ctl par seconde et ne prend pas la camera.
    poses, signale = 0, None
    print(f'repose inconditionnelle toutes les {PERIODE_S:.0f} s — Ctrl+C pour arreter',
          flush=True)
    while True:
        time.sleep(PERIODE_S)
        etat = lit(dev)
        derive = {n: etat.get(n) for n, v in VOULU.items() if etat.get(n) != v}
        pose(dev)
        poses += 1
        # On ne journalise que les CHANGEMENTS d'etat : une ligne par seconde
        # noierait la seule chose qui compte, le moment ou ca lache.
        if derive != signale:
            signale = derive
            marque = f'VALEUR STOCKEE DERIVEE {derive}' if derive else 'valeur stockee conforme'
            print(f'{time.strftime("%H:%M:%S")}  {marque} — reposee '
                  f'({poses} poses depuis le debut)', flush=True)


if __name__ == '__main__':
    main()
