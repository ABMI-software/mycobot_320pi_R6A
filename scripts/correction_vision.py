#!/usr/bin/env python3
"""Carte de correction vision -> realite, par ponderation inverse a la distance.

Ce que ca corrige. L'extrinseque place la camera ; elle ne corrige pas ce qui
reste APRES : distorsion residuelle, plan de table qui n'est pas tout a fait le
plan Z=0 du modele, centre apparent de la balle qui n'est pas son point de
contact, degenerescence des marqueurs coplanaires. Ces restes ne sont pas
aleatoires — ils varient LENTEMENT d'un bout a l'autre de la planche. Un
decalage unique les moyennerait et se tromperait partout ; une carte les suit.

La methode. Shepard, ponderation inverse a la distance (IDW). Soit des
echantillons (vision_i, reel_i) et leur ecart d_i = reel_i - vision_i. En un
point p :

    correction(p) = somme(w_i * d_i) / somme(w_i)      w_i = 1 / dist(p, p_i)^k

C'est un interpolateur exact : sur un echantillon, il rend son ecart. Entre
deux, il transitionne. Il n'EXTRAPOLE pas — hors de portee de tout echantillon
il tendrait vers la moyenne globale, ce qui serait une correction inventee :
d'ou `rayon_mm`, au-dela duquel on ne corrige pas du tout.

Trois garde-fous, chacun paye d'une erreur possible :

  - `rayon_mm`  — aucun echantillon a portee, aucune correction. Mieux vaut
    l'erreur connue de la vision qu'une correction extrapolee.
  - `ecart_max_mm` — une correction plus grande que ca vient d'un echantillon
    faux, pas d'un defaut de vision. On la borne.
  - validation leave-one-out (`--valider`) — chaque echantillon est predit par
    les AUTRES. Un ajustement qui reproduit ses propres points ne mesure rien.

D'ou vient « reel ». Du robot, pas d'une deuxieme vision : apres une saisie
CONFIRMEE (statut de la pince, pas l'accuse de commande), la balle etait la ou
la pince s'est fermee — FK de la pose de fermeture. A defaut, une mesure au
reglet depuis un marqueur.

    python3 scripts/correction_vision.py --ajouter --vision 312.4 -45.1 \
                                         --reel 315.0 -43.6 --note "saisie ok"
    python3 scripts/correction_vision.py --liste
    python3 scripts/correction_vision.py --valider
    python3 scripts/correction_vision.py --carte
"""
from __future__ import annotations

import argparse
import json
from datetime import datetime
from pathlib import Path

import numpy as np

RACINE = Path(__file__).resolve().parents[1]
FICHIER = RACINE / 'scripts' / 'correction_vision.json'

PUISSANCE = 2.0        # k de Shepard. 1 = tres lisse, 3 = presque du plus-proche-voisin
RAYON_MM = 250.0       # au-dela, pas de correction du tout
ECART_MAX_MM = 25.0    # borne : au-dela l'echantillon est faux, pas la vision
COINCIDENCE_MM = 0.5   # en deca, on est SUR l'echantillon


def _defaut():
    return {'puissance': PUISSANCE, 'rayon_mm': RAYON_MM,
            'ecart_max_mm': ECART_MAX_MM, 'echantillons': []}


def charge(fichier: Path = FICHIER) -> dict:
    try:
        d = json.loads(fichier.read_text())
    except (OSError, json.JSONDecodeError):
        return _defaut()
    base = _defaut()
    base.update(d)
    return base


def enregistre(d: dict, fichier: Path = FICHIER):
    fichier.write_text(json.dumps(d, indent=2, ensure_ascii=False))


def ajoute(vision_xy, reel_xy, note='', camera_xyz=None, fichier: Path = FICHIER):
    d = charge(fichier)
    d['echantillons'].append({
        'vision': [round(float(vision_xy[0]), 2), round(float(vision_xy[1]), 2)],
        'reel': [round(float(reel_xy[0]), 2), round(float(reel_xy[1]), 2)],
        'ecart': [round(float(reel_xy[0] - vision_xy[0]), 2),
                  round(float(reel_xy[1] - vision_xy[1]), 2)],
        'date': f'{datetime.now():%Y-%m-%d %H:%M}',
        'camera': ([round(float(c), 1) for c in camera_xyz]
                   if camera_xyz is not None else None),
        'note': note,
    })
    enregistre(d, fichier)
    return d


def _tableaux(d):
    ech = d['echantillons']
    if not ech:
        return np.zeros((0, 2)), np.zeros((0, 2))
    p = np.array([e['vision'] for e in ech], float)
    delta = np.array([[e['reel'][0] - e['vision'][0],
                       e['reel'][1] - e['vision'][1]] for e in ech], float)
    return p, delta


def _shepard(xy, points, deltas, puissance, rayon, borne):
    if len(points) == 0:
        return np.zeros(2)
    dist = np.linalg.norm(points - np.asarray(xy, float), axis=1)
    colle = dist < COINCIDENCE_MM
    if colle.any():
        d = deltas[colle].mean(axis=0)
    else:
        proche = dist <= rayon
        if not proche.any():
            return np.zeros(2)
        w = 1.0 / dist[proche] ** puissance
        d = (w[:, None] * deltas[proche]).sum(axis=0) / w.sum()
    norme = float(np.linalg.norm(d))
    return d * (borne / norme) if norme > borne else d


class Carte:
    """Carte chargee une fois, interrogee a chaque detection."""

    def __init__(self, fichier: Path = FICHIER):
        d = charge(fichier)
        self.puissance = float(d['puissance'])
        self.rayon = float(d['rayon_mm'])
        self.borne = float(d['ecart_max_mm'])
        self.points, self.deltas = _tableaux(d)
        self.source = fichier.name
        self.echantillons = d['echantillons']

    def __len__(self):
        return len(self.points)

    def ecart(self, xy):
        return _shepard(xy, self.points, self.deltas, self.puissance,
                        self.rayon, self.borne)

    def corrige(self, xy):
        return np.asarray(xy, float) + self.ecart(xy)

    def valide(self):
        """Leave-one-out : chaque echantillon predit par les autres.

        Rend (erreurs par echantillon, erreur sans correction). Si la premiere
        n'est pas nettement plus basse que la seconde, la carte n'apporte rien.
        """
        erreurs, brutes = [], []
        for i in range(len(self.points)):
            m = np.ones(len(self.points), bool)
            m[i] = False
            predit = _shepard(self.points[i], self.points[m], self.deltas[m],
                              self.puissance, self.rayon, self.borne)
            erreurs.append(float(np.linalg.norm(self.deltas[i] - predit)))
            brutes.append(float(np.linalg.norm(self.deltas[i])))
        return np.array(erreurs), np.array(brutes)


def branche(classe_vision, fichier: Path = FICHIER):
    """Pose la carte sur `Vision.balle`, et sur rien d'autre.

    La balle est la seule grandeur dont on ait des couples (vision, reel)
    pour apprendre l'ecart : l'appliquer aux cartons ou aux marqueurs
    reviendrait a leur imposer une carte apprise sur autre chose.

    Rend la carte. Une carte vide ne touche a rien et le dit en le rendant :
    `len(carte) == 0`.
    """
    carte = Carte(fichier)
    if not len(carte):
        return carte
    origine = classe_vision.balle

    def balle(self, image):
        vu = origine(self, image)
        if vu is None:
            return None
        xy, tache = vu
        return carte.corrige(xy), tache

    classe_vision.balle = balle
    return carte


def main():
    a = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    a.add_argument('--ajouter', action='store_true')
    a.add_argument('--vision', nargs=2, type=float, metavar=('X', 'Y'))
    a.add_argument('--reel', nargs=2, type=float, metavar=('X', 'Y'))
    a.add_argument('--note', default='')
    a.add_argument('--liste', action='store_true')
    a.add_argument('--valider', action='store_true')
    a.add_argument('--carte', action='store_true')
    a.add_argument('--ou', nargs=2, type=float, metavar=('X', 'Y'),
                   help='afficher la correction en ce point')
    args = a.parse_args()

    if args.ajouter:
        if not (args.vision and args.reel):
            raise SystemExit('--ajouter exige --vision X Y et --reel X Y')
        d = ajoute(args.vision, args.reel, args.note)
        e = d['echantillons'][-1]
        print(f"echantillon {len(d['echantillons'])} : vision {e['vision']} -> "
              f"reel {e['reel']}, ecart ({e['ecart'][0]:+.2f}, {e['ecart'][1]:+.2f}) mm")
        return

    carte = Carte()
    if args.liste or not (args.valider or args.carte or args.ou):
        print(f'{len(carte)} echantillon(s) — puissance {carte.puissance}, '
              f'rayon {carte.rayon:.0f} mm, borne {carte.borne:.0f} mm')
        for i, e in enumerate(carte.echantillons, 1):
            print(f"  {i:2d}  vision ({e['vision'][0]:7.1f}, {e['vision'][1]:7.1f}) "
                  f"ecart ({e['ecart'][0]:+6.2f}, {e['ecart'][1]:+6.2f}) mm  "
                  f"{e['date']}  {e['note']}")
        if not len(carte):
            print('  aucun — la correction est neutre tant qu il n y en a pas')

    if args.ou:
        d = carte.ecart(args.ou)
        p = carte.corrige(args.ou)
        print(f'en ({args.ou[0]:.1f}, {args.ou[1]:.1f}) : correction '
              f'({d[0]:+.2f}, {d[1]:+.2f}) mm -> ({p[0]:.1f}, {p[1]:.1f})')

    if args.valider:
        if len(carte) < 3:
            raise SystemExit('au moins 3 echantillons pour valider')
        err, brut = carte.valide()
        print(f'\nleave-one-out sur {len(carte)} echantillons :')
        for i, (e, b) in enumerate(zip(err, brut), 1):
            print(f'  {i:2d}  sans correction {b:5.2f} mm  ->  predit par les '
                  f'autres {e:5.2f} mm')
        print(f'  moyenne {brut.mean():.2f} mm -> {err.mean():.2f} mm  '
              f'| pire {brut.max():.2f} -> {err.max():.2f}')
        if err.mean() >= brut.mean():
            print('  -> la carte n APPORTE RIEN sur ces points : les ecarts ne '
                  'sont pas spatialement coherents, ou il en manque.')

    if args.carte:
        print('\ncorrection interpolee, pas de 100 mm (norme en mm) :')
        for y in range(200, -201, -100):
            ligne = []
            for x in range(100, 551, 100):
                n = float(np.linalg.norm(carte.ecart((x, y))))
                ligne.append('   .  ' if n == 0 else f'{n:5.2f} ')
            print(f'  Y={y:+5d}  ' + ''.join(ligne))
        print('        X=' + ''.join(f'{x:6d}' for x in range(100, 551, 100)))


if __name__ == '__main__':
    main()
