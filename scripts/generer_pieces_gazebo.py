#!/usr/bin/env python3
"""Genere les 8 modeles Gazebo des pieces peintes, depuis `tri_couleur`.

Les cotes ne sont PAS retapees ici : elles sont lues dans `tri_couleur.py`, qui
porte deja le dossier de fabrication (plans_cotes, feuilles 2 a 5) et sert de
reference a yolo26, au tri reel et aux hauteurs de prise. Une piece redimensionnee
la-bas se propage ici par une simple regeneration ; recopiee, elle divergerait en
silence — et c'est la simulation qui aurait tort sans qu'on le voie.

    /usr/bin/python3 scripts/generer_pieces_gazebo.py

Repere local de chaque modele : origine au CENTRE DE LA BASE, +Z vers le haut.
Pose un objet a Z=0 sur la planche et il repose dessus, sans calcul de demi-hauteur.
"""
from pathlib import Path
import sys

import cv2

RACINE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(RACINE / 'scripts'))

import tri_couleur as tc

MODELES = RACINE / 'mycobot_description' / 'models'
DENSITE = 500.0          # kg/m3 — bois peint / impression 3D remplie
EPAISSEUR_BAC = 0.002    # m — paroi du bac

GABARIT_CONFIG = """<?xml version="1.0"?>
<!-- Genere par scripts/generer_pieces_gazebo.py — regenerer plutot qu'editer. -->
<model>
  <name>{nom}</name>
  <version>1.0</version>
  <sdf version="1.9">model.sdf</sdf>
  <description>{description}</description>
</model>
"""


# Saturation et valeur de la peinture. `tri_couleur` mesure les huit pieces entre
# 226 et 255 de saturation (le bac bleu, le moins sature, a 178) : 240 est au
# milieu de la bande. La valeur est celle d'une peinture mate bien eclairee.
SATURATION_PEINTURE = 240
VALEUR_PEINTURE = 165


def rgb(couleur):
    """Teinte MESUREE SUR LA PEINTURE -> triplet RGB normalise pour SDF.

    On part de `tc.TEINTES` et non de `tc.BGR`. L'en-tete de `tri_couleur` le dit
    en toutes lettres : les H sont « MESURES sur la peinture, et non tires du
    SDF », tandis que `BGR` n'est qu'une couleur de TRACE pour les apercus. Les
    deux divergent — le vert de trace est a H=60 quand la peinture est a 39 — et
    c'est la peinture que yolo26 a apprise. Le premier jet rendait le banc avec
    les couleurs de trace : le bac jaune n'etait alors pas reconnu du tout.
    """
    import numpy as np
    h = tc.TEINTES[couleur]
    hsv = np.uint8([[[h, SATURATION_PEINTURE, VALEUR_PEINTURE]]])
    b, g, r = cv2.cvtColor(hsv, cv2.COLOR_HSV2BGR)[0][0]
    return r / 255.0, g / 255.0, b / 255.0


def materiau(couleur, indent):
    r, g, b = rgb(couleur)
    pad = ' ' * indent
    return (f'{pad}<material>\n'
            f'{pad}  <ambient>{r:.3f} {g:.3f} {b:.3f} 1</ambient>\n'
            f'{pad}  <diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>\n'
            f'{pad}  <specular>0.1 0.1 0.1 1</specular>\n'
            f'{pad}</material>')


def inertie_boite(masse, lx, ly, lz):
    k = masse / 12.0
    return k * (ly*ly + lz*lz), k * (lx*lx + lz*lz), k * (lx*lx + ly*ly)


def piece(couleur):
    """Le solide a saisir : boite ou cylindre, selon le dossier de fabrication."""
    nom = tc.NOM_PIECE[couleur]
    _, dessus = tc.OBJET_PAR_COULEUR[couleur]
    lx, ly, lz = (d / 1000.0 for d in dessus[0])
    rond = 'cylindre' in tc.OBJET_PAR_COULEUR[couleur][0]
    if rond:
        import math
        rayon = lx / 2.0
        volume = math.pi * rayon * rayon * lz
        geo = (f'<cylinder><radius>{rayon:.6f}</radius>'
               f'<length>{lz:.6f}</length></cylinder>')
    else:
        volume = lx * ly * lz
        geo = f'<box><size>{lx:.6f} {ly:.6f} {lz:.6f}</size></box>'
    masse = DENSITE * volume
    if rond:
        ixx = iyy = masse * (3 * rayon * rayon + lz * lz) / 12.0
        izz = masse * rayon * rayon / 2.0
    else:
        ixx, iyy, izz = inertie_boite(masse, lx, ly, lz)
    forme = (f'        <geometry>{geo}</geometry>')
    return f"""<?xml version="1.0"?>
<!-- Genere par scripts/generer_pieces_gazebo.py — regenerer plutot qu'editer.
     Cotes lues dans tri_couleur.OBJET_PAR_COULEUR['{couleur}'] : {dessus[0]} mm.
     Origine au CENTRE DE LA BASE : une pose a Z=0 repose sur la planche. -->
<sdf version="1.9">
  <model name="{nom}">
    <link name="corps">
      <pose>0 0 {lz / 2.0:.6f} 0 0 0</pose>
      <inertial>
        <mass>{masse:.6f}</mass>
        <inertia>
          <ixx>{ixx:.9f}</ixx><iyy>{iyy:.9f}</iyy><izz>{izz:.9f}</izz>
          <ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>
        </inertia>
      </inertial>
      <collision name="collision">
{forme}
        <surface>
          <friction><ode><mu>1.0</mu><mu2>1.0</mu2></ode></friction>
        </surface>
      </collision>
      <visual name="visual">
{forme}
{materiau(couleur, 8)}
      </visual>
    </link>
  </model>
</sdf>
"""


def bac(couleur):
    """Bac ouvert : un fond et quatre parois, aux cotes de tri_couleur.BAC."""
    cote_x, cote_y, haut = (d / 1000.0 for d in tc.BAC)
    e = EPAISSEUR_BAC
    murs = [
        ('fond', cote_x, cote_y, e, 0.0, 0.0, e / 2.0),
        ('paroi_x_plus', e, cote_y, haut, (cote_x - e) / 2.0, 0.0, haut / 2.0),
        ('paroi_x_moins', e, cote_y, haut, -(cote_x - e) / 2.0, 0.0, haut / 2.0),
        ('paroi_y_plus', cote_x - 2 * e, e, haut, 0.0, (cote_y - e) / 2.0, haut / 2.0),
        ('paroi_y_moins', cote_x - 2 * e, e, haut, 0.0, -(cote_y - e) / 2.0, haut / 2.0),
    ]
    blocs = []
    for nom, sx, sy, sz, px, py, pz in murs:
        geo = (f'        <geometry><box><size>{sx:.6f} {sy:.6f} {sz:.6f}'
               f'</size></box></geometry>')
        blocs.append(
            f'      <collision name="{nom}_collision">\n'
            f'        <pose>{px:.6f} {py:.6f} {pz:.6f} 0 0 0</pose>\n{geo}\n'
            f'      </collision>\n'
            f'      <visual name="{nom}">\n'
            f'        <pose>{px:.6f} {py:.6f} {pz:.6f} 0 0 0</pose>\n{geo}\n'
            f'{materiau(couleur, 8)}\n'
            f'      </visual>')
    return f"""<?xml version="1.0"?>
<!-- Genere par scripts/generer_pieces_gazebo.py — regenerer plutot qu'editer.
     Cotes lues dans tri_couleur.BAC : {tc.BAC} mm, paroi {EPAISSEUR_BAC * 1000:.0f} mm.
     Fixe (static) : un bac deplace par un choc invaliderait le largage. -->
<sdf version="1.9">
  <model name="bac_{couleur}">
    <static>true</static>
    <link name="corps">
{chr(10).join(blocs)}
    </link>
  </model>
</sdf>
"""


def ecrit(nom, sdf, description):
    dossier = MODELES / nom
    dossier.mkdir(parents=True, exist_ok=True)
    (dossier / 'model.sdf').write_text(sdf)
    (dossier / 'model.config').write_text(
        GABARIT_CONFIG.format(nom=nom, description=description))
    print(f'  {nom:16s} {dossier.relative_to(RACINE)}')


def main():
    print(f'8 modeles depuis tri_couleur.py -> {MODELES.relative_to(RACINE)}')
    for couleur in tc.TEINTES:
        lx, ly, lz = tc.OBJET_PAR_COULEUR[couleur][1][0]
        ecrit(tc.NOM_PIECE[couleur], piece(couleur),
              f'Piece peinte {couleur} : {lx:.0f} x {ly:.0f} x {lz:.0f} mm.')
        ecrit(f'bac_{couleur}', bac(couleur),
              f'Bac {couleur} ouvert : {tc.BAC[0]:.0f} x {tc.BAC[1]:.0f} x '
              f'{tc.BAC[2]:.0f} mm.')
    print(f'{len(tc.TEINTES) * 2} modeles ecrits — les 8 classes de yolo26.')


if __name__ == '__main__':
    main()
