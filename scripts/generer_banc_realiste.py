#!/usr/bin/env python3
"""Banc REALISTE : les 4 pieces peintes et leurs bacs, vus par les cameras du banc.

Part de `real_table.sdf` — le plateau 622 x 449 mm mesure et ses quatre ArUco —
et en fait la scene que yolo26 sait lire :

  * les restes de l'ancien banc (cube rouge, bac rouge de demonstration) partent ;
  * les 4 pieces et leurs 4 bacs arrivent, aux cotes de `tri_couleur.py` ;
  * les marqueurs sont remis aux positions de `planche_actuelle.yaml`, la seule
    reference MESUREE AU ROBOT — le monde portait encore la forme au ruban, fausse
    de 10 a 16 mm ;
  * l'arducam et la SVPRO sont posees a LEUR pose calibree, avec LEURS
    intrinseques. C'est tout l'interet : l'extrinseque du banc reel devient
    valable en simulation, et `Vision.vers_base` rend des millimetres robot sans
    qu'une ligne de la chaine de tri ne change.

    /usr/bin/python3 scripts/generer_monde_yolo26.py

La DISTORSION est le seul point ou la simulation ne suit pas. L'arducam a un
modele rationnel (k1 = 5,4, k3 = -48,2) que Gazebo ne sait pas rendre : son
<distortion> est un plumb-bob a cinq parametres. On ecrit donc des intrinseques
SIMULEES a distorsion nulle, avec les memes fx, fy, cx, cy, et une extrinseque
qui les designe. Melanger les deux — image sans distorsion, coefficients reels —
decalerait chaque point de plusieurs millimetres sans rien signaler.
"""
from pathlib import Path
import json
import sys
import xml.etree.ElementTree as ET

import numpy as np
import yaml
from scipy.spatial.transform import Rotation

RACINE = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(RACINE / 'scripts'))

import tri_couleur as tc

CALIB = RACINE / 'training' / 'calibration'
MONDES = RACINE / 'mycobot_description' / 'worlds'
SOURCE = MONDES / 'real_table.sdf'
SORTIE = MONDES / 'banc_realiste_yolo26.sdf'

# Ogre2 casse au chargement des capteurs sur ce poste : « HLMS Datablock [Hash
# 0x64f0b670] already exists » — les quatre ArUco portent des visuels de memes
# noms (white_cell_1_2, black_square...) et leurs materiaux entrent en collision.
# Ogre v1 n'a pas ce gestionnaire de datablocks.
MOTEUR_RENDU = 'ogre'

# Le monde d'origine est eclaire pour etre regarde, pas pour etre photographie :
# sa luminance moyenne vue par l'arducam simulee monte a 226 avec 14 % de pixels
# ecretes, quand une VRAIE trame de l'arducam a l'exposition 75 est a 76,6
# (training/calibration/arducam_calib_raw.png) et que la fiche de calibration
# demande de verifier ~85. Une scene qui sature efface le contraste local, et
# yolo26 — entraine sur les trames reelles — n'y retrouve plus ses reperes.
# Facteur pose par mesure, puis verifie sur l'image rendue.
# 1.0 = eclairage NORMAL de la scene, celui qu'on regarde. Le baisser (0,33 a
# ete mesure) ramene l'image des cameras a la luminance de l'arducam reelle a
# l'exposition 75 — 76,6 — mais assombrit aussi la vue de Gazebo. A plein
# eclairage la vue est confortable et 14 % des pixels de la camera saturent.
FACTEUR_LUMIERE = 1.0
LUMINANCE_REELLE = 76.6


def attenue_les_lumieres(monde):
    """Ramene l'eclairage a la luminance de l'arducam reelle a l'exposition 75."""
    def echelle(noeud, nom):
        champ = noeud.find(nom)
        if champ is None:
            return
        v = [float(x) for x in champ.text.split()]
        champ.text = ' '.join(f'{x * FACTEUR_LUMIERE:.4f}' if i < 3 else f'{x:.4f}'
                              for i, x in enumerate(v))
    for lumiere in monde.findall('light'):
        echelle(lumiere, 'diffuse')
        echelle(lumiere, 'specular')
    scene = monde.find('scene')
    if scene is not None:
        echelle(scene, 'ambient')
    if FACTEUR_LUMIERE == 1.0:
        print('eclairage NORMAL, identique a real_table.sdf')
    else:
        print(f'lumieres attenuees x{FACTEUR_LUMIERE} — cible {LUMINANCE_REELLE:.0f} '
              f'de luminance, celle de l arducam reelle a l exposition 75')

# Les modeles de l'ancien banc : ils ne portent aucune classe de yolo26 et le
# cube rouge de demonstration se ferait detecter comme une piece.
A_RETIRER = ('red_cube', 'red_bin')

# Bacs : dans l'anneau atteignable (PORTEE_MIN 170, PORTEE_MAX 430), assez
# ecartes pour qu'un largage manque ne tombe pas dans le voisin (105 mm de cote).
BACS_XY = {'rouge': (300.0, 170.0), 'jaune': (395.0, 60.0),
           'vert': (395.0, -60.0), 'bleu': (300.0, -170.0)}
# Pieces : plus pres, dans la zone ou la prise verticale tient
# (PORTEE_VERTICALE_MAX 355), et a plus de 105 mm de tout bac.
PIECES_XY = {'rouge': (200.0, 120.0), 'jaune': (250.0, 55.0),
             'vert': (250.0, -35.0), 'bleu': (200.0, -120.0)}

# Gazebo oriente sa camera +X vers l'avant, +Y a gauche, +Z en haut ; OpenCV
# +Z vers l'avant, +X a droite, +Y en bas. Colonnes : [z_cv, -x_cv, -y_cv].
CV_VERS_GZ = np.array([[0., -1., 0.], [0., 0., -1.], [1., 0., 0.]])


def pose_camera(extrinseque):
    """(position m, roulis-tangage-lacet rad) de la camera dans le repere base."""
    T = np.array(extrinseque['T_cam_world'], float)
    R_monde_vers_cam, t = T[:3, :3], T[:3, 3]
    centre = -R_monde_vers_cam.T @ t
    rpy = Rotation.from_matrix(R_monde_vers_cam.T @ CV_VERS_GZ).as_euler('xyz')
    return centre, rpy


def intrinseques_simulees(stem, largeur, hauteur):
    """fx, fy, cx, cy du stem, remis a l'echelle de la trame lue."""
    meta = json.loads((CALIB / f'{stem}.meta.json').read_text())
    r = meta['results']
    cw, ch = meta.get('resolution', [largeur, hauteur])
    sx, sy = largeur / cw, hauteur / ch
    return (r['fx'] * sx, r['fy'] * sy, r['cx'] * sx, r['cy'] * sy)


def ecrit_calibration_simulee(nom, extrinseque, fxfycxcy, largeur, hauteur):
    """Jumeau SANS DISTORSION de l'extrinseque reelle, pour les images Gazebo."""
    fx, fy, cx, cy = fxfycxcy
    stem = f'sim_{nom}'
    (CALIB / f'{stem}.meta.json').write_text(json.dumps({
        'name': stem,
        'commentaire': ('Intrinseques SIMULEES : memes fx, fy, cx, cy que '
                        f"{extrinseque['intrinsics_stem']}, distorsion NULLE. "
                        'Gazebo rend une projection pinhole pure ; appliquer les '
                        'coefficients reels a ces images decalerait chaque point.'),
        'resolution': [largeur, hauteur],
        'results': {'fx': fx, 'fy': fy, 'cx': cx, 'cy': cy,
                    'dist_coeffs': [0.0, 0.0, 0.0, 0.0, 0.0]},
    }, indent=2) + '\n')
    jumeau = dict(extrinseque)
    jumeau['intrinsics_stem'] = stem
    jumeau['source'] = (f"Jumeau SIMULE de {nom} : meme T_cam_world que le banc reel, "
                        f"intrinseques sans distorsion. Genere par "
                        f"scripts/generer_monde_yolo26.py.")
    jumeau['resolution'] = [largeur, hauteur]
    (CALIB / f'{nom}_extrinsic_sim.yaml').write_text(yaml.safe_dump(jumeau, sort_keys=False))
    return stem


# Corps de camera : un boitier et un fut d'objectif, orientes selon +X (l'axe de
# visee de Gazebo). Purement visuels et sans collision — ils ne servent qu'a
# MONTRER ou sont les cameras. Sans eux le modele n'a qu'un capteur, invisible,
# et la scene semble ne pas en avoir.
CORPS_CAMERA = """
        <visual name="boitier">
          <pose>-0.02 0 0 0 0 0</pose>
          <geometry><box><size>0.04 0.075 0.03</size></box></geometry>
          <material>
            <ambient>0.15 0.15 0.17 1</ambient>
            <diffuse>0.2 0.2 0.22 1</diffuse>
          </material>
        </visual>
        <visual name="objectif">
          <pose>0.008 0 0 0 1.5707963 0</pose>
          <geometry><cylinder><radius>0.011</radius><length>0.022</length></cylinder></geometry>
          <material>
            <ambient>0.05 0.05 0.06 1</ambient>
            <diffuse>0.08 0.08 0.1 1</diffuse>
            <specular>0.6 0.6 0.6 1</specular>
          </material>
        </visual>
"""


def modele_camera(nom, centre, rpy, fxfycxcy, largeur, hauteur, sujet):
    fx, fy, cx, cy = fxfycxcy
    fov = 2.0 * np.arctan(largeur / (2.0 * fx))
    return ET.fromstring(f"""
    <model name="{nom}">
      <static>true</static>
      <pose>{centre[0]:.6f} {centre[1]:.6f} {centre[2]:.6f} {rpy[0]:.6f} {rpy[1]:.6f} {rpy[2]:.6f}</pose>
      <link name="camera_link">{CORPS_CAMERA}
        <sensor name="{nom}" type="camera">
          <always_on>true</always_on>
          <update_rate>10</update_rate>
          <topic>/{sujet}/image_raw</topic>
          <camera>
            <camera_info_topic>/{sujet}/camera_info</camera_info_topic>
            <horizontal_fov>{fov:.9f}</horizontal_fov>
            <image>
              <width>{largeur}</width>
              <height>{hauteur}</height>
              <format>R8G8B8</format>
            </image>
            <lens>
              <intrinsics>
                <fx>{fx:.6f}</fx><fy>{fy:.6f}</fy>
                <cx>{cx:.6f}</cx><cy>{cy:.6f}</cy><s>0</s>
              </intrinsics>
            </lens>
            <clip><near>0.02</near><far>4.0</far></clip>
          </camera>
        </sensor>
      </link>
    </model>
""")


def inclus(nom, x_mm, y_mm):
    return ET.fromstring(f"""
    <include>
      <uri>package://mycobot_description/models/{nom}</uri>
      <name>{nom}</name>
      <pose>{x_mm / 1000.0:.6f} {y_mm / 1000.0:.6f} 0 0 0 0</pose>
    </include>
""")


def main():
    arbre = ET.parse(SOURCE)
    monde = arbre.getroot().find('world')
    # Le NOM DU MONDE, pas seulement celui du fichier : Gazebo sert ses services
    # sous /world/<nom>/... et `ros_gz_sim create` les cherche sous le nom passe
    # au lancement. Laisser « real_table » ici, c'est un monde qui se charge sans
    # erreur et un robot qui n'apparait jamais, sur une attente muette.
    monde.set('name', SORTIE.stem)

    # SANS CE SYSTEME, AUCUNE CAMERA NE REND. `real_table.sdf` declare un
    # `table_camera` mais pas le systeme qui l'anime : le monde se charge, le
    # capteur existe, et /camera/image_raw reste muet — rien ne le signale.
    if monde.find("plugin[@filename='gz-sim-sensors-system']") is None:
        capteurs = ET.Element('plugin', {
            'filename': 'gz-sim-sensors-system',
            'name': 'gz::sim::systems::Sensors'})
        ET.SubElement(capteurs, 'render_engine').text = MOTEUR_RENDU
        monde.insert(3, capteurs)
        print('ajoute  gz-sim-sensors-system (ogre2) — sans lui les cameras sont muettes')

    for nom in A_RETIRER:
        noeud = monde.find(f"model[@name='{nom}']")
        if noeud is not None:
            monde.remove(noeud)
            print(f'retire  {nom}')

    attenue_les_lumieres(monde)

    reference = yaml.safe_load((CALIB / 'planche_actuelle.yaml').read_text())
    for ident, (x, y, _) in reference['markers'].items():
        noeud = monde.find(f"model[@name='aruco_{ident}']/pose")
        if noeud is None:
            continue
        ancien = [float(v) for v in noeud.text.split()]
        noeud.text = (f'{x / 1000.0:.6f} {y / 1000.0:.6f} 0 '
                      f'{ancien[3]:.6f} {ancien[4]:.6f} {ancien[5]:.6f}')
        ecart = np.hypot(x / 1000.0 - ancien[0], y / 1000.0 - ancien[1]) * 1000.0
        print(f'marqueur {ident} remis a ({x:.1f}, {y:.1f}) mm — {ecart:.1f} mm de deplacement')

    for couleur in tc.TEINTES:
        monde.append(inclus(f'bac_{couleur}', *BACS_XY[couleur]))
        monde.append(inclus(tc.NOM_PIECE[couleur], *PIECES_XY[couleur]))
    print(f'ajoute  {2 * len(tc.TEINTES)} modeles : 4 pieces et 4 bacs')

    ancienne = monde.find("model[@name='table_camera']")
    if ancienne is not None:
        monde.remove(ancienne)
    for nom, fichier, sujet, largeur, hauteur in (
            ('arducam', 'arducam_extrinsic_pick.yaml', 'arducam', 640, 480),
            ('svpro', 'svpro_extrinsic_servo.yaml', 'svpro', 640, 480)):
        extrinseque = yaml.safe_load((CALIB / fichier).read_text())
        centre, rpy = pose_camera(extrinseque)
        k = intrinseques_simulees(extrinseque['intrinsics_stem'], largeur, hauteur)
        stem = ecrit_calibration_simulee(nom, extrinseque, k, largeur, hauteur)
        monde.append(modele_camera(nom, centre, rpy, k, largeur, hauteur, sujet))
        print(f'camera  {nom:8s} a ({centre[0]*1000:.0f}, {centre[1]*1000:.0f}, '
              f'{centre[2]*1000:.0f}) mm — {stem}, /{sujet}/image_raw')

    ET.indent(arbre, '  ')
    arbre.write(SORTIE, encoding='utf-8', xml_declaration=True)
    print(f'\n{SORTIE.relative_to(RACINE)}')


if __name__ == '__main__':
    main()
