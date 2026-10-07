#!/usr/bin/env python3
"""Tri par COULEUR des pieces peintes : chaque objet va au bac de sa couleur.

    .venv/bin/python scripts/tri_couleur.py                 # vision seule, aucun mouvement
    .venv/bin/python scripts/tri_couleur.py --image photo.png
    .venv/bin/python scripts/tri_couleur.py --teintes        # n'affiche que les H/S/V mesures
    .venv/bin/python scripts/tri_couleur.py --etiquette training/yolo/pieces

Suite de `tri_taille.py`, qui classait ces memes pieces quand elles etaient
toutes noires (« une fois peintes, la couleur remplacera la taille »). La chaine
ne change que par son milieu, le reste est importe tel quel :

  1. YOLOE-26l sans consigne trouve les boites (inchange) — sur l'image du
     18/09, il sort les 8 pieces, bacs a 0,36-0,69 et objets a 0,68-0,94 ;
  2. le coeur de chaque boite est NOMME par sa teinte, puis la piece est
     decoupee dans le masque de SA couleur ;
  3. la taille la VERIFIE contre le dossier de fabrication, et l'objet part au
     bac de sa couleur.

Ce qui separe les couleurs, mesure sur la peinture reelle (arducam, exposition
75, image du 18/09, mediane circulaire au coeur de chaque piece) :

    piece      H     S        bois de la planche : H 16 (q05 11, q95 26)
    rouge   179,5   194                           S 183 (q95 209)
    jaune    21     255                           V 185
    vert     39     253
    bleu     98     178

Deux consequences, contre-intuitives et mesurees :

* **la saturation ne separe pas la peinture du bois.** Ce bois-la est a S 183,
  plus sature que le bac bleu (178). Une premiere version seuillait la
  saturation « au-dessus du bois » : elle ne trouvait que 2 taches sur 8 et les
  nommait toutes jaunes. C'est la TEINTE qui nomme.
* **sauf pour le jaune**, dont la teinte (21) tombe en plein dans celle du bois
  (11-26). Lui seul est tranche par la saturation : 255 contre 209 au 95e
  centile du bois. D'ou une porte de saturation posee sur le jaune uniquement.

Les teintes du monde Gazebo (`pick_and_place_sorting.sdf`), dont le dossier de
fabrication est tire, NE conviennent PAS : elles donnaient bleu 116 et vert 62,
soit 17 et 22 d'ecart avec la peinture reelle. Elles sont remplacees ici par les
valeurs mesurees. Si l'eclairage ou l'exposition changent, relancer `--teintes`
et les recaler : le compte rendu imprime le H/S/V de chaque piece vue.
"""
import argparse
import contextlib
import sys
import time
from pathlib import Path

import cv2
import numpy as np

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE / 'scripts'))

import pick_dashboard as pd  # noqa: E402
import tri_taille as tt  # noqa: E402
import yolo_capture as yc  # noqa: E402

# H OpenCV (0-179) MESURES sur la peinture, et non tires du SDF (voir l'en-tete).
TEINTES = {'rouge': 0, 'jaune': 21, 'vert': 39, 'bleu': 98}
# Demi-largeur de la bande de teinte d'une couleur. Les references mesurees sont
# ecartees de 21 au plus serre (rouge-jaune) : 8 laisse un couloir de bois entre
# les deux sans mordre sur la piece.
BANDE_TEINTE = 8
# Au-dela, la tache n'est d'aucune des quatre couleurs : on ne classe pas.
ECART_TEINTE_MAX = 9
# Le jaune seul partage la teinte du bois : sa saturation le tranche. Mesure du
# 18/09 sur les DEUX cameras, 8 trames, 170 000 px de bois dans la bande jaune :
#
#                bois q99,9   pave jaune   bac jaune
#     arducam        218        254-255      243-244
#     SVPRO          195        226-228      233-234
#
# La vue rasante de la SVPRO delave le bois (mediane 146 contre 180) mais delave
# le jaune avec : le pave y tombe a 226. Pose a 230, la porte le rejetait a trois
# unites pres — c'etait la cause du `pave_jaune` manquant sur toutes les trames
# SVPRO, et son contour ne se remplissait qu'a 5 %. A 220 il passe avec six
# unites de marge, deux au-dessus du bois arducam, et le remplissage monte a
# 53 %. Le bois qui fuit ne fait aucun amas : l'ouverture 3x3 de `masque_couleur`
# le balaie entierement, jusqu'a 215.
SATURATION_JAUNE = 220
# Plancher commun : sous cette saturation, c'est une ombre ou un gris, pas une
# piece peinte. La moins saturee des huit (bac bleu) est a 178.
SATURATION_MIN = 120
# Deux boites qui se recouvrent autant designent la meme piece.
RECOUVREMENT_DOUBLON = 0.5
# Seuil de confiance d'un modele ENTRAINE sur les 8 classes. Il n'a rien a voir
# avec celui de YOLOE (0,20) : voir `pieces()` pour les mesures qui le fixent.
SEUIL_MODELE_ENTRAINE = 0.10
# Dessus vu (longueur, largeur) et hauteur en mm, par couleur, d'apres le dossier
# de fabrication. Le pave peut etre pose sur n'importe laquelle de ses faces ;
# le cylindre, vu de dessus, donne un carre de son diametre.
OBJET_PAR_COULEUR = {
    'rouge': ('cube_40', [(40.0, 40.0, 40.0)]),
    'bleu': ('cube_50', [(50.0, 50.0, 50.0)]),
    'vert': ('cylindre', [(44.0, 44.0, 50.0)]),
    'jaune': ('pave', [(50.0, 30.0, 40.0), (50.0, 40.0, 30.0), (40.0, 30.0, 50.0)]),
}
BAC = (105.0, 105.0, 30.0)
BGR = {'rouge': (0, 0, 255), 'jaune': (0, 200, 255), 'vert': (0, 200, 0), 'bleu': (255, 100, 0)}
# Nom de chaque objet, tel qu'ecrit dans le dossier de fabrication (plans_cotes,
# feuilles 2 a 5) : on ne reinvente pas une nomenclature, on reprend la sienne.
NOM_PIECE = {'rouge': 'cube_rouge', 'jaune': 'pave_jaune',
             'vert': 'cylindre_vert', 'bleu': 'cube_bleu'}
# Les 8 classes du yolo26 des pieces peintes. L'ordre des indices est fige : les
# jeux d'images deja etiquetes le portent, un changement d'ordre les invaliderait
# tous en silence.
CLASSES_PIECES = tuple([f'bac_{c}' for c in TEINTES] + [NOM_PIECE[c] for c in TEINTES])
# Recette du run `main_v1_yolo26s` (args.yaml), qui a donne 0,984 de mAP50 sur
# 148 images : c'est la seule recette YOLO26 mesuree sur cette machine.
ENTRAINEMENT = ('model=yolo26s.pt', 'epochs=100', 'imgsz=640', 'batch=16',
                'patience=30', 'seed=1509')


def ecart_teinte(h, reference):
    """Distance circulaire en H OpenCV : le rouge est a cheval sur 0 et 179."""
    d = abs(float(h) - float(reference))
    return min(d, 180.0 - d)


def nomme(h, s):
    """(couleur, ecart de teinte) ; (None, ecart) si la tache n'est d'aucune couleur.

    La teinte nomme, la saturation ne sert qu'au jaune — seule couleur que le
    bois imite en teinte. Un bois a H 16 tombe au plus pres du jaune (5) et ne
    doit sa mise a l'ecart qu'a sa saturation.
    """
    ecart, couleur = min((ecart_teinte(h, ref), nom) for nom, ref in TEINTES.items())
    if ecart > ECART_TEINTE_MAX or s < SATURATION_MIN:
        return None, ecart
    if couleur == 'jaune' and s < SATURATION_JAUNE:
        return None, ecart
    return couleur, ecart


def masque_couleur(hsv, couleur, planche):
    """Pixels de la planche appartenant a CETTE couleur."""
    h = hsv[:, :, 0].astype(int)
    s = hsv[:, :, 1]
    d = np.abs(h - TEINTES[couleur])
    dedans = (np.minimum(d, 180 - d) <= BANDE_TEINTE) & (s >= SATURATION_MIN) & (planche > 0)
    if couleur == 'jaune':
        dedans &= s >= SATURATION_JAUNE
    # OPEN puis CLOSE, noyau 5 pour la fermeture : recette mesuree le 25/08 sur les
    # rouleaux — c'est l'ouverture qui casse, la fermeture qui recolle (aire 206 px
    # contre 148 avec l'ouverture seule). Les pieces sont imprimees en 3D et striees
    # par leurs lignes de remplissage : sur le bac jaune la teinte ne concorde que sur
    # 74 % des pixels, son masque sort crible et l'ouverture seule le pulverisait
    # (55 % de remplissage, contour a 92 x 81 mm pour un plan de 105 x 105).
    brut = dedans.astype(np.uint8) * 255
    ouvert = cv2.morphologyEx(brut, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
    return cv2.morphologyEx(ouvert, cv2.MORPH_CLOSE, np.ones((5, 5), np.uint8))


def coeur(hsv, boite):
    """H (mediane circulaire), S et V au centre de la boite, loin de son pourtour.

    La boite du detecteur deborde de 10 a 20 mm (flancs en perspective et ombre,
    mesure du 15/09) : prise entiere, elle melange la piece et le bois. Le quart
    central ne contient que la piece.
    """
    a, d, b, e = boite
    cu, cv_ = (a + b) // 2, (d + e) // 2
    lu, lv = max(2, (b - a) // 4), max(2, (e - d) // 4)
    z = hsv[max(0, cv_ - lv):cv_ + lv, max(0, cu - lu):cu + lu].reshape(-1, 3).astype(float)
    if not len(z):
        return 0.0, 0.0, 0.0
    t = z[:, 0] * (np.pi / 90.0)
    h = (np.degrees(np.arctan2(np.sin(t).mean(), np.cos(t).mean())) / 2.0) % 180.0
    return h, float(np.median(z[:, 1])), float(np.median(z[:, 2]))


def recouvre(p, q):
    """Intersection sur union de deux boites — YOLO26 est SANS NMS et sort des
    quasi-doublons (le pave jaune trois fois, boites a un pixel pres, 18/09)."""
    ax, ay, bx, by = p
    cx, cy, dx, dy = q
    inter = max(0, min(bx, dx) - max(ax, cx)) * max(0, min(by, dy) - max(ay, cy))
    union = (bx - ax) * (by - ay) + (dx - cx) * (dy - cy) - inter
    return inter / union if union else 0.0


def contour_ou_boite(masque, boite):
    """Contour de la couleur dans la boite ; a defaut, la boite elle-meme.

    Rendre None revenait a perdre la piece. Une boite un peu large donne un
    centre utilisable ; rien ne donne rien.
    """
    coins = tt.contour_noir(masque, boite)
    if coins is not None:
        return coins
    a, d, b, e = boite
    return np.array([[a, d], [b, d], [b, e], [a, e]], np.float32)


def classe(vision, coins, couleur, debord=0.0, genre=None):
    """Meilleure hypothese de geometrie pour une tache DONT LA COULEUR est connue.

    genre : 'bac' ou 'objet' quand le detecteur l'a deja dit — on ne cherche alors
    que parmi les hypotheses de ce genre-la. None (YOLOE, qui ne nomme pas) = les
    deux, et c'est la taille qui tranche.
    """
    nom_objet, dessus = OBJET_PAR_COULEUR[couleur]
    hypotheses = ([('bac', BAC)] if genre in (None, 'bac') else []) + \
                 ([(nom_objet, d) for d in dessus] if genre in (None, 'objet') else [])
    meilleure = None
    for nom, (lg, lr, haut) in hypotheses:
        centre, longueur, largeur, angle = tt.en_mm(vision, coins, haut)
        longueur, largeur = longueur - debord, largeur - debord
        ecart = float(np.hypot(longueur - lg, largeur - lr))
        if meilleure is None or ecart < meilleure['ecart']:
            meilleure = dict(nom=nom, plan=(lg, lr, haut), centre=centre, longueur=longueur,
                             largeur=largeur, angle=angle, ecart=ecart)
    return meilleure


def range_par_aire(taches, vision):
    """Genre decide par le RANG D'AIRE dans la couleur, sans reconstruction metrique.

    Mesure du 18/09 sur la SVPRO (vue rasante) : le controle dimensionnel y est
    faux. Le contour ramasse les FLANCS des pieces, hautes de 30 a 50 mm, et
    l'extrinseque ne vaut que dans le plan Z = 0 — un cube de 50 en sort a
    79 x 48 mm, le bac vert a 110 x 76 pour 105 x 105. Le debord mesure sur les
    bacs est un scalaire : il ne corrige pas une deformation qui a une direction.

    L'aire en PIXELS, elle, ne demande aucune extrinseque : sur les 4 trames, le
    bac et son objet se separent d'un facteur 2,2 a 4,9, sans une seule inversion
    la ou le modele entraine peut l'attester.

    On ne devine pas : une couleur qui ne donne pas exactement deux taches sur la
    planche est abandonnee entiere.
    """
    sur_planche = [t for t in taches
                   if tt.sur_la_planche(tt.en_mm(vision, t['coins'], 0.0)[0])]
    gardees = []
    for couleur in TEINTES:
        lot = [t for t in sur_planche if t['couleur'] == couleur]
        if len(lot) != 2:
            continue
        lot.sort(key=lambda t: -cv2.contourArea(t['coins'].astype(np.float32)))
        gardees += [dict(t, genre=genre) for t, genre in zip(lot, ('bac', 'objet'))]
    return gardees


def pieces(image, vision, modele, par_aire=False):
    """(pieces classees, taches ecartees, debord mesure sur les bacs).

    Trois regimes, selon ce que le detecteur sait dire de lui-meme et selon
    l'obliquite de la vue :

    * **modele ENTRAINE sur les 8 classes du dossier** — il nomme la piece. Sa
      classe FAIT FOI : la couleur ne sert plus qu'a decouper le contour, et la
      taille devient un controle affiche, jamais un motif de rejet. Autrement on
      jette des pieces correctement reconnues — mesure du 18/09 : le bac rouge
      deborde du quadrilatere des marqueurs (12 % de sa boite, contour coupe a
      114 x 87 mm) et le bac jaune, imprime strie, rend un masque court.
    * **modele SANS consigne** (YOLOE, 4 585 noms approximatifs) — il ne sait pas
      nommer : la teinte nomme, et la taille tranche entre le bac et l'objet de
      cette couleur.
    * **modele sans consigne et vue RASANTE** (`par_aire`) — la teinte nomme
      toujours, mais c'est le rang d'aire en pixels qui tranche bac/objet, la
      taille en mm n'etant plus mesurable. Voir `range_par_aire`.
    """
    h, w = image.shape[:2]
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    planche = vision.masque_planche((h, w))
    masques = {c: masque_couleur(hsv, c, planche) for c in TEINTES}
    noms = getattr(modele, 'names', None) or {}
    entraine = bool(noms) and set(noms.values()) <= set(CLASSES_PIECES)
    # Le seuil de YOLOE ne vaut pas pour un modele entraine : mesure du 18/09 sur
    # 6 trames fraiches, il voit les 8 pieces 6 fois sur 6, mais a des confiances
    # basses et STABLES a +-0,01 — bac_vert 0,87, cube_bleu 0,74, bac_bleu 0,69,
    # bac_jaune 0,57, pave_jaune 0,36, cylindre_vert 0,25, bac_rouge 0,23,
    # cube_rouge 0,13. A 0,20 le cube rouge disparaissait : pas par hesitation du
    # modele, par un seuil herite d'un autre detecteur. Ces confiances basses sont
    # le prix d'un entrainement sur 12 images quasi identiques ; le remede est un
    # jeu d'images varie, pas un seuil plus bas.
    seuil = SEUIL_MODELE_ENTRAINE if entraine else tt.SEUIL_YOLO
    r = modele.predict(image, conf=seuil, imgsz=tt.TAILLE_YOLO, verbose=False)[0]

    taches = []
    for bt in sorted(r.boxes, key=lambda b: -float(b.conf)):
        boite = tuple(int(v) for v in bt.xyxy[0])
        if (boite[2] - boite[0]) * (boite[3] - boite[1]) > yc.AIRE_OBJET_MAX * w * h:
            continue
        if any(recouvre(boite, t['boite']) > RECOUVREMENT_DOUBLON for t in taches):
            continue                      # meme piece, deja retenue avec une meilleure confiance
        mesure = coeur(hsv, boite)
        if entraine:
            nom_classe = noms[int(bt.cls)]
            couleur, genre, ecart_t = nom_classe.rsplit('_', 1)[-1], \
                ('bac' if nom_classe.startswith('bac_') else 'objet'), \
                ecart_teinte(mesure[0], TEINTES[nom_classe.rsplit('_', 1)[-1]])
        else:
            couleur, ecart_t = nomme(mesure[0], mesure[1])
            genre = None
            if couleur is None:
                continue                  # bureau, cables, clavier
        taches.append(dict(boite=boite, conf=float(bt.conf), couleur=couleur, genre=genre,
                           ecart_teinte=ecart_t, hsv=mesure,
                           coins=contour_ou_boite(masques[couleur], boite)))

    if par_aire and not entraine:
        taches = range_par_aire(taches, vision)

    # Premiere passe sans correction : seuls les bacs, nettement plus grands,
    # servent a mesurer le debord des contours.
    debords = [(p['longueur'] + p['largeur']) / 2 - BAC[0]
               for p in (classe(vision, t['coins'], t['couleur'], 0.0, t['genre']) for t in taches)
               if p['nom'] == 'bac' and p['ecart'] <= tt.ECART_MAX_MM and tt.sur_la_planche(p['centre'])]
    debord = float(np.median(debords)) if debords else 0.0

    trouvees, ecartees = [], []
    for t in taches:
        p = classe(vision, t['coins'], t['couleur'], debord, t['genre'])
        p.update(t)
        p['verifie'] = p['ecart'] <= tt.ECART_MAX_MM
        if not tt.sur_la_planche(p['centre']):
            continue
        (trouvees if (entraine or par_aire or p['verifie']) else ecartees).append(p)
    return tt.sans_doublon(trouvees), ecartees, debord


def affecte(trouvees):
    """Chaque objet va au bac de SA couleur. Plus d'ordre a donner : la peinture le dit."""
    bacs = {p['couleur']: p for p in trouvees if p['nom'] == 'bac'}
    plan = [(p, bacs.get(p['couleur'])) for p in trouvees if p['nom'] != 'bac']
    return bacs, plan


def dessine(image, trouvees, ecartees):
    vue = image.copy()
    for p in trouvees:
        couleur = BGR[p['couleur']]
        cv2.polylines(vue, [p['coins'].astype(np.int32)], True, couleur, 2)
        u, v = p['coins'].mean(axis=0).astype(int)
        cv2.putText(vue, f"{p['couleur']} {p['nom']} {p['longueur']:.0f}x{p['largeur']:.0f}",
                    (u - 50, v - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.45, couleur, 2)
    for p in ecartees:
        cv2.polylines(vue, [p['coins'].astype(np.int32)], True, (128, 128, 128), 1)
        u, v = p['coins'].mean(axis=0).astype(int)
        cv2.putText(vue, f"{p['longueur']:.0f}x{p['largeur']:.0f}", (u - 30, v - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, (128, 128, 128), 1)
    return vue


def etiquette(image, trouvees, sortie, nom, boite_detecteur=False):
    """Ecrit image + etiquettes YOLO + apercu, pour entrainer yolo26 sur les 8 classes.

    **Quelle boite ecrire depend de l'obliquite de la vue**, mesure du 18/09 en
    tracant les deux sur la meme image :

    * vue plongeante (arducam) — celle du CONTOUR. La boite YOLOE y deborde de
      10 a 20 mm, et c'est elle que le futur modele apprendrait ; le contour, lui,
      tient entre 85 et 130 % de l'aire du detecteur, centre sur 100 %.
    * vue rasante (SVPRO, `boite_detecteur`) — celle du DETECTEUR. Le contour y
      devient erratique : 60 % de l'aire sur le bac jaune, dont la camera voit
      l'interieur et dont le masque n'attrape qu'une partie, mais 151 % sur le
      cylindre vert ou il deborde. Ce n'est pas un biais rattrapable, c'est du
      bruit dans les deux sens ; la boite du detecteur, elle, reste reguliere.

    L'apercu est la piece maitresse : une pre-annotation se RELIT avant de servir
    (celle des `mors`, le 15/09, etait fausse).
    """
    h, w = image.shape[:2]
    lignes, ecrites = [], []
    for p in trouvees:
        if boite_detecteur:
            a, d, b, e = p['boite']
        else:
            a, d = p['coins'].min(axis=0)
            b, e = p['coins'].max(axis=0)
        classe = f"bac_{p['couleur']}" if p['nom'] == 'bac' else NOM_PIECE[p['couleur']]
        numero = CLASSES_PIECES.index(classe)
        lignes.append(f'{numero} {(a + b) / 2 / w:.6f} {(d + e) / 2 / h:.6f} '
                      f'{(b - a) / w:.6f} {(e - d) / h:.6f}')
        ecrites.append((classe, (a, d, b, e)))
    for sous in ('images', 'labels', 'apercu'):
        (sortie / sous).mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(sortie / 'images' / f'{nom}.png'), image)
    (sortie / 'labels' / f'{nom}.txt').write_text(''.join(l + '\n' for l in lignes))
    # L'apercu trace la boite REELLEMENT ECRITE, et non le contour : sans cela il
    # montrerait autre chose que l'etiquette des que `boite_detecteur` est vrai, et
    # la relecture — seule garantie de cette chaine — ne garantirait plus rien.
    vue = image.copy()
    for classe, (a, d, b, e) in ecrites:
        couleur = BGR[classe.rsplit('_', 1)[-1]]
        cv2.rectangle(vue, (int(a), int(d)), (int(b), int(e)), couleur, 2)
        cv2.putText(vue, classe, (int(a), max(11, int(d) - 4)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, couleur, 1)
    cv2.imwrite(str(sortie / 'apercu' / f'{nom}.png'), vue)
    noms = ''.join(f'  {i}: {n}\n' for i, n in enumerate(CLASSES_PIECES))
    (sortie / 'data.yaml').write_text(f'path: {sortie}\ntrain: images\nval: images\nnames:\n{noms}')
    return len(lignes)


def image_arducam():
    """Une image de l'arducam, en V4L2 explicite et SANS rien imposer.

    Mesure du 18/09, OpenCV 5.0 : `VideoCapture(index)` sans backend tombe sur
    obsensor et n'ouvre rien ; et toute ecriture de propriete pendant le flux
    (FOURCC MJPG, largeur, hauteur) rend cette camera muette — 0 image sur 12
    essais avec, 1 sur 1 sans. Le mode par defaut est deja le 640 x 480 attendu
    par l'extrinseque : il n'y a rien a imposer. L'exposition, elle, se pose
    AVANT l'ouverture, par v4l2-ctl (recette de `pick_dashboard`).
    """
    spec = next(s for s in pd.registre.detect_cameras(probe_capture=False) if s.name == 'arducam')
    pd.regle_exposition(spec.v4l2_index, spec.manual_exposure)
    time.sleep(0.4)
    cap = cv2.VideoCapture(spec.v4l2_index, cv2.CAP_V4L2)
    image = None
    for _ in range(12):
        ok, trame = cap.read()
        if ok:
            image = trame
            break
        time.sleep(0.4)
    cap.release()
    if image is None:
        sys.exit('arducam muette (deja ouverte par un autre programme ?)')
    if image.shape[:2] != (pd.registre.CAPTURE_H, pd.registre.CAPTURE_W):
        sys.exit(f'arducam en {image.shape[1]}x{image.shape[0]} :'
                 ' l extrinseque est calibree en 640x480')
    return image


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--image', type=Path, help='photo existante au lieu de la camera')
    ap.add_argument('--teintes', action='store_true', help='n affiche que les H/S/V mesures')
    ap.add_argument('--sortie', type=Path, default=Path('tri_couleur_apercu.png'))
    ap.add_argument('--modele', type=Path,
                    help='poids yolo26 entraine sur les pieces ; par defaut YOLOE-26l sans consigne')
    ap.add_argument('--etiquette', type=Path,
                    help='dossier ou ecrire image + etiquettes YOLO + apercu, pour entrainer yolo26')
    ap.add_argument('--extrinseque', help="nom de l'extrinseque, sans .yaml (defaut : celle de l arducam)")
    ap.add_argument('--par-aire', action='store_true',
                    help='vue rasante : le rang d aire tranche bac/objet, la taille en mm ne vaut plus')
    args = ap.parse_args()

    image = cv2.imread(str(args.image)) if args.image else image_arducam()
    if image is None:
        sys.exit(f'image illisible : {args.image}')
    vision = pd.Vision(args.extrinseque) if args.extrinseque else pd.Vision()
    if args.modele:
        from ultralytics import YOLO
        modele = YOLO(str(args.modele))
    else:
        from ultralytics import YOLOE
        with contextlib.chdir(yc.POIDS):
            modele = YOLOE(str(yc.POIDS / yc.MODELE_OBJETS))
    trouvees, ecartees, debord = pieces(image, vision, modele, par_aire=args.par_aire)
    bacs, plan = affecte(trouvees)

    print(f'extrinseque : {vision.source} ; debord mesure sur les bacs : {debord:+.1f} mm')
    print(f"{'piece':22s} {'H':>6s} {'S':>6s} {'V':>6s}   ecart de teinte")
    for p in sorted(trouvees + ecartees, key=lambda p: -p['conf']):
        h, s, v = p['hsv']
        print(f"{p['couleur'] + ' ' + p['nom']:22s} {h:6.1f} {s:6.1f} {v:6.1f}   {p['ecart_teinte']:.1f}")
    if args.teintes:
        cv2.imwrite(str(args.sortie), dessine(image, trouvees, ecartees))
        print(f'apercu -> {args.sortie}')
        return

    # Une piece nommee par le modele est gardee meme si son contour ne colle pas au
    # plan : le dire, sinon rien ne distingue plus un contour sain d'un contour rogne.
    def marque(p):
        return '' if p['verifie'] else '  [CONTOUR HORS PLAN]'

    for couleur, b in bacs.items():
        print(f"bac {couleur:6s}: centre ({b['centre'][0]:.0f}, {b['centre'][1]:.0f}) mm, "
              f"{b['longueur']:.0f} x {b['largeur']:.0f} mm (plan 105 x 105), "
              f"ecart {b['ecart']:.0f}{marque(b)}")
    for p, bac in plan:
        lg, lr, haut = p['plan']
        cible = (f"-> bac {p['couleur']} ({bac['centre'][0]:.0f}, {bac['centre'][1]:.0f})"
                 if bac is not None else f"-> AUCUN BAC {p['couleur']} VU")
        print(f"{p['couleur']:6s} {p['nom']:8s}: centre ({p['centre'][0]:.0f}, {p['centre'][1]:.0f}) mm, "
              f"{p['longueur']:.0f} x {p['largeur']:.0f} mm (plan {lg:.0f} x {lr:.0f}), ecart {p['ecart']:.0f}, "
              f"grand cote a {p['angle']:.0f} deg, prise a Z {haut / 2:.0f} mm  {cible}{marque(p)}")
    for p in ecartees:
        print(f"non classe : {p['couleur']} {p['longueur']:.0f} x {p['largeur']:.0f} mm, "
              f"plus proche {p['nom']} a {p['ecart']:.0f} mm")
    manque_bac = [c for c in TEINTES if c not in bacs]
    manque_objet = [c for c in TEINTES if c not in [p['couleur'] for p, _ in plan]]
    if manque_bac or manque_objet:
        print(f'ATTENTION : bacs non vus {manque_bac or "aucun"} ; objets non vus {manque_objet or "aucun"}')
    if args.etiquette:
        nom = args.image.stem if args.image else f'arducam_{int(time.time())}'
        n = etiquette(image, trouvees, args.etiquette, nom, boite_detecteur=args.par_aire)
        print(f'{n} etiquette(s) -> {args.etiquette} (A RELIRE sur l apercu avant d entrainer)')
    cv2.imwrite(str(args.sortie), dessine(image, trouvees, ecartees))
    print(f'apercu -> {args.sortie}')


if __name__ == '__main__':
    main()
