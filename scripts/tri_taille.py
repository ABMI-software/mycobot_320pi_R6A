#!/usr/bin/env python3
"""Tri par TAILLE des pieces noires imprimees : bacs et objets vus par l'arducam.

    .venv/bin/python scripts/tri_taille.py                 # vision seule, aucun mouvement
    .venv/bin/python scripts/tri_taille.py --image photo.png
    .venv/bin/python scripts/tri_taille.py --ordre cube_50 pave cube_40

Pieces : dossier de fabrication `pick_and_place_sorting.sdf` (15/09), toutes noires
pour l'instant. Une fois peintes, la couleur remplacera la taille.

Chaine, par piece :
  1. YOLOE-26l sans consigne trouve la boite (les noms sont faux, `speaker`,
     `ipad` : seules la position et la taille servent) ;
  2. dans la boite, Otsu separe le noir du bois ; le plus grand contour donne le
     rectangle d'aire minimale. La boite YOLO, elle, deborde de 10 a 20 mm (flancs
     vus en perspective et ombre, mesure du 15/09) ;
  3. ses 4 coins passent dans le repere robot a CHAQUE hauteur candidate
     (dessus de la piece) ; on garde la piece du plan dont les cotes collent le
     mieux. Un dessus a 50 mm vu comme s'il etait a 40 fausse la taille : la
     hauteur fait partie de l'hypothese.

Limite assumee : cube 50 et cube 40 ne different que de 10 mm, a ~2 mm/px.
"""
import argparse
import contextlib
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE / 'scripts'))

import pick_dashboard as pd  # noqa: E402
import yolo_capture as yc  # noqa: E402

# Dessus vu (longueur, largeur) et hauteur, en mm, d'apres le dossier de fabrication.
# Le pave peut etre pose sur n'importe laquelle de ses faces : trois dessus possibles.
PIECES = {
    'bac': [(105.0, 105.0, 30.0)],
    'cube_50': [(50.0, 50.0, 50.0)],
    'cube_40': [(40.0, 40.0, 40.0)],
    'pave': [(50.0, 30.0, 40.0), (50.0, 40.0, 30.0), (40.0, 30.0, 50.0)],
}
# Les contours noirs debordent (flancs en perspective, ombre) : 112-116 mm pour un bac
# de 105 le 15/09. Le debord est MESURE sur les bacs de la meme image et retire.
COTE_BAC_MM = 105.0
OBJETS = ('cube_50', 'cube_40', 'pave')
ORDRE_DEFAUT = ('cube_50', 'cube_40', 'pave')     # objet destine au bac 1, 2, 3
ECART_MAX_MM = 18.0          # au-dela, aucune piece du plan ne colle : on ne classe pas
# 0,30 manquait le bac du milieu (pieces rapprochees et tournees, 15/09).
SEUIL_YOLO = 0.20
TAILLE_YOLO = 960            # comme yolo_service : les petits objets sortent mieux a 960 px
MARGE_BOITE_PX = 6
# Plus petite piece : cube de 40 mm, ~20 x 20 px a ~2 mm/px. En dessous, bruit du bois.
AIRE_TACHE_MIN_PX = 150
FRACTION_SEUIL_NOIR = 0.35
REFERENCE = RACINE / 'training' / 'calibration' / 'planche_actuelle.yaml'
COULEURS = {'bac': (255, 150, 0), 'cube_50': (0, 0, 255), 'cube_40': (0, 200, 255), 'pave': (0, 200, 0)}


def contour_noir(noir, boite):
    """Rectangle d'aire minimale de la tache sombre la plus proche du CENTRE de la boite.

    La boite YOLO dit ou chercher : sans elle, pieces voisines et ombres se soudaient
    (259 x 199 mm) et le bord sombre de la planche sortait en faux paves. La plus proche
    du centre, pas la plus grande : une boite deborde souvent sur le bac voisin."""
    h, w = noir.shape[:2]
    a, d, b, e = boite
    a, d = max(0, a - MARGE_BOITE_PX), max(0, d - MARGE_BOITE_PX)
    b, e = min(w, b + MARGE_BOITE_PX), min(h, e + MARGE_BOITE_PX)
    contours, _ = cv2.findContours(noir[d:e, a:b].copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    contours = [c for c in contours if cv2.contourArea(c) >= AIRE_TACHE_MIN_PX]
    if not contours:
        return None
    centre = np.array([(b - a) / 2.0, (e - d) / 2.0])

    def distance(c):
        m = cv2.moments(c)
        return np.linalg.norm(np.array([m['m10'] / m['m00'], m['m01'] / m['m00']]) - centre)
    rect = cv2.minAreaRect(min(contours, key=distance))
    return cv2.boxPoints(rect) + np.array([a, d], np.float32)


def en_mm(vision, coins_px, z):
    coins = np.array([vision.vers_base(p, z)[:2] for p in coins_px])
    cotes = [np.linalg.norm(coins[(k + 1) % 4] - coins[k]) for k in range(4)]
    longueur, largeur = sorted([(cotes[0] + cotes[2]) / 2, (cotes[1] + cotes[3]) / 2], reverse=True)
    grand = coins[1] - coins[0] if cotes[0] >= cotes[1] else coins[2] - coins[1]
    angle = float(np.degrees(np.arctan2(grand[1], grand[0])))
    return coins.mean(axis=0), longueur, largeur, angle


def classe(vision, coins_px, debord=0.0):
    """dict de la meilleure hypothese : nom, plan (lg, lr, haut), centre, cotes corrigees, angle, ecart."""
    meilleure = None
    for nom, dessus in PIECES.items():
        for lg, lr, haut in dessus:
            centre, longueur, largeur, angle = en_mm(vision, coins_px, haut)
            longueur, largeur = longueur - debord, largeur - debord
            ecart = float(np.hypot(longueur - lg, largeur - lr))
            if meilleure is None or ecart < meilleure['ecart']:
                meilleure = dict(nom=nom, plan=(lg, lr, haut), centre=centre, longueur=longueur,
                                 largeur=largeur, angle=angle, ecart=ecart)
    return meilleure


def marqueurs_planche():
    return [np.array(v, float) for v in yaml.safe_load(REFERENCE.read_text())['markers'].values()]


def sur_la_planche(centre):
    marqueurs = [m[:2] for m in marqueurs_planche()]
    return (yc.TABLE_X_MM[0] <= centre[0] <= yc.TABLE_X_MM[1] and yc.TABLE_Y_MM[0] <= centre[1] <= yc.TABLE_Y_MM[1]
            and np.hypot(*centre) >= yc.RAYON_BASE_MM
            and all(np.hypot(*(centre - m)) >= yc.RAYON_MARQUEUR_MM for m in marqueurs))


def masque_noir(image, planche, bois):
    """Pixels sombres de la planche, par UN seuil d'Otsu calcule sur le bois seul.

    Seuil local a une boite YOLO : la piece noire occupe presque tout, le seuil devenait
    instable et le contour prenait la boite entiere (bacs a 140 mm, 15/09).
    Seuil sur le masque de planche du dashboard : il est ELARGI, contient la table
    blanche, et Otsu y separait le blanc du bois (une tache de 432 x 431 mm).
    `bois` = interieur du quadrilatere des marqueurs : bois et pieces noires seulement."""
    gris = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    # Pas Otsu : mesure du 15/09 (luminance 61-64), pieces noires a 13-23, bois a 45-50
    # en mediane, Otsu a 38-42 — il coupait dans le bois et prenait les ombres (24-40).
    # Seuil = 5e centile + FRACTION de l'ecart a la mediane : ~25-27 sur les deux scenes.
    bas, milieu = np.percentile(gris[bois > 0], [5, 50])
    seuil = bas + FRACTION_SEUIL_NOIR * (milieu - bas)
    noir = ((gris <= seuil) & (planche > 0)).astype(np.uint8) * 255
    return cv2.morphologyEx(noir, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))


def pieces(image, vision, modele):
    """(pieces classees, pieces non classees, debord mesure sur les bacs en mm)."""
    h, w = image.shape[:2]
    # Pas d'elargissement : essaye le 15/09, il soudait au bac du coin le sombre du bord de planche
    # (133 x 119 mm) et creait une fausse bande de 81 x 15 mm. Les pieces se posent a l'interieur.
    planche = vision.masque_planche((h, w))
    bois = np.zeros((h, w), np.uint8)
    centres = [vision.vers_pixel(m) for m in marqueurs_planche()]
    cv2.fillConvexPoly(bois, cv2.convexHull(np.array(centres, np.float32)).astype(np.int32), 255)
    noir = masque_noir(image, planche, bois)
    r = modele.predict(image, conf=SEUIL_YOLO, imgsz=TAILLE_YOLO, verbose=False)[0]
    contours = []
    for bt in r.boxes:
        boite = tuple(int(v) for v in bt.xyxy[0])
        if (boite[2] - boite[0]) * (boite[3] - boite[1]) > yc.AIRE_OBJET_MAX * w * h:
            continue
        coins = contour_noir(noir, boite)
        if coins is not None:
            contours.append((coins, float(bt.conf)))
    # Premiere passe sans correction : seuls les bacs, nettement plus grands, servent a mesurer le debord.
    bruts = [(classe(vision, coins), coins, conf) for coins, conf in contours]
    debords = [(p['longueur'] + p['largeur']) / 2 - COTE_BAC_MM for p, _, _ in bruts
               if p['nom'] == 'bac' and p['ecart'] <= ECART_MAX_MM and sur_la_planche(p['centre'])]
    debord = float(np.median(debords)) if debords else 0.0
    trouvees, ecartees = [], []
    for coins, conf in contours:
        p = classe(vision, coins, debord)
        if not sur_la_planche(p['centre']):
            continue
        p.update(coins=coins, conf=conf)
        (trouvees if p['ecart'] <= ECART_MAX_MM else ecartees).append(p)
    return sans_doublon(trouvees), ecartees, debord


def sans_doublon(liste):
    gardees = []
    for p in sorted(liste, key=lambda p: -p['conf']):
        if all(np.linalg.norm(p['centre'] - g['centre']) > 30.0 for g in gardees):
            gardees.append(p)
    return gardees


def affecte(trouvees, ordre, vision):
    """Bacs numerotes de gauche a droite dans l'image arducam ; objet de l'ordre i -> bac i."""
    bacs = sorted([p for p in trouvees if p['nom'] == 'bac'], key=lambda p: vision.vers_pixel([*p['centre'], 0.0])[0])
    plan = []
    for p in trouvees:
        if p['nom'] == 'bac':
            continue
        i = ordre.index(p['nom']) if p['nom'] in ordre else None
        plan.append((p, bacs[i] if i is not None and i < len(bacs) else None, i))
    return bacs, plan


def dessine(image, trouvees, ecartees, bacs, vision):
    vue = image.copy()
    for p in trouvees:
        cv2.polylines(vue, [p['coins'].astype(np.int32)], True, COULEURS[p['nom']], 2)
        u, v = p['coins'].mean(axis=0).astype(int)
        numero = next((i for i, b in enumerate(bacs, 1) if b is p), None)
        etiquette = f'bac {numero}' if numero is not None else p['nom']
        cv2.putText(vue, f"{etiquette} {p['longueur']:.0f}x{p['largeur']:.0f}", (u - 40, v - 8),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.45, COULEURS[p['nom']], 2)
    for p in ecartees:
        cv2.polylines(vue, [p['coins'].astype(np.int32)], True, (128, 128, 128), 1)
    return vue


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--image', type=Path, help='photo arducam existante au lieu de la camera')
    ap.add_argument('--ordre', nargs=3, choices=OBJETS, default=list(ORDRE_DEFAUT),
                    help='objet destine au bac 1, 2 et 3 (bacs de gauche a droite dans l image arducam)')
    ap.add_argument('--sortie', type=Path, default=Path('tri_taille_apercu.png'))
    args = ap.parse_args()
    if len(set(args.ordre)) != 3:
        sys.exit('--ordre : trois objets differents')

    if args.image:
        image = cv2.imread(str(args.image))
    else:
        spec = next(s for s in pd.registre.detect_cameras(probe_capture=False) if s.name == 'arducam')
        cap = yc.ouvre_camera(spec.v4l2_index)
        if cap is None:
            sys.exit('arducam muette (deja ouverte par un autre programme ?)')
        pd.regle_exposition(spec.v4l2_index, spec.manual_exposure)
        for _ in range(15):
            ok, image = cap.read()
        cap.release()
    vision = pd.Vision()
    from ultralytics import YOLOE
    with contextlib.chdir(yc.POIDS):
        modele = YOLOE(str(yc.POIDS / yc.MODELE_OBJETS))
    trouvees, ecartees, debord = pieces(image, vision, modele)
    bacs, plan = affecte(trouvees, args.ordre, vision)

    print(f'extrinseque : {vision.source} ; debord des contours mesure sur les bacs : {debord:+.1f} mm')
    for i, b in enumerate(bacs, 1):
        print(f"bac {i} : centre ({b['centre'][0]:.0f}, {b['centre'][1]:.0f}) mm, "
              f"{b['longueur']:.0f} x {b['largeur']:.0f} mm (plan 105 x 105), ecart {b['ecart']:.0f}")
    for p, bac, i in plan:
        lg, lr, haut = p['plan']
        cible = f"-> bac {i + 1}" if bac is not None else '-> AUCUN BAC (pas assez de bacs vus)'
        print(f"{p['nom']:8s}: centre ({p['centre'][0]:.0f}, {p['centre'][1]:.0f}) mm, "
              f"{p['longueur']:.0f} x {p['largeur']:.0f} mm (plan {lg:.0f} x {lr:.0f}), ecart {p['ecart']:.0f}, "
              f"grand cote a {p['angle']:.0f} deg, prise a Z {haut / 2:.0f} mm  {cible}")
    for p in ecartees:
        print(f"non classe : centre ({p['centre'][0]:.0f}, {p['centre'][1]:.0f}), "
              f"{p['longueur']:.0f} x {p['largeur']:.0f} mm, plus proche {p['nom']} a {p['ecart']:.0f} mm")
    manquants = [o for o in OBJETS if o not in [p['nom'] for p, _, _ in plan]]
    if len(bacs) != 3 or manquants:
        print(f'ATTENTION : {len(bacs)} bac(s) vu(s) sur 3 ; objets non vus : {manquants or "aucun"}')
    cv2.imwrite(str(args.sortie), dessine(image, trouvees, ecartees, bacs, vision))
    print(f'apercu -> {args.sortie}')


if __name__ == '__main__':
    main()
