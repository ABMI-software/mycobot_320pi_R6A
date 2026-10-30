#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Schema d'architecture : jumeau Gazebo -> caméras -> YOLO -> cible 3D -> FSM,
avec validation dynamique DREAM.

Chaque bloc porte une VRAIE sortie du banc, pas une vignette d'illustration :
c'est ce qui rend le schema opposable en presentation. La provenance de chaque
image est ecrite sous elle.

Seule la « Localisation 3D » est un schema trace, faute de capture : l'etape
n'a pas de rendu propre, elle a des chiffres. Ils sont affiches tels quels.

    /usr/bin/python3 scripts/generer_figure_architecture.py
"""
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

S = 2.5
W, H = int(1600 * S), int(900 * S)

RACINE = Path(__file__).resolve().parents[1]
SORTIE = RACINE / 'docs'
SCRATCH = Path('/tmp/claude-1000/-home-genji-Osama-ws-src-mycobot-R6A/'
               '3136b2df-3fb7-4d19-848a-78d932afe4a0/scratchpad')

BLANC = (255, 255, 255)
ENCRE = (20, 32, 43)
GRIS = (95, 107, 119)
FILET = (216, 223, 230)
ARDOISE = (42, 58, 74)

# Teintes pastel : un fond clair et un filet soutenu par etage.
TEINTES = {
    'scene': ((219, 233, 246), (137, 174, 205), (26, 70, 108)),
    'cam': ((214, 239, 229), (130, 193, 174), (23, 89, 74)),
    'yolo': ((250, 240, 205), (211, 184, 106), (110, 85, 12)),
    'loc': ((250, 223, 209), (216, 160, 128), (130, 66, 32)),
    'fsm': ((228, 222, 240), (172, 158, 205), (72, 55, 118)),
    'dream': ((222, 231, 238), (154, 175, 191), (38, 66, 88)),
}

LATO = '/usr/share/fonts/truetype/lato/Lato-%s.ttf'


def police(v, t):
    return ImageFont.truetype(LATO % v, int(t * S))


F_TITRE = police('Black', 30)
F_SOUS = police('Regular', 15.5)
F_CADRE = police('Black', 15)
F_BLOC = police('Black', 18)
F_TEXTE = police('Regular', 13.5)
F_PETIT = police('Regular', 11.5)
F_LEGENDE = police('Italic', 11)
F_FLECHE = police('Bold', 12)


def px(v):
    return int(round(v * S))


def boite(d, x, y, w, h, fond=None, filet=None, rayon=12, ep=1.4):
    d.rounded_rectangle([px(x), px(y), px(x + w), px(y + h)], radius=px(rayon),
                        fill=fond, outline=filet, width=max(1, px(ep)))


def texte(d, x, y, s, f, c=ENCRE, espacement=0):
    if espacement:
        cx = px(x)
        for ch in s:
            d.text((cx, px(y)), ch, font=f, fill=c)
            cx += int(d.textlength(ch, font=f)) + px(espacement)
        return
    d.text((px(x), px(y)), s, font=f, fill=c)


def largeur(d, s, f):
    return d.textlength(s, font=f) / S


def centre(d, x, w, y, s, f, c=ENCRE):
    texte(d, x + (w - largeur(d, s, f)) / 2, y, s, f, c)


def coupe(d, w, s, f):
    lignes, ligne = [], ''
    for mot in s.split():
        essai = f'{ligne} {mot}'.strip()
        if largeur(d, essai, f) > w and ligne:
            lignes.append(ligne)
            ligne = mot
        else:
            ligne = essai
    if ligne:
        lignes.append(ligne)
    return lignes


def paragraphe(d, x, y, w, s, f, c=ENCRE, il=18):
    for ligne in coupe(d, w, s, f):
        texte(d, x, y, ligne, f, c)
        y += il
    return y


def fleche(d, x, y, w, c, etiquette=None, retour=False):
    e = max(2, px(2.6))
    if retour:
        d.line([px(x + w), px(y), px(x + 12), px(y)], fill=c, width=e)
        d.polygon([(px(x), px(y)), (px(x + 13), px(y - 7)),
                   (px(x + 13), px(y + 7))], fill=c)
    else:
        d.line([px(x), px(y), px(x + w - 12), px(y)], fill=c, width=e)
        d.polygon([(px(x + w), px(y)), (px(x + w - 13), px(y - 7)),
                   (px(x + w - 13), px(y + 7))], fill=c)
    if etiquette:
        lw = largeur(d, etiquette, F_FLECHE) + 14
        boite(d, x + (w - lw) / 2, y - 26, lw, 20, BLANC, c, 6, 1.2)
        centre(d, x + (w - lw) / 2, lw, y - 22, etiquette, F_FLECHE, c)


def contenir(chemin, w, h, fond=(248, 249, 250)):
    """Image mise a l'echelle SANS deformation, centree sur un fond neutre.

    Deformer une capture pour remplir une case fausse ce qu'elle montre —
    une bbox etiree n'est plus la bbox du detecteur.
    """
    src = Image.open(chemin).convert('RGB')
    cible = Image.new('RGB', (px(w), px(h)), fond)
    k = min(px(w) / src.width, px(h) / src.height)
    red = src.resize((max(1, int(src.width * k)), max(1, int(src.height * k))),
                     Image.LANCZOS)
    cible.paste(red, ((px(w) - red.width) // 2, (px(h) - red.height) // 2))
    return cible


def mosaique(chemins, w, h, ecart=4):
    """Deux vues cote a cote dans une seule case."""
    cible = Image.new('RGB', (px(w), px(h)), BLANC)
    cw = (px(w) - px(ecart)) // 2
    for i, c in enumerate(chemins[:2]):
        src = Image.open(c).convert('RGB')
        k = min(cw / src.width, px(h) / src.height)
        red = src.resize((int(src.width * k), int(src.height * k)), Image.LANCZOS)
        x = i * (cw + px(ecart)) + (cw - red.width) // 2
        cible.paste(red, (x, (px(h) - red.height) // 2))
    return cible


def schema_localisation(d, x, y, w, h, teinte):
    """Rayon camera -> plan de table -> point cible. Trace, faute de capture."""
    fond, filet, encre = teinte
    d.rectangle([px(x), px(y), px(x + w), px(y + h)], fill=(250, 250, 251))
    cx, cy = x + w * 0.22, y + h * 0.2
    # capteur
    d.rounded_rectangle([px(cx - 16), px(cy - 10), px(cx + 16), px(cy + 12)],
                        radius=px(4), outline=encre, width=max(1, px(1.8)))
    d.ellipse([px(cx - 6), px(cy - 5), px(cx + 6), px(cy + 7)], outline=encre,
              width=max(1, px(1.6)))
    # plan de table, en perspective
    px_ = x + w * 0.12, y + h * 0.72
    plan = [(x + w * 0.16, y + h * 0.66), (x + w * 0.9, y + h * 0.56),
            (x + w * 0.96, y + h * 0.9), (x + w * 0.22, y + h * 0.96)]
    d.polygon([(px(a), px(b)) for a, b in plan], fill=fond, outline=filet)
    # rayon
    tx, ty = x + w * 0.66, y + h * 0.73
    n = 16
    for i in range(n):
        if i % 2:
            continue
        t0, t1 = i / n, (i + 0.85) / n
        d.line([px(cx + (tx - cx) * t0), px(cy + 12 + (ty - cy - 12) * t0),
                px(cx + (tx - cx) * t1), px(cy + 12 + (ty - cy - 12) * t1)],
               fill=encre, width=max(1, px(1.6)))
    d.ellipse([px(tx - 6), px(ty - 6), px(tx + 6), px(ty + 6)], fill=encre)
    d.ellipse([px(tx - 12), px(ty - 12), px(tx + 12), px(ty + 12)],
              outline=encre, width=max(1, px(1.4)))
    texte(d, tx + 20, ty - 9, 'cible en base', F_PETIT, encre)
    texte(d, cx - 16, cy - 30, 'K + extrinsèque', F_PETIT, encre)
    texte(d, x + w * 0.2, y + h * 0.84, 'plan Z = 0', F_PETIT, GRIS)
    d.rectangle([px(x), px(y), px(x + w), px(y + h)], outline=FILET,
                width=max(1, px(1)))


img = Image.new('RGB', (W, H), BLANC)
d = ImageDraw.Draw(img)

X0, X1 = 40, 1560

texte(d, X0, 34, 'Architecture — jumeau Gazebo, YOLO et validation dynamique DREAM',
      F_TITRE)
texte(d, X0, 76, 'MyCobot 320 Pi — chaque étage est illustré par une sortie réelle '
                 'du banc, sa provenance écrite sous l’image', F_SOUS, GRIS)
d.line([px(X0), px(110), px(X1), px(110)], fill=FILET, width=max(1, px(1.2)))

# ── Cadre du jumeau ────────────────────────────────────────────────────────
CY, CH = 128, 505
boite(d, X0, CY, X1 - X0, CH, (252, 253, 254), ARDOISE, 14, 1.8)
texte(d, X0 + 22, CY + 16, 'GAZEBO HARMONIC — jumeau numérique', F_CADRE, ARDOISE,
      espacement=1.2)

BW, GAP = 336, 44
BY, BH = CY + 50, 375
BX = [X0 + 22 + i * (BW + GAP) for i in range(4)]

ETAGES = [
    ('scene', 'Scène simulée', RACINE / 'docs/real_table_wood_gazebo.png',
     'worlds/real_table.sdf — vue Gazebo',
     ['MyCobot 320 Pi, table 622 × 449 × 8,5 mm',
      '4 ArUco aux positions mesurées au robot',
      '8 pièces peintes + 4 bacs']),
    ('cam', 'Caméras RGB', None,
     'capteurs du monde banc_realiste_yolo26.sdf',
     ['arducam fx 496,3 · SVPRO fx 447,8',
      'intrinsèques et extrinsèques réelles reproduites',
      '640 × 480, 10 Hz']),
    ('yolo', 'YOLO 26', SCRATCH / 'detect_arducam.png',
     'détection sur la vue arducam simulée',
     ['8 classes : bacs + pièces',
      'classe · bbox · score',
      '7 classes sur 8 retrouvées en simulation']),
    ('loc', 'Localisation 3D', None,
     'schéma — cette étape n’a pas de rendu propre',
     ['rayon caméra + K + extrinsèque',
      'intersection avec le plan de table',
      'écart mesuré : arducam 3,5 mm · SVPRO 9,0 mm']),
]

for x, (cle, titre, chemin, legende, lignes) in zip(BX, ETAGES):
    fond, filet, encre = TEINTES[cle]
    boite(d, x, BY, BW, BH, BLANC, filet, 12, 1.6)
    boite(d, x, BY, BW, 44, fond, None, 12)
    d.rectangle([px(x), px(BY + 32), px(x + BW), px(BY + 44)], fill=fond)
    centre(d, x, BW, BY + 11, titre, F_BLOC, encre)

    ix, iy, iw = x + 14, BY + 56, BW - 28
    ih = int(iw * 0.72)
    if cle == 'cam':
        img.paste(mosaique([SCRATCH / 'sim_arducam.png', SCRATCH / 'sim_svpro.png'],
                           iw, ih), (px(ix), px(iy)))
        d.rectangle([px(ix), px(iy), px(ix + iw), px(iy + ih)], outline=FILET,
                    width=max(1, px(1)))
    elif cle == 'loc':
        schema_localisation(d, ix, iy, iw, ih, TEINTES[cle])
    else:
        img.paste(contenir(chemin, iw, ih), (px(ix), px(iy)))
        d.rectangle([px(ix), px(iy), px(ix + iw), px(iy + ih)], outline=FILET,
                    width=max(1, px(1)))
    texte(d, ix, iy + ih + 6, legende, F_LEGENDE, GRIS)

    ly = iy + ih + 26
    for ligne in lignes:
        d.ellipse([px(ix), px(ly + 6), px(ix + 4), px(ly + 10)], fill=filet)
        ly = paragraphe(d, ix + 11, ly, iw - 11, ligne, F_TEXTE, ENCRE, 18) + 3

for i, etiq in enumerate(('images', 'RGB', 'bbox 2D')):
    fleche(d, BX[i] + BW + 6, BY + 150, GAP - 12, ARDOISE, etiq)

# Retour « mouvement » : la scene bouge, donc les images changent — la boucle
# est ce qui distingue une validation DYNAMIQUE d'un jeu d'images figees.
RY = BY + BH + 26
d.line([px(BX[3] + BW / 2), px(BY + BH + 4), px(BX[3] + BW / 2), px(RY)],
       fill=ARDOISE, width=max(2, px(2.6)))
d.line([px(BX[0] + BW / 2), px(RY), px(BX[3] + BW / 2), px(RY)],
       fill=ARDOISE, width=max(2, px(2.6)))
d.line([px(BX[0] + BW / 2), px(RY), px(BX[0] + BW / 2), px(BY + BH + 16)],
       fill=ARDOISE, width=max(2, px(2.6)))
d.polygon([(px(BX[0] + BW / 2), px(BY + BH + 4)),
           (px(BX[0] + BW / 2 - 7), px(BY + BH + 17)),
           (px(BX[0] + BW / 2 + 7), px(BY + BH + 17))], fill=ARDOISE)
lw = largeur(d, 'mouvement du bras — la scène change à chaque trame', F_FLECHE) + 16
boite(d, (X0 + X1) / 2 - lw / 2, RY - 11, lw, 22, BLANC, ARDOISE, 6, 1.2)
centre(d, (X0 + X1) / 2 - lw / 2, lw, RY - 7,
       'mouvement du bras — la scène change à chaque trame', F_FLECHE, ARDOISE)

# ── Bande basse : ce que la chaîne alimente ────────────────────────────────
FY, FH, FW = 660, 200, 740
fleche(d, X0 + 250, CY + CH + 14, 0.1, ARDOISE)
d.line([px(X0 + 370), px(CY + CH), px(X0 + 370), px(FY - 14)],
       fill=ARDOISE, width=max(2, px(2.6)))
d.polygon([(px(X0 + 370), px(FY)), (px(X0 + 363), px(FY - 14)),
           (px(X0 + 377), px(FY - 14))], fill=ARDOISE)
lw = largeur(d, 'cible 3D', F_FLECHE) + 14
boite(d, X0 + 382, (CY + CH + FY) / 2 - 11, lw, 22, BLANC, ARDOISE, 6, 1.2)
centre(d, X0 + 382, lw, (CY + CH + FY) / 2 - 7, 'cible 3D', F_FLECHE, ARDOISE)

BAS = [
    ('fsm', 'FSM pick & place', RACINE / 'docs/pick_and_place_sim.png',
     'cycle de tri en simulation',
     'Approche, descente asservie, prise vérifiée, dépose dans le bac de la '
     'couleur. Même machine à états que sur le banc réel.'),
    ('dream', 'Validation dynamique DREAM', RACINE / 'docs/fk_vs_dream.png',
     'encodeurs (FK) contre estimation caméra',
     'Le taux de détection et l’écart angulaire sont comptés PAR TRAME, '
     'bras en mouvement — même définition qu’evaluate_dream.'),
]

for i, (cle, titre, chemin, legende, corps) in enumerate(BAS):
    x = X0 + i * (FW + 40)
    fond, filet, encre = TEINTES[cle]
    boite(d, x, FY, FW, FH, BLANC, filet, 12, 1.6)
    boite(d, x, FY, FW, 40, fond, None, 12)
    d.rectangle([px(x), px(FY + 28), px(x + FW), px(FY + 40)], fill=fond)
    texte(d, x + 18, FY + 9, titre, F_BLOC, encre)
    iw, ih = 230, 130
    img.paste(contenir(chemin, iw, ih), (px(x + 16), px(FY + 52)))
    d.rectangle([px(x + 16), px(FY + 52), px(x + 16 + iw), px(FY + 52 + ih)],
                outline=FILET, width=max(1, px(1)))
    texte(d, x + 16, FY + 52 + ih + 5, legende, F_LEGENDE, GRIS)
    paragraphe(d, x + 16 + iw + 18, FY + 56, FW - iw - 52, corps, F_TEXTE, ENCRE, 19)

SORTIE.mkdir(exist_ok=True)
img.save(SORTIE / 'architecture_gazebo_yolo_dream.png', 'PNG')
img.convert('RGB').save(SORTIE / 'architecture_gazebo_yolo_dream.jpg', 'JPEG',
                        quality=95, subsampling=0)
print(f'docs/architecture_gazebo_yolo_dream.png  ·  .jpg   {img.size[0]}x{img.size[1]}')
