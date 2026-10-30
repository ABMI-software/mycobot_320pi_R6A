#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Figure « Avant -> Modifications -> Maintenant » du realisme Gazebo.

Les trois modifications sont posees ENTRE les deux rendus, en blocs pastel
facon schema d'architecture. Elles sont cote a cote dans un meme cadre et SANS
fleche entre elles : ce ne sont pas des etapes successives, elles s'appliquent
ensemble et se desactivent separement. Une fleche les mettrait en sequence et
raconterait quelque chose de faux.

Dessinee en coordonnees logiques 1600x900 puis rendue a l'echelle S.

Sortie : la planche entiere, ET chaque bloc decoupe a part pour un devoilement
progressif en slide.

    /usr/bin/python3 scripts/generer_figure_realisme.py
"""
import math
from pathlib import Path

from PIL import Image, ImageDraw, ImageFont

S = 2.5                       # 1600x900 logique -> 4000x2250
W, H = int(1600 * S), int(900 * S)

RACINE = Path(__file__).resolve().parents[1]
SORTIE = RACINE / 'docs'
AVANT = RACINE / 'training/dream/dream_data/synthetic/011804.rgb.png'
MAINTENANT = RACINE / 'training/dream/dream_data/synthetic_50k/images/left/002811.png'

BLANC = (255, 255, 255)
ENCRE = (20, 32, 43)
GRIS = (93, 105, 117)
FILET = (218, 225, 232)
PANNEAU = (247, 249, 250)
ARDOISE = (42, 58, 74)
ARGILE = (192, 99, 43)
ARGILE_CLAIR = (253, 243, 235)

# Un pastel et un filet soutenu par modification. Famille FROIDE pour les
# trois : l'encart camera garde le seul accent chaud, donc il ressort sans
# qu'on ait a le souligner.
TEINTES = [
    ((219, 233, 246), (128, 168, 202), (26, 70, 108)),     # robot
    ((214, 239, 229), (122, 188, 168), (23, 89, 74)),      # table
    ((228, 222, 240), (166, 152, 202), (72, 55, 118)),     # eclairage
]

LATO = '/usr/share/fonts/truetype/lato/Lato-%s.ttf'


def police(v, t):
    return ImageFont.truetype(LATO % v, int(t * S))


F_TITRE = police('Black', 31)
F_SOUS = police('Regular', 16)
F_ETIQ = police('Black', 13)
F_CADRE = police('Black', 12.5)
F_BLOC = police('Black', 17)
F_NUM = police('Black', 13)
F_TEXTE = police('Regular', 13.5)
F_TEXTE_G = police('Semibold', 14)
F_PETIT = police('Regular', 12.5)
F_BADGE = police('Bold', 11)
F_LEGENDE = police('Italic', 12.5)
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


def largeur(d, s, f, espacement=0):
    return d.textlength(s, font=f) / S + espacement * max(0, len(s) - 1)


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


def puce(d, x, y, w, s, accent, f=F_TEXTE, c=ENCRE, il=18):
    d.ellipse([px(x), px(y + 5.5), px(x + 4), px(y + 9.5)], fill=accent)
    return paragraphe(d, x + 11, y, w - 11, s, f, c, il)


def fleche(d, x, y, w, c=ARDOISE, etiquette=None):
    e = max(2, px(3))
    d.line([px(x), px(y), px(x + w - 12), px(y)], fill=c, width=e)
    d.polygon([(px(x + w), px(y)), (px(x + w - 14), px(y - 7.5)),
               (px(x + w - 14), px(y + 7.5))], fill=c)
    if etiquette:
        lw = largeur(d, etiquette, F_FLECHE) + 14
        boite(d, x + (w - lw) / 2, y - 28, lw, 20, BLANC, c, 6, 1.2)
        centre(d, x + (w - lw) / 2, lw, y - 24, etiquette, F_FLECHE, c)


# ── Icones, trace au filet ────────────────────────────────────────────────
def icone_materiau(d, x, y, t, c):
    """Sphere eclairee : le sujet, c'est le rendu de surface."""
    e = max(1, px(1.7))
    d.ellipse([px(x), px(y), px(x + t), px(y + t)], outline=c, width=e)
    d.arc([px(x + t * 0.22), px(y + t * 0.18), px(x + t * 0.62), px(y + t * 0.58)],
          200, 340, fill=c, width=e)


def icone_table(d, x, y, t, c):
    e = max(1, px(1.7))
    d.line([px(x), px(y + t * 0.34), px(x + t), px(y + t * 0.34)], fill=c, width=e)
    d.line([px(x), px(y + t * 0.34), px(x + t * 0.12), px(y + t)], fill=c, width=e)
    d.line([px(x + t), px(y + t * 0.34), px(x + t * 0.88), px(y + t)], fill=c, width=e)
    for k in (0.55, 0.76):
        d.line([px(x + t * 0.16), px(y + t * k), px(x + t * 0.84), px(y + t * k)],
               fill=c, width=max(1, px(1)))


def icone_lumiere(d, x, y, t, c):
    e = max(1, px(1.7))
    r = t * 0.24
    cx, cy = x + t / 2, y + t / 2
    d.ellipse([px(cx - r), px(cy - r), px(cx + r), px(cy + r)], outline=c, width=e)
    for i in range(8):
        a = math.radians(i * 45)
        d.line([px(cx + math.cos(a) * r * 1.55), px(cy + math.sin(a) * r * 1.55),
                px(cx + math.cos(a) * r * 2.15), px(cy + math.sin(a) * r * 2.15)],
               fill=c, width=e)


def icone_camera(d, x, y, t, c):
    e = max(1, px(1.8))
    d.rounded_rectangle([px(x), px(y + t * 0.22), px(x + t), px(y + t * 0.92)],
                        radius=px(t * 0.12), outline=c, width=e)
    d.line([px(x + t * 0.26), px(y + t * 0.22), px(x + t * 0.36), px(y + t * 0.06)],
           fill=c, width=e)
    d.line([px(x + t * 0.64), px(y + t * 0.22), px(x + t * 0.54), px(y + t * 0.06)],
           fill=c, width=e)
    r = t * 0.2
    cx, cy = x + t / 2, y + t * 0.57
    d.ellipse([px(cx - r), px(cy - r), px(cx + r), px(cy + r)], outline=c, width=e)


def vignette(chemin, w, h):
    return Image.open(chemin).convert('RGB').resize((px(w), px(h)), Image.LANCZOS)


# ── Toile ──────────────────────────────────────────────────────────────────
img = Image.new('RGB', (W, H), BLANC)
d = ImageDraw.Draw(img)

X0, X1 = 40, 1560

texte(d, X0, 36, 'Rendre le robot réaliste dans Gazebo', F_TITRE)
texte(d, X0, 78, 'MyCobot 320 Pi — le gain de réalisme vient de l’apparence et de '
                 'l’environnement, jamais de la géométrie', F_SOUS, GRIS)
d.line([px(X0), px(112), px(X1), px(112)], fill=FILET, width=max(1, px(1.2)))

# ── Le flux : image -> les trois modifications -> image ────────────────────
RY, RH = 140, 450
IW = 380                       # carte image
FW = 44                        # fleche
KX, KW = X0 + IW + FW, 672     # cadre des modifications
D2X = KX + KW + FW
BW, BG = 200, 22
BX = [KX + 14 + i * (BW + BG) for i in range(3)]


def carte_image(x, etiquette, chemin, legende, annotation, accent):
    boite(d, x, RY, IW, RH, PANNEAU, FILET, 12, 1.4)
    boite(d, x, RY, IW, 5, accent, None, 0)
    p = 14
    iw = IW - 2 * p
    ih = int(iw * 480 / 640)
    ah = 24 + len(coupe(d, iw - 24, annotation, F_TEXTE_G)) * 19
    contenu = 26 + ih + 8 + 17 + 12 + ah
    y = RY + 20 + max(0, (RH - 30 - contenu) / 2)

    texte(d, x + p, y, etiquette, F_ETIQ, accent, espacement=1.6)
    y += 26
    img.paste(vignette(chemin, iw, ih), (px(x + p), px(y)))
    d.rectangle([px(x + p), px(y), px(x + p + iw), px(y + ih)], outline=FILET,
                width=max(1, px(1)))
    y += ih + 8
    texte(d, x + p, y, legende, F_LEGENDE, GRIS)
    y += 29
    boite(d, x + p, y, iw, ah, BLANC, FILET, 8, 1.2)
    paragraphe(d, x + p + 12, y + 12, iw - 24, annotation, F_TEXTE_G, ENCRE, 19)


carte_image(X0, 'AVANT', AVANT, 'Rendu d’origine — capteur Gazebo',
            'Géométrie, meshes DAE et joints identiques', GRIS)
carte_image(D2X, 'MAINTENANT', MAINTENANT, 'Rendu actuel — capteur Gazebo',
            'Robot blanc satiné · table bois · scène éclairée', TEINTES[0][2])

fleche(d, X0 + IW + 6, RY + RH / 2, FW - 12)
fleche(d, KX + KW + 6, RY + RH / 2, FW - 12)

# Cadre commun : les trois agissent ENSEMBLE. Pas de fleche entre eux.
boite(d, KX, RY, KW, RH, (252, 253, 254), ARDOISE, 12, 1.8)
texte(d, KX + 14, RY + 13, 'MODIFICATIONS APPLIQUÉES', F_CADRE, ARDOISE,
      espacement=1.2)
sw = largeur(d, 'MODIFICATIONS APPLIQUÉES', F_CADRE, 1.2)
texte(d, KX + 24 + sw, RY + 14, '— trois changements indépendants, appliqués '
                                'ensemble', F_PETIT, GRIS)

BY = RY + 40
ZONE = RH - 40 - 14

BLOCS = [
    ('01', icone_materiau, 'Robot', [
        'Coques blanc cassé — RVB 0,88 · 0,87 · 0,84',
        'Rendu satiné : rugosité 0,48 ; base mate 0,72',
        'Colliers argent / sombre aux articulations',
    ], 'GÉOMÉTRIE INCHANGÉE'),
    ('02', icone_table, 'Table', [
        'Texture bois PBR — albedo reconstruit depuis les photos du banc',
        'Dimensions conservées : 622 × 449 × 8,5 mm mesurés',
    ], 'MÊMES COLLISIONS'),
    ('03', icone_lumiere, 'Éclairage', [
        'Lumière ambiante relevée dans le bloc <scene>',
        'Sun directionnel + fill light',
        'Relief et ombres lisibles sur toute la scène',
    ], 'CAPTEUR INCHANGÉ'),
]

BH = max(58 + sum(len(coupe(d, BW - 39, s_, F_TEXTE)) * 18 + 5
                  for _, _, _, lg, _ in [b] for s_ in lg) + 30 + 14
         for b in BLOCS)
BY += max(0, (ZONE - BH) / 2)

for x, (num, icone, titre, lignes, badge), (fond, filet, encre) in zip(
        BX, BLOCS, TEINTES):
    boite(d, x, BY, BW, BH, BLANC, filet, 11, 1.5)
    boite(d, x, BY, BW, 42, fond, None, 11)
    d.rectangle([px(x), px(BY + 30), px(x + BW), px(BY + 42)], fill=fond)
    d.ellipse([px(x + 12), px(BY + 10), px(x + 34), px(BY + 32)], fill=encre)
    centre(d, x + 12, 22, BY + 13, num, F_NUM, BLANC)
    texte(d, x + 42, BY + 11, titre, F_BLOC, encre)
    icone(d, x + BW - 34, BY + 11, 20, encre)

    ly = BY + 58
    for ligne in lignes:
        ly = puce(d, x + 14, ly, BW - 28, ligne, filet) + 5

    bw = min(BW - 28, largeur(d, badge, F_BADGE) + 22)
    boite(d, x + 14, BY + BH - 36, bw, 24, fond, None, 12)
    texte(d, x + 25, BY + BH - 32, badge, F_BADGE, encre)

# ── Bande basse : les deux réserves ────────────────────────────────────────
FY, FH = 620, 240
boite(d, X0, FY, 1000, FH, PANNEAU, FILET, 12, 1.4)
boite(d, X0, FY, 1000, 5, ENCRE, None, 0)
texte(d, X0 + 22, FY + 22, 'CE QUI NE CHANGE PAS', F_ETIQ, ENCRE, espacement=1.6)
paragraphe(d, X0 + 22, FY + 48, 950,
           'Géométrie, physique et étiquettes identiques entre les deux rendus — '
           'seule l’apparence diffère. La vérité terrain des keypoints est donc '
           'valable pour les deux, sans retoucher un seul label.',
           F_PETIT, GRIS, 19)

cx, cy = X0 + 22, FY + 106
for p in ('fichiers DAE', 'dimensions', 'frames', 'joints', 'collisions',
          'inerties', 'contrôleurs', 'vérité terrain des keypoints'):
    w = largeur(d, p, F_TEXTE) + 28
    if cx + w > X0 + 978:
        cx, cy = X0 + 22, cy + 40
    boite(d, cx, cy, w, 34, BLANC, FILET, 17, 1.2)
    d.ellipse([px(cx + 13), px(cy + 14.5), px(cx + 19), px(cy + 20.5)], fill=ENCRE)
    texte(d, cx + 26, cy + 8, p, F_TEXTE)
    cx += w + 9

KCX, KCW = X0 + 1020, 500
boite(d, KCX, FY, KCW, FH, ARGILE_CLAIR, ARGILE, 12, 1.8)
icone_camera(d, KCX + 22, FY + 20, 24, ARGILE)
texte(d, KCX + 58, FY + 24, 'CAMÉRA', F_ETIQ, ARGILE, espacement=1.6)
texte(d, KCX + 46, FY + 58, '« Rendu plus réaliste »', F_TEXTE_G, ENCRE)
texte(d, KCX + 22, FY + 74, '≠', police('Black', 22), ARGILE)
texte(d, KCX + 46, FY + 82, '« Caméra plus réaliste »', F_TEXTE_G, ENCRE)
d.line([px(KCX + 22), px(FY + 112), px(KCX + KCW - 22), px(FY + 112)],
       fill=(234, 207, 187), width=max(1, px(1.2)))
yy = paragraphe(d, KCX + 22, FY + 124, KCW - 44,
                'Aucun changement démontré ici.', F_TEXTE_G, ENCRE, 21)
paragraphe(d, KCX + 22, yy + 4, KCW - 44,
           'Intrinsèques, résolution, champ de vision et extrinsèques restent à '
           'vérifier séparément.', F_TEXTE, GRIS, 20)

# ── Sortie : la planche, puis chaque bloc a part ───────────────────────────
SORTIE.mkdir(exist_ok=True)


def enregistre(image, nom):
    image.save(SORTIE / f'{nom}.png', 'PNG')
    image.convert('RGB').save(SORTIE / f'{nom}.jpg', 'JPEG', quality=95,
                              subsampling=0)
    print(f'docs/{nom}.png  ·  docs/{nom}.jpg   {image.size[0]}x{image.size[1]}')


def decoupe(x, y, w, h, marge=16):
    return img.crop((px(x - marge), px(y - marge),
                     px(x + w + marge), px(y + h + marge)))


enregistre(img, 'realisme_gazebo')
enregistre(decoupe(X0, RY, D2X + IW - X0, RH), 'realisme_flux')
for x, (num, _, titre, _, _) in zip(BX, BLOCS):
    nom = titre.lower().replace('é', 'e').replace('ç', 'c')
    enregistre(decoupe(x, BY, BW, BH), f'realisme_{num}_{nom}')
enregistre(decoupe(X0, FY, 1000, FH), 'realisme_ce_qui_ne_change_pas')
enregistre(decoupe(KCX, FY, KCW, FH), 'realisme_camera')
