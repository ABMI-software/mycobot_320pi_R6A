# -*- coding: utf-8 -*-
"""Genere le support de presentation de la campagne de precision.

Source unique : training/calibration/PROTOCOLE_ESSAIS_PRECISION.md.
Aucun chiffre n'est introduit ici qui ne s'y trouve deja.
"""
from pptx import Presentation
from pptx.util import Inches, Pt, Emu
from pptx.dml.color import RGBColor
from pptx.enum.text import PP_ALIGN, MSO_ANCHOR

ENCRE   = RGBColor(0x1F, 0x2A, 0x44)
ACCENT  = RGBColor(0x1F, 0x6F, 0xB2)
ACCENT_F= RGBColor(0x14, 0x4E, 0x7E)
GRIS    = RGBColor(0x6E, 0x77, 0x87)
GRIS_CL = RGBColor(0xE6, 0xEA, 0xF0)
BANDE   = RGBColor(0xF3, 0xF6, 0xFA)
ROUGE   = RGBColor(0xA6, 0x2B, 0x1F)
VERT    = RGBColor(0x2E, 0x6E, 0x4F)
BLANC   = RGBColor(0xFF, 0xFF, 0xFF)

TITRE_F = 'Calibri Light'
CORPS_F = 'Calibri'

prs = Presentation()
prs.slide_width  = Inches(13.333)
prs.slide_height = Inches(7.5)
VIDE = prs.slide_layouts[6]
L, R = Inches(0.72), Inches(12.61)
LARGEUR = R - L


def zone(slide, x, y, w, h):
    tb = slide.shapes.add_textbox(x, y, w, h)
    tf = tb.text_frame
    tf.word_wrap = True
    tf.margin_left = tf.margin_right = tf.margin_top = tf.margin_bottom = 0
    return tf


def par(tf, texte, taille=14, gras=False, couleur=ENCRE, police=CORPS_F,
        espace_avant=0, espace_apres=4, interligne=1.0, italique=False,
        premier=False, align=PP_ALIGN.LEFT):
    p = tf.paragraphs[0] if premier else tf.add_paragraph()
    p.alignment = align
    p.space_before = Pt(espace_avant)
    p.space_after = Pt(espace_apres)
    p.line_spacing = interligne
    r = p.add_run()
    r.text = texte
    r.font.size = Pt(taille)
    r.font.bold = gras
    r.font.italic = italique
    r.font.color.rgb = couleur
    r.font.name = police
    return p


def riche(tf, morceaux, taille=14, couleur=ENCRE, espace_avant=0,
          espace_apres=5, interligne=1.05, premier=False, puce=None):
    """morceaux = [(texte, gras), ...] pour melanger gras et normal."""
    p = tf.paragraphs[0] if premier else tf.add_paragraph()
    p.space_before = Pt(espace_avant)
    p.space_after = Pt(espace_apres)
    p.line_spacing = interligne
    if puce:
        r = p.add_run()
        r.text = puce
        r.font.size = Pt(taille)
        r.font.color.rgb = ACCENT
        r.font.name = CORPS_F
        r.font.bold = True
    for texte, gras in morceaux:
        r = p.add_run()
        r.text = texte
        r.font.size = Pt(taille)
        r.font.bold = gras
        r.font.color.rgb = couleur
        r.font.name = CORPS_F
    return p


def rect(slide, x, y, w, h, remplissage=None, ligne=None, epaisseur=1.0):
    from pptx.enum.shapes import MSO_SHAPE
    f = slide.shapes.add_shape(MSO_SHAPE.RECTANGLE, x, y, w, h)
    f.shadow.inherit = False
    if remplissage is None:
        f.fill.background()
    else:
        f.fill.solid()
        f.fill.fore_color.rgb = remplissage
    if ligne is None:
        f.line.fill.background()
    else:
        f.line.color.rgb = ligne
        f.line.width = Pt(epaisseur)
    f.text_frame.word_wrap = True
    return f


NUM = [0]


def entete(slide, titre, surtitre=None, couleur_barre=ACCENT):
    rect(slide, L, Inches(0.46), Inches(0.09), Inches(0.62), couleur_barre)
    tf = zone(slide, Inches(0.98), Inches(0.40), LARGEUR - Inches(0.26), Inches(0.9))
    if surtitre:
        par(tf, surtitre.upper(), 11.5, True, ACCENT, CORPS_F,
            espace_apres=2, premier=True)
        par(tf, titre, 27, False, ENCRE, TITRE_F, espace_apres=0)
    else:
        par(tf, titre, 27, False, ENCRE, TITRE_F, espace_apres=0, premier=True)


def pied(slide, texte=''):
    NUM[0] += 1
    tf = zone(slide, L, Inches(6.93), LARGEUR, Inches(0.3))
    p = tf.paragraphs[0]
    p.space_after = Pt(0)
    r = p.add_run()
    r.text = texte
    r.font.size = Pt(9.5)
    r.font.color.rgb = GRIS
    r.font.name = CORPS_F
    tf2 = zone(slide, Inches(12.2), Inches(6.93), Inches(1.0), Inches(0.3))
    p2 = tf2.paragraphs[0]
    p2.alignment = PP_ALIGN.RIGHT
    r2 = p2.add_run()
    r2.text = str(NUM[0])
    r2.font.size = Pt(9.5)
    r2.font.color.rgb = GRIS
    r2.font.name = CORPS_F


def nouvelle(titre=None, surtitre=None, bas='', couleur_barre=ACCENT):
    s = prs.slides.add_slide(VIDE)
    if titre:
        entete(s, titre, surtitre, couleur_barre)
    pied(s, bas)
    return s


def tableau(slide, x, y, w, entetes, lignes, largeurs, hauteur_entete=0.34,
            hauteur_ligne=0.32, taille=11.5, gras_col0=True,
            couleurs_ligne=None):
    nl, nc = len(lignes) + 1, len(entetes)
    h = Inches(hauteur_entete + hauteur_ligne * len(lignes))
    gt = slide.shapes.add_table(nl, nc, x, y, w, h).table
    gt.rows[0].height = Inches(hauteur_entete)
    for i in range(1, nl):
        gt.rows[i].height = Inches(hauteur_ligne)
    total = sum(largeurs)
    for j, lg in enumerate(largeurs):
        gt.columns[j].width = Emu(int(w * lg / total))
    for j, txt in enumerate(entetes):
        c = gt.cell(0, j)
        c.text = ''
        c.fill.solid()
        c.fill.fore_color.rgb = ACCENT_F
        c.vertical_anchor = MSO_ANCHOR.MIDDLE
        c.margin_left = c.margin_right = Inches(0.07)
        c.margin_top = c.margin_bottom = Inches(0.02)
        p = c.text_frame.paragraphs[0]
        r = p.add_run()
        r.text = txt
        r.font.size = Pt(taille)
        r.font.bold = True
        r.font.color.rgb = BLANC
        r.font.name = CORPS_F
    for i, ligne in enumerate(lignes):
        for j, cellule in enumerate(ligne):
            c = gt.cell(i + 1, j)
            c.text = ''
            c.fill.solid()
            c.fill.fore_color.rgb = BLANC if i % 2 else BANDE
            c.vertical_anchor = MSO_ANCHOR.MIDDLE
            c.margin_left = c.margin_right = Inches(0.07)
            c.margin_top = c.margin_bottom = Inches(0.02)
            p = c.text_frame.paragraphs[0]
            morceaux = cellule if isinstance(cellule, list) else [(cellule, False)]
            for texte, gras in morceaux:
                r = p.add_run()
                r.text = texte
                r.font.size = Pt(taille)
                r.font.bold = gras or (gras_col0 and j == 0)
                coul = ENCRE
                if couleurs_ligne and couleurs_ligne.get(i) and j > 0:
                    coul = couleurs_ligne[i]
                r.font.color.rgb = coul
                r.font.name = CORPS_F
    return gt


# ---------------------------------------------------------------- 1. titre
s = prs.slides.add_slide(VIDE)
rect(s, Inches(0), Inches(0), prs.slide_width, prs.slide_height, ENCRE)
rect(s, Inches(0), Inches(3.02), prs.slide_width, Inches(0.035), ACCENT)
tf = zone(s, Inches(1.1), Inches(1.55), Inches(11), Inches(1.4))
par(tf, 'CAMPAGNE DE PRECISION  ·  8 - 10 SEPTEMBRE 2026', 13, True,
    RGBColor(0x8F, 0xB8, 0xDC), CORPS_F, espace_apres=10, premier=True)
par(tf, 'Protocole des essais de précision', 40, False, BLANC, TITRE_F,
    espace_apres=2)
par(tf, 'myCobot 320 Pi — bras seul, vision, et système complet', 22,
    False, RGBColor(0xC8, 0xD6, 0xE4), TITRE_F)
tf = zone(s, Inches(1.1), Inches(3.45), Inches(10.6), Inches(2.4))
par(tf, 'Treize essais, chacun rattaché à sa méthode.', 16.5, True,
    RGBColor(0xE8, 0xEE, 0xF5), CORPS_F, espace_apres=12, premier=True)
for t in ['ISO 9283:1998 — répétabilité et exactitude de pose du manipulateur',
          'VDI/VDE 2634 Blatt 1 — erreur de mesure de longueur d\'un système optique',
          'ISO 9787:1999 · ISO 8373 — repères et vocabulaire',
          'Protocoles construits pour l\'occasion — leave-one-out, validation croisée, boucle fermée']:
    riche(tf, [(t, False)], 14, RGBColor(0xC8, 0xD6, 0xE4), espace_apres=7,
          puce='—   ')
tf = zone(s, Inches(1.1), Inches(6.35), Inches(10.6), Inches(0.6))
par(tf, 'Source : training/calibration/PROTOCOLE_ESSAIS_PRECISION.md   ·   '
        'en cas de divergence avec le classeur ou le rapport, le .md fait foi',
    11, False, RGBColor(0x7E, 0x92, 0xA8), CORPS_F, premier=True)

# ------------------------------------------------- 2. objet de la campagne
s = nouvelle('Ce que la campagne mesure', 'Objet',
             'PROTOCOLE_ESSAIS_PRECISION.md § 1 et § 4')
tf = zone(s, L, Inches(1.5), Inches(6.05), Inches(5.2))
par(tf, 'Trois grandeurs différentes, souvent confondues', 16, True, ACCENT_F,
    CORPS_F, espace_apres=12, premier=True)
for titre, texte in [
    ('Répétabilité du bras',
     'la dispersion quand le robot revient au même point par le même chemin. '
     'Ne dit rien de la justesse.'),
    ('Exactitude de pose',
     "l'écart entre le point commandé et le point atteint. Exige une mesure "
     'externe que le banc n\'a pas.'),
    ('Précision du système vision + robot',
     'quand la caméra dit « la cible est là », à combien le robot y arrive. '
     "C'est le chiffre qui compte pour un asservissement visuel."),
]:
    riche(tf, [(titre + ' — ', True), (texte, False)], 14, espace_apres=11,
          puce='')
tf = zone(s, Inches(7.1), Inches(1.5), Inches(5.5), Inches(5.2))
par(tf, 'Les quatre supports de la campagne', 16, True, ACCENT_F, CORPS_F,
    espace_apres=10, premier=True)
tableau(s, Inches(7.1), Inches(1.95), Inches(5.5),
        ['support', 'ce qu\'il porte'],
        [['Résultats', 'les chiffres, leur lecture, les réserves'],
         ['Méthodologie', 'les normes, et ce que la méthode ne peut pas mesurer'],
         ['Données', 'classeur, 17 feuilles de relevés bruts'],
         ['Rapport', '17 sections, 35 tableaux — version diffusable'],
         ['Protocole', 'ce document : la table de correspondance']],
        [1.05, 2.6], hauteur_ligne=0.46, taille=12)
tf = zone(s, Inches(7.1), Inches(4.6), Inches(5.5), Inches(1.6))
riche(tf, [('Chaque essai porte un bloc « Liens partagés » à cinq entrées — '
            'résultat, méthode, feuille du classeur, section du rapport, '
            'script. Un chiffre cité ailleurs se retrouve toujours par là.',
            False)], 13, GRIS, premier=True)

# ------------------------------------------------------ 3. la méthodologie
s = nouvelle('La méthodologie appelée, norme par norme', 'Méthodologie',
             'Aucun essai n\'est présenté comme conforme à un protocole qu\'il ne suit pas')
tableau(s, L, Inches(1.55), LARGEUR,
        ['norme', 'ce qu\'elle définit', 'ce qu\'on en a pris', 'essais'],
        [['ISO 9283:1998',
          'répétabilité de pose RP = l̄ + 3·S_l, exactitude de pose AP',
          [('la définition de RP, pas le protocole', True)], 'A · F-UNI · G · B'],
         ['VDI/VDE 2634-1',
          'erreur de mesure de longueur Δl = l_m − l_k, tolérance E = A + K·L ≤ B',
          [('la grandeur mesurée, sans artefact étalonné', True)], 'E · L · C'],
         ['ISO 9787:1999',
          'systèmes de coordonnées du robot',
          [('suivie — tout est exprimé en base_link', True)], 'tous'],
         ['ISO 8373',
          'vocabulaire de la robotique',
          [('suivie — termes employés au sens de la norme', True)], 'tous'],
         ['aucune norme connue',
          'précision d\'un système vision + robot en boucle fermée',
          [('protocole construit et déclaré intégralement', True)],
          'H · K · essai 7']],
        [1.35, 4.25, 3.55, 1.85], hauteur_ligne=0.62, taille=12.5)
b = rect(s, L, Inches(5.45), LARGEUR, Inches(1.28), BANDE, GRIS_CL)
tf = b.text_frame
tf.margin_left = tf.margin_right = Inches(0.22)
tf.margin_top = Inches(0.14)
riche(tf, [('Ce que la norme autorise. ', True),
           ('ISO 9283, introduction : « It is intended that the user of this '
            'International Standard selects which performance characteristics '
            'are to be tested… The tests described in this International '
            'Standard may be applied in whole or in part. »', False)],
      12.5, premier=True, espace_apres=6)
riche(tf, [('Un protocole partiel est légitime ', False),
           ('à condition d\'être déclaré', True),
           ('. C\'est l\'objet de la diapositive « Conformité ».', False)], 12.5)

# ------------------------------------------- 4. règles de méthode communes
s = nouvelle('Les six règles de méthode, communes à tous les essais',
             'Méthodologie', 'Chacune vient d\'une erreur constatée pendant la campagne')
cartes = [
    ('RP = l̄ + 3·S_l',
     "Jamais l'écart maximum : il sous-estime la répétabilité systématiquement."),
    ('Lire après stabilisation',
     "FK(q_lu) une fois le bras arrêté — jamais l'accusé de commande, qui ne "
     'prouve que la réception.'),
    ('Un seul sens d\'approche',
     'Une série qui mélange les directions ne mesure pas une répétabilité, '
     'mais un jeu mécanique.'),
    ('Six trames médianées',
     'Une trame MJPEG isolée peut être fausse de 17,5 mm. Ne jamais commander '
     'le robot sur une seule.'),
    ('Valider hors des données d\'ajustement',
     'Un ajustement qui reproduit ses propres points ne prouve rien : il est '
     'circulaire. D\'où le leave-one-out.'),
    ('Une grandeur insensible au biais',
     'Deux mesures qui partagent une cause d\'erreur ne se confirment pas. '
     'Il faut un contrôle de nature différente.'),
]
x0, y0, cw, ch, gx, gy = L, Inches(1.62), Inches(3.95), Inches(1.5), Inches(0.19), Inches(0.24)
for i, (t, d) in enumerate(cartes):
    col, row = i % 3, i // 3
    x = x0 + col * (cw + gx)
    y = y0 + row * (ch + gy)
    c = rect(s, x, y, cw, ch, BANDE, GRIS_CL)
    rect(s, x, y, Inches(0.055), ch, ACCENT)
    tf = c.text_frame
    tf.margin_left = Inches(0.22)
    tf.margin_right = Inches(0.14)
    tf.margin_top = Inches(0.13)
    par(tf, t, 14.5, True, ACCENT_F, CORPS_F, espace_apres=5, premier=True)
    par(tf, d, 12, False, ENCRE, CORPS_F, interligne=1.03)
tf = zone(s, L, Inches(5.6), LARGEUR, Inches(1.1))
riche(tf, [('La règle 6 est celle qui a coûté le plus cher. ', True),
           ("Six mesures concordantes donnaient les marqueurs à 48,3 mm ; "
            "elles partageaient toutes le même biais de détection. La "
            "conclusion a été retirée le 10/09 — voir l'essai L.", False)],
      13.5, premier=True)

# ------------------------------------------------------ 5. tableau maître
s = nouvelle('Les treize essais', 'Vue d\'ensemble',
             'Résultats détaillés dans les diapositives suivantes')
tableau(s, L, Inches(1.42), LARGEUR,
        ['code', 'essai', 'méthode', 'résultat'],
        [['A', 'Répétabilité, 10 A/R verticaux', 'ISO 9283:1998', [('0,403 mm', True)]],
         ['F-UNI', 'Répétabilité, 6 A/R (contrôle indépendant)', 'ISO 9283:1998', [('0,557 mm', True)]],
         ['G', 'Répétabilité, 40 A/R, 10 par côté', 'ISO 9283:1998', [('0,000 à 0,838 mm', True)]],
         ['F-MULTI', 'Sens d\'approche mélangés', 'hors norme', [('5,847 mm', True)]],
         ['D', 'Approche latérale', 'hors norme', [('8,091 mm', True)]],
         ['B', 'Erreur de cible absolue, boucle ouverte', 'ISO 9283 § exactitude', [('14,84 mm → ≈2 mm compensé', True)]],
         ['C', 'Bruit propre de la vision, 60 trames', 'VDI/VDE 2634-1', [('0,725 mm  ·  17,5 mm sur 1 trame', True)]],
         ['E', 'Échelle métrique de la vision, 8 objets', 'VDI/VDE 2634-1 § Δl', [('−0,044 %', True)]],
         ['H', 'Extrinsèque, leave-one-out', 'construit', [('1,86 mm', True)]],
         ['7 / I', 'Précision globale vision + robot', 'construit', [('10,79 → 0,63 mm', True)]],
         ['J', 'Bruit sous-pixel, trames corrompues', 'diagnostic', [('17,5 mm sur 1 trame', True)]],
         ['K', 'Validation croisée deux caméras', 'construit', [('1,92 mm', True)]],
         ['L', 'Taille réelle des marqueurs', 'VDI/VDE 2634-1 § artefact', [('50 mm  (50,65 extrapolé)', True)]]],
        [0.85, 4.6, 2.5, 3.3], hauteur_ligne=0.375, taille=12.3)


# ------------------------------------------------------- fiche d'un essai
def fiche(code, titre, norme, objet, mode, resultats, lecture=None,
          reserve=None, liens='', couleur=ACCENT):
    """resultats = [(valeur, legende), ...] affichés en encart à droite."""
    s = nouvelle(titre, code, liens, couleur_barre=couleur)
    # bandeau norme
    b = rect(s, L, Inches(1.34), LARGEUR, Inches(0.60), BANDE, None)
    rect(s, L, Inches(1.34), Inches(0.055), Inches(0.60), couleur)
    tf = b.text_frame
    tf.margin_left = Inches(0.22)
    tf.margin_top = Inches(0.05)
    riche(tf, [('Méthode  ·  ', True), (norme, False)], 12.5, ENCRE,
          premier=True, espace_apres=0)

    gauche = Inches(8.05)
    tf = zone(s, L, Inches(2.02), gauche - Inches(0.45), Inches(4.7))
    par(tf, 'Objet', 13, True, ACCENT_F, CORPS_F, espace_apres=4, premier=True)
    par(tf, objet, 13.5, False, ENCRE, CORPS_F, espace_apres=11, interligne=1.05)
    par(tf, 'Mode opératoire', 13, True, ACCENT_F, CORPS_F, espace_apres=5)
    for i, etape in enumerate(mode):
        morceaux = etape if isinstance(etape, list) else [(etape, False)]
        riche(tf, morceaux, 13, espace_apres=4, puce='%d.  ' % (i + 1))
    if lecture:
        par(tf, 'Lecture', 13, True, ACCENT_F, CORPS_F, espace_avant=9,
            espace_apres=4)
        riche(tf, lecture if isinstance(lecture, list) else [(lecture, False)],
              13, espace_apres=0)
    if reserve:
        par(tf, 'Réserve', 13, True, ROUGE, CORPS_F, espace_avant=9,
            espace_apres=4)
        riche(tf, reserve if isinstance(reserve, list) else [(reserve, False)],
              13, espace_apres=0)

    # encart résultats
    y = Inches(2.02)
    for valeur, legende in resultats:
        h = Inches(1.24)
        c = rect(s, gauche, y, R - gauche, h, couleur, None)
        tf = c.text_frame
        tf.margin_left = Inches(0.26)
        tf.margin_top = Inches(0.14)
        par(tf, valeur, 30, True, BLANC, TITRE_F, espace_apres=2, premier=True)
        par(tf, legende, 12, False, RGBColor(0xDC, 0xE7, 0xF1), CORPS_F,
            interligne=1.0)
        y = y + h + Inches(0.16)
    return s


fiche('Essais A et F-UNI',
      'Répétabilité de pose, approche unidirectionnelle',
      'ISO 9283:1998 — définition de la répétabilité de pose, RP = l̄ + 3·S_l. '
      'La définition est respectée, le protocole ne l\'est pas.',
      'La dispersion du bras quand il revient au même point, toujours par le '
      'même chemin.',
      ['Une pose de travail unique, bras à vide.',
       'Dix allers-retours : montée de 40 mm, redescente sur le même point.',
       [('Sens d\'approche constant, toujours par le haut.', True)],
       'Lecture de FK(q_lu) après stabilisation, pas après accusé de commande.',
       'Barycentre des dix points, puis l̄ + 3·S_l.',
       [('F-UNI : ', True), ('série indépendante de six A/R, même protocole.',
                             False)]],
      [('0,403 mm', 'essai A — 10 allers-retours'),
       ('0,557 mm', 'essai F-UNI — contrôle indépendant, 6 allers-retours')],
      lecture=[('Même ordre de grandeur sur deux séries indépendantes : '
                'la conclusion tient.', False)],
      reserve=[('C\'est une borne inférieure : ', True),
               ('mesure reconstruite aux codeurs, aveugle à la flexion des '
                'liens et au jeu des réducteurs. Écarts au protocole '
                'normatif : 1 pose au lieu de 5, 10 cycles au lieu de 30, à '
                'vide, à vitesse réduite.', False)],
      liens='Résultats § 1  ·  Méthode § 3 essai 1  ·  feuilles « A repetabilite haut » et '
            '« F-UNI repetabilite haut »  ·  rapport § 3 et § 4')

fiche('Essai G',
      'Répétabilité côté par côté, quarante allers-retours',
      'ISO 9283:1998 — quatre séries unidirectionnelles traitées séparément. '
      'La dépendance au sens d\'approche n\'est pas imposée par la norme.',
      'La répétabilité dépend-elle de la direction d\'approche ?',
      ['Quatre séries de dix allers-retours, une par côté : +X, −X, +Y, −Y.',
       [('Chaque série est traitée séparément, jamais mélangée.', True)],
       'Calcul de RP indépendamment sur chacune.'],
      [('0,000 à 0,838 mm', 'selon le côté d\'approche'),
       ('6 séries sur 6', 'sous la spécification constructeur de 1 mm')],
      lecture=[('Chaque direction prise isolément est bonne. ', True),
               ('C\'est le seul essai qui explore la dépendance au sens '
                'd\'approche — et il établit que le problème n\'est pas dans '
                'la direction elle-même, mais dans leur mélange.', False)],
      liens='Résultats § 2  ·  Méthode § 3 essai 1  ·  feuille « G 4 directions x10 »  ·  rapport § 5')

fiche('Essais F-MULTI et D',
      'Sens d\'approche mélangés — la conséquence la plus importante',
      'Hors du champ d\'ISO 9283 par construction : la norme définit la '
      'répétabilité « in the same direction ». Ce qui est mesuré ici est un '
      'biais de jeu mécanique, pas une répétabilité.',
      'Ce qui se passe quand on mélange les directions d\'approche entre '
      'l\'apprentissage d\'un point et sa reprise.',
      ['F-MULTI : la même pose reprise alternativement par quatre directions.',
       'D : approche latérale, contre une approche verticale de référence.',
       'Comparaison directe aux séries unidirectionnelles de l\'essai G.'],
      [('5,847 mm', 'F-MULTI — quatre directions mélangées'),
       ('8,091 mm', 'D — approche latérale'),
       ('× 10 à × 20', 'par rapport aux séries unidirectionnelles')],
      lecture=[('Ne jamais mélanger les sens d\'approche entre '
                'l\'apprentissage d\'un point et sa reprise. ', True),
               ('Un point appris par le haut doit être repris par le haut. '
                'C\'est la conséquence opérationnelle la plus importante de '
                'la campagne.', False)],
      liens='Résultats § 2  ·  Méthode § 3 essai 1  ·  feuilles « F-MULTI 4 directions » et '
            '« D approche laterale »  ·  rapport § 6',
      couleur=ROUGE)

fiche('Essai B',
      'Erreur de cible absolue, en boucle ouverte',
      'Relève de l\'exactitude de pose (AP) d\'ISO 9283 — mais n\'y satisfait '
      'pas : l\'AP exige une mesure externe, la nôtre est reconstruite aux codeurs.',
      'L\'écart entre la position commandée et la position atteinte, sans '
      'aucune correction.',
      ['Envoi d\'une consigne cartésienne, sans boucle de retour.',
       'Lecture de la pose atteinte par FK(q_lu) après stabilisation.',
       'Mesure de l\'écart, puis application de la compensation '
       'd\'affaissement gravitaire.'],
      [('14,84 mm', 'brut, sans compensation'),
       ('≈ 2 mm', 'après compensation de l\'affaissement gravitaire')],
      reserve=[('Ce chiffre ne peut pas être présenté comme une exactitude au '
                'sens de la norme. ', True),
               ('Il faudrait pour cela une mesure externe — c\'est l\'objet '
                'de l\'essai 3, non fait, qui exige un comparateur.', False)],
      liens='Résultats § 3  ·  Méthode § 3 essai 2  ·  feuille « B erreur absolue »  ·  rapport § 7')

fiche('Essais C et J',
      'Bruit propre de la vision',
      'VDI/VDE 2634-1 traite l\'erreur de palpage et déclare son essai séparé '
      'non nécessaire. Gardé ici comme diagnostic, pas comme essai normatif.',
      'De combien la vision bouge toute seule, scène parfaitement immobile.',
      ['Soixante trames consécutives, aucun mouvement dans la scène.',
       'Deux traitements comparés : avec et sans raffinement sous-pixel.',
       'Puis mesure de la dispersion sur des moyennes de six trames.'],
      [('0,725 mm', 'RP sur moyenne de six trames, maximum 0,80 mm'),
       ('17,5 mm', 'une seule trame MJPEG corrompue'),
       ('sans effet', 'raffinement sous-pixel : 0,610 → 0,632 mm')],
      lecture=[('Ne jamais commander le robot sur une seule trame. ', True),
               ('Six trames médianées, au minimum. C\'est ce diagnostic — et '
                'non un essai normatif — qui a trouvé les trames corrompues.',
                False)],
      liens='Résultats § 10  ·  Méthode § 3 essai 4  ·  feuilles « C bruit vision » et '
            '« J bruit vision sous-pixel »  ·  rapport § 8 et § 14')

fiche('Essai E',
      'Échelle métrique de la vision',
      'VDI/VDE 2634-1 — erreur de mesure de longueur Δl = l_m − l_k, avec '
      'tolérance E = A + K·L ≤ B.',
      'La vision a-t-elle une erreur d\'échelle ?',
      ['Deux familles de longueurs mesurées sur la même image.',
       [('Longues : ', True), ('distances entre centres de marqueurs, '
                               '383 à 580 mm.', False)],
       [('Courtes : ', True), ('côtés des marqueurs de 50 mm.', False)],
       'Comparaison des deux écarts relatifs.'],
      [('−0,044 %', 'sur les longues distances, 383 à 580 mm'),
       ('−2,6 %', 'sur les côtés de 50 mm')],
      lecture=[('Une vraie erreur d\'échelle frapperait les deux à '
                'l\'identique. Elle ne le fait pas. ', True),
               ('Il n\'y a donc pas d\'erreur d\'échelle : le −2,6 % sur les '
                'petits côtés est un biais de détection, traité à l\'essai L.',
                False)],
      reserve=[('Non-conformité assumée : ', True),
               ('artefact non étalonné, un seul plan au lieu du volume, '
                'aucune tolérance A + K·L déclarée. C\'est cette '
                'non-conformité qui a laissé vivre deux jours une erreur '
                'd\'interprétation.', False)],
      liens='Résultats § 4  ·  Méthode § 3 essai 5 (réécrit le 10/09)  ·  feuille « E echelle vision »  ·  rapport § 9')

fiche('Essai L',
      'Taille réelle des marqueurs — correction du 10/09',
      'VDI/VDE 2634-1 § artefact étalonné. L\'artefact manque toujours : la '
      'conclusion repose sur une extrapolation et un contrôle croisé.',
      'Les marqueurs font-ils vraiment 50 mm ? Une conclusion antérieure — '
      '« imprimés à 48,3 mm, confirmé six fois » — était fausse.',
      ['Cinq marqueurs identiques, douze acquisitions chacun.',
       'Répétabilité sur un même marqueur, puis étalement entre marqueurs.',
       [('Corrélation du côté mesuré avec l\'obliquité de la vue.', True)],
       'Extrapolation à une vue de face.',
       [('Contrôle par une grandeur insensible au biais : les longues '
         'distances.', True)]],
      [('50 mm', 'taille réelle — 50,7 ± 0,7 mm'),
       ('r = −0,920', 'corrélation côté mesuré ↔ obliquité de la vue'),
       ('± 0,108 mm', 'répétabilité sur un même marqueur, contre 1,98 mm '
                      'd\'étalement entre marqueurs identiques')],
      lecture=[('La détection ArUco tire les coins d\'un petit carré vers '
                'l\'intérieur, d\'autant plus qu\'il est vu de biais. Le '
                'biais porte sur la taille, jamais sur la position : ', False),
               ('les quatre coins sont tirés symétriquement, le centre reste '
                'juste à 0,11–0,42 mm.', True)],
      reserve=[('Ne jamais corriger marker_size_mm sur la foi d\'une mesure '
                'optique.', True)],
      liens='Résultats § 10  ·  Méthode § 3 essai 5 et § 4 (les deux réécrits)  ·  '
            'feuille « L taille des marqueurs »  ·  rapport § 16',
      couleur=ROUGE)

# ------------------------------------------- la leçon de méthode de l'essai L
s = nouvelle('Pourquoi six mesures concordantes étaient toutes fausses',
             'Leçon de méthode', 'PROTOCOLE_ESSAIS_PRECISION.md § 3 essai L et § 8',
             couleur_barre=ROUGE)
tf = zone(s, L, Inches(1.5), Inches(5.95), Inches(5.2))
par(tf, 'Ce qui a été cru pendant deux jours', 15, True, ROUGE, CORPS_F,
    espace_apres=8, premier=True)
riche(tf, [('« Les tags sont imprimés à 48,3 mm, erreur de −3,38 %, '
            'confirmée six fois. »', False)], 14, GRIS, espace_apres=14)
par(tf, 'Pourquoi c\'était faux', 15, True, ROUGE, CORPS_F, espace_apres=8)
for t in ['Les six mesures partageaient la même cause d\'erreur : le biais '
          'de détection ArUco sur les petits carrés vus de biais.',
          'L\'une d\'elles ajustait le côté en minimisant la reprojection sur '
          'les coins détectés — donc sur des données déjà biaisées. Elle '
          'était circulaire : elle reproduisait le biais au lieu de le révéler.']:
    riche(tf, [(t, False)], 13.5, espace_apres=8, puce='—   ')
tf = zone(s, Inches(7.05), Inches(1.5), Inches(5.55), Inches(5.2))
par(tf, 'Ce qui l\'a démontré', 15, True, VERT, CORPS_F, espace_apres=8,
    premier=True)
for t in [('Un contrôle de nature différente. ',
           'Les longues distances entre centres de marqueurs sont mesurées '
           'par la même chaîne optique, mais ne subissent pas le biais des '
           'petits côtés : −0,044 % contre −2,6 %.'),
          ('Une variable explicative. ',
           'Le côté mesuré corrèle à r = −0,920 avec l\'obliquité de la vue. '
           'Une vraie erreur d\'impression ne dépendrait pas de l\'angle de '
           'prise de vue.'),
          ('Un étalement incohérent. ',
           'Répétabilité de ± 0,108 mm sur un même marqueur, mais 1,98 mm '
           'd\'étalement entre marqueurs identiques — dix-huit fois plus.')]:
    riche(tf, [(t[0], True), (t[1], False)], 13.5, espace_apres=11)
b = rect(s, L, Inches(5.55), LARGEUR, Inches(1.18), BANDE, GRIS_CL)
tf = b.text_frame
tf.margin_left = Inches(0.24)
tf.margin_top = Inches(0.16)
riche(tf, [('La règle qui en sort. ', True),
           ('Des mesures qui partagent une cause d\'erreur ne se confirment '
            'pas les unes les autres. Le seul contrôle réellement valable '
            'serait un artefact étalonné — un pied à coulisse — ce qu\'exige '
            'VDI/VDE 2634-1 et ce qui manque toujours.', False)], 14,
      premier=True)

fiche('Essai H',
      'Justesse de l\'extrinsèque, par leave-one-out',
      'Aucune norme connue. Protocole construit pour l\'occasion, sur le '
      'principe qu\'un ajustement ne peut pas se valider sur ses propres données.',
      'L\'extrinsèque prédit-elle un marqueur qu\'elle n\'a pas servi à ajuster ?',
      ['Ajuster la pose caméra sur trois marqueurs seulement.',
       'Prédire la position du quatrième.',
       'Comparer à son relevé.',
       'Recommencer en changeant le marqueur exclu.'],
      [('1,86 mm', 'écart de prédiction en interpolation'),
       ('0,4823 px', 'RMS de l\'ajustement de production')],
      lecture=[('Un ajustement sur quatre marqueurs qui reproduit ces quatre '
                'marqueurs ne prouve rien — il est circulaire. ', True),
               ('Seule la prédiction d\'un point exclu mesure quelque chose.',
                False)],
      liens='Résultats § 8  ·  Méthode § 4  ·  feuille « H extrinseque leave-one-out »  ·  rapport § 10  ·  '
            'script scripts/calibrer_extrinseque_4aruco.py (versionné)')

fiche('Essai K',
      'Validation croisée deux caméras, et la limite du plan',
      'Aucune norme ; l\'esprit de VDI/VDE 2634-3 (vues multiples). Deux '
      'chaînes optiques séparées, aucune donnée partagée.',
      'Deux caméras indépendantes voient-elles le même point au même endroit ?',
      ['Choisir un point neuf, jamais utilisé par aucun des deux ajustements.',
       'Le mesurer par l\'arducam, puis par la SVPRO.',
       'Comparer les deux positions reconstruites.',
       [('Puis trianguler les mors de la pince à 172 mm de haut — hors du '
         'plan d\'étalonnage.', True)]],
      [('1,92 mm', 'écart entre les deux caméras, au sol'),
       ('Z = −12 mm', 'triangulation des mors à 172 mm de haut : sous la table')],
      lecture=[('Les deux extrinsèques ne valent que dans le plan de la '
                'table. ', True),
               ('Tous les marqueurs sont à Z = 0 : des points coplanaires '
                'contraignent mal la rotation hors plan, et le résidu ne '
                'peut pas le voir puisqu\'il est mesuré sur ce plan.',
                False)],
      reserve=[('C\'est aussi pourquoi la question du déport d\'outil n\'a '
                'pas pu être tranchée optiquement.', False)],
      liens='Résultats § 11  ·  Méthode § 9  ·  feuille « K svpro et validite du plan »  ·  rapport § 15  ·  '
            'fichier svpro_extrinsic_0909.yaml, champ VALABLE_DANS_LE_PLAN_SEULEMENT')

# ----------------------------------------------------------- essai 7 (I)
s = nouvelle('Précision globale vision + robot, en boucle fermée',
             'Essai 7 (I) — le chiffre qui compte',
             'Résultats § 9  ·  Méthode § 8  ·  feuille « I test 7 boucle fermee »  ·  rapport § 13',
             couleur_barre=VERT)
b = rect(s, L, Inches(1.34), LARGEUR, Inches(0.60), BANDE, None)
rect(s, L, Inches(1.34), Inches(0.055), Inches(0.60), VERT)
tf = b.text_frame
tf.margin_left = Inches(0.22)
tf.margin_top = Inches(0.05)
riche(tf, [('Méthode  ·  ', True),
           ('Aucune norme. ISO 9283 mesure le bras seul, VDI/VDE 2634 '
            'l\'optique seule ; un système asservi par vision tombe entre '
            'les deux — d\'où l\'obligation de décrire le protocole '
            'intégralement.', False)], 12.5, premier=True, espace_apres=0)
tf = zone(s, L, Inches(2.0), Inches(6.0), Inches(4.7))
par(tf, 'Mode opératoire', 13, True, ACCENT_F, CORPS_F, espace_apres=5,
    premier=True)
for i, e in enumerate([
        [('Détecter la cible à la caméra, ', False), ('six trames médianées', True), ('.', False)],
        [('Résoudre l\'IK, envoyer, ', False), ('attendre la stabilisation', True),
         (' — pas l\'accusé de commande.', False)],
        [('Mesurer l\'écart, puis réinjecter l\'écart articulaire : '
          'correction += (q_cible − q_lu).', False)],
        [('Répéter deux fois.', False)],
        [('Vingt essais', True), (', trois mesures chacun : brut, une '
                                  'correction, deux corrections.', False)]]):
    riche(tf, e, 13.5, espace_apres=6, puce='%d.  ' % (i + 1))
par(tf, 'Contrôles de validité', 13, True, ACCENT_F, CORPS_F, espace_avant=10,
    espace_apres=5)
riche(tf, [('Le tag n\'a pas bougé de plus de 0,767 mm pendant la série ; '
            'résidu IK 0,021 mm.', False)], 13.5)
tableau(s, Inches(6.55), Inches(2.02), R - Inches(6.55),
        ['', 'écart moyen', 'dispersion'],
        [['boucle ouverte', [('10,79 mm', True)], '0,230 mm'],
         ['après 1 correction', [('1,29 mm', True)], '0,291 mm'],
         ['après 2 corrections', [('0,63 mm', True)], '0,337 mm']],
        [2.0, 1.5, 1.35], hauteur_ligne=0.44, taille=13)
c = rect(s, Inches(6.55), Inches(3.85), R - Inches(6.55), Inches(0.95), BANDE, GRIS_CL)
tf = c.text_frame
tf.margin_left = Inches(0.22)
tf.margin_top = Inches(0.13)
riche(tf, [('En Z : −9,78 mm → +0,06 mm.', True)], 14, premier=True,
      espace_apres=4)
riche(tf, [('18 essais sur 20 sous le millimètre.', True)], 14)
c = rect(s, Inches(6.55), Inches(4.98), R - Inches(6.55), Inches(1.72), VERT, None)
tf = c.text_frame
tf.margin_left = Inches(0.26)
tf.margin_right = Inches(0.22)
tf.margin_top = Inches(0.16)
par(tf, 'La lecture, en une phrase', 12.5, True,
    RGBColor(0xC6, 0xE0, 0xD2), CORPS_F, espace_apres=6, premier=True)
par(tf, 'Le biais s\'effondre d\'un facteur 20 pendant que la dispersion '
        'reste plate — donc l\'erreur est purement systématique, donc '
        'corrigeable par une boucle visuelle. C\'est ce qui justifie '
        'l\'asservissement plutôt qu\'un recalage statique.', 13, False,
    BLANC, CORPS_F, interligne=1.04)


# --------------------------------------------- couverture des 7 types
s = nouvelle('Les sept types d\'essai — couverture', 'Bilan',
             'Un seul essai reste non fait, et c\'est le seul qui exige un achat')
tableau(s, L, Inches(1.55), LARGEUR,
        ['#', 'type d\'essai', 'état', 'résultat'],
        [['1', 'Répétabilité du bras', 'fait — borne inférieure',
          [('0,00 à 0,84 mm — 6/6 sous la spec', True)]],
         ['2', 'Erreur cartésienne reconstruite', 'fait',
          [('14,84 mm brut  ·  ≈2 mm compensé', True)]],
         ['3', 'Précision physique absolue', [('non fait', True)],
          [('exige un comparateur (~60 €)', True)]],
         ['4', 'Répétabilité de la vision', 'refait le 10/09',
          [('0,725 mm sur six trames', True)]],
         ['5', 'Précision métrique de la vision', 'repris le 10/09',
          [('−0,044 %  ·  tag = 50 mm', True)]],
         ['6', 'Calibration extrinsèque', 'fait — et sa limite trouvée',
          [('1,86 mm  ·  nulle hors du plan', True)]],
         ['7', 'Précision globale vision + robot', 'fait le 10/09',
          [('10,79 → 0,63 mm', True)]]],
        [0.5, 4.3, 3.1, 4.4], hauteur_ligne=0.48, taille=13,
        couleurs_ligne={2: ROUGE})
b = rect(s, L, Inches(5.6), LARGEUR, Inches(1.1), BANDE, GRIS_CL)
tf = b.text_frame
tf.margin_left = Inches(0.24)
tf.margin_top = Inches(0.15)
riche(tf, [('Les trois résultats les plus intéressants de la campagne — H, K '
            'et l\'essai 7 — sont précisément ceux qu\'aucune norme ne '
            'couvre.', True)], 14, premier=True)

# ----------------------------------------------------- conformité honnête
s = nouvelle('Conformité aux normes — la lecture honnête', 'Déclaration',
             'PROTOCOLE_ESSAIS_PRECISION.md § 5  ·  METHODOLOGIE_PRECISION.md § 10',
             couleur_barre=ROUGE)
tf = zone(s, L, Inches(1.42), LARGEUR, Inches(0.7))
riche(tf, [('Sur onze lignes de traçabilité : ', False),
           ('deux sont conformes', True),
           (' — ISO 9787:1999 et ISO 8373, les deux plus faciles ; ', False),
           ('trois suivent une définition normalisée sans son protocole', True),
           (' — A, F-UNI, G ; ', False),
           ('cinq n\'ont aucune norme applicable', True), ('.', False)],
      14, premier=True)
tableau(s, L, Inches(2.30), LARGEUR,
        ['exigence normative', 'source', 'notre situation'],
        [['incertitude de mesure ≤ 25 % de la grandeur mesurée',
          'ISO 9283 § 6.5',
          [('budget 0,10–0,20 mm ; plancher codeur 0,099 mm — à la limite', True)]],
         ['100 % de la charge nominale, obligatoire', 'ISO 9283 Table 1',
          [('essais à vide', True)]],
         ['100 % de la vitesse nominale, obligatoire', 'ISO 9283 Table 2',
          [('essais à vitesse réduite', True)]],
         ['20 °C ± 2, stabilisation « de préférence une nuit »',
          'ISO 9283 § 6.3.2.2', [('non contrôlé', True)]],
         ['artefact étalonné', 'VDI/VDE 2634-1',
          [('absent — cause directe de l\'erreur de deux jours', True)]],
         ['sept lignes de mesure dans le volume', 'VDI/VDE 2634-1',
          [('un seul plan', True)]],
         ['nombre de cycles normatif', 'ISO 9283 § 6.9',
          [('inconnu — page 15, absente de l\'extrait public', True)]]],
        [5.2, 2.35, 4.75], hauteur_ligne=0.40, taille=12.3,
        couleurs_ligne={i: ROUGE for i in range(7)})
b = rect(s, L, Inches(5.42), LARGEUR, Inches(1.28), BANDE, GRIS_CL)
tf = b.text_frame
tf.margin_left = Inches(0.24)
tf.margin_top = Inches(0.15)
riche(tf, [('Ce n\'est pas un défaut à cacher. ', True),
           ('La précision d\'un système vision + robot en boucle fermée '
            'n\'est normalisée nulle part à notre connaissance. Le chiffre '
            'qui compte pour un asservissement visuel — quand la caméra dit '
            'd\'aller là, on arrive à combien ? — tombe entre ISO 9283 et '
            'VDI/VDE 2634. D\'où l\'obligation de décrire le protocole '
            'intégralement, et de déclarer ce tableau.', False)], 13.5,
      premier=True)

# ------------------------------------------------- état du banc au 10/09
s = nouvelle('Avertissement — l\'état du banc a changé le 10/09',
             'À lire avant toute reprise',
             'PROTOCOLE_ESSAIS_PRECISION.md § 10', couleur_barre=ROUGE)
tf = zone(s, L, Inches(1.45), LARGEUR, Inches(1.0))
riche(tf, [('La planche a bougé. ', True),
           ('Mesure sur les quatre marqueurs : rotation −1,750°, translation '
            '(18,8 · −6,5) mm, résidu 0,39 mm, distances entre centres '
            'conservées à 0,14 % — donc un mouvement rigide, pas une '
            'déformation.', False)], 14.5, premier=True, espace_apres=8)
riche(tf, [('Ce n\'est pas la caméra : ', True),
           ('le trépied au sol, l\'objet de fond le plus net, n\'a bougé que '
            'de 0,2 px. Un déplacement caméra de 19 mm l\'aurait décalé de '
            'plusieurs dizaines de pixels, et un basculement aurait cassé le '
            'modèle rigide, qui tient.', False)], 14.5)
par(zone(s, L, Inches(3.28), LARGEUR, Inches(0.4)),
    'Deux conséquences opposées, à ne pas confondre', 15, True, ENCRE,
    CORPS_F, premier=True)
c = rect(s, L, Inches(3.78), Inches(6.05), Inches(1.95), BANDE, VERT, 1.5)
tf = c.text_frame
tf.margin_left = Inches(0.24)
tf.margin_right = Inches(0.2)
tf.margin_top = Inches(0.17)
par(tf, 'arducam_extrinsic_pick.yaml — RESTE VALABLE', 14.5, True, VERT,
    CORPS_F, espace_apres=7, premier=True)
par(tf, 'Le lien caméra ↔ base robot est intact. Ne pas la recalibrer : la '
        'refaire contre des positions nominales périmées y injecterait les '
        '19 mm.', 13.5, False, ENCRE, CORPS_F, interligne=1.05)
c = rect(s, Inches(7.28), Inches(3.78), R - Inches(7.28), Inches(1.95), BANDE,
         ROUGE, 1.5)
tf = c.text_frame
tf.margin_left = Inches(0.24)
tf.margin_right = Inches(0.2)
tf.margin_top = Inches(0.17)
par(tf, 'workspace_markers.yaml — PÉRIMÉ', 14.5, True, ROUGE, CORPS_F,
    espace_apres=7, premier=True)
par(tf, 'Toute calibration qui s\'appuie dessus sera fausse de 12 à 28 mm '
        'selon le marqueur. Tout point enseigné en coordonnées planche avant '
        'le 10/09 est décalé d\'autant.', 13.5, False, ENCRE, CORPS_F,
    interligne=1.05)
tf = zone(s, L, Inches(5.95), LARGEUR, Inches(0.9))
riche(tf, [('Deux pièges consignés dans les fichiers eux-mêmes : ', True),
           ('après un rebranchement USB, la même exposition ne rend plus la '
            'même luminance et les anciens scripts remettent le gain à 0 ; '
            'un câble noir touchant la bordure d\'un marqueur le rend '
            'invisible à ArUco.', False)], 13, premier=True)

# ------------------------------------------------------ ce qui reste à faire
s = nouvelle('Ce qui reste à faire', 'Suite', 'PROTOCOLE_ESSAIS_PRECISION.md § 9')
tableau(s, L, Inches(1.6), LARGEUR,
        ['priorité', 'action', 'ce que ça débloque'],
        [['1', [('Mesurer un marqueur au pied à coulisse', True)],
          'l\'artefact étalonné qu\'exige VDI/VDE 2634-1. Son absence est '
          'exactement ce qui a laissé vivre l\'erreur deux jours'],
         ['2', [('Recalibrer avec des marqueurs à plusieurs hauteurs', True)],
          'lève la dégénérescence du plan, rend les extrinsèques valables en '
          'volume, débloque la question du déport d\'outil'],
         ['3', 'Essai 3 — précision physique absolue',
          'la seule mesure d\'exactitude au sens de la norme ; exige un '
          'comparateur (~60 €)'],
         ['4', 'ISO 9283 § 7.3 — exactitude et répétabilité de distance',
          'seule caractéristique normalisée qu\'une mesure aux codeurs peut '
          'rapporter en valeur vraie : un décalage constant s\'annule dans '
          'la différence'],
         ['5', 'Remonter les scripts du bac à sable dans le dépôt',
          'sans quoi aucun essai n\'est rejouable']],
        [1.0, 4.6, 6.7], hauteur_ligne=0.66, taille=12.8)
b = rect(s, L, Inches(5.35), LARGEUR, Inches(1.3), BANDE, ROUGE, 1.5)
tf = b.text_frame
tf.margin_left = Inches(0.24)
tf.margin_top = Inches(0.15)
riche(tf, [('Reproductibilité — la réserve la plus lourde. ', True),
           ('Deux scripts seulement sont versionnés : '
            'scripts/calibrer_extrinseque_4aruco.py (essai H) et '
            'scripts/aruco_check.py. Les onze autres vivent dans le '
            'répertoire de session, qui est effacé. En l\'état, onze essais '
            'sur treize ne sont pas rejouables.', False)], 13.5, premier=True)

# ---------------------------------------------------------- clôture
s = prs.slides.add_slide(VIDE)
rect(s, Inches(0), Inches(0), prs.slide_width, prs.slide_height, ENCRE)
rect(s, Inches(0), Inches(2.28), prs.slide_width, Inches(0.035), ACCENT)
tf = zone(s, Inches(1.1), Inches(1.15), Inches(11), Inches(1.0))
par(tf, 'CE QU\'IL FAUT RETENIR', 13, True, RGBColor(0x8F, 0xB8, 0xDC),
    CORPS_F, espace_apres=10, premier=True)
par(tf, 'Trois chiffres, et une règle', 34, False, BLANC, TITRE_F)
y = Inches(2.72)
for valeur, texte in [
        ('0,63 mm',
         'la précision du système vision + robot après deux corrections en '
         'boucle fermée, contre 10,79 mm en boucle ouverte. L\'erreur est '
         'systématique, donc corrigeable.'),
        ('0,4 à 0,8 mm',
         'la répétabilité du bras, six séries unidirectionnelles sur six '
         'sous la spécification de 1 mm — mais c\'est une borne inférieure, '
         'mesurée aux codeurs.'),
        ('5,8 à 8,1 mm',
         'ce que coûte le mélange des sens d\'approche. Un point appris par '
         'le haut doit être repris par le haut.')]:
    tf = zone(s, Inches(1.1), y, Inches(2.65), Inches(0.6))
    par(tf, valeur, 26, True, RGBColor(0x7E, 0xB8, 0xE8), TITRE_F,
        premier=True, align=PP_ALIGN.RIGHT)
    tf = zone(s, Inches(4.05), y + Inches(0.06), Inches(8.2), Inches(1.0))
    par(tf, texte, 14.5, False, RGBColor(0xD5, 0xDF, 0xEA), CORPS_F,
        premier=True, interligne=1.06)
    y = y + Inches(1.12)
b = rect(s, Inches(1.1), Inches(6.18), Inches(11.13), Inches(0.82),
         RGBColor(0x2A, 0x39, 0x59), None)
tf = b.text_frame
tf.margin_left = Inches(0.28)
tf.margin_top = Inches(0.14)
riche(tf, [('Et la règle : ', True),
           ('des mesures qui partagent une cause d\'erreur ne se confirment '
            'pas les unes les autres. Un contrôle n\'en est un que s\'il est '
            'de nature différente.', False)], 14.5, BLANC, premier=True)

import sys
sortie = sys.argv[1] if len(sys.argv) > 1 else 'protocole.pptx'
prs.save(sortie)
print('%d diapositives -> %s' % (len(prs.slides.__iter__.__self__._sldIdLst), sortie))
