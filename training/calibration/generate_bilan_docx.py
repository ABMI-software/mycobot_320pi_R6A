#!/usr/bin/env python3
"""Génère le bilan expérimental Word de la campagne de précision.

    .venv/bin/python training/calibration/generate_bilan_docx.py

Sources versionnées : PROTOCOLE_ESSAIS_PRECISION.md (table de correspondance),
PRECISION_MYCOBOT_320PI.md (chiffres), METHODOLOGIE_PRECISION.md (normes),
docs/PICK_AND_PLACE_REAL.md § « Calibration contre le robot » (correction du 15/09).
"""
from pathlib import Path

from docx import Document
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.shared import Pt, RGBColor

CAL = Path(__file__).resolve().parent
OUT = CAL / "BILAN_EXPERIMENTAL_PRECISION_320PI.docx"

doc = Document()
for nom in ("Normal", "Body Text"):
    try:
        doc.styles[nom].font.size = Pt(10.5)
    except KeyError:
        pass


def para(texte, gras=False, italique=False, gris=False):
    p = doc.add_paragraph()
    r = p.add_run(texte)
    r.bold, r.italic = gras, italique
    if gris:
        r.font.size = Pt(9)
        r.font.color.rgb = RGBColor(0x55, 0x55, 0x55)
    return p


def puces(lignes):
    for l in lignes:
        doc.add_paragraph(l, style="List Bullet")


def table(entetes, lignes, legende=None):
    t = doc.add_table(rows=1, cols=len(entetes))
    t.style = "Table Grid"
    for c, e in zip(t.rows[0].cells, entetes):
        c.text = ""
        r = c.paragraphs[0].add_run(e)
        r.bold = True
        r.font.size = Pt(9)
    for ligne in lignes:
        for c, v in zip(t.add_row().cells, ligne):
            c.text = ""
            r = c.paragraphs[0].add_run(str(v))
            r.font.size = Pt(9)
    if legende:
        p = doc.add_paragraph()
        r = p.add_run(legende)
        r.italic = True
        r.font.size = Pt(9)
        r.font.color.rgb = RGBColor(0x55, 0x55, 0x55)
    doc.add_paragraph()
    return t


# ─────────────────────────── Titre ───────────────────────────
doc.add_heading("Bilan expérimental — précision du myCobot 320 Pi", 0)
p = doc.add_paragraph()
r = p.add_run("Campagne des 8, 9 et 10 septembre 2026, prolongée par la recalibration "
              "contre le robot du 15 septembre 2026\n"
              "Chaîne vision + bras : Arducam et SVPRO, marqueurs ArUco 4×4 de 50 mm, "
              "MyCobot 320 Pi, pince Pro adaptative")
r.italic = True
p.alignment = WD_ALIGN_PARAGRAPH.CENTER

# ─────────────────────── Résumé exécutif ───────────────────────
doc.add_heading("Résumé exécutif", level=1)
para("Treize essais ont été conduits pour chiffrer la précision de la chaîne complète "
     "caméra → robot, et pour séparer ce qui vient du bras, de la vision, et du calibrage "
     "qui les relie. Les résultats, dans l'ordre de leur importance opérationnelle :")
puces([
    "Le terme dominant n'est pas la répétabilité du bras, c'est le SENS D'APPROCHE : "
    "5,85 à 8,09 mm de biais quand les directions sont mélangées, contre 0,00 à 0,84 mm "
    "quand chaque direction est prise isolément — un facteur 10 à 20.",
    "L'erreur de la chaîne complète est SYSTÉMATIQUE, donc corrigeable : en boucle fermée, "
    "l'écart moyen passe de 10,79 mm à 0,63 mm en deux corrections, pendant que la "
    "dispersion reste plate (0,230 à 0,337 mm). C'est le résultat qui justifie "
    "l'asservissement visuel plutôt qu'un recalage statique.",
    "Le bras tient son chiffre constructeur : six séries unidirectionnelles sur six sont "
    "sous le 1 mm de la fiche officielle, la pire à 0,838 mm. Réserve dirimante : la mesure "
    "étant relevée aux codeurs, elle échoue à infirmer la conformité, elle ne la démontre pas.",
    "La vision n'a pas d'erreur d'échelle : −0,044 % sur les distances entre centres de "
    "marqueurs (383 à 580 mm). Le −2,6 % observé sur les côtés de 50 mm est un biais de "
    "détection lié à l'obliquité, pas une erreur d'impression.",
    "Les deux extrinsèques ne valent que DANS LE PLAN de la table : 1,92 mm en validation "
    "croisée deux caméras au sol, aucune validité à 17 cm de haut. Tous les marqueurs "
    "d'étalonnage sont coplanaires à Z = 0.",
])
para("Deux conclusions antérieures ont été retirées en cours de campagne après avoir été "
     "reconnues circulaires. Elles sont exposées en section 8 : ce sont les enseignements de "
     "méthode les plus transférables de ce travail.", italique=True)

# ───────────────────── 1. Objectif et périmètre ─────────────────────
doc.add_heading("1. Objectif et périmètre", level=1)
para("La question posée est opérationnelle : quand la caméra dit que la cible est à tel "
     "endroit, à combien de millimètres le robot y arrive-t-il ? Elle ne se réduit ni à la "
     "spécification du bras ni à celle de l'optique.")
para("Le périmètre couvre donc quatre maillons et leurs interfaces :")
puces([
    "le bras seul — répétabilité et erreur de cible en boucle ouverte ;",
    "la vision seule — bruit temporel et justesse métrique ;",
    "le calibrage extrinsèque, qui convertit les pixels en millimètres robot ;",
    "la chaîne complète asservie, seule grandeur réellement utile à une saisie.",
])
para("Hors périmètre, et déclaré comme tel : la précision physique absolue du point atteint, "
     "qui exige un moyen de mesure externe (comparateur) dont le banc ne dispose pas ; et le "
     "comportement en charge et à vitesse nominale, qu'ISO 9283 exige et que la campagne n'a "
     "pas reproduit.")

# ─────────────── 2. Dispositif expérimental ───────────────
doc.add_heading("2. Dispositif expérimental", level=1)
table(
    ["élément", "caractéristiques"],
    [
        ["Bras", "MyCobot 320 Pi, 6 axes, rayon nominal 350 mm, pince Pro adaptative "
                 "(gripper_id 14). Commande par pont TCP, port 5005."],
        ["Caméra 1 — Arducam", "640 × 480, montage zénithal, exposition manuelle 75, "
                               "intrinsèque cam_3 étalonnée au ChArUco."],
        ["Caméra 2 — SVPRO", "800 × 600 ramené à 640 × 480, vue oblique, intrinsèque cam_2."],
        ["Artefacts", "4 marqueurs ArUco DICT_4X4, identifiants 19, 23, 25 et 26, côté "
                      "nominal 50 mm, posés à plat sur le plateau (Z = 0)."],
        ["Plateau", "622 × 449 × 8,5 mm, mesuré le 09/09/2026."],
        ["Repère", "base_link du robot, conforme ISO 9787:1999. X vers l'avant, Y vers la "
                   "gauche, Z vers le haut ; dessus du plateau à Z = 0."],
        ["Domaine d'allonge", "outil vertical jusqu'à ~320 mm ; outil incliné −15° à −45° "
                              "jusqu'à ~390 mm ; au-delà, aucune solution IK."],
    ],
    "Tableau 1 — Le banc. L'inclinaison de l'outil n'est pas un choix de confort : à 332 mm "
    "d'allonge, l'outil vertical n'a aucune solution IK, vérifié sur huit hauteurs.")

# ──────────── 3. Ce que mesure la méthode ────────────
doc.add_heading("3. Ce que la méthode mesure — et ce qu'elle ne mesure pas", level=1)
para("Toute la campagne repose sur une seule grandeur : la position reconstruite par "
     "cinématique directe sur les angles RELUS aux codeurs, FK(q_lu). Ce choix conditionne la "
     "portée de chaque chiffre et doit être énoncé avant tout résultat.")
para("Ce qu'elle voit.", gras=True)
para("Toute la dispersion en amont des codeurs : bruit de commande, stabilisation des "
     "servomoteurs, résolution de lecture, non-reproductibilité de la consigne. C'est "
     "suffisant pour une répétabilité, qui est une dispersion et non une justesse.")
para("Ce qu'elle ne voit pas.", gras=True)
para("Tout ce qui se trouve en AVAL des codeurs : jeu des réducteurs, souplesse des liens, "
     "flexion de l'outil, erreur du modèle cinématique lui-même, définition du point outil. "
     "Le constructeur, lui, mesure par un moyen externe.")
para("Conséquence, énoncée sans atténuation : tout chiffre de répétabilité produit ici est "
     "une BORNE INFÉRIEURE. Passer sous un seuil de spécification n'en démontre pas la "
     "conformité — cela échoue seulement à l'infirmer.", gras=True)
para("Plancher instrumental : 0,099 mm, soit un bit de poids faible de 0,01° propagé sur les "
     "six axes. Les valeurs mesurées se situent 3 à 8 fois au-dessus : elles ne sont donc pas "
     "limitées par la résolution de lecture.")

# ──────────── 4. Tableau maître ────────────
doc.add_heading("4. Tableau maître des treize essais", level=1)
table(
    ["code", "essai", "norme de référence", "résultat"],
    [
        ["A", "Répétabilité, 10 allers-retours verticaux", "ISO 9283:1998 (définition seule)", "0,403 mm"],
        ["F-UNI", "Répétabilité, 6 allers-retours (contrôle)", "ISO 9283:1998 (définition seule)", "0,557 mm"],
        ["G", "40 allers-retours, 10 par côté", "ISO 9283:1998 (définition seule)", "0,000 à 0,838 mm"],
        ["F-MULTI", "Sens d'approche mélangés", "hors norme par construction", "5,847 mm"],
        ["D", "Approche latérale", "hors norme par construction", "8,091 mm"],
        ["B", "Erreur de cible absolue, boucle ouverte", "ISO 9283 exactitude AP — non satisfaite", "14,84 mm → ≈ 2 mm compensé"],
        ["C", "Bruit propre de la vision, 60 trames", "VDI/VDE 2634-1 — essai déclaré redondant", "0,725 mm sur 6 trames"],
        ["E", "Échelle métrique de la vision", "VDI/VDE 2634-1, erreur de longueur Δl", "−0,044 %"],
        ["H", "Extrinsèque par leave-one-out", "aucune norme connue", "1,86 mm"],
        ["7 (I)", "Précision globale vision + robot", "aucune norme", "10,79 → 0,63 mm"],
        ["J", "Bruit sous-pixel, trames corrompues", "diagnostic", "17,5 mm sur une trame"],
        ["K", "Validation croisée deux caméras", "aucune norme ; esprit VDI/VDE 2634-3", "1,92 mm"],
        ["L", "Taille réelle des marqueurs", "VDI/VDE 2634-1, artefact étalonné", "50 mm (50,65 extrapolé)"],
    ],
    "Tableau 2 — Les treize essais. Les données brutes figurent dans le classeur "
    "precision_campagne_2026-09-09.xlsx, dix-sept feuilles, une par essai.")

# ──────────── 5. Résultats par famille ────────────
doc.add_heading("5. Résultats par famille", level=1)

doc.add_heading("5.1 Répétabilité du bras — essais A, F-UNI et G", level=2)
para("Protocole. Une pose de travail unique, bras à vide, cible à 332 mm d'allonge, outil "
     "incliné à −15°. Retrait de 40 mm puis retour sur le même point, sens d'approche "
     "CONSTANT. Lecture après stabilisation, jamais sur accusé de commande. Répétabilité "
     "calculée selon la définition ISO 9283, RP = moyenne des distances au barycentre + 3σ. "
     "L'écart maximum n'est jamais utilisé : il sous-estime RP systématiquement.")
table(
    ["série", "n", "RP ISO 9283", "écart max", "portée du départ", "vs 0,5 mm", "vs 1 mm"],
    [
        ["A — par le haut", "10", "0,403 mm", "0,306", "—", "sous", "sous"],
        ["F-UNI — par le haut", "6", "0,557 mm", "0,424", "—", "au-dessus", "sous"],
        ["G — depuis l'arrière", "10", "0,000 mm", "0,000", "292 mm", "sous", "sous"],
        ["G — depuis la droite", "10", "0,186 mm", "0,138", "331 mm", "sous", "sous"],
        ["G — depuis l'avant", "10", "0,794 mm", "0,551", "372 mm", "au-dessus", "sous"],
        ["G — depuis la gauche", "10", "0,838 mm", "0,783", "338 mm", "au-dessus", "sous"],
    ],
    "Tableau 3 — Six séries unidirectionnelles. La série la plus dispersée est la seule dont "
    "le point de départ, à 372 mm, sort du rayon nominal de 350 mm : hors enveloppe, aucune "
    "spécification ne s'applique.")
para("Une difficulté propre à ce poste : le constructeur se contredit. La page produit du "
     "320 Pi annonce 1 mm de répétabilité et 850 g ; le GitBook technique annonce ±0,5 mm et "
     "3 kg. Le robot pèse effectivement ~3 kg, ce qui affaiblit la page produit sans prouver "
     "que son 1 mm soit faux. Selon la source retenue, trois séries sur six ou six sur six "
     "sont conformes. Tant que le constructeur n'est pas départagé, la campagne ne conclut "
     "pas et donne les deux colonnes.")

doc.add_heading("5.2 Le sens d'approche — essais F-MULTI et D", level=2)
para("Statut normatif. Hors du champ d'ISO 9283 PAR CONSTRUCTION : la norme définit la "
     "répétabilité pour des approches faites dans la même direction. Une série "
     "multidirectionnelle ne mesure pas une répétabilité au sens normatif — elle mesure un "
     "biais de jeu mécanique. L'essai est donc rapporté comme tel.")
table(
    ["série", "condition", "résultat", "rapport aux séries unidirectionnelles"],
    [
        ["F-MULTI", "quatre directions mélangées", "5,847 mm", "× 7 à × 10"],
        ["D", "approche latérale contre approche verticale", "8,091 mm", "× 10 à × 20"],
    ],
    "Tableau 4 — Même point d'arrivée, même consigne ; seule l'origine du dernier segment "
    "change. Trois mesures indépendantes donnent le même ordre de grandeur.")
para("C'est le résultat opérationnel le plus important de la campagne : ne jamais mélanger "
     "les sens d'approche entre l'apprentissage d'un point et sa reprise. Un point appris par "
     "le haut doit être repris par le haut. Ce terme vaut à lui seul sept fois la pire "
     "répétabilité et six fois la spécification constructeur ; dans une tâche de tri réelle, "
     "il écrase tous les autres.", gras=True)

doc.add_heading("5.3 Erreur de cible absolue — essai B", level=2)
para("Mesure de l'écart entre position commandée et position atteinte, sans aucune "
     "correction : 14,84 mm brut, ramené à environ 2 mm une fois l'affaissement gravitaire "
     "compensé par réinjection de l'écart articulaire.")
para("Réserve dirimante. Cette grandeur relève de l'exactitude de pose (AP) d'ISO 9283, mais "
     "n'y satisfait pas : l'AP exige une mesure EXTERNE, la nôtre est reconstruite aux "
     "codeurs et contient donc l'erreur du modèle qu'elle prétend évaluer. Le chiffre ne peut "
     "pas être présenté comme une exactitude au sens de la norme ; il est rapporté comme "
     "l'amplitude d'un terme systématique dont la compensation, elle, est vérifiée.")

doc.add_heading("5.4 La vision — bruit (C, J) et justesse métrique (E, L)", level=2)
para("Bruit propre, essai C.", gras=True)
para("Soixante trames consécutives, scène strictement immobile, avec et sans raffinement "
     "sous-pixel. Trois enseignements : le raffinement sous-pixel est sans effet utile "
     "(0,610 contre 0,632 mm) ; une trame ISOLÉE peut être fausse de 17,5 mm, par corruption "
     "du flux MJPEG ; la médiane sur six trames ramène la dispersion à 0,725 mm, maximum "
     "0,80 mm. Règle opérationnelle : ne jamais commander le robot sur une seule trame.")
para("C'est ce diagnostic, et non un essai normatif, qui a révélé les trames corrompues. "
     "VDI/VDE 2634-1 déclare l'essai d'erreur de palpage non nécessaire ; il a été conservé "
     "ici comme diagnostic, et il a payé.", italique=True)
para("Justesse métrique, essais E et L.", gras=True)
table(
    ["grandeur mesurée", "valeur", "lecture"],
    [
        ["distances entre centres de marqueurs, 383 à 580 mm", "−0,044 %", "juste"],
        ["côtés de marqueurs de 50 mm", "−2,6 %", "biaisé"],
        ["répétabilité sur un même marqueur", "± 0,108 mm", "excellente"],
        ["étalement entre marqueurs identiques", "1,98 mm, soit 4,0 %", "18 fois la répétabilité"],
        ["corrélation côté mesuré ↔ obliquité de la vue", "r = −0,920", "cause identifiée"],
        ["extrapolation à une vue de face", "50,65 mm", "conforme au nominal"],
    ],
    "Tableau 5 — Une vraie erreur d'échelle frapperait les deux familles de longueurs à "
    "l'identique. Elle ne le fait pas : il n'y a donc pas d'erreur d'échelle.")
para("Conclusion des essais E et L : les marqueurs font bien 50 mm (50,7 ± 0,7 mm). La "
     "détection ArUco tire les coins d'un petit carré vers l'intérieur, d'autant plus qu'il "
     "est vu de biais. Le biais porte sur la TAILLE, jamais sur la POSITION : les quatre "
     "coins sont tirés symétriquement, le centre reste juste à 0,11–0,42 mm. Une erreur de "
     "taille ne se propage donc pas en erreur de position.")
para("Consigne ferme qui en découle : ne jamais corriger marker_size_mm sur la foi d'une "
     "mesure optique. Le fichier de calibration porte le champ "
     "« taille_marqueur_NE_PAS_CORRIGER » pour le rappeler sur place.", gras=True)

doc.add_heading("5.5 Le calibrage extrinsèque — essais H et K", level=2)
para("Essai H — leave-one-out.", gras=True)
para("Aucune norme connue ne couvre cette vérification ; le protocole a été construit pour "
     "l'occasion. La pose caméra est ajustée sur TROIS marqueurs seulement, puis on prédit la "
     "position du quatrième et on la compare à son relevé, en changeant à chaque fois le "
     "marqueur exclu. Motif : un ajustement sur quatre marqueurs qui reproduit ces quatre "
     "marqueurs ne prouve rien — il est circulaire. Seule la prédiction d'un point exclu "
     "mesure quelque chose. Résultat : 1,86 mm en interpolation.")
para("Essai K — validation croisée deux caméras.", gras=True)
para("Un point NEUF, jamais utilisé par aucun des deux ajustements, mesuré indépendamment "
     "par l'Arducam et par la SVPRO : 1,92 mm d'écart. C'est le contrôle le plus solide de la "
     "campagne — deux chaînes optiques séparées, aucune donnée partagée.")
para("Et la limite que cet essai a révélée.", gras=True)
para("Trianguler les mors de la pince à 172 mm de hauteur donnait Z = −12 mm, soit sous la "
     "table. Cause : tous les marqueurs d'étalonnage sont dans le plan Z = 0. Des points "
     "coplanaires fixent la pose DANS le plan mais contraignent mal la rotation hors plan, et "
     "le résidu d'ajustement ne peut pas le détecter puisqu'il est lui-même mesuré sur ce "
     "plan. Les deux extrinsèques ne valent donc que dans le plan de la table : 1,92 mm au "
     "sol, sans aucune valeur à 17 cm de haut. C'est aussi la raison pour laquelle la "
     "question du déport d'outil n'a pas pu être tranchée optiquement.")

doc.add_heading("5.6 La chaîne complète asservie — essai 7 (I)", level=2)
para("C'est le chiffre qui compte. Protocole : détection de la cible sur six trames "
     "médianées, résolution de l'IK, envoi, attente de STABILISATION et non d'accusé de "
     "commande, mesure de l'écart, puis réinjection de l'écart articulaire "
     "(correction += q_cible − q_lu), répétée deux fois. Vingt essais, trois mesures chacun.")
table(
    ["régime", "écart moyen", "dispersion"],
    [
        ["boucle ouverte", "10,79 mm", "0,230 mm"],
        ["après 1 correction", "1,29 mm", "0,291 mm"],
        ["après 2 corrections", "0,63 mm", "0,337 mm"],
    ],
    "Tableau 6 — Vingt essais. En Z, l'écart passe de −9,78 mm à +0,06 mm. Dix-huit essais "
    "sur vingt finissent sous le millimètre.")
para("La lecture, en une phrase : le biais s'effondre d'un facteur 20 pendant que la "
     "dispersion reste plate. L'erreur est donc PUREMENT SYSTÉMATIQUE, donc corrigeable par "
     "une boucle visuelle. C'est ce résultat, et non la spécification du bras, qui justifie "
     "l'asservissement plutôt qu'un recalage statique.", gras=True)
para("Contrôles de validité joints : le marqueur cible n'a pas bougé de plus de 0,767 mm "
     "pendant toute la série, et le résidu de résolution IK est resté à 0,021 mm. Les deux "
     "sont très inférieurs à l'effet mesuré.")

# ──────────── 6. Couverture ────────────
doc.add_heading("6. Couverture des sept types d'essai prévus", level=1)
table(
    ["n°", "type d'essai", "état", "résultat"],
    [
        ["1", "Répétabilité du bras", "fait — borne inférieure", "0,00 à 0,84 mm ; 6/6 sous la spécification"],
        ["2", "Erreur cartésienne reconstruite", "fait", "14,84 mm brut ; ≈ 2 mm compensé"],
        ["3", "Précision physique absolue", "NON FAIT", "exige un comparateur (~60 €)"],
        ["4", "Répétabilité de la vision", "refait le 10/09", "0,725 mm sur 6 trames"],
        ["5", "Précision métrique de la vision", "repris le 10/09", "−0,044 % ; marqueur = 50 mm"],
        ["6", "Calibration extrinsèque", "fait — et sa limite trouvée", "1,86 mm ; nulle hors du plan"],
        ["7", "Précision globale vision + robot", "fait le 10/09", "10,79 → 0,63 mm"],
    ],
    "Tableau 7 — Un seul essai reste non fait, et c'est le seul qui exige un achat.")

# ──────────── 7. Conformité normative ────────────
doc.add_heading("7. Conformité normative — la lecture honnête", level=1)
para("Sur onze lignes de traçabilité établies :")
puces([
    "DEUX sont conformes — ISO 9787:1999 (systèmes de coordonnées : tout est exprimé en "
    "base_link) et ISO 8373 (vocabulaire). Ce sont les deux plus faciles.",
    "TROIS suivent une définition normalisée sans son protocole — les essais A, F-UNI et G "
    "appliquent la définition ISO 9283 de la répétabilité, pas ses conditions d'essai.",
    "CINQ n'ont aucune norme applicable — dont les trois résultats les plus intéressants de "
    "la campagne : H, K et l'essai 7.",
])
para("Ce n'est pas un défaut à dissimuler. La précision d'un système vision + robot en "
     "boucle fermée n'est, à notre connaissance, normalisée nulle part : ISO 9283 mesure le "
     "bras seul, VDI/VDE 2634 mesure l'optique seule, et la grandeur utile à un "
     "asservissement visuel tombe entre les deux. D'où l'obligation de décrire intégralement "
     "le protocole, ce que fait ce bilan.")
para("Écarts au protocole normatif, déclarés.", gras=True)
table(
    ["exigence", "source", "situation du banc"],
    [
        ["incertitude de mesure ≤ 25 % de la grandeur mesurée", "ISO 9283 § 6.5",
         "répétabilités 0,4–0,8 mm → budget 0,10–0,20 mm ; plancher codeur 0,099 mm — À LA LIMITE"],
        ["100 % de la charge nominale", "ISO 9283 Table 1", "essais à vide"],
        ["100 % de la vitesse nominale", "ISO 9283 Table 2", "essais à vitesse réduite"],
        ["20 °C ± 2, stabilisation d'une nuit de préférence", "ISO 9283 § 6.3.2.2", "non contrôlé"],
        ["cinq poses, trente cycles", "ISO 9283", "une pose, dix cycles"],
        ["artefact étalonné", "VDI/VDE 2634-1", "ABSENT — cause directe de l'erreur décrite en 8.1"],
        ["sept lignes de mesure dans le volume", "VDI/VDE 2634-1", "un seul plan"],
        ["nombre de cycles normatif", "ISO 9283 § 6.9", "inconnu ; page absente de l'extrait public"],
    ],
    "Tableau 8 — Ce que la campagne ne tient pas, et pourquoi elle peut néanmoins être "
    "rapportée.")
para("ISO 9283 autorise explicitement cette démarche, en introduction : l'utilisateur de la "
     "norme choisit les caractéristiques à éprouver, et les essais décrits peuvent être "
     "appliqués en tout ou partie. Un protocole partiel est légitime À CONDITION D'ÊTRE "
     "DÉCLARÉ, ce que fait le tableau ci-dessus.", italique=True)

# ──────────── 8. Erreurs corrigées ────────────
doc.add_heading("8. Deux conclusions retirées — et ce qu'elles enseignent", level=1)
para("Les deux erreurs ci-dessous ont été commises, détectées et corrigées pendant la "
     "campagne. Elles partagent exactement la même cause : une vérification qui repassait par "
     "la grandeur qu'elle prétendait vérifier. Elles sont rapportées intégralement, parce "
     "qu'elles constituent le principal apport méthodologique de ce travail.")

doc.add_heading("8.1 La taille des marqueurs — six mesures, une seule cause d'erreur", level=2)
para("Conclusion retirée. Une note antérieure affirmait que les marqueurs étaient imprimés à "
     "48,3 mm, soit une erreur de −3,38 %, et la déclarait confirmée six fois.")
para("Pourquoi elle était fausse. Les six mesures partageaient la MÊME cause d'erreur, le "
     "biais d'obliquité de la détection ArUco. L'une d'elles ajustait même le côté en "
     "minimisant la reprojection sur les coins DÉTECTÉS, donc sur des données déjà biaisées : "
     "elle reproduisait le biais au lieu de le révéler. Elle était circulaire.")
para("Ce qui a tranché. Une grandeur INSENSIBLE au biais : les distances entre centres, "
     "justes à −0,044 %, quand les côtés de 50 mm affichaient −2,6 %. Une vraie erreur "
     "d'échelle aurait frappé les deux à l'identique.")
para("Leçon. Des mesures qui partagent une cause d'erreur ne se confirment pas les unes les "
     "autres. Le seul contrôle valable serait un artefact étalonné — un pied à coulisse — ce "
     "qu'exige VDI/VDE 2634-1 et ce qui manque toujours au banc.", gras=True)

doc.add_heading("8.2 Le calibrage extrinsèque — un contrôle à 0,3 mm pour une pince à 5 cm", level=2)
para("Conclusion retirée. Une note du 10/09 déclarait l'extrinsèque de l'Arducam valable et "
     "recommandait de ne pas la recalibrer.")
para("Le symptôme. Le contrôle de calibration affichait 0,3 mm d'écart, tandis que la pince "
     "tombait 5 à 10 cm à côté de l'objet à saisir.")
para("Le diagnostic, posé le 15/09. La chaîne d'une saisie est : pixel de l'objet → "
     "extrinsèque → millimètres robot → IK. Le bras était déjà précis à quelques millimètres. "
     "L'étape fausse était l'extrinsèque : elle était calibrée contre un fichier de référence "
     "dont les positions de marqueurs avaient elles-mêmes été relevées À TRAVERS une "
     "extrinsèque déjà fausse. Le contrôle comparait donc la caméra à elle-même.")
table(
    ["marqueur", "position mesurée au robot", "erreur de l'Arducam", "erreur de la SVPRO"],
    [
        ["19", "(87,1 ; 212,4) mm", "99,5 mm — (+99,2 ; −6,4)", "62,4 mm — (−61,8 ; +8,5)"],
        ["23", "(110,9 ; −163,0) mm", "40,2 mm — (+38,4 ; −12,1)", "97,0 mm — (−96,2 ; −13,1)"],
    ],
    "Tableau 9 — Les deux caméras se trompaient en SENS OPPOSÉS : l'Arducam trop loin du "
    "robot, la SVPRO trop près. Le tableau de bord, qui apprend l'écart entre les deux vues, "
    "masquait le problème au lieu de le révéler.")
para("Leçon, identique à la précédente sous une autre forme : une référence ne doit jamais "
     "être relevée à travers le système qu'elle sert à valider. Il faut une mesure "
     "indépendante — ici, le robot lui-même.", gras=True)

# ──────────── 9. Recalibration 15/09 ────────────
doc.add_heading("9. La référence indépendante — recalibration contre le robot, 15/09/2026", level=1)
para("Principe. Le robot sert de règle. La pince est placée À LA MAIN sur un marqueur, le "
     "marqueur au milieu des doigts, et l'on lit SEULEMENT les codeurs, sans aucun ordre "
     "moteur : médiane de cinq lectures. Aucune caméra n'intervient dans la référence.")
para("Mise en œuvre.", gras=True)
puces([
    "Mesurer au robot les marqueurs 19 et 23, les seuls à portée — le bras atteint ~390 mm, "
    "les marqueurs 25 et 26 sont à ~580 mm.",
    "Déduire 25 et 26 de la forme de la planche vue par l'Arducam (distances entre centres "
    "justes à 0,04 %), posée rigidement sur les deux points mesurés au robot. Résidu 3,3 mm.",
    "Écrire cette référence dans planche_actuelle.yaml, puis recalibrer sans forcer : la "
    "validation leave-one-out doit passer seule.",
    "Contrôler en lecture seule, puis par UNE saisie réelle.",
])
table(
    ["référence essayée pour l'ajustement", "pire leave-one-out Arducam", "verdict"],
    [
        ["workspace_markers.yaml (relevé au ruban le 18/08)", "10,9 mm sur le 25", "refusée"],
        ["forme au ruban posée sur 19/23 robot", "9,1 mm sur le 25", "refusée"],
        ["19/23 robot + distances au ruban de l'opérateur (± 1 cm)", "18,6 mm", "refusée"],
        ["forme Arducam posée sur 19/23 robot", "6,6 mm, puis 2,7 mm à la passe suivante", "RETENUE, sans forcer"],
    ],
    "Tableau 10 — Quatre références candidates, trois refusées par le critère leave-one-out. "
    "Le critère a donc effectivement discriminé, ce qui est son rôle.")
para("Calibration retenue pour l'Arducam : leave-one-out de 1,7 / 1,5 / 2,7 / 1,4 mm sur les "
     "marqueurs 19, 23, 25 et 26 ; RMS de reprojection 0,42 px ; contrôle 0,22 mm.")
para("Vérification par la tâche, et non par le résidu.", gras=True)
puces([
    "Balle saisie DU PREMIER COUP, sans aucun correctif de repère : vue en (279,8 ; 14,9) mm, "
    "recalage 2,0 mm, descente à 0,5 mm en XY, fermeture confirmée par le statut de la pince.",
    "Déplacement de caméra éprouvé volontairement à 15:03 : Arducam bougée, déplacement de "
    "10,2 mm détecté, recalibrée à leave-one-out ≤ 1,2 mm, balle à nouveau saisie du premier "
    "coup.",
    "SVPRO recalibrée à 15:20 après inclinaison de la caméra (ses marqueurs bas passaient de "
    "5–8 px à 54–56 px du bord). Leave-one-out 2,4 / 5,9 / 3,7 / 4,2 mm, AU-DESSUS du seuil "
    "de 5 mm : validée par la balle sur deux emplacements (3,3 et 7,9 mm d'écart à l'Arducam ; "
    "hauteur du centre triangulée à 29,8 et 26,6 mm pour 32 attendus), et non par le seul "
    "leave-one-out. L'XY reste donné par l'Arducam, la SVPRO ne corrige que la hauteur.",
])
para("Portée de ce résultat, sans exagération : une saisie réussie du premier coup, répétée "
     "sur deux calibrations indépendantes, prouve que la CAUSE est trouvée. Elle ne constitue "
     "pas une mesure de répétabilité de la saisie, qui reste à faire.", italique=True)

# ──────────── 10. Réserves ────────────
doc.add_heading("10. Réserves de méthode", level=1)
table(
    ["réserve", "portée sur les résultats"],
    [
        ["Répétabilité relevée aux codeurs",
         "borne inférieure ; aveugle au jeu et à la souplesse en aval. Les séries sous la "
         "spécification n'en démontrent pas la conformité."],
        ["Plancher instrumental 0,099 mm",
         "1 LSB de 0,01° sur six axes ; les mesures sont 3 à 8 fois au-dessus, donc non "
         "limitées par la lecture."],
        ["Une seule pose éprouvée",
         "332 mm d'allonge, outil à −15°. La répétabilité varie avec la configuration."],
        ["Une seule portée éprouvée",
         "l'affaissement gravitaire n'a pas été mesuré à plusieurs allonges."],
        ["Essai B en boucle ouverte",
         "contient l'affaissement ; environ 2 mm une fois compensé."],
        ["Position du marqueur posée à l'œil",
         "n'est pas une vérité terrain ; seule sa TAILLE en était une, jusqu'à la mesure au "
         "robot du 15/09."],
        ["Éclairage à 55 de luminance, référence 86",
         "l'extrinsèque du jour plafonne à 0,59 mm au lieu de 0,12 mm."],
        ["Retrait de 40 mm à l'essai G",
         "à 50 mm, le départ « avant » tombe à J3 = −0,58° (bras tendu) et le résidu IK passe "
         "à 0,611 mm : on aurait mesuré une singularité, pas un sens d'approche."],
        ["Extrinsèques ajustées uniquement à Z = 0",
         "validées dans le plan (1,92 mm), sans aucune validité en hauteur."],
        ["Détection ArUco sur une seule trame",
         "jusqu'à 17,5 mm d'erreur par trame MJPEG corrompue."],
        ["Taille d'un marqueur mesurée à l'image",
         "biaisée jusqu'à 4 % par l'obliquité ; la POSITION, elle, reste juste à 0,4 mm."],
        ["Mesure au robot sensible à J6",
         "le bout des doigts est à 22 mm de l'axe J6 dans le modèle ; au marqueur 23, J6 était "
         "tourné de 92° par rapport à la pose commandée."],
    ],
    "Tableau 11 — Ces réserves conditionnent la lecture de chaque chiffre du bilan.")

# ──────────── 11. Conclusions ────────────
doc.add_heading("11. Conclusions", level=1)
para("1. La spécification n'est pas le sujet — le protocole l'est.", gras=True)
para("Le bras tient son chiffre constructeur sur les six séries unidirectionnelles, et c'est "
     "secondaire : le sens d'approche coûte à lui seul 5,85 à 8,09 mm, soit sept fois la pire "
     "répétabilité et six fois la spécification. Dans une tâche de tri réelle, ce terme écrase "
     "tous les autres.")
para("2. L'erreur de la chaîne complète est systématique, donc corrigeable.", gras=True)
para("Le biais chute d'un facteur 20 en deux corrections pendant que la dispersion reste "
     "plate. C'est la justification expérimentale de l'asservissement visuel, et elle ne "
     "repose sur aucune hypothèse de modèle.")
para("3. La précision utile vient de cinq règles opérationnelles, toutes mesurées.", gras=True)
puces([
    "compenser l'affaissement par réinjection de l'écart articulaire : 14,8 mm → ≈ 2 mm ;",
    "approcher toujours dans le même sens : 5,92 mm de biais sinon ;",
    "refuser une cible hors domaine plutôt que de la tenter de travers ;",
    "médianer six trames avant toute commande : une trame isolée peut être fausse de 17,5 mm ;",
    "mesurer la vision sur une TAILLE connue, jamais sur une position posée à l'œil.",
])
para("4. Le résultat de méthode le plus transférable est négatif.", gras=True)
para("Deux conclusions ont dû être retirées, pour la même raison : la vérification repassait "
     "par la grandeur vérifiée. Un résidu d'ajustement faible ne prouve rien quand "
     "l'ajustement et son contrôle partagent leurs données. Les seuls contrôles qui ont tenu "
     "sont ceux qui introduisaient une information indépendante — le leave-one-out, la "
     "validation croisée deux caméras, les longues distances opposées aux petits côtés, et "
     "finalement la mesure au robot.")

# ──────────── 12. Suites ────────────
doc.add_heading("12. Ce qui reste à faire", level=1)
table(
    ["priorité", "action", "ce que cela débloque"],
    [
        ["1", "Mesurer un marqueur au pied à coulisse",
         "l'artefact étalonné qu'exige VDI/VDE 2634-1. Son absence est exactement ce qui a "
         "laissé vivre deux jours l'erreur de la section 8.1."],
        ["2", "Recalibrer avec des marqueurs à plusieurs hauteurs (2–3 niveaux, cale de "
              "hauteur connue)",
         "lève la dégénérescence du plan, rend les extrinsèques valables en volume, et "
         "débloque la question du déport d'outil."],
        ["3", "Essai 3 — précision physique absolue",
         "exige un comparateur à cadran (~60 €). Seul moyen de transformer les bornes "
         "inférieures en mesures."],
        ["4", "ISO 9283 § 7.3 — exactitude et répétabilité de DISTANCE",
         "seule caractéristique normalisée qu'une mesure aux codeurs peut rapporter en valeur "
         "vraie : un décalage constant (affaissement, erreur de modèle, déport faux) s'annule "
         "dans la différence."],
        ["5", "Mesurer la répétabilité de la SAISIE, et non d'un point",
         "le 15/09 n'a prouvé que la cause de l'échec, sur un essai réussi du premier coup à "
         "deux reprises."],
        ["6", "Remonter les scripts du répertoire de session dans le dépôt",
         "sans quoi la plupart des essais ne sont pas rejouables."],
    ],
    "Tableau 12 — Les priorités 1 et 2 répondent chacune à une non-conformité identifiée, pas "
    "à un souhait d'amélioration.")

# ──────────── Annexe ────────────
doc.add_heading("Annexe A — Traçabilité des sources", level=1)
table(
    ["support", "fichier", "rôle", "statut"],
    [
        ["Résultats", "training/calibration/PRECISION_MYCOBOT_320PI.md",
         "les chiffres, leur lecture, les réserves", "versionné"],
        ["Méthodologie", "training/calibration/METHODOLOGIE_PRECISION.md",
         "pourquoi ces essais, contre quelles normes, ce que la méthode ne peut pas mesurer",
         "versionné"],
        ["Protocole", "training/calibration/PROTOCOLE_ESSAIS_PRECISION.md",
         "fiche par essai et table de correspondance", "versionné"],
        ["Correction du 15/09", "docs/PICK_AND_PLACE_REAL.md § « Calibration contre le robot »",
         "la référence mesurée au robot", "versionné"],
        ["Données brutes", "precision_campagne_2026-09-09.xlsx",
         "dix-sept feuilles, une par essai", "non versionné"],
        ["Rapport détaillé", "RAPPORT_COMPLET_PRECISION_320PI.docx",
         "dix-sept sections, trente-cinq tableaux", "non versionné"],
    ],
    "Tableau 13 — En cas de divergence entre un support versionné et un support non "
    "versionné, le fichier .md fait foi.")
para("Scripts rejouables actuellement commités : scripts/calibrer_extrinseque_4aruco.py "
     "(essai H), scripts/aruco_check.py (contrôle de détection), "
     "scripts/calibration_extrinseque_auto.py et scripts/calibrate_camera_base_extrinsic.py "
     "(recalibration du 15/09). Les autres scripts d'essai sont restés dans le répertoire de "
     "session et n'y survivent pas — voir priorité 6.")
para("Généré par training/calibration/generate_bilan_docx.py — ne pas éditer le .docx à la "
     "main : éditer le script et le rejouer.", italique=True, gris=True)

doc.save(OUT)
print(f"écrit : {OUT}")
print(f"  {len(doc.paragraphs)} paragraphes, {len(doc.tables)} tableaux")
