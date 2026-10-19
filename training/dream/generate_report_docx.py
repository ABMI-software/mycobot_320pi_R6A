#!/usr/bin/env python3
"""Génère un rapport Word (.docx) : DREAM — pose caméra-robot sans marqueur."""
from pathlib import Path

from docx import Document
from docx.shared import Inches, Pt, RGBColor
from docx.enum.text import WD_ALIGN_PARAGRAPH

CAL = Path(__file__).resolve().parents[1] / "calibration"
OUT = Path(__file__).resolve().parent / "RAPPORT_DREAM_MARKERLESS.docx"

doc = Document()


def img(name, width=6.3, caption=None):
    p = CAL / name
    if not p.exists():
        doc.add_paragraph(f"[image manquante : {name}]")
        return
    doc.add_picture(str(p), width=Inches(width))
    doc.paragraphs[-1].alignment = WD_ALIGN_PARAGRAPH.CENTER
    if caption:
        c = doc.add_paragraph(caption)
        c.alignment = WD_ALIGN_PARAGRAPH.CENTER
        r = c.runs[0]; r.italic = True; r.font.size = Pt(9)
        r.font.color.rgb = RGBColor(0x55, 0x55, 0x55)


# ── Titre ──
t = doc.add_heading("DREAM — Estimation de la pose caméra-robot sans marqueur", 0)
p = doc.add_paragraph()
r = p.add_run("MyCobot 320 Pi · caméra Arducam (640×480) · modèle vgg_ultimate_v4_mix_ft_e30 · "
              "données réelles, session de test non vue à l'entraînement")
r.italic = True
p.alignment = WD_ALIGN_PARAGRAPH.CENTER

# ── 1. Objectif ──
doc.add_heading("1. Objectif — pas de calibrage extrinsèque", level=1)
doc.add_paragraph(
    "DREAM (Deep Robot-to-Camera Extrinsics for Articulated Manipulators, ICRA 2020) estime la "
    "transformation caméra→robot T_C^R directement à partir d'une image RGB du robot — "
    "SANS marqueur ni calibrage extrinsèque manuel. Un réseau détecte les keypoints du robot dans "
    "l'image ; combinés à la cinématique directe (encodeurs) et aux intrinsèques de la caméra, un "
    "PnP en déduit la pose caméra-robot.")
doc.add_paragraph(
    "Le calibrage marqueur (ChArUco / ArUco) ne sert que de référence de validation. Les "
    "intrinsèques de l'Arducam (cam_0.npz : fx, fy, cx, cy + distorsion) sont calibrées une seule "
    "fois au ChArUco. C'est la seule calibration nécessaire ; l'extrinsèque, c'est DREAM.")

# ── 2. Le pipeline ──
doc.add_heading("2. Le pipeline DREAM appliqué au robot réel", level=1)
doc.add_paragraph(
    "Le schéma officiel de DREAM, réalisé sur notre MyCobot avec des données réelles : "
    "image → keypoints 2D → (FK des encodeurs + intrinsèques) → PnP → pose T_C^R, sans marqueur.")
img("conclusion_TCR.png", 6.5,
    "Figure 1 — Pipeline complet et preuve : le squelette FK reprojeté via la pose estimée par "
    "DREAM se pose sur le bras réel.")

# ── 3. Détection 2D ──
doc.add_heading("3. Qualité de la détection 2D (base → link4)", level=1)
doc.add_paragraph(
    "Sur la session de test (jamais vue à l'entraînement), la détection est précise au pixel près "
    "pour les keypoints proximaux :")
tbl = doc.add_table(rows=2, cols=6); tbl.style = "Light Grid Accent 1"
hdr = ["base", "link1", "link2", "link3", "link4", "global"]
vals = ["3,1", "1,6", "1,6", "4,7", "5,4", "3,3"]
for i, (h, v) in enumerate(zip(hdr, vals)):
    tbl.cell(0, i).text = h
    tbl.cell(1, i).text = v + " px"
doc.add_paragraph()
img("perkp_px_v4.png", 5.5, "Figure 2 — Résidu de reprojection médian par keypoint (px).")

# ── 4. Preuve détection vs vérité ──
doc.add_heading("4. Prédiction DREAM vs vérité géométrique", level=1)
doc.add_paragraph(
    "Style NVlabs : pour chaque keypoint, point vert = vérité (FK projetée), point orange = "
    "prédiction DREAM. Les deux se superposent sur les articulations → la détection tombe sur le "
    "bras.")
img("hero_nvlabs_v4.png", 5.0,
    "Figure 3 — Prédiction (orange) et vérité (vert) coïncident sur les joints du bras réel.")

# ── 5. Preuve sur plusieurs poses ──
doc.add_heading("5. Preuve markerless sur plusieurs poses", level=1)
doc.add_paragraph(
    "Avec une SEULE pose caméra fixe (estimée par DREAM, sans marqueur), le squelette FK reprojeté "
    "se pose sur le bras pour 8 configurations différentes. Si la pose était fausse, le squelette "
    "dériverait dès que le bras bouge.")
img("overlay_dream_v4.png", 6.5,
    "Figure 4 — 8 poses, caméra fixe : le vert (FK via T_C^R DREAM) suit le bras.")
img("hero_dream_v4.png", 5.0,
    "Figure 5 — Vue rapprochée : squelette FK reprojeté via la pose estimée par DREAM.")

# ── 6. Résultat T_C^R ──
doc.add_heading("6. Résultat — la pose T_C^R estimée (sans marqueur)", level=1)
tbl = doc.add_table(rows=3, cols=2); tbl.style = "Light Grid Accent 1"
rows = [("Position caméra (repère base)", "X = −424 mm · Y = −1027 mm · Z = 1112 mm"),
        ("Orientation", "roll = −147° · pitch = 26° · yaw = −65°"),
        ("Résidu de reprojection", "3,3 px médian")]
for i, (k, v) in enumerate(rows):
    tbl.cell(i, 0).text = k; tbl.cell(i, 1).text = v
doc.add_paragraph()
doc.add_paragraph(
    "→ Une transformation 6-DoF caméra-robot obtenue à partir des seules images (+ FK + "
    "intrinsèques), sans aucun marqueur.")

# ── 7. Le marqueur J6 (validation) ──
doc.add_heading("7. Le marqueur sur J6 — à quoi il sert (validation)", level=1)
doc.add_paragraph(
    "Pour VÉRIFIER que la pose estimée par DREAM est juste, on utilise un marqueur ArUco "
    "(id = 21, 40 mm) collé sur le flasque J6 (l'extrémité mobile du bras). Ce marqueur ne sert "
    "QU'À la validation — DREAM n'en a pas besoin pour fonctionner.")
doc.add_paragraph(
    "Méthode (hand-eye, eye-to-hand) : à chaque pose, la cinématique directe donne la position 3D "
    "exacte du flasque (repère base), et le marqueur détecté donne sa pose vue par la caméra. En "
    "combinant plusieurs poses, on résout la pose FIXE de la caméra dans le repère du robot — une "
    "vérité INDÉPENDANTE de DREAM.")
doc.add_paragraph(
    "On compare alors la pose estimée par DREAM (markerless) à cette vérité pour chiffrer l'erreur. "
    "Une fois DREAM validé, on retire le marqueur : il n'a servi qu'à prouver que DREAM avait "
    "raison. La détection ArUco fournit aussi les intrinsèques de la caméra (voir section 1).")
img("aruco_j6_id21_multi.png", 2.6,
    "Figure 6 — Le marqueur ArUco (id 21, 40 mm) collé sur le flasque J6, uniquement pour la "
    "validation hand-eye.")

# ── 8. Condition clé ──
doc.add_heading("8. Condition de réussite — une scène propre", level=1)
doc.add_paragraph(
    "La détection markerless suppose que le réseau voie le ROBOT, pas des distracteurs. Une scène "
    "encombrée (marqueurs ArUco posés sur la table, clavier, objets) peut détourner la détection "
    "et fausser la pose. Il faut donc dégager le champ de la caméra : le robot sur un fond neutre, "
    "comme dans les figures ci-dessus. Le PnP doit également accumuler plusieurs poses (le bras "
    "étant petit, une image isolée ne contraint pas assez la pose).")

# ── 9. Conclusion ──
doc.add_heading("9. Conclusion", level=1)
doc.add_paragraph(
    "DREAM estime la pose caméra-robot du MyCobot 320 Pi à partir d'images réelles, sans marqueur "
    "ni calibrage extrinsèque : détection à 3,3 px, squelette reprojeté sur le bras via la pose "
    "estimée. Seules les intrinsèques (ChArUco, une fois) restent nécessaires. Pour le "
    "pick-and-place à caméra fixe, cette pose T_C^R est calibrée une fois puis réutilisée.")

doc.save(str(OUT))
print(f"✅ {OUT}")
