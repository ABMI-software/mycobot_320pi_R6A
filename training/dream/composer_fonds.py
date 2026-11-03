#!/usr/bin/env python3
"""Fonds aléatoires derrière le robot, sur une collecte caméra libre (E3.3).

Lee et al. 2020 (§ II.C) entraînent DREAM sur des rendus dont le fond est une
image COCO tirée au hasard. E3.0 a montré que la diversité de caméra seule ne
transfère pas au banc réel (PCK@5 0,014 sur real_tri_test) : notre décor
Gazebo est toujours le même (table, sol, trois objets).

Entrée : un dossier de `synthetic_data_collector_v3_camera_libre`
(`images/libre/`, `images/masque/`, `masques.csv`, `labels.csv`,
`camera_poses.csv`). Sortie : un dossier jumeau dont `images/libre/` porte,
pour une part `--part` des trames à masque frais, le robot (bras et pince)
posé sur une image de fond ; les autres trames sont des liens vers l'original.
`labels.csv` et `camera_poses.csv` sont copiés tels quels : la géométrie ne
change pas, `convert_to_ndds_camera_libre.py` s'applique ensuite sans
modification. `composition.csv` dit, par trame, si elle a été composée et sur
quel fond.

Les ombres du robot sur la table disparaissent avec elle : c'est aussi le cas
dans le papier.

Usage (venv_dream) :
    python training/dream/composer_fonds.py --collecte <dossier> \\
        --fonds ~/dream_article/fonds/coco/val2017 --sortie <dossier> \\
        [--part 0.5] [--graine 0]
"""
import argparse
import csv
import os
import random
import shutil
from pathlib import Path

import numpy as np
from PIL import Image, ImageFilter

LARGEUR, HAUTEUR = 640, 480
# Les bords du robot sont lissés par le rendu : un masque dur garderait un
# liseré du décor Gazebo. Un adoucissement d'un pixel le fond dans le fond.
FLOU_BORD_PX = 0.8
ECHELLE_MIN = 1.0
ECHELLE_MAX = 1.6


def fond_aleatoire(chemin, hasard):
    """Image de fond recadrée au hasard à 640x480, retournée une fois sur deux."""
    fond = Image.open(chemin).convert("RGB")
    echelle = max(LARGEUR / fond.width, HAUTEUR / fond.height) * hasard.uniform(ECHELLE_MIN, ECHELLE_MAX)
    fond = fond.resize((max(LARGEUR, round(fond.width * echelle)),
                        max(HAUTEUR, round(fond.height * echelle))), Image.BILINEAR)
    x = hasard.randint(0, fond.width - LARGEUR)
    y = hasard.randint(0, fond.height - HAUTEUR)
    fond = fond.crop((x, y, x + LARGEUR, y + HAUTEUR))
    if hasard.random() < 0.5:
        fond = fond.transpose(Image.FLIP_LEFT_RIGHT)
    return fond


def composer(rendu, masque, fond):
    alpha = np.asarray(masque.filter(ImageFilter.GaussianBlur(FLOU_BORD_PX)), np.float32)[..., None] / 255.0
    image = np.asarray(rendu, np.float32) * alpha + np.asarray(fond, np.float32) * (1.0 - alpha)
    return Image.fromarray(image.round().astype(np.uint8))


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--collecte", type=Path, required=True)
    p.add_argument("--fonds", type=Path, required=True, help="dossier d'images (COCO val2017)")
    p.add_argument("--sortie", type=Path, required=True)
    p.add_argument("--part", type=float, default=0.5,
                   help="part des trames à masque frais qui reçoivent un fond (défaut 0,5)")
    p.add_argument("--graine", type=int, default=0)
    args = p.parse_args()

    if args.sortie.exists() and any(args.sortie.iterdir()):
        raise SystemExit(f"{args.sortie} n'est pas vide")
    hasard = random.Random(args.graine)
    fonds = sorted(str(f) for f in args.fonds.iterdir() if f.suffix.lower() in (".jpg", ".jpeg", ".png"))
    if not fonds:
        raise SystemExit(f"aucune image dans {args.fonds}")

    (args.sortie / "images" / "libre").mkdir(parents=True)
    for nom in ("labels.csv", "camera_poses.csv", "masques.csv"):
        shutil.copy2(args.collecte / nom, args.sortie / nom)
    frais = {int(r["index"]) for r in csv.DictReader(open(args.collecte / "masques.csv")) if r["frais"] == "1"}

    n_composees = 0
    with open(args.sortie / "composition.csv", "w", newline="") as f:
        sortie_csv = csv.writer(f)
        sortie_csv.writerow(["index", "composee", "fond"])
        for rendu_chemin in sorted((args.collecte / "images" / "libre").glob("*.png")):
            index = int(rendu_chemin.stem)
            cible = args.sortie / "images" / "libre" / rendu_chemin.name
            if index in frais and hasard.random() < args.part:
                chemin_fond = hasard.choice(fonds)
                fond = fond_aleatoire(chemin_fond, hasard)
                masque = Image.open(args.collecte / "images" / "masque" / rendu_chemin.name).convert("L")
                composer(Image.open(rendu_chemin).convert("RGB"), masque, fond).save(cible)
                sortie_csv.writerow([index, 1, os.path.basename(chemin_fond)])
                n_composees += 1
            else:
                os.symlink(rendu_chemin.resolve(), cible)
                sortie_csv.writerow([index, 0, ""])
    print(f"{n_composees} trames composées sur {len(frais)} à masque frais -> {args.sortie}")


if __name__ == "__main__":
    main()
