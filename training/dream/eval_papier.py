#!/usr/bin/env python3
"""PCK au sens de Lee et al. 2020 (DREAM, arXiv:1911.09231, § III.B).

Différence avec evaluate_dream.py : un keypoint dont la projection de référence
est dans l'image compte **faux s'il n'est pas détecté**. evaluate_dream.py
calcule ses « % < N px » sur les détectés seulement, ce qui surestime la PCK
(98,1 % contre 94,1 % à 5 px sur real_tri_test, 29/10/2026).

Seuil de pic : celui du code NVlabs (0,01 après lissage σ = 3,
~/DREAM/dream/image_proc.py:925). Le texte du papier annonce 0,03.

Usage (venv_dream) :
    python eval_papier.py --weights .../best_network.pth --data <jeu_ndds> \\
        --sortie <dossier> [--sans-link2]

Sorties : resume.json (PCK par seuil, AUC 0-20 px, par keypoint), erreurs.npy
(px, NaN = non détecté) et dans_image.npy, pour recalcul.
"""
import argparse
import glob
import json
import math
import os
import sys
from pathlib import Path

import numpy as np
import torch
from PIL import Image

DREAM_DIR = os.environ.get("DREAM_DIR", os.path.expanduser("~/DREAM"))
sys.path.insert(0, DREAM_DIR)
import dream  # noqa: E402

SEUILS_PX = (2.5, 5.0, 10.0)
AUC_MAX_PX = 20.0
INDEX_LINK2 = 2


def erreurs_jeu(reseau, dossier):
    erreurs, dans_image = [], []
    for chemin in sorted(glob.glob(os.path.join(dossier, "??????.json"))):
        image = Image.open(chemin.replace(".json", ".rgb.png")).convert("RGB")
        references = [k["projected_location"]
                      for k in json.load(open(chemin))["objects"][0]["keypoints"]]
        with torch.no_grad():
            detections = reseau.keypoints_from_image(image)["detected_keypoints"]
        ligne, visible = [], []
        for i, (u, v) in enumerate(references):
            visible.append(0 <= u < image.size[0] and 0 <= v < image.size[1])
            d = detections[i] if i < len(detections) else None
            if d is None or d[0] is None or d[0] < -900 or d[1] < -900:
                ligne.append(np.nan)
            else:
                ligne.append(math.hypot(d[0] - u, d[1] - v))
        erreurs.append(ligne)
        dans_image.append(visible)
    return np.array(erreurs, dtype=float), np.array(dans_image, dtype=bool)


def pck(erreurs, dans_image, seuil):
    juste = np.nan_to_num(erreurs, nan=np.inf) < seuil
    return float(juste[dans_image].mean())


def auc(erreurs, dans_image):
    seuils = np.linspace(0.0, AUC_MAX_PX, 201)
    courbe = [pck(erreurs, dans_image, s) for s in seuils]
    return float(np.trapezoid(courbe, seuils) / AUC_MAX_PX)


def resume(erreurs, dans_image, noms):
    return {
        "images": int(erreurs.shape[0]),
        "keypoints_dans_image": int(dans_image.sum()),
        "detection": float((~np.isnan(erreurs))[dans_image].mean()),
        "pck": {f"{s:g}": pck(erreurs, dans_image, s) for s in SEUILS_PX},
        "auc_0_20px": auc(erreurs, dans_image),
        "par_keypoint": {
            nom: {f"{s:g}": pck(erreurs[:, [i]], dans_image[:, [i]], s) for s in SEUILS_PX}
            for i, nom in enumerate(noms)
        },
    }


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    p.add_argument("--weights", required=True)
    p.add_argument("--data", required=True, help="dossier NDDS (??????.json + .rgb.png)")
    p.add_argument("--sortie", required=True)
    p.add_argument("--sans-link2", action="store_true",
                   help="retire link2, même point 3D que link1")
    args = p.parse_args()

    reseau = dream.create_network_from_config_file(
        os.path.splitext(args.weights)[0] + ".yaml", args.weights)
    reseau.enable_evaluation()
    noms = list(reseau.keypoint_names)

    erreurs, dans_image = erreurs_jeu(reseau, args.data)
    if args.sans_link2:
        erreurs = np.delete(erreurs, INDEX_LINK2, axis=1)
        dans_image = np.delete(dans_image, INDEX_LINK2, axis=1)
        del noms[INDEX_LINK2]

    sortie = Path(args.sortie)
    sortie.mkdir(parents=True, exist_ok=True)
    np.save(sortie / "erreurs.npy", erreurs)
    np.save(sortie / "dans_image.npy", dans_image)
    bilan = resume(erreurs, dans_image, noms)
    bilan.update(weights=args.weights, data=args.data, sans_link2=args.sans_link2,
                 seuil_pic="0,01 (code NVlabs)")
    (sortie / "resume.json").write_text(json.dumps(bilan, indent=2, ensure_ascii=False))
    print(json.dumps({k: bilan[k] for k in ("images", "detection", "pck", "auc_0_20px")},
                     indent=2))


if __name__ == "__main__":
    main()
