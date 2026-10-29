#!/usr/bin/env python3
"""PCK, AUC et ADD au sens de Lee et al. 2020 (DREAM, arXiv:1911.09231, § III.B).

Différence avec evaluate_dream.py : un keypoint dont la projection de référence
est dans l'image compte **faux s'il n'est pas détecté**. evaluate_dream.py
calcule ses « % < N px » sur les détectés seulement, ce qui surestime la PCK
(98,1 % contre 94,1 % à 5 px sur real_tri_test, 29/10/2026).

Seuil de pic : le code NVlabs filtre à 0,01 après lissage σ = 3
(~/DREAM/dream/image_proc.py:925), le texte du papier annonce 0,03. Les deux
sont rendus : 0,03 est appliqué après coup, sur la carte lissée au point
détecté, sans modifier ~/DREAM.

ADD : PnP (EPnP puis raffinement, dream.geometric_vision.solve_pnp) sur les
keypoints détectés, distance 3D moyenne sur tous les keypoints. link2 est
retiré du PnP et de l'ADD : c'est le même point 3D que link1, il compterait
deux fois dans le minimum de 4 points. Une image où la vérité terrain met au
moins 4 points dans l'image et où le PnP échoue compte en échec, comme
dream.analysis.pnp_metrics ; l'AUC de l'ADD va de 0 à 100 mm.

Intrinsèques : celles de _camera_settings.json, sauf pour un label qui porte
source.camera (jeux réels de convert_real_tri_ndds.py, deux caméras dans un
même dossier) : on prend alors celles de la caméra, distorsion comprise,
comme pour la génération des labels.

Usage (venv_dream) :
    python eval_papier.py --weights .../best_network.pth --data <jeu_ndds> \\
        --sortie <dossier> [--split-val] [--max-images N] [--sans-link2]
    python eval_papier.py --csv keypoints_<cam>.csv --sortie <dossier> \\
        [--j5 MIN MAX]   # détections déjà calculées (dashboard réel) : PCK seule

Sorties : resume.json, erreurs.npy (px, NaN = non détecté), dans_image.npy,
score_lisse.npy, add_m.npy (NaN = PnP impossible), pour recalcul.
"""
import argparse
import csv
import glob
import json
import math
import os
import sys
from pathlib import Path

import numpy as np
from scipy.ndimage import gaussian_filter

RACINE = Path(__file__).resolve().parents[2]
SEUILS_PX = (2.5, 5.0, 10.0)
AUC_MAX_PX = 20.0
SEUILS_PIC = (0.01, 0.03)
SIGMA_PIC = 3
ADD_AUC_MAX_M = 0.1
ADD_SEUIL_CRITERE_M = 0.02
MIN_POINTS_PNP = 4
INDEX_LINK2 = 2
# Même correspondance que les extrinsèques de convert_real_tri_ndds.py
# (intrinsics_stem de arducam_extrinsic_pick.yaml et svpro_extrinsic_labels.yaml).
INTRINSEQUES_PAR_CAMERA = {"arducam": "cam_3", "svpro": "cam_2"}


def pck(erreurs, dans_image, seuil):
    juste = np.nan_to_num(erreurs, nan=np.inf) < seuil
    return float(juste[dans_image].mean())


def auc(erreurs, dans_image):
    seuils = np.linspace(0.0, AUC_MAX_PX, 201)
    courbe = [pck(erreurs, dans_image, s) for s in seuils]
    return float(np.trapezoid(courbe, seuils) / AUC_MAX_PX)


def bilan_pck(erreurs, dans_image, noms):
    return {
        "keypoints_dans_image": int(dans_image.sum()),
        "detection": float((~np.isnan(erreurs))[dans_image].mean()),
        "pck": {f"{s:g}": pck(erreurs, dans_image, s) for s in SEUILS_PX},
        "auc_0_20px": auc(erreurs, dans_image),
        "par_keypoint": {
            nom: {f"{s:g}": pck(erreurs[:, [i]], dans_image[:, [i]], s) for s in SEUILS_PX}
            for i, nom in enumerate(noms)
        },
    }


def bilan_add(add, pnp_possible):
    trouves = add[pnp_possible & ~np.isnan(add)]
    n_possibles = int(pnp_possible.sum())
    seuils = np.linspace(0.0, ADD_AUC_MAX_M, 1001)
    courbe = [(trouves <= s).sum() / n_possibles for s in seuils]
    return {
        "images_pnp_possible": n_possibles,
        "pnp_reussi": float(len(trouves) / n_possibles),
        "mediane_mm_sur_reussis": float(np.median(trouves) * 1000) if len(trouves) else None,
        "auc_0_100mm": float(np.trapezoid(courbe, seuils) / ADD_AUC_MAX_M),
        f"part_sous_{ADD_SEUIL_CRITERE_M * 1000:g}mm": float(
            (trouves <= ADD_SEUIL_CRITERE_M).sum() / n_possibles),
    }


def intrinseques(dossier, label, cache):
    camera = label.get("source", {}).get("camera")
    if camera not in cache:
        if camera is None:
            reglages = json.load(open(os.path.join(dossier, "_camera_settings.json")))
            r = reglages["camera_settings"][0]["intrinsic_settings"]
            K = np.array([[r["fx"], 0, r["cx"]], [0, r["fy"], r["cy"]], [0, 0, 1]], float)
            cache[camera] = (K, np.array([]))
        else:
            sys.path.insert(0, str(RACINE / "mycobot_gateway" / "mycobot_gateway" / "vision"))
            import camera_registry
            cache[camera] = camera_registry.load_intrinsics(INTRINSEQUES_PAR_CAMERA[camera])
    return cache[camera]


def score_lisse(carte, u, v):
    lisse = gaussian_filter(carte, sigma=SIGMA_PIC)
    i, j = int(round(v)), int(round(u))
    return float(lisse[max(i - 1, 0):i + 2, max(j - 1, 0):j + 2].max())


def add_image(dream, points_3d, detections, valides, K, dist):
    garde = [i for i in range(len(points_3d)) if i != INDEX_LINK2]
    utilises = [i for i in garde if valides[i]]
    if len(utilises) < MIN_POINTS_PNP:
        return np.nan
    ok, translation, quaternion = dream.geometric_vision.solve_pnp(
        points_3d[utilises], detections[utilises], K, dist_coeffs=dist)
    if not ok:
        return np.nan
    return float(dream.geometric_vision.add_from_pose(translation, quaternion, points_3d[garde], K))


def evaluer_jeu(reseau, dream, fichiers, dossier):
    import torch
    from PIL import Image
    erreurs, dans_image, scores, add, pnp_possible = [], [], [], {s: [] for s in SEUILS_PIC}, []
    cache_K = {}
    for chemin in fichiers:
        label = json.load(open(chemin))
        image = Image.open(chemin.replace(".json", ".rgb.png")).convert("RGB")
        kps = label["objects"][0]["keypoints"]
        references = np.array([k["projected_location"] for k in kps], float)
        points_3d = np.array([k["location"] for k in kps], float)
        with torch.no_grad():
            sortie = reseau.keypoints_from_image(image, debug=True)
        detections = np.array(sortie["detected_keypoints"], float)
        reseau_out = np.array(sortie["detected_keypoints_net_output"], float)
        cartes = sortie["belief_maps"].cpu().numpy()

        visible = (references[:, 0] >= 0) & (references[:, 0] < image.size[0]) \
            & (references[:, 1] >= 0) & (references[:, 1] < image.size[1])
        detecte = np.isfinite(detections).all(axis=1) & (detections > -900).all(axis=1)
        ligne = np.where(detecte, np.hypot(*(detections - references).T), np.nan)
        score = np.array([score_lisse(cartes[i], *reseau_out[i]) if detecte[i] else np.nan
                          for i in range(len(kps))])
        K, dist = intrinseques(dossier, label, cache_K)
        distincts = [i for i in range(len(kps)) if i != INDEX_LINK2]
        pnp_possible.append(visible[distincts].sum() >= MIN_POINTS_PNP)
        for s in SEUILS_PIC:
            add[s].append(add_image(dream, points_3d, detections,
                                    detecte & (np.nan_to_num(score) >= s), K, dist))
        erreurs.append(ligne)
        dans_image.append(visible)
        scores.append(score)
    return (np.array(erreurs), np.array(dans_image, bool), np.array(scores),
            {s: np.array(v, float) for s, v in add.items()}, np.array(pnp_possible, bool))


def lire_csv(chemin, j5):
    lignes = list(csv.DictReader(open(chemin)))
    noms = [c[:-len("_u_ref")] for c in lignes[0] if c.endswith("_u_ref")]
    if j5:
        lignes = [r for r in lignes if j5[0] <= float(r["q5"]) <= j5[1]]
    erreurs, dans_image = [], []
    for r in lignes:
        e, vis = [], []
        for n in noms:
            u_ref, v_ref = float(r[f"{n}_u_ref"]), float(r[f"{n}_v_ref"])
            vis.append(0 <= u_ref < 640 and 0 <= v_ref < 480)
            e.append(math.hypot(float(r[f"{n}_u"]) - u_ref, float(r[f"{n}_v"]) - v_ref)
                     if r[f"{n}_u"] else np.nan)
        erreurs.append(e)
        dans_image.append(vis)
    return np.array(erreurs, float), np.array(dans_image, bool), noms


def main():
    p = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    source = p.add_mutually_exclusive_group(required=True)
    source.add_argument("--data", help="dossier NDDS (??????.json + .rgb.png)")
    source.add_argument("--csv", help="keypoints_<cam>.csv d'une session du dashboard réel")
    p.add_argument("--weights", help="requis avec --data")
    p.add_argument("--sortie", required=True)
    p.add_argument("--split-val", action="store_true",
                   help="garde les 20 %% finaux, comme evaluate_dream.py --split val")
    p.add_argument("--max-images", type=int, help="sous-échantillon régulier")
    p.add_argument("--j5", nargs=2, type=float, metavar=("MIN", "MAX"),
                   help="avec --csv : garde les lignes dont J5 est dans [MIN, MAX] degrés")
    p.add_argument("--sans-link2", action="store_true",
                   help="retire link2 de la PCK, même point 3D que link1")
    args = p.parse_args()
    sortie = Path(args.sortie)
    sortie.mkdir(parents=True, exist_ok=True)

    if args.csv:
        erreurs, dans_image, noms = lire_csv(args.csv, args.j5)
        bilan = {"images": int(erreurs.shape[0]), "csv": args.csv, "j5": args.j5,
                 "seuil_pic": "celui de la session (0,01, code NVlabs)",
                 "seuil_0.01": bilan_pck(*retirer_link2(erreurs, dans_image, noms, args.sans_link2))}
    else:
        if not args.weights:
            p.error("--weights est requis avec --data")
        sys.path.insert(0, os.environ.get("DREAM_DIR", os.path.expanduser("~/DREAM")))
        import dream
        reseau = dream.create_network_from_config_file(
            os.path.splitext(args.weights)[0] + ".yaml", args.weights)
        reseau.enable_evaluation()
        noms = list(reseau.keypoint_names)
        fichiers = sorted(glob.glob(os.path.join(args.data, "??????.json")))
        if args.split_val:
            fichiers = fichiers[int(len(fichiers) * 0.8):]
        if args.max_images and len(fichiers) > args.max_images:
            pas = len(fichiers) / args.max_images
            fichiers = [fichiers[int(i * pas)] for i in range(args.max_images)]

        erreurs, dans_image, scores, add, pnp_possible = evaluer_jeu(reseau, dream, fichiers, args.data)
        np.save(sortie / "score_lisse.npy", scores)
        np.save(sortie / "add_m.npy", np.stack([add[s] for s in SEUILS_PIC], axis=1))
        np.save(sortie / "pnp_possible.npy", pnp_possible)
        bilan = {"images": len(fichiers), "weights": args.weights, "data": args.data,
                 "split_val": args.split_val, "max_images": args.max_images}
        for s in SEUILS_PIC:
            masque = np.where(np.nan_to_num(scores) >= s, erreurs, np.nan)
            bilan[f"seuil_{s:g}"] = bilan_pck(*retirer_link2(masque, dans_image, noms, args.sans_link2))
            bilan[f"seuil_{s:g}"]["add"] = bilan_add(add[s], pnp_possible)

    np.save(sortie / "erreurs.npy", erreurs)
    np.save(sortie / "dans_image.npy", dans_image)
    bilan["sans_link2"] = args.sans_link2
    (sortie / "resume.json").write_text(json.dumps(bilan, indent=2, ensure_ascii=False))
    print(json.dumps({k: ({c: v[c] for c in ("detection", "pck", "auc_0_20px", "add") if c in v}
                          if k.startswith("seuil_") else v) for k, v in bilan.items()},
                     indent=2, ensure_ascii=False))


def retirer_link2(erreurs, dans_image, noms, retirer):
    if not retirer:
        return erreurs, dans_image, noms
    return (np.delete(erreurs, INDEX_LINK2, axis=1), np.delete(dans_image, INDEX_LINK2, axis=1),
            [n for i, n in enumerate(noms) if i != INDEX_LINK2])


if __name__ == "__main__":
    main()
