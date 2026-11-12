#!/usr/bin/env python3
"""Écart de PCK entre modèles évalués sur les mêmes images, avec IC bootstrap apparié.

    python training/dream/comparer_pck.py REFERENCE AUTRE [AUTRE ...] [--bloc 10]

Chaque argument est un dossier de sortie d'`eval_papier.py` (`erreurs.npy`,
`dans_image.npy`) sur le même jeu, dans le même ordre d'images. Le tirage
bootstrap est commun à tous les modèles (apparié) et se fait par blocs
d'images consécutives : un lot de distracteurs dure 40 trames et les vues d'une
pose se suivent, des images voisines ne sont donc pas indépendantes. PCK au
sens du papier, non détecté = faux, seuil de pic du code (0,01).

Critère de E3.8 (docs/ARTICLE_DREAM.md) : `--critere` l'applique au premier
AUTRE contre REFERENCE, puis au suivant s'il échoue.
"""
import argparse
import json
from pathlib import Path

import numpy as np

SEUILS_PX = (2.5, 5.0, 10.0)
GAIN_MIN_PCK25 = 0.05
PERTE_MAX = 0.01


def charger(dossier):
    return np.load(dossier / "erreurs.npy"), np.load(dossier / "dans_image.npy")


def mesures(erreurs, dans_image, images):
    e, d = erreurs[images], dans_image[images]
    juste = np.nan_to_num(e, nan=np.inf)
    m = {f"pck@{s:g}": float((juste < s)[d].mean()) for s in SEUILS_PX}
    m["detection"] = float((~np.isnan(e))[d].mean())
    return m


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("reference", type=Path)
    ap.add_argument("autres", type=Path, nargs="+")
    ap.add_argument("--bloc", type=int, default=10, help="images consécutives par bloc (défaut 10)")
    ap.add_argument("--tirages", type=int, default=2000)
    ap.add_argument("--graine", type=int, default=0)
    ap.add_argument("--critere", action="store_true", help="appliquer la règle de choix de E3.8")
    ap.add_argument("--json", type=Path, help="écrire le bilan ici")
    args = ap.parse_args()

    modeles = {d.name: charger(d) for d in [args.reference, *args.autres]}
    n = {e.shape[0] for e, _ in modeles.values()}
    if len(n) != 1:
        raise SystemExit(f"jeux de tailles différentes : {sorted(n)} images")
    n = n.pop()
    blocs = [np.arange(i, min(i + args.bloc, n)) for i in range(0, n, args.bloc)]
    hasard = np.random.default_rng(args.graine)
    tirages = [np.concatenate([blocs[i] for i in hasard.integers(0, len(blocs), len(blocs))])
               for _ in range(args.tirages)]
    tout = np.arange(n)

    ref = args.reference.name
    bilan = {"images": n, "bloc": args.bloc, "tirages": args.tirages, "reference": ref, "modeles": {}}
    print(f"{n} images, blocs de {args.bloc}, {args.tirages} tirages ; IC 95 % des écarts à {ref}")
    for nom, (e, d) in modeles.items():
        m = mesures(e, d, tout)
        ligne = {"valeurs": m}
        if nom != ref:
            boot = {k: [] for k in m}
            for t in tirages:
                a, b = mesures(e, d, t), mesures(*modeles[ref], t)
                for k in m:
                    boot[k].append(a[k] - b[k])
            ref_m = mesures(*modeles[ref], tout)
            ligne["ecarts"] = {k: {"ecart": m[k] - ref_m[k],
                                   "ic95": [float(np.percentile(boot[k], 2.5)),
                                            float(np.percentile(boot[k], 97.5))]} for k in m}
        bilan["modeles"][nom] = ligne
        texte = "  ".join(f"{k} {v:.3f}" for k, v in m.items())
        if "ecarts" in ligne:
            texte += "\n" + " " * 4 + "  ".join(
                f"Δ{k} {x['ecart']:+.3f} [{x['ic95'][0]:+.3f} ; {x['ic95'][1]:+.3f}]"
                for k, x in ligne["ecarts"].items())
        print(f"{nom}\n    {texte}")

    if args.critere:
        retenu = ref
        for d in args.autres:
            x = bilan["modeles"][d.name]["ecarts"]
            gagne = x["pck@2.5"]["ecart"] >= GAIN_MIN_PCK25 and x["pck@2.5"]["ic95"][0] > 0
            sans_perte = all(x[k]["ecart"] >= -PERTE_MAX for k in ("pck@5", "pck@10", "detection"))
            print(f"critère {d.name} : gain PCK@2,5 {'oui' if gagne else 'non'}, "
                  f"sans perte {'oui' if sans_perte else 'non'}")
            if gagne and sans_perte:
                retenu = d.name
                break
        bilan["retenu"] = retenu
        print(f"retenu : {retenu}")
    if args.json:
        args.json.write_text(json.dumps(bilan, indent=1, ensure_ascii=False))


if __name__ == "__main__":
    main()
