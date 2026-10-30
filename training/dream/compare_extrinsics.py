#!/usr/bin/env python3
"""Compare deux extrinsèques (yaml) -> écart de position (mm) et d'orientation (deg).

Sert à chiffrer la précision de la pose estimée par DREAM contre une référence
INDÉPENDANTE (marqueur hand-eye), capturées dans la MÊME session (caméra non bougée).
Le Δ orientation est le chiffre que l'encadrant attend (« < 0,5° = parfait »).

Usage :
    python compare_extrinsics.py \\
        --ref ../calibration/arducam_extrinsic_samesession.yaml \\
        --est ../calibration/arducam_extrinsic_dream_samesession.yaml
"""
import argparse
from pathlib import Path

import numpy as np
import yaml


def load_Twc(p):
    d = yaml.safe_load(Path(p).read_text())
    return np.array(d["T_world_cam"], dtype=np.float64), d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ref", required=True, help="extrinsèque de référence (marqueur)")
    ap.add_argument("--est", required=True, help="extrinsèque estimé (DREAM)")
    ap.add_argument("--target-deg", type=float, default=0.5,
                    help="seuil 'parfait' de l'encadrant (deg)")
    args = ap.parse_args()

    Tr, dr = load_Twc(args.ref)
    Te, de = load_Twc(args.est)
    dpos = np.linalg.norm(Tr[:3, 3] - Te[:3, 3]) * 1000
    Rrel = Tr[:3, :3].T @ Te[:3, :3]
    dang = np.degrees(np.arccos(np.clip((np.trace(Rrel) - 1) / 2, -1, 1)))

    print(f"Référence : {Path(args.ref).name}")
    print(f"Estimé    : {Path(args.est).name}")
    print(f"\n  position caméra réf   : {(Tr[:3,3]*1000).round(1)} mm")
    print(f"  position caméra DREAM : {(Te[:3,3]*1000).round(1)} mm")
    print(f"\n  Δ POSITION    = {dpos:6.1f} mm")
    print(f"  Δ ORIENTATION = {dang:6.2f} °   (cible < {args.target_deg}°)")
    verdict = ("PARFAIT" if dang < args.target_deg
               else "TRÈS BON" if dang < 2 else "CORRECT" if dang < 5
               else "À AMÉLIORER (ou caméra bougée entre les 2 captures)")
    print(f"\n  Verdict orientation : {verdict}")
    if "median_reproj_px" in de:
        print(f"  (DREAM reproj {de['median_reproj_px']:.1f} px, {de.get('n_inliers','?')} inliers)")


if __name__ == "__main__":
    main()
