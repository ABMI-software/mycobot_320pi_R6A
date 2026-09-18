#!/usr/bin/env python3
"""Apercu des 4 marqueurs de planche, arducam et/ou SVPRO, et controle de l'extrinseque.

    .venv/bin/python scripts/apercu_marqueurs.py                      # les deux cameras
    .venv/bin/python scripts/apercu_marqueurs.py --camera svpro
    .venv/bin/python scripts/apercu_marqueurs.py --camera arducam --image photo.png

Pour chaque marqueur 19/23/25/26 :
  - le cadre detecte, vert / orange / rouge selon sa distance au bord de l'image
    (pres du bord la detection devient intermittente : le 25 de la SVPRO, 15/09) ;
  - une croix la ou l'extrinseque EN SERVICE l'attend, et l'ecart en mm entre ce
    que la camera voit et la reference `planche_actuelle.yaml`, mesuree au robot.
    Un ecart dit que la camera a bouge depuis sa calibration : meme verdict que la
    fenetre de calibration du dashboard (< 2 mm rien a faire, < 20 conseillee).

La luminance est affichee : les calibrations acceptees du 15/09 etaient a 83-85,
une refusee a 66. Aucun mouvement du robot. `.venv` : `cv2.aruco` fait planter
l'OpenCV du systeme (cf. `aruco_check.py`). Fermer avec q ; la camera doit etre
liberee avant de lancer une calibration.
"""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE / 'scripts'))

import calibrer_extrinseque_4aruco as aruco  # noqa: E402
import pick_dashboard as pd  # noqa: E402
from yolo_capture import ouvre_camera  # noqa: E402

MARQUEURS = (19, 23, 25, 26)
REFERENCE = RACINE / 'training' / 'calibration' / 'planche_actuelle.yaml'
EXTRINSEQUES = {'arducam': 'arducam_extrinsic_pick', 'svpro': 'svpro_extrinsic_servo'}
MARGE_BONNE_PX = 40
MARGE_JUSTE_PX = 15
ECART_RIEN_A_FAIRE_MM = 2.0
ECART_CONSEILLE_MM = 20.0
LUMINANCE_REUSSIE = (83, 85)
PERIODE_EXPOSITION_IMAGES = 120
HAUTEUR_BANDEAU_PX = 60
VERT, ORANGE, ROUGE, CYAN = (0, 200, 0), (0, 165, 255), (0, 0, 255), (255, 255, 0)


def reference():
    return {int(k): np.array(v, float) for k, v in yaml.safe_load(REFERENCE.read_text())['markers'].items()}


def etat(image, vision, ref):
    """{id: (coins, marge_px, pixel_attendu, ecart_mm)} pour les marqueurs vus, et la luminance."""
    h, w = image.shape[:2]
    vus = {}
    for ident, coins in aruco.detecte(image).items():
        if ident not in MARQUEURS:
            continue
        marge = float(min(coins[:, 0].min(), coins[:, 1].min(), w - coins[:, 0].max(), h - coins[:, 1].max()))
        vu_mm = vision.vers_base(coins.mean(axis=0), 0.0)[:2]
        vus[ident] = (coins, marge, vision.vers_pixel(ref[ident]),
                      float(np.linalg.norm(vu_mm - ref[ident][:2])))
    return vus, float(cv2.cvtColor(image, cv2.COLOR_BGR2GRAY).mean())


def verdict(vus):
    if len(vus) < len(MARQUEURS):
        manquants = [i for i in MARQUEURS if i not in vus]
        return f'manque {manquants} : pas de calibration possible', ROUGE
    pire = max(e for _, _, _, e in vus.values())
    if pire < ECART_RIEN_A_FAIRE_MM:
        return f'4/4 - extrinseque a {pire:.1f} mm au pire : rien a faire', VERT
    if pire < ECART_CONSEILLE_MM:
        return f'4/4 - extrinseque a {pire:.1f} mm au pire : recalibration conseillee', ORANGE
    return f'4/4 - extrinseque a {pire:.0f} mm au pire : recalibration NECESSAIRE', ROUGE


def dessine(image, camera, vus, luminance):
    vue = image.copy()
    for ident, (coins, marge, attendu, ecart) in sorted(vus.items()):
        couleur = VERT if marge >= MARGE_BONNE_PX else ORANGE if marge >= MARGE_JUSTE_PX else ROUGE
        cv2.polylines(vue, [coins.astype(np.int32)], True, couleur, 2)
        u, v = attendu.astype(int)
        cv2.drawMarker(vue, (u, v), CYAN, cv2.MARKER_CROSS, 14, 2)
        centre = coins.mean(axis=0).astype(int)
        # Sous le marqueur quand il est en haut : au-dessus, le texte recouvrirait le bandeau.
        y_texte = centre[1] - 18 if centre[1] - 18 > HAUTEUR_BANDEAU_PX else centre[1] + 30
        cv2.putText(vue, f'{ident}: bord {marge:.0f}px  {ecart:.1f}mm', (centre[0] - 60, y_texte),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, couleur, 2)
    texte, couleur = verdict(vus)
    lum_ok = LUMINANCE_REUSSIE[0] - 5 <= luminance <= LUMINANCE_REUSSIE[1] + 5
    cv2.putText(vue, f'{camera} - {texte}', (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.55, couleur, 2)
    cv2.putText(vue, f'luminance {luminance:.0f} (calibrations acceptees : {LUMINANCE_REUSSIE[0]}-'
                     f'{LUMINANCE_REUSSIE[1]})   croix = attendu par l extrinseque   q pour fermer',
                (8, 44), cv2.FONT_HERSHEY_SIMPLEX, 0.45, VERT if lum_ok else ORANGE, 1)
    return vue


def resume_texte(camera, vus, luminance):
    lignes = [f'{camera} : {verdict(vus)[0]} ; luminance {luminance:.0f}']
    lignes += [f'  {i} : bord {m:.0f} px, ecart extrinseque {e:.1f} mm' for i, (_, m, _, e) in sorted(vus.items())]
    return '\n'.join(lignes)


def en_direct(cameras):
    ref = reference()
    visions = {c: pd.Vision(EXTRINSEQUES[c]) for c in cameras}
    specs = {s.name: s for s in pd.registre.detect_cameras(probe_capture=False) if s.name in cameras}
    caps = {}
    for camera in cameras:
        if camera not in specs:
            print(f'{camera} : non detectee')
            continue
        caps[camera] = ouvre_camera(specs[camera].v4l2_index)
        if caps[camera] is not None:
            pd.regle_exposition(specs[camera].v4l2_index, specs[camera].manual_exposure)
        print(f'{camera} /dev/video{specs[camera].v4l2_index} : {"ouverte" if caps[camera] else "muette"}')
    caps = {c: cap for c, cap in caps.items() if cap is not None}
    if not caps:
        sys.exit('aucune camera ouverte (deja utilisee par le dashboard ?)')
    fenetre = 'marqueurs de planche'
    cv2.namedWindow(fenetre, cv2.WINDOW_NORMAL)
    n = 0
    try:
        while True:
            vues = []
            for camera, cap in caps.items():
                ok, image = cap.read()
                if not ok:
                    vues.append(np.zeros((pd.registre.CAPTURE_H, pd.registre.CAPTURE_W, 3), np.uint8))
                    continue
                vus, luminance = etat(image, visions[camera], ref)
                vues.append(dessine(image, camera, vus, luminance))
            cv2.imshow(fenetre, np.hstack(vues))
            n += 1
            # L'arducam repasse en exposition auto toute seule (constate sur le dashboard).
            if n % PERIODE_EXPOSITION_IMAGES == 0:
                for camera in caps:
                    pd.regle_exposition(specs[camera].v4l2_index, specs[camera].manual_exposure)
            touche = cv2.waitKey(1) & 0xFF
            try:
                ouverte = cv2.getWindowProperty(fenetre, cv2.WND_PROP_VISIBLE) >= 1
            except cv2.error:
                # OpenCV 5.0 LEVE (« NULL guiReceiver ») au lieu de rendre 0 quand la
                # fenetre a ete fermee a la croix : la sortie normale devenait une trace.
                ouverte = False
            if touche in (ord('q'), 27) or not ouverte:
                break
    finally:
        for cap in caps.values():
            cap.release()
        cv2.destroyAllWindows()


def sur_image(chemin, camera):
    image = cv2.imread(str(chemin))
    if image is None:
        sys.exit(f'image illisible : {chemin}')
    vus, luminance = etat(image, pd.Vision(EXTRINSEQUES[camera]), reference())
    print(resume_texte(camera, vus, luminance))
    sortie = Path(chemin).with_name(f'{Path(chemin).stem}_marqueurs.png')
    cv2.imwrite(str(sortie), dessine(image, camera, vus, luminance))
    print(f'apercu -> {sortie}')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--camera', choices=('arducam', 'svpro', 'les-deux'), default='les-deux')
    ap.add_argument('--image', type=Path, help='photo existante au lieu du direct (une seule camera)')
    args = ap.parse_args()
    cameras = ('arducam', 'svpro') if args.camera == 'les-deux' else (args.camera,)
    if args.image:
        if len(cameras) != 1:
            sys.exit('--image demande --camera arducam ou --camera svpro')
        sur_image(args.image, cameras[0])
    else:
        en_direct(cameras)


if __name__ == '__main__':
    main()
