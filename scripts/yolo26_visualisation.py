#!/usr/bin/env python3
"""Vue en direct des deux cameras, avec les 8 pieces du dossier vues par yolo26.

    .venv/bin/python scripts/yolo26_visualisation.py                    # arducam + SVPRO
    .venv/bin/python scripts/yolo26_visualisation.py --camera arducam
    .venv/bin/python scripts/yolo26_visualisation.py --modele <chemin/best.pt>

Fermer avec q, ou la croix. Aucun mouvement du robot, aucune commande de pince,
aucune ecriture sur disque.

Ce que montre la fenetre, par camera : les boites nommees (`bac_rouge`,
`cube_bleu`, `cylindre_vert`, `pave_jaune`...), le compte vu sur 8, et le temps
d'inference. Chaque piece est tracee dans sa propre couleur.

⚠ **Le modele n'a ete entrainé que sur des images ARDUCAM** (12 images d'une
scene immobile, 18/09). Sur la SVPRO — autre point de vue, autre echelle — il
n'a aucune raison de bien marcher, et la fenetre le montre sans le maquiller :
c'est une mesure a lire, pas une panne.

Trois pieges deja mesures, encodes ici (voir `tri_couleur.py`) :
  - YOLO26 est SANS NMS : ses quasi-doublons sont retires par recouvrement ;
  - son echelle de confiance n'est pas celle de YOLOE : seuil 0,10, pas 0,20,
    sinon le cube rouge (vu a 0,13 sur 6 trames sur 6) disparait ;
  - sous OpenCV 5.0, `getWindowProperty` LEVE au lieu de rendre 0 quand la
    fenetre est fermee.

**L'exposition de l'arducam est SURVEILLEE, pas seulement posee** — meme recette
que `pick_dashboard` : posee APRES l'ouverture, puis relue sur le peripherique
toutes les `PERIODE_SURVEILLANCE_S` secondes et reecrite seulement si elle a
derive. Le driver relache le reglage manuel tout seul — `auto_exposure` revient a
3 et l'exposition a 157 au lieu de 75 — et l'image se surexpose sans prevenir.
L'en-tete affiche donc l'etat LU sur le peripherique : `expo 75` en vert, ou
`AUTO (157)` en rouge avec le nombre de remises. Celle de la SVPRO, dont le
registre dit « automatique », n'est jamais touchee.
"""
import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE / 'scripts'))

import pick_dashboard as pd  # noqa: E402
import tri_couleur as tc  # noqa: E402
from yolo_capture import ouvre_camera  # noqa: E402

POIDS_DEFAUT = RACINE / 'runs' / 'detect' / 'training' / 'yolo' / 'runs' / 'pieces_v2_yolo26s' / 'weights' / 'best.pt'
PERIODE_SURVEILLANCE_S = 4.0          # le QTimer du dashboard bat a la meme cadence
FENETRE = 'pieces vues par yolo26'
GRIS = (150, 150, 150)
VERT = (60, 180, 60)
ROUGE = (40, 40, 220)


def detecte(modele, image, seuil):
    """[(nom, boite, confiance)] sans doublon, la plus sure d'abord."""
    r = modele.predict(image, conf=seuil, imgsz=640, verbose=False)[0]
    gardes = []
    for bt in sorted(r.boxes, key=lambda b: -float(b.conf)):
        boite = tuple(int(v) for v in bt.xyxy[0])
        if any(tc.recouvre(boite, g[1]) > tc.RECOUVREMENT_DOUBLON for g in gardes):
            continue
        gardes.append((modele.names[int(bt.cls)], boite, float(bt.conf)))
    return gardes


def dessine(image, camera, trouvees, ms, expo=None, avertissement=None):
    """expo = (texte, tenue) lu sur le peripherique, ou None si on ne le surveille pas."""
    vue = image.copy()
    for nom, (a, d, b, e), conf in trouvees:
        couleur = tc.BGR[nom.rsplit('_', 1)[-1]]
        cv2.rectangle(vue, (a, d), (b, e), couleur, 2)
        cv2.putText(vue, f'{nom} {conf:.2f}', (a, max(12, d - 5)),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.42, couleur, 1)
    vus = len({n for n, _, _ in trouvees})
    cv2.putText(vue, f'{camera} - {vus}/8 pieces - {ms:.0f} ms', (10, 22),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (80, 230, 255) if vus == 8 else (80, 160, 255), 2)
    ligne = 42
    if expo is not None:
        texte, tenue = expo
        cv2.putText(vue, texte, (10, ligne), cv2.FONT_HERSHEY_SIMPLEX, 0.45,
                    VERT if tenue else ROUGE, 1 if tenue else 2)
        ligne += 18
    if avertissement:
        cv2.putText(vue, avertissement, (10, ligne), cv2.FONT_HERSHEY_SIMPLEX, 0.42, GRIS, 1)
    return vue


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--camera', choices=('arducam', 'svpro'), action='append',
                    help='par defaut les deux')
    ap.add_argument('--modele', type=Path, default=POIDS_DEFAUT)
    ap.add_argument('--seuil', type=float, default=tc.SEUIL_MODELE_ENTRAINE)
    args = ap.parse_args()
    cameras = args.camera or ['arducam', 'svpro']
    if not args.modele.exists():
        sys.exit(f'poids introuvables : {args.modele}')

    from ultralytics import YOLO
    modele = YOLO(str(args.modele))
    print(f'modele : {args.modele.name} — classes {list(modele.names.values())}')

    specs = {s.name: s for s in pd.registre.detect_cameras(probe_capture=False) if s.name in cameras}
    caps = {}
    for camera in cameras:
        if camera not in specs:
            print(f'{camera} : non detectee')
            continue
        cap = ouvre_camera(specs[camera].v4l2_index)
        print(f'{camera} /dev/video{specs[camera].v4l2_index} : {"ouverte" if cap else "muette"}')
        if cap is None:
            continue
        caps[camera] = cap
        # APRES l'ouverture, comme le dashboard. L'arducam seule : sa valeur est
        # mesuree (75) et toutes les images du banc y sont prises. La SVPRO, que
        # le registre veut en automatique, n'est pas touchee.
        if camera == 'arducam':
            pd.regle_exposition(specs[camera].v4l2_index, specs[camera].manual_exposure)
            print(f'  exposition posee a {specs[camera].manual_exposure}')
    if not caps:
        sys.exit('aucune camera ouverte (deja utilisee par le dashboard ?)')

    cv2.namedWindow(FENETRE, cv2.WINDOW_NORMAL)
    attendue = specs['arducam'].manual_exposure if 'arducam' in caps else None
    expo, derives, dernier = None, 0, 0.0
    try:
        while True:
            # Relire le peripherique et ne reecrire que si le driver a relache le
            # reglage — surveillance du dashboard, et non repose aveugle : sans la
            # relecture, on ne sait pas si 75 tient, et l'image se surexpose sans
            # que rien ne le dise.
            if attendue is not None and time.time() - dernier >= PERIODE_SURVEILLANCE_S:
                dernier = time.time()
                index = specs['arducam'].v4l2_index
                etat = pd.lit_controles(index)
                if etat.get('auto_exposure') == 1 and etat.get('exposure_time_absolute') == attendue:
                    expo = (f'expo {attendue}' + (f' - {derives} remise(s)' if derives else ''), True)
                else:
                    pd.regle_exposition(index, attendue)
                    derives += 1
                    expo = (f'AUTO ({etat.get("exposure_time_absolute")}) -> remise a '
                            f'{attendue} [{derives}]', False)
            vues = []
            for camera, cap in caps.items():
                ok, image = cap.read()
                if not ok:
                    vues.append(np.zeros((pd.registre.CAPTURE_H, pd.registre.CAPTURE_W, 3), np.uint8))
                    continue
                debut = time.time()
                trouvees = detecte(modele, image, args.seuil)
                vues.append(dessine(
                    image, camera, trouvees, (time.time() - debut) * 1000,
                    expo if camera == 'arducam' else None,
                    None if camera == 'arducam' else 'modele entraine sur arducam seulement'))
            cv2.imshow(FENETRE, np.hstack(vues))
            touche = cv2.waitKey(1) & 0xFF
            try:
                ouverte = cv2.getWindowProperty(FENETRE, cv2.WND_PROP_VISIBLE) >= 1
            except cv2.error:
                ouverte = False
            if touche in (ord('q'), 27) or not ouverte:
                break
    finally:
        for cap in caps.values():
            cap.release()
        cv2.destroyAllWindows()


if __name__ == '__main__':
    main()
