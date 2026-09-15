#!/usr/bin/env python3
"""Jeu d'images YOLO du pick : prises arducam + SVPRO brutes, pre-annotees par YOLOE-26.

    .venv/bin/python scripts/yolo_capture.py capture
    .venv/bin/python scripts/yolo_capture.py preannote DOSSIER_IMAGES [--camera svpro]
    /usr/bin/python3 scripts/yolo_capture.py capture --detecteur hsv     # sans torch : balle seule

`capture` ouvre une fenetre : les deux vues en direct avec les detections,
Espace = prise, q ou Echap = quitter. Aucun mouvement du robot, aucune commande
de pince : on deplace les objets a la main entre deux prises. Les images
enregistrees sont brutes, sans les boites.

Trois couches, mesurees le 15/09/2026 sur 12 photos du banc et 14 vues SVPRO des
saisies du 14/09 :

1. Objets NOMMES (YOLOE-26 par le nom). `INVITES` donne modele, seuil et noms :
   - balle : `tennis ball` sur 26s, 0,12-0,70, aucun faux positif >= 0,08 ;
   - cylindre : `tape roll`/`bottle cap` sur 26l, 0,51-0,93 ; faux positifs a
     0,33 (la balle) et 0,12 (la base du robot), d'ou le seuil 0,40 ;
   - cube : jamais detecte par le nom (15/09), gere par la couche 3.
2. Balle dans la pince : YOLOE lui donne 0 de confiance (26s comme 26l). Le seuil
   jaune du dashboard la complete (« balle couleur ») : 14/14.
3. TOUT le reste (`objet`) : YOLOE-26l sans consigne, vocabulaire de 4 585 noms.
   Les noms sont approximatifs (balle = `opal`, cylindre = `adhesive tape`) mais
   les boites justes ; il trouve le cylindre vu de cote que le nom rate. Il
   sort aussi tout le decor : on ne garde que ce qui touche la planche, en
   millimetres via les marqueurs 19/23/25/26 (clavier, plante, ecran, robot,
   marqueurs et planche entiere ecartes). Seuil 0,35 : les morceaux de pince et
   de cable sortent a 0,25-0,29, les vrais objets a 0,42 et plus. Deux faux
   objets restaient sur les saisies du 14/09, tous deux corriges : le marqueur
   23 cache par la pince (`keycard`, place a cote par l'affine a 3 marqueurs ->
   l'homographie a 4 marqueurs est gardee une fois obtenue) et le corps de la
   pince souleve (`power see` 0,56-0,80 -> ecarte par la silhouette du bras).
   Un objet inconnu TENU dans la pince est donc ecarte aussi.

`mors` ne vient pas de YOLOE, qui pose la pince sur la base du robot : ce sont
les deux plus grandes zones sombres autour d'un objet detecte, hors boites
d'objets, et il en faut DEUX (une zone seule etait toujours le cable, un pied
d'ecran ou la base). Pre-annotation a relire, pas une verite terrain.
"""
import argparse
import contextlib
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE / 'scripts'))

import pick_dashboard as pd  # noqa: E402

CLASSES = ('balle', 'cylindre', 'cube', 'mors', 'objet')
INVITES = {
    'balle': ('yoloe-26s-seg.pt', 0.10, ('tennis ball',)),
    'cylindre': ('yoloe-26l-seg.pt', 0.40, ('tape roll', 'bottle cap')),
}
MODELE_OBJETS = 'yoloe-26l-seg-pf.pt'
SEUIL_OBJET = 0.35
AIRE_OBJET_MAX = 0.05
# Le corps de la pince souleve au-dessus de la planche sortait `power see` a 0,56-0,80
# (14/09) : un `objet` couvert a plus de PART_BRAS_MAX par la silhouette du bras est ecarte.
# `white robot arm` sur 26s : 0,79-0,98, silhouette couvrant 86-90 % du corps de pince ;
# `robot arm` seul plafonnait a 0,2.
MODELE_BRAS = 'yoloe-26s-seg.pt'
INVITE_BRAS = 'white robot arm'
SEUIL_BRAS = 0.30
PART_BRAS_MAX = 0.5
POIDS = RACINE / 'weights' / 'yoloe'
MARQUEURS = RACINE / 'training' / 'calibration' / 'workspace_markers.yaml'
# Bords de table deduits au ruban depuis les marqueurs (commentaire de workspace_markers.yaml).
TABLE_X_MM = (-50.0, 575.0)
TABLE_Y_MM = (-212.0, 246.0)
RAYON_BASE_MM = 120.0
RAYON_MARQUEUR_MM = 45.0
RECOUVREMENT_MAX = 0.5
SORTIE = RACINE / 'training' / 'yolo' / 'captures'
AIRE_BALLE_MIN = 60
AIRE_MORS_MIN = 40
SEUIL_SOMBRE_V = 70
FENETRE = 'yolo_capture'
PERIODE_EXPOSITION_S = 4.0


def recouvrement(a, b):
    """Part commune rapportee a la plus petite boite : une boite nichee dans une autre compte."""
    largeur = max(0, min(a[2], b[2]) - max(a[0], b[0]))
    hauteur = max(0, min(a[3], b[3]) - max(a[1], b[1]))
    plus_petite = min((a[2] - a[0]) * (a[3] - a[1]), (b[2] - b[0]) * (b[3] - b[1]))
    return largeur * hauteur / max(1, plus_petite)


def ajoute_sans_doublon(gardees, candidats):
    for d in candidats:
        if all(recouvrement(d[1], g[1]) < RECOUVREMENT_MAX for g in gardees):
            gardees.append(d)
    return gardees


class Planche:
    """Pixel -> millimetres dans le plan de la table, par les marqueurs de la planche.

    Derniere transformation gardee par camera : les cameras sont fixes, et une
    main qui cache un marqueur ne doit pas eteindre la detection.
    """

    def __init__(self):
        marqueurs = yaml.safe_load(MARQUEURS.read_text())['markers']
        self.marqueurs = {int(k): np.array(v[:2], np.float32) for k, v in marqueurs.items()}
        self.aruco = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50),
                                             cv2.aruco.DetectorParameters())
        self.vers_mm = {}

    def actualise(self, image, camera):
        coins, ids, _ = self.aruco.detectMarkers(image)
        vus = {} if ids is None else {int(i): c.reshape(4, 2).mean(0)
                                      for c, i in zip(coins, ids.flatten()) if int(i) in self.marqueurs}
        px = np.array(list(vus.values()), np.float32)
        mm = np.array([self.marqueurs[i] for i in vus], np.float32)
        if len(vus) == 4:
            self.vers_mm[camera] = (cv2.findHomography(px, mm)[0], True)
        # Affine a 3 marqueurs seulement tant qu'aucune homographie n'existe : sur la
        # SVPRO oblique elle place mal le 23 cache par la pince, vu `keycard` a 0,51-0,59.
        elif len(vus) == 3 and not self.vers_mm.get(camera, (None, False))[1]:
            self.vers_mm[camera] = (np.vstack([cv2.getAffineTransform(px, mm), [0, 0, 1]]), False)

    def point_mm(self, boite, camera):
        """Bas-milieu de la boite : l'endroit ou l'objet touche la table."""
        point = np.array([[[(boite[0] + boite[2]) / 2, boite[3]]]], np.float32)
        return cv2.perspectiveTransform(point, self.vers_mm[camera][0])[0, 0]

    def porte(self, boite, camera):
        if camera not in self.vers_mm:
            return False
        p = self.point_mm(boite, camera)
        return (TABLE_X_MM[0] <= p[0] <= TABLE_X_MM[1] and TABLE_Y_MM[0] <= p[1] <= TABLE_Y_MM[1]
                and np.hypot(*p) >= RAYON_BASE_MM
                and all(np.hypot(*(p - m)) >= RAYON_MARQUEUR_MM for m in self.marqueurs.values()))


class DetecteurHSV:
    bras = None

    def detecte(self, image, camera=None):
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        masque = cv2.morphologyEx(cv2.inRange(hsv, *pd.HSV_BALLE), cv2.MORPH_CLOSE,
                                  np.ones((5, 5), np.uint8))
        contours = [c for c in cv2.findContours(masque, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[0]
                    if cv2.contourArea(c) > AIRE_BALLE_MIN]
        if not contours:
            return []
        x, y, w, h = cv2.boundingRect(max(contours, key=cv2.contourArea))
        return [(CLASSES.index('balle'), (x, y, x + w, y + h), 0.0, 'couleur')]


class DetecteurYOLOE:
    def __init__(self):
        from ultralytics import YOLOE
        self.couleur = DetecteurHSV()
        self.planche = Planche()
        self.invites = {}
        for classe, (fichier, seuil, noms) in INVITES.items():
            self.invites.setdefault(fichier, []).extend((n, CLASSES.index(classe), seuil) for n in noms)
        self.invites.setdefault(MODELE_BRAS, []).append((INVITE_BRAS, None, SEUIL_BRAS))
        self.modeles = {}
        for fichier, invites in self.invites.items():
            modele = YOLOE(str(POIDS / fichier))
            # L'encodeur de texte (mobileclip2_b.ts, 250 Mo) est cherche dans le repertoire courant.
            with contextlib.chdir(POIDS):
                modele.set_classes([n for n, _, _ in invites])
            self.modeles[fichier] = modele
        self.objets = YOLOE(str(POIDS / MODELE_OBJETS))

    def nommes(self, image):
        """Objets nommes, et silhouette du bras (masque 0/1) pour ecarter ses morceaux."""
        candidats = []
        bras = np.zeros(image.shape[:2], np.uint8)
        for fichier, invites in self.invites.items():
            r = self.modeles[fichier].predict(image, conf=min(s for _, _, s in invites), verbose=False)[0]
            for i, b in enumerate(r.boxes):
                nom, classe, seuil = invites[int(b.cls)]
                if float(b.conf) < seuil:
                    continue
                if classe is None:
                    contour = r.masks.xy[i]
                    if len(contour) >= 3:
                        cv2.fillPoly(bras, [contour.astype(np.int32)], 1)
                    continue
                candidats.append((float(b.conf) / seuil,
                                  (classe, tuple(int(v) for v in b.xyxy[0]), float(b.conf), nom)))
        return [d for _, d in sorted(candidats, key=lambda t: -t[0])], bras

    def tout(self, image, camera, bras):
        self.planche.actualise(image, camera)
        h, w = image.shape[:2]
        r = self.objets.predict(image, conf=SEUIL_OBJET, verbose=False)[0]
        trouves = []
        for b in r.boxes:
            x0, y0, x1, y1 = boite = tuple(int(v) for v in b.xyxy[0])
            if ((x1 - x0) * (y1 - y0) <= AIRE_OBJET_MAX * w * h
                    and bras[y0:y1, x0:x1].mean() < PART_BRAS_MAX
                    and self.planche.porte(boite, camera)):
                trouves.append((CLASSES.index('objet'), boite, float(b.conf), r.names[int(b.cls)]))
        return sorted(trouves, key=lambda d: -d[2])

    def detecte(self, image, camera):
        nommes, bras = self.nommes(image)
        self.bras = bras
        gardees = ajoute_sans_doublon([], nommes)
        if not any(CLASSES[d[0]] == 'balle' for d in gardees):
            ajoute_sans_doublon(gardees, self.couleur.detecte(image))
        return ajoute_sans_doublon(gardees, self.tout(image, camera, bras))


def detecte_mors(image, objet, objets, bras):
    """La morphologie 7x7 efface le cable (~3 px), pas les mors. Les boites des
    objets detectes sont retirees : le dessus noir du cylindre passait pour un mors.
    Les mors font partie du bras : sans silhouette du bras dans la zone, pas de mors
    (le cable au bord de la planche passait pour une paire, 15/09)."""
    cx, cy = (objet[0] + objet[2]) / 2, (objet[1] + objet[3]) / 2
    x0, x1 = int(max(0, cx - 75)), int(min(image.shape[1], cx + 75))
    y0, y1 = int(max(0, cy - 90)), int(min(image.shape[0], cy + 35))
    if bras is not None and not bras[y0:y1, x0:x1].any():
        return []
    hsv = cv2.cvtColor(image[y0:y1, x0:x1], cv2.COLOR_BGR2HSV)
    sombre = cv2.inRange(hsv, (0, 0, 0), (180, 255, SEUIL_SOMBRE_V))
    for a, d, b, e in objets:
        sombre[max(0, d - y0):max(0, e - y0), max(0, a - x0):max(0, b - x0)] = 0
    ouvert = cv2.morphologyEx(sombre, cv2.MORPH_OPEN, np.ones((7, 7), np.uint8))
    n, _, stats, _ = cv2.connectedComponentsWithStats(ouvert)
    grands = [i for i in sorted(range(1, n), key=lambda i: -stats[i, cv2.CC_STAT_AREA])[:2]
              if stats[i, cv2.CC_STAT_AREA] > AIRE_MORS_MIN]
    # Une zone seule n'est jamais la pince : cable, pied d'ecran ou base (14-15/09).
    if len(grands) < 2:
        return []
    return [(x0 + stats[i, 0], y0 + stats[i, 1],
             x0 + stats[i, 0] + stats[i, 2], y0 + stats[i, 1] + stats[i, 3])
            for i in grands]


def preannote(detecteur, image, camera):
    boites = detecteur.detecte(image, camera)
    if camera == 'svpro':
        objets = [b for _, b, _, _ in boites]
        for objet in objets:
            mors = detecte_mors(image, objet, objets, detecteur.bras)
            if mors:
                boites += [(CLASSES.index('mors'), b, 0.0, '') for b in mors]
                break
    return boites


def dessine(image, boites):
    apercu = image.copy()
    couleurs = {'mors': (255, 0, 255), 'objet': (0, 200, 255)}
    for c, (a, d, b, e), conf, nom in boites:
        classe = CLASSES[c]
        texte = (classe if classe == 'mors' else
                 f'objet ({nom}) {conf:.2f}' if classe == 'objet' else
                 f'{classe} {conf:.2f}' if conf > 0 else f'{classe} {nom}')
        couleur = couleurs.get(classe, (0, 0, 255))
        cv2.rectangle(apercu, (a, d), (b, e), couleur, 2)
        cv2.putText(apercu, texte, (a, max(12, d - 4)), cv2.FONT_HERSHEY_SIMPLEX, 0.45, couleur, 1)
    return apercu


def resume(boites):
    return ', '.join(CLASSES[c] for c, _, _, _ in boites) or 'rien'


def enregistre(image, boites, sortie, camera, nom):
    h, w = image.shape[:2]
    for sous in ('images', 'labels', 'apercu'):
        (sortie / sous / camera).mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(sortie / 'images' / camera / f'{nom}.png'), image)
    lignes = [f'{c} {(a + b) / 2 / w:.6f} {(d + e) / 2 / h:.6f} {(b - a) / w:.6f} {(e - d) / h:.6f}'
              for c, (a, d, b, e), _, _ in boites]
    (sortie / 'labels' / camera / f'{nom}.txt').write_text(''.join(l + '\n' for l in lignes))
    cv2.imwrite(str(sortie / 'apercu' / camera / f'{nom}.png'), dessine(image, boites))
    return resume(boites)


def ecrit_data_yaml(sortie):
    noms = ''.join(f'  {i}: {n}\n' for i, n in enumerate(CLASSES))
    sortie.mkdir(parents=True, exist_ok=True)
    (sortie / 'data.yaml').write_text(f'path: {sortie}\ntrain: images\nval: images\nnames:\n{noms}')


def ouvre_camera(index, essais=5):
    """Valider par une LECTURE : une ouverture trop proche de la precedente
    s'ouvre sans delivrer d'image (meme recette que pick_dashboard)."""
    for _ in range(essais):
        cap = cv2.VideoCapture(index)
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, pd.registre.CAPTURE_W)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, pd.registre.CAPTURE_H)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
        if cap.isOpened() and cap.read()[0]:
            return cap
        cap.release()
        time.sleep(1.2)
    return None


def panneau(image, camera, boites):
    if image is None:
        vide = np.zeros((pd.registre.CAPTURE_H, pd.registre.CAPTURE_W, 3), np.uint8)
        cv2.putText(vide, f'{camera} : pas d image', (20, 240), cv2.FONT_HERSHEY_SIMPLEX, 0.8,
                    (0, 0, 255), 2)
        return vide
    vue = dessine(image, boites)
    cv2.putText(vue, f'{camera} : {resume(boites)}', (10, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                (0, 255, 255), 2)
    return vue


def capture(detecteur, sortie):
    cameras = {s.name: (s.v4l2_index, s.manual_exposure)
               for s in pd.registre.detect_cameras(probe_capture=False)
               if s.name in ('arducam', 'svpro')}
    if not cameras:
        sys.exit('aucune camera arducam/svpro detectee')
    caps = {}
    for camera, (index, exposition) in sorted(cameras.items()):
        caps[camera] = ouvre_camera(index)
        print(f'{camera} /dev/video{index} : {"ouverte" if caps[camera] else "muette"}')
        if caps[camera] is not None:
            pd.regle_exposition(index, exposition)
    ecrit_data_yaml(sortie)
    numero = len(list((sortie / 'images' / 'arducam').glob('*.png')))
    message, message_jusqu_a, derniere_exposition = '', 0.0, time.time()
    cv2.namedWindow(FENETRE, cv2.WINDOW_NORMAL)
    try:
        while True:
            images = {camera: (cap.read()[1] if cap is not None else None) for camera, cap in caps.items()}
            boites = {camera: preannote(detecteur, image, camera) if image is not None else []
                      for camera, image in images.items()}
            vue = np.hstack([panneau(images[c], c, boites[c]) for c in sorted(caps)])
            bandeau = np.zeros((34, vue.shape[1], 3), np.uint8)
            texte = message if time.time() < message_jusqu_a else \
                f'Espace = prise   q = quitter   {numero} prises dans {sortie}'
            cv2.putText(bandeau, texte, (10, 23), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
            cv2.imshow(FENETRE, np.vstack([vue, bandeau]))

            touche = cv2.waitKey(1) & 0xFF
            if touche in (ord('q'), 27) or cv2.getWindowProperty(FENETRE, cv2.WND_PROP_VISIBLE) < 1:
                break
            if touche == ord(' '):
                nom = f'{int(time.time())}_{numero:06d}'
                faites = [f'{c} : {enregistre(images[c], boites[c], sortie, c, nom)}'
                          for c in sorted(caps) if images[c] is not None]
                print(f'{nom}  ' + '  |  '.join(faites))
                message, message_jusqu_a = f'prise {nom} enregistree  ' + '  |  '.join(faites), time.time() + 1.5
                numero += 1

            # L'arducam repasse en exposition auto toute seule (constate sur le dashboard).
            if time.time() - derniere_exposition > PERIODE_EXPOSITION_S:
                for camera, (index, exposition) in cameras.items():
                    if caps[camera] is not None and exposition >= 0:
                        pd.regle_exposition(index, exposition)
                derniere_exposition = time.time()
    finally:
        for cap in caps.values():
            if cap is not None:
                cap.release()
        cv2.destroyAllWindows()


def preannote_dossier(detecteur, dossier, camera, sortie):
    ecrit_data_yaml(sortie)
    for chemin in sorted(Path(dossier).glob('*.png')):
        image = cv2.imread(str(chemin))
        boites = preannote(detecteur, image, camera)
        print(f'{chemin.name} : {enregistre(image, boites, sortie, camera, chemin.stem)}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--detecteur', choices=('yoloe', 'hsv'), default='yoloe')
    ap.add_argument('--sortie', type=Path, default=SORTIE)
    sous = ap.add_subparsers(dest='mode', required=True)
    sous.add_parser('capture')
    p = sous.add_parser('preannote')
    p.add_argument('dossier', type=Path)
    p.add_argument('--camera', choices=('arducam', 'svpro'), default='svpro')
    args = ap.parse_args()
    detecteur = DetecteurYOLOE() if args.detecteur == 'yoloe' else DetecteurHSV()
    if args.mode == 'capture':
        capture(detecteur, args.sortie)
    else:
        preannote_dossier(detecteur, args.dossier, args.camera, args.sortie)


if __name__ == '__main__':
    main()
