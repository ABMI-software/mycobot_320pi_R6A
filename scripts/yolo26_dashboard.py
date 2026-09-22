#!/usr/bin/env python3
"""yolo26 entraine dans pick_dashboard, sans toucher a `pick_dashboard.py`.

    /usr/bin/python3 scripts/lancer_pick_dashboard_final.py

Le banc a change d'objets : plus de scotch ni de petit robot imprime, mais les
QUATRE pieces peintes du dossier de fabrication et LEURS QUATRE BACS. Chaque
piece va dans le bac de sa couleur — la peinture dit la destination, il n'y a
plus d'ordre de tri a donner (`tri_couleur.affecte`).

Ce que ce module remplace dans le dashboard, et par quoi :

| avant                                   | maintenant                          |
|-----------------------------------------|-------------------------------------|
| `Vision.objets` — trou, robe sombre, cotes en mm | yolo26 : classe + boite + confiance |
| `Vision.cartons` — creux + seuil d'Otsu sur le coeur sombre | yolo26, un bac par couleur |
| `Vision.balle` — seuil HSV jaune        | rien : il n'y a plus de balle       |
| 2 cartons `grand`/`petit`, nommes par marqueurs colles | 4 bacs nommes par leur couleur |
| `DESTINATION` fixee a la main           | objet -> bac de SA couleur          |

Les DEUX cameras passent par yolo26. C'est la difference avec `yolo_dashboard.py`
(YOLOE, arducam seule) : le jeu `pieces_v5` contient 44 images arducam **et 30
SVPRO**, le modele a donc vu les deux points de vue, dans des dispositions
variees. Voir `yolo26_service.POIDS_DEFAUT` pour ce que cela a change.

**Aucun dimensionnement, aucun traitement de pixel.** La chaine tient en trois
etapes et rien d'autre :

    yolo26 -> classe + boite + confiance -> centre de la boite -> extrinseque -> mm robot

Pas de masque de couleur pour retailler le contour, pas de seuil d'aire, pas de
silhouette du bras, pas de cotes en mm pour trancher bac/objet. Ces filtres
existaient pour YOLOE, qui devinait un nom parmi 4 585 et sortait des
trouvailles absurdes qu'il fallait ecarter par la geometrie. Un modele entraine
sur ces 8 classes ne sort ni le bras, ni un cable, ni un marqueur : sa
CONFIANCE est le seul juge, et le seul reglage est son seuil.

Deux points mesures, conserves (`tri_couleur`, 18/09) :
  - yolo26 est **sans NMS** : ses quasi-doublons sont retires par recouvrement,
    dans le service ;
  - son echelle de confiance n'est pas celle de YOLOE : seuil **0,10** et non
    0,20, sinon le cube rouge (vu a 0,13 six fois sur six) disparait.

La hauteur de projection reste `module.HAUTEUR_OBJET` : c'est le plan sur lequel
l'extrinseque rend le XY, et toutes les mesures du banc y ont ete prises.
"""
from __future__ import annotations

import atexit
import json
import subprocess
import sys
import threading
import time
from pathlib import Path

import cv2
import numpy as np

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE / 'scripts'))

import pick_fsm as fsm                                                  # noqa: E402
import tri_couleur as tc                                                # noqa: E402

sys.path.insert(0, str(RACINE / 'mycobot_gateway' / 'mycobot_gateway' / 'vision'))
from multiview_localizer import pixel_ray, triangulate                  # noqa: E402

SERVICE = RACINE / 'scripts' / 'yolo26_service.py'
VENV = RACINE / '.venv' / 'bin' / 'python'

# {nom yolo26 de la piece: couleur} et {nom yolo26 du bac: couleur}.
OBJETS = {tc.NOM_PIECE[c]: c for c in tc.TEINTES}
BACS = {f'bac_{c}': c for c in tc.TEINTES}
# La peinture dit la destination : chaque piece dans le bac de SA couleur.
DESTINATION = {piece: f'bac_{couleur}' for piece, couleur in OBJETS.items()}
# Hauteur du rebord d'un bac, d'apres le dossier de fabrication (105 x 105 x 30),
# et non les 83 mm du carton d'avant. Elle sert a projeter l'ouverture au bon
# plan ; le largage, lui, reste au plancher `pick_fsm.Z_LARGAGE` (116,9 mm), qui
# domine `rebord + GARDE_LARGAGE` pour un bac aussi bas.
HAUTEUR_BAC = tc.BAC[2]
DEMI_COTE_BAC_MM = tc.BAC[0] / 2.0

# `_detecte_objet` lit la derniere image traitee sans attendre. yolo26 ayant une
# image de retard, le bras a peine degage serait juge sur une image ou il
# masquait encore la planche. Meme recette que `yolo_dashboard`.
PATIENCE_FRAICHE_S = 1.0
ATTENTE_RAFRAICHI_S = 0.12


class ServiceYOLO26:
    """`yolo26_service.py` en tache de fond, partage par les DEUX cameras.

    Un seul modele sur le GPU, une file d'au plus une image EN ATTENTE PAR
    CAMERA : la vue qui rafraichit vite ne doit pas affamer l'autre, et une
    image perimee ne vaut pas la peine d'etre calculee.
    """

    def __init__(self, poids=None, seuil=None):
        commande = [str(VENV), str(SERVICE)]
        if poids:
            commande.append(str(poids))
            if seuil is not None:
                commande.append(str(seuil))
        self.proc = subprocess.Popen(commande, stdin=subprocess.PIPE,
                                     stdout=subprocess.PIPE, cwd=str(RACINE / 'scripts'))
        ligne = self.proc.stdout.readline()
        if not ligne:
            raise RuntimeError('yolo26_service.py n a pas demarre')
        pret = json.loads(ligne)
        if 'erreur' in pret:
            raise RuntimeError(pret['erreur'])
        self.poids = pret['poids']
        self.seuil = pret['seuil']
        self.arrete = False
        self._verrou = threading.Lock()
        self._reveil = threading.Event()
        self._a_traiter = {}
        self._dernier = {}
        threading.Thread(target=self._boucle, daemon=True).start()

    def soumet(self, camera, image):
        with self._verrou:
            self._a_traiter[camera] = (image.copy(), time.time())
        self._reveil.set()

    def dernier(self, camera):
        """(image, detections, instant de l'image) du dernier aller-retour, ou None."""
        with self._verrou:
            return self._dernier.get(camera)

    def _boucle(self):
        while not self.arrete:
            self._reveil.wait()
            with self._verrou:
                en_attente, self._a_traiter = self._a_traiter, {}
                self._reveil.clear()
            for camera, (image, instant) in en_attente.items():
                detections = self._aller_retour(camera, image)
                with self._verrou:
                    if detections is None:
                        self.arrete = True
                        return
                    self._dernier[camera] = (image, detections, instant)

    def _aller_retour(self, camera, image):
        h, w = image.shape[:2]
        try:
            self.proc.stdin.write(
                (json.dumps({'h': h, 'w': w, 'camera': camera}) + '\n').encode())
            self.proc.stdin.write(np.ascontiguousarray(image).tobytes())
            self.proc.stdin.flush()
            ligne = self.proc.stdout.readline()
        except (BrokenPipeError, OSError, ValueError):
            return None
        return json.loads(ligne) if ligne else None

    def ferme(self):
        self.arrete = True
        self._reveil.set()
        if self.proc.poll() is None:
            self.proc.stdin.close()
            self.proc.wait(timeout=5)


# Les deux `Vision` vivantes, par camera : la triangulation a besoin des DEUX
# extrinseques au meme instant, et `Vision.objets` ne connait que la sienne.
_VISIONS = {}
_DERNIER = {}
# Au-dela, le point triangule n'est pas la piece : un rayon rasant mal apparie
# sort n'importe ou. Les pieces font 30 a 50 mm, les bacs 30.
Z_PLAUSIBLE_MM = (-10.0, 80.0)
FRAICHEUR_AUTRE_VUE_S = 1.0
# Ce que la triangulation a mesure en dernier, par classe : {classe: z mm}.
# Lu par le tableau de bord et par les diagnostics ; ne commande rien.
HAUTEURS_MESUREES = {}


def _rayon(vision, uv):
    """Rayon de la camera vers ce pixel, dans le repere base, en metres."""
    p = cv2.undistortPoints(np.array([[list(uv)]], float),
                            vision.K, vision.dist).reshape(2)
    return pixel_ray(p[0], p[1], np.eye(3), np.linalg.inv(vision.T))


def hauteur_triangulee(classe, camera, boite):
    """Z du centre de la piece, vu par les DEUX cameras, en mm — ou None.

    Pourquoi seulement le Z, alors que la triangulation rend un point complet :
    l'arducam est quasi zenithale et la SVPRO rasante. Une erreur de hauteur de
    20 mm deplace le XY de l'arducam d'environ 1,8 mm, et celui de la SVPRO de
    12,5 mm — c'est le meme calcul qui explique les 48 a 57 mm d'ecart mesures
    entre les deux vues. Prendre le XY triangule ferait donc entrer le biais
    hors-plan de la SVPRO, dont l'extrinseque n'est validee QUE dans le plan
    (leave-one-out 2,4 a 5,9 mm a Z = 0, rien au-dessus).

    On garde donc le XY de l'arducam, mais projete a la hauteur REELLE au lieu
    d'un plan fixe a 24 mm. Les deux cameras servent a mesurer cette hauteur,
    que l'arducam seule ne peut pas donner.

    Mesure du 22/09 sur cinq pieces : Z a +/- 7 mm de la mi-hauteur du dossier
    de fabrication.
    """
    autre = 'svpro' if camera == 'arducam' else 'arducam'
    vision_autre, vu_autre = _VISIONS.get(autre), _DERNIER.get(autre)
    if vision_autre is None or vu_autre is None:
        return None
    if time.time() - vu_autre[2] > FRAICHEUR_AUTRE_VUE_S:
        return None
    jumelle = next((d for d in vu_autre[1] if d['classe'] == classe), None)
    if jumelle is None:
        return None
    a, b, c, e = boite
    j = jumelle['boite']
    try:
        xyz = triangulate([_rayon(_VISIONS[camera], ((a + c) / 2.0, (b + e) / 2.0)),
                           _rayon(vision_autre, ((j[0] + j[2]) / 2.0, (j[1] + j[3]) / 2.0))])
    except (ValueError, np.linalg.LinAlgError, KeyError):
        return None
    z = float(xyz[2]) * 1000.0
    if not Z_PLAUSIBLE_MM[0] <= z <= Z_PLAUSIBLE_MM[1]:
        return None
    HAUTEURS_MESUREES[classe] = z
    return z


def _contour(boite):
    """La boite du detecteur, telle quelle, au format contour d'OpenCV."""
    a, d, b, e = boite
    return np.array([[[a, d]], [[b, d]], [[b, e]], [[a, e]]], np.int32)


def _centre_base(vision, boite, z_mm):
    """Centre de la boite, projete dans le repere base par l'extrinseque."""
    a, d, b, e = boite
    return np.asarray(vision.vers_base(((a + b) / 2.0, (d + e) / 2.0), z_mm)[:2], float)


def _retient(detections, noms):
    """Detections de ces classes, la plus sure d'abord.

    Aucun filtre geometrique. Le modele est entraine sur ces 8 classes et rien
    d'autre : il ne sort ni le bras, ni un cable, ni un marqueur, et sa
    CONFIANCE est le seul juge. Les filtres d'aire, de plateau, de silhouette et
    de recouvrement de `yolo_dashboard.py` existaient pour YOLOE, qui devinait un
    nom parmi 4 585 et avait besoin qu'on ecarte ses trouvailles absurdes. Les
    quasi-doublons (yolo26 est sans NMS) sont deja retires dans le service.
    """
    return [d for d in sorted(detections, key=lambda d: -d['conf']) if d['classe'] in noms]


def juge_objets(vision, image, detections, module, angles=None, marqueurs=None):
    """[(classe, xy base mm, contour)] — ce que rendait `Vision.objets`.

    Le XY est celui du MILIEU de la piece : le centre de la boite projete a la
    hauteur mesuree par les deux cameras, et non sur un plan fixe a 24 mm. A
    defaut de seconde vue fraiche, on retombe sur ce plan fixe.
    """
    camera = 'arducam' if vision.source.startswith('arducam') else 'svpro'
    rendus = []
    for d in _retient(detections, OBJETS):
        z = hauteur_triangulee(d['classe'], camera, d['boite'])
        rendus.append((d['classe'],
                       _centre_base(vision, d['boite'],
                                    module.HAUTEUR_OBJET if z is None else z),
                       _contour(d['boite'])))
    return rendus


def _boite_arducam(classe):
    """Boite de cette classe vue par l'arducam, si la vue est fraiche."""
    vu = _DERNIER.get('arducam')
    if vu is None or _VISIONS.get('arducam') is None:
        return None
    if time.time() - vu[2] > FRAICHEUR_AUTRE_VUE_S:
        return None
    return next((d['boite'] for d in vu[1] if d['classe'] == classe), None)


def juge_bacs(vision, image, detections, module, angles=None, marqueurs=None):
    """[(classe, xy base mm, contour, hauteur du rebord)] — ce que rendait `Vision.cartons`.

    **L'ARDUCAM FAIT FOI SUR LE XY, la SVPRO reste en appui.** Un bac a 30 mm
    de haut, et projeter son rebord sur le plan de la table decale le XY de
    30/tan(elevation) : l'arducam est a ~85 deg, soit 2,6 mm ; la SVPRO a ~58,
    soit 18,7. Mesure du 22/09, les deux vues placent le meme bac a 8,6 - 31,3 mm
    l'une de l'autre, et c'est exactement ce qui a fait larguer le cylindre vert
    au bord de son bac, d'ou il a roule dehors.

    L'appui de la SVPRO n'est pas retire, il est remis a sa place : quand elle
    voit un bac, on garde SA detection comme preuve de presence et son contour
    pour l'affichage, mais on prend le XY de l'arducam si celle-ci l'a vu. La
    SVPRO ne fournit le XY que lorsque l'arducam ne voit rien — le bras qui
    masque la vue de dessus reste couvert, sans payer la parallaxe le reste du
    temps.

    Le rebord reste la valeur du dossier de fabrication (30 mm) et non une
    triangulation : mesuree sur les quatre bacs le 22/09, elle rend -6,3 a
    21,1 mm, trop bruitee pour commander une hauteur de lacher.
    """
    camera = 'arducam' if vision.source.startswith('arducam') else 'svpro'
    rendus = []
    for d in _retient(detections, BACS):
        boite, vue = d['boite'], vision
        if camera != 'arducam':
            depuis_haut = _boite_arducam(d['classe'])
            if depuis_haut is not None:
                boite, vue = depuis_haut, _VISIONS['arducam']
        rendus.append((d['classe'], _centre_base(vue, boite, HAUTEUR_BAC),
                       _contour(d['boite']), HAUTEUR_BAC))
    return rendus


def _couleur(classe):
    teinte = tc.BGR[(OBJETS | BACS)[classe]]
    return teinte if classe in OBJETS else tuple(int(v * 0.55) for v in teinte)


_POLICE = cv2.FONT_HERSHEY_DUPLEX
# Les vues sont reduites par le dashboard avant affichage : ce qui se lit sur la
# trame 640x480 ne se lit plus a l'ecran. Mesure a l'oeil sur les deux panneaux,
# 0,42 en Hershey simple etait illisible une fois reduit.
_ECHELLE = 0.38
_MARGE = 3          # air autour du texte dans sa pastille
_TRAIT = 2


def _texte_sur(couleur):
    """Noir ou blanc, selon ce qui se lit sur cette pastille (luminance BT.601)."""
    b, v, r = couleur
    return (0, 0, 0) if 0.114 * b + 0.587 * v + 0.299 * r > 140 else (255, 255, 255)


def _pose_libre(ancres, occupes, taille):
    """Premiere ancre ou la pastille ne recouvre rien de deja ecrit.

    Huit pieces serrees sur une planche vue de biais, ce sont huit etiquettes
    qui se marchent dessus : sur la SVPRO « cylindre_vert » se posait en travers
    de « cube_bleu » et aucun des deux ne se lisait.
    """
    largeur, hauteur = taille
    for x, y in ancres:
        rect = (x, y, x + largeur, y + hauteur)
        if not any(rect[0] < o[2] and o[0] < rect[2]
                   and rect[1] < o[3] and o[1] < rect[3] for o in occupes):
            return rect
    x, y = ancres[0]
    return (x, y, x + largeur, y + hauteur)


def _pastille(image, x, y, texte, fond, echelle=_ECHELLE):
    """Texte plein sur fond plein. Un contour de texte ne tient pas sur une
    photo de plan de travail : le bois, les cables et le sol gris passent par
    toutes les luminances."""
    (lt, ht), base = cv2.getTextSize(texte, _POLICE, echelle, 1)
    cv2.rectangle(image, (x, y), (x + lt + 2 * _MARGE, y + ht + base + 2 * _MARGE),
                  fond, -1)
    cv2.putText(image, texte, (x + _MARGE, y + ht + _MARGE), _POLICE, echelle,
                _texte_sur(fond), 1, cv2.LINE_AA)
    return lt + 2 * _MARGE, ht + base + 2 * _MARGE


def dessine_detections(module, service):
    """Les boites de yolo26 et leur confiance, tracees sur les deux vues.

    Le dashboard ne dessine plus les siennes (voir `branche`) : ce calque est le
    seul, et il montre la detection COURANTE, pas un polygone lisse.
    """
    origine = module.Fenetre._affiche

    def _affiche(self, nom, image):
        vu = service.dernier(nom)
        if vu is not None:
            detections = vu[1]
            hauteur_vue, largeur_vue = image.shape[:2]
            occupes = []
            for d in sorted(detections, key=lambda d: (d['boite'][1], d['boite'][0])):
                a, b, c, e = d['boite']
                couleur = _couleur(d['classe'])
                cv2.rectangle(image, (a, b), (c, e), couleur, _TRAIT, cv2.LINE_AA)
                texte = f"{d['classe']} {d['conf']:.2f}"
                (lt, ht), base = cv2.getTextSize(texte, _POLICE, _ECHELLE, 1)
                taille = (lt + 2 * _MARGE, ht + base + 2 * _MARGE)
                # Collee au bord haut de la boite, puis dedans, puis sous elle,
                # puis a sa droite : la premiere place libre gagne.
                ancres = [(a, b - taille[1]), (a, b), (a, e), (c + 2, b),
                          (a, b + (e - b) // 2)]
                ancres = [(min(max(x, 0), largeur_vue - taille[0]),
                           min(max(y, 0), hauteur_vue - taille[1])) for x, y in ancres]
                x0, y0, x1, y1 = _pose_libre(ancres, occupes, taille)
                occupes.append((x0, y0, x1, y1))
                _pastille(image, x0, y0, texte, couleur)
            vus = len({d['classe'] for d in detections})
            _pastille(image, 8, 8, f'{nom}  {vus}/8 pieces',
                      (70, 150, 70) if vus == 8 else (40, 110, 190), 0.45)
        origine(self, nom, image)

    module.Fenetre._affiche = _affiche


def attend_image_fraiche(service, camera, depuis, patience=PATIENCE_FRAICHE_S):
    """Vrai des qu'un resultat yolo26 porte sur une image prise apres `depuis`."""
    limite = time.time() + patience
    while time.time() < limite and not service.arrete:
        vu = service.dernier(camera)
        if vu is not None and vu[2] >= depuis:
            time.sleep(ATTENTE_RAFRAICHI_S)
            return True
        time.sleep(0.03)
    return False


def _renomme_le_banc(module, inventaire=None):
    """Les huit noms du dossier a la place de balle/scotch/robot et grand/petit.

    Les dictionnaires du module sont MUTES, jamais reassignes : le dashboard en
    a deja capture des references a l'import.
    """
    module.INVENTAIRE.clear()
    module.INVENTAIRE.update({piece: 1 for piece in OBJETS})
    module.INVENTAIRE.update(inventaire or {})
    module.DESTINATION.clear()
    module.DESTINATION.update(DESTINATION)
    module.COULEUR_OBJET.clear()
    module.COULEUR_OBJET.update({piece: tc.BGR[c] for piece, c in OBJETS.items()})
    module.COULEUR_CARTON.clear()
    # Le bac dans sa couleur, assombri : sur l'image, la piece et son bac se
    # touchent souvent, deux traits de la meme teinte ne se distinguent plus.
    module.COULEUR_CARTON.update(
        {bac: tuple(int(v * 0.55) for v in tc.BGR[c]) for bac, c in BACS.items()})
    # Plus aucun marqueur colle sur les bacs : c'est la classe qui les nomme.
    module.MARQUEUR_CARTON.clear()
    module.HAUTEUR_CARTON = HAUTEUR_BAC


def _renomme_les_cartons(fenetre):
    """« carton » -> « bac » dans l'interface. Il n'y a plus de cartons.

    Les libelles sont poses a la construction par `pick_dashboard`, qui parle
    encore des deux boites en carton du banc d'aout. On les reecrit apres coup :
    un mot qui ne designe plus rien de ce qui est sur la table se lit comme une
    erreur, et il s'en est deja suivi une.

    Ce qui ne peut pas etre repris ici : les noms d'etats du graphe et les
    messages de journal, qui viennent de `pick_fsm` et de `pick_dashboard`
    eux-memes.
    """
    from PyQt5.QtWidgets import QGroupBox, QLabel, QPushButton

    def reecrit(texte):
        for avant, apres in (('cartons', 'bacs'), ('carton', 'bac'),
                             ('Cartons', 'Bacs'), ('Carton', 'Bac'),
                             ('CARTON', 'BAC')):
            texte = texte.replace(avant, apres)
        return texte

    for genre in (QPushButton, QLabel, QGroupBox):
        for objet in fenetre.findChildren(genre):
            lire = objet.title if genre is QGroupBox else objet.text
            ecrire = objet.setTitle if genre is QGroupBox else objet.setText
            if 'arton' in lire():
                ecrire(reecrit(lire()))
            if genre is not QGroupBox and 'arton' in (objet.toolTip() or ''):
                objet.setToolTip(reecrit(objet.toolTip()))
    # Il n'y a plus de balle : le bouton qui designe un bac en y posant la
    # balle n'a plus de mire.
    for bouton in fenetre.findChildren(QPushButton):
        if 'position balle' in bouton.text():
            bouton.hide()


def branche(module, service=None, inventaire=None, poids=None, seuil=None):
    """Remplace objets, cartons et balle des DEUX cameras par yolo26.

    A appeler AVANT `correction_vision.branche` (pour que la carte corrige ce que
    yolo26 rend) et AVANT `Fenetre()` (`ctx.detecteur` et `suivi_cartons` sont
    lies a la construction de la fenetre).
    """
    service = service or ServiceYOLO26(poids, seuil)
    atexit.register(service.ferme)
    _renomme_le_banc(module, inventaire)

    en_panne = []
    # Les marqueurs de la PLANCHE ne bougent jamais : leur dernier carre vu reste
    # valable quand une main ou une piece les cache.
    planche_vue = {}

    def _camera(vision):
        return 'arducam' if vision.source.startswith('arducam') else 'svpro'

    def _detections(vision, image, marqueurs):  # noqa: C901
        """(image jugee, detections, marqueurs cumules) ou None si rien encore."""
        camera = _camera(vision)
        service.soumet(camera, image)
        if service.arrete:
            if not en_panne:
                en_panne.append(True)
                print('yolo26_service.py arrete — plus aucune detection', flush=True)
            return None
        planche_vue.update({i: c for i, c in (marqueurs or {}).items()
                            if i in module._MARQUEURS})
        vu = service.dernier(camera)
        if vu is None:
            return None
        _VISIONS[camera], _DERNIER[camera] = vision, vu
        return vu[0], vu[1], {**planche_vue, **(marqueurs or {})}

    def objets(self, image, angles=None, marqueurs=None):
        vu = _detections(self, image, marqueurs)
        if vu is None:
            return []
        return juge_objets(self, vu[0], vu[1], module, angles, vu[2])

    def cartons(self, image, angles=None, objets=(), marqueurs=None, connus=None):
        # `objets` et `connus` ne servent plus : ils levaient l'ambiguite entre
        # deux cartons de meme gabarit et entre un carton et un objet sombre.
        # Une classe nommee ne se confond avec rien.
        vu = _detections(self, image, marqueurs)
        if vu is None:
            return []
        return juge_bacs(self, vu[0], vu[1], module, angles, vu[2])

    def balle(self, image):
        return None

    def points_interessants(self, image, angles=None, marqueurs=None):
        """Ce que la SVPRO voit, par yolo26 — positions seulement, sans nommer.

        Elle reprend son role d'appui : quand le bras masque la vue de dessus,
        elle voit encore, et la machine n'a plus a se degager pour savoir s'il
        reste quelque chose. Ce qui a change, c'est la SOURCE : cette methode
        appelait `_creux_candidats`, c'est-a-dire le seuil d'Otsu sur le coeur
        sombre — le dernier HSV qui tournait, sur la vue ou il marche le plus
        mal, et qui dessinait un second rectangle sur chaque bac.

        Elle ne nomme toujours rien : `_nomme_par_arducam` rapproche chaque
        tache de ce que l'arducam a identifie. Et ses positions restent des
        positions D'APPUI : mesure du 22/09, elle place une piece 48 a 57 mm a
        cote de l'arducam, parce que sa vue est rasante et que l'extrinseque ne
        vaut qu'a Z = 0 alors que ces pieces font 30 a 50 mm de haut. C'est
        assez pour dire « il y a quelque chose la », pas pour viser une saisie.
        """
        vu = _detections(self, image, marqueurs)
        if vu is None:
            return []
        return ([(xy, contour) for _, xy, contour
                 in juge_objets(self, vu[0], vu[1], module, angles, vu[2])]
                + [(xy, contour) for _, xy, contour, _
                   in juge_bacs(self, vu[0], vu[1], module, angles, vu[2])])

    module.Vision.objets = objets
    module.Vision.cartons = cartons
    module.Vision.balle = balle
    module.Vision.points_interessants = points_interessants

    origine_init = module.Fenetre.__init__

    def __init__(self, *a, **kw):
        origine_init(self, *a, **kw)
        # Quatre bacs au lieu des deux cartons cables dans le constructeur.
        self.suivi_cartons = {bac: module.SuiviCarton() for bac in BACS}
        # `Contexte.carton_vise` vaut 'grand' par defaut : ce nom n'existe plus,
        # et `_suivi_vise` indexe `suivi_cartons` avec, a chaque battement de
        # l'horloge. Il est remplace des qu'un objet est choisi
        # (`carton_vise = DESTINATION[classe]`) — ce n'est qu'une amorce.
        self.ctx.carton_vise = next(iter(BACS))
        # L'info-bulle des vues promet encore la designation du GRAND carton,
        # qui ne veut plus rien dire : les bacs portent leur nom.
        for vue in self.vues.values():
            vue.setToolTip('les bacs sont nommes par leur couleur — '
                           'rien a designer a la main')
        _renomme_les_cartons(self)

    module.Fenetre.__init__ = __init__

    def _designe_grand(self, camera, u, v):
        self.statusBar().showMessage(
            'les bacs sont nommes par leur couleur — rien a designer a la main')

    module.Fenetre._designe_grand = _designe_grand

    origine_detecte = module.Fenetre._detecte_objet

    def _detecte_objet(self, *a, **kw):
        choix = kw.get('exige_dessus') or not self.ctx.classe_objet
        if choix and not en_panne and threading.current_thread() is not threading.main_thread():
            attend_image_fraiche(service, 'arducam', time.time())
        return origine_detecte(self, *a, **kw)

    module.Fenetre._detecte_objet = _detecte_objet

    # UNE seule boite et UNE seule etiquette par piece. Le dashboard tracait
    # deja les siennes — contour pour les objets, rectangle suivi et libelle
    # « nom x,y » pour les bacs — et le calque yolo26 s'y ajoutait : deux
    # carres et deux libelles par piece. Ce sont celles du dashboard qui
    # partent : elles ne portent pas la confiance, et pour un bac c'est le
    # polygone LISSE qu'elles montrent, pas la detection courante.
    module.Fenetre._dessine_objets = lambda self, image, objets: None

    def _dessine_cartons(self, image):
        """Plus que la croix du point de largage, sans rectangle ni libelle.

        Ce point-la n'est pas une detection : c'est ce que la machine VISE, et
        il n'apparait nulle part ailleurs.
        """
        vise = self.ctx.carton_xy
        if vise is None or self.vision is None:
            return
        uv = self.vision.vers_pixel([vise[0], vise[1], module.HAUTEUR_CARTON])
        cv2.drawMarker(image, tuple(uv.astype(int)), (255, 0, 255),
                       cv2.MARKER_TILTED_CROSS, 18, 2)

    module.Fenetre._dessine_cartons = _dessine_cartons

    def _suivi_vise(self):
        """Le bac ou l'objet EN COURS doit aller — recalcule, jamais herite.

        `Contexte.carton_vise` vaut 'grand' a la construction, un nom qui
        n'existe plus, et `_suivi_vise` s'en sert des le premier battement
        d'horloge : il faut donc l'amorcer. Mais une amorce qui survit jusqu'au
        largage envoie la piece dans le mauvais bac — constate le 22/09, le
        pave jaune largue dans le bac rouge, a 320 mm de son bac, alors que la
        detection etait juste a quelques millimetres.

        La destination n'a aucune raison d'etre une variable d'etat : elle est
        une FONCTION de la classe tenue. On la recalcule ici, a chaque appel, et
        l'amorce ne peut plus atteindre le largage.
        """
        classe = self.ctx.classe_objet
        attendu = module.DESTINATION.get(classe)
        if attendu is not None and self.ctx.carton_vise != attendu:
            self.ctx.note(f'destination corrigee : {classe} -> {attendu} '
                          f'(etait {self.ctx.carton_vise})')
            self.ctx.carton_vise = attendu
            self.ctx.carton_resolu = None
            self.ctx.carton_xy = None
        return self.suivi_cartons[self.ctx.carton_vise].position()

    module.Fenetre._suivi_vise = _suivi_vise

    # ---- Hauteur de lacher : le plancher etait taille pour un carton --------
    origine_z_largage = fsm.z_largage

    def z_largage(ctx):
        """Lacher a GARDE_LARGAGE au-dessus du rebord, sans le plancher carton.

        `fsm.z_largage` rend `max(Z_LARGAGE, rebord + GARDE_LARGAGE)`. Le
        plancher de 116,9 mm a ete dimensionne quand la cible etait un carton
        dont le rebord mesure 83 mm : 83 + 41,9 = 124,9, le plancher ne mordait
        jamais. Un bac a un rebord de 30 mm, donc 30 + 41,9 = 71,9, et le
        plancher ajoute 45 mm : la piece tombe de 87 mm au lieu de 42.

        Sur un cube ca ne se voit pas. Le cylindre vert, lui, ROULE : largue de
        87 mm pres du bord le 22/09, il a rebondi hors du bac vert, et la
        machine l'a coche comme depose. Les trois autres pieces du meme cycle
        etaient bien placees.

        On ne touche pas au plancher quand le rebord est inconnu : c'est sa
        raison d'etre.
        """
        rebord = getattr(ctx, 'z_rebord', None)
        if rebord is None:
            return origine_z_largage(ctx)
        return rebord + fsm.GARDE_LARGAGE

    fsm.z_largage = z_largage

    # ---- Ne pas cocher une piece largue hors de son bac ---------------------
    origine_largage = fsm._largage

    def _largage(ctx):
        """`fsm._largage` coche l'inventaire des que la pince s'ouvre.

        Le 22/09 le cylindre vert est ressorti de son bac et l'inventaire
        affichait quand meme `cylindre_vert OK` : le crochet ne dit pas « la
        piece est dans le bac », seulement « le cycle est alle au bout ». On
        garde ici la seule verification disponible au moment du lacher — la
        distance entre le point de largage et le centre du bac. Elle ne
        remplace pas un controle visuel apres retrait, qui reste a faire.
        """
        classe, vise = ctx.classe_objet, ctx.carton_vise
        avant = ctx.deposes.get(classe, 0)
        resultat = origine_largage(ctx)
        if not classe or ctx.deposes.get(classe, 0) <= avant:
            return resultat
        resolu = getattr(ctx, 'carton_resolu', None)
        if not resolu:
            return resultat
        # `carton_resolu` = (centre DEMANDE, point RETENU, roulis, R). Le point
        # retenu n'est pas le centre quand l'IK n'atteint pas celui-ci :
        # `carton_atteignable` prend alors le point de l'ouverture le plus
        # proche du robot. C'est cet ecart-la qui compte.
        centre, point = np.asarray(resolu[0], float), np.asarray(resolu[1], float)
        ecart = float(np.linalg.norm(point - centre))
        if ecart > DEMI_COTE_BAC_MM:
            ctx.deposes[classe] = avant
            ctx.note(f'{classe} largue a {ecart:.0f} mm du centre de {vise}, '
                     f'soit hors du bac ({DEMI_COTE_BAC_MM:.0f} mm de demi-cote) — '
                     f'NON coche a l inventaire')
        else:
            ctx.note(f'{classe} largue a {ecart:.0f} mm du centre de {vise}')
        return resultat

    fsm._largage = _largage

    dessine_detections(module, service)
    return service
