"""DREAM a cote du tri reel : le dashboard du protocole Gazebo, Gazebo remplace par le banc.

Protocole reel, etape 9. Branche par `lancer_pick_dashboard_final.py --dream` :

    /usr/bin/python3 scripts/lancer_pick_dashboard_final.py --svpro-dream --dream

YOLO dit ou est l'objet ; DREAM dit ou est la pince. A gauche la fenetre du tri
(`pick_dashboard`, inchangee). A droite, comme dans `tri_dream_dashboard.py` :

  - par saisie : objet selon YOLO, pince selon les codeurs et selon DREAM, et les
    ecarts entre eux (plan XY, mm) ;
  - dans le temps, un cycle par piece (du depart vers la piece a la fin de son
    depot) : trois distances pince -> cible. Sans verite Gazebo, la reference est
    ce que le robot a FAIT : la cible est la pointe des codeurs au serrage (ou
    etait la piece), puis a la pose (ou est le fond du bac). Courbes :
      codeurs : pointe codeurs -> cible de reference ;
      YOLO    : pointe codeurs -> cible YOLO (piece puis bac) ;
      DREAM   : pointe DREAM   -> cible de reference.

Pointe DREAM, comme `tri_dream_dashboard.dream_point` : par camera, PnP entre les
7 keypoints DREAM et la FK des codeurs a l'instant de l'image (T_DREAM), puis
inv(T_ref) . T_DREAM . pointe, ou T_ref est l'extrinseque ArUco de la camera
(`arducam_extrinsic_pick`, `svpro_extrinsic_labels` qui ne sert qu'a ca). Vues a
>= 5 keypoints seulement, mediane des vues de la derniere seconde.

Rien n'est change dans le tri : les images sont celles que `pick_dashboard` lit
deja (un mandataire sur ses captures), les codeurs passent par la connexion du
tri sous un verrou (le pont est mono-client), les evenements viennent des
messages de `tri_fluide`. Chaque essai est aussi enregistre au format de
`enregistre_tri_dream.py` (images ~5 Hz, codeurs ~8 Hz, `session.json`) pour
les tableaux statique / dynamique de `analyse_tri_dream.py`, et par saisie dans
`tri_dream.csv`.
"""
import builtins
import csv
import json
import subprocess
import sys
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np
import pyqtgraph as pg
import yaml
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap
from PyQt5.QtWidgets import (QComboBox, QGridLayout, QGroupBox, QHBoxLayout, QHeaderView, QLabel,
                             QPlainTextEdit, QScrollArea, QTableWidget, QTableWidgetItem, QTabWidget,
                             QVBoxLayout, QWidget)

RACINE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(RACINE / 'training' / 'dream'))
sys.path.insert(0, str(RACINE / 'mycobot_gateway' / 'mycobot_gateway' / 'vision'))
import camera_registry as registre  # noqa: E402
import pick_fsm as fsm  # noqa: E402
import tri_fluide  # noqa: E402
import yolo26_dashboard  # noqa: E402
from mycobot_fk import KEYPOINT_NAMES, forward_kinematics  # noqa: E402

VENV_DREAM = Path.home() / 'ros_jazzy' / 'venv_dream' / 'bin' / 'python'
SERVICE = RACINE / 'scripts' / 'dream_service.py'
CALIB = RACINE / 'training' / 'calibration'
EXTRINSEQUES = {'arducam': 'arducam_extrinsic_pick.yaml', 'svpro': 'svpro_extrinsic_labels.yaml'}
SORTIES = RACINE / 'results' / 'dream_reel'
PERIODE_IMAGE_S = 0.2
PERIODE_CODEURS_S = 0.12
ECART_CODEURS_MAX_S = 0.3
# Un PnP sur 4 keypoints est mal conditionne : les poses DREAM aberrantes en viennent (Gazebo).
KP_MIN = 5
FUSION_S = 1.0
# Au-dela, les deux vues se contredisent et l'arducam est gardee seule. Essai 7 (15/10) :
# la SVPRO decroche au-dessus des bacs (79 mm), hors des portees d'entrainement, et la
# mediane de 2 vues (= leur moyenne) emmenait le combine a 46 mm ; 30 mm garde la
# moyenne la ou elle aide (6,5 mm vers le cylindre contre 12,3 arducam seule).
ACCORD_MM = 30.0
PORTEE_MAX_MM = 1000.0
# J5 couvert par real_tri/train (46-92 deg). Hors de cette plage — pose d'observation,
# J5 ~ 2 deg — link6 est faux et la pointe DREAM part a 240-390 mm (mediane, 14/10),
# contre 13-19 mm dedans. Ces vues sont enregistrees mais n'entrent ni dans les
# courbes ni dans le trace ni dans la saisie. A elargir apres un fine-tuning qui les couvre.
J5_DOMAINE_DEG = (45.0, 93.0)
# Le serrage dure ~1,6 s, pince immobile sur la piece, avant « saisi ».
SERRAGE_S = 2.0
JOURNAL_LIGNES = 300
FENETRE_FREQUENCE_S = 5.0
MIN_TRACE_MM = 0.1
COURBES = {'codeurs': ('codeurs (référence)', (60, 200, 90), Qt.SolidLine),
           'yolo': ('YOLO', (240, 200, 40), Qt.DashLine),
           'dream': ('DREAM', (255, 30, 230), Qt.SolidLine)}
TOUS = 'tous les objets'
TITRE = 'Tri — machine d\'état : Moore'
# Couleurs BGR des traces sur l'image, celles des courbes.
TRACE_BGR = {'codeurs': (90, 200, 60), 'dream': (230, 30, 255), 'yolo': (40, 200, 240)}


class ServiceDREAM:
    """`dream_service.py` en tache de fond : au plus une image en attente par camera."""

    def __init__(self, modele, resultat):
        self.proc = subprocess.Popen([str(VENV_DREAM), str(SERVICE), modele], stdin=subprocess.PIPE,
                                     stdout=subprocess.PIPE, cwd=str(RACINE / 'scripts'))
        pret = json.loads(self.proc.stdout.readline() or b'{"erreur": "dream_service muet"}')
        if 'erreur' in pret:
            raise RuntimeError(pret['erreur'])
        self.resultat = resultat
        self.arrete = False
        self._verrou = threading.Lock()
        self._reveil = threading.Event()
        self._a_traiter = {}
        threading.Thread(target=self._boucle, daemon=True).start()

    def soumet(self, camera, image, t):
        with self._verrou:
            self._a_traiter[camera] = (image, t)
        self._reveil.set()

    def _boucle(self):
        while not self.arrete:
            self._reveil.wait()
            with self._verrou:
                en_attente, self._a_traiter = self._a_traiter, {}
                self._reveil.clear()
            for camera, (image, t) in en_attente.items():
                h, w = image.shape[:2]
                try:
                    self.proc.stdin.write((json.dumps({'h': h, 'w': w, 'camera': camera}) + '\n').encode())
                    self.proc.stdin.write(np.ascontiguousarray(image).tobytes())
                    self.proc.stdin.flush()
                    ligne = self.proc.stdout.readline()
                except (BrokenPipeError, OSError, ValueError):
                    ligne = b''
                if not ligne:
                    self.arrete = True
                    return
                self.resultat(camera, t, json.loads(ligne))

    def ferme(self):
        self.arrete = True
        self._reveil.set()
        if self.proc.poll() is None:
            self.proc.stdin.close()
            self.proc.wait(timeout=5)


class Mandataire:
    """La capture de `pick_dashboard`, qui passe aussi chaque image a DREAM et a l'enregistrement."""

    def __init__(self, cap, camera, panneau):
        self._cap, self._camera, self._panneau = cap, camera, panneau

    def read(self):
        ok, image = self._cap.read()
        if ok:
            self._panneau.image(self._camera, image, time.monotonic())
        return ok, image

    def __getattr__(self, nom):
        return getattr(self._cap, nom)


class Panneau:
    def __init__(self, modele):
        self.verrou_pont = threading.RLock()
        self.verrou = threading.Lock()
        self.dossier = SORTIES / datetime.now().strftime('%Y-%m-%d_%H-%M-%S_dashboard')
        self.ref = {}
        for cam, nom in EXTRINSEQUES.items():
            d = yaml.safe_load((CALIB / nom).read_text())
            K, dist = registre.load_intrinsics(d['intrinsics_stem'])
            self.ref[cam] = (np.array(d['T_cam_world'], float), K, dist)
        self.codeurs_t, self.codeurs_q = [], []
        self.images = {cam: [] for cam in EXTRINSEQUES}
        self.dernier_enregistre = {cam: 0.0 for cam in EXTRINSEQUES}
        self.derniere = {}
        self.pointes = []          # (t, camera, pointe DREAM mm, dans le domaine J5)
        self.en_attente = deque()
        self.inference_ms, self.latence_ms = deque(maxlen=200), deque(maxlen=200)
        self.cycles = []
        self.mesures = {}          # (source, camera) -> instants de chaque mesure (monotonic)
        # Ce que YOLO et DREAM ont calcule EN DIRECT, pour les CSV : les recalculer apres le
        # tri sur des milliers d'images montait le PC a 96-100 degres (15/10).
        self.yolo_direct, self.kp_direct = [], []
        self.journal = deque(maxlen=JOURNAL_LIGNES)
        self.n_journal = 0
        self.ctx = None
        self.service = ServiceDREAM(modele, self.resultat)
        self.modele = modele

    # ---------------------------------------------------------------- entrees
    def image(self, camera, image, t):
        if camera not in self.ref:
            return
        copie = image.copy()
        self.derniere[camera] = copie
        self.service.soumet(camera, copie, t)
        if t - self.dernier_enregistre[camera] >= PERIODE_IMAGE_S:
            self.dernier_enregistre[camera] = t
            dossier = self.dossier / camera
            dossier.mkdir(parents=True, exist_ok=True)
            nom = f'{len(self.images[camera]):05d}.jpg'
            cv2.imwrite(str(dossier / nom), image, [cv2.IMWRITE_JPEG_QUALITY, 95])
            self.images[camera].append({'t': t, 'image': nom})

    def angles_a(self, t):
        with self.verrou:
            tc = np.array(self.codeurs_t)
            if len(tc) < 2:
                return None
            i = int(np.searchsorted(tc, t))
            if i == 0 or i == len(tc) or tc[i] - t > ECART_CODEURS_MAX_S or t - tc[i - 1] > ECART_CODEURS_MAX_S:
                return None
            a = (t - tc[i - 1]) / (tc[i] - tc[i - 1])
            return (1 - a) * self.codeurs_q[i - 1] + a * self.codeurs_q[i]

    def compte(self, source, camera):
        with self.verrou:
            self.mesures.setdefault((source, camera), []).append(time.monotonic())

    def frequence(self, source, camera):
        maintenant = time.monotonic()
        with self.verrou:
            ts = self.mesures.get((source, camera), [])
            n = sum(1 for t in reversed(ts[-500:]) if maintenant - t <= FENETRE_FREQUENCE_S)
        return n / FENETRE_FREQUENCE_S

    def resultat(self, camera, t, r):
        self.compte('dream', camera)
        with self.verrou:
            self.kp_direct.append({'t': t, 'camera': camera, 'kp': r['kp']})
        self.inference_ms.append(r['ms'])
        self.latence_ms.append((time.monotonic() - t) * 1000.0)
        self.en_attente.append((camera, t, r))

    def apparie(self):
        """Une image n'a ses angles qu'une fois encadree par deux lectures des codeurs."""
        while True:
            while self.en_attente:
                camera, t, r = self.en_attente[0]
                with self.verrou:
                    dernier = self.codeurs_t[-1] if self.codeurs_t else 0.0
                if dernier < t and time.monotonic() - t < 2 * ECART_CODEURS_MAX_S:
                    break
                self.en_attente.popleft()
                self.place(camera, t, r)
            time.sleep(0.05)

    def place(self, camera, t, r):
        vus = [k for k, p in enumerate(r['kp']) if p is not None]
        q = self.angles_a(t)
        if q is None or len(vus) < KP_MIN:
            return
        T_ref, K, dist = self.ref[camera]
        pos, _ = forward_kinematics(np.radians(q))
        obj = np.array([pos[KEYPOINT_NAMES[k]] for k in vus], float)
        img = np.array([r['kp'][k] for k in vus], float)
        ok, rvec, tvec = cv2.solvePnP(obj, img, K, dist, flags=cv2.SOLVEPNP_EPNP)
        if not ok:
            return
        rvec, tvec = cv2.solvePnPRefineLM(obj, img, K, dist, rvec, tvec)
        T_dream = np.eye(4)
        T_dream[:3, :3] = cv2.Rodrigues(rvec)[0]
        T_dream[:3, 3] = tvec.ravel()
        pointe = fsm.pointe(q) / 1000.0
        est = (np.linalg.inv(T_ref) @ T_dream @ np.array([*pointe, 1.0]))[:3] * 1000.0
        if np.linalg.norm(est) <= PORTEE_MAX_MM:
            domaine = bool(J5_DOMAINE_DEG[0] <= q[4] <= J5_DOMAINE_DEG[1])
            with self.verrou:
                self.pointes.append((t, camera, est, domaine))

    def lit_codeurs(self):
        while True:
            pont = self.ctx.pont if self.ctx is not None else None
            if pont is not None:
                with self.verrou_pont:
                    try:
                        rep = pont._envoie_brut('get_angles')
                    except (OSError, ConnectionError, ValueError):
                        rep = ''
                t = time.monotonic()
                if 'ANGLES' in rep.upper():
                    try:
                        q = np.asarray(json.loads(rep.split(':', 1)[-1].strip()), float)
                    except (json.JSONDecodeError, ValueError):
                        q = None
                    if q is not None and q.size == 6:
                        with self.verrou:
                            self.codeurs_t.append(t)
                            self.codeurs_q.append(q)
            time.sleep(PERIODE_CODEURS_S)

    def pointe_codeurs(self, t):
        q = self.angles_a(t)
        if q is None:
            with self.verrou:
                if not self.codeurs_q:
                    return None
                i = int(np.argmin(np.abs(np.array(self.codeurs_t) - t)))
                q = self.codeurs_q[i]
        return fsm.pointe(q)

    # ------------------------------------------------------------- evenements
    def ecrit(self, *morceaux, **kw):
        builtins.print(*morceaux, **kw)
        texte = ' '.join(str(m) for m in morceaux)
        self.journal.append(texte)
        self.n_journal += 1
        t = time.monotonic()
        if not self.cycles or self.cycles[-1]['fin'] is not None:
            return
        c = self.cycles[-1]
        if texte.startswith(f"{c['piece']} : saisi"):
            c['serre'] = t
            c['ref_piece'] = self.pointe_codeurs(t)
        elif texte.startswith(f"{c['piece']} : pose en"):
            c['pose'] = t
            c['ref_bac'] = self.pointe_codeurs(t)

    def trie(self, original):
        def enveloppe(ctx, piece, xy, bac_xy, variante='actuelle'):
            # Z du centre de la piece triangule par arducam + SVPRO (hauteur_triangulee) ;
            # absent avec --svpro-dream, ou sans seconde vue fraiche : YOLO reste alors en XY.
            z = yolo26_dashboard.HAUTEURS_MESUREES.get(piece)
            self.cycles.append({'piece': piece, 'yolo': np.asarray(xy, float), 'yolo_z': z,
                                'yolo_bac': np.asarray(bac_xy, float), 't0': time.monotonic(),
                                'serre': None, 'pose': None, 'fin': None, 'ref_piece': None,
                                'ref_bac': None, 'verdict': 'en cours'})
            ok = False
            try:
                ok = original(ctx, piece, xy, bac_xy, variante)
                return ok
            finally:
                c = self.cycles[-1]
                c['fin'] = time.monotonic()
                c['verdict'] = 'OK' if ok else 'échec'
                self.ecrit_fichiers()
        return enveloppe

    # --------------------------------------------------------------- calculs
    def dream_entre(self, t0, t1):
        with self.verrou:
            vues = [p for t, _, p, dom in self.pointes if dom and t0 <= t <= t1]
        return (np.median(vues, axis=0), len(vues)) if vues else (None, 0)

    def dream_fusionne(self, t0, t1):
        """[(t, pointe combinee)] dans [t0, t1], comme `DreamTips.fused` en Gazebo : a chaque
        vue, mediane de la DERNIERE vue de chaque camera de moins de FUSION_S. La mediane de
        toutes les vues de la seconde passee trainait derriere le bras en mouvement."""
        with self.verrou:
            pts = sorted((t, cam, p) for t, cam, p, dom in self.pointes if dom)
        return fusionne(pts, t0, t1)

    def cible(self, c, t, source):
        apres = c['serre'] is not None and t >= c['serre']
        ref = c['ref_bac'] if apres else c['ref_piece']
        if source != 'yolo':
            return ref
        xy = c['yolo_bac'] if apres else c['yolo']
        return None if ref is None else np.array([xy[0], xy[1], ref[2]])

    def courbes(self, c):
        fin = c['fin'] if c['fin'] is not None else time.monotonic()
        with self.verrou:
            enc = [(t, fsm.pointe(q)) for t, q in zip(self.codeurs_t, self.codeurs_q) if c['t0'] <= t <= fin]
        out = {k: [] for k in COURBES}
        for t, p in enc:
            for src in ('codeurs', 'yolo'):
                cib = self.cible(c, t, src)
                if cib is not None:
                    out[src].append((t - c['t0'], float(np.linalg.norm(p - cib))))
        for t, p in self.dream_fusionne(c['t0'], fin):
            cib = self.cible(c, t, 'dream')
            if cib is not None:
                out['dream'].append((t - c['t0'], float(np.linalg.norm(p - cib))))
        return out

    def trajectoires(self, c):
        """{'codeurs': [(t, pointe mm)], 'dream': [(t, pointe mm)]} sur le cycle `c`."""
        fin = c['fin'] if c['fin'] is not None else time.monotonic()
        with self.verrou:
            enc = [(t, fsm.pointe(q)) for t, q in zip(self.codeurs_t, self.codeurs_q) if c['t0'] <= t <= fin]
        return {'codeurs': enc, 'dream': self.dream_fusionne(c['t0'], fin)}

    def projette(self, camera, pts_mm):
        T_ref, K, dist = self.ref[camera]
        rvec = cv2.Rodrigues(T_ref[:3, :3])[0]
        return cv2.projectPoints(np.asarray(pts_mm, float).reshape(-1, 3) / 1000.0, rvec,
                                 T_ref[:3, 3], K, dist)[0].reshape(-1, 2)

    def saisie(self, c):
        """(pointe codeurs au serrage, pointe DREAM pendant le serrage, nombre de vues)."""
        if c['serre'] is None:
            return None, None, 0
        dream, n = self.dream_entre(c['serre'] - SERRAGE_S, c['serre'])
        return c['ref_piece'], dream, n

    def erreur_trajet(self, c):
        """Mediane, sur le trajet (depart vers la piece -> pose dans le bac), de |pointe DREAM
        combinee - pointe des codeurs| en 3D a chaque instant : la courbe DREAM de Gazebo."""
        fin = c['pose'] or c['fin'] or time.monotonic()
        e = []
        for t, p in self.dream_fusionne(c['t0'], fin):
            q = self.angles_a(t)
            if q is not None:
                e.append(float(np.linalg.norm(p - fsm.pointe(q))))
        return (float(np.median(e)), len(e)) if e else (None, 0)

    @staticmethod
    def erreur_yolo_objet(c, enc):
        """(erreur mm, '3D' ou 'XY') : 3D quand le Z de YOLO a ete triangule par les deux cameras."""
        if enc is None:
            return None, ''
        if c.get('yolo_z') is None:
            return float(np.linalg.norm(c['yolo'] - enc[:2])), 'XY'
        return float(np.linalg.norm(np.array([*c['yolo'], c['yolo_z']]) - enc)), '3D'

    @staticmethod
    def erreur_yolo_bac(c):
        return None if c['ref_bac'] is None else float(np.linalg.norm(c['yolo_bac'] - np.asarray(c['ref_bac'])[:2]))

    # ------------------------------------------------------------- fichiers
    def ecrit_fichiers(self):
        self.dossier.mkdir(parents=True, exist_ok=True)
        with self.verrou:
            codeurs = [{'t': t, 'q': q.tolist()} for t, q in zip(self.codeurs_t, self.codeurs_q)]
        cycles = [{k: (v.tolist() if isinstance(v, np.ndarray) else v) for k, v in c.items()}
                  for c in self.cycles]
        with self.verrou:
            pointes = [{'t': t, 'camera': cam, 'pointe_mm': p.tolist(), 'dans_domaine': dom}
                       for t, cam, p, dom in self.pointes]
        (self.dossier / 'session.json').write_text(json.dumps({
            'codeurs': codeurs, 'cameras': self.images, 'tri': True, 'modele': self.modele,
            'cycles': cycles, 'pointes_dream': pointes,
            'mesures': {f'{src}_{cam}': ts for (src, cam), ts in list(self.mesures.items())},
            'yolo_direct': list(self.yolo_direct), 'keypoints_direct': list(self.kp_direct)}))
        with (self.dossier / 'trajectoires.csv').open('w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['cycle', 'piece', 'source', 't_s', 'x_mm', 'y_mm', 'z_mm', 'cible',
                        'distance_cible_mm', 'dans_domaine_j5'])
            with self.verrou:
                vues = list(self.pointes)
            for i, c in enumerate(self.cycles, 1):
                fin = c['fin'] if c['fin'] is not None else time.monotonic()
                lignes = [(source, t, p, '') for source, pts in self.trajectoires(c).items() for t, p in pts]
                lignes += [(f'dream_vue_{cam}', t, p, 'oui' if dom else 'non')
                           for t, cam, p, dom in vues if c['t0'] <= t <= fin]
                for source, t, p, dom in sorted(lignes, key=lambda l: (l[0], l[1])):
                    apres = c['serre'] is not None and t >= c['serre']
                    cib = self.cible(c, t, 'dream' if source.startswith('dream') else source)
                    w.writerow([i, c['piece'], source, round(t - c['t0'], 3), *np.round(p, 1),
                                'bac' if apres else 'piece',
                                '' if cib is None else round(float(np.linalg.norm(p - cib)), 1), dom])
        with (self.dossier / 'tri_dream.csv').open('w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['piece', 'yolo_x_mm', 'yolo_y_mm', 'codeurs_x_mm', 'codeurs_y_mm', 'codeurs_z_mm',
                        'dream_x_mm', 'dream_y_mm', 'dream_z_mm', 'dream_codeurs_dx_mm', 'dream_codeurs_dy_mm',
                        'dream_codeurs_dz_mm', 'dream_codeurs_xy_mm', 'dream_codeurs_3d_mm',
                        'yolo_codeurs_xy_mm', 'vues_dream', 'verdict', 'dream_trajet_3d_mm',
                        'yolo_bac_codeurs_xy_mm', 'yolo_z_mm', 'yolo_codeurs_3d_mm'])
            for c in self.cycles:
                enc, dream, n = self.saisie(c)
                trajet, _ = self.erreur_trajet(c)
                bac = self.erreur_yolo_bac(c)
                w.writerow([c['piece'], *np.round(c['yolo'], 1),
                            *(('',) * 3 if enc is None else np.round(enc, 1)),
                            *(('',) * 3 if dream is None else np.round(dream, 1)),
                            *(('',) * 3 if enc is None or dream is None else np.round(dream - enc, 1)),
                            '' if enc is None or dream is None else round(float(np.linalg.norm(dream[:2] - enc[:2])), 1),
                            '' if enc is None or dream is None else round(float(np.linalg.norm(dream - enc)), 1),
                            '' if enc is None else round(float(np.linalg.norm(c['yolo'] - enc[:2])), 1),
                            n, c['verdict'], '' if trajet is None else round(trajet, 1),
                            '' if bac is None else round(bac, 1),
                            '' if c.get('yolo_z') is None else round(c['yolo_z'], 1),
                            '' if enc is None or c.get('yolo_z') is None else
                            round(float(np.linalg.norm(np.array([*c['yolo'], c['yolo_z']]) - enc)), 1)])

    def ferme(self):
        self.service.ferme()
        self.ecrit_fichiers()
        if getattr(self, 'ros_monitor', None) is not None:
            self.ros_monitor.close()


def installe(module, modele='vgg_tri_reel_ft_e10'):
    """A appeler AVANT `module.Fenetre()` : captures, pont et tri sont enveloppes a la construction."""
    panneau = Panneau(modele)
    brut = fsm.Pont.envoie

    def envoie(self, action, **kw):
        with panneau.verrou_pont:
            return brut(self, action, **kw)

    fsm.Pont._envoie_brut = brut
    fsm.Pont.envoie = envoie

    ouvre = module.Fenetre._ouvre_cameras

    def ouvre_et_enveloppe(fenetre, *k, **kw):
        ouvre(fenetre, *k, **kw)
        panneau.ctx = fenetre.ctx
        for nom, cap in fenetre.captures.items():
            if cap is not None:
                fenetre.captures[nom] = Mandataire(cap, nom, panneau)

    module.Fenetre._ouvre_cameras = ouvre_et_enveloppe
    aller_retour = yolo26_dashboard.ServiceYOLO26._aller_retour

    def aller_retour_compte(service, camera, image):
        t = time.monotonic()
        detections = aller_retour(service, camera, image)
        if detections is not None:
            panneau.compte('yolo', camera)
            with panneau.verrou:
                panneau.yolo_direct.append({'t': t, 'camera': camera, 'det': detections})
        return detections

    yolo26_dashboard.ServiceYOLO26._aller_retour = aller_retour_compte
    tri_fluide.trie = panneau.trie(tri_fluide.trie)
    tri_fluide.print = panneau.ecrit
    threading.Thread(target=panneau.lit_codeurs, daemon=True).start()
    threading.Thread(target=panneau.apparie, daemon=True).start()
    return panneau


def fusionne(pts, t0, t1, fenetre=FUSION_S, accord=ACCORD_MM):
    """pts : [(t, camera, pointe)] tries par t -> [(t, pointe combinee)] pour chaque vue dans
    [t0, t1] : mediane des dernieres vues de chaque camera de moins de `fenetre` s, sauf
    si arducam et SVPRO s'ecartent de plus de `accord` mm -> arducam seule."""
    derniere, out = {}, []
    for t, cam, p in pts:
        derniere[cam] = (t, p)
        if t0 <= t <= t1:
            vues = {c: q for c, (s, q) in derniere.items() if t - s <= fenetre}
            if len(vues) == 2 and np.linalg.norm(vues['arducam'] - vues['svpro']) > accord:
                out.append((t, vues['arducam']))
            else:
                out.append((t, np.median(list(vues.values()), axis=0)))
    return out


def ajuste_lignes(table, lignes):
    table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
    table.setMinimumHeight(table.horizontalHeader().sizeHint().height()
                           + lignes * table.verticalHeader().defaultSectionSize() + 2 * table.frameWidth())


class FenetreTriDream(QWidget):
    """La fenetre du tri a gauche ; DREAM <-> YOLO <-> codeurs a droite."""

    def __init__(self, panneau, fenetre):
        super().__init__()
        self.p, self.f = panneau, fenetre
        self.setWindowTitle(TITRE)
        grille = QGridLayout(self)
        titre = QLabel(TITRE)
        titre.setAlignment(Qt.AlignCenter)
        titre.setStyleSheet('font-size: 18px; font-weight: bold; padding: 4px')
        grille.addWidget(titre, 0, 0, 1, 2)
        fenetre.setWindowFlags(Qt.Widget)
        boite_tri = QGroupBox('Tri réel (arducam + SVPRO, YOLO)')
        # Les libelles longs du tri (inventaire, journal) elargissent sa fenetre en
        # cours de cycle : sans defilement, l'ensemble deborde de l'ecran a droite.
        zone = QScrollArea()
        zone.setWidget(fenetre)
        zone.setWidgetResizable(True)
        QVBoxLayout(boite_tri).addWidget(zone)
        grille.addWidget(boite_tri, 1, 0, 2, 1)

        compare = QGroupBox('DREAM (pince) ↔ YOLO (objet) ↔ codeurs (référence)')
        self.table = QTableWidget(0, 9)
        self.table.setHorizontalHeaderLabels(['objet', 'YOLO objet x, y (z)\n(mm)', 'codeurs pince x, y, z\n(mm)',
                                              'DREAM pince x, y, z\n(mm)', 'DREAM ↔ codeurs\n(3D, trajet, mm)',
                                              'YOLO objet ↔ codeurs\n(saisie, mm)', 'YOLO bac ↔ codeurs\n(XY, dépôt, mm)',
                                              'vues DREAM', 'tri'])
        self.table.setToolTip('mm. DREAM pince 3D : médiane sur le trajet objet → bac. YOLO objet : au '
                              'serrage, 3D si z triangulé arducam + SVPRO, sinon XY. YOLO bac : à la '
                              'pose, XY. Référence : codeurs.')
        ajuste_lignes(self.table, 4)
        self.trace = pg.PlotWidget(labels={'left': 'pince → cible (mm, log)',
                                           'bottom': 'temps depuis le départ vers la pièce (s)'})
        self.trace.setLogMode(y=True)
        self.trace.getAxis('left').enableAutoSIPrefix(False)
        self.trace.addLegend()
        self.trace.setToolTip('Cible = la pièce jusqu au serrage (trait vertical), puis son bac. '
                              'Référence = pointe des codeurs au serrage, puis à la pose.')
        self.serrage = pg.InfiniteLine(angle=90, pen=pg.mkPen((150, 150, 150), style=Qt.DotLine))
        self.trace.addItem(self.serrage)
        self.courbes = {k: self.trace.plot(pen=pg.mkPen(c, width=2, style=s), name=n, connect='finite',
                                           **({'symbol': 'o', 'symbolSize': 5, 'symbolBrush': c}
                                              if k == 'dream' else {}))
                        for k, (n, c, s) in COURBES.items()}
        self.choix = QComboBox()
        self.choix.addItems(['objet en cours', TOUS, *sorted(tri_fluide.PIECES)])
        ligne = QHBoxLayout()
        ligne.addWidget(QLabel('Visualiser :'))
        ligne.addWidget(self.choix)
        ligne.addStretch()
        self.latence = QLabel()
        self.frequences = QLabel()
        self.frequences.setStyleSheet('font-weight: bold')
        self.frequences.setToolTip(f'Nombre de mesures par seconde, sur les {FENETRE_FREQUENCE_S:.0f} '
                                   f'dernières secondes.')
        b = QVBoxLayout(compare)
        b.addWidget(self.frequences)
        b.addWidget(self.table)
        self.onglets = QTabWidget()
        self.onglets.addTab(self.trace, 'distance à la cible')
        self.vues = {}
        for cam in EXTRINSEQUES:
            vue = QLabel()
            vue.setAlignment(Qt.AlignCenter)
            vue.setMinimumSize(320, 240)
            vue.setStyleSheet('background: black')
            vue.setToolTip('Pointe de la pince projetée par l extrinsèque ArUco de la caméra : '
                           'vert = codeurs, magenta = DREAM, jaune = cible YOLO (pièce puis bac).')
            self.onglets.addTab(vue, f'trajectoire ({cam})')
            self.vues[cam] = vue
        b.addWidget(self.onglets)
        b.addLayout(ligne)
        b.addWidget(self.latence)
        grille.addWidget(compare, 1, 1)

        bas = QHBoxLayout()
        boite_journal = QGroupBox('Journal du tri')
        self.journal = QPlainTextEdit()
        self.journal.setReadOnly(True)
        QVBoxLayout(boite_journal).addWidget(self.journal)
        boite_yolo = QGroupBox('YOLO : pièces et bacs (arducam)')
        self.yolo = QTableWidget(0, 3)
        self.yolo.setHorizontalHeaderLabels(['classe', 'x mm', 'y mm'])
        ajuste_lignes(self.yolo, 8)
        QVBoxLayout(boite_yolo).addWidget(self.yolo)
        bas.addWidget(boite_journal)
        bas.addWidget(boite_yolo)
        grille.addLayout(bas, 2, 1)
        grille.setColumnStretch(0, 1)
        grille.setColumnStretch(1, 1)
        grille.setRowStretch(1, 2)
        grille.setRowStretch(2, 1)
        self.n_journal = 0
        self.minuterie = QTimer(self)
        self.minuterie.timeout.connect(self.redessine)
        self.minuterie.start(500)

        self.p.ros_monitor = None
        try:
            from tri_ros_monitor import TriROSMonitor
            self.p.ros_monitor = TriROSMonitor()
            self.ros_minuterie = QTimer(self)
            self.ros_minuterie.timeout.connect(
                lambda: self.p.ros_monitor.publish(self.p, self.f))
            self.ros_minuterie.start(500)
            print('ROS 2 : telemetrie /tri_reel/{codeurs,dream,yolo,etat} active', flush=True)
        except ImportError as exc:
            print(f'ROS 2 indisponible : {exc} (sourcer /opt/ros/jazzy/setup.bash)', flush=True)

    def redessine(self):
        p = self.p
        choix = self.choix.currentText()
        if choix == TOUS:
            montres = list(p.cycles)
        elif choix == 'objet en cours':
            montres = p.cycles[-1:]
        else:
            # Un essai avorte sans mouvement (detection fantome) ne doit pas masquer la vraie saisie.
            essais = [c for c in p.cycles if c['piece'] == choix]
            montres = ([c for c in essais if c['serre'] is not None] or essais)[-1:]
        origine = min((c['t0'] for c in montres), default=0.0)
        self.trace.setTitle(TOUS if choix == TOUS else (montres[0]['piece'] if montres else ''))
        seul = montres[0] if len(montres) == 1 else None
        self.serrage.setVisible(seul is not None and seul['serre'] is not None)
        if seul is not None and seul['serre'] is not None:
            self.serrage.setValue(seul['serre'] - seul['t0'])
        donnees = {k: ([], []) for k in COURBES}
        for c in montres:
            decale = c['t0'] - origine if choix == TOUS else 0.0
            for k, pts in p.courbes(c).items():
                if pts:
                    t, d = zip(*pts)
                    donnees[k][0].extend([*(np.array(t) + decale), np.nan])
                    donnees[k][1].extend([*np.maximum(d, MIN_TRACE_MM), np.nan])
        for k, (t, d) in donnees.items():
            self.courbes[k].setData(np.array(t, float), np.array(d, float))
        vue = self.onglets.currentWidget()
        for cam, label in self.vues.items():
            if label is vue:
                self.dessine_trajectoire(cam, montres)
        if p.inference_ms:
            vues = sum(1 for *_, dom in p.pointes if dom)
            hors = len(p.pointes) - vues
            self.latence.setText(f'DREAM {p.modele} : inférence {np.median(p.inference_ms):.0f} ms, '
                                 f'image → keypoints {np.median(p.latence_ms):.0f} ms ; '
                                 f'{vues} vues ≥ {KP_MIN} keypoints (+{hors} hors domaine J5, écartées) ; '
                                 f'enregistré dans {p.dossier.name}')

        self.frequences.setText('Fréquence de mesure — ' + '   |   '.join(
            f'{src} : ' + ', '.join(f'{cam} {p.frequence(src.lower(), cam):.1f} Hz' for cam in EXTRINSEQUES)
            for src in ('YOLO', 'DREAM')))
        self.table.setRowCount(len(p.cycles))
        for i, c in enumerate(p.cycles):
            enc, dream, n = p.saisie(c)
            trajet, _ = p.erreur_trajet(c)
            bac = p.erreur_yolo_bac(c)
            xyz = (lambda v: '—' if v is None else ', '.join(f'{x:.0f}' for x in v))
            mm = (lambda v: '—' if v is None else f'{v:.1f}')
            e_yolo, dim = p.erreur_yolo_objet(c, enc)
            yolo = c['yolo'] if c.get('yolo_z') is None else [*c['yolo'], c['yolo_z']]
            cellules = (c['piece'], xyz(yolo), xyz(enc), xyz(dream), mm(trajet),
                        '—' if e_yolo is None else f'{e_yolo:.1f} ({dim})',
                        mm(bac), str(n), c['verdict'])
            for j, v in enumerate(cellules):
                self.table.setItem(i, j, QTableWidgetItem(v))

        with self.f._verrou:
            objets = list(self.f._objets_vus)
        lignes = [(n, xy) for n, xy in objets]
        for bac, suivi in self.f.suivi_cartons.items():
            vu = suivi.position()
            if vu is not None:
                lignes.append((bac, vu[0]))
        self.yolo.setRowCount(len(lignes))
        for i, (n, xy) in enumerate(sorted(lignes, key=lambda l: l[0])):
            for j, v in enumerate((n, f'{xy[0]:.1f}', f'{xy[1]:.1f}')):
                self.yolo.setItem(i, j, QTableWidgetItem(v))
        if p.n_journal != self.n_journal:
            self.journal.setPlainText('\n'.join(p.journal))
            self.journal.verticalScrollBar().setValue(self.journal.verticalScrollBar().maximum())
            self.n_journal = p.n_journal

    def dessine_trajectoire(self, cam, montres):
        p = self.p
        image = p.derniere.get(cam)
        if image is None:
            return
        img = image.copy()
        h, w = img.shape[:2]
        for c in montres:
            traj = p.trajectoires(c)
            for source in ('codeurs', 'dream'):
                if len(traj[source]) >= 2:
                    uv = p.projette(cam, [pt for _, pt in traj[source]])
                    uv = uv[np.all(np.isfinite(uv), axis=1)].astype(np.int32)
                    cv2.polylines(img, [uv.reshape(-1, 1, 2)], False, TRACE_BGR[source], 2, cv2.LINE_AA)
                    if source == 'dream':
                        for u, v in uv:
                            cv2.circle(img, (int(u), int(v)), 3, TRACE_BGR[source], -1, cv2.LINE_AA)
            for cib in (c['yolo'], c['yolo_bac']):
                z = 0.0 if c['ref_piece'] is None else c['ref_piece'][2]
                u, v = p.projette(cam, [[cib[0], cib[1], z]])[0]
                if np.isfinite([u, v]).all():
                    cv2.drawMarker(img, (int(u), int(v)), TRACE_BGR['yolo'], cv2.MARKER_CROSS, 18, 2)
        y = 22
        for source, nom in (('codeurs', 'codeurs'), ('dream', 'DREAM'), ('yolo', 'cible YOLO')):
            cv2.putText(img, nom, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, TRACE_BGR[source], 2, cv2.LINE_AA)
            y += 22
        rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        qimg = QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888)
        label = self.vues[cam]
        label.setPixmap(QPixmap.fromImage(qimg).scaled(label.size(), Qt.KeepAspectRatio,
                                                       Qt.SmoothTransformation))

    def closeEvent(self, evenement):
        self.f.close()
        self.p.ferme()
        evenement.accept()
