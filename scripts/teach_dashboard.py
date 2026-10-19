#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Tableau de bord de teach — vue caméra + pilotage directionnel + pince.

Remplace le pilotage au clavier de calibrate_jog.py par une vraie interface :
vue arducam en direct avec le diagnostic de détection, croix directionnelle
(gauche/droite/avant/arrière/haut/bas), pas réglable, slider de pince 0-100,
et la liste des points déjà enseignés avec leur étalement.

Le principe de mesure est celui de calibrate_jog.py : l'orientation du poignet
est FIGÉE (celle du pick validé), donc le déport bride->doigts est une constante
que l'homographie absorbe. On enregistre le couple (pixel de la balle, commande
XY qui la saisit) — exactement ce que le pick rejouera.

    .venv/bin/python scripts/teach_dashboard.py

⚠️ La caméra ne s'ouvre que par UN process : arrête le launch multicam avant.
"""
from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path

os.environ.setdefault('QT_QPA_FONTDIR', '/usr/share/fonts/truetype/dejavu')

import cv2                                                              # noqa: E402
import numpy as np                                                      # noqa: E402

# opencv-python embarque SON Qt et impose son chemin de plugins à l'import. PyQt5
# charge alors le mauvais xcb et meurt en SIGABRT. On repointe donc explicitement
# vers les plugins de PyQt5 APRÈS l'import de cv2, avant tout import Qt.
import PyQt5                                                            # noqa: E402
os.environ['QT_QPA_PLATFORM_PLUGIN_PATH'] = str(
    Path(PyQt5.__file__).resolve().parent / 'Qt5' / 'plugins')

from PyQt5.QtCore import Qt, QTimer                                     # noqa: E402
from PyQt5.QtGui import QImage, QPixmap, QFont                          # noqa: E402
from PyQt5.QtWidgets import (                                           # noqa: E402
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel,
    QPushButton, QGroupBox, QSlider, QListWidget, QMessageBox, QRadioButton, QCheckBox, QDoubleSpinBox,
    QScrollArea,
)

_REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(_REPO / 'scripts'))
sys.path.insert(0, str(_REPO / 'mycobot_gateway'))

from pick_and_place_real import Bridge, GRIPPER_ID, STATUS_TXT          # noqa: E402
from calibrate_pixel_to_base import (open_camera, ball_pixel, save,     # noqa: E402
                                     read_coords, OUT)
from preview_ball_detection import candidates, draw                     # noqa: E402
from diff_ik import (fk_pose, solve_pose, solve_tcp,                    # noqa: E402
                     orientation_error_deg)
from object_localizer import load_extrinsic                             # noqa: E402
from mycobot_gateway.vision.camera_registry import load_intrinsics      # noqa: E402

MIN_POINTS = 6
_EXTRINSIC = _REPO / 'training' / 'calibration' / 'arducam_extrinsic_markers.yaml'
TOOL_FILE = _REPO / 'scripts' / 'tool_offset.json'
# Mêmes bornes que _JOINT_LIMITS de dream_validation_dashboard.py, en degrés.
JOINT_LIMITS_DEG = [(-170, 170), (-160, 160), (-160, 160),
                    (-160, 160), (-170, 170), (-175, 175)]


def project_base_point(xyz_mm, K, T_cam_world):
    """Point du repère base (mm) -> pixel."""
    p_cam = T_cam_world @ np.array([xyz_mm[0] / 1000.0, xyz_mm[1] / 1000.0,
                                    xyz_mm[2] / 1000.0, 1.0])
    if p_cam[2] <= 1e-6:
        return None
    uv = K @ (p_cam[:3] / p_cam[2])
    return np.array([uv[0], uv[1]])


def screen_jacobian(xyz_mm, K, T_cam_world, delta_mm=20.0):
    """Matrice 2x2 : un déplacement base (dX,dY) -> déplacement pixel (du,dv).

    Obtenue par différences finies sur la projection. Sert à piloter dans le
    repère ÉCRAN : la caméra est très oblique, donc « Y+ robot » n'est pas
    « la gauche » de l'image, ce qui rend le pilotage en repère robot déroutant.
    Seule la ROTATION de l'extrinsèque intervient ici — une erreur de translation
    de quelques centimètres ne change pas les directions.
    """
    base = project_base_point(xyz_mm, K, T_cam_world)
    if base is None:
        return None
    columns = []
    for axis in (0, 1):
        moved = list(xyz_mm)
        moved[axis] += delta_mm
        shifted = project_base_point(moved, K, T_cam_world)
        if shifted is None:
            return None
        columns.append((shifted - base) / delta_mm)
    return np.column_stack(columns)


class TeachDashboard(QWidget):
    def __init__(self, args):
        super().__init__()
        self.args = args
        self.scale = args.width / 640.0
        self.step = 5.0
        self.pixel = None
        self.samples = []
        self.tcp_poses = []
        self.tool_offset = None
        if TOOL_FILE.is_file():
            self.tool_offset = np.asarray(
                json.loads(TOOL_FILE.read_text())['tool_offset_mm'], dtype=float)

        self.cap = open_camera(args.width, args.height, args.exposure)
        self.bridge = Bridge(args.pi_host)
        self.bridge.get_angles()
        # L'état de référence est ANGULAIRE : send_coords est refusé ou ignoré
        # silencieusement par le firmware sur beaucoup de poses (bras immobile
        # alors que le bridge répond OK, observé le 17/08). On pilote donc en
        # angles, avec notre propre IK, et xyz est TOUJOURS recalculé par FK
        # depuis les angles réellement lus — donc exact même si l'IK n'atteint
        # pas exactement la consigne.
        self.q = list(self.bridge.get_angles())
        position, self.R_lock = fk_pose(self.q)
        self.xyz = [float(v) for v in position]

        self.K, _ = load_intrinsics('cam_3', args.width, args.height)
        self.T_cam_world = load_extrinsic(_EXTRINSIC)
        self.jacobian = None

        self.setFocusPolicy(Qt.StrongFocus)
        self.setWindowTitle('Teach pick — arducam + pilotage')
        self._build()
        self.timer = QTimer(self)
        self.timer.timeout.connect(self._refresh)
        self.timer.start(60)

    # ---------------------------------------------------------------- UI
    def _build(self):
        root = QHBoxLayout(self)

        self.view = QLabel('caméra…')
        self.view.setMinimumSize(560, 420)
        self.view.setAlignment(Qt.AlignCenter)
        root.addWidget(self.view, stretch=3)

        # Le panneau dépasse la hauteur de l'écran : sans zone défilante, la liste
        # des points et le bouton d'ajustement sont coupés en bas et inatteignables.
        side = QWidget()
        panel = QVBoxLayout(side)
        scroll = QScrollArea()
        scroll.setWidget(side)
        scroll.setWidgetResizable(True)
        scroll.setMinimumWidth(520)
        scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
        root.addWidget(scroll, stretch=2)

        self.lbl_pose = QLabel('—')
        self.lbl_pose.setFont(QFont('monospace', 12, QFont.Bold))
        pose_box = QGroupBox('Position commandée (mm)')
        pose_l = QVBoxLayout(pose_box)
        pose_l.addWidget(self.lbl_pose)
        self.lbl_error = QLabel('')
        self.lbl_error.setStyleSheet('color:#c00;')
        self.lbl_error.setWordWrap(True)
        pose_l.addWidget(self.lbl_error)
        self.chk_arm = QCheckBox('ARMER — autorise les mouvements du bras')
        self.chk_arm.setStyleSheet('font-weight:bold; color:#b40;')
        pose_l.addWidget(self.chk_arm)
        self.chk_screen = QCheckBox('Flèches suivent l\'ÉCRAN (sinon repère robot)')
        self.chk_screen.setChecked(True)
        pose_l.addWidget(self.chk_screen)
        panel.addWidget(pose_box)

        panel.addWidget(self._servo_box())
        panel.addWidget(self._pad_box())
        panel.addWidget(self._wrist_box())
        panel.addWidget(self._step_box())
        panel.addWidget(self._gripper_box())
        panel.addWidget(self._tcp_box())
        panel.addWidget(self._teach_box())
        panel.addStretch()

    def _servo_box(self):
        """Fixer / Relâcher, mêmes commandes que le dashboard de validation.

        Relâché, le bras se déplace à la main : pratique pour l'amener grossièrement
        au-dessus de la planche. Mais tout envoi d'angles exige d'avoir fixé avant,
        sinon la commande part dans le vide."""
        box = QGroupBox('Servos')
        v = QVBoxLayout(box)
        row = QHBoxLayout()
        b_lock = QPushButton('🔒 Fixer (power_on)')
        b_lock.setStyleSheet('background:#c8e6c9;')
        b_lock.setFocusPolicy(Qt.NoFocus)
        b_lock.clicked.connect(self.power_on)
        b_release = QPushButton('🔓 Relâcher (power_off)')
        b_release.setStyleSheet('background:#f0d8c8;')
        b_release.setFocusPolicy(Qt.NoFocus)
        b_release.clicked.connect(self.power_off)
        row.addWidget(b_lock)
        row.addWidget(b_release)
        v.addLayout(row)
        self.lbl_servo = QLabel('État servos : inconnu — clique Fixer avant de commander')
        self.lbl_servo.setWordWrap(True)
        v.addWidget(self.lbl_servo)
        return box

    def power_on(self):
        self.bridge.send({'action': 'power_on'})
        time.sleep(0.4)
        self.resync()
        self.lbl_servo.setText('État servos : 🔒 fixé — prêt à commander')
        self.lbl_servo.setStyleSheet('color:#2e7d32; font-weight:bold;')

    def power_off(self):
        self.bridge.send({'action': 'power_off'})
        self.lbl_servo.setText('État servos : 🔓 relâché — déplaçable à la main. '
                               'TIENS LE BRAS. Clique « Relire la pose » après.')
        self.lbl_servo.setStyleSheet('color:#b40; font-weight:bold;')

    def resync(self):
        """Recale l'état interne sur la pose réelle — après un power_on ou un
        déplacement à la main, self.q ne correspond plus à rien."""
        self.q = list(self.bridge.get_angles())
        position, self.R_lock = fk_pose(self.q)
        self.xyz = [float(v) for v in position]
        self.read_angles_into_edits()

    def _pad_box(self):
        box = QGroupBox('Déplacement  (flèches du clavier aussi)')
        g = QGridLayout(box)
        mk = lambda text, slot: (lambda b: (b.clicked.connect(slot),
                                            b.setMinimumHeight(44),
                                            b.setFocusPolicy(Qt.NoFocus), b)[-1])(QPushButton(text))
        g.addWidget(mk('▲\nhaut', lambda: self.jog(0, +1)), 0, 1)
        g.addWidget(mk('◀\ngauche', lambda: self.jog(1, +1)), 1, 0)
        g.addWidget(mk('▼\nbas', lambda: self.jog(0, -1)), 1, 1)
        g.addWidget(mk('▶\ndroite', lambda: self.jog(1, -1)), 1, 2)
        g.addWidget(mk('⤒  monter (Z+)', lambda: self.jog(2, +1)), 2, 0)
        g.addWidget(mk('⤓  descendre (Z−)', lambda: self.jog(2, -1)), 2, 2)
        g.addWidget(mk('⇧  dégagement +90', self.park), 2, 1)
        return box

    def _wrist_box(self):
        """Angles J1-J6 saisis puis envoyés d'un bloc, comme SET Angles du
        dashboard de validation. Rien ne part tant qu'on ne clique pas : des
        boutons +/- qui envoient à chaque clic font bouger le bras en continu."""
        box = QGroupBox('Angles (°) — J1-J6, sans IK')
        g = QGridLayout(box)
        self.angle_edits = []
        for column in range(6):
            g.addWidget(QLabel(f'J{column+1}'), 0, column, alignment=Qt.AlignCenter)
            spin = QDoubleSpinBox()
            low, high = JOINT_LIMITS_DEG[column]
            spin.setRange(low, high)
            spin.setSingleStep(1.0)
            spin.setDecimals(1)
            spin.setValue(float(self.q[column]))
            spin.setFixedWidth(76)
            spin.setFocusPolicy(Qt.ClickFocus)
            g.addWidget(spin, 1, column)
            self.angle_edits.append(spin)

        b_set = QPushButton('SET Angles')
        b_set.setMinimumHeight(34)
        b_set.setFocusPolicy(Qt.NoFocus)
        b_set.clicked.connect(self.set_angles)
        g.addWidget(b_set, 2, 0, 1, 3)

        b_read = QPushButton('↺ Relire la pose')
        b_read.setFocusPolicy(Qt.NoFocus)
        b_read.clicked.connect(self.resync)
        g.addWidget(b_read, 2, 3, 1, 3)

        self.lbl_wrist = QLabel('—')
        g.addWidget(self.lbl_wrist, 3, 0, 1, 6)
        return box

    def read_angles_into_edits(self):
        for spin, value in zip(self.angle_edits, self.q):
            spin.setValue(float(value))

    def set_angles(self):
        if not self.armed():
            return
        if self.samples:
            QMessageBox.warning(
                self, 'Orientation',
                'Des points sont déjà enregistrés. Changer l\'orientation du '
                'poignet maintenant les rend incohérents avec les suivants.')
        self.send_joints([spin.value() for spin in self.angle_edits])
        # L'orientation atteinte devient celle que l'IK tiendra en translation.
        self.R_lock = fk_pose(self.q)[1]


    def _step_box(self):
        box = QGroupBox('Pas')
        h = QHBoxLayout(box)
        self.step_buttons = []
        for value in (1.0, 5.0, 10.0, 20.0):
            r = QRadioButton(f'{value:.0f} mm')
            r.toggled.connect(lambda on, v=value: on and setattr(self, 'step', v))
            if value == 5.0:
                r.setChecked(True)
            h.addWidget(r)
            self.step_buttons.append(r)
        return box

    def _gripper_box(self):
        box = QGroupBox('Pince (0 = fermée, 100 = ouverte)')
        v = QVBoxLayout(box)
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setRange(0, 100)
        self.slider.setValue(100)
        self.slider.setTickPosition(QSlider.TicksBelow)
        self.slider.setTickInterval(10)
        # sliderReleased et pas valueChanged : le bridge impose 1.6 s entre deux
        # commandes pince, envoyer pendant le glissement bloquerait l'interface.
        self.slider.sliderReleased.connect(self._send_grip)
        self.lbl_grip = QLabel('consigne 100')
        v.addWidget(self.slider)
        v.addWidget(self.lbl_grip)
        h = QHBoxLayout()
        for text, value in (('Ouvrir (100)', 100), ('Serrer (20)', 20)):
            b = QPushButton(text)
            b.clicked.connect(lambda _, x=value: (self.slider.setValue(x), self._send_grip()))
            h.addWidget(b)
        v.addLayout(h)
        self.slider.valueChanged.connect(
            lambda x: self.lbl_grip.setText(f'consigne {x}'))
        return box

    def _tcp_box(self):
        """Calibration du déport bride->doigts, préalable au teach.

        Sans elle on enregistre la position de la BRIDE, qui dépend de
        l'orientation du poignet : mesuré le 17/08, 43 deg d'écart entre points
        déplacent les doigts de 66 mm et ruinent la calibration. Avec le déport,
        l'orientation peut varier librement."""
        box = QGroupBox('1) Calibration pince — balle IMMOBILE, poignet varié')
        v = QVBoxLayout(box)
        b_add = QPushButton('+ Ajouter une pose (doigts sur la balle)')
        b_add.setFocusPolicy(Qt.NoFocus)
        b_add.clicked.connect(self.add_tcp_pose)
        row = QHBoxLayout()
        b_solve = QPushButton('Résoudre le déport')
        b_solve.setFocusPolicy(Qt.NoFocus)
        b_solve.clicked.connect(self.solve_tcp_offset)
        b_clear = QPushButton('↺ Repartir à zéro')
        b_clear.setFocusPolicy(Qt.NoFocus)
        b_clear.clicked.connect(self.clear_tcp_poses)
        row.addWidget(b_solve)
        row.addWidget(b_clear)
        self.lbl_tcp = QLabel('déport inconnu — 6 poses, tourne bien le poignet '
                              'entre chaque (±25° minimum)')
        self.lbl_tcp.setWordWrap(True)
        v.addWidget(b_add)
        v.addLayout(row)
        v.addWidget(self.lbl_tcp)
        return box

    def clear_tcp_poses(self):
        self.tcp_poses = []
        self.lbl_tcp.setText('poses effacées — pince FERMÉE sur un point net '
                             '(coin d\'ArUco), poignet très varié entre chaque')

    def add_tcp_pose(self):
        position, rotation = fk_pose(self.q)
        self.tcp_poses.append((position, rotation))
        spread = 0.0
        if len(self.tcp_poses) > 1:
            spread = max(orientation_error_deg(self.q, R)
                         for _, R in self.tcp_poses[:-1])
        self.lbl_tcp.setText(f'{len(self.tcp_poses)} pose(s) — écart d\'orientation '
                             f'max {spread:.0f}° (vise ≥ 25°)')

    def solve_tcp_offset(self):
        try:
            offset, touched, residual, cond = solve_tcp(self.tcp_poses)
        except (ValueError, np.linalg.LinAlgError) as exc:
            QMessageBox.warning(self, 'Calibration pince', str(exc))
            return
        self.tool_offset = offset
        TOOL_FILE.write_text(json.dumps({
            'tool_offset_mm': offset.tolist(),
            'touched_point_mm': touched.tolist(),
            'residual_rms_mm': residual,
            'conditioning': cond,
            'n_poses': len(self.tcp_poses)}, indent=2))
        verdict = 'OK' if residual < 5.0 and cond < 40.0 else 'DOUTEUX — varie plus le poignet'
        self.lbl_tcp.setText(
            f'déport = {np.round(offset, 1).tolist()} mm  |t| = {np.linalg.norm(offset):.1f} mm\n'
            f'résidu {residual:.1f} mm · conditionnement {cond:.0f} → {verdict}')

    def _teach_box(self):
        box = QGroupBox('Teach')
        v = QVBoxLayout(box)
        self.b_measure = QPushButton('1 — Mesurer la balle  (bras écarté)')
        self.b_measure.setMinimumHeight(40)
        self.b_measure.clicked.connect(self.measure)
        self.b_record = QPushButton('2 — Enregistrer le point')
        self.b_record.setMinimumHeight(40)
        self.b_record.clicked.connect(self.record)
        self.b_fit = QPushButton('3 — Ajuster et sauver l\'homographie')
        self.b_fit.clicked.connect(self.do_fit)
        self.lbl_spread = QLabel('0 point')
        self.points = QListWidget()
        self.points.setMaximumHeight(130)
        for w in (self.b_measure, self.b_record, self.lbl_spread, self.points, self.b_fit):
            v.addWidget(w)
        return box

    # ------------------------------------------------------------ robot
    def send_joints(self, q_target):
        reply = self.bridge.send({'action': 'send_angles',
                                  'angles': [round(float(v), 2) for v in q_target],
                                  'speed': self.args.speed})
        time.sleep(self.args.settle)
        self.q = list(self.bridge.get_angles())
        position, _ = fk_pose(self.q)
        self.xyz = [float(v) for v in position]
        return reply

    def armed(self):
        """Aucun mouvement tant que l'opérateur n'a pas armé explicitement."""
        if self.chk_arm.isChecked():
            return True
        self.lbl_error.setText('🔒 Coche « ARMER » pour autoriser les mouvements.')
        return False

    def move_by(self, axis, delta):
        """Applique un déplacement et l'ANNULE si le robot refuse : sinon une
        consigne hors limite reste dans l'état et bloque TOUS les axes suivants
        (observé le 17/08 : Z coincé à 528.4 pour une limite à 523.9)."""
        if not self.armed():
            return
        target = list(self.xyz)
        target[axis] += delta
        q_target = solve_pose(self.q, target, self.R_lock)
        reply = self.send_joints(q_target)
        residual = float(np.linalg.norm(np.asarray(self.xyz) - np.asarray(target)))
        self.lbl_error.setText(
            reply.strip() if 'ERROR' in reply.upper()
            else (f'écart à la consigne {residual:.1f} mm' if residual > 3.0 else ''))

    def jog(self, axis, sign):
        """axis 0/1 = les deux directions du plan, 2 = hauteur.

        En mode ÉCRAN, 0/1 désignent l'horizontale et la verticale de l'IMAGE :
        on inverse le jacobien pour trouver le déplacement base correspondant.
        """
        if axis == 2 or not self.chk_screen.isChecked() or self.jacobian is None:
            self.move_by(axis, sign * self.step)
            return
        # u croît vers la DROITE, v vers le BAS. « gauche » (axis 1, sign +1) doit
        # donc faire DÉCROÎTRE u, et « haut » (axis 0, sign +1) décroître v.
        wanted_px = np.array([-sign * self.step, 0.0]) if axis == 1 else \
            np.array([0.0, -sign * self.step])
        try:
            delta = np.linalg.solve(self.jacobian, wanted_px)
        except np.linalg.LinAlgError:
            self.move_by(axis, sign * self.step)
            return
        if not self.armed():
            return
        target = [self.xyz[0] + float(delta[0]), self.xyz[1] + float(delta[1]), self.xyz[2]]
        reply = self.send_joints(solve_pose(self.q, target, self.R_lock))
        residual = float(np.linalg.norm(np.asarray(self.xyz) - np.asarray(target)))
        self.lbl_error.setText(
            reply.strip() if 'ERROR' in reply.upper()
            else (f'écart à la consigne {residual:.1f} mm' if residual > 3.0 else ''))

    def park(self):
        self.move_by(2, self.args.park_dz)

    def _send_grip(self):
        value = self.slider.value()
        reply = self.bridge.grip({'action': 'pro_gripper_angle', 'angle': value,
                                  'gripper_id': GRIPPER_ID})
        self.lbl_grip.setText(f'consigne {value} — {reply.strip()[:40]}')

    def keyPressEvent(self, event):
        mapping = {Qt.Key_Up: (0, +1), Qt.Key_Down: (0, -1),
                   Qt.Key_Left: (1, +1), Qt.Key_Right: (1, -1),
                   Qt.Key_PageUp: (2, +1), Qt.Key_PageDown: (2, -1)}
        if event.key() in mapping:
            self.jog(*mapping[event.key()])

    # ------------------------------------------------------------ teach
    def measure(self):
        self.pixel = ball_pixel(self.cap, self.args.hsv_lo, self.args.hsv_hi,
                                self.args.frames, self.args.spread_px, self.scale)
        self.b_measure.setText('1 — Mesurer la balle  ' +
                               ('✅ mesurée' if self.pixel is not None else '❌ échec'))

    def record(self):
        if self.pixel is None:
            QMessageBox.warning(self, 'Teach', 'Mesure la balle avant d\'enregistrer.')
            return
        if self.tool_offset is None:
            QMessageBox.warning(
                self, 'Teach',
                'Calibre d\'abord le déport de la pince (bloc 1). Sans lui on '
                'enregistre la bride, dont la position dépend de l\'orientation '
                'du poignet — 43° d\'écart = 66 mm sur les doigts.')
            return
        position, rotation = fk_pose(self.q)
        tip = position + rotation @ self.tool_offset
        self.samples.append({
            'pixel': [float(self.pixel[0]), float(self.pixel[1])],
            'coords_mm': [float(tip[0]), float(tip[1]), float(tip[2])],
            'flange_mm': [float(v) for v in position],
            'angles_deg': list(self.q)})
        self.points.addItem(
            f'{len(self.samples):2d}  px=({self.pixel[0]:.0f},{self.pixel[1]:.0f})  '
            f'doigts=({tip[0]:.0f},{tip[1]:.0f},{tip[2]:.0f})')
        self.pixel = None
        self.b_measure.setText('1 — Mesurer la balle  (bras écarté)')
        xy = np.asarray([s['coords_mm'][:2] for s in self.samples])
        if len(self.samples) >= 3:
            dx, dy = np.ptp(xy[:, 0]), np.ptp(xy[:, 1])
            warn = '  ⚠️ étale en PROFONDEUR' if dx < 0.5 * dy else '  ✅'
            self.lbl_spread.setText(
                f'{len(self.samples)} points — profondeur {dx:.0f} mm × largeur {dy:.0f} mm{warn}')
        else:
            self.lbl_spread.setText(f'{len(self.samples)} point(s)')

    def do_fit(self):
        if len(self.samples) < MIN_POINTS:
            QMessageBox.warning(self, 'Teach',
                                f'{len(self.samples)} points — il en faut {MIN_POINTS}.')
            return
        src = np.asarray([s['pixel'] for s in self.samples])
        dst = np.asarray([s['coords_mm'][:2] for s in self.samples])
        H, mask = cv2.findHomography(src, dst, cv2.RANSAC, 8.0)
        if H is None:
            QMessageBox.critical(self, 'Teach', 'Ajustement impossible (points alignés ?).')
            return
        proj = cv2.perspectiveTransform(src.reshape(-1, 1, 2), H).reshape(-1, 2)
        res = np.linalg.norm(proj - dst, axis=1)
        inl = mask.ravel().astype(bool)
        save(H, self.samples, res, inl, (self.args.width, self.args.height))
        QMessageBox.information(
            self, 'Teach',
            f'{inl.sum()}/{len(self.samples)} points retenus\n'
            f'résidu moyen {res[inl].mean():.1f} mm — max {res[inl].max():.1f} mm\n\n{OUT}')

    def _draw_compass(self, vis):
        """Où pointent X+ et Y+ du robot dans l'image, à la position courante."""
        origin = project_base_point(self.xyz, self.K, self.T_cam_world)
        if origin is None or self.jacobian is None:
            return
        for column, name, colour in ((0, 'X+', (0, 200, 255)),
                                     (1, 'Y+', (255, 120, 0))):
            direction = self.jacobian[:, column]
            norm = np.linalg.norm(direction)
            if norm < 1e-6:
                continue
            tip = origin + direction / norm * 70.0 * self.scale
            cv2.arrowedLine(vis, tuple(np.int32(origin)), tuple(np.int32(tip)),
                            colour, max(2, int(2 * self.scale)), tipLength=0.25)
            cv2.putText(vis, name, tuple(np.int32(tip)), cv2.FONT_HERSHEY_SIMPLEX,
                        0.7 * self.scale, colour, max(2, int(2 * self.scale)))
        cv2.circle(vis, tuple(np.int32(origin)), int(6 * self.scale), (255, 255, 255), -1)

    # ----------------------------------------------------------- rendu
    def _refresh(self):
        ok, frame = self.cap.read()
        if not ok:
            return
        found, best, _ = candidates(frame, self.args.hsv_lo, self.args.hsv_hi, self.scale)
        vis = draw(frame, found, best, self.args.hsv_lo, self.args.hsv_hi, self.scale)
        self.jacobian = screen_jacobian(self.xyz, self.K, self.T_cam_world)
        self._draw_compass(vis)
        rgb = cv2.cvtColor(vis, cv2.COLOR_BGR2RGB)
        h, w, _ = rgb.shape
        image = QImage(rgb.data, w, h, 3 * w, QImage.Format_RGB888)
        self.view.setPixmap(QPixmap.fromImage(image).scaled(
            self.view.width(), self.view.height(), Qt.KeepAspectRatio, Qt.SmoothTransformation))
        self.lbl_pose.setText(f'X {self.xyz[0]:7.1f}   Y {self.xyz[1]:7.1f}   Z {self.xyz[2]:7.1f}')
        self.lbl_wrist.setText(
            'réel : ' + '  '.join(f'J{i+1} {v:+.1f}' for i, v in enumerate(self.q)))

    def closeEvent(self, event):
        self.timer.stop()
        self.cap.release()
        event.accept()


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument('--pi-host', default='10.10.0.224')
    ap.add_argument('--width', type=int, default=1600)
    ap.add_argument('--height', type=int, default=1200)
    ap.add_argument('--exposure', type=int, default=75)
    ap.add_argument('--frames', type=int, default=8)
    ap.add_argument('--spread-px', type=float, default=12.0)
    ap.add_argument('--hsv-lo', nargs=3, type=int, default=[20, 170, 70])
    ap.add_argument('--hsv-hi', nargs=3, type=int, default=[35, 255, 255])
    ap.add_argument('--speed', type=int, default=20)
    ap.add_argument('--park-dz', type=float, default=90.0)
    ap.add_argument('--wrist-step', type=float, default=5.0,
                    help='pas de rotation J4/J5/J6 en degres')
    ap.add_argument('--settle', type=float, default=0.9,
                    help='attente apres send_angles avant relecture des angles')
    ap.add_argument('--orientation', nargs=3, type=float, default=None)
    args = ap.parse_args()

    if args.orientation is None:
        cal = json.loads((_REPO / 'scripts' / 'pick_adaptive_calibration.json').read_text())
        args.orientation = [float(v) for v in cal['safe_orientation_deg']]

    app = QApplication(sys.argv)
    win = TeachDashboard(args)
    win.resize(1380, 860)
    win.show()
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
