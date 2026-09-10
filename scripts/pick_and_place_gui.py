#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""GUI d'apprentissage + exécution pick-and-place RÉEL — À LANCER SUR LE PC TOUR.

Un bouton par étape (comme le GUI gripper) :
  APPRENTISSAGE : ouvrir pince · relâcher servos · capturer les 4 poses · re-tendre
  SERRAGE       : tester un angle de fermeture (voit le statut) · enregistrer l'angle
  EXÉCUTION     : aller à une pose · RUN pick-and-place (option pas-à-pas)

Toutes les commandes passent par un worker (thread) pour ne jamais figer l'UI
(le gripper impose 1.6 s entre commandes, les mouvements prennent quelques s).

Lancer : python3 scripts/pick_and_place_gui.py --host 10.10.0.221
"""
import argparse
import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
from pick_and_place_real import (Bridge, GRIPPER_ID, STATUS_TXT, WAYPOINTS,   # noqa: E402
                                 load_positions, save_positions)

from PyQt5.QtCore import QThread, pyqtSignal                                   # noqa: E402
from PyQt5.QtWidgets import (                                                  # noqa: E402
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel,
    QPushButton, QSpinBox, QGroupBox, QPlainTextEdit, QCheckBox, QComboBox,
)


class Worker(QThread):
    """Exécute une file de jobs hors du thread UI."""
    log = pyqtSignal(str)
    captured = pyqtSignal(str, list)     # (waypoint, angles)
    status = pyqtSignal(int)             # statut gripper
    need_continue = pyqtSignal(str)      # mode pas-à-pas : attend le bouton Continuer

    def __init__(self, host, speed, open_angle):
        super().__init__()
        self.host = host
        self.speed = speed
        self.open_angle = open_angle
        self.jobs = []
        self.running = True
        self._continue = threading.Event()
        self._stop_loop = threading.Event()
        self.b = None

    def submit(self, job):
        self.jobs.append(job)

    def resume(self):
        self._continue.set()

    def stop_loop(self):
        self._stop_loop.set()

    # --- helpers ---
    def _move(self, angles, speed):
        self.log.emit(f'→ send_angles {[round(a,1) for a in angles]} v={speed}')
        self.log.emit('  ' + self.b.send(
            {'action': 'send_angles', 'angles': angles, 'speed': speed}))
        time.sleep(3.0)

    def _grip(self, angle):
        # le bras et la pince partagent le port série du Pi : on laisse le robot
        # s'immobiliser avant d'envoyer la commande pince, sinon elle est perdue.
        time.sleep(1.2)
        self.log.emit('  ' + self.b.grip(
            {'action': 'pro_gripper_angle', 'angle': angle, 'gripper_id': GRIPPER_ID}))
        time.sleep(0.8)

    def _gate(self, step, msg):
        if not step:
            return True
        self._continue.clear()
        self.need_continue.emit(msg)
        while not self._continue.is_set() and self.running:
            time.sleep(0.05)
        return self.running

    def run(self):
        try:
            self.b = Bridge(self.host)
            self.log.emit(f'Connecté à {self.host}:5005')
        except Exception as e:
            self.log.emit(f'ERREUR connexion : {e}'); return

        while self.running:
            if not self.jobs:
                time.sleep(0.05); continue
            kind, *arg = self.jobs.pop(0)
            try:
                self._do(kind, arg)
            except Exception as e:
                self.log.emit(f'ERREUR {kind} : {e}')

    def _do(self, kind, arg):
        if kind == 'open':
            self._grip(self.open_angle)
        elif kind == 'release':
            self.log.emit('  ' + self.b.send({'action': 'power_off'}))
        elif kind == 'power_on':
            self.log.emit('  ' + self.b.send({'action': 'power_on'}))
        elif kind == 'capture':
            wp = arg[0]
            angles = self.b.get_angles()
            pos = load_positions(); pos[wp] = angles; save_positions(pos)
            self.captured.emit(wp, angles)
            self.log.emit(f'{wp} capturé : {[round(a,1) for a in angles]}')
        elif kind == 'goto':
            wp = arg[0]; pos = load_positions()
            if wp in pos:
                self._move(pos[wp], self.speed)
            else:
                self.log.emit(f'{wp} non capturé')
        elif kind == 'grip':
            angle = arg[0]; self._grip(angle)
            st = self.b.gripper_status()
            self.status.emit(st if st is not None else -1)
            ok = '✅ OBJET SAISI' if st == 2 else '(pas de prise)'
            self.log.emit(f'angle {angle} → statut {st} ({STATUS_TXT.get(st,"?")}) {ok}')
        elif kind == 'run':
            speed, grasp, step, loop = arg
            self._stop_loop.clear()
            n = 0
            while self.running:
                n += 1
                if loop:
                    self.log.emit(f'--- cycle {n} ---')
                self._run_seq(speed, grasp, step)
                if not loop or self._stop_loop.is_set():
                    break
            if loop:
                self.log.emit('⏹ boucle arrêtée.')

    def _run_seq(self, speed, grasp, step):
        pos = load_positions()
        missing = [w for w in WAYPOINTS if w not in pos]
        if missing:
            self.log.emit(f'⛔ poses manquantes : {missing}'); return
        grasp = int(pos.get('grasp_angle', grasp))
        self.log.emit(f'=== RUN pick-and-place (v={speed}, serrage={grasp}'
                      f'{", pas-à-pas" if step else ""}) ===')
        self._grip(self.open_angle)
        if not self._gate(step, 'approcher au-dessus du pick'): return
        self._move(pos['pick_approach'], speed)
        if not self._gate(step, 'DESCENDRE sur l\'objet'): return
        self._move(pos['pick'], max(20, speed // 2))
        if not self._gate(step, 'FERMER la pince'): return
        self._grip(grasp)
        st = self.b.gripper_status(); self.status.emit(st if st is not None else -1)
        self.log.emit(f'  statut : {st} ({STATUS_TXT.get(st,"?")})')
        if st != 2:
            self.log.emit('  ⚠️ objet non saisi (statut≠2)')
            if not self._gate(step, 'objet NON saisi — continuer quand même'):
                self._move(pos['pick_approach'], speed); return
        if not self._gate(step, 'SOULEVER'): return
        self._move(pos['pick_approach'], speed)
        if not self._gate(step, 'aller au place'): return
        self._move(pos['place_approach'], speed)
        if not self._gate(step, 'DESCENDRE place'): return
        self._move(pos['place'], max(20, speed // 2))
        if not self._gate(step, 'OUVRIR (déposer)'): return
        self._grip(self.open_angle)
        self._move(pos['place_approach'], speed)
        self.log.emit('  ' + self.b.send({'action': 'go_home'}))
        self.log.emit('✅ terminé.')

    def stop(self):
        self.running = False; self._continue.set()


class PickPlaceGUI(QWidget):
    def __init__(self, host, speed, open_angle):
        super().__init__()
        self.setWindowTitle('Pick-and-Place réel — MyCobot 320')
        self.resize(560, 680)
        self.worker = Worker(host, speed, open_angle)
        self.worker.log.connect(self._log)
        self.worker.captured.connect(self._on_captured)
        self.worker.status.connect(self._on_status)
        self.worker.need_continue.connect(self._on_need_continue)

        root = QVBoxLayout(self)

        # 1. Apprentissage
        learn = QGroupBox('1 · Apprentissage (bras déplacé à la main)')
        lg = QGridLayout(learn)
        b_open = QPushButton('🖐 Ouvrir pince')
        b_rel = QPushButton('😴 Relâcher servos (TIENS le bras)')
        b_pon = QPushButton('💪 Re-tendre servos')
        b_open.clicked.connect(lambda: self.worker.submit(('open',)))
        b_rel.clicked.connect(self._release)
        b_pon.clicked.connect(lambda: self.worker.submit(('power_on',)))
        lg.addWidget(b_open, 0, 0); lg.addWidget(b_rel, 0, 1); lg.addWidget(b_pon, 0, 2)
        self.cap_labels = {}
        for i, wp in enumerate(WAYPOINTS):
            btn = QPushButton(f'📍 Capturer {wp}')
            btn.clicked.connect(lambda _, w=wp: self.worker.submit(('capture', w)))
            lbl = QLabel('—'); self.cap_labels[wp] = lbl
            lg.addWidget(btn, 1 + i, 0); lg.addWidget(lbl, 1 + i, 1, 1, 2)
        root.addWidget(learn)

        # 2. Serrage
        grip = QGroupBox('2 · Serrage sur l\'objet')
        gg = QHBoxLayout(grip)
        self.sp_grip = QSpinBox(); self.sp_grip.setRange(0, 100); self.sp_grip.setValue(20)
        b_goto_pa = QPushButton('▸ Aller pick_approach')
        b_goto_p = QPushButton('▸ Aller pick')
        b_test = QPushButton('Tester serrage')
        b_save = QPushButton('✔ Enregistrer angle')
        self.lbl_grip = QLabel('statut : —')
        b_goto_pa.clicked.connect(lambda: self.worker.submit(('goto', 'pick_approach')))
        b_goto_p.clicked.connect(lambda: self.worker.submit(('goto', 'pick')))
        b_test.clicked.connect(lambda: self.worker.submit(('grip', self.sp_grip.value())))
        b_save.clicked.connect(self._save_grasp)
        gg.addWidget(b_goto_pa); gg.addWidget(b_goto_p)
        gg.addWidget(QLabel('angle')); gg.addWidget(self.sp_grip)
        gg.addWidget(b_test); gg.addWidget(b_save); gg.addWidget(self.lbl_grip)
        root.addWidget(grip)

        # 3. Exécution
        exe = QGroupBox('3 · Exécution')
        eg = QHBoxLayout(exe)
        self.sp_speed = QSpinBox(); self.sp_speed.setRange(1, 100); self.sp_speed.setValue(speed)
        self.chk_step = QCheckBox('pas-à-pas')
        self.chk_loop = QCheckBox('boucle continue')
        self.b_run = QPushButton('▶ RUN pick-and-place')
        self.b_cont = QPushButton('⏭ Continuer'); self.b_cont.setEnabled(False)
        self.b_stop = QPushButton('⏹ Stop boucle'); self.b_stop.setEnabled(False)
        self.b_run.clicked.connect(self._run)
        self.b_cont.clicked.connect(self._continue)
        self.b_stop.clicked.connect(lambda: self.worker.stop_loop())
        eg.addWidget(QLabel('vitesse')); eg.addWidget(self.sp_speed)
        eg.addWidget(self.chk_step); eg.addWidget(self.chk_loop)
        eg.addWidget(self.b_run); eg.addWidget(self.b_cont); eg.addWidget(self.b_stop)
        root.addWidget(exe)

        # log
        self.logbox = QPlainTextEdit(); self.logbox.setReadOnly(True)
        self.logbox.setMaximumBlockCount(500)
        root.addWidget(self.logbox, 1)

        # charge poses déjà apprises
        pos = load_positions()
        for wp in WAYPOINTS:
            if wp in pos:
                self.cap_labels[wp].setText(str([round(a, 1) for a in pos[wp]]))
        if 'grasp_angle' in pos:
            self.sp_grip.setValue(int(pos['grasp_angle']))

        self.worker.start()

    # --- handlers ---
    def _release(self):
        from PyQt5.QtWidgets import QMessageBox
        if QMessageBox.warning(self, 'Relâcher', 'TIENS le bras (il devient mou). Continuer ?',
                               QMessageBox.Yes | QMessageBox.No) == QMessageBox.Yes:
            self.worker.submit(('release',))

    def _save_grasp(self):
        pos = load_positions(); pos['grasp_angle'] = self.sp_grip.value(); save_positions(pos)
        self._log(f'angle de fermeture enregistré : {self.sp_grip.value()}')

    def _run(self):
        from PyQt5.QtWidgets import QMessageBox
        if QMessageBox.warning(self, 'RUN', '⚠️ Le bras VA BOUGER. Zone dégagée, main sur l\'arrêt ?',
                               QMessageBox.Yes | QMessageBox.No) != QMessageBox.Yes:
            return
        loop = self.chk_loop.isChecked()
        self.b_stop.setEnabled(loop)
        self.worker.submit(('run', self.sp_speed.value(), self.sp_grip.value(),
                            self.chk_step.isChecked(), loop))

    def _continue(self):
        self.b_cont.setEnabled(False)
        self.worker.resume()

    def _on_need_continue(self, msg):
        self._log(f'⏸ {msg}')
        self.b_cont.setEnabled(True)

    def _on_captured(self, wp, angles):
        self.cap_labels[wp].setText(str([round(a, 1) for a in angles]))

    def _on_status(self, st):
        txt = STATUS_TXT.get(st, '?')
        self.lbl_grip.setText(f'statut : {st} ({txt})'
                              + ('  ✅' if st == 2 else ''))

    def _log(self, msg):
        self.logbox.appendPlainText(msg)

    def closeEvent(self, e):
        self.worker.stop(); self.worker.wait(2000); e.accept()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--host', default='10.10.0.221')
    ap.add_argument('--speed', type=int, default=20)
    ap.add_argument('--open-angle', type=int, default=100)
    args = ap.parse_args()
    app = QApplication([])
    gui = PickPlaceGUI(args.host, args.speed, args.open_angle)
    gui.show()
    app.exec_()


if __name__ == '__main__':
    main()
