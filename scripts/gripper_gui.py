#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""GUI PyQt5 de contrôle du gripper Pro — À LANCER SUR LE PC TOUR.

Se connecte au bridge (gripper_bridge.py sur le Pi) en TCP et :
  - slider d'angle 0-100 (course d'ouverture) + boutons Ouvrir/Fermer
  - réglage vitesse (1-100) et couple (100-300)
  - affichage live : angle, statut, couple, vitesse
  - sonde de registres (pour trouver courant A / force affichés sur le LCD)

Elephant impose >= 1.5 s entre deux commandes gripper : le client les espace tout
seul (file d'attente + intervalle mini), donc l'UI ne bloque jamais.

Lancer :  python3 gripper_gui.py --host 10.10.0.221
Prérequis : PyQt5 (déjà installé pour le dashboard)
"""
import argparse
import json
import socket
import time
from collections import deque

from PyQt5.QtCore import Qt, QThread, pyqtSignal
from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QGridLayout, QLabel,
    QSlider, QPushButton, QSpinBox, QGroupBox, QLineEdit, QPlainTextEdit,
)

MIN_GAP = 1.6   # s entre deux commandes gripper (> 1.5 s imposé)

STATUS_TXT = {0: "en mouvement", 1: "arrêté (rien saisi)",
              2: "arrêté (objet saisi)", 3: "objet tombé"}


class GripperLink:
    """Connexion TCP au bridge, thread-safe, avec intervalle mini garanti."""
    def __init__(self, host, port=5005):
        self.host, self.port = host, port
        self.sock = None
        self._buf = ""
        self._last_motion = 0.0

    def connect(self):
        self.sock = socket.create_connection((self.host, self.port), timeout=8)
        self.sock.settimeout(8)
        self._buf = ""

    def send(self, obj, is_motion=False):
        # Le gap 1.5 s n'est imposé qu'ENTRE DEUX MOUVEMENTS du gripper.
        # Les lectures (getters) ne bloquent pas la commande suivante.
        if is_motion:
            dt = time.time() - self._last_motion
            if dt < MIN_GAP:
                time.sleep(MIN_GAP - dt)
        msg = json.dumps(obj) if isinstance(obj, dict) else str(obj)
        self.sock.sendall((msg + "\n").encode())
        while "\n" not in self._buf:
            chunk = self.sock.recv(4096).decode()
            if not chunk:
                break
            self._buf += chunk
        line, _, self._buf = self._buf.partition("\n")
        if is_motion:
            self._last_motion = time.time()
        return line.strip()


class Worker(QThread):
    """Exécute les commandes en file + poll périodique, hors du thread UI."""
    reply = pyqtSignal(str, str)     # (tag, texte réponse)
    live = pyqtSignal(dict)          # {'angle':..,'status':..,'torque':..,'speed':..}
    log = pyqtSignal(str)

    def __init__(self, link, gid):
        super().__init__()
        self.link = link
        self.gid = gid
        self.queue = deque()
        self.running = True
        self._poll_seq = [
            ('angle', 'get_pro_gripper_angle', 'PRO_GRIPPER_ANGLE:'),
            ('status', 'get_pro_gripper_status', 'PRO_GRIPPER_STATUS:'),
            ('torque', 'get_pro_gripper_torque', 'PRO_GRIPPER_TORQUE:'),
            ('speed', 'get_pro_gripper_speed', 'PRO_GRIPPER_SPEED:'),
        ]
        self._poll_i = 0

    def push(self, obj, tag="", priority=False):
        if priority:
            self.queue.appendleft((obj, tag))
        else:
            self.queue.append((obj, tag))

    def push_angle(self, obj, tag="angle"):
        # coalescing : ne garde que la DERNIÈRE consigne d'angle en attente,
        # pour que glisser le slider n'empile pas des valeurs périmées.
        self.queue = deque([(o, t) for (o, t) in self.queue if t != tag])
        self.queue.appendleft((obj, tag))

    def run(self):
        try:
            self.link.connect()
            self.log.emit(f"Connecté à {self.link.host}:{self.link.port}")
        except Exception as e:
            self.log.emit(f"ERREUR connexion: {e}")
            return
        # NE PAS forcer la vitesse ici : on laisse la vitesse interne du gripper
        # (rapide, comme le client terminal). L'utilisateur la change s'il veut.
        poll_t = 0.0
        while self.running:
            # commandes en attente d'abord
            if self.queue:
                obj, tag = self.queue.popleft()
                is_motion = tag == 'angle' or (
                    isinstance(obj, dict) and obj.get('action') in (
                        'pro_gripper_open', 'pro_gripper_close', 'pro_gripper_angle'))
                try:
                    r = self.link.send(obj, is_motion=is_motion)
                    self.reply.emit(tag, r)
                    self.log.emit(f">>> {obj}\n<<< {r}")
                except Exception as e:
                    self.log.emit(f"ERREUR envoi: {e}")
                continue
            # sinon poll live (angle + statut seulement — léger).
            # Une seule lecture par itération pour ne jamais bloquer longtemps
            # une commande utilisateur qui arriverait juste après.
            if time.time() - poll_t > 3.0:
                key, act, pfx = self._poll_seq[self._poll_i % len(self._poll_seq)]
                self._poll_i += 1
                if self._poll_i % len(self._poll_seq) == 0:
                    poll_t = time.time()
                try:
                    r = self.link.send({'action': act, 'gripper_id': self.gid})
                    if pfx in r:
                        self.live.emit({key: r.split(pfx, 1)[1].strip()})
                except Exception:
                    pass
            else:
                time.sleep(0.05)

    def stop(self):
        self.running = False


class GripperGUI(QWidget):
    def __init__(self, host, gid):
        super().__init__()
        self.gid = gid
        self.setWindowTitle("Contrôle Gripper Pro — MyCobot 320")
        self.resize(560, 620)

        self.link = GripperLink(host)
        self.worker = Worker(self.link, gid)
        self.worker.reply.connect(self._on_reply)
        self.worker.live.connect(self._on_live)
        self.worker.log.connect(self._on_log)

        root = QVBoxLayout(self)

        # --- Affichage live ---
        live_box = QGroupBox("Gripper State (recopie du LCD → PC)")
        g = QGridLayout(live_box)
        self.lbl_angle = QLabel("—"); self.lbl_status = QLabel("—")
        self.lbl_torque = QLabel("—"); self.lbl_speed = QLabel("—")
        for r, (name, w) in enumerate([("Position (%)", self.lbl_angle),
                                       ("Statut", self.lbl_status),
                                       ("Couple (réglé)", self.lbl_torque),
                                       ("Vitesse (réglée %)", self.lbl_speed)]):
            g.addWidget(QLabel(name + " :"), r, 0)
            w.setStyleSheet("font-weight:bold; font-size:15px;")
            g.addWidget(w, r, 1)
        root.addWidget(live_box)

        # --- Consigne d'angle : boîte de valeur + OK (comme simple_gui.py) ---
        ang_box = QGroupBox("Consigne d'ouverture (0-100)")
        v = QVBoxLayout(ang_box)
        row = QHBoxLayout()
        self.sp_angle = QSpinBox(); self.sp_angle.setRange(0, 100); self.sp_angle.setValue(100)
        self.sp_angle.setFixedWidth(90)
        b_ok = QPushButton("OK ▶")
        b_ok.clicked.connect(self._send_angle)
        self.sp_angle.lineEdit().returnPressed.connect(self._send_angle)
        row.addWidget(QLabel("Angle :")); row.addWidget(self.sp_angle)
        row.addWidget(b_ok); row.addStretch(1)
        v.addLayout(row)
        hb = QHBoxLayout()
        b_open = QPushButton("Ouvrir (100)"); b_close = QPushButton("Fermer (0)")
        b_open.clicked.connect(lambda: self._quick(100))
        b_close.clicked.connect(lambda: self._quick(0))
        hb.addWidget(b_open); hb.addWidget(b_close)
        v.addLayout(hb)
        root.addWidget(ang_box)

        # --- Réglages vitesse / couple ---
        set_box = QGroupBox("Réglages")
        sg = QGridLayout(set_box)
        self.sp_speed = QSpinBox(); self.sp_speed.setRange(1, 100); self.sp_speed.setValue(30)
        self.sp_torque = QSpinBox(); self.sp_torque.setRange(100, 300); self.sp_torque.setValue(150)
        b_sp = QPushButton("Appliquer vitesse"); b_tq = QPushButton("Appliquer couple")
        b_sp.clicked.connect(self._apply_speed)
        b_tq.clicked.connect(self._apply_torque)
        sg.addWidget(QLabel("Vitesse (1-100)"), 0, 0); sg.addWidget(self.sp_speed, 0, 1); sg.addWidget(b_sp, 0, 2)
        sg.addWidget(QLabel("Couple (100-300)"), 1, 0); sg.addWidget(self.sp_torque, 1, 1); sg.addWidget(b_tq, 1, 2)
        b_calib = QPushButton("Calibration zéro")
        b_calib.clicked.connect(lambda: self.worker.push(
            {'action': 'pro_gripper_calib', 'gripper_id': self.gid}, 'calib'))
        sg.addWidget(b_calib, 2, 0, 1, 3)
        root.addWidget(set_box)

        # --- Sonde de registres (courant A / force) ---
        reg_box = QGroupBox("Sonde registre (trouver courant A / force du LCD)")
        rg = QHBoxLayout(reg_box)
        self.reg_addr = QLineEdit("0"); self.reg_addr.setFixedWidth(60)
        b_reg = QPushButton("Lire registre")
        self.reg_val = QLabel("—")
        b_scan = QPushButton("Scan 0→30")
        b_reg.clicked.connect(self._read_reg)
        b_scan.clicked.connect(self._scan_regs)
        rg.addWidget(QLabel("adresse")); rg.addWidget(self.reg_addr)
        rg.addWidget(b_reg); rg.addWidget(self.reg_val); rg.addWidget(b_scan)
        root.addWidget(reg_box)

        # --- Log ---
        self.logbox = QPlainTextEdit(); self.logbox.setReadOnly(True)
        self.logbox.setMaximumBlockCount(300)
        root.addWidget(self.logbox, 1)

        self.worker.start()

    # ---- actions ----
    def _send_angle(self):
        self.worker.push_angle({'action': 'pro_gripper_angle', 'angle': self.sp_angle.value(),
                                'gripper_id': self.gid})

    def _quick(self, val):
        self.sp_angle.setValue(val)
        self.worker.push_angle({'action': 'pro_gripper_angle', 'angle': val, 'gripper_id': self.gid})

    def _apply_speed(self):
        # ne pas préremplir le label : il sera mis à jour par la lecture réelle
        self.worker.push({'action': 'set_pro_gripper_speed', 'speed': self.sp_speed.value(),
                          'gripper_id': self.gid}, 'set', priority=True)

    def _apply_torque(self):
        self.worker.push({'action': 'set_pro_gripper_torque', 'torque': self.sp_torque.value(),
                          'gripper_id': self.gid}, 'set', priority=True)

    def _read_reg(self):
        try:
            a = int(self.reg_addr.text())
        except ValueError:
            return
        self.worker.push({'action': 'get_pro_gripper', 'address': a, 'gripper_id': self.gid}, f'reg{a}')

    def _scan_regs(self):
        for a in range(0, 31):
            self.worker.push({'action': 'get_pro_gripper', 'address': a, 'gripper_id': self.gid}, f'reg{a}')

    # ---- réponses ----
    def _on_reply(self, tag, text):
        if tag.startswith('reg'):
            self.reg_val.setText(text)

    def _on_live(self, vals):
        if 'angle' in vals: self.lbl_angle.setText(vals['angle'])
        if 'status' in vals:
            try:
                s = int(vals['status']); self.lbl_status.setText(f"{s} — {STATUS_TXT.get(s, '?')}")
            except ValueError:
                self.lbl_status.setText(vals['status'])
        if 'torque' in vals: self.lbl_torque.setText(vals['torque'])
        if 'speed' in vals: self.lbl_speed.setText(vals['speed'])

    def _on_log(self, msg):
        self.logbox.appendPlainText(msg)

    def closeEvent(self, e):
        self.worker.stop(); self.worker.wait(2000); e.accept()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="10.10.0.221", help="IP du Pi")
    ap.add_argument("--id", type=int, default=14, help="gripper_id")
    args = ap.parse_args()
    app = QApplication([])
    gui = GripperGUI(args.host, args.id)
    gui.show()
    app.exec_()


if __name__ == "__main__":
    main()
