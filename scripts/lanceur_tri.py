#!/usr/bin/env python3
"""Une fenetre, un bouton par programme : plus de commande a taper.

    /usr/bin/python3 scripts/lanceur_tri.py

Un clic lance, un second clic arrete. Les quatre programmes se partagent les
cameras, le bridge de la Pi (mono-client) ou le GPU : en lancer un ferme
d'abord celui qui tourne. Chaque sortie va dans results/lanceur/<date>_<nom>.log
et dans la fenetre.

Le tri reel peut ecrire son journal (run.yaml + temperature) pendant le tri ;
« CSV du dernier tri » convertit ensuite la derniere session, comme
`essai_reel_csv.py` a la main.
"""
import os
import shlex
import signal
import socket
import subprocess
import sys
from datetime import datetime
from pathlib import Path

from PyQt5.QtCore import QProcess, QProcessEnvironment, QTimer
from PyQt5.QtGui import QFont, QTextCursor
from PyQt5.QtWidgets import (QApplication, QCheckBox, QGridLayout, QGroupBox, QHBoxLayout, QLabel,
                             QLineEdit, QMessageBox, QPlainTextEdit, QPushButton, QSpinBox,
                             QVBoxLayout, QWidget)

RACINE = Path(__file__).resolve().parents[1]
JOURNAUX = RACINE / 'results' / 'lanceur'
DREAM_REEL = RACINE / 'results' / 'dream_reel'
WORKSPACE = Path.home() / 'Osama_ws' / 'install' / 'setup.bash'
PI_DEFAUT = '10.10.0.223'
MODELE_REEL = 'vgg_tri_reel_ft_e10'
MODELE_GAZEBO = 'vgg_tri_mix_ft_e10'
# Laisse au launch Gazebo le temps de monter le monde et les controleurs avant le trieur.
ATTENTE_GAZEBO_S = 30
ARRET_DOUX_S = 8
JOURNAL_AVANT_S = 120


def environnement_ros():
    """Sans conda ni .venv : leur Python masque celui de ROS 2 (rclpy 3.12)."""
    # Les chemins ROS herites aussi : le terminal sourcait ~/ros_jazzy (copie figee au 07/07,
    # sans tri_yolo.launch.py) AVANT Osama_ws, et ros2 y cherchait le paquet.
    env = {k: v for k, v in os.environ.items()
           if not k.startswith(('CONDA', 'VIRTUAL_ENV', 'PYTHONPATH', 'PYTHONHOME', 'AMENT_',
                                'COLCON_', 'CMAKE_PREFIX_PATH', 'ROS_PACKAGE_PATH', 'LD_LIBRARY_PATH'))}
    env['PATH'] = ':'.join(p for p in os.environ.get('PATH', '').split(':')
                           if 'conda' not in p and 'venv' not in p and 'ros_jazzy' not in p)
    return env


def ros(commande):
    return (f'source /opt/ros/jazzy/setup.bash && source {WORKSPACE} && cd {RACINE} && {commande}')


def bridge_joignable(ip):
    try:
        with socket.create_connection((ip, 5005), timeout=2):
            return True
    except OSError:
        return False


def temperature_cpu():
    for hwmon in Path('/sys/class/hwmon').glob('hwmon*'):
        if (hwmon / 'name').read_text().strip() == 'coretemp':
            return int((hwmon / 'temp1_input').read_text()) / 1000
    return None


class Programme:
    """Un programme du lanceur : son bouton, son processus (groupe), son journal."""

    def __init__(self, fenetre, nom, titre, detail, reel, commande):
        self.fenetre, self.nom, self.titre, self.reel, self.commande = fenetre, nom, titre, reel, commande
        self.bouton = QPushButton(titre)
        self.bouton.setMinimumHeight(64)
        self.bouton.setFont(QFont('', 12, QFont.Bold))
        self.bouton.clicked.connect(self.bascule)
        self.detail = QLabel(detail)
        self.detail.setWordWrap(True)
        self.etat = QLabel('arrêté')
        self.processus = None
        self.log = None
        self.actualise()

    def tourne(self):
        return self.processus is not None and self.processus.state() != QProcess.NotRunning

    def actualise(self):
        actif = self.tourne()
        self.bouton.setText(f'■ Arrêter — {self.titre}' if actif else f'▶ {self.titre}')
        self.bouton.setStyleSheet('background:#c62828;color:white' if actif else '')
        self.etat.setText('en marche' if actif else 'arrêté')

    def bascule(self):
        if self.tourne():
            self.arrete()
        else:
            self.fenetre.lance(self)

    def demarre(self, script):
        JOURNAUX.mkdir(parents=True, exist_ok=True)
        self.log = (JOURNAUX / f'{datetime.now():%Y-%m-%d_%H-%M-%S}_{self.nom}.log').open('w')
        self.log.write(f'$ {script}\n\n')
        self.processus = QProcess()
        self.processus.setProcessChannelMode(QProcess.MergedChannels)
        env = QProcessEnvironment()
        for k, v in environnement_ros().items():
            env.insert(k, v)
        self.processus.setProcessEnvironment(env)
        self.processus.readyReadStandardOutput.connect(self.lit)
        self.processus.finished.connect(self.fini)
        # setsid : le programme et tous ses enfants (DREAM, YOLO, noeuds ROS) dans un groupe
        # a lui, arrete d'un seul signal, sans toucher a un autre processus du PC.
        self.processus.start('setsid', ['bash', '-c', script])
        self.processus.waitForStarted(3000)
        self.fenetre.ecrit(f'— {self.titre} : lancé (journal {self.log.name})')
        self.actualise()

    def lit(self):
        texte = bytes(self.processus.readAllStandardOutput()).decode(errors='replace')
        self.log.write(texte)
        self.log.flush()
        for ligne in texte.splitlines():
            if ligne.strip() and 'warn' not in ligne.lower():
                self.fenetre.ecrit(f'[{self.nom}] {ligne}')

    def fini(self, code, _statut):
        self.fenetre.ecrit(f'— {self.titre} : terminé (code {code})')
        if self.log:
            self.log.close()
            self.log = None
        self.fenetre.programme_fini(self)
        self.actualise()

    def signale(self, sig):
        if self.tourne():
            try:
                os.killpg(int(self.processus.processId()), sig)
            except ProcessLookupError:
                pass

    def arrete(self):
        """SIGINT (comme Ctrl+C), puis SIGTERM, puis SIGKILL si le groupe tient encore."""
        if not self.tourne():
            return
        self.fenetre.ecrit(f'— {self.titre} : arrêt demandé')
        self.signale(signal.SIGINT)
        if not self.processus.waitForFinished(ARRET_DOUX_S * 1000):
            self.signale(signal.SIGTERM)
            if not self.processus.waitForFinished(ARRET_DOUX_S * 1000):
                self.signale(signal.SIGKILL)
                self.processus.waitForFinished(3000)


class Lanceur(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle('Lanceur — tri MyCobot')
        self.journal_tri = None
        self.dernier_reel_sans_marqueurs = False

        titre = QLabel('Tri — machine d\'état : Moore')
        titre.setFont(QFont('', 16, QFont.Bold))

        self.pi = QLineEdit(PI_DEFAUT)
        self.pi.setMaximumWidth(140)
        self.etat_pi = QLabel('')
        tester = QPushButton('tester la Pi')
        tester.clicked.connect(self.teste_pi)
        self.temperature = QLabel('')

        self.graine = QSpinBox()
        self.graine.setRange(1, 99999)
        self.graine.setValue(1)
        self.sans_marqueurs = QCheckBox('sans marqueurs (extrinsèques DREAM)')
        self.journal_csv = QCheckBox('journal CSV pendant le tri')
        self.journal_csv.setChecked(True)

        self.programmes = [
            Programme(self, 'validation_dream', 'Validation dashboard DREAM',
                      f'Robot réel : DREAM ({MODELE_REEL}) contre les codeurs, arducam + SVPRO '
                      '(dream_multicam.launch.py).', True, self.cmd_validation),
            Programme(self, 'tri_synthetique', 'Tri synthétique — DREAM + YOLO',
                      f'Gazebo : tri des 4 objets, YOLO + DREAM ({MODELE_GAZEBO}) sur 4 caméras, '
                      'dashboard Gazebo / YOLO / DREAM.', False, self.cmd_synthetique),
            Programme(self, 'tri_reel_dream', 'Tri réel — YOLO + DREAM',
                      'Robot réel : YOLO arducam pour la saisie, DREAM arducam + SVPRO contre les '
                      'codeurs (panneau DREAM).', True, self.cmd_reel_dream),
            Programme(self, 'tri_reel_yolo', 'Tri réel — YOLO',
                      'Robot réel : YOLO arducam seule, sans DREAM.', True, self.cmd_reel_yolo),
        ]

        haut = QHBoxLayout()
        haut.addWidget(QLabel('Pi :'))
        haut.addWidget(self.pi)
        haut.addWidget(tester)
        haut.addWidget(self.etat_pi)
        haut.addStretch()
        haut.addWidget(self.temperature)

        grille = QGridLayout()
        for i, p in enumerate(self.programmes):
            boite = QGroupBox()
            v = QVBoxLayout(boite)
            v.addWidget(p.bouton)
            v.addWidget(p.detail)
            v.addWidget(p.etat)
            if p.nom == 'tri_synthetique':
                ligne = QHBoxLayout()
                ligne.addWidget(QLabel('graine :'))
                ligne.addWidget(self.graine)
                ligne.addStretch()
                v.addLayout(ligne)
            if p.nom == 'tri_reel_dream':
                v.addWidget(self.sans_marqueurs)
            grille.addWidget(boite, i // 2, i % 2)

        csv = QHBoxLayout()
        csv.addWidget(self.journal_csv)
        csv.addStretch()
        csv.addWidget(QLabel('essai n°'))
        self.essai = QSpinBox()
        self.essai.setRange(1, 999)
        self.essai.setValue(self.prochain_essai())
        csv.addWidget(self.essai)
        bouton_csv = QPushButton('CSV du dernier tri réel')
        bouton_csv.clicked.connect(self.genere_csv)
        csv.addWidget(bouton_csv)
        tout_arreter = QPushButton('Tout arrêter')
        tout_arreter.clicked.connect(self.arrete_tout)
        csv.addWidget(tout_arreter)

        self.sortie = QPlainTextEdit()
        self.sortie.setReadOnly(True)
        self.sortie.setMaximumBlockCount(3000)
        self.sortie.setFont(QFont('Monospace', 9))

        racine = QVBoxLayout(self)
        racine.addWidget(titre)
        racine.addLayout(haut)
        racine.addLayout(grille)
        racine.addLayout(csv)
        racine.addWidget(self.sortie, 1)
        self.resize(980, 760)

        self.minuterie = QTimer(self)
        self.minuterie.timeout.connect(self.affiche_temperature)
        self.minuterie.start(2000)
        self.affiche_temperature()

    # ------------------------------------------------------------ commandes
    def cmd_validation(self):
        return ros(f'ros2 launch mycobot_gateway dream_multicam.launch.py '
                   f'pi_ip:={self.pi.text().strip()} model_name:={MODELE_REEL}')

    def cmd_synthetique(self):
        dossier = RACINE / 'results' / 'yolo_gazebo' / f'{datetime.now():%Y-%m-%d_%H-%M-%S}_lanceur_graine{self.graine.value()}'
        # Pas de mkdir : tri_yolo.launch.py cree log_dir et refuse un dossier existant.
        lancement = (f'ros2 launch mycobot_gateway tri_yolo.launch.py seed:={self.graine.value()} '
                     f'piece_reach:=0.28 dream:=true dashboard:=true dream_rate:=1.0 '
                     f'dream_model:={MODELE_GAZEBO} dream_cameras:=front,right,left,top log_dir:={dossier}')
        trieur = ('ros2 run mycobot_gateway sim_sorting_grasp --ros-args -p use_sim_time:=true '
                  '-p world_name:=tri_yolo -p pose_source:=perception -p max_attempts:=2 '
                  f'-p wall_timeout_scale:=20.0 -p csv_path:={dossier}/tri.csv')
        return ros(f'trap "kill 0" EXIT; {lancement} & sleep {ATTENTE_GAZEBO_S}; '
                   f'echo "trieur : depart"; {trieur}; wait')

    def cmd_reel_dream(self):
        self.dernier_reel_sans_marqueurs = self.sans_marqueurs.isChecked()
        script = ('lancer_tri_extrinseque_dream.py' if self.dernier_reel_sans_marqueurs
                  else 'lancer_pick_dashboard_final.py')
        return (f'cd {RACINE} && exec env MYCOBOT_PI={self.pi.text().strip()} '
                f'/usr/bin/python3 scripts/{script} --svpro-dream --dream')

    def cmd_reel_yolo(self):
        self.dernier_reel_sans_marqueurs = False
        return (f'cd {RACINE} && exec env MYCOBOT_PI={self.pi.text().strip()} '
                f'/usr/bin/python3 scripts/lancer_pick_dashboard_final.py --svpro-dream')

    # ------------------------------------------------------------- lancement
    def lance(self, programme):
        if programme.reel and not bridge_joignable(self.pi.text().strip()):
            QMessageBox.warning(self, 'Pi injoignable',
                                f'Aucune réponse du bridge sur {self.pi.text().strip()}:5005.\n'
                                'Lance gripper_bridge.py sur la Pi, ou corrige l\'adresse.')
            return
        for autre in self.programmes:
            if autre is not programme and autre.tourne():
                autre.arrete()
        self.arrete_journal()
        if programme.nom.startswith('tri_reel') and self.journal_csv.isChecked():
            self.demarre_journal()
        programme.demarre(programme.commande())

    def programme_fini(self, programme):
        if programme.nom.startswith('tri_reel'):
            self.arrete_journal()

    def arrete_tout(self):
        for p in self.programmes:
            p.arrete()
        self.arrete_journal()

    # ------------------------------------------------------- journal et CSV
    def demarre_journal(self):
        dossier = DREAM_REEL / f'journal_{datetime.now():%Y-%m-%d_%H-%M-%S}'
        self.journal_tri = subprocess.Popen(
            [str(RACINE / '.venv' / 'bin' / 'python'), 'scripts/essai_reel_csv.py', '--journal', str(dossier)],
            cwd=RACINE, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
        self.ecrit(f'— journal CSV : {dossier.name}')

    def arrete_journal(self):
        if self.journal_tri is not None and self.journal_tri.poll() is None:
            os.killpg(self.journal_tri.pid, signal.SIGINT)
            self.journal_tri.wait(timeout=10)
            self.ecrit('— journal CSV : arrêté')
        self.journal_tri = None

    def prochain_essai(self):
        dossiers = [d.name for d in DREAM_REEL.glob(f'{datetime.now():%Y-%m-%d}_tri_essai*_dream_yolo_reel')]
        numeros = [int(n.split('_tri_essai')[1].split('_')[0]) for n in dossiers]
        return max(numeros, default=0) + 1

    def genere_csv(self):
        if any(p.tourne() and p.nom.startswith('tri_reel') for p in self.programmes):
            QMessageBox.information(self, 'Tri en cours', 'Ferme d\'abord le dashboard du tri réel.')
            return
        sessions = sorted(DREAM_REEL.glob('*_dashboard'))
        journaux = sorted(DREAM_REEL.glob('journal_*'))
        if not sessions:
            QMessageBox.warning(self, 'Aucune session', f'Aucun dossier *_dashboard dans {DREAM_REEL}.')
            return
        arguments = [sessions[-1], '--essai', str(self.essai.value())]
        # Le journal du lanceur demarre quelques secondes avant le dashboard : un plus ancien
        # appartient a un autre tri.
        date = lambda nom: datetime.strptime(nom, '%Y-%m-%d_%H-%M-%S')
        debut = date(sessions[-1].name.removesuffix('_dashboard'))
        if journaux and 0 <= (debut - date(journaux[-1].name.removeprefix('journal_'))).total_seconds() <= JOURNAL_AVANT_S:
            arguments += ['--journal', journaux[-1]]
        else:
            self.ecrit('— CSV : pas de journal pour cette session (temperature.csv vide, run.yaml absent)')
        if self.dernier_reel_sans_marqueurs:
            arguments.append('--sans-marqueurs')
        # Sur deux coeurs et en nice 19 : la conversion a chauffe le PC a 80 degres le 15/10.
        commande = (f'cd {RACINE} && nice -n 19 taskset -c 0-1 .venv/bin/python scripts/essai_reel_csv.py '
                    + ' '.join(shlex.quote(str(a)) for a in arguments))
        self.ecrit(f'— CSV : {commande}')
        self.conversion = Programme(self, 'csv', f'CSV essai {self.essai.value()}', '', False, None)
        self.conversion.demarre(commande)
        self.essai.setValue(self.essai.value() + 1)

    # --------------------------------------------------------------- divers
    def teste_pi(self):
        ok = bridge_joignable(self.pi.text().strip())
        self.etat_pi.setText('bridge OK' if ok else 'bridge injoignable')
        self.etat_pi.setStyleSheet('color:#2e7d32' if ok else 'color:#c62828')

    def affiche_temperature(self):
        t = temperature_cpu()
        if t is None:
            return
        self.temperature.setText(f'CPU {t:.0f} °C')
        self.temperature.setStyleSheet('color:#c62828;font-weight:bold' if t >= 85
                                       else 'color:#ef6c00' if t >= 75 else '')

    def ecrit(self, texte):
        self.sortie.appendPlainText(texte)
        self.sortie.moveCursor(QTextCursor.End)

    def closeEvent(self, evenement):
        if any(p.tourne() for p in self.programmes):
            reponse = QMessageBox.question(self, 'Fermer le lanceur',
                                           'Des programmes tournent. Les arrêter et fermer ?')
            if reponse != QMessageBox.Yes:
                evenement.ignore()
                return
        self.arrete_tout()
        evenement.accept()


def main():
    application = QApplication(sys.argv)
    fenetre = Lanceur()
    fenetre.show()
    sys.exit(application.exec_())


if __name__ == '__main__':
    main()
