"""Panel for the sorting scene: Randomiser (protocol step 2) and Démarrer le tri.

Démarrer runs sim_sorting_grasp once on the current scene. It is locked once a
sort has run, until Randomiser places a new scene, so no scene is sorted twice.
Randomiser is locked during a sort.

With "Enregistrer l'essai en CSV" ticked (needs log_dir), the trial gets its
own folder next to log_dir, <date>_tri_essai<k>_dream_yolo_gazebo:
scene.csv, trajectoire.csv and resultat.csv (written by tri_dream_dashboard,
tri_essai), temperature.csv (here, one row per second), and brut/ (the
sorter's tri.csv and log, and the raw DREAM / YOLO rows of the trial); one
summary row is appended to bilan_tris.csv beside the trial folders (tri_bilan).
"""

import datetime
import os
from pathlib import Path
import re
import signal
import subprocess
import tkinter as tk
from tkinter import ttk

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import Trigger

from . import tri_bilan
from .tri_scene_randomizer import ARM_JOINTS

DRAW = re.compile(r'Tirage (\d+) ')
# DREAM poses and the dashboard's grasp rows land ~1 s after their image.
BILAN_DELAY_MS = 5000
TEMPERATURE_PERIOD_MS = 1000


def cpu_temperature():
    """'Package id 0' of lm-sensors (the hottest core), °C, or None."""
    out = subprocess.run(['sensors'], capture_output=True, text=True).stdout
    match = re.search(r'Package id 0:\s+\+([\d.]+)', out)
    return float(match.group(1)) if match else None


def gazebo_frozen():
    """True when the Gazebo server is stopped (SIGSTOP of a thermal guard)."""
    pids = subprocess.run(['pgrep', '-f', 'gz sim server'], capture_output=True, text=True).stdout.split()
    for pid in pids:
        try:
            state = Path(f'/proc/{pid}/stat').read_text().split(') ')[1].split()[0]
        except (FileNotFoundError, IndexError):
            continue
        return state == 'T'
    return False


def next_essai_dir(parent):
    day = datetime.date.today().isoformat()
    taken = [int(m.group(1)) for p in Path(parent).glob(f'{day}_tri_essai*_dream_yolo_gazebo')
             if (m := re.search(r'_tri_essai(\d+)_', p.name))]
    return Path(parent) / f'{day}_tri_essai{max(taken, default=0) + 1}_dream_yolo_gazebo'


class TriScenePanel:
    def __init__(self, node):
        self.node = node
        self.log_dir = str(node.declare_parameter('log_dir', '').value)
        self.seed = int(node.declare_parameter('seed', 0).value)
        self.world = str(node.declare_parameter('world_name', 'tri_yolo').value)
        self.wall_scale = float(node.declare_parameter('wall_timeout_scale', 1.0).value)
        self.client = node.create_client(Trigger, '/tri_scene/randomize')
        self.essai_pub = node.create_publisher(String, '/tri_scene/essai', 10)
        node.create_subscription(String, '/tri_scene/status', self.on_status, 10)
        node.create_subscription(JointState, '/joint_states', self.on_joints, 10)
        self.busy = False
        self.pending = None
        self.fresh = True
        self.draw = 0
        self.sorter = None
        self.finishing = False
        self.sort = None
        self.window = tk.Tk()
        self.window.title('Scène de tri')
        self.window.geometry('500x330')
        self.window.attributes('-topmost', True)
        self.window.protocol('WM_DELETE_WINDOW', self.close)
        self.status = tk.StringVar(value='Connexion à la simulation…')
        self.record = tk.BooleanVar(value=False)
        frame = ttk.Frame(self.window, padding=18)
        frame.pack(fill='both', expand=True)
        ttk.Label(frame, text='Scène de tri', font=('Sans', 15, 'bold')).pack(anchor='w')
        ttk.Label(frame, text='Randomiser : nouvelles positions des 4 pièces et des 4 bacs, sans '
                              'relancer Gazebo. Démarrer : trie la scène une fois ; il faut '
                              'randomiser avant le tri suivant.', wraplength=460).pack(anchor='w', pady=(5, 12))
        self.random_button = ttk.Button(frame, text='🎲  Randomiser', command=self.request)
        self.random_button.pack(fill='x', ipady=7)
        self.record_box = ttk.Checkbutton(frame, text='Enregistrer l’essai en CSV', variable=self.record)
        self.record_box.pack(anchor='w', pady=(10, 0))
        if not self.log_dir:
            self.record_box.configure(state='disabled', text='Enregistrer l’essai en CSV (lancer avec log_dir:=…)')
        self.start_button = ttk.Button(frame, text='▶  Démarrer le tri', command=self.start)
        self.start_button.pack(fill='x', ipady=7, pady=(6, 0))
        ttk.Label(frame, textvariable=self.status, wraplength=460).pack(anchor='w', pady=(10, 0))
        self.window.after(50, self.tick)

    def sim_now(self):
        return self.node.get_clock().now().nanoseconds * 1e-9

    def on_status(self, msg):
        self.busy = False
        self.status.set(msg.data)
        match = DRAW.match(msg.data)
        if match:
            self.draw = int(match.group(1))
            self.fresh = True

    def on_joints(self, msg):
        if self.sort is None or self.sort['dir'] is None:
            return
        positions = dict(zip(msg.name, msg.position))
        if all(j in positions for j in ARM_JOINTS):
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            self.sort['times'].append(stamp)
            self.sort['q'].append(np.degrees([positions[j] for j in ARM_JOINTS]))

    def request(self):
        self.pending = self.client.call_async(Trigger.Request())
        self.status.set('Bras en pose d’observation, puis nouvelle scène…')

    def start(self):
        record = self.record.get() and bool(self.log_dir)
        essai = next_essai_dir(Path(self.log_dir).parent) if record else None
        cmd = ['ros2', 'run', 'mycobot_gateway', 'sim_sorting_grasp', '--ros-args',
               '-p', 'use_sim_time:=true', '-p', f'world_name:={self.world}',
               '-p', 'pose_source:=perception', '-p', 'max_attempts:=2',
               '-p', f'wall_timeout_scale:={self.wall_scale}']
        log = subprocess.DEVNULL
        if essai is not None:
            (essai / 'brut').mkdir(parents=True)
            cmd += ['-p', f'csv_path:={essai / "brut" / "tri.csv"}']
            log = open(essai / 'brut' / 'tri.log', 'w')
            self.essai_pub.publish(String(data=str(essai)))
        self.sorter = subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        self.sort = {'t0': self.sim_now(), 'since': datetime.datetime.now().isoformat(timespec='seconds'),
                     'dir': essai, 'log': log, 'times': [], 'q': [], 'temperature': None}
        self.fresh = False
        if essai is not None:
            self.sort['temperature'] = open(essai / 'temperature.csv', 'w')
            self.sort['temperature'].write('heure,temps_s,temperature_c,gel\n')
            self.window.after(TEMPERATURE_PERIOD_MS, self.log_temperature)
            self.status.set(f'Tri en cours, enregistré dans {essai.name}…')
        else:
            self.status.set('Tri en cours (sans enregistrement)…')

    def log_temperature(self):
        s = self.sort
        if s is None or s['temperature'] is None:
            return
        t = cpu_temperature()
        s['temperature'].write(f"{datetime.datetime.now().strftime('%H:%M:%S')},"
                               f"{self.sim_now() - s['t0']:.1f},{'' if t is None else f'{t:.0f}'},"
                               f"{'oui' if gazebo_frozen() else 'non'}\n")
        s['temperature'].flush()
        self.window.after(TEMPERATURE_PERIOD_MS, self.log_temperature)

    def sorter_done(self):
        self.sorter = None
        self.sort['t1'] = self.sim_now()
        if self.sort['log'] is not subprocess.DEVNULL:
            self.sort['log'].close()
        if self.sort['dir'] is None:
            self.status.set('Tri terminé (sans enregistrement). Randomiser pour le tri suivant.')
            self.sort = None
            return
        self.finishing = True
        self.status.set('Tri terminé, écriture des CSV…')
        self.window.after(BILAN_DELAY_MS, self.write_bilan)

    def write_bilan(self):
        s, self.sort = self.sort, None
        s['temperature'].close()
        self.essai_pub.publish(String(data=''))
        row = tri_bilan.write_bilan(
            self.log_dir, s['dir'], s['dir'].parent / 'bilan_tris.csv', self.seed, self.draw,
            s['t0'], s['t1'], s['since'], tri_bilan.ArmTimeline(s['times'], s['q']),
            s['dir'] / 'brut' / 'tri.csv')
        self.finishing = False
        self.status.set(
            f"{s['dir'].name} : {row['objets_tries']} triés. DREAM statique "
            f"{row['dream_stat_detection_pct']} % · {row['dream_stat_mediane_px']} px, dynamique "
            f"{row['dream_dyn_detection_pct']} % · {row['dream_dyn_mediane_px']} px. YOLO statique "
            f"{row['yolo_stat_detection_pct']} % · {row['yolo_stat_mediane_mm']} mm, dynamique "
            f"{row['yolo_dyn_detection_pct']} % · {row['yolo_dyn_mediane_mm']} mm. "
            f"Randomiser pour le tri suivant.")

    def close(self):
        if self.sorter is not None and self.sorter.poll() is None:
            os.killpg(self.sorter.pid, signal.SIGINT)
        self.window.destroy()

    def tick(self):
        if not rclpy.ok():
            self.close()
            return
        rclpy.spin_once(self.node, timeout_sec=0)
        if self.pending is not None and self.pending.done():
            result = self.pending.result()
            self.busy = result.success
            self.status.set(result.message)
            self.pending = None
        if self.sorter is not None and self.sorter.poll() is not None:
            self.sorter_done()
        sim_ready = self.client.service_is_ready()
        idle = self.pending is None and not self.busy and self.sorter is None and not self.finishing
        self.random_button.configure(state='normal' if idle and sim_ready else 'disabled')
        self.start_button.configure(state='normal' if idle and sim_ready and self.fresh else 'disabled')
        if self.log_dir:
            self.record_box.configure(state='normal' if idle else 'disabled')
        if self.pending is None and not sim_ready:
            self.status.set('En attente de la simulation…')
        self.window.after(50, self.tick)


def main(args=None):
    rclpy.init(args=args)
    node = Node('tri_scene_panel')
    panel = TriScenePanel(node)
    try:
        panel.window.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
