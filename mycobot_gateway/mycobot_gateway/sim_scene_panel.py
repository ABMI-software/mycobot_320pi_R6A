"""Small desktop panel for the Gazebo randomize / pick controls."""

import json
import time
import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger


class ScenePanel:
    def __init__(self, node):
        self.node = node
        self.window = tk.Tk()
        self.window.title('Gazebo — Cube et bac rouges')
        self.window.geometry('470x270')
        self.window.minsize(430, 250)
        self.window.attributes('-topmost', True)
        self.busy = True
        self.can_pick = False
        self.last_state = 0.
        self.pending = None
        self.status = tk.StringVar(value='Connexion à la simulation…')
        frame = ttk.Frame(self.window, padding=18)
        frame.pack(fill='both', expand=True)
        ttk.Label(frame, text='Cube et bac rouges', font=('Sans', 15, 'bold')).pack(anchor='w')
        ttk.Label(frame, text='Nouvelles positions dans la zone accessible du bras.').pack(anchor='w', pady=(5, 14))
        self.random_button = ttk.Button(frame, text='🎲  Randomiser cube + bac',
                                       command=lambda: self.request('randomize'))
        self.random_button.pack(fill='x', ipady=7)
        self.pick_button = ttk.Button(frame, text='▶  Lancer la prise',
                                     command=lambda: self.request('pick'))
        self.pick_button.pack(fill='x', ipady=7, pady=(8, 12))
        ttk.Label(frame, textvariable=self.status, wraplength=420).pack(anchor='w')
        self.clients = {action: node.create_client(Trigger, f'/real_table/{action}')
                        for action in ('randomize', 'pick')}
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String, '/real_table/control_state', self.on_state, qos)
        self.refresh_buttons()
        self.window.after(50, self.tick)

    def on_state(self, msg):
        value = json.loads(msg.data)
        self.last_state = time.monotonic()
        self.busy = value['busy']
        self.can_pick = value['can_pick']
        self.status.set(value['message'])

    def refresh_buttons(self):
        ready = (time.monotonic() - self.last_state < 10.0
                 and not self.busy and self.pending is None)
        self.random_button.configure(state='normal' if ready and self.clients['randomize'].service_is_ready() else 'disabled')
        self.pick_button.configure(state='normal' if ready and self.can_pick and self.clients['pick'].service_is_ready() else 'disabled')

    def request(self, action):
        if self.pending is not None or self.busy:
            return
        self.pending = self.clients[action].call_async(Trigger.Request())
        self.status.set('Demande envoyée…')
        self.refresh_buttons()

    def tick(self):
        if not rclpy.ok():
            self.window.destroy()
            return
        rclpy.spin_once(self.node, timeout_sec=0)
        if self.pending is not None and self.pending.done():
            try:
                result = self.pending.result()
                self.status.set(result.message)
                if result.success:
                    self.busy = True
            except Exception as exc:
                self.status.set(f'Simulation indisponible : {exc}')
            self.pending = None
        if time.monotonic() - self.last_state > 10:
            self.status.set('En attente de la simulation…')
        self.refresh_buttons()
        self.window.after(50, self.tick)


def main(args=None):
    rclpy.init(args=args)
    node = Node('sim_scene_panel')
    panel = ScenePanel(node)
    try:
        panel.window.mainloop()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
