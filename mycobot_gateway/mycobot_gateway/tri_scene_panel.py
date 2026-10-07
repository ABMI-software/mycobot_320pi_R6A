"""One-button panel for /tri_scene/randomize (sorting scene, protocol step 2)."""

import tkinter as tk
from tkinter import ttk

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class TriScenePanel:
    def __init__(self, node):
        self.node = node
        self.client = node.create_client(Trigger, '/tri_scene/randomize')
        self.pending = None
        self.window = tk.Tk()
        self.window.title('Scène de tri')
        self.window.geometry('440x200')
        self.window.attributes('-topmost', True)
        self.status = tk.StringVar(value='Connexion à la simulation…')
        frame = ttk.Frame(self.window, padding=18)
        frame.pack(fill='both', expand=True)
        ttk.Label(frame, text='Scène de tri', font=('Sans', 15, 'bold')).pack(anchor='w')
        ttk.Label(frame, text='Nouvelles positions des 4 pièces et des 4 bacs, sans relancer Gazebo. '
                              'À utiliser entre deux tris.', wraplength=400).pack(anchor='w', pady=(5, 12))
        self.button = ttk.Button(frame, text='🎲  Randomiser', command=self.request)
        self.button.pack(fill='x', ipady=7)
        ttk.Label(frame, textvariable=self.status, wraplength=400).pack(anchor='w', pady=(10, 0))
        self.window.after(50, self.tick)

    def request(self):
        self.pending = self.client.call_async(Trigger.Request())
        self.status.set('Bras en pose d’observation, puis nouvelle scène…')

    def tick(self):
        if not rclpy.ok():
            self.window.destroy()
            return
        rclpy.spin_once(self.node, timeout_sec=0)
        if self.pending is not None and self.pending.done():
            self.status.set(self.pending.result().message)
            self.pending = None
        ready = self.pending is None and self.client.service_is_ready()
        self.button.configure(state='normal' if ready else 'disabled')
        if self.pending is None and not self.client.service_is_ready():
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
