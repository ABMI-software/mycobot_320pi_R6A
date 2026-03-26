#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Simple GUI - Version Tour (PC)
Interface graphique pour contrôler le MyCobot via le bridge TCP.

Cette version tourne sur le PC Tour et envoie les commandes
au robot via le bridge_tour → bridge_pi.

Usage:
    ros2 run mycobot_gateway simple_gui
"""

import tkinter as tk
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import threading
import time


class SimpleGuiNode(Node):
    """ROS2 Node pour la communication avec le bridge"""
    
    def __init__(self):
        super().__init__('simple_gui_node')
        
        # Publisher pour envoyer les commandes au Pi
        self.cmd_publisher = self.create_publisher(String, '/to_robot', 10)
        
        # Subscriber pour recevoir les réponses du Pi
        self.feedback_sub = self.create_subscription(
            String, '/from_robot', self.feedback_callback, 10)
        
        # État
        self.last_feedback = None
        self.current_angles = [0, 0, 0, 0, 0, 0]
        self.current_coords = [0, 0, 0, 0, 0, 0]
        
        self.get_logger().info("🖥️ Simple GUI Node initialized")
    
    def feedback_callback(self, msg):
        """Traite les réponses du robot"""
        self.last_feedback = msg.data
        
        # Parser les angles si présents
        if msg.data.startswith("angles:"):
            try:
                angles_str = msg.data.replace("angles:", "")
                self.current_angles = eval(angles_str)
            except:
                pass
        
        # Parser les coords si présents
        if msg.data.startswith("coords:"):
            try:
                coords_str = msg.data.replace("coords:", "")
                self.current_coords = eval(coords_str)
            except:
                pass
    
    def send_angles(self, angles, speed=50):
        """Envoie des angles au robot"""
        cmd = {"action": "send_angles", "angles": angles, "speed": speed}
        msg = String()
        msg.data = json.dumps(cmd)
        self.cmd_publisher.publish(msg)
        self.get_logger().info(f"📤 Angles: {angles}, speed: {speed}")
    
    def send_coords(self, coords, speed=50, mode=0):
        """Envoie des coordonnées au robot"""
        cmd = {"action": "send_coords", "coords": coords, "speed": speed, "mode": mode}
        msg = String()
        msg.data = json.dumps(cmd)
        self.cmd_publisher.publish(msg)
        self.get_logger().info(f"📤 Coords: {coords}, speed: {speed}")
    
    def gripper_open(self):
        """Ouvre la pince"""
        cmd = {"action": "gripper_open"}
        msg = String()
        msg.data = json.dumps(cmd)
        self.cmd_publisher.publish(msg)
        self.get_logger().info("📤 Gripper OPEN")
    
    def gripper_close(self):
        """Ferme la pince"""
        cmd = {"action": "gripper_close"}
        msg = String()
        msg.data = json.dumps(cmd)
        self.cmd_publisher.publish(msg)
        self.get_logger().info("📤 Gripper CLOSE")
    
    def get_angles(self):
        """Demande les angles actuels"""
        cmd = {"action": "get_angles"}
        msg = String()
        msg.data = json.dumps(cmd)
        self.cmd_publisher.publish(msg)
    
    def get_coords(self):
        """Demande les coordonnées actuelles"""
        cmd = {"action": "get_coords"}
        msg = String()
        msg.data = json.dumps(cmd)
        self.cmd_publisher.publish(msg)


class SimpleGuiWindow:
    """Interface graphique Tkinter"""
    
    def __init__(self, handle, ros_node):
        self.node = ros_node
        self.win = handle
        self.win.resizable(0, 0)
        
        self.speed = 50
        self.model = 0
        
        # Variables par défaut
        self.speed_var = tk.StringVar()
        self.speed_var.set(str(self.speed))
        
        # Récupérer les données initiales
        self.res_angles = [[0, 0, 0, 0, 0, 0], self.speed, self.model]
        self.record_coords = [[0, 0, 0, 0, 0, 0], self.speed, self.model]
        
        # Taille et position de la fenêtre
        self.ws = self.win.winfo_screenwidth()
        self.hs = self.win.winfo_screenheight()
        x = (self.ws / 2) - 240
        y = (self.hs / 2) - 280
        self.win.geometry("520x500+{}+{}".format(int(x), int(y)))
        
        # Layout
        self.set_layout()
        self.need_input()
        self.show_init()
        self.create_buttons()
        
        # Mise à jour périodique
        self.update_display()
    
    def set_layout(self):
        """Configure les frames"""
        self.frmLT = tk.Frame(width=200, height=200)  # Joint inputs
        self.frmLC = tk.Frame(width=200, height=200)  # Display
        self.frmLB = tk.Frame(width=200, height=200)  # Controls
        self.frmRT = tk.Frame(width=200, height=200)  # Coord inputs
        
        self.frmLT.grid(row=0, column=0, padx=5, pady=5)
        self.frmLC.grid(row=1, column=0, columnspan=2, padx=5, pady=5)
        self.frmLB.grid(row=2, column=0, columnspan=2, padx=5, pady=5)
        self.frmRT.grid(row=0, column=1, padx=5, pady=5)
    
    def need_input(self):
        """Crée les champs de saisie"""
        # Labels Joints
        for i in range(6):
            tk.Label(self.frmLT, text=f"Joint {i+1}").grid(row=i, column=0, sticky="e")
        
        # Labels Coords
        coord_labels = ["X", "Y", "Z", "RX", "RY", "RZ"]
        for i, label in enumerate(coord_labels):
            tk.Label(self.frmRT, text=f" {label} ").grid(row=i, column=0, sticky="e")
        
        # Variables pour les joints
        self.joint_vars = []
        self.joint_entries = []
        for i in range(6):
            var = tk.StringVar()
            var.set("0")
            self.joint_vars.append(var)
            entry = tk.Entry(self.frmLT, textvariable=var, width=12)
            entry.grid(row=i, column=1, pady=2)
            self.joint_entries.append(entry)
        
        # Variables pour les coordonnées
        self.coord_vars = []
        self.coord_entries = []
        for i in range(6):
            var = tk.StringVar()
            var.set("0")
            self.coord_vars.append(var)
            entry = tk.Entry(self.frmRT, textvariable=var, width=12)
            entry.grid(row=i, column=1, pady=2)
            self.coord_entries.append(entry)
        
        # Vitesse
        tk.Label(self.frmLB, text="Vitesse:").grid(row=0, column=0, sticky="e")
        self.speed_entry = tk.Entry(self.frmLB, textvariable=self.speed_var, width=8)
        self.speed_entry.grid(row=0, column=1, padx=5)
    
    def show_init(self):
        """Crée l'affichage des valeurs actuelles"""
        # Titre
        tk.Label(self.frmLC, text="─── Valeurs Actuelles ───", font=("Arial", 10, "bold")).grid(
            row=0, column=0, columnspan=6, pady=5)
        
        # Labels affichage joints
        for i in range(6):
            tk.Label(self.frmLC, text=f"J{i+1}:").grid(row=1, column=i, sticky="e")
        
        # Variables d'affichage joints
        self.display_joint_vars = []
        for i in range(6):
            var = tk.StringVar()
            var.set("0°")
            self.display_joint_vars.append(var)
            tk.Label(self.frmLC, textvariable=var, font=("Arial", 9), width=8,
                    bg="white", relief="sunken").grid(row=2, column=i, padx=2, pady=2)
        
        # Labels affichage coords
        coord_labels = ["X:", "Y:", "Z:", "RX:", "RY:", "RZ:"]
        for i, label in enumerate(coord_labels):
            tk.Label(self.frmLC, text=label).grid(row=3, column=i, sticky="e")
        
        # Variables d'affichage coords
        self.display_coord_vars = []
        for i in range(6):
            var = tk.StringVar()
            var.set("0")
            self.display_coord_vars.append(var)
            tk.Label(self.frmLC, textvariable=var, font=("Arial", 9), width=8,
                    bg="white", relief="sunken").grid(row=4, column=i, padx=2, pady=2)
    
    def create_buttons(self):
        """Crée les boutons de contrôle"""
        # Bouton SET Joints
        tk.Button(self.frmLT, text="SET Joints", width=10, 
                 command=self.send_joints, bg="#4CAF50", fg="white").grid(
            row=6, column=0, columnspan=2, pady=5)
        
        # Bouton SET Coords
        tk.Button(self.frmRT, text="SET Coords", width=10,
                 command=self.send_coords, bg="#2196F3", fg="white").grid(
            row=6, column=0, columnspan=2, pady=5)
        
        # Boutons Gripper
        tk.Button(self.frmLB, text="🖐 Ouvrir", width=10,
                 command=self.node.gripper_open, bg="#FF9800").grid(
            row=1, column=0, padx=5, pady=10)
        tk.Button(self.frmLB, text="✊ Fermer", width=10,
                 command=self.node.gripper_close, bg="#FF5722").grid(
            row=1, column=1, padx=5, pady=10)
        
        # Boutons Home/Zero
        tk.Button(self.frmLB, text="🏠 HOME", width=10,
                 command=self.go_home, bg="#9C27B0", fg="white").grid(
            row=1, column=2, padx=5, pady=10)
        tk.Button(self.frmLB, text="0️⃣ ZERO", width=10,
                 command=self.go_zero, bg="#607D8B", fg="white").grid(
            row=1, column=3, padx=5, pady=10)
        
        # Bouton Refresh
        tk.Button(self.frmLB, text="🔄 Refresh", width=10,
                 command=self.refresh_data, bg="#00BCD4").grid(
            row=0, column=2, padx=5)
        
        # Bouton STOP
        tk.Button(self.frmLB, text="🛑 STOP", width=10,
                 command=self.emergency_stop, bg="#F44336", fg="white").grid(
            row=0, column=3, padx=5)
    
    def send_joints(self):
        """Envoie les angles joints"""
        try:
            angles = [float(var.get()) for var in self.joint_vars]
            speed = int(float(self.speed_var.get()))
            self.node.send_angles(angles, speed)
        except ValueError as e:
            self.node.get_logger().error(f"Erreur de valeur: {e}")
    
    def send_coords(self):
        """Envoie les coordonnées"""
        try:
            coords = [float(var.get()) for var in self.coord_vars]
            speed = int(float(self.speed_var.get()))
            self.node.send_coords(coords, speed, self.model)
        except ValueError as e:
            self.node.get_logger().error(f"Erreur de valeur: {e}")
    
    def go_home(self):
        """Va à la position HOME"""
        cmd = {"action": "go_home"}
        msg = String()
        msg.data = json.dumps(cmd)
        self.node.cmd_publisher.publish(msg)
        self.node.get_logger().info("📤 GO HOME")
    
    def go_zero(self):
        """Va à la position ZERO"""
        cmd = {"action": "go_zero"}
        msg = String()
        msg.data = json.dumps(cmd)
        self.node.cmd_publisher.publish(msg)
        self.node.get_logger().info("📤 GO ZERO")
    
    def emergency_stop(self):
        """Arrêt d'urgence"""
        cmd = {"action": "emergency_stop"}
        msg = String()
        msg.data = json.dumps(cmd)
        self.node.cmd_publisher.publish(msg)
        self.node.get_logger().warn("🚨 EMERGENCY STOP")
    
    def refresh_data(self):
        """Rafraîchit les données du robot"""
        self.node.get_angles()
        self.node.get_coords()
    
    def update_display(self):
        """Met à jour l'affichage périodiquement"""
        # Mettre à jour les joints affichés
        for i, var in enumerate(self.display_joint_vars):
            if i < len(self.node.current_angles):
                var.set(f"{self.node.current_angles[i]:.1f}°")
        
        # Mettre à jour les coords affichés
        for i, var in enumerate(self.display_coord_vars):
            if i < len(self.node.current_coords):
                var.set(f"{self.node.current_coords[i]:.1f}")
        
        # Relancer la mise à jour
        self.win.after(500, self.update_display)
    
    def run(self):
        """Boucle principale"""
        while True:
            try:
                self.win.update()
                rclpy.spin_once(self.node, timeout_sec=0.01)
                time.sleep(0.01)
            except tk.TclError as e:
                if "application has been destroyed" in str(e):
                    break
                else:
                    raise


def main(args=None):
    rclpy.init(args=args)
    
    # Créer le node ROS2
    ros_node = SimpleGuiNode()
    
    # Créer la fenêtre Tkinter
    window = tk.Tk()
    window.title("MyCobot GUI - Tour Control")
    
    # Démarrer la GUI
    gui = SimpleGuiWindow(window, ros_node)
    
    try:
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        ros_node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
