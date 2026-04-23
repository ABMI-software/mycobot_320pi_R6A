#!/usr/bin/env python3
"""ABMI-branded teleoperation dashboard — Home / Analytics / Tuning tabs.

Modernized from the original single-view dashboard: three tabs to separate
concerns, KPI cards for at-a-glance sim↔real comparison, quick-action
buttons, integrated camera panel, and cleaner typography on the ABMI
brand palette (navy + pink).

Topics
  - Sub /teleop/hand_xyz                           (Wilor hand position)
  - Sub /teleop/camera/image                       (webcam JPEG preview)
  - Sub /mycobot_controller/joint_trajectory       (commanded angles)
  - Sub /joint_states                              (Gazebo actual)
  - Sub /from_robot                                (real-robot ANGLES)
  - Pub /to_robot                                  (periodic get_angles poll)
  - Pub /teleop/gains                              (live gain sliders)
  - Pub /teleop/recalibrate                        (Empty — reset origin)
"""
from __future__ import annotations

import base64
import os
import re
import subprocess
import threading
import time
import tkinter as tk
from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np
import roslibpy

import ttkbootstrap as ttkb
from ttkbootstrap.constants import *

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


# ------------------------------ Constants ------------------------------ #

MYCOBOT_JOINTS = [
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
]
SHORT_NAMES = ["J1", "J2", "J3", "J4", "J5", "J6"]
LONG_NAMES = ["J1 yaw", "J2 shoulder", "J3 elbow", "J4 wrist1", "J5 yaw", "J6 roll"]

PLOT_WINDOW_S = 10.0
MAX_SAMPLES = 800
REFRESH_MS = 250
REAL_POLL_PERIOD_S = 0.3
MODE_FRESH_WINDOW_S = 2.0
CAMERA_REFRESH_MS = 120

# ---- ABMI brand palette (from the logo) ----
ABMI_NAVY   = "#1B1A3E"
ABMI_PINK   = "#E6417A"
ABMI_LIGHT  = "#F5F3FA"
ABMI_DARK   = "#0F0E23"
ABMI_MUTED  = "#3A3961"
ABMI_GREEN  = "#2FDC89"
ABMI_YELLOW = "#FFD43B"
ABMI_BLUE   = "#4DABF7"

BG     = ABMI_DARK
PANEL  = ABMI_NAVY
FG     = ABMI_LIGHT
GRID   = ABMI_MUTED
ACCENT = ABMI_PINK

JOINT_COLORS = ["#E6417A", "#f59f00", "#2FDC89", "#4DABF7", "#b197fc", "#e8590c"]

LOGO_CANDIDATES = [
    Path(__file__).parent / "assets" / "abmi_logo.png",
    Path(__file__).parent / "assets" / "abmi_logo.jpg",
    Path(__file__).parent / "assets" / "ABMI_logo.png",
]


def _style_axis(ax, title: str, xlabel: str, ylabel: str) -> None:
    ax.set_facecolor(BG)
    ax.set_title(title, color=FG, fontsize=10, pad=6)
    ax.set_xlabel(xlabel, color=FG, fontsize=9)
    ax.set_ylabel(ylabel, color=FG, fontsize=9)
    ax.tick_params(colors=FG, labelsize=8)
    for s in ax.spines.values():
        s.set_color(GRID)
    ax.grid(True, alpha=0.25, color=GRID, linestyle="-", linewidth=0.5)


# ---------------------------- ROS Client ------------------------------- #

class RosClient:
    def __init__(self, host: str = "localhost", port: int = 9090) -> None:
        self.ros = roslibpy.Ros(host=host, port=port)
        self.ros.run()
        if not self.ros.is_connected:
            raise SystemExit(f"Cannot reach rosbridge at ws://{host}:{port}")

        self._lock = threading.Lock()
        self.hand_xyz = deque(maxlen=MAX_SAMPLES)
        self.cmd_joints = deque(maxlen=MAX_SAMPLES)
        self.actual_sim = deque(maxlen=MAX_SAMPLES)
        self.actual_real = deque(maxlen=MAX_SAMPLES)
        self.last_sim_t = 0.0
        self.last_real_t = 0.0
        self._last_camera_b64: str | None = None
        self._camera_lock = threading.Lock()

        self._t0 = time.perf_counter()

        self._t_hand = roslibpy.Topic(self.ros, "/teleop/hand_xyz",
                                      "geometry_msgs/Vector3Stamped")
        self._t_hand.subscribe(self._on_hand)
        self._t_cmd = roslibpy.Topic(self.ros, "/mycobot_controller/joint_trajectory",
                                     "trajectory_msgs/JointTrajectory")
        self._t_cmd.subscribe(self._on_cmd)
        self._t_sim = roslibpy.Topic(self.ros, "/joint_states",
                                     "sensor_msgs/JointState")
        self._t_sim.subscribe(self._on_sim_actual)
        self._t_real = roslibpy.Topic(self.ros, "/from_robot",
                                      "std_msgs/String")
        self._t_real.subscribe(self._on_real_feedback)
        self._t_camera = roslibpy.Topic(self.ros, "/teleop/camera/image",
                                        "sensor_msgs/CompressedImage")
        self._t_camera.subscribe(self._on_camera)

        self._p_gains = roslibpy.Topic(self.ros, "/teleop/gains",
                                       "std_msgs/Float64MultiArray")
        self._p_gains.advertise()
        self._p_recal = roslibpy.Topic(self.ros, "/teleop/recalibrate",
                                       "std_msgs/Empty")
        self._p_recal.advertise()
        self._p_to_robot = roslibpy.Topic(self.ros, "/to_robot",
                                          "std_msgs/String")
        self._p_to_robot.advertise()

        self._poll_stop = threading.Event()
        threading.Thread(target=self._poll_real, daemon=True).start()

    def _now(self) -> float:
        return time.perf_counter() - self._t0

    # ---- subscribers ----
    def _on_hand(self, msg: dict) -> None:
        v = msg["vector"]
        with self._lock:
            self.hand_xyz.append((self._now(), v["x"], v["y"], v["z"]))

    def _on_cmd(self, msg: dict) -> None:
        points = msg.get("points") or []
        names = msg.get("joint_names") or []
        if not points or not names: return
        positions = points[0].get("positions") or []
        try: reordered = [positions[names.index(j)] for j in MYCOBOT_JOINTS]
        except ValueError: return
        with self._lock:
            self.cmd_joints.append((self._now(), reordered))

    def _on_sim_actual(self, msg: dict) -> None:
        names = msg.get("name") or []; positions = msg.get("position") or []
        try: reordered = [positions[names.index(j)] for j in MYCOBOT_JOINTS]
        except ValueError: return
        now = self._now()
        with self._lock:
            self.actual_sim.append((now, reordered)); self.last_sim_t = now

    _ANGLES_RX = re.compile(r"ANGLES:\s*\[([^\]]+)\]")

    def _on_real_feedback(self, msg: dict) -> None:
        data = msg.get("data", "")
        m = self._ANGLES_RX.search(data)
        if m is None: return
        try: values_deg = [float(x) for x in m.group(1).split(",")]
        except ValueError: return
        if len(values_deg) != 6: return
        values_rad = [np.radians(v) for v in values_deg]
        now = self._now()
        with self._lock:
            self.actual_real.append((now, values_rad)); self.last_real_t = now

    def _on_camera(self, msg: dict) -> None:
        data = msg.get("data")
        if not data: return
        with self._camera_lock:
            self._last_camera_b64 = data

    def take_camera_frame(self):
        """Return a PhotoImage of the most recent frame (or None)."""
        try:
            from PIL import Image, ImageTk
        except ImportError:
            return None
        with self._camera_lock:
            b64 = self._last_camera_b64
        if not b64:
            return None
        try:
            import io
            raw = base64.b64decode(b64)
            img = Image.open(io.BytesIO(raw))
            return ImageTk.PhotoImage(img)
        except Exception:
            return None

    # ---- periodic real-robot polling ----
    def _poll_real(self) -> None:
        while not self._poll_stop.is_set():
            try:
                self._p_to_robot.publish(roslibpy.Message({"data": "get_angles"}))
            except Exception:
                pass
            time.sleep(REAL_POLL_PERIOD_S)

    # ---- publishers ----
    def publish_gains(self, gx, gy, gz, tfs):
        self._p_gains.publish(roslibpy.Message({"data": [gx, gy, gz, tfs]}))

    def request_recalibrate(self):
        self._p_recal.publish(roslibpy.Message({}))

    def send_raw(self, text: str) -> None:
        """Publish an arbitrary string on /to_robot (e.g. 'home', 'stop')."""
        self._p_to_robot.publish(roslibpy.Message({"data": text}))

    # ---- state ----
    def detect_mode(self) -> str:
        now = self._now()
        with self._lock:
            sim_live = (now - self.last_sim_t) < MODE_FRESH_WINDOW_S and bool(self.actual_sim)
            real_live = (now - self.last_real_t) < MODE_FRESH_WINDOW_S and bool(self.actual_real)
        if sim_live and real_live: return "BOTH"
        if sim_live: return "SIM"
        if real_live: return "REAL"
        return "OFFLINE"

    def snapshot(self):
        with self._lock:
            return (list(self.hand_xyz), list(self.cmd_joints),
                    list(self.actual_sim), list(self.actual_real))

    def terminate(self) -> None:
        self._poll_stop.set()
        for t in (self._t_hand, self._t_cmd, self._t_sim, self._t_real, self._t_camera):
            try: t.unsubscribe()
            except Exception: pass
        for t in (self._p_gains, self._p_recal, self._p_to_robot):
            try: t.unadvertise()
            except Exception: pass
        self.ros.terminate()


# ------------------------- Dynamic action button ---------------------- #

class ActionButton(ttkb.Button):
    """ttkbootstrap button with hover tooltip and short-lived click feedback.

    On click it calls ``action()``, briefly shows ``"..."``, then swaps to
    ``"✓"``/``"✗"`` for ~1.4s before returning to its original label. It
    also disables itself during the in-flight window so accidental double
    clicks are absorbed, and optionally reports to a status callback so
    the Home tab can show a toast-style message.
    """

    def __init__(self, parent, *, text: str, tooltip: str, bootstyle,
                 action, on_status=None, **kwargs):
        self._base_text = text
        self._tooltip_text = tooltip
        self._on_status = on_status
        self._action = action
        self._tip_window: tk.Toplevel | None = None
        self._pending_reset = None
        super().__init__(parent, text=text, bootstyle=bootstyle,
                         command=self._handle_click, **kwargs)
        self.bind("<Enter>", self._show_tip)
        self.bind("<Leave>", self._hide_tip)

    # ---- hover tooltip ----
    def _show_tip(self, _evt=None) -> None:
        if self._tip_window is not None or not self._tooltip_text:
            return
        x = self.winfo_rootx() + 12
        y = self.winfo_rooty() + self.winfo_height() + 6
        tip = tk.Toplevel(self)
        tip.wm_overrideredirect(True)
        tip.wm_geometry(f"+{x}+{y}")
        lbl = tk.Label(tip, text=self._tooltip_text,
                       background=ABMI_NAVY, foreground=ABMI_LIGHT,
                       borderwidth=1, relief="solid",
                       font=("Segoe UI", 9), padx=8, pady=4,
                       justify="left")
        lbl.pack()
        self._tip_window = tip

    def _hide_tip(self, _evt=None) -> None:
        if self._tip_window is not None:
            self._tip_window.destroy()
            self._tip_window = None

    # ---- click with visual feedback ----
    def _handle_click(self) -> None:
        self._hide_tip()
        if self._pending_reset is not None:
            try: self.after_cancel(self._pending_reset)
            except Exception: pass
            self._pending_reset = None

        self.configure(text=f"⟳  {self._base_text.split('  ', 1)[-1]}",
                       state=DISABLED)
        self.update_idletasks()

        ok = True; err_txt = ""
        try:
            self._action()
        except Exception as e:
            ok = False; err_txt = str(e)

        mark = "✓" if ok else "✗"
        self.configure(text=f"{mark}  {self._base_text.split('  ', 1)[-1]}")
        if self._on_status is not None:
            stamp = datetime.now().strftime("%H:%M:%S")
            if ok:
                self._on_status(f"✓  {self._base_text.strip()} · {stamp}", ok=True)
            else:
                self._on_status(f"✗  {self._base_text.strip()} failed: {err_txt}", ok=False)
        self._pending_reset = self.after(1400, self._reset_label)

    def _reset_label(self) -> None:
        self._pending_reset = None
        self.configure(text=self._base_text, state=NORMAL)


# ------------------------- KPI Card widget ----------------------------- #

class KpiCard(ttkb.Frame):
    """Visually prominent status card with a title, a big value, and a
    colour-coded left accent bar. Designed for home-screen 'at-a-glance'."""

    def __init__(self, parent, title: str, *, accent: str = ABMI_PINK,
                 width: int = 200):
        super().__init__(parent, padding=0, bootstyle=DARK)
        self.configure(width=width)
        self._accent = accent

        # Left accent bar
        bar = tk.Frame(self, bg=accent, width=4)
        bar.pack(side=LEFT, fill=Y)

        body = ttkb.Frame(self, padding=(12, 10))
        body.pack(side=LEFT, fill=BOTH, expand=True)

        self.title_var = tk.StringVar(value=title)
        self.value_var = tk.StringVar(value="—")
        self.unit_var = tk.StringVar(value="")
        self.hint_var = tk.StringVar(value="")

        ttkb.Label(body, textvariable=self.title_var,
                   font=("Segoe UI", 9), foreground=ABMI_MUTED
                   ).pack(anchor="w")
        value_row = ttkb.Frame(body)
        value_row.pack(anchor="w", pady=(4, 2))
        self.value_lbl = ttkb.Label(value_row, textvariable=self.value_var,
                                    font=("Segoe UI", 22, "bold"),
                                    foreground=ABMI_LIGHT)
        self.value_lbl.pack(side=LEFT)
        ttkb.Label(value_row, textvariable=self.unit_var,
                   font=("Segoe UI", 10), foreground=ABMI_MUTED
                   ).pack(side=LEFT, padx=(6, 0), pady=(8, 0))
        ttkb.Label(body, textvariable=self.hint_var,
                   font=("Segoe UI", 8), foreground=ABMI_MUTED
                   ).pack(anchor="w")

    def set(self, value: str, *, unit: str = "", hint: str = "",
            accent: str | None = None):
        self.value_var.set(value)
        self.unit_var.set(unit)
        self.hint_var.set(hint)
        if accent:
            self._accent = accent
            for child in self.winfo_children():
                if isinstance(child, tk.Frame):
                    child.configure(bg=accent)
                    break


# --------------------------- Dashboard UI ------------------------------ #

class Dashboard:
    def __init__(self, root, client: RosClient) -> None:
        self.root = root
        self.client = client
        self._logo_tk = None
        self._camera_tk = None

        root.title("ABMI — MyCobot Teleop Performance")
        root.geometry("1500x950")
        root.minsize(1200, 800)

        self._build_top_bar()
        self._build_tabs()

        self.root.after(REFRESH_MS, self._refresh_metrics)
        self.root.after(CAMERA_REFRESH_MS, self._refresh_camera)

    # ------------------------- top bar ------------------------- #
    def _build_top_bar(self) -> None:
        hdr = ttkb.Frame(self.root, padding=(20, 14, 20, 8))
        hdr.pack(side=TOP, fill=X)

        logo = self._load_logo(height=48)
        if logo is not None:
            self._logo_tk = logo
            ttkb.Label(hdr, image=logo).pack(side=LEFT, padx=(0, 18))
        else:
            ttkb.Label(hdr, text="ABMI", font=("Segoe UI", 22, "bold"),
                       foreground=ABMI_PINK).pack(side=LEFT, padx=(0, 18))

        title_block = ttkb.Frame(hdr)
        title_block.pack(side=LEFT)
        ttkb.Label(title_block, text="MyCobot 320 Pi — Teleop Performance",
                   font=("Segoe UI", 16, "bold"), foreground=ABMI_LIGHT
                   ).pack(anchor="w")
        ttkb.Label(title_block,
                   text="Live tuning · Sim ↔ Real comparison · Signal stability",
                   font=("Segoe UI", 9), foreground=ABMI_PINK
                   ).pack(anchor="w", pady=(2, 0))

        self.mode_var = tk.StringVar(value="⚪ OFFLINE")
        self.mode_label = ttkb.Label(hdr, textvariable=self.mode_var,
                                     font=("Segoe UI", 12, "bold"),
                                     padding=(18, 10), width=16, anchor=CENTER)
        self.mode_label.pack(side=RIGHT)

    def _load_logo(self, height: int):
        try:
            from PIL import Image, ImageTk
        except ImportError:
            return None
        for path in LOGO_CANDIDATES:
            if path.is_file():
                try:
                    img = Image.open(path).convert("RGBA")
                    w, h = img.size
                    ratio = height / h
                    img = img.resize((int(w * ratio), height), Image.LANCZOS)
                    return ImageTk.PhotoImage(img)
                except Exception:
                    return None
        return None

    # ---------------------------- tabs ------------------------- #
    def _build_tabs(self) -> None:
        self.nb = ttkb.Notebook(self.root, bootstyle=PRIMARY)
        self.nb.pack(side=TOP, fill=BOTH, expand=True, padx=16, pady=(4, 12))

        self.tab_home = ttkb.Frame(self.nb, padding=12)
        self.tab_metrics = ttkb.Frame(self.nb, padding=12)
        self.tab_tuning = ttkb.Frame(self.nb, padding=12)

        self.nb.add(self.tab_home,    text="🏠  Home")
        self.nb.add(self.tab_metrics, text="📊  Analytics")
        self.nb.add(self.tab_tuning,  text="🎛️  Tuning")

        self._build_home(self.tab_home)
        self._build_metrics(self.tab_metrics)
        self._build_tuning(self.tab_tuning)

    # ============================ HOME ======================== #
    def _build_home(self, parent):
        # KPI row
        kpi_row = ttkb.Frame(parent)
        kpi_row.pack(side=TOP, fill=X, pady=(0, 16))

        self.kpi_mode = KpiCard(kpi_row, "Execution mode", accent=ABMI_MUTED)
        self.kpi_rate = KpiCard(kpi_row, "Command rate", accent=ABMI_BLUE)
        self.kpi_track_sim = KpiCard(kpi_row, "SIM  tracking — avg RMS", accent=ABMI_BLUE)
        self.kpi_track_real = KpiCard(kpi_row, "REAL tracking — avg RMS", accent=ABMI_PINK)
        self.kpi_signal = KpiCard(kpi_row, "Signal health", accent=ABMI_GREEN)

        for card in (self.kpi_mode, self.kpi_rate, self.kpi_track_sim,
                     self.kpi_track_real, self.kpi_signal):
            card.pack(side=LEFT, fill=BOTH, expand=True, padx=6)

        # Main row: left = camera preview, right = comparison chart
        main = ttkb.Frame(parent)
        main.pack(side=TOP, fill=BOTH, expand=True)

        # Camera panel
        cam_frame = ttkb.Labelframe(main, text="  📹  Operator camera (via teleop)  ",
                                    padding=8, bootstyle=INFO, width=360)
        cam_frame.pack(side=LEFT, fill=Y, padx=(0, 12))
        cam_frame.pack_propagate(False)
        self.cam_label = ttkb.Label(cam_frame, text="waiting for /teleop/camera/image ...",
                                    foreground=ABMI_MUTED, anchor=CENTER)
        self.cam_label.pack(fill=BOTH, expand=True)
        self.cam_info_var = tk.StringVar(value="no frames received")
        ttkb.Label(cam_frame, textvariable=self.cam_info_var, font=("Segoe UI", 8),
                   foreground=ABMI_MUTED).pack(anchor="w", pady=(4, 0))

        # Comparison bar chart
        chart_frame = ttkb.Labelframe(main, text="  SIM ↔ REAL tracking error — per joint  ",
                                      padding=4, bootstyle=PRIMARY)
        chart_frame.pack(side=LEFT, fill=BOTH, expand=True)

        self.home_fig = plt.Figure(figsize=(6, 4.5), dpi=96, facecolor=BG,
                                   layout="constrained")
        self.ax_home_bars = self.home_fig.add_subplot(211)
        self.ax_home_xyz = self.home_fig.add_subplot(212)
        self.home_canvas = FigureCanvasTkAgg(self.home_fig, master=chart_frame)
        self.home_canvas.get_tk_widget().pack(fill=BOTH, expand=True)

        # Action buttons row (bottom of Home)
        actions = ttkb.Labelframe(parent, text="  Quick actions  ",
                                  padding=10, bootstyle=SECONDARY)
        actions.pack(side=TOP, fill=X, pady=(12, 0))

        btn_row = ttkb.Frame(actions)
        btn_row.pack(side=TOP, fill=X)

        ActionButton(btn_row, text="🏠  Send robot home", bootstyle=SUCCESS,
                     tooltip="Publishes 'home' on /to_robot → bridge_tour → Pi.\nRobot moves smoothly to [0,8,-127,40,0,0]°.",
                     action=lambda: self._send_robot_cmd("home"),
                     on_status=self._push_status
                     ).pack(side=LEFT, padx=4)
        ActionButton(btn_row, text="⊘  Stop (release servos)", bootstyle=DANGER,
                     tooltip="Publishes 'stop' → servos are released.\nHold the arm before clicking — it will go limp!",
                     action=lambda: self._send_robot_cmd("stop"),
                     on_status=self._push_status
                     ).pack(side=LEFT, padx=4)
        ActionButton(btn_row, text="⟲  Recalibrate hand origin",
                     bootstyle=(INFO, OUTLINE),
                     tooltip="Re-zeros the Wilor→robot mapping at current hand pose.\nCentre your palm in view first.",
                     action=self._on_recalibrate,
                     on_status=self._push_status
                     ).pack(side=LEFT, padx=4)
        ActionButton(btn_row, text="📊  Run performance analyzer",
                     bootstyle=(WARNING, OUTLINE),
                     tooltip="Launches the guided bag-based analyzer script.\nGenerates an Excel report with RMS/jitter/latency.",
                     action=self._run_analyzer_guided,
                     on_status=self._push_status
                     ).pack(side=LEFT, padx=4)
        ActionButton(btn_row, text="💾  Export CSV snapshot",
                     bootstyle=(SECONDARY, OUTLINE),
                     tooltip="Dumps the current cmd/sim/real buffers\nto a timestamped CSV next to this script.",
                     action=self._export_snapshot,
                     on_status=self._push_status
                     ).pack(side=LEFT, padx=4)

        # Status toast (updates after each action)
        self.status_var = tk.StringVar(value="Ready — hover an action for details.")
        self.status_label = ttkb.Label(actions, textvariable=self.status_var,
                                       font=("JetBrains Mono", 9),
                                       foreground=ABMI_MUTED)
        self.status_label.pack(side=TOP, anchor="w", pady=(8, 0))

    # ======================== METRICS (Analytics) ================ #
    def _build_metrics(self, parent):
        self.fig = plt.Figure(figsize=(12, 7), dpi=96, facecolor=BG,
                              layout="constrained")
        gs = self.fig.add_gridspec(3, 2, height_ratios=[1, 1.3, 1],
                                   hspace=0.3, wspace=0.14)
        self.ax_xyz = self.fig.add_subplot(gs[0, :])
        self.ax_joints_sim = self.fig.add_subplot(gs[1, 0])
        self.ax_joints_real = self.fig.add_subplot(gs[1, 1])
        self.ax_err_sim = self.fig.add_subplot(gs[2, 0])
        self.ax_err_real = self.fig.add_subplot(gs[2, 1])

        self.canvas = FigureCanvasTkAgg(self.fig, master=parent)
        self.canvas.get_tk_widget().pack(fill=BOTH, expand=True)

        # Compact stats strip below the plots
        strip = ttkb.Frame(parent, padding=(0, 6))
        strip.pack(side=TOP, fill=X)
        self.strip_var = tk.StringVar(value="stats will appear here")
        ttkb.Label(strip, textvariable=self.strip_var,
                   font=("JetBrains Mono", 10), foreground=ABMI_LIGHT
                   ).pack(anchor="w")

    # ============================ TUNING ======================== #
    def _build_tuning(self, parent):
        frame = ttkb.Labelframe(parent, text="  Live gain tuning  ",
                                padding=(18, 14), bootstyle=INFO)
        frame.pack(side=TOP, fill=X, pady=(8, 0))

        self.gain_vars = {
            "x":   tk.DoubleVar(value=1.2),
            "y":   tk.DoubleVar(value=1.2),
            "z":   tk.DoubleVar(value=1.6),
            "tfs": tk.DoubleVar(value=0.25),
        }
        rows = [
            ("X gain — hand forward → J3 elbow",       "x",   0.3, 3.0, INFO,
             "More = smaller forward hand move produces bigger elbow motion"),
            ("Y gain — hand lateral → J1 base yaw",    "y",   0.3, 3.0, INFO,
             "More = smaller lateral move produces bigger base rotation"),
            ("Z gain — hand vertical → J2 + J5",       "z",   0.3, 3.0, SUCCESS,
             "More = smaller vertical move produces bigger shoulder+wrist tilt"),
            ("time_from_start (s) — trajectory duration", "tfs", 0.1, 2.5, WARNING,
             "Lower = more reactive (riskier) · Higher = smoother (laggier)"),
        ]
        for i, row in enumerate(rows):
            self._make_tuning_slider(frame, i, *row)
        for col, w in ((0, 0), (1, 1), (2, 0)):
            frame.columnconfigure(col, weight=w)

        ttkb.Separator(frame, bootstyle=SECONDARY).grid(
            row=len(rows), column=0, columnspan=3, sticky="ew", pady=(10, 6)
        )
        bottom = ttkb.Frame(frame)
        bottom.grid(row=len(rows) + 1, column=0, columnspan=3, sticky="ew")
        ActionButton(bottom, text="⟲  Recalibrate hand origin",
                     bootstyle=(SUCCESS, OUTLINE),
                     tooltip="Re-zeros the Wilor→robot mapping at current hand pose.\nCentre your palm in view first.",
                     action=self._on_recalibrate,
                     on_status=self._push_status
                     ).pack(side=LEFT)
        ttkb.Label(bottom,
                   text="  Centre your palm in view first, then click to re-zero the mapping.",
                   font=("Segoe UI", 9), foreground=ABMI_MUTED
                   ).pack(side=LEFT)

        # Preset row — active one is highlighted in pink outline
        preset_frame = ttkb.Labelframe(parent, text="  Gain presets  ",
                                       padding=10, bootstyle=SECONDARY)
        preset_frame.pack(side=TOP, fill=X, pady=(14, 0))
        self._preset_buttons: list[tuple[ttkb.Button, tuple]] = []
        presets = [
            ("🐢 Safe start", (0.6, 0.6, 0.6, 0.3),
             "Conservative gains for first-time real-robot testing."),
            ("⚙️ Nominal",    (1.2, 1.2, 1.6, 0.25),
             "Validated on real MyCobot 320 Pi (22/04/2026)."),
            ("⚡ Reactive",    (1.6, 1.6, 2.0, 0.15),
             "Snappy response — use only after the operator is tuned in."),
        ]
        for label, values, tip in presets:
            btn = ActionButton(preset_frame, text=label,
                               bootstyle=(SECONDARY, OUTLINE),
                               tooltip=f"{tip}\nx={values[0]}  y={values[1]}  z={values[2]}  tfs={values[3]}s",
                               action=lambda v=values: self._apply_preset(*v),
                               on_status=self._push_status)
            btn.pack(side=LEFT, padx=6)
            self._preset_buttons.append((btn, values))
        ttkb.Label(preset_frame,
                   text="  Presets are a starting point — always tune with the dashboard before going real.",
                   font=("Segoe UI", 9), foreground=ABMI_MUTED
                   ).pack(side=LEFT, padx=(16, 0))

    def _make_tuning_slider(self, parent, row, label, key, lo, hi, style, hint):
        ttkb.Label(parent, text=label, font=("Segoe UI", 10, "bold"),
                   foreground=ABMI_LIGHT
                   ).grid(row=row * 2, column=0, sticky="w",
                          padx=(4, 12), pady=(8, 0))
        var = self.gain_vars[key]
        scale = ttkb.Scale(parent, from_=lo, to=hi, orient=HORIZONTAL,
                           variable=var, bootstyle=style,
                           command=lambda _v: self._on_slider_change())
        scale.grid(row=row * 2, column=1, sticky="ew", padx=8, pady=(8, 0))
        value_lbl = ttkb.Label(parent, text=f"{var.get():.2f}",
                               font=("JetBrains Mono", 13, "bold"),
                               bootstyle=style, width=6, anchor=E)
        value_lbl.grid(row=row * 2, column=2, padx=(8, 4), pady=(8, 0))
        ttkb.Label(parent, text=hint, font=("Segoe UI", 9),
                   foreground=ABMI_MUTED
                   ).grid(row=row * 2 + 1, column=0, columnspan=3, sticky="w",
                          padx=4, pady=(0, 6))
        def _sync(*_a):
            value_lbl.configure(text=f"{var.get():.2f}")
        var.trace_add("write", _sync)

    def _apply_preset(self, x, y, z, tfs):
        self.gain_vars["x"].set(x)
        self.gain_vars["y"].set(y)
        self.gain_vars["z"].set(z)
        self.gain_vars["tfs"].set(tfs)
        self._on_slider_change()
        # Highlight the active preset button
        active = (x, y, z, tfs)
        for btn, values in getattr(self, "_preset_buttons", []):
            if values == active:
                btn.configure(bootstyle=SUCCESS)  # filled pink-ish accent
            else:
                btn.configure(bootstyle=(SECONDARY, OUTLINE))

    # ----------------------- callbacks / actions ----------------- #
    def _on_slider_change(self) -> None:
        self.client.publish_gains(
            self.gain_vars["x"].get(), self.gain_vars["y"].get(),
            self.gain_vars["z"].get(), self.gain_vars["tfs"].get(),
        )

    def _on_recalibrate(self) -> None:
        self.client.request_recalibrate()

    def _send_robot_cmd(self, text: str) -> None:
        self.client.send_raw(text)

    def _run_analyzer_guided(self) -> None:
        try:
            script = Path(__file__).parent / "performance_analyzer.py"
            subprocess.Popen(
                ["/home/genji/miniconda/envs/hand-teleop/bin/python",
                 str(script), "--guided"],
                cwd=str(Path(__file__).parent),
            )
        except Exception as e:
            print(f"[dashboard] failed to start analyzer: {e}")

    def _push_status(self, message: str, *, ok: bool = True) -> None:
        """Update the Home tab status toast after a dynamic action fires."""
        self.status_var.set(message)
        self.status_label.configure(foreground=ABMI_GREEN if ok else ABMI_PINK)
        # fade back to muted so the message feels ephemeral
        self.root.after(4500, lambda: self.status_label.configure(foreground=ABMI_MUTED))

    def _export_snapshot(self) -> None:
        hand, cmd, actual_sim, actual_real = self.client.snapshot()
        stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        out = Path(__file__).parent / f"snapshot_{stamp}.csv"
        try:
            with open(out, "w") as f:
                f.write("t,source,j1,j2,j3,j4,j5,j6\n")
                for t, q in cmd:
                    f.write(f"{t:.3f},cmd," + ",".join(f"{x:.4f}" for x in q) + "\n")
                for t, q in actual_sim:
                    f.write(f"{t:.3f},sim," + ",".join(f"{x:.4f}" for x in q) + "\n")
                for t, q in actual_real:
                    f.write(f"{t:.3f},real," + ",".join(f"{x:.4f}" for x in q) + "\n")
            print(f"[dashboard] snapshot written to {out}")
        except Exception as e:
            print(f"[dashboard] export failed: {e}")

    # ---------------------- refresh loops ---------------------- #
    _MODE_STYLES = {
        "SIM":     {"fg": "#ffffff", "bg": ABMI_BLUE,   "text": "🟦  SIM (Gazebo)"},
        "REAL":    {"fg": "#ffffff", "bg": ABMI_PINK,   "text": "🟥  REAL (MyCobot)"},
        "BOTH":    {"fg": ABMI_NAVY, "bg": ABMI_YELLOW, "text": "⚡  SIM + REAL"},
        "OFFLINE": {"fg": ABMI_LIGHT, "bg": ABMI_MUTED, "text": "⚪  OFFLINE"},
    }

    def _refresh_metrics(self) -> None:
        hand, cmd, actual_sim, actual_real = self.client.snapshot()
        mode = self.client.detect_mode()
        self._apply_mode_badge(mode)
        self._update_home_kpis(hand, cmd, actual_sim, actual_real, mode)
        self._redraw_home_plots(hand, cmd, actual_sim, actual_real, mode)
        self._redraw_metrics(hand, cmd, actual_sim, actual_real, mode)
        self.root.after(REFRESH_MS, self._refresh_metrics)

    def _refresh_camera(self) -> None:
        img = self.client.take_camera_frame()
        if img is not None:
            self._camera_tk = img
            self.cam_label.configure(image=img, text="")
            self.cam_info_var.set(
                f"live feed · {img.width()}×{img.height()} · updated "
                f"{datetime.now().strftime('%H:%M:%S')}"
            )
        self.root.after(CAMERA_REFRESH_MS, self._refresh_camera)

    def _apply_mode_badge(self, mode: str) -> None:
        style = self._MODE_STYLES.get(mode, self._MODE_STYLES["OFFLINE"])
        self.mode_var.set(style["text"])
        self.mode_label.configure(background=style["bg"], foreground=style["fg"])

    # ------------------------- KPIs on Home ------------------------- #
    def _update_home_kpis(self, hand, cmd, actual_sim, actual_real, mode):
        # Mode card
        mode_style = self._MODE_STYLES.get(mode, self._MODE_STYLES["OFFLINE"])
        self.kpi_mode.set(mode, hint=mode_style["text"].split("  ", 1)[-1],
                          accent=mode_style["bg"])

        # Cmd rate
        if len(cmd) >= 2:
            span = cmd[-1][0] - cmd[0][0]
            rate = (len(cmd) - 1) / span if span > 0 else 0.0
        else:
            rate = 0.0
        self.kpi_rate.set(f"{rate:.1f}", unit="Hz",
                          hint=f"{len(cmd)} samples in window",
                          accent=ABMI_GREEN if rate >= 20 else ABMI_YELLOW if rate > 0 else ABMI_MUTED)

        # Per-source avg RMS
        def _avg_rms(actual):
            if not cmd or not actual or len(actual) < 2: return None
            errs = self._tracking_errors(cmd, actual)
            non_empty = [ee for (_, ee) in errs if ee.size]
            if not non_empty: return None
            return float(np.mean([np.sqrt((e ** 2).mean()) for e in non_empty]))

        sim_rms = _avg_rms(actual_sim)
        real_rms = _avg_rms(actual_real)
        def _fmt(val):
            return (f"{val:.1f}", "°",
                    ABMI_GREEN if val < 5 else ABMI_YELLOW if val < 15 else ABMI_PINK) \
                   if val is not None else ("—", "", ABMI_MUTED)

        v, u, c = _fmt(sim_rms)
        self.kpi_track_sim.set(v, unit=u,
                               hint="Gazebo" if sim_rms is not None else "no sim data",
                               accent=c)
        v, u, c = _fmt(real_rms)
        self.kpi_track_real.set(v, unit=u,
                                hint="MyCobot 320 Pi" if real_rms is not None else "no real data",
                                accent=c)

        # Signal health = OK if no UNSTABLE anywhere
        worst = "OK"
        for act in (actual_sim, actual_real):
            stats = self._row_stats(cmd, act) if (cmd and act) else []
            for row in stats:
                if row["flag"] == "UNSTABLE":
                    worst = "UNSTABLE"
                elif row["flag"] == "JITTERY" and worst != "UNSTABLE":
                    worst = "JITTERY"
        sh_color = {"OK": ABMI_GREEN, "JITTERY": ABMI_YELLOW,
                    "UNSTABLE": ABMI_PINK}.get(worst, ABMI_MUTED)
        self.kpi_signal.set(worst,
                            hint="across all driven joints" if (cmd and (actual_sim or actual_real))
                                 else "no data yet",
                            accent=sh_color)

    # ---------------------- Home plots ---------------------- #
    def _redraw_home_plots(self, hand, cmd, actual_sim, actual_real, mode):
        # Bar chart: SIM vs REAL RMS per joint
        ax = self.ax_home_bars
        ax.cla()
        _style_axis(ax, "Per-joint RMS tracking error",
                    "joint", "RMS error (°)")
        sim_stats = self._row_stats(cmd, actual_sim) if (cmd and actual_sim) else [None]*6
        real_stats = self._row_stats(cmd, actual_real) if (cmd and actual_real) else [None]*6
        x = np.arange(6)
        w = 0.38
        sim_vals  = [s["rms"] if s else 0 for s in sim_stats]
        real_vals = [r["rms"] if r else 0 for r in real_stats]
        ax.bar(x - w/2, sim_vals, w, color=ABMI_BLUE, label="SIM",
               edgecolor=ABMI_NAVY, linewidth=0.5)
        ax.bar(x + w/2, real_vals, w, color=ABMI_PINK, label="REAL",
               edgecolor=ABMI_NAVY, linewidth=0.5)
        ax.set_xticks(x); ax.set_xticklabels(SHORT_NAMES, color=FG)
        ax.axhline(5.0, color=ABMI_LIGHT, alpha=0.4, linestyle=":", lw=1.0)
        ax.axhline(15.0, color=ABMI_PINK, alpha=0.4, linestyle=":", lw=1.0)
        if cmd and (actual_sim or actual_real):
            ax.legend(loc="upper right", fontsize=9, frameon=True,
                      facecolor=BG, edgecolor=GRID, labelcolor=FG)

        # Hand XYZ small plot (cm)
        ax2 = self.ax_home_xyz
        ax2.cla()
        _style_axis(ax2, "Hand position (relative, cm)", "time (s)", "position (cm)")
        if hand:
            arr = np.array(hand)
            t = arr[:, 0]
            now = time.perf_counter() - self.client._t0
            ax2.set_xlim(max(0.0, now - PLOT_WINDOW_S), now + 0.1)
            ax2.plot(t, arr[:, 1] * 100, color="#ff6b6b", lw=1.5, label="x (forward)")
            ax2.plot(t, arr[:, 2] * 100, color=ABMI_GREEN, lw=1.5, label="y (lateral)")
            ax2.plot(t, arr[:, 3] * 100, color=ABMI_BLUE, lw=1.5, label="z (vertical)")
            ax2.legend(loc="upper right", fontsize=8, ncol=3, frameon=True,
                       facecolor=BG, edgecolor=GRID, labelcolor=FG)
        self.home_canvas.draw_idle()

    # ---------------------- Analytics plots ---------------------- #
    def _redraw_metrics(self, hand, cmd, actual_sim, actual_real, mode):
        now = time.perf_counter() - self.client._t0
        t_min = max(0.0, now - PLOT_WINDOW_S)
        for ax in (self.ax_xyz, self.ax_joints_sim, self.ax_joints_real,
                   self.ax_err_sim, self.ax_err_real):
            ax.cla(); ax.set_xlim(t_min, now + 0.1)

        _style_axis(self.ax_xyz, "Wilor hand position", "time (s)", "position (cm)")
        if hand:
            arr = np.array(hand); t = arr[:, 0]
            self.ax_xyz.plot(t, arr[:, 1] * 100, color="#ff6b6b", lw=1.5, label="x (forward)")
            self.ax_xyz.plot(t, arr[:, 2] * 100, color=ABMI_GREEN, lw=1.5, label="y (lateral)")
            self.ax_xyz.plot(t, arr[:, 3] * 100, color=ABMI_BLUE, lw=1.5, label="z (vertical)")
            self.ax_xyz.legend(loc="upper right", fontsize=8, ncol=3, frameon=True,
                              facecolor=BG, edgecolor=GRID, labelcolor=FG)

        self._draw_joints(self.ax_joints_sim, cmd, actual_sim,
                          "Joint angles — SIM", active=(mode in ("SIM", "BOTH")))
        self._draw_joints(self.ax_joints_real, cmd, actual_real,
                          "Joint angles — REAL", active=(mode in ("REAL", "BOTH")))
        self._draw_error(self.ax_err_sim, cmd, actual_sim,
                         "Tracking error — SIM", active=(mode in ("SIM", "BOTH")))
        self._draw_error(self.ax_err_real, cmd, actual_real,
                         "Tracking error — REAL", active=(mode in ("REAL", "BOTH")))
        self.canvas.draw_idle()
        self._update_strip(cmd, actual_sim, actual_real, mode)

    def _draw_joints(self, ax, cmd, actual, title, *, active=True):
        _style_axis(ax, title, "time (s)", "angle (°)")
        if not active:
            ax.text(0.5, 0.5, "not active in current mode",
                    transform=ax.transAxes, ha="center", va="center",
                    fontsize=10, color=ABMI_MUTED); return
        if cmd:
            tc = np.array([c[0] for c in cmd]); qc = np.degrees(np.array([c[1] for c in cmd]))
            for i in range(6):
                ax.plot(tc, qc[:, i], color=JOINT_COLORS[i], lw=1.4, label=LONG_NAMES[i])
        if actual:
            ta = np.array([a[0] for a in actual]); qa = np.degrees(np.array([a[1] for a in actual]))
            for i in range(6):
                ax.plot(ta, qa[:, i], color=JOINT_COLORS[i], lw=1.0, linestyle="--", alpha=0.75)
        if cmd or actual:
            ax.legend(loc="upper right", fontsize=7, ncol=3, frameon=True,
                      facecolor=BG, edgecolor=GRID, labelcolor=FG)

    def _draw_error(self, ax, cmd, actual, title, *, active=True):
        _style_axis(ax, title, "time (s)", "error (°)")
        if not active:
            ax.text(0.5, 0.5, "not active in current mode",
                    transform=ax.transAxes, ha="center", va="center",
                    fontsize=10, color=ABMI_MUTED); return
        any_err = False
        for i, (te, ee) in enumerate(self._tracking_errors(cmd, actual)):
            if te.size == 0: continue
            any_err = True
            ax.plot(te, ee, color=JOINT_COLORS[i], lw=1.2, label=LONG_NAMES[i])
        if any_err:
            ax.legend(loc="upper right", fontsize=7, ncol=3, frameon=True,
                      facecolor=BG, edgecolor=GRID, labelcolor=FG)
            ax.axhline(5.0, color=FG, alpha=0.3, linestyle=":", lw=1.0)

    # ---------------------- Stats helpers ------------------------ #
    def _tracking_errors(self, cmd, actual):
        if not cmd or not actual or len(actual) < 2:
            return [(np.array([]), np.array([]))] * 6
        tc = np.array([c[0] for c in cmd])
        qc = np.degrees(np.array([c[1] for c in cmd]))
        ta = np.array([a[0] for a in actual])
        qa = np.degrees(np.array([a[1] for a in actual]))
        return [(tc, np.abs(qc[:, i] - np.interp(tc, ta, qa[:, i]))) for i in range(6)]

    def _row_stats(self, cmd, actual):
        errs = self._tracking_errors(cmd, actual)
        qc = np.degrees(np.array([c[1] for c in cmd])) if cmd else np.zeros((0, 6))
        jitter = (np.std(np.diff(qc, axis=0), axis=0)
                  if qc.shape[0] > 2 else np.zeros(6))
        rows = []
        for i, (_, ee) in enumerate(errs):
            rms = float(np.sqrt((ee ** 2).mean())) if ee.size else 0.0
            mx = float(ee.max()) if ee.size else 0.0
            jit = float(jitter[i])
            if jit > 10.0 or mx > 15.0: flag = "UNSTABLE"
            elif jit > 3.0 or mx > 8.0: flag = "JITTERY"
            else: flag = "OK"
            rows.append({"rms": rms, "max": mx, "jit": jit, "flag": flag})
        return rows

    def _update_strip(self, cmd, actual_sim, actual_real, mode):
        parts = []
        if mode in ("SIM", "BOTH") and cmd and actual_sim:
            s = self._row_stats(cmd, actual_sim)
            rms = np.mean([r["rms"] for r in s])
            parts.append(f"SIM avg RMS {rms:.2f}°  · flags: " +
                         "/".join(r["flag"][0] for r in s))
        if mode in ("REAL", "BOTH") and cmd and actual_real:
            r = self._row_stats(cmd, actual_real)
            rms = np.mean([x["rms"] for x in r])
            parts.append(f"REAL avg RMS {rms:.2f}°  · flags: " +
                         "/".join(x["flag"][0] for x in r))
        self.strip_var.set("    ".join(parts) if parts else "waiting for cmd + actual streams...")


# ------------------------------- main ---------------------------------- #

def main() -> None:
    client = RosClient()
    root = ttkb.Window(themename="darkly")
    style = ttkb.Style()
    style.configure("TFrame", background=BG)
    style.configure("TLabel", background=BG, foreground=ABMI_LIGHT)
    style.configure("TLabelframe", background=BG, foreground=ABMI_LIGHT,
                    bordercolor=ABMI_PINK)
    style.configure("TLabelframe.Label", background=BG, foreground=ABMI_PINK)
    style.configure("TNotebook", background=BG, borderwidth=0)
    style.configure("TNotebook.Tab", padding=(18, 8), font=("Segoe UI", 10, "bold"))
    root.configure(background=BG)

    app = Dashboard(root, client)

    def _on_close():
        client.terminate()
        root.destroy()
    root.protocol("WM_DELETE_WINDOW", _on_close)

    # Push initial gains so the teleop is aligned with the sliders
    app._on_slider_change()
    root.mainloop()


if __name__ == "__main__":
    main()
