#!/usr/bin/env python3
"""Live tuning and monitoring dashboard for MyCobot teleoperation.

Connects to rosbridge (ws://localhost:9090) and:
  - Subscribes to /teleop/hand_xyz          (hand position from Wilor)
  - Subscribes to /mycobot_controller/joint_trajectory  (commanded angles)
  - Subscribes to /joint_states             (actual robot angles)
  - Publishes to   /teleop/gains            (live x/y/z gains + time-from-start)

Modern UI (ttkbootstrap dark theme) with labeled axes, fully-annotated plots
and at-a-glance stability indicators.

Run:
    conda activate hand-teleop
    python3 teleop/teleop_dashboard.py
"""
from __future__ import annotations

import threading
import time
import tkinter as tk
from collections import deque

import numpy as np
import roslibpy

import ttkbootstrap as ttkb
from ttkbootstrap.constants import *

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg


MYCOBOT_JOINTS = [
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
]
SHORT_NAMES = ["J1 yaw", "J2 shoulder", "J3 elbow", "J4 wrist1", "J5 yaw", "J6 roll"]
PLOT_WINDOW_S = 10.0
MAX_SAMPLES = 800
REFRESH_MS = 250

# Matplotlib dark styling to match ttkbootstrap darkly
BG = "#222222"
FG = "#ebebeb"
GRID = "#444444"
ACCENT = "#4dabf7"
JOINT_COLORS = ["#e03131", "#f59f00", "#2f9e44", "#1971c2", "#9c36b5", "#e8590c"]


def _style_axis(ax, title: str, xlabel: str, ylabel: str) -> None:
    ax.set_facecolor(BG)
    ax.set_title(title, color=FG, fontsize=10, pad=6)
    ax.set_xlabel(xlabel, color=FG, fontsize=9)
    ax.set_ylabel(ylabel, color=FG, fontsize=9)
    ax.tick_params(colors=FG, labelsize=8)
    for s in ax.spines.values():
        s.set_color(GRID)
    ax.grid(True, alpha=0.25, color=GRID, linestyle="-", linewidth=0.5)


class RosClient:
    """Thread-safe buffered rosbridge client."""

    def __init__(self, host: str = "localhost", port: int = 9090) -> None:
        self.ros = roslibpy.Ros(host=host, port=port)
        self.ros.run()
        if not self.ros.is_connected:
            raise SystemExit(f"Cannot reach rosbridge at ws://{host}:{port}")

        self._lock = threading.Lock()
        self.hand_xyz = deque(maxlen=MAX_SAMPLES)
        self.cmd_joints = deque(maxlen=MAX_SAMPLES)
        self.actual_joints = deque(maxlen=MAX_SAMPLES)
        self._t0 = time.perf_counter()

        self._t_hand = roslibpy.Topic(self.ros, "/teleop/hand_xyz",
                                      "geometry_msgs/Vector3Stamped")
        self._t_hand.subscribe(self._on_hand)
        self._t_cmd = roslibpy.Topic(self.ros, "/mycobot_controller/joint_trajectory",
                                     "trajectory_msgs/JointTrajectory")
        self._t_cmd.subscribe(self._on_cmd)
        self._t_actual = roslibpy.Topic(self.ros, "/joint_states",
                                        "sensor_msgs/JointState")
        self._t_actual.subscribe(self._on_actual)
        self._p_gains = roslibpy.Topic(self.ros, "/teleop/gains",
                                       "std_msgs/Float64MultiArray")
        self._p_gains.advertise()

    def _now(self) -> float:
        return time.perf_counter() - self._t0

    def _on_hand(self, msg: dict) -> None:
        v = msg["vector"]
        with self._lock:
            self.hand_xyz.append((self._now(), v["x"], v["y"], v["z"]))

    def _on_cmd(self, msg: dict) -> None:
        points = msg.get("points") or []
        names = msg.get("joint_names") or []
        if not points or not names:
            return
        positions = points[0].get("positions") or []
        try:
            reordered = [positions[names.index(j)] for j in MYCOBOT_JOINTS]
        except ValueError:
            return
        with self._lock:
            self.cmd_joints.append((self._now(), reordered))

    def _on_actual(self, msg: dict) -> None:
        names = msg.get("name") or []
        positions = msg.get("position") or []
        try:
            reordered = [positions[names.index(j)] for j in MYCOBOT_JOINTS]
        except ValueError:
            return
        with self._lock:
            self.actual_joints.append((self._now(), reordered))

    def publish_gains(self, gx: float, gy: float, gz: float, tfs: float) -> None:
        self._p_gains.publish(roslibpy.Message({"data": [gx, gy, gz, tfs]}))

    def snapshot(self):
        with self._lock:
            return (list(self.hand_xyz), list(self.cmd_joints), list(self.actual_joints))

    def terminate(self) -> None:
        for t in (self._t_hand, self._t_cmd, self._t_actual):
            try:
                t.unsubscribe()
            except Exception:
                pass
        try:
            self._p_gains.unadvertise()
        except Exception:
            pass
        self.ros.terminate()


class Dashboard:
    def __init__(self, root, client: RosClient) -> None:
        self.root = root
        self.client = client

        root.title("MyCobot Teleop — Live Tuning & Monitoring")
        root.geometry("1400x900")
        root.minsize(1100, 750)

        self._build_header()
        self._build_gains_panel()
        self._build_connection_panel()
        self._build_stats_panel()
        self._build_plots_panel()

        self.root.after(REFRESH_MS, self._refresh)

    # ---------------------------- UI sections ---------------------------- #

    def _build_header(self) -> None:
        hdr = ttkb.Frame(self.root, padding=(16, 12, 16, 4))
        hdr.pack(side=TOP, fill=X)

        title = ttkb.Label(
            hdr, text="🦾 MyCobot 320 Pi — Teleop Dashboard",
            font=("Segoe UI", 16, "bold"), bootstyle=LIGHT,
        )
        title.pack(side=LEFT)

        subtitle = ttkb.Label(
            hdr,
            text="  Live tuning · Hand → Joint tracking · Signal stability",
            font=("Segoe UI", 10),
            bootstyle=SECONDARY,
        )
        subtitle.pack(side=LEFT, padx=(8, 0))

    def _build_gains_panel(self) -> None:
        frame = ttkb.Labelframe(self.root, text="  Live gain tuning  ",
                                padding=(12, 8), bootstyle=INFO)
        frame.pack(side=TOP, fill=X, padx=16, pady=(4, 8))

        self.gain_vars = {
            "x": tk.DoubleVar(value=1.2),
            "y": tk.DoubleVar(value=1.2),
            "z": tk.DoubleVar(value=1.6),
            "tfs": tk.DoubleVar(value=0.8),
        }
        rows = [
            ("X gain — hand forward → J2 shoulder", "x", 0.3, 3.0, INFO),
            ("Y gain — hand lateral → J1 yaw",      "y", 0.3, 3.0, INFO),
            ("Z gain — hand vertical → J3 + J5",    "z", 0.3, 3.0, SUCCESS),
            ("time_from_start — trajectory duration (s)", "tfs", 0.1, 2.5, WARNING),
        ]
        for i, (label, key, lo, hi, style) in enumerate(rows):
            self._make_slider(frame, i, label, key, lo, hi, style)

        for col, w in ((0, 0), (1, 1), (2, 0)):
            frame.columnconfigure(col, weight=w)

        ttkb.Separator(frame, bootstyle=SECONDARY).grid(
            row=len(rows), column=0, columnspan=3, sticky="ew", pady=(6, 2)
        )
        hint = ttkb.Label(
            frame,
            text="Changes apply live via rosbridge. "
                 "Higher gain = more joint motion for less hand motion.",
            font=("Segoe UI", 9), bootstyle=SECONDARY,
        )
        hint.grid(row=len(rows) + 1, column=0, columnspan=3, sticky="w", pady=(2, 0))

    def _make_slider(self, parent, row, label, key, lo, hi, style) -> None:
        ttkb.Label(parent, text=label, font=("Segoe UI", 10)).grid(
            row=row, column=0, sticky="w", padx=(4, 12), pady=4
        )
        var = self.gain_vars[key]
        scale = ttkb.Scale(
            parent, from_=lo, to=hi, orient=HORIZONTAL, variable=var,
            bootstyle=style,
            command=lambda _v: self._on_slider_change(),
        )
        scale.grid(row=row, column=1, sticky="ew", padx=8, pady=4)

        value_lbl = ttkb.Label(parent, text=f"{var.get():.2f}",
                               font=("JetBrains Mono", 11, "bold"),
                               bootstyle=style, width=6, anchor=E)
        value_lbl.grid(row=row, column=2, padx=(8, 4), pady=4)

        def _sync(*_a):
            value_lbl.configure(text=f"{var.get():.2f}")
        var.trace_add("write", _sync)

    def _build_connection_panel(self) -> None:
        frame = ttkb.Frame(self.root)
        frame.pack(side=TOP, fill=X, padx=16, pady=(0, 4))

        self.conn_var = tk.StringVar(value="● rosbridge connected")
        lbl = ttkb.Label(frame, textvariable=self.conn_var,
                        font=("Segoe UI", 9, "bold"), bootstyle=SUCCESS)
        lbl.pack(side=LEFT)

        self.topic_stats_var = tk.StringVar(value="waiting for first messages…")
        ttkb.Label(frame, textvariable=self.topic_stats_var,
                  font=("Segoe UI", 9), bootstyle=SECONDARY
                  ).pack(side=LEFT, padx=(18, 0))

    def _build_stats_panel(self) -> None:
        frame = ttkb.Labelframe(self.root,
                                text="  Tracking stability (last 10s)  ",
                                padding=(0, 8), bootstyle=SECONDARY)
        frame.pack(side=TOP, fill=X, padx=16, pady=4)

        headers = ["Joint", "RMS err", "Max err", "Cmd jitter σΔ", "Signal"]
        widths = [16, 12, 12, 18, 12]
        for col, (h, w) in enumerate(zip(headers, widths)):
            ttkb.Label(frame, text=h, width=w,
                      font=("Segoe UI", 9, "bold"),
                      bootstyle=INFO, anchor=W
                      ).grid(row=0, column=col, sticky="w", padx=6, pady=(4, 2))

        self.stat_labels = []
        for r, name in enumerate(SHORT_NAMES, start=1):
            cells = []
            for c, w in enumerate(widths):
                var = tk.StringVar(value=name if c == 0 else "—")
                lbl = ttkb.Label(frame, textvariable=var, width=w,
                                font=("JetBrains Mono", 10), anchor=W)
                lbl.grid(row=r, column=c, sticky="w", padx=6, pady=1)
                cells.append((var, lbl))
            self.stat_labels.append(cells)

    def _build_plots_panel(self) -> None:
        frame = ttkb.Labelframe(self.root, text="  Live signals  ",
                                padding=(4, 4), bootstyle=PRIMARY)
        frame.pack(side=TOP, fill=BOTH, expand=True, padx=16, pady=(4, 12))

        self.fig = plt.Figure(figsize=(12, 6.5), dpi=96, facecolor=BG, layout="constrained")
        gs = self.fig.add_gridspec(3, 1, height_ratios=[1, 1.4, 1])
        self.ax_xyz = self.fig.add_subplot(gs[0])
        self.ax_joints = self.fig.add_subplot(gs[1])
        self.ax_err = self.fig.add_subplot(gs[2])

        _style_axis(self.ax_xyz,    "Wilor hand position",
                    "time (s)", "position (m)")
        _style_axis(self.ax_joints, "Joint angles — solid = commanded, dashed = actual",
                    "time (s)", "angle (°)")
        _style_axis(self.ax_err,    "Tracking error per joint — |commanded − actual|",
                    "time (s)", "error (°)")

        self.canvas = FigureCanvasTkAgg(self.fig, master=frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=BOTH, expand=True)

    # ---------------------------- handlers ------------------------------- #

    def _on_slider_change(self) -> None:
        self.client.publish_gains(
            self.gain_vars["x"].get(),
            self.gain_vars["y"].get(),
            self.gain_vars["z"].get(),
            self.gain_vars["tfs"].get(),
        )

    def _refresh(self) -> None:
        hand, cmd, actual = self.client.snapshot()
        self._update_topic_stats(hand, cmd, actual)
        self._redraw(hand, cmd, actual)
        self._update_stats(cmd, actual)
        self.root.after(REFRESH_MS, self._refresh)

    def _update_topic_stats(self, hand, cmd, actual) -> None:
        parts = []
        parts.append(f"hand_xyz: {len(hand):>4d}")
        parts.append(f"commanded: {len(cmd):>4d}")
        parts.append(f"joint_states: {len(actual):>4d}")
        self.topic_stats_var.set("   ".join(parts))

    # ------------------------------ plots -------------------------------- #

    def _redraw(self, hand, cmd, actual) -> None:
        now = time.perf_counter() - self.client._t0
        t_min = max(0.0, now - PLOT_WINDOW_S)
        for ax in (self.ax_xyz, self.ax_joints, self.ax_err):
            ax.cla()
        _style_axis(self.ax_xyz,    "Wilor hand position",
                    "time (s)", "position (m)")
        _style_axis(self.ax_joints, "Joint angles — solid = commanded, dashed = actual",
                    "time (s)", "angle (°)")
        _style_axis(self.ax_err,    "Tracking error per joint — |commanded − actual|",
                    "time (s)", "error (°)")
        for ax in (self.ax_xyz, self.ax_joints, self.ax_err):
            ax.set_xlim(t_min, now + 0.1)

        # hand XYZ
        if hand:
            arr = np.array(hand)
            t = arr[:, 0]
            self.ax_xyz.plot(t, arr[:, 1], color="#ff6b6b", linewidth=1.5, label="x (forward)")
            self.ax_xyz.plot(t, arr[:, 2], color="#51cf66", linewidth=1.5, label="y (lateral)")
            self.ax_xyz.plot(t, arr[:, 3], color="#339af0", linewidth=1.5, label="z (vertical)")
            self.ax_xyz.legend(loc="upper right", fontsize=8, frameon=True,
                              facecolor=BG, edgecolor=GRID, labelcolor=FG, ncol=3)

        # joint cmd/actual
        if cmd:
            tc = np.array([c[0] for c in cmd])
            qc = np.degrees(np.array([c[1] for c in cmd]))
            for i in range(6):
                self.ax_joints.plot(tc, qc[:, i], color=JOINT_COLORS[i],
                                    linewidth=1.4, label=SHORT_NAMES[i])
        if actual:
            ta = np.array([a[0] for a in actual])
            qa = np.degrees(np.array([a[1] for a in actual]))
            for i in range(6):
                self.ax_joints.plot(ta, qa[:, i], color=JOINT_COLORS[i],
                                    linewidth=1.1, linestyle="--", alpha=0.75)
        if cmd or actual:
            self.ax_joints.legend(loc="upper right", fontsize=8, ncol=3,
                                  frameon=True, facecolor=BG, edgecolor=GRID,
                                  labelcolor=FG)

        # error
        errs_by_joint = self._tracking_error(cmd, actual)
        any_err = False
        for i, (te, ee) in enumerate(errs_by_joint):
            if te.size == 0:
                continue
            any_err = True
            self.ax_err.plot(te, ee, color=JOINT_COLORS[i], linewidth=1.3,
                             label=SHORT_NAMES[i])
        if any_err:
            self.ax_err.legend(loc="upper right", fontsize=8, ncol=3,
                              frameon=True, facecolor=BG, edgecolor=GRID,
                              labelcolor=FG)
            # 5° "good tracking" reference line
            self.ax_err.axhline(5.0, color=FG, alpha=0.3, linestyle=":", linewidth=1.0)
            self.ax_err.text(t_min + 0.2, 5.2, "5° target", color=FG,
                             alpha=0.6, fontsize=8)

        self.canvas.draw_idle()

    # ------------------------------ stats -------------------------------- #

    def _tracking_error(self, cmd, actual):
        if not cmd or not actual:
            return [(np.array([]), np.array([]))] * 6
        tc = np.array([c[0] for c in cmd])
        qc = np.degrees(np.array([c[1] for c in cmd]))
        ta = np.array([a[0] for a in actual])
        qa = np.degrees(np.array([a[1] for a in actual]))
        if len(ta) < 2:
            return [(np.array([]), np.array([]))] * 6
        return [(tc, np.abs(qc[:, i] - np.interp(tc, ta, qa[:, i]))) for i in range(6)]

    def _update_stats(self, cmd, actual) -> None:
        if not cmd or not actual:
            for row_cells in self.stat_labels:
                for c, (var, _) in enumerate(row_cells):
                    if c != 0:
                        var.set("—")
            return

        errs = self._tracking_error(cmd, actual)
        qc = np.degrees(np.array([c[1] for c in cmd]))
        jitter = (np.std(np.diff(qc, axis=0), axis=0)
                  if qc.shape[0] > 2 else np.zeros(6))

        for i, (te, ee) in enumerate(errs):
            rms = float(np.sqrt((ee ** 2).mean())) if ee.size else 0.0
            mx = float(ee.max()) if ee.size else 0.0
            jit = float(jitter[i])

            if jit > 10.0 or mx > 15.0:
                flag, style = "⚠ UNSTABLE", DANGER
            elif jit > 3.0 or mx > 8.0:
                flag, style = "△ JITTERY", WARNING
            else:
                flag, style = "✓ OK", SUCCESS

            cells = self.stat_labels[i]
            cells[1][0].set(f"{rms:5.2f} °")
            cells[2][0].set(f"{mx:5.2f} °")
            cells[3][0].set(f"{jit:5.2f}")
            cells[4][0].set(flag)
            cells[4][1].configure(bootstyle=style)


def main() -> None:
    client = RosClient()
    root = ttkb.Window(themename="darkly")
    app = Dashboard(root, client)

    def _on_close():
        client.terminate()
        root.destroy()
    root.protocol("WM_DELETE_WINDOW", _on_close)

    # Publish initial gains so teleop picks them up immediately
    app._on_slider_change()
    root.mainloop()


if __name__ == "__main__":
    main()
