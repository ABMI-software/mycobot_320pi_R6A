#!/usr/bin/env python3
"""Live tuning and monitoring dashboard for MyCobot teleoperation.

Connects to rosbridge (ws://localhost:9090) and:
  - Subscribes to /teleop/hand_xyz          (hand position from Wilor)
  - Subscribes to /mycobot_controller/joint_trajectory  (commanded angles)
  - Subscribes to /joint_states             (actual robot angles from Gazebo)
  - Publishes to   /teleop/gains            (live x/y/z gains + time-from-start)

Shows:
  - Sliders for x_gain, y_gain, z_gain, time_from_start (applied instantly)
  - Live plots: hand XYZ, commanded vs actual per joint, tracking error
  - Stability stats: RMS error, max error per joint

Run from the hand-teleop conda env (same one running mycobot_teleop.py):
    conda activate hand-teleop
    python3 teleop/teleop_dashboard.py
"""
from __future__ import annotations

import math
import threading
import time
import tkinter as tk
from collections import deque
from tkinter import ttk

import numpy as np

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

import roslibpy


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


class RosClient:
    """Thread-safe buffered rosbridge client."""

    def __init__(self, host: str = "localhost", port: int = 9090) -> None:
        self.ros = roslibpy.Ros(host=host, port=port)
        self.ros.run()
        if not self.ros.is_connected:
            raise SystemExit(f"Cannot reach rosbridge at ws://{host}:{port}")

        self._lock = threading.Lock()
        self.hand_xyz = deque(maxlen=MAX_SAMPLES)     # (t, x, y, z)
        self.cmd_joints = deque(maxlen=MAX_SAMPLES)   # (t, [q1..q6] rad)
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
        try:
            self._t_hand.unsubscribe()
            self._t_cmd.unsubscribe()
            self._t_actual.unsubscribe()
            self._p_gains.unadvertise()
        except Exception:
            pass
        self.ros.terminate()


class Dashboard:
    def __init__(self, root: tk.Tk, client: RosClient) -> None:
        self.root = root
        self.client = client

        root.title("MyCobot Teleop Dashboard")
        root.geometry("1200x850")

        # ---- Top frame: gain sliders ----
        gain_frame = ttk.LabelFrame(root, text="Live gain tuning")
        gain_frame.pack(side=tk.TOP, fill=tk.X, padx=8, pady=(8, 4))

        self.gain_vars = {
            "x": tk.DoubleVar(value=1.2),
            "y": tk.DoubleVar(value=1.2),
            "z": tk.DoubleVar(value=1.6),
            "tfs": tk.DoubleVar(value=0.8),
        }
        self._build_slider(gain_frame, 0, "X gain (hand forward → J2)", "x", 0.3, 3.0, 0.01)
        self._build_slider(gain_frame, 1, "Y gain (hand lateral → J1)", "y", 0.3, 3.0, 0.01)
        self._build_slider(gain_frame, 2, "Z gain (hand vertical → J3,J5)", "z", 0.3, 3.0, 0.01)
        self._build_slider(gain_frame, 3, "time_from_start (s)", "tfs", 0.1, 2.5, 0.05)

        stats_frame = ttk.LabelFrame(root, text="Tracking stability (last 10s)")
        stats_frame.pack(side=tk.TOP, fill=tk.X, padx=8, pady=4)
        self.stats_var = tk.StringVar(value="(waiting for data…)")
        ttk.Label(stats_frame, textvariable=self.stats_var,
                  font=("Monospace", 10), justify=tk.LEFT).pack(anchor="w", padx=6, pady=4)

        # ---- Plots ----
        plot_frame = ttk.LabelFrame(root, text="Live signals")
        plot_frame.pack(side=tk.TOP, fill=tk.BOTH, expand=True, padx=8, pady=(4, 8))

        self.fig = plt.Figure(figsize=(11, 6), dpi=96, layout="constrained")
        self.ax_xyz = self.fig.add_subplot(3, 1, 1)
        self.ax_joints = self.fig.add_subplot(3, 1, 2)
        self.ax_err = self.fig.add_subplot(3, 1, 3)

        self.ax_xyz.set_title("Wilor hand XYZ (m)")
        self.ax_joints.set_title("Joint angles (deg): solid = commanded, dashed = actual")
        self.ax_err.set_title("Tracking error (deg): |commanded − actual| per joint")

        self.canvas = FigureCanvasTkAgg(self.fig, master=plot_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # Periodic refresh
        self.root.after(200, self._refresh)

    def _build_slider(self, parent, row: int, label: str, key: str,
                      lo: float, hi: float, step: float) -> None:
        ttk.Label(parent, text=label, width=35).grid(row=row, column=0, sticky="w", padx=4, pady=2)
        var = self.gain_vars[key]
        value_lbl = ttk.Label(parent, textvariable=var, width=5)
        value_lbl.grid(row=row, column=2, padx=4)
        scale = ttk.Scale(parent, from_=lo, to=hi, orient=tk.HORIZONTAL, variable=var,
                          command=lambda _v: self._on_slider_change())
        scale.grid(row=row, column=1, sticky="ew", padx=4)
        parent.columnconfigure(1, weight=1)
        # Snap DoubleVar display to 2 decimals on update
        def _update_label(*_):
            value_lbl.config(text=f"{var.get():.2f}")
        var.trace_add("write", _update_label)
        _update_label()

    def _on_slider_change(self) -> None:
        self.client.publish_gains(
            self.gain_vars["x"].get(),
            self.gain_vars["y"].get(),
            self.gain_vars["z"].get(),
            self.gain_vars["tfs"].get(),
        )

    def _refresh(self) -> None:
        hand, cmd, actual = self.client.snapshot()
        self._redraw(hand, cmd, actual)
        self._update_stats(cmd, actual)
        self.root.after(250, self._refresh)

    def _redraw(self, hand, cmd, actual) -> None:
        now = time.perf_counter() - self.client._t0
        t_min = max(0.0, now - PLOT_WINDOW_S)

        self.ax_xyz.cla()
        self.ax_joints.cla()
        self.ax_err.cla()

        self.ax_xyz.set_title("Wilor hand XYZ (m)")
        self.ax_joints.set_title("Joint angles (deg): solid=commanded  dashed=actual")
        self.ax_err.set_title("Tracking error (deg): |commanded − actual| per joint")
        for ax in (self.ax_xyz, self.ax_joints, self.ax_err):
            ax.grid(True, alpha=0.3)
            ax.set_xlim(t_min, now + 0.1)

        # --- hand XYZ ---
        if hand:
            arr = np.array(hand)
            t = arr[:, 0]
            self.ax_xyz.plot(t, arr[:, 1], label="x", color="tab:red")
            self.ax_xyz.plot(t, arr[:, 2], label="y", color="tab:green")
            self.ax_xyz.plot(t, arr[:, 3], label="z", color="tab:blue")
            self.ax_xyz.legend(loc="upper right", fontsize=8)

        # --- joint commanded vs actual ---
        colors = plt.cm.tab10.colors[:6]
        if cmd:
            tc = np.array([c[0] for c in cmd])
            qc = np.degrees(np.array([c[1] for c in cmd]))
            for i in range(6):
                self.ax_joints.plot(tc, qc[:, i], color=colors[i], linewidth=1.2,
                                    label=SHORT_NAMES[i])
        if actual:
            ta = np.array([a[0] for a in actual])
            qa = np.degrees(np.array([a[1] for a in actual]))
            for i in range(6):
                self.ax_joints.plot(ta, qa[:, i], color=colors[i], linewidth=1.0,
                                    linestyle="--", alpha=0.7)
        self.ax_joints.legend(loc="upper right", fontsize=7, ncol=3)

        # --- tracking error ---
        if cmd and actual:
            errs_by_joint = self._tracking_error(cmd, actual)
            for i in range(6):
                te, ee = errs_by_joint[i]
                if te.size == 0:
                    continue
                self.ax_err.plot(te, ee, color=colors[i], label=SHORT_NAMES[i])
            self.ax_err.legend(loc="upper right", fontsize=7, ncol=3)

        self.canvas.draw_idle()

    def _tracking_error(self, cmd, actual):
        """Interpolate actual onto commanded timestamps, compute |Δ| in degrees."""
        if not cmd or not actual:
            return [(np.array([]), np.array([]))] * 6

        tc = np.array([c[0] for c in cmd])
        qc = np.degrees(np.array([c[1] for c in cmd]))
        ta = np.array([a[0] for a in actual])
        qa = np.degrees(np.array([a[1] for a in actual]))

        if len(ta) < 2:
            return [(np.array([]), np.array([]))] * 6

        out = []
        for i in range(6):
            qa_on_tc = np.interp(tc, ta, qa[:, i])
            err = np.abs(qc[:, i] - qa_on_tc)
            out.append((tc, err))
        return out

    def _update_stats(self, cmd, actual) -> None:
        if not cmd or not actual:
            self.stats_var.set("(waiting for both /mycobot_controller/joint_trajectory and /joint_states)")
            return
        errs = self._tracking_error(cmd, actual)
        lines = [
            f"{'Joint':<15} {'RMS err':>10} {'Max err':>10} {'Signal':>12}",
            "-" * 55,
        ]
        # signal stability = std dev of diff-of-cmds (high = jittery)
        qc = np.degrees(np.array([c[1] for c in cmd]))
        jitter = (np.std(np.diff(qc, axis=0), axis=0)
                  if qc.shape[0] > 2 else np.zeros(6))
        for i in range(6):
            _, ee = errs[i]
            rms = float(np.sqrt((ee ** 2).mean())) if ee.size else 0.0
            mx = float(ee.max()) if ee.size else 0.0
            sig = "OK" if jitter[i] < 3.0 else ("JITTERY" if jitter[i] < 10.0 else "UNSTABLE")
            lines.append(f"{SHORT_NAMES[i]:<15} {rms:>10.2f}° {mx:>10.2f}° "
                         f"{sig:>12} (σΔ={jitter[i]:.2f})")
        self.stats_var.set("\n".join(lines))


def main() -> None:
    client = RosClient()
    root = tk.Tk()
    app = Dashboard(root, client)

    def _on_close():
        client.terminate()
        root.destroy()

    root.protocol("WM_DELETE_WINDOW", _on_close)
    # Publish initial gains so the teleop picks them up immediately
    app._on_slider_change()
    root.mainloop()


if __name__ == "__main__":
    main()
