#!/usr/bin/env python3
"""Performance analyzer for the MyCobot 320 Pi hand teleoperation stack.

Connects to rosbridge (ws://localhost:9090) while the sim teleop is running,
records:
  - /teleop/hand_xyz                        (hand pose input)
  - /mycobot_controller/joint_trajectory    (commanded joint angles)
  - /joint_states                           (actual joint angles from Gazebo)

then computes precision + robustness metrics and exports an Excel workbook
(xlsxwriter) with per-sheet breakdowns and embedded charts.

Modes
-----

    python3 performance_analyzer.py --duration 60
        Passive recording for 60 s while you freely move. No prompts.

    python3 performance_analyzer.py --guided
        Scripted motion protocol: idle → up/down → lateral →
        forward/back → gripper → combined. Each phase is timed, the
        operator follows on-screen instructions, per-phase stats are
        produced in the output.

    python3 performance_analyzer.py --out my_run.xlsx
        Override default output filename (teleop_report_<timestamp>.xlsx).

Expected usage: run the whole teleop stack (T1 rosbridge, T2 Gazebo,
T3 mycobot_teleop.py, T4 dashboard if you want), start this tool in
a fifth terminal, follow the prompts, collect the produced .xlsx.
"""
from __future__ import annotations

import argparse
import dataclasses
import os
import sys
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import numpy as np
import pandas as pd
import roslibpy
import xlsxwriter


# ---------------------------- constants ---------------------------------- #

MYCOBOT_JOINTS = [
    "joint2_to_joint1",
    "joint3_to_joint2",
    "joint4_to_joint3",
    "joint5_to_joint4",
    "joint6_to_joint5",
    "joint6output_to_joint6",
]
JOINT_SHORT = ["J1 yaw", "J2 shoulder", "J3 elbow", "J4 wrist1", "J5 yaw", "J6 roll"]
PUBLISH_RATE_TARGET_HZ = 60.0  # what mycobot_teleop.py aims for

# Acceptance thresholds borrowed from the dashboard stability flag.
ERR_MAX_OK = 15.0       # deg — single-sample tracking error below this is fine
ERR_RMS_OK = 5.0        # deg — RMS error below this = good tracking
JITTER_OK = 3.0         # deg — cmd-to-cmd std below this = stable signal


# ---------------------------- scenarios ---------------------------------- #

@dataclasses.dataclass
class Scenario:
    name: str
    instruction: str
    duration_s: float


GUIDED_PROTOCOL: list[Scenario] = [
    Scenario("idle",          "Keep your palm still in front of the camera (baseline drift check)", 8.0),
    Scenario("up_down",       "Move your hand UP and DOWN slowly, 15–20 cm amplitude",              10.0),
    Scenario("left_right",    "Move your hand LEFT and RIGHT slowly, 15–20 cm amplitude",           10.0),
    Scenario("forward_back",  "Move your hand FORWARD and BACK (toward/away from camera)",          10.0),
    Scenario("combined",      "Free 3-D motion — trace a figure-8 or circle in space",              12.0),
    Scenario("gripper",       "Open and close your hand twice, wait, then once more",                8.0),
    Scenario("rest",          "Stop moving and hold still (settle/overshoot check)",                 6.0),
]


# ---------------------------- ROS client --------------------------------- #

class RosClient:
    def __init__(self, host: str = "localhost", port: int = 9090) -> None:
        self.ros = roslibpy.Ros(host=host, port=port)
        self.ros.run()
        if not self.ros.is_connected:
            raise SystemExit(f"cannot reach rosbridge ws://{host}:{port}")

        self._lock = threading.Lock()
        self.hand = deque()
        self.cmd = deque()
        self.actual = deque()

        self._t0 = time.perf_counter()

        self._th = roslibpy.Topic(self.ros, "/teleop/hand_xyz",
                                  "geometry_msgs/Vector3Stamped")
        self._th.subscribe(self._on_hand)
        self._tc = roslibpy.Topic(self.ros, "/mycobot_controller/joint_trajectory",
                                  "trajectory_msgs/JointTrajectory")
        self._tc.subscribe(self._on_cmd)
        self._ta = roslibpy.Topic(self.ros, "/joint_states",
                                  "sensor_msgs/JointState")
        self._ta.subscribe(self._on_actual)

    def _now(self) -> float:
        return time.perf_counter() - self._t0

    def _on_hand(self, msg: dict) -> None:
        v = msg["vector"]
        with self._lock:
            self.hand.append((self._now(), v["x"], v["y"], v["z"]))

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
            self.cmd.append((self._now(), *reordered))

    def _on_actual(self, msg: dict) -> None:
        names = msg.get("name") or []
        positions = msg.get("position") or []
        try:
            reordered = [positions[names.index(j)] for j in MYCOBOT_JOINTS]
        except ValueError:
            return
        with self._lock:
            self.actual.append((self._now(), *reordered))

    def snapshot(self):
        with self._lock:
            return list(self.hand), list(self.cmd), list(self.actual)

    def terminate(self) -> None:
        for t in (self._th, self._tc, self._ta):
            try: t.unsubscribe()
            except Exception: pass
        self.ros.terminate()


# ---------------------------- data frames -------------------------------- #

def _dfs_from_client(client: RosClient) -> tuple[pd.DataFrame, pd.DataFrame, pd.DataFrame]:
    hand, cmd, actual = client.snapshot()
    hand_df = pd.DataFrame(hand, columns=["t", "x", "y", "z"])
    cmd_cols = ["t"] + MYCOBOT_JOINTS
    actual_cols = ["t"] + MYCOBOT_JOINTS
    cmd_df = pd.DataFrame(cmd, columns=cmd_cols) if cmd else pd.DataFrame(columns=cmd_cols)
    actual_df = pd.DataFrame(actual, columns=actual_cols) if actual else pd.DataFrame(columns=actual_cols)
    return hand_df, cmd_df, actual_df


# ---------------------------- metrics ------------------------------------ #

def _slice(df: pd.DataFrame, t0: float, t1: float) -> pd.DataFrame:
    if df.empty: return df
    return df[(df["t"] >= t0) & (df["t"] <= t1)].reset_index(drop=True)


def compute_joint_tracking(cmd: pd.DataFrame, actual: pd.DataFrame) -> pd.DataFrame:
    """Interpolate actual onto commanded timestamps, compute |Δ| per joint (deg)."""
    if cmd.empty or len(actual) < 2:
        return pd.DataFrame()
    tc = cmd["t"].values
    ta = actual["t"].values
    rows = []
    for i, j in enumerate(MYCOBOT_JOINTS):
        qc_deg = np.degrees(cmd[j].values)
        qa_deg = np.degrees(actual[j].values)
        qa_on_tc = np.interp(tc, ta, qa_deg)
        err = np.abs(qc_deg - qa_on_tc)
        # cmd jitter = std of consecutive command deltas
        if len(qc_deg) > 2:
            cmd_jitter = float(np.std(np.diff(qc_deg)))
        else:
            cmd_jitter = 0.0
        # peak commanded velocity (deg/s)
        if len(qc_deg) > 2 and tc[-1] > tc[0]:
            dt = np.diff(tc)
            dt = np.where(dt > 0, dt, np.nan)
            v_cmd = np.abs(np.diff(qc_deg)) / dt
            v_cmd_peak = float(np.nanpercentile(v_cmd, 99))
        else:
            v_cmd_peak = 0.0

        rms = float(np.sqrt((err ** 2).mean())) if len(err) else 0.0
        mx = float(err.max()) if len(err) else 0.0
        p50 = float(np.percentile(err, 50)) if len(err) else 0.0
        p90 = float(np.percentile(err, 90)) if len(err) else 0.0
        p99 = float(np.percentile(err, 99)) if len(err) else 0.0

        if mx > ERR_MAX_OK or cmd_jitter > JITTER_OK * 2:
            flag = "UNSTABLE"
        elif rms > ERR_RMS_OK or cmd_jitter > JITTER_OK:
            flag = "JITTERY"
        else:
            flag = "OK"

        rows.append({
            "joint": JOINT_SHORT[i],
            "rms_err_deg": round(rms, 2),
            "max_err_deg": round(mx, 2),
            "p50_err_deg": round(p50, 2),
            "p90_err_deg": round(p90, 2),
            "p99_err_deg": round(p99, 2),
            "cmd_jitter_deg": round(cmd_jitter, 3),
            "peak_cmd_vel_deg_s": round(v_cmd_peak, 1),
            "n_samples": int(len(err)),
            "flag": flag,
        })
    return pd.DataFrame(rows)


def compute_signal_health(hand: pd.DataFrame, cmd: pd.DataFrame,
                          duration_s: float) -> dict:
    hand_n = len(hand)
    cmd_n = len(cmd)
    hand_hz = hand_n / duration_s if duration_s > 0 else 0.0
    cmd_hz = cmd_n / duration_s if duration_s > 0 else 0.0
    # Detection dropout: any gap > 0.5s in hand_xyz
    dropouts = 0
    longest_gap_s = 0.0
    if hand_n > 1:
        dt = np.diff(hand["t"].values)
        dropouts = int((dt > 0.5).sum())
        longest_gap_s = float(dt.max())
    return {
        "hand_samples": hand_n,
        "cmd_samples": cmd_n,
        "hand_rate_hz": round(hand_hz, 2),
        "cmd_rate_hz": round(cmd_hz, 2),
        "cmd_rate_target_hz": PUBLISH_RATE_TARGET_HZ,
        "rate_health_pct": round(100.0 * cmd_hz / PUBLISH_RATE_TARGET_HZ, 1),
        "detection_dropouts_count": dropouts,
        "longest_gap_s": round(longest_gap_s, 3),
        "duration_s": round(duration_s, 2),
    }


def compute_workspace(hand: pd.DataFrame) -> dict:
    if hand.empty:
        return {"x_range_mm": 0, "y_range_mm": 0, "z_range_mm": 0,
                "x_std_mm": 0, "y_std_mm": 0, "z_std_mm": 0,
                "total_path_m": 0}
    x = hand["x"].values; y = hand["y"].values; z = hand["z"].values
    path_m = float(np.sqrt(np.diff(x)**2 + np.diff(y)**2 + np.diff(z)**2).sum()) \
             if len(x) > 1 else 0.0
    return {
        "x_range_mm": round((x.max() - x.min()) * 1000, 1),
        "y_range_mm": round((y.max() - y.min()) * 1000, 1),
        "z_range_mm": round((z.max() - z.min()) * 1000, 1),
        "x_std_mm":   round(x.std() * 1000, 1),
        "y_std_mm":   round(y.std() * 1000, 1),
        "z_std_mm":   round(z.std() * 1000, 1),
        "total_path_m": round(path_m, 2),
    }


def compute_overall_verdict(joint_df: pd.DataFrame, health: dict) -> dict:
    if joint_df.empty:
        return {"verdict": "NO DATA", "reason": "no command samples recorded"}

    # Verdict based on the joints we actively drive (skip wrist1/J6 roll which
    # are held at zero).
    driven = joint_df[~joint_df["joint"].isin(["J4 wrist1", "J6 roll"])]
    n_unstable = (driven["flag"] == "UNSTABLE").sum()
    n_jittery  = (driven["flag"] == "JITTERY").sum()

    rate_ok = health["rate_health_pct"] >= 70.0
    dropout_ok = health["detection_dropouts_count"] <= 2

    if n_unstable > 0 or not rate_ok or not dropout_ok:
        verdict = "NOT READY FOR REAL"
        reason = []
        if n_unstable > 0:
            reason.append(f"{n_unstable} joints UNSTABLE")
        if not rate_ok:
            reason.append(f"publish rate {health['rate_health_pct']}% of target")
        if not dropout_ok:
            reason.append(f"{health['detection_dropouts_count']} detection dropouts")
        return {"verdict": verdict, "reason": " · ".join(reason)}

    if n_jittery > 0:
        return {"verdict": "CAUTIOUS — REAL OK WITH REDUCED SPEED",
                "reason": f"{n_jittery} joints JITTERY — halve --x/y/z-gain before real"}

    return {"verdict": "READY FOR REAL ROBOT", "reason": "all driven joints OK, rate nominal"}


# ---------------------------- Excel export ------------------------------- #

def write_report(path: Path, hand: pd.DataFrame, cmd: pd.DataFrame,
                 actual: pd.DataFrame, duration_s: float,
                 scenarios: list[tuple[str, float, float]] | None = None) -> None:
    joint_df = compute_joint_tracking(cmd, actual)
    health = compute_signal_health(hand, cmd, duration_s)
    workspace = compute_workspace(hand)
    verdict = compute_overall_verdict(joint_df, health)

    wb = xlsxwriter.Workbook(str(path), {"nan_inf_to_errors": True})

    fmt_title   = wb.add_format({"bold": True, "font_size": 14, "bg_color": "#1971c2", "font_color": "white", "align": "left", "valign": "vcenter"})
    fmt_section = wb.add_format({"bold": True, "font_size": 11, "bg_color": "#dee2e6", "align": "left"})
    fmt_label   = wb.add_format({"bold": True, "align": "left"})
    fmt_num     = wb.add_format({"num_format": "0.00"})
    fmt_int     = wb.add_format({"num_format": "0"})
    fmt_ok      = wb.add_format({"bold": True, "bg_color": "#b2f2bb", "align": "center"})
    fmt_jit     = wb.add_format({"bold": True, "bg_color": "#ffec99", "align": "center"})
    fmt_bad     = wb.add_format({"bold": True, "bg_color": "#ffa8a8", "align": "center"})
    fmt_verdict_ok  = wb.add_format({"bold": True, "font_size": 14, "bg_color": "#51cf66", "font_color": "white", "align": "center"})
    fmt_verdict_mid = wb.add_format({"bold": True, "font_size": 14, "bg_color": "#f59f00", "font_color": "white", "align": "center"})
    fmt_verdict_bad = wb.add_format({"bold": True, "font_size": 14, "bg_color": "#e03131", "font_color": "white", "align": "center"})

    def flag_fmt(flag: str):
        return {"OK": fmt_ok, "JITTERY": fmt_jit, "UNSTABLE": fmt_bad}.get(flag, fmt_num)

    # ---- SHEET 1: Summary --------------------------------------------------
    ws = wb.add_worksheet("Summary")
    ws.set_column("A:A", 32); ws.set_column("B:B", 30); ws.set_column("C:Z", 14)
    ws.merge_range("A1:E1", f"MyCobot Teleop — performance report ({datetime.now().strftime('%Y-%m-%d %H:%M')})", fmt_title)

    ws.write("A3", "VERDICT", fmt_section)
    v_fmt = fmt_verdict_ok if verdict["verdict"].startswith("READY") \
            else fmt_verdict_mid if verdict["verdict"].startswith("CAUT") \
            else fmt_verdict_bad
    ws.merge_range("B3:E3", verdict["verdict"], v_fmt)
    ws.write("A4", "Reason"); ws.merge_range("B4:E4", verdict["reason"])

    ws.write("A6", "Overall signal health", fmt_section)
    row = 7
    for k, v in health.items():
        ws.write(row, 0, k); ws.write(row, 1, v); row += 1

    ws.write(row + 1, 0, "Workspace used (hand)", fmt_section)
    row += 2
    for k, v in workspace.items():
        ws.write(row, 0, k); ws.write(row, 1, v); row += 1

    # ---- SHEET 2: Per-joint tracking --------------------------------------
    ws = wb.add_worksheet("Per-joint tracking")
    ws.set_column("A:A", 16); ws.set_column("B:J", 13)
    ws.merge_range("A1:J1", "Per-joint commanded vs actual tracking", fmt_title)
    headers = ["joint", "rms_err_deg", "max_err_deg", "p50_err_deg",
               "p90_err_deg", "p99_err_deg", "cmd_jitter_deg",
               "peak_cmd_vel_deg_s", "n_samples", "flag"]
    for c, h in enumerate(headers):
        ws.write(2, c, h, fmt_section)
    if not joint_df.empty:
        for r, (_, row_) in enumerate(joint_df.iterrows(), start=3):
            ws.write(r, 0, row_["joint"], fmt_label)
            for c, h in enumerate(headers[1:-1], start=1):
                ws.write_number(r, c, row_[h])
            ws.write(r, 9, row_["flag"], flag_fmt(row_["flag"]))

        # Bar chart of RMS error per joint
        chart = wb.add_chart({"type": "column"})
        n = len(joint_df)
        chart.add_series({
            "name": "RMS tracking error (°)",
            "categories": ["Per-joint tracking", 3, 0, 3 + n - 1, 0],
            "values":     ["Per-joint tracking", 3, 1, 3 + n - 1, 1],
            "fill": {"color": "#4dabf7"},
        })
        chart.add_series({
            "name": "Max tracking error (°)",
            "categories": ["Per-joint tracking", 3, 0, 3 + n - 1, 0],
            "values":     ["Per-joint tracking", 3, 2, 3 + n - 1, 2],
            "fill": {"color": "#f59f00"},
        })
        chart.set_title({"name": "Tracking error per joint"})
        chart.set_x_axis({"name": "joint"})
        chart.set_y_axis({"name": "error (°)"})
        chart.set_size({"width": 640, "height": 320})
        ws.insert_chart("L3", chart)

    # ---- SHEET 3: Scenario breakdown (if guided) ---------------------------
    if scenarios:
        ws = wb.add_worksheet("Scenarios")
        ws.set_column("A:A", 18); ws.set_column("B:H", 14)
        ws.merge_range("A1:H1", "Guided scenario breakdown", fmt_title)
        ws.write_row(2, 0, ["scenario", "duration_s", "hand_samples",
                            "max_err_deg (all joints)", "rms_err_deg (driven)",
                            "x_range_mm", "y_range_mm", "z_range_mm"],
                     fmt_section)
        for r, (sname, t0, t1) in enumerate(scenarios, start=3):
            h_s = _slice(hand, t0, t1); c_s = _slice(cmd, t0, t1); a_s = _slice(actual, t0, t1)
            jdf = compute_joint_tracking(c_s, a_s)
            wks = compute_workspace(h_s)
            driven = jdf[~jdf["joint"].isin(["J4 wrist1", "J6 roll"])] if not jdf.empty else jdf
            rms_mean = float(driven["rms_err_deg"].mean()) if not driven.empty else 0.0
            max_err  = float(jdf["max_err_deg"].max()) if not jdf.empty else 0.0
            ws.write(r, 0, sname, fmt_label)
            ws.write_number(r, 1, round(t1 - t0, 2))
            ws.write_number(r, 2, len(h_s))
            ws.write_number(r, 3, round(max_err, 2))
            ws.write_number(r, 4, round(rms_mean, 2))
            ws.write_number(r, 5, wks["x_range_mm"])
            ws.write_number(r, 6, wks["y_range_mm"])
            ws.write_number(r, 7, wks["z_range_mm"])

    # ---- SHEET 4: Signal health detail -------------------------------------
    ws = wb.add_worksheet("Signal health")
    ws.set_column("A:B", 28)
    ws.merge_range("A1:B1", "Streaming / detection health", fmt_title)
    row = 2
    for k, v in health.items():
        ws.write(row, 0, k, fmt_label); ws.write(row, 1, v); row += 1
    ws.write(row + 1, 0, "cmd_rate_vs_target_pct", fmt_label)
    ws.write_number(row + 1, 1, health["rate_health_pct"])
    ws.write(row + 3, 0, "thresholds used for flags", fmt_section)
    ws.write(row + 4, 0, "max tracking error ≤"); ws.write_number(row + 4, 1, ERR_MAX_OK)
    ws.write(row + 5, 0, "RMS tracking error ≤"); ws.write_number(row + 5, 1, ERR_RMS_OK)
    ws.write(row + 6, 0, "cmd jitter ≤");         ws.write_number(row + 6, 1, JITTER_OK)

    # ---- SHEET 5: Raw hand ----
    if not hand.empty:
        hand.to_excel(pd.ExcelWriter(str(path), engine="xlsxwriter", mode="a"), sheet_name="raw_hand", index=False) if False else None
        # Workaround: use a separate writer below
    # We already have the workbook open; write manually.
    ws = wb.add_worksheet("raw_hand")
    ws.write_row(0, 0, ["t_s", "x_m", "y_m", "z_m"], fmt_section)
    for r, row_ in enumerate(hand.itertuples(index=False), start=1):
        ws.write_row(r, 0, list(row_))

    # ---- SHEET 6: Raw cmd ----
    ws = wb.add_worksheet("raw_cmd")
    ws.write_row(0, 0, ["t_s"] + JOINT_SHORT, fmt_section)
    for r, row_ in enumerate(cmd.itertuples(index=False), start=1):
        vals = list(row_)
        # cmd values are radians → convert to deg for readability
        vals_deg = [vals[0]] + [np.degrees(v) for v in vals[1:]]
        ws.write_row(r, 0, vals_deg)

    # ---- SHEET 7: Raw actual ----
    ws = wb.add_worksheet("raw_actual")
    ws.write_row(0, 0, ["t_s"] + JOINT_SHORT, fmt_section)
    for r, row_ in enumerate(actual.itertuples(index=False), start=1):
        vals = list(row_)
        vals_deg = [vals[0]] + [np.degrees(v) for v in vals[1:]]
        ws.write_row(r, 0, vals_deg)

    wb.close()


# ---------------------------- main --------------------------------------- #

def run_passive(client: RosClient, duration_s: float) -> tuple[float, None]:
    print(f"\nRecording for {duration_s:.0f}s — move freely.", flush=True)
    start = time.perf_counter()
    while time.perf_counter() - start < duration_s:
        remaining = int(duration_s - (time.perf_counter() - start))
        sys.stdout.write(f"\r  remaining: {remaining:3d}s")
        sys.stdout.flush()
        time.sleep(1)
    print("\n  done.")
    return duration_s, None


def run_guided(client: RosClient) -> tuple[float, list[tuple[str, float, float]]]:
    scenarios = []
    total_start = client._now()
    for sc in GUIDED_PROTOCOL:
        print(f"\n▶  {sc.name.upper():<14} — {sc.instruction}")
        print(f"   ({sc.duration_s:.0f}s)", flush=True)
        t0 = client._now()
        start = time.perf_counter()
        while time.perf_counter() - start < sc.duration_s:
            remaining = int(sc.duration_s - (time.perf_counter() - start))
            sys.stdout.write(f"\r   {remaining:3d}s ")
            sys.stdout.flush()
            time.sleep(1)
        t1 = client._now()
        scenarios.append((sc.name, t0, t1))
        print()
    total_end = client._now()
    return total_end - total_start, scenarios


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--duration", type=float, default=None,
                   help="Passive record for N seconds (mutually exclusive with --guided)")
    p.add_argument("--guided", action="store_true",
                   help="Step through the scripted motion protocol")
    p.add_argument("--host", default="localhost")
    p.add_argument("--port", type=int, default=9090)
    p.add_argument("--out", default=None,
                   help="Output .xlsx (default: teleop_report_<timestamp>.xlsx)")
    args = p.parse_args()

    if not args.guided and args.duration is None:
        args.duration = 60.0  # sensible default

    client = RosClient(args.host, args.port)
    print(f"Connected to rosbridge at ws://{args.host}:{args.port}", flush=True)

    try:
        if args.guided:
            duration_s, scenarios = run_guided(client)
        else:
            duration_s, scenarios = run_passive(client, args.duration)
    except KeyboardInterrupt:
        print("\ninterrupted — computing report from partial data.")
        duration_s = client._now()
        scenarios = None

    hand_df, cmd_df, actual_df = _dfs_from_client(client)
    client.terminate()

    out_path = Path(args.out) if args.out else Path(
        f"teleop_report_{datetime.now().strftime('%Y%m%d_%H%M%S')}.xlsx"
    )
    write_report(out_path, hand_df, cmd_df, actual_df, duration_s, scenarios)

    # Console summary
    joint_df = compute_joint_tracking(cmd_df, actual_df)
    health = compute_signal_health(hand_df, cmd_df, duration_s)
    verdict = compute_overall_verdict(joint_df, health)

    print("\n" + "=" * 62)
    print(f"VERDICT: {verdict['verdict']}")
    print(f"Reason : {verdict['reason']}")
    print("-" * 62)
    print(f"Duration       : {duration_s:.1f}s   "
          f"hand {health['hand_rate_hz']} Hz · cmd {health['cmd_rate_hz']} Hz "
          f"({health['rate_health_pct']}% target)")
    print(f"Detection dropouts : {health['detection_dropouts_count']}   "
          f"longest gap: {health['longest_gap_s']}s")
    if not joint_df.empty:
        print(f"\n{'joint':<14} {'rms':>8} {'max':>8} {'p90':>8} {'jitter':>8}  flag")
        for _, r in joint_df.iterrows():
            print(f"{r['joint']:<14} {r['rms_err_deg']:>8.2f} "
                  f"{r['max_err_deg']:>8.2f} {r['p90_err_deg']:>8.2f} "
                  f"{r['cmd_jitter_deg']:>8.3f}  {r['flag']}")
    print("=" * 62)
    print(f"\nReport written to: {out_path.resolve()}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
