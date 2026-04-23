---
name: teleop-troubleshoot
description: Diagnostic table for common hand-teleoperation failures (Astra, Wilor, rosbridge, JTC, bridge_tour). Invoke when a teleop session is broken or the dashboard shows no data.
---

# Teleop troubleshooting

Use this table to localise the problem. Work top-down: if step N is broken, don't debug step N+1.

## Triage by dashboard state

| Dashboard shows | Likely layer | Go to |
|-----------------|--------------|-------|
| ⚪ OFFLINE badge | Rosbridge or teleop publisher | §1, §2 |
| Camera panel empty, rest OK | `mycobot_teleop.py` not publishing `/teleop/camera/image` | §2 |
| SIM avg RMS "—" but REAL OK | Gazebo not publishing `/joint_states` | §3 |
| REAL avg RMS "—" but SIM OK | `bridge_tour` or Pi not alive | §4 |
| Command rate stuck at 0 Hz | Teleop is running but not publishing commands | §2 |
| Signal health UNSTABLE on all joints | Gains too aggressive, or tracker noisy | §5 |

## §1 — Rosbridge layer

```bash
nc -zv localhost 9090
```

- `Connection refused` → rosbridge not running. Launch T1 (`rosbridge_websocket_launch.xml`).
- Port already in use by a zombie — check `lsof -iTCP:9090 -sTCP:LISTEN`, kill the PID.
- Rosbridge running but dashboard still OFFLINE → teleop script was started **without** `--use-rosbridge`.

## §2 — Teleop script (`mycobot_teleop.py`)

**Camera frames don't reach the dashboard:**
- PIL not installed in `hand-teleop` env → `pip install pillow`
- The main loop's `cap.read()` monkey-patch never fires → check that Wilor detected a hand at least once

**No joint commands published:**
- `tracking_paused=True` and never resumed → check logs for `[RECAL]`; publish an Empty on `/teleop/recalibrate`
- Wilor lost the hand → the script logs `No hand detected` — palm must be in frame

**`rclpy` import error:**
- You forgot `--use-rosbridge`. The conda `hand-teleop` env has Python 3.10, no `rclpy`. Always use rosbridge.

## §3 — Gazebo / SIM feedback

- `/joint_states` at 0 Hz → `joint_state_broadcaster` not active. `ros2 control list_controllers`.
- `/clock` missing → the bridge block in the launch file is incomplete. Symptoms: `Time jumped backward`, `/joint_states` timestamps frozen.
- The arm visually moves but Gazebo shadows don't update → DART physics glitch, restart Gazebo.

## §4 — Real robot / bridge_tour / Pi

```bash
bash scripts/real_robot_preflight.sh
```

- Pi unreachable → [`.claude/commands/real-robot-preflight.md`](../../commands/real-robot-preflight.md)
- bridge_tour receive_loop silent (no `📥 Reçu de Pi`) — **non-blocking bug**: the Pi sends fine, Tower logging is just missing. Don't spend cycles on it, documented in [`CHANGELOG.md`](../../../CHANGELOG.md) 2.1.0.
- `send_angles` returns but robot doesn't move → check that the Pi bridge is in the correct mode (`mode="angles"`).

## §5 — Signal stability

- Robot oscillates → gains too high OR `time_from_start` too low. Drop to **🐢 Safe start** preset.
- Robot "lags" visibly → `time_from_start` too high. Keep > 0.15 s on real robot.
- J6 doesn't rotate for doorknob gestures → mapping uses `yaw`, not `roll`. If broken, check that `mycobot_teleop.py` still contains `j6 = yaw * roll_gain` — regression from an older version used `roll`.

## When to escalate

If the table doesn't narrow it down within ~10 minutes, spawn the `ros2-debugger` agent with the exact topic names and error messages you've collected.

**Related:** [`docs/TELEOPERATION.md`](../../../docs/TELEOPERATION.md), [`docs/TELEOP_TUNING.md`](../../../docs/TELEOP_TUNING.md), [`docs/TELEOP_DASHBOARD.md`](../../../docs/TELEOP_DASHBOARD.md)
