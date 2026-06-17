---
name: real-robot-session
description: Full ritual for a physical-robot session with the MyCobot 320 Pi. Invoke when the user says they're about to test on hardware, when a session is starting, or when something in the live pipeline is misbehaving and you want to walk through the whole chain.
---

# Real-robot session ritual

A full session has a rhythm. Skipping steps breaks things the session after next. Follow in order.

## 0. Before you start

- Pi is powered and on `10.10.0.223` (NOT `.225` — older docs lie)
- Physical arm is secured — nothing will hit the table, no cables wrapped
- Operator is at ~50 cm from the Astra, if doing teleop
- **The current physical robot has no gripper.** Any launch must include `--no-gripper` or skip gripper bridges.

## 1. Preflight (non-negotiable)

```bash
bash scripts/real_robot_preflight.sh
```

Five steps, exit code says which failed. Do not advance if any fail. See [`.claude/commands/real-robot-preflight.md`](../../commands/real-robot-preflight.md).

## 2. Start the Pi-side bridge

```bash
ssh er@10.10.0.223
# on the Pi:
python3 bridge_pi_simple.py
```

Watch for `[bridge_pi_simple] listening on 5005`. If it crashes on start: pymycobot isn't installed or `/dev/ttyAMA0` is missing (power/serial issue).

## 3. Tour-side launch

Pick the right target for your session:

- **Simple control** (no teleop): `ros2 launch mycobot_gateway simple_gui.launch.py`
- **Teleop against real robot**: `ros2 launch mycobot_gateway mycobot_teleop.launch.py target:=real`
- **Teleop with sim mirror**: `ros2 launch mycobot_gateway mycobot_teleop.launch.py target:=both`

## 4. Verify round-trip

Send a benign command, see it round-trip:

```bash
ros2 topic echo /from_robot --once
ros2 topic pub --once /to_robot std_msgs/String 'data: "get_angles"'
# ...expect a line with ANGLES: [...] within 500ms
```

If `/from_robot` stays silent — there's a receive-loop logging bug in `bridge_tour` that makes it look dead even when the Pi is sending (documented, non-blocking — see CHANGELOG 2.1.0). You can safely proceed; the angles are arriving, they're just not being logged.

## 5. Safe calibration (for teleop)

- Dashboard open
- Tuning tab → **🐢 Safe start** preset (never anything hotter on first try)
- Home tab → **⟲ Recalibrate hand origin**, palm centred at 50 cm, hold still 1 s

## 6. The actual motion

- Tiny amplitudes first — 5 cm, slow
- Watch the **Signal health** KPI card
- First UNSTABLE flag → stop, drop gains, recalibrate

## 7. Emergency protocol

If the arm does anything surprising:

1. **Hold the arm** physically with one hand
2. Ctrl+C in the launch terminal — this does NOT release servos, the arm stays in its last pose
3. If you need servos released: dashboard **⊘ Stop (release servos)** — the arm goes limp, that's why step 1 exists
4. If Ctrl+C didn't stop the motion commands: `/kill-all-ros` (the slash command, or `pkill -9 -f "mycobot_teleop|bridge_tour"`)
5. Last resort: unplug the Pi. Always an option.

## 8. After the session

- Log what you did in [`SESSION_RESUME.md`](../../../SESSION_RESUME.md) under the dated entry
- If you validated new gain values on hardware, update [`.claude/agents/teleop-tuner.md`](../../agents/teleop-tuner.md) and [`.claude/rules/real-robot-safety.md`](../../rules/real-robot-safety.md) validated-envelope table
- Export a CSV snapshot from the dashboard if the session produced anything interesting
- Commit any logs / reports to the branch you're on (check [`git-branching.md`](../../rules/git-branching.md))

## Never

- Never skip preflight "because it worked yesterday"
- Never start teleop without Safe start, even with a validated Nominal on file
- Never release servos without holding the arm
- Never leave a session without updating the resume doc — future-you (or the next operator) will thank past-you

**Docs:** [`docs/REAL_ROBOT_TEST_PROCEDURE.md`](../../../docs/REAL_ROBOT_TEST_PROCEDURE.md), [`.claude/rules/real-robot-safety.md`](../../rules/real-robot-safety.md)
