# Rule — Real-robot safety

The MyCobot 320 Pi can be commanded from the PC Tour over a TCP bridge. Mistakes hurt (hardware, cables, the operator's hand, your ego). Before anything that moves motors:

## Mandatory checklist

1. Pi is on `10.10.0.223`. `ping -c 1 10.10.0.223` first.
2. Run [`scripts/real_robot_preflight.sh`](../../scripts/real_robot_preflight.sh) — five checks (ping, TCP port 5005, bridge TCP handshake, ROS graph, round-trip `get_angles`). Exit codes tell you which step failed.
3. On the Pi, `python3 bridge_pi_simple.py` must be running.
4. Servos at rest? `send_angles` commands return within 300 ms? If not — stop.
5. **Physically hold or secure the arm** before issuing commands that release the servos (`stop`, power cycle).

## Gains and teleop

- **Never** start a teleop session on the Nominal preset (1.2/1.2/1.6/0.25). Start with **🐢 Safe start** (0.6/0.6/0.6/0.30), validate tracking in sim first, then step up on the real robot.
- The validated Nominal gains on 22/04/2026 are the upper bound of what's been physically tested. Do not go beyond without re-running `performance_analyzer.py --guided` first.
- `time_from_start < 0.15 s` has never been tested on the real robot. Treat as experimental.

## No-gripper warning

The current physical robot **has no gripper**. Any teleop or control script must be launched with `--no-gripper` (for `mycobot_teleop.py`) or equivalent. The bridge will silently accept gripper commands but they go nowhere, which masks bugs.

## Emergency stop

- Dashboard: click **⊘ Stop (release servos)**. Hold the arm before you click — it goes limp.
- Keyboard: Ctrl+C in the launch terminal doesn't stop the servos, only kills ROS nodes. The arm holds its last commanded pose.
- Hardware: unplug the Pi's power. Always an option.

## Never commit

- Any IP or port change in launch files without updating [`docs/ARCHITECTURE.md`](../../docs/ARCHITECTURE.md)
- A commit that disables the preflight script or shortens the safe-start protocol — even if "it always works"
- Force-push to `main` or to an active feature branch someone else is reviewing
