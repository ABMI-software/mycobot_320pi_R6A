---
description: Five-step preflight before sending commands to the physical MyCobot 320 Pi
---

Run the preflight script and stop at the first failure.

```bash
bash scripts/real_robot_preflight.sh
```

## What it checks (in order)

1. **Ping** `10.10.0.223` — Pi is on the network
2. **TCP 5005** — `bridge_pi_simple.py` is accepting connections
3. **ROS2 graph alive** — `ros2 topic list` works
4. **`bridge_tour` round-trip** — launches the TCP client, sends `ping`, expects `pong`
5. **`get_angles`** — sends the JSON query, parses the returned `ANGLES: [...]` array

Exit codes: 0 pass · 1–5 corresponding step failed.

## If a step fails

| Step | Likely cause | Fix |
|------|--------------|-----|
| 1 | Pi off, wrong network, wrong IP | `ssh er@10.10.0.223` to confirm; older docs said `.225` — wrong |
| 2 | `bridge_pi_simple.py` not running | `ssh er@10.10.0.223` → `python3 bridge_pi_simple.py` |
| 3 | ROS2 env not sourced, conda active | `conda deactivate; source /opt/ros/jazzy/setup.bash; source ~/ros_jazzy/install/setup.bash` |
| 4 | `bridge_tour` not installed or TCP handshake timeout | `ros2 pkg prefix mycobot_gateway` · check firewall |
| 5 | Pi bridge deadlocked | Restart `bridge_pi_simple.py` on the Pi |

**Never skip a step** because it "worked last time." The Pi reboots unattended sometimes — state drifts.

**Docs:** [`docs/REAL_ROBOT_TEST_PROCEDURE.md`](../../docs/REAL_ROBOT_TEST_PROCEDURE.md), [`.claude/rules/real-robot-safety.md`](../rules/real-robot-safety.md)
