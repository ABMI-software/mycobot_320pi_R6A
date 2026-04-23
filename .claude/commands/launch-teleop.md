---
description: Start the full hand-teleop pipeline (4 terminals) — Astra → Wilor → rosbridge → robot
---

This is the canonical 4-terminal workflow for the hand-teleoperation pipeline (on `feature/teleoperation`).

## T1 — rosbridge

```bash
conda deactivate
source /opt/ros/jazzy/setup.bash
source ~/ros_jazzy/src/mycobot_R6A/install/setup.bash
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

Wait for: `Rosbridge WebSocket server started on port 9090`.

## T2 — Gazebo (or direct-to-real bridge)

For **simulation**:
```bash
ros2 launch mycobot_gateway mycobot_teleop.launch.py target:=sim
```

For **real robot** (bridge on Pi 10.10.0.223 must already be running):
```bash
ros2 launch mycobot_gateway mycobot_teleop.launch.py target:=real
```

For **both** (comparison mode — dashboard badge shows ⚡ SIM + REAL):
```bash
ros2 launch mycobot_gateway mycobot_teleop.launch.py target:=both
```

## T3 — teleop script

```bash
conda activate hand-teleop
cd ~/ros_jazzy/src/mycobot_R6A/teleop
python3 mycobot_teleop.py --camera astra --ros --use-rosbridge --no-gripper
```

The `--no-gripper` flag is **mandatory** on the current physical robot (no gripper installed).

## T4 — dashboard

```bash
conda activate hand-teleop
cd ~/ros_jazzy/src/mycobot_R6A/teleop
python3 teleop_dashboard.py
```

- Start on the **🏠 Home** tab
- Click **⟲ Recalibrate hand origin** with palm at 50 cm, centered
- Tune gains on the **🎛️ Tuning** tab; the **🐢 Safe start** preset is the only valid starting point on real hardware

## Protocol

1. Calibration: paume face caméra, 50 cm, centered → Recalibrate
2. Verify: move slowly in Y → SIM avg RMS card on Home should stay < 5°
3. Real robot only after sim shows ✓ OK for 30 s continuous on all driven joints

**Docs:** [`docs/TELEOPERATION.md`](../../docs/TELEOPERATION.md), [`docs/TELEOP_DASHBOARD.md`](../../docs/TELEOP_DASHBOARD.md), [`docs/REAL_ROBOT_TEST_PROCEDURE.md`](../../docs/REAL_ROBOT_TEST_PROCEDURE.md)
