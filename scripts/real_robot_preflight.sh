#!/usr/bin/env bash
# ============================================================================
# Real MyCobot 320 Pi — preflight check before a live teleop session
#
# Validates end-to-end connectivity: ping → TCP port → bridge_tour node →
# round-trip ping/pong → get_angles → optional go_home. Run this BEFORE
# starting the full teleop stack on the live robot to avoid surprises.
#
# Usage:
#   bash scripts/real_robot_preflight.sh                       # 10.10.0.225
#   bash scripts/real_robot_preflight.sh 192.168.1.50          # custom IP
#   GO_HOME=1 bash scripts/real_robot_preflight.sh             # also go_home
#
# Exit codes:
#   0  — all checks passed, robot ready for teleop
#   1  — network not reachable
#   2  — TCP port closed (bridge_pi_simple.py not running on Pi?)
#   3  — ROS2 stack not sourced
#   4  — bridge_tour round-trip failed
# ============================================================================

set -u
PI_IP="${1:-10.10.0.225}"
PI_PORT=5005
GO_HOME="${GO_HOME:-0}"

C_RED='\033[31m'; C_GREEN='\033[32m'; C_YELLOW='\033[33m'; C_RESET='\033[0m'

ok()   { echo -e " ${C_GREEN}✓${C_RESET} $1"; }
warn() { echo -e " ${C_YELLOW}△${C_RESET} $1"; }
fail() { echo -e " ${C_RED}✗${C_RESET} $1"; exit "$2"; }

echo "=========================================================="
echo "  MyCobot real-robot preflight — Pi at ${PI_IP}:${PI_PORT}"
echo "=========================================================="

# -- 1. Network reachability ------------------------------------------------
echo
echo "[1/5] Pinging Pi..."
if ping -c 2 -W 2 "$PI_IP" > /dev/null 2>&1; then
    ok "Pi is reachable at ${PI_IP}"
else
    fail "Pi at ${PI_IP} not responding to ICMP. Check cable / IP / power." 1
fi

# -- 2. TCP port --------------------------------------------------------------
echo
echo "[2/5] Checking TCP port ${PI_PORT} (bridge_pi_simple.py)..."
if nc -zv -w 3 "$PI_IP" "$PI_PORT" > /dev/null 2>&1; then
    ok "Port ${PI_PORT} open on Pi"
else
    fail "Port ${PI_PORT} closed — start bridge_pi_simple.py on the Pi:
       ssh er@${PI_IP}
       python3 bridge_pi_simple.py" 2
fi

# -- 3. ROS2 sourced ----------------------------------------------------------
echo
echo "[3/5] Verifying ROS2 environment..."
if ! command -v ros2 > /dev/null 2>&1; then
    fail "'ros2' not found in PATH. Source /opt/ros/jazzy/setup.bash first." 3
fi
if [[ -z "${AMENT_PREFIX_PATH:-}" ]]; then
    warn "AMENT_PREFIX_PATH is empty — are you sure install/setup.bash is sourced?"
fi
ok "ROS2 Jazzy available"

# -- 4. bridge_tour round-trip ping/pong --------------------------------------
echo
echo "[4/5] Spinning up a temporary bridge_tour and testing round-trip..."
LOG_FILE=$(mktemp /tmp/preflight_bridge_XXXX.log)

# Start bridge_tour in background with the target IP
ros2 run mycobot_gateway bridge_tour --ros-args -p "pi_ip:=${PI_IP}" > "$LOG_FILE" 2>&1 &
BRIDGE_PID=$!
trap 'kill -TERM $BRIDGE_PID 2>/dev/null || true' EXIT
sleep 3

if ! ps -p "$BRIDGE_PID" > /dev/null 2>&1; then
    cat "$LOG_FILE"
    fail "bridge_tour crashed at startup" 4
fi

if ! grep -q "Connecté" "$LOG_FILE"; then
    tail -20 "$LOG_FILE"
    fail "bridge_tour couldn't connect to the Pi (see log above)" 4
fi
ok "bridge_tour connected to Pi"

# Round-trip: publish ping on /to_robot, listen for reply on /from_robot
echo "  Sending ping command..."
ros2 topic pub --once /to_robot std_msgs/msg/String "data: 'ping'" > /dev/null 2>&1 &
PUB_PID=$!
REPLY=$(timeout 3 ros2 topic echo --once /from_robot std_msgs/msg/String 2>/dev/null || true)
wait "$PUB_PID" 2>/dev/null || true

if echo "$REPLY" | grep -q "pong"; then
    ok "Ping/pong successful — TCP bidir OK"
else
    warn "No pong received; check bridge_pi_simple.py responds to 'ping'"
    echo "    raw reply: $REPLY"
fi

# -- 5. get_angles (verify servo power + communication) -----------------------
echo
echo "[5/5] Reading current joint angles (validates servo bus)..."
ros2 topic pub --once /to_robot std_msgs/msg/String "data: 'get_angles'" > /dev/null 2>&1 &
PUB_PID=$!
ANGLES=$(timeout 3 ros2 topic echo --once /from_robot std_msgs/msg/String 2>/dev/null || true)
wait "$PUB_PID" 2>/dev/null || true

if echo "$ANGLES" | grep -qE "ANGLES|angles"; then
    ok "Servo bus responding"
    echo "    $ANGLES" | grep -E "data:" | head -1
else
    warn "Couldn't read angles — servos powered? tty permissions OK on the Pi?"
fi

# -- Optional: go_home --------------------------------------------------------
if [[ "$GO_HOME" == "1" ]]; then
    echo
    echo "[BONUS] Sending go_home (robot will move!)..."
    read -p "  Is the workspace clear around the robot? [y/N] " -n 1 -r
    echo
    if [[ "$REPLY" =~ ^[Yy]$ ]]; then
        ros2 topic pub --once /to_robot std_msgs/msg/String "data: 'go_home'" > /dev/null 2>&1 &
        sleep 4
        ok "go_home issued — verify the arm reached its home position"
    else
        warn "go_home skipped (workspace not confirmed clear)"
    fi
fi

echo
echo "=========================================================="
ok "Preflight PASSED — robot ready for teleop"
echo "=========================================================="
echo
echo "Next steps:"
echo "  1. Launch the teleop stack with target:=real (or both):"
echo "     ros2 launch mycobot_gateway mycobot_teleop.launch.py \\"
echo "       target:=real pi_ip:=${PI_IP}"
echo "  2. Start rosbridge in T1 if you intend to use the teleop script"
echo "  3. Source hand-teleop env and run teleop with --no-gripper"
echo "     python3 teleop/mycobot_teleop.py --camera astra --ros \\"
echo "       --use-rosbridge --no-gripper"
echo
