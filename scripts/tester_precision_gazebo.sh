#!/usr/bin/env bash
# Reconstructed automatic campaign in a separate ROS/Gazebo session.
set -e
TASK_REPO=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
TASK_BUILD="${TMPDIR:-/tmp}/mycobot_trajectory_build"
TASK_HEADLESS=false
TASK_PROTOCOL=()
TASK_CONTROL=position
TASK_CONFIG="$TASK_REPO/mycobot_description/config/controller.yaml"
TASK_ARM_MASS=3.0
TASK_GRIPPER_MASS=0.34
while [[ $# -gt 0 ]]; do
    case "$1" in
        --headless) TASK_HEADLESS=true; shift ;;
        --vertical) TASK_PROTOCOL=(--vertical); shift ;;
        --dynamic) TASK_CONTROL=effort; TASK_CONFIG="$TASK_REPO/mycobot_description/config/controller_effort.yaml"; shift ;;
        --arm-mass) TASK_ARM_MASS="${2:?Valeur en kg manquante}"; shift 2 ;;
        --gripper-mass) TASK_GRIPPER_MASS="${2:?Valeur en kg manquante}"; shift 2 ;;
        *) printf 'Argument inconnu : %s\n' "$1" >&2; exit 2 ;;
    esac
done
TASK_OUTPUT="$TASK_REPO/docs/precision_gazebo_results/$(date +%Y-%m-%d_%H-%M-%S)_$$"
export PATH="/usr/bin:/bin:$PATH"
unset PYTHONHOME
source /opt/ros/jazzy/setup.bash
cd "$TASK_REPO"
colcon --log-base "$TASK_BUILD/log" build \
    --base-paths mycobot_description mycobot_gateway \
    --build-base "$TASK_BUILD/build" --install-base "$TASK_BUILD/install" \
    --symlink-install --cmake-args -DBUILD_TESTING=OFF -DPython3_EXECUTABLE=/usr/bin/python3
source "$TASK_BUILD/install/setup.bash"
# Le domaine derive du PID : deux campagnes dont les PID sont congrus modulo 100
# atterrissent sur le meme domaine, et le Gazebo de la precedente, reste vivant,
# repond aux ordres de la suivante (« more than one action server »). Un domaine
# pose dans l'environnement a donc la priorite.
export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-$((100 + $$ % 100))}
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
export GZ_PARTITION="mycobot_precision_$$"
export ROS_LOG_DIR="$TASK_OUTPUT/ros_logs"
/usr/bin/python3 scripts/gazebo_precision_campaign.py prepare --output "$TASK_OUTPUT" \
    --arm-mass "$TASK_ARM_MASS" --mass "$TASK_GRIPPER_MASS" "${TASK_PROTOCOL[@]}" --control-mode "$TASK_CONTROL"
printf 'Domaine ROS %s ; resultats : %s\n' "$ROS_DOMAIN_ID" "$TASK_OUTPUT"
ros2 launch mycobot_gateway sim_grasp.launch.py \
    world_name:=real_table world_file:="$TASK_OUTPUT/world.sdf" \
    gui_config:="$TASK_OUTPUT/gui.config" enable_cameras:=false \
    arm_mass_kg:="$TASK_ARM_MASS" gripper_mass_kg:="$TASK_GRIPPER_MASS" \
    arm_control_mode:="$TASK_CONTROL" controller_config:="$TASK_CONFIG" \
    headless:="$TASK_HEADLESS" >"$TASK_OUTPUT/gazebo.log" 2>&1 &
TASK_SIM_PID=$!
cleanup() { kill -INT "$TASK_SIM_PID" 2>/dev/null || true; wait "$TASK_SIM_PID" 2>/dev/null || true; }
trap cleanup EXIT
trap 'exit 130' INT TERM
/usr/bin/python3 scripts/gazebo_precision_campaign.py run --output "$TASK_OUTPUT" 2>&1 | tee "$TASK_OUTPUT/campaign.log"
TASK_STATUS=${PIPESTATUS[0]}
if [[ "$TASK_STATUS" != 0 ]]; then exit "$TASK_STATUS"; fi
printf '\nRapport : %s/rapport.md\n' "$TASK_OUTPUT"
if [[ "$TASK_HEADLESS" == false ]]; then
    printf 'Termine. Fermer Gazebo ou Ctrl+C pour quitter.\n'
    wait "$TASK_SIM_PID"
fi
