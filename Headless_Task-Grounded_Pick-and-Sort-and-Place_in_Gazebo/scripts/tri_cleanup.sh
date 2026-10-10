# Sourced by record_tri_seed.sh, replay_gui.sh and run_sort_gui.sh: on exit, stop
# what they started. `kill 0` also killed the calling shell; killing only the
# script's jobs leaves Gazebo's own sub-processes running, and the next scene
# then starts beside the old one. One simulation per container is the rule here.
tri_cleanup() {
    # the callers run under set -e: a kill or pkill finding nothing must not abort
    # the cleanup at its first line
    set +e
    kill $(jobs -p) 2>/dev/null
    sleep 3
    for p in record_tri_sort sim_sorting_grasp episode_recorder_node commanded_action_relay replay_episode \
             tri_yolo.launch gazebo_ground_truth tri_scene_randomizer parameter_bridge robot_state_publisher; do
        pkill -INT -f "[${p:0:1}]${p:1}" 2>/dev/null
    done
    sleep 5
    # the recorder ignores SIGINT; whatever is left after the grace period is killed
    for p in record_tri_sort sim_sorting_grasp episode_recorder_node commanded_action_relay replay_episode              tri_yolo.launch gazebo_ground_truth tri_scene_randomizer parameter_bridge robot_state_publisher              "gz sim" "ruby.*gz"; do
        pkill -9 -f "[${p:0:1}]${p:1}" 2>/dev/null
    done
    rm -f /dev/shm/fastrtps_* /dev/shm/sem.fastrtps_* 2>/dev/null
    return 0
}
trap tri_cleanup EXIT
