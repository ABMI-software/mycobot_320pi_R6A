# Contract ground-truth leak check (Part 10.1)

```
$ for c in contracts/mycobot_sorting*.yaml; do
    grep -A20 'observation' "$c" | grep -iE "cube|cylinder|box|bin|model_pose" \
      && echo "FATAL: leak" || echo "observation stream clean"
  done
== contracts/mycobot_sorting.yaml
  observation stream clean
== contracts/mycobot_sorting_heldout.yaml
  observation stream clean
```

Both contracts' `observations.*` blocks contain only camera image topics and
`/joint_states` (arm + gripper positions) — no object or bin pose anywhere.
Per A5: the demonstrator (`run_pick_and_place.py`) is allowed to read
ground-truth object poses from Gazebo to decide where to move; the
**recorded** observation stream must never contain them, or a policy
trained on this data would learn to read a signal it will not have at
inference. Ground truth belongs only in each episode's `*.verdict.json`
metadata (Part 9.4), never in `contracts/*.yaml` or the dataset itself.
