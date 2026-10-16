#!/usr/bin/env python3
"""The scripted sorter, each piece started from a random arm pose (random_start.py).

  random_start_sorter.py --ros-args -p random_start_seed:=51 -p random_start_log:=<file> \
                         [the sim_sorting_grasp parameters]

sim_sorting_grasp unchanged, same node name (record_tri_sort.py waits for it), with one step
added in target_poses: after the piece and bin positions are read at the observation pose, the
arm moves to a pose drawn with numpy default_rng(random_start_seed), draw after draw. Then
sort_one starts, and with it the recorded episode ("▶ <piece> : saisie"). One JSON line per
draw in random_start_log: seed, draw index, piece, attempt, pose, tip, and the largest
displacement of any piece during the move (Gazebo poses before and after), so a start that
touched the board is visible. A pose the transit cannot reach safely is logged and redrawn.

System python with ROS sourced, inside gazebo_to_lerobot.
"""
import json
import sys
from collections import Counter
from pathlib import Path

import numpy as np
import rclpy

from mycobot_gateway.sim_sorting_grasp import SimSortingGrasp, tool_tip

sys.path.insert(0, str(Path(__file__).resolve().parent))
import random_start  # noqa: E402

MAX_REDRAWS = 5


class RandomStartSorter(SimSortingGrasp):
    def __init__(self):
        super().__init__()
        self.declare_parameter('random_start_seed', 0)
        self.declare_parameter('random_start_log', '')
        self.start_seed = int(self.get_parameter('random_start_seed').value)
        self.start_log = Path(str(self.get_parameter('random_start_log').value))
        self.rng = np.random.default_rng(self.start_seed)
        self.draws = 0
        self.attempts = Counter()

    def target_poses(self, target=None):
        poses = super().target_poses(target)
        self.attempts[target.model] += 1
        for _ in range(MAX_REDRAWS):
            q = random_start.sample(self.rng)
            before = self.object_poses()
            reached = self.transit(q, 'pose de depart aleatoire')
            after = self.object_poses()
            moved = max((float(np.linalg.norm(after[n] - p)) for n, p in before.items() if n in after),
                        default=float('nan'))
            entry = {'seed': self.start_seed, 'rng': 'numpy default_rng(seed)', 'draw': self.draws,
                     'piece': target.model, 'attempt': self.attempts[target.model],
                     'q_deg': np.round(q, 3).tolist(), 'tip': np.round(tool_tip(q), 4).tolist(),
                     'reached': reached, 'pieces_moved_max_mm': round(moved * 1000, 2),
                     'jitter': random_start.JITTER, 'reference': random_start.REFERENCE.name}
            self.draws += 1
            with open(self.start_log, 'a') as f:
                f.write(json.dumps(entry) + '\n')
            self.status(f'  depart aleatoire {entry["draw"]} : {entry["q_deg"]} '
                        f'(atteint={reached}, pieces deplacees {entry["pieces_moved_max_mm"]} mm)')
            if reached:
                return poses
        raise RuntimeError(f'aucune pose de depart aleatoire atteignable en {MAX_REDRAWS} tirages')


def main(args=None):
    rclpy.init(args=args)
    node = RandomStartSorter()
    try:
        results = node.run()
    finally:
        node.destroy_node()
        rclpy.shutdown()
    for model, verdict in results.items():
        print(f'  {"✔" if verdict.startswith("OK") else "✘"} {model:16} {verdict}')
    return 0 if results and all(v.startswith('OK') for v in results.values()) else 1


if __name__ == '__main__':
    sys.exit(main())
