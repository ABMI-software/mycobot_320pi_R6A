"""Invariant I4: Gazebo ground truth never feeds perception or planning.

Only validation code may read /validation/gt/* or a Gazebo pose topic.
sim_sorting_grasp reads dynamic_pose to CHECK a grasp after the fact.
"""

from pathlib import Path
import re
import unittest

ROOT = Path(__file__).resolve().parents[1]
PACKAGE = ROOT / 'mycobot_gateway' / 'mycobot_gateway'
GT = re.compile(r'/validation/gt|pose/info|dynamic_pose|gz_pose_info')
ALLOWED = {
    'gazebo_ground_truth.py',      # the ground-truth publisher itself
    'sim_sorting_grasp.py',        # post-grasp verification, never a target in vision mode
    'yolo_gt_overlay.py',          # draws YOLO vs ground truth in Gazebo, validation only
}


class GroundTruthIsolationTests(unittest.TestCase):
    def test_no_perception_or_planning_module_reads_gazebo_poses(self):
        offenders = [str(f.relative_to(PACKAGE)) for f in PACKAGE.rglob('*.py')
                     if f.name not in ALLOWED and GT.search(f.read_text())]
        self.assertEqual(offenders, [])


if __name__ == '__main__':
    unittest.main()
