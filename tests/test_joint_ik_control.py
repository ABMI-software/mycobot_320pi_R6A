import json
import sys
import unittest
from pathlib import Path

import numpy as np


SCRIPTS = Path(__file__).resolve().parents[1] / 'scripts'
sys.path.insert(0, str(SCRIPTS))

import joint_ik_control as joint  # noqa: E402
from diff_ik import fk_pose  # noqa: E402


MANUAL_Q = np.array([41.9, -128.7, 76.5, -53.4, -3.7, 21.8])
ROBOT_XYZ = np.array([320.9, 80.3, 131.0])


class FakeBridge:
    def __init__(self, angles=MANUAL_Q, xyz=ROBOT_XYZ):
        self.angles = np.asarray(angles, dtype=float)
        self.xyz = np.asarray(xyz, dtype=float)
        self.commands = []

    def send(self, command):
        self.commands.append(command)
        if command['action'] == 'get_angles':
            return 'ANGLES: ' + json.dumps(self.angles.tolist())
        if command['action'] == 'get_coords':
            return 'COORDS: ' + json.dumps(self.xyz.tolist() + [0, 0, 0])
        if command['action'] == 'send_angles':
            self.angles = np.asarray(command['angles'])
            return 'OK: angles envoyés'
        raise AssertionError(command)


class JointIKTests(unittest.TestCase):
    def test_manual_pose_is_recovered_and_preferred(self):
        model_xyz, rotation = fk_pose(MANUAL_Q)
        bias = ROBOT_XYZ - model_xyz
        solutions = joint.solve_all_ik(
            ROBOT_XYZ, rotation, MANUAL_Q, MANUAL_Q, bias,
            random_restarts=0)
        self.assertTrue(solutions)
        self.assertLess(joint.angular_distance_deg(solutions[0].angles_deg, MANUAL_Q), 0.1)
        self.assertLess(solutions[0].xyz_error_mm, 0.01)

    def test_actual_xyz_error_refuses_descent(self):
        bridge = FakeBridge(xyz=ROBOT_XYZ + [7, 0, 0])
        controller = joint.JointIKController(
            bridge, MANUAL_Q, ROBOT_XYZ, xyz_tolerance_mm=5, settle_s=0)
        controller._expected_xyz = ROBOT_XYZ.copy()
        with self.assertRaisesRegex(joint.MotionSafetyError, 'descente interdite'):
            controller.descend_vertical(ROBOT_XYZ - [0, 0, 2])
        self.assertFalse(any(c['action'] == 'send_angles' for c in bridge.commands))

    def test_only_joint_commands_are_emitted(self):
        controller = joint.JointIKController(
            FakeBridge(), MANUAL_Q, ROBOT_XYZ, dry_run=True, settle_s=0)
        controller.move_pose(ROBOT_XYZ)
        self.assertLess(np.linalg.norm(controller._actual_xyz() - ROBOT_XYZ), 0.01)

    def test_response_parser_rejects_malformed_coords(self):
        bridge = FakeBridge()
        bridge.send = lambda command: 'OK: nothing useful'
        with self.assertRaises(joint.MotionSafetyError):
            joint.read_actual_xyz(bridge)


if __name__ == '__main__':
    unittest.main()
