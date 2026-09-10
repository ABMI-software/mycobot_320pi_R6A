import sys
import unittest
from pathlib import Path


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
sys.path.insert(0, str(SCRIPTS))

import pick_and_place_vision as control  # noqa: E402


class FakeBridge:
    def __init__(self, response):
        self.response = response
        self.commands = []

    def send(self, command):
        self.commands.append(command)
        return self.response

    def grip(self, command):
        self.commands.append(command)
        return self.response


class MotionSafetyTests(unittest.TestCase):
    def test_cartesian_error_aborts_instead_of_continuing(self):
        bridge = FakeBridge("ERROR: Has invalid coord value")
        with self.assertRaisesRegex(RuntimeError, "commande cartésienne refusée"):
            control.move(bridge, [351, 0, 150, 0, 0, 0], 8, 1, False, settle=0)

    def test_cartesian_ok_is_accepted(self):
        bridge = FakeBridge("OK: coords envoyées [200, 150, 180]")
        control.move(bridge, [200, 150, 180, 0, 0, 0], 8, 1, False, settle=0)
        self.assertEqual(len(bridge.commands), 1)

    def test_gripper_error_aborts(self):
        bridge = FakeBridge("ERROR: gripper unavailable")
        with self.assertRaisesRegex(RuntimeError, "commande pince refusée"):
            control.grip(bridge, 20, False)


if __name__ == "__main__":
    unittest.main()
