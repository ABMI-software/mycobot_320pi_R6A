"""Tests for mycobot_teleop.py pure helpers and data-transform logic.

Stubs out hand_teleop, cv2, rclpy, and roslibpy so these run without
the hand-teleop conda env or a live ROS graph.

Run with:
    cd teleop && python -m pytest test_mycobot_teleop.py -v
"""
from __future__ import annotations

import sys
import types
from pathlib import Path
from unittest.mock import MagicMock

import numpy as np
import pytest

# ---------------------------------------------------------------------------
# Import stubs — must be in sys.modules before mycobot_teleop is imported
# ---------------------------------------------------------------------------

def _make_stub(name: str) -> types.ModuleType:
    m = types.ModuleType(name)
    sys.modules[name] = m
    return m


# hand_teleop
_ht = _make_stub("hand_teleop")
_ht_gp = _make_stub("hand_teleop.gripper_pose")
_ht_gp_gp = _make_stub("hand_teleop.gripper_pose.gripper_pose")
_ht_hp = _make_stub("hand_teleop.hand_pose")
_ht_hp_f = _make_stub("hand_teleop.hand_pose.factory")
_ht_tr = _make_stub("hand_teleop.tracking")
_ht_tr_t = _make_stub("hand_teleop.tracking.tracker")


class _GripperPose:
    @staticmethod
    def zero():
        p = _GripperPose()
        p.pos = np.zeros(3)
        p.rot = np.eye(3)
        return p


_ht_gp_gp.GripperPose = _GripperPose
_ht_hp_f.ModelName = str
_ht_tr_t.HandTracker = MagicMock()

# cv2
_cv2 = _make_stub("cv2")
_cv2.CAP_V4L2 = 200
_cv2.CAP_ANY = 0
_cv2.CAP_PROP_FRAME_WIDTH = 3
_cv2.CAP_PROP_FRAME_HEIGHT = 4
_cv2.VideoCapture = MagicMock()
_cv2.imencode = MagicMock(return_value=(True, b"fake"))
_cv2.resize = MagicMock(side_effect=lambda img, sz: img)

# rclpy / roslibpy (only touched if publisher constructors are called)
_make_stub("rclpy")
_make_stub("rclpy.node")
_make_stub("trajectory_msgs")
_make_stub("trajectory_msgs.msg")
_make_stub("builtin_interfaces")
_make_stub("builtin_interfaces.msg")
_make_stub("roslibpy")

# Ensure the teleop directory is on sys.path so we can import the module
sys.path.insert(0, str(Path(__file__).parent))

import mycobot_teleop as mt  # noqa: E402


# ---------------------------------------------------------------------------
# Tests — clamp
# ---------------------------------------------------------------------------

class TestClamp:
    def test_below_lo(self):
        assert mt.clamp(-5.0, 0.0, 10.0) == 0.0

    def test_above_hi(self):
        assert mt.clamp(15.0, 0.0, 10.0) == 10.0

    def test_at_lo(self):
        assert mt.clamp(0.0, 0.0, 10.0) == 0.0

    def test_at_hi(self):
        assert mt.clamp(10.0, 0.0, 10.0) == 10.0

    def test_inside(self):
        assert mt.clamp(5.5, 0.0, 10.0) == 5.5

    def test_returns_float(self):
        assert isinstance(mt.clamp(3, 0, 10), float)

    def test_negative_range(self):
        assert mt.clamp(-3.0, -5.0, -1.0) == -3.0


# ---------------------------------------------------------------------------
# Tests — map_range
# ---------------------------------------------------------------------------

class TestMapRange:
    def test_identity(self):
        assert mt.map_range(0.5, (0.0, 1.0), (0.0, 1.0)) == pytest.approx(0.5)

    def test_full_scale(self):
        assert mt.map_range(1.0, (0.0, 1.0), (0.0, 100.0)) == pytest.approx(100.0)

    def test_inversion(self):
        # src (0,1) → dst (1,0): v=0.25 → t=0.25 → 1 + 0.25*(0-1) = 0.75
        assert mt.map_range(0.25, (0.0, 1.0), (1.0, 0.0)) == pytest.approx(0.75)

    def test_clamp_below(self):
        assert mt.map_range(-1.0, (0.0, 1.0), (10.0, 20.0)) == pytest.approx(10.0)

    def test_clamp_above(self):
        assert mt.map_range(2.0, (0.0, 1.0), (10.0, 20.0)) == pytest.approx(20.0)

    def test_degenerate_src_returns_d0(self):
        # s0 == s1: avoid division by zero, return d0
        assert mt.map_range(0.5, (1.0, 1.0), (5.0, 10.0)) == pytest.approx(5.0)

    def test_midpoint(self):
        assert mt.map_range(5.0, (0.0, 10.0), (-1.0, 1.0)) == pytest.approx(0.0)

    def test_zero_origin(self):
        assert mt.map_range(0.0, (0.0, 10.0), (0.0, 100.0)) == pytest.approx(0.0)


# ---------------------------------------------------------------------------
# Tests — maybe_reverse
# ---------------------------------------------------------------------------

class TestMaybeReverse:
    def test_no_invert(self):
        assert mt.maybe_reverse((1.0, 2.0), False) == (1.0, 2.0)

    def test_invert(self):
        assert mt.maybe_reverse((1.0, 2.0), True) == (2.0, 1.0)

    def test_symmetric_range(self):
        assert mt.maybe_reverse((-5.0, 5.0), True) == (5.0, -5.0)


# ---------------------------------------------------------------------------
# Tests — xyz_to_joints_deg
# ---------------------------------------------------------------------------

class TestXyzToJointsDeg:
    def test_zero_input_gives_zero_joints(self):
        q = mt.xyz_to_joints_deg(np.zeros(3))
        np.testing.assert_array_equal(q, np.zeros(6))

    def test_y_drives_j1(self):
        # dy=0.1 m, y_gain=1.0 → j1 = 0.1 * 150 * 1.0 = 15°
        q = mt.xyz_to_joints_deg(np.array([0.0, 0.1, 0.0]),
                                  y_gain=1.0, x_gain=0.0, z_gain=0.0)
        assert q[0] == pytest.approx(15.0)

    def test_z_drives_j2(self):
        # dz=0.1 m, z_gain=1.0 → j2 = 0.1 * 150 = 15°
        q = mt.xyz_to_joints_deg(np.array([0.0, 0.0, 0.1]),
                                  z_gain=1.0, x_gain=0.0, y_gain=0.0)
        assert q[1] == pytest.approx(15.0)

    def test_x_drives_j3_with_invert(self):
        # invert_x=True: dx=0.1 → dx=-0.1 → j3 = -0.1 * 150 = -15°
        q = mt.xyz_to_joints_deg(np.array([0.1, 0.0, 0.0]),
                                  invert_x=True, x_gain=1.0,
                                  y_gain=0.0, z_gain=0.0)
        assert q[2] == pytest.approx(-15.0)

    def test_x_drives_j3_no_invert(self):
        q = mt.xyz_to_joints_deg(np.array([0.1, 0.0, 0.0]),
                                  invert_x=False, x_gain=1.0,
                                  y_gain=0.0, z_gain=0.0)
        assert q[2] == pytest.approx(15.0)

    def test_gains_scale_linearly(self):
        q1 = mt.xyz_to_joints_deg(np.array([0.0, 0.1, 0.0]), y_gain=1.0,
                                   x_gain=0.0, z_gain=0.0)
        q2 = mt.xyz_to_joints_deg(np.array([0.0, 0.1, 0.0]), y_gain=2.0,
                                   x_gain=0.0, z_gain=0.0)
        assert q2[0] == pytest.approx(2.0 * q1[0])

    def test_joint_limits_clamp_j1_positive(self):
        # 10 m lateral → 1500°, but J1 limit is +168°
        q = mt.xyz_to_joints_deg(np.array([0.0, 10.0, 0.0]), y_gain=1.0)
        assert q[0] == pytest.approx(168.0)

    def test_joint_limits_clamp_j1_negative(self):
        q = mt.xyz_to_joints_deg(np.array([0.0, -10.0, 0.0]), y_gain=1.0)
        assert q[0] == pytest.approx(-168.0)

    def test_rpy_none_j4_j5_j6_are_zero(self):
        q = mt.xyz_to_joints_deg(np.zeros(3), rpy_deg=None)
        assert q[3] == pytest.approx(0.0)
        assert q[4] == pytest.approx(0.0)
        assert q[5] == pytest.approx(0.0)

    def test_rpy_pitch_drives_j4_and_j5(self):
        # pitch=10°, pitch_gain=0.4 → j4 = j5 = 10 * 0.4 * 0.5 = 2°
        q = mt.xyz_to_joints_deg(np.zeros(3), rpy_deg=(0.0, 10.0, 0.0),
                                  pitch_gain=0.4)
        assert q[3] == pytest.approx(2.0)
        assert q[4] == pytest.approx(2.0)

    def test_rpy_yaw_drives_j6(self):
        # yaw=10°, roll_gain=0.4 → j6 = 10 * 0.4 = 4°
        q = mt.xyz_to_joints_deg(np.zeros(3), rpy_deg=(0.0, 0.0, 10.0),
                                  roll_gain=0.4)
        assert q[5] == pytest.approx(4.0)

    def test_invert_y_flips_j1(self):
        q_fwd = mt.xyz_to_joints_deg(np.array([0.0, 0.1, 0.0]), invert_y=False)
        q_inv = mt.xyz_to_joints_deg(np.array([0.0, 0.1, 0.0]), invert_y=True)
        assert q_inv[0] == pytest.approx(-q_fwd[0])

    def test_invert_z_flips_j2(self):
        q_fwd = mt.xyz_to_joints_deg(np.array([0.0, 0.0, 0.1]), invert_z=False)
        q_inv = mt.xyz_to_joints_deg(np.array([0.0, 0.0, 0.1]), invert_z=True)
        assert q_inv[1] == pytest.approx(-q_fwd[1])

    def test_invert_pitch_flips_j4_j5(self):
        q_fwd = mt.xyz_to_joints_deg(np.zeros(3), rpy_deg=(0.0, 10.0, 0.0),
                                      invert_pitch=False)
        q_inv = mt.xyz_to_joints_deg(np.zeros(3), rpy_deg=(0.0, 10.0, 0.0),
                                      invert_pitch=True)
        assert q_inv[3] == pytest.approx(-q_fwd[3])
        assert q_inv[4] == pytest.approx(-q_fwd[4])

    def test_returns_6_joints(self):
        q = mt.xyz_to_joints_deg(np.array([0.05, 0.05, 0.05]))
        assert len(q) == 6

    def test_output_within_limits(self):
        rng = np.random.default_rng(42)
        for _ in range(50):
            xyz = rng.uniform(-0.5, 0.5, 3)
            q = mt.xyz_to_joints_deg(xyz, x_gain=2.0, y_gain=2.0, z_gain=2.0)
            for i, name in enumerate(mt.DEFAULT_JOINT_NAMES):
                lo, hi = mt.JOINT_LIMITS_DEG[name]
                assert lo <= q[i] <= hi, f"Joint {name} out of limits: {q[i]}"


# ---------------------------------------------------------------------------
# Tests — get_xyz_from_pose / get_rot_from_pose
# ---------------------------------------------------------------------------

class TestGetXyzFromPose:
    def test_pos_attribute(self):
        pose = MagicMock(spec=["pos"])
        pose.pos = [0.1, 0.2, 0.3]
        xyz = mt.get_xyz_from_pose(pose)
        np.testing.assert_allclose(xyz, [0.1, 0.2, 0.3])

    def test_position_fallback(self):
        # spec excludes 'pos' → hasattr returns False → uses 'position'
        pose = MagicMock(spec=["position"])
        pose.position = [0.4, 0.5, 0.6]
        xyz = mt.get_xyz_from_pose(pose)
        np.testing.assert_allclose(xyz, [0.4, 0.5, 0.6])

    def test_returns_float_array(self):
        pose = MagicMock(spec=["pos"])
        pose.pos = [1, 2, 3]
        xyz = mt.get_xyz_from_pose(pose)
        assert xyz.dtype == float


class TestGetRotFromPose:
    def test_rot_attribute(self):
        pose = MagicMock(spec=["rot"])
        pose.rot = np.eye(3).tolist()
        rot = mt.get_rot_from_pose(pose)
        np.testing.assert_allclose(rot, np.eye(3))

    def test_rotation_fallback(self):
        pose = MagicMock(spec=["rotation"])
        pose.rotation = np.eye(3).tolist()
        rot = mt.get_rot_from_pose(pose)
        np.testing.assert_allclose(rot, np.eye(3))


# ---------------------------------------------------------------------------
# Tests — RosBridgeArmPublisher (no live connection)
# ---------------------------------------------------------------------------

def _make_bridge_pub(tfs: float = 0.25) -> mt.RosBridgeArmPublisher:
    """Create a RosBridgeArmPublisher bypassing __init__ (no rosbridge needed)."""
    pub = object.__new__(mt.RosBridgeArmPublisher)
    sec = int(tfs)
    nsec = int((tfs - sec) * 1e9)
    pub.tfs = {"sec": sec, "nanosec": nsec}
    pub.joint_names = mt.DEFAULT_JOINT_NAMES
    pub.topic = MagicMock()
    pub.hand_xyz_topic = MagicMock()
    pub.gripper_topic = MagicMock()
    pub.camera_topic = MagicMock()
    pub._gain_callback = None
    pub._tfs_callback = None
    pub._recalibrate_callback = None
    pub._last_camera_pub_t = 0.0
    return pub


class TestRosBridgeArmPublisherSetTfs:
    def test_subsecond(self):
        pub = _make_bridge_pub()
        pub.set_tfs(0.25)
        assert pub.tfs == {"sec": 0, "nanosec": 250_000_000}

    def test_whole_second(self):
        pub = _make_bridge_pub()
        pub.set_tfs(1.0)
        assert pub.tfs == {"sec": 1, "nanosec": 0}

    def test_fractional(self):
        pub = _make_bridge_pub()
        pub.set_tfs(1.5)
        assert pub.tfs["sec"] == 1
        assert pub.tfs["nanosec"] == pytest.approx(500_000_000, abs=1)

    def test_small_value(self):
        pub = _make_bridge_pub()
        pub.set_tfs(0.3)
        assert pub.tfs["sec"] == 0
        assert pub.tfs["nanosec"] == pytest.approx(300_000_000, abs=1)


class TestRosBridgeArmPublisherGainMessage:
    def test_calls_callback_with_4_args(self):
        pub = _make_bridge_pub()
        received: dict = {}
        pub._gain_callback = lambda gx, gy, gz, tfs: received.update(
            {"x": gx, "y": gy, "z": gz, "tfs": tfs}
        )
        pub._on_gain_message({"data": [1.1, 1.2, 1.6, 0.30]})
        assert received == {"x": 1.1, "y": 1.2, "z": 1.6, "tfs": 0.30}

    def test_ignores_short_data(self):
        pub = _make_bridge_pub()
        called = [False]
        pub._gain_callback = lambda *a: called.__setitem__(0, True)
        pub._on_gain_message({"data": [1.0, 1.0]})
        assert not called[0]

    def test_no_callback_does_not_raise(self):
        pub = _make_bridge_pub()
        pub._on_gain_message({"data": [1.0, 1.0, 1.0]})

    def test_tfs_is_none_when_only_3_elements(self):
        pub = _make_bridge_pub()
        received: dict = {}
        pub._gain_callback = lambda gx, gy, gz, tfs: received.update({"tfs": tfs})
        pub._on_gain_message({"data": [1.0, 1.0, 1.0]})
        assert received["tfs"] is None

    def test_updates_tfs_on_4th_element(self):
        pub = _make_bridge_pub(tfs=0.25)
        pub._gain_callback = lambda *a: None
        pub._on_gain_message({"data": [1.0, 1.0, 1.0, 0.5]})
        assert pub.tfs == {"sec": 0, "nanosec": 500_000_000}

    def test_missing_data_key_does_not_raise(self):
        pub = _make_bridge_pub()
        pub._gain_callback = lambda *a: None
        pub._on_gain_message({})


class TestRosBridgeArmPublisherGripper:
    def test_fully_open(self):
        pub = _make_bridge_pub()
        pub.send_gripper_normalized(1.0)
        data = pub.gripper_topic.publish.call_args[0][0]["data"]
        assert data == pytest.approx([0.0, 0.0, 0.0, 0.0])

    def test_fully_closed(self):
        pub = _make_bridge_pub()
        pub.send_gripper_normalized(0.0)
        data = pub.gripper_topic.publish.call_args[0][0]["data"]
        assert data == pytest.approx([-0.7, 0.7, 0.7, -0.7])

    def test_half_open(self):
        pub = _make_bridge_pub()
        pub.send_gripper_normalized(0.5)
        data = pub.gripper_topic.publish.call_args[0][0]["data"]
        # servo = -0.7 * 0.5 = -0.35
        assert data == pytest.approx([-0.35, 0.35, 0.35, -0.35])

    def test_clamp_above_one(self):
        pub = _make_bridge_pub()
        pub.send_gripper_normalized(2.0)
        data = pub.gripper_topic.publish.call_args[0][0]["data"]
        assert data == pytest.approx([0.0, 0.0, 0.0, 0.0])

    def test_clamp_below_zero(self):
        pub = _make_bridge_pub()
        pub.send_gripper_normalized(-1.0)
        data = pub.gripper_topic.publish.call_args[0][0]["data"]
        assert data == pytest.approx([-0.7, 0.7, 0.7, -0.7])

    def test_symmetric_fingers(self):
        pub = _make_bridge_pub()
        pub.send_gripper_normalized(0.3)
        data = pub.gripper_topic.publish.call_args[0][0]["data"]
        # data = [servo, -servo, -servo, servo]
        assert data[0] == pytest.approx(-data[1])
        assert data[2] == pytest.approx(-data[3])
        assert data[0] == pytest.approx(data[3])


class TestRosBridgeArmPublisherSendDeg:
    def test_converts_degrees_to_radians(self):
        pub = _make_bridge_pub()
        pub.send_deg(np.array([90.0, 0.0, 0.0, 0.0, 0.0, 0.0]))
        msg = pub.topic.publish.call_args[0][0]
        assert msg["points"][0]["positions"][0] == pytest.approx(np.radians(90.0))

    def test_joint_names_in_message(self):
        pub = _make_bridge_pub()
        pub.send_deg(np.zeros(6))
        msg = pub.topic.publish.call_args[0][0]
        assert msg["joint_names"] == mt.DEFAULT_JOINT_NAMES

    def test_tfs_in_message(self):
        pub = _make_bridge_pub(tfs=0.25)
        pub.send_deg(np.zeros(6))
        msg = pub.topic.publish.call_args[0][0]
        assert msg["points"][0]["time_from_start"] == {"sec": 0, "nanosec": 250_000_000}

    def test_all_six_positions_published(self):
        pub = _make_bridge_pub()
        pub.send_deg(np.array([10.0, 20.0, 30.0, 40.0, 50.0, 60.0]))
        msg = pub.topic.publish.call_args[0][0]
        positions = msg["points"][0]["positions"]
        expected = np.radians([10.0, 20.0, 30.0, 40.0, 50.0, 60.0]).tolist()
        assert positions == pytest.approx(expected)


class TestRosBridgeArmPublisherRecalibrate:
    def test_calls_callback(self):
        pub = _make_bridge_pub()
        called = [False]
        pub._recalibrate_callback = lambda: called.__setitem__(0, True)
        pub._on_recalibrate_message({})
        assert called[0]

    def test_no_callback_does_not_raise(self):
        pub = _make_bridge_pub()
        pub._on_recalibrate_message({})


# ---------------------------------------------------------------------------
# Tests — constants
# ---------------------------------------------------------------------------

class TestConstants:
    def test_six_joints_defined(self):
        assert len(mt.JOINT_LIMITS_DEG) == 6

    def test_default_joint_names_count(self):
        assert len(mt.DEFAULT_JOINT_NAMES) == 6

    def test_joint_names_match_limits_keys(self):
        assert set(mt.DEFAULT_JOINT_NAMES) == set(mt.JOINT_LIMITS_DEG.keys())

    def test_j1_limits(self):
        lo, hi = mt.JOINT_LIMITS_DEG["joint2_to_joint1"]
        assert lo == -168.0 and hi == 168.0

    def test_j6_limits(self):
        lo, hi = mt.JOINT_LIMITS_DEG["joint6output_to_joint6"]
        assert lo == -180.0 and hi == 180.0

    def test_all_limits_symmetric(self):
        # Official elephantrobotics spec uses symmetric limits for all joints
        for name, (lo, hi) in mt.JOINT_LIMITS_DEG.items():
            assert lo == pytest.approx(-hi), f"{name}: expected symmetric limits"

    def test_safe_range_has_xyz_and_gripper(self):
        assert set(mt.SAFE_RANGE.keys()) >= {"x", "y", "z", "g"}

    def test_safe_range_xyz_ordered(self):
        for axis in ("x", "y", "z"):
            lo, hi = mt.SAFE_RANGE[axis]
            assert lo < hi, f"SAFE_RANGE[{axis!r}] not ordered: lo={lo} hi={hi}"

    def test_base_scale_positive(self):
        assert mt.BASE_SCALE_DEG_PER_M > 0

    def test_ema_alpha_in_unit_interval(self):
        assert 0.0 < mt.DEFAULT_COMMAND_EMA_ALPHA < 1.0
