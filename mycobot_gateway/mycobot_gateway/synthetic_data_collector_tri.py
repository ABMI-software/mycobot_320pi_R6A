#!/usr/bin/env python3
"""DREAM training renders in the SORTING scene (tri_yolo), with the gripper.

Why: in the sorting scene DREAM detects 65 % of the keypoints (49.5 % with the
gripper) against 99 % in the world of its 50K renders, and T_DREAM is off by
27-32 deg; the scene, not the cameras, is out of distribution (protocol step 10).

Same cameras (dream50k layout), same labels (joint angles in labels.csv, the
keypoints are recomputed by convert_to_ndds.py from mycobot_fk), only the scene
changes: real_table plate, bins, pieces, ArUco, robot WITH its gripper.

Poses:
  - `pick_fraction` of them are the sorter's own poses: the fingertip over a
    random point of the table, tool down or tilted outward (the tilted drops),
    solved with sim_sorting_grasp's IK — the poses the arm is in when DREAM is
    compared to YOLO;
  - the rest are v4_gripper's uniform poses with the ground clearance lowered
    (`table_clearance`, 0.13 m in v3 rejected every pick pose).
The gripper opening is drawn per pose (the sorter holds objects closed).

    ros2 launch mycobot_gateway tri_yolo.launch.py seed:=1001 piece_reach:=0.28 \\
        yolo:=false headless:=true
    ros2 run mycobot_gateway synthetic_data_collector_tri --ros-args \\
        -p use_sim_time:=true -p output_dir:=<dir> -p num_samples:=400 -p seed:=1001
"""

import math
import random

import numpy as np
import rclpy
from std_msgs.msg import Float64MultiArray

from .sim_sorting_grasp import (GRIPPER_LIMITS, MAX_CLOSE, SimSortingGrasp, _rotation_about,
                                rotation_top_down, tool_tip)
from .synthetic_data_collector_v4_gripper import SyntheticDataCollectorV3 as GripperCollector

# Fingertip targets: the table area the sorter works over (pieces and bins).
TIP_AZIMUTH_DEG = (-70.0, 70.0)
TIP_RADIUS_M = (0.10, 0.42)
TIP_HEIGHT_M = (0.010, 0.160)
# Beyond the vertical-tool reach the sorter tilts the tool outward (drops).
VERTICAL_REACH_M = 0.28
TILT_DEG = (0.0, 45.0)
JITTER_DEG = 3.0
# The jittered fingertip must stay above the plate (3 deg moves it ~15 mm).
MIN_TIP_Z_M = 0.008


class TipSolver:
    """sim_sorting_grasp's fingertip IK, without its node."""

    q_deg = None
    _seeds = SimSortingGrasp._seeds
    _solve_at_rot = SimSortingGrasp._solve_at_rot


class TriSceneCollector(GripperCollector):

    def __init__(self):
        super().__init__()
        self.declare_parameter('table_clearance', 0.05)
        self.declare_parameter('pick_fraction', 0.6)
        self.declare_parameter('seed', 0)
        self.TABLE_CLEARANCE = self.get_parameter('table_clearance').value
        self.pick_fraction = self.get_parameter('pick_fraction').value
        random.seed(self.get_parameter('seed').value)
        np.random.seed(self.get_parameter('seed').value)
        # The sorting scene is the target domain: its own lights, not v3's
        # randomized world (whose light names do not exist here).
        self.domain_randomize = False
        self.solver = TipSolver()
        self.pub_grip = self.create_publisher(
            Float64MultiArray, '/gripper_position_controller/commands', 10)
        self.get_logger().info(
            f'scene de tri : {self.pick_fraction:.0%} poses du trieur, garde au sol '
            f'{self.TABLE_CLEARANCE * 1000:.0f} mm pour les autres')

    def _pick_pose(self):
        """Joint angles (rad) putting the fingertip over the table, or None."""
        for _ in range(20):
            az = math.radians(random.uniform(*TIP_AZIMUTH_DEG))
            r = random.uniform(*TIP_RADIUS_M)
            tip = np.array([r * math.cos(az), r * math.sin(az), random.uniform(*TIP_HEIGHT_M)])
            rot = rotation_top_down(math.radians(random.uniform(0.0, 180.0)))
            if r > VERTICAL_REACH_M or random.random() < 0.3:
                outward = np.array([-math.sin(az), math.cos(az), 0.0])
                rot = _rotation_about(outward, math.radians(random.uniform(*TILT_DEG))) @ rot
            q = self.solver._solve_at_rot(tip, rot, None)
            if q is None:
                continue
            q = q + np.random.uniform(-JITTER_DEG, JITTER_DEG, 6)
            if tool_tip(q)[2] >= MIN_TIP_Z_M:
                return [round(math.radians(a), 4) for a in q]
        return None

    def _random_joint_angles(self):
        if random.random() < self.pick_fraction:
            q = self._pick_pose()
            if q is not None:
                return q
        return super()._random_joint_angles()

    def _collect_next(self):
        closing = random.choice([0.0, 0.0, random.uniform(0.0, MAX_CLOSE)])
        raw = [-closing, closing, closing, -closing, -closing, closing]
        self.pub_grip.publish(Float64MultiArray(
            data=[float(np.clip(v, lo, hi)) for v, (lo, hi) in zip(raw, GRIPPER_LIMITS)]))
        super()._collect_next()


def main(args=None):
    rclpy.init(args=args)
    node = TriSceneCollector()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
