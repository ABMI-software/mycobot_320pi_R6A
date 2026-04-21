#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Pick-and-Place Orchestrator for MyCobot 320 Pi in Gazebo.

This node orchestrates a full pick-and-place cycle using:
1. DREAM vision (keypoint detection → PnP pose) for verification
2. Numerical IK (mycobot_ik.py) for motion planning
3. Direct joint commands (Gazebo ros_gz_bridge) for execution

Pipeline:
  Home → [Vision check] → Approach pick → Descend → Grasp → Lift →
  Approach place → Descend → Release → Retreat → Home → Done

Topics Published:
    /pickplace/status   (std_msgs/String)  — state machine status

Topics Subscribed:
    /dream/keypoints    (Float64MultiArray) — from dream_inference_node
    /dream/pose         (PoseStamped)       — from dream_inference_node
    /dream/status       (String)            — from dream_inference_node
    /joint_states       (JointState)        — from Gazebo bridge
"""

import os
import sys
import time
import math
import enum
from typing import List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64, Float64MultiArray, String

# Add dream module for IK/FK
DREAM_DIR_ALT = '/home/genji/ros_jazzy/src/mycobot_R6A/training/dream'
DREAM_DIR = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', '..', '..', '..', 'training', 'dream'
)
DREAM_DIR = os.path.normpath(DREAM_DIR)
for p in [DREAM_DIR, DREAM_DIR_ALT]:
    if os.path.isdir(p) and p not in sys.path:
        sys.path.insert(0, p)

from mycobot_ik import inverse_kinematics_position, fk_end_effector
from mycobot_fk import KEYPOINT_NAMES


class State(enum.Enum):
    """Pick-and-place state machine states."""
    INIT            = 'INIT'
    MOVE_HOME       = 'MOVE_HOME'
    WAIT_VISION     = 'WAIT_VISION'
    VERIFY_POSE     = 'VERIFY_POSE'
    PLAN_PICK       = 'PLAN_PICK'
    EXEC_TRAJECTORY = 'EXEC_TRAJECTORY'
    GRASP           = 'GRASP'
    RELEASE         = 'RELEASE'
    DONE            = 'DONE'
    ERROR           = 'ERROR'


class PickAndPlaceNode(Node):
    """ROS2 node for pick-and-place with DREAM vision feedback."""

    JOINT_NAMES = [
        'joint2_to_joint1',
        'joint3_to_joint2',
        'joint4_to_joint3',
        'joint5_to_joint4',
        'joint6_to_joint5',
        'joint6output_to_joint6',
    ]

    def __init__(self):
        super().__init__('pick_and_place_node')

        # ── Parameters ──
        self.declare_parameter('target_x', 0.25)
        self.declare_parameter('target_y', 0.10)
        self.declare_parameter('target_z', 0.04)
        self.declare_parameter('place_x', -0.20)
        self.declare_parameter('place_y', 0.15)
        self.declare_parameter('place_z', 0.04)
        self.declare_parameter('approach_height', 0.10)
        self.declare_parameter('grasp_settle_time', 2.0)
        self.declare_parameter('step_duration', 0.15)
        self.declare_parameter('settle_time', 2.0)
        self.declare_parameter('vision_timeout', 10.0)
        self.declare_parameter('use_vision_feedback', True)
        self.declare_parameter('interpolation_steps', 20)

        self.target = np.array([
            self.get_parameter('target_x').value,
            self.get_parameter('target_y').value,
            self.get_parameter('target_z').value,
        ])
        self.place = np.array([
            self.get_parameter('place_x').value,
            self.get_parameter('place_y').value,
            self.get_parameter('place_z').value,
        ])
        self.approach_h = self.get_parameter('approach_height').value
        self.grasp_settle = self.get_parameter('grasp_settle_time').value
        self.step_duration = self.get_parameter('step_duration').value
        self.settle_time = self.get_parameter('settle_time').value
        self.vision_timeout = self.get_parameter('vision_timeout').value
        self.use_vision = self.get_parameter('use_vision_feedback').value
        self.interp_steps = self.get_parameter('interpolation_steps').value

        # ── State machine ──
        self.state = State.INIT
        self.state_enter_time = time.time()

        # ── Trajectory execution ──
        self.trajectory_queue: List[np.ndarray] = []
        self.traj_idx = 0

        # ── Full plan segments: list of (label, trajectory) ──
        self.full_plan: List[Tuple[str, List[np.ndarray]]] = []
        self.plan_idx = 0

        # ── Sensor data ──
        self.current_joints = None
        self.dream_status = None
        self.dream_keypoints = None
        self.dream_pose = None
        self.vision_ok_count = 0

        # ── Joint publishers ──
        self.joint_pubs = {}
        for jname in self.JOINT_NAMES:
            topic = f'/model/mycobot_320/joint/{jname}/cmd_pos'
            self.joint_pubs[jname] = self.create_publisher(Float64, topic, 10)

        # ── Status publisher ──
        self.pub_status = self.create_publisher(String, '/pickplace/status', 10)

        # ── Subscribers ──
        self.create_subscription(JointState, '/joint_states', self._joint_cb, 10)
        self.create_subscription(String, '/dream/status', self._dream_status_cb, 10)
        self.create_subscription(Float64MultiArray, '/dream/keypoints', self._dream_kp_cb, 10)
        self.create_subscription(PoseStamped, '/dream/pose', self._dream_pose_cb, 10)

        # ── Main loop (10 Hz) ──
        self.create_timer(0.1, self._tick)
        self.startup_time = time.time()

        self.get_logger().info(
            f'🤖 Pick-and-Place Orchestrator\n'
            f'   Pick target : {self.target}\n'
            f'   Place target: {self.place}\n'
            f'   Approach H  : {self.approach_h}m\n'
            f'   Vision FB   : {self.use_vision}'
        )

    # ── Callbacks ──────────────────────────────────────────────
    def _joint_cb(self, msg: JointState):
        if not msg.name:
            return
        angles = [0.0] * 6
        for i, jn in enumerate(self.JOINT_NAMES):
            if jn in msg.name:
                idx = list(msg.name).index(jn)
                angles[i] = msg.position[idx]
        self.current_joints = np.array(angles)

    def _dream_status_cb(self, msg: String):
        self.dream_status = msg.data

    def _dream_kp_cb(self, msg: Float64MultiArray):
        self.dream_keypoints = list(msg.data)

    def _dream_pose_cb(self, msg: PoseStamped):
        self.dream_pose = msg

    # ── Joint control ──────────────────────────────────────────
    def _command_joints(self, angles: np.ndarray):
        for jname, angle in zip(self.JOINT_NAMES, angles):
            msg = Float64()
            msg.data = float(angle)
            self.joint_pubs[jname].publish(msg)

    def _interpolate(self, q_from: np.ndarray, q_to: np.ndarray, steps: int) -> List[np.ndarray]:
        return [q_from + (i / steps) * (q_to - q_from) for i in range(1, steps + 1)]

    # ── Status ─────────────────────────────────────────────────
    def _publish_status(self, detail: str = ''):
        msg = String()
        msg.data = f'{self.state.value}|{detail}'
        self.pub_status.publish(msg)

    def _set_state(self, new_state: State):
        self.get_logger().info(f'🔄 {self.state.value} → {new_state.value}')
        self.state = new_state
        self.state_enter_time = time.time()

    def _elapsed(self) -> float:
        return time.time() - self.state_enter_time

    # ── Main tick ──────────────────────────────────────────────
    def _tick(self):
        if time.time() - self.startup_time < 5.0:
            return
        self._publish_status()
        handler = {
            State.INIT:            self._on_init,
            State.MOVE_HOME:       self._on_move_home,
            State.WAIT_VISION:     self._on_wait_vision,
            State.VERIFY_POSE:     self._on_verify_pose,
            State.PLAN_PICK:       self._on_plan_pick,
            State.EXEC_TRAJECTORY: self._on_exec_trajectory,
            State.GRASP:           self._on_grasp,
            State.RELEASE:         self._on_release,
            State.DONE:            self._on_done,
        }.get(self.state)
        if handler:
            handler()

    # ── State handlers ─────────────────────────────────────────

    def _on_init(self):
        if self.current_joints is not None:
            self.get_logger().info('📡 Joint states received')
            self._set_state(State.MOVE_HOME)
        elif self._elapsed() > 15.0:
            self.get_logger().error('❌ No joint states after 15s')
            self._set_state(State.ERROR)

    def _on_move_home(self):
        home = np.zeros(6)
        self._command_joints(home)
        if self._elapsed() > 3.0:
            if self.use_vision:
                self._set_state(State.WAIT_VISION)
            else:
                self._set_state(State.PLAN_PICK)

    def _on_wait_vision(self):
        if self.dream_status and self.dream_status.startswith('OK'):
            self.vision_ok_count += 1
            if self.vision_ok_count >= 3:
                self.get_logger().info(f'👁️ Vision ready: {self.dream_status}')
                self._set_state(State.VERIFY_POSE)
                return
        else:
            self.vision_ok_count = 0
        if self._elapsed() > self.vision_timeout:
            self.get_logger().warn('⚠️ Vision timeout — going open-loop')
            self.use_vision = False
            self._set_state(State.PLAN_PICK)

    def _on_verify_pose(self):
        if self.dream_keypoints:
            n_kp = len(self.dream_keypoints) // 3
            detected = sum(1 for i in range(n_kp)
                           if self.dream_keypoints[i * 3 + 2] > 0.5)
            self.get_logger().info(f'🔍 DREAM: {detected}/{n_kp} keypoints detected')
            for i in range(n_kp):
                u, v = self.dream_keypoints[i*3], self.dream_keypoints[i*3+1]
                valid = self.dream_keypoints[i*3+2]
                if valid > 0.5 and i < len(KEYPOINT_NAMES):
                    self.get_logger().info(
                        f'   {KEYPOINT_NAMES[i]}: ({u:.1f}, {v:.1f})'
                    )
        self._set_state(State.PLAN_PICK)

    def _on_plan_pick(self):
        """Solve IK for all waypoints and build full trajectory plan."""
        self.get_logger().info('📐 Planning full pick-and-place trajectory...')
        q_now = self.current_joints.copy() if self.current_joints is not None else np.zeros(6)

        # Cartesian waypoints
        pick_approach = self.target.copy(); pick_approach[2] += self.approach_h
        pick_grasp    = self.target.copy(); pick_grasp[2] += 0.02
        place_approach = self.place.copy(); place_approach[2] += self.approach_h
        place_pos      = self.place.copy(); place_pos[2] += 0.02

        waypoints = [
            ('pick_approach',  pick_approach),
            ('pick_grasp',     pick_grasp),
            ('pick_lift',      pick_approach),
            ('place_approach', place_approach),
            ('place_pos',      place_pos),
            ('place_retreat',  place_approach),
        ]

        # Solve IK chain (warm-started from previous solution)
        ik = {}
        q_prev = q_now
        for name, pos in waypoints:
            ok, q_sol, res = inverse_kinematics_position(pos, q0=q_prev)
            if not ok:
                self.get_logger().error(
                    f'❌ IK failed for {name}: pos={pos}, err={res*1000:.1f}mm'
                )
                self._set_state(State.ERROR)
                return
            ee = fk_end_effector(q_sol)
            err = np.linalg.norm(ee - pos) * 1000
            self.get_logger().info(
                f'  ✓ {name:16s}: {np.round(np.degrees(q_sol), 1)}° err={err:.1f}mm'
            )
            ik[name] = q_sol
            q_prev = q_sol

        # Build plan segments
        N = self.interp_steps
        n2 = max(N // 2, 3)
        self.full_plan = [
            ('1→ Approach pick',  self._interpolate(q_now, ik['pick_approach'], N)),
            ('2→ Descend grasp',  self._interpolate(ik['pick_approach'], ik['pick_grasp'], n2)),
            ('GRASP', []),
            ('3→ Lift object',    self._interpolate(ik['pick_grasp'], ik['pick_lift'], n2)),
            ('4→ Move to place',  self._interpolate(ik['pick_lift'], ik['place_approach'], N)),
            ('5→ Descend place',  self._interpolate(ik['place_approach'], ik['place_pos'], n2)),
            ('RELEASE', []),
            ('6→ Retreat',        self._interpolate(ik['place_pos'], ik['place_retreat'], n2)),
            ('7→ Return home',    self._interpolate(ik['place_retreat'], np.zeros(6), N)),
        ]
        self.plan_idx = 0
        total_steps = sum(len(s[1]) for s in self.full_plan)
        self.get_logger().info(
            f'✅ Plan: {len(self.full_plan)} segments, {total_steps} steps'
        )
        self._start_next_segment()

    def _start_next_segment(self):
        """Advance to the next segment in the plan."""
        if self.plan_idx >= len(self.full_plan):
            self._set_state(State.DONE)
            return

        label, traj = self.full_plan[self.plan_idx]
        self.plan_idx += 1

        if label == 'GRASP':
            self._set_state(State.GRASP)
        elif label == 'RELEASE':
            self._set_state(State.RELEASE)
        else:
            self.get_logger().info(f'▶️ {label} ({len(traj)} steps)')
            self.trajectory_queue = traj
            self.traj_idx = 0
            self._set_state(State.EXEC_TRAJECTORY)

    def _on_exec_trajectory(self):
        """Execute trajectory: send one joint config per step_duration."""
        if self.traj_idx >= len(self.trajectory_queue):
            # Done sending — wait settle_time for robot to reach final config
            total_send_time = len(self.trajectory_queue) * self.step_duration
            if self._elapsed() > total_send_time + self.settle_time:
                self._start_next_segment()
            return

        expected_time = self.traj_idx * self.step_duration
        if self._elapsed() >= expected_time:
            q = self.trajectory_queue[self.traj_idx]
            self._command_joints(q)
            self.traj_idx += 1
            if self.traj_idx >= len(self.trajectory_queue):
                ee = fk_end_effector(q)
                self.get_logger().info(f'  ✓ Done, EE={np.round(ee, 4)}')

    def _on_grasp(self):
        if self._elapsed() < 0.5:
            self.get_logger().info('🤏 GRASPING (simulated)')
        if self._elapsed() > self.grasp_settle:
            self.get_logger().info('  ✓ Grasp complete')
            self._start_next_segment()

    def _on_release(self):
        if self._elapsed() < 0.5:
            self.get_logger().info('📦 RELEASING (simulated)')
        if self._elapsed() > self.grasp_settle:
            self.get_logger().info('  ✓ Release complete')
            self._start_next_segment()

    def _on_done(self):
        if not hasattr(self, '_done_logged'):
            self._done_logged = True
            self._command_joints(np.zeros(6))
            self.get_logger().info('=' * 60)
            self.get_logger().info('🎉 PICK-AND-PLACE COMPLETE!')
            self.get_logger().info(f'   Picked from : {self.target}')
            self.get_logger().info(f'   Placed at   : {self.place}')
            if self.dream_status:
                self.get_logger().info(f'   Final vision: {self.dream_status}')
            self.get_logger().info('=' * 60)
            self._publish_status('COMPLETE')


def main(args=None):
    rclpy.init(args=args)
    node = PickAndPlaceNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
