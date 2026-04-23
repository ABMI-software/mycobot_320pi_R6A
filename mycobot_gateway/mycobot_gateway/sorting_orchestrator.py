#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Multi-object color-sorting orchestrator for MyCobot 320 Pi in Gazebo.

Loops over color-tagged detections from /sorting/detections (published by
color_object_detector) and runs a pick-and-place per object, dropping each
into its colour-matching bin. Joint motion is the same pattern as
pick_and_place_node (numerical IK + interpolated joint command stream).

Because the simulated robot has no working gripper, "grasp" and "release"
are emulated by teleporting the object's model in Gazebo via the world's
set_pose service: during the carry segments the object is snapped to the
current end-effector position; on release it lands at the bin XY.

Topics:
  Subs:  /sorting/detections (String, "color,x,y;color,x,y;...")
         /joint_states (sensor_msgs/JointState)
  Pubs:  /pickplace/status (String, "STATE|detail")
         /model/mycobot_320/joint/<j>/cmd_pos  (Float64, six joints)
"""

import enum
import os
import subprocess
import sys
import time
from typing import Dict, List, Optional, Tuple

import numpy as np

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64, String


DREAM_DIR_ALT = '/home/genji/ros_jazzy/src/mycobot_R6A/training/dream'
DREAM_DIR = os.path.normpath(os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    '..', '..', '..', '..', 'training', 'dream',
))
for p in [DREAM_DIR, DREAM_DIR_ALT]:
    if os.path.isdir(p) and p not in sys.path:
        sys.path.insert(0, p)

from mycobot_ik import inverse_kinematics_position, fk_end_effector  # noqa: E402


# Color → Gazebo model name to teleport during grasp/release.
COLOR_TO_MODEL: Dict[str, str] = {
    'red':    'red_cube',
    'blue':   'blue_cube',
    'green':  'green_cylinder',
    'yellow': 'yellow_box',
}

# Color → bin centre XY in robot base frame (matches pick_and_place_sorting.sdf).
COLOR_TO_BIN: Dict[str, Tuple[float, float]] = {
    'red':    (-0.22, -0.18),
    'blue':   (-0.22, -0.06),
    'green':  (-0.22,  0.06),
    'yellow': (-0.22,  0.18),
}


class State(enum.Enum):
    INIT            = 'INIT'
    MOVE_HOME       = 'MOVE_HOME'
    WAIT_DETECTIONS = 'WAIT_DETECTIONS'
    NEXT_TARGET     = 'NEXT_TARGET'
    PLAN_PICK       = 'PLAN_PICK'
    EXEC_TRAJECTORY = 'EXEC_TRAJECTORY'
    GRASP           = 'GRASP'
    RELEASE         = 'RELEASE'
    DONE            = 'DONE'
    ERROR           = 'ERROR'


class SortingOrchestrator(Node):

    JOINT_NAMES = [
        'joint2_to_joint1',
        'joint3_to_joint2',
        'joint4_to_joint3',
        'joint5_to_joint4',
        'joint6_to_joint5',
        'joint6output_to_joint6',
    ]

    def __init__(self):
        super().__init__('sorting_orchestrator')

        self.declare_parameter('world_name', 'pick_and_place_sorting')
        self.declare_parameter('approach_height', 0.10)
        self.declare_parameter('grasp_z', 0.04)
        self.declare_parameter('release_z', 0.05)
        self.declare_parameter('grasp_settle_time', 1.0)
        self.declare_parameter('step_duration', 0.15)
        self.declare_parameter('settle_time', 1.5)
        self.declare_parameter('detection_timeout', 15.0)
        self.declare_parameter('interpolation_steps', 12)
        self.declare_parameter('startup_delay', 5.0)
        # If true, skip detector and use SDF-known positions (smoke-test mode).
        self.declare_parameter('use_detector', True)
        self.declare_parameter('process_order',
                               'red,blue,green,yellow')

        self.world = str(self.get_parameter('world_name').value)
        self.approach_h = float(self.get_parameter('approach_height').value)
        self.grasp_z = float(self.get_parameter('grasp_z').value)
        self.release_z = float(self.get_parameter('release_z').value)
        self.grasp_settle = float(self.get_parameter('grasp_settle_time').value)
        self.step_duration = float(self.get_parameter('step_duration').value)
        self.settle_time = float(self.get_parameter('settle_time').value)
        self.det_timeout = float(self.get_parameter('detection_timeout').value)
        self.interp_steps = int(self.get_parameter('interpolation_steps').value)
        self.startup_delay = float(self.get_parameter('startup_delay').value)
        self.use_detector = bool(self.get_parameter('use_detector').value)
        order_str = str(self.get_parameter('process_order').value)
        self.process_order = [c.strip() for c in order_str.split(',') if c.strip()]

        # Fallback positions when use_detector=False (mirrors the SDF).
        self.fallback_positions: Dict[str, Tuple[float, float]] = {
            'red':    (0.22, -0.12),
            'blue':   (0.22,  0.12),
            'green':  (0.27, -0.05),
            'yellow': (0.27,  0.05),
        }

        self.state = State.INIT
        self.state_enter_time = time.time()
        self.startup_time = time.time()

        self.current_joints: Optional[np.ndarray] = None
        self.detections: Dict[str, Tuple[float, float]] = {}
        self.det_received_at: Optional[float] = None
        self.det_stable_count = 0

        # Per-target state
        self.target_color: Optional[str] = None
        self.target_pick: Optional[Tuple[float, float]] = None
        self.target_bin: Optional[Tuple[float, float]] = None
        self.processed_colors: List[str] = []

        # Trajectory execution
        self.full_plan: List[Tuple[str, List[np.ndarray]]] = []
        self.plan_idx = 0
        self.trajectory_queue: List[np.ndarray] = []
        self.traj_idx = 0
        self.carrying = False  # if True, teleport the held model to the EE

        self.joint_pubs = {
            jn: self.create_publisher(
                Float64, f'/model/mycobot_320/joint/{jn}/cmd_pos', 10
            ) for jn in self.JOINT_NAMES
        }
        self.pub_status = self.create_publisher(String, '/pickplace/status', 10)

        self.create_subscription(JointState, '/joint_states', self._joint_cb, 10)
        self.create_subscription(String, '/sorting/detections', self._det_cb, 10)

        self.create_timer(0.1, self._tick)
        self.get_logger().info(
            f'Sorting orchestrator — world={self.world}, '
            f'order={self.process_order}, use_detector={self.use_detector}'
        )

    # ── Callbacks ──────────────────────────────────────────────
    def _joint_cb(self, msg: JointState):
        if not msg.name:
            return
        angles = np.zeros(6)
        names = list(msg.name)
        for i, jn in enumerate(self.JOINT_NAMES):
            if jn in names:
                angles[i] = msg.position[names.index(jn)]
        self.current_joints = angles

    def _det_cb(self, msg: String):
        if not msg.data:
            return
        parsed: Dict[str, Tuple[float, float]] = {}
        for entry in msg.data.split(';'):
            parts = entry.split(',')
            if len(parts) != 3:
                continue
            try:
                parsed[parts[0]] = (float(parts[1]), float(parts[2]))
            except ValueError:
                continue
        self.detections = parsed
        self.det_received_at = time.time()
        self.det_stable_count += 1

    # ── Helpers ─────────────────────────────────────────────────
    def _command_joints(self, q: np.ndarray):
        for jn, ang in zip(self.JOINT_NAMES, q):
            self.joint_pubs[jn].publish(Float64(data=float(ang)))

    @staticmethod
    def _interpolate(q_from: np.ndarray, q_to: np.ndarray, steps: int):
        return [q_from + (i / steps) * (q_to - q_from) for i in range(1, steps + 1)]

    def _publish_status(self, detail: str = ''):
        self.pub_status.publish(String(data=f'{self.state.value}|{detail}'))

    def _set_state(self, new_state: State):
        self.get_logger().info(f'⇢ {self.state.value} → {new_state.value}')
        self.state = new_state
        self.state_enter_time = time.time()

    def _elapsed(self) -> float:
        return time.time() - self.state_enter_time

    def _gz_set_pose(self, model_name: str, x: float, y: float, z: float) -> bool:
        """Teleport a Gazebo model via the world's set_pose service."""
        req = (
            f'name: "{model_name}", '
            f'position: {{x: {x}, y: {y}, z: {z}}}, '
            f'orientation: {{x: 0, y: 0, z: 0, w: 1}}'
        )
        cmd = [
            'gz', 'service', '-s', f'/world/{self.world}/set_pose',
            '--reqtype', 'gz.msgs.Pose',
            '--reptype', 'gz.msgs.Boolean',
            '--timeout', '500',
            '--req', req,
        ]
        try:
            res = subprocess.run(cmd, capture_output=True, text=True, timeout=2.0)
            if res.returncode != 0:
                self.get_logger().warn(
                    f'set_pose failed for {model_name}: {res.stderr.strip()}'
                )
                return False
            return True
        except (subprocess.TimeoutExpired, FileNotFoundError) as e:
            self.get_logger().warn(f'gz service call error: {e}')
            return False

    # ── Main tick ──────────────────────────────────────────────
    def _tick(self):
        if time.time() - self.startup_time < self.startup_delay:
            return
        self._publish_status()
        handler = {
            State.INIT:            self._on_init,
            State.MOVE_HOME:       self._on_move_home,
            State.WAIT_DETECTIONS: self._on_wait_detections,
            State.NEXT_TARGET:     self._on_next_target,
            State.PLAN_PICK:       self._on_plan_pick,
            State.EXEC_TRAJECTORY: self._on_exec_trajectory,
            State.GRASP:           self._on_grasp,
            State.RELEASE:         self._on_release,
            State.DONE:            self._on_done,
        }.get(self.state)
        if handler:
            handler()

    # ── States ─────────────────────────────────────────────────
    def _on_init(self):
        if self.current_joints is not None:
            self.get_logger().info('Joint states received')
            self._set_state(State.MOVE_HOME)
        elif self._elapsed() > 15.0:
            self.get_logger().error('No joint states after 15 s')
            self._set_state(State.ERROR)

    def _on_move_home(self):
        self._command_joints(np.zeros(6))
        if self._elapsed() > 3.0:
            if self.use_detector:
                self._set_state(State.WAIT_DETECTIONS)
            else:
                self._set_state(State.NEXT_TARGET)

    def _on_wait_detections(self):
        # Need at least 3 detection messages so the HSV mask has settled,
        # and at least 2 colors detected.
        if self.det_stable_count >= 3 and len(self.detections) >= 2:
            self.get_logger().info(
                f'Detections stable: {sorted(self.detections.keys())}'
            )
            self._set_state(State.NEXT_TARGET)
            return
        if self._elapsed() > self.det_timeout:
            self.get_logger().warn(
                'Detector timeout — falling back to SDF-known positions'
            )
            self.use_detector = False
            self._set_state(State.NEXT_TARGET)

    def _on_next_target(self):
        # Pick the next color in process_order that we have a position for
        # and haven't processed yet.
        for color in self.process_order:
            if color in self.processed_colors:
                continue
            pos = (self.detections.get(color)
                   if self.use_detector else self.fallback_positions.get(color))
            if pos is None:
                self.get_logger().info(
                    f'Skipping {color}: no position available'
                )
                self.processed_colors.append(color)
                continue
            if color not in COLOR_TO_BIN:
                self.get_logger().warn(f'No bin mapping for color {color}')
                self.processed_colors.append(color)
                continue
            self.target_color = color
            self.target_pick = pos
            self.target_bin = COLOR_TO_BIN[color]
            self.get_logger().info(
                f'▶ Sorting {color}: pick={pos} → bin={self.target_bin}'
            )
            self._set_state(State.PLAN_PICK)
            return
        # Nothing left.
        self._set_state(State.DONE)

    def _on_plan_pick(self):
        q_now = (self.current_joints.copy()
                 if self.current_joints is not None else np.zeros(6))
        px, py = self.target_pick
        bx, by = self.target_bin

        pick_approach = np.array([px, py, self.grasp_z + self.approach_h])
        pick_grasp    = np.array([px, py, self.grasp_z])
        pick_lift     = pick_approach.copy()
        place_approach = np.array([bx, by, self.release_z + self.approach_h])
        place_pos      = np.array([bx, by, self.release_z])
        place_retreat  = place_approach.copy()

        waypoints = [
            ('pick_approach',  pick_approach),
            ('pick_grasp',     pick_grasp),
            ('pick_lift',      pick_lift),
            ('place_approach', place_approach),
            ('place_pos',      place_pos),
            ('place_retreat',  place_retreat),
        ]

        ik: Dict[str, np.ndarray] = {}
        q_prev = q_now
        for name, pos in waypoints:
            ok, q_sol, res = inverse_kinematics_position(pos, q0=q_prev)
            if not ok:
                self.get_logger().error(
                    f'IK failed at {name}: pos={pos}, err={res*1000:.1f} mm'
                )
                # Skip this color and move on; don't crash the whole demo.
                self.processed_colors.append(self.target_color)
                self._set_state(State.NEXT_TARGET)
                return
            ik[name] = q_sol
            q_prev = q_sol

        N = self.interp_steps
        n2 = max(N // 2, 3)
        self.full_plan = [
            ('approach_pick',  self._interpolate(q_now,                ik['pick_approach'], N),  False),
            ('descend_grasp',  self._interpolate(ik['pick_approach'],  ik['pick_grasp'],    n2), False),
            ('GRASP',          [],                                                              False),
            ('lift_object',    self._interpolate(ik['pick_grasp'],     ik['pick_lift'],     n2), True),
            ('move_to_bin',    self._interpolate(ik['pick_lift'],      ik['place_approach'], N), True),
            ('descend_place',  self._interpolate(ik['place_approach'], ik['place_pos'],     n2), True),
            ('RELEASE',        [],                                                              False),
            ('retreat',        self._interpolate(ik['place_pos'],      ik['place_retreat'], n2), False),
            ('return_home',    self._interpolate(ik['place_retreat'],  np.zeros(6),         N),  False),
        ]
        self.plan_idx = 0
        self._start_next_segment()

    def _start_next_segment(self):
        if self.plan_idx >= len(self.full_plan):
            # Finished this color; move to next.
            self.processed_colors.append(self.target_color)
            self._set_state(State.NEXT_TARGET)
            return

        label, traj, carrying = self.full_plan[self.plan_idx]
        self.plan_idx += 1
        self.carrying = carrying

        if label == 'GRASP':
            self._set_state(State.GRASP)
        elif label == 'RELEASE':
            self._set_state(State.RELEASE)
        else:
            self.get_logger().info(
                f'  {label} ({len(traj)} steps, carrying={carrying})'
            )
            self.trajectory_queue = traj
            self.traj_idx = 0
            self._set_state(State.EXEC_TRAJECTORY)

    def _on_exec_trajectory(self):
        if self.traj_idx >= len(self.trajectory_queue):
            total_send = len(self.trajectory_queue) * self.step_duration
            if self._elapsed() > total_send + self.settle_time:
                self._start_next_segment()
            return

        expected_t = self.traj_idx * self.step_duration
        if self._elapsed() >= expected_t:
            q = self.trajectory_queue[self.traj_idx]
            self._command_joints(q)
            if self.carrying and self.target_color is not None:
                # Snap the held object to the EE position so it visually
                # follows the gripper through the carry segments.
                ee = fk_end_effector(q)
                model = COLOR_TO_MODEL.get(self.target_color)
                if model is not None:
                    self._gz_set_pose(
                        model, float(ee[0]), float(ee[1]),
                        max(float(ee[2]) - 0.015, 0.02),
                    )
            self.traj_idx += 1

    def _on_grasp(self):
        if self._elapsed() < 0.05:
            self.get_logger().info(f'  GRASP {self.target_color} (simulated)')
            # Snap the object up to the current EE so the carry phase looks
            # natural from frame 1.
            if self.current_joints is not None:
                ee = fk_end_effector(self.current_joints)
                model = COLOR_TO_MODEL.get(self.target_color, '')
                if model:
                    self._gz_set_pose(
                        model, float(ee[0]), float(ee[1]),
                        max(float(ee[2]) - 0.015, 0.02),
                    )
        if self._elapsed() > self.grasp_settle:
            self._start_next_segment()

    def _on_release(self):
        if self._elapsed() < 0.05:
            self.get_logger().info(f'  RELEASE {self.target_color} into bin')
            bx, by = self.target_bin
            model = COLOR_TO_MODEL.get(self.target_color, '')
            if model:
                # Drop the object onto the bin floor so gravity settles it.
                self._gz_set_pose(model, float(bx), float(by), 0.06)
        if self._elapsed() > self.grasp_settle:
            self._start_next_segment()

    def _on_done(self):
        if not hasattr(self, '_done_logged'):
            self._done_logged = True
            self._command_joints(np.zeros(6))
            self.get_logger().info('=' * 60)
            self.get_logger().info(f'Sorting complete: {self.processed_colors}')
            self.get_logger().info('=' * 60)
            self._publish_status('COMPLETE')


def main(args=None):
    rclpy.init(args=args)
    node = SortingOrchestrator()
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
