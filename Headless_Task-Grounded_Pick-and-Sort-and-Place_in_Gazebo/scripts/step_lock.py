"""Step-locked control of the tri_yolo scene: Gazebo never runs freely, it advances an exact number
of 1 ms physics steps at a time, and every command is sent while it is paused.

Free-running control lets wall-clock timing into the simulation: a ROS command lands on whichever
physics step is current when it arrives, a pause takes effect a variable number of steps late, and
the newest camera frame depends on both. A grasp amplifies those differences (27/10: 0.02 deg at
the start of a piece became 2.3 mm at the close and 13 mm one chunk later). Here:

  * the simulation is paused once (lock) and then only advanced with WorldControl multi_step,
    `pause: true`, so it stops on an exact step; the target time is waited for on /clock;
  * an arm trajectory is stamped with the current simulation time, a gripper command applies at
    the controllers' next update; both are published while paused, then DELIVERY_S of wall time
    passes before any step (DDS on one host delivers in milliseconds);
  * the cameras render at 10 Hz from the robot's spawn time: the phase is read from their stamps
    and every observation is taken on a tick, from frames stamped exactly that time;
  * moves wait a fixed number of steps (duration + SETTLE_STEPS), not "until close enough".

What it cannot make identical is the free-running start-up before lock (controllers switch on only
while the simulation runs); each piece therefore starts after START_SETTLE_STEPS of holding the
observation pose, which damps the start-up differences, and the start state is fingerprinted at
full precision so a test shows whether two runs really start alike.
"""
import hashlib
import json
import math
import subprocess
import time

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration, Time
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from mycobot_gateway.sim_sorting_grasp import ARM_JOINTS, GRIPPER_JOINTS, JOINT_SPEED_DPS, SETTLE_TOL_DEG, \
    TRANSIT_Z, tool_tip

STEP_NS = 1_000_000            # max_step_size 0.001 in the tri_yolo world
CONTROL_NS = 10_000_000        # controller_manager update_rate 100 in the container's
                               # mycobot_description (the repo's says 500); joint_state_broadcaster
                               # (150 Hz) then publishes at every update
TICK_NS = 100_000_000          # camera update_rate 10
DELIVERY_S = 0.25
SETTLE_STEPS = 500
START_SETTLE_STEPS = 3000
GRIPPER_SETTLE_STEPS = 1500    # the free-running evaluator's spin_for(1.5)


def sha(obj):
    return hashlib.sha256(json.dumps(obj, sort_keys=True).encode()).hexdigest()


class StepLock:
    """Mixed into a SimSortingGrasp node (PolicyRunner). Call lock() once, then only these methods."""

    def init_step_lock(self):
        self.joint_full = None
        self.joint_stamp_ns = -1
        self.create_subscription(JointState, '/joint_states', self._joint_full_cb, 10)
        self.locked = False
        self.phase_ns = None

    def _joint_full_cb(self, msg):
        names = list(msg.name)
        if all(j in names for j in ARM_JOINTS) and GRIPPER_JOINTS[0] in names:
            self.joint_full = [msg.position[names.index(j)] for j in ARM_JOINTS + [GRIPPER_JOINTS[0]]]
            self.joint_stamp_ns = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def now_ns(self):
        return self.get_clock().now().nanoseconds

    def _control(self, req):
        subprocess.run(['gz', 'service', '-s', f'/world/{self.world}/control',
                        '--reqtype', 'gz.msgs.WorldControl', '--reptype', 'gz.msgs.Boolean',
                        '--timeout', '5000', '--req', req], check=True, capture_output=True, timeout=30)

    def _drain(self, seconds):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.01)

    def lock(self):
        """Pause, then wait until /clock has arrived and holds still (the node may not have had one yet)."""
        self._control('pause: true')
        deadline = time.monotonic() + 60.0
        last = -1
        while True:
            self._drain(0.5)
            now = self.now_ns()
            if now > 0 and now == last and self.joint_stamp_ns > 0:
                break
            if time.monotonic() > deadline:
                raise RuntimeError(f'step-locked: clock not settled after pause ({last} -> {now} ns)')
            last = now
        self.locked = True
        self.lock_ns = now

    def step(self, n):
        """Advance exactly n physics steps; return once /clock and /joint_states show the end."""
        target = self.now_ns() + n * STEP_NS
        self._control(f'multi_step: {n}, pause: true')
        watchdog = time.monotonic() + 60.0 + n * 0.01 * self.wall_scale
        while self.now_ns() < target or self.joint_stamp_ns <= target - CONTROL_NS:
            if time.monotonic() > watchdog:
                raise RuntimeError(f'step-locked: clock at {self.now_ns()} ns, joints at '
                                   f'{self.joint_stamp_ns} ns, target {target} ns')
            rclpy.spin_once(self, timeout_sec=0.01)
        if self.now_ns() != target:
            raise RuntimeError(f'step-locked: clock at {self.now_ns()} ns after multi_step, expected {target}')
        self._drain(0.05)

    def stamp(self):
        ns = self.now_ns()
        return Time(sec=ns // 1_000_000_000, nanosec=ns % 1_000_000_000)

    def publish_arm(self, points, dt=None):
        """points: list of 6-joint positions (rad); time_from_start dt*(k+1), or one point at dt."""
        traj = JointTrajectory()
        traj.header.stamp = self.stamp()
        traj.joint_names = ARM_JOINTS
        for k, q in enumerate(points):
            p = JointTrajectoryPoint()
            p.positions = [float(v) for v in q]
            if len(points) == 1:
                p.velocities = [0.0] * 6
            t = dt * (k + 1)
            p.time_from_start = Duration(sec=int(t), nanosec=int(round((t % 1) * 1e9)))
            traj.points.append(p)
        self.pub_arm.publish(traj)

    def deliver(self):
        self._drain(DELIVERY_S)

    def locked_move(self, q_deg):
        """move_to with a fixed number of steps: duration from the travel as move_to computes it."""
        q_deg = np.asarray(q_deg, dtype=float)
        travel = float(np.max(np.abs(q_deg - self.q_deg)))
        duration = float(np.clip(travel / JOINT_SPEED_DPS, 0.6, 4.0))
        duration = math.ceil(duration * 1000) / 1000
        self.publish_arm([np.radians(q_deg)], dt=duration)
        self.deliver()
        self.step(int(round(duration * 1000)) + SETTLE_STEPS)
        if float(np.max(np.abs(self.q_deg - q_deg))) >= SETTLE_TOL_DEG:
            raise RuntimeError(f'step-locked: arm {np.round(self.q_deg, 2).tolist()} not at {q_deg.tolist()}')

    def locked_transit(self, q_goal, label=''):
        """transit() with locked moves: straight if the path clears the table, else up first."""
        if self._path_clears_table(self.q_deg, q_goal):
            self.locked_move(q_goal)
            return True
        tip_now = tool_tip(self.q_deg)
        q_up = self.solve_tip([tip_now[0], tip_now[1], TRANSIT_Z])
        if q_up is None or not self._path_clears_table(q_up, q_goal):
            return False
        self.locked_move(q_up)
        self.locked_move(q_goal)
        return True

    def locked_gripper(self, g, settle_steps=GRIPPER_SETTLE_STEPS):
        self.set_gripper(-float(g), settle=0.0)
        self.deliver()
        if settle_steps:
            self.step(settle_steps)

    def align_to_tick(self):
        """Step to the next camera tick (phase from the frames' stamps, all three cameras alike)."""
        if self.phase_ns is None:
            self._drain(0.5)
            phases = {c: (m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec) % TICK_NS
                      for c, m in self.images.items()}
            if len(phases) != 3 or len(set(phases.values())) != 1:
                raise RuntimeError(f'step-locked: camera phases {phases}')
            self.phase_ns = next(iter(phases.values()))
        n = ((self.phase_ns - self.now_ns()) % TICK_NS) // STEP_NS
        if n:
            self.step(n)

    def locked_observe(self, decode):
        """The three frames stamped exactly now (a camera tick), the full-precision state."""
        t = self.now_ns()
        if (t - self.phase_ns) % TICK_NS:
            raise RuntimeError(f'step-locked: observing at {t} ns, off the camera ticks')
        deadline = time.monotonic() + 60.0
        while True:
            stamps = {c: m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nanosec for c, m in self.images.items()}
            if len(stamps) == 3 and all(s == t for s in stamps.values()):
                break
            if time.monotonic() > deadline:
                raise RuntimeError(f'step-locked: no frames stamped {t} ns (have {stamps})')
            rclpy.spin_once(self, timeout_sec=0.02)
        images = {c: decode(m) for c, m in self.images.items()}
        hashes = {c: hashlib.sha256(img.tobytes()).hexdigest() for c, img in images.items()}
        return images, np.array(self.joint_full, dtype=np.float64), hashes
