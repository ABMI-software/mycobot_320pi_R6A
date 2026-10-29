"""Step-locked control of the tri_yolo scene: Gazebo never runs freely, it advances an exact number
of 1 ms physics steps at a time, and every command is sent while it is paused.

Free-running control lets wall-clock timing into the simulation: a ROS command lands on whichever
physics step is current when it arrives, a pause takes effect a variable number of steps late, and
the newest camera frame depends on both. A grasp amplifies those differences (27/10: 0.02 deg at
the start of a piece became 2.3 mm at the close and 13 mm one chunk later). Here:

  * the simulation is paused once (lock) and then only advanced with WorldControl multi_step,
    `pause: true`, so it stops on an exact step; the target time is waited for on /clock;
  * an arm trajectory is stamped with the current simulation time, a gripper command applies at
    the controllers' next update; both are published while paused, COPIES times, then DELIVERY_S
    of wall time passes before any step. Both controllers subscribe BEST_EFFORT, so under load a
    single message can be dropped (29/10, test A: a lost gripper command gave the arm a 1e-5 rad
    kick at step 14); a copy is harmless (the same value; the same stamped trajectory changes
    nothing once past its first point);
  * the cameras render at 10 Hz from the robot's spawn time: the phase is read from their stamps
    and every observation is taken on a tick, from frames stamped exactly that time;
  * moves wait a fixed number of steps (duration + SETTLE_STEPS), not "until close enough".

Start-up is stepped too (eval_tri_seed.sh STEP_LOCKED=1: the server starts paused, the robot is
spawned at time 0, a stepper advances 100-step blocks until lock), but it still depends on wall
timing (when the controllers switch on). So every piece starts from an exact state reset
(start_from, gz_systems/state_reset.py): a snapshot of the 12 robot joints, the gripper command
and the four pieces' poses, taken by the batch run (or loaded from it, --reset-from), is put back
to the bit with zero velocities, the controllers are given identical hold commands, and the scene
is redrawn. From there, the run depends on nothing that happened before.
"""
import base64
import hashlib
import json
import math
import os
import re
import subprocess
import time
from pathlib import Path

import numpy as np
import rclpy
from builtin_interfaces.msg import Duration, Time
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from mycobot_gateway.sim_sorting_grasp import ARM_JOINTS, GRIPPER_JOINTS, JOINT_SPEED_DPS, SETTLE_TOL_DEG, \
    TRANSIT_Z, tool_tip
from mycobot_gateway.vision import tri_scene

STEP_NS = 1_000_000            # max_step_size 0.001 in the tri_yolo world
CONTROL_NS = 10_000_000        # controller_manager update_rate 100 in the container's
                               # mycobot_description (the repo's says 500); joint_state_broadcaster
                               # (150 Hz) then publishes at every update
TICK_NS = 100_000_000          # camera update_rate 10
DELIVERY_S = 0.5
COPIES = 3                     # each command published this many times, COPY_GAP_S apart
COPY_GAP_S = 0.02
RESEND_S = 20.0               # a step block not started this long after its request was lost
DRIFT_CHECK_S = 0.3            # after a block, the clock must still be on its target this much later
SETTLE_STEPS = 500
START_SETTLE_STEPS = 3000
GRIPPER_SETTLE_STEPS = 1500    # the free-running evaluator's spin_for(1.5)
WARM_UP_OFFSET_DEG = np.array([10.0, -8.0, 6.0, 5.0, -5.0, 10.0])
FAR_LIFT_M = 0.005             # pieces raised for the redraw before the exact reset
ROBOT = 'mycobot_320'
RESET_RESEND_S = 2.0
RESET_TIMEOUT_S = 60.0


def sha(obj):
    return hashlib.sha256(json.dumps(obj, sort_keys=True).encode()).hexdigest()


class StepLock:
    """Mixed into a SimSortingGrasp node (PolicyRunner). Call lock() once, then only these methods."""

    def init_step_lock(self):
        self.joint_full = None
        self.joint_all = {}
        self.joint_velocity = {}
        self.grip_cmd = None
        self.reset_count = 0
        self.joint_stamp_ns = -1
        self.create_subscription(JointState, '/joint_states', self._joint_full_cb, 10)
        self.locked = False
        self.phase_ns = None

    def _joint_full_cb(self, msg):
        names = list(msg.name)
        self.joint_all = dict(zip(names, msg.position))
        self.joint_velocity = dict(zip(names, msg.velocity))
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
        """Stop the start-up stepper (eval_tri_seed.sh), pause, wait until /clock holds still."""
        marker = os.environ.get('STEP_LOCK_DIR')
        if marker:
            Path(marker, 'lock_requested').touch()
            deadline = time.monotonic() + 120.0
            while not Path(marker, 'stepper_stopped').exists():
                if time.monotonic() > deadline:
                    raise RuntimeError('step-locked: the start-up stepper did not stop')
                self._drain(0.2)
        self._control('pause: true')
        deadline = time.monotonic() + 120.0
        seen = []
        while True:
            self._drain(0.5)
            now = self.now_ns()
            seen = (seen + [now])[-4:]
            if now > 0 and len(seen) == 4 and len(set(seen)) == 1 and self.joint_stamp_ns > 0:
                break
            if time.monotonic() > deadline:
                raise RuntimeError(f'step-locked: clock not settled after pause ({seen} ns)')
        self.locked = True
        self.lock_ns = now

    def step(self, n):
        """Advance exactly n physics steps; return once /clock and /joint_states show the end."""
        start = self.now_ns()
        target = start + n * STEP_NS
        self._control(f'multi_step: {n}, pause: true')
        watchdog = time.monotonic() + 60.0 + n * 0.01 * self.wall_scale
        sent = time.monotonic()
        while self.now_ns() < target or self.joint_stamp_ns <= target - CONTROL_NS:
            # A request can go unanswered under load (29/10, A1: the clock stayed on its start). A
            # queued block moves the clock within ~1 s here, so a clock still on `start` after
            # RESEND_S means nothing was queued, and the block is sent again; requests add up, so
            # it is never re-sent once the clock has moved.
            if self.now_ns() == start and time.monotonic() - sent > RESEND_S:
                if self.gz_sim_ns() == start:
                    self._control(f'multi_step: {n}, pause: true')
                sent = time.monotonic()
            if time.monotonic() > watchdog:
                raise RuntimeError(f'step-locked: clock at {self.now_ns()} ns, joints at '
                                   f'{self.joint_stamp_ns} ns, target {target} ns')
            rclpy.spin_once(self, timeout_sec=0.01)
        self._drain(DRIFT_CHECK_S)
        if self.now_ns() != target:
            raise RuntimeError(f'step-locked: clock at {self.now_ns()} ns after multi_step, expected {target} '
                               '(the simulation ran past the block)')

    def gz_sim_ns(self):
        """Gazebo's own simulation time (its /stats), independent of the ROS /clock relay."""
        out = subprocess.run(['gz', 'topic', '-e', '-n', '1', '-t', f'/world/{self.world}/stats'],
                             capture_output=True, text=True, timeout=30).stdout
        block = out.split('sim_time {', 1)[1].split('}', 1)[0] if 'sim_time {' in out else ''
        sec = re.search(r'(?<!n)sec: (\d+)', block)
        nsec = re.search(r'nsec: (\d+)', block)
        return (int(sec.group(1)) if sec else 0) * 1_000_000_000 + (int(nsec.group(1)) if nsec else 0)

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
        for k in range(COPIES):
            self.pub_arm.publish(traj)
            time.sleep(COPY_GAP_S)

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

    def warm_up(self, q_observe_deg):
        """One identical locked motion of the arm and gripper right after lock.

        The renderer redraws a link only when its pose changes by more than a small threshold, so
        after the free-running start-up each run draws the settled arm at a pose that differs by
        ~1e-8 (29/10: 30-60 specks of up to 15 levels on the arm in the first frame, identical
        frames after any locked motion). Moving the arm and gripper the same way in every run makes
        the first frame the policy sees identical too.
        """
        q = np.asarray(q_observe_deg, dtype=float)
        self.locked_move(q)
        self.locked_move(q + WARM_UP_OFFSET_DEG)
        self.locked_move(q)
        self.locked_gripper(0.0)
        self.locked_gripper(-0.28)
        self.step(START_SETTLE_STEPS)

    def piece_poses(self):
        """The four pieces' model poses [x, y, z, qw, qx, qy, qz], as Gazebo prints them (round-trip exact)."""
        out = subprocess.run(['gz', 'topic', '-e', '-n', '1', '-t', f'/world/{self.world}/dynamic_pose/info'],
                             capture_output=True, text=True, timeout=30).stdout
        poses = {}
        for block in out.split('pose {')[1:]:
            name = re.search(r'name: "([^"]+)"', block)
            if not name or name.group(1) not in tri_scene.OBJECTS:
                continue
            def field(section, key):
                m = re.search(section + r' \{[^}]*?' + key + r': ([-\d.e+]+)', block)
                return float(m.group(1)) if m else 0.0
            poses[name.group(1)] = [field('position', k) for k in 'xyz'] + [field('orientation', k) for k in 'wxyz']
        if set(poses) != set(tri_scene.OBJECTS):
            raise RuntimeError(f'step-locked: piece poses {sorted(poses)}')
        return poses

    def snapshot(self):
        """Everything start_from needs to put the scene back to the bit (velocities are reset to 0)."""
        return {'joints': dict(self.joint_all), 'grip_cmd': self.grip_cmd, 'poses': self.piece_poses()}

    def _reset(self, joints, poses, repeat=1):
        """Hand a reset to gz_systems/state_reset.py; it applies in each of the next `repeat` simulated steps."""
        self.reset_count += 1
        rid = f'{os.getpid()}-{self.reset_count}'
        data = base64.b64encode(json.dumps({'id': rid, 'model': ROBOT, 'joints': joints, 'poses': poses,
                                            'repeat': repeat}).encode()).decode()
        topic = f'/world/{self.world}/state_reset'
        # One publish can be lost before the plugin's subscription connects (29/10, under load), so
        # the request is sent again every RESET_RESEND_S until "received <id>" comes back; the plugin
        # applies an id once. The simulation is paused throughout, so nothing moves meanwhile.
        echo = subprocess.Popen(['gz', 'topic', '-e', '-t', topic + '/ack'], stdout=subprocess.PIPE)
        os.set_blocking(echo.stdout.fileno(), False)   # bytes: a non-blocking text read fails on an empty pipe
        reply, deadline = '', time.monotonic() + RESET_TIMEOUT_S
        try:
            time.sleep(1.0)
            while f'received {rid}' not in reply:
                if time.monotonic() > deadline:
                    raise RuntimeError(f'step-locked: state_reset did not acknowledge {rid}: {reply[-300:]!r}')
                subprocess.run(['gz', 'topic', '-t', topic, '-m', 'gz.msgs.StringMsg', '-p', f'data: "{data}"'],
                               check=True, capture_output=True, timeout=30)
                end = time.monotonic() + RESET_RESEND_S
                while time.monotonic() < end and f'received {rid}' not in reply:
                    reply += (echo.stdout.read() or b'').decode(errors='replace')
                    time.sleep(0.1)
        finally:
            echo.kill()

    def start_from(self, snap):
        """Exact state reset to `snap` on a camera tick that is also a controller update.

        One tick earlier the arm is reset WARM_UP_OFFSET_DEG away and the pieces FAR_LIFT_M up, so
        the renderer, which redraws a link only past a small pose change, redraws everything at the
        exact reset. One controller period before the tick, paused just after a controller update,
        the arm gets a hold trajectory that ends at the tick on the snapshot's arm joints (so the
        controller's target there is the snapshot whatever it held before) and the gripper the
        snapshot's command; then the exact reset is applied at each of the CONTROL_NS steps up to
        the tick (a moving joint is only braked, not stopped, by one reset), so at the tick's
        controller update the controllers read the snapshot at rest.
        """
        if (self.phase_ns - self.now_ns()) % TICK_NS == 0:
            self.step(TICK_NS // STEP_NS)
        n = ((self.phase_ns - STEP_NS - self.now_ns()) % TICK_NS) // STEP_NS
        if n:
            self.step(n)
        far = dict(snap['joints'])
        for name, d in zip(ARM_JOINTS, np.radians(WARM_UP_OFFSET_DEG)):
            far[name] += float(d)
        self._reset(far, {k: v[:2] + [v[2] + FAR_LIFT_M] + v[3:] for k, v in snap['poses'].items()})
        self.step(1)
        self.step((TICK_NS - CONTROL_NS) // STEP_NS)
        q_arm = [snap['joints'][j] for j in ARM_JOINTS]
        self.publish_arm([q_arm], dt=CONTROL_NS / 1e9)
        self.send_gripper(snap['grip_cmd'])
        self.deliver()
        self._reset(snap['joints'], snap['poses'], repeat=CONTROL_NS // STEP_NS)
        self.step(CONTROL_NS // STEP_NS)
        if (self.now_ns() - self.phase_ns) % TICK_NS or self.joint_stamp_ns != self.now_ns():
            raise RuntimeError(f'step-locked: exact reset not on a tick and a controller update '
                               f'(now {self.now_ns()}, joints {self.joint_stamp_ns})')
        # Gazebo's last step leaves ~1e-14 rad and ~1e-11 rad/s (29/10); what must be identical is two
        # runs from the same snapshot, which the fingerprints compare. This only catches a failed reset.
        bad = {j: (self.joint_all.get(j), v) for j, v in snap['joints'].items()
               if self.joint_all.get(j) is None or abs(self.joint_all[j] - v) > 1e-9}
        moving = {j: v for j, v in self.joint_velocity.items() if abs(v) > 1e-9}
        if bad or moving:
            raise RuntimeError(f'step-locked: after the exact reset, joints differ {bad}, velocities {moving}')

    def send_gripper(self, g):
        """g as the recorded gripper action (data[0] of the command), COPIES times."""
        self.grip_cmd = float(g)
        for k in range(COPIES):
            self.set_gripper(-float(g), settle=0.0)
            time.sleep(COPY_GAP_S)

    def locked_gripper(self, g, settle_steps=GRIPPER_SETTLE_STEPS):
        self.send_gripper(g)
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
