"""Gazebo system (PythonSystemLoader): exact state reset for step-locked evaluations (step_lock.py).

Listens on /world/<world>/state_reset for a gz.msgs.StringMsg whose data is base64 JSON:

  {"id": "...", "model": "mycobot_320", "joints": {name: position}, "poses": {model: [x, y, z, qw, qx, qy, qz]},
   "repeat": n}

acknowledges receipt on /world/<world>/state_reset/ack ("received <id>"), and applies the reset in
the Update of the next simulated (not paused) iteration: every listed joint gets its position
reset and its velocity reset to exactly 0, every listed model its world pose and a zero linear and
angular velocity on its canonical link. In that same iteration the joints' velocity command is set
to 0: gz_ros2_control turns position commands into a velocity command recomputed from the state it
read at its last update, so without this the step after the reset would act on the pre-reset state.
gz_ros2_control writes that command in PreUpdate; every PreUpdate runs before any Update, and the
Physics system steps in Update, so this system works in Update with a priority below Physics's
(gz:system_priority -100, set where the world is written; 29/10: done in PreUpdate after
gz_ros2_control, the stale command still moved the arm). The reset must land on a
controller-update iteration so the controllers read the reset state at the end of it (step_lock.py
times it). It then acknowledges "applied <id> <sim ns>". With "repeat": n it applies the same reset in each of the
next n simulated iterations: the zero velocity command goes through a force-limited actuator, so a
joint moving when the reset lands is only braked in one step (29/10: -1.57 -> -0.68 rad/s on
joint 1), and the repeats bring every joint to the snapshot at rest before the controllers read it.

A request with "state": true instead returns, in the next system call (paused or not), the
positions and velocities of every joint of the model as Gazebo holds them, with Gazebo's own
simulation time: "state <id> <base64 JSON {t_ns, positions, velocities}>". The controllers' joint
states are stamped with their nodes' ROS clock, which follows a BEST_EFFORT /clock and can lag a
step under load (29/10, test A); this reply cannot.

Loaded by tri_yolo.launch.py with state_reset:=true; GZ_SIM_SYSTEM_PLUGIN_PATH must hold this
directory.
"""
import base64
import json

from gz.math7 import Pose3d, Quaterniond, Vector3d
from gz.msgs10.stringmsg_pb2 import StringMsg
from gz.sim8 import Joint, Link, Model, World
from gz.transport13 import Node


ROBOT = 'mycobot_320'


class StateReset:
    def configure(self, entity, sdf, ecm, event_mgr):
        self.world = World(entity)
        name = self.world.name(ecm)
        self.node = Node()
        self.ack = self.node.advertise(f'/world/{name}/state_reset/ack', StringMsg)
        self.pending = None
        self.state_request = None
        self.checks_enabled = False
        self.seen = set()
        self.node.subscribe(StringMsg, f'/world/{name}/state_reset', self.on_request)

    def on_request(self, msg):
        request = json.loads(base64.b64decode(msg.data))
        # The sender repeats a request until it sees "received" (a single publish can be lost before
        # the subscription connects): a repeat is only acknowledged again, never applied twice.
        if request.get('state'):
            self.state_request = request
            return
        if request['id'] not in self.seen:
            self.seen.add(request['id'])
            self.pending = request
            self.remaining = int(request.get('repeat', 1))
        self.publish(f"received {request['id']}")

    def pre_update(self, info, ecm):
        # Joint position and velocity components exist only once asked for: enabled as soon as the
        # robot is there (start-up), so they are filled by the time a state is requested.
        if not self.checks_enabled:
            entity = self.world.model_by_name(ecm, ROBOT)
            if not entity:
                return
            for j in (Joint(e) for e in Model(entity).joints(ecm)):
                j.enable_position_check(ecm, True)
                j.enable_velocity_check(ecm, True)
            self.checks_enabled = True
            return
        request = self.state_request
        if request is None:
            return
        joints = [Joint(e) for e in Model(self.world.model_by_name(ecm, request['model'])).joints(ecm)]
        positions = {j.name(ecm): j.position(ecm) for j in joints}
        velocities = {j.name(ecm): j.velocity(ecm) for j in joints}
        moving = [n for n, v in positions.items() if v]
        if not moving:
            return
        state = {'t_ns': int(round(info.sim_time.total_seconds() * 1e9)),
                 'positions': {n: positions[n][0] for n in moving},
                 'velocities': {n: velocities[n][0] for n in moving if velocities[n]}}
        self.state_request = None
        self.publish(f"state {request['id']} " + base64.b64encode(json.dumps(state).encode()).decode())

    def publish(self, text):
        out = StringMsg()
        out.data = text
        self.ack.publish(out)

    def update(self, info, ecm):
        if info.paused or self.pending is None:
            return
        request = self.pending
        self.remaining -= 1
        if self.remaining <= 0:
            self.pending = None
        robot = Model(self.world.model_by_name(ecm, request['model']))
        for name, position in request['joints'].items():
            joint = Joint(robot.joint_by_name(ecm, name))
            joint.reset_position(ecm, [float(position)])
            joint.reset_velocity(ecm, [0.0])
            joint.set_velocity(ecm, [0.0])
        for name, (x, y, z, qw, qx, qy, qz) in request['poses'].items():
            model = Model(self.world.model_by_name(ecm, name))
            model.set_world_pose_cmd(ecm, Pose3d(Vector3d(x, y, z), Quaterniond(qw, qx, qy, qz)))
            link = Link(model.canonical_link(ecm))
            link.set_linear_velocity(ecm, Vector3d(0.0, 0.0, 0.0))
            link.set_angular_velocity(ecm, Vector3d(0.0, 0.0, 0.0))
        if self.pending is None:
            self.publish(f"applied {request['id']} {int(info.sim_time.total_seconds() * 1e9)}")


def get_system():
    return StateReset()
