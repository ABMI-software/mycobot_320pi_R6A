"""Persistent, serialized controls for the Gazebo cube/bin scene."""

import json
import re
import subprocess

import numpy as np
import rclpy
from geometry_msgs.msg import PointStamped
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
from std_srvs.srv import Trigger

from .sim_sorting_grasp import SimSortingGrasp, APPROACH_Z, TRANSIT_Z
from .vision.sim_multicam_geometry import sample_scene_xy


class SimSceneControl(SimSortingGrasp):
    def __init__(self):
        super().__init__()
        if self.world != 'real_table' or self.pose_source != 'vision':
            raise ValueError('scene controls require world_name:=real_table and pose_source:=vision')
        self.declare_parameter('autostart', False)
        self.pending = 'pick' if self.get_parameter('autostart').value else None
        self.busy = self.pending is not None
        self.can_pick = True
        self.message = 'Démarrage du cycle…' if self.busy else 'Prêt. Randomiser ou lancer la prise.'
        self.rng = np.random.default_rng()
        self.target = next(t for t in self.targets if t.model == 'red_cube')
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub_scene = self.create_publisher(String, '/real_table/control_state', qos)
        self.pub_bin = self.create_publisher(PointStamped, '/real_table/bin_position', qos)
        self.create_service(Trigger, '/real_table/randomize',
                            lambda req, res: self.request_action('randomize', res))
        self.create_service(Trigger, '/real_table/pick',
                            lambda req, res: self.request_action('pick', res))
        self.create_timer(1.0, self.publish_state)
        self.publish_bin()
        self.publish_state()

    def publish_bin(self):
        msg = PointStamped()
        msg.header.frame_id = 'world'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.point.x, msg.point.y = map(float, self.target.bin_xy)
        self.pub_bin.publish(msg)

    def publish_state(self):
        self.pub_scene.publish(String(data=json.dumps({
            'busy': self.busy, 'can_pick': self.can_pick,
            'message': self.message, 'bin_xy': list(self.target.bin_xy),
        }, ensure_ascii=False)))

    def status(self, text):
        super().status(text)
        self.message = text
        self.publish_state()

    def request_action(self, action, response):
        if self.busy:
            response.success = False
            response.message = 'Un mouvement est déjà en cours. Attendre sa fin.'
        elif action == 'pick' and not self.can_pick:
            response.success = False
            response.message = 'Randomiser la scène avant une nouvelle prise.'
        else:
            self.busy = True
            self.pending = action
            self.message = ('Préparation des nouvelles positions…' if action == 'randomize'
                            else 'Localisation et prise du cube…')
            self.publish_state()
            response.success = True
            response.message = self.message
        return response

    def randomize(self):
        self.wait_until_ready()
        # Empty and raise the gripper before resetting either model.
        self.open_gripper()
        park = self.solve_tip([.25, 0., TRANSIT_Z])
        if park is None or not self.transit(park, 'préparation de la scène'):
            raise RuntimeError('Impossible de dégager la pince avant randomisation')
        self.spin_for(.5)
        self.status('Recherche de deux positions accessibles…')
        for _ in range(12):
            cube_xy, bin_xy = sample_scene_xy(self.rng)
            pick, _ = self.solve_column(*cube_xy, [APPROACH_Z, .018, TRANSIT_Z])
            if pick is None:
                continue
            place, _ = self.solve_column(*bin_xy, [TRANSIT_Z, .023], q_ref=pick[-1])
            if (place is not None and self._path_clears_table(self.q_deg, pick[0])
                    and self._path_clears_table(pick[-1], place[0])):
                break
        else:
            raise RuntimeError('Aucune paire de positions accessible trouvée')
        self.status('Repositionnement du cube et du bac…')
        # One Gazebo request updates both objects. This is scene setup only;
        # the subsequent pick still relies on images and physical contacts.
        request = ' '.join(
            f'pose {{ name: "{name}" position {{ x: {xy[0]:.9f} y: {xy[1]:.9f} z: {z} }} '
            'orientation { w: 1 } }'
            for name, xy, z in (('red_cube', cube_xy, .020), ('red_bin', bin_xy, 0.)))
        result = subprocess.run([
            'gz', 'service', '-s', f'/world/{self.world}/set_pose_vector',
            '--reqtype', 'gz.msgs.Pose_V', '--reptype', 'gz.msgs.Boolean',
            '--timeout', '4000', '--req', request,
        ], capture_output=True, text=True, timeout=8)
        if result.returncode or not re.search(r'data:\s*true', result.stdout):
            raise RuntimeError('Gazebo n a pas confirmé le repositionnement')
        self.target.bin_xy = tuple(map(float, bin_xy))
        self.publish_bin()
        self.vision_pose = None
        self.spin_for(1.0)
        self.can_pick = True
        self.status('Cube et bac repositionnés. Cliquer sur « Lancer la prise ».')

    def process_pending(self):
        action, self.pending = self.pending, None
        if action is None:
            return
        try:
            self.can_pick = False
            if action == 'randomize':
                self.randomize()
            else:
                results = super().run()
                self.status(results.get('red_cube', 'Cycle sans résultat'))
        except Exception as exc:
            self.status(f'Échec : {exc}')
        finally:
            self.busy = False
            self.publish_state()


def main(args=None):
    rclpy.init(args=args)
    node = SimSceneControl()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=.1)
            node.process_pending()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
