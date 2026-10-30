#!/usr/bin/env python3
"""Reconstructed A/F-UNI/G precision trials in Gazebo; never connects to hardware.

B is evaluated from the ten A arrivals, as in the historical workbook.
The historical TCP is virtual in the current gripper geometry. No physical
metrology claim is made. All motion assumptions and observations are saved.
"""
from __future__ import annotations
import argparse
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import sys
import time
import xml.etree.ElementTree as ET
from zipfile import ZipFile

import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / 'scripts'))
from diff_ik import fk_pose

TARGET = np.array([330.476, 31.843, 60.0])
JOINTS = ['joint2_to_joint1', 'joint3_to_joint2', 'joint4_to_joint3',
          'joint5_to_joint4', 'joint6_to_joint5', 'joint6output_to_joint6']
GROUPS = [('A', 'haut', 10), ('F-UNI', 'haut', 6),
          ('G', 'devant', 10), ('G', 'derriere', 10),
          ('G', 'gauche', 10), ('G', 'droite', 10)]
OFFSETS = {'target': [0, 0, 0], 'haut': [0, 0, 40], 'devant': [40, 0, 0],
           'derriere': [-40, 0, 0], 'gauche': [0, 40, 0], 'droite': [0, -40, 0]}
DEFAULT_XLSX = Path('/home/genji/Downloads/precision_campagne_2026-09-09.xlsx')


def dump(path, value):
    Path(path).write_text(json.dumps(value, ensure_ascii=False, indent=2,
                                   default=lambda v: v.tolist(), allow_nan=False) + '\n')


def position(q_deg, tool):
    p, r = fk_pose(q_deg)
    return p + r @ tool


def model_masses(xml):
    """Inspect the expanded robot actually published, without counting an item twice."""
    root = ET.fromstring(xml)
    arm_names = {'base', *('link' + str(i) for i in range(1, 7))}
    groups = {'arm': {}, 'gripper': {}, 'other': {}}
    for link in root.findall('link'):
        inertial = link.find('inertial')
        if inertial is None:
            continue
        name = link.get('name')
        mass = float(inertial.find('mass').get('value'))
        if not math.isfinite(mass) or mass <= 0:
            raise ValueError(f'Invalid mass for {name}: {mass}')
        group = 'arm' if name in arm_names else ('gripper' if name.startswith('gripper_') else 'other')
        if name in groups[group]:
            raise ValueError(f'Duplicate inertial link: {name}')
        groups[group][name] = mass
    if set(groups['arm']) != arm_names or len(groups['gripper']) != 7:
        raise ValueError('Expected base + six arm links and seven gripper links')
    return {'links_kg': groups, 'arm_kg': sum(groups['arm'].values()),
            'gripper_kg': sum(groups['gripper'].values()),
            'other_kg': sum(groups['other'].values()),
            'arm_and_gripper_kg': sum(groups['arm'].values()) + sum(groups['gripper'].values())}


def check_requested_masses(actual, arm_kg, gripper_kg):
    for name, expected in (('arm_kg', arm_kg), ('gripper_kg', gripper_kg)):
        if not math.isfinite(expected) or expected <= 0:
            raise ValueError(f'Invalid requested {name}: {expected}')
        if not math.isclose(actual[name], expected, rel_tol=0., abs_tol=1e-9):
            raise ValueError(f'Loaded {name}={actual[name]} kg, requested {expected} kg; motion refused')


def metrics(points, target=TARGET):
    p = np.asarray(points, dtype=float)
    if p.ndim != 2 or p.shape[1] != 3 or len(p) < 2 or not np.isfinite(p).all():
        raise ValueError('At least two finite XYZ observations are required')
    center = p.mean(axis=0)
    distances = np.linalg.norm(p - center, axis=1)
    return {'n': len(p), 'centroid_mm': center.tolist(),
            'rp_mm': float(distances.mean() + 3 * distances.std(ddof=1)),
            'mean_error_mm': float(np.linalg.norm(p - target, axis=1).mean()),
            'bias_xyz_mm': (center - target).tolist()}


def summarize(groups):
    result = {name: metrics(points) for name, points in groups.items()}
    centers = np.array([result['G/' + d]['centroid_mm']
                        for d in ('devant', 'derriere', 'gauche', 'droite')])
    result['G_inter'] = {
        'bbox_diagonal_mm': float(np.linalg.norm(np.ptp(centers, axis=0))),
        'max_pairwise_mm': float(np.linalg.norm(centers[:, None] - centers[None, :], axis=2).max())}
    result['B'] = {'n': result['A/haut']['n'],
                   'mean_error_mm': result['A/haut']['mean_error_mm'],
                   'source': 'The same ten arrivals as A, not ten additional trials'}
    return result


def reference(path):
    ns = {'s': 'http://schemas.openxmlformats.org/spreadsheetml/2006/main'}
    with ZipFile(path) as z:
        strings = []
        if 'xl/sharedStrings.xml' in z.namelist():
            strings = [''.join(t.text or '' for t in si.findall('.//s:t', ns))
                       for si in ET.fromstring(z.read('xl/sharedStrings.xml')).findall('s:si', ns)]
        def rows(number):
            out = []
            for row in ET.fromstring(z.read(f'xl/worksheets/sheet{number}.xml')).findall('.//s:row', ns):
                values = {'_row': int(row.get('r'))}
                for c in row.findall('s:c', ns):
                    v = c.find('s:v', ns)
                    text = v.text if v is not None else ''.join(t.text or '' for t in c.findall('.//s:t', ns))
                    if c.get('t') == 's':
                        text = strings[int(text)]
                    values[''.join(x for x in c.get('r') if x.isalpha())] = text
                out.append(values)
            return out
        groups = {}
        for number, key, count in ((3, 'A/haut', 10), (4, 'F-UNI/haut', 6)):
            groups[key] = [[float(row[c]) for c in ('B', 'C', 'D')]
                           for row in rows(number) if row.get('A', '').isdigit()]
            if len(groups[key]) != count:
                raise ValueError(f'Unexpected workbook structure: {key}')
        names = {'depuis l avant': 'devant', 'depuis l arriere': 'derriere',
                 'depuis la gauche': 'gauche', 'depuis la droite': 'droite'}
        for label, direction in names.items():
            groups['G/' + direction] = [[float(row[c]) for c in ('C', 'D', 'E')]
                                        for row in rows(5)
                                        if 8 <= row['_row'] <= 47 and row.get('A') == label
                                        and row.get('B', '').isdigit()]
            if len(groups['G/' + direction]) != 10:
                raise ValueError(f'Unexpected workbook structure: {direction}')
    return {'source': str(path), 'sha256': hashlib.sha256(path.read_bytes()).hexdigest(),
            'points_mm': groups, 'metrics': summarize(groups)}


def prepare(output, xlsx, mass, arm_mass=3.0):
    output.mkdir(parents=True, exist_ok=True)
    if (output / 'samples.jsonl').exists():
        raise ValueError('Refusing to overwrite the plan of an existing measurement run')
    tool = np.array(json.loads((ROOT / 'scripts/tool_offset.json').read_text())['tool_offset_mm'])
    q_reference = [45.43, -86.57, -96.76, 135.52, 1.84, -43.33]
    azimuth = math.degrees(math.atan2(TARGET[1], TARGET[0]))
    axis = [-math.sin(math.radians(azimuth)), math.cos(math.radians(azimuth)), 0]
    rot = (Rotation.from_rotvec(np.array(axis) * math.radians(-15)).as_matrix()
           @ Rotation.from_euler('z', azimuth - 7.2, degrees=True).as_matrix()
           @ fk_pose(q_reference)[1])
    lower = np.degrees([-2.93, -2.35, -2.53, -2.53, -2.93, -3.14])
    upper = -lower
    poses = {}
    for name, offset in OFFSETS.items():
        target = TARGET + offset
        def residual(q):
            p, r = fk_pose(q)
            return np.r_[p + r @ tool - target,
                         Rotation.from_matrix(rot @ r.T).as_rotvec() * 400]
        candidates = []
        for seed in ([azimuth, -90, -60, 120, 0, 0],
                     [azimuth, -60, -40, 90, 0, 0], q_reference):
            sol = least_squares(residual, np.clip(seed, lower, upper), bounds=(lower, upper),
                                max_nfev=500, xtol=1e-11, ftol=1e-11, gtol=1e-11)
            if sol.x[2] <= 0:
                candidates.append(sol)
        if not candidates:
            raise ValueError(f'No elbow-up solution: {name}')
        sol = min(candidates, key=lambda s: np.linalg.norm(s.fun))
        if np.linalg.norm(sol.fun) > 0.05:
            raise ValueError(f'IK residual too large: {name}, {sol.fun}')
        poses[name] = {'q_deg': sol.x.tolist(), 'target_mm': target.tolist(),
                       'ik_position_error_mm': float(np.linalg.norm(position(sol.x, tool) - target)),
                       'model_pad_position_mm': position(sol.x, np.array([-1., 7.8, 166.])).tolist()}
    plan = {'target_mm': TARGET.tolist(), 'historical_tcp_mm': tool.tolist(),
            'orientation_matrix': rot.tolist(), 'inclination_deg': -15,
            'arm_mass_kg': arm_mass, 'gripper_mass_kg': mass,
            'arm_and_gripper_mass_kg': arm_mass + mass,
            'mass_distribution': 'All seven arm masses and inertias scaled by arm_mass_kg/1.58; original COMs retained, not measured.',
            'poses': poses, 'groups': GROUPS,
            'max_joint_velocity_deg_s': 30.0, 'minimum_move_duration_s': 2.0,
            'settle_minimum_s': 1.0, 'settle_maximum_s': 4.0,
            'settle_window_s': 0.25, 'settle_velocity_rad_s': 0.005,
            'settle_range_deg': 0.01, 'encoder_rounding_deg': 0.01,
            'physics_step_s': 0.001, 'requested_real_time_factor': 2.0,
            'gripper_close_command_rad': [-1.02, 1.02, 1.02, -1.02, -1.02, 1.02],
            'limitations': [
                'Reconstructed procedure, not an exact replay: historical speeds, timings and joint commands are missing.',
                'The historical FK TCP differs from the geometric pad centre of the current adaptive-gripper meshes.',
                f'Nominal arm mass {arm_mass} kg + gripper {mass} kg; individual mass distribution, COMs, inertias and servos are not identified.',
                'Rigid position-controlled model, without identified backlash or compliance.',
                'No camera, no object, no Cartesian correction. B reuses the ten A arrivals.',
                'One point, reduced sample counts: not an ISO conformity assessment.',
                'Gripper closure is limited by the existing simulated geometry, not a calibrated real jaw opening.'
            ],
            'source_sha256': {p: hashlib.sha256((ROOT / p).read_bytes()).hexdigest()
                              for p in ['scripts/tool_offset.json', 'scripts/diff_ik.py',
                                        'training/dream/mycobot_fk.py',
                                        'mycobot_description/urdf/320_pi/mycobot_pro_320_pi_gazebo.urdf',
                                        'mycobot_description/config/controller.yaml']}}
    dump(output / 'plan.json', plan)
    dump(output / 'reference.json', reference(xlsx))
    world = ET.parse(ROOT / 'mycobot_description/worlds/real_table.sdf')
    w = world.getroot().find('world')
    for model in list(w.findall('model')):
        if model.get('name') in ('red_cube', 'red_bin') or model.findall('.//sensor'):
            w.remove(model)
    w.find('physics/real_time_factor').text = '2.0'
    world.write(output / 'world.sdf', encoding='utf-8', xml_declaration=True)
    config = (ROOT / 'mycobot_description/config/trajectory_gui.config').read_text()
    config = config.replace('Trajectoire pince — bleu', 'TCP campagne reconstruit — bleu')
    config = config.replace('<offset>-0.001 0.0078 0.166</offset>',
                            '<offset>' + ' '.join(str(v / 1000) for v in tool) + '</offset>')
    (output / 'gui.config').write_text(config)
    dump(output / 'status.json', {'complete': False, 'prepared': True, 'arrivals': 0})
    print(f'Prepared 56 arrivals / 6 series, arm {arm_mass} + gripper {mass} = {arm_mass+mass:.3f} kg. Output: {output}', flush=True)


def run(output, finish_return=False):
    import rclpy
    from rclpy.action import ActionClient
    from rclpy.node import Node
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Float64MultiArray, String
    from trajectory_msgs.msg import JointTrajectoryPoint
    from control_msgs.action import FollowJointTrajectory
    from controller_manager_msgs.srv import ListControllers
    from rclpy.qos import QoSProfile, DurabilityPolicy

    plan = json.loads((output / 'plan.json').read_text())
    tool = np.array(plan.get('tcp_mm', plan.get('historical_tcp_mm')))
    target = np.array(plan['target_mm'])
    vertical = plan.get('protocol') == 'vertical_gripper_down'
    # Require an isolated launcher, not the default ROS graph used by hardware.
    if not os.environ.get('GZ_PARTITION', '').startswith('mycobot_precision_'):
        raise RuntimeError('Use tester_precision_gazebo.sh (isolated simulation required).')
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true'])

    class Campaign(Node):
        def __init__(self):
            super().__init__('gazebo_precision_campaign')
            self.q = None
            self.velocity = None
            self.stamp = None
            self.last_wall = 0
            self.last_trace = -1
            self.series = 'initialization'
            self.history = []
            self.samples = []
            self.actual_masses = None
            self.trace_file = (output / 'trajectory.csv').open('a' if finish_return else 'w')
            self.trace = csv.writer(self.trace_file)
            if not finish_return:
                self.trace.writerow(['sim_s', 'series', 'x_mm', 'y_mm', 'z_mm', *JOINTS])
            self.sub = self.create_subscription(JointState, '/joint_states', self.state, 20)
            self.robot_xml = self.create_subscription(
                String, '/robot_description', self.inspect_robot,
                QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
            self.arm = ActionClient(self, FollowJointTrajectory, '/mycobot_controller/follow_joint_trajectory')
            self.controllers = self.create_client(ListControllers, '/controller_manager/list_controllers')
            self.gripper = self.create_publisher(Float64MultiArray, '/gripper_position_controller/commands', 10)

        def inspect_robot(self, message):
            (output / 'robot.urdf').write_text(message.data)
            xml = ET.fromstring(message.data)
            expected_mode = plan.get('control_mode', 'position')
            interfaces = {j.attrib['name']: [c.attrib['name'] for c in j.findall('command_interface')]
                          for j in xml.find('ros2_control').findall('joint')}
            if any(interfaces.get(j) != [expected_mode] for j in JOINTS):
                raise RuntimeError('Loaded arm command interfaces do not match the planned control mode')
            actual = model_masses(message.data)
            dump(output / 'loaded_masses.json', actual)
            check_requested_masses(actual, plan.get('arm_mass_kg', 1.58), plan['gripper_mass_kg'])
            if self.actual_masses is None:
                self.get_logger().info(f"Masses verifiees AVANT mouvement : bras {actual['arm_kg']:.3f} kg + "
                                       f"pince {actual['gripper_kg']:.3f} kg = {actual['arm_and_gripper_kg']:.3f} kg")
            self.actual_masses = actual

        def now(self):
            return self.get_clock().now().nanoseconds / 1e9

        def state(self, msg):
            values = dict(zip(msg.name, msg.position))
            velocities = dict(zip(msg.name, msg.velocity))
            if not all(j in values and j in velocities for j in JOINTS):
                return
            stamp = msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9
            q = np.degrees([values[j] for j in JOINTS])
            v = np.array([velocities[j] for j in JOINTS])
            if not np.isfinite(q).all() or not np.isfinite(v).all() or stamp == self.stamp:
                return
            self.q, self.velocity, self.stamp = q, v, stamp
            self.last_wall = time.monotonic()
            self.history.append((stamp, q.copy(), v.copy()))
            self.history = [h for h in self.history if h[0] >= stamp - 1]
            if stamp - self.last_trace >= 0.1:
                self.trace.writerow([stamp, self.series, *position(q, tool), *q])
                self.last_trace = stamp

        def spin(self):
            if not rclpy.ok():
                raise RuntimeError('ROS stopped')
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.q is not None and time.monotonic() - self.last_wall > 45:
                raise RuntimeError('No fresh joint states for 45 wall seconds')

        def await_future(self, future, wall_timeout=20):
            deadline = time.monotonic() + wall_timeout
            while not future.done():
                self.spin()
                if time.monotonic() > deadline:
                    raise RuntimeError('Service/action timeout')
            return future.result()

        def startup(self):
            deadline = time.monotonic() + 120
            while time.monotonic() < deadline:
                self.spin()
                if self.controllers.service_is_ready():
                    response = self.await_future(self.controllers.call_async(ListControllers.Request()))
                    active = {c.name for c in response.controller if c.state == 'active'}
                    if (self.actual_masses is not None and self.q is not None and self.now() > 0 and self.arm.server_is_ready()
                            and {'mycobot_controller', 'gripper_position_controller'} <= active
                            and self.gripper.get_subscription_count() > 0):
                        return
                time.sleep(0.1)
            raise RuntimeError('Gazebo controllers unavailable')

        def wait_sim(self, duration):
            end = self.now() + duration
            while self.now() < end:
                self.spin()

        def move(self, q, initial=False, path=None):
            goal = FollowJointTrajectory.Goal()
            goal.trajectory.joint_names = JOINTS
            start = JointTrajectoryPoint()
            start.positions = np.radians(self.q).tolist()
            start.velocities = [0.] * 6
            duration = max(2., 1.5 * float(np.max(np.abs(self.q - q))) / 30., 8. if initial else 0.)
            end = JointTrajectoryPoint()
            end.positions = np.radians(q).tolist()
            end.velocities = [0.] * 6
            end.time_from_start.sec = int(duration)
            end.time_from_start.nanosec = int((duration % 1) * 1e9)
            goal.trajectory.points = [start, end]
            if path is not None:
                positions = np.radians(np.array(path))
                positions[0] = np.radians(self.q)
                duration = plan['vertical_segment_duration_s']
                times = np.linspace(0., duration, len(positions))
                velocities = np.gradient(positions, times, axis=0)
                velocities[[0, -1]] = 0.
                goal.trajectory.points = []
                for pos, vel, stamp in zip(positions, velocities, times):
                    point = JointTrajectoryPoint()
                    point.positions = pos.tolist()
                    point.velocities = vel.tolist()
                    point.time_from_start.sec = int(stamp)
                    point.time_from_start.nanosec = int(round((stamp % 1) * 1e9))
                    goal.trajectory.points.append(point)
            goal.goal_time_tolerance.sec = 2
            handle = self.await_future(self.arm.send_goal_async(goal))
            if not handle.accepted:
                raise RuntimeError('Simulated controller rejected goal')
            result = handle.get_result_async()
            timeout = self.now() + duration + 6
            while not result.done():
                self.spin()
                if self.now() > timeout:
                    self.await_future(handle.cancel_goal_async())
                    raise RuntimeError('Trajectory did not return a result')
            response = result.result().result
            return {'duration_s': duration, 'action_code': response.error_code,
                    'action_message': response.error_string}

        def settle(self):
            start = self.now()
            self.wait_sim(plan['settle_minimum_s'])
            while True:
                self.spin()
                window = [h for h in self.history if h[0] >= self.stamp - plan['settle_window_s']]
                covered = len(window) >= 8 and window[-1][0] - window[0][0] >= 0.20
                stable = (covered and np.max(np.abs([h[2] for h in window])) < plan['settle_velocity_rad_s']
                          and np.max(np.ptp([h[1] for h in window], axis=0)) < plan['settle_range_deg'])
                if stable or self.now() - start >= plan['settle_maximum_s']:
                    return bool(stable), self.now() - start

        def final_return(self):
            self.series = 'A-vertical/haut'
            motion = self.move(np.array(plan['poses']['haut']['q_deg']),
                               path=plan['vertical_waypoints_up_deg'])
            stable, duration = self.settle()
            dump(output / 'final_return.json', {'motion': motion, 'settled': stable,
                                               'settle_time_s': duration, 'sim_s': self.stamp})
            if motion['action_code'] != 0 or not stable:
                raise RuntimeError('Final upward return failed')

        def execute(self):
            self.startup()
            self.gripper.publish(Float64MultiArray(data=plan['gripper_close_command_rad']))
            self.wait_sim(2)
            target_q = np.array(plan['poses']['target']['q_deg'])
            self.move(np.array(plan['poses']['haut']['q_deg']) if vertical else target_q, initial=True)
            self.settle()
            for series, direction, count in plan['groups']:
                self.series = series + '/' + direction
                departure = np.array(plan['poses'][direction]['q_deg'])
                for trial in range(1, count + 1):
                    away = ({'duration_s': 0., 'action_code': 0} if vertical and trial == 1 else
                            self.move(departure, path=plan['vertical_waypoints_up_deg'] if vertical else None))
                    self.wait_sim(0.25)
                    motion = self.move(target_q, path=plan['vertical_waypoints_up_deg'][::-1] if vertical else None)
                    stable, settle_time = self.settle()
                    q = self.q.copy()
                    rounded = np.round(q, 2)
                    sample = {'series': series, 'direction': direction, 'trial': trial,
                              'sim_s': self.stamp, 'q_cmd_deg': target_q.tolist(), 'q_read_deg': q.tolist(),
                              'q_read_rounded_deg': rounded.tolist(),
                              'xyz_fk_mm': position(q, tool).tolist(),
                              'xyz_fk_rounded_mm': position(rounded, tool).tolist(),
                              'model_pad_fk_mm': position(q, np.array([-1., 7.8, 166.])).tolist(),
                              'settled': stable, 'settle_time_s': settle_time,
                              'departure_motion': away, 'arrival_motion': motion}
                    self.samples.append(sample)
                    with (output / 'samples.jsonl').open('a') as f:
                        f.write(json.dumps(sample, allow_nan=False) + '\n')
                    self.trace_file.flush()
                    self.get_logger().info(f'{self.series} {trial}/{count}: '
                        f'erreur={np.linalg.norm(position(rounded, tool)-target):.4f} mm, stable={stable}')
            if vertical:
                self.final_return()
            return self.samples

    node = Campaign()
    try:
        if finish_return:
            node.samples = [json.loads(line) for line in (output / 'samples.jsonl').read_text().splitlines()]
            if not vertical or len(node.samples) != 10 or (output / 'final_return.json').exists():
                raise RuntimeError('Final return requires exactly ten vertical arrivals without a recorded return')
            node.startup()
            node.final_return()
            samples = node.samples
        elif (output / 'samples.jsonl').exists():
            raise RuntimeError('Output already has samples; use a new directory')
        else:
            samples = node.execute()
        dump(output / 'status.json', {'complete': True, 'arrivals': len(samples)})
    except BaseException as exc:
        dump(output / 'status.json', {'complete': False, 'error': str(exc), 'arrivals': len(node.samples)})
        raise
    finally:
        node.trace_file.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    report(output)


def report(output):
    samples = [json.loads(line) for line in (output / 'samples.jsonl').read_text().splitlines()]
    plan = json.loads((output / 'plan.json').read_text())
    if plan.get('protocol') == 'vertical_gripper_down':
        from gazebo_vertical_repeatability import report_vertical
        return report_vertical(output, samples, plan)
    ref = json.loads((output / 'reference.json').read_text())
    groups, raw_groups = {}, {}
    for sample in samples:
        key = sample['series'] + '/' + sample['direction']
        groups.setdefault(key, []).append(sample['xyz_fk_rounded_mm'])
        raw_groups.setdefault(key, []).append(sample['xyz_fk_mm'])
    for series, direction, expected in GROUPS:
        if len(groups.get(series + '/' + direction, [])) != expected:
            raise ValueError('Incomplete campaign; no full comparison report can be produced')
    sim = summarize(groups)
    failures = [s for s in samples if not s['settled'] or s['arrival_motion']['action_code'] != 0
                or s['departure_motion']['action_code'] != 0]
    result = {'simulated_rounded': sim, 'simulated_unrounded': summarize(raw_groups),
              'real': ref['metrics'], 'flagged_arrivals': len(failures),
              'limitations': plan['limitations']}
    actual = model_masses((output / 'robot.urdf').read_text())
    check_requested_masses(actual, plan.get('arm_mass_kg', 1.58), plan['gripper_mass_kg'])
    result['loaded_masses'] = actual
    dump(output / 'comparison.json', result)
    lines = ['# Comparaison Gazebo / campagne réelle du 9 septembre', '',
             f"Masses du modèle chargé : bras **{actual['arm_kg']:.3f} kg** + "
             f"pince **{actual['gripper_kg']:.3f} kg** = **{actual['arm_and_gripper_kg']:.3f} kg**. "
             'La répartition entre segments et leurs centres de masse restent des hypothèses.', '',
             f'{len(samples)} arrivées simulées ; pince nominale {plan["gripper_mass_kg"]*1000:.0f} g ; '
             f'{len(failures)} arrivées signalées (stabilisation ou action).', '',
             'Les positions comparées sont reconstruites par FK, avec angles simulés arrondis à 0,01°.',
             'Les données non arrondies sont conservées dans `comparison.json` et `samples.jsonl`.', '',
             '| Grandeur | Réel (mm) | Gazebo (mm) | Gazebo − réel (mm) |',
             '|---|---:|---:|---:|']
    table = []
    for key in groups:
        table.append(('RP ' + key, ref['metrics'][key]['rp_mm'], sim[key]['rp_mm']))
    table.extend([
        ('G — diagonale des barycentres', ref['metrics']['G_inter']['bbox_diagonal_mm'], sim['G_inter']['bbox_diagonal_mm']),
        ('G — distance maximale entre barycentres', ref['metrics']['G_inter']['max_pairwise_mm'], sim['G_inter']['max_pairwise_mm']),
        ('B — moyenne des erreurs à la cible', ref['metrics']['B']['mean_error_mm'], sim['B']['mean_error_mm'])])
    for title, real, simulated in table:
        lines.append(f'| {title} | {real:.6f} | {simulated:.6f} | {simulated-real:+.6f} |')
    lines += ['', '## Portée du résultat', '',
              f'Erreur B sans arrondir les angles : {result["simulated_unrounded"]["B"]["mean_error_mm"]:.3e} mm. '
              'La différence entre cette valeur et la colonne Gazebo provient de la quantification '
              'des angles à 0,01°. Un RP affiché à zéro ne prouve pas une précision physique parfaite.', '',
              'Cette exécution teste le modèle actuel avec une procédure reconstruite. '
              'Elle ne valide pas la conformité ISO ni la précision physique du robot réel.',
              'Une différence simulation/réel met en évidence les limites du modèle ou du rejeu ; '
              'elle ne réfute pas les observations réelles. Aucun paramètre n’a été ajusté sur les résultats.', '',
              '### Hypothèses et limites', ''] + ['- ' + x for x in plan['limitations']]
    (output / 'rapport.md').write_text('\n'.join(lines) + '\n')
    import matplotlib
    matplotlib.use('Agg')
    import matplotlib.pyplot as plt
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5), layout='constrained')
    labels = list(groups)
    index = np.arange(len(labels))
    axes[0].bar(index-.18, [ref['metrics'][k]['rp_mm'] for k in labels], .36, label='Réel')
    axes[0].bar(index+.18, [sim[k]['rp_mm'] for k in labels], .36, label='Gazebo')
    axes[0].set_xticks(index, labels, rotation=45, ha='right')
    axes[0].set_title('Répétabilité RP (mm)'); axes[0].legend()
    for axis, item in zip(axes[1:], [table[-3], table[-1]]):
        axis.bar(['Réel', 'Gazebo'], item[1:], color=['tab:blue', 'tab:orange'])
        axis.set_title(item[0] + ' (mm)')
        for i, val in enumerate(item[1:]):
            axis.text(i, val, f'{val:.3f}', ha='center', va='bottom')
        axis.set_ylim(0, max(item[1:])*1.2 + .01)
    fig.suptitle('Comparaison exploratoire — TCP historique reconstruit, modèle non calibré')
    fig.savefig(output / 'comparaison.png', dpi=160)
    plt.close(fig)
    with (output / 'trajectory.csv').open() as f:
        trace = list(csv.DictReader(f))
    fig = plt.figure(figsize=(9, 7), layout='constrained')
    axis = fig.add_subplot(projection='3d')
    for group in dict.fromkeys(row['series'] for row in trace):
        if group == 'initialization':
            continue
        points = np.array([[float(row[k]) for k in ('x_mm', 'y_mm', 'z_mm')]
                           for row in trace if row['series'] == group])
        axis.plot(*points.T, label=group, linewidth=1)
    axis.scatter(*[[v] for v in TARGET], color='black', s=45, label='Cible')
    axis.set(xlabel='X (mm)', ylabel='Y (mm)', zlabel='Z (mm)',
             title='Trajectoires enregistrées — TCP historique reconstruit\n'
                   f'Gazebo, {plan["gripper_mass_kg"]*1000:.0f} g — hors mouvement initial')
    axis.legend(loc='upper left')
    axis.view_init(elev=28, azim=-65)
    fig.savefig(output / 'trajectoires.png', dpi=160)
    plt.close(fig)
    print('\n'.join(lines[:20]), flush=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('mode', choices=['prepare', 'run', 'report', 'finish-return'])
    parser.add_argument('--output', required=True, type=Path)
    parser.add_argument('--xlsx', type=Path, default=DEFAULT_XLSX)
    parser.add_argument('--mass', type=float, default=.34)
    parser.add_argument('--arm-mass', type=float, default=3.0)
    parser.add_argument('--vertical', action='store_true')
    parser.add_argument('--control-mode', choices=['position', 'effort'], default='position')
    args = parser.parse_args()
    if not math.isfinite(args.mass) or args.mass <= 0:
        parser.error('--mass must be positive and finite')
    if not math.isfinite(args.arm_mass) or args.arm_mass <= 0:
        parser.error('--arm-mass must be positive and finite')
    if args.mode == 'prepare':
        if args.vertical:
            from gazebo_vertical_repeatability import prepare_vertical
            prepare_vertical(args.output, args.xlsx, args.mass, args.arm_mass)
        else:
            prepare(args.output, args.xlsx, args.mass, args.arm_mass)
        if args.control_mode == 'effort':
            plan = json.loads((args.output / 'plan.json').read_text())
            plan['control_mode'] = 'effort'
            plan['limitations'] = [s for s in plan['limitations'] if 'position-controlled' not in s]
            plan['limitations'].append('Experimental torque PD control; gains, torque limits, friction and inertias are not identified on the real robot. No noise fitted to match real RP.')
            config = ROOT / 'mycobot_description/config/controller_effort.yaml'
            (args.output / 'controller_effort.yaml').write_text(config.read_text())
            dump(args.output / 'plan.json', plan)
    elif args.mode in ('run', 'finish-return'):
        run(args.output, finish_return=args.mode == 'finish-return')
    else:
        report(args.output)


if __name__ == '__main__':
    main()
