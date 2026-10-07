"""YOLO against DREAM in the sorting scene (protocol, step 10).

YOLO says where the object is; DREAM says where the gripper is (the robot pose
seen by each camera, T_DREAM(t), its PnP on the FK of the joint angles of the
image instant; the tip is placed in the base frame through the camera's
calibrated pose, fused over the views with >= 5 keypoints). When the gripper
closes on an object both should coincide, so the dashboard compares them to
each other — no Gazebo ground truth here (that comparison is logged by
dream_fk_compare and yolo_gt_overlay):

  - per grasp: object according to YOLO, gripper according to DREAM, distance;
  - in time: distance from DREAM's gripper to the object YOLO gave the sorter,
    with the same distance from the gripper read on the joint encoders as reference;
  - with embed_gazebo, the Gazebo GUI window itself in the top-left cell (one
    window; the gripper paths are drawn in Gazebo by tri_trajectoires_gazebo).

    ros2 launch mycobot_gateway tri_yolo.launch.py dream:=true dashboard:=true

With log_dir, the per-grasp table is also written to <log_dir>/dream_vs_yolo.csv
(rewritten whenever a row changes: DREAM poses arrive ~1 s after their image).
"""

from collections import deque
import csv
import os
import re
from pathlib import Path
import subprocess
import sys

from ament_index_python.packages import get_package_share_directory
import numpy as np
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QWindow
from PyQt5.QtWidgets import (QApplication, QGridLayout, QHeaderView, QGroupBox, QLabel,
                             QPlainTextEdit,
                             QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget)
import pyqtgraph as pg
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from vision_msgs.msg import Detection3DArray

from .dream_fk_compare import PREFIXES
from .joint_history import JointHistory
from .sim_sorting_grasp import tool_tip
from .vision.sim_multicam_geometry import load_cameras

sys.path.insert(0, str(Path(os.path.realpath(__file__)).parents[2] / 'training' / 'dream'))
from mycobot_fk import forward_kinematics  # noqa: E402

CAMERAS = tuple(PREFIXES)
HISTORY_S = 60.0
RUN_POSES = 50000
MAX_JOINT_GAP_S = 0.2
STATUS_LINES = 200
# A PnP on 4 keypoints is ill-conditioned: the wild DREAM poses (up to 10^9 mm)
# come from it. The fused tip only keeps views with at least this many.
MIN_KP_FUSED = 5
FUSION_WINDOW_S = 1.0
# A DREAM tip farther than this from the base is not a pose the arm can reach
# (~0.4 m with the gripper): a physical bound, no ground truth used.
REACH_BOUND_M = 1.0
# WM_CLASS of the gz sim GUI window as xwininfo prints it (the client window,
# not the window manager's frame around it, which carries the same title).
GAZEBO_CLASS = '"gz-sim-gui" "Gazebo GUI"'
SAISIE = re.compile(r'^▶ (\S+) : saisie en \(([-+\d.]+), ([-+\d.]+)\)')
VERDICT = re.compile(r'^(\S+) \(essai \d+\) : (.*)$')


def flange_estimate(T_gt, T_dream, q):
    """(true flange, flange placed through T_DREAM) in the base frame, metres."""
    _, transforms = forward_kinematics(q)
    p = transforms[6][:3, 3]
    return p, dream_point(T_gt, T_dream, p)


def dream_point(T_gt, T_dream, p):
    """Where DREAM puts the base-frame point p, which is fixed to the robot.

    T_gt, T_dream: T_cam<-base. The camera sees p at T_dream @ p and truly sits at
    inv(T_gt), so DREAM places p at inv(T_gt) @ T_dream @ p.
    """
    return (np.linalg.inv(T_gt) @ T_dream @ np.array([*p, 1.0]))[:3]


def encoder_tip(q):
    """Gripper tip from the joint angles (rad), exactly as sim_sorting_grasp aims it."""
    return tool_tip(np.degrees(q))


class DreamTips:
    """The gripper tip as DREAM places it, per camera and fused (median of the views
    with >= MIN_KP_FUSED keypoints)."""

    def __init__(self, T_gt):
        self.T_gt = T_gt
        self.n_valid = {cam: {} for cam in T_gt}
        self.latest = {}
        # The whole run: a grasp's row reads its interval back at every redraw.
        self.history = deque(maxlen=RUN_POSES)

    def keypoints(self, cam, msg):
        table = self.n_valid[cam]
        table[round(msg.data[21] + msg.data[22] * 1e-9, 6)] = int(sum(msg.data[2:21:3]))
        while len(table) > 50:
            table.pop(next(iter(table)))

    def pose(self, cam, t, T_dream, q):
        """Keep this view's tip if it is usable; (true tip, DREAM tip or None)."""
        true = encoder_tip(q)
        est = dream_point(self.T_gt[cam], T_dream, true)
        if (self.n_valid[cam].get(round(t, 6), 0) >= MIN_KP_FUSED
                and np.linalg.norm(est) <= REACH_BOUND_M):
            self.latest[cam] = (t, est)
            self.history.append((t, est))
            return true, est
        return true, None

    def between(self, t0, t1):
        """(median tip, number of views) of the usable views imaged in [t0, t1], or (None, 0)."""
        views = [e for s, e in self.history if t0 <= s <= t1]
        return (np.median(views, axis=0), len(views)) if views else (None, 0)

    def fused(self, t):
        """(median tip, number of views) of the usable views around t, or (None, 0)."""
        recent = [e for s, e in self.latest.values() if abs(t - s) <= FUSION_WINDOW_S]
        return (np.median(recent, axis=0), len(recent)) if recent else (None, 0)


def pose_matrix(msg):
    p, o = msg.pose.position, msg.pose.orientation
    T = np.eye(4)
    T[:3, :3] = Rotation.from_quat([o.x, o.y, o.z, o.w]).as_matrix()
    T[:3, 3] = [p.x, p.y, p.z]
    return T


class DashboardNode(Node):
    def __init__(self):
        super().__init__('tri_dream_dashboard')
        desc = get_package_share_directory('mycobot_description')
        cameras = load_cameras(
            os.path.join(desc, 'urdf', '320_pi', 'mycobot_pro_320_pi_gazebo.urdf'),
            {'camera_layout': 'dream50k'})
        self.joints = JointHistory(HISTORY_S)
        self.tips = DreamTips({cam: np.linalg.inv(cameras[cam].world_from_optical)
                               for cam in CAMERAS})
        self.distances = deque()
        self.inference_ms, self.latency_ms = deque(maxlen=200), deque(maxlen=200)
        self.status = deque(maxlen=STATUS_LINES)
        self.status_count = 0
        self.grasps = []
        self.target = None
        self.yolo = None
        log_dir = self.declare_parameter('log_dir', '').value
        self.grasp_csv = Path(log_dir) / 'dream_vs_yolo.csv' if log_dir else None
        self.written_rows = None
        self.create_subscription(JointState, '/joint_states', self.joints.add, 50)
        self.create_subscription(String, '/pickplace/status', self.on_status, 50)
        self.create_subscription(Detection3DArray, '/yolo/objects_3d',
                                 lambda m: setattr(self, 'yolo', m), 10)
        for cam in CAMERAS:
            self.create_subscription(Float64MultiArray, f'{PREFIXES[cam]}/keypoints',
                                     lambda m, c=cam: self.on_keypoints(c, m), 10)
            self.create_subscription(PoseStamped, f'{PREFIXES[cam]}/pose',
                                     lambda m, c=cam: self.on_pose(c, m), 10)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def trim(series):
        while series and series[-1][0] - series[0][0] > HISTORY_S:
            series.popleft()

    def yolo_object(self, name):
        """Centre of `name` in the latest fused YOLO objects, or None."""
        for d in (self.yolo.detections if self.yolo is not None else []):
            if d.results[0].hypothesis.class_id == name:
                c = d.results[0].pose.pose.position
                return np.array([c.x, c.y, c.z])
        return None

    def on_status(self, msg):
        self.status.append(msg.data)
        self.status_count += 1
        text = msg.data.strip()
        saisie, verdict = SAISIE.match(text), VERDICT.match(text)
        if saisie:
            # The object position the sorter received from YOLO.
            name = saisie.group(1)
            xy = np.array([float(saisie.group(2)), float(saisie.group(3))])
            centre = self.yolo_object(name)
            self.target = (name, np.array([*xy, centre[2] if centre is not None else 0.0]))
            self.grasps.append({'name': name, 'yolo': xy, 'dream': None, 'dream_mm': np.nan,
                                'encoder_mm': np.nan, 'views': 0, 'verdict': 'en cours',
                                'still': [None, None]})
        elif text.startswith('pointe visee') and self.grasps and self.joints.qs:
            # The gripper has reached the object and stays on it until it is held.
            g = self.grasps[-1]
            g['still'][0] = self.joints.times[-1]
            g['encoder_mm'] = np.linalg.norm(encoder_tip(self.joints.qs[-1])[:2] - g['yolo']) * 1000
        elif text.startswith('tenu a') and self.grasps and self.joints.qs:
            self.grasps[-1]['still'][1] = self.joints.times[-1]
        elif verdict:
            self.target = None
            for g in reversed(self.grasps):
                if g['name'] == verdict.group(1):
                    g['verdict'] = 'OK' if verdict.group(2).startswith('OK') else verdict.group(2)
                    break

    def update_grasps(self):
        # DREAM poses arrive ~1 s after their image: recomputed at each call.
        for g in self.grasps:
            if None not in g['still']:
                g['dream'], g['views'] = self.tips.between(*g['still'])
                g['dream_mm'] = (np.nan if g['dream'] is None
                                 else np.linalg.norm(g['dream'][:2] - g['yolo']) * 1000)
        if self.grasp_csv is not None:
            self.write_grasps()

    def write_grasps(self):
        rows = [[g['name'], *(f'{v * 1000:.1f}' for v in g['yolo']),
                 *(('', '', '') if g['dream'] is None else (f'{v * 1000:.1f}' for v in g['dream'])),
                 '' if np.isnan(g['dream_mm']) else f"{g['dream_mm']:.1f}",
                 '' if np.isnan(g['encoder_mm']) else f"{g['encoder_mm']:.1f}",
                 g['views'], *('' if t is None else f'{t:.3f}' for t in g['still']), g['verdict']]
                for g in self.grasps]
        if rows == self.written_rows:
            return
        with open(self.grasp_csv, 'w', newline='') as fh:
            w = csv.writer(fh)
            w.writerow(['object', 'yolo_x_mm', 'yolo_y_mm', 'dream_x_mm', 'dream_y_mm', 'dream_z_mm',
                        'dream_yolo_xy_mm', 'encoder_yolo_xy_mm', 'dream_views',
                        'still_from_s', 'still_to_s', 'verdict'])
            w.writerows(rows)
        self.written_rows = rows

    def on_keypoints(self, cam, msg):
        self.tips.keypoints(cam, msg)
        self.inference_ms.append(msg.data[23])

    def on_pose(self, cam, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        self.latency_ms.append((self.now_s() - t) * 1000)
        q, gap = self.joints.at(t)
        if gap > MAX_JOINT_GAP_S:
            return
        _, est = self.tips.pose(cam, t, pose_matrix(msg), q)
        if est is None:
            return
        fused, _ = self.tips.fused(t)
        if self.target is not None:
            goal = self.target[1]
            self.distances.append((t, np.linalg.norm(fused - goal) * 1000,
                                   np.linalg.norm(encoder_tip(q) - goal) * 1000))
            self.trim(self.distances)


def fit_rows(table, rows):
    """Every column in the width and `rows` rows in the height, without scrolling."""
    table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
    table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
    table.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
    table.setMinimumHeight(table.horizontalHeader().sizeHint().height()
                           + rows * table.verticalHeader().defaultSectionSize()
                           + 2 * table.frameWidth())


def find_gazebo_window():
    """X11 id of the Gazebo GUI window, or None while it is not mapped."""
    tree = subprocess.run(['xwininfo', '-root', '-tree'], capture_output=True, text=True).stdout
    for line in tree.splitlines():
        if GAZEBO_CLASS in line:
            return int(line.split()[0], 16)
    return None


class DashboardWindow(QWidget):
    """With embed_gazebo, the Gazebo GUI window (its own X11 window, reparented
    here, still interactive) takes the top-left cell of the panel."""

    def __init__(self, node, embed_gazebo):
        super().__init__()
        self.node = node
        self.setWindowTitle('Tri : YOLO (objets) contre DREAM (pince)')
        self.grid = grid = QGridLayout(self)

        self.gazebo_slot = QLabel('En attente de la fenêtre Gazebo…' if embed_gazebo
                                  else 'Gazebo sans interface (headless)')
        self.gazebo_slot.setAlignment(Qt.AlignCenter)
        self.gazebo_box = QGroupBox('Gazebo : trajectoires de la pince (une couleur par objet, '
                                    'magenta = DREAM)')
        QVBoxLayout(self.gazebo_box).addWidget(self.gazebo_slot)
        grid.addWidget(self.gazebo_box, 0, 0)

        compare_box = QGroupBox('DREAM (pince) ↔ YOLO (objet)')
        self.grasp_table = QTableWidget(0, 7)
        self.grasp_table.setHorizontalHeaderLabels(
            ['objet', 'YOLO objet x, y', 'DREAM pince x, y, z', 'DREAM ↔ YOLO',
             'codeurs ↔ YOLO', 'images DREAM', 'tri'])
        self.grasp_table.setToolTip('Au serrage, la pince est sur l objet. Positions et écarts '
                                    'en mm, écarts dans le plan XY. codeurs = pince lue aux '
                                    'codeurs du robot, référence.')
        fit_rows(self.grasp_table, 4)
        self.distance_plot = pg.PlotWidget(labels={
            'left': 'pince → objet YOLO visé (mm, log)', 'bottom': 'temps simulé (s)'})
        self.distance_plot.setLogMode(y=True)
        self.distance_plot.getAxis('left').enableAutoSIPrefix(False)
        self.distance_plot.addLegend()
        self.dream_curve = self.distance_plot.plot(pen=pg.mkPen((255, 30, 230), width=2),
                                                   name='DREAM')
        self.encoder_curve = self.distance_plot.plot(
            pen=pg.mkPen((200, 200, 200), width=1, style=Qt.DashLine), name='codeurs')
        self.latency = QLabel()
        box = QVBoxLayout(compare_box)
        box.addWidget(self.grasp_table)
        box.addWidget(self.distance_plot)
        box.addWidget(self.latency)
        grid.addWidget(compare_box, 0, 1)

        status_box = QGroupBox('Journal du tri (/pickplace/status)')
        self.status = QPlainTextEdit()
        self.status.setReadOnly(True)
        QVBoxLayout(status_box).addWidget(self.status)
        grid.addWidget(status_box, 1, 0)
        yolo_box = QGroupBox('YOLO : objets fusionnés (4 caméras)')
        self.yolo_table = QTableWidget(0, 5)
        self.yolo_table.setHorizontalHeaderLabels(['classe', 'x mm', 'y mm', 'z mm', 'caméras'])
        fit_rows(self.yolo_table, 8)
        QVBoxLayout(yolo_box).addWidget(self.yolo_table)
        grid.addWidget(yolo_box, 1, 1)
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)
        grid.setRowStretch(0, 2)
        grid.setRowStretch(1, 1)

        self.shown_status = 0
        self.timer = QTimer()
        self.timer.timeout.connect(self.tick)
        self.timer.start(30)
        self.refresh = QTimer()
        self.refresh.timeout.connect(self.redraw)
        self.refresh.start(500)
        if embed_gazebo:
            self.seek = QTimer()
            self.seek.timeout.connect(self.embed_gazebo)
            self.seek.start(1000)

    def embed_gazebo(self):
        wid = find_gazebo_window()
        if wid is None:
            return
        self.seek.stop()
        container = QWidget.createWindowContainer(QWindow.fromWinId(wid), self.gazebo_box)
        self.gazebo_box.layout().replaceWidget(self.gazebo_slot, container)
        self.gazebo_slot.deleteLater()

    def tick(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)

    def redraw(self):
        n = self.node
        if n.distances:
            t, d_dream, d_enc = (np.array(c) for c in zip(*n.distances))
            self.dream_curve.setData(t, d_dream)
            self.encoder_curve.setData(t, d_enc)
        if n.inference_ms:
            self.latency.setText(f'DREAM, latence médiane : inférence {np.median(n.inference_ms):.0f} ms, '
                                 f'image → pose {np.median(n.latency_ms):.0f} ms')

        n.update_grasps()
        self.grasp_table.setRowCount(len(n.grasps))
        for i, g in enumerate(n.grasps):
            dream = '—' if g['dream'] is None else ', '.join(f'{v * 1000:.0f}' for v in g['dream'])
            cells = (g['name'], ', '.join(f'{v * 1000:.0f}' for v in g['yolo']), dream,
                     '—' if np.isnan(g['dream_mm']) else f"{g['dream_mm']:.1f}",
                     '—' if np.isnan(g['encoder_mm']) else f"{g['encoder_mm']:.1f}",
                     str(g['views']), g['verdict'])
            for col, v in enumerate(cells):
                self.grasp_table.setItem(i, col, QTableWidgetItem(v))

        detections = n.yolo.detections if n.yolo is not None else []
        self.yolo_table.setRowCount(len(detections))
        for i, d in enumerate(detections):
            c = d.results[0].pose.pose.position
            cells = (d.results[0].hypothesis.class_id, f'{c.x * 1000:.1f}', f'{c.y * 1000:.1f}',
                     f'{c.z * 1000:.1f}', f'{round(d.results[0].hypothesis.score * 4)}/4')
            for col, v in enumerate(cells):
                self.yolo_table.setItem(i, col, QTableWidgetItem(v))
        if n.status_count != self.shown_status:
            self.status.setPlainText('\n'.join(n.status))
            self.status.verticalScrollBar().setValue(self.status.verticalScrollBar().maximum())
            self.shown_status = n.status_count


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    embed = node.declare_parameter('embed_gazebo', False).value
    app = QApplication(sys.argv)
    window = DashboardWindow(node, embed)
    if embed:
        window.showMaximized()
    else:
        window.resize(1500, 900)
        window.show()
    try:
        app.exec_()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
