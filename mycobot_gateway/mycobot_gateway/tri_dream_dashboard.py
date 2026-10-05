"""YOLO + DREAM dashboard of the sorting scene — VALIDATION ONLY (protocol, step 10).

YOLO sorts: the sorter's status and the fused 3D objects, each against the
Gazebo ground truth. DREAM estimates the robot pose seen by each camera,
T_DREAM(t) (its PnP takes the FK of the joint angles of the image instant); it
is compared to T_GT, the camera's true pose (world = base, invariant I3):

  - flange trajectory: true = FK(q(t)); estimated = the same flange placed in the
    base frame through T_DREAM instead of T_GT;
  - dXYZ (mm) of the flange, rotation error (deg) of T_DREAM, latency (inference
    and image -> pose).

Reads /validation/gt/objects: nothing here feeds perception or planning (I4).

    ros2 launch mycobot_gateway tri_yolo.launch.py dream:=true dashboard:=true
"""

from collections import deque
import os
from pathlib import Path
import sys

from ament_index_python.packages import get_package_share_directory
import numpy as np
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtWidgets import (QApplication, QHeaderView, QGridLayout, QGroupBox, QLabel, QPlainTextEdit,
                             QTableWidget, QTableWidgetItem, QVBoxLayout, QWidget)
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from scipy.spatial.transform import Rotation
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray, String
from vision_msgs.msg import Detection3DArray

from .dream_fk_compare import PREFIXES, pose_error
from .joint_history import JointHistory
from .vision.sim_multicam_geometry import load_cameras

sys.path.insert(0, str(Path(os.path.realpath(__file__)).parents[2] / 'training' / 'dream'))
from mycobot_fk import forward_kinematics  # noqa: E402

CAMERAS = tuple(PREFIXES)
LABELS = {'synth_camera': 'avant', 'synth_camera_right': 'droite',
          'synth_camera_left': 'gauche', 'synth_camera_top': 'dessus'}
COLOURS = {'synth_camera': (230, 90, 60), 'synth_camera_right': (60, 160, 230),
           'synth_camera_left': (90, 200, 90), 'synth_camera_top': (220, 190, 50)}
HISTORY_S = 60.0
MAX_JOINT_GAP_S = 0.2
STATUS_LINES = 200


def flange_estimate(T_gt, T_dream, q):
    """(true flange, flange placed through T_DREAM) in the base frame, metres.

    T_gt, T_dream: T_cam<-base. The flange seen by the camera is T_dream @ p; the
    camera is truly at inv(T_gt), so DREAM puts the flange at inv(T_gt) @ T_dream @ p.
    """
    _, transforms = forward_kinematics(q)
    p = transforms[6][:, 3]
    return p[:3], (np.linalg.inv(T_gt) @ T_dream @ p)[:3]


class DashboardNode(Node):
    def __init__(self):
        super().__init__('tri_dream_dashboard')
        desc = get_package_share_directory('mycobot_description')
        cameras = load_cameras(
            os.path.join(desc, 'urdf', '320_pi', 'mycobot_pro_320_pi_gazebo.urdf'),
            {'camera_layout': 'dream50k'})
        self.T_gt = {cam: np.linalg.inv(cameras[cam].world_from_optical) for cam in CAMERAS}
        self.joints = JointHistory(HISTORY_S)
        self.true_path = deque()
        self.estimates = {cam: deque() for cam in CAMERAS}
        self.inference_ms = {cam: {} for cam in CAMERAS}
        self.n_valid = {cam: {} for cam in CAMERAS}
        self.status = deque(maxlen=STATUS_LINES)
        self.yolo, self.gt = None, None
        self.create_subscription(JointState, '/joint_states', self.on_joints, 50)
        self.status_count = 0
        self.create_subscription(String, '/pickplace/status', self.on_status, 50)
        self.create_subscription(Detection3DArray, '/yolo/objects_3d',
                                 lambda m: setattr(self, 'yolo', m), 10)
        self.create_subscription(Detection3DArray, '/validation/gt/objects',
                                 lambda m: setattr(self, 'gt', m), 10)
        for cam in CAMERAS:
            self.create_subscription(Float64MultiArray, f'{PREFIXES[cam]}/keypoints',
                                     lambda m, c=cam: self.on_keypoints(c, m), 10)
            self.create_subscription(PoseStamped, f'{PREFIXES[cam]}/pose',
                                     lambda m, c=cam: self.on_pose(c, m), 10)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_joints(self, msg):
        before = self.joints.times[-1] if self.joints.times else None
        self.joints.add(msg)
        if self.joints.times and self.joints.times[-1] != before:
            _, transforms = forward_kinematics(self.joints.qs[-1])
            self.true_path.append((self.joints.times[-1], transforms[6][:3, 3]))
            while self.true_path[-1][0] - self.true_path[0][0] > HISTORY_S:
                self.true_path.popleft()

    def on_status(self, msg):
        self.status.append(msg.data)
        self.status_count += 1

    def on_keypoints(self, cam, msg):
        t = round(msg.data[21] + msg.data[22] * 1e-9, 6)
        self.inference_ms[cam][t] = msg.data[23]
        self.n_valid[cam][t] = int(sum(msg.data[2:21:3]))
        for table in (self.inference_ms[cam], self.n_valid[cam]):
            while len(table) > 50:
                table.pop(next(iter(table)))

    def on_pose(self, cam, msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        q, gap = self.joints.at(t)
        if gap > MAX_JOINT_GAP_S:
            return
        p, o = msg.pose.position, msg.pose.orientation
        T_dream = np.eye(4)
        T_dream[:3, :3] = Rotation.from_quat([o.x, o.y, o.z, o.w]).as_matrix()
        T_dream[:3, 3] = [p.x, p.y, p.z]
        true, est = flange_estimate(self.T_gt[cam], T_dream, q)
        _, angle = pose_error(self.T_gt[cam], T_dream)
        key = round(t, 6)
        self.estimates[cam].append({
            't': t, 'true': true, 'est': est, 'd_mm': (est - true) * 1000, 'rot_deg': angle,
            'latency_ms': (self.now_s() - t) * 1000,
            'inference_ms': self.inference_ms[cam].get(key, np.nan),
            'n_valid': self.n_valid[cam].get(key, 0)})
        while self.estimates[cam][-1]['t'] - self.estimates[cam][0]['t'] > HISTORY_S:
            self.estimates[cam].popleft()

    def yolo_rows(self):
        """[(class, x mm, y mm, error to Gazebo mm)] of the fused YOLO objects."""
        truth = {}
        if self.gt is not None:
            for d in self.gt.detections:
                c = d.bbox.center.position
                truth[d.id] = np.array([c.x, c.y])
        rows = []
        for d in (self.yolo.detections if self.yolo is not None else []):
            name = d.results[0].hypothesis.class_id
            c = d.results[0].pose.pose.position
            err = (np.linalg.norm(np.array([c.x, c.y]) - truth[name]) * 1000
                   if name in truth else np.nan)
            rows.append((name, c.x * 1000, c.y * 1000, err))
        return rows


def fit_rows(table, rows):
    """Every column in the width and `rows` rows in the height, without scrolling."""
    table.horizontalHeader().setSectionResizeMode(QHeaderView.Stretch)
    table.setHorizontalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
    table.setVerticalScrollBarPolicy(Qt.ScrollBarAlwaysOff)
    table.setMinimumHeight(table.horizontalHeader().sizeHint().height()
                           + rows * table.verticalHeader().defaultSectionSize()
                           + 2 * table.frameWidth())


class DashboardWindow(QWidget):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle('Tri YOLO + pose DREAM — Gazebo, validation')
        grid = QGridLayout(self)

        view_box = QGroupBox('Trajectoire de la bride (repère base) : blanc = vraie, couleurs = DREAM')
        self.view = gl.GLViewWidget()
        self.view.setCameraPosition(distance=1.2, elevation=25, azimuth=-60)
        grid_item = gl.GLGridItem()
        grid_item.setSize(0.8, 0.8)
        grid_item.setSpacing(0.1, 0.1)
        self.view.addItem(grid_item)
        self.true_line = gl.GLLinePlotItem(color=(1, 1, 1, 1), width=2, antialias=True)
        self.view.addItem(self.true_line)
        self.cam_dots = {}
        for cam in CAMERAS:
            r, g, b = (v / 255 for v in COLOURS[cam])
            self.cam_dots[cam] = gl.GLScatterPlotItem(color=(r, g, b, 0.9), size=6)
            self.view.addItem(self.cam_dots[cam])
        self.objects = gl.GLScatterPlotItem(color=(1, 0.6, 0.1, 1), size=12)
        self.view.addItem(self.objects)
        legend = '   '.join(f'<span style="color:rgb{COLOURS[c]}">■ {LABELS[c]}</span>'
                           for c in CAMERAS)
        box = QVBoxLayout(view_box)
        box.addWidget(self.view, stretch=1)
        box.addWidget(QLabel(legend + '   <span style="color:rgb(255,153,25)">● objets YOLO</span>'))
        grid.addWidget(view_box, 0, 0)

        dream_box = QGroupBox('DREAM : T_DREAM contre T_GT, médianes sur 60 s '
                              '(écarts de la bride en mm, latence inférence/totale)')
        self.dream_table = QTableWidget(len(CAMERAS), 8)
        self.dream_table.setHorizontalHeaderLabels(
            ['poses', 'kp', 'dX', 'dY', 'dZ', '|dXYZ|', 'rot °', 'lat. ms'])
        self.dream_table.setToolTip('dX, dY, dZ, |dXYZ| : écart de la bride en mm ; rot : '
                                    'écart de rotation de T_DREAM ; lat. : inférence / image → pose')
        self.dream_table.setVerticalHeaderLabels([LABELS[c] for c in CAMERAS])
        fit_rows(self.dream_table, len(CAMERAS))
        # Log scale: one wild PnP (tens of metres) must not flatten the others.
        self.error_plot = pg.PlotWidget(labels={'left': '|dXYZ| bride (mm, log)',
                                                'bottom': 'temps simulé (s)'})
        self.error_plot.setLogMode(y=True)
        self.error_plot.getAxis('left').enableAutoSIPrefix(False)
        self.error_plot.addLegend()
        self.curves = {cam: self.error_plot.plot(pen=pg.mkPen(COLOURS[cam], width=2),
                                                 name=LABELS[cam]) for cam in CAMERAS}
        box = QVBoxLayout(dream_box)
        box.addWidget(self.dream_table)
        box.addWidget(self.error_plot)
        grid.addWidget(dream_box, 0, 1)

        yolo_box = QGroupBox('YOLO : objets fusionnés (4 caméras) contre Gazebo')
        self.yolo_table = QTableWidget(0, 4)
        self.yolo_table.setHorizontalHeaderLabels(['classe', 'x mm', 'y mm', 'écart Gazebo mm'])
        fit_rows(self.yolo_table, 8)
        QVBoxLayout(yolo_box).addWidget(self.yolo_table)
        grid.addWidget(yolo_box, 1, 1)
        status_box = QGroupBox('État du tri (/pickplace/status)')
        self.status = QPlainTextEdit()
        self.status.setReadOnly(True)
        QVBoxLayout(status_box).addWidget(self.status)
        grid.addWidget(status_box, 1, 0)
        grid.setColumnStretch(0, 1)
        grid.setColumnStretch(1, 1)

        self.shown_status = 0
        self.timer = QTimer()
        self.timer.timeout.connect(self.tick)
        self.timer.start(30)
        self.refresh = QTimer()
        self.refresh.timeout.connect(self.redraw)
        self.refresh.start(500)

    def tick(self):
        rclpy.spin_once(self.node, timeout_sec=0.0)

    def redraw(self):
        n = self.node
        if len(n.true_path) > 1:
            self.true_line.setData(pos=np.array([p for _, p in n.true_path]))
        for row, cam in enumerate(CAMERAS):
            est = list(n.estimates[cam])
            if est:
                self.cam_dots[cam].setData(pos=np.array([e['est'] for e in est]))
                d = np.array([e['d_mm'] for e in est])
                cells = [len(est), f"{np.median([e['n_valid'] for e in est]):.0f}/7",
                         *(f'{v:+.1f}' for v in np.median(d, axis=0)),
                         f'{np.median(np.linalg.norm(d, axis=1)):.1f}',
                         f"{np.median([e['rot_deg'] for e in est]):.2f}",
                         f"{np.nanmedian([e['inference_ms'] for e in est]):.0f}/"
                         f"{np.median([e['latency_ms'] for e in est]):.0f}"]
                self.curves[cam].setData([e['t'] for e in est],
                                         np.linalg.norm(d, axis=1))
            else:
                cells = [0, '—', '—', '—', '—', '—', '—', '—']
            for col, v in enumerate(cells):
                self.dream_table.setItem(row, col, QTableWidgetItem(str(v)))
        rows = n.yolo_rows()
        self.yolo_table.setRowCount(len(rows))
        for i, (name, x, y, err) in enumerate(rows):
            for col, v in enumerate((name, f'{x:.1f}', f'{y:.1f}',
                                     '—' if np.isnan(err) else f'{err:.1f}')):
                self.yolo_table.setItem(i, col, QTableWidgetItem(v))
        if n.yolo is not None and n.yolo.detections:
            self.objects.setData(pos=np.array([[d.results[0].pose.pose.position.x,
                                                d.results[0].pose.pose.position.y,
                                                d.results[0].pose.pose.position.z]
                                               for d in n.yolo.detections]))
        if n.status_count != self.shown_status:
            self.status.setPlainText('\n'.join(n.status))
            self.status.verticalScrollBar().setValue(self.status.verticalScrollBar().maximum())
            self.shown_status = n.status_count


def main(args=None):
    rclpy.init(args=args)
    node = DashboardNode()
    app = QApplication(sys.argv)
    window = DashboardWindow(node)
    window.resize(1500, 900)
    window.show()
    try:
        app.exec_()
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
