"""YOLO vs Gazebo ground truth, drawn like the real bench — VALIDATION ONLY.

For every camera: its image, cropped on the pieces it sees and enlarged, with
the yolo26 boxes and "class confidence" labels drawn by the real bench
dashboard (yolo26_dashboard.trace_detections), plus the XY error of every
piece's 3D estimate FROM THAT CAMERA against the Gazebo ground truth:
green = right class, orange = wrong class, red = not detected. No fusion
(protocol step 7). The four views go to a 2x2 mosaic shown in a Gazebo GUI
panel (config/tri_yolo_gui.config). Reads /validation/gt/objects, so nothing
here feeds perception or planning (invariant I4).

    /<camera>/image + /yolo/<camera>/detections + /yolo/<camera>/objects_3d
    + /validation/gt/objects -> /validation/yolo_gt/<camera>/image
                             -> /validation/yolo_gt/image   (2x2 mosaic)
                             -> csv_path (optional)

Not in the 3D view: TEXT markers do not exist under ogre2 ("Invalid Marker
type 7", 29/09), and the ogre1 GUI that has them did not run here.
"""

import csv
import os
from pathlib import Path
import sys

import cv2
from ament_index_python.packages import get_package_share_directory
from cv_bridge import CvBridge
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection3DArray

from .vision.sim_multicam_geometry import load_cameras

_SCRIPTS = Path(__file__).resolve().parents[2] / 'scripts'
if str(_SCRIPTS) not in sys.path:
    sys.path.insert(0, str(_SCRIPTS))

import yolo26_dashboard as y  # noqa: E402

# Farther than this in XY, a YOLO estimate is not the same piece: footprints
# are at least 30 mm apart (tri_scene.MIN_GAP), measured error is under 10 mm.
MATCH_M = 0.03
COLOURS = {'ok': (60, 170, 60), 'wrong_class': (0, 140, 255), 'missed': (40, 40, 220)}
VIEW_W, VIEW_H = 1280, 960
# Around the outermost piece centres, in source pixels: room for the bins' walls
# and their labels.
VIEW_MARGIN_PX = 45
LABEL_SCALE = 0.75
CSV_FIELDS = ('stamp', 'camera', 'object_id', 'yolo_class', 'confidence', 'verdict',
              'x_gt', 'y_gt', 'x_yolo', 'y_yolo', 'error_xy_mm')
# Raw images kept to find the one a detection was computed on (~2 s of camera).
IMAGE_CACHE = 8
CAMERAS = ('synth_camera', 'synth_camera_right', 'synth_camera_left', 'synth_camera_top')
TILE_W, TILE_H = VIEW_W // 2, VIEW_H // 2


def match(gt, yolo, gate=MATCH_M):
    """Greedy nearest in XY. gt: {name: (xyz, height)}, yolo: [(class, conf, xyz)].

    Returns {name: (class, conf, xyz, error_m) or None} and the unmatched YOLO list.
    """
    pairs = sorted((float(np.hypot(*(np.asarray(est[2][:2]) - g[0][:2]))), name, i)
                   for name, g in gt.items() for i, est in enumerate(yolo))
    result, used = {name: None for name in gt}, set()
    for dist, name, i in pairs:
        if dist > gate or result[name] is not None or i in used:
            continue
        used.add(i)
        result[name] = (*yolo[i], dist)
    return result, [est for i, est in enumerate(yolo) if i not in used]


def verdict(name, found):
    if found is None:
        return 'missed'
    return 'ok' if found[0] == name else 'wrong_class'


def label(name, found):
    if found is None:
        return f'{name} NON DETECTE'
    err = f'{found[3] * 1000:.1f} mm'
    return err if found[0] == name else f'{err} GT {name}'


def summary_line(gt, matches, extra):
    ok = [m for n, m in matches.items() if verdict(n, m) == 'ok']
    errors = [m[3] * 1000 for m in matches.values() if m is not None]
    err = f'med {np.median(errors):.1f} mm max {max(errors):.1f} mm' if errors else 'aucune'
    return f'vs GT : {len(ok)}/{len(gt)} classe juste | XY {err} | {len(extra)} en trop'


def visible(camera, gt):
    """The pieces whose centre projects inside this camera's image."""
    out = {}
    for name, (xyz, height) in gt.items():
        try:
            u, v = camera.project(xyz + [0.0, 0.0, height / 2])[0]
        except ValueError:
            continue
        if 0 <= u < camera.width and 0 <= v < camera.height:
            out[name] = (xyz, height)
    return out


def mosaic(views, cameras):
    """2x2 of the latest views, in `cameras` order; a grey tile until a view exists."""
    tiles = []
    for cam in cameras:
        view = views.get(cam)
        if view is None:
            tile = np.full((TILE_H, TILE_W, 3), 60, np.uint8)
            y._pastille(tile, 10, 10, f'{cam} : en attente', (90, 90, 90), 0.5)
        else:
            tile = cv2.resize(view, (TILE_W, TILE_H), interpolation=cv2.INTER_AREA)
        tiles.append(tile)
    while len(tiles) < 4:
        tiles.append(np.full((TILE_H, TILE_W, 3), 60, np.uint8))
    return np.vstack([np.hstack(tiles[:2]), np.hstack(tiles[2:4])])


def view_window(camera, gt):
    """(x0, y0, scale): 4:3 crop of the source image around the pieces."""
    if not gt:
        return 0.0, 0.0, VIEW_W / camera.width
    uv = camera.project(np.array([xyz + [0.0, 0.0, h / 2] for xyz, h in gt.values()]))
    x0, y0 = uv.min(axis=0) - VIEW_MARGIN_PX
    x1, y1 = uv.max(axis=0) + VIEW_MARGIN_PX
    w = max(x1 - x0, (y1 - y0) * VIEW_W / VIEW_H)
    w = min(w, camera.width, camera.height * VIEW_W / VIEW_H)
    h = w * VIEW_H / VIEW_W
    cx, cy = (x0 + x1) / 2, (y0 + y1) / 2
    x0 = min(max(cx - w / 2, 0), camera.width - w)
    y0 = min(max(cy - h / 2, 0), camera.height - h)
    return x0, y0, VIEW_W / w


def crop(image, window):
    x0, y0, scale = window
    M = np.array([[scale, 0, -x0 * scale], [0, scale, -y0 * scale]])
    return cv2.warpAffine(image, M, (VIEW_W, VIEW_H), flags=cv2.INTER_CUBIC)


def to_view(u, v, window):
    x0, y0, scale = window
    return (u - x0) * scale, (v - y0) * scale


def detections_in_view(msg, window):
    out = []
    for d in msg.detections:
        c, sx, sy = d.bbox.center.position, d.bbox.size_x / 2, d.bbox.size_y / 2
        a, b = to_view(c.x - sx, c.y - sy, window)
        e, f = to_view(c.x + sx, c.y + sy, window)
        out.append({'classe': d.results[0].hypothesis.class_id,
                    'conf': d.results[0].hypothesis.score, 'boite': (a, b, e, f)})
    return out


def draw_errors(image, camera, gt, matches, extra, window, echelle=LABEL_SCALE):
    """Error pastille under the projected ground-truth centre of every piece."""
    h, w = image.shape[:2]
    for name in sorted(gt):
        xyz, height = gt[name]
        found = matches[name]
        u, v = to_view(*camera.project(xyz + [0.0, 0.0, height / 2])[0], window)
        colour = COLOURS[verdict(name, found)]
        cv2.drawMarker(image, (int(u), int(v)), colour, cv2.MARKER_CROSS, 16, 3)
        text = label(name, found)
        (tw, th), _ = cv2.getTextSize(text, y._POLICE, echelle, 1)
        y._pastille(image, int(min(max(u - tw / 2, 0), w - tw - 8)),
                    int(min(v + 12, h - th - 16)), text, colour, echelle)
    (_, th), _ = cv2.getTextSize('X', y._POLICE, echelle, 1)
    y._pastille(image, 10, h - th - 22, summary_line(gt, matches, extra), (60, 60, 60), echelle)


class YoloGtOverlay(Node):
    def __init__(self):
        super().__init__('yolo_gt_overlay')
        self.declare_parameter('cameras', list(CAMERAS))
        self.declare_parameter('camera_layout', 'dream50k')
        self.declare_parameter('csv_path', '')
        self.cams = list(self.get_parameter('cameras').value)
        desc = get_package_share_directory('mycobot_description')
        cameras = load_cameras(
            os.path.join(desc, 'urdf', '320_pi', 'mycobot_pro_320_pi_gazebo.urdf'),
            {'camera_layout': str(self.get_parameter('camera_layout').value)})
        self.cameras = {cam: cameras[cam] for cam in self.cams}
        self.bridge = CvBridge()
        self.gt = None
        self.yolo, self.images, self.views, self.pub_cam = {}, {}, {}, {}
        self.create_subscription(Detection3DArray, '/validation/gt/objects',
                                 lambda m: setattr(self, 'gt', m), 10)
        for cam in self.cams:
            self.images[cam] = {}
            self.pub_cam[cam] = self.create_publisher(Image, f'/validation/yolo_gt/{cam}/image', 2)
            self.create_subscription(Detection3DArray, f'/yolo/{cam}/objects_3d',
                                     lambda m, c=cam: self.yolo.__setitem__(c, m), 10)
            self.create_subscription(Image, f'/{cam}/image',
                                     lambda m, c=cam: self.on_image(c, m), qos_profile_sensor_data)
            self.create_subscription(Detection2DArray, f'/yolo/{cam}/detections',
                                     lambda m, c=cam: self.on_detections(c, m), 10)
        self.pub = self.create_publisher(Image, '/validation/yolo_gt/image', 2)
        path = str(self.get_parameter('csv_path').value)
        self.csv = None
        if path:
            Path(path).parent.mkdir(parents=True, exist_ok=True)
            self.csv_file = open(path, 'w', newline='')
            self.csv = csv.DictWriter(self.csv_file, fieldnames=CSV_FIELDS)
            self.csv.writeheader()
        self.get_logger().info(f'YOLO vs GT, {self.cams} -> /validation/yolo_gt/* (VALIDATION ONLY)')

    def on_image(self, cam, msg):
        cache = self.images[cam]
        cache[(msg.header.stamp.sec, msg.header.stamp.nanosec)] = msg
        while len(cache) > IMAGE_CACHE:
            cache.pop(next(iter(cache)))

    def on_detections(self, cam, msg):
        # yolo_gazebo_node copies the source image header onto its detections.
        source = self.images[cam].get((msg.header.stamp.sec, msg.header.stamp.nanosec))
        if source is None or self.gt is None or cam not in self.yolo:
            return
        camera = self.cameras[cam]
        gt = {}
        for d in self.gt.detections:
            p = d.results[0].pose.pose.position
            gt[d.id] = (np.array([p.x, p.y, p.z]), d.bbox.size.z)
        gt = visible(camera, gt)
        yolo = []
        for d in self.yolo[cam].detections:
            hyp = d.results[0]
            p = hyp.pose.pose.position
            yolo.append((hyp.hypothesis.class_id, hyp.hypothesis.score, np.array([p.x, p.y, p.z])))
        matches, extra = match(gt, yolo)
        window = view_window(camera, gt)
        view = crop(self.bridge.imgmsg_to_cv2(source, 'bgr8'), window)
        y.trace_detections(view, detections_in_view(msg, window), cam, LABEL_SCALE)
        draw_errors(view, camera, gt, matches, extra, window)
        if self.csv:
            self.write_csv(cam, gt, matches)
        self.views[cam] = view
        self.pub_cam[cam].publish(self.to_msg(view, msg.header))
        self.pub.publish(self.to_msg(mosaic(self.views, self.cams), msg.header))

    def to_msg(self, image, header):
        # rgb8: the Gazebo ImageDisplay panel reads RGB_INT8.
        out = self.bridge.cv2_to_imgmsg(cv2.cvtColor(image, cv2.COLOR_BGR2RGB), 'rgb8')
        out.header = header
        return out

    def write_csv(self, cam, gt, matches):
        s = self.yolo[cam].header.stamp
        for name in sorted(gt):
            found = matches[name]
            row = {'stamp': f'{s.sec}.{s.nanosec:09d}', 'camera': cam, 'object_id': name,
                   'verdict': verdict(name, found),
                   'x_gt': round(gt[name][0][0], 5), 'y_gt': round(gt[name][0][1], 5)}
            if found is not None:
                row.update({'yolo_class': found[0], 'confidence': round(found[1], 4),
                            'x_yolo': round(float(found[2][0]), 5),
                            'y_yolo': round(float(found[2][1]), 5),
                            'error_xy_mm': round(found[3] * 1000, 2)})
            self.csv.writerow(row)
        self.csv_file.flush()

    def destroy_node(self):
        if self.csv:
            self.csv_file.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = YoloGtOverlay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()
