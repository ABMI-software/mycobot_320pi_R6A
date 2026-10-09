"""Calibrated geometry for the four fixed Gazebo cameras (metres, optical axes).

Extrinsics come from the simulated camera mounts, not physical-camera YAMLs.
Object coordinates are fitted to image silhouettes; no Gazebo object pose is
used. The model prior is an upright, axis-aligned 40 mm cube on the tabletop.
"""

from dataclasses import dataclass
from itertools import combinations, product
import math
import xml.etree.ElementTree as ET

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation
import xacro


CAMERA_NAMES = ('synth_camera', 'synth_camera_right',
                'synth_camera_left', 'synth_camera_top')
BIN_XY = np.array([0.22, 0.10])
CUBE_SIZE = 0.04


@dataclass
class Camera:
    name: str
    K: np.ndarray
    world_from_optical: np.ndarray
    width: int
    height: int

    def project(self, xyz):
        xyz = np.atleast_2d(xyz)
        T = self.world_from_optical
        optical = (xyz - T[:3, 3]) @ T[:3, :3]
        if np.any(optical[:, 2] <= 0):
            raise ValueError('point behind camera')
        uvw = optical @ self.K.T
        return uvw[:, :2] / uvw[:, 2:3]

    def on_plane(self, uv, z):
        ray = self.world_from_optical[:3, :3] @ np.linalg.solve(
            self.K, [*uv, 1.0])
        origin = self.world_from_optical[:3, 3]
        if abs(ray[2]) < 1e-8:
            raise ValueError('ray parallel to table')
        distance = (z - origin[2]) / ray[2]
        if distance <= 0:
            raise ValueError('table behind camera')
        return origin + distance * ray


def load_cameras(urdf_path, mappings=None):
    # The camera blocks sit inside <xacro:if>: parse what Gazebo is given, not the source.
    expanded = xacro.process_file(str(urdf_path), mappings=mappings or {})
    root = ET.fromstring(expanded.toxml())
    cameras = {}
    # Gazebo sensor: +X forward, +Y left, +Z up. Optical: +Z forward,
    # +X right, +Y down. Columns are optical axes expressed in sensor frame.
    sensor_from_optical = np.array([[0, 0, 1], [-1, 0, 0], [0, -1, 0]])
    for gazebo in root.findall('gazebo'):
        sensor = gazebo.find('sensor')
        if sensor is None or sensor.get('name') not in CAMERA_NAMES:
            continue
        link = gazebo.get('reference')
        joint = next(j for j in root.findall('joint')
                     if j.find('child').get('link') == link)
        if joint.get('type') != 'fixed' or joint.find('parent').get('link') != 'world':
            raise ValueError(f'{link} must be fixed directly to world')
        origin = joint.find('origin')
        xyz = np.fromstring(origin.get('xyz'), sep=' ')
        rpy = np.fromstring(origin.get('rpy'), sep=' ')
        T = np.eye(4)
        T[:3, :3] = Rotation.from_euler('xyz', rpy).as_matrix() @ sensor_from_optical
        T[:3, 3] = xyz
        camera = sensor.find('camera')
        width = int(camera.findtext('image/width'))
        height = int(camera.findtext('image/height'))
        f = width / (2 * math.tan(float(camera.findtext('horizontal_fov')) / 2))
        K = np.array([[f, 0, width / 2], [0, f, height / 2], [0, 0, 1.]])
        name = sensor.get('name')
        cameras[name] = Camera(name, K, T, width, height)
    if set(cameras) != set(CAMERA_NAMES):
        raise ValueError('four fixed cameras required in the URDF')
    return cameras


def box_vertices(center, size):
    return np.asarray(center) + np.array(list(product((-0.5, 0.5), repeat=3))) * size


def projected_box(camera, xy):
    uv = camera.project(box_vertices([*xy, CUBE_SIZE / 2], CUBE_SIZE))
    return np.r_[uv.min(axis=0), uv.max(axis=0)]


def red_candidates(image, camera, bin_xy=BIN_XY):
    """Reject the known bin region and clutter outside the tabletop workspace."""
    if image.shape[:2] != (camera.height, camera.width):
        raise ValueError('image dimensions do not match camera calibration')
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, (0, 110, 65), (12, 255, 255))
    mask |= cv2.inRange(hsv, (168, 110, 65), (180, 255, 255))
    # The bin is red too. Mask its entire projected volume, including walls.
    bin_uv = camera.project(box_vertices([*bin_xy, .017], [.108, .108, .038]))
    polygon = cv2.convexHull(np.rint(bin_uv).astype(np.int32))
    cv2.fillConvexPoly(mask, polygon, 0)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = []
    for contour in contours:
        if cv2.contourArea(contour) < 35:
            continue
        x, y, w, h = cv2.boundingRect(contour)
        if min(w, h) < 5 or max(w, h) > 110:
            continue
        bbox = np.array([x, y, x + w - 1, y + h - 1], dtype=float)
        try:
            point = camera.on_plane((bbox[:2] + bbox[2:]) / 2, .02)
        except ValueError:
            continue
        if not (.08 < point[0] < .42 and -.19 < point[1] < .21):
            continue
        if np.linalg.norm(point[:2] - bin_xy) < .07:
            continue
        candidates.append(bbox)
    return candidates, mask


def fit_cube(cameras, detections, min_views=2, max_error_px=3.0, bin_xy=BIN_XY):
    """Robust multi-view silhouette fit, rejecting occlusions and false blobs.

Every hypothesis requires two independent views; reprojection in all other
views determines consensus. The known table plane and cube size remove the
bias caused by triangulating different visible-face centroids.
    """
    names = [name for name in cameras if detections.get(name)]
    if len(names) < min_views:
        return None

    def residual(xy, observations):
        return np.concatenate([projected_box(cameras[n], xy) - box
                               for n, box in observations])

    best = None
    for a, b in combinations(names, 2):
        for ba, bb in product(detections[a], detections[b]):
            start = cameras[a].on_plane((ba[:2] + ba[2:]) / 2, .02)[:2]
            start = np.clip(start, [.08, -.19], [.42, .21])
            fit = least_squares(residual, start, args=([(a, ba), (b, bb)],),
                                bounds=([.08, -.19], [.42, .21]), max_nfev=25)
            inliers = []
            for name in names:
                pred = projected_box(cameras[name], fit.x)
                box = min(detections[name], key=lambda v: np.linalg.norm(pred - v))
                error = float(np.sqrt(np.mean((pred - box) ** 2)))
                if error <= max_error_px:
                    inliers.append((name, box))
            if len(inliers) < min_views:
                continue
            fit = least_squares(residual, fit.x, args=(inliers,), max_nfev=25)
            error = float(np.sqrt(np.mean(residual(fit.x, inliers) ** 2)))
            if error > max_error_px or np.linalg.norm(fit.x - bin_xy) < .08:
                continue
            score = (-len(inliers), error)
            if best is None or score < best[0]:
                best = (score, np.r_[fit.x, .02], [n for n, _ in inliers], error)
    return None if best is None else best[1:]


def sample_cube_xy(rng, bin_xy=BIN_XY):
    """Reachable front sector, clear of base, bin, markers and board edges."""
    for _ in range(10000):
        xy = rng.uniform([.14, -.145], [.285, .165])
        if (.19 <= np.linalg.norm(xy) <= .285
                and np.linalg.norm(xy - bin_xy) >= .135):
            return xy
    raise RuntimeError('could not sample a collision-free cube position')


def sample_scene_xy(rng):
    """Random cube AND bin centres; both fit on the board and within arm reach."""
    for _ in range(10000):
        bin_xy = rng.uniform([.17, -.11], [.265, .15])
        if not .20 <= np.linalg.norm(bin_xy) <= .28:
            continue
        cube_xy = sample_cube_xy(rng, bin_xy)
        if np.linalg.norm(cube_xy - bin_xy) >= .16:
            return cube_xy, bin_xy
    raise RuntimeError('could not sample a separated cube and bin')
