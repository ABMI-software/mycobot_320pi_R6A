"""Controlled randomization of the four-object / four-bin sorting scene.

Footprints come from the Gazebo model files and the tabletop and ArUco markers
from the world file, so the sampler constrains exactly what Gazebo simulates.
Objects and bins keep yaw 0 in V1.
"""

from dataclasses import dataclass
from itertools import product
from pathlib import Path
import xml.etree.ElementTree as ET

import numpy as np


PAIRS = {'cube_rouge': 'bac_rouge', 'pave_jaune': 'bac_jaune',
         'cylindre_vert': 'bac_vert', 'cube_bleu': 'bac_bleu'}
OBJECTS = tuple(PAIRS)
BINS = tuple(PAIRS.values())

# Grasp column (110 mm hover, grasp, lift) and bin drop column both solve with
# sim_sorting_grasp.solve_column up to r = 0.28 m and fail at 0.30 m, at every
# azimuth tested (0, 50, 100, -100 deg) — measured 29/09/2026. Reported, not
# enforced: as on the real bench the pieces lie between the markers, where the
# vertical-tool IK does not reach (the real arm reaches ~0.39 m, tool tilted).
REACH_MAX = 0.28
# The work area is the band between the marker pairs, as on the real bench:
# in front of 19/23 (near the robot), short of 25/26.
NEAR_MARKERS = ('aruco_19', 'aruco_23')
FAR_MARKERS = ('aruco_25', 'aruco_26')
BASE_KEEPOUT = 0.10
# Objects keep room around them; static bins may nearly touch. With 30 mm
# between bins only three fit inside the reachable band (measured 29/09).
MIN_GAP = 0.03
BIN_GAP = 0.01
EDGE_MARGIN = 0.01
IMAGE_MARGIN_PX = 5.0
# A view farther than this from the median of the views is left out of the
# fusion. Localized views agree within 2-6 mm (01/10, yolo26 v6c, 4 cameras).
FUSION_GATE = 0.03
# Arm leaned back behind the board: its top-camera shadow falls at x ~ -0.15 m,
# off the tabletop; 4.3 % of the board hidden versus 13.4 % at q = 0.
OBSERVATION_Q_DEG = (0.0, 60.0, -70.0, 0.0, 0.0, 0.0)


@dataclass(frozen=True)
class Footprint:
    half: np.ndarray
    height: float


@dataclass(frozen=True)
class Board:
    lo: np.ndarray
    hi: np.ndarray
    markers: dict

    def work_x(self):
        near = max(self.markers[m][0][0] + self.markers[m][1] for m in NEAR_MARKERS)
        far = min(self.markers[m][0][0] - self.markers[m][1] for m in FAR_MARKERS)
        return near + EDGE_MARGIN, far - EDGE_MARGIN


def _pose(node):
    text = node.findtext('pose')
    return np.zeros(6) if text is None else np.array(text.split(), dtype=float)


def model_footprint(model_sdf):
    """Axis-aligned extent of every collision of a yaw-0 model, origin at its base."""
    lo, hi = np.full(3, np.inf), np.full(3, -np.inf)
    for link in ET.parse(model_sdf).getroot().iter('link'):
        link_xyz = _pose(link)[:3]
        for collision in link.findall('collision'):
            center = link_xyz + _pose(collision)[:3]
            geometry = collision.find('geometry')
            if geometry.find('box') is not None:
                half = np.array(geometry.findtext('box/size').split(), dtype=float) / 2
            elif geometry.find('cylinder') is not None:
                r = float(geometry.findtext('cylinder/radius'))
                half = np.array([r, r, float(geometry.findtext('cylinder/length')) / 2])
            else:
                raise ValueError(f'{model_sdf}: unsupported collision geometry')
            lo, hi = np.minimum(lo, center - half), np.maximum(hi, center + half)
    if not np.isfinite(lo).all():
        raise ValueError(f'{model_sdf}: no collision')
    if not np.allclose(lo[:2], -hi[:2]) or abs(lo[2]) > 1e-9:
        raise ValueError(f'{model_sdf}: origin must be the centre of the base')
    return Footprint(hi[:2].copy(), float(hi[2]))


def load_footprints(models_dir):
    return {name: model_footprint(Path(models_dir) / name / 'model.sdf')
            for name in OBJECTS + BINS}


def load_board(world_sdf, models_dir):
    world = ET.parse(world_sdf).getroot().find('world')
    table = world.find("model[@name='table']")
    size = np.array(table.find('.//collision/geometry/box/size').text.split(), dtype=float)
    center = _pose(table)[:2]
    markers = {}
    for include in world.findall('include'):
        name = include.findtext('name')
        if name and name.startswith('aruco_'):
            sdf = Path(models_dir) / name / 'model.sdf'
            side = max(float(s.text.split()[0])
                       for s in ET.parse(sdf).getroot().iter('size'))
            markers[name] = (_pose(include)[:2], side / 2)
    return Board(center - size[:2] / 2, center + size[:2] / 2, markers)


def _box_gap(ca, ha, cb, hb):
    """Clearance between two axis-aligned rectangles (negative when they overlap)."""
    return float(np.max(np.abs(ca - cb) - ha - hb))


def _distance_to_origin(c, h):
    return float(np.linalg.norm(np.maximum(np.abs(c) - h, 0.0)))


def corners(xy, footprint):
    return np.array([[xy[0] + sx * footprint.half[0], xy[1] + sy * footprint.half[1], z]
                     for sx, sy, z in product((-1, 1), (-1, 1), (0.0, footprint.height))])


def box_centre(camera, xy, footprint):
    uv = camera.project(corners(xy, footprint))
    return (uv.min(axis=0) + uv.max(axis=0)) / 2


def locate_from_box(camera, uv, footprint, iterations=10):
    """Base XY whose projected yaw-0 3D box has its 2D box centre at uv.

    The ray through the box centre misses the object centre under perspective:
    the box frames the near side faces too. Seen obliquely the miss reaches
    5-7 mm on a bin (29/09 campaign, true boxes); this removes it.
    """
    z = footprint.height / 2
    target = camera.on_plane(uv, z)[:2]
    xy = target.copy()
    for _ in range(iterations):
        xy = xy + target - camera.on_plane(box_centre(camera, xy, footprint), z)[:2]
    return np.array([*xy, z])


def best_per_class(detections):
    """[(class, xyz, confidence)] of one camera -> {class: xyz}; yolo26 runs without NMS."""
    best = {}
    for name, xyz, confidence in detections:
        if name not in best or confidence > best[name][1]:
            best[name] = (np.asarray(xyz, float), confidence)
    return {name: xyz for name, (xyz, _) in best.items()}


def fuse_cameras(per_camera, gate=FUSION_GATE):
    """{camera: {class: xyz}} -> {class: (xyz, cameras kept)}, median of the cameras.

    Once each view is localized from its box (locate_from_box), the camera errors
    point in different directions: the median halves the top-camera median error
    (0.70 vs 1.41 mm, 80 pieces, 01/10) and drops a view that is plainly wrong.
    """
    fused = {}
    for name in {n for seen in per_camera.values() for n in seen}:
        views = {cam: seen[name] for cam, seen in per_camera.items() if name in seen}
        points = np.array(list(views.values()))
        centre = np.median(points, axis=0)
        kept = [cam for cam, p in views.items() if np.hypot(*(p[:2] - centre[:2])) <= gate]
        if kept:
            centre = np.median(np.array([views[cam] for cam in kept]), axis=0)
        fused[name] = (centre, sorted(kept))
    return fused


def sorting_pairs(fused):
    """{piece: (piece xyz, bin xyz)} for every piece whose bin is seen too, paired by class."""
    return {piece: (fused[piece][0], fused[bin_][0]) for piece, bin_ in PAIRS.items()
            if piece in fused and bin_ in fused}


def in_view(camera, xy, footprint, margin=IMAGE_MARGIN_PX):
    try:
        uv = camera.project(corners(xy, footprint))
    except ValueError:
        return False
    return bool(np.all(uv >= margin) and np.all(uv[:, 0] <= camera.width - 1 - margin)
                and np.all(uv[:, 1] <= camera.height - 1 - margin))


def item_errors(name, xy, footprints, board, top_camera, piece_reach=None):
    """Constraints on one item alone."""
    c, f = np.asarray(xy), footprints[name]
    errors = []
    if piece_reach and name in OBJECTS and np.hypot(*c) > piece_reach:
        errors.append(f'{name}: beyond {piece_reach * 1000:.0f} mm, out of grasp reach')
    if np.any(c - f.half < board.lo + EDGE_MARGIN) or np.any(c + f.half > board.hi - EDGE_MARGIN):
        errors.append(f'{name}: off the board')
    x_min, x_max = board.work_x()
    if c[0] - f.half[0] < x_min or c[0] + f.half[0] > x_max:
        errors.append(f'{name}: outside the band between the markers')
    if _distance_to_origin(c, f.half) < BASE_KEEPOUT:
        errors.append(f'{name}: on the robot base')
    for marker, (mc, mh) in board.markers.items():
        if _box_gap(c, f.half, mc, np.array([mh, mh])) < EDGE_MARGIN:
            errors.append(f'{name}: covers {marker}')
    if not in_view(top_camera, c, f):
        errors.append(f'{name}: outside the top camera')
    return errors


def required_gap(a, b):
    return BIN_GAP if a in BINS and b in BINS else MIN_GAP


def pair_errors(a, ca, b, cb, footprints):
    gap = required_gap(a, b)
    if _box_gap(np.asarray(ca), footprints[a].half, np.asarray(cb), footprints[b].half) < gap:
        return [f'{a} / {b}: closer than {gap * 1000:.0f} mm']
    return []


def placement_errors(layout, footprints, board, top_camera, piece_reach=None):
    """Every violated constraint, empty when the layout is valid."""
    names = list(layout)
    errors = [e for n in names
              for e in item_errors(n, layout[n], footprints, board, top_camera, piece_reach)]
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            errors += pair_errors(a, layout[a], b, layout[b], footprints)
    return errors


def sample_tri_scene(rng, footprints, board, top_camera, attempts=2000, tries_per_item=400,
                     piece_reach=None):
    """Bins then objects, each drawn uniformly in the reachable part of the board.

    `piece_reach` (m) keeps the four objects within grasp reach of the vertical
    tool, as on the real bench (pieces at 0.21-0.26 m, bins 0.36-0.45 m reached
    with the tool tilted, 30/09 and 01/10); bins stay anywhere in the band.
    """
    lo, hi = board.lo.copy(), board.hi.copy()
    lo[0], hi[0] = board.work_x()
    for _ in range(attempts):
        layout = {}
        for name in BINS + OBJECTS:
            for _ in range(tries_per_item):
                xy = rng.uniform(lo, hi)
                if item_errors(name, xy, footprints, board, top_camera, piece_reach):
                    continue
                if any(pair_errors(name, xy, other, c, footprints) for other, c in layout.items()):
                    continue
                layout[name] = xy
                break
            else:
                break
        if len(layout) == len(BINS) + len(OBJECTS):
            return {name: layout[name] for name in OBJECTS + BINS}
    raise RuntimeError('could not sample a valid sorting scene')


# Leftovers of the single-cube demo: red_cube/red_bin are not yolo26 classes and
# table_camera is a fifth, non-DREAM camera.
REMOVED_MODELS = ('red_cube', 'red_bin', 'table_camera')


def build_world(source_sdf, layout, world_name, drop=()):
    """real_table.sdf with the demo leftovers removed and the eight pieces placed.

    drop: more models or includes to leave out by name (the ArUco markers for
    the DREAM ablation, protocol step 10).
    """
    tree = ET.parse(source_sdf)
    world = tree.getroot().find('world')
    # Gazebo serves /world/<name>/...; spawn and bridges look for this name.
    world.set('name', world_name)
    for name in REMOVED_MODELS:
        model = world.find(f"model[@name='{name}']")
        if model is not None:
            world.remove(model)
    for include in world.findall('include'):
        if include.findtext('name') in drop:
            world.remove(include)
    for name, (x, y) in layout.items():
        include = ET.SubElement(world, 'include')
        ET.SubElement(include, 'uri').text = f'package://mycobot_description/models/{name}'
        ET.SubElement(include, 'name').text = name
        ET.SubElement(include, 'pose').text = f'{x:.6f} {y:.6f} 0 0 0 0'
    ET.indent(tree, '  ')
    return ET.tostring(tree.getroot(), encoding='unicode', xml_declaration=True)


def beyond_reach(layout):
    """Items the vertical-tool IK of sim_sorting_grasp cannot serve (step 9)."""
    return [name for name, xy in layout.items() if np.linalg.norm(xy) > REACH_MAX]
