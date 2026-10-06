"""YOLO 2D boxes against the Gazebo ground truth: reference boxes and matching.

There is no 2D ground truth in Gazebo: the reference is each model's 3D box
(model.sdf footprint, Gazebo pose) projected with the camera and framed. AMODAL
box: a piece behind another keeps its whole box; it is flagged hidden when more
than half of it is covered by a nearer piece. Pure geometry — the caller brings
the poses, so this module never reads Gazebo itself (protocol invariant I4).
"""

import numpy as np

from . import tri_scene

IOU_MIN = 0.5
HIDDEN_COVER = 0.5
# A piece with less than 10 % of its box inside the image is not expected.
VISIBLE_MIN = 0.1


def gt_box(camera, xyz, footprint):
    uv = camera.project(tri_scene.corners(xyz[:2], footprint) + [0, 0, xyz[2]])
    return np.r_[uv.min(axis=0), uv.max(axis=0)]


def iou(a, b):
    ix = max(0.0, min(a[2], b[2]) - max(a[0], b[0]))
    iy = max(0.0, min(a[3], b[3]) - max(a[1], b[1]))
    inter = ix * iy
    union = (a[2] - a[0]) * (a[3] - a[1]) + (b[2] - b[0]) * (b[3] - b[1]) - inter
    return inter / union if union > 0 else 0.0


def area(box):
    return max(0.0, box[2] - box[0]) * max(0.0, box[3] - box[1])


def clip(box, camera):
    return np.clip(box, 0, [camera.width - 1, camera.height - 1] * 2)


def cover(box, other):
    ix = max(0.0, min(box[2], other[2]) - max(box[0], other[0]))
    iy = max(0.0, min(box[3], other[3]) - max(box[1], other[1]))
    return ix * iy / ((box[2] - box[0]) * (box[3] - box[1]))


def references(camera, poses, footprints):
    """{name: {'box', 'truncated', 'hidden', 'dist'}} of the pieces this camera sees.

    poses: {name: model origin xyz in the base frame}.
    """
    refs = {}
    for name, xyz in poses.items():
        try:
            whole = gt_box(camera, xyz, footprints[name])
        except ValueError:
            continue
        box = clip(whole, camera)
        part = area(box) / area(whole)
        if part >= VISIBLE_MIN:
            centre = xyz + [0, 0, footprints[name].height / 2]
            refs[name] = {'box': box, 'truncated': part < 0.99,
                          'dist': np.linalg.norm(centre - camera.world_from_optical[:3, 3])}
    for name, r in refs.items():
        r['hidden'] = any(cover(r['box'], o['box']) > HIDDEN_COVER
                          for other, o in refs.items() if other != name and o['dist'] < r['dist'])
    return refs


def match_boxes(refs, detections):
    """Greedy on decreasing IoU: (matches [(name, i, iou)], unmatched detections, missed refs)."""
    pairs = sorted(((iou(r['box'], d['boite']), name, i)
                    for name, r in refs.items() for i, d in enumerate(detections)), reverse=True)
    used_r, used_d, matches = set(), set(), []
    for score, name, i in pairs:
        if score < IOU_MIN or name in used_r or i in used_d:
            continue
        used_r.add(name)
        used_d.add(i)
        matches.append((name, i, score))
    return matches, [i for i in range(len(detections)) if i not in used_d], \
        [n for n in refs if n not in used_r]
