"""Pixel size of each piece and its bin in the 4 tri_yolo cameras, seeds 1-10.

Layouts are regenerated exactly as tri_yolo.launch.py does (same function,
same seed, piece_reach 0.28). Size = area of the convex hull of the projected
3D box corners (the exact silhouette of a box), clipped to the image.
Arm occlusion is NOT modelled.
"""
import os
import cv2
import numpy as np
from ament_index_python.packages import get_package_share_directory
from mycobot_gateway.vision import tri_scene
from mycobot_gateway.vision.sim_multicam_geometry import load_cameras

desc = get_package_share_directory('mycobot_description')
models = os.path.join(desc, 'models')
cams = load_cameras(os.path.join(desc, 'urdf', '320_pi', 'mycobot_pro_320_pi_gazebo.urdf'),
                    {'camera_layout': 'dream50k'})
footprints = tri_scene.load_footprints(models)
board = tri_scene.load_board(os.path.join(desc, 'worlds', 'real_table.sdf'), models)
SHORT = {'synth_camera': 'front', 'synth_camera_right': 'right',
         'synth_camera_left': 'left', 'synth_camera_top': 'top'}

def visible_area(cam, xy, fp):
    uv = cam.project(tri_scene.corners(xy, fp)).astype(np.float32)
    hull = cv2.convexHull(uv)
    full = cv2.contourArea(hull)
    clip = np.array([[0, 0], [cam.width, 0], [cam.width, cam.height], [0, cam.height]], np.float32)
    inside, _ = cv2.intersectConvexConvex(hull, clip)
    return inside, (inside / full if full else 0.0)

rows = []
for seed in range(1, 11):
    layout = tri_scene.sample_tri_scene(np.random.default_rng(seed), footprints, board,
                                        cams['synth_camera_top'], piece_reach=0.28)
    if seed == 1:
        print('seed 1 layout (mm):', {n: tuple(int(round(v * 1000)) for v in xy) for n, xy in layout.items()})
    for piece, bin_ in tri_scene.PAIRS.items():
        for role, name in (('piece', piece), ('bin', bin_)):
            for cam, c in cams.items():
                area, frac = visible_area(c, layout[name], footprints[name])
                rows.append((seed, piece, role, SHORT[cam], area, frac))

print(f"\n{'':6s} {'camera':6s} {'median px':>9s} {'min px':>7s} {'<150 px':>8s} {'cut by frame':>12s}")
for role in ('piece', 'bin'):
    for cam in ('front', 'right', 'left', 'top'):
        a = np.array([r[4] for r in rows if r[2] == role and r[3] == cam])
        f = np.array([r[5] for r in rows if r[2] == role and r[3] == cam])
        print(f"{role:6s} {cam:6s} {np.median(a):9.0f} {a.min():7.0f} {np.sum(a < 150):5d}/40 {np.sum(f < 0.99):9d}/40")

print('\nper pick: which of front/left gives the larger piece, and the smallest side view (right or left)')
better = {'front': 0, 'left': 0}
worst_side = []
for seed in range(1, 11):
    for piece in tri_scene.PAIRS:
        a = {r[3]: r[4] for r in rows if r[0] == seed and r[1] == piece and r[2] == 'piece'}
        better['front' if a['front'] > a['left'] else 'left'] += 1
        worst_side.append((min(a['front'], max(a['right'], a['left'])), seed, piece, a))
print('front larger than left:', better['front'], '/ left larger than front:', better['left'])
print('largest piece size in right+left (min over picks):',
      f"{min(max(w[3]['right'], w[3]['left']) for w in worst_side):.0f} px")
print('largest piece size in right+front (min over picks):',
      f"{min(max(w[3]['right'], w[3]['front']) for w in worst_side):.0f} px")
small = sorted(worst_side, key=lambda w: min(w[3]['right'], w[3]['left'], w[3]['front']))[:6]
print('\nsmallest cases (px):')
for _, seed, piece, a in small:
    print(f"  seed {seed:2d} {piece:14s} front {a['front']:5.0f}  right {a['right']:5.0f}  left {a['left']:5.0f}  top {a['top']:5.0f}")

f = sorted(r[5] for r in rows if r[2] == 'bin' and r[3] == 'front')
print(f"\nfront, visible fraction of the bins: min {f[0]:.0%}, median {np.median(f):.0%}; "
      f"under 90 %: {sum(v < 0.9 for v in f)}/40, under 75 %: {sum(v < 0.75 for v in f)}/40")
by_bin = {}
for r in rows:
    if r[2] == 'bin' and r[3] == 'front' and r[5] < 0.99:
        by_bin.setdefault(tri_scene.PAIRS[r[1]], []).append(round(r[5], 2))
print('front, clipped bins by colour:', by_bin)
