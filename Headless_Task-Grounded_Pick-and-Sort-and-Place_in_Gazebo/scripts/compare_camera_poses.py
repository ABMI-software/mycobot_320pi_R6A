import math, re
import numpy as np
from pathlib import Path
from scipy.spatial.transform import Rotation
from ament_index_python.packages import get_package_share_directory
from mycobot_gateway.vision.sim_multicam_geometry import load_cameras

desc = Path(get_package_share_directory('mycobot_description'))
sim = load_cameras(desc / 'urdf/320_pi/mycobot_pro_320_pi_gazebo.urdf', {'camera_layout': 'dream50k'})
world = (desc / 'worlds/banc_realiste_yolo26.sdf').read_text()

def describe(origin, axis, fov_deg):
    hit = origin - axis * origin[2] / axis[2] if axis[2] < 0 else None
    return origin, axis / np.linalg.norm(axis), hit, fov_deg

cams = {}
for name, c in sim.items():
    T = c.world_from_optical
    fov = math.degrees(2 * math.atan(c.width / 2 / c.K[0, 0]))
    cams[name] = describe(T[:3, 3], T[:3, 2], fov) + (f'{c.width}x{c.height}',)
for name in ('arducam', 'svpro'):
    block = re.search(rf'<model name="{name}">.*?</model>', world, re.S).group(0)
    pose = np.array(re.search(r'<pose>([^<]+)</pose>', block).group(1).split(), float)
    R = Rotation.from_euler('xyz', pose[3:]).as_matrix()
    fov = math.degrees(float(re.search(r'<horizontal_fov>([^<]+)', block).group(1)))
    w, h = re.search(r'<width>(\d+)</width>\s*<height>(\d+)', block).groups()
    cams[name] = describe(pose[:3], R[:, 0], fov) + (f'{w}x{h}',)

print(f"{'camera':20s} {'position (m)':24s} {'dist to base':>12s} {'elev':>6s} {'azim':>6s} {'FOV':>5s} {'image':>8s}  axis hits table at")
for n, (o, a, hit, fov, res) in cams.items():
    elev = math.degrees(math.asin(-a[2]))
    azim = math.degrees(math.atan2(o[1], o[0]))
    hit_s = f'({hit[0]:+.3f}, {hit[1]:+.3f})' if hit is not None else 'never'
    print(f"{n:20s} ({o[0]:+.3f},{o[1]:+.3f},{o[2]:+.3f})   {np.linalg.norm(o):12.3f} {elev:6.1f} {azim:+6.0f} {fov:5.1f} {res:>8s}  {hit_s}")

print('\nangle between viewing directions (deg) / distance between camera centres (m)')
print(f"{'':20s} {'arducam':>18s} {'svpro':>18s}")
for n in sim:
    row = []
    for r in ('arducam', 'svpro'):
        ang = math.degrees(math.acos(np.clip(cams[n][1] @ cams[r][1], -1, 1)))
        row.append(f'{ang:6.1f} deg {np.linalg.norm(cams[n][0] - cams[r][0]):5.2f} m')
    print(f"{n:20s} {row[0]:>18s} {row[1]:>18s}")
