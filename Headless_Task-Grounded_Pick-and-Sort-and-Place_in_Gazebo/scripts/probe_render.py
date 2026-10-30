#!/usr/bin/env python3
"""Render the encoder-probe layouts (config/probe_protocol.yaml) in one Gazebo session.

  probe_render.py --first 5001 --last 5800 --out /workspace/probe/render

Needs tri_yolo.launch.py running (any seed, headless, no YOLO). The arm goes to the
observation pose with the gripper at -0.28, as in every recorded first frame. For each seed:
the layout from tri_scene.sample_tri_scene (default_rng(seed), piece_reach 0.28), the 8
models moved with set_pose_vector while paused, 0.3 s of sim time to settle, then the first
frames that show the new layout (a top frame with each piece's colour at its projected centre,
and the side frames stamped no earlier); a layout not seen within 5 s goes to stale.jsonl and is
skipped; a seed the sampler rejects goes to unsampleable.jsonl. Writes <out>/<seed>.npz (top, left, right uint8 RGB) and one line per seed in
<out>/layouts.jsonl: drawn x, y and Gazebo's x, y after settling, and the largest settling
drift. A seed already rendered is skipped, so a stopped run resumes.

/workspace/venv_lerobot/bin/python with ROS sourced (it reuses the evaluation node), inside
gazebo_to_lerobot.
"""
import argparse
import json
import re
import subprocess
import sys
from pathlib import Path

import numpy as np
import rclpy
from ament_index_python.packages import get_package_share_directory

from mycobot_gateway.vision import tri_scene
from mycobot_gateway.vision.sim_multicam_geometry import load_cameras

sys.path.insert(0, str(Path(__file__).resolve().parent))
from eval_smolvla_tri_sort import START_GRIPPER, PolicyRunner, decode  # noqa: E402

SETTLE_S = 0.3
FRESH_TIMEOUT_S = 5.0
# Flat top-view colour of each piece; a frame shows the new layout once all four are found there.
TOP_COLOURS = {'cube_rouge': (212, 57, 58), 'pave_jaune': (212, 187, 58),
               'cylindre_vert': (135, 191, 52), 'cube_bleu': (58, 187, 215)}
COLOUR_TOL = 25


def layout_sampler():
    desc = Path(get_package_share_directory('mycobot_description'))
    # The same camera models and sampler arguments as tri_yolo.launch.py.
    cameras = load_cameras(str(desc / 'urdf' / '320_pi' / 'mycobot_pro_320_pi_gazebo.urdf'),
                           {'camera_layout': 'dream50k'})
    footprints = tri_scene.load_footprints(str(desc / 'models'))
    board = tri_scene.load_board(str(desc / 'worlds' / 'real_table.sdf'), str(desc / 'models'))
    sample = lambda seed: tri_scene.sample_tri_scene(np.random.default_rng(seed), footprints, board,  # noqa: E731
                                                      cameras['synth_camera_top'], piece_reach=0.28)

    def shows(img, layout):
        for piece, colour in TOP_COLOURS.items():
            u, v = tri_scene.box_centre(cameras['synth_camera_top'], np.asarray(layout[piece]), footprints[piece])[:2]
            if np.abs(img[int(round(v)), int(round(u))].astype(int) - colour).max() > COLOUR_TOL:
                return False
        return True
    return sample, shows


def stamp(msg):
    return msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9


def fresh_frames(node, layout, shows):
    """Running: the camera frames stamped from the first top frame that shows the new layout on.
    The renderer picks a pose change up 0.1-1.1 s of sim time after it (measured 17/10), and a
    frame stamped after the move can still show the old scene, so a timestamp alone proves nothing."""
    t0 = node.now()
    seen = None
    while node.now() - t0 < FRESH_TIMEOUT_S:
        rclpy.spin_once(node, timeout_sec=0.02)
        if seen is None:
            top = node.images.get('top')
            if top is not None and stamp(top) >= t0 and shows(decode(top), layout):
                seen = stamp(top)
        if seen is not None and len(node.images) == 3 and min(map(stamp, node.images.values())) >= seen:
            return {c: decode(m) for c, m in node.images.items()}
    return None


def set_poses(world, layout):
    req = ' '.join(f'pose {{ name: "{n}" position {{ x: {x:.6f} y: {y:.6f} z: 0.0005 }} '
                   f'orientation {{ w: 1 }} }}' for n, (x, y) in layout.items())
    subprocess.run(['gz', 'service', '-s', f'/world/{world}/set_pose_vector',
                    '--reqtype', 'gz.msgs.Pose_V', '--reptype', 'gz.msgs.Boolean',
                    '--timeout', '5000', '--req', req], check=True, capture_output=True, timeout=30)


def model_poses(world, names):
    """x, y, z of the named models; pose/info, because dynamic_pose/info leaves out the static bins."""
    out = subprocess.run(['gz', 'topic', '-e', '-n', '1', '-t', f'/world/{world}/pose/info'],
                         capture_output=True, text=True, timeout=10).stdout
    poses = {}
    for block in out.split('pose {')[1:]:
        name = re.search(r'name: "([^"]+)"', block)
        pos = re.search(r'position \{\s*x: ([-\d.e+]+)\s*y: ([-\d.e+]+)\s*z: ([-\d.e+]+)', block)
        if name and pos and name.group(1) in names and name.group(1) not in poses:
            poses[name.group(1)] = np.array([float(pos.group(i)) for i in (1, 2, 3)])
    return poses


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--first', type=int, required=True)
    ap.add_argument('--last', type=int, required=True)
    ap.add_argument('--out', type=Path, required=True)
    args, ros_args = ap.parse_known_args()
    args.out.mkdir(parents=True, exist_ok=True)
    sample, shows = layout_sampler()
    rclpy.init(args=['--ros-args', '-p', 'use_sim_time:=true', '-p', 'world_name:=tri_yolo',
                     '-p', 'pose_source:=ground_truth', '-p', 'wall_timeout_scale:=15.0',
                     '-p', 'startup_timeout:=600.0'] + ros_args)
    node = PolicyRunner()
    node.wait_until_ready()
    if not node.transit(np.array(tri_scene.OBSERVATION_Q_DEG), 'pose d observation'):
        raise RuntimeError('observation pose unreachable')
    node.set_gripper_value(START_GRIPPER)
    node.spin_for(1.5)
    log = args.out / 'layouts.jsonl'
    for seed in range(args.first, args.last + 1):
        npz = args.out / f'{seed}.npz'
        if npz.exists():
            continue
        try:
            layout = sample(seed)
        except RuntimeError as e:
            # tri_scene gives up on some seeds (seed 5515: no valid scene); tri_yolo.launch.py would too.
            with open(args.out / 'unsampleable.jsonl', 'a') as f:
                f.write(json.dumps({'seed': seed, 'error': str(e)}) + '\n')
            node.status(f'[probe] seed {seed}: {e}, skipped')
            continue
        node.gz_pause(True)
        set_poses(node.world, layout)
        node.gz_pause(False)
        node.spin_for(SETTLE_S)
        images = fresh_frames(node, layout, shows)
        node.gz_pause(True)
        poses = model_poses(node.world, layout)
        node.gz_pause(False)
        if images is None:
            with open(args.out / 'stale.jsonl', 'a') as f:
                f.write(json.dumps({'seed': seed}) + '\n')
            node.status(f'[probe] seed {seed}: new layout never seen in {FRESH_TIMEOUT_S} s, skipped')
            continue
        drift = {n: round(float(np.linalg.norm(poses[n][:2] - np.asarray(layout[n]))) * 1000, 2)
                 for n in layout}
        np.savez_compressed(npz, **images)
        with open(log, 'a') as f:
            f.write(json.dumps({'seed': seed,
                                'drawn': {n: np.round(xy, 5).tolist() for n, xy in layout.items()},
                                'gazebo': {n: np.round(poses[n], 5).tolist() for n in layout},
                                'drift_mm': drift, 'drift_max_mm': max(drift.values())}) + '\n')
        node.status(f'[probe] seed {seed}: drift max {max(drift.values()):.1f} mm')
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
