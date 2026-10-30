#!/usr/bin/env python3
"""Offline validation of the DREAM angle solver against real_3cam ground truth.

Runs the exact live-dashboard pipeline (session pose anchor via multi-frame
median PnP + solve_joint_angles_fixed_pose with soft regularization) against
real captured data instead of synthetic noise, so the fix chosen for
reg_weight / ANCHOR_N_SAMPLES is backed by a real-encoder comparison before
spending a real-robot session on it.

Per session (5 independent camera setups — the camera was physically moved
between capture sessions, so a pose anchor from one session is meaningless in
another): build the anchor from the first ANCHOR_N arducam poses (median PnP
using the KNOWN encoder angles), then solve DREAM-only angles for the
remaining poses and compare to the real encoder ground truth in labels.csv.

Poses in this dataset are i.i.d. random configurations captured for training
diversity, not a continuous trajectory, so unlike the live dashboard there is
no meaningful "previous frame" to warm-start from. Rather than pick one
warm-start and re-run the (expensive, GPU-bound) DREAM inference per
experiment, detections are cached to disk on first run so several q_init
strategies can be compared cheaply afterwards:
  - 'cold'    : q_init = 0 — no prior at all, worst case.
  - 'encoder' : q_init = true encoder angle — best case, isolates whether the
                anchor + solver themselves are trustworthy given a good
                starting point (this is what live tracking should stay near
                to, frame after frame, once it has locked on).
"""
import json
import os
import sys

import cv2
import numpy as np
import pandas as pd
import torch
from PIL import Image as PILImage

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
sys.path.insert(0, '/tmp/DREAM')

from mycobot_fk import forward_kinematics, KEYPOINT_NAMES
from dream_angle_solver import solve_joint_angles_fixed_pose

_REPO = os.path.dirname(os.path.dirname(_HERE))
DATA_DIR = os.path.join(_REPO, 'training', 'dream_data', 'real_3cam')
CKPT_DIR = os.path.join(_HERE, 'checkpoints_dream', 'vgg_ultimate_v4_mix_ft_e30')
CALIB_PATH = os.path.join(_REPO, 'training', 'calibration', 'cam_3.meta.json')
CACHE_PATH = os.path.join(_HERE, 'angle_solver_validation_cache.npz')

ANCHOR_N = 30
CAMERA = 'arducam'
JOINT_COLS = ['j1_rad', 'j2_rad', 'j3_rad', 'j4_rad', 'j5_rad', 'j6_rad']


def load_intrinsics():
    meta = json.loads(open(CALIB_PATH).read())
    r = meta['results']
    K = np.array([[r['fx'], 0.0, r['cx']],
                  [0.0, r['fy'], r['cy']],
                  [0.0, 0.0, 1.0]], dtype=np.float64)
    dist = np.array(r['dist_coeffs'], dtype=np.float64)
    return K, dist


def load_dream_network():
    import dream
    from ruamel.yaml import YAML
    yaml_parser = YAML(typ='safe')
    cfg_path = os.path.join(CKPT_DIR, 'best_network.yaml')
    with open(cfg_path) as f:
        cfg = yaml_parser.load(f)
    cfg['training']['platform']['gpu_ids'] = [0]
    net = dream.create_network_from_config_data(cfg)
    map_loc = 'cuda:0' if torch.cuda.is_available() else 'cpu'
    net.model.load_state_dict(
        torch.load(os.path.join(CKPT_DIR, 'best_network.pth'),
                   map_location=map_loc, weights_only=True))
    net.enable_evaluation()
    if torch.cuda.is_available():
        net.model = net.model.cuda()
    return net


def detect_keypoints(net, image_path, img_w, img_h):
    pil_image = PILImage.open(image_path).convert('RGB')
    result = net.keypoints_from_image(pil_image, debug=False)
    detected = result['detected_keypoints']
    kp_2d = np.zeros((7, 2), dtype=np.float64)
    kp_valid = np.zeros(7, dtype=bool)
    for i, kp in enumerate(detected):
        if kp is None or len(kp) != 2:
            continue
        u, v = float(kp[0]), float(kp[1])
        if np.isnan(u) or np.isnan(v):
            continue
        if 0 <= u < img_w and 0 <= v < img_h:
            kp_2d[i] = [u, v]
            kp_valid[i] = True
    return kp_2d, kp_valid


def build_session_anchor(kp_2d_list, kp_valid_list, q_true_list, camera_K, camera_dist):
    rvecs, tvecs = [], []
    for kp_2d, kp_valid, q in zip(kp_2d_list, kp_valid_list, q_true_list):
        if kp_valid.sum() < 5:
            continue
        positions, _ = forward_kinematics(q)
        pts_3d = np.array([positions[n] for n in KEYPOINT_NAMES], dtype=np.float64)
        idx = np.where(kp_valid)[0]
        ok, rvec, tvec = cv2.solvePnP(
            pts_3d[idx], kp_2d[idx], camera_K, camera_dist, flags=cv2.SOLVEPNP_EPNP)
        if not ok:
            continue
        rvec, tvec = cv2.solvePnPRefineLM(pts_3d[idx], kp_2d[idx], camera_K, camera_dist, rvec, tvec)
        rvecs.append(rvec.flatten())
        tvecs.append(tvec.flatten())
    if len(rvecs) < 5:
        return None
    return {'rvec': np.median(np.array(rvecs), axis=0),
            'tvec': np.median(np.array(tvecs), axis=0)}


def build_cache():
    """Run DREAM inference once over the whole arducam set and save to disk."""
    df = pd.read_csv(os.path.join(DATA_DIR, 'labels.csv'))
    df = df[df['camera'] == CAMERA].reset_index(drop=True)
    net = load_dream_network()

    sessions, pose_idx, kp_2d_all, kp_valid_all, q_true_all, is_anchor = [], [], [], [], [], []

    for session, sess_df in df.groupby(df['index'] // 500):
        sess_df = sess_df.reset_index(drop=True)
        print(f'Session {session + 1}: detecting keypoints on {len(sess_df)} images...')
        for i, row in sess_df.iterrows():
            img_path = os.path.join(DATA_DIR, row['image_path'])
            with PILImage.open(img_path) as im:
                w, h = im.size
            kp_2d, kp_valid = detect_keypoints(net, img_path, w, h)
            sessions.append(session + 1)
            pose_idx.append(int(row['index']))
            kp_2d_all.append(kp_2d)
            kp_valid_all.append(kp_valid)
            q_true_all.append(row[JOINT_COLS].to_numpy(dtype=np.float64))
            is_anchor.append(i < ANCHOR_N)

    np.savez(
        CACHE_PATH,
        session=np.array(sessions), pose_idx=np.array(pose_idx),
        kp_2d=np.array(kp_2d_all), kp_valid=np.array(kp_valid_all),
        q_true=np.array(q_true_all), is_anchor=np.array(is_anchor),
    )
    print(f'Cache written to {CACHE_PATH}')


def run_experiment(q_init_mode: str):
    data = np.load(CACHE_PATH)
    camera_K, camera_dist = load_intrinsics()
    sessions = data['session']

    all_errors = []
    for session in np.unique(sessions):
        mask = sessions == session
        kp_2d_s = data['kp_2d'][mask]
        kp_valid_s = data['kp_valid'][mask]
        q_true_s = data['q_true'][mask]
        pose_idx_s = data['pose_idx'][mask]
        is_anchor_s = data['is_anchor'][mask]

        anchor = build_session_anchor(
            kp_2d_s[is_anchor_s], kp_valid_s[is_anchor_s], q_true_s[is_anchor_s],
            camera_K, camera_dist)
        if anchor is None:
            continue

        held = ~is_anchor_s
        for kp_2d, kp_valid, q_true, idx in zip(
                kp_2d_s[held], kp_valid_s[held], q_true_s[held], pose_idx_s[held]):
            if kp_valid.sum() < 4:
                continue
            q_init = np.zeros(6) if q_init_mode == 'cold' else q_true.copy()
            res = solve_joint_angles_fixed_pose(
                kp_2d, kp_valid, camera_K, anchor['rvec'], anchor['tvec'], q_init=q_init)
            if res is None:
                continue
            err_deg = np.degrees(np.abs(res['q'] - q_true))
            all_errors.append((session, int(idx), err_deg, int(kp_valid.sum())))

    err_arr = np.array([e[2] for e in all_errors])
    print(f"\n=== q_init='{q_init_mode}' — {len(all_errors)} held-out poses ===")
    rms_per_joint = np.sqrt(np.mean(err_arr ** 2, axis=0))
    max_per_joint = np.max(err_arr, axis=0)
    for j in range(6):
        print(f'  J{j+1}: RMS={rms_per_joint[j]:6.2f}°  max={max_per_joint[j]:6.2f}°')
    print(f'  Global RMS (J1-J6) : {np.sqrt(np.mean(err_arr ** 2)):.2f}°')
    print(f'  Global RMS (J1-J5) : {np.sqrt(np.mean(err_arr[:, :5] ** 2)):.2f}°')

    out_csv = os.path.join(_HERE, f'angle_solver_validation_{q_init_mode}.csv')
    rows = [{'session': s, 'pose_index': idx, 'n_valid_kp': n,
             **{f'err_j{j+1}_deg': err[j] for j in range(6)}}
            for s, idx, err, n in all_errors]
    pd.DataFrame(rows).to_csv(out_csv, index=False)
    print(f'  Detail saved to {out_csv}')


def main():
    if not os.path.isfile(CACHE_PATH):
        build_cache()
    else:
        print(f'Reusing cached detections from {CACHE_PATH}')

    run_experiment('cold')
    run_experiment('encoder')


if __name__ == '__main__':
    main()
