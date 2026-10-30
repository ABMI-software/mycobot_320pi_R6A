#!/usr/bin/env python3
"""Pure geometric explanation for why J1/J2/J4/J5 all peak in [-90,-60]deg
(see analyze_failure_modes.py, Part C) — no solver, no new data. Uses only
forward_kinematics + camera projection on EXISTING real_3cam cached poses.

For each joint whose worst bucket is [-90,-60]:
  1. Image-plane Jacobian d(keypoint 2D)/d(joint angle) at REAL cached poses
     in that bucket vs elsewhere — does per-degree pixel sensitivity actually
     drop there? (units: px/deg)
  2. Pairwise 2D distance between all keypoints at those poses — do any
     become near-coincident in the image (near-alignment)?
  3. Foreshortening: angle between each link segment's 3D direction and the
     camera viewing ray to that segment — near 0 deg = link points almost
     straight at/away from the camera (minimal parallax, that segment
     collapses toward a point in 2D).
  4. Saves annotated overlays on REAL captured images (session 1, confirmed
     mapped to dream_data/real_3cam/labels.csv) for the worst vs a
     well-observed reference pose.
"""
import csv
import json
import math
import os
import sys

import cv2
import numpy as np

_HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, _HERE)
from mycobot_fk import forward_kinematics, KEYPOINT_NAMES

_REPO = os.path.dirname(os.path.dirname(_HERE))
CALIB_PATH = os.path.join(_REPO, 'training', 'calibration', 'cam_3.meta.json')
CACHE_PATH = os.path.join(_HERE, 'angle_solver_validation_cache.npz')
LABELS_PATH = os.path.join(_HERE, 'dream_data', 'real_3cam', 'labels.csv')
IMAGES_ROOT = os.path.join(_HERE, 'dream_data', 'real_3cam')
OUT_DIR = os.path.join(_HERE, 'geometry_analysis_neg90_neg60')

JOINT_NAMES = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6']
KP_LABELS = [n.replace('mycobot320_', '') for n in KEYPOINT_NAMES]


def load_intrinsics():
    meta = json.loads(open(CALIB_PATH).read())
    r = meta['results']
    K = np.array([[r['fx'], 0.0, r['cx']], [0.0, r['fy'], r['cy']], [0.0, 0.0, 1.0]])
    dist = np.array(r['dist_coeffs'], dtype=np.float64)
    return K, dist


def build_session_anchor(kp_2d_list, kp_valid_list, q_true_list, camera_K, camera_dist):
    rvecs, tvecs = [], []
    for kp_2d, kp_valid, q in zip(kp_2d_list, kp_valid_list, q_true_list):
        if kp_valid.sum() < 5:
            continue
        positions, _ = forward_kinematics(q)
        pts_3d = np.array([positions[n] for n in KEYPOINT_NAMES])
        idx = np.where(kp_valid)[0]
        ok, rvec, tvec = cv2.solvePnP(pts_3d[idx], kp_2d[idx], camera_K, camera_dist, flags=cv2.SOLVEPNP_EPNP)
        if not ok:
            continue
        rvec, tvec = cv2.solvePnPRefineLM(pts_3d[idx], kp_2d[idx], camera_K, camera_dist, rvec, tvec)
        rvecs.append(rvec.flatten())
        tvecs.append(tvec.flatten())
    if len(rvecs) < 5:
        return None
    return np.median(np.array(rvecs), axis=0), np.median(np.array(tvecs), axis=0)


def project(q, rvec, tvec, K, dist):
    positions, _ = forward_kinematics(q)
    pts_3d = np.array([positions[n] for n in KEYPOINT_NAMES])
    proj, _ = cv2.projectPoints(pts_3d, rvec, tvec, K, dist)
    return proj.reshape(-1, 2), pts_3d


def image_jacobian(q, rvec, tvec, K, dist, joint_idx, step_deg=0.5):
    """Numerical d(kp_2d [14])/d(q[joint_idx]) at q, central difference."""
    step = math.radians(step_deg)
    q_plus, q_minus = q.copy(), q.copy()
    q_plus[joint_idx] += step
    q_minus[joint_idx] -= step
    p_plus, _ = project(q_plus, rvec, tvec, K, dist)
    p_minus, _ = project(q_minus, rvec, tvec, K, dist)
    d_px_d_rad = (p_plus - p_minus) / (2 * step)   # (7,2), px/rad
    return d_px_d_rad * math.radians(1.0)            # px/deg


def foreshortening_angles(pts_3d, rvec, tvec):
    """Angle (deg) between each consecutive link's 3D direction (in camera
    frame) and the viewing ray to that link's midpoint. Near 0/180 = link
    points almost straight along the optical axis -> collapses toward a
    point in 2D (minimal parallax)."""
    R, _ = cv2.Rodrigues(rvec)
    t = tvec.reshape(3)
    pts_cam = (R @ pts_3d.T).T + t   # (7,3) in camera frame
    angles = []
    for i in range(6):
        link_dir = pts_cam[i + 1] - pts_cam[i]
        midpoint = (pts_cam[i + 1] + pts_cam[i]) / 2.0
        cos_a = np.dot(link_dir, midpoint) / (np.linalg.norm(link_dir) * np.linalg.norm(midpoint) + 1e-9)
        cos_a = np.clip(cos_a, -1.0, 1.0)
        angle = math.degrees(math.acos(abs(cos_a)))  # 0 = parallel to ray (foreshortened), 90 = perpendicular (max parallax)
        angles.append(angle)
    return angles  # index i = segment KEYPOINT_NAMES[i] -> KEYPOINT_NAMES[i+1]


def pairwise_distances(pts_2d, exclude_pairs=frozenset()):
    n = len(pts_2d)
    out = []
    for i in range(n):
        for j in range(i + 1, n):
            if (KP_LABELS[i], KP_LABELS[j]) in exclude_pairs:
                continue
            d = float(np.linalg.norm(pts_2d[i] - pts_2d[j]))
            out.append((KP_LABELS[i], KP_LABELS[j], d))
    return sorted(out, key=lambda x: x[2])


# link1/link2 sit at (almost) the same 3D point on this robot's geometry —
# constant regardless of pose (elbow-adjacent joint offset ~0), confirmed by
# a live capture showing them at (216.68,256.34) vs (216.68,256.34). Exclude
# from "closest pair" so it doesn't mask a pose-DEPENDENT near-alignment.
_ALWAYS_CLOSE_PAIRS = frozenset({('link1', 'link2')})


def full_jacobian(q, rvec, tvec, K, dist, step_deg=0.5):
    """(14,6) numerical Jacobian of all 7 keypoints' (u,v) w.r.t. all 6
    joint angles, px/deg, at a fixed (oracle) camera pose."""
    step = math.radians(step_deg)
    cols = []
    for j in range(6):
        q_plus, q_minus = q.copy(), q.copy()
        q_plus[j] += step
        q_minus[j] -= step
        p_plus, _ = project(q_plus, rvec, tvec, K, dist)
        p_minus, _ = project(q_minus, rvec, tvec, K, dist)
        d = (p_plus - p_minus) / (2 * step) * math.radians(1.0)  # px/deg
        cols.append(d.flatten())  # (14,)
    return np.array(cols).T  # (14,6)


def column_cosine_matrix(J):
    """Cosine similarity between each pair of the Jacobian's 6 joint
    columns — near +-1 means those two joints move the keypoints in nearly
    the SAME (or opposite) image direction: a real image cannot tell which
    of the two actually rotated, i.e. a live gauge ambiguity between them,
    independent of any solver regularization."""
    norms = np.linalg.norm(J, axis=0)
    C = np.zeros((6, 6))
    for a in range(6):
        for b in range(6):
            if norms[a] < 1e-9 or norms[b] < 1e-9:
                C[a, b] = np.nan
            else:
                C[a, b] = np.dot(J[:, a], J[:, b]) / (norms[a] * norms[b])
    return C


def draw_overlay(image_path, pts_2d, out_path, title):
    img = cv2.imread(image_path)
    if img is None:
        return False
    for i, (u, v) in enumerate(pts_2d):
        u, v = int(round(u)), int(round(v))
        cv2.circle(img, (u, v), 5, (0, 255, 0), -1)
        cv2.putText(img, KP_LABELS[i], (u + 6, v - 4), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
    for i in range(6):
        p1 = tuple(int(round(x)) for x in pts_2d[i])
        p2 = tuple(int(round(x)) for x in pts_2d[i + 1])
        cv2.line(img, p1, p2, (0, 180, 255), 2)
    cv2.putText(img, title, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
    cv2.imwrite(out_path, img)
    return True


def main():
    os.makedirs(OUT_DIR, exist_ok=True)
    K, dist = load_intrinsics()
    data = np.load(CACHE_PATH)

    session1_mask = data['session'] == 1
    kp_2d_s1 = data['kp_2d'][session1_mask]
    kp_valid_s1 = data['kp_valid'][session1_mask]
    q_true_s1 = data['q_true'][session1_mask]
    is_anchor_s1 = data['is_anchor'][session1_mask]
    pose_idx_s1 = data['pose_idx'][session1_mask]

    rvec, tvec = build_session_anchor(kp_2d_s1[is_anchor_s1], kp_valid_s1[is_anchor_s1],
                                       q_true_s1[is_anchor_s1], K, dist)
    print(f'Session-1 anchor pose: rvec={rvec.round(3)}, tvec={tvec.round(3)}\n')

    labels_rows = {int(r['index']): r for r in csv.DictReader(open(LABELS_PATH)) if r['camera'] == 'arducam'}

    joints_to_check = [0, 1, 3, 4]  # J1, J2, J4, J5 -- worst bucket = [-90,-60] per analyze_failure_modes.py

    # ── 1+2+3: Jacobian / pairwise distances / foreshortening, bad vs good ──
    for jidx in joints_to_check:
        jn = JOINT_NAMES[jidx]
        deg = np.degrees(q_true_s1[:, jidx])
        bad_mask = (deg > -90) & (deg <= -60)
        good_mask = np.abs(deg) < 20   # near-home reference for that joint

        if bad_mask.sum() < 3 or good_mask.sum() < 3:
            print(f'{jn}: pas assez de frames (bad={bad_mask.sum()}, good={good_mask.sum()}) — skip')
            continue

        print(f'=== {jn}  (bad n={int(bad_mask.sum())}, good n={int(good_mask.sum())}) ===')

        bucket_results = {}
        for label, mask in (('MAUVAISE zone [-90,-60]', bad_mask), ('BONNE zone (proche 0)', good_mask)):
            idxs = np.where(mask)[0][:15]  # cap for speed
            jac_norms, min_dists, foreshort_min, cond_nums, worst_cos = [], [], [], [], []
            for i in idxs:
                q = q_true_s1[i]
                jac = image_jacobian(q, rvec, tvec, K, dist, jidx)          # (7,2) px/deg
                jac_norm = np.linalg.norm(jac, axis=1)                     # per-keypoint px/deg
                observing = jac_norm[jidx + 1:]
                jac_norms.append(observing.mean() if len(observing) else 0.0)

                pts_2d, pts_3d = project(q, rvec, tvec, K, dist)
                pd = pairwise_distances(pts_2d, exclude_pairs=_ALWAYS_CLOSE_PAIRS)
                min_dists.append(pd[0][2])

                fs = foreshortening_angles(pts_3d, rvec, tvec)
                foreshort_min.append(min(fs))

                J_full = full_jacobian(q, rvec, tvec, K, dist)
                # J6's column is identically zero (no keypoint depends on its
                # own rotation, by schema construction) -> J_full is always
                # exactly rank-deficient regardless of pose. Drop it for a
                # conditioning number that actually reflects J1-J5.
                cond_nums.append(np.linalg.cond(J_full[:, :5]))
                C = column_cosine_matrix(J_full)
                # highest |cosine| between THIS joint's column and any other
                others = [abs(C[jidx, k]) for k in range(6) if k != jidx and not np.isnan(C[jidx, k])]
                worst_cos.append(max(others) if others else np.nan)

            bucket_results[label] = dict(jac=np.mean(jac_norms), dist=np.mean(min_dists),
                                          fs=np.mean(foreshort_min), cond=np.median(cond_nums),
                                          cos=np.nanmean(worst_cos))
            print(f'  {label:26s} | sensibilite kp-aval (px/deg) = {np.mean(jac_norms):6.2f} '
                  f'| dist min (hors link1/2) = {np.mean(min_dists):6.1f}px '
                  f'| angle min foreshortening = {np.mean(foreshort_min):5.1f}deg '
                  f'| cond(J1-J5) med = {np.median(cond_nums):8.1f} '
                  f'| cos max avec un autre joint = {np.nanmean(worst_cos):.3f}')
        print()

    # ── 4: illustration on real images (session 1) ──
    print('=== Illustration sur images reelles ===')
    # worst J2 frame (biggest MAE region) and a well-observed reference
    deg_j2 = np.degrees(q_true_s1[:, 1])
    bad_candidates = np.where((deg_j2 > -90) & (deg_j2 <= -60) & (~is_anchor_s1))[0]
    good_candidates = np.where((np.abs(deg_j2) < 15) & (~is_anchor_s1))[0]

    for tag, cands in (('bad_J2', bad_candidates), ('good_J2', good_candidates)):
        if len(cands) == 0:
            continue
        i = cands[0]
        pidx = int(pose_idx_s1[i])
        if pidx not in labels_rows:
            continue
        img_path = os.path.join(IMAGES_ROOT, labels_rows[pidx]['image_path'])
        q = q_true_s1[i]
        pts_2d, pts_3d = project(q, rvec, tvec, K, dist)
        pd = pairwise_distances(pts_2d, exclude_pairs=_ALWAYS_CLOSE_PAIRS)
        fs = foreshortening_angles(pts_3d, rvec, tvec)
        J_full = full_jacobian(q, rvec, tvec, K, dist)
        cond5 = np.linalg.cond(J_full[:, :5])
        C = column_cosine_matrix(J_full)
        best_pair, best_val = None, 0.0
        for a in range(6):
            for b in range(a + 1, 6):
                if not np.isnan(C[a, b]) and abs(C[a, b]) > best_val:
                    best_val = abs(C[a, b])
                    best_pair = (JOINT_NAMES[a], JOINT_NAMES[b])
        title = f'{tag} q={np.degrees(q).round(0)}'
        out_path = os.path.join(OUT_DIR, f'{tag}_pose{pidx}.png')
        ok = draw_overlay(img_path, pts_2d, out_path, tag)
        print(f'  {tag}: pose_idx={pidx}, q(deg)={np.degrees(q).round(1).tolist()}')
        print(f'    paire de keypoints la plus proche (hors link1/2): {pd[0][0]}-{pd[0][1]} = {pd[0][2]:.1f}px')
        print(f'    angles lien/rayon-camera (deg, 0=colineaire=foreshortened): '
              f'{[round(a,1) for a in fs]}')
        print(f'    cond(Jacobien J1-J5, J6 exclu car colonne nulle) = {cond5:.1f}')
        print(f'    paire de JOINTS la plus confondue (cosine colonnes) : {best_pair} = {best_val:.3f}')
        print(f'    image sauvegardee: {out_path} ({"OK" if ok else "ECHEC lecture image"})')


if __name__ == '__main__':
    main()
