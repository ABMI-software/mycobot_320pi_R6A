#!/usr/bin/env python3
"""Trace un PNG situant les 7 keypoints DREAM, keypoint 7 (link6) mis en avant,
avec l'axe de rotation de J6 — pour montrer que link6 est SUR l'axe de J6
(donc invisible à la rotation J6)."""
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parent))
from mycobot_fk import (forward_kinematics, KEYPOINT_NAMES, get_camera_transform,
                        keypoints_in_camera_frame, project_keypoints, GAZEBO_INTRINSICS)

CAM = "front"

def project(q):
    cam = keypoints_in_camera_frame(q, get_camera_transform(CAM))
    return np.array(project_keypoints(cam, GAZEBO_INTRINSICS))

def link6_for_j6(base5, j6_deg):
    return project(np.radians(base5 + [j6_deg]))[6]

base5 = [15.0, -30.0, 40.0, -20.0, 25.0]
pts = project(np.radians(base5 + [10.0]))          # 7 keypoints en 2D

fig, ax = plt.subplots(figsize=(8, 9))
ax.set_facecolor("#f4f4f4")

# squelette base -> link1 -> ... -> link6
ax.plot(pts[:, 0], pts[:, 1], '-', color="#7aa6c2", lw=3, zorder=1)

# les 6 premiers keypoints (base..link5)
for i in range(6):
    ax.scatter(*pts[i], s=220, color="#377eb8", zorder=3, edgecolors="white", linewidths=2)
    ax.annotate(f"{i+1}", pts[i], ha="center", va="center", color="white",
                fontsize=11, fontweight="bold", zorder=4)
    ax.annotate(KEYPOINT_NAMES[i].replace("mycobot320_", ""),
                pts[i] + np.array([12, -4]), fontsize=9, color="#333")

# keypoint 7 (link6) MIS EN AVANT
p7 = pts[6]
ax.scatter(*p7, s=520, color="#e41a1c", zorder=5, edgecolors="black", linewidths=2)
ax.annotate("7", p7, ha="center", va="center", color="white",
            fontsize=15, fontweight="bold", zorder=6)
ax.annotate("keypoint 7 = link6\n(bout du poignet)", p7 + np.array([16, 6]),
            fontsize=11, color="#e41a1c", fontweight="bold")

# axe de rotation de J6 : link6 pour J6 = -90 et +90 -> montre qu'ils COÏNCIDENT
for j6 in (-90, 0, 90):
    q = link6_for_j6(base5, j6)
    ax.scatter(*q, s=90, facecolors="none", edgecolors="#e41a1c", linewidths=1.5, zorder=7)
ax.annotate("← J6 tourne de -90° à +90° :\n   le point NE bouge PAS\n   (il est SUR l'axe de J6)",
            p7 + np.array([16, 40]), fontsize=10, color="#b30000")

ax.set_title("Les 7 keypoints DREAM (vue caméra 'front')\n"
             "Keypoint 7 (link6) est sur l'axe de J6 -> J6 inobservable", fontsize=12)
ax.set_xlabel("u (px)"); ax.set_ylabel("v (px)")
ax.set_aspect("equal"); ax.invert_yaxis(); ax.grid(alpha=0.3)
fig.tight_layout()
out = Path(__file__).resolve().parent / "j6_keypoint_location.png"
fig.savefig(out, dpi=130)
print(f"PNG -> {out}")
