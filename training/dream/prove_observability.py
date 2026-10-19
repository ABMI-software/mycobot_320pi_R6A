"""Preuve numérique : quel keypoint bouge quand on tourne chaque joint.

Pour chaque joint m, on perturbe q[m] et on mesure le déplacement 3D de
chaque keypoint k : ||p_k(q+e·m) - p_k(q)||  (norme de la jacobienne
geometrique dp_k/dq_m, par differences finies). Non nul => observable.

On moyenne sur plusieurs poses aleatoires pour ne pas tomber par hasard
sur une configuration singuliere.
"""
import sys
from pathlib import Path
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, "/home/genji/Osama_ws/src/mycobot_R6A/training/dream")
from mycobot_fk import forward_kinematics, KEYPOINT_NAMES

N_KP = len(KEYPOINT_NAMES)   # 7
N_J = 6
EPS = 1e-4                   # rad
rng = np.random.default_rng(0)

def kp_positions(q):
    pos, _ = forward_kinematics(q)
    return np.array([pos[n] for n in KEYPOINT_NAMES])   # (7,3)

# Matrice observabilite : lignes = joints (J1..J6), colonnes = keypoints kp0..kp6
# valeur = deplacement 3D moyen (mm) du keypoint quand on tourne le joint de EPS
obs = np.zeros((N_J, N_KP))
for _ in range(200):
    q = rng.uniform(-1.0, 1.0, size=6)
    base = kp_positions(q)
    for m in range(N_J):
        dq = q.copy(); dq[m] += EPS
        moved = kp_positions(dq)
        disp = np.linalg.norm(moved - base, axis=1)   # (7,) metres
        obs[m] += disp
obs /= 200
obs_mm_per_deg = obs * 1000.0 * np.deg2rad(1) / EPS    # mm de deplacement par degre

# Seuil numerique : en dessous = "ne bouge pas" (bruit machine ~1e-12)
THRESH = 1e-6
observable = obs_mm_per_deg > THRESH

print("Matrice d'observabilite (mm de deplacement du keypoint / degre du joint)\n")
header = "        " + "".join(f"kp{k:<8}" for k in range(N_KP))
print(header)
for m in range(N_J):
    row = "".join(f"{v:<11.4f}" if v > THRESH else f"{'.':<11}" for v in obs_mm_per_deg[m])
    print(f"J{m+1:<6} {row}")

print("\nDeduit numeriquement -> keypoints observant chaque joint :")
for m in range(N_J):
    ks = [f"kp{k}" for k in range(N_KP) if observable[m, k]]
    print(f"  J{m+1} -> {ks if ks else 'AUCUN'}")

# ── PNG : heatmap log + surlignage de la frontiere kp[m+2:] ──
fig, ax = plt.subplots(figsize=(9, 5.2))
data = obs_mm_per_deg.copy()
data_log = np.log10(np.where(data > THRESH, data, THRESH/10))
im = ax.imshow(data_log, cmap="viridis", aspect="auto")

for m in range(N_J):
    for k in range(N_KP):
        v = obs_mm_per_deg[m, k]
        txt = f"{v:.2f}" if v > THRESH else "0"
        color = "white" if data_log[m, k] < data_log.max() - 1.2 else "black"
        ax.text(k, m, txt, ha="center", va="center", fontsize=8, color=color)
    # frontiere theorique : premier keypoint observant = m+2
    if m + 2 <= N_KP:
        ax.add_patch(plt.Rectangle((m + 2 - 0.5, m - 0.5), N_KP - (m + 2), 1,
                                   fill=False, edgecolor="red", lw=2.2))

ax.set_xticks(range(N_KP)); ax.set_xticklabels([f"kp{k}\n{KEYPOINT_NAMES[k].replace('mycobot320_','')}" for k in range(N_KP)], fontsize=8)
ax.set_yticks(range(N_J)); ax.set_yticklabels([f"J{m+1}" for m in range(N_J)])
ax.set_xlabel("keypoint"); ax.set_ylabel("joint tourné")
ax.set_title("Observabilité : déplacement du keypoint (mm/°) quand on tourne le joint\n"
             "cadre rouge = prédiction kp[m+2:]  ·  '0' = keypoint insensible au joint")
cbar = fig.colorbar(im, ax=ax); cbar.set_label("log10(mm/°)")
fig.tight_layout()
out = Path(__file__).resolve().parent / "observability_proof.png"
fig.savefig(out, dpi=130)
print(f"\nPNG -> {out}")
