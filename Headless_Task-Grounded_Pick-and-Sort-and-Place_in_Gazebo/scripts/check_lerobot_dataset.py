"""Load a ported dataset with LeRobot's own LeRobotDataset and report what a
training run would see; save one contact sheet of decoded frames."""
import sys
import numpy as np
import cv2
from lerobot.datasets.lerobot_dataset import LeRobotDataset

root, repo_id, sheet = sys.argv[1], sys.argv[2], sys.argv[3]
ds = LeRobotDataset(repo_id, root=root)
print(f"episodes={ds.num_episodes} frames={ds.num_frames} fps={ds.fps}")
for key, f in ds.features.items():
    print(f"  {key:28s} {f['dtype']:8s} {f.get('shape')} {f.get('names') if 'state' in key or key == 'action' else ''}")
cams = [k for k in ds.meta.camera_keys]
picks = [0, len(ds) // 3, 2 * len(ds) // 3, len(ds) - 1]
rows = []
for i in picks:
    item = ds[i]
    tiles = []
    for c in cams:
        img = (item[c].permute(1, 2, 0).numpy() * 255).astype(np.uint8)[:, :, ::-1].copy()
        cv2.putText(img, f"frame {i} t={float(item['timestamp']):.1f}s {c.split('.')[-1]}", (8, 24),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 2)
        tiles.append(cv2.resize(img, (400, 300)))
    rows.append(np.hstack(tiles))
cv2.imwrite(sheet, np.vstack(rows))
item = ds[picks[1]]
print("task:", repr(item["task"]))
print("image tensor:", tuple(item[cams[0]].shape), item[cams[0]].dtype,
      f"range {float(item[cams[0]].min()):.2f}..{float(item[cams[0]].max()):.2f}")
hf = ds.hf_dataset
st = np.stack([np.asarray(r, float) for r in hf["observation.state"]])
ac = np.stack([np.asarray(r, float) for r in hf["action"]])
ep = np.array([int(e) for e in hf["episode_index"]])
ts = np.array([float(t) for t in hf["timestamp"]])
print("state range per joint (rad):", np.round(st.min(0), 2), np.round(st.max(0), 2))
print("gripper min/max:", round(float(st[:, 6].min()), 3), round(float(st[:, 6].max()), 3))
print("action == state on every frame:", bool(np.allclose(st, ac)))
steps = sorted(set(np.round(np.diff(ts)[np.diff(ep) == 0], 3)))
lengths = np.bincount(ep)
print(f"episodes {len(lengths)}: frames per episode min {lengths.min()} median {int(np.median(lengths))} "
      f"max {lengths.max()}; timestamp steps within episodes {steps}")
tasks = {}
for e, k in zip(hf["episode_index"], hf["task_index"]):
    tasks.setdefault(int(k), set()).add(int(e))
for k, eps in sorted(tasks.items()):
    print(f"  task {k}: {len(eps)} episodes -- {ds.meta.tasks.index[k]!r}")
