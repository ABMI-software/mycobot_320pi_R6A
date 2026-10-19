"""Does the action lead the state? Per joint: mean |action - state|, and the
shift k (frames) that best matches action[t] with state[t+k]."""
import sys
import numpy as np
from lerobot.datasets.lerobot_dataset import LeRobotDataset
ds = LeRobotDataset(sys.argv[2], root=sys.argv[1])
st = np.stack([ds[i]["observation.state"].numpy() for i in range(len(ds))])
ac = np.stack([ds[i]["action"].numpy() for i in range(len(ds))])
names = [n.split('.')[-1] for n in ds.features["action"]["names"]]
print(f"frames {len(st)}   action == state on every frame: {bool(np.allclose(st, ac))}")
print(f"{'joint':24s} {'mean|a-s| deg':>13s} {'max|a-s| deg':>12s} {'best k':>6s} {'err at k deg':>12s}")
for j, n in enumerate(names):
    d = np.degrees(np.abs(ac[:, j] - st[:, j]))
    errs = [np.degrees(np.mean(np.abs(ac[:len(st) - k, j] - st[k:, j]))) for k in range(0, 11)]
    k = int(np.argmin(errs))
    print(f"{n:24s} {d.mean():13.2f} {d.max():12.2f} {k:6d} {errs[k]:12.2f}")
g = names.index('gripper_controller')
def events(x):
    """(frame opened, frame closed again) for the grasp: open = above -0.2 rad, closed = below -0.4."""
    o = int(np.argmax(x > -0.2)); c = o + int(np.argmax(x[o:] < -0.4))
    return o, c
(oa, ca), (os_, cs) = events(ac[:, g]), events(st[:, g])
print(f"gripper at frame 0: commanded {ac[0, g]:.3f}, measured {st[0, g]:.3f} rad")
print(f"gripper opens:  commanded frame {oa} (t={oa/ds.fps:.1f}s), measured frame {os_} (t={os_/ds.fps:.1f}s)")
print(f"gripper closes: commanded frame {ca} (t={ca/ds.fps:.1f}s), measured frame {cs} (t={cs/ds.fps:.1f}s)")
