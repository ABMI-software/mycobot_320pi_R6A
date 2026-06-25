"""Why is link6 detectable in one synthetic dataset and not another?

link6 (end-effector) detects at 96% on the 20K dataset but 43% on the 50K, on the
*same* network/loss. The difference is therefore in the DATA. This script measures,
per keypoint, the data-side factors that make a keypoint hard to localise — so we
fix the real cause instead of guessing:

  1. in-frame %        : keypoint inside [margin, W-margin]x[margin, H-margin]
  2. depth z (m)       : location[2] — far = small in image = harder
  3. nearest-neighbour : min pixel distance to ANY other keypoint
                         (small = overlap/occlusion/ambiguity)
  4. last-segment span : pixel distance link5->link6
                         (small = end-effector points at/away from camera =
                          foreshortened = ambiguous)

Run on two datasets and compare the link6 rows.
"""
import os, sys, glob, json, argparse
import numpy as np

sys.path.insert(0, os.path.dirname(__file__))
from mycobot_fk import KEYPOINT_NAMES   # base, link1..link6

W, H, MARGIN = 640, 480, 15


def short(name):
    return name.replace("mycobot320_", "")


def analyze(data_dir, n_max):
    jfs = sorted(glob.glob(os.path.join(data_dir, "??????.json")))
    if not jfs:
        print(f"  ❌ aucun .json dans {data_dir}")
        return None
    if len(jfs) > n_max:
        step = len(jfs) / n_max
        jfs = [jfs[int(i * step)] for i in range(n_max)]

    K = len(KEYPOINT_NAMES)
    in_frame = [0] * K
    total    = [0] * K
    depth    = [[] for _ in range(K)]
    nn_dist  = [[] for _ in range(K)]   # nearest other-keypoint pixel distance
    seg_span = [[] for _ in range(K)]   # pixel dist to previous keypoint in chain

    for jf in jfs:
        kps = json.load(open(jf))["objects"][0]["keypoints"]
        by = {short(k["name"]): k for k in kps}
        proj = {}
        for ki, name in enumerate(KEYPOINT_NAMES):
            s = short(name)
            k = by.get(s)
            if k is None:
                continue
            u, v = k["projected_location"]
            z = k["location"][2]
            proj[ki] = (u, v)
            total[ki] += 1
            if MARGIN <= u < W - MARGIN and MARGIN <= v < H - MARGIN:
                in_frame[ki] += 1
            depth[ki].append(z)
        # pairwise pixel distances (only among present keypoints)
        for ki, (u, v) in proj.items():
            others = [np.hypot(u - uu, v - vv)
                      for kj, (uu, vv) in proj.items() if kj != ki]
            if others:
                nn_dist[ki].append(min(others))
            if ki > 0 and (ki - 1) in proj:
                pu, pv = proj[ki - 1]
                seg_span[ki].append(np.hypot(u - pu, v - pv))

    def med(x):
        return float(np.median(x)) if x else float("nan")

    rows = []
    for ki, name in enumerate(KEYPOINT_NAMES):
        rows.append({
            "kp": short(name),
            "inframe": 100.0 * in_frame[ki] / max(total[ki], 1),
            "depth": med(depth[ki]),
            "nn": med(nn_dist[ki]),
            "span": med(seg_span[ki]),
        })
    return rows


def print_table(name, rows):
    print(f"\n=== {name} ===")
    print(f"{'kp':8s} {'in-frame%':>9s} {'depth(m)':>9s} {'nn-px':>7s} {'prev-span-px':>12s}")
    for r in rows:
        span = f"{r['span']:.1f}" if r['span'] == r['span'] else "  —"
        print(f"{r['kp']:8s} {r['inframe']:9.1f} {r['depth']:9.3f} "
              f"{r['nn']:7.1f} {span:>12s}")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", "-d", nargs="+", required=True,
                    help="un ou plusieurs dossiers NDDS à comparer")
    ap.add_argument("--n", type=int, default=2000)
    args = ap.parse_args()

    results = {}
    for d in args.data:
        rows = analyze(d, args.n)
        if rows:
            results[os.path.basename(os.path.normpath(d))] = rows
            print_table(d, rows)

    if len(results) == 2:
        (na, ra), (nb, rb) = list(results.items())
        print("\n" + "=" * 64)
        print(f"DELTA link4/5/6  ({na}  vs  {nb})")
        print("=" * 64)
        print(f"{'kp':8s} {'Δin-frame':>10s} {'Δdepth':>8s} {'Δnn-px':>8s} {'Δspan':>8s}")
        for ki in (4, 5, 6):
            a, b = ra[ki], rb[ki]
            print(f"{a['kp']:8s} {a['inframe']-b['inframe']:+10.1f} "
                  f"{a['depth']-b['depth']:+8.3f} {a['nn']-b['nn']:+8.1f} "
                  f"{a['span']-b['span']:+8.1f}")
        print("\nLecture: un link6 plus souvent hors-cadre, plus loin (depth+),")
        print("plus proche d'un autre keypoint (nn-) ou avec un dernier segment")
        print("plus court (span-) explique une détection plus basse.")


if __name__ == "__main__":
    main()
