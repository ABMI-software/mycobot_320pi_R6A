"""Belief-map peak-intensity diagnostic.

If distal keypoints (link4/5/6) detect poorly while their MSE looks fine, the
suspected cause is belief-map collapse: the network outputs a near-flat map
(low MSE against a mostly-zero target) with no detectable peak. This measures
the actual peak height (max of the predicted belief map) per keypoint, on
frames where the GT keypoint is well inside the frame, so a low peak can only
mean the network failed to form one.
"""
import os, sys, glob, json
import numpy as np
import torch
from PIL import Image as PILImage

sys.path.insert(0, os.path.dirname(__file__))
import dream
from mycobot_fk import KEYPOINT_NAMES

TRAIN_FRAC, VAL_FRAC = 0.80, 0.10
W, H, MARGIN = 640, 480, 15  # keypoint must be >=MARGIN px from every border


def main():
    weights = "dream/vgg_w357_clahe_50k_e50/best_network.pth"
    data    = "dream_data/synth_50k_clahe"
    n_max   = 200

    config = os.path.splitext(weights)[0] + ".yaml"
    net = dream.create_network_from_config_file(config, weights)
    net.enable_evaluation()

    jfs = sorted(glob.glob(os.path.join(data, "??????.json")))
    n_total = len(jfs)
    n_train = int(n_total * TRAIN_FRAC); n_val = int(n_total * VAL_FRAC)
    jfs = jfs[n_train:n_train + n_val]
    step = len(jfs) / n_max
    jfs = [jfs[int(i * step)] for i in range(n_max)]

    K = len(KEYPOINT_NAMES)
    peaks_in   = [[] for _ in range(K)]   # peak height where GT is in-frame
    peaks_off  = [[] for _ in range(K)]   # peak height where GT is off-frame

    for jf in jfs:
        fid = os.path.basename(jf).replace(".json", "")
        img_path = os.path.join(data, f"{fid}.rgb.png")
        if not os.path.exists(img_path):
            continue
        kps = json.load(open(jf))["objects"][0]["keypoints"]
        gt = {k["name"].replace("mycobot320_", ""): k["projected_location"] for k in kps}
        image = PILImage.open(img_path).convert("RGB")
        with torch.no_grad():
            res = net.keypoints_from_image(image, debug=True)
        bmaps = res.get("belief_maps")
        if bmaps is None:
            print("❌ pas de belief_maps dans le résultat (debug)"); return
        if torch.is_tensor(bmaps):
            bmaps = bmaps.detach().cpu().numpy()
        else:
            bmaps = np.asarray([np.asarray(b.detach().cpu() if torch.is_tensor(b) else b)
                                for b in bmaps])
        for ki, name in enumerate(KEYPOINT_NAMES):
            short = name.replace("mycobot320_", "")
            peak = float(bmaps[ki].max())
            x, y = gt.get(short, (-999, -999))
            in_frame = (MARGIN <= x < W - MARGIN) and (MARGIN <= y < H - MARGIN)
            (peaks_in if in_frame else peaks_off)[ki].append(peak)

    print("\n" + "=" * 64)
    print("INTENSITÉ DES PICS DE BELIEF MAP (max prédit, 0–1)")
    print("Un pic ~0 sur keypoint EN CADRE = effondrement (carte aplatie)")
    print("=" * 64)
    print(f"{'Keypoint':10s} {'pic(en cadre)':>14s} {'n':>4s} {'pic(hors cadre)':>16s} {'n':>4s}")
    base_in = np.mean(peaks_in[0]) if peaks_in[0] else 0
    for ki, name in enumerate(KEYPOINT_NAMES):
        short = name.replace("mycobot320_", "")
        pin = np.mean(peaks_in[ki]) if peaks_in[ki] else float("nan")
        poff = np.mean(peaks_off[ki]) if peaks_off[ki] else float("nan")
        rel = f"({pin/base_in*100:3.0f}% de base)" if base_in else ""
        print(f"{short:10s} {pin:14.3f} {len(peaks_in[ki]):4d} {poff:16.3f} "
              f"{len(peaks_off[ki]):4d}  {rel}")
    print("=" * 64)
    print("Lecture: si link5/6 EN CADRE ont un pic << base → effondrement confirmé")
    print("         (le réseau n'a pas appris à former le pic, pas un pb de localisation)")


if __name__ == "__main__":
    main()
