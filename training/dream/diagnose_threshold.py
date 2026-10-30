"""Threshold test: are the weak distal peaks correctly located but rejected?

For each in-frame keypoint, take the RAW belief-map argmax (no threshold at all)
and compare to GT in net-output coords. If link5/6 argmax is close to GT in most
frames, the peaks are informative and lowering the detection threshold would
recover real detections (free win). If the argmax is far (noise), only a
sigma-retrain helps.
"""
import os, sys, glob, json
import numpy as np
import torch
from PIL import Image as PILImage

sys.path.insert(0, os.path.dirname(__file__))
import dream
from dream.image_proc import (convert_keypoints_to_netin_from_raw,
                              convert_keypoints_to_netout_from_netin)
from mycobot_fk import KEYPOINT_NAMES

TRAIN_FRAC, VAL_FRAC = 0.80, 0.10
W, H, MARGIN = 640, 480, 15


def main():
    weights = "dream/vgg_w357_50k_e50/best_network.pth"
    data    = "dream_data/synth_50k_ndds"
    n_max   = 200

    config = os.path.splitext(weights)[0] + ".yaml"
    net = dream.create_network_from_config_file(config, weights)
    net.enable_evaluation()
    net_in  = net.trained_net_input_resolution()
    net_out = net.net_output_resolution_from_input_resolution(net_in)
    preproc = net.image_preprocessing()
    print(f"net_in={net_in} net_out={net_out} preproc={preproc}")

    jfs = sorted(glob.glob(os.path.join(data, "??????.json")))
    n_total = len(jfs)
    n_train = int(n_total * TRAIN_FRAC); n_val = int(n_total * VAL_FRAC)
    jfs = jfs[n_train:n_train + n_val]
    step = len(jfs) / n_max
    jfs = [jfs[int(i * step)] for i in range(n_max)]

    K = len(KEYPOINT_NAMES)
    dists = [[] for _ in range(K)]   # netout-space argmax→GT distance, in-frame only

    for jf in jfs:
        fid = os.path.basename(jf).replace(".json", "")
        img_path = os.path.join(data, f"{fid}.rgb.png")
        if not os.path.exists(img_path):
            continue
        kps = json.load(open(jf))["objects"][0]["keypoints"]
        raw = [k["projected_location"] for k in kps]
        # GT raw -> netin -> netout
        netin = convert_keypoints_to_netin_from_raw(raw, (W, H), net_in, preproc)
        gt_netout = convert_keypoints_to_netout_from_netin(netin, net_in, net_out)

        image = PILImage.open(img_path).convert("RGB")
        with torch.no_grad():
            res = net.keypoints_from_image(image, debug=True)
        bmaps = res.get("belief_maps")
        if torch.is_tensor(bmaps):
            bmaps = bmaps.detach().cpu().numpy()

        for ki in range(K):
            gx, gy = raw[ki]
            if not (MARGIN <= gx < W - MARGIN and MARGIN <= gy < H - MARGIN):
                continue  # in-frame only
            bm = bmaps[ki]
            ay, ax = np.unravel_index(int(bm.argmax()), bm.shape)  # netout coords
            gxo, gyo = gt_netout[ki]
            dists[ki].append(float(np.hypot(ax - gxo, ay - gyo)))

    print("\n" + "=" * 70)
    print("ARGMAX BRUT (sans seuil) vs GT — coords net-output, keypoints EN CADRE")
    print("=" * 70)
    print(f"{'Keypoint':10s} {'n':>4s} {'médiane':>8s} {'<2px':>7s} {'<5px':>7s} {'<10px':>7s}")
    for ki, name in enumerate(KEYPOINT_NAMES):
        short = name.replace("mycobot320_", "")
        d = np.array(dists[ki])
        if len(d) == 0:
            print(f"{short:10s}    0"); continue
        f2  = 100 * np.mean(d < 2)
        f5  = 100 * np.mean(d < 5)
        f10 = 100 * np.mean(d < 10)
        print(f"{short:10s} {len(d):4d} {np.median(d):8.1f} {f2:6.0f}% {f5:6.0f}% {f10:6.0f}%")
    print("=" * 70)
    print("Si link5/6 ont <5px élevé (≈ proximaux) → argmax bon, seuil trop strict → baisser aide.")
    print("Si link5/6 ont <5px faible (≈ détection 30%) → pics = bruit → sigma-retrain requis.")


if __name__ == "__main__":
    main()
