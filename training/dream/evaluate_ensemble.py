#!/usr/bin/env python3
"""Ensemble evaluation : moyenne de 2 modèles DREAM."""

import sys
import os
import torch
import numpy as np

DREAM_DIR = "/tmp/DREAM"
sys.path.insert(0, DREAM_DIR)
import dream

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))


def load_model(weights_path):
    import dream
    from ruamel.yaml import YAML
    yaml_path = weights_path.replace(".pth", ".yaml")
    yaml_parser = YAML(typ="safe")
    with open(yaml_path) as f:
        network_config = yaml_parser.load(f)
    dream_network = dream.create_network_from_config_data(network_config)
    # Charger les poids directement
    state_dict = torch.load(weights_path, map_location="cuda")
    dream_network.model.load_state_dict(state_dict)
    dream_network.enable_evaluation()
    return dream_network

def main():
    import argparse
    parser = argparse.ArgumentParser()
    parser.add_argument("--weights1", required=True)
    parser.add_argument("--weights2", required=True)
    parser.add_argument("--data",     required=True)
    parser.add_argument("--max-samples", type=int, default=1000)
    args = parser.parse_args()

    print(f"Chargement modèle 1 : {args.weights1}")
    model1 = load_model(args.weights1)
    print(f"Chargement modèle 2 : {args.weights2}")
    model2 = load_model(args.weights2)

    found_data = dream.utilities.find_ndds_data_in_dir(args.data)
    image_raw_resolution = dream.utilities.load_image_resolution(found_data[1]["camera"])
    net_input_res, net_output_res = model1.net_resolutions_from_image_raw_resolution(image_raw_resolution)

    dataset = dream.datasets.ManipulatorNDDSDataset(
        found_data, model1.manipulator_name, model1.keypoint_names,
        net_input_res, net_output_res,
        model1.image_normalization, model1.image_preprocessing(),
        augment_data=False, include_ground_truth=True, include_belief_maps=True,
    )

    n = min(args.max_samples, len(dataset))
    print(f"\nEnsemble sur {n} frames...")

    detected = 0
    total = 0
    threshold = 20.0

    for i in range(n):
        sample = dataset[i]
        img = sample["image_rgb_input"].unsqueeze(0).cuda()

        with torch.no_grad():
            out1 = model1.model(img)
            out2 = model2.model(img)
            pred1 = out1[0] if isinstance(out1, list) else out1
            pred2 = out2[0] if isinstance(out2, list) else out2
            ensemble = (pred1 + pred2) / 2.0

        belief_maps = ensemble[0].cpu().numpy()
        gt_projected = sample["keypoint_projections_raw"]

        for kp_idx in range(len(model1.keypoint_names)):
            bmap = belief_maps[kp_idx]
            peak_idx = np.unravel_index(np.argmax(bmap), bmap.shape)
            peak_val = bmap[peak_idx]

            if peak_val > 0.01:
                pred_y = peak_idx[0] * 4
                pred_x = peak_idx[1] * 4
                gt = gt_projected[kp_idx]
                if gt[0] > -900:
                    dist = np.sqrt((pred_x - gt[0])**2 + (pred_y - gt[1])**2)
                    if dist < threshold:
                        detected += 1
                    total += 1

        if (i+1) % 100 == 0:
            print(f"  {i+1}/{n} frames...")

    pct = 100.0 * detected / max(total, 1)
    print(f"\n{'='*50}")
    print(f"ENSEMBLE RESULTS")
    print(f"  Detection rate: {detected}/{total} ({pct:.1f}%)")
    print(f"{'='*50}")

if __name__ == "__main__":
    main()