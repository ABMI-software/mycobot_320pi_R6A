#!/usr/bin/env python3
"""Evaluate a DREAM model with relaxed peak-detection thresholds.

Wraps evaluate_dream.py and monkey-patches two knobs that normally
reject low-confidence peaks:

  - dream.image_proc.peaks_from_belief_maps(thresh_map_after_gaussian_filter)
    hardcoded at 0.01 in the vendored library. Lowered via function replacement.
  - dream_net.belief_peak_next_best_score (default 0.25). Lowered on the
    live instance after the network is loaded.

Usage mirrors evaluate_dream.py plus two new flags:

  --peak-thresh 0.001        # belief-map threshold (default: 0.001)
  --next-best-score 0.05     # multi-peak disambiguation margin (default: 0.05)

No changes to /tmp/DREAM/ library code.
"""
from __future__ import annotations

import argparse
import os
import sys

import numpy as np
import torch
from scipy.ndimage import gaussian_filter

sys.path.insert(0, "/tmp/DREAM")
import dream
import dream.image_proc as _ip

import evaluate_dream as _evd


def make_relaxed_peaks(peak_thresh: float):
    """Return a drop-in replacement for dream.image_proc.peaks_from_belief_maps
    with a configurable intensity threshold."""

    sigma = 3  # matches upstream

    def peaks_from_belief_maps_relaxed(belief_map_tensor, offset_due_to_upsampling):
        all_peaks = []
        for j in range(belief_map_tensor.size()[0]):
            belief_map = belief_map_tensor[j].clone()
            map_ori = belief_map.cpu().data.numpy()

            m = gaussian_filter(map_ori, sigma=sigma)
            p = 1
            map_left = np.zeros(m.shape);  map_left[p:, :]  = m[:-p, :]
            map_right = np.zeros(m.shape); map_right[:-p, :] = m[p:, :]
            map_up = np.zeros(m.shape);    map_up[:, p:]    = m[:, :-p]
            map_down = np.zeros(m.shape);  map_down[:, :-p] = m[:, p:]

            peaks_binary = np.logical_and.reduce((
                m >= map_left, m >= map_right,
                m >= map_up,   m >= map_down,
                m > peak_thresh,
            ))
            peaks = list(zip(np.nonzero(peaks_binary)[1], np.nonzero(peaks_binary)[0]))

            win = 5; ran = win // 2
            peaks_avg = []
            for p_value in range(len(peaks)):
                pp = peaks[p_value]
                weights = np.zeros((win, win))
                i_values = np.zeros((win, win))
                j_values = np.zeros((win, win))
                for i in range(-ran, ran + 1):
                    for jj in range(-ran, ran + 1):
                        if (pp[1] + i < 0 or pp[1] + i >= map_ori.shape[0]
                                or pp[0] + jj < 0 or pp[0] + jj >= map_ori.shape[1]):
                            continue
                        i_values[jj + ran, i + ran] = pp[1] + i
                        j_values[jj + ran, i + ran] = pp[0] + jj
                        weights[jj + ran, i + ran] = map_ori[pp[1] + i, pp[0] + jj]

                try:
                    peaks_avg.append((
                        np.average(j_values, weights=weights) + offset_due_to_upsampling,
                        np.average(i_values, weights=weights) + offset_due_to_upsampling,
                        float(m[pp[1], pp[0]]),
                    ))
                except Exception:
                    peaks_avg.append((
                        float(pp[0]) + offset_due_to_upsampling,
                        float(pp[1]) + offset_due_to_upsampling,
                        float(m[pp[1], pp[0]]),
                    ))
            all_peaks.append(peaks_avg)
        return all_peaks

    return peaks_from_belief_maps_relaxed


def main():
    parser = argparse.ArgumentParser(description="Evaluate DREAM with relaxed peaks")
    parser.add_argument("--weights", "-w", required=True)
    parser.add_argument("--data", "-d", required=True)
    parser.add_argument("--split", "-s", default="val",
                        choices=["train", "val", "all"])
    parser.add_argument("--max-samples", "-n", type=int, default=500)
    parser.add_argument("--viz-dir", default=None)
    parser.add_argument("--max-viz", type=int, default=50)
    parser.add_argument("--visualize", action="store_true")
    parser.add_argument("--peak-thresh", type=float, default=0.001,
                        help="Belief-map intensity threshold (library default 0.01)")
    parser.add_argument("--next-best-score", type=float, default=0.05,
                        help="Multi-peak disambiguation margin (library default 0.25)")
    args = parser.parse_args()

    if args.visualize and not args.viz_dir:
        args.viz_dir = "/tmp/dream_eval_viz_relaxed"

    # ---- Monkey-patch 1: replace peaks_from_belief_maps with relaxed version
    _ip.peaks_from_belief_maps = make_relaxed_peaks(args.peak_thresh)

    # ---- Monkey-patch 2: every DreamNetwork loaded from now on gets the
    #                      lowered next_best_score. We hook the loader.
    _orig_create = dream.create_network_from_config_file
    next_best = args.next_best_score

    def _create_patched(*a, **kw):
        net = _orig_create(*a, **kw)
        net.belief_peak_next_best_score = next_best
        return net

    dream.create_network_from_config_file = _create_patched

    print(f"[relaxed] peak_thresh = {args.peak_thresh}")
    print(f"[relaxed] next_best_score = {args.next_best_score}")
    print()

    _evd.evaluate(args)


if __name__ == "__main__":
    main()
