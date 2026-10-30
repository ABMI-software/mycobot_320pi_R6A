#!/usr/bin/env python3
r"""Compare DREAM on matched original/realistic renders, without moving a robot.

Usage (venv_dream):
  python scripts/compare_robot_appearance_dream.py \
    --images /tmp/robot_appearance_comparison \
    --weights training/dream/checkpoints_dream/vgg_ultimate_v4_mix_ft_e30/best_network.pth \
    --output /tmp/robot_appearance_comparison/dream

Images must be original/<id>.png and realistic/<id>.png. An optional manifest
(--manifest, or images/manifest.json if present) contains keypoint_names and
samples: [{id, keypoints: [[u,v], ...], keypoints_valid: [bool, ...]}]. Coordinates
are native image pixels; keypoints/keypoints_valid are optional. The validity
mask can exclude points behind the camera. It does not imply an occlusion test.

Detection means a non-sentinel, finite prediction inside the native frame.
PCK@5px includes ALL eligible GT points in its denominator: missed predictions
are failures. Mean/median errors include only detected, eligible GT points.
Eligibility means finite GT inside the native frame and an optional true mask.
Results describe this small synthetic comparison, not real-world improvement.
"""

import argparse
import csv
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import statistics
import sys

from PIL import Image, ImageDraw


VARIANTS = ("original", "realistic")
PCK_THRESHOLD_PX = 5.0


def finite_point(value):
    if value is None or len(value) != 2:
        return None
    try:
        point = [float(v) for v in value]
    except (TypeError, ValueError):
        return None
    return point if all(math.isfinite(v) for v in point) else None


def in_frame(point, size):
    return point is not None and 0 <= point[0] < size[0] and 0 <= point[1] < size[1]


def summarize(rows):
    errors = [r["error_px"] for r in rows if r["error_px"] is not None]
    eligible = sum(r["gt_eligible"] for r in rows)
    detected = sum(r["detected_in_frame"] for r in rows)
    correct = sum(r["correct_5px"] is True for r in rows)
    return {
        "keypoint_count": len(rows),
        "non_sentinel_count": sum(r["non_sentinel"] for r in rows),
        "detected_in_frame_count": detected,
        "detection_rate": detected / len(rows) if rows else None,
        "gt_eligible_count": eligible,
        "detected_gt_eligible_count": len(errors),
        "detection_recall_on_gt": len(errors) / eligible if eligible else None,
        "mean_error_px_detected": statistics.mean(errors) if errors else None,
        "median_error_px_detected": statistics.median(errors) if errors else None,
        "correct_5px_count": correct,
        "pck_5px": correct / eligible if eligible else None,
    }


def evaluate_points(result, gt, valid, names, order, size):
    predictions = result["detected_keypoints"]
    net_predictions = result["detected_keypoints_net_output"]
    if len(predictions) != len(order) or len(net_predictions) != len(order):
        raise ValueError("Checkpoint returned an unexpected number of keypoints")
    rows = []
    for index, (name, model_index) in enumerate(zip(names, order)):
        predicted = finite_point(predictions[model_index])
        net_point = finite_point(net_predictions[model_index])
        sentinel = net_point is not None and all(abs(v + 999.999) < 0.01 for v in net_point)
        non_sentinel = net_point is not None and predicted is not None and not sentinel
        detected = non_sentinel and in_frame(predicted, size)
        target = finite_point(gt[index]) if gt is not None else None
        eligible = in_frame(target, size) and (valid is None or valid[index])
        error = math.dist(predicted, target) if detected and eligible else None
        rows.append({
            "keypoint": name,
            "prediction": predicted,
            "prediction_net_output": net_point,
            "non_sentinel": bool(non_sentinel),
            "detected_in_frame": bool(detected),
            "ground_truth": target,
            "gt_eligible": bool(eligible),
            "error_px": error,
            "correct_5px": bool(error is not None and error <= PCK_THRESHOLD_PX) if eligible else None,
        })
    return rows


def load_inputs(images, manifest_path):
    paths = {v: {p.stem: p for p in (images / v).glob("*.png")} for v in VARIANTS}
    if not paths["original"] or paths["original"].keys() != paths["realistic"].keys():
        raise ValueError("original/ and realistic/ must contain the same nonempty set of PNG IDs")
    manifest = json.loads(manifest_path.read_text()) if manifest_path else {}
    samples = manifest.get("samples", [{"id": s} for s in sorted(paths["original"])])
    ids = [sample["id"] for sample in samples]
    if len(set(ids)) != len(ids) or set(ids) != set(paths["original"]):
        raise ValueError("Manifest IDs must be unique and match the paired PNG IDs exactly")
    return paths, manifest, samples


def keypoint_order(config, manifest):
    definitions = config["manipulator"]["keypoints"]
    names = manifest.get("keypoint_names", [p["name"] for p in definitions])
    if len(names) != len(definitions) or len(set(names)) != len(names):
        raise ValueError("keypoint_names must contain each model keypoint exactly once")
    aliases = {}
    for index, definition in enumerate(definitions):
        for key in ("name", "ros_frame", "friendly_name"):
            if definition.get(key):
                aliases[definition[key]] = index
    try:
        order = [aliases[name] for name in names]
    except KeyError as exc:
        raise ValueError(f"Unknown manifest keypoint {exc.args[0]!r}; expected {list(aliases)}") from exc
    if len(set(order)) != len(order):
        raise ValueError("Manifest names refer to the same model keypoint more than once")
    return names, order


def write_board(path, images, rows, sample_id):
    width, height = images[0].size
    header = 32
    board = Image.new("RGB", (2 * width, 2 * (height + header)), "#20252a")
    draw = ImageDraw.Draw(board)
    for column, (variant, source, points) in enumerate(zip(VARIANTS, images, rows)):
        x = column * width
        draw.text((x + 8, 8), f"{sample_id} | {variant} | native {width}x{height}", fill="white")
        board.paste(source, (x, header))
        y = height + header
        draw.text((x + 8, y + 8), "GT: green crosses | DREAM: magenta circles", fill="white")
        overlay = source.copy()
        pen = ImageDraw.Draw(overlay)
        for index, point in enumerate(points):
            if point["gt_eligible"]:
                u, v = point["ground_truth"]
                pen.line((u - 5, v, u + 5, v), fill="#00ff60", width=2)
                pen.line((u, v - 5, u, v + 5), fill="#00ff60", width=2)
            if point["detected_in_frame"]:
                u, v = point["prediction"]
                pen.ellipse((u - 4, v - 4, u + 4, v + 4), outline="#ff30d0", width=2)
                pen.text((u + 6, v + 2), str(index), fill="#ff30d0", stroke_width=1, stroke_fill="black")
        board.paste(overlay, (x, y + header))
    board.save(path)


def write_csv(path, rows):
    with path.open("w", newline="") as handle:
        writer = csv.DictWriter(handle, fieldnames=list(rows[0]))
        writer.writeheader()
        writer.writerows(rows)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--images", type=Path, required=True)
    parser.add_argument("--weights", type=Path, required=True)
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--manifest", type=Path)
    parser.add_argument("--config", type=Path, help="Default: weights path with .yaml suffix")
    parser.add_argument("--boards", type=int, default=4, help="Number of evenly spaced sample boards")
    args = parser.parse_args()
    manifest_path = args.manifest or (args.images / "manifest.json" if (args.images / "manifest.json").exists() else None)
    paths, manifest, samples = load_inputs(args.images, manifest_path)
    if not args.weights.is_file():
        parser.error(f"Checkpoint not found: {args.weights}")
    config_path = args.config or args.weights.with_suffix(".yaml")
    if not config_path.is_file():
        parser.error(f"Checkpoint config not found: {config_path}")
    args.output.mkdir(parents=True, exist_ok=True)
    os.environ.setdefault("MPLCONFIGDIR", str(args.output.resolve() / ".matplotlib"))
    os.environ.setdefault("NO_ALBUMENTATIONS_UPDATE", "1")
    if Path("/tmp/DREAM/dream").is_dir():
        sys.path.insert(0, "/tmp/DREAM")
    import dream
    import torch
    from ruamel.yaml import YAML

    config = YAML(typ="safe").load(config_path.read_text())
    names, order = keypoint_order(config, manifest)
    for sample in samples:
        for field in ("keypoints", "keypoints_valid"):
            if sample.get(field) is not None and len(sample[field]) != len(names):
                raise ValueError(f"{sample['id']}: {field} must have {len(names)} entries")
        if sample.get("keypoints_valid") is not None and any(type(v) is not bool for v in sample["keypoints_valid"]):
            raise ValueError(f"{sample['id']}: keypoints_valid must contain booleans")
    if not torch.cuda.is_available():
        parser.error("CUDA is unavailable. Upstream DREAM requires a GPU-enabled environment.")
    device = "cuda:0"
    config["training"]["platform"]["gpu_ids"] = [0]
    records = []
    board_count = min(max(args.boards, 0), len(samples))
    board_indices = {round(i * (len(samples) - 1) / max(board_count - 1, 1)) for i in range(board_count)}
    print(f"Loading {args.weights} on {device}; {len(samples)} matched pairs", flush=True)
    with torch.inference_mode():
        network = dream.create_network_from_config_data(config)
        state = torch.load(args.weights, map_location=device, weights_only=True)
        network.model.load_state_dict(state, strict=True)
        network.enable_evaluation()
        for index, sample in enumerate(samples):
            rendered, sample_rows = [], []
            for variant in VARIANTS:
                with Image.open(paths[variant][sample["id"]]) as image_file:
                    image = image_file.convert("RGB")
                if rendered and image.size != rendered[0].size:
                    raise ValueError(f"{sample['id']}: paired images have different resolutions")
                result = network.keypoints_from_image(image, image_preprocessing_override=None, debug=True)
                points = evaluate_points(result, sample.get("keypoints"), sample.get("keypoints_valid"), names, order, image.size)
                for point in points:
                    records.append({"id": sample["id"], "variant": variant, "width": image.width, "height": image.height, **point})
                rendered.append(image)
                sample_rows.append(points)
            if index in board_indices:
                write_board(args.output / f"comparison_{sample['id']}.png", rendered, sample_rows, sample["id"])
            print(f"[{index + 1}/{len(samples)}] {sample['id']}", flush=True)

    summaries = {v: summarize([r for r in records if r["variant"] == v]) for v in VARIANTS}
    per_keypoint = {v: {name: summarize([r for r in records if r["variant"] == v and r["keypoint"] == name]) for name in names} for v in VARIANTS}
    delta = {key: summaries["realistic"][key] - summaries["original"][key] if all(summaries[v][key] is not None for v in VARIANTS) else None for key in summaries["original"]}
    with args.weights.open("rb") as handle:
        weight_hash = hashlib.file_digest(handle, "sha256").hexdigest()
    report = {
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "scope": "Initial matched synthetic render comparison; does not establish improved real-world accuracy.",
        "weights": str(args.weights.resolve()), "weights_sha256": weight_hash,
        "config": str(config_path.resolve()), "config_sha256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
        "images": str(args.images.resolve()), "manifest": str(manifest_path.resolve()) if manifest_path else None,
        "device": device, "python": sys.executable, "torch_version": torch.__version__, "dream_module": dream.__file__,
        "image_preprocessing": network.image_preprocessing(), "keypoint_names": names,
        "sample_count": len(samples),
        "metric_definitions": {
            "coordinates": "Native input-image pixels; no PCK normalization. DREAM applies checkpoint preprocessing internally.",
            "non_sentinel": "Finite raw and net-output prediction, excluding DREAM's [-999.999, -999.999] net-output sentinel.",
            "detected_in_frame": "Non-sentinel prediction satisfying 0 <= u < image width and 0 <= v < image height; not a correctness measure.",
            "gt_eligible": "Finite native-image GT inside the frame and true optional keypoints_valid mask; no occlusion filtering.",
            "error_px_detected": "Euclidean pixel error on detected-in-frame predictions with eligible GT only; missing detections excluded.",
            "pck_5px": "Count of detected eligible GT points with error <= 5 native pixels, divided by ALL eligible GT points. Missed detections count as failures.",
            "delta_realistic_minus_original": "Positive detection/PCK delta is higher; negative pixel-error delta is lower. Error subsets may differ between variants.",
        },
        "summary": summaries, "per_keypoint": per_keypoint, "delta_realistic_minus_original": delta,
        "manifest_metadata": {k: v for k, v in manifest.items() if k != "samples"},
        "records": records,
    }
    (args.output / "report.json").write_text(json.dumps(report, indent=2, allow_nan=False) + "\n")
    write_csv(args.output / "summary.csv", [{"variant": v, "keypoint": "ALL", **summaries[v]} for v in VARIANTS] + [{"variant": v, "keypoint": name, **per_keypoint[v][name]} for v in VARIANTS for name in names])
    csv_records = []
    for record in records:
        flat = {k: v for k, v in record.items() if k not in ("prediction", "prediction_net_output", "ground_truth")}
        for field in ("prediction", "prediction_net_output", "ground_truth"):
            for axis, value in zip(("u", "v"), record[field] or [None, None]):
                flat[f"{field}_{axis}"] = value
        csv_records.append(flat)
    write_csv(args.output / "predictions.csv", csv_records)
    print(json.dumps(summaries, indent=2, allow_nan=False))
    print(f"Saved report.json, summary.csv, predictions.csv and {board_count} boards in {args.output}")


if __name__ == "__main__":
    main()
