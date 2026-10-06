"""Evaluate labelled real color frames with one frozen HSV configuration."""

import argparse
import csv
import hashlib
import json
from pathlib import Path
import shutil
import sys

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws/src/target_detector"))
from target_detector.detector import DetectorConfig, detect, draw


def read_dataset(path, scene, expected_count):
    path = Path(path).resolve()
    data = json.loads(path.read_text())
    if not data.get("success") or data.get("error") or data.get("scene") != scene:
        raise ValueError(f"Incomplete or differently labelled dataset: {path}")
    frames = data.get("frames", [])
    if len(frames) != expected_count or data.get("frames_saved") != expected_count:
        raise ValueError(f"Expected {expected_count} saved frames: {path}")
    if data.get("annotation"):
        annotation = data["annotation"]
        raw_path = (path.parent / annotation["source_dataset"]).resolve()
        if raw_path.parent != path.parent or raw_path == path:
            raise ValueError("Raw annotation source must be another file in the dataset directory")
        raw_bytes = raw_path.read_bytes()
        if hashlib.sha256(raw_bytes).hexdigest() != annotation["source_dataset_sha256"]:
            raise ValueError("Raw annotation source hash mismatch")
        raw_data = json.loads(raw_bytes)
        raw_frames = raw_data.get("frames", [])
        if raw_data.get("scene") != "unlabelled" or len(raw_frames) != len(frames):
            raise ValueError("Annotation does not match unlabelled original")
        for raw_frame, labelled_frame in zip(raw_frames, frames):
            raw_values = {k: v for k, v in raw_frame.items() if k != "target_present"}
            labelled_values = {k: v for k, v in labelled_frame.items() if k != "target_present"}
            if raw_frame.get("target_present") is not None or raw_values != labelled_values:
                raise ValueError("Annotation changed original frame evidence")
    source = path.parent / "capture_source.py"
    if hashlib.sha256(source.read_bytes()).hexdigest() != data["source_sha256"]:
        raise ValueError(f"Capture source hash mismatch: {source}")
    previous = -1
    loaded = []
    for frame in frames:
        if frame.get("target_present") is not (scene == "present"):
            raise ValueError("Frame ground truth disagrees with physical scene label")
        image_path = (path.parent / frame["file"]).resolve()
        if image_path.parent != path.parent:
            raise ValueError("Frame path must remain in dataset directory")
        if hashlib.sha256(image_path.read_bytes()).hexdigest() != frame["sha256"]:
            raise ValueError(f"Image hash mismatch: {image_path}")
        stamp = frame["header"]["stamp"]
        value = stamp["sec"] * 1_000_000_000 + stamp["nanosec"]
        if stamp["sec"] <= 0 or not 0 <= stamp["nanosec"] < 1_000_000_000 or value <= previous:
            raise ValueError("Source stamps must be valid, distinct and increasing")
        previous = value
        bgr = cv2.imread(str(image_path))
        if bgr is None or bgr.shape != (frame["height"], frame["width"], 3):
            raise ValueError(f"Image dimensions disagree with metadata: {image_path}")
        loaded.append((frame, image_path, bgr))
    return path, data, loaded


def metrics(rows):
    counts = {name: sum(r["classification"] == name for r in rows) for name in ("TP", "FN", "FP", "TN")}
    tp, fn, fp, tn = (counts[n] for n in ("TP", "FN", "FP", "TN"))
    return {**counts, "target_detection_rate": tp / (tp + fn),
            "false_positive_rate_on_absent": fp / (fp + tn),
            "presence_accuracy": (tp + tn) / len(rows),
            "precision": tp / (tp + fp) if tp + fp else None}


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--present", required=True, help="Present scene dataset.json (30 frames)")
    ap.add_argument("--absent", required=True, help="Absent scene dataset.json (10 frames)")
    ap.add_argument("--config", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--exclude-images", nargs="*", default=[], help="Tuning/earlier scene images to exclude")
    args = ap.parse_args()
    datasets = [read_dataset(args.present, "present", 30), read_dataset(args.absent, "absent", 10)]
    all_frames = [item for _, _, loaded in datasets for item in loaded]
    stamps = [(f["header"]["frame_id"], f["header"]["stamp"]["sec"],
               f["header"]["stamp"]["nanosec"]) for f, _, _ in all_frames]
    if len(set(stamps)) != 40:
        raise ValueError("Source frames overlap between scenes")
    formats = {(f["width"], f["height"], f["encoding"]) for f, _, _ in all_frames}
    if len(formats) != 1:
        raise ValueError("Both scenes must use the same image dimensions and encoding")
    excluded = {hashlib.sha256(Path(p).read_bytes()).hexdigest() for p in args.exclude_images}
    if any(f["sha256"] in excluded for f, _, _ in all_frames):
        raise ValueError("Evaluation contains an excluded tuning/earlier image")
    config_path = Path(args.config).resolve()
    cfg = DetectorConfig.from_yaml(config_path)
    out = Path(args.out).resolve()
    out.mkdir(parents=True, exist_ok=False)
    shutil.copyfile(config_path, out / "perception-used.yaml")
    shutil.copyfile(__file__, out / "evaluation_source.py")
    detector_path = Path(__file__).resolve().parents[1] / "ros2_ws/src/target_detector/target_detector/detector.py"
    shutil.copyfile(detector_path, out / "detector_source.py")
    rows, tiles = [], []
    for manifest, metadata, loaded in datasets:
        scene = metadata["scene"]
        shutil.copyfile(manifest, out / (scene + "-dataset.json"))
        for index, (frame, path, bgr) in enumerate(loaded, 1):
            image = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB) if cfg.input_encoding == "rgb8" else bgr
            detection, mask = detect(image, cfg)
            truth = frame["target_present"]
            classification = ("TP" if detection.found else "FN") if truth else ("FP" if detection.found else "TN")
            stem = f"{scene}-{index:03d}"
            images = out / stem
            images.mkdir()
            shutil.copyfile(path, images / "original.png")
            for name, array in (("mask.png", mask), ("detection.png", draw(bgr, detection))):
                if not cv2.imwrite(str(images / name), array):
                    raise RuntimeError("Result image write failed")
            rows.append({"frame": stem, "original_sha256": frame["sha256"], "source": str(path),
                         "target_present": truth, "found": detection.found, "classification": classification,
                         "x": detection.ex, "y": detection.ey, "z": detection.area_ratio,
                         "cx": detection.cx, "cy": detection.cy, "header": frame["header"]})
            tile = cv2.resize(bgr, (240, 180))
            cv2.putText(tile, stem, (5, 18), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 255, 255), 1)
            tiles.append(tile)
    for start in range(0, 40, 10):
        batch = tiles[start:start + 10]
        sheet = np.vstack([np.hstack(batch[i:i + 5]) for i in (0, 5)])
        if not cv2.imwrite(str(out / f"original-contact-{start + 1:02d}-{start + 10:02d}.png"), sheet):
            raise RuntimeError("Contact sheet write failed")
    summary = {"purpose": "Frame-level target presence evaluation; not localization or tracking accuracy",
               "config_sha256": hashlib.sha256(config_path.read_bytes()).hexdigest(),
               "detector_sha256": hashlib.sha256(detector_path.read_bytes()).hexdigest(),
               "evaluation_source_sha256": hashlib.sha256(Path(__file__).read_bytes()).hexdigest(),
               "opencv_version": cv2.__version__, "input_formats": [list(f) for f in formats],
               "ground_truth_basis": {d["scene"]: d["label_basis"] for _, d, _ in datasets},
               "excluded_training_hashes": sorted(excluded), "metrics": metrics(rows), "frames": rows}
    (out / "evaluation.json").write_text(json.dumps(summary, ensure_ascii=False, indent=2) + "\n")
    fields = ("frame", "target_present", "found", "classification", "x", "y", "z", "cx", "cy")
    with (out / "frames.csv").open("w", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fields, extrasaction="ignore")
        writer.writeheader()
        writer.writerows(rows)
    print(json.dumps({"metrics": summary["metrics"], "output": str(out)}, ensure_ascii=False))


if __name__ == "__main__":
    main()
