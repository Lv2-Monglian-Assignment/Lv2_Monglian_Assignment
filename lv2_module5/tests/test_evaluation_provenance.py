"""Synthetic fixtures test provenance guards; these are not camera evaluation frames."""

import hashlib
import json
from pathlib import Path
import sys
import tempfile
import unittest

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws/src/target_detector"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from scripts.evaluate_perception_dataset import metrics, read_dataset


class EvaluationProvenanceTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.root = Path(self.temp.name)
        (self.root / "capture_source.py").write_text("# synthetic provenance fixture\n")
        self.data = {"success": True, "error": None, "scene": "present", "frames_saved": 2,
                     "source_sha256": hashlib.sha256((self.root / "capture_source.py").read_bytes()).hexdigest(),
                     "frames": []}
        for i in range(2):
            image = self.root / f"frame-{i}.png"
            cv2.imwrite(str(image), np.full((20, 30, 3), i, np.uint8))
            self.data["frames"].append({"file": image.name, "target_present": True,
                                        "sha256": hashlib.sha256(image.read_bytes()).hexdigest(),
                                        "width": 30, "height": 20,
                                        "header": {"stamp": {"sec": i + 1, "nanosec": 0}}})
        self.manifest = self.root / "dataset.json"

    def tearDown(self):
        self.temp.cleanup()

    def read(self):
        self.manifest.write_text(json.dumps(self.data))
        return read_dataset(self.manifest, "present", 2)

    def test_unaltered_fixture_loads(self):
        self.assertEqual(len(self.read()[2]), 2)

    def test_image_replaced_after_capture_is_rejected(self):
        (self.root / "frame-0.png").write_bytes(b"tampered")
        with self.assertRaisesRegex(ValueError, "hash mismatch"):
            self.read()

    def test_repeated_source_frame_is_rejected(self):
        self.data["frames"][1]["header"] = self.data["frames"][0]["header"]
        with self.assertRaisesRegex(ValueError, "distinct"):
            self.read()

    def test_ground_truth_disagreement_is_rejected(self):
        self.data["frames"][0]["target_present"] = False
        with self.assertRaisesRegex(ValueError, "ground truth"):
            self.read()

    def test_frame_path_cannot_escape_dataset(self):
        self.data["frames"][0]["file"] = "../outside.png"
        with self.assertRaisesRegex(ValueError, "remain"):
            self.read()

    def test_all_missed_targets_are_counted(self):
        rows = [{"classification": "FN"}] * 30 + [{"classification": "TN"}] * 10
        result = metrics(rows)
        self.assertEqual(result["FN"], 30)
        self.assertEqual(result["target_detection_rate"], 0)
        self.assertEqual(result["presence_accuracy"], 0.25)
        self.assertIsNone(result["precision"])

    def annotate_fixture(self):
        self.data["scene"] = "unlabelled"
        for frame in self.data["frames"]:
            frame["target_present"] = None
        raw = json.dumps(self.data).encode()
        (self.root / "dataset-raw.json").write_bytes(raw)
        self.data["scene"] = "present"
        for frame in self.data["frames"]:
            frame["target_present"] = True
        self.data["annotation"] = {"source_dataset": "dataset-raw.json",
                                   "source_dataset_sha256": hashlib.sha256(raw).hexdigest()}

    def test_separate_annotation_preserves_raw_evidence(self):
        self.annotate_fixture()
        self.assertEqual(len(self.read()[2]), 2)
        raw = json.loads((self.root / "dataset-raw.json").read_text())
        self.assertIsNone(raw["frames"][0]["target_present"])

    def test_annotation_cannot_change_source_stamp(self):
        self.annotate_fixture()
        self.data["frames"][0]["header"]["stamp"]["nanosec"] = 1
        with self.assertRaisesRegex(ValueError, "changed original"):
            self.read()


if __name__ == "__main__":
    unittest.main()
