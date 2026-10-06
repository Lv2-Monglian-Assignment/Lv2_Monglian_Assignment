"""합성 이미지 계산 테스트 — 실제 카메라 측정 자료가 아님.

실행: .venv-perception/bin/python -m pytest tests  (또는 python tests/test_detector.py)
"""
import sys
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws/src/target_detector"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from target_detector.detector import DetectorConfig, detect  # noqa: E402

CFG = DetectorConfig.from_yaml(Path(__file__).resolve().parents[1] / "config/hsv.yaml")
BLUE_RGB = (0, 0, 255)


def blank(w=640, h=480):
    return np.full((h, w, 3), 200, np.uint8)


def test_no_target_is_zero():
    det, _ = detect(blank(), CFG)
    assert not det.found and (det.ex, det.ey, det.area_ratio) == (0, 0, 0)


def test_center_is_zero():
    img = blank()
    cv2.rectangle(img, (300, 220), (340, 260), BLUE_RGB, -1)
    det, _ = detect(img, CFG)
    assert det.found and abs(det.ex) < 0.01 and abs(det.ey) < 0.01


def test_right_and_down_are_positive():
    img = blank()
    cv2.rectangle(img, (500, 380), (560, 440), BLUE_RGB, -1)
    det, _ = detect(img, CFG)
    assert det.ex > 0 and det.ey > 0
    assert abs(det.ex - (530 - 320) / 320) < 0.01
    assert abs(det.ey - (410 - 240) / 240) < 0.01


def test_uses_actual_frame_size():
    img = blank(w=320, h=240)
    cv2.rectangle(img, (0, 0), (39, 39), BLUE_RGB, -1)
    det, _ = detect(img, CFG)
    assert abs(det.ex - (19.5 - 160) / 160) < 0.02
    assert abs(det.area_ratio - 39 * 39 / (320 * 240)) < 0.002


def test_picks_largest_and_ignores_small():
    img = blank()
    cv2.rectangle(img, (10, 10), (25, 25), BLUE_RGB, -1)     # 최소 면적 미만
    cv2.rectangle(img, (100, 100), (140, 140), BLUE_RGB, -1)
    cv2.rectangle(img, (400, 300), (500, 400), BLUE_RGB, -1)  # 가장 큼
    det, _ = detect(img, CFG)
    assert abs(det.cx - 450) < 1 and abs(det.cy - 350) < 1


def test_small_only_is_not_detected():
    img = blank()
    cv2.rectangle(img, (10, 10), (25, 25), BLUE_RGB, -1)
    det, _ = detect(img, CFG)
    assert not det.found


def test_rgb_not_bgr():
    img = blank()
    cv2.rectangle(img, (300, 220), (340, 260), (255, 0, 0), -1)  # RGB 빨강 → 검출되면 안 됨
    det, _ = detect(img, CFG)
    assert not det.found


if __name__ == "__main__":
    tests = [v for k, v in dict(globals()).items() if k.startswith("test_")]
    for t in tests:
        t()
        print("PASS", t.__name__)
    print(f"{len(tests)}개 통과 (합성 계산 테스트)")
