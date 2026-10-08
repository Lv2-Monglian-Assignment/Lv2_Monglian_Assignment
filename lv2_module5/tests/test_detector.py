"""합성 이미지 계산 테스트 — 실제 카메라 측정 자료가 아님.

실행: .venv-perception/bin/python -m pytest tests  (또는 python tests/test_detector.py)
검출 코드는 패키지의 target_detector/detection.py, 설정은 config/hsv.yaml(target_detector 노드 항목)을 그대로 쓴다.
깊이 영상이 없으므로 크기 검증(size_check)은 판정 없이 후보를 남긴다(노드와 같은 규칙).
"""
import sys
from dataclasses import fields
from pathlib import Path

import cv2
import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws/src/target_detector"))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from target_detector.detection import DetectorConfig, detect as _detect  # noqa: E402


def load_config(path):
    """config yaml의 /** 공통 값 + target_detector 노드 값을 DetectorConfig로 바꾼다(노드가 받는 값과 같음)."""
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    params = {}
    for key in ("/**", "target_detector"):
        params.update((data.get(key) or {}).get("ros__parameters") or {})
    names = {f.name for f in fields(DetectorConfig)}
    return DetectorConfig(**{k: tuple(v) if k in ("hsv_lower", "hsv_upper") else v
                             for k, v in params.items() if k in names})


def detect(rgb, cfg):
    return _detect(rgb, "rgb8", cfg)


CFG = load_config(Path(__file__).resolve().parents[1] / "config/hsv.yaml")
BLUE_RGB = (30, 110, 220)   # OpenCV HSV (107, 220, 220): hsv.yaml의 실측 범위 H 102~110 안의 파랑


def blank(w=640, h=480):
    return np.full((h, w, 3), 200, np.uint8)


def test_no_target_is_zero():
    det, _ = detect(blank(), CFG)
    assert not det.detected and (det.ex, det.ey, det.area_ratio) == (0, 0, 0)


def test_center_is_zero():
    img = blank()
    cv2.rectangle(img, (300, 220), (340, 260), BLUE_RGB, -1)
    det, _ = detect(img, CFG)
    assert det.detected and abs(det.ex) < 0.01 and abs(det.ey) < 0.01


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
    cv2.rectangle(img, (10, 10), (17, 17), BLUE_RGB, -1)     # 최소 면적(min_area_px 100) 미만: 윤곽 면적 49 px^2
    cv2.rectangle(img, (100, 100), (140, 140), BLUE_RGB, -1)
    cv2.rectangle(img, (400, 300), (500, 400), BLUE_RGB, -1)  # 가장 큼
    det, _ = detect(img, CFG)
    assert abs(det.cx - 450) < 1 and abs(det.cy - 350) < 1


def test_small_only_is_not_detected():
    img = blank()
    cv2.rectangle(img, (10, 10), (17, 17), BLUE_RGB, -1)
    det, _ = detect(img, CFG)
    assert not det.detected


def test_rgb_not_bgr():
    img = blank()
    cv2.rectangle(img, (300, 220), (340, 260), (255, 0, 0), -1)  # RGB 빨강 → 검출되면 안 됨
    det, _ = detect(img, CFG)
    assert not det.detected


if __name__ == "__main__":
    tests = [v for k, v in dict(globals()).items() if k.startswith("test_")]
    for t in tests:
        t()
        print("PASS", t.__name__)
    print(f"{len(tests)}개 통과 (합성 계산 테스트)")
