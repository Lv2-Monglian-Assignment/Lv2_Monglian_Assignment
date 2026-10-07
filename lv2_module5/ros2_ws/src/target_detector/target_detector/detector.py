"""HSV + Contour 목표 검출 (ROS 비의존 순수 함수).

요구사항 1~3, 5:
  HSV 마스크 → open/close → 컨투어 → 최소 면적 이상 중 최대 선택
  ex=(cx-W/2)/(W/2), ey=(cy-H/2)/(H/2), area_ratio=area/(W*H)
  오른쪽·아래가 양수. 미검출이면 (0, 0, 0).
"""
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml


@dataclass
class DetectorConfig:
    hsv_lower: tuple
    hsv_upper: tuple
    kernel_size: int = 5
    min_area_px: float = 400.0
    input_encoding: str = "rgb8"

    @classmethod
    def from_yaml(cls, path):
        params = yaml.safe_load(Path(path).read_text())["target_detector"]["ros__parameters"]
        return cls(
            hsv_lower=tuple(params["hsv_lower"]),
            hsv_upper=tuple(params["hsv_upper"]),
            kernel_size=int(params["kernel_size"]),
            min_area_px=float(params["min_area_px"]),
            input_encoding=params.get("input_encoding", "rgb8"),
        )


@dataclass
class Detection:
    found: bool
    ex: float = 0.0
    ey: float = 0.0
    area_ratio: float = 0.0
    cx: float = 0.0
    cy: float = 0.0
    contour: np.ndarray | None = None


def make_mask(image, cfg):
    code = cv2.COLOR_RGB2HSV if cfg.input_encoding == "rgb8" else cv2.COLOR_BGR2HSV
    hsv = cv2.cvtColor(image, code)
    mask = cv2.inRange(hsv, np.array(cfg.hsv_lower, np.uint8), np.array(cfg.hsv_upper, np.uint8))
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (cfg.kernel_size, cfg.kernel_size))
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)


def detect(image, cfg):
    """image: HxWx3 (cfg.input_encoding 순서). 실제 프레임 크기로 정규화한다."""
    h, w = image.shape[:2]
    mask = make_mask(image, cfg)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    candidates = [c for c in contours if cv2.contourArea(c) >= cfg.min_area_px]
    if not candidates:
        return Detection(found=False), mask

    best = max(candidates, key=cv2.contourArea)
    area = cv2.contourArea(best)
    m = cv2.moments(best)
    cx, cy = m["m10"] / m["m00"], m["m01"] / m["m00"]
    return Detection(
        found=True,
        ex=(cx - w / 2) / (w / 2),
        ey=(cy - h / 2) / (h / 2),
        area_ratio=area / (w * h),
        cx=cx,
        cy=cy,
        contour=best,
    ), mask


def draw(image_bgr, det):
    out = image_bgr.copy()
    h, w = out.shape[:2]
    cv2.drawMarker(out, (w // 2, h // 2), (255, 255, 255), cv2.MARKER_CROSS, 20, 1)
    if det.found:
        cv2.drawContours(out, [det.contour], -1, (0, 255, 0), 2)
        cv2.circle(out, (int(det.cx), int(det.cy)), 5, (0, 0, 255), -1)
        label = f"ex={det.ex:+.3f} ey={det.ey:+.3f} area={det.area_ratio:.4f}"
    else:
        label = "no target (0, 0, 0)"
    cv2.putText(out, label, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
    return out
