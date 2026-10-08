"""저장된 실제 사진에 검출을 돌려 원본·마스크·검출 이미지를 저장한다 (요구사항 10).

사용:
  .venv-perception/bin/python scripts/detect_image.py results/<폴더>/*.png --out results/<장면이름>
검출 코드는 패키지의 target_detector/detection.py를 그대로 쓴다. 컬러 사진만 쓰므로 크기 검증(size_check)은 판정 없이 후보를 남긴다.
"""
import argparse
import json
import sys
from dataclasses import fields
from pathlib import Path

import cv2
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws/src/target_detector"))
from target_detector.detection import DetectorConfig, detect, draw_overlay  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]


def load_config(path):
    """config yaml의 /** 공통 값 + target_detector 노드 값을 DetectorConfig로 바꾼다(노드가 받는 값과 같음)."""
    data = yaml.safe_load(Path(path).read_text(encoding="utf-8")) or {}
    params = {}
    for key in ("/**", "target_detector"):
        params.update((data.get(key) or {}).get("ros__parameters") or {})
    names = {f.name for f in fields(DetectorConfig)}
    return DetectorConfig(**{k: tuple(v) if k in ("hsv_lower", "hsv_upper") else v
                             for k, v in params.items() if k in names})


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("images", nargs="+")
    ap.add_argument("--out", required=True)
    ap.add_argument("--config", default=str(ROOT / "config/hsv.yaml"))
    args = ap.parse_args()

    cfg = load_config(args.config)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    rows = []
    for p in map(Path, args.images):
        bgr = cv2.imread(str(p))
        if bgr is None:
            print(f"읽기 실패: {p}")
            continue
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)  # 파일은 BGR로 읽히므로 ROS rgb8과 같게 변환
        det, mask = detect(rgb, "rgb8", cfg)
        cv2.imwrite(str(out / f"{p.stem}_original.png"), bgr)
        cv2.imwrite(str(out / f"{p.stem}_mask.png"), mask)
        cv2.imwrite(str(out / f"{p.stem}_detection.png"), draw_overlay(bgr, det))
        row = {"image": str(p), "found": det.detected, "x": det.ex, "y": det.ey, "z": det.area_ratio}
        rows.append(row)
        print(json.dumps(row, ensure_ascii=False))
    (out / "detections.json").write_text(json.dumps(
        {"config": args.config, "hsv_lower": cfg.hsv_lower, "hsv_upper": cfg.hsv_upper,
         "kernel_size": cfg.morph_kernel, "min_area_px": cfg.min_area_px, "detect_scale": cfg.detect_scale,
         "results": rows},
        ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
