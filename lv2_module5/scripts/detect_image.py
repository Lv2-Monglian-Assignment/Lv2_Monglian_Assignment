"""저장된 실제 사진에 검출을 돌려 원본·마스크·검출 이미지를 저장한다 (요구사항 10).

사용:
  .venv-perception/bin/python tools/detect_image.py results/<폴더>/*.png --out results/<장면이름>
"""
import argparse
import json
import sys
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "ros2_ws/src/target_detector"))
from target_detector.detector import DetectorConfig, detect, draw  # noqa: E402

ROOT = Path(__file__).resolve().parents[1]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("images", nargs="+")
    ap.add_argument("--out", required=True)
    ap.add_argument("--config", default=str(ROOT / "config/hsv.yaml"))
    args = ap.parse_args()

    cfg = DetectorConfig.from_yaml(args.config)
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    rows = []
    for p in map(Path, args.images):
        bgr = cv2.imread(str(p))
        if bgr is None:
            print(f"읽기 실패: {p}")
            continue
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)  # 파일은 BGR로 읽히므로 ROS rgb8과 같게 변환
        det, mask = detect(rgb, cfg)
        cv2.imwrite(str(out / f"{p.stem}_original.png"), bgr)
        cv2.imwrite(str(out / f"{p.stem}_mask.png"), mask)
        cv2.imwrite(str(out / f"{p.stem}_detection.png"), draw(bgr, det))
        row = {"image": str(p), "found": det.found, "x": det.ex, "y": det.ey, "z": det.area_ratio}
        rows.append(row)
        print(json.dumps(row, ensure_ascii=False))
    (out / "detections.json").write_text(json.dumps(
        {"config": args.config, "hsv_lower": cfg.hsv_lower, "hsv_upper": cfg.hsv_upper,
         "kernel_size": cfg.kernel_size, "min_area_px": cfg.min_area_px, "results": rows},
        ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
