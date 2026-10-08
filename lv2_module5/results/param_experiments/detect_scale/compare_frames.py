#!/usr/bin/env python3
"""실험 1-1: 같은 프레임(사람 판정이 끝난 40장)에 detect_scale 1.0과 0.5를 적용해 검출·위치·처리 시간을 비교한다.

lv2_module5 폴더에서 실행 (Raspberry Pi, 카메라·모터 사용 안 함):
  python3 results/param_experiments/detect_scale/compare_frames.py --out results/param_experiments/detect_scale/frames_<시각>
설정은 config/hsv.yaml을 읽고 detect_scale만 메모리에서 바꾼다(파일은 바꾸지 않음).
입력: 목표 있음 30장 results/logs/perception/issue34-present-color-review-001/present-*/original.png (사람 판정: 30장 모두 목표)
      목표 없음 10장 results/logs/perception/issue34-forty-frame-evaluation-002/absent-*/original.png (사람 확인: 목표 없음)
"""
import argparse
import csv
import json
import platform
import statistics
import sys
import time
from dataclasses import replace
from glob import glob
from pathlib import Path

import cv2

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / 'scripts'))
sys.path.insert(0, str(ROOT / 'ros2_ws/src/target_detector'))
from detect_image import load_config  # noqa: E402  (config/hsv.yaml -> DetectorConfig, 노드와 같은 규칙)
from target_detector.detection import detect, draw_overlay  # noqa: E402

PRESENT = 'results/logs/perception/issue34-present-color-review-001/present-*/original.png'
ABSENT = 'results/logs/perception/issue34-forty-frame-evaluation-002/absent-*/original.png'


def run(rgb, cfg, reps):
    det, _ = detect(rgb, 'rgb8', cfg)
    times = []
    for _ in range(reps):
        t0 = time.perf_counter()
        detect(rgb, 'rgb8', cfg)
        times.append((time.perf_counter() - t0) * 1e3)
    return det, statistics.mean(times)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', required=True)
    ap.add_argument('--config', default=str(ROOT / 'config/hsv.yaml'))
    ap.add_argument('--scale', type=float, default=0.5)
    ap.add_argument('--reps', type=int, default=20)
    args = ap.parse_args()
    out = Path(args.out)
    (out / 'diff_images').mkdir(parents=True, exist_ok=True)
    base = load_config(args.config)
    cfgs = {'1.0': replace(base, detect_scale=1.0), str(args.scale): replace(base, detect_scale=args.scale)}
    s = str(args.scale)
    files = [(p, 'present') for p in sorted(glob(str(ROOT / PRESENT)))] + \
            [(p, 'absent') for p in sorted(glob(str(ROOT / ABSENT)))]
    assert sum(1 for _, l in files if l == 'present') == 30 and sum(1 for _, l in files if l == 'absent') == 10, len(files)
    for _ in range(3):  # 예열 (첫 호출의 메모리 할당 시간 제외)
        detect(cv2.cvtColor(cv2.imread(files[0][0]), cv2.COLOR_BGR2RGB), 'rgb8', cfgs[s])
    rows = []
    for path, label in files:
        bgr = cv2.imread(path)
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)  # 파일은 BGR, 노드 입력(rgb8)과 같게
        r = {'frame': str(Path(path).relative_to(ROOT)), 'label': label}
        dets = {}
        for k, cfg in cfgs.items():
            det, ms = run(rgb, cfg, args.reps)
            dets[k] = det
            r.update({f'found_{k}': int(det.detected), f'ex_{k}': det.ex, f'ey_{k}': det.ey,
                      f'area_ratio_{k}': det.area_ratio, f'ms_{k}': round(ms, 3)})
        both = dets['1.0'].detected and dets[s].detected
        r['abs_dex'] = abs(dets['1.0'].ex - dets[s].ex) if both else ''
        r['abs_dey'] = abs(dets['1.0'].ey - dets[s].ey) if both else ''
        if dets['1.0'].detected != dets[s].detected:
            for k in cfgs:
                cv2.imwrite(str(out / 'diff_images' / f"{Path(path).parent.name}_scale{k}.png"), draw_overlay(bgr, dets[k]))
        rows.append(r)
    with open(out / 'frames.csv', 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys()))
        w.writeheader()
        w.writerows(rows)

    def agg(k):
        pres = [r for r in rows if r['label'] == 'present']
        absn = [r for r in rows if r['label'] == 'absent']
        return {'present_found': sum(r[f'found_{k}'] for r in pres), 'absent_found': sum(r[f'found_{k}'] for r in absn),
                'ms_mean': round(statistics.mean(r[f'ms_{k}'] for r in rows), 3),
                'ms_present_mean': round(statistics.mean(r[f'ms_{k}'] for r in pres), 3)}
    dex = [r['abs_dex'] for r in rows if r['abs_dex'] != '']
    dey = [r['abs_dey'] for r in rows if r['abs_dey'] != '']
    summary = {'date': time.strftime('%Y-%m-%d %H:%M:%S'), 'host': platform.node(), 'opencv': cv2.__version__,
               'config': str(Path(args.config).relative_to(ROOT)), 'reps': args.reps,
               'scale_1.0': agg('1.0'), f'scale_{s}': agg(s),
               'both_detected': len(dex), 'abs_dex_mean': statistics.mean(dex) if dex else None, 'abs_dex_max': max(dex, default=None),
               'abs_dey_mean': statistics.mean(dey) if dey else None, 'abs_dey_max': max(dey, default=None),
               'found_mismatch': [r['frame'] for r in rows if r['found_1.0'] != r[f'found_{s}']]}
    summary['ms_reduction_pct'] = round(100 * (1 - summary[f'scale_{s}']['ms_mean'] / summary['scale_1.0']['ms_mean']), 1)
    (out / 'summary.json').write_text(json.dumps(summary, ensure_ascii=False, indent=2))
    print(json.dumps(summary, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
