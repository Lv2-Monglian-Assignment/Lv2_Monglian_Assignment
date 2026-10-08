#!/usr/bin/env python3
"""실험 1-2 분석: run_live.sh 결과 폴더에서 detect_scale별 처리 FPS·proc_ms·검출 비율·CPU·온도를 계산한다.

  python3 results/param_experiments/detect_scale/analyze_live.py <결과 폴더>
측정 구간 = run_<scale>.txt의 t_start ~ t_end (top 측정과 같은 벽시계 구간). 행 선택은 검출 기록의 pub_time_s(발행 시각) 기준.
처리 FPS = 구간 안 처리 완료 프레임 수 / 구간 길이 [s] (카메라 30 fps라 30을 넘을 수 없음 → 비교는 proc_ms로 한다)
"""
import csv
import json
import statistics
import sys
from glob import glob
from pathlib import Path


def kv(path):
    return dict(line.strip().split('=', 1) for line in open(path) if '=' in line)


def cpu(top_path, pid):
    vals = []
    for line in open(top_path):
        f = line.split()
        if f and f[0] == pid and len(f) > 8:
            vals.append(float(f[8].replace(',', '.')))
    return vals[1:] if len(vals) > 1 else vals          # top 첫 샘플은 실행 시작 이후 누적값이라 뺀다


def p95(v):
    v = sorted(v)
    return v[min(len(v) - 1, int(round(0.95 * (len(v) - 1))))]


def main():
    out = Path(sys.argv[1])
    res = {}
    for run in sorted(glob(str(out / 'run_*.txt'))):
        m = kv(run)
        s = m['scale']
        t0, t1 = float(m['t_start']), float(m['t_end'])
        rows = [r for r in csv.DictReader(open(out / f"{m['run_id']}_detect.csv"))
                if r['pub_time_s'] and t0 <= float(r['pub_time_s']) <= t1]
        ms = [float(r['proc_ms']) for r in rows if r['proc_ms']]
        det = [r for r in rows if r['detected'] == '1']
        ex = [float(r['ex']) for r in det]
        c = cpu(out / f'top_{s}.txt', m['detector_pid'])
        res[s] = {'run_id': m['run_id'], 'window_s': round(t1 - t0, 2), 'frames': len(rows),
                  'processing_fps': round(len(rows) / (t1 - t0), 2),
                  'proc_ms_mean': round(statistics.mean(ms), 2), 'proc_ms_median': round(statistics.median(ms), 2),
                  'proc_ms_p95': round(p95(ms), 2), 'detect_ratio': round(len(det) / len(rows), 4) if rows else None,
                  'ex_mean': round(statistics.mean(ex), 4) if ex else None, 'ex_std': round(statistics.pstdev(ex), 4) if ex else None,
                  'cpu_pct_mean': round(statistics.mean(c), 1) if c else None, 'cpu_samples': len(c),
                  'temp_start_c': float(m['temp_start_c']), 'temp_end_c': float(m['temp_end_c']), 'commit': m['commit']}
    if '1.0' in res and len(res) > 1:
        for s, r in res.items():
            if s != '1.0':
                r['proc_ms_reduction_pct'] = round(100 * (1 - r['proc_ms_mean'] / res['1.0']['proc_ms_mean']), 1)
                r['detect_ratio_diff_pp'] = round(100 * (r['detect_ratio'] - res['1.0']['detect_ratio']), 2)
    (out / 'summary.json').write_text(json.dumps(res, ensure_ascii=False, indent=2))
    print(json.dumps(res, ensure_ascii=False, indent=2))


if __name__ == '__main__':
    main()
