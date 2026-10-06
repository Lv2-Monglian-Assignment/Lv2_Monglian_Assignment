#!/usr/bin/env python3
"""controller_node 기록 CSV(~/lv2_module5_logs/<run_id>.csv)를 1초 단위 표로 요약한다 (Pi에서 실행, ROS 불필요).

사용: python3 summarize_run.py <run_id 또는 CSV 경로> [--last 초] [--save]
  --last N : 마지막 N초만 (기본: 전체)
  --save   : 같은 폴더에 <run_id>_summary.txt로도 저장
열: 상태(가장 많은 것), 검출 행 비율, ex·ey 평균(검출 행), 팬·틸트 명령 평균 [deg/s], 마지막 팬·틸트 각도 [deg]
"""
import csv
import math
import os
import sys
from collections import Counter

args = [a for a in sys.argv[1:] if not a.startswith('--')]
last = None
if '--last' in sys.argv:
    last = float(sys.argv[sys.argv.index('--last') + 1])
    args = [a for a in args if a != sys.argv[sys.argv.index('--last') + 1]]
path = args[0]
if not path.endswith('.csv'):
    path = os.path.expanduser(f'~/lv2_module5_logs/{path}.csv')


def num(v):
    try:
        return float(v)
    except ValueError:
        return math.nan


def mean(vals):
    vals = [v for v in vals if math.isfinite(v)]
    return sum(vals) / len(vals) if vals else math.nan


rows = list(csv.DictReader(open(path)))
if not rows:
    sys.exit('기록 없음')
t_end = num(rows[-1]['time_s'])
if last:
    rows = [r for r in rows if num(r['time_s']) >= t_end - last]
t0 = num(rows[0]['time_s'])
lines = [f'{os.path.basename(path)}  {len(rows)}행  {t_end - t0:.1f}s',
         ' 초 | 상태                     | 검출% |    ex     ey | 팬cmd 틸트cmd | 팬deg  틸트deg']
for k in range(int(t_end - t0) + 1):
    sec = [r for r in rows if t0 + k <= num(r['time_s']) < t0 + k + 1]
    if not sec:
        continue
    det = [r for r in sec if r['detected'] == '1']
    state = Counter(f"{r['state']}:{r['reason']}" for r in sec).most_common(1)[0][0]
    lines.append(f"{k:3d} | {state:24s} | {100 * len(det) / len(sec):4.0f}% | "
                 f"{mean(num(r['ex']) for r in det):+.3f} {mean(num(r['ey']) for r in det):+.3f} | "
                 f"{mean(num(r['pan_cmd']) for r in sec):+6.2f} {mean(num(r['tilt_cmd']) for r in sec):+6.2f} | "
                 f"{num(sec[-1]['pan_deg']):+6.2f} {num(sec[-1]['tilt_deg']):+6.2f}")
states = Counter(f"{r['state']}:{r['reason']}" for r in rows)
lines.append('상태 분포: ' + ', '.join(f'{k} {v}' for k, v in states.most_common(5)))
text = '\n'.join(lines)
print(text)
if '--save' in sys.argv:
    out = path[:-4] + '_summary.txt'
    open(out, 'w').write(text + '\n')
    print('저장:', out)
