#!/usr/bin/env python3
"""tracker 기록 CSV에서 시험 구간만 앞뒤 5줄을 붙여 따로 저장하고 1초 요약표를 만든다 (Pi에서 실행).

사용: python3 extract_window.py <run_id> <시험이름> <시작 시각> <끝 시각>
  시각은 Pi 벽시계 초(date +%s.%N)이며 CSV의 ros_time_s 열과 비교한다.
출력: ~/lv2_module5_logs/<run_id>_<시험이름>.csv, <run_id>_<시험이름>_summary.txt
전체 기록(<run_id>.csv)은 지우거나 바꾸지 않는다.
"""
import csv
import os
import subprocess
import sys

MARGIN_ROWS = 5
run_id, label, t_start, t_end = sys.argv[1], sys.argv[2], float(sys.argv[3]), float(sys.argv[4])
log_dir = os.path.expanduser('~/lv2_module5_logs')
src = os.path.join(log_dir, f'{run_id}.csv')
dst = os.path.join(log_dir, f'{run_id}_{label}.csv')

with open(src) as f:
    reader = csv.reader(f)
    header = next(reader)
    rows = list(reader)
col = header.index('ros_time_s')
inside = [i for i, r in enumerate(rows) if r[col] and t_start <= float(r[col]) <= t_end]
if not inside:
    sys.exit(f'구간 안 기록 없음: {t_start} ~ {t_end}')
lo = max(0, inside[0] - MARGIN_ROWS)
hi = min(len(rows), inside[-1] + 1 + MARGIN_ROWS)
with open(dst, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(header)
    w.writerows(rows[lo:hi])
print(f'저장: {dst}  ({hi - lo}행 = 구간 {len(inside)}행 + 앞뒤 {MARGIN_ROWS}행, {t_end - t_start:.1f}s)')
subprocess.run([sys.executable, os.path.join(os.path.dirname(os.path.abspath(__file__)), 'summarize_run.py'),
                dst, '--save'], check=False)
