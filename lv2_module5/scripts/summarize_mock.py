#!/usr/bin/env python3
"""mock_inputs_test.sh 결과 CSV(경우별 tracker 기록)를 읽어 판정표를 만든다.

사용: python3 summarize_mock.py <로그 폴더> <TAG>
출력: 화면 표 + <로그 폴더>/<TAG>_summary.csv
판정 기준(config/control.yaml·safety.yaml: pan_direction=-1, tilt_direction=+1, Kp 20/15, 데드밴드 0.03/0.05, 타임아웃 0.5 s)
"""
import csv
import glob
import os
import statistics
import sys

log_dir, tag = sys.argv[1], sys.argv[2]
CASES = {
    'c1_center':   ('x=0, z>0', 'TRACKING', lambda p, t: p == 0 and t == 0, '팬·틸트 0'),
    'c2_right':    ('x=+0.4, z>0', 'TRACKING', lambda p, t: p < 0 and t == 0, '팬 음수(오른쪽)'),
    'c3_left':     ('x=-0.4, z>0', 'TRACKING', lambda p, t: p > 0 and t == 0, '팬 양수(왼쪽)'),
    'c4_nodetect': ('z=0', 'LOST', lambda p, t: p == 0 and t == 0, 'LOST:no_detection, 0'),
    'c5_silence':  ('발행 중단', 'LOST', lambda p, t: p == 0 and t == 0, '0.5 s 뒤 LOST:input_timeout, 0'),
    'c6_down':     ('y=+0.4, z>0', 'TRACKING', lambda p, t: p == 0 and t > 0, '틸트 양수(아래)'),
}


def fnum(v):
    try:
        return float(v)
    except ValueError:
        return float('nan')


summary = []
for name, (desc, want_state, ok_cmd, expect) in CASES.items():
    files = glob.glob(os.path.join(log_dir, f'{tag}_{name}.csv'))
    if not files:
        summary.append([name, desc, expect, 'CSV 없음', '', '', '', 'FAIL'])
        continue
    rows = list(csv.DictReader(open(files[0])))
    fed = [r for r in rows if int(r['target_seq'] or 0) > 0]   # 첫 입력 이후
    if not fed:
        summary.append([name, desc, expect, '입력 수신 없음', '', '', '', 'FAIL'])
        continue
    t_first = fnum(fed[0]['time_s'])
    note = ''
    if name == 'c5_silence':
        # 발행이 끊긴 뒤: 마지막 신선한 입력 이후 처음 input_timeout이 된 행의 target_age = 감지 지연
        to = [r for r in fed if r['reason'] == 'input_timeout']
        if to:
            judge_rows = [r for r in fed if fnum(r['time_s']) >= fnum(to[0]['time_s'])]
            note = f'타임아웃 감지: 마지막 입력 후 {fnum(to[0]["target_age_s"]):.3f} s'
            before = [r for r in fed if fnum(r['time_s']) < fnum(to[0]['time_s']) and r['state'] == 'TRACKING']
            if before:
                note += f', 중단 전 팬 {statistics.median(fnum(r["pan_cmd"]) for r in before):+.2f} deg/s'
        else:
            judge_rows = []
            note = 'input_timeout 없음'
    else:
        # 안정 구간: 첫 입력 0.3 s 뒤부터, 입력이 계속 신선한 동안(target_age < 0.1 s)
        judge_rows = [r for r in fed if fnum(r['time_s']) >= t_first + 0.3 and fnum(r['target_age_s']) < 0.1]
    if not judge_rows:
        summary.append([name, desc, expect, '판정 구간 없음', '', '', note, 'FAIL'])
        continue
    states = sorted({f"{r['state']}:{r['reason']}" for r in judge_rows})
    pan = statistics.median(fnum(r['pan_cmd']) for r in judge_rows)
    tilt = statistics.median(fnum(r['tilt_cmd']) for r in judge_rows)
    all_state_ok = all(r['state'] == want_state for r in judge_rows)
    all_cmd_ok = all(ok_cmd(fnum(r['pan_cmd']), fnum(r['tilt_cmd'])) for r in judge_rows)
    verdict = 'PASS' if all_state_ok and all_cmd_ok else 'FAIL'
    summary.append([name, desc, expect, ' '.join(states), f'{pan:+.2f}', f'{tilt:+.2f}',
                    note + f' (판정 행 {len(judge_rows)})', verdict])

header = ['case', 'input', 'expected', 'state:reason', 'pan_cmd_deg_s', 'tilt_cmd_deg_s', 'note', 'verdict']
out = os.path.join(log_dir, f'{tag}_summary.csv')
with open(out, 'w', newline='') as f:
    w = csv.writer(f)
    w.writerow(header)
    w.writerows(summary)
for r in summary:
    print(f'{r[7]:4s} {r[0]:12s} {r[1]:12s} 기대: {r[2]:28s} → {r[3]}  팬 {r[4]} 틸트 {r[5]}  {r[6]}')
print('판정표:', out)
