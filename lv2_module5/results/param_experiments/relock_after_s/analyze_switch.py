#!/usr/bin/env python3
"""실험 3 분석: 물체 2개 장면에서 가림 회차마다 목표가 사라진 시각, 정지(미검출) 시간, 다시 고른 물체 번호·위치를 구한다.

assignment_D.py의 판정(후보가 하나도 없는 순간 = 가림)은 다른 물체가 계속 보이는 장면에 맞지 않아, 인지 기록의
detected·target_id로 직접 판단한다.
  python3 results/param_experiments/relock_after_s/analyze_switch.py <assignment_D 결과 폴더>
"""
import csv
import glob
import os
import sys


def main(d):
    marks = list(csv.DictReader(open(os.path.join(d, 'marks.csv'))))
    det = list(csv.DictReader(open(glob.glob(os.path.join(d, '*_detect.csv'))[0])))
    ctl_path = [p for p in glob.glob(os.path.join(d, 'assignment_D_*.csv')) if not p.endswith('_detect.csv')][0]
    ctl = list(csv.DictReader(open(ctl_path)))
    rows = []
    for m in marks:
        h, s = float(m['hide_prompt_t']), float(m['show_prompt_t'])
        w = [r for r in det if h - 0.5 <= float(r['stamp_s']) <= s + 3.5]
        before = [r for r in w if float(r['stamp_s']) < h and r['target_id']]
        lost = [r for r in w if float(r['stamp_s']) >= h and r['detected'] == '0']
        row = {'trial': m['trial'], 'id_before': before[-1]['target_id'] if before else '',
               'cx_before': before[-1]['cx_px'] if before else ''}
        if not lost:
            row['result'] = '가림 미확인 (추적 물체가 사라지지 않음)'
            rows.append(row)
            continue
        t0 = float(lost[0]['stamp_s'])
        after = [r for r in w if float(r['stamp_s']) > t0 and r['detected'] == '1']
        if not after:
            row.update(result='재검출 없음', lost_from_hide_s=round(t0 - h, 2))
            rows.append(row)
            continue
        t1 = float(after[0]['stamp_s'])
        lost_ctl = [r for r in ctl if t0 <= float(r['ros_time_s']) < t1 and r['state'] == 'LOST']
        row.update(lost_from_hide_s=round(t0 - h, 2), no_target_s=round(t1 - t0, 2),
                   reacquired_from_hide_s=round(t1 - h, 2), show_from_hide_s=round(s - h, 2),
                   id_after=after[0]['target_id'], cx_after=after[0]['cx_px'],
                   same_id=int(after[0]['target_id'] == row['id_before']),
                   max_abs_cmd_while_lost=max((max(abs(float(r['pan_cmd'])), abs(float(r['tilt_cmd']))) for r in lost_ctl), default=''),
                   result='재검출')
        rows.append(row)
    keys = []
    for r in rows:
        keys += [k for k in r if k not in keys]
    out = os.path.join(d, 'switch_analysis.csv')
    with open(out, 'w', newline='') as f:
        wr = csv.DictWriter(f, fieldnames=keys)
        wr.writeheader()
        wr.writerows(rows)
    for r in rows:
        print(r)


if __name__ == '__main__':
    main(sys.argv[1])
