#!/usr/bin/env python3
"""문제 3 — 객체 중심 기반 추적 제어: Kp 2종 × 3회를 같은 순서로 시험하고 오차·명령·실제 각도를 비교한다.

시험 순서(발제 권장): 목표를 왼쪽 표시 3 s → 중앙 3 s → 오른쪽 3 s → 중앙 3 s. 화면 안내(삐 소리)에 맞춰 사람이 목표를 옮긴다.
바닥·배경의 표시 위치, 목표와 카메라 거리, 해상도는 회차마다 같게 둔다. 실제 모터를 쓰므로 낮은 Kp부터 시작한다.

  python3 assignment/assignment3.py run --pan-kp 2.0 --trial 1        # 1회 (Kp는 config 복사본에서만 바꿈)
  python3 assignment/assignment3.py run --pan-kp 1.0 --trial 1
  python3 assignment/assignment3.py run --pan-kp 2.0 --trial 1 --dry-run   # 모터 출력 없이 절차 확인
  python3 assignment/assignment3.py analyze                           # 회차별 지표·Kp별 평균·그래프
결과: results/assignment3/runs/<run_id>/ (설정·marks.csv·노드 기록), results/assignment3/kp_compare.csv·summary.md,
      results/plots/assignment3_*.png, results/metrics.csv (test=assignment3)
지표: 수평 RMSE(검출·TRACKING 행), 유효 추적 비율, 구간 응답 시간(|ex| <= 0.1 도달), 명령 부호 바뀜(흔들림)
심화(데드밴드·필터 비교)는 assignment_C.py
"""
import argparse
import glob
import math
import os
import re
import statistics
import time

import common as C

SCHEDULE = [('LEFT', '왼쪽 표시', 3.0), ('CENTER', '중앙 표시', 3.0), ('RIGHT', '오른쪽 표시', 3.0), ('CENTER', '중앙 표시', 3.0)]
SETTLE_EX = 0.1          # 응답 시간 판정: |ex| <= 0.1 (화면 폭의 5 %, 약 3°)


def run_trial(test, run_id, overrides, dry_run=False, extra_note=''):
    """설정 복사본(overrides)으로 전체 노드를 띄우고 SCHEDULE대로 안내하며 기록한다. 기록 폴더를 돌려준다."""
    dest = C.out_dir(test, 'runs', run_id)
    cfg = C.config_variant(os.path.join(dest, 'config'), overrides)
    print(f'[{run_id}] 바꾼 설정: {overrides} {extra_note}')
    proc, node, st = C.start_stack(run_id, dest, dry_run=dry_run, config_dir=cfg)
    marks = []
    try:
        C.ask('목표를 중앙 표시에 두세요. Enter를 누르면 추적을 켜고 2초 기다린 뒤 순서대로 안내합니다')
        C.set_tracking(node, st, True)
        C.spin_for(node, 2.0)
        for label, guide, secs in SCHEDULE:
            print(f'  → 목표를 {guide}로 옮기세요')
            t = time.time()
            end = t + secs
            while time.time() < end:
                C.spin_for(node, 0.1)
                left = end - time.time()
                print(f'\r  \a{label:6s} {left:4.1f} s  상태 {st["status"]}      ' if left > secs - 0.15 else
                      f'\r  {label:6s} {left:4.1f} s  상태 {st["status"]}      ', end='', flush=True)
            print()
            marks.append({'phase': label, 'start_t': t, 'end_t': end})
        C.set_tracking(node, st, False)
        C.spin_for(node, 1.5)                 # 기록이 디스크에 저장되도록(1 s 주기)
    finally:
        node.destroy_node()
        proc.stop()
    C.write_csv(os.path.join(dest, 'marks.csv'), marks)
    C.keep_logs(run_id, dest)
    open(os.path.join(dest, 'note.txt'), 'w').write(
        f'run_id {run_id}\n날짜 {time.strftime("%Y-%m-%d %H:%M:%S")}\n커밋 {C.git_commit()}\n바꾼 설정 {overrides}\n'
        f'dry_run {dry_run}\n{extra_note}\n')
    print(f'  기록: {dest}')
    return dest


def phase_response(rows, marks):
    """구간마다 시작 → |ex| <= SETTLE_EX(검출·TRACKING) 첫 도달까지 시간 [s]. 첫 구간(LEFT)과 이후 이동 구간을 모두 잰다."""
    out = []
    for m in marks:
        t0, t1 = float(m['start_t']), float(m['end_t'])
        seg = [r for r in rows if t0 <= C.fnum(r['ros_time_s']) <= t1]
        hit = [C.fnum(r['ros_time_s']) - t0 for r in seg
               if r['state'] == 'TRACKING' and r['detected'] == '1' and abs(C.fnum(r['ex'])) <= SETTLE_EX]
        out.append(hit[0] if hit else math.nan)
    return out


def analyze(test, group_key, title):
    """runs/*/의 회차를 group_key(note의 바뀐 설정 값)로 묶어 지표·평균·그래프를 만든다."""
    runs = sorted(glob.glob(os.path.join(C.RESULTS, test, 'runs', '*')))
    rows_out, by_group, plots = [], {}, {}
    for d in runs:
        run_id = os.path.basename(d)
        ctl = os.path.join(d, f'{run_id}.csv')
        mk = os.path.join(d, 'marks.csv')
        if not (os.path.exists(ctl) and os.path.exists(mk)):
            print(f'  건너뜀 {run_id}: 기록 없음')
            continue
        rows, marks = C.read_csv(ctl), C.read_csv(mk)
        t0, t1 = float(marks[0]['start_t']), float(marks[-1]['end_t'])
        m = C.tracking_metrics(rows, t0, t1)
        if not m:
            print(f'  건너뜀 {run_id}: 시험 구간에 추적 기록 없음')
            continue
        resp = phase_response(rows, marks)
        group = C.read_param(group_key, os.path.join(d, 'config'))
        dry = 'dry_run True' in open(os.path.join(d, 'note.txt')).read()
        valid_resp = [r for r in resp if r == r]
        r = {'test': test, 'run_id': run_id, 'condition': f'{group_key}={group}' + (' (dry_run)' if dry else ''),
             'date': time.strftime('%Y-%m-%d', time.localtime(t0)), group_key: group,
             'rmse_ex': m.get('rmse_ex'), 'mean_abs_ex': m.get('mean_abs_ex'), 'max_abs_ex': m.get('max_abs_ex'),
             'tracking_ratio': m.get('tracking_ratio'), 'node_detect_ratio': m.get('node_detect_ratio'),
             'rmse_rows': m.get('rmse_rows'), 'excluded_rows': m.get('excluded_rows'),
             'response_mean_s': statistics.mean(valid_resp) if valid_resp else math.nan,
             'response_missing': len(resp) - len(valid_resp), 'cmd_flips_per_s': m.get('cmd_flips_per_s'),
             'duration_s': m.get('duration_s')}
        rows_out.append(r)
        by_group.setdefault(group, []).append(r)
        for key in ('ex', 'pan_deg', 'pan_cmd'):
            xs, ys = C.series_from([x for x in rows if t0 - 1 <= C.fnum(x['ros_time_s']) <= t1 + 1], key, t0,
                                   only_tracking=(key == 'ex'))
            plots.setdefault(key, []).append((f'{group_key[:7]} {group} t{(re.search(r"_t(\d+)", run_id) or [None, "?"])[1]}', xs, ys))
    if not rows_out:
        raise SystemExit('분석할 회차가 없습니다(run 먼저)')
    summary = []
    for g, rs in sorted(by_group.items()):
        def ms(k):
            v = [x[k] for x in rs if isinstance(x[k], (int, float)) and x[k] == x[k]]
            return (statistics.mean(v), statistics.stdev(v) if len(v) > 1 else 0.0) if v else (math.nan, math.nan)
        summary.append({group_key: g, 'trials': len(rs),
                        'rmse_ex_mean': ms('rmse_ex')[0], 'rmse_ex_sd': ms('rmse_ex')[1],
                        'tracking_ratio_mean': ms('tracking_ratio')[0], 'response_mean_s': ms('response_mean_s')[0],
                        'cmd_flips_per_s_mean': ms('cmd_flips_per_s')[0]})
    dest = C.out_dir(test)
    C.write_csv(os.path.join(dest, 'compare_runs.csv'), rows_out)
    C.write_csv(os.path.join(dest, 'compare_summary.csv'), summary)
    C.update_metrics(rows_out)
    pdir = C.out_dir('plots')
    files = []
    for key, ylabel, h in (('ex', 'ex (normalized, TRACKING only)', [(0, '0'), (SETTLE_EX, '+0.1'), (-SETTLE_EX, '-0.1')]),
                           ('pan_deg', 'pan angle measured [deg]', [(0, '0')]),
                           ('pan_cmd', 'pan command [deg/s]', [(0, '0')])):
        f = C.line_plot(os.path.join(pdir, f'{test}_{key}.png'), plots[key], f'{title}: {key} vs time',
                        'time from LEFT phase start [s]  (LEFT 0-3, CENTER 3-6, RIGHT 6-9, CENTER 9-12)', ylabel, h)
        files.append(f)
    md = [f'# {title}', '', '회차별 (발제 문제 3·4 산식, 제어 기록 50 Hz 행 기준)', '',
          C.md_table(rows_out, ['run_id', group_key, 'rmse_ex', 'tracking_ratio', 'response_mean_s', 'response_missing',
                                'cmd_flips_per_s', 'rmse_rows', 'excluded_rows']),
          '', '설정별 평균', '', C.md_table(summary, list(summary[0].keys())), '',
          f'- RMSE = sqrt(mean(ex²)), 검출·TRACKING 행만 (제외 행 수 병기). 응답 시간 = 구간 시작 → |ex| <= {SETTLE_EX} 첫 도달',
          '- 흔들림 = TRACKING 중 팬 명령 부호가 바뀐 횟수 / TRACKING 시간', '- pan_deg는 모터가 회신한 실제 각도(명령 적분 아님). dry_run 회차는 시뮬레이션 각도',
          '', '그래프: ' + ', '.join(os.path.relpath(f, C.LV2) for f in files if f)]
    open(os.path.join(dest, 'summary.md'), 'w').write('\n'.join(md) + '\n')
    print('\n'.join(md))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('run', help='Kp 하나로 1회 시험')
    r.add_argument('--pan-kp', type=float, required=True, help='팬 Kp [1/s] (팀 결정 2.0)')
    r.add_argument('--tilt-kp', type=float, help='틸트 Kp [1/s] (생략하면 config 값 2.5)')
    r.add_argument('--trial', type=int, required=True)
    r.add_argument('--dry-run', action='store_true', help='모터 출력 없이(브리지 시뮬레이션) 절차 확인')
    sub.add_parser('analyze', help='회차별 지표·Kp별 평균·그래프')
    args = ap.parse_args()
    if args.cmd == 'run':
        over = {'pan_kp': args.pan_kp}
        if args.tilt_kp is not None:
            over['tilt_kp'] = args.tilt_kp
        run_trial('assignment3', f'assignment3_kp{args.pan_kp:g}_t{args.trial}{"_dry" if args.dry_run else ""}', over,
                  args.dry_run)
    else:
        analyze('assignment3', 'pan_kp', 'Problem 3 Kp compare')


if __name__ == '__main__':
    main()
