#!/usr/bin/env python3
"""도전 D (문제 4 심화) — 목표 소실 복구의 강건성: 가림 후 재등장을 반복하고 성공·실패를 모두 기록해 실패 원인을 분류한다.

  python3 assignment/assignment_D.py run --trials 10                                  # 기본 설정
  python3 assignment/assignment_D.py run --trials 10 --param relock_after_s --value 1.5   # 조건 하나만 변경
  python3 assignment/assignment_D.py analyze                                          # 설정별 성공률·복구 시간·원인 비교
바꿀 수 있는 조건(한 번에 하나): recover_frames(복귀 연속 프레임, 기본 3), relock_after_s(번호 유지 재선택, 기본 3.0),
                                  input_timeout_s(입력 타임아웃, 기본 0.5)
실패 원인 분류
  검출 실패          재등장 뒤에도 후보(n_candidates)가 나오지 않음
  입력 타임아웃       재등장 뒤 3 s 안에 LOST:input_timeout (인지가 멈춤)
  제어 통신 중단      재등장 구간에 모터 각도 회신(pan_deg)이 끊김
  복귀 조건 미충족    후보는 있는데 /target이 미검출(번호 유지 대기) 또는 연속 프레임 확인 중
또 확인: 가림 중 명령 0(이전 속도 유지 없음), 복귀 후 같은 번호(잘못된 목표 추적 없음), 탐색 시간 상한
결과: results/assignment_D/<run_id>/ (trials.csv 원인 포함), results/assignment_D/compare.csv·summary.md
"""
import argparse
import glob
import math
import os
import time

import common as C
from assignment4 import occlusion

PARAMS = {'recover_frames': 3, 'relock_after_s': 3.0, 'input_timeout_s': 0.5}


def classify(trial, ctl, det, limit=3.0):
    if trial['result'] in ('성공', '가림 미확인'):
        return '' if trial['result'] == '성공' else '시험 무효(가림 미확인)'
    if trial['result'] == '재등장 미검출':
        return '검출 실패'
    t_re = trial['reappear_t']
    win_c = [r for r in ctl if t_re <= C.fnum(r['ros_time_s']) <= t_re + limit]
    win_d = [r for r in det if t_re <= C.fnum(r['stamp_s']) <= t_re + limit]
    if any(r['reason'] == 'input_timeout' for r in win_c):
        return '입력 타임아웃'
    if win_c and sum(r['pan_deg'] == '' for r in win_c) > len(win_c) / 2:
        return '제어 통신 중단'
    if any(r['n_candidates'] != '0' and r['detected'] == '0' for r in win_d):
        return '복귀 조건 미충족(번호 유지 대기)'
    if any(r['reason'].startswith('confirming') for r in win_c):
        return '복귀 조건 미충족(연속 프레임)'
    return '기타(로그 확인)'


def target_ids(det, trial):
    """가림 전 마지막 번호와 재등장 뒤 첫 번호 (번호 유지 사용 시). 다르면 다른 물체로 바뀌었을 수 있다."""
    before = [r['target_id'] for r in det if C.fnum(r['stamp_s']) < trial['hide_prompt_t'] and r['target_id']]
    t_re = trial.get('reappear_t', math.nan)
    after = [r['target_id'] for r in det if t_re == t_re and C.fnum(r['stamp_s']) >= t_re and r['target_id']]
    return (before[-1] if before else ''), (after[0] if after else '')


def run(args):
    over = {args.param: args.value} if args.param else None
    name = f'assignment_D_{args.param}{args.value:g}_{C.tag()}' if args.param else f'assignment_D_base_{C.tag()}'
    trials, row, dest = occlusion(args, test='assignment_D', overrides=over, dest_name=name)
    logs = C.node_logs(name)
    ctl, det = C.read_csv(logs['control']), C.read_csv(logs['detect'])
    for t in trials:
        t['cause'] = classify(t, ctl, det)
        t['id_before'], t['id_after'] = target_ids(det, t)
    segs = C.state_segments(ctl, by_state=True)
    search = [s[1] - s[0] for s in segs if s[2] == 'SEARCHING']
    C.write_csv(os.path.join(dest, 'trials.csv'), trials)
    lines = [f'# 도전 D 복구 강건성 ({name})', '', f'- 조건: {over or "기본 설정"}', '',
             C.md_table(trials, ['trial', 'result', 'cause', 'recovery_s', 'occluded_s', 'max_cmd_while_lost', 'id_before', 'id_after']), '',
             f'- 성공 {row["success"]}/{row["trials"]} ({C.fmt(row["success_rate_pct"])} %), 성공 평균 {C.fmt(row["recovery_mean_s"])} s',
             f'- 가림 중 최대 |명령| {C.fmt(row["max_cmd_while_lost"])} deg/s (0이면 이전 속도 유지 없음)',
             f'- SEARCHING 구간: {len(search)}회, 최장 {max(search) if search else 0:.2f} s (상한 {C.read_param("search_timeout_s")} s, 탐색 켜짐 {C.read_param("search_enabled")})']
    open(os.path.join(dest, 'summary.md'), 'w').write('\n'.join(lines) + '\n')
    print('\n'.join(lines))


def analyze(_args):
    rows = []
    for d in sorted(glob.glob(os.path.join(C.RESULTS, 'assignment_D', 'assignment_D_*'))):
        f = os.path.join(d, 'trials.csv')
        if not os.path.exists(f):
            continue
        tr = [t for t in C.read_csv(f) if t['result'] != '가림 미확인']
        ok = [C.fnum(t['recovery_s']) for t in tr if t['result'] == '성공']
        causes = {}
        for t in tr:
            if t.get('cause'):
                causes[t['cause']] = causes.get(t['cause'], 0) + 1
        rows.append({'run_id': os.path.basename(d), 'valid_trials': len(tr), 'success': len(ok),
                     'success_rate_pct': 100 * len(ok) / len(tr) if tr else math.nan,
                     'recovery_mean_s': sum(ok) / len(ok) if ok else math.nan, 'recovery_max_s': max(ok) if ok else math.nan,
                     'failure_causes': '; '.join(f'{k} {v}' for k, v in causes.items())})
    if not rows:
        raise SystemExit('run 결과가 없습니다')
    dest = C.out_dir('assignment_D')
    C.write_csv(os.path.join(dest, 'compare.csv'), rows)
    md = ['# 도전 D 설정별 비교', '', C.md_table(rows, list(rows[0].keys())), '',
          '- 조건은 한 번에 하나만 바꾼다(run_id에 표시). 가림 미확인 회차는 분모에서 빼고 따로 적는다']
    open(os.path.join(dest, 'summary.md'), 'w').write('\n'.join(md) + '\n')
    print('\n'.join(md))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('run')
    r.add_argument('--trials', type=int, default=10)
    r.add_argument('--hide', type=float, default=2.0)
    r.add_argument('--param', choices=PARAMS)
    r.add_argument('--value', type=float)
    sub.add_parser('analyze')
    args = ap.parse_args()
    if args.cmd == 'run':
        if bool(args.param) != (args.value is not None):
            raise SystemExit('--param과 --value는 함께 지정')
        if args.param == 'recover_frames':
            args.value = int(args.value)
        C.check_ros()
        run(args)
    else:
        analyze(args)


if __name__ == '__main__':
    main()
