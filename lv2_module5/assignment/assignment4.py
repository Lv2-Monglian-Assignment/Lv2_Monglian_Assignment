#!/usr/bin/env python3
"""문제 4 — 성능 측정과 목표 소실 복구: 필수 시험 4종과 검출률 판정을 실행하고 지표를 계산한다.

  python3 assignment/assignment4.py normal --seconds 35         # 정상 추적 30 s 이상: 처리 FPS·노드 검출 비율·RMSE·유효 추적 비율
  python3 assignment/assignment4.py eval                         # 검출률 평가 프레임 저장 (목표 30장 + 없음 10장)
  python3 assignment/assignment4.py score results/assignment4/eval_<시각>/labels.csv   # 사람 대조 후 검출률·배경 오검출
  python3 assignment/assignment4.py occlusion --trials 5         # 약 2 s 가림 후 시야 안 재등장 5회: 정지·복귀 여부·복귀 시간
  python3 assignment/assignment4.py topic-stop                   # 인지 입력 중단(검출 노드 kill -9): 0.5 s 타임아웃 정지
  python3 assignment/assignment4.py control-stop --node controller   # 제어 노드 kill -9: 브리지가 0.2 s 뒤 V 0 0 → 정지
  python3 assignment/assignment4.py control-stop --node bridge       # 브리지 kill -9: OpenCR 300 ms 타임아웃 정지(토크 유지)
결과: results/assignment4/<시험>_<시각>/ (summary.md, 표, 노드 기록), results/metrics.csv (test=assignment4_*)
모든 시험은 실제 모터를 쓴다(--dry-run 가능한 시험은 표시). 실패 회차도 통계에 넣는다.
심화(복구 강건성·실패 원인 분류)는 assignment_D.py
"""
import argparse
import math
import os
import time

import common as C


def joint_probe(node):
    """/pan_tilt/joint_states의 팬·틸트 속도 [deg/s]를 시각과 함께 모은다(브리지가 회신한 실제 값)."""
    from sensor_msgs.msg import JointState
    hist = []
    node.create_subscription(JointState, '/pan_tilt/joint_states',
                             lambda m: hist.append((time.time(), *(math.degrees(v) for v in m.velocity[:2])))
                             if len(m.velocity) >= 2 else None, 50)
    return hist


def finish(test, run_id, dest, md, metrics_rows=()):
    open(os.path.join(dest, 'summary.md'), 'w').write('\n'.join(md) + '\n')
    if metrics_rows:
        C.update_metrics(list(metrics_rows))
    print('\n'.join(md))
    print(f'\n결과: {dest}')


# ---------------- 정상 추적 30 s ----------------
def normal(args):
    run_id = f'assignment4_normal_{C.tag()}'
    dest = C.out_dir('assignment4', run_id)
    proc, node, st = C.start_stack(run_id, dest, dry_run=args.dry_run)
    try:
        C.ask(f'목표를 시야 안에 두세요. 추적을 켜고 {args.seconds:g} s 기록합니다(같은 조건 유지: 거리·조명·이동 방식)')
        C.set_tracking(node, st, True)
        C.spin_for(node, 1.0)
        t0 = time.time()
        while time.time() - t0 < args.seconds:
            C.spin_for(node, 0.2)
            print(f'\r  기록 {time.time() - t0:5.1f}/{args.seconds:g} s  상태 {st["status"]}     ', end='', flush=True)
        t1 = time.time()
        print()
        C.set_tracking(node, st, False)
        C.spin_for(node, 1.5)
    finally:
        node.destroy_node()
        proc.stop()
    C.keep_logs(run_id, dest)
    logs = C.node_logs(run_id)
    det, ctl = C.read_csv(logs['detect']), C.read_csv(logs['control'])
    fps, cam, n = C.processing_fps(det, t0, t1)
    m = C.tracking_metrics(ctl, t0, t1)
    row = {'test': 'assignment4_normal', 'run_id': run_id, 'condition': f'{args.seconds:g} s' + (' dry_run' if args.dry_run else ''),
           'date': time.strftime('%Y-%m-%d'), 'processing_fps': fps, 'camera_fps': cam, 'frames': n, **m}
    md = [f'# 문제 4 정상 추적 ({run_id})', '', C.md_table([row], ['processing_fps', 'camera_fps', 'frames', 'duration_s',
          'node_detect_ratio', 'tracking_ratio', 'rmse_ex', 'rmse_rows', 'excluded_rows', 'max_abs_ex', 'cmd_flips_per_s']), '',
          '- 처리 FPS = 처리 완료 프레임 수 / 실제 경과 초(인지 노드 발행 시각), 카메라 FPS = stamp 기준 (서로 다름)',
          '- node_detect_ratio는 노드의 detected 비율이다. 사람 대조 검출률은 eval·score로 따로 낸다',
          '- RMSE = sqrt(mean(ex²)), 검출·TRACKING 행만 사용(제외 행 수 병기). 유효 추적 비율 = TRACKING 행 / 전체 행']
    xs, ys = C.series_from([r for r in ctl if t0 <= C.fnum(r['ros_time_s']) <= t1], 'ex', t0, only_tracking=True)
    C.line_plot(os.path.join(C.out_dir('plots'), f'{run_id}_ex.png'), [('ex', xs, ys)], 'Normal tracking ex', 'time [s]', 'ex',
                [(0, '0')])
    finish('normal', run_id, dest, md, [row])


# ---------------- 검출률 평가 프레임 ----------------
def evaluate(args):
    dest = C.out_dir('assignment4', f'eval_{C.tag()}')
    proc = None if args.no_launch else C.launch('perception.launch.py', os.path.join(dest, 'launch.log'),
                                                run_id=f'assignment4_eval_{C.tag()}')
    try:
        C.capture_eval(dest, [('target', args.target), ('empty', args.empty)], args.interval)
    finally:
        if proc:
            proc.stop()


def score(args):
    res = C.score_eval(args.labels)
    dest = os.path.dirname(os.path.abspath(args.labels))
    rows = [{'test': 'assignment4_detection', 'run_id': os.path.basename(dest), 'condition': r['config'],
             'date': time.strftime('%Y-%m-%d'), **r} for r in res]
    C.write_csv(os.path.join(dest, 'score.csv'), res)
    md = [f'# 문제 4 검출률·배경 오검출 ({os.path.basename(dest)})', '',
          C.md_table(res, ['config', 'visible_frames', 'correct', 'detection_rate_pct', 'missed', 'wrong_object',
                           'empty_frames', 'false_detections', 'proc_fps', 'unlabeled']), '',
          '- 검출률 = 올바른 검출 / 목표가 실제 보이는 평가 프레임 x100 (사람이 *_detect.png 대조, labels.csv의 correct)',
          '- 배경 오검출 = 목표 없는 평가 프레임의 검출 수 (검출률과 따로 기록)', '- 평가 프레임 목록과 판정: labels.csv']
    finish('detection', os.path.basename(dest), dest, md, rows)


# ---------------- 가림 후 재등장 ----------------
def occlusion(args, test='assignment4_occlusion', overrides=None, dest_name=None):
    run_id = dest_name or f'assignment4_occlusion_{C.tag()}'
    dest = C.out_dir(test.split('_occlusion')[0] if test.endswith('occlusion') else test, run_id)
    cfg = C.config_variant(os.path.join(dest, 'config'), overrides) if overrides else C.CONFIG
    proc, node, st = C.start_stack(run_id, dest, dry_run=False, config_dir=cfg)
    marks = []
    try:
        C.ask(f'목표를 시야 중앙 근처에 두세요. 추적을 켠 뒤 {args.trials}회: "가리세요" → {args.hide:g} s → "치우세요"(같은 자리에 다시 보이게)')
        C.set_tracking(node, st, True)
        C.spin_for(node, 2.0)
        for i in range(1, args.trials + 1):
            print(f' [{i}/{args.trials}]')
            t = time.time()
            while time.time() - t < 3.0:                      # 추적 확인
                C.spin_for(node, 0.1)
            print('  \a가리세요!')
            hide_t = time.time()
            while time.time() - hide_t < args.hide:
                C.spin_for(node, 0.1)
            print('  \a치우세요!')
            show_t = time.time()
            while time.time() - show_t < 4.0:
                C.spin_for(node, 0.1)
            marks.append((hide_t, show_t))
            print(f'  상태 {st["status"]}')
        C.set_tracking(node, st, False)
        C.spin_for(node, 1.5)
    finally:
        node.destroy_node()
        proc.stop()
    C.keep_logs(run_id, dest)
    C.write_csv(os.path.join(dest, 'marks.csv'), [{'trial': i + 1, 'hide_prompt_t': a, 'show_prompt_t': b}
                                                  for i, (a, b) in enumerate(marks)])
    logs = C.node_logs(run_id)
    ctl, det = C.read_csv(logs['control']), C.read_csv(logs['detect'])
    trials = C.recovery_trials(ctl, det, marks)
    ok = [t for t in trials if t['result'] == '성공']
    times = [t['recovery_s'] for t in ok]
    row = {'test': test, 'run_id': run_id, 'condition': str(overrides or 'config 기본'), 'date': time.strftime('%Y-%m-%d'),
           'trials': len(trials), 'success': len(ok), 'success_rate_pct': 100 * len(ok) / len(trials) if trials else math.nan,
           'recovery_mean_s': sum(times) / len(times) if times else math.nan,
           'recovery_max_s': max(times) if times else math.nan,
           'max_cmd_while_lost': max((t.get('max_cmd_while_lost', math.nan) for t in trials
                                      if t.get('max_cmd_while_lost', math.nan) == t.get('max_cmd_while_lost', math.nan)),
                                     default=math.nan)}
    C.write_csv(os.path.join(dest, 'trials.csv'), trials)
    md = [f'# 가림 후 재등장 {len(trials)}회 ({run_id})', '',
          C.md_table(trials, ['trial', 'result', 'occluded_s', 'recovery_s', 'max_cmd_while_lost']), '',
          f'- 복구 성공률 = 재등장 후 3 s 이내 TRACKING 복귀 / {len(trials)} x100 = {C.fmt(row["success_rate_pct"])} %',
          f'- 복구 시간 = TRACKING 복귀 시각 − 재등장 시각, 성공 회차 평균 {C.fmt(row["recovery_mean_s"])} s (실패는 0 s가 아니라 실패로 표시)',
          '- 재등장 시각: 후보가 없던 영상 다음 첫 후보 영상의 stamp (인지 기록 n_candidates). 사람이 본 시각과 다를 수 있다',
          '- 정지 확인: 가림 중 미검출 행의 최대 |명령| (0이어야 함, 이전 속도 유지 금지)',
          f'- 복귀 조건: 신선한 검출 연속 {C.read_param("recover_frames", cfg)}프레임, 번호 유지 재선택 {C.read_param("relock_after_s", cfg)} s']
    finish(test, run_id, dest, md, [row])
    return trials, row, dest


# ---------------- 인지 입력 중단 ----------------
def topic_stop(args):
    run_id = f'assignment4_topicstop_{C.tag()}'
    dest = C.out_dir('assignment4', run_id)
    proc, node, st = C.start_stack(run_id, dest, dry_run=args.dry_run)
    try:
        C.ask('목표를 시야 안에 두세요. 추적을 켜고 3 s 뒤 검출 노드를 kill -9로 끊습니다')
        C.set_tracking(node, st, True)
        C.wait_until(node, lambda: (st['status'] or '').startswith('TRACKING'), 10)
        C.spin_for(node, 3.0)
        before = st['status']
        t_kill, killed = C.kill9(C.NODE_PATTERNS['detector'])
        print(f'  검출 노드 kill -9 (pid {killed}), 직전 상태 {before}')
        C.spin_for(node, 3.0)
        C.set_tracking(node, st, False)
        C.spin_for(node, 1.5)
    finally:
        node.destroy_node()
        proc.stop()
    C.keep_logs(run_id, dest)
    ctl = C.read_csv(C.node_logs(run_id)['control'])
    after = [r for r in ctl if C.fnum(r['ros_time_s']) >= t_kill and r['state'] != 'IDLE']
    lost = [r for r in after if r['reason'] == 'input_timeout']
    t_lost = C.fnum(lost[0]['ros_time_s']) - t_kill if lost else math.nan
    last_in = max((C.fnum(r['stamp_s']) for r in ctl if C.fnum(r['ros_time_s']) <= t_kill and r['stamp_s']), default=math.nan)
    moving_after = [r for r in after if C.fnum(r['ros_time_s']) >= t_kill + (t_lost if t_lost == t_lost else 0)
                    and (abs(C.fnum(r['pan_cmd'])) > 0 or abs(C.fnum(r['tilt_cmd'])) > 0)]
    row = {'test': 'assignment4_topic_stop', 'run_id': run_id, 'condition': 'detector kill -9' + (' dry_run' if args.dry_run else ''),
           'date': time.strftime('%Y-%m-%d'), 'status_before': before, 'kill_to_timeout_s': t_lost,
           'input_timeout_cfg_s': C.read_param('input_timeout_s'), 'nonzero_cmd_rows_after_timeout': len(moving_after),
           'verdict': 'PASS' if lost and not moving_after else 'FAIL'}
    md = [f'# 인지 입력 중단 ({run_id})', '', C.md_table([row], ['status_before', 'kill_to_timeout_s', 'input_timeout_cfg_s',
          'nonzero_cmd_rows_after_timeout', 'verdict']), '',
          f'- kill 시각 {t_kill:.3f} (시스템 시계), 마지막 입력 stamp {last_in:.3f}. 타임아웃은 마지막 신선한 입력 수신 기준이라 kill 시각과 최대 1프레임(33 ms) 차이',
          '- 판정: LOST:input_timeout 진입, 그 뒤 명령 0만 발행']
    finish('topic_stop', run_id, dest, md, [row])


# ---------------- 제어 통신 중단 ----------------
def control_stop(args):
    run_id = f'assignment4_ctlstop_{args.node}_{C.tag()}'
    dest = C.out_dir('assignment4', run_id)
    proc, node, st = C.start_stack(run_id, dest, dry_run=False)
    joints = joint_probe(node)
    serial_lines = []
    try:
        C.ask('목표를 천천히 좌우로 계속 움직이세요(카메라가 도는 중에 끊어야 정지를 확인할 수 있음). Enter 후 추적 시작, 4 s 뒤 kill -9')
        C.set_tracking(node, st, True)
        C.spin_for(node, 4.0)
        t_kill, killed = C.kill9(C.NODE_PATTERNS[args.node])
        print(f'  {args.node} kill -9 (pid {killed})')
        if args.node == 'bridge':
            # 브리지가 죽으면 포트가 비므로 바로 열어 OpenCR 상태 줄을 읽는다(보드 타임아웃 → 속도 0, 토크 유지 확인)
            import serial
            time.sleep(0.05)
            ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.05)
            t_open = time.time()
            while time.time() - t_open < 1.5:
                line = ser.readline().decode(errors='replace').strip()
                if line:
                    serial_lines.append((time.time(), line))
            ser.close()
        else:
            C.spin_for(node, 2.0)
    finally:
        node.destroy_node()
        proc.stop()
    C.keep_logs(run_id, dest)
    row = {'test': 'assignment4_control_stop', 'run_id': run_id, 'condition': f'{args.node} kill -9',
           'date': time.strftime('%Y-%m-%d')}
    if args.node == 'controller':
        before = [abs(p) for t, p, _ in joints if t_kill - 0.5 <= t < t_kill]
        after = [(t - t_kill, abs(p), abs(q)) for t, p, q in joints if t >= t_kill]
        moving = [dt for dt, p, q in after if p > 1.0 or q > 1.0]      # 1 deg/s 넘는 회전
        stop_s = moving[-1] if moving else 0.0
        row.update({'pan_speed_before_dps': max(before, default=math.nan), 'last_motion_after_kill_s': stop_s,
                    'bridge_cmd_timeout_cfg_s': C.read_param('cmd_timeout_s'),
                    'verdict': 'PASS' if after and stop_s <= 1.0 and max(before, default=0) > 1.0 else
                    ('CHECK(정지 전 회전 없음)' if after and max(before, default=0) <= 1.0 else 'FAIL')})
        note = ['- 측정: 브리지가 회신한 /pan_tilt/joint_states 속도(실제 모터). 1 deg/s 넘는 마지막 시각 = 정지까지 걸린 시간',
                '- 기대: 브리지 cmd_timeout 0.2 s 뒤 V 0 0 + 감속 시간']
    else:
        st_lines = [(t - t_kill, l) for t, l in serial_lines if l.startswith('S ')]
        evt = [(t - t_kill, l) for t, l in serial_lines if l.startswith('E ')]
        last = st_lines[-1][1].split() if st_lines else []
        row.update({'serial_read_from_s': serial_lines[0][0] - t_kill if serial_lines else math.nan,
                    'timeout_event': '; '.join(f'{dt:.3f}s {l}' for dt, l in evt) or '(포트를 열기 전에 지나감)',
                    'final_state': last[-1] if last else '', 'final_speed_dps': f'{last[4]} {last[5]}' if last else '',
                    'verdict': 'PASS' if last and last[-1] in ('HOLD', 'OFF') and abs(float(last[4])) < 1 and abs(float(last[5])) < 1
                    else 'FAIL'})
        note = ['- 측정: 브리지 kill 직후 시리얼을 직접 열어 OpenCR 상태 줄(S ...)을 1.5 s 읽음. 기대: 300 ms 안에 속도 0, 상태 HOLD(토크 유지)']
        C.write_csv(os.path.join(dest, 'serial_after_kill.csv'), [{'t_after_kill_s': t - t_kill, 'line': l} for t, l in serial_lines])
    C.write_csv(os.path.join(dest, 'joint_speed.csv'), [{'t_after_kill_s': t - t_kill, 'pan_dps': p, 'tilt_dps': q} for t, p, q in joints])
    md = [f'# 제어 통신 중단: {args.node} kill -9 ({run_id})', '', C.md_table([row], [k for k in row if k not in ('test', 'run_id', 'date')]),
          ''] + note
    finish('control_stop', run_id, dest, md, [row])


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    n = sub.add_parser('normal')
    n.add_argument('--seconds', type=float, default=35.0)
    n.add_argument('--dry-run', action='store_true')
    e = sub.add_parser('eval')
    e.add_argument('--target', type=int, default=30)
    e.add_argument('--empty', type=int, default=10)
    e.add_argument('--interval', type=float, default=0.5)
    e.add_argument('--no-launch', action='store_true')
    s = sub.add_parser('score')
    s.add_argument('labels')
    o = sub.add_parser('occlusion')
    o.add_argument('--trials', type=int, default=5)
    o.add_argument('--hide', type=float, default=2.0)
    t = sub.add_parser('topic-stop')
    t.add_argument('--dry-run', action='store_true')
    c = sub.add_parser('control-stop')
    c.add_argument('--node', choices=('controller', 'bridge'), required=True)
    args = ap.parse_args()
    C.check_ros() if args.cmd != 'score' else None
    {'normal': normal, 'eval': evaluate, 'score': score, 'occlusion': occlusion, 'topic-stop': topic_stop,
     'control-stop': control_stop}[args.cmd](args)


if __name__ == '__main__':
    main()
