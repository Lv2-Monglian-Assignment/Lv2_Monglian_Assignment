#!/usr/bin/env python3
"""도전 B (문제 2 심화) — 인터페이스 완성도와 SEARCHING: 모의 입력 재확인 + 시야 밖 탐색의 성공·취소·미발견 정지 시험.

모터 출력 없이 실행한다. SEARCHING 시험은 제어 + 브리지(dry_run, 각도 시뮬레이션) + 가상 물체(scripts/mock_target_pub.py mode:=virtual).
가상 물체는 기준 좌표에 있고, 카메라 자세(모의 각도)로 화면 위치를 계산한다. 가림 구간에는 미검출을 발행한다.

  python3 assignment/assignment_B.py interface     # 문제 2 입력 + 같은 stamp 재전송 (assignment2와 같은 절차)
  python3 assignment/assignment_B.py search        # SEARCHING 3종: 성공(found) · 취소(cancel) · 미발견(notfound)
결과: results/assignment_B/ (mock_*/ 판정표, search_<시각>/ 상태 전이 기록·표, state_table.md)
탐색 설정(config/control.yaml): search_delay_s, search_timeout_s, search_kp, search_speed_limit_deg_s, search_pan_max_deg, search_tilt_max_deg
시야 내 재등장 복구(문제 4)와 시야 밖 탐색 성공은 별도 결과로 기록한다.
"""
import argparse
import os
import time

import common as C

STATE_TABLE = """# 상태 전이표 (tracker_controller, search_enabled=true)

| 상태 | 들어가는 조건 | 출력 명령 | 나가는 조건 |
|---|---|---|---|
| IDLE | 시작, /tracking_enable false | 0 | /tracking_enable true → LOST(confirming) |
| TRACKING | 신선한 검출 연속 recover_frames(3)프레임 | clamp(direction × Kp × 각도 오차) | 미검출 → LOST:no_detection, 입력 0.5 s 없음 → LOST:input_timeout |
| LOST | 미검출·입력 타임아웃·탐색 실패 | 0 (첫 프레임부터) | 미검출이 search_delay_s 이상 + 기억한 목표 있음 → SEARCHING, 검출 3프레임 → TRACKING |
| SEARCHING | (심화) 기억한 목표의 예측 방향으로 | 각도 P: clamp(search_kp × (예측 각 − 현재 각), ±search_speed_limit), 각도는 ±search_*_max_deg 안 | 검출 3프레임 → TRACKING(reacquired_search), search_timeout_s 초과 → LOST:search_failed(정지, 다음 TRACKING 전까지 재탐색 없음), /tracking_enable false → IDLE |
"""

SCENARIOS = [  # 이름, mock 파라미터, 시험 시간 [s], 설명
    ('found', {'obj_vy_m_s': -0.12, 'hide_from_s': 4.0, 'hide_to_s': 5.6}, 11.0,
     '물체가 오른쪽으로 움직이다 1.6 s 가려짐(그동안 시야 밖으로 이동) → 예측 방향 탐색 → 재검출'),
    ('cancel', {'obj_vy_m_s': -0.12, 'hide_from_s': 4.0, 'hide_to_s': 60.0}, 9.0,
     '가려진 채 탐색 중에 /tracking_enable false → IDLE, 명령 0'),
    ('notfound', {'obj_vy_m_s': -0.12, 'hide_from_s': 4.0, 'hide_to_s': 60.0}, 12.0,
     '계속 가려짐 → search_timeout_s 뒤 LOST:search_failed, 명령 0'),
]


def search(args):
    C.check_ros()
    tagv = C.tag()
    dest = C.out_dir('assignment_B', f'search_{tagv}')
    cfg = C.config_variant(os.path.join(dest, 'config'), {'search_enabled': True})
    results = []
    for name, mock_p, secs, desc in SCENARIOS:
        print(f'=== {name}: {desc}')
        run_id = f'assignment_B_{name}_{tagv}'
        ctl = C.launch('control.launch.py', os.path.join(dest, f'{name}_control.log'), dry_run=True, auto_enable=True,
                       run_id=run_id, config_dir=cfg)
        params = {'mode': 'virtual', 'obj_x_m': 0.8, 'obj_y_m': 0.0, 'obj_z_m': 0.0, 'hfov_deg': 55.7, 'vfov_deg': 43.2,
                  'pan_direction': int(C.read_param('pan_direction')), 'tilt_direction': int(C.read_param('tilt_direction')),
                  **mock_p}
        node, st = C.make_probe()
        time.sleep(3.0)                                     # 제어·브리지 준비
        mock = C.run_mock(os.path.join(dest, f'{name}_mock.log'), params=params)
        t0 = time.time()
        cancelled_t = None
        cmds = []
        while time.time() - t0 < secs:
            C.spin_for(node, 0.02)
            if st['cmd'] is not None:
                cmds.append((time.time() - t0, st['status'], *st['cmd']))
            if name == 'cancel' and cancelled_t is None and (st['status'] or '').startswith('SEARCHING'):
                C.spin_for(node, 0.5)
                C.set_tracking(node, st, False)
                cancelled_t = time.time() - t0
                print(f'  탐색 중 취소: {cancelled_t:.2f} s')
        hist = [(t - t0, s) for t, s in st['history'] if t >= t0]
        node.destroy_node()
        mock.stop()
        ctl.stop()
        trans = []
        for t, s in hist:
            if not trans or trans[-1][1] != s:
                trans.append((t, s))
        seq = ' → '.join(f'{s}@{t:.2f}' for t, s in trans)
        final = trans[-1][1] if trans else ''
        tail = [c for c in cmds if c[0] >= secs - 1.0]
        final_cmd = max((max(abs(c[2]), abs(c[3])) for c in tail), default=float('nan'))
        searched = [s for _, s in trans if s.startswith('SEARCHING')]
        seg = C.state_segments(C.read_csv(C.node_logs(run_id)['control']), by_state=True) if os.path.exists(C.node_logs(run_id)['control']) else []
        search_len = max((b - a for a, b, s, _ in seg if s == 'SEARCHING'), default=0.0)
        expect = {'found': lambda: searched and 'TRACKING:reacquired_search' in [s for _, s in trans],
                  'cancel': lambda: searched and final.startswith('IDLE') and final_cmd == 0,
                  'notfound': lambda: searched and 'LOST:search_failed' in [s for _, s in trans] and final_cmd == 0
                  and search_len <= float(C.read_param('search_timeout_s', cfg)) + 0.2}[name]
        res = {'scenario': name, 'expected': desc, 'transitions': seq, 'final_state': final,
               'final_max_cmd': final_cmd, 'longest_search_s': search_len,
               'search_timeout_cfg_s': C.read_param('search_timeout_s', cfg), 'verdict': 'PASS' if expect() else 'FAIL'}
        results.append(res)
        C.write_csv(os.path.join(dest, f'{name}_commands.csv'),
                    [{'t_s': c[0], 'status': c[1], 'pan_cmd': c[2], 'tilt_cmd': c[3]} for c in cmds])
        C.keep_logs(run_id, dest)
        print(f'  {seq}\n  → {res["verdict"]}')
    C.write_csv(os.path.join(dest, 'search.csv'), results)
    open(os.path.join(C.out_dir('assignment_B'), 'state_table.md'), 'w').write(STATE_TABLE)
    keys = ['search_delay_s', 'search_timeout_s', 'search_kp', 'search_speed_limit_deg_s', 'search_pan_max_deg', 'search_tilt_max_deg']
    md = ['# 도전 B SEARCHING 시험 (모터 출력 없음: 브리지 dry_run + 가상 물체)', '',
          '- 탐색 설정: ' + ', '.join(f'{k} {C.read_param(k, cfg)}' for k in keys), '',
          C.md_table(results, ['scenario', 'verdict', 'final_state', 'final_max_cmd', 'longest_search_s', 'transitions']), '',
          '- 상태 전이표: state_table.md. 시야 밖 탐색 성공(found)은 문제 4의 시야 내 재등장 복구와 별도 통계다',
          '- 미발견·취소 모두 명령 0으로 끝나야 한다(무제한 탐색 없음)']
    open(os.path.join(dest, 'summary.md'), 'w').write('\n'.join(md) + '\n')
    C.update_metrics([{'test': 'assignment_B_search', 'run_id': f'{r["scenario"]}_{tagv}', 'condition': 'dry_run virtual',
                       'date': time.strftime('%Y-%m-%d'), 'verdict': r['verdict'], 'longest_search_s': r['longest_search_s']}
                      for r in results])
    print('\n'.join(md))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    sub.add_parser('interface')
    sub.add_parser('search')
    args = ap.parse_args()
    if args.cmd == 'interface':
        from assignment2 import run_all
        run_all('assignment_B')
        open(os.path.join(C.out_dir('assignment_B'), 'state_table.md'), 'w').write(STATE_TABLE)
    else:
        search(args)


if __name__ == '__main__':
    main()
