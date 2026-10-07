#!/usr/bin/env python3
"""문제 2 — 인지·제어 노드 연결: 모터 출력을 끈 상태에서 /target 입력별 명령·상태를 기록하고 판정한다.

제어 노드(tracker_controller)만 실행한다. 브리지(opencr_bridge)를 띄우지 않으므로 OpenCR·모터에 명령이 가지 않는다.
이 프로그램이 /target(PointStamped, best-effort)을 직접 발행하고 /pan_tilt/command·/tracking_status를 기록한다.

  python3 assignment/assignment2.py
결과: results/assignment2/<시각>/ (cases.csv 판정표, <case>.csv 시계열, structure.md 구조도·인터페이스 표)

입력 (발제 문제 2 표 + 확인용 2개)
  center   x=0,    z>0  → 회전 없음          right  x=+0.4, z>0 → 오른쪽 오차를 줄이는 명령(팬 음수)
  left     x=-0.4, z>0  → 반대 방향(팬 양수)   nodetect z=0      → 이전 목표를 쫓지 않고 정지
  silence  2 s 발행 후 중단 → 0.5 s 뒤 LOST:input_timeout, 정지
  stale    같은 stamp를 계속 재전송 → 신선한 입력이 아니므로 타임아웃 정지 (카메라가 멈췄는데 새 시각을 붙이지 않는 규칙 확인)
  down     y=+0.4, z>0  → 아래 오차를 줄이는 틸트 명령(틸트 양수)
심화(인터페이스 완성도·SEARCHING)는 assignment_B.py
"""
import argparse
import math
import os
import time

import common as C

CASES = [  # 이름, ex, ey, 면적, 발행 시간 [s], 같은 stamp 재전송, 설명, 판정 함수(명령 팬·틸트, 상태) -> bool
    ('center', 0.0, 0.0, 0.05, 3.0, False, 'x=0, z>0: 회전 없음',
     lambda p, t, s: s.startswith('TRACKING') and p == 0 and t == 0),
    ('right', 0.4, 0.0, 0.05, 3.0, False, 'x=+0.4: 팬 음수(오른쪽으로 회전)',
     lambda p, t, s: s.startswith('TRACKING') and p < 0 and t == 0),
    ('left', -0.4, 0.0, 0.05, 3.0, False, 'x=-0.4: 팬 양수',
     lambda p, t, s: s.startswith('TRACKING') and p > 0 and t == 0),
    ('nodetect', 0.4, 0.0, 0.0, 3.0, False, 'z=0: LOST:no_detection, 명령 0',
     lambda p, t, s: s == 'LOST:no_detection' and p == 0 and t == 0),
    ('silence', 0.4, 0.0, 0.05, 2.0, False, '발행 중단: 0.5 s 뒤 LOST:input_timeout, 명령 0',
     lambda p, t, s: s == 'LOST:input_timeout' and p == 0 and t == 0),
    ('stale', 0.4, 0.0, 0.05, 3.0, True, '같은 stamp 재전송: 신선한 입력 아님 → input_timeout, 명령 0',
     lambda p, t, s: s.startswith('LOST') and p == 0 and t == 0),
    ('down', 0.0, 0.4, 0.05, 3.0, False, 'y=+0.4: 틸트 양수(아래로 회전)',
     lambda p, t, s: s.startswith('TRACKING') and t > 0 and p == 0),
]

STRUCTURE = """# 노드·연결 구조 (문제 2)

```mermaid
flowchart LR
  cam[realsense2_camera\\nColor·정렬 Depth 640x480@30] -->|/camera/camera/color/image_raw\\n/aligned_depth_to_color/image_raw\\n/color/camera_info| det[target_detector]
  det -->|/target PointStamped\\nx=ex y=ey z=면적비, 0=미검출| ctl[tracker_controller]
  det -->|/target_depth, /target/position_cam| ctl
  ctl -->|/pan_tilt/command Vector3Stamped deg/s 50 Hz| br[opencr_bridge]
  br -->|USB 시리얼 V pan tilt 50 Hz| fw[OpenCR opencr_tracker]
  fw -->|S ms pan tilt dps dps state 50 Hz| br
  br -->|/pan_tilt/joint_states rad| ctl
  br -->|/pan_tilt/joint_states| det
  ctl -->|/tracking_status 상태:사유| log[(기록·bag)]
  user[/tracking_enable Bool/] --> ctl
```

| 노드 | 책임 | 실패 시 동작 |
|---|---|---|
| target_detector | HSV·Contour·크기 검증·선택, /target 발행 (영상마다, 원본 stamp 유지) | 카메라가 멈추면 발행하지 않음, 미검출이면 z=0 |
| tracker_controller | IDLE/TRACKING/LOST, 각도 Kp P 제어, 속도 상한·데드밴드 | z=0 첫 프레임부터 0, 입력 0.5 s 끊기면 LOST:input_timeout |
| opencr_bridge | 명령 → 시리얼 V, 상태 줄 → joint_states | 명령 0.2 s 끊기면 V 0 0 |
| OpenCR | 속도 실행, 각도·속도 제한 | V 300 ms 끊기면 속도 0 (토크 유지) |

인터페이스 정의: docs/interface.md
"""


def run_case(node, st, pub, case, log_rows, info):
    from geometry_msgs.msg import PointStamped
    name, ex, ey, area, secs, stale, _, _ = case
    t_start = time.time()
    first = node.get_clock().now().to_msg()
    stop_pub = t_start + secs
    end = t_start + secs + 1.5           # 발행 중단 뒤 정지까지 기록
    period = 1 / 30
    next_pub = t_start
    stop_t = None
    last_pub = None
    info['n_pub'], info['t_abs'] = 0, (t_start, end)     # 보낸 개수·시험 구간(전달 확인용)
    while time.time() < end:
        now = time.time()
        if now < stop_pub and now >= next_pub:
            m = PointStamped()
            m.header.stamp = first if stale else node.get_clock().now().to_msg()
            m.header.frame_id = 'camera_color_optical_frame'
            m.point.x, m.point.y, m.point.z = (ex, ey, area) if area > 0 else (0.0, 0.0, 0.0)
            pub.publish(m)
            info['n_pub'] += 1
            last_pub = now - t_start            # 실제로 마지막 메시지를 보낸 시각 (타임아웃은 여기서부터 잰다)
            next_pub += period
        if now >= stop_pub and stop_t is None:
            stop_t = now
        C.spin_for(node, 0.005)
        if st['cmd'] is not None:
            log_rows.append({'case': name, 't_s': now - t_start, 'status': st['status'],
                             'pan_cmd': st['cmd'][0], 'tilt_cmd': st['cmd'][1], 'publishing': int(now < stop_pub),
                             'last_pub_s': last_pub})
    return last_pub


def delivery(ctl_rows, info):
    """제어 노드가 실제로 받은 신선한 입력 수(target_seq 증가) / 보낸 수. 같은 stamp 재전송은 첫 1개만 신선한 입력."""
    t0, t1 = info['t_abs']
    seqs = {r['target_seq'] for r in ctl_rows if t0 <= C.fnum(r['ros_time_s']) <= t1 + 0.2 and r['target_seq'] not in ('', '0')}
    expected = 1 if info['stale'] else info['n_pub']
    return len(seqs), expected


def judge(case, rows, stop_t, t0):
    """판정: 발행 구간 마지막 0.6 s(침묵 시험은 발행 중단 1 s 뒤부터)의 상태·명령 중앙값"""
    name, _, _, _, secs, stale, desc, ok = case
    win = [r for r in rows if r['case'] == name and
           ((secs - 0.6 <= r['t_s'] <= secs) if name != 'silence' else r['t_s'] >= secs + 1.0)]
    if not win:
        return {'case': name, 'expected': desc, 'verdict': 'NO DATA'}
    pan = sorted(r['pan_cmd'] for r in win)[len(win) // 2]
    tilt = sorted(r['tilt_cmd'] for r in win)[len(win) // 2]
    status = max({r['status'] for r in win}, key=lambda s: sum(r['status'] == s for r in win))
    res = {'case': name, 'expected': desc, 'status': status, 'pan_cmd': pan, 'tilt_cmd': tilt,
           'verdict': 'PASS' if ok(pan, tilt, status) else 'FAIL'}
    if name == 'silence':     # 실제 마지막 입력 → LOST:input_timeout, → 명령 0 까지 시간
        last = stop_t if stop_t is not None else secs
        after = [r for r in rows if r['case'] == name and r['t_s'] >= last]
        lost = [r for r in after if r['status'] == 'LOST:input_timeout']
        zero = [r for r in after if r['status'].startswith('LOST') and r['pan_cmd'] == 0 and r['tilt_cmd'] == 0]
        res['timeout_s'] = lost[0]['t_s'] - last if lost else math.nan
        res['cmd_zero_s'] = zero[0]['t_s'] - last if zero else math.nan
    return res


def run_all(test='assignment2'):
    """모의 입력 전 경우를 실행하고 (결과 목록, 결과 폴더)를 돌려준다. assignment_B도 이 함수를 쓴다."""
    C.check_ros()
    run_id = f'{test}_mock_{C.tag()}'
    dest = C.out_dir(test, run_id)
    ctl = C.run_node('tracker_controller', 'controller_node', 'tracker_controller',
                     os.path.join(dest, 'controller.log'), params={'auto_enable': True, 'run_id': run_id})
    node, st = C.make_probe()
    from geometry_msgs.msg import PointStamped
    from rclpy.qos import qos_profile_sensor_data
    pub = node.create_publisher(PointStamped, '/target', qos_profile_sensor_data)
    try:
        if not C.wait_until(node, lambda: st['status'] is not None, 20):
            raise SystemExit(f'제어 노드가 뜨지 않았습니다: {ctl.log_path}')
        rows, results, runs = [], [], []
        for case in CASES:
            print(f'=== {case[0]}: {case[6]}')
            C.spin_for(node, 1.0)          # 이전 경우의 입력이 끊긴 상태(LOST:input_timeout)에서 시작
            info = {'stale': case[5]}
            stop_t = run_case(node, st, pub, case, rows, info)
            r = judge(case, rows, stop_t, 0)
            results.append(r)
            runs.append(info)
            print(f'    → {r.get("status")} 팬 {C.fmt(r.get("pan_cmd", ""))} 틸트 {C.fmt(r.get("tilt_cmd", ""))} '
                  f'{"타임아웃 %.3f s, 명령 0까지 %.3f s " % (r["timeout_s"], r["cmd_zero_s"]) if "timeout_s" in r else ""}[{r["verdict"]}]')
        ctl.stop()                         # 제어 기록을 끝까지 저장한 뒤 전달 확인
        ctl_rows = C.read_csv(C.node_logs(run_id)['control'])
        for r, info in zip(results, runs):
            got, sent = delivery(ctl_rows, info)
            r['inputs_received'] = f'{got}/{sent}'
            if sent and got / sent < 0.9 and r['verdict'] == 'PASS':
                r['verdict'] = 'RETEST(입력 손실)'      # 입력이 제대로 안 갔으면 판정을 믿을 수 없다
        for r in results:
            print(f'  {r["case"]:9s} 받은 입력 {r["inputs_received"]:>7s}  [{r["verdict"]}]')
        for case in CASES:
            C.write_csv(os.path.join(dest, f'{case[0]}.csv'), [r for r in rows if r['case'] == case[0]])
        C.write_csv(os.path.join(dest, 'cases.csv'), results,
                    ['case', 'expected', 'status', 'pan_cmd', 'tilt_cmd', 'timeout_s', 'cmd_zero_s', 'inputs_received', 'verdict'])
        open(os.path.join(dest, 'structure.md'), 'w').write(STRUCTURE)
        md = [f'# 모의 입력 ({run_id})', '', f'- Kp 팬 {C.read_param("pan_kp")}·틸트 {C.read_param("tilt_kp")} [1/s], '
              f'direction 팬 {C.read_param("pan_direction")}·틸트 {C.read_param("tilt_direction")}, '
              f'입력 타임아웃 {C.read_param("input_timeout_s")} s, 모터 출력 없음(브리지 미실행)', '',
              C.md_table(results, ['case', 'expected', 'status', 'pan_cmd', 'tilt_cmd', 'timeout_s', 'cmd_zero_s', 'inputs_received', 'verdict']),
              '', '- inputs_received: 제어 노드가 받은 신선한 입력 / 보낸 입력(같은 stamp 재전송은 첫 1개). 90 % 미만이면 RETEST',
              '- timeout_s·cmd_zero_s: 실제로 마지막 /target을 보낸 시각부터 LOST:input_timeout·명령 0이 관측될 때까지 (상태·명령은 50 Hz로 받음)',
              '', '- 미검출(z=0)은 정상 영상에서 목표가 없다는 뜻이라 즉시 정지(LOST:no_detection),'
              ' 토픽 침묵은 인지가 멈췄다는 뜻이라 0.5 s 뒤 정지(LOST:input_timeout)로 구분한다.',
              f'- 제어 기록: ~/lv2_module5_logs/{run_id}.csv, 구조도: structure.md']
        open(os.path.join(dest, 'summary.md'), 'w').write('\n'.join(md) + '\n')
        n_pass = sum(r['verdict'] == 'PASS' for r in results)
        print(f'\n{n_pass}/{len(results)} PASS → {dest}')
    finally:
        node.destroy_node()
        ctl.stop()
        C.keep_logs(run_id, dest)
    return results, dest


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.parse_args()
    run_all()


if __name__ == '__main__':
    main()
