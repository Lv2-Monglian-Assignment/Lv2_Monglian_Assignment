#!/usr/bin/env python3
"""bag 결과 재분석·입력 재처리 비교 (문제 5). 모터·노드 불필요, bag만 읽는다 (ROS 환경을 source한 셸에서 실행).

사용:
  python3 scripts/analyze_bag.py recordings/<run_id>                       # 저장된 /target·상태·명령으로 지표 재계산
  python3 scripts/analyze_bag.py recordings/<run_id> --csv ~/lv2_module5_logs/<run_id>.csv
                                                                           # 실행 중 제어 기록 CSV로 같은 지표를 계산해 대조
  python3 scripts/analyze_bag.py recordings/<run_id> --replay recordings/<run_id>_replay [--replay recordings/<run_id>_kpB ...]
                                                                           # 입력 재처리(/target_replay)를 저장된 /target과 같은 영상 stamp끼리 비교
  --save : results/logs/replay/<run_id>_analysis.txt 저장 + results/metrics.csv에 행 추가

시각 기준: 모든 시간 계산은 bag에 기록된 수신 시각(같은 실행의 Pi 시계)만 쓴다. 현재 벽시계는 쓰지 않는다.
재처리 비교의 짝은 영상 header.stamp(검출기가 입력 영상 header를 그대로 복사)로 맞춘다.
"""
import argparse
import csv
import math
import os
import sys

TRACKING, LOST, SEARCHING = 'TRACKING', 'LOST', 'SEARCHING'
FIELDS = ['run_id', 'source', 'duration_s', 'target_hz', 'detect_rate', 'tracking_ratio', 'rmse_ex', 'rmse_ey',
          'lost_events', 'recover_mean_s', 'recover_max_s', 'unrecovered', 'cmd_when_stopped', 'max_abs_cmd_dps']


# ================= 지표 (ROS 없이 계산, tests/test_analyze_bag.py가 검사) =================
def metrics(targets, statuses, cmds):
    """targets : [(t, stamp_ns, ex, ey, z)]  z > 0 이면 검출 (/target 규약: 미검출은 0,0,0)
    statuses: [(t, state)]                  /tracking_status 'STATE:reason'의 STATE
    cmds    : [(t, pan_dps, tilt_dps)]      /pan_tilt/command
    t는 모두 같은 시계의 초. 반환: FIELDS 중 run_id·source를 뺀 dict"""
    ts = [x[0] for x in targets] + [x[0] for x in statuses] + [x[0] for x in cmds]
    m = {'duration_s': max(ts) - min(ts) if ts else math.nan}
    span = targets[-1][0] - targets[0][0] if len(targets) > 1 else 0
    m['target_hz'] = (len(targets) - 1) / span if span > 0 else math.nan
    m['detect_rate'] = sum(x[4] > 0 for x in targets) / len(targets) if targets else math.nan
    m['tracking_ratio'] = sum(s == TRACKING for _, s in statuses) / len(statuses) if statuses else math.nan

    state_at = _state_lookup(statuses)
    err = [(ex, ey) for t, _, ex, ey, z in targets if z > 0 and state_at(t) == TRACKING]
    m['rmse_ex'] = math.sqrt(sum(e[0] ** 2 for e in err) / len(err)) if err else math.nan
    m['rmse_ey'] = math.sqrt(sum(e[1] ** 2 for e in err) / len(err)) if err else math.nan

    # 소실 = TRACKING -> 그 밖의 상태(LOST·SEARCHING), 복귀 = 다음 TRACKING까지 걸린 시간
    recover, lost_t, prev = [], None, None
    for t, s in statuses:
        if prev == TRACKING and s != TRACKING and lost_t is None:
            lost_t = t
        if s == TRACKING and lost_t is not None:
            recover.append(t - lost_t)
            lost_t = None
        prev = s
    m['lost_events'] = len(recover) + (lost_t is not None)
    m['recover_mean_s'] = sum(recover) / len(recover) if recover else math.nan
    m['recover_max_s'] = max(recover) if recover else math.nan
    m['unrecovered'] = int(lost_t is not None)

    # 안전 확인: 추적·탐색이 아닌데 0이 아닌 명령이 나간 횟수 (0이어야 정상)
    m['cmd_when_stopped'] = sum(1 for t, p, q in cmds if (p or q) and state_at(t) not in (TRACKING, SEARCHING))
    m['max_abs_cmd_dps'] = max((max(abs(p), abs(q)) for _, p, q in cmds), default=math.nan)
    return m


def _state_lookup(statuses):
    """시각 t 직전(포함)에 발행된 상태. 상태 기록 전이면 None"""
    import bisect
    times = [t for t, _ in statuses]

    def at(t):
        i = bisect.bisect_right(times, t) - 1
        return statuses[i][1] if i >= 0 else None
    return at


def compare_replay(orig, replay):
    """orig·replay: [(t, stamp_ns, ex, ey, z)]. 같은 영상 stamp끼리 짝지어 검출 일치율·오차 차이를 낸다"""
    by_stamp = {x[1]: x for x in orig}
    pairs = [(by_stamp[r[1]], r) for r in replay if r[1] in by_stamp]
    both = [(o, r) for o, r in pairs if o[4] > 0 and r[4] > 0]
    mean = lambda v: sum(v) / len(v) if v else math.nan  # noqa: E731
    return {
        'orig_frames': len(orig), 'replay_frames': len(replay), 'matched': len(pairs),
        'match_ratio': len(pairs) / len(orig) if orig else math.nan,
        'detect_agree': mean([(o[4] > 0) == (r[4] > 0) for o, r in pairs]),
        'orig_detect_rate': mean([o[4] > 0 for o, _ in pairs]),
        'replay_detect_rate': mean([r[4] > 0 for _, r in pairs]),
        'mean_abs_dex': mean([abs(o[2] - r[2]) for o, r in both]),
        'mean_abs_dey': mean([abs(o[3] - r[3]) for o, r in both]),
        'max_abs_dex': max((abs(o[2] - r[2]) for o, r in both), default=math.nan),
    }


# ================= 입력 =================
def read_bag(path, topics):
    """{topic: [(t_s, msg)]} — t_s는 bag 수신 시각"""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=''), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    want = [t for t in topics if t in types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=want))
    out = {t: [] for t in topics}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        out[topic].append((t_ns * 1e-9, deserialize_message(data, get_message(types[topic]))))
    return out


def target_rows(msgs):
    return [(t, m.header.stamp.sec * 10**9 + m.header.stamp.nanosec, m.point.x, m.point.y, m.point.z) for t, m in msgs]


def from_bag(path):
    d = read_bag(path, ['/target', '/tracking_status', '/pan_tilt/command'])
    return (target_rows(d['/target']),
            [(t, m.data.split(':')[0]) for t, m in d['/tracking_status']],
            [(t, m.vector.x, m.vector.y) for t, m in d['/pan_tilt/command']])


def from_csv(path):
    """controller_node 기록 CSV: 제어 주기(50 Hz)마다 1행. 목표는 target_seq가 바뀐 첫 행만 1프레임으로 센다"""
    rows = list(csv.DictReader(open(os.path.expanduser(path))))
    num = lambda v: float(v) if v not in ('', None) else math.nan  # noqa: E731
    targets, seen = [], set()
    for r in rows:
        if r['target_seq'] not in seen and r['stamp_s']:
            seen.add(r['target_seq'])
            z = num(r['area_ratio']) if r['detected'] == '1' else 0.0
            targets.append((num(r['ros_time_s']), round(num(r['stamp_s']) * 1e9), num(r['ex']), num(r['ey']), z))
    statuses = [(num(r['ros_time_s']), r['state']) for r in rows]
    cmds = [(num(r['ros_time_s']), num(r['pan_cmd']), num(r['tilt_cmd'])) for r in rows]
    return targets, statuses, cmds


# ================= 출력 =================
def fmt(v):
    return f'{v:.4f}' if isinstance(v, float) else str(v)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--csv', help='같은 실행의 controller CSV (~/lv2_module5_logs/<run_id>.csv)')
    ap.add_argument('--replay', action='append', default=[], help='replay_bag.sh 결과 bag (여러 번 가능)')
    ap.add_argument('--save', action='store_true')
    a = ap.parse_args()
    bag = a.bag.rstrip('/')
    run_id = os.path.basename(bag)
    lines, rows = [f'# {run_id} 분석 (bag: {bag})'], []

    targets, statuses, cmds = from_bag(bag)
    if not targets:
        sys.exit('bag에 /target 메시지가 없음')
    results = {'bag': metrics(targets, statuses, cmds)}
    if a.csv:
        results['csv'] = metrics(*from_csv(a.csv))
    lines.append('\n## 결과 재분석 (저장된 /target·/tracking_status·/pan_tilt/command)')
    lines.append('| 지표 | ' + ' | '.join(results) + (' | 차이 |' if 'csv' in results else ''))
    lines.append('|---|' + '---|' * (len(results) + ('csv' in results)))
    for k in FIELDS[2:]:
        vals = [results[s][k] for s in results]
        diff = f' | {fmt(vals[1] - vals[0])}' if len(vals) == 2 else ''
        lines.append(f'| {k} | ' + ' | '.join(fmt(v) for v in vals) + diff + ' |')
    for s, m in results.items():
        rows.append({'run_id': run_id, 'source': s, **{k: fmt(v) for k, v in m.items()}})

    for rp in a.replay:
        rp = rp.rstrip('/')
        replay = target_rows(read_bag(rp, ['/target_replay'])['/target_replay'])
        c = compare_replay(targets, replay)
        lines.append(f'\n## 입력 재처리 비교: {os.path.basename(rp)} (/target_replay vs 저장된 /target, 같은 영상 stamp)')
        lines += [f'- {k}: {fmt(v)}' for k, v in c.items()]
        r = metrics(replay, [], [])
        rows.append({'run_id': run_id, 'source': os.path.basename(rp),
                     **{k: fmt(r[k]) for k in ('duration_s', 'target_hz', 'detect_rate')}})

    text = '\n'.join(lines)
    print(text)
    if a.save:
        lv2 = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        out = os.path.join(lv2, 'results', 'logs', 'replay', f'{run_id}_analysis.txt')
        os.makedirs(os.path.dirname(out), exist_ok=True)
        open(out, 'w').write(text + '\n')
        mpath = os.path.join(lv2, 'results', 'metrics.csv')
        new = not os.path.exists(mpath) or os.path.getsize(mpath) == 0
        with open(mpath, 'a', newline='') as f:
            w = csv.DictWriter(f, FIELDS)
            if new:
                w.writeheader()
            w.writerows(rows)
        print(f'\n저장: {out}\n추가: {mpath} ({len(rows)}행)')


if __name__ == '__main__':
    main()
