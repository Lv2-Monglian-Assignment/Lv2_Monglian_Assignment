#!/usr/bin/env python3
"""문제 5 — bag 재현과 팀 협업: 성공·소실 장면 bag 기록, 입력 재처리(모터 출력 없음), 결과 재분석, 다른 팀원 실행 기록.

  python3 assignment/assignment5.py record --name success --seconds 20   # 추적 성공 장면 (노드를 띄우고 추적 켠 채 기록)
  python3 assignment/assignment5.py record --name lost --seconds 20      # 소실·복귀 장면 (안내에 따라 가렸다가 치움)
  python3 assignment/assignment5.py replay recordings/<run_id>           # 입력 재처리: bag 영상 → 검출기 → /target_replay (모터 없음)
  python3 assignment/assignment5.py reanalyze recordings/<run_id>        # 결과 재분석: 저장된 /target·상태·명령으로 지표 재계산
  python3 assignment/assignment5.py reproduce --who <이름>               # 작성자가 아닌 팀원의 README 실행 기록
결과: recordings/<run_id>/ (bag, git 제외) · recordings/<run_id>_info.txt (토픽·메시지 수·기간·크기·sha256·커밋·설정)
      recordings/README.md 표 · results/assignment5/<run_id>/ (재처리·재분석 표와 그래프)
재처리할 때 저장된 /target과 새 검출 결과(/target_replay)를 섞지 않는다. 시간은 bag 시계(--clock, use_sim_time)를 쓴다.
심화(고정 bag 회귀 비교)는 assignment_E.py
"""
import argparse
import glob
import hashlib
import math
import os
import shutil
import subprocess
import time

import common as C

TOPICS = ['/camera/camera/color/image_raw', '/camera/camera/color/camera_info',
          '/camera/camera/aligned_depth_to_color/image_raw', '/target', '/target_depth', '/target/position_cam',
          '/tracking_status', '/pan_tilt/command', '/pan_tilt/joint_states']
REC_DIR = os.path.join(C.LV2, 'recordings')


# ---------------- bag 읽기 ----------------
def read_bag(bag, topics):
    """{토픽: [(stamp 또는 기록 시각 [s], msg)]}. header가 있으면 header.stamp, 없으면 기록 시각."""
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=''), rosbag2_py.ConverterOptions('cdr', 'cdr'))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    want = [t for t in topics if t in types]
    reader.set_filter(rosbag2_py.StorageFilter(topics=want))
    out = {t: [] for t in want}
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        msg = deserialize_message(data, get_message(types[topic]))
        hdr = getattr(msg, 'header', None)
        t = hdr.stamp.sec + hdr.stamp.nanosec * 1e-9 if hdr is not None else t_ns * 1e-9
        out[topic].append((t, msg, t_ns * 1e-9))
    return out


def bag_info(bag):
    return subprocess.run(['ros2', 'bag', 'info', bag], capture_output=True, text=True).stdout


def bag_duration(bag):
    import yaml
    meta = yaml.safe_load(open(os.path.join(bag, 'metadata.yaml')))['rosbag2_bagfile_information']
    return meta['duration']['nanoseconds'] * 1e-9


def sha256s(bag):
    out = []
    for f in sorted(os.listdir(bag)):
        h = hashlib.sha256()
        with open(os.path.join(bag, f), 'rb') as fp:
            for chunk in iter(lambda: fp.read(1 << 20), b''):
                h.update(chunk)
        out.append((f, os.path.getsize(os.path.join(bag, f)), h.hexdigest()))
    return out


# ---------------- 기록 ----------------
def record(args):
    run_id = f'assignment5_{args.name}_{C.tag()}'
    dest = C.out_dir('assignment5', run_id)
    bag = os.path.join(REC_DIR, run_id)
    proc, node, st = C.start_stack(run_id, dest, dry_run=args.dry_run)
    rec = None
    marks = []
    try:
        C.ask(f'[{args.name}] 목표를 시야 안에 두세요. 추적을 켜고 {args.seconds:g} s 기록합니다'
              + (' — 중간에 "가리세요"·"치우세요" 안내를 따르세요' if args.name == 'lost' else ''))
        C.set_tracking(node, st, True)
        C.spin_for(node, 1.0)
        # --disable-keyboard-controls: Lyrical 기록기는 키보드 제어로 터미널을 읽는다(백그라운드에서 멈춤, 2026-10-07 #43)
        rec = C.Proc(['ros2', 'bag', 'record', '-o', bag, '--disable-keyboard-controls', '--topics', *TOPICS],
                     os.path.join(dest, 'bag_record.log'))
        print('  기록기 준비 중(모든 토픽 구독까지 기다림)...')
        t_rec = time.time()
        while time.time() - t_rec < 20 and len(C.wait_bag_ready(rec.log_path, TOPICS, 0.1)) < len(TOPICS):
            C.spin_for(node, 0.2)            # 기다리는 동안에도 상태를 받는다
        got = C.wait_bag_ready(rec.log_path, TOPICS, 0.1)
        print(f'  기록 시작: 토픽 {len(got)}/{len(TOPICS)} 구독 ({time.time() - t_rec:.1f} s)'
              + (f', 빠짐 {sorted(set(TOPICS) - got)}' if len(got) < len(TOPICS) else ''))
        t0 = time.time()
        plan = [(args.seconds * 0.35, '가리세요!'), (args.seconds * 0.35 + 2.0, '치우세요!')] if args.name == 'lost' else []
        while time.time() - t0 < args.seconds:
            C.spin_for(node, 0.1)
            if plan and time.time() - t0 >= plan[0][0]:
                print(f'\n  \a{plan[0][1]}')
                marks.append((plan[0][1], time.time()))
                plan.pop(0)
            print(f'\r  기록 {time.time() - t0:5.1f}/{args.seconds:g} s  상태 {st["status"]}    ', end='', flush=True)
        print()
    finally:
        if rec:
            rec.stop()
        C.set_tracking(node, st, False)
        C.spin_for(node, 1.5)
        node.destroy_node()
        proc.stop()
    C.keep_logs(run_id, dest)
    shutil.copytree(C.CONFIG, os.path.join(dest, 'config'))
    info = bag_info(bag)
    sums = sha256s(bag)
    size = sum(s for _, s, _ in sums)
    text = [f'run_id: {run_id}', f'장면: {args.name}', f'기록: {time.strftime("%Y-%m-%d %H:%M:%S")}', f'커밋: {C.git_commit()}',
            f'설정: results/assignment5/{run_id}/config/ (기록 당시 config 복사본)',
            f'노드 기록: results/assignment5/{run_id}/ (제어 {run_id}.csv, 인지 {run_id}_detect.csv, 브리지 시리얼 로그)',
            f'안내 시각: {marks}', f'크기: {size / 1e6:.1f} MB', '', info, 'sha256:'] + [f'  {h}  {f} ({s} B)' for f, s, h in sums]
    open(f'{bag}_info.txt', 'w').write('\n'.join(text) + '\n')
    readme = os.path.join(REC_DIR, 'README.md')
    if not os.path.exists(readme) or os.path.getsize(readme) == 0:
        open(readme, 'w').write('# bag 기록\n\nbag 원본(`recordings/<run_id>/`)은 용량 때문에 git에서 제외한다. '
                                '위치·메타데이터·해시는 `<run_id>_info.txt`에 있다. 재생: README 7절, `assignment/assignment5.py replay`.\n\n'
                                '| run_id | 장면 | 기간 [s] | 크기 [MB] | 커밋 | 보관 위치 |\n|---|---|---|---|---|---|\n')
    with open(readme, 'a') as f:
        f.write(f'| {run_id} | {args.name} | {bag_duration(bag):.1f} | {size / 1e6:.1f} | {C.git_commit()} | '
                f'Pi `~/git/Lv2_Monglian_Assignment/lv2_module5/recordings/{run_id}/` (#todo 공유 위치 링크) |\n')
    print(info)
    print(f'정보: {bag}_info.txt, 크기 {size / 1e6:.1f} MB')


# ---------------- 입력 재처리 ----------------
def replay_bag(bag, run_id, dest, config_dir=C.CONFIG):
    """replay.launch: bag 영상·정렬 Depth·모터 각도만 재생(--clock) → 검출기(use_sim_time) → /target_replay. 모터 노드 없음.
    재생이 끝나면 노드를 끄고 검출기 기록(<run_id>_detect.csv) 경로를 돌려준다."""
    C.check_ros()
    dur = bag_duration(bag)
    proc = C.launch('replay.launch.py', os.path.join(dest, f'{run_id}_launch.log'), bag=os.path.abspath(bag), run_id=run_id,
                    config_dir=config_dir)
    t0 = time.time()
    while time.time() - t0 < dur + 8 and proc.alive():
        print(f'\r  재생 {time.time() - t0:5.1f}/{dur:.1f} s   ', end='', flush=True)
        time.sleep(0.5)
        if 'ros2-2]: process has finished' in open(proc.log_path).read():
            break
    print()
    time.sleep(1.5)                       # 검출기 기록 저장 주기 1 s
    proc.stop()
    det = C.node_logs(run_id)['detect']
    if not os.path.exists(det):
        raise SystemExit(f'재처리 기록이 없습니다: {det} (로그 {proc.log_path})')
    shutil.copy2(det, dest)
    return os.path.join(dest, os.path.basename(det))


def compare_replay(bag, det_csv):
    """저장된 /target(원본 검출)과 재처리 /target_replay(인지 기록)를 같은 영상 stamp로 맞춰 비교한다."""
    orig = {round(t, 6): m for t, m, _ in read_bag(bag, ['/target']).get('/target', [])}
    rep = {round(C.fnum(r['stamp_s']), 6): r for r in C.read_csv(det_csv)}
    common = sorted(set(orig) & set(rep))
    agree = sum((orig[t].point.z > 0) == (rep[t]['detected'] == '1') for t in common)
    both = [t for t in common if orig[t].point.z > 0 and rep[t]['detected'] == '1']
    dex = [abs(orig[t].point.x - C.fnum(rep[t]['ex'])) for t in both]
    res = {'orig_frames': len(orig), 'replay_frames': len(rep), 'matched_frames': len(common),
           'orig_detect_ratio': sum(m.point.z > 0 for m in orig.values()) / len(orig) if orig else math.nan,
           'replay_detect_ratio': sum(r['detected'] == '1' for r in rep.values()) / len(rep) if rep else math.nan,
           'detect_agreement_pct': 100 * agree / len(common) if common else math.nan,
           'ex_mean_abs_diff': sum(dex) / len(dex) if dex else math.nan,
           'ex_max_abs_diff': max(dex) if dex else math.nan}
    t0 = common[0] if common else 0
    series = [('bag /target ex', [t - t0 for t in common], [orig[t].point.x if orig[t].point.z > 0 else math.nan for t in common]),
              ('replay ex', [t - t0 for t in common], [C.fnum(rep[t]['ex']) if rep[t]['detected'] == '1' else math.nan for t in common])]
    return res, series


def replay(args):
    bag = args.bag.rstrip('/')
    name = os.path.basename(bag)
    run_id = f'replay_{name}_{C.tag()}'
    dest = C.out_dir('assignment5', f'{name}_replay')
    det = replay_bag(bag, run_id, dest)
    res, series = compare_replay(bag, det)
    plot = C.line_plot(os.path.join(C.out_dir('plots'), f'{name}_replay_ex.png'), series,
                       f'Input reprocessing: {name}', 'time [s]', 'ex', [(0, '0')])
    md = [f'# 입력 재처리 ({name})', '', '- 명령: `ros2 launch tracker_bringup replay.launch.py bag:=... run_id:=...` '
          '(bag의 영상·CameraInfo·정렬 Depth·모터 각도만 재생, 검출 결과는 /target_replay로 분리, 모터 노드 없음)',
          f'- 기준 커밋 {C.git_commit()}, 설정 config/', '', C.md_table([res], list(res.keys())), '',
          '- 같은 영상 stamp끼리 비교: 검출 여부 일치율, 둘 다 검출한 프레임의 ex 차이',
          '- 번호 유지(object_tracker)는 모터 각도(/pan_tilt/joint_states)를 쓰므로 bag에 각도가 있어야 같은 선택이 재현된다',
          f'- 그래프: {os.path.relpath(plot, C.LV2) if plot else "없음"}, 재처리 기록: {os.path.basename(det)}']
    open(os.path.join(dest, 'summary.md'), 'w').write('\n'.join(md) + '\n')
    print('\n'.join(md))


# ---------------- 결과 재분석 ----------------
def bag_metrics(bag):
    """bag에 저장된 /target·/tracking_status·/pan_tilt/command만으로 지표를 다시 계산한다(검출기를 다시 돌리지 않음)."""
    data = read_bag(bag, ['/target', '/tracking_status', '/pan_tilt/command'])
    tg = data.get('/target', [])
    stt = [(rt, m.data) for _, m, rt in data.get('/tracking_status', [])]
    if not tg:
        return {}
    times = [t for t, _, _ in tg]
    det = [m.point.z > 0 for _, m, _ in tg]
    ex_trk, trk = [], 0
    j, cur = 0, None
    for t, m, rt in tg:
        while j < len(stt) and stt[j][0] <= rt:
            cur = stt[j][1]
            j += 1
        if cur and cur.startswith('TRACKING'):
            trk += 1
            if m.point.z > 0:
                ex_trk.append(m.point.x)
    trans = sum(1 for a, b in zip(stt, stt[1:]) if not a[1].startswith('TRACKING') and b[1].startswith('TRACKING'))
    cmds = [abs(m.vector.x) for _, m, _ in data.get('/pan_tilt/command', [])]
    return {'frames': len(tg), 'duration_s': times[-1] - times[0], 'target_rate_hz': (len(tg) - 1) / (times[-1] - times[0]),
            'node_detect_ratio': sum(det) / len(det), 'tracking_ratio': trk / len(tg),
            'rmse_ex': math.sqrt(sum(e * e for e in ex_trk) / len(ex_trk)) if ex_trk else math.nan,
            'rmse_rows': len(ex_trk), 'to_tracking_transitions': trans, 'max_abs_pan_cmd': max(cmds, default=math.nan)}


def reanalyze(args):
    bag = args.bag.rstrip('/')
    name = os.path.basename(bag)
    dest = C.out_dir('assignment5', name)
    b = bag_metrics(bag)
    rows = [{'source': 'bag 재분석 (/target 30 Hz 기준)', **b}]
    ctl = os.path.join(dest, f'{name}.csv')
    if os.path.exists(ctl):
        ro = C.read_csv(ctl)
        inbag = [r for r in ro if r['state'] != 'IDLE']
        m = C.tracking_metrics(inbag)
        rows.append({'source': '원본 제어 기록 (50 Hz 행 기준)', 'duration_s': m.get('duration_s'),
                     'node_detect_ratio': m.get('node_detect_ratio'), 'tracking_ratio': m.get('tracking_ratio'),
                     'rmse_ex': m.get('rmse_ex'), 'rmse_rows': m.get('rmse_rows')})
    md = [f'# 결과 재분석 ({name})', '', C.md_table(rows, ['source', 'frames', 'duration_s', 'target_rate_hz', 'node_detect_ratio',
          'tracking_ratio', 'rmse_ex', 'rmse_rows', 'to_tracking_transitions', 'max_abs_pan_cmd']), '',
          '- 재분석은 저장된 결과 토픽만 읽어 지표를 다시 계산한다(검출기 재실행 아님). 입력 재처리와 구분한다',
          '- 원본 제어 기록은 추적을 켠 전체 구간, bag은 기록 구간이라 길이가 다를 수 있다. 표본 단위(30 Hz 영상 / 50 Hz 제어)도 다르다']
    C.write_csv(os.path.join(dest, 'reanalysis.csv'), rows)
    open(os.path.join(dest, 'reanalysis.md'), 'w').write('\n'.join(md) + '\n')
    C.update_metrics([{'test': 'assignment5_reanalysis', 'run_id': name, 'condition': 'bag', 'date': time.strftime('%Y-%m-%d'), **b}])
    print('\n'.join(md))


# ---------------- 다른 팀원 재현 기록 ----------------
def reproduce(args):
    dest = C.out_dir('assignment5')
    path = os.path.join(dest, f'reproduction_{args.who}_{C.tag()}.md')
    items = ['저장소 clone·빌드 (README 3절)', '펌웨어 업로드·시리얼 확인 (README 4절)', '실행·정지 (README 5·6절)',
             'bag 재처리·재분석 (README 7절, assignment5 replay/reanalyze)']
    print(f'확인자 {args.who}, 기준 커밋 {C.git_commit()}. 항목마다 결과(성공/실패/수정한 내용)를 입력하세요.')
    lines = [f'# 재현 확인 기록', '', f'- 확인자: {args.who}', f'- 날짜: {time.strftime("%Y-%m-%d %H:%M")}',
             f'- 기준 커밋: {C.git_commit()}', '', '| 항목 | 결과 | 누락·수정한 경로·설정 |', '|---|---|---|']
    for it in items:
        res = input(f'  {it} 결과: ').strip()
        fix = input('    누락·수정 내용(없으면 Enter): ').strip()
        lines.append(f'| {it} | {res} | {fix} |')
    open(path, 'w').write('\n'.join(lines) + '\n')
    print(f'저장: {path} (README 9절 표에도 옮겨 적기)')


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('record')
    r.add_argument('--name', choices=('success', 'lost'), required=True)
    r.add_argument('--seconds', type=float, default=20.0)
    r.add_argument('--dry-run', action='store_true')
    for n in ('replay', 'reanalyze'):
        p = sub.add_parser(n)
        p.add_argument('bag')
    w = sub.add_parser('reproduce')
    w.add_argument('--who', required=True)
    args = ap.parse_args()
    if args.cmd in ('record',):
        C.check_ros()
    {'record': record, 'replay': replay, 'reanalyze': reanalyze, 'reproduce': reproduce}[args.cmd](args)


if __name__ == '__main__':
    main()
