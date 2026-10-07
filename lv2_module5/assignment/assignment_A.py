#!/usr/bin/env python3
"""도전 A (문제 1 심화) — 환경 변화에 강한 검출: 조명 또는 거리 하나만 바꾼 두 조건에서 검출률·배경 오검출·처리 속도를 비교한다.

조건마다 같은 방식으로 평가 프레임(목표 보임 30장 이상 + 목표 없음 10장 이상)을 저장하고, 같은 원본 프레임에
기본 설정과 개선 설정(검출 설정 하나만 변경, 선택)을 함께 적용한다. 사람이 검출 이미지를 대조해 correct를 채운다.

  python3 assignment/assignment_A.py capture --condition bright_0.6m                       # 기본 조건
  python3 assignment/assignment_A.py capture --condition dim_0.6m --param obj_area_min_cm2 --value 1.5   # 바뀐 조건 + 개선 설정
  (각 폴더의 labels.csv correct 열 채우기)
  python3 assignment/assignment_A.py score results/assignment_A/bright_0.6m_<시각> results/assignment_A/dim_0.6m_<시각>
결과: results/assignment_A/<조건>_<시각>/ (평가 프레임 이미지·labels.csv·설정 2종), compare.csv·summary.md, results/plots/assignment_A_*.png
조건표(조명 lux 또는 거리 m, 시각, 배경)는 capture 때 입력한 메모와 함께 condition.txt에 남는다.
"""
import argparse
import os
import time

import common as C


def capture(args):
    dest = C.out_dir('assignment_A', f'{args.condition}_{C.tag()}')
    configs = {'base': C.config_variant(os.path.join(dest, 'config_base'), {})}
    if args.param:
        configs['improved'] = C.config_variant(os.path.join(dest, 'config_improved'), {args.param: args.value})
    memo = input('조건 메모(예: 천장등 끔 120 lux, 거리 0.6 m, 흰 벽 배경): ').strip()
    open(os.path.join(dest, 'condition.txt'), 'w').write(
        f'조건: {args.condition}\n메모: {memo}\n시각: {time.strftime("%Y-%m-%d %H:%M:%S")}\n커밋: {C.git_commit()}\n'
        f'개선 설정: {args.param}={args.value}\n' if args.param else
        f'조건: {args.condition}\n메모: {memo}\n시각: {time.strftime("%Y-%m-%d %H:%M:%S")}\n커밋: {C.git_commit()}\n개선 설정: 없음\n')
    proc = None if args.no_launch else C.launch('perception.launch.py', os.path.join(dest, 'launch.log'),
                                                run_id=f'assignment_A_{C.tag()}')
    try:
        C.capture_eval(dest, [('target', args.target), ('empty', args.empty)], args.interval, configs)
    finally:
        if proc:
            proc.stop()


def score(args):
    rows = []
    for d in args.dirs:
        cond = open(os.path.join(d, 'condition.txt')).read().splitlines()[0].split(': ', 1)[1]
        for r in C.score_eval(os.path.join(d, 'labels.csv')):
            rows.append({'condition': cond, 'folder': os.path.basename(d.rstrip('/')), **r})
    dest = C.out_dir('assignment_A')
    C.write_csv(os.path.join(dest, 'compare.csv'), rows)
    C.update_metrics([{'test': 'assignment_A', 'run_id': f'{r["folder"]}_{r["config"]}', 'condition': r['condition'],
                       'date': time.strftime('%Y-%m-%d'), **{k: v for k, v in r.items() if k not in ('condition', 'folder')}}
                      for r in rows])
    pdir = C.out_dir('plots')
    f1 = C.bar_plot(os.path.join(pdir, 'assignment_A_detection_rate.png'),
                    [(f'{r["condition"]}/{r["config"]}', r['detection_rate_pct']) for r in rows], 'Detection rate by condition', '%')
    f2 = C.bar_plot(os.path.join(pdir, 'assignment_A_false_detections.png'),
                    [(f'{r["condition"]}/{r["config"]}', r['false_detections']) for r in rows], 'False detections (empty frames)', 'count')
    f3 = C.bar_plot(os.path.join(pdir, 'assignment_A_proc_fps.png'),
                    [(f'{r["condition"]}/{r["config"]}', r['proc_fps']) for r in rows], 'Processing speed', 'FPS')
    md = ['# 도전 A 조건별 검출 비교', '',
          C.md_table(rows, ['condition', 'config', 'visible_frames', 'detection_rate_pct', 'missed', 'wrong_object',
                            'empty_frames', 'false_detections', 'proc_fps', 'unlabeled']), '',
          '- 조건은 조명 또는 거리 하나만 바꾼다. 기본 설정(base)과 개선 설정(improved)은 같은 원본 프레임에 적용했다',
          '- 검출률 = 올바른 검출 / 목표가 보이는 프레임 x100 (사람 대조), 배경 오검출 = 목표 없는 프레임의 검출 수',
          '- 처리 속도 = 1000 / 평균 처리 시간 (같은 함수, 같은 Pi)',
          '- 실패 장면: labels.csv에서 detected=0 또는 correct=0 행의 이미지',
          '그래프: ' + ', '.join(os.path.relpath(f, C.LV2) for f in (f1, f2, f3) if f)]
    open(os.path.join(dest, 'summary.md'), 'w').write('\n'.join(md) + '\n')
    print('\n'.join(md))


def main():
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    sub = ap.add_subparsers(dest='cmd', required=True)
    c = sub.add_parser('capture')
    c.add_argument('--condition', required=True, help='조건 이름(영문): bright_0.6m, dim_0.6m, bright_1.5m ...')
    c.add_argument('--param', help='개선 설정에서 바꿀 검출 설정 키 (선택)')
    c.add_argument('--value')
    c.add_argument('--target', type=int, default=30)
    c.add_argument('--empty', type=int, default=10)
    c.add_argument('--interval', type=float, default=0.5)
    c.add_argument('--no-launch', action='store_true')
    s = sub.add_parser('score')
    s.add_argument('dirs', nargs='+')
    args = ap.parse_args()
    if args.cmd == 'capture':
        if bool(args.param) != (args.value is not None):
            raise SystemExit('--param과 --value는 함께 지정')
        C.check_ros()
        capture(args)
    else:
        score(args)


if __name__ == '__main__':
    main()
